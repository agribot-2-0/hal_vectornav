#include "hal_vectornav/VectornavNode.h"

void VectornavNode::initializeAndConnect(ros::NodeHandle* nh,
                                         vn::sensors::VnSensor::AsyncPacketReceivedHandler callback_function)
{
  // Pass pointer to ros::NodeHandle
  nh_ = nh;

  try
  {
    // Load parameters
    loadParameters();

    // Register publihsers
    registerPublishers();

    // Prepare sensor config
    vn::sensors::BinaryOutputRegister bor = prepareBinaryOutputRegister();

    // Connect to Vectornav
    connectToVectornav(bor, callback_function);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("[VectornavNode] " << e.what() << " Shutting down...\n");
    shutdown();
  }
  catch (...)
  {
    ROS_ERROR_STREAM("[VectornavNode] Unknown exception. Shutting down...\n");
    shutdown();
  }
}

void VectornavNode::run()
{
  ros::spin();
}

void VectornavNode::loadParameters()
{
  // ned_frame_id = ros_common_functions::getRosParam<std::string>(nh_, "ned_frame_id");
  ned_frame_id = ros_common_functions::getRosParam<std::string>("~ned_frame_id");
  enu_frame_id = ros_common_functions::getRosParam<std::string>("~enu_frame_id");

  serial_port = ros_common_functions::getRosParam<std::string>("~serial_port");
  serial_baud = ros_common_functions::getRosParam<int>("~serial_baud");

  async_output_rate = ros_common_functions::getRosParam<int>("~async_output_rate");
  fixed_imu_rate = ros_common_functions::getRosParam<int>("~fixed_imu_rate");
}

void VectornavNode::registerPublishers()
{
  imu_ned_pub = nh_->advertise<sensor_msgs::Imu>("vectornav/imu/raw/data", 1);
  imu_enu_pub = nh_->advertise<sensor_msgs::Imu>("vectornav/imu/rotated/data", 1);
  mag_ned_pub = nh_->advertise<sensor_msgs::MagneticField>("vectornav/imu/raw/mag", 1);
  mag_enu_pub = nh_->advertise<sensor_msgs::MagneticField>("vectornav/imu/rotated/mag", 1);
  navsatfix_pub = nh_->advertise<sensor_msgs::NavSatFix>("vectornav/gnss", 1);
}

vn::sensors::BinaryOutputRegister VectornavNode::prepareBinaryOutputRegister()
{
  using namespace vn::protocol::uart;
  vn::sensors::BinaryOutputRegister bor(
      ASYNCMODE_PORT1,
      fixed_imu_rate / async_output_rate,  // update rate [ms]
      COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES,
      TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_POSU,
      ATTITUDEGROUP_YPRU,  //<-- returning yaw pitch roll uncertainties
      INSGROUP_INSSTATUS | INSGROUP_POSLLA | INSGROUP_POSECEF | INSGROUP_VELBODY | INSGROUP_ACCELECEF, GPSGROUP_NONE);

  return bor;
}

void VectornavNode::connectToVectornav(const vn::sensors::BinaryOutputRegister& bor,
                                       vn::sensors::VnSensor::AsyncPacketReceivedHandler callback_function)
{
  vs.setResponseTimeoutMs(1000);  // Wait for up to 1000 ms for response
  vs.setRetransmitDelayMs(50);    // Retransmit every 50 ms
  vs.connect(serial_port, serial_baud);

  if (!vs.verifySensorConnectivity())
  {
    throw std::runtime_error("Connectivity verification failed. ");
  }

  vs.readModelNumber();
  vs.readFirmwareVersion();
  uint32_t hv = vs.readHardwareRevision();
  uint32_t sn = vs.readSerialNumber();
  ROS_WARN("Hardware Revision : %d, Serial Number : %d", hv, sn);

  // Set the device info for passing to the packet callback function
  vs.determineDeviceFamily();

  using namespace vn::protocol::uart;
  vn::sensors::BinaryOutputRegister bor2(
      ASYNCMODE_PORT1,
      fixed_imu_rate / async_output_rate,  // update rate [ms]
      COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES,
      TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_POSU,
      ATTITUDEGROUP_YPRU,  //<-- returning yaw pitch roll uncertainties
      INSGROUP_INSSTATUS | INSGROUP_POSLLA | INSGROUP_POSECEF | INSGROUP_VELBODY | INSGROUP_ACCELECEF, GPSGROUP_NONE);

  vs.writeAsyncDataOutputFrequency(async_output_rate);
  vs.writeBinaryOutput1(bor2);
  vs.writeAsyncDataOutputFrequency(async_output_rate);
  vs.registerAsyncPacketReceivedHandler(nullptr, callback_function);
}

void VectornavNode::vectornavCallback(void* user_data, vn::protocol::uart::Packet& packet, size_t index)
{
  // Parse packet
  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(packet);

  try
  {
    /*** Get data ***/
    // IMU
    if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration() && cd.hasAttitudeUncertainty())
    {
      vn::math::vec4f q = cd.quaternion();
      vn::math::vec3f ar = cd.angularRate();
      vn::math::vec3f al = cd.acceleration();
      vn::math::vec3f attitude_uncertainty = cd.attitudeUncertainty();
      publishImuData(q, ar, al, attitude_uncertainty);
    }

    // Magnetic field
    if (cd.hasMagnetic())
    {
      vn::math::vec3f mag = cd.magnetic();
      publishMagneticFieldData(mag);
    }

    if (cd.hasPositionEstimatedLla() && cd.hasPositionUncertaintyEstimated() && cd.hasInsStatus())
    {
      vn::math::vec3d lla = cd.positionEstimatedLla();
      float uncertainty = cd.positionUncertaintyEstimated();
      float variance = uncertainty * uncertainty;
      vn::protocol::uart::InsStatus ins_status = cd.insStatus();
      publishNavSatFix(lla, variance, ins_status);
    }
  }
  catch (const vn::invalid_operation& e)
  {
    ROS_ERROR_STREAM("[VectornavNode] Callback - " << e.what() << " Shutting down...\n");
    shutdown();
  }
}

std::int8_t VectornavNode::navSatFixStatusFromInsStatus(const vn::protocol::uart::InsStatus& ins_status)
{
  if (ins_status_mode_set(ins_status, vn::protocol::uart::InsStatus::INSSTATUS_TRACKING))
  {
    // STATUS_GBAS_FIX doesn't mean RTK fix, but indicates that internal filter works
    return sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
  }
  else if (ins_status & vn::protocol::uart::InsStatus::INSSTATUS_GPS_FIX)
  {
    return sensor_msgs::NavSatStatus::STATUS_FIX;
  }

  return sensor_msgs::NavSatStatus::STATUS_NO_FIX;
}

void VectornavNode::publishImuData(const vn::math::vec4f& quaternion, const vn::math::vec3f& angular_rate,
                                   const vn::math::vec3f& acceleration, const vn::math::vec3f& attitude_uncertainty)
{
  vn::math::vec3f attitude_uncertainty_radians;
  attitude_uncertainty_radians[0] = common_functions::radians(attitude_uncertainty[0]);
  attitude_uncertainty_radians[1] = common_functions::radians(attitude_uncertainty[1]);
  attitude_uncertainty_radians[2] = common_functions::radians(attitude_uncertainty[2]);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = ned_frame_id;
  imu_msg.angular_velocity.x = angular_rate[0];
  imu_msg.angular_velocity.y = angular_rate[1];
  imu_msg.angular_velocity.z = angular_rate[2];
  imu_msg.linear_acceleration.x = acceleration[0];
  imu_msg.linear_acceleration.y = acceleration[1];
  imu_msg.linear_acceleration.z = acceleration[2];
  // TODO - add linear acceleration and angular velocity covariances
  imu_msg.orientation.x = quaternion[0];
  imu_msg.orientation.y = quaternion[1];
  imu_msg.orientation.z = quaternion[2];
  imu_msg.orientation.w = quaternion[3];
  imu_msg.orientation_covariance[0] = attitude_uncertainty_radians[2] * attitude_uncertainty_radians[2];
  imu_msg.orientation_covariance[4] = attitude_uncertainty_radians[1] * attitude_uncertainty_radians[1];
  imu_msg.orientation_covariance[8] = attitude_uncertainty_radians[0] * attitude_uncertainty_radians[0];
  imu_ned_pub.publish(imu_msg);

  imu_msg = nedToEnu(imu_msg);
  imu_enu_pub.publish(imu_msg);
}

void VectornavNode::publishMagneticFieldData(const vn::math::vec3f& mag)
{
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.stamp = ros::Time::now();
  mag_msg.header.frame_id = ned_frame_id;
  mag_msg.magnetic_field.x = mag[0];
  mag_msg.magnetic_field.y = mag[1];
  mag_msg.magnetic_field.z = mag[2];
  // TODO - add magnetometer covariance
  mag_ned_pub.publish(mag_msg);

  mag_msg = nedToEnu(mag_msg);
  mag_enu_pub.publish(mag_msg);
}

void VectornavNode::publishNavSatFix(const vn::math::vec3d& lla, const float position_variance,
                                     const vn::protocol::uart::InsStatus& ins_status)
{
  sensor_msgs::NavSatFix navsatfix_msg;
  navsatfix_msg.header.stamp = ros::Time::now();
  navsatfix_msg.header.frame_id = ned_frame_id;
  navsatfix_msg.latitude = lla[0];
  navsatfix_msg.longitude = lla[1];
  navsatfix_msg.altitude = lla[2];
  navsatfix_msg.position_covariance[0] = position_variance;
  navsatfix_msg.position_covariance[4] = position_variance;
  navsatfix_msg.position_covariance[8] = position_variance;
  // TODO - status returned from Vectornav is sometimes incorrect
  // (switches between STATUS_NO_FIX, STATUS_FIX and STATUS_GBAS_FIX)
  navsatfix_msg.status.status = navSatFixStatusFromInsStatus(ins_status);
  navsatfix_pub.publish(navsatfix_msg);
}

sensor_msgs::Imu VectornavNode::nedToEnu(const sensor_msgs::Imu& ned_imu)
{
  sensor_msgs::Imu enu_imu;
  enu_imu.header.frame_id = enu_frame_id;
  enu_imu.header.stamp = ned_imu.header.stamp;
  enu_imu.linear_acceleration.x = ned_imu.linear_acceleration.y;
  enu_imu.linear_acceleration.y = ned_imu.linear_acceleration.x;
  enu_imu.linear_acceleration.z = -ned_imu.linear_acceleration.z;
  enu_imu.angular_velocity.x = ned_imu.angular_velocity.y;
  enu_imu.angular_velocity.y = ned_imu.angular_velocity.x;
  enu_imu.angular_velocity.z = -ned_imu.angular_velocity.z;
  double ned_roll;
  double ned_pitch;
  double ned_yaw;
  tf2::getEulerYPR(ned_imu.orientation, ned_yaw, ned_pitch, ned_roll);
  double enu_roll = ned_pitch;
  double enu_pitch = ned_roll;
  double enu_yaw = -ned_yaw + M_PI_2;
  enu_imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(enu_roll, enu_pitch, enu_yaw);
  enu_imu.orientation_covariance[0] = ned_imu.orientation_covariance[4];
  enu_imu.orientation_covariance[4] = ned_imu.orientation_covariance[0];
  enu_imu.orientation_covariance[8] = ned_imu.orientation_covariance[8];

  return enu_imu;
}

sensor_msgs::MagneticField VectornavNode::nedToEnu(const sensor_msgs::MagneticField& ned_mag)
{
  sensor_msgs::MagneticField enu_mag;
  enu_mag.header.frame_id = enu_frame_id;
  enu_mag.header.stamp = ned_mag.header.stamp;
  enu_mag.magnetic_field.x = ned_mag.magnetic_field.y;
  enu_mag.magnetic_field.y = ned_mag.magnetic_field.x;
  enu_mag.magnetic_field.z = -ned_mag.magnetic_field.z;
  enu_mag.magnetic_field_covariance[0] = ned_mag.magnetic_field_covariance[4];
  enu_mag.magnetic_field_covariance[4] = ned_mag.magnetic_field_covariance[0];
  enu_mag.magnetic_field_covariance[8] = ned_mag.magnetic_field_covariance[8];

  return enu_mag;
}

void VectornavNode::shutdown()
{
  try
  {
    vs.unregisterAsyncPacketReceivedHandler();
    ros::Duration(0.5).sleep();
    vs.disconnect();
    ros::Duration(0.5).sleep();
  }
  catch (const vn::invalid_operation& e)
  {
    // Ignore
    // It means that Vectornav sensor was not connected
  }

  ros::shutdown();
}

double inline VectornavNode::gauss_to_tesla(double gauss)
{
  return 0.0001 * gauss;
}

bool inline VectornavNode::ins_status_mode_set(const vn::protocol::uart::InsStatus& ins_status,
                                               const vn::protocol::uart::InsStatus& bitmask)
{
  return ((ins_status & 0x03) == bitmask);
}
