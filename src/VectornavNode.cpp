#include "hal_vectornav/VectornavNode.h"

void VectornavNode::initializeAndConnect(vn::sensors::VnSensor::AsyncPacketReceivedHandler callback_function)
{
  // Load parameters
  try
  {
    loadParameters();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("[VectornavNode] " << e.what() << " Shutting down...\n");
    shutdown();
  }

  // Prepare sensor config
  vn::sensors::BinaryOutputRegister bor = prepareBinaryOutputRegister();

  // Connect to Vectornav
  try
  {
    connectToVectornav(bor, callback_function);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("[VectornavNode] " << e.what() << " Shutting down...\n");
    shutdown();
  }
}

void VectornavNode::run()
{
  ros::spin();
}

void VectornavNode::loadParameters()
{
  // TODO - check if parameters shouldn't be taken with private node handle
  board_frame_id = ros_common_functions::getRosParam<std::string>("board_frame_id");
  antenna_a_frame_id = ros_common_functions::getRosParam<std::string>("antenna_a_frame_id");
  antenna_b_frame_id = ros_common_functions::getRosParam<std::string>("antenna_b_frame_id");

  serial_port = ros_common_functions::getRosParam<std::string>("serial_port");
  serial_baud = ros_common_functions::getRosParam<int>("serial_baud");

  async_output_rate = ros_common_functions::getRosParam<int>("async_output_rate");
  fixed_imu_rate = ros_common_functions::getRosParam<int>("fixed_imu_rate");
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

void VectornavNode::connectToVectornav(vn::sensors::BinaryOutputRegister& bor,
                                       vn::sensors::VnSensor::AsyncPacketReceivedHandler callback_function)
{
  vs.setResponseTimeoutMs(1000);  // Wait for up to 1000 ms for response
  vs.setRetransmitDelayMs(50);    // Retransmit every 50 ms
  vs.connect(serial_port, serial_baud);

  if (vs.verifySensorConnectivity())
  {
    throw "Connectivity verification failed. ";
  }

  vs.writeAsyncDataOutputFrequency(async_output_rate);
  vs.writeBinaryOutput1(bor);
  vs.writeAsyncDataOutputFrequency(async_output_rate);
  vs.registerAsyncPacketReceivedHandler(nullptr, callback_function);
}

void VectornavNode::vectornavCallback(void* user_data, vn::protocol::uart::Packet& packet, size_t index)
{
}

void VectornavNode::shutdown()
{
  vs.unregisterAsyncPacketReceivedHandler();
  ros::Duration(0.5).sleep();
  vs.disconnect();
  ros::Duration(0.5).sleep();
  ros::shutdown();
}

double inline VectornavNode::gauss_to_tesla(double gauss)
{
  return 0.0001 * gauss;
}
