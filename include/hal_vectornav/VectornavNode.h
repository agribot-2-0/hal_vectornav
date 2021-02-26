#pragma once

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include "agribot_common/CommonFunctions/common_functions.h"
#include "agribot_common/CommonFunctions/ros_common_functions.h"

#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

class VectornavNode
{
private:
  vn::sensors::VnSensor vs;

  ros::NodeHandle* nh_;

  ros::Publisher imu_ned_pub;
  ros::Publisher imu_enu_pub;
  ros::Publisher mag_ned_pub;
  ros::Publisher mag_enu_pub;
  ros::Publisher navsatfix_pub;
  ros::Publisher temp_pub;
  ros::Publisher pub_pres;

  // Parameters
  std::string ned_frame_id;
  std::string enu_frame_id;

  std::string serial_port;
  int serial_baud;

  int async_output_rate;
  int fixed_imu_rate;

  void loadParameters();
  void registerPublishers();
  vn::sensors::BinaryOutputRegister prepareBinaryOutputRegister();
  void connectToVectornav(const vn::sensors::BinaryOutputRegister& bor,
                          vn::sensors::VnSensor::AsyncPacketReceivedHandler callback_function);
  void shutdown();
  void publishImuData(const vn::math::vec4f& quaternion, const vn::math::vec3f& angular_rate,
                      const vn::math::vec3f& acceleration, const vn::math::vec3f& attitude_uncertainty);
  void publishMagneticFieldData(const vn::math::vec3f& mag);
  void publishNavSatFix(const vn::math::vec3d& lla, const float position_variance,
                        const vn::protocol::uart::InsStatus& ins_status);
  sensor_msgs::Imu nedToEnu(const sensor_msgs::Imu& ned_imu);
  sensor_msgs::MagneticField nedToEnu(const sensor_msgs::MagneticField& ned_mag);

  // Helper functions
  double inline gauss_to_tesla(double gauss);
  bool inline ins_status_mode_set(const vn::protocol::uart::InsStatus& ins_status,
                                  const vn::protocol::uart::InsStatus& bitmask);

public:
  VectornavNode() = default;
  ~VectornavNode() = default;
  void initializeAndConnect(ros::NodeHandle* nh, vn::sensors::VnSensor::AsyncPacketReceivedHandler callback_function);
  void run();
  void vectornavCallback(void* user_data, vn::protocol::uart::Packet& packet, size_t index);
  std::int8_t navSatFixStatusFromInsStatus(const vn::protocol::uart::InsStatus& ins_status);
};
