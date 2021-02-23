#pragma once

#include <string>

#include <ros/ros.h>
#include "agribot_common/CommonFunctions/ros_common_functions.h"

#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

class VectornavNode
{
private:
  vn::sensors::VnSensor vs;

  ros::NodeHandle n;

  ros::Publisher imu_pub;
  ros::Publisher mag_pub;
  ros::Publisher gps_pub;
  ros::Publisher temp_pub;
  ros::Publisher pub_pres;

  // Parameters
  std::string board_frame_id;
  std::string antenna_a_frame_id;
  std::string antenna_b_frame_id;

  std::string serial_port;
  int serial_baud;

  int async_output_rate;
  int fixed_imu_rate;

  void loadParameters();
  vn::sensors::BinaryOutputRegister prepareBinaryOutputRegister();
  void connectToVectornav(vn::sensors::BinaryOutputRegister& bor,
                          vn::sensors::VnSensor::AsyncPacketReceivedHandler callback_function);
  void shutdown();

  // Helper functions
  double inline gauss_to_tesla(double gauss);

public:
  VectornavNode() = default;
  ~VectornavNode() = default;
  void initializeAndConnect(vn::sensors::VnSensor::AsyncPacketReceivedHandler callback_function);
  void run();
  void vectornavCallback(void* user_data, vn::protocol::uart::Packet& packet, size_t index);
};
