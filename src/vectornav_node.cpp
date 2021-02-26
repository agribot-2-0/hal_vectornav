#include "hal_vectornav/VectornavNode.h"

VectornavNode vectornav;

void passCallback(void* user_data, vn::protocol::uart::Packet& packet, size_t index)
{
  vectornav.vectornavCallback(user_data, packet, index);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vectornav_node");
  ros::NodeHandle nh("~");
  vectornav.initializeAndConnect(&nh, passCallback);
  vectornav.run();
}
