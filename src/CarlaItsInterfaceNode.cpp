#include <CarlaItsInterfaceNode.h>


namespace carla {


ItsInterface::ItsInterface() {
  ROS_INFO("CarlaItsInterface starting...");

  ros::spin();
}

}  // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CarlaItsInterface");

  carla::ItsInterface node;

  return 0;
}

