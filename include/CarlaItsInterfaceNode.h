#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>

namespace carla {

class ItsInterface {

  public:
    ItsInterface();

  private:
    ros::NodeHandle private_node_handle_;


    ros::Publisher pub_;
    ros::Subscriber sub_;
};


}  // end of namespace carla
