#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>


namespace carla {

class ItsInterface {

  public:
    ItsInterface();

  private:
    void timerCallback(const ros::TimerEvent& event);
    void messageCallback(const std_msgs::String& msg);

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;


    ros::Publisher pub_;
    ros::Subscriber sub_;

    ros::Timer timer_;

    float parameter_float_;
    bool parameter_bool_;
    std::string parameter_string_;

    bool create_publisher_;
    bool create_subscriber_;

};


}  // end of namespace carla
