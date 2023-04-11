#pragma once

#include <memory>
#include <string>

// ROS2

#ifdef MODE_ROS2
#include <perception_interfaces/object_access.hpp>
#include <rclcpp/rclcpp.hpp>
#include <derived_object_msgs/msg/object_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#endif

#ifdef MODE_ROS1
#include <ros/ros.h>

#include <derived_object_msgs/ObjectArray.h>
#include <perception_interfaces/ObjectList.h>
#include <perception_interfaces/ISCACTR.h>
#include <perception_interfaces/object_access.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_perception_msgs/tf2_perception_msgs.h>
#endif


namespace obj_acc = perception_interfaces::object_access;


namespace carla {

class ItsInterface {

  public:
    ItsInterface();

  private:
#ifdef MODE_ROS1
    void objectsCallback(const derived_object_msgs::ObjectArray::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry& msg);
    ros::NodeHandle private_node_handle_;
    
    ros::Subscriber sub_objects_;
    ros::Subscriber sub_odometry_;
    ros::Publisher pub_objects_;
    ros::Publisher pub_objects_base_link_;

#endif
#ifdef MODE_ROS2
    void objectsCallback(const derived_object_msgs::msg::ObjectArray::ConstPtr& msg);
    void odometryCallback(const nav_msgs::msg::Odometry& msg);
#endif

    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    tf2_ros::Buffer tfBuffer;

    perception_interfaces::ObjectList msg_object_list_;

    double fov_range_ = 75.0;
    double center_to_baselink_ = 1.2645;
};


}  // end of namespace carla
