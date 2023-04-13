#pragma once

#include <memory>
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>

// ROS2

#ifdef MODE_ROS2
#include <rclcpp/rclcpp.hpp>

#include <perception_interfaces/object_access.hpp>
#include <derived_object_msgs/msg/object_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#define ROS_LOG_STREAM(level, ...) RCLCPP_##level##_STREAM(this->get_logger(), __VA_ARGS__)
#endif

#ifdef MODE_ROS1
#include <ros/ros.h>

#include <perception_interfaces/object_access.h>
#include <derived_object_msgs/ObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <perception_interfaces/ObjectList.h>
#include <perception_interfaces/ISCACTR.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_perception_msgs/tf2_perception_msgs.h>

#define ROS_LOG_STREAM(level, ...) ROS_##level(__VA_ARGS__)
#endif


namespace obj_acc = perception_interfaces::object_access;

#ifdef MODE_ROS1
namespace dom = derived_object_msgs;
namespace nam = nav_msgs;
namespace pin = perception_interfaces;
namespace gm = geometry_msgs;
#endif
#ifdef MODE_ROS2
namespace dom = derived_object_msgs::msg;
namespace nam = nav_msgs::msg;
namespace pin = perception_interfaces::msg;
namespace gm = geometry_msgs::msg;
#endif

namespace carla {

#ifdef MODE_ROS1
class ItsInterface {
#endif
#ifdef MODE_ROS2
class ItsInterface : public rclcpp::Node {
#endif

  public:
    ItsInterface();

  private:
    void objectsCallback(const dom::ObjectArray::ConstPtr& msg);
    void odometryCallback(const nam::Odometry& msg);
#ifdef MODE_ROS1
    ros::NodeHandle private_node_handle_;
    
    ros::Subscriber sub_objects_;
    ros::Subscriber sub_odometry_;
    ros::Publisher pub_objects_;
    ros::Publisher pub_objects_base_link_;
#endif

    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;


    pin::ObjectList msg_object_list_;

    double fov_range_ = 75.0;
    double center_to_baselink_ = 1.2645;
};


}  // end of namespace carla
