#pragma once

#include <memory>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#ifdef MODE_ROS1
#include <derived_object_msgs/ObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <perception_interfaces/ISCACTR.h>
#include <perception_interfaces/object_access.h>
#include <perception_interfaces/ObjectList.h>
#include <ros/ros.h>
#include <tf2_perception_msgs/tf2_perception_msgs.h>

#define ROS_LOG_STREAM(level, ...) ROS_##level(__VA_ARGS__)

namespace dom = derived_object_msgs;
namespace nam = nav_msgs;
namespace pin = perception_interfaces;
namespace gm = geometry_msgs;

template<typename T>
using Subscriber = ros::Subscriber;
template<typename T>
using Publisher = ros::Publisher;

#elif MODE_ROS2
#include <derived_object_msgs/msg/object_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <perception_interfaces/object_access.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define ROS_LOG_STREAM(level, ...) RCLCPP_##level##_STREAM(this->get_logger(), __VA_ARGS__)

namespace dom = derived_object_msgs::msg;
namespace nam = nav_msgs::msg;
namespace pin = perception_interfaces::msg;
namespace gm = geometry_msgs::msg;

template<typename T>
using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;
template<typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

#endif

namespace obj_acc = perception_interfaces::object_access;


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
    void objectsCallback(const dom::ObjectArray::ConstPtr &msg);
    void odometryCallback(const nam::Odometry::ConstPtr &msg);
    bool loadParameters();
#ifdef MODE_ROS1
    ros::NodeHandle private_node_handle_;
    tf2_ros::Buffer tf2_buffer_;
#elif MODE_ROS2
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
#endif

    Subscriber<dom::ObjectArray> sub_objects_;
    Subscriber<nam::Odometry> sub_odometry_;

    Publisher<pin::ObjectList> pub_objects_carla_map_;
    Publisher<pin::ObjectList> pub_objects_ego_vehicle_;
    Publisher<pin::ObjectList> pub_objects_map_;
    Publisher<pin::ObjectList> pub_objects_base_link_;

    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    pin::ObjectList msg_object_list_;

    double fov_range_ = 75.0;
    double center_to_baselink_ = 1.2645;

    bool publish_carla_map_;
    bool publish_ego_vehicle_;
    bool publish_map_;
    bool publish_base_link_;
};


}  // end of namespace carla
