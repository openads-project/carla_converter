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
#include <geometry_msgs/Accel.h>
#include <shape_msgs/SolidPrimitive.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>

#define ROS_LOG_STREAM(level, ...) ROS_##level(__VA_ARGS__)

namespace dom = derived_object_msgs;
namespace nam = nav_msgs;
namespace pin = perception_interfaces;
namespace gm = geometry_msgs;
namespace sm = shape_msgs;
namespace cm = carla_msgs;

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/accel.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_info.hpp>
// #include <Matrix3x3.h>

#define ROS_LOG_STREAM(level, ...) RCLCPP_##level##_STREAM(this->get_logger(), __VA_ARGS__)

namespace dom = derived_object_msgs::msg;
namespace nam = nav_msgs::msg;
namespace pin = perception_interfaces::msg;
namespace gm = geometry_msgs::msg;
namespace sm = shape_msgs::msg;
namespace cm = carla_msgs::msg;

template<typename T>
using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;
template<typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

#endif

namespace obj_acc = perception_interfaces::object_access;


namespace carla {

#ifdef MODE_ROS1
class ItsConverter {
#endif
#ifdef MODE_ROS2
class ItsConverter : public rclcpp::Node {
#endif

  public:
    ItsConverter();

  private:
    void objectsCallback(const dom::ObjectArray::ConstPtr msg);
    void odometryCallback(const nam::Odometry::ConstPtr msg);
    void vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg);
    void vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg);
    bool loadParameters();
#ifdef MODE_ROS1
    ros::NodeHandle private_node_handle_;
    tf2_ros::Buffer tf2_buffer_;
#elif MODE_ROS2
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
#endif

    Subscriber<dom::ObjectArray> sub_objects_;
    Subscriber<nam::Odometry> sub_odometry_;
    Subscriber<cm::CarlaEgoVehicleStatus> sub_vehicle_status_;
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info_;

    Publisher<pin::ObjectList> pub_objects_carla_map_;
    Publisher<pin::ObjectList> pub_objects_ego_vehicle_;
    Publisher<pin::EgoData> pub_ego_data_;

    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    pin::ObjectList msg_object_list_;
    pin::EgoData msg_ego_data_;

    bool publish_object_list_carla_map_frame_;
    bool publish_object_list_ego_vehicle_frame_;
    bool publish_ego_data_;

    double fov_range_;
    double center_to_baselink_;

    int ego_id_;
    float ego_steering_angle_;
    gm::Accel ego_acceleration_;
    sm::SolidPrimitive ego_shape_;
    bool ego_shape_set_ = false;
    bool ego_status_set_ = false;
    bool ego_id_set_ = false;
};


}  // end of namespace carla
