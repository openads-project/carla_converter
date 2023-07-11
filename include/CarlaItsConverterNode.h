#pragma once

#include <memory>
#include <string>
#include <sstream>
#include <map>
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
namespace nm = nav_msgs;
namespace pi = perception_interfaces;
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
namespace nm = nav_msgs::msg;
namespace pi = perception_interfaces::msg;
namespace gm = geometry_msgs::msg;
namespace sm = shape_msgs::msg;
namespace cm = carla_msgs::msg;

template<typename T>
using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;
template<typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

#endif

namespace oa = perception_interfaces::object_access;


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
    void odometryCallback(const nm::Odometry::ConstPtr msg, std::string role_name);
    void vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg, std::string role_name);
    void vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg, std::string role_name);
    bool loadParameters();

#ifdef MODE_ROS1
    ros::NodeHandle private_node_handle_;
    tf2_ros::Buffer tf2_buffer_;
#elif MODE_ROS2
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
#endif

    Subscriber<dom::ObjectArray> sub_objects_;

    std::map<std::string, Subscriber<nm::Odometry>> sub_odometry_map_;
    std::map<std::string, Subscriber<cm::CarlaEgoVehicleStatus>> sub_vehicle_status_map_;
    std::map<std::string, Subscriber<cm::CarlaEgoVehicleInfo>> sub_vehicle_info_map_;

    Publisher<pi::ObjectList> pub_objects_carla_map_;

    std::map<std::string, Publisher<pi::ObjectList>> pub_objects_ego_vehicle_map_;
    std::map<std::string, Publisher<pi::EgoData>> pub_ego_data_map_;

    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    pi::ObjectList msg_object_list_;
    pi::EgoData msg_ego_data_;

    // ros parameters
    bool publish_ego_vehicle_;
    std::string role_names_string_;
    std::vector<std::string> role_names_;

    // ego information
    std::map<std::string, int> ego_id_map_;
    std::map<std::string, float> ego_steering_angle_map_;
    std::map<std::string, double> ego_steering_angle_max_map_;
    std::map<std::string, gm::Accel> ego_acceleration_map_;
    std::map<std::string, sm::SolidPrimitive> ego_shape_map_;

    // set flags
    std::map<std::string, bool> show_transform_success_map_;

    std::map<std::string, bool> ego_shape_set_map_;
    std::map<std::string, bool> ego_status_set_map_;
    std::map<std::string, bool> ego_info_set_map_;
};


}  // end of namespace carla
