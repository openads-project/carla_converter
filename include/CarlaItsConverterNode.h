#pragma once

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#ifdef ROS1
#include <ros/ros.h>

#include <derived_object_msgs/ObjectArray.h>
#include <geometry_msgs/Accel.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf2_perception_msgs/tf2_perception_msgs.h>

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <etsi_its_msgs_utils/cam_access.h>
#include <perception_msgs_utils/object_access.h>

#define ROS_LOG_STREAM(level, ...) ROS_##level(__VA_ARGS__)

namespace dom = derived_object_msgs;
namespace gm = geometry_msgs;
namespace nm = nav_msgs;
namespace pi = perception_msgs;
namespace sm = shape_msgs;
namespace ssm = sensor_msgs;

namespace cm = carla_msgs;
namespace etsi_cam = etsi_its_cam_msgs;
namespace ca = etsi_its_cam_msgs::access;

template<typename T>
using Subscriber = ros::Subscriber;
template<typename T>
using Publisher = ros::Publisher;

#else

#include <rclcpp/rclcpp.hpp>

#include <derived_object_msgs/msg/object_array.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>

#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_info.hpp>
#include <etsi_its_msgs_utils/cam_access.hpp>
#include <perception_msgs_utils/object_access.hpp>

#define ROS_LOG_STREAM(level, ...) RCLCPP_##level##_STREAM(this->get_logger(), __VA_ARGS__)

namespace dom = derived_object_msgs::msg;
namespace gm = geometry_msgs::msg;
namespace nm = nav_msgs::msg;
namespace pi = perception_msgs::msg;
namespace sm = shape_msgs::msg;
namespace ssm = sensor_msgs::msg;

namespace cm = carla_msgs::msg;
namespace etsi_cam = etsi_its_cam_msgs::msg;
namespace ca = etsi_its_cam_msgs::access;

template<typename T>
using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;
template<typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

#endif

namespace oa = perception_msgs::object_access;
namespace ca = etsi_its_cam_msgs::access;

namespace carla {

#ifdef ROS1
class ItsConverter
#else
class ItsConverter : public rclcpp::Node
#endif
{
  public:
    ItsConverter();

  private:
    void gnssCallback(const ssm::NavSatFix::ConstPtr msg, std::string role_name);
    void objectsCallback(const dom::ObjectArray::ConstPtr msg);
    void idealObjectsCallback(const dom::ObjectArray::ConstPtr msg, std::string role_name);
    void odometryCallback(const nm::Odometry::ConstPtr msg, std::string role_name);
    void vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg, std::string role_name);
    void vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg, std::string role_name);
    bool loadParameters();

    etsi_cam::CAM convertToEtsiCam(const pi::EgoData& ego_data, ssm::NavSatFix gnss);
    pi::ObjectList convertObjectArray(const dom::ObjectArray::ConstPtr msg);
    pi::ObjectList transformFrame(pi::ObjectList& msg_object_list, std::string role_name);

#ifdef ROS1
    ros::NodeHandle private_node_handle_;
    tf2_ros::Buffer tf2_buffer_;
#else
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
#endif

    Subscriber<dom::ObjectArray> sub_objects_;

    std::map<std::string, Subscriber<ssm::NavSatFix>> sub_gnss_map_;
    std::map<std::string, Subscriber<nm::Odometry>> sub_odometry_map_;
    std::map<std::string, Subscriber<cm::CarlaEgoVehicleStatus>> sub_vehicle_status_map_;
    std::map<std::string, Subscriber<cm::CarlaEgoVehicleInfo>> sub_vehicle_info_map_;
    std::map<std::string, Subscriber<dom::ObjectArray>> sub_ideal_objects_map_;

    Publisher<pi::ObjectList> pub_objects_carla_map_;
    std::map<std::string, Publisher<pi::ObjectList>> pub_objects_map_;
    std::map<std::string, Publisher<pi::EgoData>> pub_ego_data_map_;
    std::map<std::string, Publisher<etsi_cam::CAM>> pub_etsi_cam_map_;
    std::map<std::string, Publisher<pi::ObjectList>> pub_ideal_objects_map_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    pi::EgoData msg_ego_data_;

    // ros parameters
    std::vector<std::string> role_names_;
    std::vector<std::string> its_stations_;
    double pos_variances_;
    double vel_variances_;
    double acc_variances_;
    double yaw_variances_;
    double yaw_rate_variances_;

    // ego information
    std::map<std::string, int> ego_id_map_;
    std::map<std::string, float> ego_steering_angle_map_;
    std::map<std::string, double> ego_steering_angle_max_map_;
    std::map<std::string, gm::Accel> ego_acceleration_map_;
    std::map<std::string, sm::SolidPrimitive> ego_shape_map_;
    std::map<std::string, ssm::NavSatFix> ego_gnss_map_;

    // set flags
    std::map<std::string, bool> show_transform_success_map_;

    std::map<std::string, bool> ego_shape_set_map_;
    std::map<std::string, bool> ego_status_set_map_;
    std::map<std::string, bool> ego_gnss_set_map_;
    std::map<std::string, bool> ego_info_set_map_;
};


}  // end of namespace carla
