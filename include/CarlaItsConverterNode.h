#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <regex>

#include <etsi_its_spatem_ts_msgs/msg/spatem.hpp>
#include <etsi_its_mapem_ts_msgs/msg/mapem.hpp>

#include <derived_object_msgs/msg/object_array.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>

#include <ad2etsi_converters/Converters.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_info.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <carla_msgs/msg/carla_traffic_light_info_list.hpp>
#include <carla_msgs/msg/carla_traffic_light_status_list.hpp>
#include <perception_msgs_utils/object_access.hpp>

#define ROS_LOG_STREAM(level, ...) RCLCPP_##level##_STREAM(this->get_logger(), __VA_ARGS__)

namespace dom = derived_object_msgs::msg;
namespace gm = geometry_msgs::msg;
namespace nm = nav_msgs::msg;
namespace pi = perception_msgs::msg;
namespace sm = shape_msgs::msg;
namespace ssm = sensor_msgs::msg;

namespace cm = carla_msgs::msg;
namespace oa = perception_msgs::object_access;
namespace etsi_cam = etsi_its_cam_msgs::msg;
namespace etsi_mapem = etsi_its_mapem_ts_msgs::msg;
namespace etsi_spatem = etsi_its_spatem_ts_msgs::msg;

template <typename T>
using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;
template <typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;
namespace carla {

class ItsConverter : public rclcpp::Node {
 public:
  ItsConverter();

 private:
  bool loadParameters();
  void subscribeCustomTopics();

  void gnssCallback(const ssm::NavSatFix::ConstPtr msg, std::string actor_name);
  void vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg, std::string actor_name);
  void vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg, std::string actor_name);
  void trafficInfoCallback(const cm::CarlaTrafficLightInfoList::ConstPtr msg, std::string actor_name);
  void trafficStatusCallback(const cm::CarlaTrafficLightStatusList::ConstPtr msg, std::string actor_name);
  
  void odometryCallback(const nm::Odometry::ConstPtr msg, std::string actor_name);
  void objectsCallback(const dom::ObjectArray::ConstPtr msg);
  void customObjectsCallback(const dom::ObjectArray::ConstPtr msg, std::string topic_name);

  pi::ObjectList convertObjectArray(const dom::ObjectArray::ConstPtr msg);
  etsi_cam::CAM convertEgoDataCam(const pi::EgoData msg);
  bool transformFrame(const pi::ObjectList& msg_object_list, pi::ObjectList& msg_object_list_transformed,
                      std::string target_frame);

  // tf amd timing variables
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_cam_msg_;
  rclcpp::Time last_mapem_msg_;
  rclcpp::Time last_spatem_msg_;

  // subscriber and publisher
  Subscriber<dom::ObjectArray> sub_objects_;

  std::map<std::string, Subscriber<ssm::NavSatFix>> sub_gnss_map_;
  std::map<std::string, Subscriber<nm::Odometry>> sub_odometry_map_;
  std::map<std::string, Subscriber<cm::CarlaEgoVehicleStatus>> sub_vehicle_status_map_;
  std::map<std::string, Subscriber<cm::CarlaEgoVehicleInfo>> sub_vehicle_info_map_;
  std::map<std::string, Subscriber<dom::ObjectArray>> sub_custom_objects_map_;
  std::map<std::string, Subscriber<cm::CarlaTrafficLightInfoList>> sub_traffic_light_info_list_;
  std::map<std::string, Subscriber<cm::CarlaTrafficLightStatusList>> sub_traffic_light_status_list_;

  Publisher<pi::ObjectList> pub_objects_carla_map_;
  std::map<std::string, Publisher<pi::ObjectList>> pub_objects_map_;
  std::map<std::string, Publisher<pi::EgoData>> pub_ego_data_map_;
  std::map<std::string, Publisher<etsi_cam::CAM>> pub_etsi_cam_map_;
  std::map<std::string, Publisher<etsi_mapem::MAPEM>> pub_etsi_mapem_map_;
  std::map<std::string, Publisher<etsi_spatem::SPATEM>> pub_etsi_spatem_map_;
  std::map<std::string, Publisher<pi::ObjectList>> pub_custom_objects_map_;
  

  // ros parameters
  std::vector<std::string> ego_data_actors_;
  std::vector<std::string> object_data_actors_;
  double pos_variances_;
  double vel_variances_;
  double acc_variances_;
  double angle_variances_;
  double angle_rate_variances_;

  // ego information
  std::map<std::string, int> ego_id_map_;
  std::map<std::string, float> ego_steering_angle_map_;
  std::map<std::string, double> ego_steering_angle_max_map_;
  std::map<std::string, gm::Accel> ego_acceleration_map_;
  std::map<std::string, sm::SolidPrimitive> ego_shape_map_;
  std::map<std::string, ssm::NavSatFix> ego_gnss_map_;

  pi::EgoData msg_ego_data_;

  // set flags
  std::map<std::string, bool> ego_shape_set_map_;
  std::map<std::string, bool> ego_status_set_map_;
  std::map<std::string, bool> ego_gnss_set_map_;
  std::map<std::string, bool> ego_info_set_map_;
};

}  // end of namespace carla
