// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <GeographicLib/UTMUPS.hpp>
#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <regex>

#include <derived_object_msgs/msg/object_array.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>

#include <carla_converter/etsi_conversion.hpp>
#include <carla_msgs/msg/carla_actor_list.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_info.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <carla_msgs/msg/carla_traffic_light_info_list.hpp>
#include <carla_msgs/msg/carla_traffic_light_status_list.hpp>
#include <carla_msgs/msg/carla_world_info.hpp>
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <etsi_its_cpm_ts_msgs/msg/collective_perception_message.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

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
namespace etsi_cpm = etsi_its_cpm_ts_msgs::msg;
namespace stm = std_msgs::msg;

template <typename T>
using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;
template <typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

namespace carla_converter {

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

/**
 * @brief CarlaConverter class
 */
class CarlaConverter : public rclcpp::Node {
 public:
  /**
   * @brief Creates the CARLA converter node, loads parameters, and initializes interfaces
   */
  CarlaConverter();

 private:
  /**
   * @brief Declares and loads a ROS parameter
   *
   * @param name name
   * @param param parameter variable to load into
   * @param description description
   * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
   * @param is_required whether failure to load parameter will stop node
   * @param read_only set parameter to read-only
   * @param from_value parameter range minimum
   * @param to_value parameter range maximum
   * @param step_value parameter range step
   * @param additional_constraints additional constraints description
   */
  template <typename T>
  void declareAndLoadParameter(const std::string& name,
                               T& param,
                               const std::string& description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double>& from_value = std::nullopt,
                               const std::optional<double>& to_value = std::nullopt,
                               const std::optional<double>& step_value = std::nullopt,
                               const std::string& additional_constraints = "");

  /**
   * @brief Handles reconfiguration when a parameter value is changed
   *
   * @param parameters parameters
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Sets up subscribers, publishers, timers, and tf2 to configure the node
   */
  void setup();

  /**
   * @brief Periodically scans all active topics and subscribes to any new
   *        CARLA ObjectArray topics not yet covered by a static subscription,
   *        newly appeared vehicle actors (see setupEgoDataActor), and, if
   *        cpm_topic_pattern is set, any perception_msgs ObjectList topic
   *        matching that pattern not already published by this node
   */
  void subscribeCustomTopics();

  /**
   * @brief Sets up ego_data/etsi_cam publishers for the given actor and registers it
   *        as an ego data actor if not already one. If actor_id is not set, also
   *        subscribes the odometry/vehicle_status/vehicle_info/gnss/imu topics for the
   *        actor (topic-driven actor). If actor_id is set, no such topics exist for the
   *        actor (e.g. a co-simulation-driven actor with no individual CARLA ROS bridge
   *        topics); its identity/state is instead seeded directly and it is later driven
   *        by matching /carla/objects entries (see actorListCallback, objectsCallback)
   *
   * @param actor_name name to register/publish the actor under
   * @param actor_id CARLA actor id, only set for actors with no individual topics
   */
  void setupEgoDataActor(const std::string& actor_name, std::optional<cm::CarlaEgoVehicleInfo::_id_type> actor_id = std::nullopt);

  /**
   * @brief Discovers vehicle actors from the CARLA actor list that have no individual
   *        ROS bridge topics and, based on cam_penetration_rate, registers a subset of 
   *        them as ego data actors
   *
   * @param msg incoming CarlaActorList message
   */
  void actorListCallback(const cm::CarlaActorList::ConstSharedPtr msg);

  /**
   * @brief Fills EgoData from the given kinematic state and publishes it; also attempts
   *        CAM conversion. Shared by odometry-topic-driven actors (via odometryCallback)
   *        and actors driven directly from /carla/objects (via objectsCallback)
   *
   * @param actor_name name of the CARLA actor
   * @param header header (frame/stamp) of the source message
   * @param pose pose of the actor
   * @param velocity_linear linear velocity of the actor (child frame)
   * @param yaw_rate yaw rate of the actor (child frame)
   * @param acceleration_linear linear acceleration of the actor (child frame)
   */
  void publishEgoDataAndCam(const std::string& actor_name,
                            const std_msgs::msg::Header& header,
                            const gm::Pose& pose,
                            const gm::Vector3& velocity_linear,
                            double yaw_rate,
                            const gm::Vector3& acceleration_linear);

  /**
   * @brief Stores the latest GNSS fix for the given actor
   *
   * @param msg incoming NavSatFix message
   * @param actor_name name of the CARLA actor
   */
  void gnssCallback(const ssm::NavSatFix::ConstSharedPtr msg, std::string actor_name);

  /**
   * @brief Stores the latest IMU linear acceleration for the given actor
   *
   * @param msg incoming IMU message
   * @param actor_name name of the CARLA actor
   */
  void imuCallback(const ssm::Imu::ConstSharedPtr msg, std::string actor_name);

  /**
   * @brief Stores the latest steering angle for the given actor
   *
   * @param msg incoming CarlaEgoVehicleStatus message
   * @param actor_name name of the CARLA actor
   */
  void vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstSharedPtr msg, std::string actor_name);

  /**
   * @brief Stores the vehicle id, maximum steering angle, and marks ego info as set
   *
   * @param msg incoming CarlaEgoVehicleInfo message
   * @param actor_name name of the CARLA actor
   */
  void vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstSharedPtr msg, std::string actor_name);

  /**
   * @brief Converts odometry to EgoData and publishes it; also attempts CAM conversion
   *
   * @param msg incoming Odometry message
   * @param actor_name name of the CARLA actor
   */
  void odometryCallback(const nm::Odometry::ConstSharedPtr msg, std::string actor_name);

  /**
   * @brief Converts the global CARLA object array and publishes per-actor transformed object lists
   *
   * @param msg incoming ObjectArray message
   */
  void objectsCallback(const dom::ObjectArray::ConstSharedPtr msg);

  /**
   * @brief Converts a cpm_actor's own ideal object sensor ObjectArray (/carla/<actor_name>/v2x_objectlist)
   *        to CPM and publishes it.
   *
   * @param msg incoming ObjectArray message
   * @param actor_name name of the CARLA actor
   */
  void cpmObjectsCallback(const dom::ObjectArray::ConstSharedPtr msg, std::string actor_name);

  /**
   * @brief Converts a custom ObjectArray topic and publishes the result, optionally transformed
   *
   * @param msg incoming ObjectArray message
   * @param topic_name source topic name (without the /carla/ prefix)
   */
  void customObjectsCallback(const dom::ObjectArray::ConstSharedPtr msg, std::string topic_name);

  /**
   * @brief Converts a dynamically discovered ObjectList topic (matching cpm_topic_pattern) to
   *        a CPM and publishes it
   *
   * @param msg incoming ObjectList message
   * @param topic_name source topic name
   */
  void dynamicObjectListCpmCallback(const pi::ObjectList::ConstSharedPtr msg, std::string topic_name);

  /**
   * @brief Builds the internal traffic light object list from static CARLA traffic light info
   *
   * @param msg incoming CarlaTrafficLightInfoList message
   */
  void trafficLightInfoCallback(const cm::CarlaTrafficLightInfoList::ConstSharedPtr msg);

  /**
   * @brief Updates traffic light signal states in the internal object list
   *
   * @param msg incoming CarlaTrafficLightStatusList message
   */
  void trafficLightStatusCallback(const cm::CarlaTrafficLightStatusList::ConstSharedPtr msg);

  /**
   * @brief Extracts and publishes the CARLA map name from the world info message
   *
   * @param msg incoming CarlaWorldInfo message
   */
  void worldInfoCallback(const cm::CarlaWorldInfo::ConstSharedPtr msg);

  /**
   * @brief Publishes the current traffic light object list at the configured frequency
   */
  void publishTrafficLights();

  /**
   * @brief Converts a CARLA ObjectArray to a perception_msgs ObjectList
   *
   * @param msg CARLA ObjectArray to convert
   * @return converted ObjectList
   */
  pi::ObjectList convertObjectArray(const dom::ObjectArray::ConstSharedPtr msg);

  /**
   * @brief Converts an EgoData message to an ETSI CAM message using the UTM transform
   *
   * @param msg EgoData to convert
   * @return converted CAM message
   */
  etsi_cam::CAM convertEgoDataCam(const pi::EgoData msg);

  /**
   * @brief Converts an ObjectList message to an ETSI CPM message using the UTM transform
   *
   * @param msg ObjectList to convert
   * @param station_id ETSI station id to report the CPM under
   * @return converted CollectivePerceptionMessage
   */
  etsi_cpm::CollectivePerceptionMessage convertObjectListCpm(
      const pi::ObjectList& msg,
      uint32_t station_id,
      uint8_t station_type = etsi_cpm::WrappedCpmContainer::CHOICE_CONTAINER_DATA_ORIGINATING_VEHICLE_CONTAINER);

  /**
   * @brief Resolves the UTM zone/hemisphere to convert the given frame into, shared by both the CAM
   *        and CPM conversion paths. Anchors the frame in WGS84 via its closest "utm_<zone><N|S>"
   *        ancestor in the TF tree and returns the zone natural to the frame's own position, which
   *        is the zone etsi_its_msgs derives from the reference position it stores
   *
   * @param frame_id source TF frame
   * @param stamp source message stamp
   * @param zone output UTM zone number
   * @param is_northp output UTM hemisphere flag
   * @return whether a usable UTM zone was found
   */
  bool resolveUtmZone(const std::string& frame_id, const rclcpp::Time& stamp, int& zone, bool& is_northp);

  /**
   * @brief Resolves the ETSI station id to report CPMs from a dynamically discovered ObjectList
   *        topic. Taken from the objects' sensor_id, which is the inverse of what ad2etsi's
   *        cpm2objectlist writes, and therefore preserves the station identity across a relayed
   *        object list.
   *
   * @param msg incoming ObjectList
   * @param topic_name source topic name
   * @return station id to report the CPM under
   */
  uint32_t resolveDynamicStationId(const pi::ObjectList& msg, const std::string& topic_name);

  /**
   * @brief Transforms an ObjectList into the target TF frame, falling back to parent frames
   *
   * @param msg_object_list input ObjectList in source frame
   * @param msg_object_list_transformed output ObjectList in target frame
   * @param target_frame desired TF frame name
   */
  bool transformFrame(const pi::ObjectList& msg_object_list,
                      pi::ObjectList& msg_object_list_transformed,
                      std::string target_frame);

  /**
   * @brief Auto-reconfigurable parameters for dynamic reconfiguration
   */
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter&)>>> auto_reconfigurable_params_;

  /**
   * @brief Callback handle for dynamic parameter reconfiguration
   */
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  // tf and timing variables
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_traffic_lights_;
  rclcpp::Time last_cam_msg_;
  rclcpp::Time last_cpm_msg_;

  // subscriber and publisher
  Subscriber<dom::ObjectArray> sub_objects_;
  Subscriber<cm::CarlaActorList> sub_actor_list_;
  Subscriber<cm::CarlaTrafficLightInfoList> sub_traffic_light_info_;
  Subscriber<cm::CarlaTrafficLightStatusList> sub_traffic_light_status_;
  Subscriber<cm::CarlaWorldInfo> sub_world_info_;

  std::map<std::string, Subscriber<ssm::NavSatFix>> sub_gnss_map_;
  std::map<std::string, Subscriber<ssm::Imu>> sub_imu_map_;
  std::map<std::string, Subscriber<nm::Odometry>> sub_odometry_map_;
  std::map<std::string, Subscriber<cm::CarlaEgoVehicleStatus>> sub_vehicle_status_map_;
  std::map<std::string, Subscriber<cm::CarlaEgoVehicleInfo>> sub_vehicle_info_map_;
  std::map<std::string, Subscriber<dom::ObjectArray>> sub_custom_objects_map_;
  std::map<std::string, Subscriber<dom::ObjectArray>> sub_cpm_objects_map_;
  std::map<std::string, Subscriber<pi::ObjectList>> sub_dynamic_cpm_map_;

  // station id established for each dynamically discovered CPM topic, see resolveDynamicStationId
  std::map<std::string, uint32_t> dynamic_station_id_map_;

  Publisher<pi::ObjectList> pub_objects_carla_map_;
  Publisher<pi::ObjectList> pub_traffic_lights_carla_map_;
  Publisher<stm::String> pub_map_info_;

  std::map<std::string, Publisher<pi::ObjectList>> pub_objects_map_;
  std::map<std::string, Publisher<pi::EgoData>> pub_ego_data_map_;
  std::map<std::string, Publisher<etsi_cam::CAM>> pub_etsi_cam_map_;
  std::map<std::string, Publisher<pi::ObjectList>> pub_custom_objects_map_;
  std::map<std::string, Publisher<etsi_cpm::CollectivePerceptionMessage>> pub_etsi_cpm_map_;
  std::map<std::string, Publisher<etsi_cpm::CollectivePerceptionMessage>> pub_dynamic_cpm_map_;

  // ros parameters
  std::string carla_fixed_frame_id_ = "carla_map";
  std::string ego_data_actors_string_ = "ego_vehicle";
  std::string object_list_actors_string_ = "ego_vehicle";
  std::string cpm_actors_string_ = "";
  std::vector<std::string> ego_data_actors_;
  std::vector<std::string> object_list_actors_;
  std::vector<std::string> cpm_actors_;
  std::string cpm_topic_pattern_ = "";
  double pos_variances_ = 0.0;
  double vel_variances_ = 0.0;
  double acc_variances_ = 0.0;
  double angle_variances_ = 0.0;
  double angle_rate_variances_ = 0.0;
  double steering_angle_ack_variance_ = 0.0;
  double standstill_threshold_ = 0.01;
  bool enable_traffic_lights_ = false;
  double traffic_light_frequency_ = 10.0;
  double cam_penetration_rate_ = 0.0;

  // ego information
  std::map<std::string, cm::CarlaEgoVehicleInfo::_id_type> ego_id_map_;
  std::map<std::string, float> ego_steering_angle_map_;
  std::map<std::string, double> ego_steering_angle_max_map_;
  std::map<std::string, gm::Accel> ego_acceleration_map_;
  std::map<std::string, sm::SolidPrimitive> ego_shape_map_;
  std::map<std::string, ssm::NavSatFix> ego_gnss_map_;

  // CARLA actor ids already registered as an ego data actor, via either vehicleInfoCallback
  // (topic-driven actors) or actorListCallback (actors with no individual topics)
  std::set<cm::CarlaEgoVehicleInfo::_id_type> known_vehicle_ids_;

  pi::EgoData msg_ego_data_;
  pi::ObjectList::SharedPtr msg_traffic_lights_;

  // set flags
  std::map<std::string, bool> ego_shape_set_map_;
  std::map<std::string, bool> ego_status_set_map_;
  std::map<std::string, bool> ego_gnss_set_map_;
  std::map<std::string, bool> ego_info_set_map_;
};

}  // namespace carla_converter
