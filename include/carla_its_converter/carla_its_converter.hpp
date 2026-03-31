#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <regex>

#include <derived_object_msgs/msg/object_array.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>

#include <ad2etsi_converters/Converters.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_info.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <carla_msgs/msg/carla_traffic_light_info_list.hpp>
#include <carla_msgs/msg/carla_traffic_light_status_list.hpp>
#include <carla_msgs/msg/carla_world_info.hpp>
#include <perception_msgs_utils/object_access.hpp>
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
namespace stm = std_msgs::msg;

template <typename T>
using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;
template <typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

namespace carla_converter {

template <typename C> struct is_vector : std::false_type {};
template <typename T, typename A> struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;


/**
 * @brief CarlaConverter class
 */
class CarlaConverter : public rclcpp::Node {

 public:

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
   *        CARLA ObjectArray topics not yet covered by a static subscription
   */
  void subscribeCustomTopics();

  /**
   * @brief Stores the latest GNSS fix for the given actor
   *
   * @param msg incoming NavSatFix message
   * @param actor_name name of the CARLA actor
   */
  void gnssCallback(const ssm::NavSatFix::ConstPtr msg, std::string actor_name);

  /**
   * @brief Applies a low-pass filter to the IMU linear acceleration and stores the result
   *
   * @param msg incoming IMU message
   * @param actor_name name of the CARLA actor
   */
  void imuCallback(const ssm::Imu::ConstPtr msg, std::string actor_name);

  /**
   * @brief Stores the latest steering angle for the given actor
   *
   * @param msg incoming CarlaEgoVehicleStatus message
   * @param actor_name name of the CARLA actor
   */
  void vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg, std::string actor_name);

  /**
   * @brief Stores the vehicle id, maximum steering angle, and marks ego info as set
   *
   * @param msg incoming CarlaEgoVehicleInfo message
   * @param actor_name name of the CARLA actor
   */
  void vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg, std::string actor_name);

  /**
   * @brief Converts odometry to EgoData and publishes it; also attempts CAM conversion
   *
   * @param msg incoming Odometry message
   * @param actor_name name of the CARLA actor
   */
  void odometryCallback(const nm::Odometry::ConstPtr msg, std::string actor_name);

  /**
   * @brief Converts the global CARLA object array and publishes per-actor transformed object lists
   *
   * @param msg incoming ObjectArray message
   */
  void objectsCallback(const dom::ObjectArray::ConstPtr msg);

  /**
   * @brief Converts a custom ObjectArray topic and publishes the result, optionally transformed
   *
   * @param msg incoming ObjectArray message
   * @param topic_name source topic name (without the /carla/ prefix)
   */
  void customObjectsCallback(const dom::ObjectArray::ConstPtr msg, std::string topic_name);

  /**
   * @brief Builds the internal traffic light object list from static CARLA traffic light info
   *
   * @param msg incoming CarlaTrafficLightInfoList message
   */
  void trafficLightInfoCallback(const cm::CarlaTrafficLightInfoList::ConstPtr msg);

  /**
   * @brief Updates traffic light signal states in the internal object list
   *
   * @param msg incoming CarlaTrafficLightStatusList message
   */
  void trafficLightStatusCallback(const cm::CarlaTrafficLightStatusList::ConstPtr msg);

  /**
   * @brief Extracts and publishes the CARLA map name from the world info message
   *
   * @param msg incoming CarlaWorldInfo message
   */
  void worldInfoCallback(const cm::CarlaWorldInfo::ConstPtr msg);

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
  pi::ObjectList convertObjectArray(const dom::ObjectArray::ConstPtr msg);

  /**
   * @brief Converts an EgoData message to an ETSI CAM message using the UTM transform
   *
   * @param msg EgoData to convert
   * @return converted CAM message
   */
  etsi_cam::CAM convertEgoDataCam(const pi::EgoData msg);

  /**
   * @brief Transforms an ObjectList into the target TF frame, falling back to parent frames
   *
   * @param msg_object_list input ObjectList in source frame
   * @param msg_object_list_transformed output ObjectList in target frame
   * @param target_frame desired TF frame name
   */
  bool transformFrame(const pi::ObjectList& msg_object_list, pi::ObjectList& msg_object_list_transformed,
                      std::string target_frame);

 private:

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

  // subscriber and publisher
  Subscriber<dom::ObjectArray> sub_objects_;
  Subscriber<cm::CarlaTrafficLightInfoList> sub_traffic_light_info_;
  Subscriber<cm::CarlaTrafficLightStatusList> sub_traffic_light_status_;
  Subscriber<cm::CarlaWorldInfo> sub_world_info_;

  std::map<std::string, Subscriber<ssm::NavSatFix>> sub_gnss_map_;
  std::map<std::string, Subscriber<ssm::Imu>> sub_imu_map_;
  std::map<std::string, Subscriber<nm::Odometry>> sub_odometry_map_;
  std::map<std::string, Subscriber<cm::CarlaEgoVehicleStatus>> sub_vehicle_status_map_;
  std::map<std::string, Subscriber<cm::CarlaEgoVehicleInfo>> sub_vehicle_info_map_;
  std::map<std::string, Subscriber<dom::ObjectArray>> sub_custom_objects_map_;

  Publisher<pi::ObjectList> pub_objects_carla_map_;
  Publisher<pi::ObjectList> pub_traffic_lights_carla_map_;
  Publisher<stm::String> pub_map_info_;

  std::map<std::string, Publisher<pi::ObjectList>> pub_objects_map_;
  std::map<std::string, Publisher<pi::EgoData>> pub_ego_data_map_;
  std::map<std::string, Publisher<etsi_cam::CAM>> pub_etsi_cam_map_;
  std::map<std::string, Publisher<pi::ObjectList>> pub_custom_objects_map_;

  // ros parameters
  std::string ego_data_actors_string_ = "ego_vehicle";
  std::string object_data_actors_string_ = "ego_vehicle";
  std::vector<std::string> ego_data_actors_;
  std::vector<std::string> object_data_actors_;
  double pos_variances_ = oa::CONTINUOUS_STATE_COVARIANCE_INVALID;
  double vel_variances_ = oa::CONTINUOUS_STATE_COVARIANCE_INVALID;
  double acc_variances_ = oa::CONTINUOUS_STATE_COVARIANCE_INVALID;
  double angle_variances_ = oa::CONTINUOUS_STATE_COVARIANCE_INVALID;
  double angle_rate_variances_ = oa::CONTINUOUS_STATE_COVARIANCE_INVALID;
  double traffic_light_frequency_ = 10.0;
  std::string carla_fixed_frame_id_ = "carla_map";
  double acceleration_filter_alpha_ = 1.0;

  // ego information
  std::map<std::string, int> ego_id_map_;
  std::map<std::string, float> ego_steering_angle_map_;
  std::map<std::string, double> ego_steering_angle_max_map_;
  std::map<std::string, gm::Accel> ego_acceleration_map_;
  std::map<std::string, gm::Vector3> ego_acceleration_filtered_map_;
  std::map<std::string, bool> ego_acceleration_initialized_map_;
  std::map<std::string, sm::SolidPrimitive> ego_shape_map_;
  std::map<std::string, ssm::NavSatFix> ego_gnss_map_;

  pi::EgoData msg_ego_data_;
  pi::ObjectList::SharedPtr msg_traffic_lights_;

  // set flags
  std::map<std::string, bool> ego_shape_set_map_;
  std::map<std::string, bool> ego_status_set_map_;
  std::map<std::string, bool> ego_gnss_set_map_;
  std::map<std::string, bool> ego_info_set_map_;
};


}  // namespace carla_converter
