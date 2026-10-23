// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <carla_converter/carla_converter.hpp>

namespace carla_converter {

namespace {

const std::string kCarlaObjectsTopic = "/carla/objects";
const std::string kCarlaActorListTopic = "/carla/actor_list";
const std::string kCarlaTrafficLightsInfoTopic = "/carla/traffic_lights/info";
const std::string kCarlaTrafficLightsStatusTopic = "/carla/traffic_lights/status";
const std::string kCarlaWorldInfoTopic = "/carla/world_info";

}  // namespace

CarlaConverter::CarlaConverter() : Node("carla_converter") {
  // declare and load parameters
  this->declareAndLoadParameter("ego_data_actors", ego_data_actors_string_,
                                "Comma-separated list of actor names to publish ego data for", false, false, true);
  this->declareAndLoadParameter("object_list_actors", object_list_actors_string_,
                                "Comma-separated list of actor names to publish object lists for", false, false, true);
  this->declareAndLoadParameter("cpm_actors", cpm_actors_string_,
                                "Comma-separated subset of object_list_actors to publish CPM messages. Each actor's CPM is "
                                "sourced from its own ideal object sensor topic (/carla/<actor_name>/v2x_objectlist)."
                                "Use cpm_topic_pattern instead if an actor has no such topic but publishes a custom"
                                "object list.",
                                false, false, true);
  this->declareAndLoadParameter("cpm_topic_pattern", cpm_topic_pattern_,
                                "Regex matched against discovered perception_msgs/ObjectList topics; matching topics not "
                                "already published by this node get an additional CPM conversion and publisher.",
                                false, false, true);
  this->declareAndLoadParameter("pos_variances", pos_variances_, "Position covariance value", true, false, false);
  this->declareAndLoadParameter("vel_variances", vel_variances_, "Velocity covariance value", true, false, false);
  this->declareAndLoadParameter("acc_variances", acc_variances_, "Acceleration covariance value", true, false, false);
  this->declareAndLoadParameter("angle_variances", angle_variances_, "Angle covariance value", true, false, false);
  this->declareAndLoadParameter("angle_rate_variances", angle_rate_variances_, "Angle rate covariance value", true, false, false);
  this->declareAndLoadParameter("standstill_threshold", standstill_threshold_,
                                "Velocity threshold to indicate standstill of vehicle [m/s]", true, false, false);
  this->declareAndLoadParameter("enable_traffic_lights", enable_traffic_lights_,
                                "Enable traffic light subscriptions and publishing", false, false, true);
  this->declareAndLoadParameter("traffic_light_frequency", traffic_light_frequency_,
                                "Publishing frequency for traffic lights in Hz", false, false, true);
  this->declareAndLoadParameter("carla_fixed_frame_id", carla_fixed_frame_id_, "Fixed frame ID used for the CARLA map");
  this->declareAndLoadParameter("cam_penetration_rate", cam_penetration_rate_,
                                "Percentage of vehicle actors (besides those in ego_data_actors, which always send CAM) "
                                "that additionally send CAM messages",
                                true, false, false, 0.0, 100.0);

  this->setup();
}

template <typename T>
void CarlaConverter::declareAndLoadParameter(const std::string& name,
                                             T& param,
                                             const std::string& description,
                                             const bool add_to_auto_reconfigurable_params,
                                             const bool is_required,
                                             const bool read_only,
                                             const std::optional<double>& from_value,
                                             const std::optional<double>& to_value,
                                             const std::optional<double>& step_value,
                                             const std::string& additional_constraints) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr (is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
      ss << "]";
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) { param = p.get_value<T>(); };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

rcl_interfaces::msg::SetParametersResult CarlaConverter::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s' to: %s", param.get_name().c_str(),
                    param.value_to_string().c_str());
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

void CarlaConverter::setup() {
  // callback for dynamic parameter configuration
  parameters_callback_ =
      this->add_on_set_parameters_callback(std::bind(&CarlaConverter::parametersCallback, this, std::placeholders::_1));

  // parse comma-separated actor name strings into vectors
  std::string actor_name;
  std::stringstream ego_data_actors_ss(ego_data_actors_string_);
  while (std::getline(ego_data_actors_ss, actor_name, ',')) {
    actor_name.erase(std::remove_if(actor_name.begin(), actor_name.end(), isspace), actor_name.end());
    ego_data_actors_.push_back(actor_name);
  }
  std::stringstream object_list_actors_ss(object_list_actors_string_);
  while (std::getline(object_list_actors_ss, actor_name, ',')) {
    actor_name.erase(std::remove_if(actor_name.begin(), actor_name.end(), isspace), actor_name.end());
    object_list_actors_.push_back(actor_name);
  }
  std::stringstream cpm_actors_ss(cpm_actors_string_);
  while (std::getline(cpm_actors_ss, actor_name, ',')) {
    actor_name.erase(std::remove_if(actor_name.begin(), actor_name.end(), isspace), actor_name.end());
    if (actor_name.empty()) continue;
    if (std::find(object_list_actors_.begin(), object_list_actors_.end(), actor_name) == object_list_actors_.end()) {
      RCLCPP_WARN(this->get_logger(), "cpm_actors entry '%s' is not in object_list_actors, ignoring it", actor_name.c_str());
      continue;
    }
    cpm_actors_.push_back(actor_name);
  }

  // setup callback functions
  auto vehicleInfoArgCallback = [this](const std::string& actor_name) {
    return [this, actor_name](const cm::CarlaEgoVehicleInfo::ConstSharedPtr msg) -> void {
      CarlaConverter::vehicleInfoCallback(msg, actor_name);
    };
  };
  auto cpmObjectsArgCallback = [this](const std::string& actor_name) {
    return [this, actor_name](const dom::ObjectArray::ConstSharedPtr msg) -> void {
      CarlaConverter::cpmObjectsCallback(msg, actor_name);
    };
  };

  // setup tf2 buffer
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_buffer_->setUsingDedicatedThread(true);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // setup qos
  rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
  qosLatching.transient_local();
  qosLatching.reliable();

  // setup subscriber and publisher
  sub_objects_ = this->create_subscription<dom::ObjectArray>(
      kCarlaObjectsTopic, 1, std::bind(&CarlaConverter::objectsCallback, this, std::placeholders::_1));
  pub_objects_carla_map_ = this->create_publisher<pi::ObjectList>("~/object_list", 1);
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_objects_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_objects_carla_map_->get_topic_name());

  // discovers vehicle actors with no individual ROS bridge topics so cam_penetration_rate can also apply
  // to them, driven from /carla/objects instead
  sub_actor_list_ = this->create_subscription<cm::CarlaActorList>(
      kCarlaActorListTopic, 1, std::bind(&CarlaConverter::actorListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_actor_list_->get_topic_name());

  if (enable_traffic_lights_) {
    sub_traffic_light_info_ = this->create_subscription<cm::CarlaTrafficLightInfoList>(
        kCarlaTrafficLightsInfoTopic, 1, std::bind(&CarlaConverter::trafficLightInfoCallback, this, std::placeholders::_1));
    sub_traffic_light_status_ = this->create_subscription<cm::CarlaTrafficLightStatusList>(
        kCarlaTrafficLightsStatusTopic, 1, std::bind(&CarlaConverter::trafficLightStatusCallback, this, std::placeholders::_1));
    pub_traffic_lights_carla_map_ = this->create_publisher<pi::ObjectList>("~/traffic_lights", 1);
    timer_traffic_lights_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / traffic_light_frequency_)),
                                              std::bind(&CarlaConverter::publishTrafficLights, this));
    RCLCPP_INFO(this->get_logger(), "Subscribed to '%s' and '%s'", sub_traffic_light_info_->get_topic_name(),
                sub_traffic_light_status_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_traffic_lights_carla_map_->get_topic_name());
  } else {
    RCLCPP_INFO(this->get_logger(), "Traffic light subscriptions and publishing are disabled.");
  }

  sub_world_info_ = this->create_subscription<cm::CarlaWorldInfo>(
      kCarlaWorldInfoTopic, qosLatching, std::bind(&CarlaConverter::worldInfoCallback, this, std::placeholders::_1));
  pub_map_info_ = this->create_publisher<stm::String>("~/map_info", qosLatching);
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_world_info_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_map_info_->get_topic_name());

  // setup subscriber and publisher for the configured ego_data_actors_
  for (size_t i = 0; i < ego_data_actors_.size(); i++) {
    setupEgoDataActor(ego_data_actors_[i]);
  }

  // setup object subscriber and publisher for object_list_actors_
  for (std::string& actor_name : object_list_actors_) {
    // setup subscriber depending on actor_name
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info = this->create_subscription<cm::CarlaEgoVehicleInfo>(
        "/carla/" + actor_name + "/vehicle_info", qosLatching, vehicleInfoArgCallback(actor_name));

    // save subscriber in map with actor_name as key
    sub_vehicle_info_map_.insert({actor_name, sub_vehicle_info});

    // setup publisher depending on actor_name
    Publisher<pi::ObjectList> pub_objects = this->create_publisher<pi::ObjectList>("~/" + actor_name + "/object_list", 1);

    // save publisher in map with actor_name as key
    pub_objects_map_.insert({actor_name, pub_objects});

    ROS_LOG_STREAM(
        INFO, "Subscribed /carla object topics for actor " << actor_name << " and publishing ~/" << actor_name << "/object_list");

    // additionally setup a CPM publisher for actors configured as CPM sources, sourced from that
    // actor's own ideal object sensor topic (/carla/<actor_name>/v2x_objectlist).
    // A custom object list topic matching cpm_topic_pattern is the other supported CPM source.
    if (std::find(cpm_actors_.begin(), cpm_actors_.end(), actor_name) != cpm_actors_.end()) {
      Publisher<etsi_cpm::CollectivePerceptionMessage> pub_cpm =
          this->create_publisher<etsi_cpm::CollectivePerceptionMessage>("~/" + actor_name + "/cpm", 1);
      pub_etsi_cpm_map_.insert({actor_name, pub_cpm});

      Subscriber<dom::ObjectArray> sub_cpm_objects = this->create_subscription<dom::ObjectArray>(
          "/carla/" + actor_name + "/v2x_objectlist", 1, cpmObjectsArgCallback(actor_name));
      sub_cpm_objects_map_.insert({actor_name, sub_cpm_objects});

      ROS_LOG_STREAM(INFO, "Publishing CPM for actor " << actor_name << " to ~/" << actor_name << "/cpm, sourced from '/carla/"
                                                       << actor_name << "/v2x_objectlist'");
    }
  }

  // create 1s timer to subscribe to custom topics
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&CarlaConverter::subscribeCustomTopics, this));

  // init etsi msg timestamp
  last_cam_msg_ = this->now();
  last_cpm_msg_ = this->now();

  RCLCPP_INFO(this->get_logger(), "carla_converter running...");
}

void CarlaConverter::setupEgoDataActor(const std::string& actor_name, std::optional<cm::CarlaEgoVehicleInfo::_id_type> actor_id) {
  // skip if already fully set up (e.g. re-discovered actor)
  if (pub_ego_data_map_.find(actor_name) != pub_ego_data_map_.end()) return;

  // actors not driven by carla
  if (actor_id.has_value()) {
    ego_id_map_[actor_name] = *actor_id;
    ego_info_set_map_[actor_name] = true;
    ego_status_set_map_[actor_name] = true;
    // no vehicle_status/vehicle_info feed available for these actors, so steering angle
    // is left at 0 for now; could be approximated later from yaw rate & speed if needed
    ego_steering_angle_map_[actor_name] = 0.0;
    ego_steering_angle_max_map_[actor_name] = 0.0;
    known_vehicle_ids_.insert(*actor_id);
  } else {
    auto odometryArgCallback = [this](const std::string& actor_name) {
      return [this, actor_name](const nm::Odometry::ConstSharedPtr msg) -> void {
        CarlaConverter::odometryCallback(msg, actor_name);
      };
    };
    auto vehicleStatusArgCallback = [this](const std::string& actor_name) {
      return [this, actor_name](const cm::CarlaEgoVehicleStatus::ConstSharedPtr msg) -> void {
        CarlaConverter::vehicleStatusCallback(msg, actor_name);
      };
    };
    auto vehicleInfoArgCallback = [this](const std::string& actor_name) {
      return [this, actor_name](const cm::CarlaEgoVehicleInfo::ConstSharedPtr msg) -> void {
        CarlaConverter::vehicleInfoCallback(msg, actor_name);
      };
    };
    auto gnssArgCallback = [this](const std::string& actor_name) {
      return
          [this, actor_name](const ssm::NavSatFix::ConstSharedPtr msg) -> void { CarlaConverter::gnssCallback(msg, actor_name); };
    };
    auto imuArgCallback = [this](const std::string& actor_name) {
      return [this, actor_name](const ssm::Imu::ConstSharedPtr msg) -> void { CarlaConverter::imuCallback(msg, actor_name); };
    };

    rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
    qosLatching.transient_local();
    qosLatching.reliable();

    Subscriber<nm::Odometry> sub_odometry =
        this->create_subscription<nm::Odometry>("/carla/" + actor_name + "/odometry", 1, odometryArgCallback(actor_name));
    Subscriber<cm::CarlaEgoVehicleStatus> sub_vehicle_status = this->create_subscription<cm::CarlaEgoVehicleStatus>(
        "/carla/" + actor_name + "/vehicle_status", 1, vehicleStatusArgCallback(actor_name));
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info = this->create_subscription<cm::CarlaEgoVehicleInfo>(
        "/carla/" + actor_name + "/vehicle_info", qosLatching, vehicleInfoArgCallback(actor_name));
    Subscriber<ssm::NavSatFix> sub_gnss =
        this->create_subscription<ssm::NavSatFix>("/carla/" + actor_name + "/gnss", 1, gnssArgCallback(actor_name));
    Subscriber<ssm::Imu> sub_imu =
        this->create_subscription<ssm::Imu>("/carla/" + actor_name + "/imu", 1, imuArgCallback(actor_name));

    // save subscriber in map with actor_name as key
    sub_odometry_map_.insert({actor_name, sub_odometry});
    sub_vehicle_status_map_.insert({actor_name, sub_vehicle_status});
    sub_vehicle_info_map_.insert({actor_name, sub_vehicle_info});
    sub_gnss_map_.insert({actor_name, sub_gnss});
    sub_imu_map_.insert({actor_name, sub_imu});
  }

  // setup publisher depending on actor_name
  Publisher<pi::EgoData> pub_ego_data = this->create_publisher<pi::EgoData>("~/" + actor_name + "/ego_data", 1);
  Publisher<etsi_cam::CAM> pub_etsi_cam = this->create_publisher<etsi_cam::CAM>("~/" + actor_name + "/etsi_cam", 1);

  // save publisher in map with actor_name as key
  pub_ego_data_map_.insert({actor_name, pub_ego_data});
  pub_etsi_cam_map_.insert({actor_name, pub_etsi_cam});

  // convertObjectArray() looks up each ego actor's shape via ego_data_actors_,
  // so actors added dynamically (not from the ego_data_actors parameter) must
  // be registered here too
  if (std::find(ego_data_actors_.begin(), ego_data_actors_.end(), actor_name) == ego_data_actors_.end()) {
    ego_data_actors_.push_back(actor_name);
  }

  if (actor_id.has_value()) {
    ROS_LOG_STREAM(INFO, "Registered actor " << actor_name << " (id " << *actor_id
                                             << ") for /carla/objects-driven ego data, publishing ~/" << actor_name
                                             << "/ego_data");
  } else {
    ROS_LOG_STREAM(
        INFO, "Subscribed /carla state topics for actor " << actor_name << " and publishing ~/" << actor_name << "/ego_data");
  }
}

void CarlaConverter::actorListCallback(const cm::CarlaActorList::ConstSharedPtr msg) {
  for (const auto& actor : msg->actors) {
    // fill CPM station IDs also for RSU units via the carla actor_list
    const bool is_configured_actor =
        std::find(object_list_actors_.begin(), object_list_actors_.end(), actor.rolename) != object_list_actors_.end() ||
        std::find(cpm_actors_.begin(), cpm_actors_.end(), actor.rolename) != cpm_actors_.end() ||
        std::find(ego_data_actors_.begin(), ego_data_actors_.end(), actor.rolename) != ego_data_actors_.end();
    if (is_configured_actor && ego_id_map_[actor.rolename] == 0) {
      ego_id_map_[actor.rolename] = actor.id;
      ROS_LOG_STREAM(INFO, "Resolved CARLA id " << actor.id << " for configured actor '" << actor.rolename << "' (type '"
                                                << actor.type << "') from the actor list");
    }

    if (actor.type.rfind("vehicle.", 0) != 0) continue;
    if (known_vehicle_ids_.find(actor.id) != known_vehicle_ids_.end()) continue;

    // guard against double registering topic-driven actors without yet arrived vehicle info toipic.
    if (std::find(ego_data_actors_.begin(), ego_data_actors_.end(), actor.rolename) != ego_data_actors_.end()) continue;

    // deterministic per-actor percentile in [0, 100)
    double actor_percentile = static_cast<double>(std::hash<cm::CarlaEgoVehicleInfo::_id_type>{}(actor.id) % 10000) / 100.0;
    if (actor_percentile >= cam_penetration_rate_) continue;

    std::string actor_name = actor.rolename + "_" + std::to_string(actor.id);
    setupEgoDataActor(actor_name, actor.id);
    ROS_LOG_STREAM(INFO, "Auto-enabled CAM for co-simulation vehicle actor "
                             << actor_name << " (cam_penetration_rate: " << cam_penetration_rate_ << ")");
  }
}

void CarlaConverter::subscribeCustomTopics() {
  auto customObjectsArgCallback = [this](const std::string& topic_name) {
    return [this, topic_name](const dom::ObjectArray::ConstSharedPtr msg) -> void {
      CarlaConverter::customObjectsCallback(msg, topic_name);
    };
  };
  auto dynamicCpmArgCallback = [this](const std::string& topic_name) {
    return [this, topic_name](const pi::ObjectList::ConstSharedPtr msg) -> void {
      CarlaConverter::dynamicObjectListCpmCallback(msg, topic_name);
    };
  };
  // iterate over all topics
  std::map<std::string, std::vector<std::string>> topics = this->get_topic_names_and_types();
  for (const auto& topic : topics) {
    std::string topic_name = topic.first;
    std::string topic_type = topic.second[0];

    if (topic_type == "derived_object_msgs/msg/ObjectArray") {
      // skip the global feed and any per-actor "ideal" object sensor topic (/carla/<actor>/v2x_objectlist)
      std::regex pattern_objects("/carla/.*\\/objects");
      if (topic_name == kCarlaObjectsTopic || std::regex_match(topic_name, pattern_objects)) continue;

      // remove "/carla/"
      size_t pos = topic_name.find("/carla/");
      if (pos != std::string::npos) {
        topic_name.erase(pos, 7);
      }

      // skip if topic is already subscribed
      if (sub_custom_objects_map_.find(topic_name) != sub_custom_objects_map_.end()) continue;

      Subscriber<dom::ObjectArray> sub_custom_objects =
          this->create_subscription<dom::ObjectArray>("/carla/" + topic_name, 1, customObjectsArgCallback(topic_name));
      sub_custom_objects_map_.insert({topic_name, sub_custom_objects});

      Publisher<pi::ObjectList> pub_custom_objects = this->create_publisher<pi::ObjectList>("~/" + topic_name, 1);
      pub_custom_objects_map_.insert({topic_name, pub_custom_objects});

      ROS_LOG_STREAM(INFO, "Subscribed custom topic /carla/" << topic_name << " and publishing ~/" << topic_name);

    } else if (topic_type == "carla_msgs/msg/CarlaEgoVehicleInfo") {
      // discover vehicle actors not covered by ego_data_actors_/object_list_actors_ yet
      std::smatch match;
      std::regex pattern_vehicle_info(R"(/carla/([^/]+)/vehicle_info)");
      if (!std::regex_match(topic_name, match, pattern_vehicle_info)) continue;
      std::string actor_name = match[1];

      // skip if actor is already known (configured or previously discovered)
      if (sub_vehicle_info_map_.find(actor_name) != sub_vehicle_info_map_.end()) continue;

      // deterministic per-actor percentile in [0, 100) so the same actor
      // always gets the same outcome across restarts/reconfigures
      double actor_percentile = static_cast<double>(std::hash<std::string>{}(actor_name) % 10000) / 100.0;
      if (actor_percentile < cam_penetration_rate_) {
        setupEgoDataActor(actor_name);
        ROS_LOG_STREAM(INFO, "Auto-enabled CAM for discovered vehicle actor "
                                 << actor_name << " (cam_penetration_rate: " << cam_penetration_rate_ << ")");
      }

    } else if (topic_type == "perception_msgs/msg/ObjectList") {
      // dynamic ObjectList -> CPM discovery is opt-in
      if (cpm_topic_pattern_.empty()) continue;

      // skip if topic is already subscribed
      if (sub_dynamic_cpm_map_.find(topic_name) != sub_dynamic_cpm_map_.end()) continue;

      // skip actors already converted to CPM directly via cpm_actors
      bool is_own_cpm_topic = false;
      for (const auto& entry : pub_etsi_cpm_map_) {
        auto pub_objects_it = pub_objects_map_.find(entry.first);
        if (pub_objects_it != pub_objects_map_.end() && pub_objects_it->second->get_topic_name() == topic_name) {
          is_own_cpm_topic = true;
          break;
        }
      }
      if (is_own_cpm_topic) continue;

      // match against the configured pattern
      std::regex pattern_cpm_topic(cpm_topic_pattern_);
      if (!std::regex_match(topic_name, pattern_cpm_topic)) continue;

      // derive the mirrored output topic name (.../object_list -> .../cpm, else append /cpm)
      std::string cpm_topic_suffix = topic_name;
      if (!cpm_topic_suffix.empty() && cpm_topic_suffix.front() == '/') cpm_topic_suffix.erase(0, 1);
      const std::string object_list_suffix = "object_list";
      if (cpm_topic_suffix.size() >= object_list_suffix.size() &&
          cpm_topic_suffix.compare(cpm_topic_suffix.size() - object_list_suffix.size(), object_list_suffix.size(),
                                   object_list_suffix) == 0) {
        cpm_topic_suffix.replace(cpm_topic_suffix.size() - object_list_suffix.size(), object_list_suffix.size(), "cpm");
      } else {
        cpm_topic_suffix += "/cpm";
      }

      Subscriber<pi::ObjectList> sub_dynamic_cpm =
          this->create_subscription<pi::ObjectList>(topic_name, 1, dynamicCpmArgCallback(topic_name));
      sub_dynamic_cpm_map_.insert({topic_name, sub_dynamic_cpm});

      Publisher<etsi_cpm::CollectivePerceptionMessage> pub_dynamic_cpm =
          this->create_publisher<etsi_cpm::CollectivePerceptionMessage>("~/" + cpm_topic_suffix, 1);
      pub_dynamic_cpm_map_.insert({topic_name, pub_dynamic_cpm});

      ROS_LOG_STREAM(INFO, "Subscribed discovered ObjectList topic '"
                               << topic_name << "' (matches cpm_topic_pattern) and publishing ~/" << cpm_topic_suffix);
    }
  }
}

void CarlaConverter::gnssCallback(const ssm::NavSatFix::ConstSharedPtr msg, std::string actor_name) {
  // get gnss position from actor_name vehicle
  ego_gnss_map_[actor_name] = *msg;
  ego_gnss_set_map_[actor_name] = true;
}

void CarlaConverter::imuCallback(const ssm::Imu::ConstSharedPtr msg, std::string actor_name) {
  // get imu acceleration from actor_name vehicle
  ego_acceleration_map_[actor_name].linear = msg->linear_acceleration;
}

void CarlaConverter::vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstSharedPtr msg, std::string actor_name) {
  // get steering_angle and acceleration from actor_name vehicle
  ego_steering_angle_map_[actor_name] = msg->control.steer;
  ego_status_set_map_[actor_name] = true;
}

void CarlaConverter::vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstSharedPtr msg, std::string actor_name) {
  // get id from actor_name vehicle
  ego_id_map_[actor_name] = msg->id;
  ego_steering_angle_max_map_[actor_name] = 0.0;
  for (const auto& wheel : msg->wheels) {
    ego_steering_angle_max_map_[actor_name] =
        std::max(ego_steering_angle_max_map_[actor_name], static_cast<double>(wheel.max_steer_angle));
  }
  ego_info_set_map_[actor_name] = true;
  // marks this actor's id as covered by the topic-driven path, so actorListCallback
  // does not also register it via the /carla/objects-driven path
  known_vehicle_ids_.insert(msg->id);
}

void CarlaConverter::trafficLightInfoCallback(const cm::CarlaTrafficLightInfoList::ConstSharedPtr msg) {
  try {
    msg_traffic_lights_ = std::make_shared<pi::ObjectList>();

    for (auto& traffic_light : msg->traffic_lights) {
      pi::Object pi_light;
      pi_light.id = traffic_light.id;
      pi_light.existence_probability = 1.0;
      oa::initializeState(pi_light, pi::TRAFFICLIGHT::MODEL_ID);
      oa::setPose(pi_light.state, traffic_light.transform);
      oa::setTrafficLightType(pi_light.state, pi::TRAFFICLIGHT::TYPE_UNKNOWN);
      oa::setTrafficLightState(pi_light.state, pi::TRAFFICLIGHT::STATE_UNKNOWN);

      msg_traffic_lights_->objects.push_back(pi_light);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Skip TrafficLightInfo processing: %s", e.what());
  }
}

/**
 * @brief Converts a CARLA traffic light status value to a perception traffic light state
 *
 * @param status_carla CARLA traffic light status value
 * @return matching perception traffic light state, or unknown if unsupported
 */
int convert_traffic_status(const int status_carla) {
  switch (status_carla) {
    case carla_msgs::msg::CarlaTrafficLightStatus::RED:
      return pi::TRAFFICLIGHT::STATE_RED;
    case carla_msgs::msg::CarlaTrafficLightStatus::YELLOW:
      return pi::TRAFFICLIGHT::STATE_YELLOW;
    case carla_msgs::msg::CarlaTrafficLightStatus::GREEN:
      return pi::TRAFFICLIGHT::STATE_GREEN;
    case carla_msgs::msg::CarlaTrafficLightStatus::UNKNOWN:
      return pi::TRAFFICLIGHT::STATE_UNKNOWN;
    default:
      return pi::TRAFFICLIGHT::STATE_UNKNOWN;
  }
}

void CarlaConverter::trafficLightStatusCallback(const cm::CarlaTrafficLightStatusList::ConstSharedPtr msg) {
  try {
    if (!msg_traffic_lights_ || msg_traffic_lights_->objects.empty()) {
      RCLCPP_WARN(this->get_logger(), "No traffic lights available to process status.");
      return;
    }

    for (const auto carla_traffic_light : msg->traffic_lights) {
      for (auto& pi_traffic_light : msg_traffic_lights_->objects) {
        if (pi_traffic_light.id == carla_traffic_light.id) {
          oa::setTrafficLightState(pi_traffic_light.state, convert_traffic_status(carla_traffic_light.state));
          break;
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Skip TrafficLightStatus processing: %s", e.what());
  }
}

void CarlaConverter::worldInfoCallback(const cm::CarlaWorldInfo::ConstSharedPtr msg) {
  const std::string current_map_name = msg->map_name;
  std::smatch match;
  std::string carla_map_name;

  // check if the string matches the default pattern
  std::regex pattern_default_map(R"(Carla/Maps/([^/]+))");
  if (std::regex_match(current_map_name, match, pattern_default_map)) {
    carla_map_name = match[1];
  }

  // check if the string matches the custom pattern
  std::regex pattern_custom_map(R"((.+)/Maps/([^/]+)/\2)");
  if (std::regex_match(current_map_name, match, pattern_custom_map)) {
    carla_map_name = match[2];
  }

  if (carla_map_name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Wrong format of CARLA map name: '%s'", current_map_name.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received world info for map '%s'.", carla_map_name.c_str());

  stm::String map_info_msg;
  map_info_msg.data = carla_map_name;
  pub_map_info_->publish(map_info_msg);
}

void CarlaConverter::publishTrafficLights() {
  if (msg_traffic_lights_ == nullptr) {
    RCLCPP_INFO(this->get_logger(), "No traffic lights available to publish.");
  } else {
    msg_traffic_lights_->header.frame_id = carla_fixed_frame_id_;
    msg_traffic_lights_->header.stamp = this->now();
    pub_traffic_lights_carla_map_->publish(*msg_traffic_lights_);
  }
}

void CarlaConverter::odometryCallback(const nm::Odometry::ConstSharedPtr msg, std::string actor_name) {
  if (ego_shape_set_map_[actor_name] && ego_status_set_map_[actor_name]) {
    publishEgoDataAndCam(actor_name, msg->header, msg->pose.pose, msg->twist.twist.linear, msg->twist.twist.angular.z,
                         ego_acceleration_map_[actor_name].linear);
  }
}

void CarlaConverter::publishEgoDataAndCam(const std::string& actor_name,
                                          const std_msgs::msg::Header& header,
                                          const gm::Pose& pose,
                                          const gm::Vector3& velocity_linear,
                                          double yaw_rate,
                                          const gm::Vector3& acceleration_linear) {
  // map ego data from CARLA to the perception_msgs EgoData format
  msg_ego_data_.header = header;

  // get euler angles
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Quaternion quat_tf;
  tf2::fromMsg(pose.orientation, quat_tf);
  tf2::Matrix3x3 matrix(quat_tf);
  matrix.getRPY(roll, pitch, yaw);

  // fill state
  oa::initializeState(msg_ego_data_, pi::EGO::MODEL_ID);
  msg_ego_data_.state.header = msg_ego_data_.header;
  oa::setPose(msg_ego_data_.state, pose);
  oa::setZ(msg_ego_data_, pose.position.z + ego_shape_map_[actor_name].dimensions[2] / 2.0);

  oa::setYawRate(msg_ego_data_.state, yaw_rate);          // twist is defined in child frame (no transformation needed)
  oa::setVelocity(msg_ego_data_.state, velocity_linear);  // twist is defined in child frame (no transformation needed)
  oa::setAcceleration(msg_ego_data_.state, acceleration_linear);

  // SteeringAngleMax is given in rad (contrary to the description in the documentation)
  // https://github.com/carla-simulator/ros-bridge/blob/e9063d97ff5a724f76adbb1b852dc71da1dcfeec/carla_ros_bridge/src/carla_ros_bridge/ego_vehicle.py#L145C46-L145C81
  oa::setSteeringAngleAck(msg_ego_data_.state, -ego_steering_angle_map_[actor_name] * ego_steering_angle_max_map_[actor_name]);
  oa::setStandstill(msg_ego_data_.state, std::sqrt(pow(velocity_linear.x, 2) + pow(velocity_linear.y, 2) +
                                                   pow(velocity_linear.z, 2)) <= standstill_threshold_);

  // reference point for object position
  msg_ego_data_.state.reference_point.value = pi::ObjectReferencePoint::GEOMETRIC_CENTER;

  // fill variances
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::X, pi::EGO::X, pos_variances_);
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::Y, pi::EGO::Y, pos_variances_);
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::Z, pi::EGO::Z, pos_variances_);
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::VEL_LON, pi::EGO::VEL_LON, vel_variances_);
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::VEL_LAT, pi::EGO::VEL_LAT, vel_variances_);
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::ACC_LON, pi::EGO::ACC_LON, acc_variances_);
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::ACC_LAT, pi::EGO::ACC_LAT, acc_variances_);
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::ROLL, pi::EGO::ROLL, angle_variances_);
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::PITCH, pi::EGO::PITCH, angle_variances_);
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::YAW, pi::EGO::YAW, angle_variances_);
  oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::YAW_RATE, pi::EGO::YAW_RATE, angle_rate_variances_);

  // set vehicle id and dimensions
  msg_ego_data_.vehicle_id = ego_id_map_[actor_name];
  msg_ego_data_.length = ego_shape_map_[actor_name].dimensions[0];
  msg_ego_data_.width = ego_shape_map_[actor_name].dimensions[1];
  msg_ego_data_.height = ego_shape_map_[actor_name].dimensions[2];

  // publish ego_data in carla_map frame
  pub_ego_data_map_[actor_name]->publish(msg_ego_data_);

  // try to convert ego_data to CAM
  try {
    etsi_cam::CAM msg_cam = convertEgoDataCam(msg_ego_data_);
    pub_etsi_cam_map_[actor_name]->publish(msg_cam);
  } catch (const std::exception& e) {
    if (this->now() - last_cam_msg_ > rclcpp::Duration(1, 0)) {
      RCLCPP_WARN(this->get_logger(),
                  "Skip EgoData to CAM conversion: %s (Make sure that utm frame exist and unix time is used!)", e.what());
      last_cam_msg_ = this->now();
    }
  }
}

void CarlaConverter::objectsCallback(const dom::ObjectArray::ConstSharedPtr msg) {
  pi::ObjectList msg_object_list_ = CarlaConverter::convertObjectArray(msg);

  // publish object_list in carla_map frame
  pub_objects_carla_map_->publish(msg_object_list_);

  // ego_data_actors_ with no odometry subscription have no individual /carla/<actor_name>/...
  // topics and are instead driven directly from this /carla/objects feed
  for (std::string& actor_name : ego_data_actors_) {
    if (sub_odometry_map_.find(actor_name) != sub_odometry_map_.end()) continue;
    if (!ego_shape_set_map_[actor_name]) continue;

    for (size_t i = 0; i < msg->objects.size(); i++) {
      if (!msg->objects[i].object_classified) continue;
      if (msg->objects[i].id != ego_id_map_[actor_name]) continue;

      publishEgoDataAndCam(actor_name, msg->objects[i].header, msg->objects[i].pose, msg->objects[i].twist.linear,
                           msg->objects[i].twist.angular.z, msg->objects[i].accel.linear);
      break;
    }
  }

  // iterate over each actor_name
  for (std::string& actor_name : object_list_actors_) {
    pi::ObjectList msg_object_list_copy = msg_object_list_;

    // Set increasing sensor ID per role name. As `ego_data_actors_` doesnt change, this assigns a fixed sensor ID to each role.
    for (auto& object : msg_object_list_copy.objects) {
      object.state.sensor_id[0] = ego_id_map_[actor_name];
    }

    // remove element with the actor_name id
    msg_object_list_copy.objects.erase(
        std::remove_if(msg_object_list_copy.objects.begin(), msg_object_list_copy.objects.end(),
                       [this, actor_name](const pi::Object& obj) { return obj.id == ego_id_map_[actor_name]; }),
        msg_object_list_copy.objects.end());

    // transform the object_list from carla_map to actor_name frame
    pi::ObjectList msg_object_list_transformed;
    if (CarlaConverter::transformFrame(msg_object_list_copy, msg_object_list_transformed, actor_name)) {
      pub_objects_map_[actor_name]->publish(msg_object_list_transformed);
    }
  }
}

void CarlaConverter::cpmObjectsCallback(const dom::ObjectArray::ConstSharedPtr msg, std::string actor_name) {
  pi::ObjectList msg_object_list = CarlaConverter::convertObjectArray(msg);

  // Set fixed sensor ID and remove the actor's own entry, same as for the ground-truth object_list_actors_ path
  for (auto& object : msg_object_list.objects) {
    object.state.sensor_id[0] = ego_id_map_[actor_name];
  }
  msg_object_list.objects.erase(
      std::remove_if(msg_object_list.objects.begin(), msg_object_list.objects.end(),
                     [this, actor_name](const pi::Object& obj) { return obj.id == ego_id_map_[actor_name]; }),
      msg_object_list.objects.end());

  // transform the object_list from its source frame to the actor_name frame
  pi::ObjectList msg_object_list_transformed;
  if (!CarlaConverter::transformFrame(msg_object_list, msg_object_list_transformed, actor_name)) return;

  auto pub_cpm = pub_etsi_cpm_map_.find(actor_name);
  if (pub_cpm == pub_etsi_cpm_map_.end()) return;

  try {
    etsi_cpm::CollectivePerceptionMessage msg_cpm = convertObjectListCpm(msg_object_list_transformed, ego_id_map_[actor_name]);
    pub_cpm->second->publish(msg_cpm);
  } catch (const std::exception& e) {
    if (this->now() - last_cpm_msg_ > rclcpp::Duration(1, 0)) {
      RCLCPP_WARN(this->get_logger(),
                  "Skip ObjectList to CPM conversion for actor '%s': %s "
                  "(Make sure that utm frame exist and unix time is used!)",
                  actor_name.c_str(), e.what());
      last_cpm_msg_ = this->now();
    }
  }
}

void CarlaConverter::customObjectsCallback(const dom::ObjectArray::ConstSharedPtr msg, std::string topic_name) {
  pi::ObjectList msg_object_list_ = CarlaConverter::convertObjectArray(msg);
  pi::ObjectList msg_object_list_transformed;

  // transform the object_list from carla_map to topic frame if possible
  if (CarlaConverter::transformFrame(msg_object_list_, msg_object_list_transformed, topic_name)) {
    pub_custom_objects_map_[topic_name]->publish(msg_object_list_transformed);
  } else {
    pub_custom_objects_map_[topic_name]->publish(msg_object_list_);
  }
}

uint32_t CarlaConverter::resolveDynamicStationId(const pi::ObjectList& msg, const std::string& topic_name) {
  // deterministic id derived from topic name, whenever message carries no usable identity.
  const uint32_t fallback_station_id = static_cast<uint32_t>(std::hash<std::string>{}(topic_name)) | 0x80000000u;

  auto station_id_it = dynamic_station_id_map_.find(topic_name);
  if (station_id_it != dynamic_station_id_map_.end()) return station_id_it->second;

  if (msg.objects.empty()) return fallback_station_id;

  // ad2etsi's inverse conversion writes a CPM's station id into sensor_id (cpm2objectlist), so
  // reading it back out here is that exact inverse and preserves the station identity across a
  // relayed object list.
  uint64_t sensor_id = 0;
  for (const pi::Object& object : msg.objects) {
    if (object.state.sensor_id.empty() || object.state.sensor_id[0] == 0 ||
        object.state.sensor_id[0] > std::numeric_limits<uint32_t>::max() ||
        (sensor_id != 0 && object.state.sensor_id[0] != sensor_id)) {
      sensor_id = 0;
      break;
    }
    sensor_id = object.state.sensor_id[0];
  }

  const uint32_t station_id = (sensor_id == 0) ? fallback_station_id : static_cast<uint32_t>(sensor_id);
  dynamic_station_id_map_[topic_name] = station_id;

  ROS_LOG_STREAM(INFO,
                 "Using station id " << station_id << " for CPMs from '" << topic_name << "' ("
                                     << (station_id == fallback_station_id ? "derived from the topic name, objects carry no "
                                                                             "consistent sensor id"
                                                                           : "taken from the objects' sensor id")
                                     << ")");
  return station_id;
}

void CarlaConverter::dynamicObjectListCpmCallback(const pi::ObjectList::ConstSharedPtr msg, std::string topic_name) {
  try {
    etsi_cpm::CollectivePerceptionMessage msg_cpm = convertObjectListCpm(*msg, resolveDynamicStationId(*msg, topic_name));
    pub_dynamic_cpm_map_[topic_name]->publish(msg_cpm);
  } catch (const std::exception& e) {
    if (this->now() - last_cpm_msg_ > rclcpp::Duration(1, 0)) {
      RCLCPP_WARN(this->get_logger(),
                  "Skip ObjectList to CPM conversion for topic '%s': %s "
                  "(Make sure that utm frame exist and unix time is used!)",
                  topic_name.c_str(), e.what());
      last_cpm_msg_ = this->now();
    }
  }
}

pi::ObjectList CarlaConverter::convertObjectArray(const dom::ObjectArray::ConstSharedPtr msg) {
  // map the objects from the CARLA format to the perception_msgs format
  pi::ObjectList msg_object_list_;
  msg_object_list_.header = msg->header;

  for (size_t i = 0; i < msg->objects.size(); i++) {
    // only add classified objects
    if (!msg->objects[i].object_classified) continue;

    for (auto& actor_name : ego_data_actors_) {
      // only continue if ego_info_set_
      if (!ego_info_set_map_[actor_name]) continue;

      // get shape from actor_name vehicle
      if (static_cast<uint32_t>(ego_id_map_[actor_name]) == msg->objects[i].id) {
        ego_shape_map_[actor_name] = msg->objects[i].shape;
        ego_shape_set_map_[actor_name] = true;
      }
    }

    pi::Object objectTemp;
    objectTemp.id = msg->objects[i].id;
    objectTemp.existence_probability = 1.0;  // always 1.0 as source is CARLA

    // get yaw angle
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Quaternion quat_tf;
    tf2::fromMsg(msg->objects[i].pose.orientation, quat_tf);
    tf2::Matrix3x3 matrix(quat_tf);
    matrix.getRPY(roll, pitch, yaw);

    // fill state
    oa::initializeState(objectTemp, pi::HEXAMOTION::MODEL_ID);
    objectTemp.state.header = msg->objects[i].header;
    oa::setPose(objectTemp.state, msg->objects[i].pose);
    oa::setVelocityXYZYaw(objectTemp.state, msg->objects[i].twist.linear, yaw);
    oa::setAccelerationXYZYaw(objectTemp.state, msg->objects[i].accel.linear, yaw);
    oa::setRollRate(objectTemp.state, msg->objects[i].twist.angular.x);
    oa::setPitchRate(objectTemp.state, msg->objects[i].twist.angular.y);
    oa::setYawRate(objectTemp.state, msg->objects[i].twist.angular.z);
    oa::setLength(objectTemp, msg->objects[i].shape.dimensions[0]);
    oa::setWidth(objectTemp, msg->objects[i].shape.dimensions[1]);
    oa::setHeight(objectTemp, msg->objects[i].shape.dimensions[2]);

    // fill variances
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::X, pi::HEXAMOTION::X, pos_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::Y, pi::HEXAMOTION::Y, pos_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::Z, pi::HEXAMOTION::Z, pos_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::VEL_LON, pi::HEXAMOTION::VEL_LON, vel_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::VEL_LAT, pi::HEXAMOTION::VEL_LAT, vel_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::ACC_LON, pi::HEXAMOTION::ACC_LON, acc_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::ACC_LAT, pi::HEXAMOTION::ACC_LAT, acc_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::ROLL, pi::HEXAMOTION::ROLL, angle_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::PITCH, pi::HEXAMOTION::PITCH, angle_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::YAW, pi::HEXAMOTION::YAW, angle_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::ROLL_RATE, pi::HEXAMOTION::ROLL_RATE, angle_rate_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::PITCH_RATE, pi::HEXAMOTION::PITCH_RATE, angle_rate_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::YAW_RATE, pi::HEXAMOTION::YAW_RATE, angle_rate_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::LENGTH, pi::HEXAMOTION::LENGTH, pos_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::WIDTH, pi::HEXAMOTION::WIDTH, pos_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::HEIGHT, pi::HEXAMOTION::HEIGHT, pos_variances_);

    // global object list: Sensor ID 0
    objectTemp.state.sensor_id.push_back(0);

    objectTemp.state.classifications.resize(1);
    objectTemp.state.classifications[0].probability = 1.0;  // always 1.0 as source is CARLA
    switch (static_cast<int>(msg->objects[i].classification)) {
      case dom::Object::CLASSIFICATION_PEDESTRIAN:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::PEDESTRIAN;
        break;
      case dom::Object::CLASSIFICATION_BIKE:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::BICYCLE;
        break;
      case dom::Object::CLASSIFICATION_MOTORCYCLE:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::MOTORCYCLE;
        break;
      case dom::Object::CLASSIFICATION_CAR:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::CAR;
        break;
      case dom::Object::CLASSIFICATION_TRUCK:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::UTILITY;
        break;
      default:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::UNCLASSIFIED;
        break;
    }

    // set z for vehicles to center
    if (objectTemp.state.classifications[0].type != pi::ObjectClassification::PEDESTRIAN) {
      oa::setZ(objectTemp, msg->objects[i].pose.position.z + msg->objects[i].shape.dimensions[2] / 2.0);
    }

    objectTemp.state.reference_point.value = pi::ObjectReferencePoint::GEOMETRIC_CENTER;

    msg_object_list_.objects.push_back(objectTemp);
  }
  return msg_object_list_;
}

bool CarlaConverter::resolveUtmZone(const std::string& frame_id, const rclcpp::Time& stamp, int& zone, bool& is_northp) {
  // walk up the TF tree to the closest "utm_<zone><N|S>" ancestor (e.g. utm_31N -> carla_map), which
  // anchors the source frame in WGS84
  std::regex pattern_utm_frame(R"(utm_([0-9]+)([NS]))");
  std::smatch match;
  std::string frame = frame_id;
  std::string parent;

  while (!std::regex_match(frame, match, pattern_utm_frame)) {
    if (!tf2_buffer_->_getParent(frame, tf2::TimePointZero, parent)) return false;
    frame = parent;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf2_buffer_->lookupTransform(frame, frame_id, stamp);
  } catch (const tf2::TransformException&) {
    return false;
  }

  // etsi_its_msgs re-derives the zone from the reference position stored in the message and rejects
  // every perceived object whose zone differs, so the conversion has to run in the zone natural to
  // the source frame itself, not in the anchor's zone - for a CARLA map georeferenced to lat/lon 0/0
  // the two differ for anything west of or south of the map origin. The reference position is stored
  // in units of 1e-7 deg, so round to that before deriving the zone
  double lat = 0.0;
  double lon = 0.0;
  double gamma = 0.0;
  double k = 0.0;
  double easting = 0.0;
  double northing = 0.0;
  GeographicLib::UTMUPS::Reverse(std::stoi(match[1]), match[2] == "N", tf.transform.translation.x, tf.transform.translation.y,
                                 lat, lon, gamma, k);
  GeographicLib::UTMUPS::Forward(std::round(lat * 1e7) * 1e-7, std::round(lon * 1e7) * 1e-7, zone, is_northp, easting, northing);

  return tf2_buffer_->canTransform("utm_" + std::to_string(zone) + (is_northp ? "N" : "S"), frame_id, stamp);
}

etsi_cam::CAM CarlaConverter::convertEgoDataCam(const pi::EgoData msg) {
  etsi_cam::CAM msg_cam;
  int zone = 0;
  bool is_northp = true;
  if (resolveUtmZone(msg.header.frame_id, msg.header.stamp, zone, is_northp)) {
    ad2etsi::egodata2cam(msg_ego_data_, msg_cam, *tf2_buffer_.get(), zone, is_northp);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Tranformation to global utm frame not found.");
  }

  return msg_cam;
}

etsi_cpm::CollectivePerceptionMessage CarlaConverter::convertObjectListCpm(const pi::ObjectList& msg,
                                                                           uint32_t station_id,
                                                                           uint8_t station_type) {
  etsi_cpm::CollectivePerceptionMessage msg_cpm;
  int zone = 0;
  bool is_northp = true;
  if (!resolveUtmZone(msg.header.frame_id, msg.header.stamp, zone, is_northp)) {
    throw std::runtime_error("Tranformation to global utm frame not found.");
  }
  ad2etsi::objectlist2cpm(msg, msg_cpm, *tf2_buffer_.get(), station_type, station_id, "", zone, is_northp);

  return msg_cpm;
}

bool CarlaConverter::transformFrame(const pi::ObjectList& msg_object_list,
                                    pi::ObjectList& msg_object_list_transformed,
                                    std::string target_frame) {
  if (msg_object_list.objects.empty()) return false;

  try {
    while (true) {
      if (tf2_buffer_->_frameExists(target_frame)) {
        msg_object_list_transformed = tf2_buffer_->transform(msg_object_list, target_frame, tf2::durationFromSec(0.1));
        return true;
      }

      // if target_frame frame does not exist, try to transform to sub_frame
      size_t pos = target_frame.rfind("/");
      if (pos != std::string::npos) {
        target_frame = target_frame.substr(0, pos);
      } else {
        return false;
      }
    }
  } catch (tf2::TransformException& e) {
    ROS_LOG_STREAM(WARN, "Transformation from '" << target_frame << "' or its subframes is not available");
    ROS_LOG_STREAM(WARN, e.what());
    return false;
  }
}

}  // namespace carla_converter

/**
 * @brief Starts the carla_converter ROS node
 *
 * @param argc command-line argument count
 * @param argv command-line argument values
 * @return process exit code
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<carla_converter::CarlaConverter>();
  rclcpp::executors::SingleThreadedExecutor executor;
  RCLCPP_INFO(node->get_logger(), "Spinning node '%s' with %s", node->get_fully_qualified_name(), "SingleThreadedExecutor");
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
