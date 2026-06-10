#include <carla_converter/carla_converter.hpp>

namespace carla_converter {


CarlaConverter::CarlaConverter() : Node("carla_converter") {

  // declare and load parameters
  this->declareAndLoadParameter("ego_data_actors", ego_data_actors_string_,
    "Comma-separated list of actor names to publish ego data for", false, false, true);
  this->declareAndLoadParameter("object_list_actors", object_list_actors_string_,
    "Comma-separated list of actor names to publish object lists for", false, false, true);
  this->declareAndLoadParameter("pos_variances", pos_variances_,
    "Position covariance value", true, false, false);
  this->declareAndLoadParameter("vel_variances", vel_variances_,
    "Velocity covariance value", true, false, false);
  this->declareAndLoadParameter("acc_variances", acc_variances_,
    "Acceleration covariance value", true, false, false);
  this->declareAndLoadParameter("angle_variances", angle_variances_,
    "Angle covariance value", true, false, false);
  this->declareAndLoadParameter("angle_rate_variances", angle_rate_variances_,
    "Angle rate covariance value", true, false, false);
  this->declareAndLoadParameter("enable_traffic_lights", enable_traffic_lights_,
    "Enable traffic light subscriptions and publishing", false, false, true);
   this->declareAndLoadParameter("traffic_light_frequency", traffic_light_frequency_,
    "Publishing frequency for traffic lights in Hz", false, false, true);
 this->declareAndLoadParameter("carla_fixed_frame_id", carla_fixed_frame_id_,
    "Fixed frame ID used for the CARLA map");

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
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}


rcl_interfaces::msg::SetParametersResult CarlaConverter::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s' to: %s",
                    param.get_name().c_str(), param.value_to_string().c_str());
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
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&CarlaConverter::parametersCallback, this, std::placeholders::_1));

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

  // setup callback functions
  auto odometryArgCallback = [this](const std::string& actor_name) {
    return [this, actor_name](const nm::Odometry::ConstPtr msg) -> void {
      CarlaConverter::odometryCallback(msg, actor_name);
    };
  };

  auto vehicleStatusArgCallback = [this](const std::string& actor_name) {
    return [this, actor_name](const cm::CarlaEgoVehicleStatus::ConstPtr msg) -> void {
      CarlaConverter::vehicleStatusCallback(msg, actor_name);
    };
  };

  auto vehicleInfoArgCallback = [this](const std::string& actor_name) {
    return [this, actor_name](const cm::CarlaEgoVehicleInfo::ConstPtr msg) -> void {
      CarlaConverter::vehicleInfoCallback(msg, actor_name);
    };
  };

  auto gnssArgCallback = [this](const std::string& actor_name) {
    return [this, actor_name](const ssm::NavSatFix::ConstPtr msg) -> void {
      CarlaConverter::gnssCallback(msg, actor_name);
    };
  };

  auto imuArgCallback = [this](const std::string& actor_name) {
    return [this, actor_name](const ssm::Imu::ConstPtr msg) -> void {
      CarlaConverter::imuCallback(msg, actor_name);
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
      "/carla/objects", 1, std::bind(&CarlaConverter::objectsCallback, this, std::placeholders::_1));
  pub_objects_carla_map_ = this->create_publisher<pi::ObjectList>("~/object_list", 1);
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_objects_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_objects_carla_map_->get_topic_name());

  if (enable_traffic_lights_) {
    sub_traffic_light_info_ = this->create_subscription<cm::CarlaTrafficLightInfoList>(
        "/carla/traffic_lights/info", 1,
        std::bind(&CarlaConverter::trafficLightInfoCallback, this, std::placeholders::_1));
    sub_traffic_light_status_ = this->create_subscription<cm::CarlaTrafficLightStatusList>(
        "/carla/traffic_lights/status", 1,
        std::bind(&CarlaConverter::trafficLightStatusCallback, this, std::placeholders::_1));
    pub_traffic_lights_carla_map_ = this->create_publisher<pi::ObjectList>("~/traffic_lights", 1);
    timer_traffic_lights_ = create_wall_timer(
        std::chrono::milliseconds(int(1000 / traffic_light_frequency_)),
        std::bind(&CarlaConverter::publishTrafficLights, this));
    RCLCPP_INFO(this->get_logger(), "Subscribed to '%s' and '%s'",
                sub_traffic_light_info_->get_topic_name(), sub_traffic_light_status_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_traffic_lights_carla_map_->get_topic_name());
  } else {
    RCLCPP_INFO(this->get_logger(), "Traffic light subscriptions and publishing are disabled.");
  }

  sub_world_info_ = this->create_subscription<cm::CarlaWorldInfo>(
      "/carla/world_info", qosLatching,
      std::bind(&CarlaConverter::worldInfoCallback, this, std::placeholders::_1));
  pub_map_info_ = this->create_publisher<stm::String>("~/map_info", qosLatching);
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_world_info_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_map_info_->get_topic_name());

  // setup subscriber and publisher depending on actor_name
  for (std::string& actor_name : ego_data_actors_) {
    Subscriber<nm::Odometry> sub_odometry = this->create_subscription<nm::Odometry>(
        "/carla/" + actor_name + "/odometry", 1, odometryArgCallback(actor_name));
    Subscriber<cm::CarlaEgoVehicleStatus> sub_vehicle_status = this->create_subscription<cm::CarlaEgoVehicleStatus>(
        "/carla/" + actor_name + "/vehicle_status", 1, vehicleStatusArgCallback(actor_name));
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info = this->create_subscription<cm::CarlaEgoVehicleInfo>(
        "/carla/" + actor_name + "/vehicle_info", qosLatching, vehicleInfoArgCallback(actor_name));
    Subscriber<ssm::NavSatFix> sub_gnss = this->create_subscription<ssm::NavSatFix>(
        "/carla/" + actor_name + "/gnss", 1, gnssArgCallback(actor_name));
    Subscriber<ssm::Imu> sub_imu = this->create_subscription<ssm::Imu>(
        "/carla/" + actor_name + "/imu", 1, imuArgCallback(actor_name));

    // save subscriber in map with actor_name as key
    sub_odometry_map_.insert({actor_name, sub_odometry});
    sub_vehicle_status_map_.insert({actor_name, sub_vehicle_status});
    sub_vehicle_info_map_.insert({actor_name, sub_vehicle_info});
    sub_gnss_map_.insert({actor_name, sub_gnss});
    sub_imu_map_.insert({actor_name, sub_imu});

    // setup publisher depending on actor_name
    Publisher<pi::EgoData> pub_ego_data =
        this->create_publisher<pi::EgoData>("~/" + actor_name + "/ego_data", 1);
    Publisher<etsi_cam::CAM> pub_etsi_cam =
        this->create_publisher<etsi_cam::CAM>("~/" + actor_name + "/etsi_cam", 1);

    // save publisher in map with actor_name as key
    pub_ego_data_map_.insert({actor_name, pub_ego_data});
    pub_etsi_cam_map_.insert({actor_name, pub_etsi_cam});

    ROS_LOG_STREAM(INFO, "Subscribed /carla state topics for actor "
                             << actor_name << " and publishing ~/" << actor_name << "/ego_data");
  }

  // setup object subscriber and publisher for object_list_actors_
  for (std::string& actor_name : object_list_actors_) {
    // setup subscriber depending on actor_name
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info = this->create_subscription<cm::CarlaEgoVehicleInfo>(
        "/carla/" + actor_name + "/vehicle_info", qosLatching, vehicleInfoArgCallback(actor_name));

    // save subscriber in map with actor_name as key
    sub_vehicle_info_map_.insert({actor_name, sub_vehicle_info});

    // setup publisher depending on actor_name
    Publisher<pi::ObjectList> pub_objects =
        this->create_publisher<pi::ObjectList>("~/" + actor_name + "/object_list", 1);

    // save publisher in map with actor_name as key
    pub_objects_map_.insert({actor_name, pub_objects});

    ROS_LOG_STREAM(INFO, "Subscribed /carla object topics for actor "
                             << actor_name << " and publishing ~/" << actor_name << "/object_list");
  }

  // create 1s timer to subscribe to custom topics
  timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&CarlaConverter::subscribeCustomTopics, this));

  // init etsi msg timestamp
  last_cam_msg_ = this->now();

  RCLCPP_INFO(this->get_logger(), "carla_converter running...");
}


void CarlaConverter::subscribeCustomTopics() {
  auto customObjectsArgCallback = [this](const std::string& topic_name) {
    return [this, topic_name](const dom::ObjectArray::ConstPtr msg) -> void {
      CarlaConverter::customObjectsCallback(msg, topic_name);
    };
  };
  // iterate over all topics
  std::map<std::string, std::vector<std::string>> topics = this->get_topic_names_and_types();
  for (const auto& topic : topics) {
    std::string topic_name = topic.first;
    std::string topic_type = topic.second[0];

    // skip if topic does not match type
    if (topic_type != "derived_object_msgs/msg/ObjectArray") continue;

    // skip if topic matches standard object topic pattern
    std::regex pattern_objects("/carla/.*\\/objects");
    if (topic_name == "/carla/objects" || std::regex_match(topic_name, pattern_objects)) continue;

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

    Publisher<pi::ObjectList> pub_custom_objects =
        this->create_publisher<pi::ObjectList>("~/" + topic_name, 1);
    pub_custom_objects_map_.insert({topic_name, pub_custom_objects});

    ROS_LOG_STREAM(INFO, "Subscribed custom topic /carla/" << topic_name
                                                           << " and publishing ~/" << topic_name);
  }
}

void CarlaConverter::gnssCallback(const ssm::NavSatFix::ConstPtr msg, std::string actor_name) {
  // get gnss position from actor_name vehicle
  ego_gnss_map_[actor_name] = *msg;
  ego_gnss_set_map_[actor_name] = true;
}

void CarlaConverter::imuCallback(const ssm::Imu::ConstPtr msg, std::string actor_name) {
  // get imu acceleration from actor_name vehicle
  ego_acceleration_map_[actor_name].linear = msg->linear_acceleration;
}

void CarlaConverter::vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg, std::string actor_name) {
  // get steering_angle and acceleration from actor_name vehicle
  ego_steering_angle_map_[actor_name] = msg->control.steer;
  ego_status_set_map_[actor_name] = true;
}

void CarlaConverter::vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg, std::string actor_name) {
  // get id from actor_name vehicle
  ego_id_map_[actor_name] = msg->id;
  ego_steering_angle_max_map_[actor_name] = 0.0;
  for (int i = 0; i < (int)msg->wheels.size(); i++) {
    ego_steering_angle_max_map_[actor_name] =
        std::max(ego_steering_angle_max_map_[actor_name], (double)msg->wheels[i].max_steer_angle);
  }
  ego_info_set_map_[actor_name] = true;
}


void CarlaConverter::trafficLightInfoCallback(const cm::CarlaTrafficLightInfoList::ConstPtr msg) {
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

void CarlaConverter::trafficLightStatusCallback(const cm::CarlaTrafficLightStatusList::ConstPtr msg) {
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

void CarlaConverter::worldInfoCallback(const cm::CarlaWorldInfo::ConstPtr msg) {
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

void CarlaConverter::odometryCallback(const nm::Odometry::ConstPtr msg, std::string actor_name) {
  // map ego data from CARLA to the perception_msgs EgoData format
  if (ego_shape_set_map_[actor_name] && ego_status_set_map_[actor_name]) {
    msg_ego_data_.header = msg->header;

    // get euler angles
    double roll, pitch, yaw;
    tf2::Quaternion quat_tf;
    tf2::fromMsg(msg->pose.pose.orientation, quat_tf);
    tf2::Matrix3x3 matrix(quat_tf);
    matrix.getRPY(roll, pitch, yaw);

    // fill state
    oa::initializeState(msg_ego_data_, pi::EGO::MODEL_ID);
    msg_ego_data_.state.header = msg_ego_data_.header;
    oa::setPose(msg_ego_data_.state, msg->pose.pose);
    oa::setZ(msg_ego_data_, msg->pose.pose.position.z + ego_shape_map_[actor_name].dimensions[2] / 2.0);

    oa::setYawRate(msg_ego_data_.state, msg->twist.twist.angular.z);  // twist is defined in child frame (no transformation needed)
    oa::setVelocity(msg_ego_data_.state, msg->twist.twist.linear);    // twist is defined in child frame (no transformation needed)
    oa::setAcceleration(msg_ego_data_.state, ego_acceleration_map_[actor_name].linear);

    // SteeringAngleMax is given in rad (contrary to the description in the documentation)
    // https://github.com/carla-simulator/ros-bridge/blob/e9063d97ff5a724f76adbb1b852dc71da1dcfeec/carla_ros_bridge/src/carla_ros_bridge/ego_vehicle.py#L145C46-L145C81
    oa::setSteeringAngleAck(msg_ego_data_.state,
                            -ego_steering_angle_map_[actor_name] * ego_steering_angle_max_map_[actor_name]);
    oa::setStandstill(msg_ego_data_.state,
                      std::sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2) +
                                pow(msg->twist.twist.linear.z, 2)) <= 0.01);

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
    oa::setContinuousStateCovarianceAt(msg_ego_data_.state, pi::EGO::YAW_RATE, pi::EGO::YAW_RATE,
                                       angle_rate_variances_);

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
                    "Skip EgoData to CAM conversion: %s (Make sure that utm frame exist and unix time is used!)",
                    e.what());
        last_cam_msg_ = this->now();
      }
    }
  }
}

void CarlaConverter::objectsCallback(const dom::ObjectArray::ConstPtr msg) {
  pi::ObjectList msg_object_list_ = CarlaConverter::convertObjectArray(msg);

  // publish object_list in carla_map frame
  pub_objects_carla_map_->publish(msg_object_list_);

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

void CarlaConverter::customObjectsCallback(const dom::ObjectArray::ConstPtr msg, std::string topic_name) {
  pi::ObjectList msg_object_list_ = CarlaConverter::convertObjectArray(msg);
  pi::ObjectList msg_object_list_transformed;

  // transform the object_list from carla_map to topic frame if possible
  if (CarlaConverter::transformFrame(msg_object_list_, msg_object_list_transformed, topic_name)) {
    pub_custom_objects_map_[topic_name]->publish(msg_object_list_transformed);
  } else {
    pub_custom_objects_map_[topic_name]->publish(msg_object_list_);
  }
}

pi::ObjectList CarlaConverter::convertObjectArray(const dom::ObjectArray::ConstPtr msg) {
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
      if (ego_id_map_[actor_name] == msg->objects[i].id) {
        ego_shape_map_[actor_name] = msg->objects[i].shape;
        ego_shape_set_map_[actor_name] = true;
      }
    }

    pi::Object objectTemp;
    objectTemp.id = msg->objects[i].id;
    objectTemp.existence_probability = 1.0; // always 1.0 as source is CARLA

    // get yaw angle
    double roll, pitch, yaw;
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
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::ROLL_RATE, pi::HEXAMOTION::ROLL_RATE,
                                       angle_rate_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::PITCH_RATE, pi::HEXAMOTION::PITCH_RATE,
                                       angle_rate_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::YAW_RATE, pi::HEXAMOTION::YAW_RATE,
                                       angle_rate_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::LENGTH, pi::HEXAMOTION::LENGTH, pos_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::WIDTH, pi::HEXAMOTION::WIDTH, pos_variances_);
    oa::setContinuousStateCovarianceAt(objectTemp, pi::HEXAMOTION::HEIGHT, pi::HEXAMOTION::HEIGHT, pos_variances_);

    // global object list: Sensor ID 0
    objectTemp.state.sensor_id.push_back(0);

    objectTemp.state.classifications.resize(1);
    objectTemp.state.classifications[0].probability = 1.0; // always 1.0 as source is CARLA
    switch ((int)msg->objects[i].classification) {
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

etsi_cam::CAM CarlaConverter::convertEgoDataCam(const pi::EgoData msg) {
  etsi_cam::CAM msg_cam;
  // TODO: get direct parent frame of carla_map dynamically
  if (tf2_buffer_->_frameExists("utm_32N") &&
      tf2_buffer_->canTransform("utm_32N", msg.header.frame_id, msg.header.stamp)) {
    ad2etsi::egodata2cam(msg_ego_data_, msg_cam, *tf2_buffer_.get(), 32, true);
  } else if (tf2_buffer_->_frameExists("utm_31N") &&
             tf2_buffer_->canTransform("utm_31N", msg.header.frame_id, msg.header.stamp)) {
    ad2etsi::egodata2cam(msg_ego_data_, msg_cam, *tf2_buffer_.get(), 31, true);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Tranformation to global utm frame not found.");
  }

  return msg_cam;
}

bool CarlaConverter::transformFrame(const pi::ObjectList& msg_object_list,
                                       pi::ObjectList& msg_object_list_transformed,
                                       std::string target_frame) {
  if (msg_object_list.objects.size() == 0) return false;

  try {
    while (true) {
      if (tf2_buffer_->_frameExists(target_frame)) {
        msg_object_list_transformed =
            tf2_buffer_->transform(msg_object_list, target_frame, tf2::durationFromSec(0.1));
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


int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<carla_converter::CarlaConverter>();
  rclcpp::executors::SingleThreadedExecutor executor;
  RCLCPP_INFO(node->get_logger(), "Spinning node '%s' with %s", node->get_fully_qualified_name(),
              "SingleThreadedExecutor");
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
