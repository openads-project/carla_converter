#include <CarlaItsConverterNode.h>

namespace carla {

ItsConverter::ItsConverter() : Node("CarlaItsConverter") {
  // load Parameters
  if (!loadParameters()) return;

  // setup callback functions
  auto odometryArgCallback = [this](const std::string& actor_name) {
    return [this, actor_name](const nm::Odometry::ConstPtr msg) -> void {
      ItsConverter::odometryCallback(msg, actor_name);
    };
  };

  auto vehicleStatusArgCallback = [this](const std::string& actor_name) {
    return [this, actor_name](const cm::CarlaEgoVehicleStatus::ConstPtr msg) -> void {
      ItsConverter::vehicleStatusCallback(msg, actor_name);
    };
  };

  auto vehicleInfoArgCallback = [this](const std::string& actor_name) {
    return [this, actor_name](const cm::CarlaEgoVehicleInfo::ConstPtr msg) -> void {
      ItsConverter::vehicleInfoCallback(msg, actor_name);
    };
  };

  auto gnssArgCallback = [this](const std::string& actor_name) {
    return
        [this, actor_name](const ssm::NavSatFix::ConstPtr msg) -> void { ItsConverter::gnssCallback(msg, actor_name); };
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
      "/carla/objects", 1, std::bind(&ItsConverter::objectsCallback, this, std::placeholders::_1));
  pub_objects_carla_map_ = this->create_publisher<pi::ObjectList>("/carla_its_converter/object_list", 1);
  ROS_LOG_STREAM(INFO, "Subscribed to /carla/objects and publishing to /carla_its_converter/object_list");

  // info: mapem
  sub_traffic_light_info_list_ = this->create_subscription<cm::CarlaTrafficLightInfoList>(
    "/carla/traffic_lights/info", 1, std::bind(&ItsConverter::trafficInfoCallback, this, std::placeholders::_1));
  pub_etsi_mapem_ = this->create_publisher<etsi_mapem::MAPEM>("/carla_its_converter/etsi_mapem", 1);
  timer_publisher_mapem_ = create_wall_timer(
    std::chrono::milliseconds((long)(1000 / publisher_mapem_frequency_)),
    std::bind(&ItsConverter::publishMapemData, this));
  ROS_LOG_STREAM(INFO, "Subscribed to /carla/traffic_lights/info and publishing to /carla_its_converter/etsi_mapem");

  sub_traffic_light_status_list_ = this->create_subscription<cm::CarlaTrafficLightStatusList>(
    "/carla/traffic_lights/status", 1, std::bind(&ItsConverter::trafficStatusCallback, this, std::placeholders::_1));
  pub_etsi_spatem_ = this->create_publisher<etsi_spatem::SPATEM>("/carla_its_converter/etsi_spatem", 1);
  ROS_LOG_STREAM(INFO, "Subscribed to /carla/traffic_lights/status and publishing to /carla_its_converter/etsi_spatem");

  // setup subscriber and publisher depending on actor_name
  for (std::string& actor_name : ego_data_actors_) {
    // setup subscriber depending on actor_name
    Subscriber<nm::Odometry> sub_odometry = this->create_subscription<nm::Odometry>(
        "/carla/" + actor_name + "/odometry", 1, odometryArgCallback(actor_name));
    Subscriber<cm::CarlaEgoVehicleStatus> sub_vehicle_status = this->create_subscription<cm::CarlaEgoVehicleStatus>(
        "/carla/" + actor_name + "/vehicle_status", 1, vehicleStatusArgCallback(actor_name));
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info = this->create_subscription<cm::CarlaEgoVehicleInfo>(
        "/carla/" + actor_name + "/vehicle_info", qosLatching, vehicleInfoArgCallback(actor_name));
    Subscriber<ssm::NavSatFix> sub_gnss =
        this->create_subscription<ssm::NavSatFix>("/carla/" + actor_name + "/gnss", 1, gnssArgCallback(actor_name));

    // save subscriber in map with actor_name as key
    sub_odometry_map_.insert({actor_name, sub_odometry});
    sub_vehicle_status_map_.insert({actor_name, sub_vehicle_status});
    sub_vehicle_info_map_.insert({actor_name, sub_vehicle_info});
    sub_gnss_map_.insert({actor_name, sub_gnss});

    // setup publisher depending on actor_name
    Publisher<pi::EgoData> pub_ego_data =
        this->create_publisher<pi::EgoData>("/carla_its_converter/" + actor_name + "/ego_data", 1);
    Publisher<etsi_cam::CAM> pub_etsi_cam =
        this->create_publisher<etsi_cam::CAM>("/carla_its_converter/" + actor_name + "/etsi_cam", 1);

    // save publisher in map with actor_name as key
    pub_ego_data_map_.insert({actor_name, pub_ego_data});
    pub_etsi_cam_map_.insert({actor_name, pub_etsi_cam});

    ROS_LOG_STREAM(INFO, "Subscribed /carla state topics for actor "
                             << actor_name << " and publishing /carla_its_converter/" << actor_name << "/ego_data");
  }

  // setup object subscriber and publisher for object_data_actors_
  for (std::string& actor_name : object_data_actors_) {
    // setup subscriber depending on actor_name
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info = this->create_subscription<cm::CarlaEgoVehicleInfo>(
        "/carla/" + actor_name + "/vehicle_info", qosLatching, vehicleInfoArgCallback(actor_name));

    // save subscriber in map with actor_name as key
    sub_vehicle_info_map_.insert({actor_name, sub_vehicle_info});

    // setup publisher depending on actor_name
    Publisher<pi::ObjectList> pub_objects =
        this->create_publisher<pi::ObjectList>("/carla_its_converter/" + actor_name + "/object_list", 1);

    // save publisher in map with actor_name as key
    pub_objects_map_.insert({actor_name, pub_objects});

    ROS_LOG_STREAM(INFO, "Subscribed /carla object topics for actor "
                             << actor_name << " and publishing /carla_its_converter/" << actor_name << "/object_list");
  }

  // create 1s timer to subscribe to custom topics
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ItsConverter::subscribeCustomTopics, this));

  // init etsi msg timestpamp
  last_cam_msg_ = this->now();
  last_spatem_msg_ = this->now();
  last_mapem_msg_ = this->now();

  ROS_LOG_STREAM(INFO, "carla_its_converter running...");
}

bool ItsConverter::loadParameters() {
  std::string ego_data_actors_string;
  std::string object_data_actors_string;

  // load publish parameters
  this->declare_parameter("ego_data_actors", "ego_vehicle");
  try {
    ego_data_actors_string = this->get_parameter("ego_data_actors").as_string();
  } catch (rclcpp::exceptions::ParameterNotDeclaredException&) {
    ego_data_actors_string = "ego_vehicle";
    ROS_LOG_STREAM(WARN, "Parameter \'ego_data_actors\' not set, defaulting to " << ego_data_actors_string);
  }

  this->declare_parameter("object_data_actors", "ego_vehicle");
  try {
    object_data_actors_string = this->get_parameter("object_data_actors").as_string();
  } catch (rclcpp::exceptions::ParameterNotDeclaredException&) {
    object_data_actors_string = "ego_vehicle";
    ROS_LOG_STREAM(WARN, "Parameter \'object_data_actors\' not set, defaulting to " << object_data_actors_string);
  }

  this->declare_parameter("pos_variances", oa::CONTINUOUS_STATE_COVARIANCE_INVALID);
  pos_variances_ = this->get_parameter("pos_variances").as_double();
  this->declare_parameter("vel_variances", oa::CONTINUOUS_STATE_COVARIANCE_INVALID);
  vel_variances_ = this->get_parameter("vel_variances").as_double();
  this->declare_parameter("acc_variances", oa::CONTINUOUS_STATE_COVARIANCE_INVALID);
  acc_variances_ = this->get_parameter("acc_variances").as_double();
  this->declare_parameter("angle_variances", oa::CONTINUOUS_STATE_COVARIANCE_INVALID);
  angle_variances_ = this->get_parameter("angle_variances").as_double();
  this->declare_parameter("angle_rate_variances", oa::CONTINUOUS_STATE_COVARIANCE_INVALID);
  angle_rate_variances_ = this->get_parameter("angle_rate_variances").as_double();

  std::string actor_name;
  std::stringstream ego_data_actors_string_stream(ego_data_actors_string);
  std::stringstream object_data_actors_string_stream(object_data_actors_string);

  while (std::getline(ego_data_actors_string_stream, actor_name, ',')) {
    // remove spaces from actor_name
    actor_name.erase(std::remove_if(actor_name.begin(), actor_name.end(), isspace), actor_name.end());

    // save actor_name in vector
    ego_data_actors_.push_back(actor_name);
  }

  while (std::getline(object_data_actors_string_stream, actor_name, ',')) {
    // remove spaces from actor_name
    actor_name.erase(std::remove_if(actor_name.begin(), actor_name.end(), isspace), actor_name.end());

    // save actor_name in vector
    object_data_actors_.push_back(actor_name);
  }

  return true;
}

void ItsConverter::subscribeCustomTopics() {
  auto customObjectsArgCallback = [this](const std::string& topic_name) {
    return [this, topic_name](const dom::ObjectArray::ConstPtr msg) -> void {
      ItsConverter::customObjectsCallback(msg, topic_name);
    };
  };

  // iterate over all topic
  std::map<std::string, std::vector<std::string> > topics = this->get_topic_names_and_types();
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
      topic_name.erase(pos, 7);  // length of "/carla/" is 7
    }

    // skip if topic is already subscribed
    if (sub_custom_objects_map_.find(topic_name) != sub_custom_objects_map_.end()) continue;

    // setup subscriber depending on topic name and save subscriber in vector
    Subscriber<dom::ObjectArray> sub_custom_objects =
        this->create_subscription<dom::ObjectArray>("/carla/" + topic_name, 1, customObjectsArgCallback(topic_name));
    sub_custom_objects_map_.insert({topic_name, sub_custom_objects});

    // setup publisher depending on topic name and save publisher in vector
    Publisher<pi::ObjectList> pub_custom_objects =
        this->create_publisher<pi::ObjectList>("/carla_its_converter/" + topic_name, 1);
    pub_custom_objects_map_.insert({topic_name, pub_custom_objects});

    ROS_LOG_STREAM(
        INFO, "Subscribed custom topic /carla/" << topic_name << " and publishing /carla_its_converter/" << topic_name);
  }
}

void ItsConverter::gnssCallback(const ssm::NavSatFix::ConstPtr msg, std::string actor_name) {
  // get gnss position from actor_name vehicle
  ego_gnss_map_[actor_name] = *msg;
  ego_gnss_set_map_[actor_name] = true;
}

void ItsConverter::vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg, std::string actor_name) {
  // get steering_angle and acceleration from actor_name vehicle
  ego_steering_angle_map_[actor_name] = msg->control.steer;
  ego_acceleration_map_[actor_name] = msg->acceleration;
  ego_status_set_map_[actor_name] = true;
}

void ItsConverter::vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg, std::string actor_name) {
  // get id from actor_name vehicle
  ego_id_map_[actor_name] = msg->id;
  ego_steering_angle_max_map_[actor_name] = 0.0;
  for (int i = 0; i < msg->wheels.size(); i++) {
    ego_steering_angle_max_map_[actor_name] =
        std::max(ego_steering_angle_max_map_[actor_name], (double)msg->wheels[i].max_steer_angle);
  }

  ego_info_set_map_[actor_name] = true;
}

uint16_t trafficLightIdToIntersectionId(const int id)
{
  // todo: check how the id is encoded
  return id;// / 100;
}

uint16_t trafficLightIdToMSignalGroupId(const int id)
{
  // todo: check how the id is encoded
  return id;// % 100;
}

uint16_t trafficLightIdToLaneId(const int id)
{
  // todo: check how the id is encoded
  return id;// / 100;
}

etsi_mapem::MAPEM convertCarlaToEtsi(const cm::CarlaTrafficLightInfoList::ConstPtr msg)
{
  etsi_mapem::MAPEM mapem;

  mapem.map.msg_issue_revision.value = 0; // are we interested in getting a revision from the CARLA simulation, e.g. a version number of the scene?
  
  for (auto &traffic_light : msg->traffic_lights) {

    // intersection geometry
    etsi_mapem::IntersectionGeometry intersection_geometry;
    intersection_geometry.id.id.value = trafficLightIdToIntersectionId(traffic_light.id);
    intersection_geometry.ref_point.lat.value = 0; // todo
    intersection_geometry.ref_point.lon.value = 0; // todo
    intersection_geometry.ref_point.elevation_is_present = true;
    intersection_geometry.ref_point.elevation.value = 0;

    // generic lane
    etsi_mapem::GenericLane generic_lane;
    generic_lane.lane_id.value = trafficLightIdToLaneId(traffic_light.id);
    //generic_lane.lane_attributes.directional_use = // todo: lane_attributes necessary
    
    // needed to find the corresponding SPAT messages
    etsi_mapem::Connection connection;
    connection.signal_group_is_present = true;
    connection.signal_group.value = trafficLightIdToMSignalGroupId(traffic_light.id);

    // nodes
    etsi_mapem::NodeXY node_traffic;
    
    generic_lane.node_list.choice = etsi_mapem::NodeListXY::CHOICE_NODES; // node type is arbritrary in our case?
    // TODO: generic_lane.lane_attributes = ??  set lane direction
    
    node_traffic.attributes.d_elevation_is_present = true;
    node_traffic.delta.node_xy1.x.value = traffic_light.transform.position.x * 1e2;
    node_traffic.delta.node_xy1.y.value = traffic_light.transform.position.y * 1e2;
    node_traffic.attributes.d_elevation.value = traffic_light.transform.position.z * 1e2;

    // fill arrays
    generic_lane.connects_to_is_present = true;
    generic_lane.connects_to.array.push_back(connection);
    generic_lane.node_list.nodes.array.push_back(node_traffic);

    intersection_geometry.lane_set.array.push_back(generic_lane);

    mapem.map.intersections_is_present = true;
    mapem.map.intersections.array.push_back(intersection_geometry);
  }

  return mapem;
}

etsi_spatem::SPATEM convertCarlaToEtsi(const cm::CarlaTrafficLightStatusList::ConstPtr msg)
{
  etsi_spatem::SPATEM spatem;
  
  spatem.spat.name_is_present = true;
  spatem.spat.name.value = "Carla traffic light status";

  int i = 0;

  for (auto &traffic_light : msg->traffic_lights) {
    // Intersection State
    etsi_spatem::IntersectionState intersection_state;
    intersection_state.id.id.value = trafficLightIdToIntersectionId(traffic_light.id);
    intersection_state.revision.value = 0; // are we interested in getting a revision from the CARLA simulation, e.g. a version number of the scene?
    
    intersection_state.name_is_present = true;
    intersection_state.name.value = "Intersection State name";

    // Movement State
    etsi_spatem::MovementState movement_state;
    movement_state.signal_group.value = trafficLightIdToMSignalGroupId(traffic_light.id); // signal group id

    // Movement event
    etsi_spatem::MovementEvent movement_event;

    // todo: discuss which ETSI values to take since the Carla values are ambiguous 
    switch (traffic_light.state) {
      case cm::CarlaTrafficLightStatus::RED:
        movement_event.event_state.value = etsi_spatem::MovementPhaseState::STOP_THEN_PROCEED;
        break;
      case cm::CarlaTrafficLightStatus::YELLOW:
      movement_event.event_state.value = etsi_spatem::MovementPhaseState::PRE_MOVEMENT;
        break;
      case cm::CarlaTrafficLightStatus::GREEN:
        movement_event.event_state.value = etsi_spatem::MovementPhaseState::PERMISSIVE_MOVEMENT_ALLOWED;
        break;
      case cm::CarlaTrafficLightStatus::OFF:
        movement_event.event_state.value = etsi_spatem::MovementPhaseState::DARK;
        break;
      case cm::CarlaTrafficLightStatus::UNKNOWN:
        movement_event.event_state.value = etsi_spatem::MovementPhaseState::DARK;
        break;
    default:
      movement_event.event_state.value = etsi_spatem::MovementPhaseState::STOP_THEN_PROCEED;
      break;
    }

    movement_state.state_time_speed.array.push_back(movement_event);

    intersection_state.states.array.push_back(movement_state);

    spatem.spat.intersections.array.push_back(intersection_state);
  }

  return spatem;
}

void ItsConverter::trafficInfoCallback(const cm::CarlaTrafficLightInfoList::ConstPtr msg) {
  // try to convert TrafficListInfoList to MAPEM
  try {
    mapem_converted_ = std::make_shared<etsi_mapem::MAPEM>(convertCarlaToEtsi(msg));
  } catch (const std::exception& e) {
    if (this->now() - last_mapem_msg_ > rclcpp::Duration(1, 0)) {
      RCLCPP_WARN(this->get_logger(),
                  "Skip TrafficLightInfoList to MAPEM conversion: %s (Make sure that utm frame exist unix time is used!)", e.what());
      last_mapem_msg_ = this->now();
    }
  }
}

void ItsConverter::trafficStatusCallback(const cm::CarlaTrafficLightStatusList::ConstPtr msg) {
  // try to convert CarlaTrafficLightStatusList to SPATEM
  try {
    etsi_spatem::SPATEM msg_spatem = convertCarlaToEtsi(msg);
    pub_etsi_spatem_->publish(msg_spatem);
    
    RCLCPP_INFO(this->get_logger(), "SPATEM published");
  } catch (const std::exception& e) {
    if (this->now() - last_spatem_msg_ > rclcpp::Duration(1, 0)) {
      RCLCPP_WARN(this->get_logger(),
                  "Skip TrafficLightStatusList to SPATEM conversion: %s (Make sure that utm frame exist unix time is used!)", e.what());
      last_spatem_msg_ = this->now();
    }
  }
}

void ItsConverter::publishMapemData() {
  if (mapem_converted_ == nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "No MAPEM data available to publish.");
  }
  else
  {
    pub_etsi_mapem_->publish(*mapem_converted_);
    RCLCPP_INFO(this->get_logger(), "MAPEM published");
  }
}

void ItsConverter::odometryCallback(const nm::Odometry::ConstPtr msg, std::string actor_name) {
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
    oa::setZ(msg_ego_data_, msg->pose.pose.position.z +
                                ego_shape_map_[actor_name].dimensions[2] / 2.0);  // set z to the center of the object

    // twist in
    oa::setYawRate(msg_ego_data_.state,
                   msg->twist.twist.angular.z);  // twist is defined in child frame (no transformation needed)
    oa::setVelocity(msg_ego_data_.state,
                    msg->twist.twist.linear);  // twist is defined in child frame (no transformation needed)
    oa::setAccelerationXYZYaw(msg_ego_data_.state, ego_acceleration_map_[actor_name].linear,
                              yaw);  // accleration defined in carla_map frame (transformation needed)

    // SteeringAngleMax is given in rad (contrary to the description in the documentation)
    // https://github.com/carla-simulator/ros-bridge/blob/e9063d97ff5a724f76adbb1b852dc71da1dcfeec/carla_ros_bridge/src/carla_ros_bridge/ego_vehicle.py#L145C46-L145C81
    oa::setSteeringAngleAck(msg_ego_data_.state,
                            -ego_steering_angle_map_[actor_name] * ego_steering_angle_max_map_[actor_name]);
    oa::setStandstill(msg_ego_data_.state,
                      std::sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2) +
                                pow(msg->twist.twist.linear.z, 2)) <= 0.01);

    // reference point for object position
    msg_ego_data_.state.reference_point.value = pi::ObjectReferencePoint::GEOMETRIC_CENTER;

    // # continuous state covariance matrix (N*N flattened)
    // float64[] continuous_state_covariance

    // # Planned trajectory of the ego_vehicle
    // ObjectState[] trajectory_planned

    // # Past trajectory of the ego_vehicle
    // ObjectState[] trajectory_past

    // # Planned route of the ego_vehicle
    // geometry_msgs/Point[] route_planned

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
                    "Skip EgoData to CAM conversion: %s (Make sure that utm frame exist unix time is used!)", e.what());
        last_cam_msg_ = this->now();
      }
    }
  }
}

void ItsConverter::objectsCallback(const dom::ObjectArray::ConstPtr msg) {
  pi::ObjectList msg_object_list_ = ItsConverter::convertObjectArray(msg);

  // publish object_list in carla_map frame
  pub_objects_carla_map_->publish(msg_object_list_);

  // iterate over each actor_name
  for (std::string& actor_name : object_data_actors_) {
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
    if (ItsConverter::transformFrame(msg_object_list_copy, msg_object_list_transformed, actor_name)) {
      pub_objects_map_[actor_name]->publish(msg_object_list_transformed);
    };
  }
}

void ItsConverter::customObjectsCallback(const dom::ObjectArray::ConstPtr msg, std::string topic_name) {
  pi::ObjectList msg_object_list_ = ItsConverter::convertObjectArray(msg);
  pi::ObjectList msg_object_list_transformed;

  // transform the object_list from carla_map to topic frame if possible
  if (ItsConverter::transformFrame(msg_object_list_, msg_object_list_transformed, topic_name)) {
    pub_custom_objects_map_[topic_name]->publish(msg_object_list_transformed);
  } else {
    pub_custom_objects_map_[topic_name]->publish(msg_object_list_);
  }
}

pi::ObjectList ItsConverter::convertObjectArray(const dom::ObjectArray::ConstPtr msg) {
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
    objectTemp.existence_probability = 1.0;  // probability that the object exists is always 1.0 as source is CARLA

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

    // Fill variances
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

    // Global object list: Sensor ID 0
    objectTemp.state.sensor_id.push_back(0);

    // classification for object state
    objectTemp.state.classifications.resize(1);
    objectTemp.state.classifications[0].probability =
        1.0;  // probability that the object is of this type is always 1.0 as source is CARLA
    switch ((int)msg->objects[i].classification) {
      case dom::Object::CLASSIFICATION_PEDESTRIAN:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::PEDESTRIAN;
        break;
      case dom::Object::CLASSIFICATION_BIKE:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::BICYCLE;
        break;
      case dom::Object::CLASSIFICATION_MOTORCYCLE:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::MOTORBIKE;
        break;
      case dom::Object::CLASSIFICATION_CAR:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::CAR;
        break;
      case dom::Object::CLASSIFICATION_TRUCK:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::TRUCK;
        break;
      case dom::Object::CLASSIFICATION_BARRIER:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::ROAD_OBSTACLE;
        break;
      default:
        objectTemp.state.classifications[0].type = pi::ObjectClassification::UNCLASSIFIED;
        break;
    }

    // set z for vehicles to center
    if (objectTemp.state.classifications[0].type != pi::ObjectClassification::PEDESTRIAN) {
      oa::setZ(objectTemp, msg->objects[i].pose.position.z +
                               msg->objects[i].shape.dimensions[2] / 2.0);  // Set z to the center of the object
    }

    // reference point for object state position
    objectTemp.state.reference_point.value = pi::ObjectReferencePoint::GEOMETRIC_CENTER;

    // add object to object list
    msg_object_list_.objects.push_back(objectTemp);
  }
  return msg_object_list_;
}

etsi_cam::CAM ItsConverter::convertEgoDataCam(const pi::EgoData msg) {
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

bool ItsConverter::transformFrame(const pi::ObjectList& msg_object_list, pi::ObjectList& msg_object_list_transformed,
                                  std::string target_frame) {
  if (msg_object_list.objects.size() == 0) return false;

  try {
    while (true) {
      // try to transform to target_frame
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

}  // namespace carla

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<carla::ItsConverter>());
  rclcpp::shutdown();
  return 0;
}
