#include <CarlaItsConverterNode.h>


namespace carla {

ItsConverter::ItsConverter() : Node("CarlaItsConverter")
{
  // load Parameters and if not successful, return
  if(!loadParameters()) return;

  auto odometryArgCallback = [this](const std::string & actor_name) {
    return [this, actor_name](const nm::Odometry::ConstPtr msg) -> void {
      ItsConverter::odometryCallback(msg, actor_name);
    };
  };

  auto vehicleStatusArgCallback = [this](const std::string & actor_name) {
    return [this, actor_name](const cm::CarlaEgoVehicleStatus::ConstPtr msg) -> void {
      ItsConverter::vehicleStatusCallback(msg, actor_name);
    };
  };

  auto vehicleInfoArgCallback = [this](const std::string & actor_name) {
    return [this, actor_name](const cm::CarlaEgoVehicleInfo::ConstPtr msg) -> void {
      ItsConverter::vehicleInfoCallback(msg, actor_name);
    };
  };

  auto gnssArgCallback = [this](const std::string & actor_name) {
    return [this, actor_name](const ssm::NavSatFix::ConstPtr msg) -> void {
      ItsConverter::gnssCallback(msg, actor_name);
    };
  };

  auto customObjectsArgCallback = [this](const std::string & topic_name) {
    return [this, topic_name](const dom::ObjectArray::ConstPtr msg) -> void {
      ItsConverter::customObjectsCallback(msg, topic_name);
    };
  };

  // setup buffer
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_buffer_->setUsingDedicatedThread(true);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  
  rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
  qosLatching.transient_local();
  qosLatching.reliable();
  
  // setup subscriber and publisher
  sub_objects_ = this->create_subscription<dom::ObjectArray>("/carla/objects", 1, std::bind(&ItsConverter::objectsCallback, this, std::placeholders::_1));
  pub_objects_carla_map_ = this->create_publisher<pi::ObjectList>("/carla_its_converter/objects", 1);
  
  // setup subscriber and publisher depending on actor_name
  for(std::string& actor_name : ego_data_actors_) { 
    
    // setup subscriber depending on actor_name    
    Subscriber<nm::Odometry> sub_odometry = this->create_subscription<nm::Odometry>("/carla/" + actor_name +"/odometry", 1, odometryArgCallback(actor_name));
    Subscriber<cm::CarlaEgoVehicleStatus> sub_vehicle_status = this->create_subscription<cm::CarlaEgoVehicleStatus>("/carla/" + actor_name +"/vehicle_status", 1, vehicleStatusArgCallback(actor_name));
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info = this->create_subscription<cm::CarlaEgoVehicleInfo>("/carla/" + actor_name +"/vehicle_info", qosLatching, vehicleInfoArgCallback(actor_name));
    Subscriber<ssm::NavSatFix> sub_gnss = this->create_subscription<ssm::NavSatFix>("/carla/" + actor_name +"/gnss", 1, gnssArgCallback(actor_name));

    // save subscriber in map with actor_name as key
    sub_odometry_map_.insert({actor_name, sub_odometry});
    sub_vehicle_status_map_.insert({actor_name, sub_vehicle_status});
    sub_vehicle_info_map_.insert({actor_name, sub_vehicle_info});
    sub_gnss_map_.insert({actor_name, sub_gnss});

    // setup publisher depending on actor_name
    Publisher<pi::EgoData> pub_ego_data = this->create_publisher<pi::EgoData>("/carla_its_converter/" + actor_name + "/ego_data", 1);
    Publisher<etsi_cam::CAM> pub_etsi_cam = this->create_publisher<etsi_cam::CAM>("/carla_its_converter/" + actor_name + "/etsi_its/cam", 1);

    // save publisher in map with actor_name as key
    pub_ego_data_map_.insert({actor_name, pub_ego_data});
    pub_etsi_cam_map_.insert({actor_name, pub_etsi_cam});
  }

  // setup subscriber and publisher depending on actor_name
  for(std::string& actor_name : object_data_actors_) { 
    
    // setup subscriber depending on actor_name    
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info = this->create_subscription<cm::CarlaEgoVehicleInfo>("/carla/" + actor_name +"/vehicle_info", qosLatching, vehicleInfoArgCallback(actor_name));

    // save subscriber in map with actor_name as key
    sub_vehicle_info_map_.insert({actor_name, sub_vehicle_info});

    // setup publisher depending on actor_name
    Publisher<pi::ObjectList> pub_objects = this->create_publisher<pi::ObjectList>("/carla_its_converter/" + actor_name + "/objects", 1);

    // save publisher in map with actor_name as key
    pub_objects_map_.insert({actor_name, pub_objects});
  }
  
  // setup custom subscriber and publisher depending on topic type

  // get topic names and types
  sleep(2); // wait for topics to be available (TODO: improve this)
  std::map<std::string, std::vector<std::string> > topics;
  std::vector<std::string> topics_selected;
  topics = this->get_topic_names_and_types();

  // filter topics by datatype
  for (const auto& topic : topics){
    std::string topic_type;
    for (const auto& tt : topic.second){
      topic_type += tt;
    }
    if (topic_type == "derived_object_msgs/msg/ObjectArray"){
      topics_selected.push_back(topic.first);
    }
  }
  
  // filter local object topics
  std::regex pattern_objects("/carla/.*\\/objects");

  for (const std::string& topic : topics_selected){
    if (topic == "/carla/objects" || std::regex_match(topic, pattern_objects)){
      continue;
    }

    // remove "/carla/" from topic name
    const std::string prefix = "/carla/";
    std::size_t pos = topic.find(prefix);
    if (pos == std::string::npos){
      continue;
    }
    std::string topic_name = topic.substr(pos + prefix.length());
    
    // setup subscriber depending on topic name and save subscriber in vector
    Subscriber<dom::ObjectArray> sub_custom_objects = this->create_subscription<dom::ObjectArray>(prefix + topic_name, 1, customObjectsArgCallback(topic_name));
    sub_custom_objects_map_.insert({topic_name, sub_custom_objects});

    // setup publisher depending on topic name and save publisher in vector 
    Publisher<pi::ObjectList> pub_custom_objects = this->create_publisher<pi::ObjectList>("/carla_its_converter/" + topic_name, 1);
    pub_custom_objects_map_.insert({topic_name, pub_custom_objects});
  }

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

void ItsConverter::gnssCallback(const ssm::NavSatFix::ConstPtr msg, std::string actor_name) {
  // get gnss position from actor_name vehicle
  ego_gnss_map_[actor_name] = *msg;
  ego_gnss_set_map_[actor_name] = true;
}

void ItsConverter::vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg, std::string actor_name){
  // get steering_angle and acceleration from actor_name vehicle
  ego_steering_angle_map_[actor_name] = msg->control.steer;
  ego_acceleration_map_[actor_name] = msg->acceleration;
  ego_status_set_map_[actor_name] = true;
}

void ItsConverter::vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg, std::string actor_name){
  // get id from actor_name vehicle
  ego_id_map_[actor_name] = msg->id;
  ego_steering_angle_max_map_[actor_name] = 0.0;
  for(int i=0; i<msg->wheels.size(); i++){
    ego_steering_angle_max_map_[actor_name] = std::max(ego_steering_angle_max_map_[actor_name], (double)msg->wheels[i].max_steer_angle);
  }

  ego_info_set_map_[actor_name] = true;
}

void ItsConverter::odometryCallback(const nm::Odometry::ConstPtr msg, std::string actor_name) {
  // map ego data from CARLA to the perception_msgs format
  if(ego_shape_set_map_[actor_name] && ego_status_set_map_[actor_name]){
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
    oa::setZ(msg_ego_data_, msg->pose.pose.position.z + ego_shape_map_[actor_name].dimensions[2] / 2.0); // set z to the center of the object
    
    // twist in 
    oa::setYawRate(msg_ego_data_.state, msg->twist.twist.angular.z);  // twist is defined in child frame (no transformation needed)
    oa::setVelocity(msg_ego_data_.state, msg->twist.twist.linear);    // twist is defined in child frame (no transformation needed)
    oa::setAccelerationXYZYaw(msg_ego_data_.state, ego_acceleration_map_[actor_name].linear, yaw); // accleration defined in carla_map frame (transformation needed)

    // SteeringAngleMax is given in rad (contrary to the description in the documentation)
    // https://github.com/carla-simulator/ros-bridge/blob/e9063d97ff5a724f76adbb1b852dc71da1dcfeec/carla_ros_bridge/src/carla_ros_bridge/ego_vehicle.py#L145C46-L145C81
    oa::setSteeringAngleAck(msg_ego_data_.state, -ego_steering_angle_map_[actor_name]*ego_steering_angle_max_map_[actor_name]);
    oa::setStandstill(msg_ego_data_.state, std::sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2) + pow(msg->twist.twist.linear.z, 2)) <= 0.01);

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

    // additionally publish etsi_cam
    if (ego_gnss_set_map_[actor_name]) {
      etsi_cam::CAM msg_cam = convertToEtsiCam(msg_ego_data_, ego_gnss_map_[actor_name]);

      pub_etsi_cam_map_[actor_name]->publish(msg_cam);
    }

  } 
}

etsi_cam::CAM ItsConverter::convertToEtsiCam(const pi::EgoData &ego_data, ssm::NavSatFix gnss) {
  
  etsi_cam::CAM cam;
  
  ca::setItsPduHeader(cam, ego_data.vehicle_id);
  
  uint64_t nns = ego_data.header.stamp.sec * 1e9 + ego_data.header.stamp.nanosec;
    
  ca::setGenerationDeltaTime(cam, nns, etsi_its_msgs::getLeapSecondInsertionsSince2004((uint64_t)ego_data.header.stamp.sec));
  
  ca::setReferencePosition(cam, gnss.latitude, gnss.longitude, gnss.altitude + ego_data.height/2);
  
  // convert and crop heading to etsi range [0,360]
  double heading = 90 - oa::getYawInDeg(ego_data);
  if (heading > 360) heading -= 360;
  if (heading < 0) heading += 360;
  ca::setHeading(cam, heading);

  ca::setVehicleDimensions(cam, ego_data.length, ego_data.width);
  
  ca::setSpeed(cam, oa::getVelocityMagnitude(ego_data));
  ca::setLongitudinalAcceleration(cam, oa::getAccLon(ego_data));
  ca::setLateralAcceleration(cam, oa::getAccLat(ego_data));

  return cam;
}

pi::ObjectList ItsConverter::convertObjectArray(const dom::ObjectArray::ConstPtr msg) {
  // map the objects from the CARLA format to the perception_msgs format
  pi::ObjectList msg_object_list_;
  msg_object_list_.header = msg->header;

  for (size_t i = 0; i < msg->objects.size(); i++) {
    // only add classified objects
    if(!msg->objects[i].object_classified) continue;
    
    for(auto & actor_name : ego_data_actors_){
      // only continue if ego_info_set_ 
      if (!ego_info_set_map_[actor_name]) continue;

      // get shape from actor_name vehicle
      if(ego_id_map_[actor_name] == msg->objects[i].id){
        ego_shape_map_[actor_name] = msg->objects[i].shape;
        ego_shape_set_map_[actor_name] = true;
      } 
    } 
    
    pi::Object objectTemp;
    objectTemp.id = msg->objects[i].id;
    objectTemp.existence_probability = 1.0; // probability that the object exists is always 1.0 as source is CARLA

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
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::X, pi::HEXAMOTION::X, pos_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::Y, pi::HEXAMOTION::Y, pos_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::Z, pi::HEXAMOTION::Z, pos_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::VEL_LON, pi::HEXAMOTION::VEL_LON,
      vel_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::VEL_LAT, pi::HEXAMOTION::VEL_LAT,
      vel_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::ACC_LON, pi::HEXAMOTION::ACC_LON,
      acc_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::ACC_LAT, pi::HEXAMOTION::ACC_LAT,
      acc_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::ROLL, pi::HEXAMOTION::ROLL, angle_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::PITCH, pi::HEXAMOTION::PITCH, angle_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::YAW, pi::HEXAMOTION::YAW, angle_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::ROLL_RATE, pi::HEXAMOTION::ROLL_RATE,
      angle_rate_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::PITCH_RATE, pi::HEXAMOTION::PITCH_RATE,
      angle_rate_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::YAW_RATE, pi::HEXAMOTION::YAW_RATE,
      angle_rate_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::LENGTH, pi::HEXAMOTION::LENGTH, pos_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::WIDTH, pi::HEXAMOTION::WIDTH, pos_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::HEXAMOTION::HEIGHT, pi::HEXAMOTION::HEIGHT, pos_variances_);

    // Global object list: Sensor ID 0
    objectTemp.state.sensor_id.push_back(0);

    // classification for object state
    objectTemp.state.classifications.resize(1);
    objectTemp.state.classifications[0].probability = 1.0; // probability that the object is of this type is always 1.0 as source is CARLA
    switch ((int) msg->objects[i].classification)
    {
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
    if(objectTemp.state.classifications[0].type != pi::ObjectClassification::PEDESTRIAN){
      oa::setZ(objectTemp, msg->objects[i].pose.position.z + msg->objects[i].shape.dimensions[2] / 2.0); // Set z to the center of the object
    }

    // reference point for object state position
    objectTemp.state.reference_point.value = pi::ObjectReferencePoint::GEOMETRIC_CENTER;

    // add object to object list
    msg_object_list_.objects.push_back(objectTemp);
  }
  return msg_object_list_;
}

bool ItsConverter::transformFrame(const pi::ObjectList& msg_object_list, pi::ObjectList& msg_object_list_transformed, std::string topic_name) {

  if (msg_object_list.objects.size() == 0) return false;

  try {
    while (true){
      // try to transform to topic_name frame
      if (tf2_buffer_->_frameExists(topic_name)){
        msg_object_list_transformed = tf2_buffer_->transform(msg_object_list, topic_name);
        return true;
      }

      // if topic_name frame does not exist, try to transform to subframes of topic_name
      size_t pos = topic_name.rfind("/");
      if (pos != std::string::npos){
        topic_name = topic_name.substr(0, pos);
      } else {
        throw std::runtime_error("Break because topic_name string is empty");
      }
    }
  } catch (tf2::TransformException& e) {
    ROS_LOG_STREAM(WARN, "Transformation from '" << topic_name << "' or its subframes is not available");
    ROS_LOG_STREAM(WARN, e.what());
    return false;
  }
}

void ItsConverter::objectsCallback(const dom::ObjectArray::ConstPtr msg) {
  pi::ObjectList msg_object_list_ = ItsConverter::convertObjectArray(msg);

  // publish object_list in carla_map frame
  pub_objects_carla_map_->publish(msg_object_list_);

  // iterate over each actor_name
  for(std::string & actor_name: object_data_actors_){
    pi::ObjectList msg_object_list_copy = msg_object_list_;

    // Set increasing sensor ID per role name. As `ego_data_actors_` doesnt change, this assigns a fixed sensor ID to each role.
    for (auto &object: msg_object_list_copy.objects)
    {
      object.state.sensor_id[0] = ego_id_map_[actor_name];
    }

    // remove element with the actor_name id
    msg_object_list_copy.objects.erase(std::remove_if(msg_object_list_copy.objects.begin(), msg_object_list_copy.objects.end(), [this, actor_name](const pi::Object& obj) {
      return obj.id == ego_id_map_[actor_name];
    }), msg_object_list_copy.objects.end());

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

}  // end of namespace


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<carla::ItsConverter>());
  rclcpp::shutdown();
  return 0;
}

