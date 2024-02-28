#include <CarlaItsConverterNode.h>


namespace carla {

#ifdef ROS1
  ItsConverter::ItsConverter()
#else
  ItsConverter::ItsConverter() : Node("CarlaItsConverter")
#endif
{
  // load Parameters and if not successful, return
  if(!loadParameters()) return;

  auto odometryArgCallback = [this](const std::string & role_name) {
    return [this, role_name](const nm::Odometry::ConstPtr msg) -> void {
      ItsConverter::odometryCallback(msg, role_name);
    };
  };

  auto vehicleStatusArgCallback = [this](const std::string & role_name) {
    return [this, role_name](const cm::CarlaEgoVehicleStatus::ConstPtr msg) -> void {
      ItsConverter::vehicleStatusCallback(msg, role_name);
    };
  };

  auto vehicleInfoArgCallback = [this](const std::string & role_name) {
    return [this, role_name](const cm::CarlaEgoVehicleInfo::ConstPtr msg) -> void {
      ItsConverter::vehicleInfoCallback(msg, role_name);
    };
  };

  auto gnssArgCallback = [this](const std::string & role_name) {
    return [this, role_name](const ssm::NavSatFix::ConstPtr msg) -> void {
      ItsConverter::gnssCallback(msg, role_name);
    };
  };

  auto idealObjectsArgCallback = [this](const std::string & role_name) {
    return [this, role_name](const dom::ObjectArray::ConstPtr msg) -> void {
      ItsConverter::idealObjectsCallback(msg, role_name);
    };
  };

#ifdef ROS1
  // setup buffer
  tf2_buffer_.setUsingDedicatedThread(true);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(tf2_buffer_);
  
  // setup subscriber and publisher
  sub_objects_ = private_node_handle_.subscribe("/carla/objects", 1, &ItsConverter::objectsCallback, this);
  pub_objects_carla_map_ = private_node_handle_.advertise<pi::ObjectList>("/carla_its_converter/objects", 1);

  // setup subscriber and publisher depending on role_name
  for(std::string& role_name : role_names_) { 
    // remove spaces from role_name
    role_name.erase(std::remove_if(role_name.begin(), role_name.end(), isspace), role_name.end());

    // setup subscriber depending on role_name
    Subscriber<nm::Odometry> sub_odometry_ = private_node_handle_.subscribe<nm::Odometry>("/carla/" + role_name +"/odometry", 1, odometryArgCallback(role_name));
    Subscriber<cm::CarlaEgoVehicleStatus> sub_vehicle_status_ = private_node_handle_.subscribe<cm::CarlaEgoVehicleStatus>("/carla/" + role_name +"/vehicle_status", 1, vehicleStatusArgCallback(role_name));
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info_ = private_node_handle_.subscribe<cm::CarlaEgoVehicleInfo>("/carla/" + role_name +"/vehicle_info", 1, vehicleInfoArgCallback(role_name));
    Subscriber<ssm::NavSatFix> sub_gnss_ = private_node_handle_.subscribe<ssm::NavSatFix>("/carla/" + role_name +"/gnss", 1, gnssArgCallback(role_name));
    Subscriber<dom::ObjectArray> sub_ideal_objects = private_node_handle_.subscribe<dom::ObjectArray>("/carla/" + role_name +"/ideal_objects", 1, idealObjectsArgCallback(role_name));

    // save subscriber in map with role_name as key
    sub_odometry_map_.insert({role_name, sub_odometry_});
    sub_vehicle_status_map_.insert({role_name, sub_vehicle_status_});
    sub_vehicle_info_map_.insert({role_name, sub_vehicle_info_});
    sub_gnss_map_.insert({role_name, sub_gnss_});
    sub_ideal_objects_map_.insert({role_name, sub_ideal_objects});

    // setup publisher depending on role_name
    Publisher<pi::ObjectList> pub_objects = private_node_handle_.advertise<pi::ObjectList>("/carla_its_converter/" + role_name + "/objects", 1);
    Publisher<pi::EgoData> pub_ego_data = private_node_handle_.advertise<pi::EgoData>("/carla_its_converter/" + role_name + "/ego_data", 1);
    Publisher<etsi_cam::CAM> pub_etsi_cam = private_node_handle_.advertise<etsi_cam::CAM>("/carla_its_converter/" + role_name + "/etsi_its/cam", 1);
    Publisher<pi::ObjectList> pub_ideal_objects = private_node_handle_.advertise<pi::ObjectList>("/carla_its_converter/" + role_name + "/ideal_objects", 1);

    // save publisher in map with role_name as key
    pub_objects_map_.insert({role_name, pub_objects});
    pub_ego_data_map_.insert({role_name, pub_ego_data});
    pub_etsi_cam_map_.insert({role_name, pub_etsi_cam});
    pub_ideal_objects_map_.insert({role_name, pub_ideal_objects});
  }
  
#else
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
  
  // setup subscriber and publisher depending on role_name
  for(std::string& role_name : role_names_) { 
    // remove spaces from role_name
    role_name.erase(std::remove_if(role_name.begin(), role_name.end(), isspace), role_name.end());

    // setup subscriber depending on role_name    
    Subscriber<nm::Odometry> sub_odometry = this->create_subscription<nm::Odometry>("/carla/" + role_name +"/odometry", 1, odometryArgCallback(role_name));
    Subscriber<cm::CarlaEgoVehicleStatus> sub_vehicle_status = this->create_subscription<cm::CarlaEgoVehicleStatus>("/carla/" + role_name +"/vehicle_status", 1, vehicleStatusArgCallback(role_name));
    Subscriber<cm::CarlaEgoVehicleInfo> sub_vehicle_info = this->create_subscription<cm::CarlaEgoVehicleInfo>("/carla/" + role_name +"/vehicle_info", qosLatching, vehicleInfoArgCallback(role_name));
    Subscriber<ssm::NavSatFix> sub_gnss = this->create_subscription<ssm::NavSatFix>("/carla/" + role_name +"/gnss", 1, gnssArgCallback(role_name));
    Subscriber<dom::ObjectArray> sub_ideal_objects = this->create_subscription<dom::ObjectArray>("/carla/" + role_name +"/ideal_objects", 1, idealObjectsArgCallback(role_name));


    // save subscriber in map with role_name as key
    sub_odometry_map_.insert({role_name, sub_odometry});
    sub_vehicle_status_map_.insert({role_name, sub_vehicle_status});
    sub_vehicle_info_map_.insert({role_name, sub_vehicle_info});
    sub_gnss_map_.insert({role_name, sub_gnss});
    sub_ideal_objects_map_.insert({role_name, sub_ideal_objects});


    // setup publisher depending on role_name
    Publisher<pi::ObjectList> pub_objects = this->create_publisher<pi::ObjectList>("/carla_its_converter/" + role_name + "/objects", 1);
    Publisher<pi::EgoData> pub_ego_data = this->create_publisher<pi::EgoData>("/carla_its_converter/" + role_name + "/ego_data", 1);
    Publisher<etsi_cam::CAM> pub_etsi_cam = this->create_publisher<etsi_cam::CAM>("/carla_its_converter/" + role_name + "/etsi_its/cam", 1);
    Publisher<pi::ObjectList> pub_ideal_objects = this->create_publisher<pi::ObjectList>("/carla_its_converter/" + role_name + "/ideal_objects", 1);


    // save publisher in map with role_name as key
    pub_objects_map_.insert({role_name, pub_objects});
    pub_ego_data_map_.insert({role_name, pub_ego_data});
    pub_etsi_cam_map_.insert({role_name, pub_etsi_cam});
    pub_ideal_objects_map_.insert({role_name, pub_ideal_objects});
  }

#endif

  ROS_LOG_STREAM(INFO, "carla_its_converter running...");  
}


bool ItsConverter::loadParameters() {
  std::string role_names_string;

  // load publish parameters
#ifdef ROS1
  if(!private_node_handle_.param<std::string>("/carla_its_converter/role_names", role_names_string, "ego_vehicle")) {
    ROS_WARN("Parameter \'role_names\' not set, defaulting to %s.", role_names_string);
  }
  if(!private_node_handle_.param<double>("/carla_its_converter/pos_variances", pos_variances_, oa::CONTINUOUS_STATE_COVARIANCE_INVALID)) {
    ROS_WARN("Parameter \'pos_variances\' not set, defaulting to %f.", pos_variances_);
  }
  if(!private_node_handle_.param<double>("/carla_its_converter/vel_variances", vel_variances_, oa::CONTINUOUS_STATE_COVARIANCE_INVALID)) {
    ROS_WARN("Parameter \'vel_variances\' not set, defaulting to %f.", vel_variances_);
  }
  if(!private_node_handle_.param<double>("/carla_its_converter/acc_variances", acc_variances_, oa::CONTINUOUS_STATE_COVARIANCE_INVALID)) {
    ROS_WARN("Parameter \'acc_variances\' not set, defaulting to %f.", acc_variances_);
  }
  if(!private_node_handle_.param<double>("/carla_its_converter/yaw_variances", yaw_variances_, oa::CONTINUOUS_STATE_COVARIANCE_INVALID)) {
    ROS_WARN("Parameter \'yaw_variances\' not set, defaulting to %f.", yaw_variances_);
  }
  if(!private_node_handle_.param<double>("/carla_its_converter/yaw_rate_variances", yaw_rate_variances_, oa::CONTINUOUS_STATE_COVARIANCE_INVALID)) {
    ROS_WARN("Parameter \'yaw_rate_variances\' not set, defaulting to %f.", yaw_rate_variances_);
  }
#else
  this->declare_parameter("role_names", "ego_vehicle");
  try {
    role_names_string = this->get_parameter("role_names").as_string();
  } catch (rclcpp::exceptions::ParameterNotDeclaredException&) {
    role_names_string = "ego_vehicle";
    ROS_LOG_STREAM(WARN, "Parameter \'role_names\' not set, defaulting to " << role_names_string);
  }
  this->declare_parameter("pos_variances", oa::CONTINUOUS_STATE_COVARIANCE_INVALID);
  pos_variances_ = this->get_parameter("pos_variances").as_double();
  this->declare_parameter("vel_variances", oa::CONTINUOUS_STATE_COVARIANCE_INVALID);
  vel_variances_ = this->get_parameter("vel_variances").as_double();
  this->declare_parameter("acc_variances", oa::CONTINUOUS_STATE_COVARIANCE_INVALID);
  acc_variances_ = this->get_parameter("acc_variances").as_double();
  this->declare_parameter("yaw_variances", oa::CONTINUOUS_STATE_COVARIANCE_INVALID);
  yaw_variances_ = this->get_parameter("yaw_variances").as_double();
  this->declare_parameter("yaw_rate_variances", oa::CONTINUOUS_STATE_COVARIANCE_INVALID);
  yaw_rate_variances_ = this->get_parameter("yaw_rate_variances").as_double();
#endif

  std::string role_name;
  std::stringstream role_names_string_stream(role_names_string);

  // save role_names in vector
  while (std::getline(role_names_string_stream, role_name, ',')) {
    role_names_.push_back(role_name);
  }

  return true;
}

void ItsConverter::gnssCallback(const ssm::NavSatFix::ConstPtr msg, std::string role_name) {
  // get gnss position from role_name vehicle
  ego_gnss_map_[role_name] = *msg;
  ego_gnss_set_map_[role_name] = true;
}

pi::ObjectList ItsConverter::convertObjectArray(const dom::ObjectArray::ConstPtr msg) {
  // map the objects from the CARLA format to the perception_msgs format
  pi::ObjectList msg_object_list_;
  msg_object_list_.header = msg->header;

  for (size_t i = 0; i < msg->objects.size(); i++) {
    // only add classified objects
    if(!msg->objects[i].object_classified) continue;
    
    for(auto & role_name : role_names_){
      // only continue if ego_info_set_ 
      if (!ego_info_set_map_[role_name]) continue;

      // get shape from role_name vehicle
      if(ego_id_map_[role_name] == msg->objects[i].id){
        ego_shape_map_[role_name] = msg->objects[i].shape;
        ego_shape_set_map_[role_name] = true;
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
    oa::initializeState(objectTemp, pi::ISCACTR::MODEL_ID);
    objectTemp.state.header = msg->objects[i].header;
    oa::setPose(objectTemp.state, msg->objects[i].pose);
    oa::setVelocityXYZYaw(objectTemp.state, msg->objects[i].twist.linear, yaw);
    oa::setAccelerationXYZYaw(objectTemp.state, msg->objects[i].accel.linear, yaw);
    oa::setYawRate(objectTemp.state, msg->objects[i].twist.angular.z);
    oa::setLength(objectTemp, msg->objects[i].shape.dimensions[0]);
    oa::setWidth(objectTemp, msg->objects[i].shape.dimensions[1]);
    oa::setHeight(objectTemp, msg->objects[i].shape.dimensions[2]);

    // Fill variances
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::X, pi::ISCACTR::X, pos_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::Y, pi::ISCACTR::Y, pos_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::Z, pi::ISCACTR::Z, pos_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::VEL_LON, pi::ISCACTR::VEL_LON,
      vel_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::VEL_LAT, pi::ISCACTR::VEL_LAT,
      vel_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::ACC_LON, pi::ISCACTR::ACC_LON,
      acc_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::ACC_LAT, pi::ISCACTR::ACC_LAT,
      acc_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::YAW, pi::ISCACTR::YAW, yaw_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::YAW_RATE, pi::ISCACTR::YAW_RATE,
      yaw_rate_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::LENGTH, pi::ISCACTR::LENGTH, pos_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::WIDTH, pi::ISCACTR::WIDTH, pos_variances_);
    oa::setContinuousStateCovarianceAt(
      objectTemp, pi::ISCACTR::HEIGHT, pi::ISCACTR::HEIGHT, pos_variances_);

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

void ItsConverter::objectsCallback(const dom::ObjectArray::ConstPtr msg) {
  pi::ObjectList msg_object_list_ = ItsConverter::convertObjectArray(msg);

  // publish object_list in carla_map frame
#ifdef ROS1
    pub_objects_carla_map_.publish(msg_object_list_);
#else
    pub_objects_carla_map_->publish(msg_object_list_);
#endif

  // iterate over each role_name
  for(std::string & role_name: role_names_){
#ifdef ROS1
    auto timeout = ros::Duration(1.0);
#else
    auto timeout = rclcpp::Duration::from_seconds(1.0);
#endif
    pi::ObjectList msg_object_list_copy = msg_object_list_;

    // Set increasing sensor ID per role name. As `role_names_` doesnt change, this assigns a fixed sensor ID to each role.
    for (auto &object: msg_object_list_copy.objects)
    {
      object.state.sensor_id[0] = ego_id_map_[role_name];
    }

    // remove element with the role_name id
    msg_object_list_copy.objects.erase(std::remove_if(msg_object_list_copy.objects.begin(), msg_object_list_copy.objects.end(), [this, role_name](const pi::Object& obj) {
      return obj.id == ego_id_map_[role_name];
    }), msg_object_list_copy.objects.end());

    // transform the object_list from carla_map to role_name frame
    pi::ObjectList msg_object_list_role_name;
    gm::TransformStamped carla_map_to_role_name_tf;
#ifdef ROS1
    try {
      // get transformation from carla_map to role_name
      if (tf2_buffer_._frameExists(role_name)){
        carla_map_to_role_name_tf = tf2_buffer_.lookupTransform(role_name, "carla_map", msg_object_list_copy.header.stamp, timeout);
      } else {
        ROS_LOG_STREAM(WARN, "Frame '%s' does not exist", std::string(role_name).c_str());
        show_transform_success_map_[role_name] = true;
          continue;
      }

      tf2::doTransform(msg_object_list_copy, msg_object_list_role_name, carla_map_to_role_name_tf);
      
      // log success once
      if(!show_transform_success_map_.count(role_name) || show_transform_success_map_[role_name]){
        ROS_LOG_STREAM(INFO, "Transformation from 'carla_map' to '%s' published successfully", std::string(role_name).c_str());
        show_transform_success_map_[role_name] = false;
      } 
    } catch (tf2::TransformException& e) {
      ROS_LOG_STREAM(WARN, "Transformation from 'carla_map' to '%s' is not available", role_name.c_str());
      show_transform_success_map_[role_name] = true;
        continue;
    }
#else
    try {
      // get transformation from carla_map to role_name
      if (tf2_buffer_->_frameExists(role_name)){
        carla_map_to_role_name_tf = tf2_buffer_->lookupTransform(role_name, "carla_map", msg_object_list_copy.header.stamp, timeout);
      } else {
        ROS_LOG_STREAM(WARN, "Frame '"<< role_name << "' does not exist");
        show_transform_success_map_[role_name] = true;
          continue;
      }

      tf2::doTransform(msg_object_list_copy, msg_object_list_role_name, carla_map_to_role_name_tf);
      
      // log success once
      if(!show_transform_success_map_.count(role_name) || show_transform_success_map_[role_name]){
        ROS_LOG_STREAM(INFO, "Tranformation from 'carla_map' to '" << role_name << "' published successfully");
        show_transform_success_map_[role_name] = false;
      } 
    } catch (tf2::TransformException& e) {
      ROS_LOG_STREAM(WARN, "Tranformation from 'carla_map' to '" << role_name << "' is not available");
      show_transform_success_map_[role_name] = true;
        continue;
    }
#endif

    // publish object list in role_name frame
#ifdef ROS1
    pub_objects_map_[role_name].publish(msg_object_list_role_name);
#else
    pub_objects_map_[role_name]->publish(msg_object_list_role_name);
#endif
  }
}

void ItsConverter::idealObjectsCallback(const dom::ObjectArray::ConstPtr msg, std::string role_name) {
  pi::ObjectList msg_object_list_ = ItsConverter::convertObjectArray(msg);

    // publish ideal object list in role_name frame
#ifdef ROS1
    pub_ideal_objects_map_[role_name].publish(msg_object_list_);
#else
    pub_ideal_objects_map_[role_name]->publish(msg_object_list_);
#endif 
}

void ItsConverter::odometryCallback(const nm::Odometry::ConstPtr msg, std::string role_name) {
  // map ego data from CARLA to the perception_msgs format
  if(ego_shape_set_map_[role_name] && ego_status_set_map_[role_name]){
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
    oa::setZ(msg_ego_data_, msg->pose.pose.position.z + ego_shape_map_[role_name].dimensions[2] / 2.0); // set z to the center of the object
    
    // twist in 
    oa::setYawRate(msg_ego_data_.state, msg->twist.twist.angular.z);  // twist is defined in child frame (no transformation needed)
    oa::setVelocity(msg_ego_data_.state, msg->twist.twist.linear);    // twist is defined in child frame (no transformation needed)
    oa::setAccelerationXYZYaw(msg_ego_data_.state, ego_acceleration_map_[role_name].linear, yaw); // accleration defined in carla_map frame (transformation needed)

    // SteeringAngleMax is given in rad (contrary to the description in the documentation)
    // https://github.com/carla-simulator/ros-bridge/blob/e9063d97ff5a724f76adbb1b852dc71da1dcfeec/carla_ros_bridge/src/carla_ros_bridge/ego_vehicle.py#L145C46-L145C81
    oa::setSteeringAngleAck(msg_ego_data_.state, -ego_steering_angle_map_[role_name]*ego_steering_angle_max_map_[role_name]);
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
    msg_ego_data_.vehicle_id = ego_id_map_[role_name];
    msg_ego_data_.length = ego_shape_map_[role_name].dimensions[0];
    msg_ego_data_.width = ego_shape_map_[role_name].dimensions[1];
    msg_ego_data_.height = ego_shape_map_[role_name].dimensions[2];


    // publish ego_data in carla_map frame
#ifdef ROS1
    pub_ego_data_map_[role_name].publish(msg_ego_data_);
#else
    pub_ego_data_map_[role_name]->publish(msg_ego_data_);
#endif 

    // additionally publish etsi_cam
    if (ego_gnss_set_map_[role_name]) {
      etsi_cam::CAM msg_cam = convertToEtsiCam(msg_ego_data_, ego_gnss_map_[role_name]);

#ifdef ROS1
      pub_etsi_cam_map_[role_name].publish(msg_cam);
#else
      pub_etsi_cam_map_[role_name]->publish(msg_cam);
#endif 
    }

  } 
}

void ItsConverter::vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg, std::string role_name){
  // get steering_angle and acceleration from role_name vehicle
  ego_steering_angle_map_[role_name] = msg->control.steer;
  ego_acceleration_map_[role_name] = msg->acceleration;
  ego_status_set_map_[role_name] = true;
}

void ItsConverter::vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg, std::string role_name){
  // get id from role_name vehicle
  ego_id_map_[role_name] = msg->id;
  ego_steering_angle_max_map_[role_name] = 0.0;
  for(int i=0; i<msg->wheels.size(); i++){
    ego_steering_angle_max_map_[role_name] = std::max(ego_steering_angle_max_map_[role_name], (double)msg->wheels[i].max_steer_angle);
  }

  ego_info_set_map_[role_name] = true;
}

etsi_cam::CAM ItsConverter::convertToEtsiCam(const pi::EgoData &ego_data, ssm::NavSatFix gnss) {
  
  etsi_cam::CAM cam;
  
  ca::setItsPduHeader(cam, ego_data.vehicle_id);
  
#ifdef ROS1
  uint64_t nns = ego_data.header.stamp.sec * 1e9 + ego_data.header.stamp.nsec;
#else
  uint64_t nns = ego_data.header.stamp.sec * 1e9 + ego_data.header.stamp.nanosec;
#endif
    
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

}  // end of namespace


int main(int argc, char **argv)
{
#ifdef ROS1
  ros::init(argc, argv, "CarlaItsConverter");
  carla::ItsConverter node;
  ros::spin();
#else
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<carla::ItsConverter>());
  rclcpp::shutdown();
#endif
  return 0;
}

