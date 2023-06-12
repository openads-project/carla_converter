#include <CarlaItsConverterNode.h>


namespace carla {

#ifdef MODE_ROS1
ItsConverter::ItsConverter() {
  tf2_buffer_.setUsingDedicatedThread(true);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(tf2_buffer_);

  sub_objects_ = private_node_handle_.subscribe("/carla/objects", 1, &ItsConverter::objectsCallback, this);
  sub_odometry_ = private_node_handle_.subscribe("/carla/ego_vehicle/odometry", 1, &ItsConverter::odometryCallback, this);
  sub_vehicle_status_ = private_node_handle_.subscribe("/carla/ego_vehicle/vehicle_status", 1, &ItsConverter::vehicleStatusCallback, this);
  sub_vehicle_info_ = private_node_handle_.subscribe("/carla/ego_vehicle/vehicle_info", 1, &ItsConverter::vehicleInfoCallback, this);
  
  pub_objects_carla_map_ = private_node_handle_.advertise<pi::ObjectList>("/carla_its_converter/object_list/carla_map", 1);
  pub_objects_ego_vehicle_ = private_node_handle_.advertise<pi::ObjectList>("/carla_its_converter/object_list/ego_vehicle", 1);
  pub_ego_data_ = private_node_handle_.advertise<pi::EgoData>("/carla_its_converter/ego_data", 1);
  
#elif MODE_ROS2
ItsConverter::ItsConverter() : Node("CarlaItsConverter") {
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_buffer_->setUsingDedicatedThread(true);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  
  rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
  qosLatching.transient_local();
  qosLatching.reliable();
  
  sub_objects_ = this->create_subscription<dom::ObjectArray>("/carla/objects", 1, std::bind(&ItsConverter::objectsCallback, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<nm::Odometry>("/carla/ego_vehicle/odometry", 1, std::bind(&ItsConverter::odometryCallback, this, std::placeholders::_1));
  sub_vehicle_status_ = this->create_subscription<cm::CarlaEgoVehicleStatus>("/carla/ego_vehicle/vehicle_status", 1, std::bind(&ItsConverter::vehicleStatusCallback, this, std::placeholders::_1));
  sub_vehicle_info_ = this->create_subscription<cm::CarlaEgoVehicleInfo>("/carla/ego_vehicle/vehicle_info", qosLatching, std::bind(&ItsConverter::vehicleInfoCallback, this, std::placeholders::_1));

  pub_objects_carla_map_ = this->create_publisher<pi::ObjectList>("/carla_its_converter/object_list/carla_map", 1);
  pub_objects_ego_vehicle_ = this->create_publisher<pi::ObjectList>("/carla_its_converter/object_list/ego_vehicle", 1);
  pub_ego_data_ = this->create_publisher<pi::EgoData>("/carla_its_converter/ego_data", 1);
  
#endif

  // load Parameters and if not successful, return
  if(!loadParameters()) return;

  ROS_LOG_STREAM(INFO, "CarlaItsConverter running...");  
}


bool ItsConverter::loadParameters() {
  // load publish parameters
#ifdef MODE_ROS1
  if(!private_node_handle_.param<bool>("/carla_its_converter/publish_ego_vehicle", publish_ego_vehicle_, false)) {
    ROS_WARN("publish_ego_vehicle not set, defaulting to %d.", publish_ego_vehicle_);
  }
#elif MODE_ROS2
  this->declare_parameter("publish_ego_vehicle", true);
  try {
    publish_ego_vehicle_ = this->get_parameter("publish_ego_vehicle").as_bool();
  } catch (rclcpp::exceptions::ParameterNotDeclaredException&) {
    ROS_LOG_STREAM(ERROR, "Parameter \'publish_ego_vehicle\' is required");
    return false;
  }
#endif

  return true;
}

void ItsConverter::objectsCallback(const dom::ObjectArray::ConstPtr msg) {
  // map the objects from the CARLA format to the perception_interfaces format
  msg_object_list_.objects.clear();
  msg_object_list_.header = msg->header;

  for (size_t i = 0; i < msg->objects.size(); i++) {
    // only add classified objects
    if(!msg->objects[i].object_classified) continue;
    
    // only continue if ego_info_set_ 
    if (!ego_info_set_) continue;

    // only get shape from ego vehicle
    if(ego_id_ == msg->objects[i].id){
      ego_shape_ = msg->objects[i].shape;
      ego_shape_set_ = true;
      continue;
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
    oa::setZ(objectTemp, msg->objects[i].pose.position.z + msg->objects[i].shape.dimensions[2] / 2.0); // Set z to the center of the object
    oa::setVelocityXYZYaw(objectTemp.state, msg->objects[i].twist.linear, yaw);
    oa::setAccelerationXYZYaw(objectTemp.state, msg->objects[i].accel.linear, yaw);
    oa::setYawRate(objectTemp.state, msg->objects[i].twist.angular.z);
    oa::setLength(objectTemp, msg->objects[i].shape.dimensions[0]);
    oa::setWidth(objectTemp, msg->objects[i].shape.dimensions[1]);
    oa::setHeight(objectTemp, msg->objects[i].shape.dimensions[2]);

    // classification for object state
    objectTemp.state.classifications.resize(1);
    objectTemp.state.classifications[0].probability = 1.0; // Probability that the object is of this type is always 1.0 as source is CARLA
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

    // reference point for object state position
    objectTemp.state.reference_point.value = pi::ObjectReferencePoint::GEOMETRIC_CENTER;

    // add object to object list
    msg_object_list_.objects.push_back(objectTemp);
  }

  // publish object_list in carla_map frame
#ifdef MODE_ROS1
    pub_objects_carla_map_.publish(msg_object_list_);
#elif MODE_ROS2
    pub_objects_carla_map_->publish(msg_object_list_);
#endif

  if(publish_ego_vehicle_){
#ifdef MODE_ROS1
    auto timeout = ros::Duration(1.0);
#elif MODE_ROS2
    auto timeout = rclcpp::Duration::from_seconds(1.0);
#endif

    // transform the object_list from carla_map to ego_vehicle
    pi::ObjectList msg_object_list_ego_vehicle;
    gm::TransformStamped carla_map_to_ego_vehicle_tf;
    try {
#ifdef MODE_ROS1
      if (tf2_buffer_._frameExists("ego_vehicle")){
        carla_map_to_ego_vehicle_tf = tf2_buffer_.lookupTransform("ego_vehicle", "carla_map", msg_object_list_.header.stamp, timeout);
      } else {
        ROS_LOG_STREAM(WARN, "Tranformation from 'carla_map' to 'ego_vehicle' is not available");
        show_next_transform_success_log_ = true;
        return;
      }
#elif MODE_ROS2
      if (tf2_buffer_->_frameExists("ego_vehicle")){
        carla_map_to_ego_vehicle_tf = tf2_buffer_->lookupTransform("ego_vehicle", "carla_map", msg_object_list_.header.stamp, timeout);
      } else {
        ROS_LOG_STREAM(WARN, "Tranformation from 'carla_map' to 'ego_vehicle' is not available");
        show_next_transform_success_log_ = true;
        return;
      }
#endif
      tf2::doTransform(msg_object_list_, msg_object_list_ego_vehicle, carla_map_to_ego_vehicle_tf);
      if(show_next_transform_success_log_){
        ROS_LOG_STREAM(INFO, "Tranformation from 'carla_map' to 'ego_vehicle' published successfully");
        show_next_transform_success_log_ = false;
      } 
    } catch (tf2::TransformException& e) {
      ROS_LOG_STREAM(WARN, "Tranformation from 'carla_map' to 'ego_vehicle' is not available");
      show_next_transform_success_log_ = true;
      return;
    }

    // publish object list in ego_vehicle frame
#ifdef MODE_ROS1
    pub_objects_ego_vehicle_.publish(msg_object_list_ego_vehicle);
#elif MODE_ROS2
    pub_objects_ego_vehicle_->publish(msg_object_list_ego_vehicle);
#endif
  }
}

void ItsConverter::odometryCallback(const nm::Odometry::ConstPtr msg) {
  // map ego data from CARLA to the perception_interfaces format
  if(ego_shape_set_ && ego_status_set_){
    msg_ego_data_.header = msg->header;
    
    // get euler angles
    double roll, pitch, yaw;
    tf2::Quaternion quat_tf;
    tf2::fromMsg(msg->pose.pose.orientation, quat_tf);
    tf2::Matrix3x3 matrix(quat_tf);
    matrix.getRPY(roll, pitch, yaw);

    // fill state
    oa::initializeState(msg_ego_data_, pi::EGO::MODEL_ID);
    oa::setPose(msg_ego_data_.state, msg->pose.pose);
    oa::setZ(msg_ego_data_, msg->pose.pose.position.z + ego_shape_.dimensions[2] / 2.0); // Set z to the center of the object
    oa::setYawRate(msg_ego_data_.state, msg->twist.twist.angular.z);
    oa::setVelocityXYZYaw(msg_ego_data_.state, msg->twist.twist.linear, yaw);
    oa::setAccelerationXYZYaw(msg_ego_data_.state, ego_acceleration_.linear, yaw);
    oa::setSteeringAngleAck(msg_ego_data_.state, -ego_steering_angle_*(ego_steering_angle_max_ * (M_PI / 180)));
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

    // fill ego data
    msg_ego_data_.vehicle_id = ego_id_;
    msg_ego_data_.length = ego_shape_.dimensions[0];
    msg_ego_data_.width = ego_shape_.dimensions[1];
    msg_ego_data_.height = ego_shape_.dimensions[2];

    // publish ego_data in carla_map frame
#ifdef MODE_ROS1
    pub_ego_data_.publish(msg_ego_data_);
#elif MODE_ROS2
    pub_ego_data_->publish(msg_ego_data_);
#endif 
  } 
}

void ItsConverter::vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg){
  // get steering_angle and acceleration from ego vehicle
  ego_steering_angle_ = msg->control.steer;
  ego_acceleration_ = msg->acceleration;
  ego_status_set_ = true;
}

void ItsConverter::vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg){
  // get id from ego vehicle
  ego_id_ = msg->id;
  ego_steering_angle_max_=0.0;
  for(int i=0; i<msg->wheels.size(); i++)
  {
    ego_steering_angle_max_ = std::max(ego_steering_angle_max_, (double)msg->wheels[i].max_steer_angle);
  }

  ego_info_set_ = true;
}
}  // end of namespace


int main(int argc, char **argv)
{
#ifdef MODE_ROS1
  ros::init(argc, argv, "CarlaItsConverter");
  carla::ItsConverter node;
  ros::spin();
#elif MODE_ROS2
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<carla::ItsConverter>());
  rclcpp::shutdown();
#endif
  return 0;
}

