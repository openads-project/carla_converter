#include <CarlaItsConverterNode.h>


namespace carla {

#ifdef MODE_ROS1
ItsConverter::ItsConverter() {
  sub_objects_ = private_node_handle_.subscribe("/carla/objects", 1, &ItsConverter::objectsCallback, this);
  sub_odometry_ = private_node_handle_.subscribe("/carla/ego_vehicle/odometry", 1, &ItsConverter::odometryCallback, this);
  sub_vehicle_status_ = private_node_handle_.subscribe("/carla/ego_vehicle/vehicle_status", 1, &ItsConverter::vehicleStatusCallback, this);
  sub_vehicle_info_ = private_node_handle_.subscribe("/carla/ego_vehicle/vehicle_info", 1, &ItsConverter::vehicleInfoCallback, this);
  pub_objects_carla_map_ = private_node_handle_.advertise<pin::ObjectList>("/carla_its_converter/objectList/carla_map", 1);
  pub_objects_ego_vehicle_ = private_node_handle_.advertise<pin::ObjectList>("/carla_its_converter/objectList/ego_vehicle", 1);
  pub_ego_data_ = private_node_handle_.advertise<pin::EgoData>("/carla_its_converter/egoData", 1);
  
  tf2_buffer_.setUsingDedicatedThread(true);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(tf2_buffer_);

#elif MODE_ROS2
ItsConverter::ItsConverter() : Node("CarlaItsConverter") {
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  sub_objects_ = this->create_subscription<dom::ObjectArray>("/carla/objects", 1, std::bind(&ItsConverter::objectsCallback, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<nam::Odometry>("/carla/ego_vehicle/odometry", 1, std::bind(&ItsConverter::odometryCallback, this, std::placeholders::_1));
  sub_vehicle_status_ = this->create_subscription<cm::CarlaEgoVehicleStatus>("/carla/ego_vehicle/vehicle_status", 1, std::bind(&ItsConverter::vehicleStatusCallback, this, std::placeholders::_1));
  
  rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
  qosLatching.transient_local();
  qosLatching.reliable();
  sub_vehicle_info_ = this->create_subscription<cm::CarlaEgoVehicleInfo>("/carla/ego_vehicle/vehicle_info", qosLatching, std::bind(&ItsConverter::vehicleInfoCallback, this, std::placeholders::_1));

  pub_objects_carla_map_ = this->create_publisher<pin::ObjectList>("/carla_its_converter/objectList/carla_map", 1);
  pub_objects_ego_vehicle_ = this->create_publisher<pin::ObjectList>("/carla_its_converter/objectList/ego_vehicle", 1);
  pub_ego_data_ = this->create_publisher<pin::EgoData>("/carla_its_converter/egoData", 1);
  
  tf2_buffer_->setUsingDedicatedThread(true);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
#endif

  // Load Parameters and if not successful, return
  if(!loadParameters()) return;

  ROS_LOG_STREAM(INFO, "CarlaItsConverter running...");  
}


bool ItsConverter::loadParameters() {
  // Load publish parameters
  std::vector<std::string> v_parameter_str = {"object_list_carla_map_frame", "object_list_ego_vehicle_frame", "ego_data"};
  std::vector<bool> v_parameter_bool;
  for (auto parameter_str : v_parameter_str) {
#ifdef MODE_ROS1
    bool b_param;
    if (!private_node_handle_.getParam("ItsConverterNode/ros__parameters/publish/" + parameter_str, b_param)) {
      std::string error_msg = "Parameter \'ItsConverterNode/ros__parameters/publish/" + parameter_str + "\' is required";
      ROS_LOG_STREAM(ERROR, error_msg.c_str());
      return false;
    }
    v_parameter_bool.push_back(b_param);
#elif MODE_ROS2
    this->declare_parameter("publish." + parameter_str, false);
    try {
      v_parameter_bool.push_back(this->get_parameter("publish." + parameter_str).as_bool());
    } catch (rclcpp::exceptions::ParameterNotDeclaredException&) {
      std::string error_msg = "Parameter \'publish." + parameter_str + "\' is required";
      ROS_LOG_STREAM(ERROR, error_msg);
      return false;
    }
#endif
  }
  publish_object_list_carla_map_frame_ = v_parameter_bool[0];
  publish_object_list_ego_vehicle_frame_ = v_parameter_bool[1];
  publish_ego_data_ = v_parameter_bool[2];

  return true;
}

void ItsConverter::objectsCallback(const dom::ObjectArray::ConstPtr msg) {
  // Map the objects from the CARLA format to the perception_interfaces format
  msg_object_list_.objects.clear();
  msg_object_list_.header = msg->header;

  for (size_t i = 0; i < msg->objects.size(); i++) {
    if(msg->objects[i].object_classified) { // Insanity Check: Only add objects that are classified
      if(ego_id_set_){
        if(ego_id_ == msg->objects[i].id){
          ego_shape_ = msg->objects[i].shape;
          ego_shape_set_ = true;
        } else {
          pin::Object objectTemp;
          objectTemp.id = msg->objects[i].id;
          objectTemp.existence_probability = 1.0; // Probability that the object exists is always 1.0 as source is CARLA
          obj_acc::initializeState(objectTemp, pin::ISCACTR::MODEL_ID);
          objectTemp.state.header = msg->objects[i].header;  // optional
          obj_acc::setPose(objectTemp.state, msg->objects[i].pose);
          obj_acc::setZ(objectTemp, msg->objects[i].pose.position.z + msg->objects[i].shape.dimensions[2] / 2.0); // Set z to the center of the object
          
          // get yaw angle
          double roll, pitch, yaw;
          tf2::Quaternion quat_tf;
          tf2::fromMsg(msg->objects[i].pose.orientation, quat_tf);
          tf2::Matrix3x3 matrix(quat_tf);
          matrix.getRPY(roll, pitch, yaw);

          obj_acc::setVelocityXYZYaw(objectTemp.state, msg->objects[i].twist.linear, yaw);
          obj_acc::setAccelerationXYZYaw(objectTemp.state, msg->objects[i].accel.linear, yaw);
          obj_acc::setYawRate(objectTemp.state, msg->objects[i].twist.angular.z);
          obj_acc::setLength(objectTemp, msg->objects[i].shape.dimensions[0]);
          obj_acc::setWidth(objectTemp, msg->objects[i].shape.dimensions[1]);
          obj_acc::setHeight(objectTemp, msg->objects[i].shape.dimensions[2]);

          objectTemp.state.classifications.resize(1);
          objectTemp.state.classifications[0].probability = 1.0; // Probability that the object is of this type is always 1.0 as source is CARLA
          switch ((int) msg->objects[i].classification)
          {
            case dom::Object::CLASSIFICATION_PEDESTRIAN:
            objectTemp.state.classifications[0].type = pin::ObjectClassification::PEDESTRIAN;
            break;
          case dom::Object::CLASSIFICATION_BIKE:
            objectTemp.state.classifications[0].type = pin::ObjectClassification::BICYCLE;
            break;
          case dom::Object::CLASSIFICATION_MOTORCYCLE:
            objectTemp.state.classifications[0].type = pin::ObjectClassification::MOTORBIKE;
            break;
          case dom::Object::CLASSIFICATION_CAR:
            objectTemp.state.classifications[0].type = pin::ObjectClassification::CAR;
            break;
          case dom::Object::CLASSIFICATION_TRUCK:
            objectTemp.state.classifications[0].type = pin::ObjectClassification::TRUCK;
            break;
          case dom::Object::CLASSIFICATION_BARRIER:
            objectTemp.state.classifications[0].type = pin::ObjectClassification::ROAD_OBSTACLE;
            break;
          default:
            objectTemp.state.classifications[0].type = pin::ObjectClassification::UNCLASSIFIED;
            break;
          }

          msg_object_list_.objects.push_back(objectTemp);
        }
      }
    }
  }

  if(publish_object_list_carla_map_frame_){
    // publish objectList in carla_map frame
#ifdef MODE_ROS1
    pub_objects_carla_map_.publish(msg_object_list_);
#elif MODE_ROS2
    pub_objects_carla_map_->publish(msg_object_list_);
#endif
  }

  if(publish_object_list_ego_vehicle_frame_){
#ifdef MODE_ROS1
    auto timeout = ros::Duration(1.0);
#elif MODE_ROS2
    auto timeout = rclcpp::Duration::from_seconds(1.0);
#endif

    // Transform the objectList from carla_map to ego_vehicle
    pin::ObjectList msg_object_list_ego_vehicle;
    gm::TransformStamped carla_map_to_ego_vehicle_tf;
    try {
#ifdef MODE_ROS1
      carla_map_to_ego_vehicle_tf = tf2_buffer_.lookupTransform("ego_vehicle", "carla_map", msg_object_list_.header.stamp, timeout);
#elif MODE_ROS2
      carla_map_to_ego_vehicle_tf = tf2_buffer_->lookupTransform("ego_vehicle", "carla_map", msg_object_list_.header.stamp, timeout);
#endif
    } catch (tf2::TransformException& ex) {
      ROS_LOG_STREAM(ERROR, "\"Exception caught: \" << ex.what()");
      return;
    }
    tf2::doTransform(msg_object_list_, msg_object_list_ego_vehicle, carla_map_to_ego_vehicle_tf);

    // publish objectList in ego_vehicle frame
#ifdef MODE_ROS1
    pub_objects_ego_vehicle_.publish(msg_object_list_ego_vehicle);
#elif MODE_ROS2
    pub_objects_ego_vehicle_->publish(msg_object_list_ego_vehicle);
#endif
  }
}

void ItsConverter::odometryCallback(const nam::Odometry::ConstPtr msg) {
  // Map the ego from the CARLA format to the perception_interfaces format
  if(publish_ego_data_){ 
    if(ego_shape_set_ && ego_status_set_){
      msg_ego_data_.header = msg->header;
      obj_acc::initializeState(msg_ego_data_, 1); //TODO constant instead of 1?
      obj_acc::setPose(msg_ego_data_.state, msg->pose.pose);
      obj_acc::setZ(msg_ego_data_, msg->pose.pose.position.z + ego_shape_.dimensions[2] / 2.0); // Set z to the center of the object
      obj_acc::setYawRate(msg_ego_data_.state, msg->twist.twist.angular.z);

      // get yaw angle
      double roll, pitch, yaw;
      tf2::Quaternion quat_tf;
      tf2::fromMsg(msg->pose.pose.orientation, quat_tf);
      tf2::Matrix3x3 matrix(quat_tf);
      matrix.getRPY(roll, pitch, yaw);

      obj_acc::setVelocityXYZYaw(msg_ego_data_.state, msg->twist.twist.linear, yaw);
      obj_acc::setAccelerationXYZYaw(msg_ego_data_.state, ego_acceleration_.linear, yaw);
      obj_acc::setSteeringAngleAck(msg_ego_data_.state, ego_steering_angle_);
      // obj_acc::setSteeringAngleRateAck(msg_ego_data_.state, msg); //TODO no data available
      obj_acc::setStandstill(msg_ego_data_.state, (msg->twist.twist.linear.x + msg->twist.twist.linear.y + msg->twist.twist.linear.z) == 0);

      // # continuous state covariance matrix (N*N flattened)
      // float64[] continuous_state_covariance

      // # classification incl. probabilities
      // ObjectClassification[] classifications

      // # reference point for object position
      // ObjectReferencePoint reference_point

      // # Planned trajectory of the ego_vehicle
      // ObjectState[] trajectory_planned

      // # Past trajectory of the ego_vehicle
      // ObjectState[] trajectory_past

      // # Planned route of the ego_vehicle
      // geometry_msgs/Point[] route_planned

      msg_ego_data_.vehicle_id = ego_id_;
      msg_ego_data_.length = ego_shape_.dimensions[0];
      msg_ego_data_.width = ego_shape_.dimensions[1];
      msg_ego_data_.height = ego_shape_.dimensions[2];

      // publish egoData in carla_map frame
#ifdef MODE_ROS1
      pub_ego_data_.publish(msg_ego_data_);
#elif MODE_ROS2
      pub_ego_data_->publish(msg_ego_data_);
#endif 
    } 
  }
}

void ItsConverter::vehicleStatusCallback(const cm::CarlaEgoVehicleStatus::ConstPtr msg){
  // get steering_angle and acceleration form ego
  ego_steering_angle_ = msg->control.steer;
  ego_acceleration_ = msg->acceleration;
  ego_status_set_ = true;
}

void ItsConverter::vehicleInfoCallback(const cm::CarlaEgoVehicleInfo::ConstPtr msg){
  // get id from ego
  ego_id_ = msg->id;
  ego_id_set_ = true;
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

