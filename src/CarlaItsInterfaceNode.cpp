#include <CarlaItsInterfaceNode.h>


namespace carla {

#ifdef MODE_ROS1
ItsInterface::ItsInterface() {
  sub_objects_ = private_node_handle_.subscribe("/carla/ego_vehicle/objects", 1, &ItsInterface::objectsCallback, this);
  sub_odometry_ = private_node_handle_.subscribe("/carla/ego_vehicle/odometry", 1, &ItsInterface::odometryCallback, this);

  pub_objects_carla_map_ = private_node_handle_.advertise<pin::ObjectList>("/carla_its_interface/objectList/carla_map", 1);
  pub_objects_ego_vehicle_ = private_node_handle_.advertise<pin::ObjectList>("/carla_its_interface/objectList/ego_vehicle", 1);
  pub_objects_map_ = private_node_handle_.advertise<pin::ObjectList>("/global/objectList", 1);
  pub_objects_base_link_ = private_node_handle_.advertise<pin::ObjectList>("/carla_its_interface/objectList/base_link", 1);

  // Load publish parameters
  std::vector<std::string> v_parameter_str = {"publish/carla_map", "publish/ego_vehicle", "publish/map", "publish/base_link"};
  std::vector<bool> v_parameter_bool;
  for (auto parameter_str : v_parameter_str) {
    bool b_param;
    if (!private_node_handle_.getParam("ItsInterfaceNode/ros__parameters/" + parameter_str, b_param)) {
      std::string error_msg = "Parameter \'ItsInterfaceNode/ros__parameters/" + parameter_str + "\' is required";
      ROS_LOG_STREAM(ERROR, "Parameter Error"); // ToDo: Make Error with error_msg work
      return;
    }
    v_parameter_bool.push_back(b_param);
  }
  publish_carla_ = v_parameter_bool[0];
  publish_ego_vehicle_ = v_parameter_bool[1];
  publish_map_ = v_parameter_bool[2];
  publish_base_link_ = v_parameter_bool[3];

  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(tf2_buffer_);
  

#elif MODE_ROS2
ItsInterface::ItsInterface() : Node("CarlaItsInterface") {
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  sub_objects_ = this->create_subscription<dom::ObjectArray>("/carla/ego_vehicle/objects", 1, std::bind(&ItsInterface::objectsCallback, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<nam::Odometry>("/carla/ego_vehicle/odometry", 1, std::bind(&ItsInterface::odometryCallback, this, std::placeholders::_1));

  pub_objects_carla_map_ = this->create_publisher<pin::ObjectList>("/carla_its_interface/objectList/carla_map", 1);
  pub_objects_ego_vehicle_ = this->create_publisher<pin::ObjectList>("/carla_its_interface/objectList/ego_vehicle", 1);
  pub_objects_map_ = this->create_publisher<pin::ObjectList>("/global/objectList", 1);
  pub_objects_base_link_ = this->create_publisher<pin::ObjectList>("/carla_its_interface/objectList/base_link", 1);

  // Load publish parameters
  std::vector<std::string> v_parameter_str = {"publish.carla_map", "publish.ego_vehicle", "publish.map", "publish.base_link"};
  std::vector<bool> v_parameter_bool;
  for (auto parameter_str : v_parameter_str) {
    this->declare_parameter(parameter_str, rclcpp::ParameterType::PARAMETER_BOOL);
    try {
      v_parameter_bool.push_back(this->get_parameter(parameter_str).as_bool());
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
      std::string error_msg = "Parameter \'" + parameter_str + "\' is required";
      ROS_LOG_STREAM(ERROR, error_msg);
      return;
    }
  }

  publish_carla_map_ = v_parameter_bool[0];
  publish_ego_vehicle_ = v_parameter_bool[1];
  publish_map_ = v_parameter_bool[2];
  publish_base_link_ = v_parameter_bool[3];

  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
#endif

  ROS_LOG_STREAM(INFO, "CarlaItsInterface running...");  
}

void ItsInterface::objectsCallback(const dom::ObjectArray::ConstPtr &msg) {
  // Map the objects from the CARLA format to the perception_interfaces format
  msg_object_list_.objects.clear();
  msg_object_list_.header = msg->header;
  
  msg_object_list_.objects.resize(msg->objects.size());
  for (size_t i = 0; i < msg->objects.size(); i++) {
    msg_object_list_.objects[i].id = msg->objects[i].id;
    msg_object_list_.objects[i].existence_probability = 1.0; // Probability that the object exists is always 1.0 as source is CARLA
    obj_acc::initializeState(msg_object_list_.objects[i], pin::ISCACTR::MODEL_ID);
    msg_object_list_.objects[i].state.header = msg->objects[i].header;  // optional
    obj_acc::setPose(msg_object_list_.objects[i].state, msg->objects[i].pose);
    obj_acc::setZ(msg_object_list_.objects[i], msg->objects[i].pose.position.z + msg->objects[i].shape.dimensions[2] / 2.0); // Set z to the center of the object
    obj_acc::setVelocity(msg_object_list_.objects[i].state, msg->objects[i].twist.linear);
    obj_acc::setAcceleration(msg_object_list_.objects[i].state, msg->objects[i].accel.linear);
    obj_acc::setYawRate(msg_object_list_.objects[i], msg->objects[i].twist.angular.z);
    obj_acc::setLength(msg_object_list_.objects[i], msg->objects[i].shape.dimensions[0]);
    obj_acc::setWidth(msg_object_list_.objects[i], msg->objects[i].shape.dimensions[1]);
    obj_acc::setHeight(msg_object_list_.objects[i], msg->objects[i].shape.dimensions[2]);

    msg_object_list_.objects[i].state.classifications.resize(1);
    msg_object_list_.objects[i].state.classifications[0].probability = 1.0; // Probability that the object is of this type is always 1.0 as source is CARLA
    switch ((int) msg->objects[i].classification)
    {
    case dom::Object::CLASSIFICATION_PEDESTRIAN:
      msg_object_list_.objects[i].state.classifications[0].type = pin::ObjectClassification::PEDESTRIAN;
      break;
    case dom::Object::CLASSIFICATION_BIKE:
      msg_object_list_.objects[i].state.classifications[0].type = pin::ObjectClassification::BICYCLE;
      break;
    case dom::Object::CLASSIFICATION_MOTORCYCLE:
      msg_object_list_.objects[i].state.classifications[0].type = pin::ObjectClassification::MOTORBIKE;
      break;
    case dom::Object::CLASSIFICATION_CAR:
      msg_object_list_.objects[i].state.classifications[0].type = pin::ObjectClassification::CAR;
      break;
    case dom::Object::CLASSIFICATION_TRUCK:
      msg_object_list_.objects[i].state.classifications[0].type = pin::ObjectClassification::TRUCK;
      break;
    case dom::Object::CLASSIFICATION_BARRIER:
      msg_object_list_.objects[i].state.classifications[0].type = pin::ObjectClassification::ROAD_OBSTACLE;
      break;
    default:
      msg_object_list_.objects[i].state.classifications[0].type = pin::ObjectClassification::UNCLASSIFIED;
      break;
    }
  }

#ifdef MODE_ROS1
    auto timeout = ros::Duration(1.0);
#elif MODE_ROS2
    auto timeout = rclcpp::Duration::from_seconds(1.0);
#endif

  if(publish_carla_map_){
    // publish objectList in carla_map frame
#ifdef MODE_ROS1
    pub_objects_carla_map_.publish(msg_object_list_);
#elif MODE_ROS2
    pub_objects_carla_map_->publish(msg_object_list_);
#endif
  }
  if(publish_ego_vehicle_){
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

  if(publish_map_) {
    // Transform the objectList from carla_map to map frame
    pin::ObjectList msg_object_list_map;
    gm::TransformStamped carla_map_to_map_tf;
    try {
#ifdef MODE_ROS1
      carla_map_to_map_tf = tf2_buffer_.lookupTransform("map", "carla_map", msg_object_list_.header.stamp, timeout);
#elif MODE_ROS2
      carla_map_to_map_tf = tf2_buffer_->lookupTransform("map", "carla_map", msg_object_list_.header.stamp, timeout);
#endif
    } catch (tf2::TransformException& ex) {
      ROS_LOG_STREAM(ERROR, "\"Exception caught: \" << ex.what()");
      return;
    }
    tf2::doTransform(msg_object_list_, msg_object_list_map, carla_map_to_map_tf);

    // publish objectList in map frame
#ifdef MODE_ROS1
    pub_objects_map_.publish(msg_object_list_map);
#elif MODE_ROS2
    pub_objects_map_->publish(msg_object_list_map);
#endif
  }
  
  if(publish_base_link_) {
    // Transform the objectList from map to base_link frame
    pin::ObjectList msg_object_list_base_link;
    gm::TransformStamped carla_map_to_base_link_tf;
    try {
#ifdef MODE_ROS1
      carla_map_to_base_link_tf = tf2_buffer_.lookupTransform("base_link", "carla_map", msg_object_list_.header.stamp, timeout);  // ToDo: Direct tf from carla_map to base_link possible?
#elif MODE_ROS2
      carla_map_to_base_link_tf = tf2_buffer_->lookupTransform("base_link", "carla_map", msg_object_list_.header.stamp, timeout); // ToDo: Direct tf from carla_map to base_link possible?
#endif
    } catch (tf2::TransformException& ex) {
      ROS_LOG_STREAM(ERROR, "\"Exception caught: \" << ex.what()");
      return;
    }
    tf2::doTransform(msg_object_list_, msg_object_list_base_link, carla_map_to_base_link_tf);

    // Only consider objects that are within the fov_range
    pin::ObjectList msg_object_list_base_link_filtered;
    msg_object_list_base_link_filtered.header = msg_object_list_base_link.header;
    for (size_t i = 0; i < msg_object_list_base_link.objects.size(); i++) {
      double x = obj_acc::getX(msg_object_list_base_link.objects[i]);
      double y = obj_acc::getY(msg_object_list_base_link.objects[i]);
      if (sqrt(x*x + y*y) <= fov_range_) {
        msg_object_list_base_link_filtered.objects.push_back(msg_object_list_base_link.objects[i]);
      }
    }
    
    // publish objectList in base_link frame within fov_range
#ifdef MODE_ROS1
    pub_objects_base_link_.publish(msg_object_list_base_link_filtered);
#elif MODE_ROS2
    pub_objects_base_link_->publish(msg_object_list_base_link_filtered);
#endif
  }
}


void ItsInterface::odometryCallback(const nam::Odometry::ConstPtr &msg) 
{
  // Set up a transformation link between CARLA map and map
#ifdef MODE_ROS1
  auto timezero = ros::Time(0);
#elif MODE_ROS2
  auto timezero = tf2::TimePointZero;
#endif

  try
  {
    // check if transformation is already defined
    gm::TransformStamped transform;
#ifdef MODE_ROS1
    transform = tf2_buffer_.lookupTransform("map", "carla_map", timezero);
#elif MODE_ROS2
    transform = tf2_buffer_->lookupTransform("map", "carla_map", timezero);
#endif
  }
  catch(const tf2::TransformException& e)
  {
    gm::TransformStamped carla_ego_vehicle_tf;
    gm::TransformStamped base_link_map_tf;
    tf2::Transform ego_vehicle_base_link_tf;


    // CARLA map to ego_vehicle transform
    try
    {
#ifdef MODE_ROS1
      carla_ego_vehicle_tf = tf2_buffer_.lookupTransform("ego_vehicle", "carla_map", timezero);
#elif MODE_ROS2
      carla_ego_vehicle_tf = tf2_buffer_->lookupTransform("ego_vehicle", "carla_map", timezero);
#endif
    }
    catch(const tf2::TransformException& e)
    {
      return;
    }

    // ego_vehicle to base_link
    ego_vehicle_base_link_tf.setOrigin(tf2::Vector3(center_to_baselink_,0,0));
    ego_vehicle_base_link_tf.setRotation(tf2::Quaternion(0,0,0,1));

    // base_link to map
    try
    {
#ifdef MODE_ROS1
      base_link_map_tf = tf2_buffer_.lookupTransform("map", "base_link", timezero);
#elif MODE_ROS2
      base_link_map_tf = tf2_buffer_->lookupTransform("map", "base_link", timezero);
#endif
    }
    catch(const tf2::TransformException& e)
    {
      return;
    }

    // combine transformations
    // carla_map -> ego_vehicle -> base_link -> map
    tf2::Transform base_link_map;
    tf2::Transform carla_ego_map;
    tf2::convert(base_link_map_tf.transform, base_link_map);
    tf2::convert(carla_ego_vehicle_tf.transform, carla_ego_map);
    tf2::Transform br_tf = base_link_map * ego_vehicle_base_link_tf * carla_ego_map;
    br_tf = br_tf.inverse();
    // map -> base_link -> ego_vehicle -> carla_map

    // broadcast transformation between carla_map and map
    gm::TransformStamped static_transformStamped;

#ifdef MODE_ROS1
    static tf2_ros::StaticTransformBroadcaster static_br_tf_;
    static_transformStamped.header.stamp = ros::Time::now();
#elif MODE_ROS2
    static tf2_ros::StaticTransformBroadcaster static_br_tf_(this);
    static_transformStamped.header.stamp = this->get_clock()->now();
#endif
    static_transformStamped.header.frame_id = "carla_map";
    static_transformStamped.child_frame_id = "map";
    static_transformStamped.transform.translation.x = br_tf.getOrigin().x();
    static_transformStamped.transform.translation.y = br_tf.getOrigin().y();
    static_transformStamped.transform.translation.z = br_tf.getOrigin().z();
    static_transformStamped.transform.rotation.x = br_tf.getRotation().x();
    static_transformStamped.transform.rotation.y = br_tf.getRotation().y();
    static_transformStamped.transform.rotation.z = br_tf.getRotation().z();
    static_transformStamped.transform.rotation.w = br_tf.getRotation().w();

    static_br_tf_.sendTransform(static_transformStamped);

    ROS_LOG_STREAM(INFO, "Published static transform between carla_map and map");

  }

}


}  // end of namespace


int main(int argc, char **argv)
{
#ifdef MODE_ROS1
  ros::init(argc, argv, "CarlaItsInterface");
  carla::ItsInterface node;
  ros::spin();
#elif MODE_ROS2
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<carla::ItsInterface>());
  rclcpp::shutdown();
#endif
  return 0;
}

