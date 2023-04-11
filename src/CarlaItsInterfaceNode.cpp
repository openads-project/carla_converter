#include <CarlaItsInterfaceNode.h>


namespace carla {


ItsInterface::ItsInterface() {
  
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer);

#ifdef ROS1
  ROS_INFO("CarlaItsInterface starting...");
  sub_objects_ = private_node_handle_.subscribe("/carla/ego_vehicle/objects", 1, &ItsInterface::objectsCallback, this);
  sub_odometry_ = private_node_handle_.subscribe("/carla/ego_vehicle/odometry", 1, &ItsInterface::odometryCallback, this);
  pub_objects_ = private_node_handle_.advertise<perception_interfaces::ObjectList>("/global/objectList", 1);
  pub_objects_base_link_ = private_node_handle_.advertise<perception_interfaces::ObjectList>("objectList/base_link", 1);
  ros::spin();
#endif

  
}

#ifdef ROS1
void ItsInterface::objectsCallback(const derived_object_msgs::ObjectArray::ConstPtr &msg) {
#endif
#ifdef ROS2
void ItsInterface::objectsCallback(const derived_object_msgs::msg::ObjectArray::ConstPtr& msg) {
#endif
  // Map the objects from the CARLA format to the perception_interfaces format
  msg_object_list_.objects.clear();
  msg_object_list_.header = msg->header;
  
  msg_object_list_.objects.resize(msg->objects.size());
  for (size_t i = 0; i < msg->objects.size(); i++) {
    msg_object_list_.objects[i].id = msg->objects[i].id;
    msg_object_list_.objects[i].existence_probability = 1.0; // Probability that the object exists is always 1.0 as source is CARLA
    obj_acc::initializeState(msg_object_list_.objects[i], perception_interfaces::ISCACTR::MODEL_ID);
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
    case derived_object_msgs::Object::CLASSIFICATION_PEDESTRIAN:
      msg_object_list_.objects[i].state.classifications[0].type = perception_interfaces::ObjectClassification::PEDESTRIAN;
      break;
    case derived_object_msgs::Object::CLASSIFICATION_BIKE:
      msg_object_list_.objects[i].state.classifications[0].type = perception_interfaces::ObjectClassification::BICYCLE;
      break;
    case derived_object_msgs::Object::CLASSIFICATION_MOTORCYCLE:
      msg_object_list_.objects[i].state.classifications[0].type = perception_interfaces::ObjectClassification::MOTORBIKE;
      break;
    case derived_object_msgs::Object::CLASSIFICATION_CAR:
      msg_object_list_.objects[i].state.classifications[0].type = perception_interfaces::ObjectClassification::CAR;
      break;
    case derived_object_msgs::Object::CLASSIFICATION_TRUCK:
      msg_object_list_.objects[i].state.classifications[0].type = perception_interfaces::ObjectClassification::TRUCK;
      break;
    case derived_object_msgs::Object::CLASSIFICATION_BARRIER:
      msg_object_list_.objects[i].state.classifications[0].type = perception_interfaces::ObjectClassification::ROAD_OBSTACLE;
      break;
    default:
      msg_object_list_.objects[i].state.classifications[0].type = perception_interfaces::ObjectClassification::UNCLASSIFIED;
      break;
    }
  }

  // Transform the objectList from carla_map to map frame
  geometry_msgs::TransformStamped carla_map_to_map_tf;
  try {
    carla_map_to_map_tf = tfBuffer.lookupTransform("map", "carla_map", msg_object_list_.header.stamp, ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }

  perception_interfaces::ObjectList msg_object_list_map;  
  tf2::doTransform(msg_object_list_, msg_object_list_map, carla_map_to_map_tf);

  // publish objectList in map frame
  pub_objects_.publish(msg_object_list_map);


  // Transform the objectList from map to base_link frame
  geometry_msgs::TransformStamped map_to_base_link_tf;
  try {
    map_to_base_link_tf = tfBuffer.lookupTransform("base_link", "map", msg_object_list_.header.stamp, ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }

  perception_interfaces::ObjectList msg_object_list_base_link;
  tf2::doTransform(msg_object_list_map, msg_object_list_base_link, map_to_base_link_tf);

  // Only consider objects that are within the fov_range
  perception_interfaces::ObjectList msg_object_list_base_link_filtered;
  for (size_t i = 0; i < msg_object_list_base_link.objects.size(); i++) {
    double x = obj_acc::getX(msg_object_list_base_link.objects[i]);
    double y = obj_acc::getY(msg_object_list_base_link.objects[i]);
    if (sqrt(x*x + y*y) <= fov_range_) {
      msg_object_list_base_link_filtered.objects.push_back(msg_object_list_base_link.objects[i]);
    }
  }
  
  // publish objectList in base_link frame within fov_range
  pub_objects_base_link_.publish(msg_object_list_base_link_filtered);
}


void ItsInterface::odometryCallback(const nav_msgs::Odometry& msg) 
{
  // Set up a transformation link between CARLA map and map
  try
  {
    // check if transformation is already defined
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("map", "carla_map", ros::Time(0), transform);
  }
  catch(const std::exception& e)
  {
    tf::StampedTransform carla_ego_vehicle_tf;
    tf::StampedTransform base_link_map_tf;
    tf::Transform ego_vehicle_base_link_tf;

    // CARLA map to ego_vehicle transform
    try
    {
      tf_listener_.lookupTransform("ego_vehicle", "carla_map", ros::Time(0), carla_ego_vehicle_tf);
    }
    catch(const std::exception& e)
    {
      return;
    }

    // ego_vehicle to base_link
    ego_vehicle_base_link_tf.setOrigin(tf::Vector3(center_to_baselink_,0,0));
    ego_vehicle_base_link_tf.setRotation(tf::Quaternion(0,0,0,1));

    // base_link to map
    try
    {
      tf_listener_.lookupTransform("map", "base_link", ros::Time(0), base_link_map_tf);
    }
    catch(const std::exception& e)
    {
      return;
    }
              
    // combine transformations
    // carla_map -> ego_vehicle -> base_link -> map
    tf::Transform br_tf = base_link_map_tf * ego_vehicle_base_link_tf * carla_ego_vehicle_tf;
    br_tf = br_tf.inverse();
    // map -> base_link -> ego_vehicle -> carla_map

    // broadcast transformation between carla_map and map

    static tf2_ros::StaticTransformBroadcaster static_br_tf_;  
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
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

    ROS_INFO("Published static transform between carla_map and map");
  }

}


}  // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CarlaItsInterface");

  carla::ItsInterface node;

  return 0;
}

