#include <CarlaItsInterfaceNode.h>


namespace carla {


ItsInterface::ItsInterface() {
  ROS_INFO("CarlaItsInterface starting...");

  sub_objects_ = private_node_handle_.subscribe("/carla/ego_vehicle/objects", 1, &ItsInterface::objectsCallback, this);
  sub_odometry_ = private_node_handle_.subscribe("/carla/ego_vehicle/odometry", 1, &ItsInterface::odometryCallback, this);
  pub_objects_ = private_node_handle_.advertise<perception_interfaces::ObjectList>("/global/objectList", 1);

  ros::spin();
}

void ItsInterface::objectsCallback(const derived_object_msgs::ObjectArray::ConstPtr &msg) {
  // Map the objects from the CARLA format to the perception_interfaces format
  msg_object_list_.objects.clear();
  msg_object_list_.header = msg->header;
  
  msg_object_list_.objects.resize(msg->objects.size());
  for (size_t i = 0; i < msg->objects.size(); i++) {
    msg_object_list_.objects[i].id = msg->objects[i].id;
    msg_object_list_.objects[i].existence_probability = 1.0; // Probability that the object exists is always 1.0 as source is CARLA
    perception_interfaces::object_access::initializeState(msg_object_list_.objects[i], perception_interfaces::ISCACTR::MODEL_ID);
    msg_object_list_.objects[i].state.header = msg->objects[i].header;  // optional
    perception_interfaces::object_access::setPose(msg_object_list_.objects[i].state, msg->objects[i].pose);
    perception_interfaces::object_access::setZ(msg_object_list_.objects[i], msg->objects[i].pose.position.z + msg->objects[i].shape.dimensions[2] / 2.0); // Set z to the center of the object
    perception_interfaces::object_access::setVelocity(msg_object_list_.objects[i].state, msg->objects[i].twist.linear);
    perception_interfaces::object_access::setAcceleration(msg_object_list_.objects[i].state, msg->objects[i].accel.linear);
    perception_interfaces::object_access::setYawRate(msg_object_list_.objects[i], msg->objects[i].twist.angular.z);
    perception_interfaces::object_access::setLength(msg_object_list_.objects[i], msg->objects[i].shape.dimensions[0]);
    perception_interfaces::object_access::setWidth(msg_object_list_.objects[i], msg->objects[i].shape.dimensions[1]);
    perception_interfaces::object_access::setHeight(msg_object_list_.objects[i], msg->objects[i].shape.dimensions[2]);

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

  pub_objects_.publish(msg_object_list_);
}


void ItsInterface::odometryCallback(const nav_msgs::Odometry& msg) 
{
  ROS_INFO("Odometry callback");
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

    // base_link to adp map
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
  }

}


}  // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CarlaItsInterface");

  carla::ItsInterface node;

  return 0;
}

