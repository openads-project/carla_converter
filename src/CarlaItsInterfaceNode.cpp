#include <CarlaItsInterfaceNode.h>


namespace carla {


ItsInterface::ItsInterface() {
  ROS_INFO("CarlaItsInterface starting...");

  sub_objects_ = private_node_handle_.subscribe("/carla/ego_vehicle/objects", 1, &ItsInterface::objectsCallback, this);
  pub_objects_ = private_node_handle_.advertise<perception_interfaces::ObjectList>("/global/objectList", 1);

  ros::spin();
}

void ItsInterface::objectsCallback(const derived_object_msgs::ObjectArray::ConstPtr &msg) {
  msg_object_list_.objects.clear();
  msg_object_list_.header = msg->header;
  msg_object_list_.header.frame_id = "map";
  
  msg_object_list_.objects.resize(msg->objects.size());
  for (size_t i = 0; i < msg->objects.size(); i++) {
    msg_object_list_.objects[i].id = msg->objects[i].id;
    msg_object_list_.objects[i].existence_probability = 1.0; // Probability that the object exists is always 1.0 as source is CARLA
    perception_interfaces::object_access::initializeState(msg_object_list_.objects[i], perception_interfaces::ISCACTR::MODEL_ID);
    msg_object_list_.objects[i].state.header = msg_object_list_.header; // optional
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

}  // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CarlaItsInterface");

  carla::ItsInterface node;

  return 0;
}

