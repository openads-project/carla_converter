#include <CarlaItsInterfaceNode.h>


namespace carla {


ItsInterface::ItsInterface() {
  ROS_INFO("CarlaItsInterface starting...");

  sub_objects_ = private_node_handle_.subscribe("/carla/ego_vehicle/objects", 1, &ItsInterface::objectsCallback, this);
  pub_objects_ = private_node_handle_.advertise<perception_interfaces::ObjectList>("objectList", 1);

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
    perception_interfaces::ObjectAccess::initializeState(msg_object_list_.objects[i], perception_interfaces::ISCACTR::MODEL_ID);
    msg_object_list_.objects[i].state.header = msg_object_list_.header; // optional
    perception_interfaces::ObjectAccess::setX(msg_object_list_.objects[i], msg->objects[i].pose.position.x);
    perception_interfaces::ObjectAccess::setY(msg_object_list_.objects[i], msg->objects[i].pose.position.y);
    perception_interfaces::ObjectAccess::setZ(msg_object_list_.objects[i], msg->objects[i].pose.position.z + msg->objects[i].shape.dimensions[2] / 2.0);
    perception_interfaces::ObjectAccess::setVelX(msg_object_list_.objects[i], msg->objects[i].twist.linear.x);
    perception_interfaces::ObjectAccess::setVelY(msg_object_list_.objects[i], msg->objects[i].twist.linear.y);
    perception_interfaces::ObjectAccess::setAccX(msg_object_list_.objects[i], msg->objects[i].accel.linear.x);
    perception_interfaces::ObjectAccess::setAccY(msg_object_list_.objects[i], msg->objects[i].accel.linear.y);

    tf2::Quaternion q;
    tf2::fromMsg(msg->objects[i].pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    perception_interfaces::ObjectAccess::setYaw(msg_object_list_.objects[i], yaw);

    perception_interfaces::ObjectAccess::setYawRate(msg_object_list_.objects[i], msg->objects[i].twist.angular.z);
    perception_interfaces::ObjectAccess::setLength(msg_object_list_.objects[i], msg->objects[i].shape.dimensions[0]);
    perception_interfaces::ObjectAccess::setWidth(msg_object_list_.objects[i], msg->objects[i].shape.dimensions[1]);
    perception_interfaces::ObjectAccess::setHeight(msg_object_list_.objects[i], msg->objects[i].shape.dimensions[2]);
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

