#include <CarlaItsInterfaceNode.h>


namespace carla {


ItsInterface::ItsInterface() {
  ROS_INFO("CarlaItsInterface starting...");

  sub_objects_ = private_node_handle_.subscribe("/carla/ego_vehicle/objects", 1, &ItsInterface::objectsCallback, this);

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
    msg_object_list_.objects[i].state.header = msg_object_list_.header; // optional
    msg_object_list_.objects[i].state.model_id = perception_interfaces::ISCACTR::MODEL_ID;
    msg_object_list_.objects[i].state.state.resize((int)perception_interfaces::ISCACTR::STATE_SIZE);
    msg_object_list_.objects[i].state.state[(int)perception_interfaces::ISCACTR::X] = msg->objects[i].pose.position.x;
    msg_object_list_.objects[i].state.state[(int)perception_interfaces::ISCACTR::Y] = msg->objects[i].pose.position.y;
    msg_object_list_.objects[i].state.state[(int)perception_interfaces::ISCACTR::Z] = msg->objects[i].pose.position.z + msg->objects[i].shape.dimensions[2] / 2.0;
    msg_object_list_.objects[i].state.state[(int)perception_interfaces::ISCACTR::VEL_X] = msg->objects[i].twist.linear.x;
    msg_object_list_.objects[i].state.state[(int)perception_interfaces::ISCACTR::VEL_Y] = msg->objects[i].twist.linear.y;
    msg_object_list_.objects[i].state.state[(int)perception_interfaces::ISCACTR::ACC_X] = msg->objects[i].accel.linear.x;
    msg_object_list_.objects[i].state.state[(int)perception_interfaces::ISCACTR::ACC_Y] = msg->objects[i].accel.linear.y;

    
    //msg_object_list_.objects[i].ex
  }

}

}  // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CarlaItsInterface");

  carla::ItsInterface node;

  return 0;
}

