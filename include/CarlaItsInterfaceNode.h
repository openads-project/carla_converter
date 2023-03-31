#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>


#include <derived_object_msgs/ObjectArray.h>
#include <perception_interfaces/ObjectList.h>
#include <perception_interfaces/ISCACTR.h>
#include <perception_interfaces/object_access.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace carla {

class ItsInterface {

  public:
    ItsInterface();

  private:
    void objectsCallback(const derived_object_msgs::ObjectArray::ConstPtr& msg);

    ros::NodeHandle private_node_handle_;

    ros::Subscriber sub_objects_;
    ros::Publisher pub_objects_;
    
    perception_interfaces::ObjectList msg_object_list_;
};


}  // end of namespace carla
