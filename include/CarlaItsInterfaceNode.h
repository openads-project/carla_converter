#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>


#include <derived_object_msgs/ObjectArray.h>
#include <perception_interfaces/ObjectList.h>
#include <perception_interfaces/ISCACTR.h>
#include <perception_interfaces/object_access.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_perception_msgs/tf2_perception_msgs.h>

namespace obj_acc = perception_interfaces::object_access;


namespace carla {

class ItsInterface {

  public:
    ItsInterface();

  private:
    void objectsCallback(const derived_object_msgs::ObjectArray::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry& msg);

    ros::NodeHandle private_node_handle_;
    
    ros::Subscriber sub_objects_;
    ros::Subscriber sub_odometry_;
    ros::Publisher pub_objects_;
    ros::Publisher pub_objects_base_link_;
    
    tf::TransformListener tf_listener_;

    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    tf2_ros::Buffer tfBuffer;

    perception_interfaces::ObjectList msg_object_list_;

    double fov_range_ = 75.0;
    double center_to_baselink_ = 1.2645;
};


}  // end of namespace carla
