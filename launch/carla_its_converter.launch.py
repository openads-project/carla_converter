#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    remappable_topics = [
        DeclareLaunchArgument("objects_topic", default_value="/carla/objects", description="input topic for CARLA object array"),
        DeclareLaunchArgument("traffic_light_info_topic", default_value="/carla/traffic_lights/info", description="input topic for CARLA traffic light info"),
        DeclareLaunchArgument("traffic_light_status_topic", default_value="/carla/traffic_lights/status", description="input topic for CARLA traffic light status"),
        DeclareLaunchArgument("world_info_topic", default_value="/carla/world_info", description="input topic for CARLA world info"),
        DeclareLaunchArgument("object_list_topic", default_value="/carla_its_converter/object_list", description="output topic for converted object list"),
        DeclareLaunchArgument("traffic_lights_topic", default_value="/carla_its_converter/traffic_lights", description="output topic for converted traffic lights"),
        DeclareLaunchArgument("map_info_topic", default_value="/carla_its_converter/map_info", description="output topic for map info"),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="carla_its_converter", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("carla_its_converter"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="use simulation clock"),
    ]

    nodes = [
        Node(
            package="carla_its_converter",
            executable="carla_its_converter",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        )
    ]

    transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("carla_its_converter"), "launch", "transforms.launch.py")
        )
    )

    return LaunchDescription([
        *remappable_topics,
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        transforms,
        *nodes,
    ])
