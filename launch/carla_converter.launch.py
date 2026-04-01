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
        DeclareLaunchArgument("object_list_topic", default_value="~/object_list", description="output topic for converted object list"),
        DeclareLaunchArgument("traffic_lights_topic", default_value="~/traffic_lights", description="output topic for converted traffic lights"),
        DeclareLaunchArgument("map_info_topic", default_value="~/map_info", description="output topic for map info"),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="carla_converter", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("carla_converter"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="use simulation clock"),
        DeclareLaunchArgument("acceleration_filter_alpha", default_value="1.0", description="Low-pass filter alpha for IMU acceleration (0.0 = no update, 1.0 = no filtering"),
        *remappable_topics
    ]

    nodes = [
        Node(
            package="carla_converter",
            executable="carla_converter",
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
            os.path.join(get_package_share_directory("carla_converter"), "launch", "transforms.launch.py")
        )
    )

    return LaunchDescription([
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        transforms,
        *nodes,
    ])
