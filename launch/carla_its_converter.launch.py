import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode

def generate_launch_description():

    publish_ego_vehicle_launch_arg = DeclareLaunchArgument(
        name='publish_ego_vehicle',
        default_value='True'
    )
    config = PathJoinSubstitution([
        LaunchConfiguration('publish_ego_vehicle')
    ])

    carla_its_converter_node = LifecycleNode(
        package="carla_its_converter",
        executable="carla_its_converter_node",
        name="carla_its_converter",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[config],
    )

    return launch.LaunchDescription([
        publish_ego_vehicle_launch_arg,
        carla_its_converter_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
