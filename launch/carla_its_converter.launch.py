import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time_lauch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True'
    )
    ego_data_actors_launch_arg = DeclareLaunchArgument(
        name='ego_data_actors',
        default_value='ego_vehicle'
    )
    object_data_actors_launch_arg = DeclareLaunchArgument(
        name='object_data_actors',
        default_value='ego_vehicle'
    )
    pos_variances_launch_arg = DeclareLaunchArgument(
        name='pos_variances',
        default_value='0.2'
    )
    vel_variances_launch_arg = DeclareLaunchArgument(
        name='vel_variances',
        default_value='-1.0'
    )
    acc_variances_launch_arg = DeclareLaunchArgument(
        name='acc_variances',
        default_value='-1.0'
    )
    angle_variances_launch_arg = DeclareLaunchArgument(
        name='angle_variances',
        default_value='0.1'
    )
    angle_rate_variances_launch_arg = DeclareLaunchArgument(
        name='angle_rate_variances',
        default_value='-1.0'
    )
    traffic_light_frequency_launch_arg = DeclareLaunchArgument(
        name='traffic_light_frequency',
        default_value='10.0'
    )

    carla_its_converter_node = LifecycleNode(
        package="carla_its_converter",
        executable="carla_its_converter_node",
        name="carla_its_converter",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "ego_data_actors": LaunchConfiguration('ego_data_actors'),
                "object_data_actors": LaunchConfiguration('object_data_actors'),
                "pos_variances": LaunchConfiguration('pos_variances'),
                "vel_variances": LaunchConfiguration('vel_variances'),
                "acc_variances": LaunchConfiguration('acc_variances'),
                "angle_variances": LaunchConfiguration('angle_variances'),
                "angle_rate_variances": LaunchConfiguration('angle_rate_variances'),
                "traffic_light_frequency": LaunchConfiguration('traffic_light_frequency'),
                "use_sim_time": LaunchConfiguration('use_sim_time')
            },
        ],
    )

    transforms = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_its_converter'), 'launch', 'transforms.launch.py')
            )
        )

    return launch.LaunchDescription([
        use_sim_time_lauch_arg,
        ego_data_actors_launch_arg,
        object_data_actors_launch_arg,
        pos_variances_launch_arg,
        vel_variances_launch_arg,
        acc_variances_launch_arg,
        angle_variances_launch_arg,
        angle_rate_variances_launch_arg,
        traffic_light_frequency_launch_arg,
        transforms,
        carla_its_converter_node
    ])


if __name__ == '__main__':
    generate_launch_description()
