import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():

    role_names_launch_arg = DeclareLaunchArgument(
        name='role_names',
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
    yaw_variances_launch_arg = DeclareLaunchArgument(
        name='yaw_variances',
        default_value='0.1'
    )
    yaw_rate_variances_launch_arg = DeclareLaunchArgument(
        name='yaw_rate_variances',
        default_value='-1.0'
    )

    carla_its_converter_node = LifecycleNode(
        package="carla_its_converter",
        executable="carla_its_converter_node",
        name="carla_its_converter",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "role_names": LaunchConfiguration('role_names'),
            "pos_variances": LaunchConfiguration('pos_variances'),
            "vel_variances": LaunchConfiguration('vel_variances'),
            "acc_variances": LaunchConfiguration('acc_variances'),
            "yaw_variances": LaunchConfiguration('yaw_variances'),
            "yaw_rate_variances": LaunchConfiguration('yaw_rate_variances'),
        }
        ],
    )

    return launch.LaunchDescription([
        role_names_launch_arg,
        pos_variances_launch_arg,
        vel_variances_launch_arg,
        acc_variances_launch_arg,
        yaw_variances_launch_arg,
        yaw_rate_variances_launch_arg,
        carla_its_converter_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
