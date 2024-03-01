import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():

    use_sim_time_lauch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True'
    )
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
        parameters=[
            {
                "role_names": LaunchConfiguration('role_names'),
                "pos_variances": LaunchConfiguration('pos_variances'),
                "vel_variances": LaunchConfiguration('vel_variances'),
                "acc_variances": LaunchConfiguration('acc_variances'),
                "yaw_variances": LaunchConfiguration('yaw_variances'),
                "yaw_rate_variances": LaunchConfiguration('yaw_rate_variances')
            },
            {
                "use_sim_time": LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    transform_utm_31N = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['-166021.443', '0', '0', '0', '0', '0', 'world_origin', 'utm_31N']
    )
    transform_utm_31S = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['-166021.443', '-10000000.000', '0', '0', '0', '0', 'world_origin', 'utm_31S']
    )
    transform_utm_30N = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['-833978.557', '0', '0', '0', '0', '0', 'world_origin', 'utm_30N']
    )
    transform_utm_30S = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['-833978.557', '-10000000.000', '0', '0', '0', '0',  'world_origin', 'utm_30S']
    )

    return launch.LaunchDescription([
        use_sim_time_lauch_arg,
        role_names_launch_arg,
        pos_variances_launch_arg,
        vel_variances_launch_arg,
        acc_variances_launch_arg,
        yaw_variances_launch_arg,
        yaw_rate_variances_launch_arg,
        carla_its_converter_node,
        transform_utm_31N,
        transform_utm_31S,
        transform_utm_30N,
        transform_utm_30S
    ])


if __name__ == '__main__':
    generate_launch_description()
