import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():

    role_names_launch_arg = DeclareLaunchArgument(
        name='role_names',
        default_value='ego_vehicle'
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
        }
        ],
    )

    return launch.LaunchDescription([
        role_names_launch_arg,
        carla_its_converter_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
