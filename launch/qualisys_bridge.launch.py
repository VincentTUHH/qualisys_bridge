from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_launch_args(launch_description: LaunchDescription):
    action = DeclareLaunchArgument(name='vehicle_name')
    launch_description.add_action(action)

    default = get_package_share_path('qualisys_bridge') / 'config/bridge.yaml'
    action = DeclareLaunchArgument(name='bridge_config_file',
                                   default_value=str(default))
    launch_description.add_action(action)


def add_node(launch_description: LaunchDescription):
    action = Node(package='qualisys_bridge',
                  executable='bridge_node',
                  name='qualisys',
                  namespace=LaunchConfiguration('vehicle_name'))
    launch_description.add_action(action)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_node(launch_description=launch_description)
    return launch_description
