from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def declare_launch_args(launch_description: LaunchDescription):
    default = get_package_share_path('qualisys_bridge') / 'config/bridge.yaml'
    action = DeclareLaunchArgument(
        name='bridge_config_file', default_value=str(default)
    )
    launch_description.add_action(action)

    default_px4 = str(False)
    action_px4 = DeclareLaunchArgument(
        name='use_px4',
        default_value=default_px4,
        description='use EKF2 from px4 for odometry',
    )
    launch_description.add_action(action_px4)

def add_node(launch_description: LaunchDescription):
    config_file = LaunchConfiguration('bridge_config_file')
    use_px4 = LaunchConfiguration('use_px4')

    node_without_px4 = Node(
        package='qualisys_bridge',
        executable='bridge_node',
        name='qualisys',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        remappings=[('klopsi00/ground_truth/odometry', 'klopsi00/odometry')],
        condition=UnlessCondition(use_px4)
    )

    node_with_px4 = Node(
        package='qualisys_bridge',
        executable='bridge_node',
        name='qualisys',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_px4)
    )

    launch_description.add_action(node_without_px4)
    launch_description.add_action(node_with_px4)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_node(launch_description=launch_description)
    return launch_description
