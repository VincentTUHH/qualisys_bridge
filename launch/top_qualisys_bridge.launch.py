import launch
import launch_ros
from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    uvms_kin_ctrl_path = get_package_share_path('uvms_kinematic_ctrl')
    qualisys_bridge_path = get_package_share_path('qualisys_bridge')

    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')

    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='klopsi00',
        description='used for node namespace',
    )

    use_px4 = launch.substitutions.LaunchConfiguration('use_px4')

    use_px4_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_px4',
        default_value=str(True),
        description='use EKF2 from px4 for odometry',
    )

    qualisys_bridge = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(qualisys_bridge_path / 'launch/qualisys_bridge.launch.py')
        ),
        launch_arguments={
            'use_px4': use_px4,
        }.items(),
    )

    px4_odometry = launch.actions.GroupAction(
        condition=launch.conditions.IfCondition(use_px4),
        actions=[
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(uvms_kin_ctrl_path / 'launch/odometry_map.launch.py')
                ),
                launch_arguments={
                    'vehicle_name': vehicle_name,
                }.items(),
            )
        ],
    )

    return launch.LaunchDescription(
        [
            vehicle_name_launch_arg,
            use_px4_launch_arg,
            qualisys_bridge,
            px4_odometry,
        ]
    )