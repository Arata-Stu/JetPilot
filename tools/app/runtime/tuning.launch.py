"""Exclusive tuning planner/controller; localization and operation run separately."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controller = str(Path(get_package_share_directory('jetpilot_controller')) / 'config/controller.param.yaml')
    bridge = ExecuteProcess(cmd=['python3', str(Path(__file__).with_name('tuning_bridge.py')),
                                '--map-dir', LaunchConfiguration('map_dir')], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('map_dir', description='Map used by the running localization system'),
        DeclareLaunchArgument('controller_config', default_value=controller),
        DeclareLaunchArgument('throttle_calibration_file', default_value=str(
            Path(get_package_share_directory('jetpilot_controller')) / 'config/throttle_calibration.empty.param.yaml')),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Tuning bridge exited; stopping tuning nodes.'))])),
        bridge,
        Node(package='jetpilot_hdmap_publisher', executable='drivable_guard_node.py',
             name='drivable_guard', output='screen',
             parameters=[str(Path(get_package_share_directory('jetpilot_hdmap_publisher')) /
                             'config/drivable_guard.param.yaml')],
             remappings=[('/planning/trajectory', '/tuning/trajectory'),
                         ('/planning/trajectory_profile', '/tuning/trajectory_profile'),
                         ('/planning/target_speed', '/tuning/target_speed'),
                         ('/hd_map/drivable_area', '/tuning/drivable_area'),
                         ('/planning/safety_status', '/tuning/safety_status')]),
        Node(package='jetpilot_controller', executable='path_tracking_controller_node',
             name='path_tracking_controller_node', output='screen',
             remappings=[('/planning/safety_status', '/tuning/safety_status')],
             parameters=[LaunchConfiguration('controller_config'), LaunchConfiguration('throttle_calibration_file'), {
                 'trajectory_topic': '/tuning/trajectory',
                 'trajectory_profile_topic': '/tuning/trajectory_profile',
                 'target_speed_topic': '/tuning/target_speed',
                 'planning_ready_topic': '/tuning/ready',
             }]),
    ])
