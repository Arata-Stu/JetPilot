"""Exclusive tuning planner/controller; localization and operation run separately."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controller = str(Path(get_package_share_directory('jetpilot_controller')) / 'config/controller.param.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_dir', description='Map used by the running localization system'),
        DeclareLaunchArgument('controller_config', default_value=controller),
        ExecuteProcess(cmd=['python3', str(Path(__file__).with_name('tuning_bridge.py')),
                            '--map-dir', LaunchConfiguration('map_dir')], output='screen'),
        Node(package='jetpilot_controller', executable='path_tracking_controller_node',
             name='path_tracking_controller_node', output='screen',
             parameters=[LaunchConfiguration('controller_config'), {
                 'trajectory_topic': '/tuning/trajectory',
                 'trajectory_profile_topic': '/tuning/trajectory_profile',
                 'target_speed_topic': '/tuning/target_speed',
                 'planning_ready_topic': '/tuning/ready',
             }]),
    ])
