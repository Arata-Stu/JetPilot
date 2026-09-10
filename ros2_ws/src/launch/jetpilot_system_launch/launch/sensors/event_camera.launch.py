# SPDX-License-Identifier: Apache-2.0
"""OpenEB event camera only, with a container for downstream E2E components."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("container_name", default_value="multi_sensor_container"),
        DeclareLaunchArgument("run_standalone", default_value="true"),
        DeclareLaunchArgument("silky_evcam_debug", default_value="false"),
        DeclareLaunchArgument("silky_evcam_event_image_fps", default_value="100.0"),
        DeclareLaunchArgument("silky_evcam_event_image_window_ms", default_value="50.0"),
        DeclareLaunchArgument("silky_evcam_event_image_stride_ms", default_value="10.0"),
        ComposableNodeContainer(
            name=LaunchConfiguration("container_name"),
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[],
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_standalone")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("jetpilot_system_launch"),
                "launch/sensors/realsense_silky_evcam.launch.py",
            ])),
            launch_arguments={
                "enable_realsense": "false",
                "enable_silky_evcam": "true",
                "run_standalone": "false",
                "silky_evcam_event_image_enabled": "true",
                "silky_evcam_event_image_encoding": "bgr8",
                "silky_evcam_event_image_style": "gep",
                "silky_evcam_event_image_percentile": "90.0",
                "silky_evcam_event_image_fps":
                    LaunchConfiguration("silky_evcam_event_image_fps"),
                "silky_evcam_event_image_window_ms":
                    LaunchConfiguration("silky_evcam_event_image_window_ms"),
                "silky_evcam_event_image_stride_ms":
                    LaunchConfiguration("silky_evcam_event_image_stride_ms"),
                "silky_evcam_event_image_publish_empty": "true",
                "silky_evcam_debug": LaunchConfiguration("silky_evcam_debug"),
            }.items(),
        ),
    ])
