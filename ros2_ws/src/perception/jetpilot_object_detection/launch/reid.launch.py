from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration
    component = ComposableNode(
        package="jetpilot_object_detection",
        plugin="jetpilot_object_detection::ReidNode",
        name="opponent_reid",
        parameters=[config("param_file"), {
            "engine_path": config("engine_path"),
            "source_width": ParameterValue(config("source_width"), value_type=int),
            "source_height": ParameterValue(config("source_height"), value_type=int),
            "use_sim_time": ParameterValue(config("use_sim_time"), value_type=bool),
        }],
        remappings=[
            ("image", config("image_topic")),
            ("detections", config("detections_topic")),
            ("matches", "/perception/opponents/reid/matches"),
            ("diagnostics", "/perception/opponents/reid/diagnostics"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("engine_path", description="Required target-device ReID TensorRT engine"),
        DeclareLaunchArgument("container_name", default_value="multi_sensor_container"),
        DeclareLaunchArgument("run_standalone", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("source_width", default_value="424"),
        DeclareLaunchArgument("source_height", default_value="240"),
        DeclareLaunchArgument("image_topic", default_value="/perception/object_detection/image"),
        DeclareLaunchArgument("detections_topic", default_value="/perception/detections"),
        DeclareLaunchArgument("param_file", default_value=PathJoinSubstitution([
            FindPackageShare("jetpilot_object_detection"), "config", "reid.param.yaml"])),
        ComposableNodeContainer(
            name=config("container_name"), namespace="", package="rclcpp_components",
            executable="component_container_mt", composable_node_descriptions=[],
            output="screen", condition=IfCondition(config("run_standalone"))),
        LoadComposableNodes(target_container=config("container_name"),
                            composable_node_descriptions=[component]),
    ])
