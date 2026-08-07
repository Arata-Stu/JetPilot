from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("jetpilot_e2e_inference")
    model_root = LaunchConfiguration("model_root")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "image_topic", default_value="/realsense/color/image_raw"
            ),
            DeclareLaunchArgument(
                "control_cmd_topic", default_value="/auto/control_cmd"
            ),
            DeclareLaunchArgument(
                "model_root", default_value="/workspaces/ros2_ws/models/e2e/latest"
            ),
            DeclareLaunchArgument(
                "model_file_path",
                default_value=PathJoinSubstitution([model_root, "model.pt"]),
            ),
            DeclareLaunchArgument("model_format", default_value="auto"),
            DeclareLaunchArgument("model_kind", default_value="pilotnet"),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("use_half", default_value="false"),
            DeclareLaunchArgument("use_checkpoint_config", default_value="true"),
            DeclareLaunchArgument("input_width", default_value="212"),
            DeclareLaunchArgument("input_height", default_value="120"),
            DeclareLaunchArgument("max_inference_rate_hz", default_value="0.0"),
            DeclareLaunchArgument(
                "param_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "e2e_pytorch.param.yaml"]
                ),
            ),
            Node(
                package="jetpilot_e2e_inference",
                executable="e2e_pytorch_inference_node.py",
                name="e2e_pytorch_inference",
                output="screen",
                parameters=[
                    LaunchConfiguration("param_file"),
                    {
                        "model_file_path": LaunchConfiguration("model_file_path"),
                        "model_format": LaunchConfiguration("model_format"),
                        "model_kind": LaunchConfiguration("model_kind"),
                        "device": LaunchConfiguration("device"),
                        "use_half": ParameterValue(
                            LaunchConfiguration("use_half"), value_type=bool
                        ),
                        "use_checkpoint_config": ParameterValue(
                            LaunchConfiguration("use_checkpoint_config"), value_type=bool
                        ),
                        "input_width": ParameterValue(
                            LaunchConfiguration("input_width"), value_type=int
                        ),
                        "input_height": ParameterValue(
                            LaunchConfiguration("input_height"), value_type=int
                        ),
                        "max_inference_rate_hz": ParameterValue(
                            LaunchConfiguration("max_inference_rate_hz"), value_type=float
                        ),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    },
                ],
                remappings=[
                    ("image", LaunchConfiguration("image_topic")),
                    ("control_cmd", LaunchConfiguration("control_cmd_topic")),
                ],
            ),
        ]
    )
