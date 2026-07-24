from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("jetpilot_e2e_inference")
    trt_share = FindPackageShare("isaac_ros_tensor_rt")

    model_root = LaunchConfiguration("model_root")
    model_file_path = LaunchConfiguration("model_file_path")
    engine_file_path = LaunchConfiguration("engine_file_path")
    param_file = LaunchConfiguration("param_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/realsense/color/image_raw"),
            DeclareLaunchArgument("control_cmd_topic", default_value="/auto/control_cmd"),
            DeclareLaunchArgument("tensor_input_topic", default_value="/e2e/tensor_input"),
            DeclareLaunchArgument("tensor_output_topic", default_value="/e2e/tensor_output"),
            DeclareLaunchArgument("model_root", default_value="/opt/jetpilot/models/e2e/latest"),
            DeclareLaunchArgument(
                "model_file_path",
                default_value=PathJoinSubstitution([model_root, "model.onnx"]),
            ),
            DeclareLaunchArgument(
                "engine_file_path",
                default_value=PathJoinSubstitution([model_root, "model.plan"]),
            ),
            DeclareLaunchArgument(
                "param_file",
                default_value=PathJoinSubstitution([pkg_share, "config", "e2e_inference.param.yaml"]),
            ),
            DeclareLaunchArgument("force_engine_update", default_value="false"),
            DeclareLaunchArgument("enable_fp16", default_value="true"),
            DeclareLaunchArgument("input_tensor_format", default_value="nitros_tensor_list_nchw_rgb_f32"),
            DeclareLaunchArgument("output_tensor_format", default_value="nitros_tensor_list_nchw_rgb_f32"),
            Node(
                package="jetpilot_e2e_inference",
                executable="e2e_image_encoder_node.py",
                name="e2e_image_encoder",
                output="screen",
                parameters=[param_file],
                remappings=[
                    ("image", LaunchConfiguration("image_topic")),
                    ("tensor_pub", LaunchConfiguration("tensor_input_topic")),
                ],
            ),
            GroupAction(
                [
                    SetRemap(src="tensor_pub", dst=LaunchConfiguration("tensor_input_topic")),
                    SetRemap(src="tensor_sub", dst=LaunchConfiguration("tensor_output_topic")),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [trt_share, "launch", "isaac_ros_tensor_rt.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "model_file_path": model_file_path,
                            "engine_file_path": engine_file_path,
                            "force_engine_update": LaunchConfiguration("force_engine_update"),
                            "input_tensor_names": "['input_tensor']",
                            "input_binding_names": "['image']",
                            "input_tensor_formats": ["['", LaunchConfiguration("input_tensor_format"), "']"],
                            "output_tensor_names": "['output_tensor']",
                            "output_binding_names": "['control']",
                            "output_tensor_formats": ["['", LaunchConfiguration("output_tensor_format"), "']"],
                            "enable_fp16": LaunchConfiguration("enable_fp16"),
                        }.items(),
                    ),
                ]
            ),
            Node(
                package="jetpilot_e2e_inference",
                executable="e2e_control_decoder_node.py",
                name="e2e_control_decoder",
                output="screen",
                parameters=[param_file],
                remappings=[
                    ("tensor_sub", LaunchConfiguration("tensor_output_topic")),
                    ("control_cmd", LaunchConfiguration("control_cmd_topic")),
                ],
            ),
        ]
    )
