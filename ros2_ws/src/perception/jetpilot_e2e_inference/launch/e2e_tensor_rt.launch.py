from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("jetpilot_e2e_inference")

    model_root = LaunchConfiguration("model_root")
    model_file_path = LaunchConfiguration("model_file_path")
    engine_file_path = LaunchConfiguration("engine_file_path")
    param_file = LaunchConfiguration("param_file")
    container_name = LaunchConfiguration("container_name")

    image_encoder = ComposableNode(
        package="isaac_ros_dnn_image_encoder",
        plugin="nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode",
        name="e2e_image_encoder",
        namespace="",
        parameters=[
            {
                "input_image_width": ParameterValue(
                    LaunchConfiguration("input_image_width"), value_type=int
                ),
                "input_image_height": ParameterValue(
                    LaunchConfiguration("input_image_height"), value_type=int
                ),
                "network_image_width": ParameterValue(
                    LaunchConfiguration("network_image_width"), value_type=int
                ),
                "network_image_height": ParameterValue(
                    LaunchConfiguration("network_image_height"), value_type=int
                ),
                "input_encoding": LaunchConfiguration("input_encoding"),
                "enable_padding": ParameterValue(
                    LaunchConfiguration("enable_padding"), value_type=bool
                ),
                "image_mean": LaunchConfiguration("image_mean"),
                "image_stddev": LaunchConfiguration("image_stddev"),
                "tensor_name": LaunchConfiguration("input_tensor_name"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
        remappings=[
            ("image", LaunchConfiguration("image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
            ("tensors", LaunchConfiguration("tensor_input_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    tensor_rt = ComposableNode(
        package="isaac_ros_tensor_rt",
        plugin="nvidia::isaac_ros::dnn_inference::TensorRTNode",
        name="e2e_tensor_rt",
        namespace="",
        parameters=[
            {
                "model_file_path": model_file_path,
                "engine_file_path": engine_file_path,
                "force_engine_update": ParameterValue(
                    LaunchConfiguration("force_engine_update"), value_type=bool
                ),
                "enable_fp16": ParameterValue(
                    LaunchConfiguration("enable_fp16"), value_type=bool
                ),
                "input_tensor_names": LaunchConfiguration("input_tensor_names"),
                "input_binding_names": LaunchConfiguration("input_binding_names"),
                "input_tensor_formats": LaunchConfiguration("input_tensor_formats"),
                "output_tensor_names": LaunchConfiguration("output_tensor_names"),
                "output_binding_names": LaunchConfiguration("output_binding_names"),
                "output_tensor_formats": LaunchConfiguration("output_tensor_formats"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
        remappings=[
            ("tensor_pub", LaunchConfiguration("tensor_input_topic")),
            ("tensor_sub", LaunchConfiguration("tensor_output_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    control_decoder = ComposableNode(
        package="jetpilot_e2e_inference",
        plugin="jetpilot_e2e_inference::E2EControlDecoderNode",
        name="e2e_control_decoder",
        namespace="",
        parameters=[
            param_file,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "nitros_tensor_format": LaunchConfiguration("decoder_tensor_format"),
            },
        ],
        remappings=[
            ("tensor_sub", LaunchConfiguration("tensor_output_topic")),
            ("control_cmd", LaunchConfiguration("control_cmd_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    trajectory_decoder = ComposableNode(
        package="jetpilot_e2e_inference",
        plugin="jetpilot_e2e_inference::E2ETrajectoryDecoderNode",
        name="e2e_trajectory_decoder",
        namespace="",
        parameters=[
            LaunchConfiguration("trajectory_param_file"),
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "nitros_tensor_format": LaunchConfiguration("decoder_tensor_format"),
                "trajectory_points": ParameterValue(
                    LaunchConfiguration("trajectory_points"), value_type=int
                ),
                "trajectory_scale_m": ParameterValue(
                    LaunchConfiguration("trajectory_scale_m"), value_type=float
                ),
                "target_speed_mps": ParameterValue(
                    LaunchConfiguration("trajectory_target_speed_mps"), value_type=float
                ),
            },
        ],
        remappings=[
            ("tensor_sub", LaunchConfiguration("tensor_output_topic")),
            ("trajectory", LaunchConfiguration("trajectory_topic")),
            ("target_speed", LaunchConfiguration("target_speed_topic")),
            ("planning_ready", LaunchConfiguration("planning_ready_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    inference_components = [image_encoder, tensor_rt]

    return LaunchDescription(
        [
            DeclareLaunchArgument("container_name", default_value="multi_sensor_container"),
            DeclareLaunchArgument("run_standalone", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("image_topic", default_value="/realsense/color/image_raw"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/realsense/color/camera_info"
            ),
            DeclareLaunchArgument("control_cmd_topic", default_value="/auto/control_cmd"),
            DeclareLaunchArgument("trajectory_topic", default_value="/planning/trajectory"),
            DeclareLaunchArgument("target_speed_topic", default_value="/planning/target_speed"),
            DeclareLaunchArgument("planning_ready_topic", default_value="/planning/ready"),
            DeclareLaunchArgument("output_task", default_value="control"),
            DeclareLaunchArgument("trajectory_points", default_value="10"),
            DeclareLaunchArgument("trajectory_scale_m", default_value="5.0"),
            DeclareLaunchArgument("trajectory_target_speed_mps", default_value="0.8"),
            DeclareLaunchArgument("tensor_input_topic", default_value="/e2e/tensor_input"),
            DeclareLaunchArgument("tensor_output_topic", default_value="/e2e/tensor_output"),
            DeclareLaunchArgument("input_image_width", default_value="424"),
            DeclareLaunchArgument("input_image_height", default_value="240"),
            DeclareLaunchArgument("network_image_width", default_value="212"),
            DeclareLaunchArgument("network_image_height", default_value="120"),
            DeclareLaunchArgument("input_encoding", default_value="rgb8"),
            DeclareLaunchArgument("enable_padding", default_value="true"),
            DeclareLaunchArgument(
                "image_mean", default_value="[0.485, 0.456, 0.406]"
            ),
            DeclareLaunchArgument(
                "image_stddev", default_value="[0.229, 0.224, 0.225]"
            ),
            DeclareLaunchArgument("input_tensor_name", default_value="input_tensor"),
            DeclareLaunchArgument(
                "model_root", default_value="/workspaces/ros2_ws/models/e2e/latest"
            ),
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
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "e2e_inference.param.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "trajectory_param_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "e2e_trajectory.param.yaml"]
                ),
            ),
            DeclareLaunchArgument("force_engine_update", default_value="false"),
            DeclareLaunchArgument("enable_fp16", default_value="true"),
            DeclareLaunchArgument(
                "input_tensor_names", default_value="['input_tensor']"
            ),
            DeclareLaunchArgument("input_binding_names", default_value="['image']"),
            DeclareLaunchArgument(
                "input_tensor_formats",
                default_value="['nitros_tensor_list_nchw_rgb_f32']",
            ),
            DeclareLaunchArgument(
                "output_tensor_names", default_value="['output_tensor']"
            ),
            DeclareLaunchArgument(
                "output_binding_names",
                default_value=PythonExpression(
                    [
                        "\"['trajectory']\" if '",
                        LaunchConfiguration("output_task"),
                        "' == 'trajectory' else \"['control']\"",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "output_tensor_formats",
                default_value="['nitros_tensor_list_nchw_rgb_f32']",
            ),
            DeclareLaunchArgument(
                "decoder_tensor_format",
                default_value="nitros_tensor_list_nchw_rgb_f32",
            ),
            ComposableNodeContainer(
                name=container_name,
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[],
                output="screen",
                condition=IfCondition(LaunchConfiguration("run_standalone")),
            ),
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=inference_components,
            ),
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[control_decoder],
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("output_task"), "' == 'control'"])
                ),
            ),
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[trajectory_decoder],
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("output_task"), "' == 'trajectory'"])
                ),
            ),
        ]
    )
