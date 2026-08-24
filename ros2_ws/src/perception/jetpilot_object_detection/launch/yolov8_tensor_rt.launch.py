from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("jetpilot_object_detection")
    container_name = LaunchConfiguration("container_name")
    model_root = LaunchConfiguration("model_root")

    image_gate = ComposableNode(
        package="jetpilot_object_detection",
        plugin="jetpilot_object_detection::ImageGateNode",
        name="object_detection_image_gate",
        namespace="",
        parameters=[
            {
                "max_fps": ParameterValue(
                    LaunchConfiguration("max_inference_fps"), value_type=float
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
        remappings=[
            ("image_input", LaunchConfiguration("image_topic")),
            ("camera_info_input", LaunchConfiguration("camera_info_topic")),
            ("image_output", LaunchConfiguration("gated_image_topic")),
            ("camera_info_output", LaunchConfiguration("gated_camera_info_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    image_encoder = ComposableNode(
        package="isaac_ros_dnn_image_encoder",
        plugin="nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode",
        name="object_detection_image_encoder",
        namespace="",
        parameters=[
            {
                "input_image_width": ParameterValue(
                    LaunchConfiguration("source_width"), value_type=int
                ),
                "input_image_height": ParameterValue(
                    LaunchConfiguration("source_height"), value_type=int
                ),
                "network_image_width": ParameterValue(
                    LaunchConfiguration("network_width"), value_type=int
                ),
                "network_image_height": ParameterValue(
                    LaunchConfiguration("network_height"), value_type=int
                ),
                "input_encoding": LaunchConfiguration("input_encoding"),
                "enable_padding": True,
                # Ultralytics inference uses RGB values scaled to [0, 1].
                "image_mean": "[0.0, 0.0, 0.0]",
                "image_stddev": "[1.0, 1.0, 1.0]",
                "tensor_name": "input_tensor",
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
        remappings=[
            ("image", LaunchConfiguration("gated_image_topic")),
            ("camera_info", LaunchConfiguration("gated_camera_info_topic")),
            ("tensors", LaunchConfiguration("tensor_input_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    tensor_rt = ComposableNode(
        package="isaac_ros_tensor_rt",
        plugin="nvidia::isaac_ros::dnn_inference::TensorRTNode",
        name="object_detection_tensor_rt",
        namespace="",
        parameters=[
            {
                "model_file_path": PathJoinSubstitution([model_root, "model.onnx"]),
                "engine_file_path": PathJoinSubstitution([model_root, "model.plan"]),
                "force_engine_update": ParameterValue(
                    LaunchConfiguration("force_engine_update"), value_type=bool
                ),
                "enable_fp16": True,
                "input_tensor_names": ["input_tensor"],
                "input_binding_names": ["images"],
                "input_tensor_formats": ["nitros_tensor_list_nchw_rgb_f32"],
                "output_tensor_names": ["output_tensor"],
                "output_binding_names": ["output0"],
                "output_tensor_formats": ["nitros_tensor_list_nchw_rgb_f32"],
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

    decoder = ComposableNode(
        package="jetpilot_object_detection",
        plugin="jetpilot_object_detection::YoloV8DecoderNode",
        name="yolov8_decoder",
        namespace="",
        parameters=[
            LaunchConfiguration("decoder_param_file"),
            {
                "network_width": ParameterValue(
                    LaunchConfiguration("network_width"), value_type=int
                ),
                "network_height": ParameterValue(
                    LaunchConfiguration("network_height"), value_type=int
                ),
                "source_width": ParameterValue(
                    LaunchConfiguration("source_width"), value_type=int
                ),
                "source_height": ParameterValue(
                    LaunchConfiguration("source_height"), value_type=int
                ),
                "confidence_threshold": ParameterValue(
                    LaunchConfiguration("confidence_threshold"), value_type=float
                ),
                "nms_threshold": ParameterValue(
                    LaunchConfiguration("nms_threshold"), value_type=float
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            },
        ],
        remappings=[
            ("tensor_sub", LaunchConfiguration("tensor_output_topic")),
            ("detections_output", LaunchConfiguration("detections_topic")),
            ("diagnostics", LaunchConfiguration("diagnostics_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    components = [image_gate, image_encoder, tensor_rt, decoder]
    return LaunchDescription(
        [
            DeclareLaunchArgument("container_name", default_value="multi_sensor_container"),
            DeclareLaunchArgument("run_standalone", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("image_topic", default_value="/realsense/color/image_raw"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/realsense/color/camera_info"
            ),
            DeclareLaunchArgument("source_width", default_value="424"),
            DeclareLaunchArgument("source_height", default_value="240"),
            DeclareLaunchArgument("network_width", default_value="224"),
            DeclareLaunchArgument("network_height", default_value="224"),
            DeclareLaunchArgument("max_inference_fps", default_value="15.0"),
            DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
            DeclareLaunchArgument("nms_threshold", default_value="0.45"),
            DeclareLaunchArgument("input_encoding", default_value="rgb8"),
            DeclareLaunchArgument(
                "model_root", default_value="/workspaces/ros2_ws/models/yolov8/latest"
            ),
            DeclareLaunchArgument(
                "decoder_param_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "yolov8.param.yaml"]
                ),
            ),
            DeclareLaunchArgument("force_engine_update", default_value="false"),
            DeclareLaunchArgument(
                "tensor_input_topic", default_value="/perception/object_detection/tensor_input"
            ),
            DeclareLaunchArgument(
                "gated_image_topic", default_value="/perception/object_detection/image"
            ),
            DeclareLaunchArgument(
                "gated_camera_info_topic",
                default_value="/perception/object_detection/camera_info",
            ),
            DeclareLaunchArgument(
                "tensor_output_topic", default_value="/perception/object_detection/tensor_output"
            ),
            DeclareLaunchArgument(
                "detections_topic", default_value="/perception/detections"
            ),
            DeclareLaunchArgument(
                "diagnostics_topic", default_value="/perception/object_detection/diagnostics"
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
                composable_node_descriptions=components,
            ),
        ]
    )
