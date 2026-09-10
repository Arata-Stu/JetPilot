from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    e2e_share = FindPackageShare("jetpilot_e2e_inference")
    detection_share = FindPackageShare("jetpilot_object_detection")
    container_name = LaunchConfiguration("container_name")
    model_root = LaunchConfiguration("model_root")

    def event_value(event, camera):
        return PythonExpression([
            "'", event, "' if '", LaunchConfiguration("event_image_mode"),
            "'.lower() == 'true' else '", camera, "'",
        ])

    image_encoder = ComposableNode(
        package="isaac_ros_dnn_image_encoder",
        plugin="nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode",
        name="shared_vit_image_encoder",
        parameters=[{
            "input_qos": "SENSOR_DATA",
            "input_image_width": ParameterValue(
                event_value(LaunchConfiguration("network_width"), LaunchConfiguration("source_width")), value_type=int
            ),
            "input_image_height": ParameterValue(
                event_value(LaunchConfiguration("network_height"), LaunchConfiguration("source_height")), value_type=int
            ),
            "network_image_width": ParameterValue(LaunchConfiguration("network_width"), value_type=int),
            "network_image_height": ParameterValue(LaunchConfiguration("network_height"), value_type=int),
            "input_encoding": event_value("rgb8", LaunchConfiguration("input_encoding")),
            "enable_padding": True,
            "image_mean": event_value(LaunchConfiguration("event_image_mean"), LaunchConfiguration("image_mean")),
            "image_stddev": event_value(LaunchConfiguration("event_image_stddev"), LaunchConfiguration("image_stddev")),
            "tensor_name": "input_tensor",
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        remappings=[
            ("image", event_value("/shared_vit/event/image", LaunchConfiguration("image_topic"))),
            ("camera_info", event_value("/shared_vit/event/camera_info", LaunchConfiguration("camera_info_topic"))),
            ("tensors", LaunchConfiguration("tensor_input_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    tensor_rt = ComposableNode(
        package="isaac_ros_tensor_rt",
        plugin="nvidia::isaac_ros::dnn_inference::TensorRTNode",
        name="shared_vit_tensor_rt",
        parameters=[{
            "model_file_path": PathJoinSubstitution([model_root, "model.onnx"]),
            "engine_file_path": PathJoinSubstitution([model_root, "model.plan"]),
            "force_engine_update": ParameterValue(LaunchConfiguration("force_engine_update"), value_type=bool),
            "enable_fp16": True,
            "input_tensor_names": ["input_tensor"],
            "input_binding_names": ["image"],
            "input_tensor_formats": ["nitros_tensor_list_nchw_rgb_f32"],
            "output_tensor_names": ["control_output", "detection_output"],
            "output_binding_names": ["control", "detections"],
            "output_tensor_formats": [
                "nitros_tensor_list_nchw_rgb_f32",
                "nitros_tensor_list_nchw_rgb_f32",
            ],
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
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
        parameters=[
            PathJoinSubstitution([e2e_share, "config", "e2e_inference.param.yaml"]),
            {
                "output_tensor_name": "control_output",
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            },
        ],
        remappings=[
            ("tensor_sub", LaunchConfiguration("tensor_output_topic")),
            ("control_cmd", LaunchConfiguration("control_cmd_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    detection_decoder = ComposableNode(
        package="jetpilot_object_detection",
        plugin="jetpilot_object_detection::YoloV8DecoderNode",
        name="yolov8_decoder",
        parameters=[
            PathJoinSubstitution([detection_share, "config", "yolov8.param.yaml"]),
            {
                "output_tensor_name": "detection_output",
                "network_width": ParameterValue(LaunchConfiguration("network_width"), value_type=int),
                "network_height": ParameterValue(LaunchConfiguration("network_height"), value_type=int),
                "source_width": ParameterValue(LaunchConfiguration("source_width"), value_type=int),
                "source_height": ParameterValue(LaunchConfiguration("source_height"), value_type=int),
                "resize_mode": "letterbox",
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            },
        ],
        remappings=[
            ("tensor_sub", LaunchConfiguration("tensor_output_topic")),
            ("detections_output", LaunchConfiguration("detections_topic")),
            ("diagnostics", LaunchConfiguration("detection_diagnostics_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("container_name", default_value="multi_sensor_container"),
        DeclareLaunchArgument("run_standalone", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("event_image_mode", default_value="false"),
        DeclareLaunchArgument("image_topic", default_value="/realsense/color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/realsense/color/camera_info"),
        DeclareLaunchArgument("source_width", default_value="424"),
        DeclareLaunchArgument("source_height", default_value="240"),
        DeclareLaunchArgument("network_width", default_value="212"),
        DeclareLaunchArgument("network_height", default_value="120"),
        DeclareLaunchArgument("input_encoding", default_value="rgb8"),
        DeclareLaunchArgument("image_mean", default_value="[0.485, 0.456, 0.406]"),
        DeclareLaunchArgument("image_stddev", default_value="[0.229, 0.224, 0.225]"),
        DeclareLaunchArgument("event_image_mean", default_value="[0.8993729785, 0.7969581015, 0.8928228776]"),
        DeclareLaunchArgument("event_image_stddev", default_value="[0.2204336077, 0.2921656668, 0.2204992771]"),
        DeclareLaunchArgument("model_root", default_value="/workspaces/ros2_ws/models/e2e/shared_vit"),
        DeclareLaunchArgument("tensor_input_topic", default_value="/shared_vit/tensor_input"),
        DeclareLaunchArgument("tensor_output_topic", default_value="/shared_vit/tensor_output"),
        DeclareLaunchArgument("control_cmd_topic", default_value="/auto/control_cmd"),
        DeclareLaunchArgument("detections_topic", default_value="/perception/detections"),
        DeclareLaunchArgument("detection_diagnostics_topic", default_value="/perception/object_detection/diagnostics"),
        DeclareLaunchArgument("force_engine_update", default_value="false"),
        ComposableNodeContainer(
            name=container_name,
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[],
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_standalone")),
        ),
        Node(
            package="jetpilot_e2e_inference",
            executable="event_image_adapter.py",
            name="shared_vit_event_image_adapter",
            parameters=[{
                "width": ParameterValue(LaunchConfiguration("network_width"), value_type=int),
                "height": ParameterValue(LaunchConfiguration("network_height"), value_type=int),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
            remappings=[
                ("event_image", LaunchConfiguration("image_topic")),
                ("image", "/shared_vit/event/image"),
                ("camera_info", "/shared_vit/event/camera_info"),
            ],
            condition=IfCondition(LaunchConfiguration("event_image_mode")),
        ),
        LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[image_encoder, tensor_rt, control_decoder, detection_decoder],
        ),
    ])
