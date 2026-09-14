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
    container_name = LaunchConfiguration("container_name")
    rgb_model_root = LaunchConfiguration("rgb_model_root")
    updater_model_root = LaunchConfiguration("updater_model_root")

    image_encoder = ComposableNode(
        package="isaac_ros_dnn_image_encoder",
        plugin="nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode",
        name="rgb_latent_image_encoder",
        parameters=[{
            "input_qos": "SENSOR_DATA",
            "input_image_width": ParameterValue(
                LaunchConfiguration("rgb_input_width"), value_type=int),
            "input_image_height": ParameterValue(
                LaunchConfiguration("rgb_input_height"), value_type=int),
            "network_image_width": ParameterValue(
                LaunchConfiguration("network_width"), value_type=int),
            "network_image_height": ParameterValue(
                LaunchConfiguration("network_height"), value_type=int),
            "input_encoding": LaunchConfiguration("rgb_input_encoding"),
            "enable_padding": ParameterValue(
                LaunchConfiguration("rgb_enable_padding"), value_type=bool),
            "image_mean": LaunchConfiguration("rgb_mean"),
            "image_stddev": LaunchConfiguration("rgb_stddev"),
            "tensor_name": LaunchConfiguration("rgb_input_tensor_name"),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        remappings=[
            ("image", LaunchConfiguration("rgb_image_topic")),
            ("camera_info", LaunchConfiguration("rgb_camera_info_topic")),
            ("tensors", LaunchConfiguration("rgb_tensor_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    rgb_tensor_rt = ComposableNode(
        package="isaac_ros_tensor_rt",
        plugin="nvidia::isaac_ros::dnn_inference::TensorRTNode",
        name="rgb_latent_tensor_rt",
        parameters=[{
            "model_file_path": PathJoinSubstitution(
                [rgb_model_root, LaunchConfiguration("rgb_model_filename")]),
            "engine_file_path": PathJoinSubstitution(
                [rgb_model_root, LaunchConfiguration("rgb_engine_filename")]),
            "force_engine_update": ParameterValue(
                LaunchConfiguration("force_engine_update"), value_type=bool),
            "enable_fp16": ParameterValue(
                LaunchConfiguration("enable_fp16"), value_type=bool),
            "input_tensor_names": LaunchConfiguration("rgb_input_tensor_names"),
            "input_binding_names": LaunchConfiguration("rgb_input_binding_names"),
            "input_tensor_formats": LaunchConfiguration("rgb_input_tensor_formats"),
            "output_tensor_names": LaunchConfiguration("rgb_output_tensor_names"),
            "output_binding_names": LaunchConfiguration("rgb_output_binding_names"),
            "output_tensor_formats": LaunchConfiguration("rgb_output_tensor_formats"),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        remappings=[
            ("tensor_pub", LaunchConfiguration("rgb_tensor_topic")),
            ("tensor_sub", LaunchConfiguration("rgb_latent_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    event_encoder = ComposableNode(
        package="jetpilot_e2e_inference",
        plugin="jetpilot_e2e_inference::EventTensorEncoderNode",
        name="async_event_tensor_encoder",
        parameters=[{
            "bins": ParameterValue(LaunchConfiguration("event_bins"), value_type=int),
            "width": ParameterValue(LaunchConfiguration("network_width"), value_type=int),
            "height": ParameterValue(LaunchConfiguration("network_height"), value_type=int),
            "window_ms": ParameterValue(
                LaunchConfiguration("event_window_ms"), value_type=float),
            "stride_ms": ParameterValue(
                LaunchConfiguration("event_stride_ms"), value_type=float),
            "output_rate_hz": ParameterValue(
                LaunchConfiguration("event_output_rate_hz"), value_type=float),
            "polarity_mode": LaunchConfiguration("event_polarity_mode"),
            "polarity_layout": LaunchConfiguration("event_polarity_layout"),
            "temporal_interpolation": LaunchConfiguration("event_temporal_interpolation"),
            "incremental_mode": LaunchConfiguration("event_incremental_mode"),
            "representation_backend": LaunchConfiguration("event_representation_backend"),
            "inference_policy": LaunchConfiguration("event_inference_policy"),
            "cuda_update_us": ParameterValue(
                LaunchConfiguration("event_cuda_update_us"), value_type=int),
            "timestamp_backward_tolerance_us": ParameterValue(
                LaunchConfiguration("event_timestamp_backward_tolerance_us"),
                value_type=int),
            "cuda_events_per_transfer": ParameterValue(
                LaunchConfiguration("event_cuda_events_per_transfer"), value_type=int),
            "inference_watchdog_ms": ParameterValue(
                LaunchConfiguration("inference_watchdog_ms"), value_type=float),
            "channel_mean": LaunchConfiguration("event_mean"),
            "channel_stddev": LaunchConfiguration("event_stddev"),
            "tensor_name": LaunchConfiguration("event_tensor_name"),
            "publish_empty": True,
            "debug": ParameterValue(LaunchConfiguration("debug"), value_type=bool),
            "statistics_interval_s": ParameterValue(
                LaunchConfiguration("statistics_interval_s"), value_type=float),
            "diagnostics_topic": LaunchConfiguration("event_diagnostics_topic"),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        remappings=[
            ("events", LaunchConfiguration("event_topic")),
            ("tensor", LaunchConfiguration("event_tensor_topic")),
            ("tensor_feedback", LaunchConfiguration("event_feedback_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    async_event_preprocessor = ComposableNode(
        package="jetpilot_e2e_inference",
        plugin="jetpilot_e2e_inference::AsyncEventTensorPreprocessorNode",
        name="async_event_tensor_preprocessor",
        parameters=[{
            "bins": ParameterValue(LaunchConfiguration("event_bins"), value_type=int),
            "width": ParameterValue(LaunchConfiguration("network_width"), value_type=int),
            "height": ParameterValue(LaunchConfiguration("network_height"), value_type=int),
            "window_ms": ParameterValue(
                LaunchConfiguration("event_window_ms"), value_type=float),
            "stride_ms": ParameterValue(
                LaunchConfiguration("event_stride_ms"), value_type=float),
            "output_rate_hz": ParameterValue(
                LaunchConfiguration("event_output_rate_hz"), value_type=float),
            "polarity_mode": LaunchConfiguration("event_polarity_mode"),
            "polarity_layout": LaunchConfiguration("event_polarity_layout"),
            "temporal_interpolation": LaunchConfiguration("event_temporal_interpolation"),
            "timestamp_backward_tolerance_us": ParameterValue(
                LaunchConfiguration("event_timestamp_backward_tolerance_us"), value_type=int),
            "cuda_events_per_transfer": ParameterValue(
                LaunchConfiguration("event_cuda_events_per_transfer"), value_type=int),
            "gpu_chunk_events": ParameterValue(
                LaunchConfiguration("event_async_gpu_chunk_events"), value_type=int),
            "packet_queue_capacity": ParameterValue(
                LaunchConfiguration("event_async_packet_queue_capacity"), value_type=int),
            "decoded_queue_capacity": ParameterValue(
                LaunchConfiguration("event_async_decoded_queue_capacity"), value_type=int),
            "max_queue_age_ms": ParameterValue(
                LaunchConfiguration("event_async_max_queue_age_ms"), value_type=float),
            "deadline_ms": ParameterValue(
                LaunchConfiguration("event_async_deadline_ms"), value_type=float),
            "channel_mean": LaunchConfiguration("event_mean"),
            "channel_stddev": LaunchConfiguration("event_stddev"),
            "tensor_name": LaunchConfiguration("event_tensor_name"),
            "publish_empty": True,
            "debug": ParameterValue(LaunchConfiguration("debug"), value_type=bool),
            "statistics_interval_s": ParameterValue(
                LaunchConfiguration("statistics_interval_s"), value_type=float),
            "memory_pool_num_blocks": ParameterValue(
                LaunchConfiguration("event_async_memory_pool_num_blocks"), value_type=int),
            "diagnostics_topic": LaunchConfiguration("event_diagnostics_topic"),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        remappings=[
            ("events", LaunchConfiguration("event_topic")),
            ("tensor", LaunchConfiguration("event_tensor_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    state_manager = ComposableNode(
        package="jetpilot_e2e_inference",
        plugin="jetpilot_e2e_inference::LatentStateManagerNode",
        name="latent_state_manager",
        parameters=[{
            "rgb_latent_tensor_name": LaunchConfiguration("rgb_latent_tensor_name"),
            "state_input_tensor_name": LaunchConfiguration("state_input_tensor_name"),
            "state_output_tensor_name": LaunchConfiguration("state_output_tensor_name"),
            "event_source_tensor_name": LaunchConfiguration("event_tensor_name"),
            "event_input_tensor_name": LaunchConfiguration("event_tensor_name"),
            "delta_t_tensor_name": LaunchConfiguration("delta_t_tensor_name"),
            "include_delta_t": ParameterValue(
                LaunchConfiguration("include_delta_t"), value_type=bool),
            "initial_delta_t_s": ParameterValue(
                LaunchConfiguration("initial_delta_t_s"), value_type=float),
            "max_delta_t_s": ParameterValue(
                LaunchConfiguration("max_delta_t_s"), value_type=float),
            "inference_watchdog_ms": ParameterValue(
                LaunchConfiguration("inference_watchdog_ms"), value_type=float),
            "statistics_interval_s": ParameterValue(
                LaunchConfiguration("statistics_interval_s"), value_type=float),
            "diagnostics_topic": LaunchConfiguration("state_diagnostics_topic"),
            "debug": ParameterValue(LaunchConfiguration("debug"), value_type=bool),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        remappings=[
            ("rgb_latent", LaunchConfiguration("rgb_latent_topic")),
            ("event_tensor", LaunchConfiguration("event_tensor_topic")),
            ("updater_input", LaunchConfiguration("updater_input_topic")),
            ("updater_output", LaunchConfiguration("updater_output_topic")),
            ("output", LaunchConfiguration("output_topic")),
            ("event_feedback", LaunchConfiguration("event_feedback_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    updater_tensor_rt = ComposableNode(
        package="isaac_ros_tensor_rt",
        plugin="nvidia::isaac_ros::dnn_inference::TensorRTNode",
        name="evs_latent_updater_tensor_rt",
        parameters=[{
            "model_file_path": PathJoinSubstitution(
                [updater_model_root, LaunchConfiguration("updater_model_filename")]),
            "engine_file_path": PathJoinSubstitution(
                [updater_model_root, LaunchConfiguration("updater_engine_filename")]),
            "force_engine_update": ParameterValue(
                LaunchConfiguration("force_engine_update"), value_type=bool),
            "enable_fp16": ParameterValue(
                LaunchConfiguration("enable_fp16"), value_type=bool),
            "input_tensor_names": LaunchConfiguration("updater_input_tensor_names"),
            "input_binding_names": LaunchConfiguration("updater_input_binding_names"),
            "input_tensor_formats": LaunchConfiguration("updater_input_tensor_formats"),
            "output_tensor_names": LaunchConfiguration("updater_output_tensor_names"),
            "output_binding_names": LaunchConfiguration("updater_output_binding_names"),
            "output_tensor_formats": LaunchConfiguration("updater_output_tensor_formats"),
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
        remappings=[
            ("tensor_pub", LaunchConfiguration("updater_input_topic")),
            ("tensor_sub", LaunchConfiguration("updater_output_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    control_decoder = ComposableNode(
        package="jetpilot_e2e_inference",
        plugin="jetpilot_e2e_inference::E2EControlDecoderNode",
        name="async_rgb_evs_control_decoder",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "e2e_inference.param.yaml"]),
            {
                "output_tensor_name": LaunchConfiguration("control_tensor_name"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
            },
        ],
        remappings=[
            ("tensor_sub", LaunchConfiguration("output_topic")),
            ("control_cmd", LaunchConfiguration("control_cmd_topic")),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    arguments = [
        DeclareLaunchArgument("container_name", default_value="multi_sensor_container"),
        DeclareLaunchArgument("run_standalone", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("model_root"),
        DeclareLaunchArgument("rgb_model_root", default_value=LaunchConfiguration("model_root")),
        DeclareLaunchArgument(
            "updater_model_root", default_value=LaunchConfiguration("model_root")),
        DeclareLaunchArgument("rgb_model_filename", default_value="rgb_encoder.onnx"),
        DeclareLaunchArgument("rgb_engine_filename", default_value="rgb_encoder.plan"),
        DeclareLaunchArgument("updater_model_filename", default_value="event_updater.onnx"),
        DeclareLaunchArgument("updater_engine_filename", default_value="event_updater.plan"),
        DeclareLaunchArgument("force_engine_update", default_value="false"),
        DeclareLaunchArgument("enable_fp16", default_value="true"),
        DeclareLaunchArgument("rgb_image_topic", default_value="/realsense/color/image_raw"),
        DeclareLaunchArgument(
            "rgb_camera_info_topic", default_value="/realsense/color/camera_info"),
        DeclareLaunchArgument("rgb_input_width", default_value="424"),
        DeclareLaunchArgument("rgb_input_height", default_value="240"),
        DeclareLaunchArgument("network_width", default_value="212"),
        DeclareLaunchArgument("network_height", default_value="120"),
        DeclareLaunchArgument("rgb_input_encoding", default_value="rgb8"),
        DeclareLaunchArgument("rgb_enable_padding", default_value="true"),
        DeclareLaunchArgument("rgb_mean", default_value="[0.485, 0.456, 0.406]"),
        DeclareLaunchArgument("rgb_stddev", default_value="[0.229, 0.224, 0.225]"),
        DeclareLaunchArgument("event_topic", default_value="/event_camera/events"),
        DeclareLaunchArgument("event_preprocessor_mode", default_value="legacy"),
        DeclareLaunchArgument("event_bins", default_value="10"),
        DeclareLaunchArgument("event_window_ms", default_value="40.0"),
        DeclareLaunchArgument("event_stride_ms", default_value="4.0"),
        DeclareLaunchArgument("event_output_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("event_polarity_mode", default_value="separate"),
        DeclareLaunchArgument("event_polarity_layout", default_value="polarity_major"),
        DeclareLaunchArgument("event_temporal_interpolation", default_value="none"),
        DeclareLaunchArgument("event_incremental_mode", default_value="auto"),
        DeclareLaunchArgument("event_representation_backend", default_value="cuda"),
        DeclareLaunchArgument("event_inference_policy", default_value="periodic"),
        DeclareLaunchArgument("event_cuda_update_us", default_value="1000"),
        DeclareLaunchArgument(
            "event_timestamp_backward_tolerance_us", default_value="4000"),
        DeclareLaunchArgument("event_cuda_events_per_transfer", default_value="8192"),
        DeclareLaunchArgument("event_async_gpu_chunk_events", default_value="8192"),
        DeclareLaunchArgument("event_async_packet_queue_capacity", default_value="64"),
        DeclareLaunchArgument("event_async_decoded_queue_capacity", default_value="64"),
        DeclareLaunchArgument("event_async_max_queue_age_ms", default_value="20.0"),
        DeclareLaunchArgument("event_async_memory_pool_num_blocks", default_value="16"),
        DeclareLaunchArgument("event_async_deadline_ms", default_value="4.0"),
        DeclareLaunchArgument("event_mean", default_value="[0.0]"),
        DeclareLaunchArgument("event_stddev", default_value="[1.0]"),
        DeclareLaunchArgument("inference_watchdog_ms", default_value="100.0"),
        DeclareLaunchArgument("statistics_interval_s", default_value="1.0"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("include_delta_t", default_value="true"),
        DeclareLaunchArgument("initial_delta_t_s", default_value="0.001"),
        DeclareLaunchArgument("max_delta_t_s", default_value="0.1"),
        DeclareLaunchArgument("rgb_input_tensor_name", default_value="rgb_input"),
        DeclareLaunchArgument("rgb_latent_tensor_name", default_value="rgb_latent"),
        DeclareLaunchArgument("event_tensor_name", default_value="event_tensor"),
        DeclareLaunchArgument("state_input_tensor_name", default_value="state_in"),
        DeclareLaunchArgument("state_output_tensor_name", default_value="state_out"),
        DeclareLaunchArgument("delta_t_tensor_name", default_value="delta_t"),
        DeclareLaunchArgument("control_tensor_name", default_value="control"),
        DeclareLaunchArgument("rgb_input_tensor_names", default_value="['rgb_input']"),
        DeclareLaunchArgument("rgb_input_binding_names", default_value="['rgb']"),
        DeclareLaunchArgument(
            "rgb_input_tensor_formats", default_value="['nitros_tensor_list_nchw_rgb_f32']"),
        DeclareLaunchArgument("rgb_output_tensor_names", default_value="['rgb_latent']"),
        DeclareLaunchArgument("rgb_output_binding_names", default_value="['rgb_latent']"),
        DeclareLaunchArgument(
            "rgb_output_tensor_formats", default_value="['nitros_tensor_list_nchw_rgb_f32']"),
        DeclareLaunchArgument(
            "updater_input_tensor_names", default_value="['state_in', 'event_tensor', 'delta_t']"),
        DeclareLaunchArgument(
            "updater_input_binding_names", default_value="['state_in', 'event_tensor', 'delta_t']"),
        DeclareLaunchArgument(
            "updater_input_tensor_formats",
            default_value="['nitros_tensor_list_nchw_rgb_f32']"),
        DeclareLaunchArgument(
            "updater_output_tensor_names", default_value="['state_out', 'control']"),
        DeclareLaunchArgument(
            "updater_output_binding_names", default_value="['state_out', 'control']"),
        DeclareLaunchArgument(
            "updater_output_tensor_formats",
            default_value="['nitros_tensor_list_nchw_rgb_f32']"),
        DeclareLaunchArgument("rgb_tensor_topic", default_value="/e2e/latent/rgb_input"),
        DeclareLaunchArgument("rgb_latent_topic", default_value="/e2e/latent/rgb"),
        DeclareLaunchArgument("event_tensor_topic", default_value="/e2e/latent/event"),
        DeclareLaunchArgument("updater_input_topic", default_value="/e2e/latent/updater_input"),
        DeclareLaunchArgument("updater_output_topic", default_value="/e2e/latent/updater_output"),
        DeclareLaunchArgument("output_topic", default_value="/e2e/tensor_output"),
        DeclareLaunchArgument(
            "event_feedback_topic", default_value="/e2e/latent/event_feedback"),
        DeclareLaunchArgument(
            "event_diagnostics_topic", default_value="/e2e/event_tensor/diagnostics"),
        DeclareLaunchArgument(
            "state_diagnostics_topic", default_value="/e2e/latent_state/diagnostics"),
        DeclareLaunchArgument("control_cmd_topic", default_value="/auto/control_cmd"),
    ]

    container = ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[],
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_standalone")),
    )
    nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            image_encoder,
            rgb_tensor_rt,
            state_manager,
            updater_tensor_rt,
            control_decoder,
        ],
    )
    legacy_event_node = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[event_encoder],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("event_preprocessor_mode"), "'.lower() != 'async'",
        ])),
    )
    async_event_node = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[async_event_preprocessor],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("event_preprocessor_mode"), "'.lower() == 'async'",
        ])),
    )
    return LaunchDescription(arguments + [container, nodes, legacy_event_node, async_event_node])
