# SPDX-License-Identifier: Apache-2.0

import os
import re

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, OrSubstitution


_TRUE_VALUES = {'1', 'true', 'yes', 'on'}
_ABSOLUTE_TOPIC_PATTERN = re.compile(
    r'^/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$')
_REPLAY_ISOLATED_TOPICS = (
    '/joy',
    '/rc/channels',
    '/teleop/control_cmd',
    '/propo/control_cmd',
    '/auto/control_cmd',
    '/vehicle/control_cmd',
    '/control_cmd',
    '/ackermann_cmd',
    '/operation_mode/request',
    '/operation_mode/state',
    '/planning/requested_lane',
    '/planning/raceline_path',
    '/planning/trajectory',
    '/planning/target_speed',
    '/planning/selected_lane',
    '/planning/ready',
    '/controller/ready',
    '/controller/lookahead_point',
    '/bag/request',
    '/bag/status',
    '/commands/motor/duty_cycle',
    '/commands/motor/current',
    '/commands/motor/speed',
    '/commands/motor/brake',
    '/commands/motor/position',
    '/commands/servo/position',
    '/steer_offset_inc',
    '/steer_offset_dec',
    '/speed_offset_inc',
    '/speed_offset_dec',
    '/localization/trigger',
    '/localization/pose_hint',
    '/initialpose',
    '/visual_slam/trigger_hint',
    '/visual_localization/trigger_localization',
    '/visual_localization/pose',
    '/visual_slam/tracking/odometry',
    '/visual_slam/tracking/slam_path',
    '/diagnostics',
    '/localization/diagnostics',
    '/planning/diagnostics',
    '/controller/diagnostics',
    '/jetson/diagnostics',
    '/localization/pose_hint_required',
    '/localization/pose_hint_state',
    '/localization/current_section',
    '/tf',
)


def _as_bool(value) -> bool:
    return str(value).strip().lower() in _TRUE_VALUES


def _normalized_isolated_topics(extra_topics=()) -> tuple[str, ...]:
    topics = []
    for value in (*_REPLAY_ISOLATED_TOPICS, *extra_topics):
        topic = str(value or '').strip()
        if not _ABSOLUTE_TOPIC_PATTERN.fullmatch(topic):
            raise RuntimeError(
                f'Cannot safely isolate invalid or relative replay topic: {topic!r}')
        if topic not in topics:
            topics.append(topic)
    return tuple(topics)


def _compose_replay_arguments(
        additional_args, allow_unsafe_control_topics: bool, extra_topics=()) -> str:
    additional = str(additional_args or '').strip()
    if allow_unsafe_control_topics:
        return additional

    tokens = additional.split()
    if any(
            token == '-m' or token.startswith('-m=') or token.startswith('-m/')
            or token.startswith('--remap')
            for token in tokens):
        raise RuntimeError(
            'replay_additional_args cannot contain --remap/-m during safe replay. '
            'JetPilot reserves remapping to isolate recorded control topics. Set '
            'allow_unsafe_replay_control_topics:=true only in an isolated test domain.'
        )

    remap_arguments = '--remap ' + ' '.join(
        f'{topic}:=/replay{topic}'
        for topic in _normalized_isolated_topics(extra_topics)
    )
    return ' '.join(part for part in (additional, remap_arguments) if part)


class _LaunchBoolean(lut.Substitution):
    """Use one boolean vocabulary in guards, node conditions, and replay args."""

    def __init__(self, value):
        super().__init__()
        self._value = value

    def perform(self, context) -> str:
        value = lu.perform_context(context, self._value)
        return 'true' if _as_bool(value) else 'false'

    def describe(self) -> str:
        return 'normalized JetPilot launch boolean'


class _ReplayArguments(lut.Substitution):
    def __init__(self, additional_args, allow_unsafe_control_topics, extra_topics=()):
        super().__init__()
        self._additional_args = additional_args
        self._allow_unsafe_control_topics = allow_unsafe_control_topics
        self._extra_topics = tuple(extra_topics)

    def perform(self, context) -> str:
        additional = lu.perform_context(context, self._additional_args)
        unsafe = _as_bool(lu.perform_context(context, self._allow_unsafe_control_topics))
        extra_topics = tuple(
            lu.perform_context(context, topic) for topic in self._extra_topics)
        return _compose_replay_arguments(additional, unsafe, extra_topics)

    def describe(self) -> str:
        return 'JetPilot safe rosbag replay arguments'


def _launch_bool(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context)
    return _as_bool(value)


def _validate_replay_vehicle_safety(context):
    replay_enabled = _launch_bool(context, 'enable_rosbag_replay')
    rosbag = LaunchConfiguration('rosbag').perform(context).strip()
    vehicle_enabled = _launch_bool(context, 'enable_vehicle')
    override_enabled = _launch_bool(context, 'allow_unsafe_replay_with_vehicle')
    unsafe_controls = _launch_bool(context, 'allow_unsafe_replay_control_topics')

    if replay_enabled and rosbag:
        additional_args = LaunchConfiguration('replay_additional_args').perform(context)
        _compose_replay_arguments(additional_args, unsafe_controls)

    if replay_enabled and rosbag and vehicle_enabled and not override_enabled:
        raise RuntimeError(
            'Unsafe launch configuration rejected: rosbag replay and the vehicle '
            'interface cannot be enabled together. Set enable_vehicle:=false for '
            'offline replay. For an intentional hardware-in-the-loop test, isolate '
            'the ROS domain and explicitly set '
            'allow_unsafe_replay_with_vehicle:=true.'
        )

    return []


def workspace_param_path(filename: str, fallback_package: str, fallback_package_path: str) -> str:
    ros2_ws = os.environ.get('ROS2_WS', '/workspaces/ros2_ws')
    generated_path = os.path.join(ros2_ws, 'joy_profiles', filename)
    if os.path.exists(generated_path):
        return generated_path
    return lu.get_path(fallback_package, fallback_package_path)


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg('use_sim_time', False, cli=True)

    args.add_arg('enable_rosbag_replay', False, cli=True)
    args.add_arg('rosbag', '', cli=True)
    args.add_arg('replay_rate', '1.0', cli=True)
    args.add_arg('replay_additional_args', '', cli=True)
    args.add_arg('rosbag_start_delay_s', '0.0', cli=True)
    args.add_arg('rosbag_shutdown_on_exit', True, cli=True)
    args.add_arg('allow_unsafe_replay_control_topics', False, cli=True)
    args.add_arg('allow_unsafe_replay_with_vehicle', False, cli=True)

    args.add_arg('enable_tool', True, cli=True)
    args.add_arg('enable_bag_manager', True, cli=True)
    args.add_arg('enable_joy', True, cli=True)
    args.add_arg('enable_teleop', True, cli=True)
    args.add_arg('enable_rc_serial', False, cli=True)
    args.add_arg('enable_jetson_stats', False, cli=True)
    args.add_arg('jetson_stats_diagnostics_topic', '/jetson/diagnostics', cli=True)
    args.add_arg('jetson_stats_interval', '0.5', cli=True)
    args.add_arg('control_authority', 'hardware_mux', cli=True)
    args.add_arg(
        'bag_manager_param',
        lu.get_path('jetpilot_system_launch', 'config/tool/bag_manager.param.yaml'),
        cli=True)
    args.add_arg(
        'teleop_cmd_param',
        workspace_param_path(
            'teleop_cmd.param.yaml',
            'jetpilot_system_launch',
            'config/tool/teleop_cmd.param.yaml'),
        cli=True)
    args.add_arg(
        'teleop_button_mapping_param',
        workspace_param_path(
            'joy_button_mapping.param.yaml',
            'jetpilot_system_launch',
            'config/tool/joy_button_mapping.param.yaml'),
        cli=True)
    args.add_arg(
        'serial_reader_param',
        workspace_param_path(
            'serial_reader_node.param.yaml',
            'rc_serial_reader',
            'config/serial_reader_node.param.yaml'),
        cli=True)
    args.add_arg('rc_channels_topic', '/rc/channels', cli=True)
    args.add_arg('propo_control_topic', '/propo/control_cmd', cli=True)
    args.add_arg('joy_autorepeat_rate', '50.0', cli=True)
    args.add_arg('joy_deadzone', '0.05', cli=True)
    args.add_arg('joy_device_path', '', cli=True)
    args.add_arg('joy_prefer_evdev', False, cli=True)

    args.add_arg('enable_operation', True, cli=True)
    args.add_arg(
        'operation_param',
        lu.get_path('jetpilot_system_launch', 'config/operation/operation.param.yaml'),
        cli=True)

    args.add_arg('enable_control', False, cli=True)
    args.add_arg(
        'control_param',
        lu.get_path('jetpilot_controller', 'config/controller.param.yaml'),
        cli=True)
    args.add_arg('controller_diagnostics_topic', '/controller/diagnostics', cli=True)

    args.add_arg('enable_planning', False, cli=True)
    args.add_arg(
        'planning_param',
        lu.get_path('jetpilot_planning', 'config/route_lane_selector.param.yaml'),
        cli=True)
    args.add_arg('enable_raceline_publisher', False, cli=True)
    args.add_arg(
        'raceline_config_file',
        lu.get_path('jetpilot_planning', 'config/raceline_path_publisher.param.yaml'),
        cli=True)
    args.add_arg('raceline_root', '', cli=True)
    args.add_arg('raceline_csv', '', cli=True)
    args.add_arg('raceline_path_topic', '/planning/raceline_path', cli=True)
    args.add_arg('planning_diagnostics_topic', '/planning/diagnostics', cli=True)

    args.add_arg('enable_sensor_kit', False, cli=True)
    args.add_arg('sensor_kit_interface_pkg', 'jetpilot_system_launch', cli=True)
    args.add_arg('sensor_kit_interface_launch', 'launch/sensors/realsense.launch.py', cli=True)
    args.add_arg('sensor_kit_camera_name', 'realsense', cli=True)
    args.add_arg('sensor_kit_container_name', 'sensor_kit_container', cli=True)
    args.add_arg('sensor_kit_enable_depth', False, cli=True)
    args.add_arg('sensor_kit_enable_color', False, cli=True)
    args.add_arg('sensor_kit_enable_rtp_stream', False, cli=True)
    args.add_arg('sensor_kit_rtp_image_topic', '/realsense/color/image_raw', cli=True)
    args.add_arg('sensor_kit_rtp_host', '', cli=True)
    args.add_arg('sensor_kit_rtp_port', '5004', cli=True)
    args.add_arg('sensor_kit_rtp_codec', 'h264', cli=True)
    args.add_arg('sensor_kit_rtp_fps', '60', cli=True)
    args.add_arg('sensor_kit_rtp_bitrate', '4000000', cli=True)
    args.add_arg('sensor_kit_rtp_gop', '60', cli=True)
    args.add_arg('sensor_kit_rtp_mtu', '1200', cli=True)
    args.add_arg('sensor_kit_rtp_payload', '96', cli=True)
    args.add_arg('sensor_kit_rtp_encoder', 'auto', cli=True)
    args.add_arg('sensor_kit_rtp_enable_status_log', False, cli=True)
    args.add_arg('sensor_kit_enable_flir', True, cli=True)
    args.add_arg('sensor_kit_flir_namespace', 'flir', cli=True)
    args.add_arg('sensor_kit_flir_node_name', 'boson', cli=True)
    args.add_arg('sensor_kit_flir_camera_name', 'boson', cli=True)
    args.add_arg('sensor_kit_flir_frame_id', 'boson_optical_frame', cli=True)
    args.add_arg('sensor_kit_flir_video_device', '/dev/video0', cli=True)
    args.add_arg('sensor_kit_flir_pixel_format', 'mono16', cli=True)
    args.add_arg('sensor_kit_flir_image_width', '640', cli=True)
    args.add_arg('sensor_kit_flir_image_height', '512', cli=True)
    args.add_arg('sensor_kit_flir_framerate', '60.0', cli=True)
    args.add_arg('sensor_kit_flir_io_method', 'mmap', cli=True)
    args.add_arg('sensor_kit_silky_evcam_raw_recording_enabled', False, cli=True)
    args.add_arg(
        'sensor_kit_silky_evcam_raw_recording_request_topic',
        '/event_camera/raw_recording/request',
        cli=True)
    args.add_arg('sensor_kit_silky_evcam_raw_recording_auto_start', True, cli=True)
    args.add_arg(
        'sensor_kit_silky_evcam_raw_recording_dir',
        '/workspaces/record/openeb_raw',
        cli=True)
    args.add_arg('sensor_kit_silky_evcam_raw_recording_basename', 'openeb', cli=True)
    args.add_arg(
        'sensor_kit_silky_evcam_raw_recording_split_duration_s', '0.0', cli=True)

    args.add_arg('enable_localization', False, cli=True)
    args.add_arg('localization_camera_name', 'realsense', cli=True)
    args.add_arg('localization_container_name', 'nova_container', cli=True)
    args.add_arg('localization_run_standalone', True, cli=True)
    args.add_arg('map_dir', '', cli=True)
    args.add_arg('localization_base_frame', 'base_link', cli=True)
    args.add_arg('enable_vslam', True, cli=True)
    args.add_arg('vslam_enable_slam', True, cli=True)
    args.add_arg('vslam_enable_ground_constraint_in_odometry', False, cli=True)
    args.add_arg('vslam_enable_ground_constraint_in_slam', False, cli=True)
    args.add_arg('vslam_enable_visualization', False, cli=True)
    args.add_arg('vslam_localize_on_startup', False, cli=True)
    args.add_arg('vslam_hint_request_topic', '/visual_slam/trigger_hint', cli=True)
    args.add_arg('vslam_pose_hint_topic', '/localization/pose_hint', cli=True)
    args.add_arg('vslam_save_map_folder_path', '', cli=True)
    args.add_arg('manual_pose_topic', '/initialpose', cli=True)
    args.add_arg(
        'vgl_trigger_service', '/visual_localization/trigger_localization', cli=True)
    args.add_arg('vgl_pose_topic', '/visual_localization/pose', cli=True)
    args.add_arg('localization_trigger_topic', '/localization/trigger', cli=True)
    args.add_arg('localization_trigger_service', '/localization/relocalize', cli=True)
    args.add_arg('localization_diagnostics_topic', '/localization/diagnostics', cli=True)
    args.add_arg(
        'pose_hint_required_topic', '/localization/pose_hint_required', cli=True)
    args.add_arg('pose_hint_state_topic', '/localization/pose_hint_state', cli=True)
    args.add_arg('enable_localization_manager', True, cli=True)
    args.add_arg(
        'localization_manager_param',
        lu.get_path(
            'jetpilot_localization_manager',
            'config/localization_manager.param.yaml'),
        cli=True)
    args.add_arg('enable_vgl', True, cli=True)
    args.add_arg(
        'vgl_topic_config_file',
        lu.get_path('jetpilot_system_launch', 'config/localization/vgl_camera_topics.yaml'),
        cli=True)
    args.add_arg(
        'vgl_model_dir',
        '/workspaces/ros2_ws/isaac_ros_assets/models/visual_global_localization',
        cli=True)
    args.add_arg('vgl_image_qos_profile', 'SENSOR_DATA', cli=True)
    args.add_arg('enable_occupancy_map_server', False, cli=True)
    args.add_arg('enable_occupancy_map_lifecycle_manager', False, cli=True)
    args.add_arg('enable_omap_frame', False, cli=True)
    args.add_arg('occupancy_map_yaml_path', '', cli=True)
    args.add_arg('enable_hd_map_publisher', False, cli=True)
    args.add_arg('enable_section_localizer', False, cli=True)
    args.add_arg('hd_map_publisher_param', '', cli=True)
    args.add_arg('hd_map_yaml_path', '', cli=True)

    args.add_arg('enable_vehicle', True, cli=True)
    args.add_arg('vehicle_interface_pkg', 'pca9685_rc_driver', cli=True)
    args.add_arg('vehicle_interface_launch', 'launch/pca9685_rc_interface.launch.xml', cli=True)
    args.add_arg(
        'vehicle_driver_param',
        workspace_param_path(
            'pca9685_rc_driver_node.param.yaml',
            'pca9685_rc_driver',
            'config/pca9685_rc_driver_node.param.yaml'),
        cli=True)
    args.add_arg('vehicle_control_topic', '/vehicle/control_cmd', cli=True)
    args.add_arg('publish_vehicle_description', True, cli=True)
    args.add_arg('vehicle_description_base_frame', 'base_link', cli=True)
    args.add_arg('vehicle_description_camera_frame', 'realsense_camera_link', cli=True)
    args.add_arg('vehicle_description_camera_x', '0.2075', cli=True)
    args.add_arg('vehicle_description_camera_y', '0.019', cli=True)
    args.add_arg('vehicle_description_camera_z', '0.065', cli=True)
    args.add_arg('vehicle_description_camera_roll', '0.0', cli=True)
    args.add_arg('vehicle_description_camera_pitch', '0.0', cli=True)
    args.add_arg('vehicle_description_camera_yaw', '0.0', cli=True)

    args.add_arg('enable_vslam_snapshot', False, cli=True)
    args.add_arg('vslam_snapshot_output', '/tmp/vslam_reference_snapshot.json', cli=True)
    args.add_arg('vslam_snapshot_path_topic', '/visual_slam/tracking/slam_path', cli=True)
    args.add_arg('vslam_snapshot_odom_topic', '/visual_slam/tracking/odometry', cli=True)
    args.add_arg('vslam_snapshot_landmarks_topic', '/visual_slam/vis/landmarks_cloud', cli=True)
    args.add_arg(
        'vslam_snapshot_localization_state_topic',
        '/localization/pose_hint_state',
        cli=True)
    args.add_arg('vslam_snapshot_tf_topic', '/tf', cli=True)
    args.add_arg('vslam_snapshot_map_frame', 'map', cli=True)
    args.add_arg('vslam_snapshot_require_localized_map', False, cli=True)
    args.add_arg('vslam_snapshot_write_interval_s', '5.0', cli=True)

    args.add_arg('enable_rviz', False, cli=True)
    args.add_arg(
        'rviz_config_file',
        lu.get_path('jetpilot_system_launch', 'rviz/default.rviz'),
        cli=True)

    actions = args.get_launch_actions()
    actions.append(OpaqueFunction(function=_validate_replay_vehicle_safety))

    rosbag_replay_enabled = lut.AndSubstitution(
        _LaunchBoolean(args.enable_rosbag_replay),
        lu.is_valid(args.rosbag),
    )
    safe_replay_enabled = lut.AndSubstitution(
        rosbag_replay_enabled,
        lut.NotSubstitution(_LaunchBoolean(args.allow_unsafe_replay_control_topics)),
    )
    actuation_nodes_allowed = lut.NotSubstitution(safe_replay_enabled)
    vehicle_interface_enabled = lut.AndSubstitution(
        lu.is_true(args.enable_vehicle), actuation_nodes_allowed)
    vehicle_launch_enabled = OrSubstitution(
        vehicle_interface_enabled, lu.is_true(args.publish_vehicle_description))
    actions.append(
        lu.play_rosbag(
            args.rosbag,
            rate=args.replay_rate,
            delay=args.rosbag_start_delay_s,
            additional_bag_play_args=_ReplayArguments(
                args.replay_additional_args,
                args.allow_unsafe_replay_control_topics,
                (
                    args.vehicle_control_topic,
                    args.rc_channels_topic,
                    args.propo_control_topic,
                    args.vslam_hint_request_topic,
                    args.vslam_pose_hint_topic,
                    args.manual_pose_topic,
                    args.vgl_pose_topic,
                    args.localization_trigger_topic,
                    args.localization_diagnostics_topic,
                    args.planning_diagnostics_topic,
                    args.controller_diagnostics_topic,
                    args.jetson_stats_diagnostics_topic,
                    args.pose_hint_required_topic,
                    args.pose_hint_state_topic,
                ),
            ),
            shutdown_on_exit=args.rosbag_shutdown_on_exit,
            condition=IfCondition(rosbag_replay_enabled),
        ))
    actions.append(
        lu.log_info(
            'Safe replay: recorded control/mode topics are isolated under /replay and '
            'live joy, teleop, RC, operation, and autonomous control nodes are disabled.',
            condition=IfCondition(safe_replay_enabled),
        ))
    actions.append(
        lu.log_info(
            'A rosbag path was provided, but enable_rosbag_replay is false; replay is disabled.',
            condition=IfCondition(lut.AndSubstitution(
                lut.NotSubstitution(_LaunchBoolean(args.enable_rosbag_replay)),
                lu.is_valid(args.rosbag),
            )),
        ))

    actions.append(
        lu.include(
            'jetpilot_planning',
            'launch/jetpilot_planning.launch.xml',
            launch_arguments={
                'config_file': args.planning_param,
                'enable_raceline_publisher': args.enable_raceline_publisher,
                'raceline_config_file': args.raceline_config_file,
                'raceline_root': args.raceline_root,
                'raceline_csv': args.raceline_csv,
                'raceline_path_topic': args.raceline_path_topic,
                'diagnostics_topic': args.planning_diagnostics_topic,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(args.enable_planning),
        ))

    actions.append(
        lu.include(
            'jetpilot_system_launch',
            'launch/tool.launch.py',
            launch_arguments={
                'bag_manager_param': args.bag_manager_param,
                'teleop_cmd_param': args.teleop_cmd_param,
                'teleop_button_mapping_param': args.teleop_button_mapping_param,
                'serial_reader_param': args.serial_reader_param,
                'enable_bag_manager': args.enable_bag_manager,
                'enable_joy': lut.AndSubstitution(
                    lu.is_true(args.enable_joy), actuation_nodes_allowed),
                'enable_teleop': lut.AndSubstitution(
                    lu.is_true(args.enable_teleop), actuation_nodes_allowed),
                'enable_rc_serial': lut.AndSubstitution(
                    lu.is_true(args.enable_rc_serial), actuation_nodes_allowed),
                'enable_jetson_stats': args.enable_jetson_stats,
                'jetson_stats_diagnostics_topic': args.jetson_stats_diagnostics_topic,
                'jetson_stats_interval': args.jetson_stats_interval,
                'control_authority': args.control_authority,
                'rc_channels_topic': args.rc_channels_topic,
                'propo_control_topic': args.propo_control_topic,
                'localization_trigger_topic': args.localization_trigger_topic,
                'joy_autorepeat_rate': args.joy_autorepeat_rate,
                'joy_deadzone': args.joy_deadzone,
                'joy_device_path': args.joy_device_path,
                'joy_prefer_evdev': args.joy_prefer_evdev,
                'enable_vslam_snapshot': args.enable_vslam_snapshot,
                'vslam_snapshot_output': args.vslam_snapshot_output,
                'vslam_snapshot_path_topic': args.vslam_snapshot_path_topic,
                'vslam_snapshot_odom_topic': args.vslam_snapshot_odom_topic,
                'vslam_snapshot_landmarks_topic':
                    args.vslam_snapshot_landmarks_topic,
                'vslam_snapshot_localization_state_topic':
                    args.vslam_snapshot_localization_state_topic,
                'vslam_snapshot_tf_topic': args.vslam_snapshot_tf_topic,
                'vslam_snapshot_map_frame': args.vslam_snapshot_map_frame,
                'vslam_snapshot_require_localized_map':
                    args.vslam_snapshot_require_localized_map,
                'vslam_snapshot_write_interval_s':
                    args.vslam_snapshot_write_interval_s,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(args.enable_tool),
        ))

    actions.append(
        lu.include(
            'jetpilot_operation',
            'launch/jetpilot_operation.launch.xml',
            launch_arguments={
                'config_file': args.operation_param,
                'control_authority': args.control_authority,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(lut.AndSubstitution(
                lu.is_true(args.enable_operation), actuation_nodes_allowed)),
        ))

    actions.append(
        lu.include(
            'jetpilot_system_launch',
            'launch/control.launch.py',
            launch_arguments={
                'control_param': args.control_param,
                'diagnostics_topic': args.controller_diagnostics_topic,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(lut.AndSubstitution(
                lu.is_true(args.enable_control), actuation_nodes_allowed)),
        ))

    actions.append(
        lu.include(
            'jetpilot_system_launch',
            'launch/sensor_kit.launch.py',
            launch_arguments={
                'sensor_interface_pkg': args.sensor_kit_interface_pkg,
                'sensor_interface_launch': args.sensor_kit_interface_launch,
                'camera_name': args.sensor_kit_camera_name,
                'container_name': args.sensor_kit_container_name,
                'enable_depth': args.sensor_kit_enable_depth,
                'enable_color': args.sensor_kit_enable_color,
                'enable_rtp_stream': args.sensor_kit_enable_rtp_stream,
                'rtp_image_topic': args.sensor_kit_rtp_image_topic,
                'rtp_host': args.sensor_kit_rtp_host,
                'rtp_port': args.sensor_kit_rtp_port,
                'rtp_codec': args.sensor_kit_rtp_codec,
                'rtp_fps': args.sensor_kit_rtp_fps,
                'rtp_bitrate': args.sensor_kit_rtp_bitrate,
                'rtp_gop': args.sensor_kit_rtp_gop,
                'rtp_mtu': args.sensor_kit_rtp_mtu,
                'rtp_payload': args.sensor_kit_rtp_payload,
                'rtp_encoder': args.sensor_kit_rtp_encoder,
                'rtp_enable_status_log': args.sensor_kit_rtp_enable_status_log,
                'enable_flir': args.sensor_kit_enable_flir,
                'flir_namespace': args.sensor_kit_flir_namespace,
                'flir_node_name': args.sensor_kit_flir_node_name,
                'flir_camera_name': args.sensor_kit_flir_camera_name,
                'flir_frame_id': args.sensor_kit_flir_frame_id,
                'flir_video_device': args.sensor_kit_flir_video_device,
                'flir_pixel_format': args.sensor_kit_flir_pixel_format,
                'flir_image_width': args.sensor_kit_flir_image_width,
                'flir_image_height': args.sensor_kit_flir_image_height,
                'flir_framerate': args.sensor_kit_flir_framerate,
                'flir_io_method': args.sensor_kit_flir_io_method,
                'silky_evcam_raw_recording_enabled':
                    args.sensor_kit_silky_evcam_raw_recording_enabled,
                'silky_evcam_raw_recording_request_topic':
                    args.sensor_kit_silky_evcam_raw_recording_request_topic,
                'silky_evcam_raw_recording_auto_start':
                    args.sensor_kit_silky_evcam_raw_recording_auto_start,
                'silky_evcam_raw_recording_dir':
                    args.sensor_kit_silky_evcam_raw_recording_dir,
                'silky_evcam_raw_recording_basename':
                    args.sensor_kit_silky_evcam_raw_recording_basename,
                'silky_evcam_raw_recording_split_duration_s':
                    args.sensor_kit_silky_evcam_raw_recording_split_duration_s,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(args.enable_sensor_kit),
        ))

    actions.append(
        lu.include(
            'jetpilot_system_launch',
            'launch/localization.launch.py',
            launch_arguments={
                'camera_name': args.localization_camera_name,
                'container_name': args.localization_container_name,
                'run_standalone': args.localization_run_standalone,
                'map_dir': args.map_dir,
                'localization_base_frame': args.localization_base_frame,
                'enable_vslam': args.enable_vslam,
                'vslam_enable_slam': args.vslam_enable_slam,
                'vslam_enable_ground_constraint_in_odometry':
                    args.vslam_enable_ground_constraint_in_odometry,
                'vslam_enable_ground_constraint_in_slam':
                    args.vslam_enable_ground_constraint_in_slam,
                'vslam_enable_visualization': args.vslam_enable_visualization,
                'vslam_localize_on_startup': args.vslam_localize_on_startup,
                'vslam_hint_request_topic': args.vslam_hint_request_topic,
                'vslam_pose_hint_topic': args.vslam_pose_hint_topic,
                'vslam_save_map_folder_path': args.vslam_save_map_folder_path,
                'manual_pose_topic': args.manual_pose_topic,
                'vgl_trigger_service': args.vgl_trigger_service,
                'vgl_pose_topic': args.vgl_pose_topic,
                'localization_trigger_topic': args.localization_trigger_topic,
                'localization_trigger_service': args.localization_trigger_service,
                'localization_diagnostics_topic': args.localization_diagnostics_topic,
                'pose_hint_required_topic': args.pose_hint_required_topic,
                'pose_hint_state_topic': args.pose_hint_state_topic,
                'enable_localization_manager': args.enable_localization_manager,
                'localization_manager_param': args.localization_manager_param,
                'enable_vgl': args.enable_vgl,
                'vgl_topic_config_file': args.vgl_topic_config_file,
                'vgl_model_dir': args.vgl_model_dir,
                'vgl_image_qos_profile': args.vgl_image_qos_profile,
                'enable_occupancy_map_server': args.enable_occupancy_map_server,
                'enable_occupancy_map_lifecycle_manager':
                    args.enable_occupancy_map_lifecycle_manager,
                'enable_omap_frame': args.enable_omap_frame,
                'occupancy_map_yaml_path': args.occupancy_map_yaml_path,
                'enable_hd_map_publisher': args.enable_hd_map_publisher,
                'enable_section_localizer': args.enable_section_localizer,
                'hd_map_publisher_param': args.hd_map_publisher_param,
                'hd_map_yaml_path': args.hd_map_yaml_path,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(args.enable_localization),
        ))

    actions.append(
        lu.include(
            'jetpilot_system_launch',
            'launch/vehicle.launch.py',
            launch_arguments={
                'vehicle_interface_pkg': args.vehicle_interface_pkg,
                'vehicle_interface_launch': args.vehicle_interface_launch,
                'enable_vehicle_interface': vehicle_interface_enabled,
                'vehicle_control_topic': args.vehicle_control_topic,
                'vehicle_driver_param': args.vehicle_driver_param,
                'publish_vehicle_description': args.publish_vehicle_description,
                'vehicle_description_base_frame': args.vehicle_description_base_frame,
                'vehicle_description_camera_frame':
                    args.vehicle_description_camera_frame,
                'vehicle_description_camera_x': args.vehicle_description_camera_x,
                'vehicle_description_camera_y': args.vehicle_description_camera_y,
                'vehicle_description_camera_z': args.vehicle_description_camera_z,
                'vehicle_description_camera_roll':
                    args.vehicle_description_camera_roll,
                'vehicle_description_camera_pitch':
                    args.vehicle_description_camera_pitch,
                'vehicle_description_camera_yaw': args.vehicle_description_camera_yaw,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(vehicle_launch_enabled),
        ))

    actions.append(
        lu.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', args.rviz_config_file],
            parameters=[{
                'use_sim_time': lut.ParameterValue(args.use_sim_time, value_type=bool),
            }],
            condition=IfCondition(args.enable_rviz),
        ))

    return lut.LaunchDescription(actions)
