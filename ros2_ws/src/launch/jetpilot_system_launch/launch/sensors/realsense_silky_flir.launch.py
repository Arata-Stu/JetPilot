# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def launch_sensor_kit(args: lu.ArgumentContainer) -> list[lut.Action]:
    actions = []

    if lu.is_true(args.enable_realsense):
        actions.extend([
            lu.include(
                'jetpilot_system_launch',
                'launch/sensors/realsense.launch.py',
                launch_arguments={
                    'camera_name': args.camera_name,
                    'container_name': args.container_name,
                    'run_standalone': str(lu.is_true(args.run_standalone)).lower(),
                    'enable_depth': str(lu.is_true(args.enable_depth)).lower(),
                    'enable_color': str(lu.is_true(args.enable_color)).lower(),
                    'enable_rtp_stream': str(lu.is_true(args.enable_rtp_stream)).lower(),
                    'rtp_image_topic': args.rtp_image_topic,
                    'rtp_host': args.rtp_host,
                    'rtp_port': args.rtp_port,
                    'rtp_codec': args.rtp_codec,
                    'rtp_fps': args.rtp_fps,
                    'rtp_bitrate': args.rtp_bitrate,
                    'rtp_gop': args.rtp_gop,
                    'rtp_mtu': args.rtp_mtu,
                    'rtp_payload': args.rtp_payload,
                    'rtp_encoder': args.rtp_encoder,
                    'rtp_enable_status_log': str(
                        lu.is_true(args.rtp_enable_status_log)).lower(),
                    'use_sim_time': str(lu.is_true(args.use_sim_time)).lower(),
                },
            ),
            lu.log_info([
                'RealSense sensor enabled: ',
                args.camera_name,
            ]),
        ])

    if lu.is_true(args.enable_silky_evcam):
        actions.extend([
            lu.include(
                'openeb_ros2',
                args.silky_evcam_launch,
                launch_arguments={
                    'namespace': args.silky_evcam_namespace,
                    'serial': args.silky_evcam_serial,
                    'device_format': args.silky_evcam_device_format,
                    'frame_id': args.silky_evcam_frame_id,
                    'raw_recording_enabled': str(
                        lu.is_true(args.silky_evcam_raw_recording_enabled)).lower(),
                    'raw_recording_request_topic':
                        args.silky_evcam_raw_recording_request_topic,
                    'raw_recording_auto_start': str(
                        lu.is_true(args.silky_evcam_raw_recording_auto_start)).lower(),
                    'raw_recording_dir': args.silky_evcam_raw_recording_dir,
                    'raw_recording_basename': args.silky_evcam_raw_recording_basename,
                    'raw_recording_split_duration_s':
                        args.silky_evcam_raw_recording_split_duration_s,
                    'packet_duration_us': args.silky_evcam_packet_duration_us,
                    'statistics_interval_s': args.silky_evcam_statistics_interval_s,
                    'debug': str(lu.is_true(args.silky_evcam_debug)).lower(),
                    'event_image_enabled': str(
                        lu.is_true(args.silky_evcam_event_image_enabled)).lower(),
                    'event_image_fps': args.silky_evcam_event_image_fps,
                    'event_image_encoding': args.silky_evcam_event_image_encoding,
                    'event_image_publish_empty': str(
                        lu.is_true(args.silky_evcam_event_image_publish_empty)).lower(),
                    'event_image_publisher_depth': args.silky_evcam_event_image_publisher_depth,
                },
            ),
            lu.log_info([
                'SilkyEvCam/OpenEB sensor enabled: ',
                args.silky_evcam_namespace,
                ', frame: ',
                args.silky_evcam_frame_id,
            ]),
        ])

    if lu.is_true(args.enable_flir):
        flir_run_standalone = (
            lu.is_true(args.run_standalone) and not lu.is_true(args.enable_realsense)
        )
        actions.extend([
            lu.include(
                'jetpilot_system_launch',
                'launch/sensors/flir_boson.launch.py',
                launch_arguments={
                    'container_name': args.container_name,
                    'run_standalone': str(flir_run_standalone).lower(),
                    'flir_namespace': args.flir_namespace,
                    'flir_node_name': args.flir_node_name,
                    'flir_camera_name': args.flir_camera_name,
                    'flir_frame_id': args.flir_frame_id,
                    'flir_video_device': args.flir_video_device,
                    'flir_pixel_format': args.flir_pixel_format,
                    'flir_image_width': args.flir_image_width,
                    'flir_image_height': args.flir_image_height,
                    'flir_framerate': args.flir_framerate,
                    'flir_io_method': args.flir_io_method,
                    'use_sim_time': str(lu.is_true(args.use_sim_time)).lower(),
                },
            ),
        ])

    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg('enable_realsense', True)
    args.add_arg('camera_name', 'realsense')
    args.add_arg('container_name', 'multi_sensor_container')
    args.add_arg('run_standalone', True)
    args.add_arg('enable_depth', False)
    args.add_arg('enable_color', True)
    args.add_arg('enable_rtp_stream', False)
    args.add_arg('rtp_image_topic', '/realsense/color/image_raw')
    args.add_arg('rtp_host', '')
    args.add_arg('rtp_port', '5004')
    args.add_arg('rtp_codec', 'h264')
    args.add_arg('rtp_fps', '60')
    args.add_arg('rtp_bitrate', '4000000')
    args.add_arg('rtp_gop', '60')
    args.add_arg('rtp_mtu', '1200')
    args.add_arg('rtp_payload', '96')
    args.add_arg('rtp_encoder', 'auto')
    args.add_arg('rtp_enable_status_log', False)

    args.add_arg('enable_silky_evcam', True)
    args.add_arg('silky_evcam_launch', 'launch/pipeline.launch.py')
    args.add_arg('silky_evcam_namespace', 'event_camera')
    args.add_arg('silky_evcam_serial', '')
    args.add_arg('silky_evcam_device_format', '')
    args.add_arg('silky_evcam_frame_id', 'event_camera')
    args.add_arg('silky_evcam_raw_recording_enabled', False)
    args.add_arg(
        'silky_evcam_raw_recording_request_topic', '/event_camera/raw_recording/request')
    args.add_arg('silky_evcam_raw_recording_auto_start', True)
    args.add_arg('silky_evcam_raw_recording_dir', '/workspaces/record/openeb_raw')
    args.add_arg('silky_evcam_raw_recording_basename', 'openeb')
    args.add_arg('silky_evcam_raw_recording_split_duration_s', '0.0')
    args.add_arg('silky_evcam_packet_duration_us', '1000')
    args.add_arg('silky_evcam_statistics_interval_s', '1.0')
    args.add_arg('silky_evcam_debug', False)
    args.add_arg('silky_evcam_event_image_enabled', True)
    args.add_arg('silky_evcam_event_image_fps', '25.0')
    args.add_arg('silky_evcam_event_image_encoding', 'bgr8')
    args.add_arg('silky_evcam_event_image_publish_empty', True)
    args.add_arg('silky_evcam_event_image_publisher_depth', '2')

    args.add_arg('enable_flir', True)
    args.add_arg('flir_namespace', 'flir')
    args.add_arg('flir_node_name', 'boson')
    args.add_arg('flir_camera_name', 'boson')
    args.add_arg('flir_frame_id', 'boson_optical_frame')
    args.add_arg('flir_video_device', '/dev/video0')
    args.add_arg('flir_pixel_format', 'mono16')
    args.add_arg('flir_image_width', '640')
    args.add_arg('flir_image_height', '512')
    args.add_arg('flir_framerate', '60.0')
    args.add_arg('flir_io_method', 'mmap')

    args.add_arg('use_sim_time', False)
    args.add_opaque_function(launch_sensor_kit)

    return lut.LaunchDescription(args.get_launch_actions())
