# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut

def add_sensor_interface(args: lu.ArgumentContainer):
    if not (
        lu.is_true(args.enable_sensor_interface) and
        lu.is_true(args.enable_realsense)
    ):
        return []

    launch_arguments = {
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
        'enable_flir': str(lu.is_true(args.enable_flir)).lower(),
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
        'silky_evcam_raw_recording_enabled': str(
            lu.is_true(args.silky_evcam_raw_recording_enabled)).lower(),
        'silky_evcam_raw_recording_request_topic':
            args.silky_evcam_raw_recording_request_topic,
        'silky_evcam_raw_recording_auto_start': str(
            lu.is_true(args.silky_evcam_raw_recording_auto_start)).lower(),
        'silky_evcam_raw_recording_dir': args.silky_evcam_raw_recording_dir,
        'silky_evcam_raw_recording_basename': args.silky_evcam_raw_recording_basename,
        'use_sim_time': str(lu.is_true(args.use_sim_time)).lower(),
    }

    return [
        lu.include(
            args.sensor_interface_pkg,
            args.sensor_interface_launch,
            launch_arguments=launch_arguments,
        ),
        lu.log_info([
            'Sensor interface: ',
            args.sensor_interface_pkg,
            '/',
            args.sensor_interface_launch,
            ', camera: ',
            args.camera_name,
        ]),
    ]


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg('enable_sensor_interface', True, cli=True)
    args.add_arg('sensor_interface_pkg', 'jetpilot_system_launch', cli=True)
    args.add_arg('sensor_interface_launch', 'launch/sensors/realsense.launch.py', cli=True)
    args.add_arg('enable_realsense', True, cli=True)
    args.add_arg('camera_name', 'realsense', cli=True)
    args.add_arg('container_name', 'multi_sensor_container', cli=True)
    args.add_arg('run_standalone', True, cli=True)
    args.add_arg('enable_depth', False, cli=True)
    args.add_arg('enable_color', True, cli=True)
    args.add_arg('enable_rtp_stream', False, cli=True)
    args.add_arg('rtp_image_topic', '/realsense/color/image_raw', cli=True)
    args.add_arg('rtp_host', '', cli=True)
    args.add_arg('rtp_port', '5004', cli=True)
    args.add_arg('rtp_codec', 'h264', cli=True)
    args.add_arg('rtp_fps', '60', cli=True)
    args.add_arg('rtp_bitrate', '4000000', cli=True)
    args.add_arg('rtp_gop', '60', cli=True)
    args.add_arg('rtp_mtu', '1200', cli=True)
    args.add_arg('rtp_payload', '96', cli=True)
    args.add_arg('rtp_encoder', 'auto', cli=True)
    args.add_arg('rtp_enable_status_log', False, cli=True)
    args.add_arg('enable_flir', True, cli=True)
    args.add_arg('flir_namespace', 'flir', cli=True)
    args.add_arg('flir_node_name', 'boson', cli=True)
    args.add_arg('flir_camera_name', 'boson', cli=True)
    args.add_arg('flir_frame_id', 'boson_optical_frame', cli=True)
    args.add_arg('flir_video_device', '/dev/video0', cli=True)
    args.add_arg('flir_pixel_format', 'mono16', cli=True)
    args.add_arg('flir_image_width', '640', cli=True)
    args.add_arg('flir_image_height', '512', cli=True)
    args.add_arg('flir_framerate', '60.0', cli=True)
    args.add_arg('flir_io_method', 'mmap', cli=True)
    args.add_arg('silky_evcam_raw_recording_enabled', True, cli=True)
    args.add_arg(
        'silky_evcam_raw_recording_request_topic',
        '/event_camera/raw_recording/request',
        cli=True)
    args.add_arg('silky_evcam_raw_recording_auto_start', False, cli=True)
    args.add_arg(
        'silky_evcam_raw_recording_dir', '/workspaces/record/openeb_raw', cli=True)
    args.add_arg('silky_evcam_raw_recording_basename', 'openeb', cli=True)
    args.add_arg('use_sim_time', False, cli=True)

    args.add_opaque_function(add_sensor_interface)

    return lut.LaunchDescription(args.get_launch_actions())
