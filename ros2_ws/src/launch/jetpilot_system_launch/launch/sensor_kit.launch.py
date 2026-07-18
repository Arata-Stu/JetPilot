# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut

def add_sensor_interface(args: lu.ArgumentContainer):
    if not (
        lu.is_true(args.enable_sensor_interface) and
        lu.is_true(args.enable_realsense)
    ):
        return []

    return [
        lu.include(
            args.sensor_interface_pkg,
            args.sensor_interface_launch,
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
                'use_sim_time': str(lu.is_true(args.use_sim_time)).lower(),
            },
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
    args.add_arg('container_name', 'sensor_kit_container', cli=True)
    args.add_arg('run_standalone', True, cli=True)
    args.add_arg('enable_depth', False, cli=True)
    args.add_arg('enable_color', False, cli=True)
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
    args.add_arg('use_sim_time', False, cli=True)

    args.add_opaque_function(add_sensor_interface)

    return lut.LaunchDescription(args.get_launch_actions())
