# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from launch_ros.actions import ComposableNodeContainer


def launch_flir_boson(args: lu.ArgumentContainer) -> list[lut.Action]:
    if not lu.is_true(args.enable_flir):
        return []

    composable_nodes = [
        lut.ComposableNode(
            package='usb_cam',
            plugin='usb_cam::UsbCamNode',
            name=args.flir_node_name,
            namespace=args.flir_namespace,
            parameters=[{
                'video_device': args.flir_video_device,
                'io_method': args.flir_io_method,
                'pixel_format': args.flir_pixel_format,
                'image_width': lut.ParameterValue(
                    args.flir_image_width, value_type=int),
                'image_height': lut.ParameterValue(
                    args.flir_image_height, value_type=int),
                'framerate': lut.ParameterValue(
                    args.flir_framerate, value_type=float),
                'camera_name': args.flir_camera_name,
                'frame_id': args.flir_frame_id,
                'use_sim_time': lu.is_true(args.use_sim_time),
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    ]

    actions = [
        lu.log_info([
            'FLIR Boson sensor enabled: ',
            args.flir_video_device,
            ', namespace: ',
            args.flir_namespace,
            ', frame: ',
            args.flir_frame_id,
        ]),
    ]

    if lu.is_true(args.run_standalone):
        actions.append(ComposableNodeContainer(
            name=args.container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[],
            output='screen',
        ))

    actions.append(lu.load_composable_nodes(args.container_name, composable_nodes))
    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg('container_name', 'sensor_kit_container')
    args.add_arg('run_standalone', True)
    args.add_arg('camera_name', 'realsense')
    args.add_arg('enable_depth', False)
    args.add_arg('enable_color', False)
    args.add_arg('enable_rtp_stream', False)
    args.add_arg('rtp_image_topic', '/flir/image_raw')
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
    args.add_opaque_function(launch_flir_boson)

    return lut.LaunchDescription(args.get_launch_actions())
