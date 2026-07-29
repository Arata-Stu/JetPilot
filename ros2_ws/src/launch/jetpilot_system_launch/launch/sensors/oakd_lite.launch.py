# SPDX-License-Identifier: Apache-2.0

import os

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer


def launch_oakd_lite(args: lu.ArgumentContainer) -> list[lut.Action]:
    config_yaml = os.path.join(
        get_package_share_directory('jetpilot_system_launch'),
        'config/sensing',
        'oakd_lite.param.yaml',
    )

    oakd_lite_node = lut.ComposableNode(
        package='depthai_ros_driver_v3',
        plugin='depthai_ros_driver::Driver',
        name=args.camera_name,
        namespace='',
        parameters=[
            config_yaml,
            {
                'use_sim_time': lu.is_true(args.use_sim_time),
                'rgb.i_publish_topic': lu.is_true(args.enable_color),
                'stereo.i_publish_topic': lu.is_true(args.enable_depth),
                'driver.i_tf_device_name': args.camera_name,
                'driver.i_tf_base_frame': args.camera_name,
            },
        ],
        remappings=[
            ('/diagnostics', f'/{args.camera_name}/diagnostics'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    composable_nodes = [oakd_lite_node]

    if lu.is_true(args.enable_rtp_stream):
        rtp_config_yaml = os.path.join(
            get_package_share_directory('jetpilot_system_launch'),
            'config/sensing',
            'image_rtp_sender.param.yaml',
        )
        composable_nodes.append(
            lut.ComposableNode(
                package='jetpilot_rtp_tools',
                plugin='jetpilot_rtp_tools::ImageRtpSenderComponent',
                name='image_rtp_sender',
                namespace='',
                parameters=[
                    rtp_config_yaml,
                    {
                        'image_topic': args.rtp_image_topic,
                        'host': args.rtp_host,
                        'port': lut.ParameterValue(args.rtp_port, value_type=int),
                        'codec': args.rtp_codec,
                        'fps': lut.ParameterValue(args.rtp_fps, value_type=int),
                        'bitrate': lut.ParameterValue(
                            args.rtp_bitrate, value_type=int),
                        'gop': lut.ParameterValue(args.rtp_gop, value_type=int),
                        'mtu': lut.ParameterValue(args.rtp_mtu, value_type=int),
                        'payload': lut.ParameterValue(
                            args.rtp_payload, value_type=int),
                        'encoder': args.rtp_encoder,
                        'enable_status_log': lu.is_true(
                            args.rtp_enable_status_log),
                        'use_sim_time': lu.is_true(args.use_sim_time),
                    },
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    actions: list[lut.Action] = [
        lu.log_info([
            'Using OAK-D Lite camera: ',
            args.camera_name,
            ' (RGB 640x400@35, rectified stereo 640x400@117, '
            'ROS depth disabled, IMU enabled)',
        ])
    ]

    if lu.is_true(args.run_standalone):
        actions.append(
            ComposableNodeContainer(
                name=args.container_name,
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[],
                output='screen',
            )
        )

    actions.append(
        lu.load_composable_nodes(args.container_name, composable_nodes)
    )
    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', 'multi_sensor_container')
    args.add_arg('run_standalone', False)
    args.add_arg('camera_name', 'oakd_lite')
    args.add_arg('enable_depth', False)
    args.add_arg('enable_color', True)
    args.add_arg('enable_rtp_stream', False)
    args.add_arg('rtp_image_topic', '/oakd_lite/rgb/image_raw')
    args.add_arg('rtp_host', '')
    args.add_arg('rtp_port', '5004')
    args.add_arg('rtp_codec', 'h264')
    args.add_arg('rtp_fps', '35')
    args.add_arg('rtp_bitrate', '4000000')
    args.add_arg('rtp_gop', '35')
    args.add_arg('rtp_mtu', '1200')
    args.add_arg('rtp_payload', '96')
    args.add_arg('rtp_encoder', 'auto')
    args.add_arg('rtp_enable_status_log', False)
    args.add_arg('enable_flir', False)
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
    args.add_opaque_function(launch_oakd_lite)

    return lut.LaunchDescription(args.get_launch_actions())
