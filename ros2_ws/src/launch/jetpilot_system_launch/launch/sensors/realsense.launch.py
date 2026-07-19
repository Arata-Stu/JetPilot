# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

import isaac_ros_launch_utils.all_types as lut
import isaac_ros_launch_utils as lu

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer


def launch_realsense(args: lu.ArgumentContainer) -> list[lut.Action]:

    actions = []

    # Prepare parameters
    parameters = []

    config_yaml = os.path.join(
        get_package_share_directory('jetpilot_system_launch'),
        'config/sensing',
        'realsense.param.yaml'
    )

    parameters.append(config_yaml)
    parameters.append({
        'use_sim_time': lu.is_true(args.use_sim_time),
        'enable_infra1': True,
        'enable_infra2': True,
        'enable_depth': lu.is_true(args.enable_depth),
        'enable_color': lu.is_true(args.enable_color) or lu.is_true(args.enable_rtp_stream),
        'enable_rgbd': False,
        'enable_accel': False,
        'enable_gyro': False,
        'enable_sync': False,
        'align_depth.enable': False,
        'colorizer.enable': False,
        'decimation_filter.enable': False,
        'disparity_filter.enable': False,
        'disparity_to_depth.enable': False,
        'filter_by_sequence_id.enable': False,
        'hdr_merge.enable': False,
        'hole_filling_filter.enable': False,
        'pointcloud.enable': False,
        'spatial_filter.enable': False,
        'temporal_filter.enable': False,
    })

    realsense_node = lut.ComposableNode(
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        name=args.camera_name,
        namespace='',
        parameters=parameters,
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    composable_nodes = [realsense_node]

    if lu.is_true(args.enable_rtp_stream):
        rtp_config_yaml = os.path.join(
            get_package_share_directory('jetpilot_system_launch'),
            'config/sensing',
            'image_rtp_sender.param.yaml'
        )
        rtp_sender_node = lut.ComposableNode(
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
                    'bitrate': lut.ParameterValue(args.rtp_bitrate, value_type=int),
                    'gop': lut.ParameterValue(args.rtp_gop, value_type=int),
                    'mtu': lut.ParameterValue(args.rtp_mtu, value_type=int),
                    'payload': lut.ParameterValue(args.rtp_payload, value_type=int),
                    'encoder': args.rtp_encoder,
                    'enable_status_log': lu.is_true(args.rtp_enable_status_log),
                    'use_sim_time': lu.is_true(args.use_sim_time),
                },
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
        composable_nodes.append(rtp_sender_node)

    actions.append(
        lu.log_info(
            f'Using RealSense camera: {args.camera_name} with default parameters'
        )
    )

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
    args.add_arg('container_name', 'nova_container')
    args.add_arg('run_standalone', False)
    args.add_arg('camera_name', 'realsense')
    args.add_arg('enable_depth', False)
    args.add_arg('enable_color', False)
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
    args.add_opaque_function(launch_realsense)

    return lut.LaunchDescription(args.get_launch_actions())
