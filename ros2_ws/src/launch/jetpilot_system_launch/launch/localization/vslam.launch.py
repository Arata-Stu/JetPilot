# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from ament_index_python.packages import get_package_share_directory
import isaac_ros_launch_utils.all_types as lut
import isaac_ros_launch_utils as lu
import yaml


def remap(i: int, name: str, identifier: str, rectified: bool) -> list[tuple[str, str]]:
    if rectified:
        return [
            (f'visual_slam/image_{i}', f'/{name}/{identifier}/image_rect'),
            (f'visual_slam/camera_info_{i}', f'/{name}/{identifier}/camera_info_rect'),
        ]
    else:
        return [
            (f'visual_slam/image_{i}', f'/{name}/{identifier}/image_raw'),
            (f'visual_slam/camera_info_{i}', f'/{name}/{identifier}/camera_info'),
        ]


def remap_from_config(topic_config_file: str) -> tuple[list[tuple[str, str]], int]:
    with open(topic_config_file, 'r', encoding='utf-8') as file:
        config = yaml.safe_load(file)

    stereo_cameras = config.get('stereo_cameras', [])
    remappings = []
    camera_id = 0
    for stereo_camera in stereo_cameras:
        for image_name in ('left', 'right'):
            camera_info_name = f'{image_name}_camera_info'
            if image_name not in stereo_camera or camera_info_name not in stereo_camera:
                raise ValueError(
                    f'Missing {image_name} or {camera_info_name} in {topic_config_file}')
            remappings.extend([
                (f'visual_slam/image_{camera_id}', stereo_camera[image_name]),
                (f'visual_slam/camera_info_{camera_id}',
                 stereo_camera[camera_info_name]),
            ])
            camera_id += 1

    if camera_id == 0:
        raise ValueError(f'No stereo cameras found in {topic_config_file}')
    return remappings, camera_id


def add_vslam(args: lu.ArgumentContainer) -> list[lut.Action]:
    camera_names = args.vslam_enabled_stereo_cameras.split(',')
    use_rectified_images = lu.is_true(args.vslam_use_rectified_images)

    remappings = [
        ('visual_slam/imu', '/front_stereo_imu/imu'),
        ('visual_slam/initial_pose', args.vslam_initial_pose_topic),
        ('visual_slam/trigger_hint', args.vslam_trigger_hint_topic),
        ('/diagnostics', args.vslam_diagnostics_topic),
    ]
    if args.vslam_topic_config_file:
        camera_remappings, num_cameras = remap_from_config(
            args.vslam_topic_config_file)
        remappings.extend(camera_remappings)
    else:
        num_cameras = 2 * len(camera_names)
        for i, camera_name in enumerate(camera_names):
            remappings.extend(
                remap(2 * i, camera_name, 'left', use_rectified_images))
            remappings.extend(
                remap(2 * i + 1, camera_name, 'right', use_rectified_images))

    actions = [
        lu.log_info([
            'VSLAM using remappings: ',
            ', '.join([f'{source} -> {target}' for source, target in remappings]),
        ]),
    ]

    camera_optical_frames = []
    if args.vslam_camera_optical_frames:
        camera_optical_frames = args.vslam_camera_optical_frames.split(',')

    # =========================================================================
    # 1. YAMLファイルのパスを取得
    # =========================================================================
    config_yaml = os.path.join(
        get_package_share_directory('jetpilot_system_launch'),
        'config/localization',
        'vslam.param.yaml'
    )

    # =========================================================================
    # 2. パラメータの重ね合わせリストを作成
    # =========================================================================
    parameters = []
    
    parameters.append(config_yaml)
    parameters.append({
        # スクリプト内で動的に計算されるパラメータ
        'num_cameras': num_cameras,
        'min_num_images': num_cameras,
        'camera_optical_frames': camera_optical_frames,
        'enable_localization_n_mapping': lut.ParameterValue(args.vslam_enable_slam, value_type=bool),
        'enable_ground_constraint_in_odometry': lut.ParameterValue(args.vslam_enable_ground_constraint_in_odometry, value_type=bool),
        'enable_ground_constraint_in_slam': lut.ParameterValue(args.vslam_enable_ground_constraint_in_slam, value_type=bool),
        'enable_imu_fusion': lut.ParameterValue(args.vslam_enable_imu, value_type=bool),
        'image_qos': args.vslam_image_qos,
        'save_map_folder_path': args.vslam_save_map_folder_path,
        'load_map_folder_path': args.vslam_load_map_folder_path,
        'localize_on_startup': lut.ParameterValue(args.vslam_localize_on_startup, value_type=bool),
        'enable_request_hint': args.vslam_enable_request_hint,
        'rectified_images': use_rectified_images,
        'img_mask_bottom': args.vslam_img_mask_bottom,
        'img_mask_left': args.vslam_img_mask_left,
        'img_mask_right': args.vslam_img_mask_right,
        'map_frame': args.vslam_map_frame,
        'odom_frame': args.vslam_odom_frame,
        'base_frame': args.vslam_base_frame,
        'publish_odom_to_base_tf': args.publish_odom_to_base_tf,
        'publish_map_to_odom_tf': args.vslam_publish_map_to_odom_tf,
        'invert_odom_to_base_tf': args.invert_odom_to_base_tf,
        'enable_slam_visualization': lut.ParameterValue(args.vslam_enable_visualization, value_type=bool),
        'enable_observations_view': lut.ParameterValue(args.vslam_enable_visualization, value_type=bool),
        'enable_landmarks_view': lut.ParameterValue(args.vslam_enable_visualization, value_type=bool),
        'use_sim_time': lut.ParameterValue(args.use_sim_time, value_type=bool),
    })

    visual_slam_node = lut.ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=parameters, 
        remappings=remappings,
    )

    actions.append(lu.load_composable_nodes(args.container_name, [visual_slam_node]))
    actions.append(
        lu.log_info(["Enabling standard VSLAM for cameras '",
                    args.vslam_enabled_stereo_cameras, "'"]))

    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', 'nova_container')
    args.add_arg('vslam_image_qos', 'SENSOR_DATA')
    args.add_arg('invert_odom_to_base_tf', False)
    args.add_arg('is_sim', False)
    args.add_arg('publish_odom_to_base_tf', True)
    args.add_arg('vslam_enable_imu', False)
    args.add_arg('vslam_publish_map_to_odom_tf', True)
    args.add_arg('vslam_enable_slam', False)
    args.add_arg('vslam_enabled_stereo_cameras', '')
    args.add_arg('vslam_topic_config_file', '')
    args.add_arg('vslam_load_map_folder_path', '')
    args.add_arg('vslam_save_map_folder_path', '')
    args.add_arg('vslam_localize_on_startup', False)
    args.add_arg('vslam_map_frame', 'map')
    args.add_arg('vslam_odom_frame', 'odom')
    args.add_arg('vslam_enable_request_hint', True)
    args.add_arg('vslam_initial_pose_topic', '/localization/pose_hint')
    args.add_arg('vslam_trigger_hint_topic', '/visual_slam/trigger_hint')
    args.add_arg('vslam_diagnostics_topic', '/diagnostics')
    args.add_arg('vslam_use_rectified_images', False)
    args.add_arg('vslam_enable_ground_constraint_in_odometry', False)
    args.add_arg('vslam_enable_ground_constraint_in_slam', False)
    args.add_arg('vslam_img_mask_bottom', 0)
    args.add_arg('vslam_img_mask_left', 0)
    args.add_arg('vslam_img_mask_right', 0)
    args.add_arg('vslam_enable_visualization', False)
    args.add_arg('vslam_camera_optical_frames', '')
    args.add_arg('vslam_base_frame', 'base_link')
    args.add_arg('use_sim_time', False)

    args.add_opaque_function(add_vslam)

    return lut.LaunchDescription(args.get_launch_actions())
