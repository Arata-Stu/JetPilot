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

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
import os
import yaml


def localization_component_container(container_name: str):
    return lut.Node(
        name=container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )


def camera_optical_frames_from_topic_config(topic_config_file: str) -> str:
    if not topic_config_file or not os.path.exists(topic_config_file):
        return 'realsense_infra1_optical_frame,realsense_infra2_optical_frame'

    with open(topic_config_file, 'r', encoding='utf-8') as file:
        config = yaml.safe_load(file) or {}

    frames = []
    for camera in config.get('stereo_cameras', []):
        if not isinstance(camera, dict):
            continue
        for key in ('left_frame_id', 'right_frame_id'):
            frame = camera.get(key)
            if frame:
                frames.append(str(frame))

    return ','.join(frames) if frames else 'realsense_infra1_optical_frame,realsense_infra2_optical_frame'


def saved_localization_map_availability(map_dir: str) -> tuple[bool, bool]:
    """Return whether the saved cuVGL and cuVSLAM map directories are available."""
    if not map_dir:
        return False, False

    cuvgl_map_dir = os.path.join(map_dir, 'cuvgl_map')
    cuvslam_map_dir = os.path.join(map_dir, 'cuvslam_map')
    return os.path.isdir(cuvgl_map_dir), os.path.isdir(cuvslam_map_dir)


def add_nodes(args: lu.ArgumentContainer):
    use_sim_time = lu.is_true(args.use_sim_time)
    enable_vgl = lu.is_true(args.enable_vgl)
    enable_vslam = lu.is_true(args.enable_vslam)
    enable_localization_manager = lu.is_true(args.enable_localization_manager)
    enable_occupancy_map_server = lu.is_true(args.enable_occupancy_map_server)
    enable_occupancy_map_lifecycle_manager = lu.is_true(
        args.enable_occupancy_map_lifecycle_manager)
    enable_omap_frame = (
        lu.is_true(args.enable_omap_frame) or enable_occupancy_map_server)
    enable_hd_map_publisher = lu.is_true(args.enable_hd_map_publisher)
    enable_section_localizer = lu.is_true(args.enable_section_localizer)
    cuvgl_map_available, cuvslam_map_available = saved_localization_map_availability(
        args.map_dir)
    mapping_mode = bool(args.vslam_save_map_folder_path)
    vgl_started = False

    actions = []
    if lu.is_true(args.run_standalone):
        actions.append(localization_component_container(args.container_name))

    camera_optical_frames = camera_optical_frames_from_topic_config(args.vgl_topic_config_file)
    base_frame = args.localization_base_frame
    if enable_vgl:
        cuvgl_map_dir = os.path.join(args.map_dir, 'cuvgl_map') if args.map_dir else ''
        if cuvgl_map_available:
            vgl_started = True
            actions.append(
                lu.include(
                    'jetpilot_system_launch',
                    'launch/localization/vgl.launch.py',
                    launch_arguments={
                        'container_name': args.container_name,
                        'vgl_enabled_stereo_cameras': args.camera_name,
                        'vgl_do_rectify_images': False,
                        'vgl_map_frame': 'map',
                        'vgl_publish_rectified_images': False,
                        'vgl_camera_optical_frames': camera_optical_frames,
                        'vgl_map_dir': cuvgl_map_dir,
                        'vgl_base_frame': base_frame,
                        'topic_config_file': args.vgl_topic_config_file,
                        'vgl_model_dir': args.vgl_model_dir,
                        'vgl_trigger_service': args.vgl_trigger_service,
                        'vgl_pose_topic': args.vgl_pose_topic,
                        'use_sim_time': use_sim_time,
                        'vgl_config_dir': lu.get_path(
                            'isaac_ros_visual_mapping',
                            'configs/single_stereo_localizer')
                    },
                ))
        else:
            actions.append(
                lu.log_info(
                    f'VGL is enabled but its map directory is unavailable: '
                    f'{cuvgl_map_dir or "<unset>"}. Skipping VGL.'))

    if enable_vslam:
        params = {
            'container_name': args.container_name,
            'vslam_enabled_stereo_cameras': args.camera_name,
            'vslam_topic_config_file': args.vgl_topic_config_file,
            'vslam_map_frame': 'map',
            'vslam_odom_frame': 'odom',
            'vslam_image_qos': 'SENSOR_DATA',
            'vslam_publish_map_to_odom_tf': True,
            'vslam_enable_slam': lu.is_true(args.vslam_enable_slam),
            'vslam_enable_visualization': lu.is_true(args.vslam_enable_visualization),
            'vslam_enable_ground_constraint_in_odometry':
                lu.is_true(args.vslam_enable_ground_constraint_in_odometry),
            'vslam_enable_ground_constraint_in_slam':
                lu.is_true(args.vslam_enable_ground_constraint_in_slam),
            'vslam_camera_optical_frames': camera_optical_frames,
            'vslam_base_frame': base_frame,
            'vslam_use_rectified_images': True,
            'vslam_initial_pose_topic': args.vslam_pose_hint_topic,
            'vslam_trigger_hint_topic': args.vslam_hint_request_topic,
            'vslam_diagnostics_topic': args.localization_diagnostics_topic,
            'use_sim_time': use_sim_time,
        }
        if args.vslam_save_map_folder_path:
            params['vslam_save_map_folder_path'] = args.vslam_save_map_folder_path

        cuvslam_map_dir = os.path.join(args.map_dir, 'cuvslam_map') if args.map_dir else ''
        if cuvslam_map_available and not mapping_mode:
            params['vslam_load_map_folder_path'] = cuvslam_map_dir
        elif cuvslam_map_available and mapping_mode:
            actions.append(
                lu.log_info(
                    'VSLAM map output is configured; starting mapping without '
                    'loading or localizing against the existing saved map.'))
        elif args.map_dir:
            actions.append(
                lu.log_info(
                    f'No cuVSLAM map found at {cuvslam_map_dir}; '
                    'starting VSLAM without a saved map.'))

        actions.append(
            lu.include(
                'jetpilot_system_launch',
                'launch/localization/vslam.launch.py',
                launch_arguments=params,
            ))

        if enable_localization_manager and not mapping_mode:
            localization_manager_params = [{
                'use_vgl': vgl_started,
                'autostart': cuvslam_map_available and not mapping_mode,
                'use_sim_time': use_sim_time,
                'vslam_hint_request_topic': args.vslam_hint_request_topic,
                'vslam_pose_hint_topic': args.vslam_pose_hint_topic,
                'manual_pose_topic': args.manual_pose_topic,
                'vgl_trigger_service': args.vgl_trigger_service,
                'vgl_pose_topic': args.vgl_pose_topic,
                'localization_trigger_topic': args.localization_trigger_topic,
                'localization_trigger_service': args.localization_trigger_service,
                'diagnostics_topic': args.localization_diagnostics_topic,
                'pose_hint_required_topic': args.pose_hint_required_topic,
                'pose_hint_state_topic': args.pose_hint_state_topic,
            }]
            if args.localization_manager_param:
                localization_manager_params.insert(0, args.localization_manager_param)

            actions.append(lu.Node(
                package='jetpilot_localization_manager',
                executable='jetpilot_localization_manager_node',
                name='localization_manager',
                output='screen',
                parameters=localization_manager_params,
            ))
        elif enable_localization_manager and mapping_mode:
            actions.append(
                lu.log_info(
                    'Localization manager is disabled for this VSLAM mapping run.'))

    occupancy_map_yaml_path = args.occupancy_map_yaml_path
    if not occupancy_map_yaml_path and args.map_dir:
        occupancy_map_yaml_path = os.path.join(args.map_dir, 'occupancy_map.yaml')

    occupancy_map_available = (
        occupancy_map_yaml_path and os.path.isfile(occupancy_map_yaml_path))
    if enable_occupancy_map_server and occupancy_map_available:
        actions.append(lu.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': occupancy_map_yaml_path,
                'frame_id': 'omap',
                'use_sim_time': use_sim_time,
            }],
        ))

        if enable_occupancy_map_lifecycle_manager:
            actions.append(lut.TimerAction(
                period='5.0',
                actions=[
                    lu.Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_map_server',
                        output='screen',
                        parameters=[{
                            'autostart': True,
                            'node_names': ['map_server'],
                            'use_sim_time': use_sim_time,
                        }],
                    ),
                ],
            ))
    elif enable_occupancy_map_server:
        actions.append(
            lu.log_info(
                f'Occupancy map server is enabled but its YAML is unavailable: '
                f'{occupancy_map_yaml_path or "<unset>"}. '
                'skipping occupancy map server.'))
    elif enable_occupancy_map_lifecycle_manager:
        actions.append(
            lu.log_info(
                'Occupancy map lifecycle manager is enabled, but the occupancy map '
                'server is disabled. Skipping the lifecycle manager.'))

    if enable_omap_frame:
        ground_plane_published = False
        ground_plane_file = (
            os.path.join(args.map_dir, 'ground_plane.yaml') if args.map_dir else '')
        if ground_plane_file and os.path.exists(ground_plane_file):
            actions.append(lu.log_info(f'Publishing ground plane from {ground_plane_file}'))
            actions.append(
                lu.include(
                    'isaac_mapping_ros',
                    'launch/tools/publish_ground_plane.launch.py',
                    launch_arguments={
                        'ground_plane_file': ground_plane_file,
                        'parent_frame': 'map',
                        'child_frame': 'omap',
                    },
                ))
            ground_plane_published = True

        if not ground_plane_published:
            actions.append(lu.Node(
                name='map_to_omap_static_transform_publisher',
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'omap'],
                output='screen'
            ))
            actions.append(
                lu.log_info('No ground plane file found, publishing identity transform'))

    hd_map_yaml_path = args.hd_map_yaml_path
    if not hd_map_yaml_path and args.map_dir:
        map_dir_name = os.path.basename(os.path.normpath(args.map_dir))
        hd_map_yaml_path = os.path.join(args.map_dir, f'{map_dir_name}_hd_map.yaml')

    hd_map_available = hd_map_yaml_path and os.path.isfile(hd_map_yaml_path)
    if (enable_hd_map_publisher or enable_section_localizer) and not hd_map_available:
        actions.append(
            lu.log_info(
                f'HD map features are enabled but their YAML is unavailable: '
                f'{hd_map_yaml_path or "<unset>"}. Skipping HD map nodes.'))

    if enable_hd_map_publisher and hd_map_available:
        hd_map_parameters = [{
            'hd_map_yaml_path': hd_map_yaml_path,
            'use_sim_time': use_sim_time,
        }]
        if args.hd_map_publisher_param:
            hd_map_parameters.insert(0, args.hd_map_publisher_param)

        actions.append(lu.Node(
            package='jetpilot_hdmap_publisher',
            executable='hd_map_publisher_node.py',
            name='hd_map_publisher',
            output='screen',
            parameters=hd_map_parameters,
            remappings=[
                ('lane_markers', args.hd_map_lane_markers_topic),
                ('section_markers', args.hd_map_section_markers_topic),
                ('primary_centerline_path', args.hd_map_primary_centerline_path_topic),
            ],
        ))

    if enable_section_localizer and hd_map_available:
        actions.append(lu.Node(
            package='jetpilot_hdmap_publisher',
            executable='hd_map_section_localizer_node.py',
            name='hd_map_section_localizer',
            output='screen',
            parameters=[{
                'hd_map_yaml_path': hd_map_yaml_path,
                'use_sim_time': use_sim_time,
                'base_frame': args.hd_map_base_frame,
                'current_section_topic': args.current_section_topic,
                'current_section_marker_topic': args.current_section_marker_topic,
            }],
        ))

    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg('camera_name', 'realsense', cli=True)
    args.add_arg('container_name', 'nova_container', cli=True)
    args.add_arg('run_standalone', True, cli=True)
    args.add_arg('shutdown_on_container_exit', True, cli=True)
    args.add_arg('map_dir', '', cli=True)
    args.add_arg('localization_base_frame', 'base_link', cli=True)
    args.add_arg('use_sim_time', False, cli=True)

    # vslam parameters
    args.add_arg('enable_vslam', True, cli=True)
    args.add_arg('vslam_enable_slam', True, cli=True)
    args.add_arg('vslam_enable_ground_constraint_in_odometry', False, cli=True)
    args.add_arg('vslam_enable_ground_constraint_in_slam', False, cli=True)
    args.add_arg('vslam_enable_visualization', False, cli=True)
    args.add_arg('vslam_hint_request_topic', '/visual_slam/trigger_hint', cli=True)
    args.add_arg('vslam_pose_hint_topic', '/localization/pose_hint', cli=True)
    args.add_arg('vslam_save_map_folder_path', '', cli=True)
    args.add_arg('manual_pose_topic', '/initialpose', cli=True)
    args.add_arg(
        'vgl_trigger_service', '/visual_localization/trigger_localization', cli=True)
    args.add_arg('vgl_pose_topic', '/visual_localization/pose', cli=True)
    args.add_arg('localization_trigger_topic', '/localization/trigger', cli=True)
    args.add_arg('localization_trigger_service', '/localization/relocalize', cli=True)
    args.add_arg('localization_diagnostics_topic', '/diagnostics', cli=True)
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

    args.add_arg('enable_occupancy_map_server', False, cli=True)
    args.add_arg('enable_occupancy_map_lifecycle_manager', False, cli=True)
    args.add_arg('enable_omap_frame', False, cli=True)
    args.add_arg('occupancy_map_yaml_path', '', cli=True)

    args.add_arg('enable_hd_map_publisher', False, cli=True)
    args.add_arg('enable_section_localizer', False, cli=True)
    args.add_arg('hd_map_publisher_param', '', cli=True)
    args.add_arg('hd_map_yaml_path', '', cli=True)
    args.add_arg('hd_map_base_frame', 'base_link', cli=True)
    args.add_arg('hd_map_lane_markers_topic', '/hd_map/lane_markers', cli=True)
    args.add_arg('hd_map_section_markers_topic', '/hd_map/section_markers', cli=True)
    args.add_arg('hd_map_primary_centerline_path_topic', '/hd_map/primary_centerline_path', cli=True)
    args.add_arg('current_section_topic', '/localization/current_section', cli=True)
    args.add_arg('current_section_marker_topic', '/localization/current_section_marker', cli=True)

    args.add_arg('type_negotiation_duration_s', lu.get_default_negotiation_time(), cli=True)

    args.add_opaque_function(add_nodes)

    actions = args.get_launch_actions()

    actions.append(
        lut.SetParameter('type_negotiation_duration_s', args.type_negotiation_duration_s))
    actions.append(
        lu.log_info(['Using type negotiation duration: ', args.type_negotiation_duration_s]))

    return lut.LaunchDescription(actions)
