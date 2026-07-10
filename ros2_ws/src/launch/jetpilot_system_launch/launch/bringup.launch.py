# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from launch.conditions import IfCondition


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg('use_sim_time', False, cli=True)

    args.add_arg('enable_rosbag_replay', False, cli=True)
    args.add_arg('rosbag', '', cli=True)
    args.add_arg('replay_rate', '1.0', cli=True)
    args.add_arg('replay_additional_args', '', cli=True)
    args.add_arg('rosbag_start_delay_s', '0.0', cli=True)
    args.add_arg('rosbag_shutdown_on_exit', True, cli=True)

    args.add_arg('enable_tool', True, cli=True)
    args.add_arg('enable_bag_manager', True, cli=True)
    args.add_arg('enable_joy', True, cli=True)
    args.add_arg('enable_teleop', True, cli=True)
    args.add_arg('enable_rc_serial', False, cli=True)
    args.add_arg('control_authority', 'hardware_mux', cli=True)
    args.add_arg('bag_manager_param', lu.get_path('jetpilot_bag_tools', 'config/bag_manager.param.yaml'), cli=True)
    args.add_arg('teleop_cmd_param', lu.get_path('jetpilot_teleop_tools', 'config/teleop_cmd.param.yaml'), cli=True)
    args.add_arg('teleop_button_mapping_param', lu.get_path('jetpilot_teleop_tools', 'config/joy_button_mapping.param.yaml'), cli=True)
    args.add_arg('serial_reader_param', lu.get_path('rc_serial_reader', 'config/serial_reader_node.param.yaml'), cli=True)
    args.add_arg('rc_channels_topic', '/rc/channels', cli=True)
    args.add_arg('propo_control_topic', '/propo/control_cmd', cli=True)
    args.add_arg('joy_autorepeat_rate', 50.0, cli=True)
    args.add_arg('joy_deadzone', 0.05, cli=True)

    args.add_arg('enable_operation', True, cli=True)
    args.add_arg('operation_param', lu.get_path('jetpilot_operation', 'config/operation.param.yaml'), cli=True)

    args.add_arg('enable_control', False, cli=True)
    args.add_arg('control_param', lu.get_path('jetpilot_control', 'config/autonomous_control.param.yaml'), cli=True)

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
    args.add_arg('vslam_hint_request_topic', '/visual_slam/trigger_hint', cli=True)
    args.add_arg('vslam_pose_hint_topic', '/localization/pose_hint', cli=True)
    args.add_arg('vslam_save_map_folder_path', '', cli=True)
    args.add_arg('enable_localization_manager', False, cli=True)
    args.add_arg('localization_manager_param', '', cli=True)
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

    args.add_arg('enable_vehicle', True, cli=True)
    args.add_arg('vehicle_interface_pkg', 'pca9685_rc_driver', cli=True)
    args.add_arg('vehicle_interface_launch', 'launch/pca9685_rc_interface.launch.xml', cli=True)
    args.add_arg(
        'vehicle_driver_param',
        lu.get_path('pca9685_rc_driver', 'config/pca9685_rc_driver_node.param.yaml'),
        cli=True)
    args.add_arg('vehicle_control_topic', '/vehicle/control_cmd', cli=True)
    args.add_arg('publish_vehicle_description', False, cli=True)

    args.add_arg('enable_vslam_snapshot', False, cli=True)
    args.add_arg('vslam_snapshot_output', '/tmp/vslam_reference_snapshot.json', cli=True)
    args.add_arg('vslam_snapshot_path_topic', '/visual_slam/tracking/slam_path', cli=True)
    args.add_arg('vslam_snapshot_odom_topic', '/visual_slam/tracking/odometry', cli=True)
    args.add_arg('vslam_snapshot_landmarks_topic', '/visual_slam/vis/landmarks_cloud', cli=True)
    args.add_arg('vslam_snapshot_write_interval_s', '5.0', cli=True)

    args.add_arg('enable_rviz', False, cli=True)
    args.add_arg(
        'rviz_config_file',
        lu.get_path('jetpilot_system_launch', 'rviz/default.rviz'),
        cli=True)

    actions = args.get_launch_actions()

    rosbag_replay_enabled = lut.AndSubstitution(
        lu.is_true(args.enable_rosbag_replay),
        lu.is_valid(args.rosbag),
    )
    actions.append(
        lu.play_rosbag(
            args.rosbag,
            rate=args.replay_rate,
            delay=args.rosbag_start_delay_s,
            additional_bag_play_args=args.replay_additional_args,
            shutdown_on_exit=args.rosbag_shutdown_on_exit,
            condition=IfCondition(rosbag_replay_enabled),
        ))
    actions.append(
        lu.log_info(
            'A rosbag path was provided, but enable_rosbag_replay is false; replay is disabled.',
            condition=IfCondition(lut.AndSubstitution(
                lut.NotSubstitution(lu.is_true(args.enable_rosbag_replay)),
                lu.is_valid(args.rosbag),
            )),
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
                'enable_joy': args.enable_joy,
                'enable_teleop': args.enable_teleop,
                'enable_rc_serial': args.enable_rc_serial,
                'control_authority': args.control_authority,
                'rc_channels_topic': args.rc_channels_topic,
                'propo_control_topic': args.propo_control_topic,
                'joy_autorepeat_rate': args.joy_autorepeat_rate,
                'joy_deadzone': args.joy_deadzone,
                'enable_vslam_snapshot': args.enable_vslam_snapshot,
                'vslam_snapshot_output': args.vslam_snapshot_output,
                'vslam_snapshot_path_topic': args.vslam_snapshot_path_topic,
                'vslam_snapshot_odom_topic': args.vslam_snapshot_odom_topic,
                'vslam_snapshot_landmarks_topic':
                    args.vslam_snapshot_landmarks_topic,
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
            condition=IfCondition(args.enable_operation),
        ))

    actions.append(
        lu.include(
            'jetpilot_system_launch',
            'launch/control.launch.py',
            launch_arguments={
                'control_param': args.control_param,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(args.enable_control),
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
                'vslam_hint_request_topic': args.vslam_hint_request_topic,
                'vslam_pose_hint_topic': args.vslam_pose_hint_topic,
                'vslam_save_map_folder_path': args.vslam_save_map_folder_path,
                'enable_localization_manager': args.enable_localization_manager,
                'localization_manager_param': args.localization_manager_param,
                'enable_vgl': args.enable_vgl,
                'vgl_topic_config_file': args.vgl_topic_config_file,
                'vgl_model_dir': args.vgl_model_dir,
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
                'vehicle_control_topic': args.vehicle_control_topic,
                'vehicle_driver_param': args.vehicle_driver_param,
                'publish_vehicle_description': args.publish_vehicle_description,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(args.enable_vehicle),
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
