# SPDX-License-Identifier: Apache-2.0

import os

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def workspace_param_path(filename: str, fallback_package: str, fallback_package_path: str) -> str:
    ros2_ws = os.environ.get('ROS2_WS', '/workspaces/ros2_ws')
    generated_path = os.path.join(ros2_ws, 'joy_profiles', filename)
    if os.path.exists(generated_path):
        return generated_path
    return lu.get_path(fallback_package, fallback_package_path)


def add_nodes(args: lu.ArgumentContainer):
    actions = []
    use_sim_time = lu.is_true(args.use_sim_time)

    if lu.is_true(args.enable_bag_manager):
        actions.append(lu.Node(
            package='jetpilot_bag_tools',
            executable='bag_manager_node.py',
            name='bag_manager_node',
            output='screen',
            parameters=[args.bag_manager_param, {'use_sim_time': use_sim_time}],
        ))

    if lu.is_true(args.enable_joy):
        actions.append(lu.Node(
            package='custom_joy_node',
            executable='custom_joy_node',
            name='custom_joy_node',
            output='screen',
            parameters=[{
                'device_path': str(args.joy_device_path),
                'publish_rate_hz': float(args.joy_autorepeat_rate),
                'deadzone': float(args.joy_deadzone),
                'prefer_evdev': lu.is_true(args.joy_prefer_evdev),
                'use_sim_time': use_sim_time,
            }],
        ))

    if lu.is_true(args.enable_teleop):
        actions.append(lu.Node(
            package='jetpilot_teleop_tools',
            executable='teleop_button_manager_node',
            name='teleop_button_manager_node',
            output='screen',
            parameters=[
                args.teleop_button_mapping_param,
                {'use_sim_time': use_sim_time},
            ],
        ))
        actions.append(lu.Node(
            package='jetpilot_teleop_tools',
            executable='teleop_cmd_node',
            name='teleop_cmd_node',
            output='screen',
            parameters=[
                args.teleop_cmd_param,
                {'use_sim_time': use_sim_time},
            ],
        ))

    if lu.is_true(args.enable_rc_serial):
        actions.append(lu.Node(
            package='rc_serial_reader',
            executable='serial_reader_node',
            name='serial_reader_node',
            output='screen',
            parameters=[
                args.serial_reader_param,
                {
                    'control_authority': args.control_authority,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('/rc/channels', args.rc_channels_topic),
                ('/propo/control_cmd', args.propo_control_topic),
            ],
        ))

    if lu.is_true(args.enable_vslam_snapshot):
        actions.append(lu.Node(
            package='vslam_map_tools',
            executable='record_vslam_reference_snapshot.py',
            name='vslam_reference_snapshot_recorder',
            output='screen',
            arguments=[
                '--path-topic', args.vslam_snapshot_path_topic,
                '--odom-topic', args.vslam_snapshot_odom_topic,
                '--landmarks-topic', args.vslam_snapshot_landmarks_topic,
                '--output', args.vslam_snapshot_output,
                '--write-interval-sec', str(args.vslam_snapshot_write_interval_s),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ))

    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

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
    args.add_arg('enable_bag_manager', False, cli=True)
    args.add_arg('enable_joy', False, cli=True)
    args.add_arg('enable_teleop', False, cli=True)
    args.add_arg('enable_rc_serial', False, cli=True)
    args.add_arg('control_authority', 'hardware_mux', cli=True)
    args.add_arg('rc_channels_topic', '/rc/channels', cli=True)
    args.add_arg('propo_control_topic', '/propo/control_cmd', cli=True)
    args.add_arg('joy_autorepeat_rate', '50.0', cli=True)
    args.add_arg('joy_deadzone', '0.05', cli=True)
    args.add_arg('joy_device_path', '', cli=True)
    args.add_arg('joy_prefer_evdev', False, cli=True)
    args.add_arg('enable_vslam_snapshot', False, cli=True)
    args.add_arg(
        'vslam_snapshot_output',
        '/tmp/vslam_reference_snapshot.json',
        cli=True)
    args.add_arg(
        'vslam_snapshot_path_topic',
        '/visual_slam/tracking/slam_path',
        cli=True)
    args.add_arg(
        'vslam_snapshot_odom_topic',
        '/visual_slam/tracking/odometry',
        cli=True)
    args.add_arg(
        'vslam_snapshot_landmarks_topic',
        '/visual_slam/vis/landmarks_cloud',
        cli=True)
    args.add_arg('vslam_snapshot_write_interval_s', '5.0', cli=True)
    args.add_arg('use_sim_time', False, cli=True)

    args.add_opaque_function(add_nodes)

    actions = args.get_launch_actions()
    return lut.LaunchDescription(actions)
