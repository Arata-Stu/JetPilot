# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from launch.conditions import IfCondition


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg('enable_realsense', True, cli=True)
    args.add_arg('camera_name', 'realsense', cli=True)
    args.add_arg('container_name', 'sensor_kit_container', cli=True)
    args.add_arg('run_standalone', True, cli=True)
    args.add_arg('enable_depth', False, cli=True)
    args.add_arg('enable_color', False, cli=True)
    args.add_arg('use_sim_time', False, cli=True)

    actions = args.get_launch_actions()
    actions.append(
        lu.include(
            'jetpilot_system_launch',
            'launch/sensors/realsense.launch.py',
            launch_arguments={
                'camera_name': args.camera_name,
                'container_name': args.container_name,
                'run_standalone': args.run_standalone,
                'enable_depth': args.enable_depth,
                'enable_color': args.enable_color,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(args.enable_realsense),
        ))

    return lut.LaunchDescription(actions)
