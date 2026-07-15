# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg(
        'control_param',
        lu.get_path('jetpilot_system_launch', 'config/control/autonomous_control.param.yaml'),
        cli=True)
    args.add_arg('use_sim_time', False, cli=True)

    actions = args.get_launch_actions()
    actions.append(
        lu.include(
            'jetpilot_control',
            'launch/jetpilot_control.launch.xml',
            launch_arguments={
                'config_file': args.control_param,
                'use_sim_time': args.use_sim_time,
            },
        ))

    return lut.LaunchDescription(actions)
