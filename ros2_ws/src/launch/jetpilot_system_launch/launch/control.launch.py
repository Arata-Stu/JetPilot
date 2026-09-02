# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg(
        'control_param',
        lu.get_path('jetpilot_controller', 'config/controller.param.yaml'),
        cli=True)
    args.add_arg(
        'throttle_calibration_file',
        lu.get_path(
            'jetpilot_controller',
            'config/throttle_calibration.empty.param.yaml'),
        cli=True)
    args.add_arg('diagnostics_topic', '/controller/diagnostics', cli=True)
    args.add_arg('use_sim_time', False, cli=True)

    actions = args.get_launch_actions()
    actions.append(
        lu.include(
            'jetpilot_controller',
            'launch/jetpilot_controller.launch.xml',
            launch_arguments={
                'config_file': args.control_param,
                'throttle_calibration_file': args.throttle_calibration_file,
                'diagnostics_topic': args.diagnostics_topic,
                'use_sim_time': args.use_sim_time,
            },
        ))

    return lut.LaunchDescription(actions)
