# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from launch.conditions import IfCondition


def add_sensor_interface(args: lu.ArgumentContainer):
    return [
        lu.include(
            str(args.sensor_interface_pkg),
            str(args.sensor_interface_launch),
            launch_arguments={
                'camera_name': args.camera_name,
                'container_name': args.container_name,
                'run_standalone': args.run_standalone,
                'enable_depth': args.enable_depth,
                'enable_color': args.enable_color,
                'use_sim_time': args.use_sim_time,
            },
            condition=IfCondition(lut.AndSubstitution(
                lu.is_true(args.enable_sensor_interface),
                lu.is_true(args.enable_realsense),
            )),
        ),
        lu.log_info([
            'Sensor interface: ',
            args.sensor_interface_pkg,
            '/',
            args.sensor_interface_launch,
            ', camera: ',
            args.camera_name,
        ],
            condition=IfCondition(lut.AndSubstitution(
                lu.is_true(args.enable_sensor_interface),
                lu.is_true(args.enable_realsense),
            )),
        ),
    ]


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg('enable_sensor_interface', True, cli=True)
    args.add_arg('sensor_interface_pkg', 'jetpilot_system_launch', cli=True)
    args.add_arg('sensor_interface_launch', 'launch/sensors/realsense.launch.py', cli=True)
    args.add_arg('enable_realsense', True, cli=True)
    args.add_arg('camera_name', 'realsense', cli=True)
    args.add_arg('container_name', 'sensor_kit_container', cli=True)
    args.add_arg('run_standalone', True, cli=True)
    args.add_arg('enable_depth', False, cli=True)
    args.add_arg('enable_color', False, cli=True)
    args.add_arg('use_sim_time', False, cli=True)

    args.add_opaque_function(add_sensor_interface)

    return lut.LaunchDescription(args.get_launch_actions())
