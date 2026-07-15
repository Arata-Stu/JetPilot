# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut

def add_sensor_interface(args: lu.ArgumentContainer):
    if not (
        lu.is_true(args.enable_sensor_interface) and
        lu.is_true(args.enable_realsense)
    ):
        return []

    return [
        lu.include(
            str(args.sensor_interface_pkg),
            str(args.sensor_interface_launch),
            launch_arguments={
                'camera_name': str(args.camera_name),
                'container_name': str(args.container_name),
                'run_standalone': str(lu.is_true(args.run_standalone)).lower(),
                'enable_depth': str(lu.is_true(args.enable_depth)).lower(),
                'enable_color': str(lu.is_true(args.enable_color)).lower(),
                'use_sim_time': str(lu.is_true(args.use_sim_time)).lower(),
            },
        ),
        lu.log_info([
            'Sensor interface: ',
            str(args.sensor_interface_pkg),
            '/',
            str(args.sensor_interface_launch),
            ', camera: ',
            str(args.camera_name),
        ]),
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
