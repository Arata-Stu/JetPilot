# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def add_vehicle(args: lu.ArgumentContainer):
    return [
        lu.include(
            str(args.vehicle_interface_pkg),
            str(args.vehicle_interface_launch),
            launch_arguments={
                'vehicle_control_topic': args.vehicle_control_topic,
                'driver_param': args.vehicle_driver_param,
                'publish_description': args.publish_vehicle_description,
                'use_sim_time': args.use_sim_time,
            },
        ),
        lu.log_info([
            'Vehicle interface: ',
            args.vehicle_interface_pkg,
            '/',
            args.vehicle_interface_launch,
            ', input: ',
            args.vehicle_control_topic,
        ]),
    ]


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg('vehicle_interface_pkg', 'pca9685_rc_driver', cli=True)
    args.add_arg('vehicle_interface_launch', 'launch/pca9685_rc_interface.launch.xml', cli=True)
    args.add_arg('vehicle_control_topic', '/vehicle/control_cmd', cli=True)
    args.add_arg(
        'vehicle_driver_param',
        lu.get_path('pca9685_rc_driver', 'config/pca9685_rc_driver_node.param.yaml'),
        cli=True)
    args.add_arg('publish_vehicle_description', False, cli=True)
    args.add_arg('use_sim_time', False, cli=True)

    args.add_opaque_function(add_vehicle)

    return lut.LaunchDescription(args.get_launch_actions())
