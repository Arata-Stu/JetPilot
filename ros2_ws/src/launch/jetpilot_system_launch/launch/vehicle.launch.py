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


def add_vehicle(args: lu.ArgumentContainer):
    return [
        lu.include(
            str(args.vehicle_interface_pkg),
            str(args.vehicle_interface_launch),
            launch_arguments={
                'vehicle_control_topic': args.vehicle_control_topic,
                'driver_param': args.vehicle_driver_param,
                'publish_description': args.publish_vehicle_description,
                'description_base_frame': args.vehicle_description_base_frame,
                'description_camera_frame': args.vehicle_description_camera_frame,
                'description_camera_x': args.vehicle_description_camera_x,
                'description_camera_y': args.vehicle_description_camera_y,
                'description_camera_z': args.vehicle_description_camera_z,
                'description_camera_roll': args.vehicle_description_camera_roll,
                'description_camera_pitch': args.vehicle_description_camera_pitch,
                'description_camera_yaw': args.vehicle_description_camera_yaw,
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
        workspace_param_path(
            'pca9685_rc_driver_node.param.yaml',
            'pca9685_rc_driver',
            'config/pca9685_rc_driver_node.param.yaml'),
        cli=True)
    args.add_arg('publish_vehicle_description', True, cli=True)
    args.add_arg('vehicle_description_base_frame', 'base_link', cli=True)
    args.add_arg('vehicle_description_camera_frame', 'realsense_camera_link', cli=True)
    args.add_arg('vehicle_description_camera_x', '0.2075', cli=True)
    args.add_arg('vehicle_description_camera_y', '0.019', cli=True)
    args.add_arg('vehicle_description_camera_z', '0.065', cli=True)
    args.add_arg('vehicle_description_camera_roll', '0.0', cli=True)
    args.add_arg('vehicle_description_camera_pitch', '0.0', cli=True)
    args.add_arg('vehicle_description_camera_yaw', '0.0', cli=True)
    args.add_arg('use_sim_time', False, cli=True)

    args.add_opaque_function(add_vehicle)

    return lut.LaunchDescription(args.get_launch_actions())
