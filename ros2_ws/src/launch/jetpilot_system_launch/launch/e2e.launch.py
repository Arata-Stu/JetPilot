# SPDX-License-Identifier: Apache-2.0

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from launch.substitutions import PathJoinSubstitution


def add_nodes(args: lu.ArgumentContainer):
    actions = []
    if lu.is_true(args.enable_sensor_kit):
        actions.append(
            lu.include(
                "jetpilot_system_launch",
                "launch/sensor_kit.launch.py",
                launch_arguments={
                    "enable_sensor_interface": True,
                    "sensor_interface_pkg": args.sensor_kit_interface_pkg,
                    "sensor_interface_launch": args.sensor_kit_interface_launch,
                    "camera_name": args.camera_name,
                    "container_name": args.container_name,
                    "run_standalone": True,
                    "enable_depth": args.enable_depth,
                    "enable_color": True,
                    "use_sim_time": args.use_sim_time,
                },
            )
        )

    actions.append(
        lu.include(
            "jetpilot_e2e_inference",
            "launch/e2e_pytorch.launch.py",
            launch_arguments={
                "image_topic": args.image_topic,
                "control_cmd_topic": args.control_cmd_topic,
                "model_root": args.model_root,
                "model_file_path": args.model_file_path,
                "model_format": args.model_format,
                "model_kind": args.model_kind,
                "device": args.device,
                "use_half": args.use_half,
                "use_checkpoint_config": args.use_checkpoint_config,
                "input_width": args.input_width,
                "input_height": args.input_height,
                "max_inference_rate_hz": args.max_inference_rate_hz,
                "use_sim_time": args.use_sim_time,
            },
        )
    )
    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()

    args.add_arg("enable_sensor_kit", True, cli=True)
    args.add_arg("sensor_kit_interface_pkg", "jetpilot_system_launch", cli=True)
    args.add_arg(
        "sensor_kit_interface_launch", "launch/sensors/realsense.launch.py", cli=True
    )
    args.add_arg("camera_name", "realsense", cli=True)
    args.add_arg("container_name", "multi_sensor_container", cli=True)
    args.add_arg("enable_depth", False, cli=True)

    args.add_arg("image_topic", "/realsense/color/image_raw", cli=True)
    args.add_arg("control_cmd_topic", "/auto/control_cmd", cli=True)
    args.add_arg("model_root", "/workspaces/ros2_ws/models/e2e/latest", cli=True)
    args.add_arg(
        "model_file_path",
        PathJoinSubstitution([args.model_root, "model.pt"]),
        cli=True,
    )
    args.add_arg("model_format", "auto", cli=True)
    args.add_arg("model_kind", "pilotnet", cli=True)
    args.add_arg("device", "cpu", cli=True)
    args.add_arg("use_half", False, cli=True)
    args.add_arg("use_checkpoint_config", True, cli=True)
    args.add_arg("input_width", 212, cli=True)
    args.add_arg("input_height", 120, cli=True)
    args.add_arg("max_inference_rate_hz", 0.0, cli=True)
    args.add_arg("use_sim_time", False, cli=True)

    args.add_opaque_function(add_nodes)
    return lut.LaunchDescription(args.get_launch_actions())
