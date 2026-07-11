# Joy Calibration

`jetpilot_teleop_tools` includes a terminal calibrator for creating controller
profiles from `/dev/input/js*` devices. The raw joy publisher remains separate:
`custom_joy_node` publishes `/joy`, and the generated YAML files configure the
teleop nodes that consume `/joy`.

## Calibrate a Controller

After building and sourcing the ROS workspace:

```bash
ros2 run jetpilot_teleop_tools joy_calibrator.py calibrate \
  --profile ros2_ws/src/tool/jetpilot_teleop_tools/config/joy_profile.yaml \
  --teleop-cmd ros2_ws/src/tool/jetpilot_teleop_tools/config/teleop_cmd.generated.yaml \
  --button-mapping ros2_ws/src/tool/jetpilot_teleop_tools/config/joy_button_mapping.generated.yaml
```

The wizard first asks which `/dev/input/js*` device to use, observes the idle
axis centers, then walks through buttons, triggers, sticks, and d-pad inputs.
Detection ignores already assigned inputs and requires axis movement above a
threshold so stick noise is not accepted as a button or trigger.

## Check the Result

```bash
ros2 run jetpilot_teleop_tools joy_calibrator.py test \
  --profile ros2_ws/src/tool/jetpilot_teleop_tools/config/joy_profile.yaml
```

Add `--raw` to show raw axis values next to the normalized tester output.

## Use Generated Parameters

Launch teleop with the generated parameter files:

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_joy:=true \
  enable_teleop:=true \
  teleop_cmd_param:=ros2_ws/src/tool/jetpilot_teleop_tools/config/teleop_cmd.generated.yaml \
  teleop_button_mapping_param:=ros2_ws/src/tool/jetpilot_teleop_tools/config/joy_button_mapping.generated.yaml
```

The full `joy_profile.yaml` is intended for calibration and tester tools. The
`teleop_cmd.generated.yaml` and `joy_button_mapping.generated.yaml` files are
the runtime inputs for the current ROS nodes.
