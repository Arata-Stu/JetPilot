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

The terminal prompts are shown in Japanese and English so the same tool can be
used by mixed-language teams. Calibration also writes a standalone HTML editor
next to the profile by default, for example `joy_profile.html`.

## Check the Result

```bash
ros2 run jetpilot_teleop_tools joy_calibrator.py test \
  --profile ros2_ws/src/tool/jetpilot_teleop_tools/config/joy_profile.yaml
```

Add `--raw` to show raw axis values next to the normalized tester output.

## Open the HTML Editor

Open the generated HTML file in a browser:

```bash
open ros2_ws/src/tool/jetpilot_teleop_tools/config/joy_profile.html
```

The editor can load a profile YAML file, edit button/axis assignments, export
the full profile YAML, export `teleop_cmd_node` parameters, export
`teleop_button_manager_node` parameters, and run a browser-based Joy Tester
using the Gamepad API when the browser can see the controller.

To generate the editor again from an existing profile:

```bash
ros2 run jetpilot_teleop_tools joy_calibrator.py report \
  --profile ros2_ws/src/tool/jetpilot_teleop_tools/config/joy_profile.yaml
```

To generate a blank editor first and load YAML from the browser UI:

```bash
ros2 run jetpilot_teleop_tools joy_calibrator.py ui
open ros2_ws/src/tool/jetpilot_teleop_tools/config/joy_profile_editor.html
```

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
