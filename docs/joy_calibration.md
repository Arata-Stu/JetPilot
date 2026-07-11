# Joy Calibration

`jetpilot_teleop_tools` includes a terminal calibrator for creating controller
profiles from `/dev/input/js*` devices. The raw joy publisher remains separate:
`custom_joy_node` publishes `/joy`, and the generated YAML files configure the
teleop nodes that consume `/joy`.

## Calibrate a Controller

After building and sourcing the ROS workspace:

```bash
ros2 run jetpilot_teleop_tools joy_calibrator.py calibrate \
  --profile joy_profile.yaml
```

By default this writes all generated files next to `--profile`:

```text
joy_profile.yaml
joy_profile.html
teleop_cmd.generated.yaml
joy_button_mapping.generated.yaml
```

The wizard first asks which `/dev/input/js*` device to use, observes the idle
axis centers, then walks through buttons, triggers, sticks, and d-pad inputs.
Detection ignores already assigned inputs and requires axis movement above a
threshold so stick noise is not accepted as a button or trigger.

The terminal prompts are shown in Japanese and English so the same tool can be
used by mixed-language teams. Calibration also writes a standalone HTML editor
next to the profile by default, for example `joy_profile.html`.

The blank editor defaults to DualShock4 values:

```yaml
device:
  name: DualShock4
  path: /dev/input/js0
  vendor_id: "054c"
  product_id: "09cc"
```

`054c` is Sony's vendor ID. `09cc` is common for the newer DualShock 4
CUH-ZCT2 controller. Older DualShock 4 controllers may report product ID
`05c4`. Running the calibrator against the real `/dev/input/js*` device
overwrites these defaults with the detected values when Linux exposes them.

## Check the Result

```bash
ros2 run jetpilot_teleop_tools joy_calibrator.py test \
  --profile joy_profile.yaml
```

Add `--raw` to show raw axis values next to the normalized tester output.

## Open the Browser Editor

Inside Docker, do not run `open` or `xdg-open`. To start only the Joy Profile
Editor, run:

```bash
/workspaces/tools/app/scripts/start_joy_profile_editor.sh --host 0.0.0.0 --port 8766
```

Then open this URL on the host:

```text
http://127.0.0.1:8766/joy-profile-editor
```

If you want the full JetPilot Console as well, start the existing Console
server instead:

```bash
/workspaces/tools/app/scripts/start.sh --host 0.0.0.0 --port 8765
```

Then open this URL on the host:

```text
http://127.0.0.1:8765/joy-profile-editor
```

In the full Console, the same editor is also available from the `Joy Profile`
tab. It can load a profile YAML file, edit button/axis assignments, export the
full profile YAML, export `teleop_cmd_node` parameters, export
`teleop_button_manager_node` parameters, and run a browser-based Joy Tester
using the Gamepad API when the browser can see the controller.

Capture buttons are available for button mappings, trigger axis/button fields,
stick axes, stick center values, d-pad axes/values, and d-pad direction buttons.
Axis values are captured from the browser Gamepad API and converted to the same
`-32767..32767` style range used in the profile YAML. For sticks, use `Idle Cap`
while not touching the stick to record the center values and estimate a
deadzone from the observed idle noise.

To generate the editor again from an existing profile:

```bash
ros2 run jetpilot_teleop_tools joy_calibrator.py report \
  --profile joy_profile.yaml
```

To generate a blank editor first and load YAML from the browser UI:

```bash
ros2 run jetpilot_teleop_tools joy_calibrator.py ui
```

## Use Generated Parameters

Launch teleop with the generated parameter files:

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_joy:=true \
  enable_teleop:=true \
  teleop_cmd_param:=$(pwd)/teleop_cmd.generated.yaml \
  teleop_button_mapping_param:=$(pwd)/joy_button_mapping.generated.yaml
```

The full `joy_profile.yaml` is intended for calibration and tester tools. The
`teleop_cmd.generated.yaml` and `joy_button_mapping.generated.yaml` files are
the runtime inputs for the current ROS nodes.

When running inside Docker, prefer explicit paths such as `$(pwd)/...` or paths
under the mounted workspace. Host paths and container paths are not the same.
The browser UI can download YAML files, but those downloads go to the browser
environment. For files that ROS launch should read inside Docker, prefer the
CLI-generated YAML paths shown above.
