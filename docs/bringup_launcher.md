# Bringup launcher

`scripts/bringup.sh` shortens the long `bringup.launch.py` command and provides the same choices as
named presets or an interactive terminal UI. With no arguments it opens the selector; `fzf` is used
when available and a numbered menu is used otherwise.

Inside the JetPilot Docker workspace:

```bash
/workspaces/scripts/bringup.sh
```

Common presets:

```bash
# Vehicle interface only
/workspaces/scripts/bringup.sh vehicle --vehicle pca
/workspaces/scripts/bringup.sh vehicle --vehicle vesc

# Joy/teleop/operation plus a vehicle interface
/workspaces/scripts/bringup.sh teleop --vehicle pca
/workspaces/scripts/bringup.sh teleop --vehicle vesc

# Live sensor plus joy/teleop/operation and vehicle interface
/workspaces/scripts/bringup.sh drive --vehicle pca
/workspaces/scripts/bringup.sh drive --vehicle vesc

# Live camera + localization, without an actuator driver
/workspaces/scripts/bringup.sh localization \
  --map /workspaces/map/course_a

# Full live runtime; autonomous control remains OFF until explicitly requested
/workspaces/scripts/bringup.sh runtime --vehicle pca \
  --map /workspaces/map/course_a

# Custom one-shot component selection
/workspaces/scripts/bringup.sh custom \
  --components sensor,joy,teleop,operation,vehicle \
  --vehicle vesc

# Map localization + HD map + lane planning + Pure Pursuit
/workspaces/scripts/bringup.sh custom \
  --components sensor,localization,hd-map,control,vehicle \
  --vehicle vesc \
  --map /workspaces/map/course_a \
  --raceline /workspaces/map/course_a/course_a_raceline.csv

# Safe offline localization replay
/workspaces/scripts/bringup.sh replay-localization \
  --bag /workspaces/record/run_01 \
  --map /workspaces/map/course_a \
  --rate 0.5
```

Use `--list-presets` for the full list and `--dry-run` to inspect the exact ROS command. A final
launch argument can be overridden without creating duplicates:

```bash
/workspaces/scripts/bringup.sh localization \
  --map /workspaces/map/course_a \
  --dry-run \
  -- enable_rviz:=false enable_hd_map_publisher:=true
```

The first selector chooses one preset with arrow keys and Enter. For `vehicle`, `teleop`, `drive`,
and `runtime`, the next selector chooses the vehicle interface (`pca` or `vesc`). Presets that enable
live sensors then open the sensor kit selector in the same way. Existing names such as `drive-pca`
and `drive-vesc` remain accepted as compatibility aliases for scripts, but are not shown in the TUI.

Select `custom` only when you want to build your own component set. Inside `custom`, press Tab or
Space on each component to turn it ON, then press Enter to confirm all selected components at once.
Selecting `vehicle` opens the vehicle interface selector next. The same selection can be reused
non-interactively with `--components`; available names include `sensor`, `replay`, `localization`,
`bag-manager`, `joy`, `teleop`, `operation`, `planning`, `control`, `rviz`, and `vehicle`. Pass
`--vehicle pca` or `--vehicle vesc` when a non-interactive preset/component enables `vehicle`.
Selecting `control` also enables `planning` and the operation command mux;
selecting `planning` alone is useful for validating the selected trajectory without publishing
autonomous commands. Passing `--raceline` enables the C++ CSV publisher and automatically selects
the matching two-lane planner configuration; the CSV path must be absolute inside Docker.
When `localization` is selected interactively, the launcher asks for a map next. In non-interactive
commands, pass the map explicitly:

```bash
/workspaces/scripts/bringup.sh custom \
  --components sensor,localization,vehicle \
  --vehicle vesc \
  --map /workspaces/map/course_a
```

The interactive map selector searches `MAP_ROOT` (`/workspaces/map` by default). It offers
`/workspaces/map/latest`, direct child directories such as `/workspaces/map/course_a`, and parents
of discovered `cuvgl_map` or `cuvslam_map` directories. If there is exactly one candidate, it is
selected automatically.

## Safety behavior

Every preset starts from explicit all-OFF module settings instead of inheriting the relatively broad
defaults of `bringup.launch.py`. Hardware presets require confirmation unless `--yes` is supplied.
The launcher rejects vehicle hardware combined with rosbag replay and rejects both unsafe replay
override flags; intentional HIL tests must use the underlying launch command directly in an isolated
ROS domain.

PCA9685 and VESC select their own package, launch file, and parameter YAML as one unit. The vehicle
camera transform is independent from the actuator interface, so `sensor` and localization presets can
publish `base_link -> realsense_camera_link` without opening either hardware driver.
