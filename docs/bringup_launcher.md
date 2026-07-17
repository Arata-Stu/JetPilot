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
/workspaces/scripts/bringup.sh vehicle-pca
/workspaces/scripts/bringup.sh vehicle-vesc

# Joy/teleop/operation plus a vehicle interface
/workspaces/scripts/bringup.sh teleop-pca
/workspaces/scripts/bringup.sh teleop-vesc

# Live camera + localization, without an actuator driver
/workspaces/scripts/bringup.sh localization \
  --map /workspaces/map/course_a

# Full live runtime; autonomous control remains OFF until explicitly requested
/workspaces/scripts/bringup.sh runtime-pca \
  --map /workspaces/map/course_a

# Custom one-shot component selection
/workspaces/scripts/bringup.sh custom \
  --components sensor,joy,teleop,operation,vehicle-vesc

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

The `custom` preset opens a multi-select UI when used interactively. Select every component that
should be ON at once, then confirm the generated launch command. The same selection can be reused
non-interactively with `--components`; available names include `sensor`, `replay`, `localization`,
`bag-manager`, `joy`, `teleop`, `operation`, `control`, `rviz`, `vehicle-pca`, and `vehicle-vesc`.

## Safety behavior

Every preset starts from explicit all-OFF module settings instead of inheriting the relatively broad
defaults of `bringup.launch.py`. Hardware presets require confirmation unless `--yes` is supplied.
The launcher rejects vehicle hardware combined with rosbag replay and rejects both unsafe replay
override flags; intentional HIL tests must use the underlying launch command directly in an isolated
ROS domain.

PCA9685 and VESC select their own package, launch file, and parameter YAML as one unit. The vehicle
camera transform is independent from the actuator interface, so `sensor` and localization presets can
publish `base_link -> realsense_camera_link` without opening either hardware driver.
