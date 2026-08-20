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

# VSLAM-map-aligned HD map and initial-pose input from Foxglove
/workspaces/scripts/bringup.sh custom \
  --components sensor,hd-map,foxglove \
  --map /workspaces/map/course_a
```

Use `--list-presets` for the full list and `--dry-run` to inspect the exact ROS command. A final
launch argument can be overridden without creating duplicates:

```bash
/workspaces/scripts/bringup.sh localization \
  --map /workspaces/map/course_a \
  --dry-run \
  -- enable_rviz:=false enable_hd_map_publisher:=true
```

Vehicle interface and sensor kit choices are loaded dynamically from JSON profiles. Use
`--list-vehicles`, `--list-sensor-kits`, and `--validate-profiles` to inspect them. Adding a new
hardware profile does not require editing `scripts/bringup.sh`; follow
[Bringup profile追加ルール](bringup_profile_rules.md).

The first selector chooses one preset with arrow keys and Enter. For `vehicle`, `teleop`, `drive`,
and `runtime`, the next selector lists the discovered vehicle interface profiles. Presets that enable
live sensors then list the discovered sensor kit profiles in the same way. Existing names such as
`drive-pca` and `drive-vesc` remain accepted as compatibility aliases for scripts, but are not shown
in the TUI.

Select `custom` only when you want to build your own component set. Inside `custom`, press Tab or
Space on each component to turn it ON, then press Enter to confirm all selected components at once.
Selecting `vehicle` opens the vehicle interface selector next. The same selection can be reused
non-interactively with `--components`; available names include `sensor`, `replay`, `localization`,
`occupancy-map`, `hd-map`, `bag-manager`, `foxglove`, `joy`, `teleop`, `operation`, `planning`,
`control`, `rviz`, and `vehicle`. Pass
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

## Foxglove bridge

Foxglove is OFF by default and starts only when the `foxglove` custom component or
`enable_foxglove:=true` is selected. It listens on port `8767`; ports `8765` and `8766` are reserved
for the JetPilot Console and Joy profile editor. The default outbound allowlist contains only
`/tf`, `/tf_static`, `/clock`, topics ending in `/diagnostics`, selected localization state and
HD-map marker/path topics, `/visual_slam/tracking/odometry`, and `/visual_localization/pose`.
Images, point clouds, and OccupancyGrid topics are not exposed. Service and parameter access are
disabled. The bridge's own system monitor is also disabled because
`/jetson/diagnostics` already provides jtop data.
The QoS history cap remains large enough for late-joining clients to receive transforms from
multiple `/tf_static` publishers.

The fixed frame is the Isaac ROS VSLAM `map` frame. The HD map was generated in that same frame, so
no OccupancyGrid is needed. In Foxglove's 3D panel, select `map` as the display frame and enable
`/hd_map/lane_markers`, `/hd_map/section_markers`, and optionally
`/hd_map/primary_centerline_path`. Configure the **2D pose estimate** click-to-publish tool to use
`/initialpose`.
The map directory must contain `<map_dir>/<map_dir_name>_hd_map.yaml`, unless
`hd_map_yaml_path` is set explicitly.

Foxglove Bridge 3.4.x currently parses `client_topic_whitelist` but does not enforce it. With the
`clientPublish` capability enabled for initial-pose input, a connected client can therefore publish
other ROS topics too. Use this only when the client is trusted; the configured `/initialpose`
whitelist is retained for compatibility with a future upstream fix.

Override the port or outbound allowlist with the integrated launch arguments when needed:

```bash
/workspaces/scripts/bringup.sh custom \
  --components hd-map,foxglove \
  --map /workspaces/map/course_a \
  --set foxglove_port:=8877 \
  --set 'foxglove_topic_whitelist:=["^/tf$", "^/tf_static$", "^/hd_map/.*$"]'
```

The default `0.0.0.0` bind address is intended only for a trusted LAN. To avoid exposing the bridge
on the Jetson network interfaces, bind it to loopback and forward the port from the notebook:

```bash
# Jetson
/workspaces/scripts/bringup.sh custom \
  --components sensor,hd-map,foxglove \
  --map /workspaces/map/course_a \
  -- foxglove_address:=127.0.0.1

# Notebook
ssh -N -L 8767:127.0.0.1:8767 jetson@JETSON_IP
```

Foxglove then connects to `ws://localhost:8767`. Without image topics, the expected continuous CPU
cost is small, although it still depends on TF rate, HD-map marker update size, and the number
of connected clients. Check the actual target with jtop in three states: bridge OFF, bridge ON with
no client, and Foxglove connected with the intended panels and topics subscribed. Static HD-map
markers may cause a short transfer spike when their panel first subscribes.

## Safety behavior

Every preset starts from explicit all-OFF module settings instead of inheriting the relatively broad
defaults of `bringup.launch.py`. Hardware presets require confirmation unless `--yes` is supplied.
The launcher rejects vehicle hardware combined with rosbag replay and rejects both unsafe replay
override flags; intentional HIL tests must use the underlying launch command directly in an isolated
ROS domain.

PCA9685 and VESC select their own package, launch file, and parameter YAML as one unit. The vehicle
camera transform is independent from the actuator interface, so `sensor` and localization presets can
publish `base_link -> realsense_camera_link` without opening either hardware driver.
