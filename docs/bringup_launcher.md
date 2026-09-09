# Bringup launcher

`scripts/bringup.sh` shortens the long `bringup.launch.py` command and provides the same choices as
named presets or an interactive terminal UI. With no arguments it opens the selector; `fzf` is used
when available and a numbered menu is used otherwise.

Inside the JetPilot Docker workspace:

```bash
/workspaces/scripts/bringup.sh
```

自己位置推定とVSLAMが有効な構成では、TUIの「VSLAM 追跡モード」で
`vo`（ステレオ画像のみ）／`vio`（ステレオ画像 + IMU）を選択できます。
`--vslam-mode vio` または `--set vslam_mode:=vio` を明示した場合は選択画面を省略します。
非対話起動の既定値は `vo` です。VIOでbagを再生する場合は、画像とIMUの両方が必要です。

JetPilotは平坦な床での走行を前提に、全launchでodometryとSLAMの地面制約を
常時有効にします。無効を指定すると起動時にエラーになります。
`create_map.sh`も、地図生成本体に`--override=cuvslam.cfg_planar=true`を渡し、
snapshot再生ではodometryとSLAM両方の地面制約を明示します。
既存地図は変更されないため、この条件で比較するには地図を再生成してください。

RealSenseの既定値はRGB 424×240 / 30 Hz、赤外424×240 / 60 Hzです。
TUIでは引き続き30 / 60 / 90 Hzを選べます。記録済みbagのFPSは変わりません。

`multicam_mode`はIsaac ROSでは0=Moderate、1=Performance、2=Precisionです。
TUIの「VSLAM 処理モード」で選択できます。既定値は1です。
`--set vslam_multicam_mode:=2`を指定すると、この選択画面を省略してPrecisionで起動します。
選択値はlaunchからYAMLより優先して適用されます。VO/VIOを選ぶ`tracking_mode`とは別です。

保存地図で自己位置推定する構成では、TUIの「初期位置の決め方」で次を選択できます。

- **VGL**：画像から初期位置を推定します。
- **Foxglove**：VGLを無効にして、Foxgloveから`/initialpose`へ送る初期位置を使用します。
- **地図原点**：保存地図の原点から開始します。

`--localization-init`や関連する`--set`の明示指定がある場合は選択を省略します。
地図なしの計測・キャリブレーションではこの選択は表示しません。

「走行ライン」では **現在の構成を維持 / Centerline / Raceline / Custom Line** を
選択できます。RacelineとCustom Lineは選択したmap内のCSV候補、または手入力で指定します。
既存の`--raceline`・`--custom-line`・走行ライン用の設定指定は優先します。
ラインの選択によって制御や車両インターフェースを自動で有効にはしません。
`competition`ではRaceline / Custom Lineを`primary`レーンの入力として差し替え、
分岐のレーンID・Sectionルール・信号停止・復帰処理を維持します。
Custom Lineの開閉設定は保存したメタデータ（または明示CLI設定）に従います。

周回コースではHD Mapの`closed_loop: true`を使います。HD map publisherは周回の
Centerline Pathの末尾に始点を補い、点間隔が30cmを超えても制御側が開いた経路と
誤判定しないようにします。`stop_at_path_end`は無効化せず、開いた経路の終端停止は維持します。

TUI で `realsense-silky` または `realsense-silky-flir` を選ぶと、起動確認の前に
`SilkyEvCam bias file` の選択画面を表示します。
`ros2_ws/src/launch/jetpilot_system_launch/config/sensing/silkyevcam/` 内の
`.bias` ファイル（`E522.bias` など）を自動で一覧に表示します。
「読み込まない」はカメラの現在設定を使用し、「パスを手入力...」では別の場所にある
読み取り可能な `.bias` を指定できます。JSON はこの一覧には含めません。
選択したファイルは確認画面の `Silky bias` に表示され、ドライバーが撮影開始前に読み込みます。
Docker 内ではコンテナから見えるパスを指定してください。

CLI の `sensor_kit_silky_evcam_bias_file:=...` による明示指定は TUI より優先され、
バイアス選択画面を省略します。空文字の明示指定も同様です。

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

# Throttle calibration: live sensor + mapless VSLAM odometry + teleop + vehicle + bag manager
/workspaces/scripts/bringup.sh calibration --vehicle jpbb

# Live RealSense RGB + TensorRT E2E direct control + manual/STOP fallback
/workspaces/scripts/bringup.sh e2e --vehicle vesc

# Live camera + localization, without an actuator driver
/workspaces/scripts/bringup.sh localization \
  --map /workspaces/map/course_a

# Start at the saved VSLAM map origin without an external startup hint
/workspaces/scripts/bringup.sh localization \
  --map /workspaces/map/course_a \
  --localization-init map-origin

# Full live runtime; autonomous control remains OFF until explicitly requested
/workspaces/scripts/bringup.sh runtime --vehicle pca \
  --map /workspaces/map/course_a

# Custom one-shot component selection
/workspaces/scripts/bringup.sh custom \
  --components sensor,joy,teleop,operation,vehicle \
  --vehicle vesc

# Map localization + HD map + lane planning + Pure Pursuit
/workspaces/scripts/bringup.sh custom \
  --components sensor,localization,hd-map,foxglove,control,vehicle \
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

The `calibration` preset temporarily reuses the configured localization-trigger button (OPTIONS in
the DualShock 4 profile) as a one-shot `/speed_offset_inc` button. Each press increases
`teleop_cmd_node.throttle_scale` by `throttle_scale_step`. Other presets keep the button assigned to
the localization trigger, and no speed-decrease button is assigned.

The `e2e` preset enables the sensor kit, E2E inference, joy/teleop, the operation command mux, and
the selected vehicle interface. Localization, planning, and the rule-based controller remain OFF.
Override the deployed model when needed with `--set e2e_model_root:=/absolute/path/in/docker`.
The `control` and `e2e` command sources are mutually exclusive because both publish
`/auto/control_cmd`; the wrapper and the underlying integrated launch both reject enabling them
together.

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

## VSLAM initialization mode

`--localization-init` selects the startup localization path. It is a startup setting; changing it
while VSLAM is running requires restarting the localization stack.

| Mode | Startup behavior | VGL | Manual fallback |
| --- | --- | --- | --- |
| `pose-hint` (default) | Localization manager sends a VGL or `/initialpose` hint | Configurable; ON in live presets | Available |
| `foxglove` | Manager waits for Foxglove `/initialpose` immediately | Forced OFF | Required at startup |
| `map-origin` | VSLAM localizes with the saved map's identity pose | Forced OFF | Restart in `pose-hint` mode if origin localization fails |

Use `foxglove` when VGL should not be loaded or triggered:

```bash
/workspaces/scripts/bringup.sh localization \
  --map /workspaces/map/course_a \
  --localization-init foxglove \
  --set enable_hd_map_publisher:=true
```

This mode forces the Foxglove bridge ON, keeps `localize_on_startup=false`, and starts
the manager in `waiting_for_manual`. Send a `map`-frame 2D Pose Estimate to `/initialpose`; it is
validated and forwarded to `/localization/pose_hint`. A direct-child `*.mdb` database in
`<map_dir>/cuvslam_map` is required. It cannot be combined with `vslam_save_map_folder_path`;
use the dedicated mapping workflow when creating a map.

`map-origin` requires a direct child `*.mdb` database in `<map_dir>/cuvslam_map` and VSLAM
localization/mapping mode to be enabled.
Strictly speaking, Isaac ROS implements this as
`localize_on_startup=true`, which uses the identity pose in the `map` frame as an internal hint; no
external `/localization/pose_hint` is sent at startup. The localization manager remains active so
the controller still receives `/localization/pose_hint_state`. It reports `localized` after VSLAM
diagnostics confirm success. During the origin attempt, manual pose and relocalize requests are
blocked to avoid two localization jobs racing. Isaac ROS does not expose cancellation or a definite
completion signal for its detached origin-localization job, so a manager timeout does not unlock
`/initialpose`. It reports `map_origin_restart_required`; restart the localization stack in
`pose-hint` mode before sending a Foxglove pose. A late origin-success diagnostic is still accepted
until the restart. The confirmation timeout starts after the first valid VSLAM localization
diagnostic rather than at Manager process startup. If no valid diagnostic arrives at all, a
separate 120-second readiness watchdog reports the same restart-required state.

`--no-pose-hint` is an alias for `--localization-init map-origin`; `--pose-hint` selects the default
mode explicitly. The older combination `pose-hint --set enable_vgl:=false` remains available, but
`--localization-init foxglove` is the explicit and self-contained form for Foxglove-only startup.

The first selector chooses one preset with arrow keys and Enter. For `vehicle`, `teleop`, `drive`,
`calibration`, `e2e`, and `runtime`, the next selector lists the discovered vehicle interface profiles. Presets that
enable live sensors then list the discovered sensor kit profiles in the same way. Existing names such as
`drive-pca` and `drive-vesc` remain accepted as compatibility aliases for scripts, but are not shown
in the TUI.

Select `custom` only when you want to build your own component set. Inside `custom`, press Tab or
Space on each component to turn it ON, then press Enter to confirm all selected components at once.
Selecting `vehicle` opens the vehicle interface selector next. The same selection can be reused
non-interactively with `--components`; available names include `sensor`, `replay`, `localization`,
`occupancy-map`, `hd-map`, `bag-manager`, `foxglove`, `joy`, `teleop`, `operation`, `planning`,
`control`, `e2e`, `rviz`, and `vehicle`. Pass
`--vehicle pca` or `--vehicle vesc` to override the default `jpbb` when a preset/component enables `vehicle`.
Selecting `control` also enables `planning` and the operation command mux.
Selecting `e2e` enables E2E inference and the operation command mux but leaves the sensor and
vehicle choices explicit. `control` and `e2e` cannot be selected together.
Selecting `planning` alone is useful for validating the selected trajectory without publishing
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

The underlying `bringup.launch.py` argument remains OFF by default. Live `localization-only`,
`localization`/`localize-live`, and `runtime` presets enable Foxglove automatically as a standby
manual-pose path for production. Replay/offline presets keep it OFF, and `custom` enables it only
when the `foxglove` component, `enable_foxglove:=true`, or the explicit
`--localization-init foxglove` mode is selected. It listens on port `8767`;
ports `8765` and `8766` are reserved
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
The live localization presets start the bridge but do not enable the HD-map publisher by
themselves. To see the course outline while placing the pose, add
`--set enable_hd_map_publisher:=true` or select both `hd-map` and `foxglove` in `custom`.

The localization manager accepts both `/visual_localization/pose` from VGL and `/initialpose`
from Foxglove. It validates either pose in the VSLAM `map` frame, waits until Isaac ROS VSLAM is
ready when necessary, and forwards the selected pose to `/localization/pose_hint`. A manual pose
supersedes the pending VGL result, and a late VGL result is ignored. This does not cancel VGL
inference that is already running. Use `--set enable_vgl:=false` when the VGL compute cost itself
must be avoided; the Foxglove manual path remains available. Monitor
`/localization/pose_hint_required` and `/localization/pose_hint_state` in Foxglove to decide when
manual input is required.
Use `--set enable_foxglove:=false` if a regular `pose-hint` live run must not start the standby
bridge. The explicit `--localization-init foxglove` mode requires the bridge and therefore takes
precedence over that generic override.

Replay/offline presets remain bridge-OFF by default. Explicitly selecting
`--localization-init foxglove` during `replay-localization` or `offline-localization` enables live
`/initialpose` input, so that replay is intentionally interactive rather than deterministic.

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

Foxglove then connects to `ws://localhost:8767`. The production presets may leave the bridge process
running with no client connected. Without image topics, its standby and continuous forwarding cost
is expected to be small, although it still depends on TF rate, HD-map marker update size, and the
number of connected clients. Check the actual target with jtop in three states: bridge OFF, bridge ON
with no client, and Foxglove connected with the intended panels and topics subscribed. Static HD-map
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


## 実車調整と既定の車両

TUIの`tuning`は、センサー・自己位置推定・操作系・車両・実車調整サービスを一括起動します。
NotebookのUIでラインと区間速度を選択・適用するため、TUIの走行ライン選択は省略します。
Map選択は`cuvgl_map/`または`cuvslam_map/`のある実体フォルダーを対象にします。

```sh
/workspaces/scripts/bringup.sh tuning --map /workspaces/map/course_a
```

車両を使うpreset/componentで`--vehicle`を省略した場合は`jpbb`を使用します。
TUIでも`jpbb`を先頭に表示します。明示した車両プロファイルや既存の`drive-vesc`等は優先されます。
既定値は`BRINGUP_DEFAULT_VEHICLE`でも変更できます。センサーのみ・localizationのみ・replayでは、
車両は引き続き有効になりません。

更新後は`jetpilot_system_launch`をビルドし直し、setupを読み込んでください。
詳細は[実車調整手順](../tools/app/runtime/README.md)を参照してください。
