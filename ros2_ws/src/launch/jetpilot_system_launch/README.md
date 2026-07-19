# jetpilot_system_launch

JetPilot 全体の bringup をまとめる launch package です。tool、operation、planning、control、sensor、localization、vehicle interface を個別に有効化し、topic 名と安全条件を1箇所で揃えます。

## 主な launch

| Launch | 用途 |
| --- | --- |
| `bringup.launch.py` | 実機・offline replay 共通の統合 bringup |
| `tool.launch.py` | joy、teleop、bag manager、VSLAM snapshot recorder |
| `control.launch.py` | `jetpilot_controller` を起動 |
| `localization.launch.py` | Isaac ROS VSLAM/VGL、localization manager、HD map publisher |
| `sensor_kit.launch.py` | camera launch と RTP stream をまとめる |
| `vehicle.launch.py` | vehicle interface と camera static TF |

## Sensor kit variants

`sensor_kit.launch.py` は既定では RealSense を起動します。`sensor_interface_launch` を差し替えると、次の構成を選べます。

| Launch | Sensors |
| --- | --- |
| `launch/sensors/realsense.launch.py` | RealSense |
| `launch/sensors/flir_boson.launch.py` | FLIR Boson via `usb_cam` composable node |
| `launch/sensors/realsense_silky_evcam.launch.py` | RealSense + SilkyEvCam/OpenEB |
| `launch/sensors/realsense_silky_flir.launch.py` | RealSense + SilkyEvCam/OpenEB + FLIR Boson |

FLIR の既定 topic は `/flir/camera_info` と `/flir/image_raw`、既定 frame は `boson_optical_frame` です。デバイスや format は `sensor_kit_flir_video_device:=/dev/video0`、`sensor_kit_flir_pixel_format:=mono16` のように bringup 引数で上書きできます。

## Topic flow

標準的な自律走行の制御 topic は次の流れです。

```text
/hd_map/primary_centerline_path or /planning/raceline_path
  -> /planning/trajectory + /planning/target_speed + /planning/ready
  -> /auto/control_cmd
  -> /vehicle/control_cmd
  -> /control_cmd or vehicle-driver-specific commands
```

手動時は `/joy -> /teleop/control_cmd -> /vehicle/control_cmd`、プロポ入力を Jetson で mux する場合は `/rc/channels -> /propo/control_cmd -> /vehicle/control_cmd` です。どの入力を採用するかは `/operation_mode/state` で決まります。

## Safe rosbag replay

`enable_rosbag_replay:=true` で bag を再生すると、既定では記録済みの制御・mode 系 topic を `/replay/...` に remap します。これにより、bag に入っている `/vehicle/control_cmd` や `/operation_mode/request` が実車の driver へ流れ込むことを防ぎます。

同時に live joy、teleop、RC、operation、自律 control node は安全側で無効化されます。実機 interface と replay を同時に有効化する構成は拒否され、hardware-in-the-loop のような意図的な試験では `allow_unsafe_replay_with_vehicle:=true` と isolated ROS domain を明示します。

## 代表的な起動例

実機の最小 bringup:

```bash
ros2 launch jetpilot_system_launch bringup.launch.py
```

localization、HD map、planning、controller まで有効化する例:

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_localization:=true \
  map_dir:=/workspaces/map/course_a \
  enable_hd_map_publisher:=true \
  enable_section_localizer:=true \
  enable_planning:=true \
  enable_control:=true
```

offline replay:

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_rosbag_replay:=true \
  rosbag:=/workspaces/record/session/take \
  enable_vehicle:=false \
  use_sim_time:=true
```

## Map directory convention

`map_dir` を指定すると、HD map は既定で `<map_dir>/<map_dir_name>_hd_map.yaml`、raceline は `raceline_root` と `raceline_csv` の組み合わせで読みます。明示的に `hd_map_yaml_path` や `raceline_csv` を渡すとこの規約を上書きできます。
