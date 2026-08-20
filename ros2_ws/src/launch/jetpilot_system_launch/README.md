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
| `vehicle.launch.py` | vehicle interface とcamera／EVS／thremoのstatic TF |

## Sensor kit variants

`sensor_kit.launch.py` は既定では RealSense を起動します。`sensor_interface_launch` を差し替えると、次の構成を選べます。

| Launch | Sensors |
| --- | --- |
| `launch/sensors/realsense.launch.py` | RealSense |
| `launch/sensors/oakd_lite.launch.py` | OAK-D Lite RGB + rectified stereo + IMU |
| `launch/sensors/flir_boson.launch.py` | FLIR Boson via `usb_cam` composable node |
| `launch/sensors/realsense_silky_evcam.launch.py` | RealSense + SilkyEvCam/OpenEB |
| `launch/sensors/realsense_silky_flir.launch.py` | RealSense + SilkyEvCam/OpenEB + FLIR Boson |

FLIR の既定 topic は `/flir/camera_info` と `/flir/image_raw`、既定 frame は `boson_optical_frame` です。デバイスや format は `sensor_kit_flir_video_device:=/dev/video0`、`sensor_kit_flir_pixel_format:=mono16` のように bringup 引数で上書きできます。

OAK-D Liteは`depthai_ros_driver_v3`のcomposable componentとして
`multi_sensor_container`へ読み込まれます。ROS depth、point cloud、NN処理は無効で、既定の出力は
次のとおりです。

| Data | Topic | Configuration |
| --- | --- | --- |
| Color image | `/oakd_lite/rgb/image_raw` | 640x400 @ 35 Hz |
| Color camera info | `/oakd_lite/rgb/camera_info` | color imageに追従 |
| Left mono image | `/oakd_lite/left/image_rect` | 640x400 @ 117 Hz、補正済み |
| Left camera info | `/oakd_lite/left/camera_info` | left imageと同一stamp |
| Right mono image | `/oakd_lite/right/image_rect` | 640x400 @ 117 Hz、補正済み |
| Right camera info | `/oakd_lite/right/camera_info` | right imageと同一stamp |
| IMU | `/oakd_lite/imu/data` | accel 480 Hz、gyro 400 Hzを要求 |
| Diagnostics | `/oakd_lite/diagnostics` | DepthAI device状態 |

OAK-D Liteを選択するには
`scripts/bringup.sh sensor --sensor-kit oakd-lite`を使用します。BMI270は要求値を
実機が公開できる最大rateへ丸めるため、IMUの実測publish rateは要求値より低くなる
場合があります。また、Kickstarter版OAK-D Liteの一部にはIMUが搭載されていません。
この構成ではIMU搭載モデルを前提とします。左右画像はDepthAI内で同期され、同一のROS
timestampでpublishされます。`oakd-lite`プロファイルを選ぶと、VSLAM/VGLには
`config/localization/vgl_camera_topics_oakd_lite.yaml`が自動選択され、VSLAMのIMU入力先も
`/oakd_lite/imu/data`へ切り替わります。IMU融合はノイズ値とTFの実機確認前に誤差を
増やさないよう既定OFFです。有効化する場合は`--set vslam_enable_imu:=true`を指定します。

117 Hzの左右非圧縮画像は記録帯域が大きいため、Bag Manager使用時は保存先の書き込み速度と
空き容量を確認してください。

診断topicは発行元ごとに分離されます。主な名前は`/oakd_lite/diagnostics`、
`/realsense/diagnostics`、`/localization/vslam/diagnostics`、
`/localization/vgl/diagnostics`です。Localization Managerは
`/localization/vslam/diagnostics`を購読します。

SilkyEvCam/OpenEBを含むsensor kitでは、RAW記録機能が既定で待機状態になります。Bag Managerを有効にして `/bag/request` へSTARTを送ると、rosbag directoryの生成後に `/event_camera/raw_recording/request` へ同じ開始要求が転送され、MCAP、RAW、`*.raw.metadata.yaml` が同じsession directoryへ保存されます。STOPも同じトリガーでRAW停止要求を先に送ってからrosbagを終了します。カメラ起動直後の自動RAW記録は無効です。

MCAPとRAWのduration分割は`config/tool/bag_manager.param.yaml`の`recording_split_duration_s`で一括指定します。統合launchにはRAW専用の分割引数を公開していないため、異なるdurationは指定できません。`0`は分割無効、例えば`600`は両方を約10分周期で分割します。

Jetson 上の通常起動では `isaac_ros_jetson_stats` が既定で有効になり、診断情報を `/jetson/diagnostics` へ publish します。必要に応じて `enable_jetson_stats:=false` で無効化できます。x86 imageには同packageが配布されないため既定で無効になり、明示的な有効化も拒否します。`scripts/bringup.sh` のオフライン再生プリセットでは tool stack自体を停止するため、Jetson statsも起動しません。

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
