# jetpilot_system_launch

JetPilot 全体の bringup をまとめる launch package です。tool、operation、planning、control、sensor、localization、vehicle interface を個別に有効化し、topic 名と安全条件を1箇所で揃えます。

## 主な launch

| Launch | 用途 |
| --- | --- |
| `bringup.launch.py` | 実機・offline replay 共通の統合bringupと独立したFoxglove bridge |
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

## VSLAM初期位置モード

`bringup.sh`の`--localization-init`で、VSLAM起動時の初期位置推定経路を選択できます。

| mode | 起動時の動作 | VGL | 手動fallback |
| --- | --- | --- | --- |
| `pose-hint`（既定） | VGLまたは`/initialpose`をManagerからVSLAMへ送る | 設定に従う。実機presetではON | 使用可能 |
| `map-origin` | 保存VSLAM mapの原点から推定する | 強制OFF | 失敗時は`pose-hint`で再起動 |

```bash
# VGL→Foxglove fallbackを使う既定動作
/workspaces/scripts/bringup.sh localization \
  --map /workspaces/map/course_a \
  --localization-init pose-hint

# 外部pose hintを送らず、保存mapの原点から開始
/workspaces/scripts/bringup.sh localization \
  --map /workspaces/map/course_a \
  --localization-init map-origin
```

`map-origin`には`<map_dir>/cuvslam_map`直下の`*.mdb` databaseとVSLAMの
localization/mapping modeが必要です。Isaac ROS内部では
`localize_on_startup=true`として実装され、`map` frameのidentity poseを内部hintとして探索します。
起動時に外部`/localization/pose_hint`はpublishしません。Managerは停止せずVSLAM diagnosticsを監視し、
成功後にcontroller向けの`localized` stateをpublishします。原点探索中は競合防止のため
`/initialpose`とrelocalize要求を受け付けません。Isaac ROSは別threadで動く原点探索のcancelや明確な
完了signalを公開していないため、ManagerのtimeoutだけではFoxglove入力を解禁しません。
`map_origin_restart_required`を通知したら、`pose-hint` modeでlocalization stackを再起動してから
`/initialpose`を送ります。再起動前に遅れて成功diagnosticsが届いた場合は成功として受理します。
timeoutの計測はManager起動時ではなく、最初の有効なVSLAM localization diagnosticsを受けてから開始します。
有効なdiagnosticsが一度も届かない場合は、別の120秒readiness watchdogが同じ再起動要求を通知します。
この切替は起動時設定なので、走行中の変更にはlocalization stackの再起動が必要です。

`--no-pose-hint`は`--localization-init map-origin`、`--pose-hint`は`pose-hint`のaliasです。
VGLを使わず最初からFoxgloveの手動poseを待つ場合は、`pose-hint`のまま
`--set enable_vgl:=false`を追加します。

## Foxglove bridge

`bringup.launch.py`単体の`enable_foxglove`は既定OFFです。一方、実機用の
`localization-only`、`localization`／`localize-live`、`runtime` presetでは、VGLが使えない
場合の手動pose入力を常に確保するためFoxglove bridgeを待機起動します。replay／offline presetは
OFFのままです。`custom`では`foxglove` componentまたは`enable_foxglove:=true`を明示した場合だけ
起動します。既定portは`8767`です。JetPilot Consoleの
`8765`とJoy profile editorの`8766`を避けています。VSLAMのmap座標系に揃えたHD map表示と
initial pose指定だけを含む
最小構成は次のとおりです。

```bash
/workspaces/scripts/bringup.sh custom \
  --components sensor,hd-map,foxglove \
  --map /workspaces/map/course_a
```

既定のoutbound whitelistは`/tf`、`/tf_static`、`/clock`、末尾が`/diagnostics`のtopic、
選択したlocalization stateとHD mapのmarker/path topic、
VSLAM odometryの`/visual_slam/tracking/odometry`、VGL poseの
`/visual_localization/pose`だけです。画像、point cloud、OccupancyGridのtopicは含めません。
service呼び出しとparameter操作は無効です。Foxglove bridge独自のsysinfoも、jtop由来の
`/jetson/diagnostics`と重複するため`false`にしています。
後から接続したclientでも複数の`/tf_static` publisherからTFを受け取れるよう、QoS historyの
上限は`25`を維持します。

基準frameはIsaac ROS VSLAMの`map`です。HD mapも同じ座標系から生成されているため、
OccupancyGridは使用しません。Foxgloveの3D panelではdisplay frameを`map`にし、
`/hd_map/lane_markers`、`/hd_map/section_markers`、必要なら
`/hd_map/primary_centerline_path`を表示します。initial poseはclick-to-publishの
**2D pose estimate**を選び、publish先を`/initialpose`に設定します。
map directoryには既定で`<map_dir>/<map_dir_name>_hd_map.yaml`が必要です。別の場所にある場合は
`hd_map_yaml_path`を明示します。
実機用localization presetが自動で起動するのはbridgeまでで、HD map publisherは自動では有効に
なりません。コース外形を見ながらposeを置く場合は`--set enable_hd_map_publisher:=true`を追加するか、
`custom`で`hd-map`と`foxglove`を両方選択します。

初期位置推定では、VGLの`/visual_localization/pose`とFoxgloveの`/initialpose`を
localization managerが受け、VSLAMの`map` frameとして検証した後、
`/localization/pose_hint`へ転送します。VSLAMの準備前にposeを受けた場合は準備完了まで保持します。
手動poseは保留中のVGL結果より優先され、後から届いたVGL結果は無視されます。ただし、すでに開始した
VGL推論の計算処理そのものは中断しません。VGLの計算負荷自体を避ける場合は
`--set enable_vgl:=false`を指定し、Foxgloveの手動経路だけを使用します。
`/localization/pose_hint_required`と`/localization/pose_hint_state`を確認すると、手動入力が必要な
状態を判断できます。
特定の実機runでbridgeを待機させたくない場合は`--set enable_foxglove:=false`で停止できます。

Foxglove Bridge 3.4.xには、`client_topic_whitelist`を読み込んでもclient publish時に適用しない
上流不具合があります。initial pose入力のため`clientPublish`を有効にしているので、接続clientは
実際には他のROS topicにもpublishできます。信頼できるclientだけで使用してください。
`/initialpose`の設定値は将来の上流修正に備えて残しています。

portとoutbound whitelistはbringup引数で変更できます。

```bash
/workspaces/scripts/bringup.sh custom \
  --components hd-map,foxglove \
  --map /workspaces/map/course_a \
  --set foxglove_port:=8877 \
  --set 'foxglove_topic_whitelist:=["^/tf$", "^/tf_static$", "^/hd_map/.*$"]'
```

既定の`foxglove_address:=0.0.0.0`は信頼できるLAN内だけで使用してください。bridgeを
Jetsonのnetwork interfaceへ公開しない場合はloopbackへbindし、notebookからSSH tunnelを
使用します。

```bash
# Jetson
/workspaces/scripts/bringup.sh custom \
  --components sensor,hd-map,foxglove \
  --map /workspaces/map/course_a \
  -- foxglove_address:=127.0.0.1

# Notebook
ssh -N -L 8767:127.0.0.1:8767 jetson@JETSON_IP
```

この場合はFoxgloveから`ws://localhost:8767`へ接続します。実機presetではclient未接続でもbridge
processが待機します。画像を購読しない構成では待機中および接続後の継続的なCPU負荷は小さい想定ですが、
TFのrate、HD map markerの更新サイズ、接続client数に依存します。
実機ではjtopを使い、bridge OFF、bridge ONかつ未接続、Foxglove接続後に実際のpanelとtopicを
購読した状態の3条件を比較してください。静的HD map markerはpanelの購読開始時に一時的な転送負荷が
発生することがあります。

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
