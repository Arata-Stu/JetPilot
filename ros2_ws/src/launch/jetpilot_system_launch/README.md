# jetpilot_system_launch

JetPilot 全体の bringup をまとめる launch package です。tool、operation、planning、control、E2E inference、sensor、localization、vehicle interface を個別に有効化し、topic 名と安全条件を1箇所で揃えます。

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
| `launch/sensors/flir_boson.launch.py` | FLIR Boson via `usb_cam` composable node |
| `launch/sensors/realsense_silky_evcam.launch.py` | RealSense + SilkyEvCam/OpenEB |
| `launch/sensors/realsense_silky_flir.launch.py` | RealSense + SilkyEvCam/OpenEB + FLIR Boson |

FLIR の既定 topic は `/flir/camera_info` と `/flir/image_raw`、既定 frame は `boson_optical_frame` です。デバイスや format は `sensor_kit_flir_video_device:=/dev/video0`、`sensor_kit_flir_pixel_format:=mono16` のように bringup 引数で上書きできます。

## VSLAM tracking mode

VSLAMは`vo`（画像のみ、既定）と`vio`（画像＋IMU）を切り替えられます。

```bash
scripts/bringup.sh localization --map /workspaces/map/course_a --vslam-mode vo
scripts/bringup.sh localization --map /workspaces/map/course_a --vslam-mode vio
```

launchを直接起動する場合は`vslam_mode:=vo|vio`を指定します。RealSense D455はaccelとgyroを
線形補間で統合して`/realsense/imu`へpublishし、VIOはこのtopicを購読します。

D455の車両取付TFは`base_link -> realsense_camera_link`で、RGB光学中心を車両中心軸へ
合わせる公称横オフセットとして`y=+0.0115 m`を使用します。実機の取付方向が逆の場合は
`vehicle_description_camera_y:=-0.0115`へ反転してください。

診断topicは発行元ごとに分離されます。主な名前は`/realsense/diagnostics`、`/localization/vslam/diagnostics`、
`/localization/vgl/diagnostics`です。Localization Managerは
`/localization/vslam/diagnostics`を購読します。

SilkyEvCam/OpenEBを含むsensor kitでは、RAW記録機能が既定で待機状態になります。Bag Managerを有効にして `/bag/request` へSTARTを送ると、rosbag directoryの生成後に `/event_camera/raw_recording/request` へ同じ開始要求が転送され、MCAP、RAW、`*.raw.metadata.yaml` が同じsession directoryへ保存されます。STOPも同じトリガーでRAW停止要求を先に送ってからrosbagを終了します。カメラ起動直後の自動RAW記録は無効です。

MCAPとRAWのduration分割は`config/tool/bag_manager.param.yaml`の`recording_split_duration_s`で一括指定します。統合launchにはRAW専用の分割引数を公開していないため、異なるdurationは指定できません。`0`は分割無効、例えば`600`は両方を約10分周期で分割します。

Jetson 上の通常起動では `isaac_ros_jetson_stats` が既定で有効になり、診断情報を `/jetson/diagnostics` へ publish します。必要に応じて `enable_jetson_stats:=false` で無効化できます。x86 imageには同packageが配布されないため既定で無効になり、明示的な有効化も拒否します。`scripts/bringup.sh` のオフライン再生プリセットでは tool stack自体を停止するため、Jetson statsも起動しません。

## E2E direct control

RealSense RGBをTensorRT E2Eへ入力して直接制御する場合は、専用presetを使用します。

```bash
/workspaces/scripts/bringup.sh e2e --vehicle vesc
```

このpresetはsensor kit、E2E inference、joy/teleop、operation mux、指定したvehicle interfaceを
有効にします。localization、planning、従来controllerはOFFのままです。モデルを変更する場合は
`--set e2e_model_root:=/workspaces/ros2_ws/models/e2e/<model>`を追加します。

`custom`では`e2e` componentを選択できます。`control`と`e2e`はどちらも
`/auto/control_cmd`へpublishするため併用できません。`scripts/bringup.sh`と統合
`bringup.launch.py`の両方が同時有効化を起動前に拒否します。

## 軽量物体検出（任意）

`enable_object_detection:=true`で、224x224 YOLOv8 TensorRTとC++ decoderをsensor kitと
同じcomponent containerへ読み込みます。既定OFFで、depthは使用しません。モデルは
`/workspaces/ros2_ws/models/yolov8/latest/model.onnx`（engine生成後は`model.plan`）を参照します。
学習・再学習・export・アノテーション規約は
`python_ws/jetpilot_object_detection_training`のREADMEを参照してください。ROS packageは
TensorRT runtimeとC++ decoderだけを担当します。
走行時に検出器を有効化していなかったbagも、ConsoleのBag Analysisから後日TensorRT推論し、
検出sidecarとoverlayを生成できます。元bagは書き換えません。

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_sensor_kit:=true enable_localization:=true \
  enable_object_detection:=true
```

## VSLAM初期位置モード

`bringup.sh`の`--localization-init`で、VSLAM起動時の初期位置推定経路を選択できます。

| mode | 起動時の動作 | VGL | 手動fallback |
| --- | --- | --- | --- |
| `pose-hint`（既定） | VGLまたは`/initialpose`をManagerからVSLAMへ送る | 設定に従う。実機presetではON | 使用可能 |
| `foxglove` | 最初からFoxgloveの`/initialpose`を待つ | 強制OFF | 起動時に必須 |
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

# VGLと原点推定を使わず、最初からFoxglove poseを待つ
/workspaces/scripts/bringup.sh localization \
  --map /workspaces/map/course_a \
  --localization-init foxglove \
  --set enable_hd_map_publisher:=true
```

`foxglove` modeはVGLを読み込まず、`localize_on_startup=false`のままManagerを
`waiting_for_manual`へ移行させ、Foxglove bridgeを強制ONにします。Foxgloveの2D Pose Estimateを
`map` frame・`/initialpose`へ送ると、検証後に`/localization/pose_hint`へ転送します。
`<map_dir>/cuvslam_map`直下の`*.mdb`が必要です。`vslam_save_map_folder_path`とは併用できないため、
map生成時は専用のmapping workflowを使用してください。

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
従来の`pose-hint --set enable_vgl:=false`も使用できますが、Foxgloveだけで開始する意図を
明示する場合は`--localization-init foxglove`を使用してください。

## Foxglove bridge

`bringup.launch.py`単体の`enable_foxglove`は既定OFFです。一方、実機用の
`localization-only`、`localization`／`localize-live`、`runtime` presetでは、VGLが使えない
場合の手動pose入力を常に確保するためFoxglove bridgeを待機起動します。replay／offline presetは
OFFのままです。`custom`では`foxglove` componentまたは`enable_foxglove:=true`を明示した場合だけ
起動します。ただし、明示的に`--localization-init foxglove`を選んだ場合はpresetに関係なく
bridgeを起動します。既定portは`8767`です。JetPilot Consoleの
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
通常の`pose-hint`実機runでbridgeを待機させたくない場合は
`--set enable_foxglove:=false`で停止できます。`--localization-init foxglove`はbridgeが必須のため、
この汎用overrideより優先されます。

replay／offline presetは既定ではbridgeを起動しません。`replay-localization`または
`offline-localization`で明示的に`--localization-init foxglove`を選ぶとliveの
`/initialpose`入力が有効になるため、そのrunは再現性を目的としたreplayではなく対話的な
デバッグになります。

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
/hd_map/primary_centerline_path or /planning/{raceline,custom}_trajectory
  -> /planning/trajectory + /planning/trajectory_profile
  -> /planning/target_speed + /planning/ready
  -> /auto/control_cmd
  -> /vehicle/control_cmd
  -> /control_cmd or vehicle-driver-specific commands
```

E2E direct controlでは、RealSense RGBから同じoperation muxへ接続します。

```text
/realsense/color/image_raw
  -> E2E TensorRT inference
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

`map_dir` を指定すると、HD map は既定で `<map_dir>/<map_dir_name>_hd_map.yaml`、raceline は `raceline_root` と `raceline_csv` の組み合わせで読みます。名前付きcustom lineは`--custom-line`、開路は`--custom-line-open`で指定します。`custom` presetで`custom-line` componentを選び、Map内にConsoleで有効化した`<map_name>_custom_line.csv`とmetadataがあれば、そのlineを自動選択します。明示pathは常に優先されます。racelineとcustom lineは同時には選択できません。選択はlaunch時に読み込まれ、走行中のhot-swapは行いません。
