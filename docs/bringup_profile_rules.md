# Bringup profile追加ルール

JetPilotのvehicle interfaceとsensor kitは、`scripts/bringup.sh`内のcase文ではなく、
JSON形式のbringup profileから動的に検出する。新しいhardware構成を追加するときは、
共通launch契約を満たすlaunchファイルとprofile manifestを追加する。

profileを追加すると、次の箇所へ自動的に反映される。

- `scripts/bringup.sh`のTUI選択肢
- `--list-vehicles`または`--list-sensor-kits`
- `--vehicle <profile-id>`または`--sensor-kit <profile-id>`
- profile固有のpackage、launch、parameter、RTP topic候補

`scripts/bringup.sh`自体へprofile名やpackage名を追加してはいけない。

## Profileの配置

manifestは次のディレクトリへ配置する。

```text
ros2_ws/src/launch/jetpilot_system_launch/config/bringup_profiles/
├── vehicle/
│   └── <profile-id>.json
└── sensor_kit/
    └── <profile-id>.json
```

`jetpilot_system_launch`の`config/`はROS packageのshare directoryへインストールされる。
source workspaceでprofileを追加したあとは、manifestの検証と通常のworkspace buildを
行う。

一時的に別のprofile directoryを検証する場合は、`BRINGUP_PROFILE_ROOT`で探索先を
差し替えられる。

```bash
BRINGUP_PROFILE_ROOT=/workspaces/my_profiles \
  /workspaces/scripts/bringup.sh --list-vehicles
```

## 命名規則

profile IDには、小文字英数字とハイフンだけを使用する。

```text
^[a-z0-9][a-z0-9-]*$
```

ファイル名は必ず`<profile-id>.json`と一致させる。

```text
正しい例:
  vesc.json
  realsense-silky.json
  front-camera-lidar.json

使用しない例:
  VESC.json
  realsense_silky.json
  front camera.json
```

新しいaliasにもハイフン区切りを使用する。既存コマンドとの互換性が必要な場合に限り、
underscoreを含むaliasを使用できる。profile IDとaliasは、同じ種類のprofile内で
重複してはいけない。

ROS package名はROS 2の規則に従い、小文字英数字とunderscoreを使用する。推奨形式は
次のとおり。

```text
vehicle:
  jetpilot_<hardware>_interface
  <hardware>_interface
  <vehicle-name>_vehicle_interface

sensor kit:
  jetpilot_<sensor-set>_sensor_kit
  <sensor-set>_sensor_kit
  <hardware>_sensor_interface
```

## 共通Manifest項目

すべてのmanifestは次の項目を持つ。

| 項目 | 必須 | 内容 |
|---|---:|---|
| `schema_version` | yes | 現在は`1` |
| `kind` | yes | `vehicle`または`sensor_kit` |
| `id` | yes | profile ID。ファイル名と一致させる |
| `label` | yes | TUIに表示する名称 |
| `order` | no | TUIの表示順。既定値は`100` |
| `aliases` | no | CLI互換用の別名 |
| `launch.package` | yes | launchを提供するROS package |
| `launch.file` | yes | package share基準の`launch/*.launch.py`または`.xml` |
| `arguments` | no | `bringup.launch.py`へ渡すprofile固有引数 |

JSONの値は設定値としてのみ扱われる。shell command、command substitution、任意の
Pythonコードは記述できない。vehicle profileの`arguments`には`vehicle_control_topic`、
`vehicle_description_*`、`publish_vehicle_*`だけを指定できる。sensor kit profileでは
`sensor_kit_*`だけを指定でき、interface packageとlaunch自体は`launch`項目で指定する。

## Vehicle interfaceの追加

### 1. 共通launch契約を満たす

vehicle interface launchは、少なくとも次の引数を宣言して受け取れるようにする。

```text
vehicle_control_topic
driver_param
publish_description
description_base_frame
description_camera_frame
description_camera_x
description_camera_y
description_camera_z
description_camera_roll
description_camera_pitch
description_camera_yaw
use_sim_time
```

JetPilotからの正規化された制御入力は、`vehicle_control_topic`で受け取る。既定topicは
`/vehicle/control_cmd`である。PWM、VESC eRPM、CAN、serialなどへの変換は個別package
側へ閉じ込める。

launchファイルの基本形は次のようになる。

```text
<vehicle-interface-package>/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── <profile-id>_interface.launch.xml
└── config/
    └── <profile-id>.param.yaml
```

### 2. Vehicle profileを追加する

例:

```json
{
  "schema_version": 1,
  "kind": "vehicle",
  "id": "my-can",
  "label": "My CAN vehicle interface",
  "order": 30,
  "aliases": [],
  "launch": {
    "package": "jetpilot_my_can_interface",
    "file": "launch/my_can_interface.launch.xml"
  },
  "driver_param": {
    "package": "jetpilot_my_can_interface",
    "path": "config/my_can.param.yaml",
    "workspace_override": "my_can.param.yaml"
  },
  "arguments": {
    "publish_vehicle_description": true,
    "publish_vehicle_evs_description": false,
    "publish_vehicle_thremo_description": false
  }
}
```

`driver_param`はvehicle profileで必須である。

| 項目 | 内容 |
|---|---|
| `package` | parameter YAMLを提供するpackage |
| `path` | package share基準の相対パス |
| `workspace_override` | 任意。`ros2_ws/joy_profiles/`以下で優先するファイル |

parameterは次の順番で解決される。

1. `ROS2_WS/joy_profiles/<workspace_override>`
2. project側`ros2_ws/joy_profiles/<workspace_override>`
3. source workspace内の対象package
4. install space内の対象package

`arguments`にはvehicle descriptionの有効・無効やprofile固有の共通bringup引数を記述
する。package名、launch名、parameterパスを`scripts/bringup.sh`へ追加しない。

### 3. 動作確認

```bash
/workspaces/scripts/bringup.sh --validate-profiles
/workspaces/scripts/bringup.sh --list-vehicles
/workspaces/scripts/bringup.sh vehicle --vehicle my-can --dry-run
/workspaces/scripts/bringup.sh drive --vehicle my-can --dry-run
```

## Sensor kit launchの追加

sensor kitは単一sensorだけでなく、camera、IMU、LiDAR、event cameraなどを組み合わせた
一つの起動構成として扱える。

### 1. 共通launch契約を満たす

sensor kit launchは`launch/sensor_kit.launch.py`からincludeされる。新しいlaunchは、
共通launchから渡される引数を宣言し、使用しない引数は安全に無視する。

主な共通引数は次のとおり。

```text
camera_name
container_name
run_standalone
enable_depth
enable_color
enable_rtp_stream
rtp_image_topic
rtp_host
rtp_port
rtp_codec
rtp_fps
rtp_bitrate
rtp_gop
rtp_mtu
rtp_payload
rtp_encoder
rtp_enable_status_log
use_sim_time
```

現在の共通launchは、FLIR BosonおよびSilkyEvCam/OpenEB用の引数も渡す。新しいwrapperを
作るときは、既存の
`ros2_ws/src/launch/jetpilot_system_launch/launch/sensors/`以下のlaunchを雛形にし、
`sensor_kit.launch.py`が渡す引数との互換性を保つ。

独自sensor driverの引数名やtopic名はwrapper launch内で共通引数へ変換する。
localizationやrecordingが利用するframe名・image topic・IMU topicは、packageのREADME
にも記載する。

### 2. Sensor kit profileを追加する

例:

```json
{
  "schema_version": 1,
  "kind": "sensor_kit",
  "id": "front-camera-lidar",
  "label": "Front camera + LiDAR",
  "order": 50,
  "aliases": [],
  "launch": {
    "package": "front_camera_lidar_sensor_kit",
    "file": "launch/front_camera_lidar.launch.py"
  },
  "arguments": {
    "sensor_kit_camera_name": "front",
    "sensor_kit_rtp_image_topic": "/front/image_raw"
  },
  "rtp_topics": [
    "/front/image_raw",
    "/front/image_rect"
  ]
}
```

Sensor kit profileの`arguments`には、`bringup.launch.py`の引数名を記述する。そのため
profile側では`camera_name`ではなく`sensor_kit_camera_name`、
`rtp_image_topic`ではなく`sensor_kit_rtp_image_topic`を使用する。

`rtp_topics`はTUIのRTP image topic候補として表示される。実際にpublishされる絶対topic
名だけを記載し、先頭を`/`にする。RTP非対応のsensor kitでは空配列にできる。

### 3. 動作確認

```bash
/workspaces/scripts/bringup.sh --validate-profiles
/workspaces/scripts/bringup.sh --list-sensor-kits
/workspaces/scripts/bringup.sh sensor \
  --sensor-kit front-camera-lidar \
  --dry-run
/workspaces/scripts/bringup.sh drive \
  --vehicle vesc \
  --sensor-kit front-camera-lidar \
  --dry-run
```

対話TUIでもprofileが表示され、sensor kit選択後のRTP topic候補に`rtp_topics`が反映
されることを確認する。

## Manifest検証

すべてのprofileを検証する。

```bash
./scripts/bringup.sh --validate-profiles
```

loaderを直接使う場合:

```bash
python3 scripts/launch_profiles.py \
  --root ros2_ws/src/launch/jetpilot_system_launch/config/bringup_profiles \
  validate
```

検証では、schema version、未知のfield、IDとファイル名の不一致、重複ID・alias、不正な
package名、危険な相対パス、重複RTP topicなどを拒否する。

## 追加時のチェックリスト

### Vehicle

- interface packageが`/vehicle/control_cmd`相当の入力を受け取れる
- 共通vehicle launch引数を宣言している
- driver parameterをpackageの`config/`へ配置した
- `<profile-id>.json`を`vehicle/`へ追加した
- profile内にpackage、launch、driver parameter、description設定を記載した
- `--validate-profiles`と`--dry-run`が成功する
- 実機起動前にdriver parameterとdevice pathを確認した

### Sensor kit

- sensor kit launchが共通sensor launch引数を宣言している
- frame名とpublish topicをREADMEへ記載した
- `<profile-id>.json`を`sensor_kit/`へ追加した
- `sensor_kit_*`形式のbringup引数をmanifestへ記載した
- RTP対応時は実在するimage topicを`rtp_topics`へ記載した
- `--validate-profiles`と`--dry-run`が成功する
- TUIに表示され、RTP topic候補が正しいことを確認した

## Presetとの境界

`vehicle`、`teleop`、`drive`、`runtime`などのpresetは「何を起動するか」を表す。
vehicle profileとsensor kit profileは「どのhardware構成を使うか」を表す。

新しいhardwareを追加するために`drive-my-can`や`runtime-my-can`のようなpresetを増やして
はいけない。既存presetとprofileを組み合わせる。

```bash
./scripts/bringup.sh runtime \
  --vehicle my-can \
  --sensor-kit front-camera-lidar \
  --map /workspaces/map/course_a
```
