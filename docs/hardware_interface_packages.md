# Hardware Interface Package Design

JetPilot は、車体ごとに異なる hardware 依存部分を個別 package として切り出す。
system launch 側は「どの package のどの launch を読むか」だけを知り、実際の
driver node、parameter、frame、topic の詳細は個別 package 側に閉じ込める。

この方針は vehicle interface と sensor interface の両方で揃える。

## 基本方針

- `jetpilot_system_launch` は共通の起動入口を持つ。
- hardware 固有の driver、adapter、parameter、description は個別 package に置く。
- 個別 package は `packages.repos` で取り込める単位にする。
- 共通 launch から渡される引数名はできるだけ揃える。
- JetPilot の共通 topic 名に合わせる変換は、個別 package 側で行う。

## 配置

repository 内で直接管理する場合も、外部 repository から取り込む場合も、ROS 2
workspace 内の分類は次のように揃える。

```text
ros2_ws/src/
  vehicle/
    <vehicle_interface_package>/
  sensor/
    <sensor_interface_package>/
```

既存の vehicle package は `ros2_ws/src/vehicle/` 配下に置く。sensor package も
同じ考え方で `ros2_ws/src/sensor/` 配下に置く。

外部 repository として管理する場合は、`packages.repos` に追加する。

```yaml
repositories:
  ros2_ws/src/sensor/<sensor_interface_package>:
    type: git
    url: https://github.com/<owner>/<sensor_interface_package>.git
    version: <tag-or-branch>
```

## 名前の付け方

package 名は「対象 hardware」または「車両構成」が分かる名前にする。

vehicle interface:

```text
<hardware>_interface
jetpilot_<hardware>_interface
<vehicle_name>_vehicle_interface
```

例:

```text
pca9685_rc_driver
jetpilot_vesc_interface
tt02_vehicle_interface
```

sensor interface:

```text
<sensor_set>_sensor_kit
<hardware>_sensor_interface
jetpilot_<hardware>_sensor_interface
```

例:

```text
realsense_sensor_kit
zed_sensor_interface
dual_camera_sensor_kit
```

単体 sensor driver だけではなく、複数 sensor、frame、remap、同期設定をまとめる
場合は `sensor_kit` という名前を使う。

## Package 内の構造

個別 package は、最低限次の構造を持つ。

```text
<interface_package>/
  package.xml
  CMakeLists.txt
  launch/
    <interface>.launch.py
  config/
    <driver_or_interface>.param.yaml
```

必要に応じて次を追加する。

```text
  urdf/
  rviz/
  scripts/
  src/
  include/
```

driver をそのまま起動するだけなら `launch/` と `config/` 中心でよい。
JetPilot の共通 message や topic へ変換が必要な場合は、adapter node を同じ package
に置く。

## Vehicle Interface

vehicle interface package は、JetPilot の制御出力を実車の actuator driver に接続する。

共通 launch からは次の引数で差し替える。

```bash
vehicle_interface_pkg:=<package_name>
vehicle_interface_launch:=launch/<launch_file>
```

個別 package 側の launch は、少なくとも次の引数を受け取れるようにする。

```text
vehicle_control_topic
driver_param
publish_description
description_base_frame
description_camera_frame
use_sim_time
```

必要に応じて、camera 位置の引数も受け取る。

```text
description_camera_x
description_camera_y
description_camera_z
description_camera_roll
description_camera_pitch
description_camera_yaw
```

JetPilot 側の標準入力 topic は次を使う。

```text
/vehicle/control_cmd
```

driver 固有の topic 名、serial port、PWM range、VESC 設定などは個別 package の
parameter に閉じ込める。

## Sensor Interface

sensor interface package は、カメラ、IMU、LiDAR などの sensor driver と、それに必要な
frame、remap、parameter をまとめる。

共通 launch からは次の引数で差し替える。

```bash
sensor_kit_interface_pkg:=<package_name>
sensor_kit_interface_launch:=launch/<launch_file>
```

`sensor_kit.launch.py` を直接使う場合は、次の名前で差し替える。

```bash
sensor_interface_pkg:=<package_name>
sensor_interface_launch:=launch/<launch_file>
```

個別 package 側の launch は、基本的に次の引数を受け取れるようにする。

```text
camera_name
container_name
run_standalone
enable_depth
enable_color
use_sim_time
```

RealSense 以外の sensor では、使わない引数があってもよい。共通 launch から同じ形で
渡せることを優先する。

frame 名、camera topic、IMU topic は localization や mapping の設定と対応するため、
package README か config に明記する。

例:

```text
camera frame: <camera_name>_camera_link
left image: /<camera_name>/infra1/image_rect_raw
right image: /<camera_name>/infra2/image_rect_raw
imu: /<camera_name>/imu
```

## Set として扱う場合

sensor は単体 driver ではなく、車両ごとの set として扱うことが多い。

例えば次のような違いは、sensor interface package を分けて表現する。

- RealSense 1 台
- stereo camera 1 台
- front/rear camera
- camera + IMU
- camera + LiDAR

この場合、package 名は sensor 単体名ではなく set 名にする。

```text
realsense_front_sensor_kit
dual_camera_sensor_kit
camera_lidar_sensor_kit
```

## JetPilot 側に残すもの

`jetpilot_system_launch` に残すのは、次のような hardware 非依存の起動構成だけにする。

- bringup 全体の on/off
- vehicle interface package の選択
- sensor interface package の選択
- localization、control、tool の共通起動
- 共通 topic、共通 frame を前提にした設定

特定 hardware の詳細設定は、原則として個別 package に移す。

## 追加時の確認ポイント

- `packages.repos` で取り込めるか。
- `colcon build` で対象 package が build できるか。
- `vehicle_interface_pkg` / `sensor_kit_interface_pkg` で差し替えて起動できるか。
- JetPilot の共通 topic 名に接続されているか。
- frame 名が localization や description と一致しているか。
- hardware 固有 parameter が `jetpilot_system_launch` に漏れていないか。
