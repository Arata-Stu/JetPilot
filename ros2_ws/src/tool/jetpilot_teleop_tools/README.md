# jetpilot_teleop_tools

Joystick 入力を JetPilot の mode request、bag request、正規化 control command に変換する package です。`custom_joy_node` または ROS 標準の joy node が publish する `/joy` を入力にします。

## Nodes

| Node | 役割 |
| --- | --- |
| `teleop_cmd_node` | axis/button から `/teleop/control_cmd` を生成する |
| `teleop_button_manager_node` | mode 切替、bag 操作、steer offset、localization trigger を publish する |
| `joy_calibrator.py` | joystick profile 作成補助 |

## Topic契約

| 方向 | Topic | 型 | 用途 |
| --- | --- | --- | --- |
| input | `/joy` | `sensor_msgs/msg/Joy` | joystick の axes/buttons |
| output | `/teleop/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | manual mode で mux が採用する正規化指令 |
| output | `/operation_mode/request` | `jetpilot_msgs/msg/OperationModeRequest` | AUTO/MANUAL/STOP の切替 |
| output | `/bag/request` | `jetpilot_msgs/msg/BagRequest` | recording START/STOP |
| output | `/steer_offset_inc`, `/steer_offset_dec` | `std_msgs/msg/Bool` | vehicle driver 側の steering offset 調整 |
| output | `/localization/trigger` | `std_msgs/msg/Bool` | localization 再試行 trigger |

## Control algorithm

`teleop_cmd_node` は deadman button が押されている間だけ joystick 値を指令へ変換します。deadman が無効または離されている場合は steering/throttle/brake/reverse をすべて0にします。

steering は `steering_axis` に deadzone と scale をかけ、`[-1.0, 1.0]` に clamp します。throttle と reverse は trigger axis を `trigger_min` から `trigger_max` の範囲で `[0.0, 1.0]` に正規化し、個別の min/max/inverted parameter で controller ごとの差を吸収します。brake button が押された場合は `brake_value` を出します。

## Button algorithm

`teleop_button_manager_node` は誤操作を減らすため、AUTO/MANUAL/STOP を `hold_time_s` 以上の長押しで発火します。bag start/stop と offset 調整は押下 edge で1回だけ発火します。localization trigger は他の割当と button が衝突した場合、自動的に無効化します。

## 起動

```bash
ros2 launch jetpilot_teleop_tools jetpilot_teleop_tools.launch.xml
```

通常は `jetpilot_system_launch` の `enable_joy:=true`、`enable_teleop:=true` から起動します。profile editor が生成した parameter は `/workspaces/ros2_ws/joy_profiles` にあればそちらを優先します。
