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

`/speed_offset_inc` または `/speed_offset_dec` に `std_msgs/msg/Bool(data=true)` を1回送ると、`throttle_scale_step` ずつスロットルスケールを変更します。範囲は `throttle_scale_min` から `throttle_scale_max` までに制限され、現在値は `ros2 param get /teleop_cmd_node throttle_scale` で確認できます。D-padの既存割り当ては変更しません。

たとえば、初期値を0.10にして10秒ごとに4回増加させ、最終的に0.30にする場合は次のように実行します。

```bash
ros2 param set /teleop_cmd_node throttle_scale 0.10
for step in 1 2 3 4; do
  sleep 10
  ros2 topic pub --once /speed_offset_inc std_msgs/msg/Bool "{data: true}"
done
```

steering は `steering_axis` に deadzone と scale をかけ、`[-1.0, 1.0]` に clamp します。throttle と reverse は trigger axis を `trigger_min` から `trigger_max` の範囲で `[0.0, 1.0]` に正規化し、個別の min/max/inverted parameter で controller ごとの差を吸収します。brake button が押された場合は `brake_value` を出します。

## Button algorithm

`teleop_button_manager_node` は誤操作を減らすため、AUTO/MANUAL を `hold_time_s` 以上長押ししている間だけ有効にします。対応するbuttonを離すとSTOPへ戻り、AUTO/MANUALの同時押しもSTOPになります。STOPも`hold_time_s`以上の長押しで優先されます。bag start/stop と offset 調整は押下 edge で1回だけ発火します。steer offsetはbuttonに加えて、`steer_offset_inc_axis`／`steer_offset_dec_axis`と方向値を指定することでHat axis型の十字キーにも対応します。axis入力は`steer_offset_axis_threshold`を超えたときに押下と判定します。localization trigger は他の割当と button が衝突した場合、自動的に無効化します。

## 起動

```bash
ros2 launch jetpilot_teleop_tools jetpilot_teleop_tools.launch.xml
```

通常は `jetpilot_system_launch` の `enable_joy:=true`、`enable_teleop:=true` から起動します。profile editor が生成した parameter は `/workspaces/ros2_ws/joy_profiles` にあればそちらを優先します。
