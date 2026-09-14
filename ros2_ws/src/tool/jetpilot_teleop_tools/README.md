# jetpilot_teleop_tools

## Purpose

Joystick 入力を JetPilot の mode request、bag request、正規化 control command に変換する package です。`custom_joy_node` または ROS 標準の joy node が publish する `/joy` を入力にします。

## Nodes

| Node | 役割 |
| --- | --- |
| `teleop_cmd_node` | axis/button から `/teleop/control_cmd` を生成する |
| `teleop_button_manager_node` | mode切替、bag操作、steer offset、throttle scale、localization triggerをpublishする |
| `joy_calibrator.py` | joystick profile 作成補助 |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `teleop_cmd_node` | `/joy` | `sensor_msgs/msg/Joy` | Reliable / Volatile | joystick axes/buttons |
| `teleop_cmd_node` | `/steer_offset_inc` | `std_msgs/msg/Bool` | Reliable / Volatile | steering offset増加edge |
| `teleop_cmd_node` | `/steer_offset_dec` | `std_msgs/msg/Bool` | Reliable / Volatile | steering offset減少edge |
| `teleop_cmd_node` | `/speed_offset_inc` | `std_msgs/msg/Bool` | Reliable / Volatile | throttle scale増加edge |
| `teleop_cmd_node` | `/speed_offset_dec` | `std_msgs/msg/Bool` | Reliable / Volatile | throttle scale減少edge |
| `teleop_button_manager_node` | `/joy` | `sensor_msgs/msg/Joy` | Reliable / Volatile | button mapping入力 |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `teleop_cmd_node` | `/teleop/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Best Effort / Volatile | manual mode用正規化指令 |
| `teleop_button_manager_node` | `/operation_mode/request` | `jetpilot_msgs/msg/OperationModeRequest` | Reliable / Volatile | AUTO/MANUAL/STOP切替 |
| `teleop_button_manager_node` | `/bag/request` | `jetpilot_msgs/msg/BagRequest` | Reliable / Volatile | recording START/STOP |
| `teleop_button_manager_node` | `/steer_offset_inc` | `std_msgs/msg/Bool` | Reliable / Volatile | vehicle steering offset増加 |
| `teleop_button_manager_node` | `/steer_offset_dec` | `std_msgs/msg/Bool` | Reliable / Volatile | vehicle steering offset減少 |
| `teleop_button_manager_node` | `/speed_offset_inc` | `std_msgs/msg/Bool` | Reliable / Volatile | teleop throttle scale増加 |
| `teleop_button_manager_node` | `/speed_offset_dec` | `std_msgs/msg/Bool` | Reliable / Volatile | teleop throttle scale減少 |
| `teleop_button_manager_node` | `/localization/trigger` | `std_msgs/msg/Bool` | Reliable / Volatile | localization再試行trigger |

## Parameters

axis、deadman、scaleは[`config/teleop_cmd.param.yaml`](config/teleop_cmd.param.yaml)、button mappingは
[`config/joy_button_mapping.param.yaml`](config/joy_button_mapping.param.yaml)を参照してください。

## Assumptions / Known limits

- joystick profileごとにaxis/button番号と符号を校正する必要があります。
- deadman解除時にもzero commandをpublishし、最終timeoutはcommand muxとvehicle側で独立して監視します。

## Control algorithm

`teleop_cmd_node` は deadman button が押されている間だけ joystick 値を指令へ変換します。deadman が無効または離されている場合は steering/throttle/brake/reverse をすべて0にします。

既定ではD-pad左右が`steering_offset`を`steering_offset_step`ずつ増減し、上下が`throttle_scale`を`throttle_scale_step`ずつ増減します。いずれも押下edgeで1回だけ変更されます。Joy Profile Editorの「Button Functions」で各方向の割り当てを、「Joy Adjustment」でoffsetとscaleの初期値・刻み・範囲を設定できます。

`/speed_offset_inc` または `/speed_offset_dec` に `std_msgs/msg/Bool(data=true)` を1回送ると、`throttle_scale_step` ずつスロットルスケールを変更します。範囲は `throttle_scale_min` から `throttle_scale_max` までに制限され、現在値は `ros2 param get /teleop_cmd_node throttle_scale` で確認できます。既定のD-pad割り当ては、上がスケール増加、下が減少です。左右は従来どおりsteering offsetを調整します。

スケール変更に成功すると、`teleop_cmd_node`は変更前・変更後・刻み幅をINFOログへ出力します。上下限で値が変わらなかった場合も、現在値とともに上限・下限到達を通知します。

`speed_offset_inc_uses_localization_button=true`の場合は、通常localization triggerに使うボタンを一時的に`/speed_offset_inc`へ割り当てます。`calibration` presetがこの切替を使用し、通常presetではlocalization triggerの割り当てを維持します。

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

`teleop_button_manager_node` は誤操作を減らすため、AUTO/MANUAL を `hold_time_s` 以上長押ししている間だけ有効にします。対応するbuttonを離すとSTOPへ戻り、AUTO/MANUALの同時押しもSTOPになります。STOPも`hold_time_s`以上の長押しで優先されます。bag start/stop、offset、throttle scale調整は押下edgeで1回だけ発火します。steer offsetとthrottle scaleはbuttonに加えて、各`*_axis`と方向値を指定することでHat axis型の十字キーにも対応します。axis入力は各thresholdを超えたときに押下と判定します。localization triggerは他のbutton割当と衝突した場合、自動的に無効化します。

## How to launch

```bash
ros2 launch jetpilot_teleop_tools jetpilot_teleop_tools.launch.xml
```

通常は `jetpilot_system_launch` の `enable_joy:=true`、`enable_teleop:=true` から起動します。profile editor が生成した parameter は `/workspaces/ros2_ws/joy_profiles` にあればそちらを優先します。
