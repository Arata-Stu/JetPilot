# jetpilot_control (legacy bench publisher)

This package publishes a fixed `/auto/control_cmd` for bench checks. It is no longer included by
`jetpilot_system_launch`; autonomous driving uses `jetpilot_controller` and its Pure Pursuit input
watchdogs. Do not launch both packages together because they publish the same command topic.

## Topic契約

| 方向 | Topic | 型 | 用途 |
| --- | --- | --- | --- |
| output | `/auto/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | parameter で固定した steering/throttle/brake/reverse を周期 publish |

## Algorithm

起動時に `steering`、`throttle`、`brake`、`reverse` parameter を読み、範囲外の値を正規化範囲へ clamp します。`publish_rate_hz` の周期で同じ `ControlCommand` を publish し続けます。feedback や path tracking は行いません。

既定では `brake=1.0`、`throttle=0.0` のため、単体起動時は停止指令になります。実車で throttle を与える場合は、必ずタイヤを浮かせた状態で符号と mux mode を確認してください。

## 起動

```bash
ros2 launch jetpilot_control jetpilot_control.launch.xml
```
