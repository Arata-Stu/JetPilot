# jetpilot_control

## Purpose

This package publishes a fixed `/auto/control_cmd` for bench checks. It is no longer included by
`jetpilot_system_launch`; autonomous driving uses `jetpilot_controller` and its Pure Pursuit input
watchdogs. Do not launch both packages together because they publish the same command topic.

## Nodes

| Node | Executable | Description |
| --- | --- | --- |
| `autonomous_control_node` | `autonomous_control_node` | 固定した正規化制御指令をbench確認用に周期配信する |

## Inputs / Outputs

### Input topics

なし。

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `autonomous_control_node` | `/auto/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Best Effort / Volatile | parameterで固定したsteering/throttle/brake/reverse |

## Parameters

| Name | Type | Default Value | Description |
| --- | --- | --- | --- |
| `publish_rate_hz` | `double` | `20.0` | 指令のpublish周期 [Hz] |
| `steering` | `double` | `0.0` | 正規化操舵指令 |
| `throttle` | `double` | `0.0` | 正規化前進指令 |
| `brake` | `double` | `1.0` | 正規化制動指令 |
| `reverse` | `double` | `0.0` | 正規化後進指令 |

標準値は [`config/autonomous_control.param.yaml`](config/autonomous_control.param.yaml) にあります。

## Assumptions / Known limits

feedbackやpath trackingを持たないlegacy bench publisherです。`jetpilot_controller`またはE2E制御と
同時に起動してはいけません。

## Algorithm

起動時に `steering`、`throttle`、`brake`、`reverse` parameter を読み、範囲外の値を正規化範囲へ clamp します。`publish_rate_hz` の周期で同じ `ControlCommand` を publish し続けます。feedback や path tracking は行いません。

既定では `brake=1.0`、`throttle=0.0` のため、単体起動時は停止指令になります。実車で throttle を与える場合は、必ずタイヤを浮かせた状態で符号と mux mode を確認してください。

## How to launch

```bash
ros2 launch jetpilot_control jetpilot_control.launch.xml
```
