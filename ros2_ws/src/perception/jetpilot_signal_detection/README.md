# jetpilot_signal_detection

## Purpose

YOLO one-stage の `vision_msgs/Detection2DArray` を、HD map 上で有効な区間だけ時系列投票し、安定した左右・直進判断へ変換する。

学習クラス名は `arrow_left`, `arrow_straight`, `arrow_right`。学習時の class index 順序と decoder の `class_names` を必ず一致させる。

## Nodes

| Node | Executable | Description |
| --- | --- | --- |
| `signal_detection_node` | `signal_detection_node` | 有効区間内の検出結果を時系列投票して方向信号を確定する |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `signal_detection_node` | `/hd_map/junctions` | `jetpilot_msgs/msg/JunctionArray` | Reliable / Transient Local | signalと有効sectionの対応 |
| `signal_detection_node` | `/localization/current_section` | `std_msgs/msg/String` | Reliable / Volatile | 現在section ID |
| `signal_detection_node` | `/perception/signal/detections` | `vision_msgs/msg/Detection2DArray` | Reliable / Volatile | 矢印YOLOの検出結果 |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `signal_detection_node` | `/perception/direction_signal` | `jetpilot_msgs/msg/DirectionSignal` | Reliable / Transient Local | 確定方向またはunknown |

## Parameters

| Name | Type | Default Value | Description |
| --- | --- | --- | --- |
| `minimum_confidence` | `double` | `0.60` | 投票対象とする最小confidence |
| `vote_window_size` | `int` | `5` | 時系列投票window |
| `minimum_votes` | `int` | `3` | 確定に必要な票数 |
| `detection_timeout_s` | `double` | `0.50` | 検出途絶をunknownに戻す時間 |

全標準値は[`config/signal_detection.param.yaml`](config/signal_detection.param.yaml)を参照してください。

## Assumptions / Known limits

- 現在sectionがJunctionのactivation sectionに含まれる場合だけ検出を採用します。
- class名とYOLO decoderの`class_names`順序が一致している必要があります。
- 信号灯の追跡や位置同定は行わず、2D検出の時系列投票だけを担当します。

## How to launch

```bash
ros2 launch jetpilot_signal_detection signal_detection.launch.xml
```
