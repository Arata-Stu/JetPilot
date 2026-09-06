# jetpilot_recovery_planner

## Purpose

通常走行中の odometry を breadcrumb として保持し、衝突復帰要求時に来た軌跡を逆順へ並べた後退軌道を生成する。単純な直線後退ではないため、controller は過去のコース形状に沿ってステアしながら戻り、復帰終了時の車体向きを元のコース方向へ合わせやすい。

## Nodes

| Node | Executable | Description |
| --- | --- | --- |
| `breadcrumb_recovery_planner_node` | `breadcrumb_recovery_planner_node` | odometry履歴から後退trajectoryを生成する |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `breadcrumb_recovery_planner_node` | `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | Reliable / Volatile | 通常走行breadcrumbと復帰進捗 |
| `breadcrumb_recovery_planner_node` | `/planning/recovery/request` | `std_msgs/msg/Bool` | Reliable / Volatile | 復帰開始・解除要求 |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `breadcrumb_recovery_planner_node` | `/planning/recovery/trajectory_profile` | `jetpilot_msgs/msg/Trajectory` | Reliable / Transient Local | 逆順breadcrumbの後退軌道 |
| `breadcrumb_recovery_planner_node` | `/planning/recovery/ready` | `std_msgs/msg/Bool` | Reliable / Transient Local | 復帰軌道が追従可能か |
| `breadcrumb_recovery_planner_node` | `/planning/recovery/status` | `jetpilot_msgs/msg/RecoveryStatus` | Reliable / Transient Local | 記録・復帰・成功・失敗状態 |

## Parameters

sampling間隔、履歴距離、復帰距離、timeout、後退速度などの標準値は
[`config/recovery_planner.param.yaml`](config/recovery_planner.param.yaml)を参照してください。

## Assumptions / Known limits

後方障害物センサはこの planner の範囲外であり、実車では `/safety/collision_detected` の解除とは別に reverse inhibit を車両 safety 層へ追加すること。

## How to launch

```bash
ros2 launch jetpilot_recovery_planner recovery_planner.launch.xml
```
