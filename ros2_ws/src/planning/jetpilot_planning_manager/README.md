# jetpilot_planning_manager

## Purpose

通常経路・矢印信号による分岐・衝突復帰を統括し、controller が購読する `/planning/*` を一箇所から配信する。

## Nodes

| Node | Executable | Description |
| --- | --- | --- |
| `planning_manager_node` | `planning_manager_node` | route、信号、衝突復帰を調停し最終planning bundleをpublishする |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `planning_manager_node` | `/hd_map/junctions` | `jetpilot_msgs/msg/JunctionArray` | Reliable / Transient Local | 分岐規則とrelease section |
| `planning_manager_node` | `/localization/current_section` | `std_msgs/msg/String` | Reliable / Volatile | 現在section |
| `planning_manager_node` | `/perception/direction_signal` | `jetpilot_msgs/msg/DirectionSignal` | Reliable / Volatile | 時系列確定した矢印方向 |
| `planning_manager_node` | `/safety/collision_detected` | `std_msgs/msg/Bool` | Reliable / Volatile | 衝突復帰要求の入力 |
| `planning_manager_node` | `/planning/route/trajectory` | `nav_msgs/msg/Path` | Reliable / Transient Local | route selectorの候補Path |
| `planning_manager_node` | `/planning/route/trajectory_profile` | `jetpilot_msgs/msg/Trajectory` | Reliable / Transient Local | route selectorのtyped候補 |
| `planning_manager_node` | `/planning/route/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Reliable / Volatile | route候補の世代・選択状態 |
| `planning_manager_node` | `/planning/recovery/trajectory_profile` | `jetpilot_msgs/msg/Trajectory` | Reliable / Transient Local | breadcrumb recovery軌道 |
| `planning_manager_node` | `/planning/recovery/ready` | `std_msgs/msg/Bool` | Reliable / Transient Local | recovery軌道の有効状態 |
| `planning_manager_node` | `/planning/recovery/status` | `jetpilot_msgs/msg/RecoveryStatus` | Reliable / Transient Local | recovery進捗 |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `planning_manager_node` | `/planning/trajectory` | `nav_msgs/msg/Path` | Reliable / Transient Local | controller向け最終Path |
| `planning_manager_node` | `/planning/trajectory_profile` | `jetpilot_msgs/msg/Trajectory` | Reliable / Transient Local | controller向け最終typed trajectory |
| `planning_manager_node` | `/planning/target_speed` | `std_msgs/msg/Float32` | Reliable / Transient Local | 最終速度上限 [m/s] |
| `planning_manager_node` | `/planning/ready` | `std_msgs/msg/Bool` | Reliable / Transient Local | 最終bundleの有効状態 |
| `planning_manager_node` | `/planning/requested_lane` | `std_msgs/msg/String` | Reliable / Volatile | route selectorへのlane要求lease |
| `planning_manager_node` | `/planning/recovery/request` | `std_msgs/msg/Bool` | Reliable / Volatile | recovery plannerの開始・解除 |
| `planning_manager_node` | `/planning/manager/status` | `jetpilot_msgs/msg/PlanningManagerStatus` | Reliable / Transient Local | manager状態とactive lane |

## Parameters

publish周期、route bundle timeout、recovery速度、Junction必須設定などは
[`config/planning_manager.param.yaml`](config/planning_manager.param.yaml)を参照してください。
Map固有のroute候補は`competition_route.param.yaml`から上書きします。

## Assumptions / Known limits

- Path、typed trajectory、diagnosticsが同一周期stampを持つ場合だけroute bundleを採用します。
- `/safety/collision_detected`のpublisherと、後方障害物によるreverse inhibitは別途必要です。
- 通常の`jetpilot_planning`最終selectorと同時に起動できません。

優先順位は `復帰 > 信号待ち/確定経路 > 通常経路`。信号が安定確定するまで `ready=false` とし、車両を停止させる。分岐 lane は、HD mapで必須指定したrelease sectionへ入るまで固定する。

競技用の `route_lane_selector_node` は、Mapごとの `competition_route.param.yaml` で
出力を `/planning/route/*` へ分離する。同梱の
`config/route_lane_selector.competition.param.yaml` を雛形として使う。
manager は selector が1周期で同じtimestampを付けた Path、typed profile、Diagnostics の3点を
同一世代として受け取った場合だけ採用する。信号確定・Junction定義変更・release時には旧世代を
即座に破棄し、選択laneが要求laneと一致する新世代が揃うまで `ready=false` を維持する。
同一内容のJunction周期再配信は無視し、位置・Section・信号ID・分岐先の実変更または削除だけを
新revisionとして扱う。

競技用planning一式は次で起動できる。

```bash
ros2 launch jetpilot_planning_manager competition_planning.launch.xml \
  route_config_file:=/workspaces/map/course_a/competition_route.param.yaml
```

Map Builder の Review は、各 Map 直下の同名ファイルを読み、Junction の
Left/Straight/Right ID が `lane_ids` に登録済みかを表示する。HD map 内に
経路が存在するだけでは実走行可能とは判定しない。さらに、managerへ接続する
`/planning/route/*` とdiagnostics、heartbeat、watchdogの設定も検証する。

信号YOLOは bringup へ以下を渡す。

```bash
enable_object_detection:=true \
object_detection_decoder_param_file:=/workspaces/ros2_ws/install/jetpilot_object_detection/share/jetpilot_object_detection/config/yolov8_signal.param.yaml \
object_detection_detections_topic:=/perception/signal/detections
```

## How to launch

```bash
ros2 launch jetpilot_planning_manager competition_planning.launch.xml \
  route_config_file:=/workspaces/map/course_a/competition_route.param.yaml
```
