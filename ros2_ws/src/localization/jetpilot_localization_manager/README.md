# jetpilot_localization_manager

## Purpose

`jetpilot_localization_manager` coordinates saved-map localization between Isaac ROS Visual
Global Localization (VGL), Isaac ROS Visual SLAM (VSLAM), RViz, joystick input, and the web UI.

## Nodes

| Node | Executable | Description |
| --- | --- | --- |
| `jetpilot_localization_manager_node` | `jetpilot_localization_manager_node` | VGL/VSLAM/manual pose hintの再測位状態を統括する |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `jetpilot_localization_manager_node` | `/visual_slam/trigger_hint` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Reliable / Volatile | VSLAMからの再試行要求 |
| `jetpilot_localization_manager_node` | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Reliable / Volatile | RViz/UIからのmanual pose |
| `jetpilot_localization_manager_node` | `/visual_localization/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Reliable / Volatile | VGL推定pose |
| `jetpilot_localization_manager_node` | `/localization/trigger` | `std_msgs/msg/Bool` | Reliable / Volatile | joystickからの再測位trigger |
| `jetpilot_localization_manager_node` | `/localization/vslam/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Reliable / Volatile | VSLAMのlocalization状態 |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `jetpilot_localization_manager_node` | `/localization/pose_hint` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Reliable / Volatile | 検証済みVSLAM pose hint |
| `jetpilot_localization_manager_node` | `/localization/pose_hint_required` | `std_msgs/msg/Bool` | Reliable / Transient Local | manual hintが必要かを示すUI状態 |
| `jetpilot_localization_manager_node` | `/localization/pose_hint_state` | `std_msgs/msg/String` | Reliable / Transient Local | 再測位state machineのJSON状態 |

### Services

| Node | Name | Type | Direction | Description |
| --- | --- | --- | --- | --- |
| `jetpilot_localization_manager_node` | `/visual_localization/trigger_localization` | `std_srvs/srv/Trigger` | Client | VGL推定を要求する |
| `jetpilot_localization_manager_node` | `/localization/relocalize` | `std_srvs/srv/Trigger` | Server | UI/CLIから再測位を開始する |

## Parameters

pose検証、試行回数、timeout、backoff、topic/service名を設定します。標準値は
[`config/localization_manager.param.yaml`](config/localization_manager.param.yaml)を参照してください。

## Assumptions / Known limits

- saved-map localizationにはVSLAM diagnosticsと、選択したmodeに応じてVGL mapまたはmanual poseが必要です。
- Isaac ROSには進行中のorigin localizationをcancelする明確なAPIがないため、timeout後にstack再起動が必要な状態があります。

## Flow

1. A request comes from `autostart`, `/localization/trigger`, or
   `/localization/relocalize`.
2. The manager waits for the VGL Trigger service and, by default, both a subscriber on the VSLAM
   pose-hint topic and the first VSLAM localization diagnostic. These readiness checks prevent
   one-shot startup messages from being lost or sent before VSLAM initialization.
3. The manager calls VGL and validates the resulting pose.
4. The pose is held until VSLAM subscribes, then forwarded to `/localization/pose_hint`.
5. If VSLAM publishes `/visual_slam/trigger_hint`, VGL is retried with bounded backoff.
6. After timeouts or exhausted attempts, the manager waits for a valid `/initialpose` message.

When `origin_startup=true`, VSLAM performs its own identity-pose localization against the saved
map. The launch layer requires at least one direct-child `*.mdb` database in `cuvslam_map/`,
matching cuVSLAM's own loader check. The manager does not start VGL or publish a pose hint. It
keeps publishing safety status,
accepts a fresh `localized_in_exist_map=Yes` diagnostic as success, and blocks manual
`/initialpose` and relocalize requests while that startup job may still be running. Isaac ROS does
not expose cancellation or an unambiguous completion signal for the detached origin-localization
job. A manager timeout therefore enters `map_origin_restart_required`; restart the localization
stack with `origin_startup=false` before sending a pose hint. A late success diagnostic is still
accepted while restart is pending. The confirmation timeout starts only after the first valid
VSLAM localization diagnostic, so component and camera cold-start time is not mistaken for an
origin-localization timeout. A separate 120-second diagnostics-readiness watchdog prevents a
missing or broken VSLAM diagnostic stream from leaving startup pending forever; it also requires a
restart and never unlocks an in-process manual pose.

When `vslam_save_map_folder_path` is set, the system launch treats the run as mapping: it does not
load an existing cuVSLAM map and does not start this manager. With a saved cuVSLAM map but no VGL
map, autostart instead enters the manual `/initialpose` fallback immediately.

Timeout and backoff deadlines use a steady clock, so they do not stall while simulated time is
waiting for `/clock`.

## Status semantics

Publishing a pose hint is **not** reported as successful localization. The manager enters
`awaiting_vslam`. It may enter `localized` when VSLAM diagnostics provide a conservative
`localized_in_exist_map` `No -> Yes` transition. cuVSLAM can retain an earlier `Yes` while a new
request is running, so an isolated `Yes` is deliberately not accepted as proof of the new request.
VSLAM's `trigger_hint` remains the authoritative retry signal.

After a confirmed success, a later `localized_in_exist_map=No` immediately changes the manager
state to `unlocalized` and sets `pose_hint_required=true`. This fail-closed transition prevents the
controller from continuing on a stale `localized` status while VSLAM has lost the saved map.

An `awaiting_vslam` deadline prevents the UI from hanging forever. If VSLAM still reports `Yes`
without a request-specific `No -> Yes` transition, the state becomes `localized_unconfirmed` rather
than claiming a confirmed success. A missing/negative result retries VGL or falls back to manual
pose input. A late valid VGL pose is still accepted while manual fallback is waiting, because the
upstream VGL service has no cancellation API.

The JSON status contains `state`, `pose_hint_required`, request source, `origin_startup`,
`restart_required`, readiness, attempt counts, last hint source, and the latest reason. The status
publishers use reliable transient-local QoS.

## Build and test

```bash
colcon build --packages-select jetpilot_localization_manager
colcon test --packages-select jetpilot_localization_manager
colcon test-result --verbose
```

The unit tests cover frame, finite-number, quaternion, covariance, and timestamp pose validation.

## Topic handling notes

`/localization/pose_hint_required` と `/localization/pose_hint_state` は reliable transient-local QoS です。UI、controller、後から起動した diagnostic node が最新状態をすぐ読めるようにしています。

`/localization/trigger` は joystick 由来の edge trigger、`/localization/relocalize` は UI/CLI 由来の service trigger として扱います。どちらも同じ state machine に入り、進行中の request がある場合は新しい request として再開します。

## How to launch

通常は`jetpilot_system_launch`の`localization.launch.py`または`bringup.launch.py`から起動します。

再測位をCLIから要求する場合:

```bash
ros2 service call /localization/relocalize std_srvs/srv/Trigger '{}'
```

## Foxgloveで再測位を切り分ける

`/localization/manager/diagnostics`（`diagnostic_msgs/msg/DiagnosticArray`、reliable /
transient-local）に、次の4項目を出力します。既定のFoxglove topic whitelistで配信されます。
Raw Messagesでこのトピックを選び、`status`を展開すると各項目の`message`と`values`を確認できます。
診断表示パネルを利用する場合も同じトピックを指定します。既定のbag記録対象にも追加しています。

| 項目 | 確認できること |
| --- | --- |
| `Localization/Manager` | 全体のstateとreason、入力通番、直近入力の受付・拒否理由、要求元、試行回数、再起動要求 |
| `Localization/VGL` | trigger送信、pose待ち、検証済みpose受信、失敗・timeout、手動poseによるスキップ |
| `Localization/VSLAM` | `vo_status`、既存地図の測位フラグ、managerの確認状態、診断が新しいか |
| `Localization/TF` | `map → odom`と`odom → base_link`それぞれの更新時刻・鮮度・hint送信後に更新されたか |

joyのtrue入力は`last_input_source=joy_topic`、Foxgloveの`/initialpose`は`manual`、
再測位serviceは`service`になります。手動poseではVGLを呼ばず、`bypassed_manual`と表示します。
不正なframeやpose、原点測位中の入力拒否は`last_input_result=rejected:...`に残ります。
直近入力情報は次の操作まで保持し、入力受付と全state遷移はINFOログにも記録します。
`input_sequence`はこのノード内の操作通番であり、VGL/VSLAMの応答IDではありません。

`localization_state_allows_control=false`は、managerの状態がcontrollerの測位条件を満たさない意味です。
車両の実停止を計測した結果ではありません。controllerの`require_localization_state=true`時は
`localized`以外を停止条件として扱います。実際の停止理由は`/controller/diagnostics`も確認してください。

### 成功の意味と限界

- `VGL/pose_validated`は検証済みposeの受信です。serviceのsuccessは受付結果であり、測位成功ではありません。
- `VSLAM/localized_in_exist_map=true`だけで今回の再測位成功とは断定しません。既存の`No → Yes`確認を維持しています。
- `TF/*_updated_after_hint=true`は、hint送信後の時刻を持つ新しいTFを観測した意味です。
  hintによる補正が反映された証明ではありません。`pose_correction_verified=unknown`と明示します。
- TF監視は標準構成の直接の動的TFを対象とし、配信元ノードの特定やTF競合の判定は行いません。
- `observation_timeout_sec`（既定1.5秒）でROS時刻と実時間の両方を確認します。
  古い時刻の再配信、未来時刻、停止したbag再生ではSTALEになり得ます。
- この追加診断は観測用です。TF/VO異常から自動的に再測位・リセットする制御は追加していません。
  VGL poseに要求IDがないため、キャンセル前の遅延応答と今回の応答を厳密に区別することもできません。

再現時は`/localization/manager/diagnostics`、`/localization/pose_hint_state`、
VSLAM/VGLのdiagnostics、`/tf`を同時に記録してください。
成功ログ後にTFがSTALEなら追跡・配信経路、TFが新しいのに位置が違うなら測位結果・座標系を調べます。
