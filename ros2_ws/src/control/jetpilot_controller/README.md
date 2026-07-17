# jetpilot_controller

JetPilotの経路追従controllerです。初期アルゴリズムとしてPure Pursuitを実装し、既存の
operation command muxを通してPCA9685/VESCのどちらにも同じ正規化指令を送ります。

## Interface

| Direction | Topic | Type | Meaning |
| --- | --- | --- | --- |
| input | `/planning/trajectory` | `nav_msgs/msg/Path` | `map`等で表現された追従経路 |
| input | `/planning/target_speed` | `std_msgs/msg/Float32` | plannerが選択した前進速度 [m/s] |
| input | `/planning/ready` | `std_msgs/msg/Bool` | plannerの経路選択が有効か |
| input | `/localization/pose_hint_state` | `std_msgs/msg/String` | managerのconfirmed `localized`状態 |
| input | `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | 速度と自己位置系の生存確認 |
| output | `/auto/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | 正規化steer/throttle/brake |
| output | `/controller/ready` | `std_msgs/msg/Bool` | 走行指令を生成できているか |
| output | `/controller/lookahead_point` | `geometry_msgs/msg/PoseStamped` | `base_link`上の追従点 |
| output | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 停止理由と制御値 |

plannerのPath frameから`base_link`へのTFが必要です。通常はlocalizationが`map -> odom`、
VSLAMが`odom -> base_link`を供給します。
plannerの標準publish周期は10 Hzで、controllerの0.5秒watchdogへheartbeatも兼ねます。

## Safety behavior

次のどれかに該当すると、steering/throttle/reverseを0にした停止指令を周期的にpublishします。

- plannerがreadyでない、空経路をpublishした
- localization managerがconfirmed `localized`でない、または状態がtimeout
- trajectory、target speed、odometryのいずれかが未受信またはtimeout
- Pathのframeが空、混在、不正値を含む、または`base_link`までのTFがない・古い
- 開いた経路の終端へ到達した
- controllerパラメータや入力速度が不正

`safety_brake_command`は車両に依存するため初期値はneutral stop (`0.0`)です。VESCのbrake
currentやPCA9685 ESCの逆転制動を実車で確認した後にだけ増やしてください。command muxと車両
driver側にも独立したcommand timeoutがあります。

## Run

```bash
ros2 launch jetpilot_controller jetpilot_controller.launch.xml
```

車体ごとに最低でも`wheelbase_m`、`max_steering_angle_rad`、lookahead、速度・出力上限を調整し、
最初はタイヤを浮かせた状態で符号を確認してください。`path_closure_mode: auto`では閉ループ
経路の先頭・末尾の距離が`closed_path_tolerance_m`以下なら自動判定し、終端で停止せずwrap
します。HD mapの閉路が先頭点を末尾へ複製しておらず点間隔も広い場合は、track用設定を
`path_closure_mode: closed`にしてください。分岐後の開いたlaneでは`open`を指定できます。

## Adding MPC

制御計算は`PathTrackingController` interfaceからROS nodeを分離しています。将来は
`MpcController`を同interfaceで実装し、nodeのfactoryに`algorithm: mpc`を追加します。
ROS topic、watchdog、安全停止、command limitはそのまま共用できます。
