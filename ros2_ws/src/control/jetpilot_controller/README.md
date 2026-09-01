# jetpilot_controller

JetPilotの経路追従controllerです。初期アルゴリズムとしてPure Pursuitを実装し、既存の
operation command muxを通してPCA9685/VESCのどちらにも同じ正規化指令を送ります。

## Interface

| Direction | Topic | Type | Meaning |
| --- | --- | --- | --- |
| input | `/planning/trajectory` | `nav_msgs/msg/Path` | `map`等で表現された追従経路 |
| input | `/planning/trajectory_profile` | `jetpilot_msgs/msg/Trajectory` | 任意。名前付きlineのgeometry、`vx/ax`、ID/hash |
| input | `/planning/target_speed` | `std_msgs/msg/Float32` | plannerが選択した前進速度 [m/s] |
| input | `/planning/ready` | `std_msgs/msg/Bool` | plannerの経路選択が有効か |
| input | `/localization/pose_hint_state` | `std_msgs/msg/String` | managerのconfirmed `localized`状態 |
| input | `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | 速度と自己位置系の生存確認 |
| input | `/perception/opponent/odometry` | `nav_msgs/msg/Odometry` | 任意。trailing有効時の相手車両pose/速度 |
| output | `/auto/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | 正規化steer/throttle/brake |
| output | `/controller/ready` | `std_msgs/msg/Bool` | 走行指令を生成できているか |
| output | `/controller/lookahead_point` | `geometry_msgs/msg/PoseStamped` | `base_link`上の追従点 |
| output | `/controller/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 停止理由と制御値 |

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
typed trajectoryではmessageの`closed`がこの設定より優先されるため、custom lineごとに開閉路を
安全に切り替えられます。

## Controller selection

制御計算は`PathTrackingController` interfaceからROS nodeを分離しています。`algorithm`
parameterで lateral controller を選択でき、ROS topic、watchdog、安全停止、command limitは
そのまま共用できます。

| algorithm | 概要 |
| --- | --- |
| `pure_pursuit` | 最小構成のPure Pursuit。基準controllerとして使います。 |
| `map_pursuit` | Pure Pursuit系に横偏差補償と高速時の操舵downscaleを加えた実車調整向けcontrollerです。 |
| `kinematic_mpc` | kinematic bicycle modelで複数の操舵候補を予測し、path/heading/steering costが最小の操舵を選びます。 |

`kinematic_mpc`は動的タイヤモデルを使わず、現時点では操舵だけを最適化します。速度、加速度、
trailing、横加速度上限、steering rate limitは既存node側で扱います。急加速で不安定になりやすい
車両では、MPCモデルを複雑にする前に`max_target_speed_mps`、`max_lateral_accel_mps2`、
`max_steering_rate_per_s`、throttle/brake gainを保守的に調整してください。

## Control algorithm

Pure Pursuit は planner の `Path` を `base_link` 座標へ変換し、現在速度に応じた lookahead 距離で追従点を選びます。選ばれた点から曲率を計算し、wheelbase と最大舵角を使って正規化 steering command へ変換します。

longitudinal側はtarget speedとodometry speedの差を見てthrottle/brakeを出します。typed trajectoryが
選択されている場合は、現在位置をlineへ射影して点ごとの速度・加速度を補間し、
`trajectory_speed_lookahead_m`区間内の最小速度を先読みします。最終速度はprofile、
`/planning/target_speed`、`max_target_speed_mps`、曲率由来上限、trailing上限の最小値です。
`ax` feed-forward gainは既定0なので、実車確認後にだけ有効化してください。
点間速度はcompiled CSVの区間一定加速度`ax=(v1²-v0²)/(2ds)`に合わせ、`v²`を距離で線形補間します。

## Opponent trailing

`trailing_enabled: true`にすると、controllerは相手車両odometryを現在のtrajectory上へ射影し、
経路に沿った前方距離を`trailing_gap_m`へ近づけるようにtarget speedを下げます。操舵はこれまで通り
Pure Pursuitがtrajectoryを追従し、trailingは縦方向の速度上限だけを担当します。

相手車両が自車より前方にいない、`trailing_max_gap_m`より遠い、またはodometryがtimeoutした場合は
trailingを解除し、planningのtarget speedへ戻ります。閉路ではtrajectory長でgapをwrapします。
相手odometryのframeは`base_frame`またはTFで`base_frame`へ変換できるframeにしてください。
# 後退軌道

`jetpilot_msgs/Trajectory.motion_direction=MOTION_REVERSE` の場合、軌道形状を後退運動座標へ変換してステアを計算し、縦制御出力を `throttle` ではなく `reverse` に出力する。速度プロファイル値そのものは前進時と同様に正の m/s で記述する。
