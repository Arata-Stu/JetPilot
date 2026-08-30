# Planning / Control architecture

プロジェクト全体のpackage、依存方向、言語、build方式は
[JetPilotプロジェクト設計方針](project_design_policy.md)に従います。

この文書は planning と controller の契約です。初期実装としてlane selectorと
Pure Pursuit controllerが入り、従来の固定値`autonomous_control_node`はsystem bringup
から外れました。分岐条件module、障害物回避とMPCはこの契約上へ追加します。

## Package boundaries

新規 package は `jetpilot_` prefix、`ament_cmake_auto`、C++ を基本とします。

| package | responsibility |
| --- | --- |
| `jetpilot_msgs` | lane graph、trajectory、planning/controller status の共通 interface |
| `jetpilot_hdmap_publisher` | HD map の静的情報を読み、可視化と共有用 map data を publish |
| `jetpilot_planning` | centerline/raceline/section を使った route 選択、lane 分岐、局所軌道生成 |
| `jetpilot_controller` | 選択済み trajectory を追従し `/auto/control_cmd` を生成 |

ファイル形式の解釈を複数 node にコピーしません。HD map の正規化処理は共有 library
または一つの loader に集約し、planning node は型付き message を受け取ります。
centerline等のlegacy候補は`nav_msgs/Path`とlane目標速度を使います。racelineとcustom lineは
`jetpilot_msgs/Trajectory`でgeometry、点ごとの速度・加速度、line ID、CSV hash、開閉路を
一体として渡します。互換用`Path`も同じtyped trajectoryから生成します。
Custom Lineの速度はUIでWaypointごとに入力せず、HD mapのSectionごとの目標上限として指定します。
offline compilerがSection GateをCustom Line上のstationへ写像し、曲率と加減速制約を適用してから、
runtime用の点ごとの速度・加速度へ展開します。

## Data flow

```mermaid
flowchart LR
  MAP["HD map / centerline / sections"] --> LOADER["jetpilot_hdmap_publisher"]
  RACE["Generated raceline CSV"] --> RACELOADER["jetpilot_planning raceline loader"]
  CUSTOM["Named custom line + speed CSV"] --> RACELOADER
  LOADER --> GRAPH["Lane candidates"]
  RACELOADER --> GRAPH
  GRAPH --> ROUTE["Route selector"]
  SIGNAL["Traffic signal state"] --> ROUTE
  POLICY["Shortcut / race policy"] --> ROUTE
  BLOCKED["Blocked lanes"] --> ROUTE
  ROUTE --> LOCAL["Local trajectory planner"]
  OBJECTS["Obstacles"] --> LOCAL
  POSE["Pose / velocity"] --> LOCAL
  LOCAL --> TRAJ["Typed trajectory / compatibility Path"]
  TRAJ --> CTRL["Pure Pursuit / future MPC"]
  POSE --> CTRL
  CTRL --> CMD["/auto/control_cmd"]
```

route selector は「どの lane を通るか」、local planner は「選んだ corridor 内を
どう避けるか」、controller は「与えられた軌道をどう追従するか」だけを担当します。
この分離により Pure Pursuit と MPC を差し替えても、lane 分岐の判断は変わりません。

## Lane graph and branching

lane は少なくとも次の静的情報を持ちます。

- 一意な lane ID、centerline、任意の raceline、左右境界
- section ID、速度上限、走行方向
- predecessor / successor connection
- connection の種別（default、shortcut、pit、signal-controlled など）
- 分岐を許可する条件 ID と静的 cost

信号色、障害物、shortcut 使用許可は map に現在値として保存せず、runtime state として
route selector に入力します。選択優先度は、走行不能・安全制約、信号などの必須条件、
route policy、通常 lane cost の順にします。候補がなくなった場合は無理に default lane
へ進まず、停止理由を status として publish します。

障害物回避は二段階に分けます。

1. lane 全体が塞がれた場合は route selector が別 successor を選ぶ。
2. 同じ lane corridor 内で回避できる場合は local planner が短い局所軌道を作る。

分岐直前に route が頻繁に切り替わらないよう、決定地点、hysteresis、最低保持時間を
connection ごとに持たせます。
動的なlane要求とsection状態はlease/watchdog付きとし、送信moduleが停止した場合は
最後の分岐を無期限に保持せずplanningをfail-closedにします。

## Message contract

`Trajectory`と`TrajectoryPoint`は実装済みです。lane graphとstatus型は分岐実装時に追加します。

- `LaneGraph`, `LaneSegment`, `LaneConnection`
- **実装済み:** `Trajectory`, `TrajectoryPoint`（pose、速度、曲率、加速度、line ID/hash、開閉路）
- `PlanningStatus`（active route、blocked reason、trajectory age）
- `ControllerStatus`（algorithm、tracking error、command age、ready/fault）

静的 map は transient-local、trajectory と状態量は期限付きの volatile QoS とします。
controller は古い trajectory、frame 不一致、localization 不良、NaN/Inf を検出したら
有効な走行指令を出しません。

## Controller sequence

最初の controller は Pure Pursuit とします。設定対象は wheelbase、最小/最大 lookahead、
速度に対する lookahead gain、最大 steering、trajectory timeout です。legacy Pathは
`/planning/target_speed`を使います。typed trajectoryでは現在stationと先読み区間から点ごとの
速度profileを読み、scalar目標速度、global上限、曲率・横加速度上限、trailing上限との最小値を
使います。CSVの区間一定加速度と合わせ、点間速度は`v²`を距離で線形補間します。

同じ入力・出力契約で `map_pursuit` と軽量な `kinematic_mpc` を選択できます。現行の
`kinematic_mpc` は動的タイヤモデルを使わず、kinematic bicycle model の操舵候補を
rolloutして選ぶ lateral controller です。controller 選択は launch parameter で
行い、実行中の無条件 hot swap は行いません。既存の固定値
`autonomous_control_node` はsystem launchでは使用せず、`jetpilot_controller`の
controller factoryと入力watchdogを使用します。

## Raceline and vehicle footprint

raceline 最適化では点ではなく車体 footprint を扱います。

```text
effective optimization width = vehicle width + 2 × per-side safety margin
```

`vehicle_width_m` は車体の最大幅、`safety_margin_m` は左右それぞれに追加する境界余裕です。
track の有効幅がこれ以下なら境界を自動的に広げず、入力 map の不整合として生成を停止
します。生成物には使用した車幅と margin を metadata として残し、別車両向けの
raceline を誤用しないようにします。実走行時の localization 誤差や制御追従誤差は、
必要に応じて offline margin に加えて local planner の corridor 制約でも扱います。

## Implementation order

1. **実装済み:** `nav_msgs/Path`による初期契約、lane selector、raceline CSV loaderを用意する。
2. **実装済み:** `jetpilot_controller`のPure Pursuit、localization/TF/input timeout時の停止を実装する。
3. **実装済み:** raceline/custom lineをtyped trajectoryへ移行し、Section指定からcompileした点ごとの速度profileをcontrollerへ接続する。
4. successor graph と shortcut / signal 条件を追加し、分岐 unit test を作る。
5. obstacle input と local avoidance、走行不能時の停止を追加する。
6. **実装済み:** 同一trajectory fixtureでPure Pursuit、MAP-like pursuit、軽量Kinematic MPCを比較できるcontroller factoryを追加する。

各段階で、狭い lane、分岐条件欠落、全候補閉鎖、古い localization、古い trajectory、
逆向き経路、loop course を自動テストします。Linux Docker 上では `colcon test` に加え、
vehicle interface を無効にした rosbag replay の integration test を実行します。
