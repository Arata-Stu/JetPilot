# jetpilot_planning

JetPilotの経路選択を担当する、C++ / `ament_cmake_auto` の最小planning基盤です。HD Map publisherが出すprimary centerlineをそのままcontrollerへ渡せる一方、将来のraceline、shortcut、信号条件、障害物回避経路を同じ契約で追加できます。

## Topic契約

| 方向 | Topic | 型 | 用途 |
| --- | --- | --- | --- |
| input | `/hd_map/primary_centerline_path` | `nav_msgs/msg/Path` | デフォルト経路 |
| input | `/planning/requested_lane` | `std_msgs/msg/String` | 条件判定moduleからのlane要求。空文字で解除 |
| input | `/localization/current_section` | `std_msgs/msg/String` | sectionに基づくlaneルール |
| candidate | `/planning/raceline_path` | `nav_msgs/msg/Path` | CSVから読み込んだraceline候補 |
| output | `/planning/trajectory` | `nav_msgs/msg/Path` | controllerが追従する選択済み経路 |
| output | `/planning/target_speed` | `std_msgs/msg/Float32` | 選択laneの目標速度（m/s） |
| output | `/planning/selected_lane` | `std_msgs/msg/String` | 選択中lane ID。未ready時は空文字 |
| output | `/planning/ready` | `std_msgs/msg/Bool` | controllerの実行可否 |
| output | `/planning/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 選択理由・不足path |

入力pathと出力trajectoryはreliable + transient-localです。出力は入力pathのframe（通常`map`）を維持し、各Poseのframeとstampを揃えます。選択可能なpathが無い場合は、古いlatched trajectoryを無効化するため空の`Path`、`target_speed=0`、`ready=false`をpublishします。controllerはこの状態で停止する必要があります。

## 選択優先順位

1. `/planning/requested_lane`の明示要求
2. `section_lane_rules`で現在sectionに割り当てたlane
3. `default_lane_id`

要求先pathが未受信・不正・timeoutの場合は標準設定で停止します。明示的に
`fallback_to_default_lane=true`へ変更した場合だけデフォルトへ戻ります。存在する別laneを
暗黙選択することはありません。

非空の`/planning/requested_lane`はleaseとして扱い、`requested_lane_timeout_sec`以内に
refreshされなければ停止します。障害物・信号selector自体を必須安全入力にする場合は
`require_requested_lane_heartbeat=true`にして、空要求の間もheartbeatを必須にします。
`section_lane_rules`を設定した場合、`/localization/current_section`も自動的にwatchdog対象となり、
timeoutまたは`unknown`では停止します。

## 起動

```bash
ros2 launch jetpilot_planning jetpilot_planning.launch.xml
```

通常は`jetpilot_hdmap_publisher`を先に起動します。標準設定は既存の`/hd_map/primary_centerline_path`を購読するため、追加設定なしで接続できます。

### 生成済みracelineを使う

Map toolが生成するF1TENTH形式
`s;x;y;psi;kappa;vx;ax`（実際のheaderは単位付きでも可）の
`<map>_raceline.csv`をC++ nodeで読み込み、`/planning/raceline_path`へ
reliable + transient-localでpublishできます。publisherは明示的に有効化した場合だけ起動し、
有効時は`raceline_csv`が必須です。

```bash
ros2 launch jetpilot_planning jetpilot_planning.launch.xml \
  enable_raceline_publisher:=true \
  raceline_root:=/workspaces/map/course_a \
  raceline_csv:=course_a_raceline.csv \
  config_file:=$(ros2 pkg prefix jetpilot_planning)/share/jetpilot_planning/config/route_lane_selector.raceline.param.yaml
```

`raceline_root`は選択中Mapのディレクトリを指定します。CSVはこのディレクトリ内の通常ファイル
だけを許可し、`..`による脱出、境界外の絶対path、symlinkを拒否します。
`raceline_root`を省略する場合は`raceline_csv`を絶対pathにする必要があります。
また、標準設定では16 MiB、200,000点を上限とし、列数、NaN/Inf、非単調な`s`、
負の`vx`、長さゼロのpathを起動時に検査します。

`route_lane_selector.raceline.param.yaml`はprimaryとracelineの両方を購読する接続例です。
標準topicは既存selectorの`lane_path_topics`へそのまま追加できます。現在の
`nav_msgs/msg/Path`には点ごとの`vx/ax`を格納できないため、parserは値を保持しますが、
controllerへ渡す速度は当面selectorの`lane_target_speeds_mps`を使用します。

## lane追加例

```yaml
/**:
  ros__parameters:
    lane_ids: [primary, raceline, shortcut, avoidance]
    lane_path_topics:
      - /hd_map/primary_centerline_path
      - /planning/raceline_path
      - /planning/shortcut_path
      - /planning/avoidance_path
    lane_target_speeds_mps: [1.5, 2.0, 1.0, 0.8]
    default_lane_id: raceline
    section_lane_rules:
      - shortcut_available=shortcut
    fallback_to_default_lane: false
```

信号判定や障害物回避はこのnodeへ直接組み込まず、専用moduleが候補`Path`をpublishしてlane IDを要求する構成にします。これにより、認識・条件判定と経路arbitrationを分離できます。標準設定は`fallback_to_default_lane=false`で、要求した回避laneが消えた場合にprimaryへ戻らず停止します。信号状態のように停止も必要な条件は、lane切替だけでなくcontrollerへ速度・停止制約を渡す別契約が必要です。

## 初期版の範囲

- centerlineのファイル読込は未実装です。生成済みraceline CSVの読込は実装済みです。
- 車幅・境界clearanceを考慮する生成は`python_ws/map_tools/generate_raceline.py`で実装済みです。
  planning packageはその生成物を検証して読み込む責務に限定しています。
- `nav_msgs/msg/Path`は目標速度を保持しないため、laneごとの速度を`lane_target_speeds_mps`で設定し、`/planning/target_speed`へ分けてpublishします。将来、点ごとの速度profileが必要になった時点で専用trajectory messageへ移行します。
- obstacleやsignalの認識・経路生成は未実装です。このpackageは、それらを追加できる安全な選択境界を提供します。
