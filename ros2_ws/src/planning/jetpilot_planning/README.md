# jetpilot_planning

`primary_trajectory_topic`を指定すると、`primary`レーンの入力を指定したtyped
Trajectoryに差し替えます。レーンIDとSectionルールは維持し、形状・速度・周回設定を
同じTrajectoryから取得します。`primary`がない設定や不整合な候補配列では起動を拒否します。
`bringup.sh`の競技モードでは、走行ライン選択からこの設定を使用します。

## Purpose

JetPilotの経路選択を担当する、C++ / `ament_cmake_auto` の最小planning基盤です。HD Map publisherが出すprimary centerlineをそのままcontrollerへ渡せる一方、将来のraceline、shortcut、信号条件、障害物回避経路を同じ契約で追加できます。

## Nodes

| Node | Executable | Description |
| --- | --- | --- |
| `route_lane_selector_node` | `route_lane_selector_node` | 候補経路から要求・section規則に一致する経路を選択する |
| `competition_route_lane_selector_node` | `route_lane_selector_node` | 競技構成で`/planning/route/*`へ候補bundleを出力する |
| `raceline_path_publisher` | `raceline_path_publisher_node` | 検証済みCSVからPathとtyped trajectoryをpublishする |
| `custom_line_path_publisher` | `raceline_path_publisher_node` | custom line CSVを同じloaderでpublishする |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `route_lane_selector_node` | `/hd_map/primary_centerline_path` | `nav_msgs/msg/Path` | Reliable / Transient Local | 標準primary経路候補 |
| `route_lane_selector_node` | `/planning/raceline_trajectory` | `jetpilot_msgs/msg/Trajectory` | Reliable / Transient Local | raceline候補。profile構成時に使用 |
| `route_lane_selector_node` | `/planning/custom_trajectory` | `jetpilot_msgs/msg/Trajectory` | Reliable / Transient Local | custom line候補。custom構成時に使用 |
| `route_lane_selector_node` | `/planning/requested_lane` | `std_msgs/msg/String` | Reliable / Volatile | lane要求lease |
| `route_lane_selector_node` | `/localization/current_section` | `std_msgs/msg/String` | Reliable / Volatile | section lane規則の入力 |
| `competition_route_lane_selector_node` | `/hd_map/primary_centerline_path` | `nav_msgs/msg/Path` | Reliable / Transient Local | 競技routeの標準候補。追加候補はMap設定で指定 |
| `competition_route_lane_selector_node` | `/planning/requested_lane` | `std_msgs/msg/String` | Reliable / Volatile | planning managerからのlane要求 |
| `competition_route_lane_selector_node` | `/localization/current_section` | `std_msgs/msg/String` | Reliable / Volatile | section lane規則の入力 |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `route_lane_selector_node` | `/planning/trajectory` | `nav_msgs/msg/Path` | Reliable / Transient Local | 選択済みlegacy Path |
| `route_lane_selector_node` | `/planning/trajectory_profile` | `jetpilot_msgs/msg/Trajectory` | Reliable / Transient Local | 選択済みtyped trajectory |
| `route_lane_selector_node` | `/planning/target_speed` | `std_msgs/msg/Float32` | Reliable / Transient Local | 選択laneの速度上限 [m/s] |
| `route_lane_selector_node` | `/planning/selected_lane` | `std_msgs/msg/String` | Reliable / Transient Local | 選択中lane ID |
| `route_lane_selector_node` | `/planning/ready` | `std_msgs/msg/Bool` | Reliable / Transient Local | 経路bundleの有効状態 |
| `route_lane_selector_node` | `/planning/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Reliable / Volatile | 選択理由と不正入力 |
| `competition_route_lane_selector_node` | `/planning/route/trajectory` | `nav_msgs/msg/Path` | Reliable / Transient Local | manager向け候補Path |
| `competition_route_lane_selector_node` | `/planning/route/trajectory_profile` | `jetpilot_msgs/msg/Trajectory` | Reliable / Transient Local | manager向けtyped候補 |
| `competition_route_lane_selector_node` | `/planning/route/target_speed` | `std_msgs/msg/Float32` | Reliable / Transient Local | manager向け速度上限 |
| `competition_route_lane_selector_node` | `/planning/route/selected_lane` | `std_msgs/msg/String` | Reliable / Transient Local | manager向け選択lane ID |
| `competition_route_lane_selector_node` | `/planning/route/ready` | `std_msgs/msg/Bool` | Reliable / Transient Local | manager向けroute ready |
| `competition_route_lane_selector_node` | `/planning/route/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Reliable / Volatile | managerが世代照合するdiagnostics |
| `raceline_path_publisher` | `/planning/raceline_path` | `nav_msgs/msg/Path` | Reliable / Transient Local | CSV由来legacy Path |
| `raceline_path_publisher` | `/planning/raceline_trajectory` | `jetpilot_msgs/msg/Trajectory` | Reliable / Transient Local | CSV由来geometryと速度profile |
| `custom_line_path_publisher` | `/planning/custom_path` | `nav_msgs/msg/Path` | Reliable / Transient Local | custom lineのlegacy Path |
| `custom_line_path_publisher` | `/planning/custom_trajectory` | `jetpilot_msgs/msg/Trajectory` | Reliable / Transient Local | custom lineのgeometryと速度profile |

## Parameters

selectorの候補topic、lane ID、速度上限、watchdogと出力topicは
[`config/route_lane_selector.param.yaml`](config/route_lane_selector.param.yaml)で定義します。
raceline publisherのファイル検証と出力設定は
[`config/raceline_path_publisher.param.yaml`](config/raceline_path_publisher.param.yaml)を参照してください。

## Assumptions / Known limits

- 候補配列`lane_ids`、`lane_path_topics`、`lane_trajectory_topics`、`lane_target_speeds_mps`の長さは一致させます。
- 認識や経路生成は担当せず、受信済み候補の検証と選択だけを行います。
- 通常selectorと競技planning managerは同じ最終`/planning/*`を出すため同時に使用しません。

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

## How to launch

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
起動後もCSVのatomic replaceを監視し、読込前後で同じfile revisionであることを確認できた場合だけ
再配信します。`source_hash`指定は起動時のintegrity確認に限り、以後の有効なGUI保存では新CSVから
計算したSHA-256を`Trajectory.source_hash`として配信します。更新CSVが欠損または不正な間は空の
Path/Trajectoryを一度配信してfail-closedにし、次の有効なatomic replaceで自動復帰します。

`route_lane_selector.raceline.param.yaml`はprimaryとracelineの両方を購読する接続例です。
raceline/custom候補は`lane_trajectory_topics`でtyped messageを購読し、geometry、`vx/ax`、
`line_id`、`source_hash`を一体として選択します。従来の`/planning/trajectory`も同じgeometryから
生成してpublishするため、Pathだけを使うconsumerとの互換性を維持します。

### 名前付きcustom lineを使う

centerlineまたはracelineをUIで複製し、形状とSectionごとの目標速度を編集して生成した7列CSVも、
同じ検証済みloaderで読みます。Waypointごとの速度入力は不要です。Section Gateの交点を境界に、
曲率・加速・減速制約を適用した速度profileがofflineで各trajectory pointへ展開されます。

```bash
./scripts/bringup.sh custom \
  --components sensor,localization,hd-map,control,vehicle \
  --custom-line /workspaces/map/course_a/custom_lines/safe-main/trajectory.csv
```

標準では閉路です。開いたlineは`--custom-line-open`を付けます。`trajectory.csv`なら親directory名を
line IDとして推定し、それ以外は`--custom-line-id NAME`で明示できます。racelineとcustom lineの
同時指定は拒否します。

UI生成bundleでは、個別lineの`custom_line.json`または有効lineの`<map>_custom_line.meta.json`から
ID、表示名、開閉路、compiled CSVのSHA-256を自動取得します。manifestのpath・形式・hashとCSV実体が
一致しない場合は起動を拒否します。Section速度profileではHD mapのSHA-256も照合し、Gate編集後の
古いtrajectoryを別のSection layoutで起動しません。明示した`--custom-line-id`、`--custom-line-name`、
`--custom-line-open/--custom-line-closed`はmanifest値より優先します。
`custom` presetで`custom-line` componentと`--map`を指定した場合は、そのMapで有効化済みの
canonical Custom Lineを自動選択します。line ID・表示名・開閉路の切替は次回launchに反映されます。
現在選択中lineのgeometryと速度profileを同じCSVへ保存した変更は、走行中も上記の安全なreload対象です。

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
    lane_trajectory_topics: ["", /planning/raceline_trajectory, "", ""]
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
- centerline等のlegacy Pathは`lane_target_speeds_mps`を使います。raceline/custom lineは専用`Trajectory`の点ごとの`vx/ax`を使用し、lane target speedは追加の上限として残します。
- obstacleやsignalの認識・経路生成は未実装です。このpackageは、それらを追加できる安全な選択境界を提供します。
