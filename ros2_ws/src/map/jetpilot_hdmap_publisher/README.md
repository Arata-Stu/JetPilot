# jetpilot_hdmap_publisher

Editable local HD map YAML から runtime 用の lane 表示と primary centerline path を publish します。
landmarks raster は HD map 作成時の下絵であり、この node は通常走行時に VSLAM
landmarks topic を必要としません。現在の VSLAM/HD map 実験フローでは VSLAM も
HD map も `map` frame を使います。

ROS package と Python module の名前は `jetpilot_hdmap_publisher` です。
既存 launch 設定との互換性のため、launch file、executable、node 名は従来名を維持しています。

## Topics

- `/hd_map/lane_markers` (`visualization_msgs/msg/MarkerArray`)
  - lane ごとの `left_bound` / `right_bound` / `centerline`
- `/hd_map/section_markers` (`visualization_msgs/msg/MarkerArray`)
  - section gate、section span、速度 override label
- `/hd_map/primary_centerline_path` (`nav_msgs/msg/Path`)
  - `primary_lane_id` の centerline
- `/localization/current_section` (`std_msgs/msg/String`)
  - `run_section_localizer:=true` のとき、`map -> base_link` TF から現在 section を publish
- `/localization/current_section_marker` (`visualization_msgs/msg/Marker`)
  - 現在 section の highlight

marker と path は YAML の `frame_id` を使います。必要なら
`frame_id_override` parameter で上書きできます。

## Algorithm

`hd_map_publisher_node.py` は YAML の lane、boundary、section gate を読み込み、RViz 表示用 marker と planning 入力用 `Path` に変換します。primary path は `primary_lane_id` の centerline だけを publish し、複数 lane の選択は `jetpilot_planning` に任せます。

`hd_map_section_localizer_node.py` は `map -> base_link` TF を周期 lookup し、section gate と lane centerline から現在 section を推定します。車両位置が lane から `max_lane_distance_m` より離れている場合や TF が取れない場合は `unknown` を publish し、planning 側の watchdog が停止判断できるようにします。

## Run

```bash
ros2 launch jetpilot_hdmap_publisher hd_map_publisher.launch.xml \
  hd_map_yaml_path:=/workspaces/map/course_a/course_a_hd_map.yaml
```

section gate から現在 section も出す場合:

```bash
ros2 launch jetpilot_hdmap_publisher hd_map_publisher.launch.xml \
  hd_map_yaml_path:=/workspaces/map/course_a/course_a_hd_map.yaml \
  run_section_localizer:=true \
  base_frame:=base_link
```

`jetpilot_system_launch` から map directory 規約を使う場合:

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_localization:=true \
  map_dir:=/workspaces/map/course_a \
  enable_hd_map_publisher:=true \
  enable_section_localizer:=true
```

`map_dir` を使うと既定で `<map_dir>/<map_dir_name>_hd_map.yaml` を読みます。
別ファイルなら `hd_map_yaml_path:=/absolute/path/to/file.yaml` を追加してください。

この package は静的 map の読込みと表示を担当し、lane 分岐、局所軌道、障害物回避、
Pure Pursuit は担当しません。raceline は offline map tool または Web Console で生成します。
runtime の planning/control は
[`docs/planning_control_architecture.md`](../../../../docs/planning_control_architecture.md)
の契約に沿って追加する予定です。

## Offline bag debug

saved cuVSLAM map と HD map/raceline 成果物を置いた `map_dir` に対して、車両
interface を無効にして bag time で起動します。

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_rosbag_replay:=true \
  rosbag:=/workspaces/record/session/take \
  enable_vehicle:=false \
  enable_localization:=true \
  map_dir:=/workspaces/map/course_a \
  enable_hd_map_publisher:=true \
  enable_section_localizer:=true \
  use_sim_time:=true
```

bagには制御topicも含まれるため、詳細は
[`docs/rosbag_replay_safety.md`](../../../../docs/rosbag_replay_safety.md) を参照してください。
