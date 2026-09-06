# jetpilot_bag_tools

## Purpose

JetPilot用のrosbag操作packageです。joystickやUIからrecordingを開始・停止し、OpenEB RAW recordingと
同じsession境界へ同期できます。offlineのthrottle-speed calibration toolも提供します。

## Nodes

| Node | Executable | Description |
| --- | --- | --- |
| `bag_manager_node` | `bag_manager_node.py` | rosbag processとOpenEB RAW recordingのsessionを管理する |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `bag_manager_node` | `/bag/request` | `jetpilot_msgs/msg/BagRequest` | Reliable / Volatile | START/STOP/MARK command |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `bag_manager_node` | `/bag/status` | `jetpilot_msgs/msg/BagStatus` | Reliable / Volatile | recording状態、出力URI、最後のevent |
| `bag_manager_node` | `/event_camera/raw_recording/request` | `jetpilot_msgs/msg/BagRequest` | Reliable / Volatile | OpenEB RAWのsession同期要求 |

## Parameters

記録topic、storage、compression、split、出力先などの標準値は
[`config/bag_manager.param.yaml`](config/bag_manager.param.yaml)を参照してください。

## Assumptions / Known limits

- `BagRequest.SPLIT`はMCAPとRAWの境界ずれを防ぐため現在無効です。
- recorder processの起動には`ros2 bag` executableと書込み可能な出力先が必要です。

## Throttle-speed calibration

Record straight runs at several fixed forward throttle commands. Keep steering close to zero and
hold each command for at least 6 seconds so the final 2 seconds can be checked for steady speed.
Start with the low-speed operating range (for example 0.20, 0.25, 0.30, 0.35); do not start with
full throttle.

```bash
ros2 bag record \
  /vehicle/control_cmd \
  /visual_slam/tracking/odometry \
  /operation_mode/state
```

Analyze the completed bag in a sourced ROS 2 environment:

```bash
ros2 run jetpilot_bag_tools calibrate_throttle_from_bag.py /path/to/bag
```

The tool rejects turning, braking, reverse, short, and still-accelerating segments. It writes:

- `throttle_speed_calibration.csv`: compact measured table;
- `throttle_speed_calibration.json`: points plus accepted/rejected run diagnostics;
- `controller_throttle_calibration.param.yaml`: controller feedforward arrays.

Multiple throttle stages belong in one bag; zero/changing-command intervals split the stages and
repeated commands are aggregated. Stationary low-command points remain in CSV/JSON as useful ESC
dead-zone measurements, but speeds below `--minimum-steady-speed-mps` (default 0.05 m/s) are
excluded from the controller interpolation table.

Pass the generated controller YAML to bringup as
`control_throttle_calibration_file:=/path/to/controller_throttle_calibration.param.yaml`.
The controller linearly interpolates target speed to throttle feedforward, then applies PID only
to the remaining error.

`BagRequest.START` の `label` は bag directory 名の一部になります。英数字、`-`、`_` 以外は `_` に置換します。`STOP` は process group に SIGINT を送り、10秒以内に終わらない場合は SIGTERM へ進みます。

## Recording algorithm

1. START を受けると、現在時刻と label から重複しない出力 directory 名を決める
2. parameter から `ros2 bag record` の option を組み立てて起動する
3. rosbagの出力 directory が作成されたことを確認する
4. 確定した絶対pathをlabelに入れ、OpenEBへRAW STARTをpublishする
5. `recording_split_duration_s` が正なら、rosbagのduration分割と同じ周期でOpenEBへSPLITをpublishする
6. STOPではOpenEBへRAW STOPをpublishしてからrosbagを正常終了する
7. `status_period_s` ごとに `/bag/status` をpublishする

`record_all=true` の場合は `-a` で全 topic を記録します。`record_all=false` の場合は `topics` parameter に列挙された topic だけを記録します。既定の `topics` は、制御、operation、vehicle feedback、TF、localization、RealSense、event camera を offline 解析しやすいようにまとめています。診断情報は`/diagnostics`へ集約せず、発行元別のtopicを記録します。

`raw_recording_request_topic` を空文字にするとOpenEB連携を無効化できます。`recording_start_timeout_s` は、rosbag出力directoryの生成を待つ上限時間です。連携時のOpenEB RAWとsidecar metadataはMCAPおよび`metadata.yaml`と同じsession directoryへ保存されます。

duration分割は`recording_split_duration_s`だけで設定します。この1つの値がrosbagの`--max-bag-duration`とOpenEBへの周期SPLITの両方に使われます。`max_bag_duration`や`extra_args`からの個別上書きはエラーになります。`0`は両方のduration分割を無効化します。手動の`BagRequest.SPLIT`は境界の不一致を防ぐため無視されます。

## How to launch

```bash
ros2 launch jetpilot_bag_tools jetpilot_bag_tools.launch.xml
```

通常は `jetpilot_system_launch` の `enable_bag_manager:=true` から起動します。出力先の既定値は `/tmp/jetpilot_bags` です。
