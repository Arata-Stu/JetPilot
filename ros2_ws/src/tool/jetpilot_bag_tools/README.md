# jetpilot_bag_tools

JetPilot 用の rosbag 操作 package です。joystick や UI から `/bag/request` を publish するだけで、走行中の記録開始・停止と状態監視を行えます。

## Node

| Node | 役割 |
| --- | --- |
| `bag_manager_node.py` | `ros2 bag record` process を起動・停止し、recording 状態を publish する |

## Topic契約

| 方向 | Topic | 型 | 用途 |
| --- | --- | --- | --- |
| input | `/bag/request` | `jetpilot_msgs/msg/BagRequest` | START/STOP/SPLIT/MARK command |
| output | `/bag/status` | `jetpilot_msgs/msg/BagStatus` | recording 中か、出力 URI、最後の event |
| output | `/event_camera/raw_recording/request` | `jetpilot_msgs/msg/BagRequest` | OpenEB RAWを同じsessionで開始・停止する |

`BagRequest.START` の `label` は bag directory 名の一部になります。英数字、`-`、`_` 以外は `_` に置換します。`STOP` は process group に SIGINT を送り、10秒以内に終わらない場合は SIGTERM へ進みます。

## Recording algorithm

1. START を受けると、現在時刻と label から重複しない出力 directory 名を決める
2. parameter から `ros2 bag record` の option を組み立てて起動する
3. rosbagの出力 directory が作成されたことを確認する
4. 確定した絶対pathをlabelに入れ、OpenEBへRAW STARTをpublishする
5. STOPではOpenEBへRAW STOPをpublishしてからrosbagを正常終了する
6. `status_period_s` ごとに `/bag/status` をpublishする

`record_all=true` の場合は `-a` で全 topic を記録します。`record_all=false` の場合は `topics` parameter に列挙された topic だけを記録します。既定の `topics` は、制御、operation、vehicle feedback、TF、localization、RealSense、event camera を offline 解析しやすいようにまとめています。

`raw_recording_request_topic` を空文字にするとOpenEB連携を無効化できます。`recording_start_timeout_s` は、rosbag出力directoryの生成を待つ上限時間です。連携時のOpenEB RAWとsidecar metadataはMCAPおよび`metadata.yaml`と同じsession directoryへ保存されます。

## 起動

```bash
ros2 launch jetpilot_bag_tools jetpilot_bag_tools.launch.xml
```

通常は `jetpilot_system_launch` の `enable_bag_manager:=true` から起動します。出力先の既定値は `/tmp/jetpilot_bags` です。
