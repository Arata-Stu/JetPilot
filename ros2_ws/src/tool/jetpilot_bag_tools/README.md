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

`BagRequest.START` の `label` は bag directory 名の一部になります。英数字、`-`、`_` 以外は `_` に置換します。`STOP` は process group に SIGINT を送り、10秒以内に終わらない場合は SIGTERM へ進みます。

## Recording algorithm

1. START を受けると、現在時刻と label から出力 directory を作る
2. parameter から `ros2 bag record` の option を組み立てる
3. 既に recording 中なら START を無視し、状態だけ更新する
4. STOP または node shutdown で recording process を終了する
5. `status_period_s` ごとに `/bag/status` を publish する

`record_all=true` の場合は `-a` で全 topic を記録します。`record_all=false` の場合は `topics` parameter に列挙された topic だけを記録します。既定の `topics` は、制御、operation、vehicle feedback、TF、localization、RealSense、event camera を offline 解析しやすいようにまとめています。

## 起動

```bash
ros2 launch jetpilot_bag_tools jetpilot_bag_tools.launch.xml
```

通常は `jetpilot_system_launch` の `enable_bag_manager:=true` から起動します。出力先の既定値は `/tmp/jetpilot_bags` です。
