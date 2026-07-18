# jetpilot_rtp_tools

ROS image topic を低遅延 RTP stream として UDP 送信する package です。Jetson では NVIDIA encoder、x86 では software encoder を自動選択し、remote monitor や実験用 receiver に映像を送れます。

## Component

| Component | 役割 |
| --- | --- |
| `image_rtp_sender` | `sensor_msgs/msg/Image` を GStreamer pipeline に流し、RTP/UDP で送信する |

## Topic契約

| 方向 | Topic | 型 | 用途 |
| --- | --- | --- | --- |
| input | `image_topic` parameter の値。既定 `/realsense/color/image_raw` | `sensor_msgs/msg/Image` | RTP 化する生画像 |

入力 QoS は `SensorDataQoS` + `KeepLast(1)` です。古い frame を溜めず、最新 frame を低遅延で送ることを優先します。

## RTP algorithm

1. 最初の image を受け取るまで GStreamer pipeline は作らない
2. image encoding を `RGB8`、`BGR8`、`RGBA8`、`BGRA8`、`MONO8` のいずれかとして検査する
3. `codec` と `encoder` parameter から encoder を選ぶ
4. `appsrc -> videoconvert -> queue leaky=downstream -> encoder/payloader -> udpsink` の pipeline を構築する
5. 各 frame を packed buffer にコピーし、`appsrc` へ push する
6. 2秒ごとに pushed/dropped frame 数と GStreamer bus の error/warning を report する

`codec` は `h264`、`h265`、`mjpeg`、`raw` を選べます。`encoder=auto` では H.264 は `nvv4l2h264enc`、`x264enc` の順、H.265 は `nvv4l2h265enc`、`x265enc` の順で存在確認します。queue は downstream leaky で、詰まったときに遅延を増やすより drop を選びます。

## 起動

```bash
ros2 launch jetpilot_rtp_tools image_rtp_sender.launch.xml \
  image_topic:=/realsense/color/image_raw \
  host:=192.168.1.10 \
  port:=5004 \
  codec:=h264
```

`jetpilot_system_launch` では `sensor_kit_enable_rtp_stream:=true` にすると camera launch と同じ container 内で起動できます。
