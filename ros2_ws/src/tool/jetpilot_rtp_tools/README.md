# jetpilot_rtp_tools

## Purpose

ROS image topic を低遅延 RTP stream として UDP 送信する package です。Jetson では NVIDIA encoder、x86 では software encoder を自動選択し、remote monitor や実験用 receiver に映像を送れます。

## Nodes

| Node | Executable / Plugin | Description |
| --- | --- | --- |
| `image_rtp_sender` | `image_rtp_sender_node` / `jetpilot_rtp_tools::ImageRtpSenderComponent` | ImageをGStreamer RTP/UDP streamへ変換する |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `image_rtp_sender` | `/realsense/color/image_raw` | `sensor_msgs/msg/Image` | Best Effort / Volatile | RTP化する標準image。`image_topic`で変更可能 |

### Output topics

なし。映像はROS topicではなくUDP/RTPで送信します。

## Parameters

送信先、port、codec、encoder、bitrate、GOP、MTUなどはlaunch引数または
`jetpilot_system_launch/config/sensing/image_rtp_sender.param.yaml`で設定します。

## Assumptions / Known limits

- UDPの到達保証、再送、暗号化は行いません。
- 対応入力encodingは`RGB8`、`BGR8`、`RGBA8`、`BGRA8`、`MONO8`です。
- 実際に利用できるcodec/encoderは実行環境のGStreamer pluginに依存します。

入力 QoS は `SensorDataQoS` + `KeepLast(1)` です。古い frame を溜めず、最新 frame を低遅延で送ることを優先します。

## RTP algorithm

1. 最初の image を受け取るまで GStreamer pipeline は作らない
2. image encoding を `RGB8`、`BGR8`、`RGBA8`、`BGRA8`、`MONO8` のいずれかとして検査する
3. `codec` と `encoder` parameter から encoder を選ぶ
4. `appsrc -> videoconvert -> queue leaky=downstream -> encoder/payloader -> udpsink` の pipeline を構築する
5. 各 frame を packed buffer にコピーし、`appsrc` へ push する
6. GStreamer busを監視し、警告とエラーを常時reportする。`enable_status_log=true`のときは2秒ごとの送信統計もreportする

`codec` は `h264`、`h265`、`mjpeg`、`raw` を選べます。`encoder=auto` では H.264 は `nvv4l2h264enc`、`x264enc` の順、H.265 は `nvv4l2h265enc`、`x265enc` の順で存在確認します。queue は downstream leaky で、詰まったときに遅延を増やすより drop を選びます。

## How to launch

```bash
ros2 launch jetpilot_rtp_tools image_rtp_sender.launch.xml \
  image_topic:=/realsense/color/image_raw \
  host:=192.168.1.10 \
  port:=5004 \
  codec:=h264 \
  encoder:=auto \
  enable_status_log:=false
```

`host` は必須で、Notebook receiver のIPアドレスを指定します。`127.0.0.1`、`localhost`、`::1` はローカル試験には使えますが、remote PCには届かないため警告を出します。

正常時のINFOログは既定で無効です。警告・エラーは常に出力されます。調査時は再起動せずに切り替えられます。

```bash
ros2 param set /image_rtp_sender enable_status_log true
ros2 param set /image_rtp_sender enable_status_log false
```

`jetpilot_system_launch` では次のようにcameraと同じcomponent container内で起動できます。

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_sensor_kit:=true \
  sensor_kit_enable_rtp_stream:=true \
  sensor_kit_rtp_host:=192.168.1.10 \
  sensor_kit_rtp_port:=5004 \
  sensor_kit_rtp_enable_status_log:=false
```

Notebook側では同じcodec/portでreceiverを先に起動します。

```bash
CODEC=h264 PORT=5004 WIDTH=424 HEIGHT=240 FPS=60 \
  ./tools/rtp_video_experiment/rtp_receiver.sh
```

## Rosbagとの同時利用

RTP componentと別processの`ros2 bag record`を同時に実行できます。RTP側はcameraと同じcontainer内のintra-process経路を使い、bag recorderはROS topicを記録します。

```bash
ros2 bag record \
  -o /workspaces/record/rtp_bag_test \
  /realsense/color/image_raw \
  /realsense/color/camera_info
```

## 診断

status logの意味は次のとおりです。

- `appsrc_pushed`: ROS imageをGStreamerが受理したframe数。UDP送信成功数ではありません。
- `rtp_packets` / `rtp_bytes`: `udpsink`がsocketへ送ったRTP統計。古いGStreamerではsink-pad直前の統計へfallbackします。
- `bus_errors`: encoder、payloader、hostname解決などのGStreamer error数です。

`appsrc_pushed`だけが増えて`rtp_packets`が増えない場合はencoder/payloaderを確認します。`rtp_packets`が増えてもUDPには到達保証がないため、実機の最終確認はJetsonとNotebookの両方でpacket captureを行ってください。

```bash
# Jetson
sudo tcpdump -ni any 'udp and dst host 192.168.1.10 and dst port 5004'

# Notebook
sudo tcpdump -ni any 'udp port 5004'
```
