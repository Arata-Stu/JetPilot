# RTP Video Experiment

RealSense D455 RGB video を Jetson から notebook PC へ RTP over UDP で送り、
codec の違いが遅延、通信量、処理負荷、遠隔操作性に与える影響を評価するための
ROS 2 非依存ツールです。

## Scope

- ROS 2 / DDS は使いません。
- video は RTP over UDP で送ります。
- 比較 codec は `raw`, `mjpeg`, `h264`, `h265` です。
- H.264/H.265 は Jetson では `nvv4l2h264enc` / `nvv4l2h265enc` を優先します。
- receiver は古い frame を溜めず、最新 frame 優先で表示します。
- software timestamp は CSV に記録します。
- 物理的な画面表示遅延は LED と monitor を high-speed camera で同時撮影して別途測ります。

## Install

Jetson / notebook の両方で最低限:

```bash
sudo apt update
sudo apt install -y \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-nice \
  gir1.2-gst-plugins-bad-1.0 \
  python3-gi \
  python3-gst-1.0 \
  v4l-utils \
  iproute2 \
  chrony
```

Jetson では JetPack の multimedia GStreamer plugin が必要です。
`gst-inspect-1.0 nvv4l2h264enc` と `gst-inspect-1.0 nvv4l2h265enc` が通ることを確認します。
Console の WebRTC 表示を使う notebook/container では、さらに
GStreamer 1.20 以上であること、および `gst-inspect-1.0 webrtcbin` と
`gst-inspect-1.0 nice` が成功することを確認します。

## Quick Start

Notebook 側で receiver を起動します。

```bash
CODEC=h264 PORT=5004 ./tools/rtp_video_experiment/rtp_receiver.sh
```

Jetson 側で sender を起動します。

```bash
HOST=192.168.55.100 DEVICE=/dev/video2 CODEC=h264 \
  ./tools/rtp_video_experiment/rtp_sender.sh
```

`HOST` は notebook PC の IP address、`DEVICE` は D455 RGB camera の video device です。

## Timestamp Probe Runner

詳細な software timestamp を CSV に残す場合は Python runner を使います。

Jetson:

```bash
python3 tools/rtp_video_experiment/gst_rtp_probe.py sender \
  --codec h264 \
  --host 192.168.55.100 \
  --device /dev/video2 \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --bitrate 8000000 \
  --gop 30 \
  --duration 60 \
  --log record/rtp_video/sender_h264.csv
```

Notebook:

```bash
python3 tools/rtp_video_experiment/gst_rtp_probe.py receiver \
  --codec h264 \
  --port 5004 \
  --width 1280 \
  --height 720 \
  --fps 30 \
  --duration 65 \
  --log record/rtp_video/receiver_h264.csv
```

CSV columns:

- `wall_ns`: system clock timestamp. PTP/chrony 同期後の機器間比較に使います。
- `mono_ns`: monotonic timestamp. 同一機器内の処理時間計算に使います。
- `stage`: `acquire`, `encode_start`, `encode_done`, `rtp_send`, `rtp_recv`,
  `frame_recv_done`, `decode_done`, `render_submit`
- `seq`: stage ごとの連番
- `pts_ns`, `dts_ns`, `duration_ns`, `offset`, `offset_end`, `size_bytes`

GStreamer の buffer PTS は codec / depayloader / decoder によって変換されるため、
厳密な frame ID join は実験後に codec ごとの挙動確認が必要です。まずは同一機器内の
stage 差分、RTP packet 到着、decode/render 投入の分布を見る用途に使います。

## Codec Matrix

```bash
# raw RGB
CODEC=raw PORT=5004 ./tools/rtp_video_experiment/rtp_receiver.sh
HOST=192.168.55.100 CODEC=raw ./tools/rtp_video_experiment/rtp_sender.sh

# MJPEG
CODEC=mjpeg JPEG_QUALITY=80 PORT=5002 ./tools/rtp_video_experiment/rtp_receiver.sh
HOST=192.168.55.100 CODEC=mjpeg JPEG_QUALITY=80 PORT=5002 \
  ./tools/rtp_video_experiment/rtp_sender.sh

# H.264
CODEC=h264 BITRATE=8000000 GOP=30 PORT=5004 ./tools/rtp_video_experiment/rtp_receiver.sh
HOST=192.168.55.100 CODEC=h264 BITRATE=8000000 GOP=30 PORT=5004 \
  ./tools/rtp_video_experiment/rtp_sender.sh

# H.265
CODEC=h265 BITRATE=6000000 GOP=30 PORT=5006 ./tools/rtp_video_experiment/rtp_receiver.sh
HOST=192.168.55.100 CODEC=h265 BITRATE=6000000 GOP=30 PORT=5006 \
  ./tools/rtp_video_experiment/rtp_sender.sh
```

## Time Sync

機器間 latency を見る前に chrony または PTP を確認します。

```bash
chronyc tracking
chronyc sources -v
```

実験ログにはこの出力も保存してください。

## System Load

Jetson / notebook の両方で別 terminal から:

```bash
IFACE=eth0 DURATION=60 OUT_DIR=record/rtp_video/h264_sender \
  ./tools/rtp_video_experiment/monitor_stats.sh
```

## Network Impairment

Notebook 側または中継 interface で:

```bash
sudo ./tools/rtp_video_experiment/netem.sh apply eth0 20ms 5ms 1%
sudo ./tools/rtp_video_experiment/netem.sh show eth0
sudo ./tools/rtp_video_experiment/netem.sh clear eth0
```

## Summary

CSV の任意 column に対して統計量を出せます。

```bash
python3 tools/rtp_video_experiment/summarize_csv.py \
  record/rtp_video/sender_h264.csv --column size_bytes
```
