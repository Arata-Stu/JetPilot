# EVS / RealSense D455 比較実験

センサ単体・native前処理を先に評価し、ROS 2、TensorRT、RCカー制御は後段の統合実験として扱います。

## データ取得

| センサ | native原本 | canonical benchmark入力 | 統合評価 |
|---|---|---|---|
| SilkyEvCam | Metavision RAW | EVSBIN | rosbag2 `/events_raw`, `/events` |
| RealSense D455 | librealsense `.bag`または直接RGBBIN | RGBBIN | rosbag2 RGB topic |

RAW / librealsense bagとrosbag2は目的が異なります。native性能の主結果にはROSを含めず、同時記録はセンサ時系列とROS topicの対応・欠落確認に使用します。記録I/Oの干渉を避ける性能測定では単独記録も用意します。

## 論文に載せる比較軸

### Sensor delivery

- nominal / actual rate
- payload MiB/s
- timestamp intervalとhost arrival intervalのp50/p95/p99/max
- sequence gap、timestamp backward/reset
- CPU使用率、メモリ、消費電力、温度

### Representation preprocessing

- CPU wall time、CPU time
- host staging、H2D、kernel、snapshot
- output当たりµs、入力1秒分当たり処理時間
- deadline余裕、最大処理容量
- 出力tensor shapeとbyte数

### Streaming

- native rate-controlled replay
- live sensor
- queue depth / wait、drop、deadline miss
- burst後の回復時間
- p50/p95/p99/max end-to-end latency

### Task-level（後段）

- 同一区間・同じteacher・独立train/validation/test
- steering/throttle error、閉ループ走行性能
- TensorRTを含むcapture-to-command latency

## 比較上の禁止事項

- EVSのevent/sとD455のframe/sを、そのまま速度倍率として比較しない。
- 3ch RGBと20ch EVSでtensorサイズが異なることを隠さない。
- CUDA enqueue時間をGPU処理時間と呼ばない。
- 平均値だけで安定性を主張しない。
- hardware timestampとhost clockを同期なしで減算し、capture latencyと呼ばない。
- offline capacity、rate-controlled streaming、live、ROS統合を同じ表の同一条件として混ぜない。

## 推奨する最初の実験セット

1. 同じシーンでSilkyEvCam RAWとD455 RGBBINを30秒取得。
2. 各入力からlow/median/high activityの2秒区間を3箇所選ぶ。
3. x86_64とOrin NanoでCPU/GPUをwarm-up 5回、測定30回。
4. clock固定と通常DVFSを別条件にする。
5. checksum一致後にのみ性能値を採用する。
6. 次にnative streamingを1×、2×、burst条件で測る。
7. 最後にROS 2/TensorRTを追加し、native結果との差をintegration overheadとして示す。

実装は[evs_benchmark](evs_benchmark/README.md)と[realsense_benchmark](realsense_benchmark/README.md)に分離しています。
