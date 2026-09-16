# RealSense D455 Native Benchmark

EVSとの共通比較方針は[../SENSOR_BENCHMARKS.md](../SENSOR_BENCHMARKS.md)を参照してください。

D455のRGB取得とRGB tensor前処理を、ROS 2を介さずx86_64とJetson Orin Nanoで評価します。EVSとの比較では、センサdeliveryとrepresentation生成を分離します。

## 測定対象

- `realsense_live_bench`: native color streamのactual FPS、RGB帯域、frame-number gap、host arrival interval、sensor timestamp interval
- `realsense_bag_to_rgbbin`: librealsense `.bag`から共通RGBBINへの変換
- `rgb_bench --backend cpu`: RGB8→nearest resize→ImageNet normalize→NCHW float32
- `rgb_bench --backend cuda`: pageable→pinned staging、H2D、resize/normalize kernelを分離計測

CUDA出力はGPUに保持し、正しさ確認用D2Hとchecksumは計測区間外です。これはEVS benchmarkと同じTensorRT直結境界です。

## ビルド

```bash
cd /workspaces/tools/realsense_benchmark
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

## Live sensor

```bash
./build/realsense_live_bench \
  --width 848 --height 480 --fps 90 \
  --warmup-s 2 --duration-s 30 \
  --output d455_live.csv
```

設定がD455・USB接続・librealsenseで提供できない場合は起動時に失敗します。成功した場合もconfigured FPSではなく`actual_fps`とp99/max intervalを使用します。

## Offline capacity

native librealsense recorderで取得した`.bag`を共通形式へ変換します。ROS 2のMCAPとは別形式です。

```bash
# 比較用RGBBINをD455から直接取得（性能測定とは別runで行う）
./build/realsense_record_rgbbin \
  --width 848 --height 480 --fps 90 --warmup-s 2 --duration-s 30 \
  --output input.rgbbin

# 既存のlibrealsense bagを使う場合
./build/realsense_bag_to_rgbbin input.bag input.rgbbin

./build/rgb_bench \
  --input input.rgbbin --backend all \
  --width 212 --height 120 \
  --segment-start-us 0 --segment-duration-us 2000000 \
  --warmup 5 --trials 30 --output results.csv
```

RGBBINも既定で2秒区間だけロードします。長時間・高FPS RGBをJetson RAMへ全展開しないためです。

CPU/GPUを別プロセスで実行し、GPU telemetryとresource usageも保存する場合：

```bash
./scripts/run_matrix.sh ./build/rgb_bench input.rgbbin results \
  --width 212 --height 120 --warmup 5 --trials 30
```

## EVSと比較するときの原則

- D455は`frame/s`、EVSは`event/s`なので、sensor throughputの数値をそのまま倍率比較しない。
- センサ固有性能：入力帯域、周期jitter、欠落、電力を比較する。
- 前処理性能：1出力当たり時間、1秒分入力を処理する時間、deadline余裕を比較する。
- D455 RGB tensorは3×212×120、EVS tensorは20×212×120なので、tensor byte数も必ず併記する。
- native最大速度、実時間streaming、ROS/TensorRT end-to-endを別表にする。
- downstream accuracyを比較する場合は同じ区間・教師制御・分割を用いる。
- D455 hardware timestampとhost steady clockの差を絶対capture latencyとして扱わない。共通clockまたは外部同期がない場合、測れるのはinterval/jitterまで。

EVSを250 Hz、D455を90 Hzで動かすnative条件に加え、処理量を揃えた比較条件も別に用意します。異なる出力頻度を隠して単純な平均latencyだけを比較しないことが重要です。
