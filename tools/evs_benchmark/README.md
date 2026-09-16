# EVS Native Benchmark

センサ間の共通比較方針は[../SENSOR_BENCHMARKS.md](../SENSOR_BENCHMARKS.md)を参照してください。

ROS 2、DDS、TensorRTを介さず、同一のイベント列から20ch event tensorを作るCPU/GPU実装を比較するための実験プロジェクトです。RCカー統合前に、センサ入力、正しさ、処理時間、転送時間、CPU/GPU使用率を切り分けます。

## 比較対象

- `cpu/full`: 各snapshotで40 ms window全体を再走査
- `cpu/incremental`: 既存9 binをshiftし、新しい1 binだけ更新
- `cuda/rolling`: pinned host bufferからH2D転送し、GPU ring histogramを更新して20ch snapshotを生成

GPUの結果はCUDA Eventで `H2D`、`update kernel`、`snapshot kernel` を個別計測します。正しさ確認用D2Hとchecksumは計測区間外です。`wall_ms`にはhost staging、CUDA API、同期を含むエンドツーエンドのホスト経過時間が入ります。

## ビルド

```bash
cd /workspaces/tools/evs_benchmark
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

CUDA compilerがなければCPU版のみ構成されます。Metavision SDKが見つかればRAW変換器も生成されます。

## 入力の作成

### RGB＋EVSを同一sessionへ収録

JetPilot bringupからセンサbenchmark専用の軽量記録を起動できます。

```bash
bash /workspaces/scripts/bringup.sh rgb-evs-benchmark
```

RCカーを走行させる条件ではvehicleを明示します。指定しなければアクチュエータは起動しません。

```bash
bash /workspaces/scripts/bringup.sh rgb-evs-benchmark --vehicle jpbb
```

このpresetはRealSense RGBをMCAPへ、SilkyEvCamをOpenEB native RAWへ記録します。
`/event_camera/events`、`events_raw`、`event_image`はMCAPへ保存しないため、native RAW取得に
ROS event packetのserialization負荷を重ねません。START/STOPは別terminalから送ります。

```bash
ros2 topic pub --once /bag/request jetpilot_msgs/msg/BagRequest \
  "{command: 1, label: factory_static_01}"

ros2 topic pub --once /bag/request jetpilot_msgs/msg/BagRequest \
  "{command: 2, label: factory_static_01}"
```

MCAP、RAW、RAW timestamp metadataは`/workspaces/record/<session>/`へまとまります。

Metavision RAWを、x86_64とJetsonで共通に読める固定長の`EVSBIN v1`へ一度だけ変換します。

```bash
./build/evs_raw_to_evbin input.raw input.evbin callbacks.csv
```

`EVSBIN`は64-byte little-endian headerと16-byte/eventから成り、timestampはµs、極性は0/1です。RAW decodeの評価とtensor生成の評価を混ぜないためのcanonical形式です。

変換器はfast playbackかつtimestamp shift無効で動作します。既定の
`--timestamp-policy reorder`では10 msのbounded reorder bufferを使い、局所的に前後したイベントを
保持したままCPU/GPUへ渡すcanonical列を単調にします。10 msより遅れて到着し、既に出力済みの
timestampより古いイベントだけを除外します。late入力数、drop数、割合、最大lateness、reorder
buffer最大要素数は`input.evbin.conversion.json`へ自動保存されます。

比較用として単純除外する`--timestamp-policy drop-nonmonotonic`と、元順序を保持する
`--timestamp-policy preserve`も残しています。ただしpreserve出力は非単調RAWではtensor benchmarkの
入力検証に失敗します。

```bash
./build/evs_raw_to_evbin input.raw input.evbin callbacks.csv \
  --timestamp-policy reorder --reorder-window-us 10000 \
  --stats conversion.json
```

表示するthroughputにはtimestamp sanitation、canonical変換、disk writeも含まれるため、純粋な
decoder速度としては扱わず、段階Aの専用decode benchmarkで別測定します。

```bash
# decoder throughput（callback内はcountのみ）
./build/evs_raw_decode_bench input.raw decode_throughput.csv --warmup 1 --trials 5

# timestamp integrity（全eventを走査するため性能表とは分離）
./build/evs_raw_decode_bench input.raw decode_integrity.csv \
  --warmup 1 --trials 5 --validate-timestamps
```

EVT3の16.777216秒timestamp wrapと逆行の関係を調べる場合は、性能測定と分けて1 trialだけ
実行します。`anomalies.csv`には全逆行と、既定では1秒以上のforward gapが記録されます。
`distance_to_evt3_wrap_us`が小さい行へ異常が集中していればwrap境界との相関があります。

```bash
./build/evs_raw_decode_bench input.raw wrap_check.csv \
  --warmup 0 --trials 1 --validate-timestamps --no-time-shift \
  --anomalies anomalies.csv \
  --evt3-wrap-proximity-us 10000 \
  --trace-forward-gap-us 1000000
```

`wrap_check.csv`では、callback境界とcallback内の逆行数、wrap前後10 ms以内の逆行数、
1秒以上のforward gap数を別々に集計します。`evt3_wrap_boundaries_crossed`は収録中に通過した
wrap境界数、`evt3_wrap_boundaries_with_backward`は逆行を伴った境界数です。
`--time-shift`との比較も可能ですが、wrap位相の評価には元timestampを保持する
`--no-time-shift`を使用してください。

逆行量の分位点、4/8 ms超の件数、100 ms時系列、異常cluster、40 ms windowへの推定影響率は、
標準Pythonだけの解析スクリプトで集計できます。

```bash
python3 scripts/analyze_timestamp_anomalies.py \
  --anomalies anomalies.csv \
  --summary wrap_check.csv \
  --hal-log hal.log \
  --output-dir timestamp_analysis \
  --window-us 40000 --stride-us 4000
```

`timestamp_anomaly_report.json`が主要集計、`backward_magnitude_histogram.csv`が逆行量の
厳密な頻度、`timestamp_anomaly_timeline.csv`が100 ms単位の時間変化、
`timestamp_anomaly_clusters.csv`が1 ms以内に連続した異常群です。

## 単一条件の実行

```bash
./build/evs_bench \
  --input input.evbin \
  --backend all --algorithm all \
  --width 212 --height 120 \
  --bins 10 --window-us 40000 --stride-us 4000 \
  --segment-start-us 0 --segment-duration-us 2000000 \
  --warmup 3 --trials 30 \
  --output results.csv --metadata metadata.json
```

入力を省略すると、決定論的なsynthetic streamを使用できます。

実EVSBINはJetsonのメモリを圧迫しないよう、既定で先頭2秒だけを読みます。別区間は`--segment-start-us`、全区間を意図的に読む場合だけ`--segment-duration-us 0`を指定します。

```bash
./build/evs_bench --synthetic-duration-s 2 --synthetic-rate-meps 5
```

## 論文用matrix

各方式を別プロセスで実行し、Jetsonでは`tegrastats`、x86 NVIDIA環境では`nvidia-smi`、両環境で`/usr/bin/time -v`を同時収集します。

```bash
./scripts/run_matrix.sh ./build/evs_bench input.evbin results \
  --width 212 --height 120 --bins 10 \
  --window-us 40000 --stride-us 4000 \
  --warmup 5 --trials 30
```

結果ディレクトリには条件ごとの生CSV、hardware metadata、git状態、入力SHA-256、実行コマンド、標準出力、resource usage、GPU telemetryと、全条件の`summary.csv`が残ります。

## 重要な解釈

- `cpu_util_pct`は単一プロセスのCPU時間÷wall時間です。100%は概ね1 core相当で、システム全体の利用率ではありません。
- GPU kernelが速くても、`host staging + H2D + update + snapshot`がCPU incrementalより遅ければ、前処理単体の優位性は主張できません。
- TensorRT直結時はD2H不要なので、主要比較からD2Hを除外しています。別用途でCPUへ戻す場合はD2H条件を別実験にします。
- throughputだけでなく、同じevent列・tensor定義に対するchecksum一致を確認します。
- 複数backendを同時選択した場合、trialごとにchecksumが一致しなければ実験を失敗として終了します。
- 現在の実装は各snapshotで同期する`latency profiling`です。非同期pipelineのthroughputは、別のstreaming実験として混ぜずに測ります。

詳細な実験条件は[EXPERIMENT_PROTOCOL.md](EXPERIMENT_PROTOCOL.md)を参照してください。
