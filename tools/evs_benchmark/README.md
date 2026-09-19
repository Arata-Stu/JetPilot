# EVS Native Benchmark

センサ間の共通比較方針は[../SENSOR_BENCHMARKS.md](../SENSOR_BENCHMARKS.md)を参照してください。
Jetson上のライブ250 Hz CUDA＋TensorRT評価は[ LIVE_EVALUATION.md ](LIVE_EVALUATION.md)を参照してください。

学習前のTensorRT node込み評価には、benchmark専用20ch dummy ONNXを生成できます。

```bash
python3 scripts/generate_dummy_event_onnx.py \
  --output-dir /workspaces/ros2_ws/models/e2e/dummy-event-tensor-20ch
```

このmodelは車両制御用presetから拒否され、`evs-tensorrt-benchmark`だけで使用できます。

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

### 論文用RGB・Event画像の同期出力

`rgb-evs-benchmark`で収録したsessionから、RGBの各フレーム周期に対応するevent画像を生成できます。
EVS RAWにはwall-clockの絶対時刻がないため、rosbagに残る
`/event_camera/raw_recording/request`のSTART受信時刻を共通アンカーにし、RAW先頭eventのsensor時刻へ
対応付けます。各event画像の既定windowは固定値ではなく、直前RGBフレームから現在フレームまでの
実測intervalです。

```bash
python3 /workspaces/tools/evs_benchmark/scripts/export_rgb_event_paper_frames.py \
  /workspaces/record/<session> \
  --output-dir /workspaces/record/<session>/paper_rgb_event
```

出力は`rgb/`、`event/`、左右連結済みの`pair/`、フレームごとの時刻とevent数を持つ
`frames.csv`、同期仮定を記録する`sync.json`です。event画像は白背景、ON=青、OFF=赤です。
RAW変換にはこのディレクトリでbuildした`evs_raw_to_evbin`を使用し、中間EVSBINと変換統計も
出力directoryへ保存します。変換器をまだbuildしていない場合は、SilkyEvCam対応Docker imageの
Metavision Python bindingへ自動的にフォールバックします。rosbagの読出しも、`rosbags`がある環境では
それを使用し、Jetson imageでは標準の`rosbag2_py`を使用するため、追加のpip installは不要です。

JetsonのDocker内では、ROS環境をsourceしたsystem Pythonで実行してください。

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
/usr/bin/python3 tools/evs_benchmark/scripts/export_rgb_event_paper_frames.py \
  /workspaces/record/<session> --auto-offset
```

開始要求からRAW記録開始までの固定遅延を目視で補正する場合、正値はより後のEVS eventを選びます。

```bash
python3 scripts/export_rgb_event_paper_frames.py /workspaces/record/<session> \
  --offset-ms 12.0 --start-sec 3 --duration-sec 5
```

動きのある収録では、RGBフレーム差分とevent数の相関から残差offsetを粗く推定できます。これは
hardware同期ではなく、論文図・動画を自然に見せるための視覚的な補助です。推定値と相関は
`sync.json`へ残ります。

```bash
python3 scripts/export_rgb_event_paper_frames.py /workspaces/record/<session> \
  --auto-offset --auto-search-ms 200 --auto-step-ms 2
```

固定40 ms蓄積、間引き、出力数制限も指定できます。

```bash
python3 scripts/export_rgb_event_paper_frames.py /workspaces/record/<session> \
  --window-ms 40 --every 3 --max-frames 100
```

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

## Event密度区間の選択

reorder済みEVSBINから固定長windowのevent数を数え、low（p10）、median（p50）、high（最大）を
再現可能に選択します。event payload全体をPythonで展開せず、sorted timestampへbinary searchする
ため、大容量EVSBINでも軽量です。

```bash
python3 scripts/select_evbin_segments.py \
  --input events_reordered.evbin \
  --output-dir density \
  --window-us 2000000 --step-us 100000 --top 10
```

`density_selection.json`に選択区間、`density_windows.csv`に全候補を保存します。各選択結果の
`start_offset_us`を`evs_bench --segment-start-us`へ渡し、同じ2秒長で比較します。

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

matrixは通常の性能trialとは別に、全snapshotをCPUへ戻して比較するcorrectness passを1回実行します。
このD2Hとhash計算は`wall_ms`へ混入しません。出力は次のとおりです。

- `correctness.csv`: CPU fullを基準にした全snapshot sequence checksumと不一致数
- `cpu_incremental/trace.csv`: CPU incrementalのsnapshot時系列
- `cuda_rolling/trace.csv`: CUDA staging / H2D / update / snapshot時系列
- `trace_summary.csv`: latencyとevent rateのmean / p50 / p95 / p99 / max、4 ms miss数
- `trace_timeline.svg`: event rateとsnapshot latencyの折れ線グラフ
- `*/tegrastats_timeline.csv`: Jetson telemetryの時系列
- `*/tegrastats_summary.json`: power、clock、温度、GPU/CPU使用率、run energy集計
- `*/nvidia_smi_summary.json`: x86 NVIDIA GPUの使用率、clock、温度、board power集計

correctness passまたはtraceだけを手動実行することもできます。

```bash
./build/evs_bench --input input.evbin --backend all --algorithm all \
  --width 212 --height 120 --bins 10 --window-us 40000 --stride-us 4000 \
  --verify-sequence-only --correctness-output correctness.csv

./build/evs_bench --input input.evbin --backend cuda --algorithm rolling \
  --width 212 --height 120 --bins 10 --window-us 40000 --stride-us 4000 \
  --warmup 2 --trace-only --trace-output cuda_trace.csv \
  --metadata cuda_trace_metadata.json
```

traceは測定値をメモリへ保持し、処理終了後にCSVへ書く。通常trialとは分けて実行するため、
詳細計測のoverheadを主要throughput値へ混ぜない。`tegrastats`のenergy値はprocess全体を100 ms
sampleで積分したend-to-end推定で、setupとwarm-upを含む。
x86の`nvidia-smi` energyはGPU board powerだけで、host全体の電力ではないため、Jetsonの
`VDD_IN`と絶対値を直接比較しない。

1分全体の時間変化だけを測る場合は、CPU fullや30 trialを実行せずtimeline専用scriptを使う。

```bash
./scripts/run_timeline.sh ./build/evs_bench input.evbin timeline_results \
  --segment-start-us 0 --segment-duration-us 0 \
  --width 212 --height 120 --bins 10 \
  --window-us 40000 --stride-us 4000
```

これはCPU incrementalとCUDA rollingを各2回warm-upした後、全区間を1回ずつ順次処理する。
`trace_timeline.svg`でevent密度とlatencyの時間変化を確認できる。主要な速度比較には、引き続き
low / median / highの固定2秒区間を30 trial測定した`run_matrix.sh`を使用する。

## 重要な解釈

- `cpu_util_pct`は単一プロセスのCPU時間÷wall時間です。100%は概ね1 core相当で、システム全体の利用率ではありません。
- GPU kernelが速くても、`host staging + H2D + update + snapshot`がCPU incrementalより遅ければ、前処理単体の優位性は主張できません。
- TensorRT直結時はD2H不要なので、主要比較からD2Hを除外しています。別用途でCPUへ戻す場合はD2H条件を別実験にします。
- throughputだけでなく、同じevent列・tensor定義に対する全snapshot sequence checksum一致を確認します。
- 複数backendを同時選択した場合、trialごとにchecksumが一致しなければ実験を失敗として終了します。
- 現在の実装は各snapshotで同期する`latency profiling`です。非同期pipelineのthroughputは、別のstreaming実験として混ぜずに測ります。

詳細な実験条件は[EXPERIMENT_PROTOCOL.md](EXPERIMENT_PROTOCOL.md)を参照してください。
