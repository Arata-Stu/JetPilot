# Jetson Live RGB-EVS 250 Hz評価手順

この手順は、センサ保存性能とオンライン推論性能を別runで測ります。RAW保存I/Oを
250 Hz latencyへ混ぜないことが目的です。

## 0. 一度だけ準備

Jetson上でworkspaceをbuildし、評価対象の`rgb_event_async`モデルを配置します。
モデルの`metadata.json`は`event_sample_hz: 250.0`、CUDA互換のevent表現
（通常は40 ms window、4 ms stride、10 bins、temporal interpolationなし）である必要があります。

```bash
cd /workspaces/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Jetsonの電力modeとclock条件を固定し、論文では使用した値を明記します。mode変更で再起動した
場合は、再起動後にこのスクリプトを再実行します。

```bash
cd /workspaces
./scripts/jetson_max_performance.sh
```

スクリプトは`MAXN_SUPER / mode 2`を確認し、CPU・GPU・EMCの最大クロック固定とファンPWM 255を
適用・検証します。mode 2でなければ自動変更せず終了するため、表示された手順に従って
`nvpmodel`を変更し、必要なら再起動してから再実行します。

## 1. センサ取得baselineを記録

Terminal A:

```bash
source /workspaces/ros2_ws/install/setup.bash
bash /workspaces/scripts/bringup.sh rgb-evs-benchmark
```

Terminal Bで60秒記録します。最初の10秒程度はwarm-upとして、記録開始前に待ちます。

```bash
source /workspaces/ros2_ws/install/setup.bash
cd /workspaces/tools/evs_benchmark
./scripts/record_live_window.sh 60 sensor_static_01
```

同じ照明・動きで最低5回繰り返します。静止だけでなく、低・中・高event密度になる条件を
それぞれ記録します。保存先は`/workspaces/record/<timestamp>_<label>/`で、MCAP、OpenEB RAW、
RAW metadataが同じsessionに入ります。

固定コースを周回して条件を揃える場合は、時間指定ではなく次の対話式protocolを使えます。
既定では通常速度、低速、高速を各3周 x 3回、合計9個の独立sessionへ保存します。
バッテリー、温度、時刻の影響が特定条件だけに偏らないよう、条件順はrepeatごとに循環します
（normal-low-high、low-high-normal、high-normal-low）。
開始地点でEnterを押すと記録が始まり、3周を終えて安全に停止してから再度Enterを押すと、
そのsessionを終了して次へ進みます。

```bash
cd /workspaces/tools/evs_benchmark
./scripts/record_lap_protocol.sh 3 3 course
```

labelは`course_normal_3lap_01`、`course_low_3lap_01`、`course_high_3lap_01`などです。
各runの開始・終了UTCと所要時間は`/workspaces/record/*_course_lap_protocol.csv`にも保存されます。
運転者自身が端末を操作する場合は、必ず車両を停止してから終了Enterを押します。

Joyでsteeringだけを操作し、条件別のthrottleをソフトウェアで固定する場合は、先に別terminalで
車両interfaceを含む収録構成を起動します。`VEHICLE_PROFILE`は実機のprofileへ置き換えます。

```bash
bash /workspaces/scripts/bringup.sh rgb-evs-benchmark \
  --vehicle VEHICLE_PROFILE \
  --set teleop_fixed_throttle_mode:=true \
  --set fixed_throttle:=INITIAL_SAFE_VALUE
```

周回収録にはnormal、low、highの3条件の固定throttleをこの順で追加指定します。値は事前に
安全な直線で校正した値を使い、ここに例示値は設けません。

```bash
./scripts/record_lap_protocol.sh 3 3 course \
  NORMAL_THROTTLE LOW_THROTTLE HIGH_THROTTLE
```

各sessionの直前にSTOP modeの確認を求めてから`/teleop_cmd_node`のparameterを更新し、実際に
適用された値を表示します。走行中は三角のdeadman buttonを押し続けてsteeringをJoyで操作し、
三角を離すとzero commandへ戻ります。三角を押した状態ではL2でも固定throttleを解除してbrakeを
かけられます。固定値はprotocol CSVにも保存され、実際の制御指令はbag内の
`/teleop/control_cmd`と`/vehicle/control_cmd`で検証できます。

## 2. RAW decodeとCPU/CUDA前処理を評価

```bash
cd /workspaces/tools/evs_benchmark
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

./build/evs_raw_to_evbin INPUT.raw events.evbin callbacks.csv \
  --timestamp-policy reorder --reorder-window-us 10000 \
  --stats conversion.json

python3 scripts/select_evbin_segments.py \
  --input events.evbin --output-dir density \
  --window-us 2000000 --step-us 100000 --top 10
```

`density/density_selection.json`にあるlow、median、highの`start_offset_us`ごとに、
同じ2秒区間・30 trialで実行します。

```bash
./scripts/run_matrix.sh ./build/evs_bench events.evbin results_low \
  --segment-start-us LOW_START_US --segment-duration-us 2000000 \
  --width 212 --height 120 --bins 10 \
  --window-us 40000 --stride-us 4000 \
  --warmup 5 --trials 30
```

`LOW_START_US`と出力directoryをmedian、highにも置き換えて実行します。
判定には各directoryの`trace_summary.csv`と`correctness.csv`を使います。

## 3. EVS-onlyライブ250 Hz CUDA＋TensorRTを評価

評価対象のモデルは`metadata.json`で`modality: event_tensor`、20ch入力、40 ms window、
4 ms stride、10 bins、temporal interpolationなしを宣言している必要があります。
このrunはevent cameraだけを起動し、アクチュエータ、event image、native RAW、event payload保存を
無効にします。

学習済みモデルがまだない場合は、学習環境の`onnx` packageを使ってbenchmark専用dummyを
一度生成します。全20ch入力に依存するglobal mean reductionだけを持つため、完全な空graphより
TensorRTで最適化消去されにくい下限モデルです。

```bash
cd /workspaces/tools/evs_benchmark
python3 scripts/generate_dummy_event_onnx.py \
  --output-dir /workspaces/ros2_ws/models/e2e/dummy-event-tensor-20ch
```

Jetson上でFP16 TensorRT engineをbuildします。

```bash
/workspaces/scripts/e2e_trt.sh \
  /workspaces/ros2_ws/models/e2e/dummy-event-tensor-20ch
```

dummyは`benchmark_only: true`として保存され、通常の車両用`e2e` presetでは拒否されます。
使用できるのは評価専用の3つのEVS TensorRT benchmark presetです。以下の
`EVENT_TENSOR_MODEL`を、学習前は`dummy-event-tensor-20ch`、学習後は実モデル名に置き換えます。

dummy結果は「TensorRT nodeを含むpipeline下限」として報告します。実モデルとの差には
GPU schedulingやqueue状態の変化も含まれるため、単純な差を純粋なモデル演算時間とは断定せず、
実モデル単体の`trtexec`結果も併記します。

Terminal A:

```bash
source /workspaces/ros2_ws/install/setup.bash
bash /workspaces/scripts/bringup.sh evs-tensorrt-benchmark \
  --e2e-model /workspaces/ros2_ws/models/e2e/EVENT_TENSOR_MODEL
```

同じモデルと記録条件でCPU incremental＋full tensor H2D、およびlegacy CUDAも比較できます。

```bash
# CPU incremental -> pinned host -> full tensor H2D -> TensorRT
bash /workspaces/scripts/bringup.sh evs-cpu-tensorrt-benchmark \
  --e2e-model /workspaces/ros2_ws/models/e2e/EVENT_TENSOR_MODEL

# legacy CUDA rolling -> GPU resident tensor -> TensorRT
bash /workspaces/scripts/bringup.sh evs-legacy-cuda-tensorrt-benchmark \
  --e2e-model /workspaces/ros2_ws/models/e2e/EVENT_TENSOR_MODEL
```

3つともevent camera以外のセンサ、アクチュエータ、event image、native RAW、event payload保存を
無効にします。同時起動せず、同じ視覚刺激で1条件ずつ測定します。主要production候補は
`evs-tensorrt-benchmark`のasync CUDAです。

起動表示で次を確認します。

- `vehicle: none`
- `sensor kit: event-camera`
- `E2E input: Raw EVS 10 bins / separate polarity（cuda backend）`
- `EVS preprocess: async`

CPU presetでは`cpu backend`、`EVS preprocess: legacy`、legacy CUDA presetでは
`cuda backend`、`EVS preprocess: legacy`が表示されることを確認します。

Terminal Bで診断と出力を確認します。

```bash
ros2 topic hz /e2e/event_tensor/diagnostics
ros2 topic hz /e2e/diagnostics
ros2 topic hz /auto/control_cmd
```

warm-up後、60秒を5回、10分を3回記録します。

```bash
cd /workspaces/tools/evs_benchmark
./scripts/record_live_window.sh 60 evs_trt_250hz_01
./scripts/record_live_window.sh 60 evs_trt_250hz_02
./scripts/record_live_window.sh 60 evs_trt_250hz_03
./scripts/record_live_window.sh 60 evs_trt_250hz_04
./scripts/record_live_window.sh 60 evs_trt_250hz_05
./scripts/record_live_window.sh 600 evs_trt_250hz_soak_01
./scripts/record_live_window.sh 600 evs_trt_250hz_soak_02
./scripts/record_live_window.sh 600 evs_trt_250hz_soak_03
```

保存先は`/workspaces/record/<timestamp>_<label>/`です。主要な論文結果にはこのEVS-only runを
使用します。

## 4. 任意: ライブasync RGB-EVSを評価

このrunはアクチュエータを起動しません。event image、native RAW、RGB/event payloadも保存せず、
診断topicと推論出力だけを軽量MCAPへ記録します。

Terminal A:

```bash
source /workspaces/ros2_ws/install/setup.bash
bash /workspaces/scripts/bringup.sh rgb-evs-e2e-benchmark \
  --e2e-model /workspaces/ros2_ws/models/e2e/MODEL_NAME
```

起動表示で次を確認します。

- `vehicle: none`
- `E2E input: RGB + Raw EVS（非同期、250.0 Hz、cuda backend）`
- `EVS preprocess: async`
- `EVS image: disabled`

別terminalでtopicが揃っていることを確認します。

```bash
ros2 topic hz /e2e/event_tensor/diagnostics
ros2 topic hz /e2e/latent_state/diagnostics
ros2 topic hz /e2e/diagnostics
ros2 topic hz /auto/control_cmd
```

診断は約1 Hz、`/auto/control_cmd`は処理能力に応じた推論出力rateです。warm-up後、まず60秒を
5回、その後10分を3回記録します。

```bash
cd /workspaces/tools/evs_benchmark
./scripts/record_live_window.sh 60 live_250hz_01
./scripts/record_live_window.sh 60 live_250hz_02
./scripts/record_live_window.sh 60 live_250hz_03
./scripts/record_live_window.sh 60 live_250hz_04
./scripts/record_live_window.sh 60 live_250hz_05
./scripts/record_live_window.sh 600 live_250hz_soak_01
./scripts/record_live_window.sh 600 live_250hz_soak_02
./scripts/record_live_window.sh 600 live_250hz_soak_03
```

収録中に値を目視する場合は次を使います。

```bash
ros2 topic echo /e2e/event_tensor/diagnostics
ros2 topic echo /e2e/latent_state/diagnostics
ros2 topic echo /e2e/diagnostics
ros2 topic echo /jetson/diagnostics
```

## 5. 合格判定

最低限、全runで次を確認します。

1. `/e2e/event_tensor/diagnostics`
   - `target_output_hz = 250`
   - `deadline_misses = 0`を基本目標とする
   - `snapshot_ms_max < 4 ms`
   - `packet_queue_dropped_packets`、`decoded_queue_dropped_batches`、
     `memory_pool_exhaustions`、`publish_errors`が0
2. async RGB-EVSを実施した場合の`/e2e/latent_state/diagnostics`
   - `watchdog_count = 0`
   - `inference_completed`が継続して増加
   - `event_stale`と`event_replaced`が継続的に増えない
   - `updater_round_trip_max_ms`を必ず報告する
3. `/e2e/diagnostics`
   - deadline miss数と出力間隔を報告する
   - capture-to-command latencyはsensor clockとROS clockが一致しているrunだけ採用する
4. `/event_camera/diagnostics`
   - callback停止、長いpublish gap、event rateの異常低下がない
5. `/jetson/diagnostics`
   - thermal throttlingがない
   - 電力、温度、CPU/GPU負荷をrun条件とともに保存する

「250 Hzで動いた」という結論は、設定値だけではなく、60秒反復runと10分soakの両方で
4 ms deadline、drop、watchdog、出力継続性を満たした場合に限定します。RAW保存ありの性能も
必要なら、同じモデルで別条件として測り、RAWなしの主要結果と混ぜずに報告します。
