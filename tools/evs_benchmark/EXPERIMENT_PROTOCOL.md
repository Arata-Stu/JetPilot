# EVS sensor / representation 実験プロトコル

## 1. 研究上の問い

1. センサから得たイベント列は、時間順序・packet間隔・欠落の観点で安定しているか。
2. 20ch representation生成において、CPU full、CPU incremental、CUDA rollingのどれが低遅延・高throughputか。
3. CUDAの利得はH2D転送とhost stagingを含めても残るか。
4. x86_64 GPUとJetson Orin Nanoで、傾向は再現するか。
5. 後段をGPUに保持できるTensorRT直結条件で、システム全体の利得があるか。これはsensor/native検証後の第二段階とする。

## 2. 実験を混ぜない

| 段階 | 入力 | 評価対象 | ROS 2 |
|---|---|---|---|
| A. Sensor / decode | Metavision RAWまたはlive sensor | event rate、callback間隔、timestamp、decode throughput | 不使用 |
| B. Native representation | 同一EVSBIN | CPU/GPU tensor生成、転送、正しさ | 不使用 |
| C. Streaming native | live callback | queue、backpressure、deadline、連続稼働 | 不使用 |
| D. Integration | ROS topic / TensorRT | DDS、end-to-end latency、RCカー制御 | 使用 |

RAW decode時間をrepresentation時間へ含めた値と、canonical event列からのrepresentation時間は別表にします。

## 3. 固定する条件

- sensor model、firmware、bias、解像度、照明、レンズ
- event representation: `B=10`、polarity-major、20ch、nearest/no interpolation
- output: 212×120、window 40 ms、stride 4 ms（250 Hz）
- 入力区間とevent数。両machineへ同一EVSBINとSHA-256をコピー
- RAW→EVSBINでは10 msの`reorder`を固定し、decoded/written/late/drop数、drop率、最大latenessを保存
- Release/O3、compiler、CUDA、driver、JetPack、power mode、clock設定
- warm-up回数、測定回数、各試行の順番

入力密度は少なくともlow/median/highの3区間を用意し、event/sとevents/snapshotを記録します。可能なら実走データだけでなく、密度を制御できるsynthetic入力も併記します。

## 4. 必須metrics

### Sensor / decode

- events/s、bytes/s、events/callback、callback Hz
- callback inter-arrival: mean、p50、p95、p99、max
- sensor timestamp gap、backward jump、reset、sequence gap
- decode wall time、CPU time、ns/event、max RSS

### Native CPU

- wall timeとCPU time（総量、snapshot当たり、event当たり）
- effective output Hz、deadline miss率
- CPU core利用率、frequency、migration、temperature、power
- fullとincrementalの速度比

### Native CUDA

- pageable→pinned host staging
- pinned H2D
- histogram update kernel
- 20ch snapshot kernel
- GPU stage合計とhost wall time
- GPU utilization、clock、temperature、power、VRAM
- 1 snapshot同期のlatencyと、非同期streamingのsteady-state throughputを別々に測定

### 正しさ

- CPU fullをreferenceとする
- CPU incremental / CUDA outputのshape、dtype、channel order、最終tensor checksum
- representative snapshotのelement-wise max error、mismatch count
- empty bin、境界timestamp、極性、座標resize、timestamp resetをunit test

## 5. 実験手順

1. 30秒以上のRAWを固定条件で複数取得する。
2. RAW→EVSBIN変換を10 msの`reorder`で一度行い、conversion JSON、callback CSV、SHA-256を保存する。
3. EVSBINをx86_64とJetsonへ同一バイト列で配布する。
4. 各machineを固定power/clock条件にし、idle温度まで待つ。
5. 各方式をwarm-up 5回後、最低30回測る。方式の実行順はrunごとに入れ替える。
6. `results.csv`、`metadata.json`、telemetry、command、git commitを保存する。
7. correctness不一致が1件でもあれば性能比較を無効とし、先に原因を解決する。
8. 平均だけでなくp50/p95/p99、標準偏差、95% confidence intervalを報告する。

## 6. 公平性と注意点

- CPU fullは単純baseline、CPU incrementalは実運用に近いoptimized baselineとして両方必要です。
- CUDA APIのenqueue時間をkernel時間と呼ばず、CUDA Event値を用います。
- 初回CUDA context生成、allocation、JITはwarm-upへ分離します。ただしstartup latencyとして別途報告可能です。
- GPU outputをTensorRTへ渡す主用途ではD2Hを主要値に含めません。CPU consumer比較ではD2Hを加えた別条件が必要です。
- JetsonのCPU/GPUは電力・メモリ帯域を共有するため、GPU使用率だけでなくVDD_INとclockを併記します。
- rosbag replay速度やDDS queueは段階Dの要因であり、段階BのCPU/GPU優位性には混ぜません。

## 7. 次の実装項目

- snapshotごとのlatency trace（集約値だけでなく分布を保存）
- CPU/GPU全要素一致を行うcorrectness subcommand
- zero-copy / unified memory / pageable memoryとの転送比較
- CUDA Graph、double buffering、非同期2-stream throughput
- native live sensor recorderと、RAW decodeのみのfast-playback benchmark
- Nsight Systems / Nsight Compute、Linux perfによる詳細profile
- 最後にROS 2 intra-process、DDS、TensorRT統合のend-to-end比較
