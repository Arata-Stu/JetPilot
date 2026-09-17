# EVS driver / 20ch前処理パイプライン技術レポート

## 1. 目的と対象範囲

本書は、SilkyEvCamの取得から20ch event representation生成、NITROS/TensorRT入力までに
JetPilotで施した設計変更と性能改善を、研究報告および再現実験に使える形でまとめる。
対象は次の3層である。

1. `openeb_ros2`: センサ取得、EVT3 packet配信、event image、native RAW記録、診断
2. `jetpilot_e2e_inference`: EVT3 decode、20ch tensor生成、CUDA rolling、非同期処理、NITROS出力
3. `evs_benchmark`: ROS 2を除外したRAW decodeおよびCPU/GPU representation比較

学習モデルそのものの精度は本書の主対象外とし、センサデータがモデル入力へ到達するまでの
性能、正しさ、時刻品質、再現性を扱う。

## 2. 現在のデータ経路

```text
SilkyEvCam / OpenEB
  -> native EVT3 callback
  -> openeb_ros2 driver: /event_camera/events_raw
  -> openeb_ros2 preprocessor: /event_camera/events
  -> AsyncEventTensorPreprocessorNode
       packet callback -> bounded packet queue
       decode worker   -> bounded decoded queue / vector pool
       GPU worker      -> pinned staging -> H2D -> CUDA rolling ring
       snapshot timer  -> [1, 20, 120, 212] FP32 NITROS tensor
  -> TensorRT
  -> control / latent-state update
```

研究用のnative benchmarkではROS 2、DDS、NITROS、TensorRTを経路から外し、RAWを一度だけ
単調時刻のEVSBINへ変換した後、同一イベント列をCPUとCUDAへ入力する。

## 3. OpenEB ROS 2 driver / preprocessorで施した工夫

### 3.1 圧縮RAW packetを維持した取得経路

driverはOpenEBのnative RAW callbackで受けたEVT3 byte列を、通常運用ではhost側でeventへ
展開せず`EventPacket`として配信する。decodeをセンサcallbackから分離できるため、取得経路の
仕事量をpacket byte数中心に保てる。

- `/event_camera/events_raw`: driverが作るEVT3 packet
- `/event_camera/events`: encoding、frame、空packetなどを検証したpreprocessor出力
- `/event_camera/event_image`: 任意で有効化する可視化用画像

event imageに購読者がいない場合はdecode/renderを行わない。20ch推論時はevent imageを無効にし、
推論に不要なCPU decodeと画像publishを避ける。

### 3.2 packet化とsubscriber-aware動作

driverはpacket durationとpacket sizeの上限でpending packetを確定する。subscriberがいない期間の
callback数も診断へ残し、不要なpublishを識別できる。packet Hzだけでセンサ能力を判断せず、
RAW callback Hz、byte/s、packet size、pending bytesを併記する設計とした。

### 3.3 event imageのwindowとstrideの分離

従来の単一FPS指定に加えて、蓄積時間`event_image_window_ms`と更新間隔
`event_image_stride_ms`を独立化した。例えば50 msの情報を保持しながら10 msごとに更新できる。
設定変更時はactive windowをclearし、異なる時間契約のeventを混在させない。

GEP互換表示ではON/OFFを別集計し、percentile scaling後にBGRへ描画する。これは可視化・既存
3chモデル用であり、20ch native tensor経路とは分離している。

### 3.4 native RAW記録

ROS event topicをMCAPへ重複保存せず、OpenEB native RAWを直接保存できる。Bag Managerから
START/STOP/SPLITを転送し、RGB MCAP、RAW、RAW metadata、biasを同じsession directoryへ置く。
これによりROS serializationとMCAP I/Oをnative EVS取得性能へ重ねず、同時にRGB参照と診断を残せる。

### 3.5 低負荷の構造化診断

hot pathでは主にatomic counterと時刻差を更新し、文字列生成とDiagnosticArray publishは既定1 Hzの
timerで行う。`debug`はconsole logだけを制御し、構造化診断はlog floodなしで記録できる。

driverで記録する主な値:

- RAW callback / publish Hz、MiB/s、平均packet size
- callback mean/max、busy率、ns/KiB
- callback inter-arrival mean/max
- no-subscriber回数、pending bytes、RAW recording状態

preprocessorで記録する主な値:

- receive / publish Hz・帯域、transport latency
- decode event rate、decode mean/max、ns/event
- event image Hz、events/image、timer cost
- GEP accumulation/render時間とbusy率
- empty、encoding、decode、out-of-bounds、no-subscriberの各count
- packet sequence gap、reorder、header lag、packet inter-arrival

`ros2 topic hz`や`ros2 topic bw`は追加subscriberとして負荷を変えるため、主要性能値には内部診断を使う。

## 4. 20ch event representation

### 4.1 表現契約

既定の表現は次のとおりである。

- bins: 10
- polarity: separate
- layout: polarity-major
- shape: `[1, 20, 120, 212]`
- dtype: FP32
- window: 40 ms
- stride: 4 ms
- temporal interpolation: none
- channel順: `positive[0:10], negative[0:10]`

線形補間は設定で選択可能だが、厳密なrolling CUDA経路はnearest histogram
（`temporal_interpolation=none`）を対象とする。

### 4.2 CPU fullとCPU incremental

CPU fullは各snapshotでwindow全体を走査する単純baselineである。CPU incrementalはbin境界へ
整列した重複windowで既存binをshiftし、新しいbinだけを更新する。

incrementalを適用できる条件:

- windowが重複する
- `window / bins`が整数µs
- strideがbin幅の整数倍
- temporal interpolationがnone

条件を満たさない場合はfullへfallbackする。半開区間を統一し、fullとincrementalの境界eventの
扱いを一致させた。

### 4.3 CUDA rolling ring

新着eventだけをGPU常駐ring histogramへ反映し、snapshot時に論理bin順へ20ch tensorを構成する。
既定では`8192 events OR 1 ms`でpinned stagingからH2D転送する。GPU出力をNITROS tensorとして
保持し、TensorRT直結経路では正しさ確認以外のD2Hを行わない。

GPU処理は次の区間を分離計測できる。

1. pageable event vectorからpinned host bufferへのstaging
2. H2D転送
3. rolling histogram update kernel
4. 20ch snapshot kernel

CUDA API enqueue時間をkernel時間とは呼ばず、kernel区間はCUDA Eventで測定する。

### 4.4 固定周期出力

イベントが少ない場合もE2E制御周期を維持するため、periodic policyではsteady clockからsensor時刻を
補間し、strideごとにwindowを前進させる。新規eventがないsnapshotでも古いbinは順にwindow外へ
抜けるため、単純な同一Tensor再送ではない。

timer遅延で飛ばしたwindow、event state age、更新なしsnapshotはそれぞれ診断へ記録する。

## 5. 単一node非同期preprocessor

従来nodeを比較用に残しつつ、250 Hz用に`AsyncEventTensorPreprocessorNode`を追加した。

### 5.1 callback、decode、GPU処理の分離

ROS subscription callbackではpacketをbounded queueへ移動するだけとし、重いdecodeを専用workerへ
分離した。decode済みeventは別のbounded queueを介してGPU workerへ渡す。GPU workerは大きな
event batchもchunkへ分割し、chunk間で4 ms snapshot deadlineを優先する。

これにより、event密度上昇時にdecodeがsnapshot timerを直接塞ぐ構造を避けた。queue overflowと
古いbatchの処理方針を暗黙にせず、drop数と最大滞留時間を診断へ残す。

### 5.2 bounded queueとstale data制御

queueは上限付きで、既定では20 msを超えて滞留したpacket/event batchを破棄する。遅れた過去stateを
後から高速で処理して制御へ渡すより、最新stateを優先する設計である。

### 5.3 decode vector buffer pool

packetごとのevent vector再確保を減らすため、GPU workerでpinned stagingへコピー後のvectorを
上限付きpoolへ返し、次のdecodeで再利用する。既定は4本、1本最大524288 events（8 MiB）。
pool hit率、miss、discard、保持event数、capacity growthを診断できる。

### 5.4 座標変換LUT

VGAから212×120への座標縮小をeventごとの除算で行わず、固定解像度用lookup tableへ置換した。
入力geometry変更時だけLUTを再構築し、`coordinate_lut_rebuilds`で確認する。

### 5.5 chunk sizeの実測調整

4096、16384、32768 eventsを比較し、転送回数、deadlineへの割込みやすさ、queue waitを評価した。
現在の主評価条件では16384 eventsを使用している。chunkを大きくしすぎるとthroughputは改善しても
snapshot期限を跨ぎやすくなるため、publish intervalとstate ageを同時に見る。

### 5.6 scheduling / DVFSを含むdecode診断

単なるdecoder関数時間だけでなく、workerのfull service wall timeとthread CPU timeを分離した。

- `decode_service_ms_*`: packet処理全体のwall時間
- `decode_thread_cpu_ms_*`: CPU上で実行された時間
- `decode_scheduling_delay_ms_*`: wallとthread CPUの差
- CPU migration数、last CPU
- current/min/max CPU frequency（診断周期sample）
- packet event数とdecode時間の相関

これにより、最大decode時間がcodec計算、vector growth、OS scheduling、DVFSのどれに由来するかを
切り分けられる。Release buildは`-O3 -DNDEBUG`であることを確認済みである。

### 5.7 診断によるhot pathへの影響抑制

診断用文字列は1 Hz timerで生成し、packet callbackとworkerではatomic値の更新を中心とする。
ただし時刻計測やatomicにもゼロではないoverheadがあるため、最終論文値では診断ON/OFF差を一度測り、
差が測定誤差内であることを示す。

## 6. timestamp逆行への対応

実RAWでは局所的なtimestamp逆行が観測された。代表例では約2.50億event中91,144件が逆行し、
中央値14 µs、p99約3.67 ms、最大8.141 msだった。大半はcallback内部で発生し、EVT3 wrap近傍だけでは
説明できなかった。

online経路では、既定で4 ms以下の逆行eventをdropしてrolling stateを維持し、4 msを超える逆行だけを
sensor clock resetとしてstate resetする。以下を診断する。

- out-of-order / dropped event数
- 最大backward jump
- timestamp reset数
- reset threshold

研究用canonical EVBINでは10 ms bounded reorderを使う。観測例では全249,730,056 eventを保持でき、
最大lateness 8.189 ms、drop 0、peak reorder buffer 244,802 eventsだった。これはoffline比較用の
sanitationであり、online pipelineへ10 msの追加遅延をそのまま導入する方針ではない。

## 7. NITROS / TensorRT統合

20ch snapshotはCUDA memory上のNITROS TensorListとしてTensorRTへ渡す。CPUへ戻して通常のROS
TensorListへ再serializeしない。event-only PilotNetでは250 Hz出力を確認している。

非同期RGB+EVS構成では、RGB encoderが低頻度でlatent stateを初期化・補正し、EVS updaterが高頻度で
stateとcontrolを更新する。State ManagerはTensorRT自体をstatefulにせず、前回のGPU出力bufferを
次回の`state_in`として保持する。in-flightは1件に限定し、入力中のstateを上書きしないping-pong構造を
採用した。

## 8. 現時点のnative benchmark結果

SilkyEvCam RAWを10 ms reorderして得た同一EVSBINから、2秒の高密度区間を選択した。

- event数: 25,945,224
- event rate: 12.972612 Mev/s
- snapshots: 481
- 最終tensor checksum: 86,570,859（3方式一致）
- 条件: 20ch、212×120、window 40 ms、stride 4 ms、30 trials

| 方式 | wall中央値 | snapshot当たり平均 | CUDA比 |
| --- | ---: | ---: | ---: |
| CPU full | 2825.209 ms | 5.874 ms | 19.97×遅い |
| CPU incremental | 336.491 ms | 0.700 ms | 2.38×遅い |
| CUDA rolling | 141.484 ms | 0.294 ms | 1.00 |

CUDA rollingの中央値内訳は、host staging 50.021 ms、H2D 22.488 ms、update kernel
18.382 ms、snapshot kernel 43.758 msだった。転送を含めてもCPU incrementalより約2.38倍高速で、
高密度になるほどCUDAの優位性が大きくなった。

この測定時点の結果はaggregate平均で、checksumも最終snapshotを対象としていた。その後、通常の
性能trialと分離した全snapshot sequence checksum、snapshot latency trace、p50/p95/p99/max、
4 ms deadline miss、時系列SVG、tegrastats自動集計を実装した。高密度RAWを使ったJetson上での
再測定と、mismatch発生時の全要素差分は未完了である。

## 9. 記録の所在

### 文書

- `ros2_ws/src/sensing/openeb_ros2/README.md`: driver、preprocessor、RAW記録、診断
- `ros2_ws/src/perception/jetpilot_e2e_inference/README.md`: 20ch、CUDA、非同期node、State Manager
- `tools/evs_benchmark/README.md`: RAW decode、EVSBIN、CPU/GPU実行方法
- `tools/evs_benchmark/EXPERIMENT_PROTOCOL.md`: 論文用の実験条件と必須metrics
- `tools/SENSOR_BENCHMARKS.md`: RGBとEVSの比較原則
- `python_ws/event_camera_analyzer/README.md`: ROS packet/imageのoffline解析
- `tools/silkyevcam_bias_tuner/README.md`: bias reset、保存、調整

### Git履歴

JetPilot本体には、CUDA backend、非同期preprocessor、timestamp tolerance、LUT、decode profiling、
buffer pool、出力rate整列、offline packet解析、native benchmarkを個別commitとして残している。
`openeb_ros2`は`ros2_ws/src/sensing/openeb_ros2`内の独立Git repositoryであり、RAW記録、bias適用、
window/stride、driver/preprocessor metricsの履歴はそちらにある。論文実験時は両repositoryのcommit
SHAを保存する。

```bash
git rev-parse HEAD
git -C ros2_ws/src/sensing/openeb_ros2 rev-parse HEAD
```

## 10. 未完了事項と次の優先順位

1. 新しいsequence correctness / traceをJetson CUDAで実測確認
2. mismatch発生時のCPU/GPU element-wise差分とmax absolute error
3. 屋内・屋外、low/median/high密度、通常clock/固定最大clockの反復測定
4. 診断ON/OFF overheadの確認
5. pageable、pinned、zero-copy / unified memoryの転送比較
6. native結果確定後、ROS intra-process、DDS、NITROS、TensorRTを加えたend-to-end比較

この順序により、センサ、decode、representation、transport、推論の要因を混同せずに報告できる。
