# EVS benchmark evaluation results

更新日: 2026-09-22

この文書は、Jetson Orin Nano上で実施したEVS pipeline評価の確定結果と、その解釈をまとめる。
再現手順は[`LIVE_EVALUATION.md`](LIVE_EVALUATION.md)、実験条件の定義は
[`EXPERIMENT_PROTOCOL.md`](EXPERIMENT_PROTOCOL.md)を参照すること。

## 1. 評価範囲

評価環境とpipelineは以下のとおり。

- NVIDIA Jetson Orin Nano Super
- `MAXN_SUPER`および`jetson_clocks`を適用
- EVS入力: 640 x 480、EVT3
- Event Tensor: 212 x 120、10 temporal bins、separate polarity、20 channels
- Event window: 40 ms
- 出力stride: 4 ms（250 Hz）
- 実際のIsaac ROS `TensorRTNode`とNITROS Tensorを使用
- `component_container_mt`内でintra-process通信を使用
- TensorRT入力前のasync event state更新にはCUDAを使用
- TensorRT出力は`/benchmark/e2e/control_cmd`へ隔離し、車両の自動制御には使用しない

TensorRTモデルには学習前の20-channel benchmark専用dummy ONNXを使用した。したがって、
本結果が評価するのはEVS前処理、NITROS受け渡し、TensorRT nodeおよびdecoderを含むpipelineの
実行性能であり、最終学習モデルの精度や計算量ではない。

## 2. 実走行データセット

同一コースで固定スロットルを使用し、以下の9セッションを記録した。

| 条件 | 固定スロットル | 取得回数 | 1セッション |
|---|---:|---:|---:|
| low | 0.2 | 3 | 3 laps |
| normal | 0.3 | 3 | 3 laps |
| high | 0.4 | 3 | 3 laps |

データセットの集計値:

- 9セッション
- 9,191,403,686 events
- 変換後EVSBIN: 136.963 GiB
- RAW合計: 19.31 GiB
- セッション平均event rate: 18.29--21.99 Mevent/s
- 選択した2秒間の高密度区間: 最大33.93 Mevent/s

Jetson内の記録および解析場所:

```text
/workspaces/record/09-19-E522
/workspaces/record/09-19-E522/analysis
```

## 3. RAW decodeおよびtimestamp検証

9セッションのRAW decode throughputは85.4--90.7 Mevent/sであり、記録時の平均入力event rateを
十分に上回った。

EVT3 timestampには最大4,095 usの一時的な逆行を観測した。RAWからEVSBINへの変換では
10 msのreorder windowを使用し、全9セッションで以下を確認した。

- decoded eventsとwritten eventsが一致
- reorder windowを超えたdropは0
- EVT3 wrap境界でのbackward eventは0
- EVSBINのheader event countと実ファイルサイズが一致

EVSBIN変換速度は約2.44--2.57 Mevent/sだった。この値は大容量のcanonical file生成速度であり、
オンラインpipelineの処理性能としては扱わない。

## 4. Offline Event Tensor benchmark

代表的な約2秒、67,868,222 events、490 snapshotsの区間で得られた結果を示す。

| backend / algorithm | 全体wall time | 1 snapshotあたりの概算 |
|---|---:|---:|
| CPU full | 7,355.29 ms | 15.01 ms |
| CPU incremental | 800.61 ms | 1.63 ms |
| CUDA rolling | 371.00 ms | 0.76 ms |

3実装のchecksumは一致した。したがって、少なくともこの評価入力ではCPUとCUDAで同じEvent Tensorを
生成している。

結果の解釈:

- CPU full再計算は4 ms周期を満たさない。
- CPU incrementalは定常状態で250 Hz相当の処理が可能。
- CUDA rollingが最速。
- low、median、high density matrixではCPU incrementalとCUDAの双方で最初のsnapshotだけが
  4 msを超えた。これは最初の40 ms windowを構築する起動時過渡であり、定常状態とは分けて扱う。

## 5. Online async CUDA + TensorRT正式結果

絶対レイテンシの正式結果には、timestamp定義を修正した`latency_v4`のみを使用する。

```text
profile: async_cuda_v4
recording: normal speed, throttle 0.3, 5 laps
duration: 43.472577013 s
outputs: 10,861
analysis:
/workspaces/record/online_async_cuda_trt_latency_v4_analysis
```

Tensor header timestampには、scheduled timeを上限とした、実際に表現へ含まれる最新event timestampを
使用した。

```text
tensor_header_timestamp_source=freshest_event_capped_at_schedule
```

これにより、未来時刻のゼロクランプを含まないAge of Informationを計測する。

### 5.1 出力周期と健全性

| 指標 | 結果 |
|---|---:|
| Output rate | 249.943 Hz |
| Output interval p99 | 4.597 ms |
| Output interval max | 15.855 ms |
| Future source timestamps | 0 |
| Matched TensorRT inputs | 10,839 / 10,861（99.80%） |
| Event deadline misses | 3 |
| Skipped windows | 2 |
| Other event errors / drops | 0 |

event deadline miss率は約0.028%、window skip率は約0.018%である。継続的な処理能力不足ではなく、
一時的なスケジューリング遅延と解釈する。したがって「missおよびskipが0」とは主張しない。

### 5.2 End-to-End Age of Information

最新event timestampからcontrol command出力までの時間を示す。

| 統計 | Latency |
|---|---:|
| Mean | 5.004 ms |
| p50 | 5.025 ms |
| p95 | 9.434 ms |
| p99 | 9.704 ms |
| Max | 18.831 ms |

### 5.3 Pipeline breakdown

| 区間 | Mean | p99 |
|---|---:|---:|
| Latest event -> TensorRT input | 4.558 ms | 9.181 ms |
| TensorRT input -> TensorRT output | 0.281 ms | 0.634 ms |
| Latest event -> TensorRT output | 4.840 ms | 9.508 ms |
| TensorRT output -> command | 約0.164 ms | 未集計 |

TensorRT区間は全体平均の約5.6%であり、このdummy modelを使った評価ではTensorRT推論は
ボトルネックではない。支配的なのはsensor-to-tensor-input区間である。

ただし、sensor-to-tensor-inputには次が含まれる。

- 最新eventが発生するまでの時間
- センサからホストへのpacket転送
- decodeおよびqueue wait
- CUDA event state更新
- 4 ms周期のsnapshot待ち
- Tensor publishおよびTensorRT inputへの受け渡し

したがって、4.558 msをEVSハードウェア単体の遅延として扱ってはならない。ハードウェア遅延を
分離するには、LEDなどの外部triggerとevent到着を比較する別実験が必要である。

## 6. 4 ms deadlineの解釈

`latency_v4`では、10,861出力中5,758出力、53.02%が4 msを超えた。この値は
「TensorRT推論の53%がdeadlineに間に合わなかった」という意味ではない。

この判定は、最新event timestampからcommandまでのAge of Informationを4 msと比較している。
一方、TensorRT区間は平均0.281 ms、p99 0.634 msであるため、TensorRT処理自体は4 ms周期に
十分収まっている。

論文では次の2点を分けて報告する。

1. 250 Hzの出力周期を維持できたか: **維持できた（249.943 Hz）**
2. 最新センサ情報から常に4 ms以内にcommandを生成できたか: **達成していない**

## 7. 結論

現時点の結果から、以下を主張できる。

- Jetson Orin Nano上で、async CUDA Event Tensor、NITROS Tensor、TensorRT nodeおよびdecoderを
  含むpipelineが実走行中に約250 Hzで動作した。
- CPU incrementalもoffline定常状態では250 Hz相当のEvent Tensor生成能力を示した。
- CUDA rollingはCPU incrementalより高速だった。
- online async pipelineのAge of Informationは平均5.00 ms、p99 9.70 msだった。
- dummy modelのTensorRT区間は平均0.281 ms、p99 0.634 msであり、主要な遅延源ではなかった。
- 主要な時間は、eventの鮮度、転送、decode、scheduleおよびTensor入力生成を含む
  sensor-to-tensor-input区間に存在した。
- 継続的なbacklogは観測されず、稀な遅延時には古いwindowをskipして現在のscheduleへ復帰した。

## 8. 結果の使用制限

現時点では、以下は未評価または暫定結果である。

- 学習済み20-channel実モデルのTensorRT latencyと精度
- モデル出力による自律走行性能
- EVSセンサハードウェア単体の応答遅延
- RGB-only pipelineとの同条件比較
- 最新timestamp定義を使用したCPU、legacy CUDA、async CUDAの完全なonline比較
- 複数の独立した5-lap runによるrun間ばらつき

旧`async latency_v3`ではlogical schedule timestampが未来になる場合があり、負値を0へclampしていた。
そのため、v3の絶対E2E latencyを論文結果へ使用しない。v3のTensorRT区間と出力rateは参考値としてのみ
扱う。正式な絶対レイテンシにはv4を使用する。

## 9. 論文向け要約文

> Jetson Orin Nano上の実走行評価において、非同期CUDAイベント表現生成、NITROS転送および
> TensorRT推論を含むpipelineは249.94 Hzの出力レートを達成した。最新イベントから制御指令までの
> Age of Informationは平均5.00 ms、p99で9.70 msであった。TensorRT入力から出力までの観測
> レイテンシは平均0.281 ms、p99で0.634 msであり、主要な遅延は推論ではなく、イベントの鮮度、
> 転送、デコードおよびスケジューリングを含むsensor-to-tensor-input区間に存在した。

## 10. 保存すべき成果物

少なくとも次を実験成果物として保持する。

```text
/workspaces/record/09-19-E522
/workspaces/record/09-19-E522/analysis/timestamp
/workspaces/record/09-19-E522/analysis/evbin
/workspaces/record/09-19-E522/analysis/benchmark_matrix
/workspaces/record/online_async_cuda_trt_latency_v4_analysis/online_tensorrt_summary.csv
/workspaces/record/online_async_cuda_trt_latency_v4_analysis/online_tensorrt_diagnostics.json
```
