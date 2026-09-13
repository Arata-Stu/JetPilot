# jetpilot_e2e_inference

## Purpose

JetPilotの画像ベースE2E制御・trajectory推論用ROS 2パイプラインです。

TensorRT経路に加えて、`sensor_msgs/Image`を直接受信するPyTorch経路を
提供します。PyTorch経路はIsaac ROS、NITROS、TensorRTを使用せず、CPUを
既定deviceとして起動します。

## Nodes

| Node | Provider | Description |
| --- | --- | --- |
| `e2e_pytorch_inference` | `jetpilot_e2e_inference` | ROS ImageからPyTorchで制御指令を推論する |
| `e2e_image_encoder` | `isaac_ros_dnn_image_encoder` | ImageをTensorRT入力tensorへ変換する |
| `e2e_event_tensor_encoder` | `jetpilot_e2e_inference` | 生eventを時系列binのNCHW tensorへ変換する |
| `latent_state_manager` | `jetpilot_e2e_inference` | RGB latentとEVS tensorを非同期調停し、GPU上のstateを再利用する |
| `e2e_tensor_rt` | `isaac_ros_tensor_rt` | ONNX/TensorRT engineを実行する |
| `e2e_control_decoder` | `jetpilot_e2e_inference` | tensorを正規化制御指令へ変換する |
| `e2e_trajectory_decoder` | `jetpilot_e2e_inference` | tensorをtrajectory、速度、readyへ変換する |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `e2e_pytorch_inference` | `/realsense/color/image_raw` | `sensor_msgs/msg/Image` | Best Effort / Volatile | PyTorch経路の標準camera入力 |
| `e2e_image_encoder` | `/realsense/color/image_raw` | `sensor_msgs/msg/Image` | Best Effort / Volatile | TensorRT経路のcamera入力 |
| `e2e_image_encoder` | `/realsense/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Best Effort / Volatile | camera calibration |
| `e2e_event_tensor_encoder` | `/event_camera/events` | `event_camera_msgs/msg/EventPacket` | Best Effort / Volatile | OpenEBの圧縮済み生event packet |
| `e2e_event_tensor_encoder` | `/e2e/tensor_output` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Reliable / Volatile | consumer-driven用TensorRT完了feedback |
| `e2e_tensor_rt` | `/e2e/tensor_input` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Best Effort / Volatile | encoder出力tensor |
| `e2e_control_decoder` | `/e2e/tensor_output` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Reliable / Volatile | control model出力tensor |
| `e2e_trajectory_decoder` | `/e2e/tensor_output` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Reliable / Volatile | trajectory model出力tensor |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `e2e_pytorch_inference` | `/auto/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Reliable / Volatile | PyTorch direct control出力 |
| `e2e_image_encoder` | `/e2e/tensor_input` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Best Effort / Volatile | 前処理済み入力tensor |
| `e2e_event_tensor_encoder` | `/e2e/tensor_input` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Reliable / Volatile | CUDA memory上のevent NCHW float32 tensor |
| `e2e_event_tensor_encoder` | `/e2e/event_tensor/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Reliable / Volatile | event rate、処理時間、増分再利用状態 |
| `e2e_tensor_rt` | `/e2e/tensor_output` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Reliable / Volatile | TensorRT推論出力 |
| `e2e_control_decoder` | `/auto/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Reliable / Volatile | TensorRT direct control出力 |
| `e2e_control_decoder` | `/e2e/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Reliable / Volatile | 推論deadlineとdecoder状態 |
| `e2e_trajectory_decoder` | `/planning/trajectory` | `nav_msgs/msg/Path` | Reliable / Transient Local | `base_link`上の予測trajectory |
| `e2e_trajectory_decoder` | `/planning/target_speed` | `std_msgs/msg/Float32` | Reliable / Transient Local | trajectory modeの目標速度 |
| `e2e_trajectory_decoder` | `/planning/ready` | `std_msgs/msg/Bool` | Reliable / Transient Local | trajectory出力の有効状態 |

## Parameters

TensorRT control decoder、trajectory decoder、PyTorch経路の標準値は、それぞれ
[`config/e2e_inference.param.yaml`](config/e2e_inference.param.yaml)、
[`config/e2e_trajectory.param.yaml`](config/e2e_trajectory.param.yaml)、
[`config/e2e_pytorch.param.yaml`](config/e2e_pytorch.param.yaml)を参照してください。

TensorRT画像前処理の`input_qos`は`SENSOR_DATA`を指定し、RealSenseの
Best Effort配信（ImageとCameraInfoの両方）を受信します。前処理の既定値
`DEFAULT`ではReliable受信となり、RealSenseとのQoS不一致で推論が始まりません。
起動ログに`RELIABILITY_QOS_POLICY`が出る場合は、インストール済みlaunchにも
この設定が反映されているか確認してください。

## Assumptions / Known limits

- online TensorRT trajectory経路は単一画像・IMUなしのmodelを前提とします。
- control decoderとtrajectory decoderは同じrunではどちらか一方だけを起動します。
- direct control modeは`jetpilot_controller`と同時に有効化できません。

## 実行構成

```text
multi_sensor_container (component_container_mt)
  ├─ camera component
  ├─ Isaac ROS DNN Image Encoder
  ├─ Isaac ROS TensorRT
  └─ C++ E2E decoder component

camera ROS image
  └─ rclcpp intra-process -> DNN Image Encoder
       └─ GPU/NITROS -> TensorRT
            ├─ GPU/NITROS -> control decoder -> ControlCommand
            └─ GPU/NITROS -> trajectory decoder -> Path
```

画像のリサイズ、RGB float32化、正規化、NCHW変換は
`isaac_ros_dnn_image_encoder`がGPU上で行います。以前のPython/OpenCV/NumPy
エンコーダは廃止しており、画像データをPythonへコピーしません。

カメラが通常の`sensor_msgs/Image`をCPUメモリでpublishする場合、
エンコーダ入口ではCPUからGPUへの転送が1回必要です。ただし、エンコーダから
TensorRT、decoderまではNITROS形式で同じプロセス内を流れます。decoderでは最終的な
ROSメッセージを作るために、必要な少数のfloatだけをGPUからCPUへコピーします。

control / trajectory decoderはC++ Composable Nodeです。TensorRTと同じcontainerで
`NitrosTensorList`をintra-process購読し、通常のROS TensorListへの変換を挟みません。
推論結果のCUDA bufferから、出力に必要な小さなfloat列だけをCPUへコピーして
`ControlCommand`または`Path`へ変換します。
launchには学習側の`metadata.json`と同じ`nitros_tensor_list_nchw_rgb_f32`を
TensorRTの既定出力formatとして残しています。decoder固有のformat設定は持たず、
受信した`NitrosTensor`がfloat32かを実データから検証します。

## モデル契約

ONNXモデルの既定binding名:

- input: `image`
- control output: `control`
- trajectory output: `trajectory`

TensorRT topicの既定tensor名:

- input tensor: `input_tensor`
- output tensor: `output_tensor`

既定の画像条件:

- カメラ入力: 424 × 240、`rgb8`
- モデル入力: 212 × 120、NCHW RGB float32
- 正規化: ImageNet mean/std

`event_image_mode:=true`では、EventState既定のDSEC 3ch GEP frame統計
（mean `[0.8993729785, 0.7969581015, 0.8928228776]`、std
`[0.2204336077, 0.2921656668, 0.2204992771]`）へ自動で切り替えます。
別datasetで蒸留したcheckpointでは`event_image_mean`と`event_image_stddev`を
server側checkpointの埋め込みconfigに合わせて上書きしてください。

### 生event tensor入力

`event_tensor_mode:=true`ではImage encoderを起動せず、OpenEBの
`EventPacket`をC++でdecodeし、CUDA rolling ringから生成したimmutable
`NitrosTensorList`をTensorRTへ渡します。
出力shapeはNCHW float32で、`signed`は`[1, B, H, W]`、`separate`は
`[1, 2B, H, W]`です。既定の`B=10`、`separate`は20chになります。

`polarity_layout:=polarity_major`のchannel順は
`positive[0:B], negative[0:B]`、`time_major`は
`positive_bin0, negative_bin0, positive_bin1, negative_bin1, ...`です。
`signed`では正eventを`+1`、負eventを`-1`としてB個のchannelへ加算します。
モデルの`metadata.json`には`"modality": "event_tensor"`と、実際の入力shape
（例: `[1, 20, 120, 212]`）を保存してください。

時間方向は次の2方式を選べます。

- `event_temporal_interpolation:=none`: 各eventを1つの時間binだけへ加算するhistogram
- `event_temporal_interpolation:=linear`: 時刻位置に応じて隣接2 binへ線形配分するvoxel表現

`event_representation_backend:=cpu`で`event_incremental_mode:=auto`を使う場合は、
窓が重複し、`window/B`が整数µs、かつ
`stride`がbin幅の整数倍の場合に限り、重複binを再利用します。たとえば
`window=50ms, stride=5ms, B=10`では9 binを保持し、新しい1 binだけを作ります。
`stride >= window`、bin境界に揃わないstride、または線形補間では全再計算へ
フォールバックします。`off`は常に全再計算、`require`は再利用できない設定を
起動エラーにします。窓とbinは半開区間として扱うため、再利用時も全再計算と
同一のhistogramになります。

既定の`cuda` backendでは、`8192 events OR 1 ms`で新着eventだけをpinned bufferから
GPUへ送り、GPU常駐ringを更新します。`periodic`方式ではsensor時刻をsteady clockで
補間し、eventがない区間も`stride`ごとに半開区間の窓を前進させます。同じTensorの
再送ではなく、古いbinが順に抜けて最終的にゼロになるため、E2E更新周期を一定に保てます。
timer遅延で周期を飛ばした回数は`fixed_rate_skipped_windows`へ出力されます。

`consumer_driven`を明示した場合はTensorRTのraw出力を前回推論の完了通知として使います。
同時in-flight推論は1件で、古いsnapshotをqueueしません。raw出力が来ない場合はwatchdogで
復旧します。現行NITROS TensorListとの互換性のためsnapshotはFP32とし、TensorRT engine
内部では従来どおりFP16最適化を利用できます。

CUDA backendは現在、厳密なrolling histogramである
`temporal_interpolation:=none`に対応します。線形補間が必要な場合は
`event_representation_backend:=cpu`を指定します。CUDAでは`window/B`が整数µsで、
`stride`がそのbin幅の整数倍になる設定が必要です。

### RGB-EVS非同期latent state更新

`async_rgb_evs_latent.launch.py`はRGBとEVSを同期せず、RGB TensorRT encoderの
出力を初期state／定期補正として利用し、その間をEVS updater TensorRTで更新します。

```text
RGB image -> image encoder -> RGB TensorRT -> rgb_latent --+
                                                         |
raw EVS -> CUDA rolling representation -> event_tensor --+-> latent_state_manager
                                                              |
                                         [state_in,event_tensor,delta_t]
                                                              |
                                                        updater TensorRT
                                                              |
                                                [state_out,trajectory]
                                                   |          |
                                                   +----------+-> Path
                                                   |
                                                   +-> 次回state_in
```

State ManagerはTensorRTエンジンをステートフル化しません。前回出力のNITROS
GPU bufferを保持し、次回入力では`state_in`という名前で参照します。次の
`state_out`はTensorRT側の別output bufferへ書かれるため、入力中のstateを
上書きしないping-pong動作になります。latentをCPUへコピーする処理はありません。

実行規則は次のとおりです。

- updaterのin-flightは常に1件
- 実行中に複数のEVS tensorが届いた場合は最新版だけを保持
- RGB latentは次の安全な推論境界でstateを置換し、EVS driftを補正
- state時刻以前のEVS tensorは破棄
- `delta_t`はstate時刻とEVS窓終端時刻から算出し、設定範囲へclamp
- updater timeout時は同時推論を発行せず、安全lockを維持して診断をERRORにする
- State Managerの専用ACKをevent encoderへ返し、次のCUDA snapshot生成を許可
  （RGB補正より古いEVS窓を破棄した場合もACKする）

既定のモデルbinding契約は以下です。updaterへwaypoint headを統合し、EVS更新ごとの
追加TensorRT往復を避けます。

| Engine | Inputs | Outputs |
| --- | --- | --- |
| RGB encoder | `rgb` | `rgb_latent` |
| EVS updater + waypoint head | `state_in`, `event_tensor`, `delta_t` | `state_out`, `trajectory` |

`rgb_latent`と`state_out`は同一shape・同一dtype（現在はNITROS float32）にします。
`event_tensor`の既定shapeは`[1,20,120,212]`、`delta_t`は秒単位の`[1]`です。
モデルが`delta_t`を使用しない場合は`include_delta_t:=false`とし、updaterの
`*_tensor_names`および`*_binding_names`も2入力へ上書きしてください。

```bash
ros2 launch jetpilot_e2e_inference async_rgb_evs_latent.launch.py \
  rgb_model_root:=/workspaces/ros2_ws/models/e2e/rgb_encoder \
  updater_model_root:=/workspaces/ros2_ws/models/e2e/evs_updater
```

診断は次の2 topicで確認できます。

```bash
ros2 topic echo /e2e/event_tensor/diagnostics
ros2 topic echo /e2e/latent_state/diagnostics
```

State Manager診断にはRGBの受信・適用・置換数、EVSの最新版置換・時刻破棄数、
state version、in-flight状態、入力組立時間、updater往復時間、watchdog回数を含みます。

空間方向は低遅延を優先し、source座標をnetwork入力寸法へ直接写像します。
画像のbilinear resize相当ではありません。CPU backendの送信用bufferは複数の
host staging bufferを循環利用し、pinned memoryを利用できなければpageable memoryへ
安全にフォールバックします。

OpenEBの通常`dark` event imageは黒背景かつ重なった極性をmagentaで描画するため、
GEP frameで学習したmodelとは互換ではありません。`event-camera` sensor profileは
生event countから白背景・赤/青・90 percentileのGEP frameを生成します。既に記録済みの
dark画像だけからcountや優勢極性を完全復元することはできないため、GEP modeでbagを
取り直す必要があります。

PyTorchノードは、学習が出力する`checkpoints/best.pt`または`last.pt`を直接
読み込めます。checkpoint内の`cfg`からモデル種別、入力寸法、mean/stdを取得します。
配備先ではcheckpointを`model.pt`という名前で配置してください。

```bash
mkdir -p /workspaces/ros2_ws/models/e2e/<run-name>
cp /path/to/checkpoints/best.pt \
  /workspaces/ros2_ws/models/e2e/<run-name>/model.pt
```

`pilotnet`はPyTorchだけで動作します。`mobilenet_v3_small` checkpointを使う場合は
追加で`torchvision`が必要です。TorchScriptファイルも`model_format:=auto`または
`model_format:=torchscript`で読み込めます。

`dinov3_vits16`は学習workspaceから固定212×120 ONNXへexportし、通常の
`e2e_trt.sh`でJetson上のFP16 `model.plan`へ変換します。ネットワーク入力寸法は
`metadata.json`から212×120へ設定されます。patch境界へのpaddingはONNX内部に含まれます。
PyTorchノードでraw checkpointを直接読む経路は対象外で、比較にはONNX、
実車にはTensorRTを使用します。

## How to launch

### センサーとモデルの選択

`bringup.sh`のTUIでは、センサーを選んだ後、学習センサーと制御方式が一致する
配備済みモデルを選択します。モデル名・入力サイズ・TensorRT engineの有無を表示します。
選択は起動時に行い、実行中のホットスワップではありません。
RealSenseとEVSの両方を含むセンサー構成では、モデル選択の前に推論入力を
RGB／EVSから選択できます。どちらか一方の入力で推論し、融合は行いません。

非対話起動ではモデルディレクトリの明示指定が必須です。Consoleの配備先種別は
学習センサー・操舵のみ設定から自動判定されますが、保存先名には選択したrun名を使います。

```bash
/workspaces/scripts/bringup.sh e2e-steering --vehicle jpbb --sensor-kit event-camera \
  --e2e-model /workspaces/ros2_ws/models/e2e/<run-name>
```

`metadata.json`を確認し、センサー・学習対象・単一画像入力が合わないモデルは拒否します。
ネットワーク入力寸法はmetadataから設定します。旧モデルで学習画像トピックの情報が
不足している場合は、正しい学習設定からONNXとmetadataを再出力してください。
候補の検索先は`E2E_MODEL_BASE`（既定`$ROS2_WS/models/e2e`）で変更できます。
実機にモデルが存在しない開発PCでの`--dry-run`では、明示したモデルが存在しない場合の
内容検証は省略します。

`e2e_trt.sh`は対象ディレクトリまたはONNXファイルの指定が必須です。

```bash
/workspaces/scripts/e2e_trt.sh /workspaces/ros2_ws/models/e2e/<run-name>
```

Jetson上でTensorRT engineを生成:

実行用コンテナ内では、プロジェクト直下のスクリプトからもビルドできます。
指定したディレクトリの`model.onnx`を使い、同じディレクトリへ`model.plan`と
`build_engine.log`を出力します（既定FP16）。ROS 2の事前ビルドは不要です。

```bash
/workspaces/scripts/e2e_trt.sh /workspaces/ros2_ws/models/e2e/<run-name>
# FP32でビルドする場合
/workspaces/scripts/e2e_trt.sh /workspaces/ros2_ws/models/e2e/<run-name> --fp32
```

ROS 2から既存スクリプトを呼び出す場合:

```bash
ros2 run jetpilot_e2e_inference build_tensorrt_engine.sh \
  /workspaces/ros2_ws/models/e2e/<run-name>/model.onnx \
  /workspaces/ros2_ws/models/e2e/<run-name>/model.plan
```

Isaac ROSまたはTensorRTを更新した後は、以前の`model.plan`を再利用せず、対象Jetsonの
現在のcontainer内でこのcommandを再実行してください。

Jetson上でアップロード済みモデルを対話的に選ぶ場合は、fzf対応TUIを使用できます。
fzfがない環境では番号選択へ自動的に切り替わります。

```bash
ros2 run jetpilot_e2e_inference deploy_tensorrt_tui.sh
```

単独起動時は`multi_sensor_container`を作成します。

```bash
ros2 launch jetpilot_e2e_inference e2e_tensor_rt.launch.py \
  image_topic:=/realsense/color/image_raw \
  control_cmd_topic:=/auto/control_cmd \
  model_root:=/workspaces/ros2_ws/models/e2e/<run-name>
```

20chの生event tensorモデルは次のように起動します。

```bash
ros2 launch jetpilot_e2e_inference e2e_tensor_rt.launch.py \
  event_tensor_mode:=true \
  event_topic:=/event_camera/events \
  event_bins:=10 \
  event_window_ms:=40.0 \
  event_stride_ms:=4.0 \
  event_polarity_mode:=separate \
  event_temporal_interpolation:=none \
  event_representation_backend:=cuda \
  event_inference_policy:=periodic \
  event_cuda_update_us:=1000 \
  event_cuda_events_per_transfer:=8192 \
  event_tensor_debug:=true \
  model_root:=/workspaces/ros2_ws/models/e2e/<event-tensor-run>
```

bringup全体から使う場合は同じ設定を`e2e_` prefix付きで指定します。

```bash
/workspaces/scripts/bringup.sh e2e --vehicle jpbb --sensor-kit event-camera \
  --e2e-model /workspaces/ros2_ws/models/e2e/<event-tensor-run> \
  --set e2e_event_tensor_mode:=true \
  --set sensor_kit_silky_evcam_event_image_enabled:=false \
  --set e2e_event_bins:=10 \
  --set e2e_event_window_ms:=40.0 \
  --set e2e_event_stride_ms:=4.0 \
  --set e2e_event_inference_policy:=periodic
```

診断は`ros2 topic echo /e2e/event_tensor/diagnostics`で確認できます。
`packet_process_ms_avg/max`、`decode_ms_avg/max`、`representation_ms_avg/max`、
`transfer_enqueue_publish_ms_avg/max`、event/tensor rate、queue内event数、
CUDA update、TensorRT round-trip、sensor age、watchdog、固定周期skip、`full_windows`と
`incremental_windows`などを1秒ごとにpublishします。
`event_tensor_debug:=true`では同じ概要をログにも出します。transfer値はGPU copy完了待ちを
含まず、CUDA転送のenqueueとpublishまでのCPU時間です。

trajectoryモデルは次のように起動します。出力はcontrollerの既定入力である
`/planning/trajectory`、`/planning/target_speed`、`/planning/ready`へpublishされます。
点数とscaleは学習runの`metadata.json`に合わせてください。

```bash
ros2 launch jetpilot_e2e_inference e2e_tensor_rt.launch.py \
  output_task:=trajectory \
  model_root:=/workspaces/ros2_ws/models/e2e/camera_trajectory \
  trajectory_points:=10 \
  trajectory_scale_m:=5.0 \
  trajectory_target_speed_mps:=0.8
```

decoderは原点`(0, 0)`を先頭に加え、正規化出力をmへ戻して`base_link`座標の
`nav_msgs/Path`へ変換します。NaN、Inf、範囲外点、要素不足を検出した場合は経路を
publishせず、`/planning/ready=false`を通知します。

Isaac ROSの既存image encoderが作る入力は単一画像のNCHW tensorです。そのため
TensorRT online経路が直接扱えるtrajectory構成は現在`trajectory_pilotnet`
（単一画像・IMUなし）です。GRUの複数画像入力やIMU融合モデルはConsoleのoffline評価で
比較できますが、online利用には対応する時系列/IMU tensor producerが必要です。

既にセンサー側がコンテナを起動済みの場合は、そのコンテナへロードします。

```bash
ros2 launch jetpilot_e2e_inference e2e_tensor_rt.launch.py \
  container_name:=multi_sensor_container \
  run_standalone:=false
```

実車では、sensor、E2E推論、joy/teleop、operation mux、vehicle interfaceをまとめる
専用presetを使用します。

```bash
/workspaces/scripts/bringup.sh e2e --vehicle vesc \
  --e2e-model /workspaces/ros2_ws/models/e2e/<run-name>
```

このbringup経路ではsensor launchが`multi_sensor_container`の
`component_container_mt`を1つだけ作り、camera driver、image encoder、TensorRT、
C++ decoderをすべてそこへロードします。E2E側は`run_standalone:=false`となるため、
推論専用の別プロセスは作りません。これがintra-process/NITROS経路を維持する既定の
実行方法です。従来controllerとE2Eはどちらも`/auto/control_cmd`へpublishするため、
`enable_control`と`enable_e2e_inference`の同時有効化は拒否されます。

## RGB・固定スロットルで操舵のみ学習

`bringup.sh`のTUIで`e2e-collect`を選ぶと、カメラ・手動操舵・vehicle・bag managerを
起動し、固定スロットル値を入力できます。既定は正規化値0.2（0〜1）です。
これは速度[m/s]ではなくスロットル指令値なので、実速度を一定に保つ制御ではありません。

```bash
/workspaces/scripts/bringup.sh e2e-collect --vehicle jpbb --sensor-kit realsense \
  --set fixed_throttle:=0.2
```

初期モードは従来どおりSTOPです。MANUALへ切り替えると、R2やデッドマンボタンを
押し続けずに固定スロットルで走行し、スティックで操舵できます。固定モードでは
R2・速度調整ボタンによってスロットル値は変わりません。
L2（通常は後退に割り当てるトリガー）をデッドゾーン以上に踏むと、
スロットルを0にして踏み込み量に応じたブレーキを優先します。後退指令は出しません。
L2を離すと固定スロットルへ戻るため、停止を維持する場合はSTOPモードへ切り替えます。
STOPへ切り替えるとmuxの出力は停止指令になります。Joyの受信が途切れた場合も
既存のcommand timeoutが働きます。bagの記録開始・停止は既存のボタン操作です。
教師には`/teleop/control_cmd`を使い、MANUALで走行した区間を収集してください。

ConsoleではLearning taskを`Control`、experimentを
`Steering only · PilotNet (fixed throttle)`（`pilotnet_steering`）にして学習します。
学習headは操舵の1出力だけで、損失と学習時validation指標も操舵のみです。
ONNXは既存の`control`契約を保つため`[steering, 0]`を出力します。
2番目の値は未学習の乱数ではなくゼロ定数で、推論ノードの固定モードで置き換えます。

ConsoleのOffline teacher comparisonも`metadata.json`の`steering_only`を読み、
ゼロ定数をスロットル予測として扱いません。スロットル予測・誤差はCSVで空欄となり、
全体・区間別のスロットル誤差指標から除外します。グラフには記録されたスロットルだけを
参考表示します。既存の解析結果は自動更新されないため、変更後に解析を再実行してください。

配備プリセットは`Camera Steering Only (fixed throttle)`です。ビルド後は次で起動します。

```bash
/workspaces/scripts/bringup.sh e2e-steering --vehicle jpbb --sensor-kit realsense \
  --e2e-model /workspaces/ros2_ws/models/e2e/<run-name> \
  --set fixed_throttle:=0.2
```

`e2e-steering`は明示指定したモデルを使い、AUTOモードで予測操舵と
固定スロットルを車両へ渡します。有効な推論結果を受信した時だけ指令をpublishするため、
推論停止時に固定スロットルだけを配信し続ける動作にはなりません。
通常の`e2e`は引き続きモデルの操舵・スロットル両方を使います。

推論ノード単独の場合は`fixed_throttle_mode:=true fixed_throttle:=0.2`を
`e2e_tensor_rt.launch.py`へ指定します。この操舵のみの経路はTensorRT用です。

## イベントカメラ単独のTensorRT制御

SilkyEvCam/OpenEBの`/event_camera/event_image`（VGA 640×480、`bgr8`）を
使う単一フレームの操舵・スロットル予測に対応します。生イベントを直接入力する
モデルではなく、ドライバーが直近50 msを蓄積し、10 msスライド（100 Hz）で生成する
GEP画像を使います。
`event-camera`センサー設定はRealSenseを起動しません。

1. データ収集時は`bringup.sh teleop --vehicle jpbb --sensor-kit event-camera --bag-manager`
   で起動し、イベント画像と教師の`/teleop/control_cmd`をrosbagへ記録します。
2. ConsoleのE2E画面で画像トピックを`/event_camera/event_image`、Learning taskを
   `Control`、画像サイズを212×120にしてデータセットを作成します。
3. 単一画像・IMUなしの`pilotnet_scratch`で学習し、ONNXへ出力します。
4. 配備対象runを選んで転送・TensorRTビルドします。
   配備先は`models/e2e/<run-name>`です。
5. Jetsonの実行用コンテナ内で起動します。

```bash
/workspaces/scripts/bringup.sh e2e --vehicle jpbb --sensor-kit event-camera \
  --e2e-model /workspaces/ros2_ws/models/e2e/<run-name>
```

手動でエンジンを作る場合:

```bash
/workspaces/scripts/e2e_trt.sh /workspaces/ros2_ws/models/e2e/<run-name>
```

実行時は`e2e_event_image_adapter`が蓄積画像を`INTER_AREA`で212×120に縮小し、
RGBへ変換して`/e2e/event/image`へ出力します。画像前処理ノードの完全時刻同期用に、
同じheaderの`/e2e/event/camera_info`も生成します。このCameraInfoは未較正
（K=0）であり、投影や自己位置推定には使いません。正規化とNCHW化、TensorRT推論、
制御出力は既存経路を利用します。画像変換はCPU上の別プロセスで動きます。

学習と実行では同じイベント蓄積周期・色設定・biasを使用してください。
別サイズで学習した場合は起動時の`e2e_network_image_width`と
`e2e_network_image_height`も合わせます。イベント用モデルを先に配備する必要があり、
`event-camera`設定でも指定したモデルだけを使用します。

既にドライバーを起動している場合の推論単独起動:

```bash
ros2 launch jetpilot_e2e_inference e2e_tensor_rt.launch.py \
  event_image_mode:=true image_topic:=/event_camera/event_image \
  model_root:=/workspaces/ros2_ws/models/e2e/event_control
```

## ViT backbone共有Control・物体検出

`shared_vit_tensor_rt.launch.py`は1つの212x120入力tensorを1つのTensorRT engineへ渡し、
同じ推論結果のTensorListをControl decoderとYOLOv8互換decoderへ分岐します。backboneは
engine内で1回だけ実行されます。

```bash
ros2 launch jetpilot_e2e_inference shared_vit_tensor_rt.launch.py \
  model_root:=/workspaces/ros2_ws/models/e2e/shared_vit
```

ONNX/TensorRT bindingは`image -> control, detections`、ROS tensor名は
`input_tensor -> control_output, detection_output`です。Control decoderとDetection decoderは
別componentですが、image encoderとTensorRT nodeは共有されます。EventStateのEVS重みを使う
場合は`event_image_mode:=true`とevent topicを指定し、Detection headも同じEVS表現で学習した
checkpointを統合してください。

## PyTorch推論（起動例）

推論ノード単体を起動する場合:

```bash
ros2 launch jetpilot_e2e_inference e2e_pytorch.launch.py \
  model_file_path:=/workspaces/ros2_ws/models/e2e/<run-name>/model.pt \
  image_topic:=/realsense/color/image_raw \
  control_cmd_topic:=/auto/control_cmd \
  device:=cpu
```

センサーとPyTorch推論をまとめて起動する場合:

```bash
ros2 launch jetpilot_system_launch e2e.launch.py \
  model_file_path:=/workspaces/ros2_ws/models/e2e/<run-name>/model.pt
```

すでにカメラを起動済みなら、重複起動を避けます。

```bash
ros2 launch jetpilot_system_launch e2e.launch.py \
  enable_sensor_kit:=false \
  image_topic:=/realsense/color/image_raw
```

CUDA対応PyTorchが安定して利用できる環境では`device:=cuda`を明示できます。
`device:=auto`はCUDAが利用可能ならCUDA、そうでなければCPUを選びます。

PyTorchノードは`rclpy`プロセスとして動作するため、`rclcpp_components`の
Composable Node containerへはロードされません。カメラcomponentは従来どおり
`multi_sensor_container`で実行されます。
