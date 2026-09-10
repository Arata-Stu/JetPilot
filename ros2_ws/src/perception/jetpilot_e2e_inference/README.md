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
| `e2e_tensor_rt` | `/e2e/tensor_input` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Best Effort / Volatile | encoder出力tensor |
| `e2e_control_decoder` | `/e2e/tensor_output` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Reliable / Volatile | control model出力tensor |
| `e2e_trajectory_decoder` | `/e2e/tensor_output` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Reliable / Volatile | trajectory model出力tensor |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `e2e_pytorch_inference` | `/auto/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Reliable / Volatile | PyTorch direct control出力 |
| `e2e_image_encoder` | `/e2e/tensor_input` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Best Effort / Volatile | 前処理済み入力tensor |
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

OpenEBの通常`dark` event imageは黒背景かつ重なった極性をmagentaで描画するため、
GEP frameで学習したmodelとは互換ではありません。`event-camera` sensor profileは
生event countから白背景・赤/青・90 percentileのGEP frameを生成します。既に記録済みの
dark画像だけからcountや優勢極性を完全復元することはできないため、GEP modeでbagを
取り直す必要があります。

PyTorchノードは、学習が出力する`checkpoints/best.pt`または`last.pt`を直接
読み込めます。checkpoint内の`cfg`からモデル種別、入力寸法、mean/stdを取得します。
配備先ではcheckpointを`model.pt`という名前で配置してください。

```bash
mkdir -p /workspaces/ros2_ws/models/e2e/latest
cp /path/to/checkpoints/best.pt \
  /workspaces/ros2_ws/models/e2e/latest/model.pt
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

非対話起動の既定モデルは次のとおりです。共通の`latest`には依存しません。

| センサー | preset | モデルディレクトリ |
| --- | --- | --- |
| RealSense RGB | `e2e` | `models/e2e/camera_control` |
| RealSense RGB | `e2e-steering` | `models/e2e/camera_steering` |
| `event-camera` | `e2e` | `models/e2e/event_control` |
| `event-camera` | `e2e-steering` | `models/e2e/event_steering` |

Consoleの配備先も学習センサー・操舵のみ設定から同じ4種類へ自動選択されます。
センサープロファイル自体はモデルの保存先を変更しません。

```bash
/workspaces/scripts/bringup.sh e2e-steering --vehicle jpbb --sensor-kit event-camera
# 配備済みの別モデルを明示指定
/workspaces/scripts/bringup.sh e2e-steering --vehicle jpbb --sensor-kit event-camera \
  --e2e-model /workspaces/ros2_ws/models/e2e/my_event_steering
```

`metadata.json`を確認し、センサー・学習対象・単一画像入力が合わないモデルは拒否します。
ネットワーク入力寸法はmetadataから設定します。旧モデルで学習画像トピックの情報が
不足している場合は、正しい学習設定からONNXとmetadataを再出力してください。
候補の検索先は`E2E_MODEL_BASE`（既定`$ROS2_WS/models/e2e`）で変更できます。
実機にモデルが存在しない開発PCでの`--dry-run`では、存在しない既定モデルの検証は省略します。

`e2e_trt.sh`単体の引数なし動作は従来どおり共通`latest`です。起動するモデルを
確実にビルドするにはディレクトリを指定します。

```bash
/workspaces/scripts/e2e_trt.sh /workspaces/ros2_ws/models/e2e/event_steering
```

Jetson上でTensorRT engineを生成:

実行用コンテナ内では、プロジェクト直下のスクリプトからもビルドできます。
引数なしで`ros2_ws/models/e2e/latest/model.onnx`を使い、同じディレクトリへ
`model.plan`と`build_engine.log`を出力します（既定FP16）。ROS 2の事前ビルドは不要です。

```bash
/workspaces/scripts/e2e_trt.sh
# モデルのディレクトリまたはONNXファイルを指定する場合
/workspaces/scripts/e2e_trt.sh /workspaces/ros2_ws/models/e2e/camera_control
# FP32でビルドする場合
/workspaces/scripts/e2e_trt.sh --fp32
```

ROS 2から既存スクリプトを呼び出す場合:

```bash
ros2 run jetpilot_e2e_inference build_tensorrt_engine.sh \
  /workspaces/ros2_ws/models/e2e/latest/model.onnx \
  /workspaces/ros2_ws/models/e2e/latest/model.plan
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
  model_root:=/workspaces/ros2_ws/models/e2e/latest
```

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
/workspaces/scripts/bringup.sh e2e --vehicle vesc
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
  --set fixed_throttle:=0.2
```

`e2e-steering`は`models/e2e/camera_steering`のモデルを使い、AUTOモードで予測操舵と
固定スロットルを車両へ渡します。有効な推論結果を受信した時だけ指令をpublishするため、
推論停止時に固定スロットルだけを配信し続ける動作にはなりません。
通常の`e2e`は引き続きモデルの操舵・スロットル両方を使います。

推論ノード単独の場合は`fixed_throttle_mode:=true fixed_throttle:=0.2`を
`e2e_tensor_rt.launch.py`へ指定します。この操舵のみの経路はTensorRT用です。

## イベントカメラ単独のTensorRT制御

SilkyEvCam/OpenEBの`/event_camera/event_image`（VGA 640×480、`bgr8`）を
使う単一フレームの操舵・スロットル予測に対応します。生イベントを直接入力する
モデルではなく、ドライバーが25 Hzで生成する蓄積画像を使います。
`event-camera`センサー設定はRealSenseを起動しません。

1. データ収集時は`bringup.sh teleop --vehicle jpbb --sensor-kit event-camera --bag-manager`
   で起動し、イベント画像と教師の`/teleop/control_cmd`をrosbagへ記録します。
2. ConsoleのE2E画面で画像トピックを`/event_camera/event_image`、Learning taskを
   `Control`、画像サイズを212×120にしてデータセットを作成します。
3. 単一画像・IMUなしの`pilotnet_scratch`で学習し、ONNXへ出力します。
4. 配備プリセット`Event Camera Control`を選んで転送・TensorRTビルドします。
   配備先は`models/e2e/event_control`です。
5. Jetsonの実行用コンテナ内で起動します。

```bash
/workspaces/scripts/bringup.sh e2e --vehicle jpbb --sensor-kit event-camera
```

手動でエンジンを作る場合:

```bash
/workspaces/scripts/e2e_trt.sh /workspaces/ros2_ws/models/e2e/event_control
```

実行時は`e2e_event_image_adapter`が蓄積画像を`INTER_AREA`で212×120に縮小し、
RGBへ変換して`/e2e/event/image`へ出力します。画像前処理ノードの完全時刻同期用に、
同じheaderの`/e2e/event/camera_info`も生成します。このCameraInfoは未較正
（K=0）であり、投影や自己位置推定には使いません。正規化とNCHW化、TensorRT推論、
制御出力は既存経路を利用します。画像変換はCPU上の別プロセスで動きます。

学習と実行では同じイベント蓄積周期・色設定・biasを使用してください。
別サイズで学習した場合は起動時の`e2e_network_image_width`と
`e2e_network_image_height`も合わせます。イベント用モデルを先に配備する必要があり、
`event-camera`設定はRGB用の`latest`を自動流用しません。

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
  model_file_path:=/workspaces/ros2_ws/models/e2e/latest/model.pt \
  image_topic:=/realsense/color/image_raw \
  control_cmd_topic:=/auto/control_cmd \
  device:=cpu
```

センサーとPyTorch推論をまとめて起動する場合:

```bash
ros2 launch jetpilot_system_launch e2e.launch.py \
  model_file_path:=/workspaces/ros2_ws/models/e2e/latest/model.pt
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
