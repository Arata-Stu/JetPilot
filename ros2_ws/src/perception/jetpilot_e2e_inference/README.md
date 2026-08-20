# jetpilot_e2e_inference

JetPilotの画像ベースE2E制御・trajectory推論用ROS 2パイプラインです。

TensorRT経路に加えて、`sensor_msgs/Image`を直接受信するPyTorch経路を
提供します。PyTorch経路はIsaac ROS、NITROS、TensorRTを使用せず、CPUを
既定deviceとして起動します。

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

## 起動

Jetson上でTensorRT engineを生成:

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

JetPilot bringupからは次のように有効化できます。RealSense colorは
E2E推論やRTPの設定に関係なく常時有効です。

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_sensor_kit:=true \
  enable_e2e_inference:=true
```

このbringup経路ではsensor launchが`multi_sensor_container`の
`component_container_mt`を1つだけ作り、camera driver、image encoder、TensorRT、
C++ decoderをすべてそこへロードします。E2E側は`run_standalone:=false`となるため、
推論専用の別プロセスは作りません。これがintra-process/NITROS経路を維持する既定の
実行方法です。

## PyTorch推論

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
