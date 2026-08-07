# jetpilot_e2e_inference

JetPilotの画像ベースE2E制御用ROS 2推論パイプラインです。

TensorRT経路に加えて、`sensor_msgs/Image`を直接受信するPyTorch経路を
提供します。PyTorch経路はIsaac ROS、NITROS、TensorRTを使用せず、CPUを
既定deviceとして起動します。

## 実行構成

```text
multi_sensor_container (component_container_mt)
  ├─ camera component
  ├─ Isaac ROS DNN Image Encoder
  └─ Isaac ROS TensorRT

GPU/NITROS tensor
  -> e2e_control_decoder
  -> jetpilot_msgs/ControlCommand
```

画像のリサイズ、RGB float32化、正規化、NCHW変換は
`isaac_ros_dnn_image_encoder`がGPU上で行います。以前のPython/OpenCV/NumPy
エンコーダは廃止しており、画像データをPythonへコピーしません。

カメラが通常の`sensor_msgs/Image`をCPUメモリでpublishする場合、
エンコーダ入口ではCPUからGPUへの転送が1回必要です。ただし、エンコーダから
TensorRTまでのテンソルはNITROS形式で同じプロセス内を流れます。

`e2e_control_decoder`は推論結果の小さなテンソルだけをROSメッセージへ変換します。
Python側では`bytes()`やNumPy配列を作らず、受信バッファの`memoryview`を参照します。

## モデル契約

ONNXモデルの既定binding名:

- input: `image`
- output: `control`

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

単独起動時は`multi_sensor_container`を作成します。

```bash
ros2 launch jetpilot_e2e_inference e2e_tensor_rt.launch.py \
  image_topic:=/realsense/color/image_raw \
  control_cmd_topic:=/auto/control_cmd \
  model_root:=/workspaces/ros2_ws/models/e2e/latest
```

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
