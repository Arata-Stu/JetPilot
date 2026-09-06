# jetpilot_object_detection

## Purpose

Jetson Orin Nano上でcuVSLAMと共存させるための、小さいYOLOv8 TensorRT runtimeです。
実機ではIsaac ROSの画像前処理とTensorRT、このpackageのC++ decoderだけを動かします。
学習時だけ必要なUltralyticsはROS workspaceへ含めません。depth画像は使用しません。
検出入力だけを既定15 Hzへ間引くframe gateを同じcomponent container内に置き、VSLAMが使う
camera streamのrateは変更しません。`max_inference_fps`は実機計測に応じて調整できます。

## Nodes

| Node | Provider | Description |
| --- | --- | --- |
| `object_detection_image_gate` | `jetpilot_object_detection` | camera入力を推論rateへ間引く |
| `object_detection_image_encoder` | `isaac_ros_dnn_image_encoder` | gated imageをYOLO入力tensorへ変換する |
| `object_detection_tensor_rt` | `isaac_ros_tensor_rt` | YOLOv8 TensorRT engineを実行する |
| `yolov8_decoder` | `jetpilot_object_detection` | raw tensorをNMS処理してDetection2DArrayへ変換する |

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `object_detection_image_gate` | `/realsense/color/image_raw` | `sensor_msgs/msg/Image` | Best Effort / Volatile | 標準RGB camera入力 |
| `object_detection_image_gate` | `/realsense/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Best Effort / Volatile | camera calibration |
| `object_detection_image_encoder` | `/perception/object_detection/image` | `sensor_msgs/msg/Image` | Best Effort / Volatile | rate制限済みRGB image |
| `object_detection_image_encoder` | `/perception/object_detection/camera_info` | `sensor_msgs/msg/CameraInfo` | Best Effort / Volatile | rate制限済みcamera info |
| `object_detection_tensor_rt` | `/perception/object_detection/tensor_input` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Best Effort / Volatile | 前処理済みtensor |
| `yolov8_decoder` | `/perception/object_detection/tensor_output` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Best Effort / Volatile | YOLOv8 raw output |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `object_detection_image_gate` | `/perception/object_detection/image` | `sensor_msgs/msg/Image` | Best Effort / Volatile | rate制限済みRGB image |
| `object_detection_image_gate` | `/perception/object_detection/camera_info` | `sensor_msgs/msg/CameraInfo` | Best Effort / Volatile | 対応するcamera info |
| `object_detection_image_encoder` | `/perception/object_detection/tensor_input` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Best Effort / Volatile | YOLO入力tensor |
| `object_detection_tensor_rt` | `/perception/object_detection/tensor_output` | `isaac_ros_tensor_list_interfaces/msg/TensorList` | Best Effort / Volatile | YOLOv8 raw output |
| `yolov8_decoder` | `/perception/detections` | `vision_msgs/msg/Detection2DArray` | Best Effort / Volatile | 検出box、class、confidence |
| `yolov8_decoder` | `/perception/object_detection/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Reliable / Volatile | decoder状態 |

## Parameters

class名、画像寸法、confidence、NMSなどの標準値は
[`config/yolov8.param.yaml`](config/yolov8.param.yaml)にあります。信号用class構成は
[`config/yolov8_signal.param.yaml`](config/yolov8_signal.param.yaml)で上書きします。

## Assumptions / Known limits

- NVIDIA GPU、Isaac ROS DNN image encoder、TensorRTを必要とします。
- ONNX出力はNMS前のchannel-major float32 tensorを前提とします。
- 標準topic名は一般物体用です。信号検出ではdetections topicを`/perception/signal/detections`へ変更します。

## モデル契約

- 入力: RGB、`1x3x224x224`、letterbox、画素値`0..1`
- 出力: Ultralytics YOLOv8のNMS前float32 tensor
  `[1, 4 + class_count, candidate_count]`
- 224入力の標準候補数: `28^2 + 14^2 + 7^2 = 1029`
- 初期クラス順: `0: vehicle`, `1: barrier`
- ROS出力: `vision_msgs/Detection2DArray` (`/perception/detections`)

候補数は固定値にせず、tensorの全要素数とクラス数からdecoderが算出します。このため
224入力だけでなく、後から入力サイズを変えたモデルにも対応できます。ONNX exportでNMSを
組み込まないでください。NMS込みの`[1, N, 6]`出力はこのdecoderの契約外です。

## 学習workspaceとの境界

dataset検査、Roboflow/アノテーション規約、学習・再学習、ONNX export、配備処理は
[`python_ws/jetpilot_object_detection_training`](../../../../python_ws/jetpilot_object_detection_training/README.md)
が担当します。このROS packageへPyTorchやUltralyticsの依存関係を追加しないでください。

学習側は`model.onnx`と`metadata.json`を出力し、対象JetsonのIsaac ROS container上で
`model.plan`を生成します。PCで生成したTensorRT engineの持ち込みは避けてください。
配備後の起動例:

```bash
ros2 launch jetpilot_object_detection yolov8_tensor_rt.launch.py \
  model_root:=/workspaces/ros2_ws/models/yolov8/latest
```

クラスを変更したときは、学習側`metadata.json`、datasetの`data.yaml`、
`config/yolov8.param.yaml`の`class_names`を同じ順番にし、decoder testとTensorRT engineを
更新します。

## rosbag評価

既定のbag設定は`/perception/detections`とdecoder diagnosticsを記録します。JetPilot Consoleの
bag解析は`Detection2DArray`を自動検出し、primary RGB画像へboxとラベルを重ねた
`/perception/detections_overlay` channelを生成します。overlayはoffline解析時だけ作られるため、
走行中のJetsonに描画処理を追加しません。

元bagに検出結果がない場合は、JetPilot ConsoleのBag Analysisで
`Run YOLOv8 after recording`を有効にします。解析taskは次の順序で動作します。

1. 指定したraw RGBとCameraInfoだけを隔離ROS domainで再生する。
2. 選択した`model.onnx`をTensorRTパイプラインへ入力する。
3. `Detection2DArray`だけを`<analysis>/detections/sidecar`へ記録する。
4. 元画像のheader timestampでsidecarと画像を同期し、overlay channelを生成する。

元rosbagは変更しません。sidecarの`manifest.json`には元bag metadataとONNX/TensorRT engineの
SHA-256、閾値、入力サイズ、推論FPS、再生速度を保存します。同じbagを異なるモデルで解析する場合も、
解析jobごとに独立したsidecarが生成されます。TensorRTを使うため、このoffline推論はJetsonまたは
対応するNVIDIA GPUを割り当てたIsaac ROS環境で実行してください。

## How to launch

```bash
ros2 launch jetpilot_object_detection yolov8_tensor_rt.launch.py \
  model_root:=/workspaces/ros2_ws/models/yolov8/latest
```

## 短期追跡ID

標準設定ではNMS後にByteTrack風の2段階Trackerを実行し、確定した追跡IDを
`Detection2D.id`へ付けます。新しいモデルの学習・engine生成は不要です。
既存のbox・class・confidenceの出力閾値は保ち、低信頼度boxは内部の追跡維持だけに使います。
Bag AnalysisはIDを保存し、検出overlayに追跡番号を表示します。

`enable_tracking: false`で無効化できます。初期値は2観測で確定、消失保持0.5秒です。
IDは短期追跡の識別子であり、周回をまたぐ相手車両の恒久的な個体IDではありません。
詳細な設定・ReIDの実装検討はrepoの `docs/object_tracking_reid.md` を参照してください。

## Map上の追跡可視化

`opponent_projection.launch.py`でID付き検出を路面へ投影し、Foxglove用のMarkerArrayと
ID別表示スロットのPathを配信できます。bringupでは `enable_opponent_projection:=true` で
有効になります。詳細な起動・表示手順はrepoの `docs/opponent_visualization.md` を参照してください。

## 外観ReID（C++ composable node）

`ReidNode` は元画像の短期バッファ、動的ROIのRGB正規化、TensorRT推論、外観ID照合を
専用ワーカーで実行します。`reid.launch.py` またはbringupの `enable_reid:=true` で有効化します。
学習済みReIDエンジンの指定が必須です。モデルと前処理を合わせて `config/reid.param.yaml` を設定してください。
出力は `/perception/opponents/reid/matches` の `jetpilot_msgs/ReidMatchArray` です。

推論は `EmbeddingBackend` に分離しており、将来NITROS/TensorRTNodeアダプタへ置換できます。
現時点の実装はTensorRT直接実行のみです。モデル契約・起動・寿命・検証範囲は
[ReID実行ガイド](../../../../docs/reid_runtime.md)を参照してください。
