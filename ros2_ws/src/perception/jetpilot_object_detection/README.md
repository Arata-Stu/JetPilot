# JetPilot YOLOv8 object detection

Jetson Orin Nano上でcuVSLAMと共存させるための、小さいYOLOv8 TensorRT runtimeです。
実機ではIsaac ROSの画像前処理とTensorRT、このpackageのC++ decoderだけを動かします。
学習時だけ必要なUltralyticsはROS workspaceへ含めません。depth画像は使用しません。
検出入力だけを既定15 Hzへ間引くframe gateを同じcomponent container内に置き、VSLAMが使う
camera streamのrateは変更しません。`max_inference_fps`は実機計測に応じて調整できます。

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
