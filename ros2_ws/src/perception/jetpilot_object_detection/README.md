# JetPilot YOLOv8 object detection

Jetson Orin Nano上でcuVSLAMと共存させるための、小さいYOLOv8 TensorRTパイプラインです。
Ultralyticsは学習・ONNX exportにだけ使用し、実機ではIsaac ROSの画像前処理とTensorRT、
このpackageのC++ decoderだけを動かします。depth画像は使用しません。
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

Ultralyticsでのexport例:

```bash
yolo export model=runs/detect/train/weights/best.pt format=onnx imgsz=224 \
  batch=1 dynamic=False simplify=True opset=17
```

Roboflow export後の学習からexportまでは、x86_64 training containerで次を実行できます。

```bash
source /opt/env/bin/activate
python3 ros2_ws/src/perception/jetpilot_object_detection/scripts/train_yolov8.py \
  --data /workspaces/datasets/jetpilot_objects_v1/data.yaml
```

スクリプトはdatasetのクラス順が`vehicle, barrier`であることを検査し、224固定・batch 1・
NMSなしのONNXを生成します。Jetson上ではこの学習スクリプトやUltralyticsを使用しません。

TensorRT engineはJetson上のTensorRT版・GPUに合わせて生成します。PCで生成した`.plan`の
持ち込みは避けてください。起動例:

```bash
ros2 launch jetpilot_object_detection yolov8_tensor_rt.launch.py \
  model_root:=/workspaces/ros2_ws/models/yolov8/latest
```

クラスを変更したときは、学習datasetの`data.yaml`と
`config/yolov8.param.yaml`の`class_names`を同じ順番にします。

## アノテーション規約

画像はcamera本来の解像度（現在は424x240）のままRoboflowへ投入し、bounding boxで
アノテーションします。224x224へのresizeは学習・推論側に任せます。

初期datasetは次の2クラスを推奨します。

| ID | ラベル | 含めるもの | 含めないもの |
|---:|---|---|---|
| 0 | `vehicle` | 走行判断の対象となる相手車両・競技車両 | 写真、反射、コース外で無関係な車両 |
| 1 | `barrier` | 衝突回避が必要な箱、バリケード、壁状の障害物 | 路面模様、影、標識、通過可能な小物 |

`static_object`という総称は、見た目と運転上の意味が広すぎるため使いません。コーンだけ
別の運転挙動が必要なら`cone`を追加します。人がコースへ入る可能性があるなら、安全停止用の
`person`を追加します。クラス追加はモデルの出力channel数も変えるので、decoder設定の更新と
TensorRT engineの再生成が必要です。

作業者間で次を統一してください。

- 物体の「見えている領域」に沿う、余白の少ないboxにする（隠れた全体を推測しない）。
- 画面端で切れていても、種類が識別でき運転判断に影響する物体は付ける。
- 遮蔽されていても種類が識別でき、走行を妨げる物体は付ける。識別不能なら付けない。
- 元画像で幅または高さが6 px未満の識別不能な物体は付けない。この閾値は実走評価後に更新する。
- 影、鏡面反射、ポスター上の車両には付けない。
- 物体がないnegative frameもdatasetへ残す。
- train/validation/testは連続frameをrandom分割せず、rosbagまたは走行session単位で分ける。
- RoboflowからはYOLOv8形式でexportし、dataset versionとクラス順を記録する。

信号機は本検出器へ混ぜません。小領域を切り出す軽量な信号機専用分類器で、
`left / right / none`（必要なら停止状態も追加）を扱う方が、クラス不均衡と小物体検出の負担を
分離できます。

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
