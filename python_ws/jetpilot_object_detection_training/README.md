# JetPilot YOLOv8 object-detection training

JetPilotの物体検出dataset検査、学習・再学習、ONNX export、Jetsonへの配備を扱う
Python workspaceです。ROS 2やIsaac ROSには依存しません。実車runtimeのTensorRT推論と
C++ decoderは`ros2_ws/src/perception/jetpilot_object_detection`が担当します。

## セットアップ

学習用x86_64 containerでインストールします。Jetson上ではUltralyticsを使用しません。

```bash
source /opt/env/bin/activate
cd python_ws/jetpilot_object_detection_training
pip install -e .
```

## NotebookのConsole UI

学習用NotebookでJetPilot Consoleを起動し、**Object Detection**タブを開くと、次の操作を
ブラウザから行えます。

1. `datasets/**/data.yaml`からdatasetを選び、全labelを検査する
2. 新規学習、以前の`best.pt`を使うfine-tuning、`last.pt`からのresumeを選ぶ
3. runのloss/mAP、checkpoint、ONNX export状態を確認し、必要なら再exportする
4. SSHでJetsonへ転送し、Jetson側の`trtexec --fp16`で`model.plan`を生成する

```bash
tools/app/scripts/start.sh --host 127.0.0.1 --port 8765
```

Roboflowから取得したYOLOv8 datasetは、既定では
`python_ws/jetpilot_object_detection_training/datasets/<dataset-name>/`へ展開します。
学習とexportはNotebook側で動きます。UIのTensorRT生成はJetson側でのみ実行されます。

## dataset契約

- 形式: RoboflowからexportしたYOLOv8 detection dataset
- 初期クラス順: `0: vehicle`, `1: barrier`
- 学習・推論入力: RGB、`1x3x224x224`
- resize: letterbox
- ONNX出力: NMS前`[1, 4 + class_count, candidate_count]`
- input binding: `images`
- output binding: `output0`

dataset例は
`src/object_detection_learning/conf/dataset.example.yaml`にあります。クラス順はROS側decoderとの
interfaceなので、単独で変更しないでください。

### アノテーション規約

元画像（現在は424x240）のままRoboflowへ投入し、bounding boxでアノテーションします。
224x224へのresizeは学習・推論側に任せます。

| ID | ラベル | 含めるもの | 含めないもの |
|---:|---|---|---|
| 0 | `vehicle` | 走行判断の対象となる相手車両・競技車両 | 写真、反射、コース外で無関係な車両 |
| 1 | `barrier` | 衝突回避が必要な箱、バリケード、壁状の障害物 | 路面模様、影、標識、通過可能な小物 |

`static_object`は見た目と運転上の意味が広すぎるため使用しません。コーンだけ別の運転挙動が
必要なら`cone`、人の進入があり得るなら安全停止用`person`を、datasetとROS decoderの契約を
同時更新したうえで追加します。信号機はこの検出器へ混ぜず、切り出し画像に対する軽量な
`left / right / none`分類器として分離します。

- 見えている領域に沿う、余白の少ないboxにする。隠れた全体を推測しない。
- 画面端で切れていても、種類が識別でき走行判断に影響する物体は付ける。
- 遮蔽されていても種類が識別でき走行を妨げる物体は付ける。識別不能なら付けない。
- 元画像で幅または高さが6 px未満の識別不能な物体は付けない。
- 影、鏡面反射、ポスター上の車両には付けない。
- 物体がないnegative frameもdatasetへ残す。
- train/validation/testは連続frameをrandom分割せず、rosbagまたは走行session単位で分ける。
- Roboflowのdataset versionとクラス順を学習runに記録する。

学習前にpath、クラス順、YOLO label値を検査できます。

```bash
python -m object_detection_learning.cli.validate_dataset \
  --data /workspaces/datasets/jetpilot_objects_v1/data.yaml
```

## 学習・再学習

新規学習:

```bash
python -m object_detection_learning.cli.train \
  --data /workspaces/datasets/jetpilot_objects_v1/data.yaml \
  --model yolov8n.pt \
  --name yolov8n_224_v1
```

既存モデルを新しいdatasetでfine-tuningする場合は、`--model`に以前の`best.pt`を渡します。

```bash
python -m object_detection_learning.cli.train \
  --data /workspaces/datasets/jetpilot_objects_v2/data.yaml \
  --model outputs/yolov8/yolov8n_224_v1/weights/best.pt \
  --name yolov8n_224_v2
```

中断したUltralytics runをoptimizer状態ごと継続する場合は`--resume`を使用します。

```bash
python -m object_detection_learning.cli.train \
  --data /workspaces/datasets/jetpilot_objects_v1/data.yaml \
  --model outputs/yolov8/yolov8n_224_v1/weights/last.pt \
  --resume
```

学習完了後、既定では`<run>/export/model.onnx`と`metadata.json`も生成します。exportだけを
やり直す場合は次を実行します。

```bash
python -m object_detection_learning.cli.export_onnx \
  --weights outputs/yolov8/yolov8n_224_v1/weights/best.pt \
  --data /workspaces/datasets/jetpilot_objects_v1/data.yaml \
  --output-dir outputs/yolov8/yolov8n_224_v1/export
```

`metadata.json`にはクラス順、binding名、入力shape、RGB/NCHW、letterbox、NMSなし、
checkpoint/ONNX SHA-256を保存します。

## Jetsonへの配備とTensorRT engine生成

TensorRT `.plan`はGPU、JetPack、CUDA、TensorRT、Isaac ROS containerに依存します。
x86学習機で生成せず、対象JetsonのIsaac ROS container内で生成してください。

NotebookからJetsonへ直接配備する場合は、SSH鍵認証を準備して次を実行します。

```bash
scripts/deploy_model.sh \
  outputs/yolov8/yolov8n_224_v1/export/model.onnx \
  --user tamiya \
  --host 10.42.0.1 \
  --remote-root /home/tamiya/workspaces/JetPilot/ros2_ws/models/yolov8 \
  --name yolov8n_224_v1 \
  --yes \
  --build-engine
```

remote配備ではONNXとmetadataを一時directoryへ転送し、その一時directory内でTensorRT
engineの生成が成功してから既存モデルと置換します。engine生成に失敗した場合、以前の
`model.onnx`と`model.plan`は維持されます。Console taskはpasswordを入力できないため、
SSH鍵またはssh-agentを使用してください。

Jetson上でscriptを直接実行し、`--host`を省略した場合は、既定の
`/workspaces/ros2_ws/models/yolov8/<name>`へlocal配備します。

ROS runtimeでは次のmodel directoryを指定します。

```bash
ros2 launch jetpilot_object_detection yolov8_tensor_rt.launch.py \
  model_root:=/workspaces/ros2_ws/models/yolov8/yolov8n_224_v1
```

## 成果物

- `weights/best.pt`: validationで最良のcheckpoint
- `weights/last.pt`: resume用checkpoint
- `export/model.onnx`: ROS/TensorRTへ渡す固定224入力、NMSなしモデル
- `export/metadata.json`: 学習側とROS decoderのinterface/provenance
- `jetpilot_training_manifest.json`: dataset、初期weights、run、exportの対応
- Jetson側`model.plan`: 対象runtimeで生成したTensorRT engine
