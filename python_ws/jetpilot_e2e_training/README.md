# jetpilot_e2e_training

JetPilot向けの画像ベースE2E学習ツールです。次の2タスクを同じデータ・学習・
ONNX export・比較基盤で扱えます。

- `control`: steering `[-1, 1]` と throttle `[0, 1]`
- `trajectory`: 現在車体座標系の将来経路 `[N, 2]`（x: 前方、y: 左）

trajectoryの教師は記録odometryから生成します。画像時刻の姿勢を原点にして、
指定horizon内の将来位置を等間隔でサンプリングします。IMUは画像時刻以前だけを
使うcausal windowなので、学習時に未来情報は入りません。

## trajectoryデータの作成

データセット作成の時刻合わせは既定で`data.timestamp_source=bag`を使い、
Offline E2E Analysisと同じbag記録時刻で画像・制御・odometry・IMUを対応付けます。
イベントカメラのデバイス時計と制御のROS時計が異なるbagでも、header時計を混在させません。
UIでは`Alignment & IMU` → `Alignment clock`から選択できます。
全センサーのheader時計を同期済みで撮影時刻を優先する場合は`header`を指定してください。
bag時刻には配信・記録の遅延が含まれるため、撮影時刻の厳密な同期を保証するものではありません。

`No aligned samples`の場合、`metadata.yaml`とエラーに時刻ソース・画像数・
時刻範囲・除外理由の件数を出力します。時刻不一致以外に画像のデコード失敗や
教師の記録区間不足もあり得るため、これらを確認してください。

```bash
source /opt/env/bin/activate
cd python_ws/jetpilot_e2e_training

python -m e2e_learning.cli.preprocess_bag \
  data.task=trajectory \
  data.bag_path=/bags/run_001 \
  data.image_topic=/realsense/color/image_raw \
  data.odometry_topic=/visual_slam/tracking/odometry \
  data.imu_topic=/realsense/imu \
  data.trajectory_horizon_sec=1.5 \
  data.trajectory_points=10 \
  data.trajectory_scale_m=5.0 \
  data.imu_window_sec=0.5 \
  data.imu_samples=10 \
  data.output_dir=datasets/trajectory_run_001
```

作成される`metadata.yaml`にtrajectory点数、距離scale、IMU窓を保存します。
学習時はこの値を自動で読み込むため、モデルと教師ラベルのshapeがずれません。

## モデル構成

RGBで操舵だけを学習する場合は`experiment=pilotnet_steering`を使います。
スロットルは損失に含めず、モデルは互換出力`[steering, 0]`を返します。
Consoleにも`Steering only · PilotNet (fixed throttle)`として表示されます。
実車では`bringup.sh e2e-collect`で固定スロットルの教師データを収集し、
配備先`Camera Steering Only (fixed throttle)`へ転送した後、
`bringup.sh e2e-steering --set fixed_throttle:=0.2`で推論します。

| experiment | 画像時系列 | IMU | 出力 |
|---|---:|---:|---|
| `dinov3_vits16_scratch` | 1 frame (212×120) | なし | control |
| `dinov3_vits16_finetune` | 1 frame (212×120) | なし | control |
| `trajectory_pilotnet` | 1 frame | なし | trajectory |
| `trajectory_pilotnet_gru` | 4 frames + GRU | なし | trajectory |
| `trajectory_pilotnet_imu` | 1 frame | GRU encoder | trajectory |
| `trajectory_pilotnet_gru_imu` | 4 frames + GRU | GRU encoder | trajectory |
| `control_pilotnet_fusion` | 1 frame | なし | control |
| `control_pilotnet_gru` | 4 frames + GRU | なし | control |
| `control_pilotnet_imu` | 1 frame | GRU encoder | control |
| `control_pilotnet_gru_imu` | 4 frames + GRU | GRU encoder | control |

### DINOv3互換ViT-S/16

`dinov3_vits16`は、将来のRGB→event知識蒸留とGEP（Generative Event
Pretraining）checkpoint利用を想定した約22M parameterのbackboneです。公開DINOv3
ViT-S/16と同じpatch 16、384次元、12 block、6 head、MLP比4、4 storage token、
axial 2-D RoPE、LayerScale、key-bias maskingを使用します。蒸留側からCLS、storage、
patch tokenとblock 2/5/8/11の中間特徴を取得できます。運転用headはCLS tokenから
`[steering, throttle]`を生成します。

scratch学習:

```bash
python -m e2e_learning.cli.train \
  experiment=dinov3_vits16_scratch \
  data.dataset_dir=datasets/control_run_001 \
  run.name=dinov3_vits16_scratch_001
```

DINOv3 ViT-S/16またはGEPのDINOv3 event-encoder checkpointからfine-tuneする場合は、
公式RGB重みの既定配置先は
`weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth`です。別の
checkpoint pathは環境変数へ指定します。GEP checkpointの`event_encoder`形式と、
DINOv3のraw state dict形式を認識します。入力channel数が異なるGEP checkpointでは、
shapeが一致しないpatch projectionだけを読み飛ばし、残りのbackboneを利用します。

```bash
export DINOV3_VITS16_WEIGHTS=/path/to/dinov3_or_gep_event_encoder.pt
python -m e2e_learning.cli.train \
  experiment=dinov3_vits16_finetune \
  data.dataset_dir=datasets/control_run_001 \
  run.name=dinov3_vits16_finetune_001
```

GEPのevent encoderは`weights/gep/`へ配置し、`model.weights_path`でファイルを明示します。
論文とともに公開されたSmall/Base重みはDINOv2初期化版なので、このDINOv3互換modelには
読み込めません。DINOv3 teacherでGEP stage 1を学習した重み、または今後明示的に公開される
DINOv3版checkpointだけを配置してください。loaderもRoPEとstorage tokenを検査し、DINOv2
checkpointを拒否します。
配置規則は[`weights/README.md`](weights/README.md)にまとめています。

EventStateで学習済みのDINOv3 EVS encoderを使う場合は、complete checkpointを
`weights/eventstate/eventstate_dinov3_vits16.pth`へ配置し、
`experiment=dinov3_vits16_eventstate`を指定します。checkpoint内の
`model.event_encoder.backbone.*`だけを抽出し、運転用headはJetPilot側で新規学習します。
このpresetはEventState既定の3ch `gep_rgb`表現とDSEC event mean/stdを前提とします。
server側runの埋め込みconfigが異なる場合は、その`representation`、時間window、mean/stdを
必ず優先してください。

外部入力はPilotNetと同じ212×120です。patch 16で割り切れないため、正規化後にモデル内部で
左右6px、上下4pxをゼロpaddingして224×128とし、14×8 patchへ変換します。ゼロは正規化前の
ImageNet mean相当で、入力画像のアスペクト比を変更しません。TensorRT向けONNXは固定batch 1、
NCHW RGB float32で出力され、paddingとRoPE定数もgraphに含まれます。

設計参照: [DINOv3公式実装](https://github.com/facebookresearch/dinov3)、
[Generative Event Pretraining公式実装](https://github.com/uzh-rpg/generative_event_pretraining)。
外部実装のソースは同梱せず、checkpointとtoken境界だけを互換にしています。

### Controlと物体検出でbackboneを共有する

実機でViT backboneを1回だけ実行する共有モデルでは、Control headを次のpresetで学習します。
このpresetは全epochでbackboneを`requires_grad=False`かつ`eval()`に固定します。

```bash
export DINOV3_VITS16_WEIGHTS=/path/to/dinov3_or_eventstate_backbone.pth
python -m e2e_learning.cli.train \
  experiment=dinov3_vits16_frozen_head \
  data.dataset_dir=datasets/control_run_001 \
  run.name=vit-dinov3-vits16-control-frozen_0910-1800
```

物体検出headは`jetpilot_object_detection_training`側で別datasetを使って学習します。
最後に`e2e_learning.cli.export_multitask_onnx`で両checkpointを統合します。export時には
backbone tensorのfingerprintを比較するため、異なるDINOv3/GEP重みを使ったheadは統合されません。
統合modelは固定212x120入力、`control`と`detections`の2出力です。

例:

```bash
python -m e2e_learning.cli.train \
  experiment=trajectory_pilotnet_gru_imu \
  data.dataset_dir=datasets/trajectory_run_001 \
  run.name=trajectory_gru_imu_001

python -m e2e_learning.cli.export_onnx \
  checkpoint=outputs/e2e/trajectory_gru_imu_001/checkpoints/best.pt
```

trajectoryではvalidation lossに加えてADE、FDE、lateral MAEをm単位で保存します。
train/validationは時系列順に分割し、隣接フレームのランダム混在を避けています。

## 一括ablation

trajectoryの4構成を同じデータと既定条件で学習・ONNX export・集計します。

```bash
scripts/run_trajectory_ablations.sh \
  datasets/trajectory_run_001 trajectory_abl_001
```

3番目以降の引数は全runへ同じHydra overrideとして渡せます。例えば短い試験なら
`train.stages.0.epochs=3 train.seed=42`を末尾へ追加します。

controlに対するIMU/GRUの効果も比較できます。

```bash
scripts/run_control_sensor_ablations.sh \
  datasets/control_run_001 control_sensor_abl_001
```

結果は`outputs/e2e_ablations/<name>/summary.csv`と`summary.md`にまとまり、
backbone、temporal、IMU有無、ADE/FDEまたはcontrol誤差を横並びで確認できます。

## Console UIでの利用

E2E Pipeline画面でLearning taskを`Trajectory`にすると、odometry、IMU、点数、
horizon、scaleを指定してデータ作成できます。学習構成には上記experimentが表示されます。

E2E Analysisでtrajectory ONNXを選ぶと、動画時刻に同期したローカル鳥瞰plotへ
予測経路とodometry由来GTを重ねて表示します。ADE/FDEの時系列、集計、worst sample、
区間別trajectory誤差も確認できます。GRU/IMUを含む全構成のoffline ONNX評価に対応します。

## 出力物

- `run.yaml`: 解決済み設定
- `progress.json`: UI向け進捗
- `metrics.json`: 構成とvalidation指標
- `checkpoints/best.pt`: best checkpoint
- `model.onnx`: exportモデル
- `metadata.json`: 入出力shape、trajectory geometry、IMU/時系列構成

## Jetsonへの配備

```bash
scripts/deploy_model.sh outputs/e2e/trajectory_run/model.onnx \
  --preset camera_trajectory
```

現行のIsaac ROS TensorRT画像パイプラインへ直接接続できるのは、4D NCHWの
単一画像・IMUなしモデル（`trajectory_pilotnet`）です。複数画像GRUとIMU入力モデルは
offline比較には対応していますが、実車online推論には時系列/IMU tensor producerの追加が
必要です。
