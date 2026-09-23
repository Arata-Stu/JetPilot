# Section Multihead E2E

Console の **E2E → Section Multihead** から、section別dataset作成、全headの学習、
head別rosbag解析、ONNX出力、Jetson転送を実行します。

## モデル

- DINOv3 ViT-S/16、RGB 212×120、ImageNet正規化。backboneは全学習期間でfreeze・eval固定。
- 既定headは `straight` / `curve` / `generic`。画面で追加・削除できます。
- 各headは操舵だけを学習します。throttleは学習対象ではなく、headに割り当てた設定値です。
- 汎用headは必ず1つ、専用headは1つ以上。他のheadが担当しないsection、定位不良、section更新途絶では汎用headを使います。
- TensorRTの契約は `image [1,3,120,212] -> control [1,H]`。
  出力列の順番は `metadata.json` の `output.head_order`。backboneを一度計算し、全headを実行します。
- decoderがheadとthrottleを選びます。エンジンや重みの実行中読み替えはありません。

## データ収集と区間ラベル

固定throttleごとのbagを従来の収集経路で作成します。VSLAM・section localizerを同時に動かせば、
新しい `/localization/section_state` も標準のbag記録対象に含まれます。
この `jetpilot_msgs/SectionState` は、判定に使ったTFの時刻、section ID、地図内容のSHA-256、有効性を持ちます。
既存の `/localization/current_section`（String）も維持します。

学習前処理では map と抽出sectionを複数選択し、最大32個のbagを1つのdatasetへまとめます。
地図一覧は `MAP_ROOT` 内の `*_hd_map.yaml` / `hd_map.yaml` を読み直して取得します。
section名をコードに固定していません。

「このdatasetに対応する実行時throttle」はモデルに引き継ぐ設定値です。
「抽出するthrottle」を空欄にすると、選んだsection内の加減速途中も含めます。
値を指定すると、教師指令のthrottleとの差が既定0.015以内のフレームだけを採用します。
固定throttleだけの周回から、throttle切り替えの過渡状態が自動的に生成されるわけではありません。

### 記録済みのsection・位置・TF

- 自動モードは `/localization/section_state` を優先し、なければ指定section topicを使います。
- section topicがない場合、指定した位置topicのmap座標、またはTFチェーンからsectionを求めます。
- 「位置・TF」モードは、地図のsection境界を編集した後の再割り当てにも使えます。
- `odom` 座標を `map` 座標として扱いません。位置topicがodom座標の場合はTFを使用します。
- 画像より後のラベルを使わず、既定0.3秒を超えて古いラベルは除外します。
- 定位状態topicがあるbagでは、`localized` 以外や更新途絶も除外します。
  状態topicのない旧bagでは、記録されたsectionまたは地図座標の正しさは記録側の前提になります。
- stamped sectionの地図hashが違う場合は停止します。位置・TF方式なら新しい境界で再計算できます。
- 旧String topicには撮影時刻がないためbag記録時刻で対応付けます。
  画像headerとbag時刻が異なる時計なら「bag記録時刻」を選択してください。

### オフラインVSLAM

Consoleが動くホストに、ビルド済みROS/Isaac ROS環境と保存済み `cuvslam_map/*.mdb` が必要です。
学習Pythonとは別に、ROS環境の `/usr/bin/python3` で再生workerを起動します。

1. 定位用mapと抽出用HD mapの内容が一致することを確認します。
2. 解析用ROS domain内でVSLAMとlocalization managerを起動します。
3. 元bagの画像・CameraInfo・IMUを再生します。記録済みTF、操舵指令、定位結果は再生しません。
   lane networkの `/planning/current_lane` が記録されている場合はラベルの選択に利用します。
4. 保存地図への `localized` を確認した後、画像header時刻のTFからsectionを付けます。
5. 定位前・TF取得不能・laneから離れたフレームを除外します。

既定は地図原点付近からの再定位です。別の開始位置ではVGLモードと対応する `cuvgl_map` を使用します。
カメラtopic設定は既定のVSLAM設定を使い、必要なら画面で設定ファイルを指定できます。
再生は既定0.5倍速。終了・失敗・キャンセル時には起動したプロセス群を終了します。

画像headerはVSLAMと同じROS時計である必要があります。オフライン方式ではbag時刻への切り替えはできません。
lane networkの地図は、オンライン判定と同様、現在laneが不明ならsectionも不明とします。

### 出力

通常の `samples.csv` と `images/` に加え `section_dataset.json` を出力します。
CSVには `section_id` / `source_bag` / bagごとの一意な `sequence_id` を保存します。
メタデータには地図のスナップショット・hash、選択section、bag別採用件数、section別件数、除外件数を保存します。
失敗・中断した出力ディレクトリは残します。再実行時は別名を指定してください。

## 学習

headごとにdatasetを1つ割り当て、一度の操作で全headを学習します。
専用headのdataset同士でsectionが重複すると拒否します。汎用headには全sectionを選択したdatasetが必要です。
全headのdatasetで地図hashが一致しなければ学習しません。

各headのDataLoaderをバッチごとに巡回し、それぞれのoptimizerでそのheadだけを更新します。
小さいdatasetを他のheadに合わせて繰り返すことはありません。各headは指定epoch数だけ学習します。
汎用headと専用headのデータが重なっていても、backboneは固定でheadのパラメータも独立しています。

学習・検証は各headのdataset内でbag単位に分割します。bagが1つの場合は時系列末尾20%を検証に使い、
学習側との間に1秒の空白を設けます。十分なデータがなければ拒否します。
headごとの最良検証MAEの重みを組み合わせて、`checkpoints/best.pt` を保存します。

学習完了後はONNX checkerとPyTorch/ONNX Runtimeの出力比較を実行し、成功したONNXだけを配備可能にします。
ONNX出力で失敗した場合もcheckpointは残り、画面の「ONNXを再出力」で再試行できます。

`metadata.json` には以下を保存します。

- 地図パス、地図内容、地図hash
- 各head名、dataset、対応section、固定throttle、汎用head名
- `section_policy`: section ID → head名・throttle
- 入出力shape、正規化、出力head順、学習設定、検証指標と分割情報

## オフライン解析

学習済みモデル、head、複数bagを選んで解析します。
選択headをbagの全フレームに適用し、教師操舵との差のMAE・RMSE・最大誤差を集計します。
sectionで自動切り替えする走行の評価ではありません。特定headの適性を比較するための解析です。
画面には教師・推論の操舵グラフを表示し、全件を `predictions.csv` に保存します。
throttleは推論しないため、操舵誤差の評価だけを行います。

## Jetson配備と実行

画面でJetsonホスト・ユーザーを指定して転送します。既定でJetson上の `trtexec` によりFP16 engineを作成します。
地図そのものの転送は従来の地図転送機能を使用してください。モデルにはHD mapの内容が埋め込まれますが、
cuVSLAM/VGLの地図データベースは含まれません。

新しいmessageとnodeがあるため、JetsonのROS環境で関連パッケージを再ビルドします。

```bash
colcon build --packages-up-to jetpilot_e2e_inference jetpilot_system_launch jetpilot_bag_tools
source install/setup.bash
```

起動例（車種・カメラ・車両接続の設定は既存の実機設定に合わせます）:

```bash
ros2 launch jetpilot_system_launch bringup.launch.py \
  enable_sensor_kit:=true \
  enable_e2e_inference:=true \
  e2e_section_multihead_mode:=true \
  e2e_model_root:=/workspaces/ros2_ws/models/e2e/section_multihead/my_model \
  enable_control:=false \
  enable_localization:=true \
  enable_localization_manager:=true \
  map_dir:=/workspaces/maps/my_map \
  vslam_localize_on_startup:=true \
  enable_vgl:=false
```

Multihead launchが学習時のHD mapからsection localizerを起動するため、
`enable_section_localizer`を別途trueにしないでください。
定位用mapディレクトリの `<directory-name>_hd_map.yaml` または `hd_map.yaml` が学習時と一致するか確認します。
通常の `scripts/e2e_trt.sh` / 単一head向けモデル選択ではMultiheadを拒否します。

実行中は `/e2e/active_head` と `/e2e/validated_section` を記録できます。
定位不良、sectionの更新途絶、古いTF、位置の不自然なジャンプでは汎用headへ切り替えます。
定位復帰は既定1秒の安定期間、section境界は既定0.15秒の安定期間を設けています。
加速方向のthrottle変化と操舵変化を制限し、throttleを下げる指令は直ちに適用します。
画像・推論の停止や古い推論結果は汎用headで補えないため、既存の指令watchdogによる停止対象です。

## 検証

この変更のMac上検証は、Python標準ライブラリの区間判定・時刻対応・ジョブ構築・退避判定テスト、
JavaScriptのUI操作テスト、構文検査です。ROS/CUDA/TensorRTのビルド・実機推論・実データ学習は別途必要です。

```bash
python3 -S -m unittest discover -s tests/section_multihead -v
node --test tools/app/frontend/tests/section_multihead.test.cjs
```
