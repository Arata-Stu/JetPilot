# C++ ReIDの実行と推論バックエンド

`jetpilot_object_detection::ReidNode` は、元画像とYOLO/ByteTrackの検出結果を入力するC++ composable nodeです。画像保持、ROI前処理、TensorRT実行、外観特徴量の照合、結果配信を実装しています。学習済みReIDモデルは同梱していません。モデルの選定・学習とRCカー画像による識別精度評価は別途必要です。

## 実装の境界

| ファイル | 責務 |
| --- | --- |
| `reid_core.hpp/.cpp` | ROS/CUDA非依存の画像バッファ、RGB前処理、外観ギャラリー |
| `reid_backend.hpp` | 推論要求・結果と `EmbeddingBackend` インターフェース |
| `reid_tensor_rt_backend.cpp` | TensorRTエンジン、CUDAバッファ、推論ストリーム |
| `reid_node.cpp` | ROS入出力、期限・負荷制限、専用ワーカー、結果の対応確認 |

初期実装はCPU上でROIを直接RGB NCHW float32へ変換し、小さな入力テンソルをGPUへ転送します。画像全体や切り出し画像の中間コピーは作りません。入力画像の購読は `ConstSharedPtr` で参照を保持します。同じコンテナに配置した場合も、上流の所有権・型変換などを含む経路全体のゼロコピーは保証しません。

推論はワーカー1本で行い、画像購読コールバック内では実行しません。GPU利用自体はYOLOなどと競合するため、実機で遅延を測定してください。CUDA呼び出しが故障等で停止した場合の強制キャンセル機能はありません。

## モデルの契約

- TensorRT 8.5以降のC++名前ベースAPIを使用します。対象Jetsonと互換性のあるエンジンをあらかじめ生成してください。
- 入力1個・出力1個、どちらもdevice上のlinear float32。内部のFP16計算は利用できます。
- 入力は `[1, 3, input_height, input_width]`。動的入力はprofile 0がこの形状を許容する場合に対応します。
- 出力は `[1, embedding_size]` または末尾に1が続く形状。データ依存の動的出力は非対応です。
- 出力は車両の外観embeddingである必要があります。YOLOの検出出力や分類logitsを流用しません。
- 元画像は `rgb8` または `bgr8`。画像サイズは検出器の `source_width/source_height` と一致させます。bboxは元画像座標、回転なしです。
- bboxを双線形補間で指定サイズへ直接伸縮し、RGB各チャンネルに `(pixel / 255 - mean) / stddev` を適用します。letterboxや別の前処理が必要なモデルはそのまま使用できません。
- デフォルトの128×128、256次元、ImageNet正規化は設定例です。採用モデルの学習・ONNX出力に必ず合わせて変更してください。
- TensorRT標準層で構成したモデルを対象とします。独自pluginの動的ロードは実装していません。

エンジンの読込失敗、I/O名や形状・型の不一致はノード起動時にエラーにします。モデル未設定時にダミー特徴量で稼働する経路はありません。

## Jetsonでの起動

既存のIsaac ROS開発環境内で、`NvInfer.h` と `libnvinfer` の開発ファイルが必要です。CMakeで検出します。TensorRTにはupstream rosdepのキーがないため、JetPack/Isaac ROS環境側の依存として扱います。

```bash
cd /workspaces/ros2_ws
colcon build --packages-up-to jetpilot_object_detection jetpilot_system_launch
source install/setup.bash
```

モデルに合わせた `reid.param.yaml` を用意し、既存の検出パイプラインが動くコンテナへ追加します。以下の `/workspaces/models/reid/...` はユーザーが用意するファイルの例です。

```bash
ros2 launch jetpilot_object_detection reid.launch.py \
  container_name:=multi_sensor_container \
  engine_path:=/workspaces/models/reid/model.plan \
  param_file:=/workspaces/models/reid/reid.param.yaml
```

bringup経由の有効化は、既存の運用コマンドに以下を追加します。

```text
--set enable_object_detection:=true
--set enable_reid:=true
--set reid_engine_path:=/workspaces/models/reid/model.plan
--set reid_param_file:=/workspaces/models/reid/reid.param.yaml
```

ReIDの画像入力はデフォルトでImageGateの出力 `/perception/object_detection/image` です。YOLOが実際に処理対象とする画像を共有します。追跡を有効にし、`Detection2D.id` が付いたvehicle検出が必要です。tracking無効ではReID対象がありません。

既存の検出結果をbagから再生する場合は `use_sim_time:=true` と再生の `/clock` を使用し、`image_topic:=/realsense/color/image_raw` を指定します。bringupでは `reid_image_topic` で変更できます。新規コンテナを作る場合だけ `run_standalone:=true` にします。別プロセスのbag再生では画像のプロセス間通信が生じます。

## 画像とジョブの寿命

`header.stamp` と `frame_id` の完全一致で対応付け、近い時刻の別画像では代用しません。画像先着・検出先着の双方に対応します。

- 空の検出結果、またはID付きvehicleがない結果では、該当画像の参照を即座に解放します。画像が後着しても短期間の完了キー記録で捨てます。
- 結果欠落時は最初の受信から `image_ttl_seconds`（初期0.5秒）の単調時計による期限で破棄します。bag停止中もメモリを保持し続けません。
- 未対応バッファは `buffer_frames`（16）と `buffer_megabytes`（32 MiB）で制限します。
- 対応済みの推論待ちは `pending_frames`（2）で制限し、溢れたら最も古い待機ジョブを破棄します。期限は対応成立時に延長しません。
- 待機ジョブと処理中ジョブはバッファとは別に画像参照を持つため、32 MiBはノード全体のメモリ上限ではありません。保持画像は最大でバッファ上限＋待機2枚＋処理中1枚です。DDSや上流ノードの保持分、テンソル、エンジンは別です。
- 対象ROIをすべてモデル入力へ変換したら、推論開始前に元画像の参照を解放します。
- 遅延した推論結果はギャラリー更新もpublishも行いません。ROS時計の巻き戻り時は世代を更新し、古い実行中ジョブも無効化します。
- 処理済みより古い画像結果は捨てます。光学フレーム変更時はギャラリーと対応状態をリセットします。

画像共有では参照がなくなった時に画素バッファが解放されます。GPU転送と推論はストリーム完了まで入力・出力を保持します。

## ID照合と出力

`/perception/opponents/reid/matches` に `jetpilot_msgs/msg/ReidMatchArray` を配信します。

- `header`: 元画像の取得時刻とフレーム。非同期出力のため受信順が画像順とは限りません。利用側は時刻で扱います。
- `session_id`: ノード起動ごとの識別子。
- `track_id`: 既存TrackerのIDをそのまま保持。
- `opponent_id`: セッション接頭辞付きの外観ID。未判定は空文字。
- `similarity`: cosine similarity。確率ではありません。比較先なし・処理省略時は-1です。
- `confirmed`: 初期設定では2回の整合した観測でtrue。学習済みモデルの正しさを保証するフラグではありません。
- `status`: `new_identity`, `tentative`, `matched`, `ambiguous`, `identity_conflict`, `gallery_full`, `rate_limited`, `frame_budget`, `invalid_roi`, `invalid_embedding`, `inference_error` 等。

embeddingをL2正規化し、最も近い外観と2番目の差も確認します。曖昧な場合はIDを確定せず、ギャラリーも更新しません。十分異なる外観だけ新しいIDにし、初期2観測で確定します。ギャラリーの上限は16件、最終照合から300秒で期限切れです。強く一致した場合にのみ特徴を移動平均で更新します。画像端で切れたbbox、小さなbboxは除外します。ブレ・遮蔽の専用判定モデルはありません。

同一フレームの複数trackへ同じIDを与えません。再推論を間引いた可視trackの既存IDも予約します。既知IDが予約済みでも、次点へ無理に割り当てません。

ReIDはtrackごとに初期0.2秒間隔、1フレーム最大3 ROIです。処理省略時の空IDは「別車両と判定した」という意味ではありません。利用側は `status` と観測時刻を見て直近の確定対応を扱ってください。空の配列は対象なしを表し、未観測時間の走行経路を生成しません。

結果と診断topicをbagの既定記録対象に追加しています。Foxgloveの既存 `/perception/opponents/.*` 許可範囲に含まれ、Raw Messagesで確認できます。既存のmap上の軌跡は引き続きtrack ID基準です。ReIDによる軌跡の結合、永続保存、plannerへの適用はこの実装では行いません。

## 将来Isaac ROS TensorRTNodeへ切り替える場合

`EmbeddingBackend::infer(InferenceRequest)` を実装するアダプタを追加し、ノードのバックエンド生成箇所を差し替えます。バッファ、ROI選択、ギャラリー、外部の `ReidMatchArray` は維持できます。

現在の境界はCPU所有のRGB NCHWテンソルです。アダプタがGPUへ転送し、Managed NITROSで `TensorRTNode` へ渡せます。前処理までGPUへ移す場合は、前処理の境界にもGPUバッファ所有権を追加します。インターフェースだけでGPUゼロコピーが完成するわけではありません。

アダプタには以下を要求します。

1. `request_id`・元画像のキー・track IDと出力の対応を保持する。
2. 同一画像に複数ROIがあるため、元の画像時刻だけで各推論結果を照合しない。TensorRTNodeのtransport headerに一意キーを割り当て、元情報への対応表を保持する等の設計が必要。
3. 結果受信を独立したcallback group/executorで進め、ワーカー側の待機には期限を設ける。失敗時は例外で返し、遅着結果・重複結果を破棄する。
4. 戻り値は `InferenceResult` に復元する。ノードがrequest ID、フレーム、track ID、次元を再検証する。
5. CUDAイベントまたは適切なNITROS所有権でGPUバッファの寿命を保証する。

NITROSアダプタそのものは未実装です。現在選択可能な `backend` は `tensorrt` のみです。

参考: [TensorRT C++ API](https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/c-api-docs.html)、[Managed NITROSによるCUDA連携](https://nvidia-isaac-ros.github.io/v/release-3.1/concepts/nitros/cuda_with_nitros.html)。

## 検証範囲

Macでは外部Python依存を追加せず、標準C++のテストで画像保持・解放、逆順到着、期限と容量、RGB/BGR/stride、双線形補間、正規化、ID復帰・競合・曖昧性・上限・期限、推論結果の対応不一致を検証します。

ROS 2 componentのロード、TensorRT/CUDAのビルド・実推論、RCカー画像での精度・遅延はJetsonでの確認が必要です。標準C++テストは推論モデルの識別精度を検証するものではありません。
