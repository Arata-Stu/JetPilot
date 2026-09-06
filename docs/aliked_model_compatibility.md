# NVIDIA配布ALIKEDと公開モデルの照合

調査日：2026-09-07

## 結論と適用範囲

NVIDIAの公開pyCuSFMから取得した`aliked.onnx`の学習済みパラメータは、公開`aliked-n16.pth`と整合する。`aliked-n16rot`ではない。ONNX内のTopKは2048で、入力はFP32 `[1, 3, 1200, 1920]`。424×240用の再出力は、`aliked-n16`とTopK 2048を基準に行う。

ユーザーがJetson側のSHA-256を取得し、下記公開ファイルと完全一致することを確認済み。したがって、この静的照合はJetson配布モデルにも適用できる。今回行ったのは静的な重み・グラフ照合であり、再出力・TensorRTビルド・数値推論・cuVGL統合の成功を示すものではない。再出力の実験手順は[専用ワークスペース](../tools/aliked_workspace/README.md)に用意した。

## 対象ファイル

- NVIDIA：[モデル説明](https://github.com/nvidia-isaac/pyCuSFM/blob/0c97a6700ee1ee6e7913ec68acc774a3ec47ccbd/pycusfm/models/aliked_lightglue/README.md)、[ONNX](https://github.com/nvidia-isaac/pyCuSFM/blob/0c97a6700ee1ee6e7913ec68acc774a3ec47ccbd/pycusfm/models/aliked_lightglue/aliked.onnx)
- 公開モデル：[aliked-n16.pth](https://github.com/ajuric/aliked-tensorrt/blob/96f1f9e7a9932a5da4f0d1aa62017c0fd48141ce/models/aliked-n16.pth)、[比較用n16rot](https://github.com/ajuric/aliked-tensorrt/blob/96f1f9e7a9932a5da4f0d1aa62017c0fd48141ce/models/aliked-n16rot.pth)
- 公開ソース：[ALIKED](https://github.com/ajuric/aliked-tensorrt/blob/96f1f9e7a9932a5da4f0d1aa62017c0fd48141ce/nets/aliked.py)、[ONNX変換](https://github.com/ajuric/aliked-tensorrt/blob/96f1f9e7a9932a5da4f0d1aa62017c0fd48141ce/convert_pytorch_to_onnx.py)

NVIDIAのモデル説明には、ALIKED-TensorRTからBSD-3-Clauseで作られたモデルと明記されている。

取得したONNXは11,452,422 bytes、SHA-256は次の値。

```text
bd4bd09d2f2dda23fd0bbe98abf47c71fbf93e3a4b5d5e7a99a4930592120193
```

Jetson上で確認するコマンド：

```bash
sha256sum /opt/ros/jazzy/share/isaac_ros_visual_mapping/models/aliked_lightglue/aliked.onnx
```

異なる場合はJetsonのファイルで再照合する。モデル入力が同じことだけでは同一性を証明できない。

## 重みの照合結果

Python標準ライブラリのみでONNXのprotobuf構造とPyTorchチェックポイントを読み取った。チェックポイントは任意クラスを読み込まない制限付きデコーダで扱い、PyTorch・ONNX・TensorRTは実行していない。

| 照合対象 | aliked-n16 | aliked-n16rot |
|---|---|---|
| 名前が保持された48テンソル | 形状・型・重みバイト列がすべて完全一致 | 48個すべて不一致 |
| Conv＋BatchNorm統合後の4組、計8テンソル | 計算値との差は最大約5.44e-7 | 明確に不一致 |
| ONNXの全initializer | 56個を上記で照合 | 対応元ではない |

統合された層は`block1.conv1`、`block1.conv2`、`block2.conv1`、`block2.conv2`。公開チェックポイントのConv・BatchNormパラメータから、epsilon=1e-5で統合後の重みとバイアスを算出した。比較はPythonの倍精度計算とONNXのFP32値の間なので、小さい丸め差を含む。公開チェックポイントの76テンソルには、推論時に不要な8個の`num_batches_tracked`も含まれる。

詳細なテンソル名と誤差は[照合結果JSON](aliked_model_audit.json)に記録した。全パラメータの対応は確認できたが、演算グラフ全体が公開コードと完全同一という証明ではない。

## 入出力・後処理

| 項目 | NVIDIA公開ONNXで確認した仕様 |
|---|---|
| 生成元メタデータ | PyTorch 2.5.0、ONNX opset 17 |
| 入力 | `image`、FP32、NCHW `[1,3,1200,1920]` |
| 特徴点 | `keypoints`、FP32、`[N,2]`、x,yの正規化座標 |
| 記述子 | `descriptors`、FP32、出力形状メタデータは記号的。グラフの集約処理から128次元、各点をL2正規化 |
| スコア | `scores`、FP32、`[N]` |
| TopK | グラフ中の定数が2048。出力メタデータ上のNは記号的 |
| 座標変換 | `2 * subpixel_xy / [1919,1199] - 1` |
| 記述子L2の下限 | 1e-12 |
| 入力余白 | Padのmode=`edge` |

公開コードの前処理はBGR→RGB、ToTensor（通常のuint8画像なら0〜1）、NCHW。公開コード側の特徴点は同じ正規化座標で、記述子はL2正規化される。NVIDIA実行ライブラリ側の画像前処理は、この静的ONNX照合だけでは確認していない。

単一画像のONNX出力にバッチ次元を追加しないこと。公開モデルのPython出力は画像ごとのリストだが、変換後の出力は`[N,2]`、`[N,128]`、`[N]`にする。公開変換スクリプトの既定モデルは`aliked-n16rot`なので、必ず`aliked-n16`を明示する。

## 424×240への変更方針

入力形状だけの書き換えでは不十分。配布ONNXの座標正規化に`[1919,1199]`が定数として入っているため、内部定数も更新されるソースからの再出力を優先する。公開コードは画像を32の倍数にパディングするので、424×240入力では内部448×256になる。パディング・切り戻しも新しい形状で生成する必要がある。

再出力の基準：

- 公開コード・重みのコミットを上記に固定。
- モデル`aliked-n16`、TopK 2048、評価モード、opset 17。
- まず1920×1200で再出力してNVIDIA版と同じ実画像の推論出力を比較する。これでサイズ変更以外の実装差を先に検出する。
- その後424×240のサンプル入力で固定形状ONNXを生成。座標正規化の除数が`[423,239]`になることを確認する。
- 特徴点数・記述子次元・出力名を維持。LightGlueを同時に変更しない。
- 最初の比較ではONNXをFP32で生成し、TensorRTのFP16化は後段で行う。
- Jetsonでビルド後、入力形状と実行用メモリ量を取得する。推論成功と既存mapへの位置推定を別途検証する。

再学習は必要ない。必要なのは学習済み重みを使った再エクスポート。公開変換コードに使われるDeformConvのONNX変換処理も含めて合わせる必要があり、重みだけを別実装に読み込ませて互換とみなさない。

同じn16の記述子を保つことで既存mapを再利用する根拠は強まったが、解像度変更による検出点・照合率の変化やcuVGL側の画像変換は未検証。mapの再生成は現時点では行わず、位置推定の実測結果で判断する。
