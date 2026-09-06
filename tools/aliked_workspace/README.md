# ALIKED 再出力・比較ワークスペース

NVIDIA配布ONNXと重みが整合した`aliked-n16`を使い、元サイズの比較→424×240での再出力→JetsonでのTensorRT生成を試す。TopKは2048、opsetは17で固定。幅・高さは変更できる。

**現状は実験用ツールを用意した段階。MacでGPU処理・依存パッケージの実行テストはしていない。** データ保護・引数・生成物検査の制御は標準ライブラリのテストで確認する。公開ソースのONNX変換処理が実機のPyTorch/torchvisionと互換か、生成モデルがcuVGLで動くかは以下の手順で検証する。

## 配置

```text
aliked_workspace/
  lab.py                 実行入口
  README.md
  test_lab.py             標準ライブラリだけのテスト
  vendor/                コミット固定の公開コード・重み・ライセンス
  reference/             NVIDIA配布ONNXの検証済みコピー
  inputs/                比較用の実画像を置く場所
  artifacts/
    baseline/            1920×1200の再出力・比較結果
    424x240/             小さいONNX・TensorRT・検査結果
```

`vendor`、`reference`、`inputs`、`artifacts`はGit管理外。別マシンへGitで反映した場合は`prepare`で再取得する。ルートの`scripts/configure_vgl_extractor.py`をビルド時に使うため、このフォルダだけではなくJetPilotリポジトリとして配置する。

既存の`/opt/ros`や通常のVGLモデル、mapは読み取るだけで上書きしない。再試験時は新しい`--name`を使う。エクスポートまたはビルドに失敗したフォルダは調査用に残り、自動削除しない。

## 1. JetsonのROSコンテナで準備

JetPilotリポジトリのルートから：

```bash
cd tools/aliked_workspace
source /workspaces/ros2_ws/install/setup.bash

# ソース取得とNVIDIAモデルのハッシュ検査・コピー。インストールはしない。
python3 lab.py prepare

# 現在のPython環境を確認。CUDA対応torch/torchvisionが必要。
python3 lab.py doctor --stage export
```

ソース固定コミットは`96f1f9e7a9932a5da4f0d1aa62017c0fd48141ce`。NVIDIA参照モデルのSHA-256は`bd4bd09d2f2dda23fd0bbe98abf47c71fbf93e3a4b5d5e7a99a4930592120193`。ユーザーがJetson側でもこの値との一致を確認済み。

`prepare --source-only`ならROSがないCUDA作業機でも公開ソース・重みだけ取得できる。比較には別途同じNVIDIAモデルを渡して`prepare --reference /path/to/aliked.onnx`を実行する。

### 必要な環境

| 段階 | 依存関係 |
|---|---|
| prepare、ヘルプ、軽量テスト | Python標準ライブラリ |
| export | CUDA対応PyTorch・互換torchvision、NumPy、ONNX、Pillow |
| compare | NumPy、Pillow、ONNX Runtime（CPU版でも可） |
| build、inspect | Jetson用TensorRT Python API、Isaac ROSの生成ツール |

元ONNXの生成メタデータはPyTorch 2.5.0。公開コードはCUDAに直接依存するため、CPU専用PyTorchではエクスポートできない。`torchvision`のDeformConvと、取得した`deform_conv2d_onnx_exporter.py`を使用する。

まずIsaac ROSコンテナで利用可能なCUDA対応torch/torchvisionを使う。`pip install torch torchvision`で既存環境を置き換えない。足りないONNX/Pillow等を追加する場合は、その環境のNumPy等と互換な版を選び、必要なら`python3 -m venv --system-site-packages .venv`で隔離する。ここでは未検証の一括インストールを自動化していない。

## 2. 比較用画像を用意

実際の走行環境の画像を`inputs/frame.png`へ置く。ツールは画像をRGBに変換し、指定寸法へPillowのbilinearでリサイズ、0〜1のFP32にして両モデルへ同じ入力を渡す。これがcuVGL内部の前処理と完全に同じかは別途確認する。

画像はMacではなく、コマンドを実行するマシン内に必要。複数場面の画像を用意するのが望ましい。

## 3. 元サイズを再出力・比較

VGLなどのGPU処理を停止してから：

```bash
python3 lab.py export --name baseline \
  --width 1920 --height 1200 --image inputs/frame.png

python3 lab.py doctor --stage compare
python3 lab.py compare --name baseline --image inputs/frame.png
```

1920×1200のPyTorch処理・ONNX変換は、TensorRT実行時よりメモリを使う可能性がある。Orin Nanoでメモリ不足になる場合は、公開ソースと重みを同じコミットに揃えたCUDA作業機でONNXまで生成する。TensorRTエンジンは実行するJetson側で生成する。

比較は既定でONNX RuntimeのCPUプロバイダーを使い、2モデルを順に実行する。高解像度では時間とRAMが必要。対応するCUDA版ONNX Runtimeがある場合は`--provider CUDAExecutionProvider`を指定できる。

結果は`comparison-画像ハッシュ.json`。特徴点を行番号で直接比較せず、ピクセル座標の相互最近傍で対応付ける。既定の暫定合格基準：

- 0.5 pixel以内の相互対応率が99%以上
- 対応した記述子の平均cosine similarityが0.999以上
- 対応したスコアの平均絶対誤差が0.001以下
- 出力形状が`[2048,2]`、`[2048,128]`、`[2048]`で全値が有限

基準を満たさない場合は終了コード1。ONNXファイルは残る。これは単一画像の回帰確認であり、数学的同一性や位置推定精度の保証ではない。別画像でも`compare`を実行する。合格前に小さいモデルを本番へ切り替えない。

## 4. 424×240版を再出力

```bash
# 幅424、高さ240が既定値
python3 lab.py export --image inputs/frame.png

# 別サイズや再試験は別の名前で
python3 lab.py export --name test-640x480 \
  --width 640 --height 480 --image inputs/frame.png
```

出力ONNXの入力形状を確認してから`manifest.json`を保存する。途中で失敗した場合は、原因を確認して新しい名前で実行する。

## 5. JetsonでTensorRT生成・メモリ確認

```bash
python3 lab.py doctor --stage build
python3 lab.py build --name 424x240

# 生成済みエンジンを再確認
python3 lab.py inspect --name 424x240
```

`source_models/aliked_lightglue/aliked.onnx`には**新しく出力したONNX**を置き、これをIsaac ROSの生成ツールへ渡す。ALIKEDは既存設定をコピーして指定サイズにし、LightGlueの設定は変更しない。エンジンの入力が`[1,3,240,424]`であることを検査する。入力が1920×1200なら成功扱いにせず停止する。

`engine-inspection.json`の`context_memory_gib`を旧版の**2.841 GiB**と比較する。この値は実行コンテキスト用メモリであり、VGL全体やjtopのUsedとは異なる。

## 6. 既存mapでVGLを試す

この段階では小さいONNXのcuVGL互換性はまだ未確認。実画像で入力サイズエラーがないか、位置推定が成功するかを確認する。

最初に、実験フォルダで実行時の設定も424×240へ揃える。エンジン生成用の設定はVGLへ自動では渡らない。

```bash
source /workspaces/ros2_ws/install/setup.bash
VGL_SHARE="$(ros2 pkg prefix --share jetpilot_system_launch)"
mkdir -p artifacts/424x240/runtime_config
cp -RL "$VGL_SHARE/config/localization/vgl_config/." artifacts/424x240/runtime_config/
python3 /workspaces/scripts/configure_vgl_extractor.py \
  "$VGL_SHARE/config/localization/vgl_config/keypoint_creation_config.pb.txt" \
  artifacts/424x240/runtime_config/keypoint_creation_config.pb.txt \
  --width 424 --height 240
```

`--symlink-install`の設定はリンクの場合があるため、`cp -RL`で実体をコピーする。以前`cp -a`で作ったディレクトリには上書きせず、新しい名前で設定ディレクトリを作成し、起動時の`vgl_config_dir`もその名前に揃える。変換ツールが`source and destination must be different`を返した場合は、元設定とコピー先が同じ実体を指している。

ターミナル1でワークスペースをsourceしてコンテナを起動：

```bash
source /workspaces/ros2_ws/install/setup.bash
ros2 run rclcpp_components component_container_mt \
  --ros-args -r __node:=vgl_test_container
```

ターミナル2はこの実験フォルダをカレントディレクトリにして：

```bash
source /workspaces/ros2_ws/install/setup.bash
ros2 launch jetpilot_system_launch vgl.launch.py \
  container_name:=vgl_test_container \
  vgl_enabled_stereo_cameras:=realsense \
  vgl_map_dir:=/workspaces/map/E522-0907-v1/2026-09-07_01-28-58_20260907_011815_joy_start/cuvgl_map \
  vgl_model_dir:="$(pwd)/artifacts/424x240/runtime_models" \
  vgl_config_dir:="$(pwd)/artifacts/424x240/runtime_config"
```

これはVGLの初期化までの試験。自己位置推定の検証にはカメラ画像・CameraInfo・TFと推定トリガーを別途供給する。各比較の前にコンテナも停止・再起動して旧ノードの残留を避ける。

全体起動でも`vgl_model_dir`と`vgl_config_dir`の両方を`bringup.sh`へ渡す。更新したlaunchファイルをJetsonへ反映し、`jetpilot_system_launch`をビルドしてから使用する。既定の設定ではALIKEDの`opt`が1920×1200のため、小さいエンジンだけ指定すると入力形状不一致で停止する可能性がある。

## 軽量テスト

### PyTorch 2.11でのexportエラー

`KeyError: <JitScalarType.FLOAT: 6>`は、PyTorchの型APIの移動に公開版exporterが対応していないことが原因。`lab.py`は登録前に新しい場所の`JitScalarType`を補い、vendorソースと重みは変更しない。使用したAPIは`manifest.json`に記録する。

このエラー後は更新した`lab.py`で、`--name baseline-v2`など新しい名前を指定して再実行する。既存の`baseline`ディレクトリは上書きしない。`prepare`やパッケージの再インストールは不要。実際のONNX出力と数値比較はDockerのCUDA環境で確認する。

### 実行方法

```bash
python3 -m unittest discover -s tools/aliked_workspace -p test_lab.py
```

リポジトリのルートから実行。GPU・ROS・NumPy・PyTorchなしで制御部分のみを確認する。

重み・入出力の詳細は[照合レポート](../../docs/aliked_model_compatibility.md)を参照。
