# VGL map生成モデルの指定

`create_map.sh`とWeb UIのMap Builderは、共通の`create_map_with_vgl.py`を使用します。
従来の既定値は1920×1200。軽量モデルを使う場合は、モデルフォルダと入力幅・高さを両方指定します。
入力サイズの指定だけではONNXやエンジンは変換されません。

## 準備

mapを生成するGPUで、ALIKEDとLightGlueのTensorRTエンジンを事前生成します。
例えばx86_64では`tools/aliked_workspace`から`python3 lab.py build --name 424x240`を実行します。
Jetson用エンジンをx86_64で流用しないでください。
map生成後に既定モデルをエクスポートする従来の確認は廃止し、生成前にエンジンを検査します。

## CLI

Docker内で、更新したリポジトリを使用します。

```bash
source /workspaces/ros2_ws/install/setup.bash
OUTPUT_MODEL_DIR=/workspaces/tools/aliked_workspace/artifacts/424x240/runtime_models \
VGL_IMAGE_WIDTH=424 VGL_IMAGE_HEIGHT=240 \
bash /workspaces/scripts/create_map.sh
```

bagは既存の`20260907_011815_joy_start`を選択し、出力は旧mapとは別の名前を指定します。

## Web UI

バックエンドを再起動し、ブラウザを再読み込みします。Map Builderで次を指定します。

- VGLモデルのフォルダ：`/workspaces/tools/aliked_workspace/artifacts/424x240/runtime_models`
- VGL入力の幅：`424`
- VGL入力の高さ：`240`
- Map name：既存mapと異なる名前

モデル欄は候補の補完に対応しています。画面の読み込み時に保存先を探索し、パスの一部を入力すると候補を絞り込めます。「モデル候補を再探索」で新しく生成したモデルも取得できます。選択した実験フォルダの`manifest.json`に幅・高さがあれば自動反映します。フォルダ名からサイズを推測せず、記録がないものは「サイズ未確認」と表示します。記録値はエンジンの実測値ではないため、生成開始時の入力形状検査も継続します。

モデルの選択範囲はROS workspaceの`isaac_ros_assets/models`と、リポジトリの`tools/aliked_workspace/artifacts`です。
パスとサイズの検証はAPIと事前確認の両方で行います。GPUでのエンジンの読み込み・形状検査はタスク開始時に行い、失敗時は画像や姿勢の生成へ進みません。

## 生成されるもの

1. 公式`create_map_offline.py`で画像と姿勢を生成します（`cuvgl`工程を除く）。
2. 公式設定の実体を新map内へコピーし、ALIKEDの入力サイズだけ変更します。
3. `create_cuvgl_map.py`へモデル・設定・画像フォルダを明示し、ALIKED特徴量とBoWインデックスを生成します。既存特徴量・既存語彙は渡しません。

mapには以下も保存します。

- `vgl_profile.json`：モデルパス、ALIKEDエンジンSHA-256、入力サイズ、生成状態
- `vgl_mapping_config/`：map生成に使用した設定
- `vgl_runtime_config/`：同じ入力サイズに揃えた位置推定用設定

位置推定では、このmapと対応するモデルを指定し、`vgl_config_dir`に新mapの`vgl_runtime_config`を渡してください。設定やモデルの自動選択は行いません。
モデルパスとエンジンSHA-256は生成環境の記録です。Jetsonでは同じONNXからJetson用エンジンを生成して使用します。

Macでは標準ライブラリの制御テストとJavaScriptテストのみを実施しています。
軽量モデルでの実際のmap生成・位置推定精度は、CUDA実行環境で確認が必要です。

## 姿勢生成後に止まった場合の再開

生成先の`latest`等のリンクは実体パスへ解決し、同じmapを重複して数えません。
画像と姿勢が生成済みで、VGL生成前に停止した場合は、次のようにVGL工程だけを実行できます。

```bash
python3 /workspaces/scripts/create_map_with_vgl.py \
  --resume-map /workspaces/map/出力名/生成された日時付きディレクトリ \
  --model-dir /workspaces/tools/aliked_workspace/artifacts/424x240/runtime_models \
  --width 424 --height 240
```

`map_frames/rectified/frames_meta.json`が必要です。既存の`cuvgl_map`やVGL設定・生成記録がある場合は上書きせず停止します。この再開はVGL生成までで、HD map等の後処理は実行しません。
