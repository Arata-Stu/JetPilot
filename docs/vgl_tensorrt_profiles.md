# VGL用TensorRTエンジンの入力サイズ指定

> 2026-09-07の実機調査で、配布ALIKED ONNXは1920×1200の固定入力と判明した。以下のスクリプトによる生成設定変更だけでは424×240エンジンにならず、新旧とも実行用メモリが2.841 GiBだった。サイズ変更にはONNXの再出力が必要。[重み・入出力の照合結果](aliked_model_compatibility.md)を参照。以下のサイズ別フォルダ名は、生成物の入力サイズが変わったことを保証しない。

JetsonのROS環境内で実行する。Pythonは標準ライブラリのみ使用し、生成にはインストール済みの`isaac_ros_visual_mapping`のツールが必要。

```bash
# 既定値：幅424、高さ240
bash scripts/export_vgl_tensorrt_engines.sh

# サイズを明示する場合
bash scripts/export_vgl_tensorrt_engines.sh --width 424 --height 240

# 別サイズ・別出力先
bash scripts/export_vgl_tensorrt_engines.sh \
  --width 640 --height 480 \
  --output-model-dir /workspaces/ros2_ws/isaac_ros_assets/models/vgl_640x480_test
```

既定の出力先は`${ROS2_WS}/isaac_ros_assets/models/visual_global_localization_424x240`。`ROS2_WS`の既定値は`/workspaces/ros2_ws`。`OUTPUT_MODEL_DIR`環境変数も引き続き利用でき、`--output-model-dir`が優先される。`--yes`で確認を省略できる。

インストール済みの生成設定をコピーし、ALIKEDのNCHW入力のmin/opt/maxをすべて`[1, 3, 高さ, 幅]`にする。コピーは出力先の`keypoint_creation_config.pb.txt`に保存する。元の設定や既存の通常モデルは変更しない。LightGlueも同じ出力先に生成するが、その設定は変更しない。サイズ指定モードでは既存エンジンがある出力先を拒否するため、再試験には新しい出力先を指定する。

**これはカメラ画像をリサイズする機能ではない。** ALIKEDに渡る実際の画像サイズ（VGL内部のリサイズ・補正後を含む）が指定サイズと一致する必要がある。エンジン生成の可否、実入力との整合、推定精度、メモリ削減量はJetson実機で確認する。画像なしのVGL初期化成功だけでは、実際の推定が動くことは確認できない。

VGL単独起動時は、生成したモデルの親ディレクトリを渡す。

```bash
ros2 launch jetpilot_system_launch vgl.launch.py \
  container_name:=vgl_test_container \
  vgl_enabled_stereo_cameras:=realsense \
  vgl_map_dir:=/workspaces/map/E522-0907-v1/2026-09-07_01-28-58_20260907_011815_joy_start/cuvgl_map \
  vgl_model_dir:=/workspaces/ros2_ws/isaac_ros_assets/models/visual_global_localization_424x240
```

先に別ターミナルで`ros2 run rclcpp_components component_container_mt --ros-args -r __node:=vgl_test_container`を起動し、両方のターミナルでワークスペースをsourceする。

従来の広い入力範囲で生成する場合は`--native-profile`を使う。サイズ指定とは併用できない。このモードの既定出力先は従来の`visual_global_localization`で、既存の生成動作を維持する。`create_map.sh`からの呼び出しもこのモードを明示し、地図作成の入力範囲を変更しない。
