# SilkyEvCam の保存済み設定

JetPilot で使用するカメラ設定を保管します。package のビルド時に
`share/jetpilot_system_launch/config/sensing/silkyevcam/` へインストールされます。

| ファイル | 内容 |
| --- | --- |
| `E522.bias` | 保存済み `silkyevcam_custom.bias` のコピー。`bias_diff_off=178`、`bias_diff_on=460` |
| `camera.bias` | 旧 `ros2_ws/data.bias`。保存済みのバイアス値。ROS 2 の `bias_file` で読み込み可能 |
| `camera_settings.json` | 旧 `ros2_ws/settings.json`。バイアス、ROI、同期、トリガー、フィルターなどの設定スナップショット |

移動時点では `camera.bias` と `camera_settings.json` のバイアス値は一致しています。
`E522.bias` は別の調整済み設定です。元ファイルの内容は変更していません。
これらを配置しただけでは自動適用されません。実機での適用確認は未実施です。
現在の ROS 2 ドライバーは `.bias` の読み込みに対応し、JSON 全体の読み込みには対応していません。

ビルド後に workspace の `install/setup.bash` を source して実行します。

```bash
ros2 launch openeb_ros2 pipeline.launch.py \
  bias_file:="$(ros2 pkg prefix --share jetpilot_system_launch)/config/sensing/silkyevcam/camera.bias"
```

bringup では同じパスを `sensor_kit_silky_evcam_bias_file` に渡します。
`scripts/bringup.sh` の TUI では、SilkyEvCam を含むセンサー構成を選ぶと
このディレクトリの `.bias` 一覧から `E522.bias` などを選択できます。
Docker 内で起動する場合はコンテナから見えるパスを使用してください。

調整中の出力は project root の `record/silkyevcam_bias/` に保管し、運用に使う設定を
このディレクトリへコピーして管理します。設定を更新した後は再ビルド・再起動が必要です。
