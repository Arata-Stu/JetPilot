# スクリプトの使い分け

普段使うコマンドは `scripts/` 直下に置く。

| コマンド | 用途 |
| --- | --- |
| `screen.sh` | screen セッションを開始・再接続 |
| `bringup.sh` | ROS 起動・録画設定 |
| `build.sh` | ROS パッケージのビルド |
| `repos.sh` | JetPilot本体・外部リポジトリの状態確認・更新 |
| `jetson_display_mode.sh` | Jetson の画面設定 |
| `jetson_max_performance.sh` | Jetson の性能設定 |
| `bluetooth.sh` | Bluetooth の復旧 |
| `e2e_trt.sh` | E2E TensorRT エンジン生成・推論時間の統計表示 |
| `foxglove.sh` | 接続先IPを選択してFoxgloveを起動 |

`./scripts/foxglove.sh` は、FoxgloveをインストールしたMacなどのデスクトップで実行する。
`192.168.11.190` / `192.168.55.1` / `10.42.0.1` / `192.168.11.11` / `manual` から選択でき、
手入力ではIPv4アドレスを指定する。ポートは `8767`。Enterで最初の候補、`q`で中止。

その他のコマンドは用途別に配置する。以下はプロジェクトルートからのパス。

| 場所 | コマンド |
| --- | --- |
| `scripts/mapping/` | `create_map.sh`, `export_vgl_tensorrt_engines.sh` |
| `scripts/setup/` | `install_isaac_ros_cli.sh`, `install_silky_evcam_udev_rules.sh`, `prepare_workspace_dirs.sh`, `setup_multi_sensor_calibration_env.sh` |
| `scripts/maintenance/` | `cleanup_isaac_ros_docker.sh`, `python_env.sh`, `scp_data.sh`, `tmux.sh` |
| `scripts/diagnostics/` | `check.sh`, `generate_topic_graph.py`, `profile_topic_bw.sh` |
| `scripts/experiments/` | `export_led_sync_batch.sh`, `run_rc_popout_auto_pipeline.sh`, `led_sync_gui.sh`, `rc_popout_experiment.sh` |
| `scripts/lib/` | コマンド・Console が共通で使う内部処理 |
| `tests/scripts/` | スクリプトの軽量テスト |

```bash
./scripts/mapping/create_map.sh
./scripts/maintenance/python_env.sh sync training
./scripts/diagnostics/check.sh quick
```

移動したコマンドの旧パスは廃止。手元のエイリアスや外部のジョブから呼んでいる場合は新しいパスに更新する。
リポジトリ内の Console・CI・ドキュメントは新しいパスを使用する。
Docker では従来どおり `scripts/` ディレクトリ全体をマウントする。
