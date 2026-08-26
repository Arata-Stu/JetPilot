# JetPilot

## Docs

- [Project Design Policy](docs/project_design_policy.md)
- [Hardware Interface Package Design](docs/hardware_interface_packages.md)
- [JPBB-01 Firmware / Build / Flash](docs/jpbb_firmware.md)
- [Isaac ROS Launch Guidelines](docs/isaac_ros_launch_guidelines.md)
- [Localization Manager](ros2_ws/src/localization/jetpilot_localization_manager/README.md)
- [Bringup presets / TUI](docs/bringup_launcher.md)
- [Bringup profile追加ルール](docs/bringup_profile_rules.md)
- [Planning / Control Architecture](docs/planning_control_architecture.md)
- [rosbag Replay Safety](docs/rosbag_replay_safety.md)
- [YOLOv8 Training / Retraining](python_ws/jetpilot_object_detection_training/README.md)
- [YOLOv8 Isaac ROS Runtime / Decoder](ros2_ws/src/perception/jetpilot_object_detection/README.md)

## Isaac ROS Docker の世代整理

Dockerイメージを繰り返しビルドした後は、次のスクリプトで古い
`additional_setting` イメージとBuildKitキャッシュを整理できます。
既定では削除予定と現在の使用量を表示した後、削除を実行するか対話形式で
確認します。

```bash
./scripts/cleanup_isaac_ros_docker.sh
```

`y` または `yes` を入力した場合だけ削除を実行します。それ以外の入力と
Enterのみの場合はキャンセルします。現行の
`cached_isaac_run_dev_image_local:latest` と同じIMAGE ID、およびコンテナが
参照しているイメージは削除されません。既定のキャッシュ設定は「7日より
古いdangling cacheを対象、保持量50GB」です。

表示だけ行い、確認プロンプトを出さずに終了する場合は `--dry-run` を使います。

```bash
./scripts/cleanup_isaac_ros_docker.sh --dry-run
```

使用されていないキャッシュ全体を対象にする場合は、`--all-cache` を付けます。
イメージは同じ保護条件のままです。

```bash
./scripts/cleanup_isaac_ros_docker.sh --all-cache
```

処理後は、`docker system df` が報告するカテゴリ別の変更前・変更後・削減量と、
合計削減量を表示します。

```text
Docker storage: 335.20 GB -> 79.80 GB (255.40 GB freed)
```

この値はDockerが報告する論理使用量の合計です。共有レイヤー、圧縮コンテンツ、
Docker Desktopの仮想ディスクなどの影響により、ホスト側の空き容量増加とは
完全に一致しない場合があります。

CIや定期メンテナンスなど、確認プロンプトを出せない場合だけ `--yes` を
併用します。

```bash
./scripts/cleanup_isaac_ros_docker.sh \
  --yes --all-cache \
  --cache-builder default --cache-until 336h --cache-keep 80GB
```
