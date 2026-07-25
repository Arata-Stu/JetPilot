# JetPilot

## Docs

- [Hardware Interface Package Design](docs/hardware_interface_packages.md)
- [Isaac ROS Launch Guidelines](docs/isaac_ros_launch_guidelines.md)
- [Localization Manager](ros2_ws/src/localization/jetpilot_localization_manager/README.md)
- [Bringup presets / TUI](docs/bringup_launcher.md)
- [Planning / Control Architecture](docs/planning_control_architecture.md)
- [rosbag Replay Safety](docs/rosbag_replay_safety.md)

## Isaac ROS Docker の世代整理

Dockerイメージを繰り返しビルドした後は、次のスクリプトで古い
`additional_setting` イメージとBuildKitキャッシュを整理できます。
既定ではプレビューのみで、Dockerのデータは変更しません。

```bash
./scripts/cleanup_isaac_ros_docker.sh
```

表示内容を確認して実行します。現行の
`cached_isaac_run_dev_image_local:latest` と同じIMAGE ID、およびコンテナが
参照しているイメージは削除されません。既定のキャッシュ設定は「7日より
古いdangling cacheを対象、保持量50GB」です。

```bash
./scripts/cleanup_isaac_ros_docker.sh --execute
```

使用されていないキャッシュ全体を対象にする場合は、`--all-cache` を付けます。
イメージは同じ保護条件のままです。

```bash
./scripts/cleanup_isaac_ros_docker.sh --execute --all-cache
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
  --execute --yes --all-cache \
  --cache-builder default --cache-until 336h --cache-keep 80GB
```
