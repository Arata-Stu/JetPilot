# Isaac ROS CLI v4.6 移行監査

更新日: 2026-08-20

## 比較基準

| 対象 | Git ref | commit |
| --- | --- | --- |
| fork の共通祖先（公式4.5） | `release-4.5` | `385bb5d59d5c5946753e5a8bd03991472c91210d` |
| JetPilot fork の比較元 | `origin/jetpilot-v1.0.0` | `a7b1e2eec7a8d38b01fb642b3cfb149965f9d526` |
| 公式4.6 release branch | `upstream/release-4.6` | `b1eca83031c8c55d69f9ee383c1319cd8cb53eba` |
| 公式4.6 release tag | `v4.6-0` | `2591da7be776848b97f0e3ff895eb71995fc1eb6` |
| JetPilot統合commit | `codex/isaac-ros-cli-v4.6-sync` | `0ac6ffa70909f9dc099130818e2d40dd085451a1` |

公式のrelease branchとtagは同じtreeです。統合ではrelease tag `v4.6-0` をmerge元にし、
履歴上でも公式releaseを追跡できるようにしています。

- [公式 isaac-ros-cli](https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli)
- [公式 v4.6-0 release](https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/releases/tag/v4.6-0)
- [公式 v4.5-0...v4.6-0 の比較](https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/compare/v4.5-0...v4.6-0)
- [Orin向けTensorRT 10.13問題の公式issue](https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/issues/22)

## 公式4.6から採用した内容

- JetPack 7.2用のplatform判定（`arm64-jetpack`）と `ISAAC_ROS_PLATFORM` のbuild arg伝播
- CUDA 13.2、TensorRT 10.16、Jetson repository r39.2のAPT preference
- Isaac ROS APT repositoryの `release-4` 化
- I2C、GPIO、DRM render deviceの実GIDに基づく汎用的な権限設定
- GMSL用FSYNC deviceの権限設定
- Docker imageの `push` 設定と `--push` / `--no-push`
- v4.6のDebian post-install、post-remove、platform preference管理
- 公式layer orderへの `franka_manipulation` 追加
- 公式のRealSense CUDA toolkit選択

## 公式実装に置き換えて削除したfork差分

- 独自 `jp72_orin` Docker layer
- 独自 `jetpilot-jp72-orin.pref`
- `Dockerfile.isaac_ros` と `Dockerfile.noble` の手動r39.2書き換え
- RealSenseのarchitecture別CUDA version固定
- entrypointのDRM render、`gpiochip0`、`i2c-7`専用処理
- 使用箇所のないJetracer、`traitlets`、`questionary`、`jetson-gpio`
- 使用箇所のない `/debug` mount
- 重複した `rosbags==0.11.3`
- コメントアウトされた旧package一覧とFoundationStereo build断片
- install時の無条件 `git pull`

scriptsのmount先は、文書とbringup commandに合わせて `/scripts` から
`/workspaces/scripts` へ修正しました。

## JetPilot固有として維持した内容

- `additional_setting` layerとJetPilotで使用するIsaac ROS／ROS package
- DepthAI v3 layer
- SilkyEvCam／OpenEB layerと外部配布plugin sourceの受け口
- CycloneDDS profile、loopback multicast、ROS用network buffer設定
- joystick、VESC、USB LiDAR向けdevice group設定
- `/dev` bindによるhot-plugとudev symlink対応
- `scripts`、`tools`、`python_ws`、`record`、`map` のworkspace mount
- Dockerfileだけでなくローカル `COPY` / `ADD` 元も含めるimage hash
- local imageの優先確認、cache tagを消さないretag、leaf-only local build
- `jetson-stats` 7.2.0のcommit固定とoptional library probe patch
- x86_64学習環境の `onnxscript==0.7.1`

## 実機で確認する保留事項

独自 `jp72_orin` layerのCUDA／TensorRT部分は公式4.6と重複するため削除しました。
一方、同layer後半にあった次のworkaroundは公式4.6に完全な同等物がありません。

- `libnvvpi4=4.1.3`
- `nvidia-l4t-multimedia-utils=39.2.0-20260601141651`
- `libnvbufsurface_nvsci.so.1.0.0` の存在確認

まず公式4.6だけでbuild・起動します。`libnvbufsurface_nvsci` 不足が再現した場合は、
CUDA／TensorRTを再固定せず、このVPI/L4T runtimeだけを扱う小さな互換layerとして戻します。

RealSenseの独自 `--no_cuda` も公式設定へ戻しました。実機buildでのみ問題が再現する場合は、
原因とbuild logを確認してから限定的に復活させます。

DepthAI layerはOAK-D用として有効化しましたが、testing repositoryとDepthAI Python packageの
version上書きを含むため、Jetson実機でcamera起動まで確認します。

## Jetsonでの確認順序

1. CLI Debian packageをbuildし、versionが `2.5.0-1` 系であることを確認する。
2. container imageをcacheなしで一度buildする。
3. container内でCUDA 13.2、TensorRT 10.16、Jetson r39.2由来packageを確認する。
4. `/dev/i2c-1`、`/dev/gpiochip*`、`/dev/dri/render*` を非root userで開けることを確認する。
5. PCA9685 steering／ESCを安全に浮かせた状態で確認する。
6. RealSense、OAK-D、SilkyEvCamを個別に起動する。
7. `jtop` socket連携とIsaac ROS diagnosticsを確認する。
8. VPI利用nodeで `libnvbufsurface_nvsci` 関連errorがないことを確認する。

## 公開時の作業

統合branchをforkへpushした後、ルートの `packages.repos` がそのbranchまたはrelease tagを
参照するように更新します。push前のlocal branch名を先に記載すると、新規 `vcs import` が
失敗するため、この更新は公開と同時に行います。
