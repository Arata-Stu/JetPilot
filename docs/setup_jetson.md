# Jetson & Isaac ROS Setup Guide

このドキュメントでは、Jetson 環境の初期設定、デバイス設定、および Isaac ROS を実行するための Docker 環境構築手順を説明します。

x86_64 環境の手順は [setup_x86_64.md](setup_x86_64.md) を参照してください。

主な Jetson 固有の差分は、`jetson-stats` (`jtop`)、`nvpmodel`、Wi-Fi ドングル設定、および JetPack 前提の Docker / NVIDIA runtime 設定です。

---

## 1. 基本システムのセットアップ

システムのアップデートと、開発に必要な基本ツールをインストールします。

```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-vcstool \
  git \
  git-lfs \
  tmux \
  screen \
  terminator \
  xrdp

git lfs install --skip-repo
```

---

## 2. リモートデスクトップ (xrdp) の設定

リモート接続時に Ubuntu の GNOME デスクトップ環境が正常に立ち上がるよう、設定ファイルを上書きします。

```bash
sudo tee /etc/xrdp/startwm.sh > /dev/null << 'EOF'
#!/bin/sh

if test -r /etc/profile; then
        . /etc/profile
fi

export GNOME_SHELL_SESSION_MODE=ubuntu
export XDG_CURRENT_DESKTOP=ubuntu:GNOME
exec gnome-session
EOF
```

---

## 3. Docker / NVIDIA Container Runtime のセットアップ

Jetson では JetPack に含まれる Docker / NVIDIA runtime を前提に、Docker のデフォルトランタイムを NVIDIA に設定します。

```bash
sudo apt update
sudo apt install -y jq

sudo sh -c 'test -s /etc/docker/daemon.json || echo "{}" > /etc/docker/daemon.json'
sudo jq '. + {"default-runtime": "nvidia"}' /etc/docker/daemon.json | \
  sudo tee /etc/docker/daemon.json.tmp > /dev/null
sudo mv /etc/docker/daemon.json.tmp /etc/docker/daemon.json

sudo systemctl daemon-reload
sudo systemctl restart docker
sudo usermod -aG docker "$USER"

# グループ変更を現在のセッションに適用します。再ログインでも反映できます。
newgrp docker
```

---

## 4. デバイス・プラットフォーム固有設定

Jetson 固有の監視ツール、電力・パフォーマンスモード、Wi-Fi ドングル、および RealSense の設定を行います。

### 4.1 jetson-stats (`jtop`)

Jetson の状態確認に使用する `jetson-stats` をインストールします。
ホスト上のサービスとコンテナ内のクライアントは `/run/jtop.sock` 経由で通信するため、
Dockerfile と同じ不変コミットに固定します。`master` やブランチは使用しないでください。

```bash
JETSON_STATS_REF=3c1ba9ac49a1307c9d7c53646bb70ba8c16b8759
sudo python3 -m pip install --break-system-packages --force-reinstall --no-deps \
  "jetson-stats @ git+https://github.com/rbonghi/jetson_stats.git@${JETSON_STATS_REF}"
python3 -c 'import jtop; print(jtop.__version__)'  # 7.2.0

# ホストサービスへ更新を反映します。
sudo systemctl restart jtop.service

# インストール後、必要に応じて再ログインまたは再起動してから確認します。
jtop
```

`--no-deps` は、APTで導入された `python3-distro` などをpipが上書き・削除しようとするのを
防ぎます。JetsonホストではOS管理のPython依存パッケージをそのまま使用します。

このコミットは `jetson-stats` のタグ `7.2.0` が指すリビジョンです。コンテナ側では、
VPI の任意依存ライブラリが存在しない場合にも診断ノードを継続できる限定的なパッチを
適用しています。通信プロトコル上のバージョンはホストと同じ `7.2.0` のままです。

Isaac ROS CLI v4.6 が JetPack 7.2 の CUDA 13.2、TensorRT 10.16、Jetson r39.2
向けAPT設定を公式に提供するため、JetPilot独自の `jp72_orin` レイヤーは使用しません。
公式との差分と実機確認項目は [Isaac ROS CLI v4.6 移行監査](isaac_ros_cli_v46_migration.md)
にまとめています。

### 4.2 パフォーマンスモードの変更

Jetson の電力・パフォーマンスモードを設定します（`-m 2` のモードを適用）。

```bash
sudo /usr/sbin/nvpmodel -m 2
```

### 4.3 Wi-Fi ドングル (RTL88x2BU) のセットアップ

競合する古いドライバーを削除し、DKMS を使用して新しいドライバーをビルド・インストールします。

```bash
sudo apt update
sudo apt install -y dkms build-essential

# 既存のドライバを削除・アンロード
sudo dkms remove rtl8821cu/5.12.0.4 --all
sudo modprobe -r 8821cu

# 新しいドライバの取得と DKMS への追加
sudo git clone https://github.com/RinCat/RTL88x2BU-Linux-Driver.git /usr/src/rtl88x2bu-git
sudo sed -i 's/PACKAGE_VERSION="@PKGVER@"/PACKAGE_VERSION="git"/g' /usr/src/rtl88x2bu-git/dkms.conf
sudo dkms add -m rtl88x2bu -v git
sudo dkms autoinstall

# ドライバのロード
sudo modprobe 88x2bu

# 接続状態の確認
ip link
iw dev
nmcli device
```

### 4.4 Intel RealSense udev ルールの設定

RealSense カメラを USB 経由で正常に認識させるための udev ルールを適用します。

```bash
wget https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.3/config/99-realsense-libusb.rules && \
sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/ && \
sudo udevadm control --reload-rules && \
sudo udevadm trigger && \
echo "Successfully added udev rules"
```

### 4.5 CenturyArks SilkyEvCam udev ルールの設定

SilkyEvCam を Docker コンテナ内の通常ユーザーから扱えるようにするため、host 側に udev ルールを適用します。

```bash
cd "${RC_AS_ROOT}"
./scripts/install_silky_evcam_udev_rules.sh
```

適用後、SilkyEvCam を抜き差しし、Docker コンテナを再起動してください。

### 4.6 Luxonis OAK udev ルールの設定

OAK-D LiteをDockerコンテナ内の通常ユーザーから扱えるようにするため、host側に
udevルールを適用します。

```bash
cd "${RC_AS_ROOT}"
./scripts/install_depthai_udev_rules.sh
```

適用後、OAK-D Liteを抜き差しし、Dockerコンテナを再起動してください。

---

## 5. ワークスペースと Isaac ROS CLI の構築

作業用リポジトリをクローンし、環境変数の設定、および `isaac-ros-cli` のビルドとインストールを行います。

このリポジトリで使用する `isaac-ros-cli` は、JetPilotのproject root `${RC_AS_ROOT}` を
containerの `/workspaces` へ1回だけmountします。そのため `ros2_ws`、`python_ws`、
`scripts`、`tools`、`record`、`map` に加え、今後追加するトップレベルdirectoryも
同じ相対pathで自動的に参照できます。ROS 2 workspaceは従来どおり
`/workspaces/ros2_ws` です。

project rootは読み書き可能な状態でmountされるため、container内での変更は`.git`や文書を
含めてホストへ反映されます。既に旧mount構成のcontainerが動いている場合は、終了してから
再度起動してください。

```bash
mkdir -p "${HOME}/workspaces"
cd "${HOME}/workspaces"
git clone git@github.com:Arata-Stu/JetPilot.git

# 環境変数の設定 

echo 'export PKG_NAME="JetPilot"' >> ~/.bashrc
echo 'export RC_AS_ROOT="${HOME}/workspaces/${PKG_NAME}"' >> ~/.bashrc
echo 'export ISAAC_ROS_WS="${RC_AS_ROOT}/ros2_ws"' >> ~/.bashrc
echo 'export ISAAC_DIR="${ISAAC_ROS_WS}"' >> ~/.bashrc

# 現在のセッションにも環境変数を適用
export PKG_NAME="JetPilot"
export RC_AS_ROOT="${HOME}/workspaces/${PKG_NAME}"
export ISAAC_ROS_WS="${RC_AS_ROOT}/ros2_ws"
export ISAAC_DIR="${ISAAC_ROS_WS}"

cd "${RC_AS_ROOT}"
./scripts/prepare_workspace_dirs.sh
vcs import < packages.repos

# CenturyArks SilkyEvCam plugin source を Docker build context に配置
# 公式からの plugin install / zip 入手は CLI では行えないため、事前にユーザーが実施する
mkdir -p "${RC_AS_ROOT}/tools/isaac-ros-cli/docker/silky_evcam_plugin_source"
cd "${HOME}/Downloads"
unzip SilkyEvCam_plugin_Source_for_MV511.zip
cp -av \
  SilkyEvCam_plugin_Source_for_MV511/hal \
  SilkyEvCam_plugin_Source_for_MV511/hal_psee_plugins \
  SilkyEvCam_plugin_Source_for_MV511/licensing \
  "${RC_AS_ROOT}/tools/isaac-ros-cli/docker/silky_evcam_plugin_source/"

# Isaac ROS CLI 用の設定ファイルを作成
mkdir -p "${ISAAC_ROS_WS}/.isaac-ros-cli"
cat > "${ISAAC_ROS_WS}/.isaac-ros-cli/config.yaml" <<'EOF'
docker:
  image:
    additional_image_keys:
      - realsense
      - silky_evcam
      - additional_setting
EOF

# Isaac ROS CLI のビルドとインストール
cd "${RC_AS_ROOT}"
./scripts/install_isaac_ros_cli.sh

# インストールの確認
dpkg -s isaac-ros-cli | grep Version
grep -n "get_container_workspace_path" /usr/lib/isaac-ros-cli/run_dev.py
grep -n "get_workspace_mount_args" /usr/lib/isaac-ros-cli/run_dev.py
grep -n 'project_root.*:/workspaces' /usr/lib/isaac-ros-cli/run_dev.py
grep -n -- '-v /dev:/dev' /usr/lib/isaac-ros-cli/run_dev.py
ls -l /etc/isaac-ros-cli/docker/Dockerfile.silky_evcam
ls -l /etc/isaac-ros-cli/docker/Dockerfile.depthai
ls -l /etc/isaac-ros-cli/docker/Dockerfile.additional_setting
```

`vcs import`ではJPBB-01の基板資料とSTM32 firmwareも
`hardware/jetpilot_bridge_board`へ取得される。ビルドと書き込みは
[JPBB-01 Firmware / Build / Flash](jpbb_firmware.md)を参照する。

以後、`isaac-ros-cli` の変更を再ビルド・再インストールする場合は、プロジェクトのルートから次を実行します。

```bash
./scripts/install_isaac_ros_cli.sh
```

---

## 6. Isaac ROS Docker コンテナのビルドと起動

Isaac ROS の Docker 環境を初期化し、コンテナをビルドして起動します。

```bash
sudo isaac-ros init docker
isaac-ros activate --build-local

# コンテナ内で mount 先を確認
findmnt -T /workspaces
ls -ld /workspaces /workspaces/ros2_ws /workspaces/python_ws \
  /workspaces/scripts /workspaces/tools /workspaces/record /workspaces/map
```

`record`、`map`、`ros2_ws/models/e2e`、E2E学習用の`datasets`と`outputs/e2e`は、
空の骨組みだけをGitで管理しています。生成物自体はGit管理されません。
Docker起動前に`prepare_workspace_dirs.sh`が必要な生成物directoryを作成し、
ホストuserと同じUID/GIDで書き込めることを確認します。
