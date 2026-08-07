# x86_64 & Isaac ROS Setup Guide

このドキュメントでは、x86_64 Ubuntu 環境の初期設定、デバイス設定、および Isaac ROS を実行するための Docker 環境構築手順を説明します。

Jetson 環境の手順は [setup_jetson.md](setup_jetson.md) を参照してください。
主な x86_64 固有の差分は、Docker Engine と NVIDIA Container Toolkit を明示的にインストールする点です。Jetson 版の `jetson-stats` (`jtop`)、`nvpmodel`、Wi-Fi ドングル設定は含めていません。

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

x86_64 では Docker Engine と NVIDIA Container Toolkit をインストールし、Docker のデフォルトランタイムを NVIDIA に設定します。

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg2 jq

# Docker の公式 GPG key と apt repository を追加
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources > /dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# NVIDIA Container Toolkit の apt repository を追加
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null

sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update

export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.19.1-1
sudo apt-get install -y \
  nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
  nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
  libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
  libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}

# Docker のデフォルトランタイムを NVIDIA に設定
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker


sudo systemctl daemon-reload
sudo systemctl restart docker
sudo usermod -aG docker "$USER"

# グループ変更を現在のセッションに適用します。再ログインでも反映できます。
newgrp docker
```

---

## 4. デバイス・プラットフォーム固有設定

x86_64環境では、使用するUSB sensorのudevルールを適用します。

### 4.1 Intel RealSense udev ルールの設定

RealSense カメラを USB 経由で正常に認識させるための udev ルールを適用します。

```bash
wget https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.3/config/99-realsense-libusb.rules && \
sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/ && \
sudo udevadm control --reload-rules && \
sudo udevadm trigger && \
echo "Successfully added udev rules"
```

### 4.2 CenturyArks SilkyEvCam udev ルールの設定

SilkyEvCam を Docker コンテナ内の通常ユーザーから扱えるようにするため、host 側に udev ルールを適用します。

```bash
cd "${RC_AS_ROOT}"
./scripts/install_silky_evcam_udev_rules.sh
```

適用後、SilkyEvCam を抜き差しし、Docker コンテナを再起動してください。

### 4.3 Luxonis OAK udev ルールの設定

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

このリポジトリで使用する `isaac-ros-cli` は、`${RC_AS_ROOT}/ros2_ws` を `/workspaces/ros2_ws` に mount します。あわせて `${RC_AS_ROOT}` 直下の `python_ws`、`record`、`map` も、それぞれ `/workspaces/python_ws`、`/workspaces/record`、`/workspaces/map` に mount されます。

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
grep -n 'python_ws.*workspaces/python_ws' /usr/lib/isaac-ros-cli/run_dev.py
grep -n 'record.*workspaces/record' /usr/lib/isaac-ros-cli/run_dev.py
grep -n 'map.*workspaces/map' /usr/lib/isaac-ros-cli/run_dev.py
grep -n -- '-v /dev:/dev' /usr/lib/isaac-ros-cli/run_dev.py
ls -l /etc/isaac-ros-cli/docker/Dockerfile.silky_evcam
ls -l /etc/isaac-ros-cli/docker/Dockerfile.depthai
ls -l /etc/isaac-ros-cli/docker/Dockerfile.additional_setting
```

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
ls -ld /workspaces/ros2_ws /workspaces/python_ws /workspaces/record /workspaces/map
```
