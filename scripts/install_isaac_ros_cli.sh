#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/prepare_workspace_dirs.sh"

cd "${ISAAC_ROS_WS}/../tools/isaac-ros-cli"

git pull

sudo apt-get update
sudo apt-get install -y build-essential dpkg-dev debhelper dh-python make

make distclean
make build-stamped

sudo apt install -y --allow-downgrades ../isaac-ros-cli_*.deb
