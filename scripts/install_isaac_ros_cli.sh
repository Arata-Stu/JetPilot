#!/usr/bin/env bash
set -euo pipefail

cd "${ISAAC_ROS_WS}/../tools/isaac-ros-cli"

git pull

sudo apt-get update
sudo apt-get install -y build-essential dpkg-dev debhelper dh-python make

make distclean
make build-stamped

sudo apt install -y ../isaac-ros-cli_*.deb
