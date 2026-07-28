#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

exec "$@"
