#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
NEEDS_RUNTIME=true
for arg in "$@"; do
  case "$arg" in --preflight|-h|--help) NEEDS_RUNTIME=false ;; esac
done
if [[ "$NEEDS_RUNTIME" == true ]]; then
  if [[ -n "${ROS2_SETUP_FILE:-}" ]]; then
    source "$ROS2_SETUP_FILE"
  else
    [[ ! -f /opt/ros/jazzy/setup.bash ]] || source /opt/ros/jazzy/setup.bash
    source "${ROS2_WS:-$PROJECT_ROOT/ros2_ws}/install/setup.bash"
  fi
fi
if [[ -n "${MULTI_SENSOR_CALIBRATION_VENV:-}" ]]; then
  VIDEO_PYTHON="$MULTI_SENSOR_CALIBRATION_VENV/bin/python3"
elif [[ -x /opt/multi_sensor_calibration_env/bin/python3 ]]; then
  VIDEO_PYTHON=/opt/multi_sensor_calibration_env/bin/python3
elif [[ -x /workspaces/.venvs/multi_sensor_calibration/bin/python3 ]]; then
  VIDEO_PYTHON=/workspaces/.venvs/multi_sensor_calibration/bin/python3
else
  VIDEO_PYTHON=python3
fi
exec "$VIDEO_PYTHON" "$PROJECT_ROOT/tools/rc_popout_static_samples.py" "$@"
