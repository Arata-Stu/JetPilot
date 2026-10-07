#!/usr/bin/env bash
set -eo pipefail
SCRIPT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
NEEDS_ROS=1
for arg in "$@"; do
  case "$arg" in --preflight|--help|-h) NEEDS_ROS=0 ;; esac
done
if [[ "$NEEDS_ROS" == 1 ]]; then
  source /opt/ros/jazzy/setup.bash
  source /workspaces/ros2_ws/install/setup.bash
fi
if [[ -n "${MULTI_SENSOR_CALIBRATION_VENV:-}" && -x "$MULTI_SENSOR_CALIBRATION_VENV/bin/python3" ]]; then
  ANALYSIS_PYTHON="$MULTI_SENSOR_CALIBRATION_VENV/bin/python3"
elif [[ -x /opt/multi_sensor_calibration_env/bin/python3 ]]; then
  ANALYSIS_PYTHON=/opt/multi_sensor_calibration_env/bin/python3
elif [[ -x /workspaces/.venvs/multi_sensor_calibration/bin/python3 ]]; then
  ANALYSIS_PYTHON=/workspaces/.venvs/multi_sensor_calibration/bin/python3
else
  ANALYSIS_PYTHON=python3
fi
exec "$ANALYSIS_PYTHON" "$SCRIPT_ROOT/tools/evaluate_rc_popout_grid_static.py" "$@"
