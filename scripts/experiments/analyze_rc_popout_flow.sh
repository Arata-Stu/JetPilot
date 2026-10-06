#!/usr/bin/env bash
set -eo pipefail
SCRIPT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
if [[ -n "${MULTI_SENSOR_CALIBRATION_VENV:-}" && -f "$MULTI_SENSOR_CALIBRATION_VENV/bin/activate" ]]; then
  source "$MULTI_SENSOR_CALIBRATION_VENV/bin/activate"
elif [[ -f /opt/multi_sensor_calibration_env/bin/activate ]]; then
  source /opt/multi_sensor_calibration_env/bin/activate
else
  source /workspaces/.venvs/multi_sensor_calibration/bin/activate
fi
exec python3 "$SCRIPT_ROOT/tools/analyze_rc_popout_flow.py" "$@"
