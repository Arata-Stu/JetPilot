#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
REPOSITORY_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

if [[ -f /workspaces/ros2_ws/install/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /workspaces/ros2_ws/install/setup.bash
  set -u
fi

if [[ -n "${MULTI_SENSOR_CALIBRATION_VENV:-}" && -f "$MULTI_SENSOR_CALIBRATION_VENV/bin/activate" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$MULTI_SENSOR_CALIBRATION_VENV/bin/activate"
  set -u
elif [[ -f /opt/multi_sensor_calibration_env/bin/activate ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/multi_sensor_calibration_env/bin/activate
  set -u
elif [[ -f /workspaces/.venvs/multi_sensor_calibration/bin/activate ]]; then
  set +u
  # shellcheck disable=SC1091
  source /workspaces/.venvs/multi_sensor_calibration/bin/activate
  set -u
fi

exec python3 "$REPOSITORY_ROOT/tools/led_sync_viewer/serve.py" "$@"
