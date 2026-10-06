#!/usr/bin/env bash
set -euo pipefail
SCRIPT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -n "${MULTI_SENSOR_CALIBRATION_VENV:-}" ]]; then
  ANALYSIS_PYTHON="$MULTI_SENSOR_CALIBRATION_VENV/bin/python3"
elif [[ -x /opt/multi_sensor_calibration_env/bin/python3 ]]; then
  ANALYSIS_PYTHON=/opt/multi_sensor_calibration_env/bin/python3
elif [[ -x /workspaces/.venvs/multi_sensor_calibration/bin/python3 ]]; then
  ANALYSIS_PYTHON=/workspaces/.venvs/multi_sensor_calibration/bin/python3
else
  ANALYSIS_PYTHON=python3
fi
exec "$ANALYSIS_PYTHON" "$SCRIPT_ROOT/tools/analyze_rc_popout_grid_background.py" "$@"
