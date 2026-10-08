#!/usr/bin/env bash
set -euo pipefail
SCRIPT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -n "${MULTI_SENSOR_CALIBRATION_VENV:-}" ]]; then
  FIGURE_PYTHON="$MULTI_SENSOR_CALIBRATION_VENV/bin/python3"
elif [[ -x /opt/multi_sensor_calibration_env/bin/python3 ]]; then
  FIGURE_PYTHON=/opt/multi_sensor_calibration_env/bin/python3
elif [[ -x /workspaces/.venvs/multi_sensor_calibration/bin/python3 ]]; then
  FIGURE_PYTHON=/workspaces/.venvs/multi_sensor_calibration/bin/python3
else
  FIGURE_PYTHON=python3
fi
exec "$FIGURE_PYTHON" "$SCRIPT_ROOT/tools/rc_popout_event_figure.py" "$@"
