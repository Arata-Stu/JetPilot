#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u
if [[ -n "${MULTI_SENSOR_CALIBRATION_VENV:-}" ]]; then
  source "$MULTI_SENSOR_CALIBRATION_VENV/bin/activate"
elif [[ -f /opt/multi_sensor_calibration_env/bin/activate ]]; then
  source /opt/multi_sensor_calibration_env/bin/activate
elif [[ -f /workspaces/.venvs/multi_sensor_calibration/bin/activate ]]; then
  source /workspaces/.venvs/multi_sensor_calibration/bin/activate
fi
set -u
exec python3 "$ROOT/scripts/experiments/share_rc_popout_roi.py" "$@"
