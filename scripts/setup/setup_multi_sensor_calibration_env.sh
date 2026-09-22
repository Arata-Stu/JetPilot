#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${SCRIPT_DIR}/maintenance/python_env.sh" sync calibration \
  --venv "${MULTI_SENSOR_CALIBRATION_VENV:-/opt/multi_sensor_calibration_env}" "$@"
