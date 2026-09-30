#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_WS="${ROS2_WS:-${PROJECT_ROOT}/ros2_ws}"
# Help and planning need neither ROS nor Metavision.
NEEDS_RUNTIME=true
for arg in "$@"; do
  case "$arg" in --dry-run|-h|--help) NEEDS_RUNTIME=false ;; esac
done
if [[ "$NEEDS_RUNTIME" == true ]]; then
  setup="${ROS2_SETUP_FILE:-${ROS2_WS}/install/setup.bash}"
  [[ -f "$setup" ]] || { echo "ROS setup not found: $setup" >&2; exit 1; }
  set +u
  source "$setup"
  if [[ -n "${MULTI_SENSOR_CALIBRATION_VENV:-}" ]]; then
    source "$MULTI_SENSOR_CALIBRATION_VENV/bin/activate"
  elif [[ -f /opt/multi_sensor_calibration_env/bin/activate ]]; then
    source /opt/multi_sensor_calibration_env/bin/activate
  elif [[ -f /workspaces/.venvs/multi_sensor_calibration/bin/activate ]]; then
    source /workspaces/.venvs/multi_sensor_calibration/bin/activate
  fi
  set -u
fi
exec python3 "$PROJECT_ROOT/scripts/experiments/generate_rc_popout_common_views.py" \
  --camchain "$ROS2_WS/src/tool/multi_sensor_calibration/config/calibrations/rc_popout_default/kalibr-camchain.yaml" "$@"
