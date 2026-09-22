#!/usr/bin/env bash
set -euo pipefail

VENV_DIR="/workspaces/.venvs/multi_sensor_calibration"

mkdir -p "$(dirname "$VENV_DIR")"
/usr/bin/python3 -m venv --clear --system-site-packages "$VENV_DIR"
"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install \
  "numpy==1.26.4" \
  "PyYAML==6.0.2" \
  "rosbags==0.11.3"

"$VENV_DIR/bin/python" - <<'PY'
import cv2
import numpy
import rosbags
import yaml

assert numpy.__version__ == "1.26.4", numpy.__version__
print(f"NumPy {numpy.__version__}")
print(f"OpenCV {cv2.__version__}")
print("rosbags / PyYAML OK")

try:
    from metavision_core.event_io import EventsIterator  # noqa: F401
except ImportError as exc:
    raise SystemExit(
        "Metavision Python bindings are not visible. "
        "Use the silky_evcam Docker image layer."
    ) from exc

print("Metavision Python bindings OK")
PY

echo
echo "Environment created: $VENV_DIR"
echo "Activate with: source $VENV_DIR/bin/activate"
