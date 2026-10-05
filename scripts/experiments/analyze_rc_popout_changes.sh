#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
# Standard-library-only analysis: no ROS or Metavision environment is needed.
exec python3 "$ROOT/tools/rc_popout_change_detection.py" "$@"
