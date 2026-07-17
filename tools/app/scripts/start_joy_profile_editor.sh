#!/usr/bin/env bash
set -euo pipefail

HOST="127.0.0.1"
PORT="8766"
ALLOW_REMOTE="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      HOST="$2"
      shift 2
      ;;
    --port)
      PORT="$2"
      shift 2
      ;;
    --allow-remote)
      ALLOW_REMOTE="true"
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
APP_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd -- "${APP_ROOT}/../.." && pwd)"

export JETPILOT_REPO_ROOT="${JETPILOT_REPO_ROOT:-$REPO_ROOT}"
export ROS2_WS="${ROS2_WS:-${REPO_ROOT}/ros2_ws}"
export PYTHONPATH="${APP_ROOT}/backend${PYTHONPATH:+:${PYTHONPATH}}"

ARGS=(--joy-only --host "$HOST" --port "$PORT")
if [[ "$ALLOW_REMOTE" == "true" ]]; then
  ARGS+=(--allow-remote)
fi

cd "$REPO_ROOT"
exec python3 -m jetpilot_console.main "${ARGS[@]}"
