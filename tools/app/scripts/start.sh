#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
APP_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd -- "${APP_ROOT}/../.." && pwd)"

export PYTHONPATH="${APP_ROOT}/backend${PYTHONPATH:+:${PYTHONPATH}}"

cd "$REPO_ROOT"
exec python3 -m jetpilot_console.main "$@"

