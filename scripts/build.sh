#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/project_env.sh"
exec python3 "${SCRIPT_DIR}/lib/build_workspace.py" "$@"
