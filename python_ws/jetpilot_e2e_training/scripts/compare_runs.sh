#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ -x /opt/env/bin/python ]]; then
  PYTHON_BIN="${PYTHON_BIN:-/opt/env/bin/python}"
else
  PYTHON_BIN="${PYTHON_BIN:-python3}"
fi
export PYTHONPATH="${PACKAGE_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

ROOT="${1:-outputs/e2e}"
"$PYTHON_BIN" -m e2e_learning.cli.compare_runs compare.root="${ROOT}"
