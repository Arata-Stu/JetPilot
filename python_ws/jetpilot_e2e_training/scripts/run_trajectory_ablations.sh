#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd -- "${PACKAGE_DIR}/../.." && pwd)"

if [[ -x /opt/env/bin/python ]]; then
  PYTHON_BIN="${PYTHON_BIN:-/opt/env/bin/python}"
else
  PYTHON_BIN="${PYTHON_BIN:-python3}"
fi
export PYTHONPATH="${PACKAGE_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

DATASET_DIR="${1:-}"
ABLATION_NAME="${2:-trajectory_ablation_$(date +%Y%m%d_%H%M%S)}"
EXTRA_OVERRIDES=("${@:3}")
if [[ -z "$DATASET_DIR" ]]; then
  echo "Usage: scripts/run_trajectory_ablations.sh DATASET_DIR [ABLATION_NAME]" >&2
  exit 1
fi

OUTPUT_ROOT="${PROJECT_ROOT}/outputs/e2e_ablations/${ABLATION_NAME}/runs"
EXPERIMENTS=(
  trajectory_pilotnet
  trajectory_pilotnet_gru
  trajectory_pilotnet_imu
  trajectory_pilotnet_gru_imu
)

for experiment in "${EXPERIMENTS[@]}"; do
  run_name="${ABLATION_NAME}_${experiment}"
  echo "=== ${experiment}: ${run_name} ==="
  "$PYTHON_BIN" -m e2e_learning.cli.train \
    experiment="${experiment}" \
    data.dataset_dir="${DATASET_DIR}" \
    run.name="${run_name}" \
    run.output_root="${OUTPUT_ROOT}" \
    "${EXTRA_OVERRIDES[@]}"
  "$PYTHON_BIN" -m e2e_learning.cli.export_onnx \
    checkpoint="${OUTPUT_ROOT}/${run_name}/checkpoints/best.pt" \
    export.output_dir="${OUTPUT_ROOT}/${run_name}"
done

"$PYTHON_BIN" -m e2e_learning.cli.compare_runs \
  compare.root="${PROJECT_ROOT}/outputs/e2e_ablations/${ABLATION_NAME}"

echo "Ablation summary: ${PROJECT_ROOT}/outputs/e2e_ablations/${ABLATION_NAME}/summary.md"
