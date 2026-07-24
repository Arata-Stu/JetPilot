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
EXPERIMENT_NAME="${2:-e2e_compare_$(date +%Y%m%d_%H%M%S)}"

if [[ -z "$DATASET_DIR" ]]; then
  echo "Usage: scripts/run_experiments.sh DATASET_DIR [EXPERIMENT_NAME]" >&2
  exit 1
fi

OUTPUT_ROOT="${PROJECT_ROOT}/outputs/e2e_experiments/${EXPERIMENT_NAME}/runs"
EXPERIMENTS=(
  pilotnet_scratch
  mobilenet_frozen_head
  mobilenet_head_then_finetune
)
FRACTIONS=(1.0 0.3 0.1)

for experiment in "${EXPERIMENTS[@]}"; do
  for fraction in "${FRACTIONS[@]}"; do
    run_name="${experiment}_frac${fraction}"
    echo "=== ${run_name} ==="
    "$PYTHON_BIN" -m e2e_learning.cli.train \
      experiment="${experiment}" \
      data.dataset_dir="${DATASET_DIR}" \
      data.fraction="${fraction}" \
      run.name="${run_name}" \
      run.output_root="${OUTPUT_ROOT}"
    "$PYTHON_BIN" -m e2e_learning.cli.export_onnx \
      experiment="${experiment}" \
      data.dataset_dir="${DATASET_DIR}" \
      data.fraction="${fraction}" \
      run.name="${run_name}" \
      run.output_root="${OUTPUT_ROOT}" \
      checkpoint="${OUTPUT_ROOT}/${run_name}/checkpoints/best.pt" \
      export.output_dir="${OUTPUT_ROOT}/${run_name}"
  done
done

"$PYTHON_BIN" -m e2e_learning.cli.compare_runs compare.root="${PROJECT_ROOT}/outputs/e2e_experiments/${EXPERIMENT_NAME}"
