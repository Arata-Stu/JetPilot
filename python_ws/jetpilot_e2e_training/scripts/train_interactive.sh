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

choose_from_list() {
  local prompt="$1"
  shift
  local items=("$@")
  local choice
  echo "${prompt}" >&2
  select choice in "${items[@]}"; do
    [[ -n "${choice:-}" ]] && {
      printf '%s\n' "$choice"
      return
    }
    echo "番号を選んでください。" >&2
  done
}

dataset_candidates=()
while IFS= read -r path; do
  dataset_candidates+=("$path")
done < <(
  find "${PROJECT_ROOT}/datasets/e2e" -maxdepth 2 -name samples.csv -print 2>/dev/null \
    | sed 's#/samples.csv$##' | sort -r
)

if ((${#dataset_candidates[@]} == 0)); then
  echo "既存datasetが見つかりません。先に前処理を実行します。" >&2
  "${SCRIPT_DIR}/preprocess_interactive.sh"
  dataset_candidates=()
  while IFS= read -r path; do
    dataset_candidates+=("$path")
  done < <(
    find "${PROJECT_ROOT}/datasets/e2e" -maxdepth 2 -name samples.csv -print 2>/dev/null \
      | sed 's#/samples.csv$##' | sort -r
  )
fi

dataset_dir="$(choose_from_list "学習に使うdatasetを選択してください:" "${dataset_candidates[@]}")"
read -r -p "run名 [pilotnet_$(date +%Y%m%d_%H%M%S)]: " run_name
run_name="${run_name:-pilotnet_$(date +%Y%m%d_%H%M%S)}"

echo "学習presetを選択してください:" >&2
select experiment in \
  "pilotnet_scratch" \
  "mobilenet_frozen_head" \
  "mobilenet_head_then_finetune"; do
  [[ -n "${experiment:-}" ]] && break
  echo "番号を選んでください。" >&2
done

echo "入力サイズpresetを選択してください:" >&2
select size_preset in "424x240 -> 212x120" "640x480 -> 320x240"; do
  case "$REPLY" in
    1) input_width=212; input_height=120; break ;;
    2) input_width=320; input_height=240; break ;;
    *) echo "番号を選んでください。" >&2 ;;
  esac
done

output_root="${PROJECT_ROOT}/outputs/e2e"
"$PYTHON_BIN" -m e2e_learning.cli.train \
  experiment="${experiment}" \
  data.dataset_dir="${dataset_dir}" \
  data.input_width="${input_width}" \
  data.input_height="${input_height}" \
  run.name="${run_name}" \
  run.output_root="${output_root}"

checkpoint="${output_root}/${run_name}/checkpoints/best.pt"
read -r -p "ONNX exportも実行しますか？ [Y/n]: " do_export
do_export="${do_export:-Y}"
if [[ "$do_export" =~ ^[Yy]$ ]]; then
  "$PYTHON_BIN" -m e2e_learning.cli.export_onnx \
    experiment="${experiment}" \
    data.dataset_dir="${dataset_dir}" \
    data.input_width="${input_width}" \
    data.input_height="${input_height}" \
    run.name="${run_name}" \
    checkpoint="${checkpoint}" \
    export.output_dir="${output_root}/${run_name}"
fi

read -r -p "Jetsonへ転送しますか？ [y/N]: " do_deploy
if [[ "$do_deploy" =~ ^[Yy]$ ]]; then
  "${SCRIPT_DIR}/deploy_model.sh" "${output_root}/${run_name}/model.onnx"
fi
