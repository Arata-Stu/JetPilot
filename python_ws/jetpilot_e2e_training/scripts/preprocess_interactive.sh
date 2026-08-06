#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd -- "${PACKAGE_DIR}/../.." && pwd)"
DATASET_ROOT="${JETPILOT_E2E_DATASET_ROOT:-${PACKAGE_DIR}/datasets}"

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

bag_candidates=()
while IFS= read -r path; do
  bag_candidates+=("$path")
done < <(find "${PROJECT_ROOT}" -type f \( -name '*.mcap' -o -name '*.db3' \) -print 2>/dev/null | sort -r)
if ((${#bag_candidates[@]} == 0)); then
  read -r -p "rosbagのpathを入力してください: " bag_path
else
  bag_path="$(choose_from_list "前処理に使うrosbagを選択してください:" "${bag_candidates[@]}")"
fi

read -r -p "dataset名 [e2e_dataset]: " dataset_name
dataset_name="${dataset_name:-e2e_dataset}"
read -r -p "画像topic [/realsense/color/image_raw]: " image_topic
image_topic="${image_topic:-/realsense/color/image_raw}"
read -r -p "control topic [/teleop/control_cmd]: " control_topic
control_topic="${control_topic:-/teleop/control_cmd}"

echo "入力サイズpresetを選択してください:" >&2
select size_preset in "424x240 -> 212x120" "640x480 -> 320x240"; do
  case "$REPLY" in
    1) input_width=212; input_height=120; break ;;
    2) input_width=320; input_height=240; break ;;
    *) echo "番号を選んでください。" >&2 ;;
  esac
done

dataset_dir="${DATASET_ROOT}/${dataset_name}"
"$PYTHON_BIN" -m e2e_learning.cli.preprocess_bag \
  data.bag_path="${bag_path}" \
  data.output_dir="${dataset_dir}" \
  data.dataset_dir="${dataset_dir}" \
  data.image_topic="${image_topic}" \
  data.control_topic="${control_topic}" \
  data.input_width="${input_width}" \
  data.input_height="${input_height}"

echo "datasetを作成しました: ${dataset_dir}"
