#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 2 ]]; then
  echo "usage: $0 KALIBR_DATASET_DIR OUTPUT_DIR" >&2
  exit 2
fi

input_dir="$1"
output_dir="$2"
image_name="${KALIBR_IMAGE:-jetpilot-kalibr:ros1-noetic}"

if [[ ! -f "${input_dir}/job.yaml" ]]; then
  echo "Kalibr dataset does not contain job.yaml: ${input_dir}" >&2
  exit 2
fi

if [[ -d "${output_dir}" ]] && [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "output directory must be empty: ${output_dir}" >&2
  exit 2
fi

mkdir -p "${output_dir}"
input_dir="$(cd "${input_dir}" && pwd -P)"
output_dir="$(cd "${output_dir}" && pwd -P)"

docker run --rm --init \
  --shm-size=2g \
  --user "$(id -u):$(id -g)" \
  --volume "${input_dir}:/input:ro" \
  --volume "${output_dir}:/output" \
  "${image_name}"
