#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
image_name="${KALIBR_IMAGE:-jetpilot-kalibr:ros1-noetic}"
kalibr_ref="${KALIBR_REF:-master}"
build_jobs="${KALIBR_BUILD_JOBS:-4}"

docker build \
  --build-arg "KALIBR_REF=${kalibr_ref}" \
  --build-arg "KALIBR_BUILD_JOBS=${build_jobs}" \
  --tag "${image_name}" \
  "${script_dir}"
