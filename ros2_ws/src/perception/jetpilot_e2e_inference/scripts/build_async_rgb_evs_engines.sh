#!/usr/bin/env bash
set -euo pipefail

MODEL_ROOT="${1:-}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"

if [[ -z "$MODEL_ROOT" ]]; then
  echo "Usage: build_async_rgb_evs_engines.sh MODEL_ROOT" >&2
  exit 1
fi
for filename in rgb_encoder.onnx event_updater.onnx; do
  if [[ ! -f "${MODEL_ROOT}/${filename}" ]]; then
    echo "Missing ${MODEL_ROOT}/${filename}" >&2
    exit 1
  fi
done
if [[ ! -x "$TRTEXEC" ]]; then
  echo "trtexec was not found: $TRTEXEC" >&2
  exit 1
fi

build_engine() {
  local stem="$1"
  local building="${MODEL_ROOT}/${stem}.plan.building"
  "$TRTEXEC" \
    "--onnx=${MODEL_ROOT}/${stem}.onnx" \
    "--saveEngine=${building}" \
    --fp16 \
    2>&1 | tee "${MODEL_ROOT}/build_${stem}.log"
  mv -- "$building" "${MODEL_ROOT}/${stem}.plan"
}

build_engine rgb_encoder
build_engine event_updater
echo "Built ${MODEL_ROOT}/rgb_encoder.plan and ${MODEL_ROOT}/event_updater.plan"
