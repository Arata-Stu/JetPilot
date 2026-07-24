#!/usr/bin/env bash
set -euo pipefail

ONNX_PATH="${1:-}"
ENGINE_PATH="${2:-}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
FP16="${E2E_TRT_FP16:-1}"

if [[ -z "$ONNX_PATH" || -z "$ENGINE_PATH" ]]; then
  echo "Usage: build_tensorrt_engine.sh MODEL.onnx MODEL.plan" >&2
  exit 1
fi
if [[ ! -f "$ONNX_PATH" ]]; then
  echo "ONNX model was not found: $ONNX_PATH" >&2
  exit 1
fi
if [[ ! -x "$TRTEXEC" ]]; then
  echo "trtexec was not found or is not executable: $TRTEXEC" >&2
  exit 1
fi

mkdir -p -- "$(dirname -- "$ENGINE_PATH")"
args=(
  "$TRTEXEC"
  "--onnx=${ONNX_PATH}"
  "--saveEngine=${ENGINE_PATH}"
  "--dumpBindings"
)
if [[ "$FP16" == "1" || "$FP16" == "true" ]]; then
  args+=("--fp16")
fi

"${args[@]}" 2>&1 | tee "$(dirname -- "$ENGINE_PATH")/build_engine.log"
