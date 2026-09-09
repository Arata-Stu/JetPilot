#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${JETPILOT_PROJECT_ROOT:-$(dirname -- "$SCRIPT_DIR")}"
ROS2_WS="${ROS2_WS:-${PROJECT_ROOT}/ros2_ws}"
MODEL_PATH="${E2E_MODEL_ROOT:-${ROS2_WS}/models/e2e/latest}"
BUILDER="${PROJECT_ROOT}/ros2_ws/src/perception/jetpilot_e2e_inference/scripts/build_tensorrt_engine.sh"
MODEL_SPECIFIED=false

usage() {
  cat <<'EOF'
Usage: scripts/e2e_trt.sh [MODEL_DIR | MODEL.onnx] [--fp16 | --fp32]

Build an E2E TensorRT engine inside the Jetson runtime container.
Defaults to ros2_ws/models/e2e/latest and FP16.
Writes model.plan and build_engine.log beside the input ONNX file.

Examples:
  scripts/e2e_trt.sh
  scripts/e2e_trt.sh /workspaces/ros2_ws/models/e2e/camera_control
  scripts/e2e_trt.sh /path/to/model.onnx --fp32

Environment: ROS2_WS, E2E_MODEL_ROOT, TRTEXEC, E2E_TRT_FP16
EOF
}

while (($# > 0)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --fp16) export E2E_TRT_FP16=1 ;;
    --fp32) export E2E_TRT_FP16=0 ;;
    -*) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    *)
      if [[ "$MODEL_SPECIFIED" == true ]]; then
        echo "Specify only one model directory or ONNX file." >&2
        exit 1
      fi
      MODEL_PATH="$1"
      MODEL_SPECIFIED=true
      ;;
  esac
  shift
done

if [[ -d "$MODEL_PATH" || "$MODEL_PATH" != *.onnx ]]; then
  MODEL_PATH="${MODEL_PATH%/}/model.onnx"
fi
ENGINE_PATH="$(dirname -- "$MODEL_PATH")/model.plan"

if [[ ! -f "$BUILDER" ]]; then
  echo "E2E build script was not found: $BUILDER" >&2
  exit 1
fi

exec bash "$BUILDER" "$MODEL_PATH" "$ENGINE_PATH"
