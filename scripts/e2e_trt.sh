#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${JETPILOT_PROJECT_ROOT:-$(dirname -- "$SCRIPT_DIR")}"
ROS2_WS="${ROS2_WS:-${PROJECT_ROOT}/ros2_ws}"
MODEL_PATH="${E2E_MODEL_ROOT:-}"
MODEL_BASE="${E2E_MODEL_BASE:-${ROS2_WS}/models/e2e}"
BUILDER="${PROJECT_ROOT}/ros2_ws/src/perception/jetpilot_e2e_inference/scripts/build_tensorrt_engine.sh"
ASYNC_BUILDER="${PROJECT_ROOT}/ros2_ws/src/perception/jetpilot_e2e_inference/scripts/build_async_rgb_evs_engines.sh"
MODEL_SPECIFIED=false

usage() {
  cat <<'EOF'
Usage: scripts/e2e_trt.sh [MODEL_DIR | MODEL.onnx] [--fp16 | --fp32]

Build a selected E2E TensorRT engine inside the Jetson runtime container.
With no MODEL_DIR, interactively select a deployed model under E2E_MODEL_BASE.
fzf is used when available; a numbered selector is the fallback.
Precision defaults to FP16. Async RGB-EVS models build both split engines.
After building, print inference timing statistics from the trtexec build logs.

Examples:
  scripts/e2e_trt.sh
  scripts/e2e_trt.sh /workspaces/ros2_ws/models/e2e/pilotnet-control_0911-1200
  scripts/e2e_trt.sh /path/to/model.onnx --fp32

Environment: ROS2_WS, E2E_MODEL_BASE, E2E_MODEL_ROOT, TRTEXEC, E2E_TRT_FP16
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

select_model() {
  [[ -d "$MODEL_BASE" ]] || die "E2E model root was not found: $MODEL_BASE"

  local candidates=()
  local labels=()
  local directory relative kind selected choice index
  while IFS= read -r directory; do
    [[ -n "$directory" ]] || continue
    relative="${directory#"${MODEL_BASE%/}/"}"
    if [[ -f "$directory/rgb_encoder.onnx" && -f "$directory/event_updater.onnx" ]]; then
      kind="RGB+EVS async (2 engines)"
    else
      kind="single model"
    fi
    candidates+=("$directory")
    labels+=("$relative  [$kind]")
  done < <(
    find "$MODEL_BASE" -type f \( -name model.onnx -o -name rgb_encoder.onnx \) \
      -exec dirname -- {} \; | sort -u
  )
  ((${#candidates[@]} > 0)) || die "No deployed E2E ONNX model was found under $MODEL_BASE"
  [[ -t 0 ]] || die "No model was specified and an interactive terminal is unavailable. Pass MODEL_DIR explicitly."

  if command -v fzf >/dev/null 2>&1; then
    selected="$({
      for index in "${!candidates[@]}"; do
        printf '%s\t%s\n' "${labels[$index]}" "${candidates[$index]}"
      done
    } | fzf --delimiter=$'\t' --with-nth=1 \
      --prompt='E2E TensorRT model > ' \
      --header='engineを生成する配備済みモデルを選択（Escで中止）')" || exit 0
    printf '%s\n' "${selected#*$'\t'}"
    return
  fi

  echo "TensorRT engineを生成するモデルを選択してください:" >&2
  for index in "${!candidates[@]}"; do
    printf '  %d) %s\n' "$((index + 1))" "${labels[$index]}" >&2
  done
  read -r -p "番号 [1]: " choice
  choice="${choice:-1}"
  [[ "$choice" =~ ^[0-9]+$ ]] && ((choice >= 1 && choice <= ${#candidates[@]})) \
    || die "invalid selection: $choice"
  printf '%s\n' "${candidates[$((choice - 1))]}"
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

if [[ -z "$MODEL_PATH" ]]; then
  MODEL_PATH="$(select_model)"
  [[ -n "$MODEL_PATH" ]] || exit 0
fi

if [[ ! -f "$BUILDER" ]]; then
  echo "E2E build script was not found: $BUILDER" >&2
  exit 1
fi

if [[ -d "$MODEL_PATH" || "$MODEL_PATH" != *.onnx ]]; then
  MODEL_PATH="${MODEL_PATH%/}"
  if [[ -f "$MODEL_PATH/rgb_encoder.onnx" && -f "$MODEL_PATH/event_updater.onnx" ]]; then
    [[ -f "$ASYNC_BUILDER" ]] || die "Async RGB-EVS build script was not found: $ASYNC_BUILDER"
    bash "$ASYNC_BUILDER" "$MODEL_PATH"
    exec python3 "${SCRIPT_DIR}/lib/trt_build_summary.py" \
      "$MODEL_PATH/build_rgb_encoder.log" "$MODEL_PATH/build_event_updater.log"
  fi
  MODEL_PATH="$MODEL_PATH/model.onnx"
fi
ENGINE_PATH="${MODEL_PATH%.onnx}.plan"

bash "$BUILDER" "$MODEL_PATH" "$ENGINE_PATH"
exec python3 "${SCRIPT_DIR}/lib/trt_build_summary.py" "$(dirname -- "$ENGINE_PATH")/build_engine.log"
