#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODEL_ROOT="${E2E_MODEL_ROOT:-/workspaces/ros2_ws/models/e2e}"
PRECISION=""

usage() {
  printf '%s\n' \
    "Usage: deploy_tensorrt_tui.sh [--root MODEL_ROOT] [--fp16|--fp32]" \
    "" \
    "Select an uploaded model.onnx and build model.plan with trtexec." \
    "fzf is used when available; a numbered selector is the fallback."
}

die() {
  echo "error: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --root) MODEL_ROOT="${2:?}"; shift 2 ;;
    --fp16) PRECISION="fp16"; shift ;;
    --fp32) PRECISION="fp32"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done

[[ -d "$MODEL_ROOT" ]] || die "model root was not found: $MODEL_ROOT"

models=()
while IFS= read -r model; do
  models+=("$model")
done < <(find "$MODEL_ROOT" -type f -name model.onnx -print | sort)
((${#models[@]} > 0)) || die "no model.onnx was found under $MODEL_ROOT"

selected=""
if command -v fzf >/dev/null 2>&1; then
  selected="$(printf '%s\n' "${models[@]}" | fzf \
    --prompt='TensorRT model > ' \
    --header='engineを生成するONNXを選択（Escで中止）')" || exit 0
else
  echo "TensorRT engineを生成するモデルを選択してください:" >&2
  for index in "${!models[@]}"; do
    printf '  %d) %s\n' "$((index + 1))" "${models[$index]#"$MODEL_ROOT"/}" >&2
  done
  read -r -p "番号 [1]: " choice
  choice="${choice:-1}"
  [[ "$choice" =~ ^[0-9]+$ ]] && ((choice >= 1 && choice <= ${#models[@]})) || die "invalid selection: $choice"
  selected="${models[$((choice - 1))]}"
fi

if [[ -z "$PRECISION" ]]; then
  if command -v fzf >/dev/null 2>&1; then
    PRECISION="$(printf '%s\n' fp16 fp32 | fzf --prompt='Precision > ' --header='TensorRT precisionを選択')" || exit 0
  else
    read -r -p "precision [fp16/fp32] (default: fp16): " PRECISION
    PRECISION="${PRECISION:-fp16}"
  fi
fi
[[ "$PRECISION" == "fp16" || "$PRECISION" == "fp32" ]] || die "precision must be fp16 or fp32"

engine_path="$(dirname -- "$selected")/model.plan"
echo "ONNX      : $selected"
echo "TensorRT  : $engine_path"
echo "precision : $PRECISION"
read -r -p "trtexecを実行しますか？ [y/N]: " confirm
[[ "$confirm" =~ ^[Yy]$ ]] || exit 0

if [[ "$PRECISION" == "fp16" ]]; then
  E2E_TRT_FP16=1 "$SCRIPT_DIR/build_tensorrt_engine.sh" "$selected" "$engine_path"
else
  E2E_TRT_FP16=0 "$SCRIPT_DIR/build_tensorrt_engine.sh" "$selected" "$engine_path"
fi

model_dir="$(dirname -- "$selected")"
latest_path="${MODEL_ROOT%/}/latest"
if [[ -e "$latest_path" && ! -L "$latest_path" ]]; then
  die "latest exists but is not a symlink: $latest_path"
fi
relative_model_dir="${model_dir#"${MODEL_ROOT%/}"/}"
[[ "$relative_model_dir" != "$model_dir" ]] || die "selected model is outside model root"
ln -sfn -- "$relative_model_dir" "$latest_path"

echo "TensorRT engineを生成しました: $engine_path"
echo "有効モデルを切り替えました: $latest_path -> $relative_model_dir"
