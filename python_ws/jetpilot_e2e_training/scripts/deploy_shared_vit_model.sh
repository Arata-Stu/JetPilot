#!/usr/bin/env bash
set -euo pipefail

ONNX=""
USER_NAME=""
HOST=""
REMOTE_ROOT=""
MODEL_NAME=""
BUILD_ENGINE=false

while (($# > 0)); do
  case "$1" in
    --onnx) ONNX="${2:?}"; shift 2 ;;
    --user) USER_NAME="${2:?}"; shift 2 ;;
    --host) HOST="${2:?}"; shift 2 ;;
    --remote-root) REMOTE_ROOT="${2:?}"; shift 2 ;;
    --name) MODEL_NAME="${2:?}"; shift 2 ;;
    --build-engine) BUILD_ENGINE=true; shift ;;
    *) echo "error: unknown argument: $1" >&2; exit 2 ;;
  esac
done

[[ -f "$ONNX" ]] || { echo "error: ONNX model was not found: $ONNX" >&2; exit 2; }
[[ -n "$USER_NAME" && -n "$HOST" && -n "$REMOTE_ROOT" && -n "$MODEL_NAME" ]] || {
  echo "error: user, host, remote-root and name are required" >&2
  exit 2
}

METADATA="$(dirname -- "$ONNX")/metadata.json"
TARGET="${USER_NAME}@${HOST}"
REMOTE_DIR="${REMOTE_ROOT%/}/${MODEL_NAME}"
CHECKSUM="$(shasum -a 256 "$ONNX" | awk '{print $1}')"

ssh "$TARGET" "mkdir -p -- '$REMOTE_DIR'"
scp "$ONNX" "${TARGET}:${REMOTE_DIR}/model.onnx.uploading"
if [[ -f "$METADATA" ]]; then
  scp "$METADATA" "${TARGET}:${REMOTE_DIR}/metadata.json.uploading"
  ssh "$TARGET" "mv -- '$REMOTE_DIR/model.onnx.uploading' '$REMOTE_DIR/model.onnx'; mv -- '$REMOTE_DIR/metadata.json.uploading' '$REMOTE_DIR/metadata.json'"
else
  ssh "$TARGET" "mv -- '$REMOTE_DIR/model.onnx.uploading' '$REMOTE_DIR/model.onnx'"
fi
ssh "$TARGET" "echo '$CHECKSUM  $REMOTE_DIR/model.onnx' > '$REMOTE_DIR/model.onnx.sha256'; rm -f -- '$REMOTE_DIR/model.plan'"
if [[ "$BUILD_ENGINE" == true ]]; then
  ssh "$TARGET" "set -e; /usr/src/tensorrt/bin/trtexec --onnx='$REMOTE_DIR/model.onnx' --saveEngine='$REMOTE_DIR/model.plan.building' --fp16 > '$REMOTE_DIR/build_engine.log' 2>&1; mv -- '$REMOTE_DIR/model.plan.building' '$REMOTE_DIR/model.plan'"
fi
ssh "$TARGET" "ln -sfn -- '$MODEL_NAME' '${REMOTE_ROOT%/}/latest'"
echo "Shared ViT deployment completed: ${TARGET}:${REMOTE_DIR}"
