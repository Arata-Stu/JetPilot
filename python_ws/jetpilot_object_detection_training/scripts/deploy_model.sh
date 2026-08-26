#!/usr/bin/env bash
set -euo pipefail

MODEL_PATH=""
METADATA_PATH=""
MODEL_ROOT="${JETPILOT_YOLO_MODEL_ROOT:-/workspaces/ros2_ws/models/yolov8}"
MODEL_NAME="latest"
BUILD_ENGINE=false
REMOTE_USER=""
REMOTE_HOST=""
REMOTE_ROOT=""
ASSUME_YES=false

usage() {
  printf '%s\n' \
    "Usage: scripts/deploy_model.sh MODEL_ONNX [options]" \
    "" \
    "Install locally, or transfer a training export to a Jetson over SSH." \
    "TensorRT engines are always built on the target where they will run." \
    "" \
    "Options:" \
    "  --metadata PATH    metadata.json to install (auto-detected beside ONNX)" \
    "  --model-root PATH  deployment root (default: ${MODEL_ROOT})" \
    "  --name NAME        model directory name (default: latest)" \
    "  --user USER        SSH user for remote deployment" \
    "  --host HOST        Jetson IP address or hostname" \
    "  --remote-root PATH remote deployment root" \
    "  --build-engine     build model.plan with target-side trtexec" \
    "  -y, --yes          deploy remotely without confirmation" \
    "  -h, --help"
}

die() {
  echo "error: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --onnx) MODEL_PATH="${2:?}"; shift 2 ;;
    --metadata) METADATA_PATH="${2:?}"; shift 2 ;;
    --model-root) MODEL_ROOT="${2:?}"; shift 2 ;;
    --name) MODEL_NAME="${2:?}"; shift 2 ;;
    --user) REMOTE_USER="${2:?}"; shift 2 ;;
    --host) REMOTE_HOST="${2:?}"; shift 2 ;;
    --remote-root) REMOTE_ROOT="${2:?}"; shift 2 ;;
    --build-engine) BUILD_ENGINE=true; shift ;;
    -y|--yes) ASSUME_YES=true; shift ;;
    -h|--help) usage; exit 0 ;;
    --*) die "unknown option: $1" ;;
    *)
      [[ -z "$MODEL_PATH" ]] || die "only one ONNX path may be specified"
      MODEL_PATH="$1"
      shift
      ;;
  esac
done

[[ -n "$MODEL_PATH" ]] || { usage >&2; exit 2; }
[[ -f "$MODEL_PATH" ]] || die "ONNX model was not found: $MODEL_PATH"
[[ "$MODEL_NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die "invalid model name: $MODEL_NAME"
[[ "$MODEL_NAME" != "." && "$MODEL_NAME" != ".." ]] || die "invalid model name: $MODEL_NAME"

if [[ -z "$METADATA_PATH" ]]; then
  candidate_metadata="$(dirname -- "$MODEL_PATH")/metadata.json"
  if [[ -f "$candidate_metadata" ]]; then
    METADATA_PATH="$candidate_metadata"
  fi
fi
if [[ -n "$METADATA_PATH" && ! -f "$METADATA_PATH" ]]; then
  die "metadata file was not found: $METADATA_PATH"
fi

checksum_file() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    die "sha256sum or shasum is required"
  fi
}

deploy_remote() {
  [[ -n "$REMOTE_USER" ]] || die "--user is required with --host"
  [[ "$REMOTE_USER" =~ ^[A-Za-z0-9._-]+$ ]] || die "SSH user contains unsupported characters"
  [[ "$REMOTE_HOST" =~ ^[A-Za-z0-9._:-]+$ ]] || die "SSH host contains unsupported characters"
  [[ -n "$REMOTE_ROOT" ]] || REMOTE_ROOT="$MODEL_ROOT"
  [[ "$REMOTE_ROOT" =~ ^/[A-Za-z0-9._/-]+$ ]] || die "remote root must be an absolute portable path"
  [[ "/$REMOTE_ROOT/" != *"/../"* ]] || die "remote root must not contain '..'"

  local remote_target remote_model_dir staging_dir backup_dir checksum confirm
  local ssh_options=(-o BatchMode=yes -o ConnectTimeout=10)
  remote_target="${REMOTE_USER}@${REMOTE_HOST}"
  remote_model_dir="${REMOTE_ROOT%/}/${MODEL_NAME}"
  staging_dir="${remote_model_dir}.uploading.$$"
  backup_dir="${remote_model_dir}.previous"
  checksum="$(checksum_file "$MODEL_PATH")"

  printf 'ONNX       : %s\n' "$MODEL_PATH"
  printf 'metadata   : %s\n' "${METADATA_PATH:-not provided}"
  printf 'destination: %s:%s\n' "$remote_target" "$remote_model_dir"
  printf 'sha256     : %s\n' "$checksum"
  printf 'build TRT  : %s\n' "$BUILD_ENGINE"
  if [[ "$ASSUME_YES" == false ]]; then
    read -r -p "Transfer this model? [y/N]: " confirm
    [[ "$confirm" =~ ^[Yy]$ ]] || exit 0
  fi

  ssh "${ssh_options[@]}" "$remote_target" "set -e; rm -rf -- '$staging_dir'; mkdir -p -- '$staging_dir'"
  scp "${ssh_options[@]}" "$MODEL_PATH" "${remote_target}:${staging_dir}/model.onnx"
  if [[ -n "$METADATA_PATH" ]]; then
    scp "${ssh_options[@]}" "$METADATA_PATH" "${remote_target}:${staging_dir}/metadata.json"
  fi
  ssh "${ssh_options[@]}" "$remote_target" "printf '%s  %s\n' '$checksum' '$remote_model_dir/model.onnx' > '$staging_dir/model.onnx.sha256'"

  if [[ "$BUILD_ENGINE" == true ]]; then
    ssh "${ssh_options[@]}" "$remote_target" "set -e; if command -v trtexec >/dev/null 2>&1; then trtexec_bin=\$(command -v trtexec); elif [ -x /usr/src/tensorrt/bin/trtexec ]; then trtexec_bin=/usr/src/tensorrt/bin/trtexec; else echo 'trtexec was not found on the Jetson' >&2; exit 1; fi; \"\$trtexec_bin\" --onnx='$staging_dir/model.onnx' --saveEngine='$staging_dir/model.plan.building' --fp16 > '$staging_dir/build_engine.log' 2>&1; mv -- '$staging_dir/model.plan.building' '$staging_dir/model.plan'"
  fi

  # Build in a staging directory first.  A failed TensorRT build therefore
  # leaves the previously deployed model and engine untouched.
  ssh "${ssh_options[@]}" "$remote_target" "set -e; [ ! -L '$remote_model_dir' ]; rm -rf -- '$backup_dir'; if [ -e '$remote_model_dir' ]; then mv -- '$remote_model_dir' '$backup_dir'; fi; if mv -- '$staging_dir' '$remote_model_dir'; then rm -rf -- '$backup_dir'; else if [ -e '$backup_dir' ]; then mv -- '$backup_dir' '$remote_model_dir'; fi; exit 1; fi"
  printf 'deployed: %s:%s/model.onnx\n' "$remote_target" "$remote_model_dir"
  if [[ "$BUILD_ENGINE" == true ]]; then
    printf 'engine:   %s:%s/model.plan\n' "$remote_target" "$remote_model_dir"
  fi
}

if [[ -n "$REMOTE_HOST" ]]; then
  deploy_remote
  exit 0
fi
[[ -z "$REMOTE_USER" && -z "$REMOTE_ROOT" ]] || die "--user/--remote-root require --host"

mkdir -p -- "$MODEL_ROOT"
TARGET_DIR="${MODEL_ROOT%/}/${MODEL_NAME}"
[[ ! -L "$TARGET_DIR" ]] || die "model directory must not be a symlink: $TARGET_DIR"
mkdir -p -- "$TARGET_DIR"

cp -- "$MODEL_PATH" "$TARGET_DIR/model.onnx.uploading"
mv -- "$TARGET_DIR/model.onnx.uploading" "$TARGET_DIR/model.onnx"
if [[ -n "$METADATA_PATH" ]]; then
  cp -- "$METADATA_PATH" "$TARGET_DIR/metadata.json.uploading"
  mv -- "$TARGET_DIR/metadata.json.uploading" "$TARGET_DIR/metadata.json"
else
  rm -f -- "$TARGET_DIR/metadata.json"
fi

# A TensorRT plan is specific to both the ONNX contents and the target runtime.
rm -f -- "$TARGET_DIR/model.plan" "$TARGET_DIR/model.plan.building"
printf '%s  %s\n' "$(checksum_file "$TARGET_DIR/model.onnx")" "$TARGET_DIR/model.onnx" > "$TARGET_DIR/model.onnx.sha256"

if [[ "$BUILD_ENGINE" == true ]]; then
  if command -v trtexec >/dev/null 2>&1; then
    TRTEXEC="$(command -v trtexec)"
  elif [[ -x /usr/src/tensorrt/bin/trtexec ]]; then
    TRTEXEC="/usr/src/tensorrt/bin/trtexec"
  else
    die "trtexec was not found; build the engine inside the target Jetson container"
  fi
  "$TRTEXEC" \
    --onnx="$TARGET_DIR/model.onnx" \
    --saveEngine="$TARGET_DIR/model.plan.building" \
    --fp16 \
    > "$TARGET_DIR/build_engine.log" 2>&1
  mv -- "$TARGET_DIR/model.plan.building" "$TARGET_DIR/model.plan"
fi

printf 'deployed: %s\n' "$TARGET_DIR/model.onnx"
if [[ "$BUILD_ENGINE" == true ]]; then
  printf 'engine:   %s\n' "$TARGET_DIR/model.plan"
else
  printf '%s\n' "engine:   not built (run this command with --build-engine on the target Jetson)"
fi
