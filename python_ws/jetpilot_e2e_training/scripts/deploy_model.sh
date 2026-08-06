#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd -- "${PACKAGE_DIR}/../.." && pwd)"

if [[ -x /opt/env/bin/python ]]; then
  PYTHON_BIN="${PYTHON_BIN:-/opt/env/bin/python}"
else
  PYTHON_BIN="${PYTHON_BIN:-python3}"
fi
export PYTHONPATH="${PACKAGE_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

DEPLOY_PROFILE_CONFIG="${E2E_DEPLOY_PROFILES:-${PACKAGE_DIR}/src/e2e_learning/conf/deploy_profiles.json}"
DEPLOY_PRESET_CONFIG="${E2E_DEPLOY_PRESETS:-${PACKAGE_DIR}/src/e2e_learning/conf/deploy_model_presets.json}"

MODEL_PATH=""
REQUESTED_PROFILE="${E2E_REMOTE_PROFILE:-}"
REQUESTED_PRESET="${E2E_MODEL_PRESET:-}"
REMOTE_USER="${E2E_REMOTE_USER:-}"
REMOTE_HOST="${E2E_REMOTE_HOST:-}"
REMOTE_ROOT="${E2E_REMOTE_MODEL_ROOT:-}"
ASSUME_YES=false
BUILD_ENGINE=false

PROFILE_IDS=()
PROFILE_LABELS=()
PROFILE_DESCRIPTIONS=()
PROFILE_USERS=()
PROFILE_HOSTS=()
PROFILE_ROOTS=()
DEFAULT_PROFILE_INDEX=0

PRESET_IDS=()
PRESET_LABELS=()
PRESET_DESCRIPTIONS=()
PRESET_MODEL_NAMES=()
PRESET_MODEL_KINDS=()
PRESET_MODALITIES=()
PRESET_METADATA_FILENAMES=()
DEFAULT_PRESET_INDEX=0

usage() {
  printf '%s\n' \
    "Usage: scripts/deploy_model.sh [ONNX_PATH] [options]" \
    "" \
    "Options:" \
    "  --onnx PATH          ONNX model to deploy" \
    "  --preset ID          Model preset from deploy_model_presets.json" \
    "  --profile ID         Connection profile from deploy_profiles.json" \
    "  --user USER          Override the SSH user" \
    "  --host HOST          Override the Jetson host" \
    "  --remote-root PATH   Override the remote model root" \
    "  --build-engine       Run trtexec on the Jetson after upload" \
    "  -y, --yes            Deploy without confirmation"
}

die() {
  echo "error: $*" >&2
  exit 1
}

parse_args() {
  while (($# > 0)); do
    case "$1" in
      --onnx) MODEL_PATH="${2:?}"; shift 2 ;;
      --preset) REQUESTED_PRESET="${2:?}"; shift 2 ;;
      --profile) REQUESTED_PROFILE="${2:?}"; shift 2 ;;
      --user) REMOTE_USER="${2:?}"; shift 2 ;;
      --host) REMOTE_HOST="${2:?}"; shift 2 ;;
      --remote-root) REMOTE_ROOT="${2:?}"; shift 2 ;;
      --build-engine) BUILD_ENGINE=true; shift ;;
      -y|--yes) ASSUME_YES=true; shift ;;
      -h|--help) usage; exit 0 ;;
      --*) die "unknown option: $1" ;;
      *) [[ -z "$MODEL_PATH" ]] || die "only one ONNX path may be specified"; MODEL_PATH="$1"; shift ;;
    esac
  done
}

load_profiles() {
  local output
  [[ -f "$DEPLOY_PROFILE_CONFIG" ]] || die "Deploy profile config was not found: $DEPLOY_PROFILE_CONFIG"
  output="$("$PYTHON_BIN" -m e2e_learning.cli.deploy_profiles --config "$DEPLOY_PROFILE_CONFIG")"
  while IFS=$'\t' read -r id label description user host root is_default; do
    [[ -n "$id" ]] || continue
    PROFILE_IDS+=("$id")
    PROFILE_LABELS+=("$label")
    PROFILE_DESCRIPTIONS+=("$description")
    PROFILE_USERS+=("$user")
    PROFILE_HOSTS+=("$host")
    PROFILE_ROOTS+=("$root")
    if [[ "$is_default" == "1" ]]; then
      DEFAULT_PROFILE_INDEX=$((${#PROFILE_IDS[@]} - 1))
    fi
  done <<< "$output"
  return 0
}

load_presets() {
  local output
  [[ -f "$DEPLOY_PRESET_CONFIG" ]] || die "Deploy preset config was not found: $DEPLOY_PRESET_CONFIG"
  output="$("$PYTHON_BIN" -m e2e_learning.cli.deploy_presets --config "$DEPLOY_PRESET_CONFIG")"
  while IFS=$'\t' read -r id label description model_name model_kind modality metadata_filename is_default; do
    [[ -n "$id" ]] || continue
    PRESET_IDS+=("$id")
    PRESET_LABELS+=("$label")
    PRESET_DESCRIPTIONS+=("$description")
    PRESET_MODEL_NAMES+=("$model_name")
    PRESET_MODEL_KINDS+=("$model_kind")
    PRESET_MODALITIES+=("$modality")
    PRESET_METADATA_FILENAMES+=("$metadata_filename")
    if [[ "$is_default" == "1" ]]; then
      DEFAULT_PRESET_INDEX=$((${#PRESET_IDS[@]} - 1))
    fi
  done <<< "$output"
  return 0
}

find_profile_index() {
  local requested="$1"
  local index
  for index in "${!PROFILE_IDS[@]}"; do
    [[ "${PROFILE_IDS[$index]}" == "$requested" ]] && {
      printf '%s\n' "$index"
      return
    }
  done
  return 1
}

find_preset_index() {
  local requested="$1"
  local index
  for index in "${!PRESET_IDS[@]}"; do
    [[ "${PRESET_IDS[$index]}" == "$requested" ]] && {
      printf '%s\n' "$index"
      return
    }
  done
  return 1
}

select_profile_index() {
  local index="$DEFAULT_PROFILE_INDEX"
  local choice
  local destination
  if [[ -n "$REQUESTED_PROFILE" ]]; then
    find_profile_index "$REQUESTED_PROFILE"
    return
  fi
  if [[ -t 0 && "$ASSUME_YES" == false ]]; then
    echo "接続profileを選択してください:" >&2
    for choice in "${!PROFILE_IDS[@]}"; do
      if [[ "${PROFILE_HOSTS[$choice]}" == "__manual__" ]]; then
        destination="IPを手動入力"
      else
        destination="${PROFILE_USERS[$choice]}@${PROFILE_HOSTS[$choice]}"
      fi
      printf '  %d) %s (%s) - %s\n' "$((choice + 1))" "${PROFILE_LABELS[$choice]}" "${PROFILE_IDS[$choice]}" "$destination" >&2
    done
    read -r -p "番号 [$((index + 1))]: " choice
    choice="${choice:-$((index + 1))}"
    [[ "$choice" =~ ^[0-9]+$ ]] && ((choice >= 1 && choice <= ${#PROFILE_IDS[@]})) || die "Invalid selection: $choice"
    index=$((choice - 1))
  fi
  printf '%s\n' "$index"
}

select_preset_index() {
  local index="$DEFAULT_PRESET_INDEX"
  local choice
  if [[ -n "$REQUESTED_PRESET" ]]; then
    find_preset_index "$REQUESTED_PRESET"
    return
  fi
  if [[ -t 0 && "$ASSUME_YES" == false ]]; then
    echo "model presetを選択してください:" >&2
    for choice in "${!PRESET_IDS[@]}"; do
      printf '  %d) %s (%s)\n' "$((choice + 1))" "${PRESET_LABELS[$choice]}" "${PRESET_IDS[$choice]}" >&2
    done
    read -r -p "番号 [$((index + 1))]: " choice
    choice="${choice:-$((index + 1))}"
    [[ "$choice" =~ ^[0-9]+$ ]] && ((choice >= 1 && choice <= ${#PRESET_IDS[@]})) || die "Invalid selection: $choice"
    index=$((choice - 1))
  fi
  printf '%s\n' "$index"
}

prompt_remote_host() {
  local host
  while true; do
    read -r -p "JetsonのIPアドレスまたはhostnameを入力: " host
    if [[ "$host" =~ ^[A-Za-z0-9._:-]+$ ]]; then
      printf '%s\n' "$host"
      return
    fi
    echo "有効なIPアドレスまたはhostnameを入力してください。" >&2
  done
}

select_model() {
  local candidates=()
  local path
  local choice
  if [[ -n "$MODEL_PATH" ]]; then
    [[ -f "$MODEL_PATH" ]] || die "ONNX model was not found: $MODEL_PATH"
    printf '%s\n' "$(cd -- "$(dirname -- "$MODEL_PATH")" && pwd)/$(basename -- "$MODEL_PATH")"
    return
  fi
  while IFS= read -r path; do
    candidates+=("$path")
  done < <(find "${PROJECT_ROOT}/outputs" "${PROJECT_ROOT}/python_ws" -type f -name '*.onnx' -print 2>/dev/null | sort -r)
  ((${#candidates[@]} > 0)) || die "No ONNX model was found"
  if [[ ! -t 0 || "$ASSUME_YES" == true ]]; then
    printf '%s\n' "${candidates[0]}"
    return
  fi
  for choice in "${!candidates[@]}"; do
    printf '  %d) %s\n' "$((choice + 1))" "${candidates[$choice]#"$PROJECT_ROOT"/}" >&2
  done
  read -r -p "ONNX番号 [1]: " choice
  choice="${choice:-1}"
  [[ "$choice" =~ ^[0-9]+$ ]] && ((choice >= 1 && choice <= ${#candidates[@]})) || die "Invalid model selection: $choice"
  printf '%s\n' "${candidates[$((choice - 1))]}"
}

main() {
  parse_args "$@"
  load_profiles
  load_presets
  local profile_index preset_index model_path metadata_path model_name remote_model_dir remote_target checksum confirm
  profile_index="$(select_profile_index)"
  if [[ -z "$REMOTE_HOST" && "${PROFILE_IDS[$profile_index]}" == "manual" ]]; then
    [[ -t 0 ]] || die "manual profile requires --host when running non-interactively"
    REMOTE_HOST="$(prompt_remote_host)"
  fi
  preset_index="$(select_preset_index)"
  model_path="$(select_model)"
  metadata_path="$(dirname -- "$model_path")/${PRESET_METADATA_FILENAMES[$preset_index]}"
  model_name="${PRESET_MODEL_NAMES[$preset_index]}"

  REMOTE_USER="${REMOTE_USER:-${PROFILE_USERS[$profile_index]}}"
  REMOTE_HOST="${REMOTE_HOST:-${PROFILE_HOSTS[$profile_index]}}"
  REMOTE_ROOT="${REMOTE_ROOT:-${PROFILE_ROOTS[$profile_index]}}"
  remote_model_dir="${REMOTE_ROOT%/}/${model_name}"
  remote_target="${REMOTE_USER}@${REMOTE_HOST}"
  checksum="$(shasum -a 256 "$model_path" | awk '{print $1}')"

  echo "ONNX       : ${model_path}"
  echo "metadata   : ${metadata_path}$([[ -f "$metadata_path" ]] || printf ' (not found)')"
  echo "destination: ${remote_target}:${remote_model_dir}"
  echo "sha256     : ${checksum}"
  echo "build TRT  : ${BUILD_ENGINE}"

  if [[ "$ASSUME_YES" == false ]]; then
    read -r -p "転送しますか？ [y/N]: " confirm
    [[ "$confirm" =~ ^[Yy]$ ]] || exit 0
  fi

  ssh "$remote_target" "mkdir -p -- '$remote_model_dir'"
  scp "$model_path" "${remote_target}:${remote_model_dir}/model.onnx.uploading"
  if [[ -f "$metadata_path" ]]; then
    scp "$metadata_path" "${remote_target}:${remote_model_dir}/metadata.json.uploading"
    ssh "$remote_target" "mv -- '$remote_model_dir/model.onnx.uploading' '$remote_model_dir/model.onnx'; mv -- '$remote_model_dir/metadata.json.uploading' '$remote_model_dir/metadata.json'; echo '$checksum  $remote_model_dir/model.onnx' > '$remote_model_dir/model.onnx.sha256'; rm -f -- '$remote_model_dir/model.plan'; ln -sfn -- '$model_name' '${REMOTE_ROOT%/}/latest'"
  else
    ssh "$remote_target" "mv -- '$remote_model_dir/model.onnx.uploading' '$remote_model_dir/model.onnx'; echo '$checksum  $remote_model_dir/model.onnx' > '$remote_model_dir/model.onnx.sha256'; rm -f -- '$remote_model_dir/model.plan' '$remote_model_dir/metadata.json'; ln -sfn -- '$model_name' '${REMOTE_ROOT%/}/latest'"
  fi

  if [[ "$BUILD_ENGINE" == true ]]; then
    ssh "$remote_target" "/usr/src/tensorrt/bin/trtexec --onnx='$remote_model_dir/model.onnx' --saveEngine='$remote_model_dir/model.plan' --fp16 --dumpBindings > '$remote_model_dir/build_engine.log' 2>&1"
  fi
  echo "転送が完了しました: ${remote_target}:${remote_model_dir}"
}

main "$@"
