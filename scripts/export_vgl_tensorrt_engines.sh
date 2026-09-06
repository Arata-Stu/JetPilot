#!/usr/bin/env bash
set -euo pipefail

ROS2_WS="${ROS2_WS:-/workspaces/ros2_ws}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_WIDTH=424
IMAGE_HEIGHT=240
NATIVE_PROFILE=false
SIZE_SPECIFIED=false
ASSUME_YES=false

die() {
  echo "error: $*" >&2
  exit 1
}

prompt_yes_no() {
  local prompt="$1"
  local answer

  while true; do
    read -r -p "${prompt} [y/N]: " answer
    case "$answer" in
      y|Y|yes|YES) return 0 ;;
      n|N|no|NO|"") return 1 ;;
      *) echo "Please answer y or n." >&2 ;;
    esac
  done
}

ensure_workspace_overlay() {
  local setup_file="${ROS2_WS}/install/setup.bash"

  if ! command -v ros2 >/dev/null 2>&1; then
    [[ -f "$setup_file" ]] \
      || die "ros2 command was not found and workspace setup is unavailable: $setup_file"
    # shellcheck disable=SC1090
    set +u
    source "$setup_file"
    set -u
  elif ! ros2 pkg prefix isaac_ros_visual_mapping >/dev/null 2>&1 \
    && [[ -f "$setup_file" ]]; then
    echo "Sourcing workspace overlay: $setup_file"
    # shellcheck disable=SC1090
    set +u
    source "$setup_file"
    set -u
  fi

  command -v ros2 >/dev/null 2>&1 || die "ros2 command was not found"
  ros2 pkg prefix isaac_ros_visual_mapping >/dev/null 2>&1 \
    || die "isaac_ros_visual_mapping is unavailable after sourcing ${setup_file}"
}

export_tensorrt_engines() {
  local visual_mapping_prefix
  local visual_mapping_share
  local lightglue_exporter
  local extractor_exporter
  local extractor_config

  visual_mapping_prefix="$(ros2 pkg prefix isaac_ros_visual_mapping)"
  visual_mapping_share="$(ros2 pkg prefix --share isaac_ros_visual_mapping)"
  lightglue_exporter="${visual_mapping_prefix}/bin/visual_mapping/export_lightglue_engine"
  extractor_exporter="${visual_mapping_prefix}/bin/visual_mapping/export_extractor_engine"

  [[ -x "$lightglue_exporter" ]] \
    || die "LightGlue exporter was not found: $lightglue_exporter"
  [[ -x "$extractor_exporter" ]] \
    || die "extractor exporter was not found: $extractor_exporter"

  mkdir -p "$OUTPUT_MODEL_DIR"
  extractor_config="${visual_mapping_share}/configs/isaac/keypoint_creation_config.pb.txt"
  if [[ "$NATIVE_PROFILE" != true ]]; then
    command -v python3 >/dev/null 2>&1 || die "python3 is required (standard library only)"
    python3 "${SCRIPT_DIR}/configure_vgl_extractor.py" \
      "$extractor_config" "$OUTPUT_MODEL_DIR/keypoint_creation_config.pb.txt" \
      --width "$IMAGE_WIDTH" --height "$IMAGE_HEIGHT"
    extractor_config="$OUTPUT_MODEL_DIR/keypoint_creation_config.pb.txt"
  fi

  "$lightglue_exporter" \
    --worker_config_file \
    "${visual_mapping_share}/configs/isaac/matching_task_worker_config.pb.txt" \
    --model_dir \
    "${visual_mapping_share}/models" \
    --output_model_dir "$OUTPUT_MODEL_DIR"

  "$extractor_exporter" \
    --configure_file \
    "$extractor_config" \
    --model_dir \
    "${visual_mapping_share}/models" \
    --output_model_dir "$OUTPUT_MODEL_DIR"
}

main() {
  while (( $# > 0 )); do
  case "$1" in
    -y|--yes) ASSUME_YES=true ;;
    --width|--height|--output-model-dir)
      (( $# >= 2 )) || die "$1 requires a value"
      case "$1" in
        --width) IMAGE_WIDTH="$2"; SIZE_SPECIFIED=true ;;
        --height) IMAGE_HEIGHT="$2"; SIZE_SPECIFIED=true ;;
        --output-model-dir) OUTPUT_MODEL_DIR="$2" ;;
      esac
      shift ;;
    --native-profile) NATIVE_PROFILE=true ;;
    -h|--help)
      echo "Usage: $(basename "$0") [--yes] [--width 424] [--height 240] [--output-model-dir DIR] [--native-profile]"
      echo "Default: fixed ALIKED input size, written to visual_global_localization_WIDTHxHEIGHT."
      echo "--native-profile preserves the installed exporter configuration for mapping."
      exit 0
      ;;
    *)
      die "unknown option: ${1}"
      ;;
  esac
  shift
  done
  [[ "$IMAGE_WIDTH" =~ ^[1-9][0-9]*$ && "$IMAGE_HEIGHT" =~ ^[1-9][0-9]*$ ]] \
    || die "width and height must be positive integers"
  [[ "$NATIVE_PROFILE" != true || "$SIZE_SPECIFIED" != true ]] \
    || die "--native-profile cannot be combined with --width or --height"
  if [[ "$NATIVE_PROFILE" == true ]]; then
    OUTPUT_MODEL_DIR="${OUTPUT_MODEL_DIR:-${ROS2_WS}/isaac_ros_assets/models/visual_global_localization}"
  else
    OUTPUT_MODEL_DIR="${OUTPUT_MODEL_DIR:-${ROS2_WS}/isaac_ros_assets/models/visual_global_localization_${IMAGE_WIDTH}x${IMAGE_HEIGHT}}"
  fi

  ensure_workspace_overlay

  echo
  echo "VGL TensorRT engine export"
  echo "Output directory : $OUTPUT_MODEL_DIR"
  if [[ "$NATIVE_PROFILE" == true ]]; then
    echo "ALIKED profile   : installed defaults"
  else
    echo "ALIKED input     : ${IMAGE_WIDTH}x${IMAGE_HEIGHT} (width x height, fixed)"
    echo "This does not resize camera images; the ALIKED input must match this size."
    if compgen -G "$OUTPUT_MODEL_DIR/aliked_lightglue/*.engine" >/dev/null; then
      die "engines already exist in $OUTPUT_MODEL_DIR; choose a new output directory"
    fi
  fi
  echo
  echo "The engines are specific to this GPU and software environment."
  echo "They are shared by all maps and normally need to be generated only once."
  echo

  if [[ "$ASSUME_YES" != "true" ]] \
    && ! prompt_yes_no "Export LightGlue and ALIKED TensorRT engines?"; then
    echo "Canceled."
    exit 0
  fi

  export_tensorrt_engines

  echo
  echo "VGL TensorRT engine export completed: $OUTPUT_MODEL_DIR"
}

main "$@"
