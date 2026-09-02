#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${ROS2_WS:-/workspaces/ros2_ws}"
PROJECT_ROOT="${JETPILOT_PROJECT_ROOT:-$(dirname -- "$SCRIPT_DIR")}"
if [[ -z "${JETPILOT_PROJECT_ROOT:-}" \
      && ! -d "${PROJECT_ROOT}/ros2_ws" \
      && -d "$ROS2_WS" ]]; then
  PROJECT_ROOT="$(dirname -- "$ROS2_WS")"
fi
ROS2_SETUP_FILE="${ROS2_SETUP_FILE:-${ROS2_WS}/install/setup.bash}"
MAP_ROOT="${MAP_ROOT:-/workspaces/map}"
RECORD_ROOT="${RECORD_ROOT:-/workspaces/record}"
LAUNCH_PACKAGE="${JETPILOT_LAUNCH_PACKAGE:-jetpilot_system_launch}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
PROFILE_HELPER="${SCRIPT_DIR}/launch_profiles.py"
PROFILE_ROOT="${BRINGUP_PROFILE_ROOT:-${PROJECT_ROOT}/ros2_ws/src/launch/jetpilot_system_launch/config/bringup_profiles}"

PRESET=''
VEHICLE_BACKEND='none'
SENSOR_KIT_PROFILE=''
MAP_DIR="${BRINGUP_MAP_DIR:-}"
ROSBAG="${BRINGUP_ROSBAG:-}"
RACELINE_CSV="${BRINGUP_RACELINE_CSV:-}"
CUSTOM_LINE_CSV="${BRINGUP_CUSTOM_LINE_CSV:-}"
CUSTOM_LINE_ID="${BRINGUP_CUSTOM_LINE_ID:-}"
CUSTOM_LINE_NAME="${BRINGUP_CUSTOM_LINE_NAME:-}"
CUSTOM_LINE_CLOSED=true
CUSTOM_LINE_ID_EXPLICIT=false
CUSTOM_LINE_NAME_EXPLICIT=false
CUSTOM_LINE_CLOSED_EXPLICIT=false
[[ -n "$CUSTOM_LINE_ID" ]] && CUSTOM_LINE_ID_EXPLICIT=true
[[ -n "$CUSTOM_LINE_NAME" ]] && CUSTOM_LINE_NAME_EXPLICIT=true
RVIZ_CONFIG="${BRINGUP_RVIZ_CONFIG:-}"
REPLAY_RATE='1.0'
DRY_RUN=false
ASSUME_YES=false
INTERACTIVE=false
CLI_BAG_MANAGER=''
CLI_SENSOR_KIT=''
CLI_LOCALIZATION_INIT=''
CLI_VSLAM_MODE=''
LOCALIZATION_INIT_MODE='pose-hint'
REQUIRES_MAP=false
REQUIRES_ROSBAG=false
REQUIRES_RACELINE=false
REQUIRES_CUSTOM_LINE=false
REQUIRES_VEHICLE=false
ARG_NAMES=()
ARG_VALUES=()
EXTRA_LAUNCH_ARGS=()
CUSTOM_COMPONENTS=''
SENSOR_KIT_RTP_TOPICS=()
FOXGLOVE_DEFAULT_TOPIC_WHITELIST="['^/tf$', '^/tf_static$', '^/clock$', '^/(.*/)?diagnostics$', '^/localization/(pose_hint_required|pose_hint_state|current_section|current_section_marker)$', '^/visual_slam/tracking/odometry$', '^/visual_localization/pose$', '^/hd_map/(lane_markers|section_markers|primary_centerline_path)$']"
FOXGLOVE_DEFAULT_CLIENT_TOPIC_WHITELIST="['^/initialpose$']"

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

is_true() {
  case "${1:-}" in
    1|true|TRUE|True|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

is_jetson_platform() {
  if [[ -n "${ISAAC_ROS_PLATFORM:-}" ]]; then
    [[ "$ISAAC_ROS_PLATFORM" == 'arm64-jetpack' ]]
    return
  fi

  case "$(uname -m)" in
    aarch64|arm64) [[ -f /etc/nv_tegra_release ]] ;;
    *) return 1 ;;
  esac
}

is_valid_port() {
  local port="${1:-}"

  [[ "$port" =~ ^[0-9]{1,5}$ ]] && ((10#$port >= 1 && 10#$port <= 65535))
}

has_cuvslam_database() {
  local candidate
  local map_database_dir="$1"

  for candidate in "${map_database_dir%/}"/*.mdb; do
    [[ -f "$candidate" ]] && return 0
  done
  return 1
}

print_presets() {
  cat <<'EOF'
sensor               Sensor kit + camera TF only (no actuator)
localization-only    Localization + Foxglove pose fallback; camera is already running
localization         Sensor + localization + Foxglove pose fallback + RViz (map required)
localize-live        Sensor + localization + Foxglove pose fallback + RViz (alias)
replay-localization  Safe rosbag replay + localization + RViz (bag/map required)
offline-vslam        Rosbag replay + VSLAM visualization + RViz (bag required)
offline-vslam-map    Rosbag replay + VSLAM mapping debug + RViz (bag/map required)
offline-localization Rosbag replay + VGL/VSLAM localization + RViz (bag/map required)
vehicle              Selected vehicle interface only
teleop               Joy/teleop/operation + selected vehicle interface
drive                Live sensor + joy/teleop/operation + selected vehicle interface
e2e                  Live RealSense + E2E inference + joy/teleop/operation + vehicle
runtime              Live sensor/localization/teleop + Foxglove pose fallback + vehicle (map required)
competition          Signal-aware rule planner + recovery + controller + live runtime (map required)
custom               Interactive component selection; all components start OFF
EOF
}

print_usage() {
  cat <<EOF
Usage:
  $(basename "$0")                         # preset/TUI selection
  $(basename "$0") --preset PRESET [options]
  $(basename "$0") PRESET [options]

Options:
  -p, --preset NAME    Use a preset (see --list-presets)
      --list-presets   List available presets
      --list-vehicles  List discovered vehicle interface profiles
      --list-sensor-kits
                        List discovered sensor kit profiles
      --validate-profiles
                        Validate every bringup profile manifest
      --map PATH       Set map_dir (or BRINGUP_MAP_DIR)
      --bag PATH       Set rosbag directory/metadata.yaml (or BRINGUP_ROSBAG)
      --raceline PATH  Enable the C++ raceline loader with this generated CSV
      --custom-line PATH
                        Use a named/custom 7-column trajectory CSV
      --custom-line-id NAME
                        Override the custom line ID published at runtime
      --custom-line-name NAME
                        Override the custom line display name
      --custom-line-open
                        Treat the custom line as open (closed is the default)
      --custom-line-closed
                        Explicitly treat the custom line as closed
      --rate RATE      Rosbag replay rate (default: 1.0)
      --vehicle PROFILE
                        Select a discovered vehicle interface profile
      --bag-manager    Enable bag manager recording control
      --no-bag-manager Disable bag manager recording control
      --sensor-kit NAME
                        Select a discovered sensor kit profile
      --rviz-config NAME_OR_PATH
                        Select RViz config: default, vslam-debug, or absolute path
      --localization-init MODE
                        VSLAM initialization: pose-hint (default), foxglove, or map-origin
      --vslam-mode MODE VSLAM tracking: vo (default) or vio
      --pose-hint      Alias for --localization-init pose-hint
      --no-pose-hint  Alias for --localization-init map-origin
      --components LIST
                        Custom component list, e.g. sensor,hd-map,foxglove
      --set ARG:=VALUE Override one bringup launch argument
      --dry-run        Print the exact command without running ROS
  -y, --yes            Skip the hardware launch confirmation
  -h, --help           Show this help
  -- ARG:=VALUE ...    Additional overrides (merged; duplicate names are replaced)

Examples:
  $(basename "$0") --preset vehicle --vehicle pca
  $(basename "$0") --preset drive --vehicle vesc
  $(basename "$0") --preset e2e --vehicle vesc
  $(basename "$0") --preset competition --vehicle vesc \
    --map /workspaces/map/course_a
  $(basename "$0") --preset localization --map /workspaces/map/course_a
  $(basename "$0") --preset localization --map /workspaces/map/course_a \
    --localization-init map-origin
  $(basename "$0") --preset localization --map /workspaces/map/course_a \
    --localization-init foxglove
  $(basename "$0") --preset localization --map /workspaces/map/course_a \
    --vslam-mode vio
  $(basename "$0") custom --components sensor,hd-map,foxglove \\
    --map /workspaces/map/course_a
  $(basename "$0") replay-localization --bag /workspaces/record/run_01 \\
    --map /workspaces/map/course_a --rate 0.5
  $(basename "$0") offline-vslam --bag /workspaces/record/run_01 --rate 0.5
  $(basename "$0") offline-vslam-map --bag /workspaces/record/run_01 \\
    --map /workspaces/map/course_a
  $(basename "$0") offline-localization --bag /workspaces/record/run_01 \\
    --map /workspaces/map/course_a
  $(basename "$0") runtime --vehicle vesc --map /workspaces/map/course_a --dry-run
  $(basename "$0") custom --components sensor,localization,hd-map,control,vehicle \\
    --vehicle vesc --map /workspaces/map/course_a \\
    --raceline /workspaces/map/course_a/course_raceline.csv
  $(basename "$0") custom --components sensor,localization,hd-map,control,vehicle \\
    --vehicle vesc --map /workspaces/map/course_a \\
    --custom-line /workspaces/map/course_a/course_a_custom_line.csv
  $(basename "$0") custom --components sensor,bag-manager,joy,teleop,operation,vehicle \\
    --vehicle vesc

The launcher starts with Jetson stats enabled and all actuator modules OFF.
Vehicle hardware is never enabled by a localization/replay preset, and replay
+ vehicle overrides are rejected. Direct ros2 launch remains available for
intentional HIL tests.
EOF
}

known_preset() {
  case "$1" in
    sensor|localization-only|localization|localize-live|replay-localization|\
      offline-vslam|offline-vslam-map|offline-localization|\
      vehicle|teleop|drive|e2e|runtime|competition|\
      vehicle-pca|vehicle-vesc|teleop-pca|teleop-vesc|\
      drive-pca|drive-vesc|runtime-pca|runtime-vesc|custom) return 0 ;;
    *) return 1 ;;
  esac
}

set_arg() {
  local name="$1"
  local value="${2-}"
  local index

  [[ "$name" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] \
    || die "invalid launch argument name: $name"
  for index in "${!ARG_NAMES[@]}"; do
    if [[ "${ARG_NAMES[$index]}" == "$name" ]]; then
      ARG_VALUES[$index]="$value"
      return
    fi
  done
  ARG_NAMES+=("$name")
  ARG_VALUES+=("$value")
}

get_arg() {
  local name="$1"
  local index

  for index in "${!ARG_NAMES[@]}"; do
    if [[ "${ARG_NAMES[$index]}" == "$name" ]]; then
      printf '%s' "${ARG_VALUES[$index]}"
      return 0
    fi
  done
  return 1
}

parse_override() {
  local assignment="$1"
  local name
  local value

  [[ "$assignment" == *':='* ]] \
    || die "launch override must use NAME:=VALUE: $assignment"
  name="${assignment%%:=*}"
  value="${assignment#*:=}"
  set_arg "$name" "$value"
}

set_base_args() {
  local enable_jetson_stats=false
  if is_jetson_platform; then
    enable_jetson_stats=true
  fi

  set_arg use_sim_time false
  set_arg enable_rosbag_replay false
  set_arg rosbag ''
  set_arg replay_rate 1.0
  set_arg allow_unsafe_replay_control_topics false
  set_arg allow_unsafe_replay_with_vehicle false

  set_arg enable_tool true
  set_arg enable_bag_manager false
  set_arg enable_joy false
  set_arg enable_teleop false
  set_arg enable_rc_serial false
  set_arg enable_jetson_stats "$enable_jetson_stats"
  set_arg enable_vslam_snapshot false
  set_arg enable_foxglove false
  set_arg foxglove_address 0.0.0.0
  set_arg foxglove_port 8767
  set_arg foxglove_topic_whitelist "$FOXGLOVE_DEFAULT_TOPIC_WHITELIST"
  set_arg foxglove_client_topic_whitelist "$FOXGLOVE_DEFAULT_CLIENT_TOPIC_WHITELIST"
  set_arg enable_operation false
  set_arg enable_planning false
  set_arg enable_competition_planning false
  set_arg competition_route_config_file ''
  set_arg enable_raceline_publisher false
  set_arg enable_custom_trajectory_publisher false
  set_arg enable_control false
  set_arg enable_e2e_inference false
  set_arg enable_object_detection false
  set_arg enable_sensor_kit false
  set_arg enable_localization false
  set_arg enable_vslam true
  set_arg vslam_enable_slam true
  set_arg vslam_mode vo
  set_arg vslam_localize_on_startup false
  set_arg enable_localization_manager true
  set_arg enable_vgl true
  set_arg enable_occupancy_map_server false
  set_arg enable_occupancy_map_lifecycle_manager false
  set_arg enable_omap_frame false
  set_arg enable_hd_map_publisher false
  set_arg enable_section_localizer false
  set_arg enable_vehicle false
  set_arg publish_vehicle_description false
  set_arg publish_vehicle_evs_description false
  set_arg publish_vehicle_thremo_description false
  set_arg enable_rviz false
}

first_existing_path() {
  local path
  local fallback="$1"
  shift
  for path in "$@"; do
    if [[ -f "$path" ]]; then
      printf '%s' "$path"
      return
    fi
  done
  printf '%s' "$fallback"
}

profile_command() {
  command -v "$PYTHON_BIN" >/dev/null 2>&1 \
    || die "Python command was not found: $PYTHON_BIN"
  [[ -f "$PROFILE_HELPER" ]] || die "bringup profile helper was not found: $PROFILE_HELPER"
  [[ -d "$PROFILE_ROOT" ]] || die "bringup profile directory was not found: $PROFILE_ROOT"
  "$PYTHON_BIN" "$PROFILE_HELPER" --root "$PROFILE_ROOT" "$@"
}

list_profiles() {
  local kind="$1"
  profile_command list --kind "$kind"
}

resolve_profile() {
  local kind="$1"
  local profile_id="$2"
  profile_command resolve \
    --kind "$kind" \
    --id "$profile_id" \
    --project-root "$PROJECT_ROOT" \
    --ros2-ws "$ROS2_WS"
}

print_profile_choices() {
  local kind="$1"
  local profile_id
  local label
  local profile_list

  profile_list="$(list_profiles "$kind")"
  while IFS=$'\t' read -r profile_id label; do
    [[ -n "$profile_id" ]] || continue
    printf '%-24s %s\n' "$profile_id" "$label"
  done <<< "$profile_list"
}

raceline_selector_param() {
  first_existing_path \
    "${ROS2_WS}/src/planning/jetpilot_planning/config/route_lane_selector.raceline.param.yaml" \
    "${ROS2_WS}/src/planning/jetpilot_planning/config/route_lane_selector.raceline.param.yaml" \
    "${PROJECT_ROOT}/ros2_ws/src/planning/jetpilot_planning/config/route_lane_selector.raceline.param.yaml" \
    "${ROS2_WS}/install/jetpilot_planning/share/jetpilot_planning/config/route_lane_selector.raceline.param.yaml"
}

custom_line_selector_param() {
  first_existing_path \
    "${ROS2_WS}/src/planning/jetpilot_planning/config/route_lane_selector.custom.param.yaml" \
    "${PROJECT_ROOT}/ros2_ws/src/planning/jetpilot_planning/config/route_lane_selector.custom.param.yaml" \
    "${ROS2_WS}/install/jetpilot_planning/share/jetpilot_planning/config/route_lane_selector.custom.param.yaml"
}

read_custom_line_metadata() {
  local metadata_path="$1"
  local trajectory_path="$2"
  local hd_map_path="${3:-}"
  "$PYTHON_BIN" -c '
import hashlib
import json
import os
import re
import sys

metadata_path, trajectory_path = sys.argv[1:3]
hd_map_path = sys.argv[3] if len(sys.argv) > 3 else ""

def reject(message):
    raise SystemExit(f"custom line metadata rejected: {message}")

try:
    with open(metadata_path, "r", encoding="utf-8") as stream:
        metadata = json.load(stream)
except (OSError, UnicodeError, json.JSONDecodeError) as error:
    reject(f"cannot read {metadata_path}: {error}")
if not isinstance(metadata, dict):
    reject("root must be an object")
if metadata.get("format") != "jetpilot_custom_line_v1":
    reject("unsupported format")

line_id = metadata.get("id")
if not isinstance(line_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", line_id):
    reject("invalid id")
line_name = metadata.get("name")
if (not isinstance(line_name, str) or not line_name or len(line_name) > 120 or
        any(ord(character) < 32 or ord(character) == 127 for character in line_name)):
    reject("invalid name")
closed_loop = metadata.get("closed_loop")
if not isinstance(closed_loop, bool):
    reject("closed_loop must be boolean")
revision = metadata.get("revision")
if not isinstance(revision, int) or isinstance(revision, bool) or revision < 1:
    reject("revision must be a positive integer")

source_hash = metadata.get("source_hash") or metadata.get("source_sha256")
if not isinstance(source_hash, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", source_hash):
    reject("source_hash must be SHA-256")
expected_hash = metadata.get("trajectory_sha256")
if not isinstance(expected_hash, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", expected_hash):
    reject("trajectory_sha256 must be SHA-256")

trajectory_reference = metadata.get("trajectory_csv")
if not isinstance(trajectory_reference, str) or not trajectory_reference:
    reject("trajectory_csv is required")
if os.path.isabs(trajectory_reference):
    referenced_path = trajectory_reference
else:
    referenced_path = os.path.join(os.path.dirname(metadata_path), trajectory_reference)
if os.path.realpath(referenced_path) != os.path.realpath(trajectory_path):
    canonical_metadata_path = os.path.splitext(trajectory_path)[0] + ".meta.json"
    relocated_canonical = (
        os.path.isabs(trajectory_reference) and
        os.path.realpath(metadata_path) == os.path.realpath(canonical_metadata_path) and
        os.path.basename(trajectory_reference) == os.path.basename(trajectory_path)
    )
    if not relocated_canonical:
        reject("trajectory_csv does not reference the selected CSV")
if os.path.islink(metadata_path) or os.path.islink(trajectory_path):
    reject("metadata and trajectory must not be symbolic links")
if not os.path.isfile(trajectory_path):
    reject("trajectory CSV is not a regular file")

digest = hashlib.sha256()
try:
    with open(trajectory_path, "rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
except OSError as error:
    reject(f"cannot hash trajectory CSV: {error}")
actual_hash = digest.hexdigest()
if actual_hash.lower() != expected_hash.lower():
    reject("trajectory_sha256 does not match the selected CSV")

speed_profile_mode = metadata.get("speed_profile_mode")
speed_authoring = metadata.get("speed_authoring")
section_authored = speed_profile_mode == "sections" or speed_authoring == "sections"
hd_map_hash_present = "hd_map_sha256" in metadata
expected_hd_map_hash = metadata.get("hd_map_sha256")
valid_hd_map_hash = (
    isinstance(expected_hd_map_hash, str) and
    re.fullmatch(r"[0-9a-fA-F]{64}", expected_hd_map_hash)
)
if section_authored and not valid_hd_map_hash:
    reject("section-authored custom line requires hd_map_sha256")
if hd_map_hash_present:
    if not valid_hd_map_hash:
        reject("hd_map_sha256 must be SHA-256")
    if not hd_map_path:
        if section_authored:
            reject("section-authored custom line requires --map PATH")
        reject("custom line metadata with hd_map_sha256 requires --map PATH")
    if os.path.islink(hd_map_path) or not os.path.isfile(hd_map_path):
        reject(f"HD map YAML is missing or not a regular file: {hd_map_path}")
    hd_map_digest = hashlib.sha256()
    try:
        with open(hd_map_path, "rb") as stream:
            while True:
                chunk = stream.read(1024 * 1024)
                if not chunk:
                    break
                hd_map_digest.update(chunk)
    except OSError as error:
        reject(f"cannot hash HD map YAML {hd_map_path}: {error}")
    if hd_map_digest.hexdigest().lower() != expected_hd_map_hash.lower():
        reject("hd_map_sha256 does not match the selected map")

if os.path.basename(trajectory_path) == "trajectory.csv":
    folder_id = os.path.basename(os.path.dirname(os.path.realpath(trajectory_path)))
    if folder_id != line_id:
        reject("manifest id does not match its custom_lines folder")

runtime_hash = actual_hash
print("\t".join((line_id, line_name, "true" if closed_loop else "false", runtime_hash)))
' "$metadata_path" "$trajectory_path" "$hd_map_path"
}

configure_raceline() {
  [[ "$RACELINE_CSV" == /* ]] \
    || die '--raceline must be an absolute CSV path inside the Docker workspace'
  if [[ "$DRY_RUN" != 'true' ]]; then
    [[ -f "$RACELINE_CSV" ]] || die "raceline CSV does not exist: $RACELINE_CSV"
    [[ ! -L "$RACELINE_CSV" ]] || die "raceline CSV must not be a symbolic link: $RACELINE_CSV"
  fi

  set_arg enable_planning true
  set_arg enable_raceline_publisher true
  set_arg planning_param "$(raceline_selector_param)"
  set_arg raceline_root "$(dirname -- "$RACELINE_CSV")"
  set_arg raceline_csv "$(basename -- "$RACELINE_CSV")"
}

configure_custom_line() {
  [[ "$CUSTOM_LINE_CSV" == /* ]] \
    || die '--custom-line must be an absolute CSV path inside the Docker workspace'
  if [[ "$DRY_RUN" != 'true' ]]; then
    [[ -f "$CUSTOM_LINE_CSV" ]] || die "custom line CSV does not exist: $CUSTOM_LINE_CSV"
    [[ ! -L "$CUSTOM_LINE_CSV" ]] \
      || die "custom line CSV must not be a symbolic link: $CUSTOM_LINE_CSV"
  fi

  local metadata_path=''
  local metadata_required=false
  local custom_basename
  custom_basename="$(basename -- "$CUSTOM_LINE_CSV")"
  if [[ "$custom_basename" == 'trajectory.csv' ]]; then
    metadata_path="$(dirname -- "$CUSTOM_LINE_CSV")/custom_line.json"
    metadata_required=true
  elif [[ "$custom_basename" == *_custom_line.csv ]]; then
    metadata_path="${CUSTOM_LINE_CSV%.csv}.meta.json"
    metadata_required=true
  elif [[ -f "${CUSTOM_LINE_CSV%.csv}.meta.json" ]]; then
    metadata_path="${CUSTOM_LINE_CSV%.csv}.meta.json"
  fi

  local metadata_id=''
  local metadata_name=''
  local metadata_closed=''
  local metadata_hash=''
  local hd_map_path=''
  if [[ -n "$MAP_DIR" ]]; then
    local map_name
    map_name="$(basename -- "${MAP_DIR%/}")"
    hd_map_path="${MAP_DIR%/}/${map_name}_hd_map.yaml"
  fi
  if [[ -n "$metadata_path" && -f "$metadata_path" ]]; then
    local metadata_output
    metadata_output="$(read_custom_line_metadata "$metadata_path" "$CUSTOM_LINE_CSV" "$hd_map_path")" \
      || die "invalid custom line metadata: $metadata_path"
    IFS=$'\t' read -r metadata_id metadata_name metadata_closed metadata_hash \
      <<< "$metadata_output"
  elif [[ "$metadata_required" == 'true' && -e "$CUSTOM_LINE_CSV" ]]; then
    die "custom line metadata is required for this bundle: $metadata_path"
  fi

  if [[ "$CUSTOM_LINE_ID_EXPLICIT" != 'true' && -n "$metadata_id" ]]; then
    CUSTOM_LINE_ID="$metadata_id"
  fi
  if [[ "$CUSTOM_LINE_NAME_EXPLICIT" != 'true' && -n "$metadata_name" ]]; then
    CUSTOM_LINE_NAME="$metadata_name"
  fi
  if [[ "$CUSTOM_LINE_CLOSED_EXPLICIT" != 'true' && -n "$metadata_closed" ]]; then
    CUSTOM_LINE_CLOSED="$metadata_closed"
  fi

  if [[ -z "$CUSTOM_LINE_ID" ]]; then
    if [[ "$(basename -- "$CUSTOM_LINE_CSV")" == 'trajectory.csv' ]]; then
      CUSTOM_LINE_ID="$(basename -- "$(dirname -- "$CUSTOM_LINE_CSV")")"
    else
      CUSTOM_LINE_ID="$(basename -- "$CUSTOM_LINE_CSV" .csv)"
      CUSTOM_LINE_ID="${CUSTOM_LINE_ID%_custom_line}"
    fi
  fi
  [[ "$CUSTOM_LINE_ID" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$ ]] \
    || die '--custom-line-id must be 1-64 characters: letters, digits, dot, underscore, hyphen'
  if [[ -z "$CUSTOM_LINE_NAME" ]]; then
    CUSTOM_LINE_NAME="$CUSTOM_LINE_ID"
  fi
  ((${#CUSTOM_LINE_NAME} <= 120)) \
    || die '--custom-line-name must be 120 characters or fewer'
  [[ ! "$CUSTOM_LINE_NAME" =~ [[:cntrl:]] ]] \
    || die '--custom-line-name must not contain control characters'

  set_arg enable_planning true
  set_arg enable_custom_trajectory_publisher true
  set_arg planning_param "$(custom_line_selector_param)"
  set_arg custom_root "$(dirname -- "$CUSTOM_LINE_CSV")"
  set_arg custom_csv "$(basename -- "$CUSTOM_LINE_CSV")"
  set_arg custom_line_id "$CUSTOM_LINE_ID"
  set_arg custom_line_name "$CUSTOM_LINE_NAME"
  if [[ -n "$metadata_hash" ]]; then
    set_arg custom_source_hash "$metadata_hash"
  fi
  set_arg custom_closed "$CUSTOM_LINE_CLOSED"
}

apply_vehicle() {
  local backend="$1"
  local records
  local key
  local value

  if [[ "$backend" == 'none' ]]; then
    VEHICLE_BACKEND='none'
    set_arg enable_vehicle false
    set_arg publish_vehicle_description false
    set_arg publish_vehicle_evs_description false
    set_arg publish_vehicle_thremo_description false
    return
  fi

  records="$(resolve_profile vehicle "$backend")"
  set_arg enable_vehicle true
  set_arg publish_vehicle_description false
  set_arg publish_vehicle_evs_description false
  set_arg publish_vehicle_thremo_description false
  while IFS=$'\t' read -r key value; do
    case "$key" in
      id) VEHICLE_BACKEND="$value" ;;
      launch_package) set_arg vehicle_interface_pkg "$value" ;;
      launch_file) set_arg vehicle_interface_launch "$value" ;;
      driver_param) set_arg vehicle_driver_param "$value" ;;
      argument:*) set_arg "${key#argument:}" "$value" ;;
    esac
  done <<< "$records"

  [[ -n "$(get_arg vehicle_interface_pkg 2>/dev/null || true)" ]] \
    || die "vehicle profile '$backend' does not define a launch package"
  [[ -n "$(get_arg vehicle_interface_launch 2>/dev/null || true)" ]] \
    || die "vehicle profile '$backend' does not define a launch file"
  [[ -n "$(get_arg vehicle_driver_param 2>/dev/null || true)" ]] \
    || die "vehicle profile '$backend' does not define a driver parameter file"
}

apply_sensor_kit() {
  local sensor_kit="$1"
  local records
  local key
  local value

  records="$(resolve_profile sensor_kit "$sensor_kit")"
  SENSOR_KIT_RTP_TOPICS=()
  while IFS=$'\t' read -r key value; do
    case "$key" in
      id) SENSOR_KIT_PROFILE="$value" ;;
      launch_package) set_arg sensor_kit_interface_pkg "$value" ;;
      launch_file) set_arg sensor_kit_interface_launch "$value" ;;
      argument:*) set_arg "${key#argument:}" "$value" ;;
      rtp_topic) SENSOR_KIT_RTP_TOPICS+=("$value") ;;
    esac
  done <<< "$records"

  [[ -n "$(get_arg sensor_kit_interface_pkg 2>/dev/null || true)" ]] \
    || die "sensor kit profile '$sensor_kit' does not define a launch package"
  [[ -n "$(get_arg sensor_kit_interface_launch 2>/dev/null || true)" ]] \
    || die "sensor kit profile '$sensor_kit' does not define a launch file"
}

resolve_rviz_config() {
  local config="$1"

  case "$config" in
    default)
      printf '%s' "${ROS2_WS}/install/${LAUNCH_PACKAGE}/share/${LAUNCH_PACKAGE}/rviz/default.rviz"
      ;;
    vslam-debug|vslam_debug|vslam)
      printf '%s' "${ROS2_WS}/install/${LAUNCH_PACKAGE}/share/${LAUNCH_PACKAGE}/rviz/vslam_debug.rviz"
      ;;
    /*)
      printf '%s' "$config"
      ;;
    *)
      printf 'error: rviz config must be default, vslam-debug, or an absolute path: %s\n' \
        "$config" >&2
      return 1
      ;;
  esac
}

enable_teleop_stack() {
  set_arg enable_tool true
  set_arg enable_bag_manager false
  set_arg enable_joy true
  set_arg enable_teleop true
  set_arg enable_operation true
}

enable_drive_stack() {
  set_arg enable_tool true
  set_arg enable_bag_manager false
  set_arg enable_joy true
  set_arg enable_teleop true
  set_arg enable_operation true
}

enable_localization_stack() {
  set_arg enable_localization true
  set_arg enable_vslam true
  set_arg enable_localization_manager true
  set_arg enable_vgl true
  set_arg publish_vehicle_description true
}

enable_live_localization_stack() {
  enable_localization_stack
  # Keep a low-overhead manual pose path ready when VGL is unavailable or fails.
  set_arg enable_foxglove true
}

set_localization_init_mode() {
  LOCALIZATION_INIT_MODE="$1"
  case "$1" in
    pose-hint)
      set_arg vslam_localize_on_startup false
      ;;
    foxglove)
      set_arg vslam_localize_on_startup false
      set_arg enable_vgl false
      set_arg enable_foxglove true
      ;;
    map-origin)
      set_arg vslam_localize_on_startup true
      ;;
    *) die "localization initialization must be pose-hint, foxglove, or map-origin: $1" ;;
  esac
}

normalize_localization_init_mode() {
  if [[ -n "$CLI_LOCALIZATION_INIT" ]]; then
    # A named mode is authoritative over contradictory generic --set overrides.
    set_localization_init_mode "$CLI_LOCALIZATION_INIT"
  elif is_true "$(get_arg vslam_localize_on_startup)"; then
    LOCALIZATION_INIT_MODE='map-origin'
  else
    LOCALIZATION_INIT_MODE='pose-hint'
  fi

  case "$LOCALIZATION_INIT_MODE" in
    map-origin)
      # Origin startup and VGL both initiate localization and must not race.
      set_arg enable_vgl false
      ;;
    foxglove)
      set_arg vslam_localize_on_startup false
      set_arg enable_vgl false
      set_arg enable_foxglove true
      ;;
  esac
}

set_vslam_mode() {
  case "$1" in
    [Vv][Oo]) set_arg vslam_mode vo ;;
    [Vv][Ii][Oo]) set_arg vslam_mode vio ;;
    *) die "VSLAM mode must be vo or vio: $1" ;;
  esac
}

normalize_vslam_mode() {
  if [[ -n "$CLI_VSLAM_MODE" ]]; then
    set_vslam_mode "$CLI_VSLAM_MODE"
  else
    set_vslam_mode "$(get_arg vslam_mode)"
  fi
}

enable_offline_replay_stack() {
  set_arg enable_rosbag_replay true
  set_arg use_sim_time true
  set_arg enable_sensor_kit false
  set_arg enable_tool false
  set_arg enable_bag_manager false
  set_arg enable_joy false
  set_arg enable_teleop false
  set_arg enable_rc_serial false
  set_arg enable_operation false
  set_arg enable_planning false
  set_arg enable_control false
  set_arg publish_vehicle_description false
  set_arg enable_rviz true
  set_arg rviz_config_file "$(resolve_rviz_config vslam-debug)"
  apply_vehicle none
  REQUIRES_ROSBAG=true
}

apply_preset() {
  local preset="$1"

  case "$preset" in
    sensor)
      set_arg enable_sensor_kit true
      set_arg publish_vehicle_description true
      ;;
    localization-only)
      enable_live_localization_stack
      REQUIRES_MAP=true
      ;;
    localization|localize-live)
      set_arg enable_sensor_kit true
      enable_live_localization_stack
      set_arg enable_rviz true
      REQUIRES_MAP=true
      ;;
    replay-localization)
      set_arg enable_rosbag_replay true
      set_arg use_sim_time true
      enable_localization_stack
      set_arg enable_rviz true
      apply_vehicle none
      REQUIRES_MAP=true
      REQUIRES_ROSBAG=true
      ;;
    offline-vslam)
      enable_offline_replay_stack
      set_arg enable_localization true
      set_arg enable_vslam true
      set_arg enable_vgl false
      set_arg enable_localization_manager false
      set_arg vslam_enable_slam true
      set_arg vslam_enable_visualization true
      ;;
    offline-vslam-map)
      enable_offline_replay_stack
      set_arg enable_localization true
      set_arg enable_vslam true
      set_arg enable_vgl false
      set_arg enable_localization_manager false
      set_arg vslam_enable_slam true
      set_arg vslam_enable_visualization true
      REQUIRES_MAP=true
      ;;
    offline-localization)
      enable_offline_replay_stack
      enable_localization_stack
      set_arg enable_vslam true
      set_arg enable_vgl true
      set_arg vslam_enable_slam true
      set_arg vslam_enable_visualization true
      set_arg vslam_localize_on_startup false
      REQUIRES_MAP=true
      ;;
    vehicle)
      REQUIRES_VEHICLE=true
      ;;
    teleop)
      enable_teleop_stack
      REQUIRES_VEHICLE=true
      ;;
    drive)
      set_arg enable_sensor_kit true
      enable_drive_stack
      REQUIRES_VEHICLE=true
      ;;
    e2e)
      set_arg enable_sensor_kit true
      enable_drive_stack
      set_arg enable_e2e_inference true
      REQUIRES_VEHICLE=true
      ;;
    runtime)
      enable_teleop_stack
      set_arg enable_sensor_kit true
      enable_live_localization_stack
      REQUIRES_VEHICLE=true
      REQUIRES_MAP=true
      ;;
    competition)
      enable_teleop_stack
      set_arg enable_sensor_kit true
      enable_live_localization_stack
      set_arg enable_hd_map_publisher true
      set_arg enable_section_localizer true
      set_arg enable_planning false
      set_arg enable_competition_planning true
      set_arg enable_object_detection true
      set_arg object_detection_detections_topic /perception/signal/detections
      set_arg enable_control true
      REQUIRES_VEHICLE=true
      REQUIRES_MAP=true
      ;;
    vehicle-pca)
      apply_vehicle pca
      ;;
    vehicle-vesc)
      apply_vehicle vesc
      ;;
    teleop-pca)
      enable_teleop_stack
      apply_vehicle pca
      ;;
    teleop-vesc)
      enable_teleop_stack
      apply_vehicle vesc
      ;;
    drive-pca)
      set_arg enable_sensor_kit true
      enable_drive_stack
      apply_vehicle pca
      ;;
    drive-vesc)
      set_arg enable_sensor_kit true
      enable_drive_stack
      apply_vehicle vesc
      ;;
    runtime-pca)
      enable_teleop_stack
      set_arg enable_sensor_kit true
      enable_live_localization_stack
      apply_vehicle pca
      REQUIRES_MAP=true
      ;;
    runtime-vesc)
      enable_teleop_stack
      set_arg enable_sensor_kit true
      enable_live_localization_stack
      apply_vehicle vesc
      REQUIRES_MAP=true
      ;;
    custom)
      ;;
    *) die "unknown preset: $preset (use --list-presets)" ;;
  esac
}

choose_one() {
  local prompt="$1"
  local selected
  local choice
  local index
  shift
  local options=("$@")

  if command -v fzf >/dev/null 2>&1; then
    selected="$(printf '%s\n' "${options[@]}" | fzf \
      --prompt="${prompt} > " --height=60% --border --reverse || true)"
    [[ -n "$selected" ]] || return 130
    printf '%s' "$selected"
    return
  fi

  printf '%s\n' "$prompt" >&2
  for index in "${!options[@]}"; do
    printf '  %2d) %s\n' "$((index + 1))" "${options[$index]}" >&2
  done
  while true; do
    read -r -p '番号を入力 (q: cancel): ' choice
    case "$choice" in
      q|Q) return 130 ;;
    esac
    if [[ "$choice" =~ ^[0-9]+$ ]] \
      && ((choice >= 1 && choice <= ${#options[@]})); then
      printf '%s' "${options[$((choice - 1))]}"
      return
    fi
    printf '一覧にある番号を入力してください。\n' >&2
  done
}

choose_many() {
  local prompt="$1"
  local selected
  local choice
  local token
  local index
  local line
  local output=''
  shift
  local options=("$@")

  if command -v fzf >/dev/null 2>&1; then
    selected="$(printf '%s\n' "${options[@]}" | fzf \
      --multi --prompt="${prompt} > " --height=80% --border --reverse \
      --header='Tab/SpaceでON/OFF、Enterで決定' || true)"
    printf '%s' "$selected"
    return
  fi

  printf '%s\n' "$prompt" >&2
  for index in "${!options[@]}"; do
    printf '  %2d) %s\n' "$((index + 1))" "${options[$index]}" >&2
  done
  while true; do
    read -r -p 'ONにする番号を複数入力 (例: 1 3 5, 空: none, q: cancel): ' choice
    case "$choice" in
      q|Q) return 130 ;;
      '') printf ''; return ;;
    esac

    output=''
    choice="${choice//,/ }"
    for token in $choice; do
      if [[ "$token" =~ ^[0-9]+$ ]] \
        && ((token >= 1 && token <= ${#options[@]})); then
        line="${options[$((token - 1))]}"
        output+="${line}"$'\n'
      else
        output=''
        break
      fi
    done
    if [[ -n "$output" ]]; then
      printf '%s' "$output"
      return
    fi
    printf '一覧にある番号を空白区切りで入力してください。\n' >&2
  done
}

prompt_yes_no() {
  local prompt="$1"
  local default="${2:-no}"
  local suffix='[y/N]'
  local answer

  [[ "$default" == 'yes' ]] && suffix='[Y/n]'
  while true; do
    read -r -p "${prompt} ${suffix}: " answer
    if [[ -z "$answer" ]]; then
      [[ "$default" == 'yes' ]]
      return
    fi
    case "$answer" in
      y|Y|yes|YES) return 0 ;;
      n|N|no|NO) return 1 ;;
      *) printf 'y または n を入力してください。\n' >&2 ;;
    esac
  done
}

prompt_value() {
  local prompt="$1"
  local default="${2:-}"
  local value

  if [[ -n "$default" ]]; then
    read -r -e -p "${prompt} [${default}]: " value
    printf '%s' "${value:-$default}"
  else
    read -r -e -p "${prompt}: " value
    printf '%s' "$value"
  fi
}

prompt_path() {
  prompt_value "$@"
}

append_unique_option() {
  local candidate="$1"
  local option

  [[ -n "$candidate" ]] || return
  for option in "${options[@]-}"; do
    [[ "$option" == "$candidate" ]] && return
  done
  options+=("$candidate")
}

apply_custom_component_token() {
  local token="$1"

  case "$token" in
    sensor|live-sensor)
      set_arg enable_sensor_kit true
      set_arg publish_vehicle_description true
      ;;
    replay|rosbag-replay)
      set_arg enable_rosbag_replay true
      set_arg use_sim_time true
      REQUIRES_ROSBAG=true
      ;;
    localization)
      enable_localization_stack
      REQUIRES_MAP=true
      ;;
    occupancy-map)
      enable_localization_stack
      set_arg enable_occupancy_map_server true
      set_arg enable_occupancy_map_lifecycle_manager true
      REQUIRES_MAP=true
      ;;
    hd-map)
      enable_localization_stack
      set_arg enable_hd_map_publisher true
      REQUIRES_MAP=true
      ;;
    section-localizer)
      enable_localization_stack
      set_arg enable_section_localizer true
      REQUIRES_MAP=true
      ;;
    tool)
      set_arg enable_tool true
      ;;
    foxglove)
      set_arg enable_foxglove true
      ;;
    bag-manager)
      set_arg enable_tool true
      set_arg enable_bag_manager true
      ;;
    joy)
      set_arg enable_tool true
      set_arg enable_joy true
      ;;
    teleop)
      set_arg enable_tool true
      set_arg enable_teleop true
      ;;
    rc-serial)
      set_arg enable_tool true
      set_arg enable_rc_serial true
      ;;
    operation)
      set_arg enable_operation true
      ;;
    planning)
      set_arg enable_planning true
      ;;
    competition-planning)
      enable_localization_stack
      set_arg enable_hd_map_publisher true
      set_arg enable_section_localizer true
      set_arg enable_planning false
      set_arg enable_competition_planning true
      set_arg enable_object_detection true
      set_arg object_detection_detections_topic /perception/signal/detections
      set_arg enable_control true
      set_arg enable_operation true
      REQUIRES_MAP=true
      ;;
    raceline)
      set_arg enable_planning true
      set_arg enable_raceline_publisher true
      REQUIRES_RACELINE=true
      ;;
    custom-line|custom-trajectory)
      set_arg enable_planning true
      set_arg enable_custom_trajectory_publisher true
      REQUIRES_CUSTOM_LINE=true
      REQUIRES_MAP=true
      ;;
    control|autonomous-control)
      set_arg enable_planning true
      set_arg enable_control true
      set_arg enable_operation true
      ;;
    e2e|e2e-inference)
      set_arg enable_e2e_inference true
      set_arg enable_operation true
      ;;
    rviz)
      set_arg enable_rviz true
      ;;
    vehicle)
      REQUIRES_VEHICLE=true
      ;;
    vehicle-pca)
      apply_vehicle pca
      ;;
    vehicle-vesc)
      apply_vehicle vesc
      ;;
    none|'')
      ;;
    *) die "unknown custom component: $token" ;;
  esac
}

apply_custom_components() {
  local selection="$1"
  local token
  local line
  local normalized
  local saw_live=false
  local saw_replay=false
  local vehicle_count=0

  if [[ -n "$selection" && "$selection" != *$'\n'* ]]; then
    normalized="${selection//,/ }"
    for token in $normalized; do
      case "$token" in
        sensor|live-sensor) saw_live=true ;;
        replay|rosbag-replay) saw_replay=true ;;
        vehicle|vehicle-pca|vehicle-vesc) vehicle_count=$((vehicle_count + 1)) ;;
      esac
      apply_custom_component_token "$token"
    done
  else
    while IFS= read -r line; do
      [[ -n "$line" ]] || continue
      token="${line%%[[:space:]]*}"
      case "$token" in
        sensor|live-sensor) saw_live=true ;;
        replay|rosbag-replay) saw_replay=true ;;
        vehicle|vehicle-pca|vehicle-vesc) vehicle_count=$((vehicle_count + 1)) ;;
      esac
      apply_custom_component_token "$token"
    done <<< "$selection"
  fi

  if [[ "$saw_live" == 'true' && "$saw_replay" == 'true' ]]; then
    die 'custom components must choose either sensor or replay, not both'
  fi
  if ((vehicle_count > 1)); then
    die 'custom components must choose only one vehicle interface'
  fi
}

discover_map() {
  local path
  local selected
  local options=()

  [[ -n "$MAP_DIR" && -d "$MAP_DIR" ]] && append_unique_option "$MAP_DIR"
  [[ -d "${MAP_ROOT%/}/latest" ]] && append_unique_option "${MAP_ROOT%/}/latest"
  if [[ -d "$MAP_ROOT" ]]; then
    if [[ -d "${MAP_ROOT%/}/cuvgl_map" || -d "${MAP_ROOT%/}/cuvslam_map" ]]; then
      append_unique_option "${MAP_ROOT%/}"
    fi
    while IFS= read -r path; do
      append_unique_option "$path"
    done < <(find "$MAP_ROOT" -mindepth 1 -maxdepth 1 -type d \
      | sort -r | head -50)
    while IFS= read -r path; do
      append_unique_option "$path"
    done < <(find "$MAP_ROOT" -mindepth 1 -maxdepth 3 -type d \
      \( -name cuvgl_map -o -name cuvslam_map \) -exec dirname {} \; \
      | sort -ru | head -50)
  fi
  if ((${#options[@]} == 1)); then
    MAP_DIR="${options[0]}"
    printf 'Selected map : %s\n' "$MAP_DIR" >&2
    return
  fi
  options+=('パスを手入力...')
  selected="$(choose_one 'Map directory' "${options[@]}")" || exit $?
  if [[ "$selected" == 'パスを手入力...' ]]; then
    MAP_DIR="$(prompt_path 'Map directory' "$MAP_DIR")"
  else
    MAP_DIR="$selected"
  fi
}

discover_rosbag() {
  local metadata
  local selected
  local options=()

  if [[ -n "$ROSBAG" ]]; then
    options+=("$ROSBAG")
  fi
  if [[ -d "$RECORD_ROOT" ]]; then
    while IFS= read -r metadata; do
      options+=("$(dirname "$metadata")")
    done < <(find "$RECORD_ROOT" -type f -name metadata.yaml | sort -r | head -50)
  fi
  options+=('パスを手入力...')
  selected="$(choose_one 'Rosbag' "${options[@]}")" || exit $?
  if [[ "$selected" == 'パスを手入力...' ]]; then
    ROSBAG="$(prompt_path 'Rosbag directory / metadata.yaml' "$ROSBAG")"
  else
    ROSBAG="$selected"
  fi
}

discover_raceline() {
  local search_root="${MAP_DIR:-$MAP_ROOT}"
  local path
  local selected
  local options=()

  if [[ -d "$search_root" ]]; then
    while IFS= read -r path; do
      options+=("$path")
    done < <(find "$search_root" -maxdepth 4 -type f -name '*raceline*.csv' | sort -r | head -50)
  fi
  options+=('パスを手入力...')
  selected="$(choose_one 'Raceline CSV' "${options[@]}")" || exit $?
  if [[ "$selected" == 'パスを手入力...' ]]; then
    RACELINE_CSV="$(prompt_path 'Raceline CSV' "$RACELINE_CSV")"
  else
    RACELINE_CSV="$selected"
  fi
}

discover_custom_line() {
  local search_root="${MAP_DIR:-$MAP_ROOT}"
  local path
  local selected
  local options=()

  if [[ -d "$search_root" ]]; then
    while IFS= read -r path; do
      options+=("$path")
    done < <(find "$search_root" -maxdepth 6 -type f \
      \( -name '*_custom_line.csv' -o -path '*/custom_lines/*/trajectory.csv' \) \
      | sort -r | head -50)
  fi
  options+=('パスを手入力...')
  selected="$(choose_one 'Custom line CSV' "${options[@]}")" || exit $?
  if [[ "$selected" == 'パスを手入力...' ]]; then
    CUSTOM_LINE_CSV="$(prompt_path 'Custom line CSV' "$CUSTOM_LINE_CSV")"
  else
    CUSTOM_LINE_CSV="$selected"
  fi
}

select_active_custom_line_from_map() {
  local map_name
  local candidate_csv
  local candidate_meta

  # No selected map simply means that the regular required-input check below
  # should explain how to provide a custom line.  Keep this helper successful
  # under `set -e` instead of aborting the launcher without a diagnostic.
  [[ -n "$MAP_DIR" ]] || return 0
  map_name="$(basename -- "${MAP_DIR%/}")"
  candidate_csv="${MAP_DIR%/}/${map_name}_custom_line.csv"
  candidate_meta="${candidate_csv%.csv}.meta.json"
  if [[ -f "$candidate_csv" && -f "$candidate_meta" ]]; then
    CUSTOM_LINE_CSV="$candidate_csv"
  fi
}

interactive_custom() {
  local selection
  local options=(
    'sensor             Live sensor kit + camera TF'
    'replay             Rosbag replay input'
    'localization       Localization stack'
    'occupancy-map      Occupancy map server'
    'hd-map             HD map publisher'
    'section-localizer  Section localizer'
    'tool               Tool container only'
    'foxglove           Foxglove WebSocket bridge'
    'bag-manager        Bag manager'
    'joy                Joy node'
    'teleop             Teleop node'
    'rc-serial          RC serial reader'
    'operation          Operation manager'
    'planning           Route/lane planning only'
    'competition-planning Signal planner + recovery + controller'
    'raceline           Planning with generated raceline CSV'
    'custom-line        Planning with named custom line + speed CSV'
    'control            Planning + Pure Pursuit control'
    'e2e                TensorRT E2E direct control'
    'rviz               RViz'
    'vehicle            Vehicle interface (select next)'
  )

  selection="$(choose_many 'Custom components' "${options[@]}")" || exit $?
  apply_custom_components "$selection"
}

choose_preset_interactively() {
  local selection
  local options=()
  local line

  while IFS= read -r line; do
    options+=("$line")
  done < <(print_presets)
  selection="$(choose_one 'JetPilot bringup preset' "${options[@]}")" || exit $?
  PRESET="${selection%%[[:space:]]*}"
}

configure_bag_manager_interactively() {
  local selection
  local current
  local options=()

  current="$(get_arg enable_bag_manager)"
  if is_true "$current"; then
    options+=('on   Bag manager ON')
    options+=('off  Bag manager OFF')
  else
    options+=('off  Bag manager OFF')
    options+=('on   Bag manager ON')
  fi

  selection="$(choose_one 'Bag manager' "${options[@]}")" || exit $?
  case "${selection%%[[:space:]]*}" in
    on)
      set_arg enable_tool true
      set_arg enable_bag_manager true
      ;;
    off)
      set_arg enable_bag_manager false
      ;;
    *) die "unknown bag manager selection: $selection" ;;
  esac
}

configure_rtp_interactively() {
  local current
  local host
  local port
  local topic
  local selected
  local options=()

  current="$(get_arg sensor_kit_enable_rtp_stream 2>/dev/null || true)"
  if is_true "$current"; then
    options+=('on   RTP送信 ON')
    options+=('off  RTP送信 OFF')
  else
    options+=('off  RTP送信 OFF')
    options+=('on   RTP送信 ON')
  fi
  selected="$(choose_one 'RTP stream' "${options[@]}")" || exit $?
  case "${selected%%[[:space:]]*}" in
    on)
      set_arg sensor_kit_enable_rtp_stream true
      ;;
    off)
      set_arg sensor_kit_enable_rtp_stream false
      return
      ;;
    *)
      die "unknown RTP stream selection: $selected"
      ;;
  esac

  host="$(get_arg sensor_kit_rtp_host 2>/dev/null || true)"
  while [[ -z "$host" ]]; do
    host="$(prompt_value 'RTP送信先IP / host' "$host")"
    [[ -n "$host" ]] || printf '送信先IPまたはhost名を入力してください。\n' >&2
  done
  set_arg sensor_kit_rtp_host "$host"

  port="$(get_arg sensor_kit_rtp_port 2>/dev/null || true)"
  port="${port:-5004}"
  while true; do
    port="$(prompt_value 'RTP送信先UDP port' "$port")"
    if is_valid_port "$port"; then
      break
    fi
    printf 'UDP portは1〜65535の整数で入力してください。\n' >&2
  done
  set_arg sensor_kit_rtp_port "$port"

  options=()
  topic="$(get_arg sensor_kit_rtp_image_topic 2>/dev/null || true)"
  append_unique_option "$topic"
  for topic in "${SENSOR_KIT_RTP_TOPICS[@]}"; do
    append_unique_option "$topic"
  done
  options+=('トピックを手入力...')

  selected="$(choose_one 'RTP image topic' "${options[@]}")" || exit $?
  if [[ "$selected" == 'トピックを手入力...' ]]; then
    topic=''
    while [[ -z "$topic" ]]; do
      topic="$(prompt_value 'RTP image topic')"
      [[ -n "$topic" ]] || printf '画像トピックを入力してください。\n' >&2
    done
  else
    topic="$selected"
  fi
  set_arg sensor_kit_rtp_image_topic "$topic"
}

configure_sensor_kit_interactively() {
  local selection
  local profile_id
  local label
  local profile_list
  local options=()

  profile_list="$(list_profiles sensor_kit)"
  while IFS=$'\t' read -r profile_id label; do
    [[ -n "$profile_id" ]] || continue
    options+=("$profile_id  $label")
  done <<< "$profile_list"

  selection="$(choose_one 'Sensor kit launch' "${options[@]}")" || exit $?
  apply_sensor_kit "${selection%%[[:space:]]*}"
  configure_rtp_interactively
}

configure_vehicle_interactively() {
  local selection
  local profile_id
  local label
  local profile_list
  local options=()

  profile_list="$(list_profiles vehicle)"
  while IFS=$'\t' read -r profile_id label; do
    [[ -n "$profile_id" ]] || continue
    options+=("$profile_id  $label")
  done <<< "$profile_list"

  selection="$(choose_one 'Vehicle interface' "${options[@]}")" || exit $?
  apply_vehicle "${selection%%[[:space:]]*}"
}

configure_rviz_interactively() {
  local selection
  local options=(
    'vslam-debug  VSLAM debug'
    'default      Default'
    'path         パスを手入力...'
  )

  selection="$(choose_one 'RViz config' "${options[@]}")" || exit $?
  case "${selection%%[[:space:]]*}" in
    vslam-debug)
      RVIZ_CONFIG="$(resolve_rviz_config vslam-debug)"
      ;;
    default)
      RVIZ_CONFIG="$(resolve_rviz_config default)"
      ;;
    path)
      RVIZ_CONFIG="$(prompt_path 'RViz config file' "$RVIZ_CONFIG")"
      ;;
    *) die "unknown RViz config selection: $selection" ;;
  esac
}

normalize_rosbag_path() {
  if [[ -f "$ROSBAG" && "$(basename "$ROSBAG")" == 'metadata.yaml' ]]; then
    ROSBAG="$(dirname "$ROSBAG")"
  fi
}

validate_configuration() {
  local configured_path
  local foxglove_startup=false
  local mapping_output
  local origin_startup
  local replay
  local vehicle

  if [[ -z "$MAP_DIR" ]] && configured_path="$(get_arg map_dir 2>/dev/null)"; then
    MAP_DIR="$configured_path"
  fi
  if [[ -z "$ROSBAG" ]] && configured_path="$(get_arg rosbag 2>/dev/null)"; then
    ROSBAG="$configured_path"
  fi
  if [[ -n "$ROSBAG" ]]; then
    set_arg enable_rosbag_replay true
    set_arg use_sim_time true
  fi
  replay="$(get_arg enable_rosbag_replay)"
  vehicle="$(get_arg enable_vehicle)"
  origin_startup="$(get_arg vslam_localize_on_startup)"
  mapping_output="$(get_arg vslam_save_map_folder_path 2>/dev/null || true)"
  if [[ "$LOCALIZATION_INIT_MODE" == 'foxglove' ]]; then
    foxglove_startup=true
  fi
  normalize_rosbag_path

  if is_true "$(get_arg allow_unsafe_replay_control_topics)" \
    || is_true "$(get_arg allow_unsafe_replay_with_vehicle)"; then
    die 'unsafe replay overrides are intentionally unsupported by this launcher'
  fi
  if is_true "$(get_arg enable_jetson_stats)" && ! is_jetson_platform; then
    die 'enable_jetson_stats is available only on Jetson (ISAAC_ROS_PLATFORM=arm64-jetpack)'
  fi
  if is_true "$replay" && is_true "$vehicle"; then
    die 'rosbag replay and vehicle hardware cannot be enabled together'
  fi
  if is_true "$(get_arg enable_control)" \
    && is_true "$(get_arg enable_e2e_inference)"; then
    die 'enable_control and enable_e2e_inference cannot be enabled together; both publish /auto/control_cmd'
  fi
  if is_true "$(get_arg enable_planning)" \
    && is_true "$(get_arg enable_competition_planning)"; then
    die 'enable_planning and enable_competition_planning cannot both be true'
  fi
  if [[ "$REQUIRES_VEHICLE" == 'true' ]] && ! is_true "$vehicle"; then
    die "preset '$PRESET' requires --vehicle PROFILE (see --list-vehicles)"
  fi
  if [[ "$REQUIRES_MAP" == 'true' ]] \
    && is_true "$(get_arg enable_localization)" \
    && [[ -z "$MAP_DIR" ]]; then
    die "preset '$PRESET' requires --map PATH"
  fi
  if is_true "$origin_startup" || is_true "$foxglove_startup"; then
    is_true "$(get_arg enable_localization)" \
      || die "${LOCALIZATION_INIT_MODE} initialization requires enable_localization=true"
    is_true "$(get_arg enable_vslam)" \
      || die "${LOCALIZATION_INIT_MODE} initialization requires enable_vslam=true"
    is_true "$(get_arg vslam_enable_slam)" \
      || die "${LOCALIZATION_INIT_MODE} initialization requires vslam_enable_slam=true"
    is_true "$(get_arg enable_localization_manager)" \
      || die "${LOCALIZATION_INIT_MODE} initialization requires the localization manager for safety status"
    [[ -n "$MAP_DIR" ]] || die "${LOCALIZATION_INIT_MODE} initialization requires --map PATH"
    [[ -z "$mapping_output" ]] \
      || die "${LOCALIZATION_INIT_MODE} initialization cannot be combined with vslam_save_map_folder_path"
  fi
  if is_true "$foxglove_startup"; then
    is_true "$(get_arg enable_foxglove)" \
      || die 'foxglove initialization requires enable_foxglove=true'
  fi
  if [[ "$REQUIRES_ROSBAG" == 'true' ]] \
    && is_true "$replay" \
    && [[ -z "$ROSBAG" ]]; then
    die "preset '$PRESET' requires --bag PATH"
  fi
  if is_true "$(get_arg enable_raceline_publisher)" \
    && [[ -z "$(get_arg raceline_csv 2>/dev/null || true)" ]]; then
    die "preset '$PRESET' requires --raceline PATH"
  fi
  if is_true "$(get_arg enable_custom_trajectory_publisher)" \
    && [[ -z "$(get_arg custom_csv 2>/dev/null || true)" ]]; then
    die "preset '$PRESET' requires --custom-line PATH"
  fi
  if [[ "$REQUIRES_CUSTOM_LINE" == 'true' && -z "$MAP_DIR" ]]; then
    die "custom-line component requires --map PATH"
  fi
  if is_true "$(get_arg enable_raceline_publisher)" \
    && is_true "$(get_arg enable_custom_trajectory_publisher)"; then
    die '--raceline and --custom-line are mutually exclusive'
  fi
  if is_true "$(get_arg enable_sensor_kit)" \
    && is_true "$(get_arg sensor_kit_enable_rtp_stream 2>/dev/null || true)"; then
    [[ -n "$(get_arg sensor_kit_rtp_host 2>/dev/null || true)" ]] \
      || die 'sensor_kit_enable_rtp_stream=true requires sensor_kit_rtp_host'
    is_valid_port "$(get_arg sensor_kit_rtp_port 2>/dev/null || true)" \
      || die 'sensor_kit_rtp_port must be an integer between 1 and 65535'
  fi
  if is_true "$(get_arg enable_foxglove)"; then
    [[ -n "$(get_arg foxglove_address 2>/dev/null || true)" ]] \
      || die 'foxglove_address must not be empty'
    is_valid_port "$(get_arg foxglove_port 2>/dev/null || true)" \
      || die 'foxglove_port must be an integer between 1 and 65535'
    [[ -n "$(get_arg foxglove_topic_whitelist 2>/dev/null || true)" ]] \
      || die 'foxglove_topic_whitelist must not be empty'
    [[ -n "$(get_arg foxglove_client_topic_whitelist 2>/dev/null || true)" ]] \
      || die 'foxglove_client_topic_whitelist must not be empty'
  fi
  if [[ -n "$MAP_DIR" ]]; then
    [[ "$DRY_RUN" == 'true' || -d "$MAP_DIR" ]] \
      || die "map directory does not exist: $MAP_DIR"
    if is_true "$origin_startup" || is_true "$foxglove_startup"; then
      [[ "$DRY_RUN" == 'true' || -d "${MAP_DIR%/}/cuvslam_map" ]] \
        || die "${LOCALIZATION_INIT_MODE} initialization requires a saved cuVSLAM map: ${MAP_DIR%/}/cuvslam_map"
      if [[ "$DRY_RUN" != 'true' ]]; then
        has_cuvslam_database "${MAP_DIR%/}/cuvslam_map" \
          || die "${LOCALIZATION_INIT_MODE} initialization requires a cuVSLAM .mdb database in: ${MAP_DIR%/}/cuvslam_map"
      fi
    fi
    set_arg map_dir "$MAP_DIR"
  fi
  if is_true "$(get_arg enable_competition_planning)"; then
    [[ -n "$MAP_DIR" ]] || die 'competition planning requires --map PATH'
    is_true "$(get_arg enable_hd_map_publisher)" \
      || die 'competition planning requires enable_hd_map_publisher=true'
    is_true "$(get_arg enable_section_localizer)" \
      || die 'competition planning requires enable_section_localizer=true'
    local competition_route_config
    competition_route_config="$(get_arg competition_route_config_file 2>/dev/null || true)"
    if [[ -z "$competition_route_config" ]]; then
      competition_route_config="${MAP_DIR%/}/competition_route.param.yaml"
      set_arg competition_route_config_file "$competition_route_config"
    fi
    [[ "$DRY_RUN" == 'true' || -f "$competition_route_config" ]] \
      || die "competition route config does not exist: $competition_route_config"
  fi
  if [[ -n "$ROSBAG" ]]; then
    [[ "$DRY_RUN" == 'true' || -f "${ROSBAG%/}/metadata.yaml" ]] \
      || die "rosbag metadata.yaml does not exist: $ROSBAG"
    set_arg rosbag "$ROSBAG"
    set_arg replay_rate "$REPLAY_RATE"
  fi

  if is_true "$replay" && [[ -z "$(get_arg rosbag)" ]]; then
    die 'enable_rosbag_replay=true requires a rosbag path'
  fi

  if is_true "$vehicle"; then
    local driver_param
    [[ "$VEHICLE_BACKEND" != 'none' ]] \
      || die 'enable_vehicle=true requires --vehicle PROFILE'
    driver_param="$(get_arg vehicle_driver_param)"
    [[ "$DRY_RUN" == 'true' || -f "$driver_param" ]] \
      || die "vehicle driver parameter file does not exist: $driver_param"
  fi
  if is_true "$(get_arg enable_rviz)" \
    && [[ -n "$(get_arg rviz_config_file 2>/dev/null || true)" ]]; then
    local rviz_config_file
    rviz_config_file="$(get_arg rviz_config_file)"
    [[ "$DRY_RUN" == 'true' || -f "$rviz_config_file" ]] \
      || die "RViz config file does not exist: $rviz_config_file"
  fi
}

ensure_ros_environment() {
  local package
  local packages=("$LAUNCH_PACKAGE")

  if ! command -v ros2 >/dev/null 2>&1 \
    || ! ros2 pkg prefix "$LAUNCH_PACKAGE" >/dev/null 2>&1; then
    [[ -f "$ROS2_SETUP_FILE" ]] \
      || die "ROS 2 environment is unavailable: $ROS2_SETUP_FILE"
    printf 'Sourcing ROS 2 workspace: %s\n' "$ROS2_SETUP_FILE"
    # shellcheck disable=SC1090
    set +u
    source "$ROS2_SETUP_FILE"
    set -u
  fi

  command -v ros2 >/dev/null 2>&1 || die 'ros2 command was not found'
  if is_true "$(get_arg enable_vehicle)"; then
    packages+=("$(get_arg vehicle_interface_pkg)")
  fi
  if is_true "$(get_arg enable_sensor_kit)"; then
    packages+=(
      "$(get_arg sensor_kit_interface_pkg 2>/dev/null || printf 'jetpilot_system_launch')"
    )
  fi
  if is_true "$(get_arg enable_localization)"; then
    packages+=(jetpilot_localization_manager)
  fi
  if is_true "$(get_arg enable_foxglove)"; then
    packages+=(foxglove_bridge)
  fi
  if is_true "$(get_arg enable_planning)"; then
    packages+=(jetpilot_planning)
  fi
  if is_true "$(get_arg enable_competition_planning)"; then
    packages+=(jetpilot_planning_manager)
  fi
  if is_true "$(get_arg enable_control)"; then
    packages+=(jetpilot_controller)
  fi
  if is_true "$(get_arg enable_object_detection)"; then
    packages+=(jetpilot_object_detection)
  fi
  if is_true "$(get_arg enable_e2e_inference)"; then
    packages+=(jetpilot_e2e_inference)
  fi
  for package in "${packages[@]}"; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 \
      || die "$package is unavailable; build the workspace first"
  done
}

print_summary() {
  local command=(ros2 launch "$LAUNCH_PACKAGE" bringup.launch.py)
  local displayed_backend="$VEHICLE_BACKEND"
  local index
  local value

  if ! is_true "$(get_arg enable_vehicle)"; then
    displayed_backend='none'
  fi

  for index in "${!ARG_NAMES[@]}"; do
    value="${ARG_VALUES[$index]}"
    [[ -n "$value" ]] || continue
    command+=("${ARG_NAMES[$index]}:=${value}")
  done

  printf '\nJetPilot bringup\n'
  printf '  preset       : %s\n' "$PRESET"
  printf '  vehicle      : %s\n' "$displayed_backend"
  printf '  sensor       : %s\n' "$(get_arg enable_sensor_kit)"
  if is_true "$(get_arg enable_sensor_kit)"; then
    printf '  sensor kit   : %s\n' "${SENSOR_KIT_PROFILE:-default}"
    printf '  sensor launch: %s/%s\n' \
      "$(get_arg sensor_kit_interface_pkg 2>/dev/null || printf 'jetpilot_system_launch')" \
      "$(get_arg sensor_kit_interface_launch 2>/dev/null || printf 'launch/sensors/realsense.launch.py')"
    if is_true "$(get_arg sensor_kit_enable_rtp_stream 2>/dev/null || true)"; then
      printf '  RTP topic    : %s\n' \
        "$(get_arg sensor_kit_rtp_image_topic 2>/dev/null || printf '/realsense/color/image_raw')"
      printf '  RTP receiver : %s:%s\n' \
        "$(get_arg sensor_kit_rtp_host)" \
        "$(get_arg sensor_kit_rtp_port 2>/dev/null || printf '5004')"
    fi
  fi
  printf '  localization : %s\n' "$(get_arg enable_localization)"
  if is_true "$(get_arg enable_localization)"; then
    printf '  VSLAM mode   : %s\n' "$(get_arg vslam_mode)"
    case "$LOCALIZATION_INIT_MODE" in
      map-origin)
        printf '  VSLAM init   : map-origin (VGL off; restart with pose-hint on failure)\n'
        ;;
      foxglove)
        printf '  VSLAM init   : foxglove (/initialpose required; VGL off)\n'
        ;;
      *) printf '  VSLAM init   : pose-hint\n' ;;
    esac
  fi
  if is_true "$(get_arg enable_foxglove)"; then
    printf '  Foxglove     : bind %s:%s\n' \
      "$(get_arg foxglove_address)" "$(get_arg foxglove_port)"
  else
    printf '  Foxglove     : disabled\n'
  fi
  printf '  tool/teleop  : %s / %s\n' \
    "$(get_arg enable_tool)" "$(get_arg enable_teleop)"
  printf '  operation    : %s\n' "$(get_arg enable_operation)"
  printf '  planning     : %s\n' "$(get_arg enable_planning)"
  printf '  competition  : %s\n' "$(get_arg enable_competition_planning)"
  printf '  raceline     : %s\n' "${RACELINE_CSV:-none}"
  printf '  custom line  : %s\n' "${CUSTOM_LINE_CSV:-none}"
  printf '  control      : %s\n' "$(get_arg enable_control)"
  printf '  E2E inference: %s\n' "$(get_arg enable_e2e_inference)"
  printf '  map          : %s\n' "${MAP_DIR:-none}"
  printf '  rosbag       : %s\n' "${ROSBAG:-none}"
  if is_true "$(get_arg enable_rviz)"; then
    printf '  rviz config  : %s\n' \
      "$(get_arg rviz_config_file 2>/dev/null || printf 'default')"
  fi
  printf '\nCommand:\n  '
  printf '%q ' "${command[@]}"
  printf '\n\n'

  LAUNCH_COMMAND=("${command[@]}")
}

while (($# > 0)); do
  case "$1" in
    -p|--preset)
      (($# >= 2)) || die "$1 requires a preset name"
      PRESET="$2"
      shift 2
      ;;
    --preset=*) PRESET="${1#*=}"; shift ;;
    --list-presets) print_presets; exit 0 ;;
    --list-vehicles) print_profile_choices vehicle; exit 0 ;;
    --list-sensor-kits) print_profile_choices sensor_kit; exit 0 ;;
    --validate-profiles) profile_command validate; exit 0 ;;
    --map)
      (($# >= 2)) || die '--map requires a path'
      MAP_DIR="$2"
      shift 2
      ;;
    --map=*) MAP_DIR="${1#*=}"; shift ;;
    --bag)
      (($# >= 2)) || die '--bag requires a path'
      ROSBAG="$2"
      shift 2
      ;;
    --bag=*) ROSBAG="${1#*=}"; shift ;;
    --raceline)
      (($# >= 2)) || die '--raceline requires a path'
      RACELINE_CSV="$2"
      shift 2
      ;;
    --raceline=*) RACELINE_CSV="${1#*=}"; shift ;;
    --custom-line)
      (($# >= 2)) || die '--custom-line requires a path'
      CUSTOM_LINE_CSV="$2"
      shift 2
      ;;
    --custom-line=*) CUSTOM_LINE_CSV="${1#*=}"; shift ;;
    --custom-line-id)
      (($# >= 2)) || die '--custom-line-id requires a name'
      CUSTOM_LINE_ID="$2"
      CUSTOM_LINE_ID_EXPLICIT=true
      shift 2
      ;;
    --custom-line-id=*)
      CUSTOM_LINE_ID="${1#*=}"
      CUSTOM_LINE_ID_EXPLICIT=true
      shift
      ;;
    --custom-line-name)
      (($# >= 2)) || die '--custom-line-name requires a name'
      CUSTOM_LINE_NAME="$2"
      CUSTOM_LINE_NAME_EXPLICIT=true
      shift 2
      ;;
    --custom-line-name=*)
      CUSTOM_LINE_NAME="${1#*=}"
      CUSTOM_LINE_NAME_EXPLICIT=true
      shift
      ;;
    --custom-line-open)
      CUSTOM_LINE_CLOSED=false
      CUSTOM_LINE_CLOSED_EXPLICIT=true
      shift
      ;;
    --custom-line-closed)
      CUSTOM_LINE_CLOSED=true
      CUSTOM_LINE_CLOSED_EXPLICIT=true
      shift
      ;;
    --rate)
      (($# >= 2)) || die '--rate requires a value'
      REPLAY_RATE="$2"
      shift 2
      ;;
    --rate=*) REPLAY_RATE="${1#*=}"; shift ;;
    --vehicle)
      (($# >= 2)) || die '--vehicle requires a profile ID'
      CLI_VEHICLE="$2"
      shift 2
      ;;
    --vehicle=*) CLI_VEHICLE="${1#*=}"; shift ;;
    --bag-manager)
      CLI_BAG_MANAGER=true
      shift
      ;;
    --no-bag-manager)
      CLI_BAG_MANAGER=false
      shift
      ;;
    --sensor-kit)
      (($# >= 2)) || die '--sensor-kit requires a profile ID'
      CLI_SENSOR_KIT="$2"
      shift 2
      ;;
    --sensor-kit=*) CLI_SENSOR_KIT="${1#*=}"; shift ;;
    --rviz-config)
      (($# >= 2)) || die '--rviz-config requires default, vslam-debug, or an absolute path'
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    --rviz-config=*) RVIZ_CONFIG="${1#*=}"; shift ;;
    --localization-init)
      (($# >= 2)) || die '--localization-init requires pose-hint, foxglove, or map-origin'
      CLI_LOCALIZATION_INIT="$2"
      shift 2
      ;;
    --localization-init=*) CLI_LOCALIZATION_INIT="${1#*=}"; shift ;;
    --vslam-mode)
      (($# >= 2)) || die '--vslam-mode requires vo or vio'
      CLI_VSLAM_MODE="$2"
      shift 2
      ;;
    --vslam-mode=*) CLI_VSLAM_MODE="${1#*=}"; shift ;;
    --pose-hint) CLI_LOCALIZATION_INIT='pose-hint'; shift ;;
    --no-pose-hint|--map-origin) CLI_LOCALIZATION_INIT='map-origin'; shift ;;
    --components)
      (($# >= 2)) || die '--components requires a comma-separated list'
      CUSTOM_COMPONENTS="$2"
      shift 2
      ;;
    --components=*) CUSTOM_COMPONENTS="${1#*=}"; shift ;;
    --set)
      (($# >= 2)) || die '--set requires NAME:=VALUE'
      EXTRA_LAUNCH_ARGS+=("$2")
      shift 2
      ;;
    --dry-run) DRY_RUN=true; shift ;;
    -y|--yes) ASSUME_YES=true; shift ;;
    -h|--help) print_usage; exit 0 ;;
    --)
      shift
      while (($# > 0)); do
        EXTRA_LAUNCH_ARGS+=("$1")
        shift
      done
      ;;
    *':='*) EXTRA_LAUNCH_ARGS+=("$1"); shift ;;
    *)
      if [[ -z "$PRESET" ]] && known_preset "$1"; then
        PRESET="$1"
        shift
      else
        die "unknown option or preset: $1"
      fi
      ;;
  esac
done

set_base_args
if [[ -z "$PRESET" ]]; then
  [[ -t 0 && -t 1 ]] || die 'use --preset in a non-interactive terminal'
  INTERACTIVE=true
  choose_preset_interactively
fi
known_preset "$PRESET" || die "unknown preset: $PRESET (use --list-presets)"
apply_preset "$PRESET"

if [[ -n "$CLI_LOCALIZATION_INIT" ]]; then
  set_localization_init_mode "$CLI_LOCALIZATION_INIT"
fi

if [[ "$PRESET" == 'custom' ]]; then
  if [[ -n "$CUSTOM_COMPONENTS" ]]; then
    apply_custom_components "$CUSTOM_COMPONENTS"
    if [[ -t 0 && -t 1 ]]; then
      INTERACTIVE=true
    fi
  else
    [[ -t 0 && -t 1 ]] || die 'custom preset requires an interactive terminal or --components'
    INTERACTIVE=true
    interactive_custom
  fi
fi
if [[ -n "${CLI_VEHICLE:-}" ]]; then
  apply_vehicle "$CLI_VEHICLE"
fi
if [[ "$INTERACTIVE" == 'true' && -z "${CLI_VEHICLE:-}" ]] \
  && [[ "$REQUIRES_VEHICLE" == 'true' ]]; then
  configure_vehicle_interactively
fi
if [[ "$INTERACTIVE" == 'true' && -z "$CLI_SENSOR_KIT" ]] \
  && is_true "$(get_arg enable_sensor_kit)"; then
  configure_sensor_kit_interactively
fi
if [[ -n "$CLI_SENSOR_KIT" ]]; then
  apply_sensor_kit "$CLI_SENSOR_KIT"
fi
if [[ "$INTERACTIVE" == 'true' && "$PRESET" != 'custom' && -z "$CLI_BAG_MANAGER" ]]; then
  configure_bag_manager_interactively
fi
if [[ -n "$CLI_BAG_MANAGER" ]]; then
  if is_true "$CLI_BAG_MANAGER"; then
    set_arg enable_tool true
    set_arg enable_bag_manager true
  else
    set_arg enable_bag_manager false
  fi
fi
if [[ "$INTERACTIVE" == 'true' && -z "$RVIZ_CONFIG" ]] \
  && is_true "$(get_arg enable_rviz)"; then
  configure_rviz_interactively
fi
if [[ -n "$RVIZ_CONFIG" ]]; then
  resolved_rviz_config="$(resolve_rviz_config "$RVIZ_CONFIG")" || exit $?
  set_arg rviz_config_file "$resolved_rviz_config"
fi

if [[ "$REQUIRES_MAP" == 'true' && -z "$MAP_DIR" && "$INTERACTIVE" == 'true' ]]; then
  discover_map
fi
if [[ "$REQUIRES_ROSBAG" == 'true' && -z "$ROSBAG" && "$INTERACTIVE" == 'true' ]]; then
  discover_rosbag
fi
if [[ "$REQUIRES_RACELINE" == 'true' && -z "$RACELINE_CSV" && "$INTERACTIVE" == 'true' ]]; then
  discover_raceline
fi
if [[ "$REQUIRES_CUSTOM_LINE" == 'true' && -z "$CUSTOM_LINE_CSV" \
  && -z "$RACELINE_CSV" ]]; then
  select_active_custom_line_from_map
fi
if [[ "$REQUIRES_CUSTOM_LINE" == 'true' && -z "$CUSTOM_LINE_CSV" \
  && "$INTERACTIVE" == 'true' ]]; then
  discover_custom_line
fi

if [[ -n "$RACELINE_CSV" && -n "$CUSTOM_LINE_CSV" ]]; then
  die '--raceline and --custom-line are mutually exclusive'
fi

if [[ -n "$RACELINE_CSV" ]]; then
  configure_raceline
fi
if [[ -n "$CUSTOM_LINE_CSV" ]]; then
  configure_custom_line
fi

if ((${#EXTRA_LAUNCH_ARGS[@]} > 0)); then
  for override in "${EXTRA_LAUNCH_ARGS[@]}"; do
    parse_override "$override"
  done
fi
normalize_localization_init_mode
normalize_vslam_mode
validate_configuration
print_summary

if [[ "$DRY_RUN" == 'true' ]]; then
  printf 'Dry-run: command was not executed.\n'
  exit 0
fi

if is_true "$(get_arg enable_vehicle)" || [[ "$INTERACTIVE" == 'true' ]]; then
  if [[ "$ASSUME_YES" != 'true' ]]; then
    [[ -t 0 && -t 1 ]] \
      || die 'hardware launch confirmation requires a TTY or --yes'
    prompt_yes_no 'この設定で起動しますか？' no || {
      printf 'Canceled.\n'
      exit 0
    }
  fi
fi

ensure_ros_environment
printf 'Starting JetPilot bringup...\n'
exec "${LAUNCH_COMMAND[@]}"
