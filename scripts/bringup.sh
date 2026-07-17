#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname -- "$SCRIPT_DIR")"
ROS2_WS="${ROS2_WS:-/workspaces/ros2_ws}"
ROS2_SETUP_FILE="${ROS2_SETUP_FILE:-${ROS2_WS}/install/setup.bash}"
MAP_ROOT="${MAP_ROOT:-/workspaces/map}"
RECORD_ROOT="${RECORD_ROOT:-/workspaces/record}"
LAUNCH_PACKAGE="${JETPILOT_LAUNCH_PACKAGE:-jetpilot_system_launch}"

PRESET=''
VEHICLE_BACKEND='none'
MAP_DIR="${BRINGUP_MAP_DIR:-}"
ROSBAG="${BRINGUP_ROSBAG:-}"
REPLAY_RATE='1.0'
DRY_RUN=false
ASSUME_YES=false
INTERACTIVE=false
REQUIRES_MAP=false
REQUIRES_ROSBAG=false
ARG_NAMES=()
ARG_VALUES=()
EXTRA_LAUNCH_ARGS=()
CUSTOM_COMPONENTS=''

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

print_presets() {
  cat <<'EOF'
sensor               Sensor kit + camera TF only (no actuator)
localization-only    Localization stack; camera input is already running
localization         Sensor kit + localization + RViz (map required)
localize-live        Sensor kit + localization + RViz (alias of localization)
replay-localization  Safe rosbag replay + localization + RViz (bag/map required)
vehicle-pca          PCA9685 vehicle interface only
vehicle-vesc         VESC vehicle interface only
teleop-pca           Joy/teleop/operation + PCA9685 vehicle
teleop-vesc          Joy/teleop/operation + VESC vehicle
drive-pca            Live sensor + joy/teleop/operation + PCA9685 vehicle
drive-vesc           Live sensor + joy/teleop/operation + VESC vehicle
runtime-pca          Live sensor/localization/teleop + PCA9685 vehicle (map required)
runtime-vesc         Live sensor/localization/teleop + VESC vehicle (map required)
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
      --map PATH       Set map_dir (or BRINGUP_MAP_DIR)
      --bag PATH       Set rosbag directory/metadata.yaml (or BRINGUP_ROSBAG)
      --rate RATE      Rosbag replay rate (default: 1.0)
      --vehicle TYPE   Override vehicle backend: none, pca, vesc
      --components LIST
                        Custom component list, e.g. sensor,joy,teleop,vehicle-vesc
      --set ARG:=VALUE Override one bringup launch argument
      --dry-run        Print the exact command without running ROS
  -y, --yes            Skip the hardware launch confirmation
  -h, --help           Show this help
  -- ARG:=VALUE ...    Additional overrides (merged; duplicate names are replaced)

Examples:
  $(basename "$0") --preset vehicle-pca
  $(basename "$0") --preset drive-vesc
  $(basename "$0") --preset localization --map /workspaces/map/course_a
  $(basename "$0") replay-localization --bag /workspaces/record/run_01 \\
    --map /workspaces/map/course_a --rate 0.5
  $(basename "$0") runtime-vesc --map /workspaces/map/course_a --dry-run
  $(basename "$0") custom --components sensor,joy,teleop,operation,vehicle-vesc

The launcher starts from explicit all-OFF module settings. Vehicle hardware is
never enabled by a localization/replay preset, and replay + vehicle overrides
are rejected. Direct ros2 launch remains available for intentional HIL tests.
EOF
}

known_preset() {
  case "$1" in
    sensor|localization-only|localization|localize-live|replay-localization|\
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
  set_arg use_sim_time false
  set_arg enable_rosbag_replay false
  set_arg rosbag ''
  set_arg replay_rate 1.0
  set_arg allow_unsafe_replay_control_topics false
  set_arg allow_unsafe_replay_with_vehicle false

  set_arg enable_tool false
  set_arg enable_bag_manager false
  set_arg enable_joy false
  set_arg enable_teleop false
  set_arg enable_rc_serial false
  set_arg enable_vslam_snapshot false
  set_arg enable_operation false
  set_arg enable_control false
  set_arg enable_sensor_kit false
  set_arg enable_localization false
  set_arg enable_vslam true
  set_arg enable_localization_manager true
  set_arg enable_vgl true
  set_arg enable_occupancy_map_server false
  set_arg enable_occupancy_map_lifecycle_manager false
  set_arg enable_omap_frame false
  set_arg enable_hd_map_publisher false
  set_arg enable_section_localizer false
  set_arg enable_vehicle false
  set_arg publish_vehicle_description false
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

pca_driver_param() {
  first_existing_path \
    "${ROS2_WS}/src/vehicle/pca9685_rc_interface/pca9685_rc_driver/config/pca9685_rc_driver_node.param.yaml" \
    "${ROS2_WS}/joy_profiles/pca9685_rc_driver_node.param.yaml" \
    "${PROJECT_ROOT}/ros2_ws/joy_profiles/pca9685_rc_driver_node.param.yaml" \
    "${ROS2_WS}/src/vehicle/pca9685_rc_interface/pca9685_rc_driver/config/pca9685_rc_driver_node.param.yaml" \
    "${PROJECT_ROOT}/ros2_ws/src/vehicle/pca9685_rc_interface/pca9685_rc_driver/config/pca9685_rc_driver_node.param.yaml" \
    "${ROS2_WS}/install/pca9685_rc_driver/share/pca9685_rc_driver/config/pca9685_rc_driver_node.param.yaml"
}

vesc_driver_param() {
  first_existing_path \
    "${ROS2_WS}/src/vehicle/vesc_interface/jetpilot_vesc_interface/config/vesc_interface.param.yaml" \
    "${ROS2_WS}/src/vehicle/vesc_interface/jetpilot_vesc_interface/config/vesc_interface.param.yaml" \
    "${PROJECT_ROOT}/ros2_ws/src/vehicle/vesc_interface/jetpilot_vesc_interface/config/vesc_interface.param.yaml" \
    "${ROS2_WS}/install/jetpilot_vesc_interface/share/jetpilot_vesc_interface/config/vesc_interface.param.yaml"
}

apply_vehicle() {
  local backend="$1"
  VEHICLE_BACKEND="$backend"
  case "$backend" in
    none)
      set_arg enable_vehicle false
      ;;
    pca)
      set_arg enable_vehicle true
      set_arg publish_vehicle_description true
      set_arg vehicle_interface_pkg pca9685_rc_driver
      set_arg vehicle_interface_launch launch/pca9685_rc_interface.launch.xml
      set_arg vehicle_driver_param "$(pca_driver_param)"
      ;;
    vesc)
      set_arg enable_vehicle true
      set_arg publish_vehicle_description true
      set_arg vehicle_interface_pkg jetpilot_vesc_interface
      set_arg vehicle_interface_launch launch/vesc_interface.launch.xml
      set_arg vehicle_driver_param "$(vesc_driver_param)"
      ;;
    *) die "vehicle backend must be none, pca, or vesc: $backend" ;;
  esac
}

enable_teleop_stack() {
  set_arg enable_tool true
  set_arg enable_bag_manager true
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

apply_preset() {
  local preset="$1"

  case "$preset" in
    sensor)
      set_arg enable_sensor_kit true
      set_arg publish_vehicle_description true
      ;;
    localization-only)
      enable_localization_stack
      REQUIRES_MAP=true
      ;;
    localization|localize-live)
      set_arg enable_sensor_kit true
      enable_localization_stack
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
      enable_localization_stack
      apply_vehicle pca
      REQUIRES_MAP=true
      ;;
    runtime-vesc)
      enable_teleop_stack
      set_arg enable_sensor_kit true
      enable_localization_stack
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

prompt_path() {
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

append_unique_option() {
  local candidate="$1"
  local option

  [[ -n "$candidate" ]] || return
  for option in "${options[@]}"; do
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
    control|autonomous-control)
      set_arg enable_control true
      ;;
    rviz)
      set_arg enable_rviz true
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
        vehicle-pca|vehicle-vesc) vehicle_count=$((vehicle_count + 1)) ;;
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
        vehicle-pca|vehicle-vesc) vehicle_count=$((vehicle_count + 1)) ;;
      esac
      apply_custom_component_token "$token"
    done <<< "$selection"
  fi

  if [[ "$saw_live" == 'true' && "$saw_replay" == 'true' ]]; then
    die 'custom components must choose either sensor or replay, not both'
  fi
  if ((vehicle_count > 1)); then
    die 'custom components must choose only one vehicle backend'
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
    'bag-manager        Bag manager'
    'joy                Joy node'
    'teleop             Teleop node'
    'rc-serial          RC serial reader'
    'operation          Operation manager'
    'control            Autonomous control'
    'rviz               RViz'
    'vehicle-pca        PCA9685 vehicle interface'
    'vehicle-vesc       VESC vehicle interface'
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

normalize_rosbag_path() {
  if [[ -f "$ROSBAG" && "$(basename "$ROSBAG")" == 'metadata.yaml' ]]; then
    ROSBAG="$(dirname "$ROSBAG")"
  fi
}

validate_configuration() {
  local configured_path
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
  normalize_rosbag_path

  if is_true "$(get_arg allow_unsafe_replay_control_topics)" \
    || is_true "$(get_arg allow_unsafe_replay_with_vehicle)"; then
    die 'unsafe replay overrides are intentionally unsupported by this launcher'
  fi
  if is_true "$replay" && is_true "$vehicle"; then
    die 'rosbag replay and vehicle hardware cannot be enabled together'
  fi
  if [[ "$REQUIRES_MAP" == 'true' ]] \
    && is_true "$(get_arg enable_localization)" \
    && [[ -z "$MAP_DIR" ]]; then
    die "preset '$PRESET' requires --map PATH"
  fi
  if [[ "$REQUIRES_ROSBAG" == 'true' ]] \
    && is_true "$replay" \
    && [[ -z "$ROSBAG" ]]; then
    die "preset '$PRESET' requires --bag PATH"
  fi
  if [[ -n "$MAP_DIR" ]]; then
    [[ "$DRY_RUN" == 'true' || -d "$MAP_DIR" ]] \
      || die "map directory does not exist: $MAP_DIR"
    set_arg map_dir "$MAP_DIR"
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
      || die 'enable_vehicle=true requires --vehicle pca or --vehicle vesc'
    driver_param="$(get_arg vehicle_driver_param)"
    [[ "$DRY_RUN" == 'true' || -f "$driver_param" ]] \
      || die "vehicle driver parameter file does not exist: $driver_param"
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
  if is_true "$(get_arg enable_localization)"; then
    packages+=(jetpilot_localization_manager)
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
  printf '  localization : %s\n' "$(get_arg enable_localization)"
  printf '  tool/teleop  : %s / %s\n' \
    "$(get_arg enable_tool)" "$(get_arg enable_teleop)"
  printf '  operation    : %s\n' "$(get_arg enable_operation)"
  printf '  control      : %s\n' "$(get_arg enable_control)"
  printf '  map          : %s\n' "${MAP_DIR:-none}"
  printf '  rosbag       : %s\n' "${ROSBAG:-none}"
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
    --rate)
      (($# >= 2)) || die '--rate requires a value'
      REPLAY_RATE="$2"
      shift 2
      ;;
    --rate=*) REPLAY_RATE="${1#*=}"; shift ;;
    --vehicle)
      (($# >= 2)) || die '--vehicle requires none, pca, or vesc'
      CLI_VEHICLE="$2"
      shift 2
      ;;
    --vehicle=*) CLI_VEHICLE="${1#*=}"; shift ;;
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

if [[ "$REQUIRES_MAP" == 'true' && -z "$MAP_DIR" && "$INTERACTIVE" == 'true' ]]; then
  discover_map
fi
if [[ "$REQUIRES_ROSBAG" == 'true' && -z "$ROSBAG" && "$INTERACTIVE" == 'true' ]]; then
  discover_rosbag
fi

if ((${#EXTRA_LAUNCH_ARGS[@]} > 0)); then
  for override in "${EXTRA_LAUNCH_ARGS[@]}"; do
    parse_override "$override"
  done
fi
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
