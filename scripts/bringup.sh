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
RACELINE_CSV="${BRINGUP_RACELINE_CSV:-}"
RVIZ_CONFIG="${BRINGUP_RVIZ_CONFIG:-}"
REPLAY_RATE='1.0'
DRY_RUN=false
ASSUME_YES=false
INTERACTIVE=false
CLI_BAG_MANAGER=''
CLI_SENSOR_KIT=''
REQUIRES_MAP=false
REQUIRES_ROSBAG=false
REQUIRES_RACELINE=false
REQUIRES_VEHICLE=false
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

is_valid_udp_port() {
  local port="${1:-}"

  [[ "$port" =~ ^[0-9]{1,5}$ ]] && ((10#$port >= 1 && 10#$port <= 65535))
}

print_presets() {
  cat <<'EOF'
sensor               Sensor kit + camera TF only (no actuator)
localization-only    Localization stack; camera input is already running
localization         Sensor kit + localization + RViz (map required)
localize-live        Sensor kit + localization + RViz (alias of localization)
replay-localization  Safe rosbag replay + localization + RViz (bag/map required)
offline-vslam        Rosbag replay + VSLAM visualization + RViz (bag required)
offline-vslam-map    Rosbag replay + VSLAM mapping debug + RViz (bag/map required)
offline-localization Rosbag replay + VGL/VSLAM localization + RViz (bag/map required)
vehicle              Selected vehicle interface only
teleop               Joy/teleop/operation + selected vehicle interface
drive                Live sensor + joy/teleop/operation + selected vehicle interface
runtime              Live sensor/localization/teleop + selected vehicle interface (map required)
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
      --raceline PATH  Enable the C++ raceline loader with this generated CSV
      --rate RATE      Rosbag replay rate (default: 1.0)
      --vehicle TYPE   Override vehicle backend: none, pca, vesc
      --bag-manager    Enable bag manager recording control
      --no-bag-manager Disable bag manager recording control
      --sensor-kit NAME
                        Select sensor kit: realsense, flir, realsense-silky,
                        realsense-silky-flir
      --rviz-config NAME_OR_PATH
                        Select RViz config: default, vslam-debug, or absolute path
      --components LIST
                        Custom component list, e.g. sensor,joy,teleop,vehicle
      --set ARG:=VALUE Override one bringup launch argument
      --dry-run        Print the exact command without running ROS
  -y, --yes            Skip the hardware launch confirmation
  -h, --help           Show this help
  -- ARG:=VALUE ...    Additional overrides (merged; duplicate names are replaced)

Examples:
  $(basename "$0") --preset vehicle --vehicle pca
  $(basename "$0") --preset drive --vehicle vesc
  $(basename "$0") --preset localization --map /workspaces/map/course_a
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
      vehicle|teleop|drive|runtime|\
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

  set_arg enable_tool true
  set_arg enable_bag_manager false
  set_arg enable_joy false
  set_arg enable_teleop false
  set_arg enable_rc_serial false
  set_arg enable_jetson_stats true
  set_arg enable_vslam_snapshot false
  set_arg enable_operation false
  set_arg enable_planning false
  set_arg enable_raceline_publisher false
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

raceline_selector_param() {
  first_existing_path \
    "${ROS2_WS}/src/planning/jetpilot_planning/config/route_lane_selector.raceline.param.yaml" \
    "${ROS2_WS}/src/planning/jetpilot_planning/config/route_lane_selector.raceline.param.yaml" \
    "${PROJECT_ROOT}/ros2_ws/src/planning/jetpilot_planning/config/route_lane_selector.raceline.param.yaml" \
    "${ROS2_WS}/install/jetpilot_planning/share/jetpilot_planning/config/route_lane_selector.raceline.param.yaml"
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

apply_vehicle() {
  local backend="$1"
  VEHICLE_BACKEND="$backend"
  case "$backend" in
    none)
      set_arg enable_vehicle false
      set_arg publish_vehicle_description false
      set_arg publish_vehicle_evs_description false
      set_arg publish_vehicle_thremo_description false
      ;;
    pca)
      set_arg enable_vehicle true
      set_arg publish_vehicle_description true
      set_arg publish_vehicle_evs_description false
      set_arg publish_vehicle_thremo_description false
      set_arg vehicle_interface_pkg pca9685_rc_driver
      set_arg vehicle_interface_launch launch/pca9685_rc_interface.launch.xml
      set_arg vehicle_driver_param "$(pca_driver_param)"
      ;;
    vesc)
      set_arg enable_vehicle true
      set_arg publish_vehicle_description true
      set_arg publish_vehicle_evs_description true
      set_arg publish_vehicle_thremo_description true
      set_arg vehicle_interface_pkg jetpilot_vesc_interface
      set_arg vehicle_interface_launch launch/vesc_interface.launch.xml
      set_arg vehicle_driver_param "$(vesc_driver_param)"
      ;;
    *) die "vehicle backend must be none, pca, or vesc: $backend" ;;
  esac
}

apply_sensor_kit() {
  local sensor_kit="$1"

  case "$sensor_kit" in
    realsense)
      set_arg sensor_kit_interface_pkg jetpilot_system_launch
      set_arg sensor_kit_interface_launch launch/sensors/realsense.launch.py
      set_arg sensor_kit_camera_name realsense
      set_arg sensor_kit_rtp_image_topic /realsense/color/image_raw
      ;;
    flir|boson|flir-boson|flir_boson)
      set_arg sensor_kit_interface_pkg jetpilot_system_launch
      set_arg sensor_kit_interface_launch launch/sensors/flir_boson.launch.py
      set_arg sensor_kit_camera_name realsense
      set_arg sensor_kit_rtp_image_topic /flir/image_raw
      set_arg sensor_kit_enable_flir true
      set_arg sensor_kit_flir_namespace flir
      set_arg sensor_kit_flir_node_name boson
      set_arg sensor_kit_flir_camera_name boson
      set_arg sensor_kit_flir_frame_id boson_optical_frame
      set_arg sensor_kit_flir_video_device /dev/video0
      set_arg sensor_kit_flir_pixel_format mono16
      set_arg sensor_kit_flir_image_width 640
      set_arg sensor_kit_flir_image_height 512
      set_arg sensor_kit_flir_framerate 60.0
      set_arg sensor_kit_flir_io_method mmap
      ;;
    realsense-silky|realsense_silky|silky)
      set_arg sensor_kit_interface_pkg jetpilot_system_launch
      set_arg sensor_kit_interface_launch launch/sensors/realsense_silky_evcam.launch.py
      set_arg sensor_kit_camera_name realsense
      set_arg sensor_kit_rtp_image_topic /realsense/color/image_raw
      ;;
    realsense-silky-flir|realsense_silky_flir|all)
      set_arg sensor_kit_interface_pkg jetpilot_system_launch
      set_arg sensor_kit_interface_launch launch/sensors/realsense_silky_flir.launch.py
      set_arg sensor_kit_camera_name realsense
      set_arg sensor_kit_rtp_image_topic /realsense/color/image_raw
      set_arg sensor_kit_enable_flir true
      set_arg sensor_kit_flir_namespace flir
      set_arg sensor_kit_flir_node_name boson
      set_arg sensor_kit_flir_camera_name boson
      set_arg sensor_kit_flir_frame_id boson_optical_frame
      set_arg sensor_kit_flir_video_device /dev/video0
      set_arg sensor_kit_flir_pixel_format mono16
      set_arg sensor_kit_flir_image_width 640
      set_arg sensor_kit_flir_image_height 512
      set_arg sensor_kit_flir_framerate 60.0
      set_arg sensor_kit_flir_io_method mmap
      ;;
    *) die "sensor kit must be realsense, flir, realsense-silky, or realsense-silky-flir: $sensor_kit" ;;
  esac
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
      set_arg vslam_enable_slam false
      set_arg vslam_enable_visualization true
      set_arg vslam_localize_on_startup true
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
    runtime)
      enable_teleop_stack
      set_arg enable_sensor_kit true
      enable_localization_stack
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
    raceline)
      set_arg enable_planning true
      set_arg enable_raceline_publisher true
      REQUIRES_RACELINE=true
      ;;
    control|autonomous-control)
      set_arg enable_planning true
      set_arg enable_control true
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
    'planning           Route/lane planning only'
    'raceline           Planning with generated raceline CSV'
    'control            Planning + Pure Pursuit control'
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
  local sensor_launch
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
    if is_valid_udp_port "$port"; then
      break
    fi
    printf 'UDP portは1〜65535の整数で入力してください。\n' >&2
  done
  set_arg sensor_kit_rtp_port "$port"

  options=()
  topic="$(get_arg sensor_kit_rtp_image_topic 2>/dev/null || true)"
  append_unique_option "$topic"
  sensor_launch="$(get_arg sensor_kit_interface_launch 2>/dev/null || true)"
  case "$sensor_launch" in
    *realsense_silky_flir.launch.py)
      append_unique_option '/realsense/color/image_raw'
      append_unique_option '/event_camera/event_image'
      append_unique_option '/flir/image_raw'
      append_unique_option '/realsense/infra1/image_rect_raw'
      append_unique_option '/realsense/infra2/image_rect_raw'
      ;;
    *realsense_silky_evcam.launch.py)
      append_unique_option '/realsense/color/image_raw'
      append_unique_option '/event_camera/event_image'
      append_unique_option '/realsense/infra1/image_rect_raw'
      append_unique_option '/realsense/infra2/image_rect_raw'
      ;;
    *flir_boson.launch.py)
      append_unique_option '/flir/image_raw'
      ;;
    *)
      append_unique_option '/realsense/color/image_raw'
      append_unique_option '/realsense/infra1/image_rect_raw'
      append_unique_option '/realsense/infra2/image_rect_raw'
      ;;
  esac
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
  local current_launch
  local options=()

  current_launch="$(get_arg sensor_kit_interface_launch 2>/dev/null || true)"
  case "$current_launch" in
    *realsense_silky_flir.launch.py)
      options+=('realsense-silky-flir  RealSense + SilkyEvCam/OpenEB + FLIR Boson')
      options+=('realsense-silky       RealSense + SilkyEvCam/OpenEB')
      options+=('flir                  FLIR Boson')
      options+=('realsense             RealSense')
      ;;
    *realsense_silky_evcam.launch.py)
      options+=('realsense-silky  RealSense + SilkyEvCam/OpenEB')
      options+=('realsense-silky-flir  RealSense + SilkyEvCam/OpenEB + FLIR Boson')
      options+=('flir             FLIR Boson')
      options+=('realsense        RealSense')
      ;;
    *flir_boson.launch.py)
      options+=('flir                  FLIR Boson')
      options+=('realsense-silky-flir  RealSense + SilkyEvCam/OpenEB + FLIR Boson')
      options+=('realsense-silky       RealSense + SilkyEvCam/OpenEB')
      options+=('realsense             RealSense')
      ;;
    *)
      options+=('realsense        RealSense')
      options+=('realsense-silky  RealSense + SilkyEvCam/OpenEB')
      options+=('realsense-silky-flir  RealSense + SilkyEvCam/OpenEB + FLIR Boson')
      options+=('flir             FLIR Boson')
      ;;
  esac

  selection="$(choose_one 'Sensor kit launch' "${options[@]}")" || exit $?
  apply_sensor_kit "${selection%%[[:space:]]*}"
  configure_rtp_interactively
}

configure_vehicle_interactively() {
  local selection
  local options=(
    'pca   PCA9685 RC vehicle interface'
    'vesc  VESC vehicle interface'
  )

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
  if [[ "$REQUIRES_VEHICLE" == 'true' ]] && ! is_true "$vehicle"; then
    die "preset '$PRESET' requires --vehicle pca or --vehicle vesc"
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
  if is_true "$(get_arg enable_raceline_publisher)" \
    && [[ -z "$(get_arg raceline_csv 2>/dev/null || true)" ]]; then
    die "preset '$PRESET' requires --raceline PATH"
  fi
  if is_true "$(get_arg enable_sensor_kit)" \
    && is_true "$(get_arg sensor_kit_enable_rtp_stream 2>/dev/null || true)"; then
    [[ -n "$(get_arg sensor_kit_rtp_host 2>/dev/null || true)" ]] \
      || die 'sensor_kit_enable_rtp_stream=true requires sensor_kit_rtp_host'
    is_valid_udp_port "$(get_arg sensor_kit_rtp_port 2>/dev/null || true)" \
      || die 'sensor_kit_rtp_port must be an integer between 1 and 65535'
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
  if is_true "$(get_arg enable_localization)"; then
    packages+=(jetpilot_localization_manager)
  fi
  if is_true "$(get_arg enable_planning)"; then
    packages+=(jetpilot_planning)
  fi
  if is_true "$(get_arg enable_control)"; then
    packages+=(jetpilot_controller)
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
  printf '  tool/teleop  : %s / %s\n' \
    "$(get_arg enable_tool)" "$(get_arg enable_teleop)"
  printf '  operation    : %s\n' "$(get_arg enable_operation)"
  printf '  planning     : %s\n' "$(get_arg enable_planning)"
  printf '  raceline     : %s\n' "${RACELINE_CSV:-none}"
  printf '  control      : %s\n' "$(get_arg enable_control)"
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
    --bag-manager)
      CLI_BAG_MANAGER=true
      shift
      ;;
    --no-bag-manager)
      CLI_BAG_MANAGER=false
      shift
      ;;
    --sensor-kit)
      (($# >= 2)) || die '--sensor-kit requires realsense or realsense-silky'
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

if [[ -n "$RACELINE_CSV" ]]; then
  configure_raceline
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
