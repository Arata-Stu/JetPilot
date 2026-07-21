#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${ROS2_WS:-/workspaces/ros2_ws}"
PYTHON_WS="${PYTHON_WS:-$(dirname -- "$ROS2_WS")/python_ws}"
PYTHON_BIN="${PYTHON_BIN:-/opt/env/bin/python}"
JETPILOT_LAUNCH_PACKAGE="${JETPILOT_LAUNCH_PACKAGE:-jetpilot_system_launch}"
RECORD_ROOT="${RECORD_ROOT:-/workspaces/record}"
MAP_ROOT="${MAP_ROOT:-/workspaces/map}"
FOUNDATIONSTEREO_MODEL_RES="${FOUNDATIONSTEREO_MODEL_RES:-low_res}"
OUTPUT_MODEL_DIR="${OUTPUT_MODEL_DIR:-/workspaces/ros2_ws/isaac_ros_assets/models/visual_global_localization}"
CREATE_MAP_OFFLINE_STEPS="${CREATE_MAP_OFFLINE_STEPS:-edex compute_poses cuvgl}"
ALLOW_OCCUPANCY_MAP_STEP="${ALLOW_OCCUPANCY_MAP_STEP:-false}"
ROSBAG_START_DELAY_S="${ROSBAG_START_DELAY_S:-5.0}"
ROSBAG_PLAY_ADDITIONAL_ARGS="${ROSBAG_PLAY_ADDITIONAL_ARGS:---clock}"
ENABLE_OFFLINE_RVIZ="${ENABLE_OFFLINE_RVIZ:-true}"
OFFLINE_RVIZ_CONFIG_FILE="${OFFLINE_RVIZ_CONFIG_FILE:-${ROS2_WS}/install/jetpilot_system_launch/share/jetpilot_system_launch/rviz/vslam_debug.rviz}"
ENABLE_HD_MAP_WORKFLOW="${ENABLE_HD_MAP_WORKFLOW:-ask}"
CAMERA_TOPIC_CONFIG_FILE="${CAMERA_TOPIC_CONFIG_FILE:-}"
VSLAM_LANDMARKS_TOPIC="${VSLAM_LANDMARKS_TOPIC:-/visual_slam/vis/landmarks_cloud}"
VSLAM_SNAPSHOT_WRITE_INTERVAL_S="${VSLAM_SNAPSHOT_WRITE_INTERVAL_S:-5.0}"
JETSON_REMOTE_USER="${JETSON_REMOTE_USER:-tamiya}"
JETSON_REMOTE_IPS="${JETSON_REMOTE_IPS:-10.42.0.1 192.168.55.1 192.168.11.190}"
JETSON_MAP_ROOT="${JETSON_MAP_ROOT:-/home/tamiya/workspaces/JetPilot/map}"
VGL_TENSORRT_EXPORT_SCRIPT="${VGL_TENSORRT_EXPORT_SCRIPT:-${SCRIPT_DIR}/export_vgl_tensorrt_engines.sh}"

die() {
  echo "error: $*" >&2
  exit 1
}

ensure_workspace_overlay() {
  local setup_file="${ROS2_WS}/install/setup.bash"
  local package_name

  if ! command -v ros2 >/dev/null 2>&1; then
    [[ -f "$setup_file" ]] || die "ros2 command was not found and workspace setup is unavailable: $setup_file"
    # shellcheck disable=SC1090
    set +u
    source "$setup_file"
    set -u
  elif ! ros2 pkg prefix "$JETPILOT_LAUNCH_PACKAGE" >/dev/null 2>&1 && [[ -f "$setup_file" ]]; then
    echo "Sourcing workspace overlay: $setup_file"
    # shellcheck disable=SC1090
    set +u
    source "$setup_file"
    set -u
  fi

  command -v ros2 >/dev/null 2>&1 || die "ros2 command was not found"

  for package_name in "$JETPILOT_LAUNCH_PACKAGE" vslam_map_tools; do
    if ! ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
      die "$package_name is not available. Build the workspace and source ${setup_file}"
    fi
  done
}

resolve_camera_topic_config_file() {
  local launch_share
  local topic_config_file

  if [[ -n "$CAMERA_TOPIC_CONFIG_FILE" ]]; then
    [[ -f "$CAMERA_TOPIC_CONFIG_FILE" ]] \
      || die "CAMERA_TOPIC_CONFIG_FILE does not exist: $CAMERA_TOPIC_CONFIG_FILE"
    printf '%s\n' "$CAMERA_TOPIC_CONFIG_FILE"
    return
  fi

  launch_share="$(ros2 pkg prefix --share "$JETPILOT_LAUNCH_PACKAGE")"
  topic_config_file="${launch_share}/config/localization/vgl_camera_topics.yaml"
  [[ -f "$topic_config_file" ]] || die "camera topic config was not found: $topic_config_file"
  printf '%s\n' "$topic_config_file"
}

sanitize_name() {
  local name="$1"
  name="${name// /_}"
  name="${name//[^A-Za-z0-9._-]/_}"
  name="${name##_}"
  name="${name%%_}"
  printf '%s' "${name:-map}"
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

is_true_value() {
  case "$1" in
    true|TRUE|yes|YES|1) return 0 ;;
    *) return 1 ;;
  esac
}

validate_create_map_offline_steps() {
  local step
  local steps=()

  read -r -a steps <<< "$CREATE_MAP_OFFLINE_STEPS"
  [[ ${#steps[@]} -gt 0 ]] || die "CREATE_MAP_OFFLINE_STEPS is empty"
  for step in "${steps[@]}"; do
    [[ "$step" =~ ^[A-Za-z0-9_.-]+$ ]] \
      || die "invalid create_map_offline step name: $step"
    if [[ "$step" =~ [Oo]ccupancy|occupancy_map|occupancy_grid ]] \
      && ! is_true_value "$ALLOW_OCCUPANCY_MAP_STEP"; then
      die "occupancy map generation is disabled. Remove '$step' from CREATE_MAP_OFFLINE_STEPS or set ALLOW_OCCUPANCY_MAP_STEP=true"
    fi
  done
}

select_jetson_ip() {
  local choice
  local index
  local selected
  local ips=()

  read -r -a ips <<< "$JETSON_REMOTE_IPS"
  [[ ${#ips[@]} -gt 0 ]] || die "JETSON_REMOTE_IPS does not contain any addresses"

  echo "Jetson connection:" >&2
  for index in "${!ips[@]}"; do
    case "${ips[$index]}" in
      10.42.0.1) printf '  [%d] %s (hotspot)\n' "$((index + 1))" "${ips[$index]}" >&2 ;;
      192.168.55.1) printf '  [%d] %s (USB)\n' "$((index + 1))" "${ips[$index]}" >&2 ;;
      *) printf '  [%d] %s\n' "$((index + 1))" "${ips[$index]}" >&2 ;;
    esac
  done
  echo >&2

  while true; do
    read -r -p "Select number or enter an IP/hostname [1]: " choice
    choice="${choice:-1}"

    if [[ "$choice" =~ ^[0-9]+$ ]] \
      && (( choice >= 1 && choice <= ${#ips[@]} )); then
      selected="${ips[$((choice - 1))]}"
    else
      selected="$choice"
    fi

    if [[ "$selected" =~ ^[A-Za-z0-9._:-]+$ ]]; then
      printf '%s\n' "$selected"
      return
    fi
    echo "Enter a valid list number, IP address, or hostname." >&2
  done
}

transfer_map_to_jetson() {
  local map_dir="$1"
  local map_name
  local relative_map_dir
  local remote_ip
  local remote_host
  local remote_map_dir
  local remote_parent
  local hd_map_yaml
  local raceline_csv
  local required_path
  local sources=()

  command -v ssh >/dev/null 2>&1 || die "ssh command was not found"
  command -v rsync >/dev/null 2>&1 || die "rsync command was not found"

  map_name="$(basename "$map_dir")"
  hd_map_yaml="${map_dir}/${map_name}_hd_map.yaml"
  raceline_csv="${map_dir}/${map_name}_raceline.csv"

  for required_path in \
    "${map_dir}/cuvgl_map" \
    "${map_dir}/cuvslam_map" \
    "$hd_map_yaml" \
    "$raceline_csv"; do
    [[ -e "$required_path" ]] || die "Jetson runtime map artifact was not found: $required_path"
    sources+=("$required_path")
  done

  if [[ "$map_dir" == "${MAP_ROOT%/}/"* ]]; then
    relative_map_dir="${map_dir#"${MAP_ROOT%/}/"}"
  else
    relative_map_dir="$map_name"
  fi

  [[ "$JETSON_REMOTE_USER" =~ ^[A-Za-z_][A-Za-z0-9_-]*$ ]] \
    || die "Invalid JETSON_REMOTE_USER: $JETSON_REMOTE_USER"
  [[ "$JETSON_MAP_ROOT" == /* && "$JETSON_MAP_ROOT" =~ ^[A-Za-z0-9_./-]+$ ]] \
    || die "JETSON_MAP_ROOT must be a simple absolute path: $JETSON_MAP_ROOT"
  [[ "$relative_map_dir" != *".."* && "$relative_map_dir" =~ ^[A-Za-z0-9_./-]+$ ]] \
    || die "Invalid map path relative to MAP_ROOT: $relative_map_dir"

  remote_ip="$(select_jetson_ip)"
  remote_host="${JETSON_REMOTE_USER}@${remote_ip}"
  remote_map_dir="${JETSON_MAP_ROOT%/}/${relative_map_dir}"
  remote_parent="$(dirname "$remote_map_dir")"

  echo
  echo "================ Jetson map transfer ================"
  echo "Source          : $map_dir"
  echo "Destination     : ${remote_host}:${remote_map_dir}"
  echo "Artifacts       : cuvgl_map, cuvslam_map, HD map, raceline"
  echo "Excluded        : logs and notebook-side intermediate files"
  echo "Remote latest   : ${remote_parent}/latest -> ${map_name}"
  echo "====================================================="
  echo

  if ! prompt_yes_no "Start rsync transfer?"; then
    echo "Canceled Jetson map transfer."
    return
  fi

  ssh "$remote_host" "mkdir -p -- '$remote_map_dir'"

  rsync -avhP \
    --exclude='logs/' \
    "${sources[@]}" \
    "${remote_host}:${remote_map_dir}/"

  ssh "$remote_host" \
    "ln -sfn -- '$map_name' '${remote_parent}/latest'"

  echo "Jetson map transfer completed: ${remote_host}:${remote_map_dir}"
}

offer_jetson_transfer() {
  local map_dir="$1"
  local map_name
  local required_path
  local missing=()

  map_name="$(basename "$map_dir")"
  for required_path in \
    "${map_dir}/cuvgl_map" \
    "${map_dir}/cuvslam_map" \
    "${map_dir}/${map_name}_hd_map.yaml" \
    "${map_dir}/${map_name}_raceline.csv"; do
    [[ -e "$required_path" ]] || missing+=("$required_path")
  done

  if [[ ${#missing[@]} -gt 0 ]]; then
    echo
    echo "Jetson transfer is unavailable because the runtime map bundle is incomplete:"
    printf '  - %s\n' "${missing[@]}"
    return
  fi

  echo
  if prompt_yes_no "Transfer the runtime map bundle to Jetson?"; then
    transfer_map_to_jetson "$map_dir"
  else
    echo "Skipped Jetson map transfer."
  fi
}

print_usage() {
  cat <<EOF
Usage:
  $(basename "$0")                         Create cuVGL and cuVSLAM maps from a rosbag
  $(basename "$0") --offline-eval          Create/evaluate a cuVSLAM map with an existing cuVGL map
  $(basename "$0") --offline-postprocess   Resume from an existing VSLAM snapshot
  $(basename "$0") --help                  Show this help

Environment:
  CREATE_MAP_OFFLINE_STEPS     Default: edex compute_poses cuvgl
  ALLOW_OCCUPANCY_MAP_STEP     Default: false
  CAMERA_TOPIC_CONFIG_FILE     Optional camera topic yaml override
  ENABLE_HD_MAP_WORKFLOW       true, false, or ask. Default: ask
  ENABLE_ROSBAG_WARMUP_STEP    Enable 2-stage warmup wait before replay. Default: true
EOF
}

select_rosbag() {
  local metadata_file
  local index
  local choice
  local selected
  local candidates=()

  [[ -d "$RECORD_ROOT" ]] || die "record root does not exist: $RECORD_ROOT"

  while IFS= read -r metadata_file; do
    candidates+=("$(dirname "$metadata_file")")
  done < <(find "$RECORD_ROOT" -type f -name metadata.yaml | sort)

  [[ ${#candidates[@]} -gt 0 ]] || die "metadata.yaml was not found under $RECORD_ROOT"

  echo "Found rosbag candidates:" >&2
  for index in "${!candidates[@]}"; do
    printf '  [%d] %s\n' "$((index + 1))" "${candidates[$index]}" >&2
  done
  echo >&2

  while true; do
    read -r -p "Select rosbag number: " choice
    if [[ "$choice" =~ ^[0-9]+$ ]] \
      && (( choice >= 1 && choice <= ${#candidates[@]} )); then
      selected="${candidates[$((choice - 1))]}"
      printf '%s\n' "$selected"
      return 0
    fi
    echo "Please enter a number from 1 to ${#candidates[@]}." >&2
  done
}

select_existing_map() {
  local cuvgl_dir
  local index
  local choice
  local selected
  local candidates=()

  [[ -d "$MAP_ROOT" ]] || die "map root does not exist: $MAP_ROOT"

  while IFS= read -r cuvgl_dir; do
    candidates+=("$(dirname "$cuvgl_dir")")
  done < <(find "$MAP_ROOT" -mindepth 2 -type d -name cuvgl_map | sort)

  [[ ${#candidates[@]} -gt 0 ]] || die "cuvgl_map was not found under $MAP_ROOT"

  echo "Found existing map candidates:" >&2
  for index in "${!candidates[@]}"; do
    printf '  [%d] %s\n' "$((index + 1))" "${candidates[$index]}" >&2
  done
  echo >&2

  while true; do
    read -r -p "Select map number: " choice
    if [[ "$choice" =~ ^[0-9]+$ ]] \
      && (( choice >= 1 && choice <= ${#candidates[@]} )); then
      selected="${candidates[$((choice - 1))]}"
      printf '%s\n' "$selected"
      return 0
    fi
    echo "Please enter a number from 1 to ${#candidates[@]}." >&2
  done
}

resolve_generated_map_dir() {
  local base_output_dir="$1"
  local cuvgl_dir
  local candidates=()

  if [[ -d "${base_output_dir}/cuvgl_map" ]]; then
    printf '%s\n' "$base_output_dir"
    return
  fi

  while IFS= read -r cuvgl_dir; do
    candidates+=("$(dirname "$cuvgl_dir")")
  done < <(find "$base_output_dir" -mindepth 2 -maxdepth 2 -type d -name cuvgl_map | sort)

  if [[ ${#candidates[@]} -eq 0 ]]; then
    die "generated cuVGL map was not found under $base_output_dir"
  fi
  if [[ ${#candidates[@]} -gt 1 ]]; then
    die "multiple generated maps were found under $base_output_dir; use --offline-eval to select one"
  fi

  printf '%s\n' "${candidates[0]}"
}

choose_output_dir() {
  local bag_dir="$1"
  local bag_name
  local timestamp
  local default_name
  local output_name
  local output_dir

  mkdir -p "$MAP_ROOT"

  bag_name="$(basename "$bag_dir")"
  timestamp="$(date +%Y%m%d_%H%M%S)"
  default_name="$(sanitize_name "${bag_name}_map_${timestamp}")"

  while true; do
    read -r -p "Output map name [${default_name}]: " output_name
    output_name="${output_name:-$default_name}"

    if [[ "$output_name" == /* || "$output_name" == *"/"* ]]; then
      echo "Please enter a directory name only, not a path." >&2
      continue
    fi

    output_name="$(sanitize_name "$output_name")"
    output_dir="${MAP_ROOT}/${output_name}"

    if [[ -e "$output_dir" ]]; then
      echo "Output already exists: $output_dir" >&2
      echo "Choose another name to avoid mixing map outputs." >&2
      continue
    fi

    printf '%s\n' "$output_dir"
    return 0
  done
}

run_vgl_tensorrt_export() {
  [[ -x "$VGL_TENSORRT_EXPORT_SCRIPT" ]] \
    || die "VGL TensorRT export script was not found or is not executable: $VGL_TENSORRT_EXPORT_SCRIPT"

  ROS2_WS="$ROS2_WS" \
    OUTPUT_MODEL_DIR="$OUTPUT_MODEL_DIR" \
    "$VGL_TENSORRT_EXPORT_SCRIPT" --yes
}

prepare_hd_map_raster() {
  local snapshot_path="$1"
  local raster_image="$2"
  local raster_yaml="$3"

  ros2 run vslam_map_tools export_aligned_landmarks_offline.py \
    --snapshot "$snapshot_path" \
    --output-image "$raster_image" \
    --output-yaml "$raster_yaml" \
    --no-path \
    --require-landmarks
}

run_hd_map_workflow() {
  local map_dir="$1"
  local snapshot_path="$2"
  local raster_yaml="$3"
  local map_name
  local hd_map_yaml
  local centerline_csv
  local raceline_csv
  local line_preview
  local hd_map_editor
  local raceline_generator
  local line_visualizer

  map_name="$(basename "$map_dir")"
  hd_map_yaml="${map_dir}/${map_name}_hd_map.yaml"
  centerline_csv="${map_dir}/${map_name}_hd_map_centerline.csv"
  raceline_csv="${map_dir}/${map_name}_raceline.csv"
  line_preview="${map_dir}/${map_name}_line_preview.png"
  hd_map_editor="${PYTHON_WS}/map_tools/hd_map_editor.py"
  raceline_generator="${PYTHON_WS}/map_tools/generate_raceline.py"
  line_visualizer="${PYTHON_WS}/map_tools/visualize_race_lines.py"

  [[ -x "$PYTHON_BIN" ]] || die "Python interpreter was not found or is not executable: $PYTHON_BIN"
  [[ -f "$hd_map_editor" ]] \
    || die "HD map editor was not found under PYTHON_WS=${PYTHON_WS}: $hd_map_editor"

  echo "Opening the HD map editor with the generated landmark raster."
  "$PYTHON_BIN" "$hd_map_editor" \
    --map-yaml "$raster_yaml" \
    --output "$hd_map_yaml" \
    --vslam-snapshot "$snapshot_path" \
    --hide-vslam-path

  if [[ ! -f "$centerline_csv" ]]; then
    echo "Centerline CSV was not created; skipping raceline generation: $centerline_csv" >&2
    return
  fi

  if ! prompt_yes_no "Generate a raceline from the edited centerline?"; then
    echo "Skipped raceline generation."
    return
  fi

  [[ -f "$raceline_generator" ]] \
    || die "raceline generator was not found under PYTHON_WS=${PYTHON_WS}: $raceline_generator"

  "$PYTHON_BIN" "$raceline_generator" \
    --centerline "$centerline_csv" \
    --output "$raceline_csv" \
    --show-progress

  [[ -f "$line_visualizer" ]] \
    || die "line visualizer was not found under PYTHON_WS=${PYTHON_WS}: $line_visualizer"

  "$PYTHON_BIN" "$line_visualizer" \
    --yaml "$raster_yaml" \
    --hd-map "$hd_map_yaml" \
    --centerline "$centerline_csv" \
    --raceline "$raceline_csv" \
    --output "$line_preview"

  echo "Line preview    : $line_preview"
}

run_offline_postprocess() {
  local map_dir="$1"
  local snapshot_path
  local raster_image
  local raster_yaml

  snapshot_path="${map_dir}/vslam_reference_snapshot.json"
  raster_image="${map_dir}/vslam_landmarks.png"
  raster_yaml="${map_dir}/vslam_landmarks.yaml"

  [[ -f "$snapshot_path" ]] || die "VSLAM snapshot was not found: $snapshot_path"

  prepare_hd_map_raster "$snapshot_path" "$raster_image" "$raster_yaml"

  echo
  echo "VSLAM snapshot  : $snapshot_path"
  echo "Landmark raster : $raster_yaml"
  echo

  run_hd_map_workflow "$map_dir" "$snapshot_path" "$raster_yaml"
}

hd_map_workflow_available() {
  local hd_map_editor="${PYTHON_WS}/map_tools/hd_map_editor.py"

  [[ -x "$PYTHON_BIN" && -f "$hd_map_editor" ]]
}

maybe_run_offline_postprocess() {
  local map_dir="$1"

  case "$ENABLE_HD_MAP_WORKFLOW" in
    true|TRUE|yes|YES|1)
      run_offline_postprocess "$map_dir"
      ;;
    false|FALSE|no|NO|0)
      echo "Skipped HD map postprocess."
      ;;
    ask|ASK|"")
      if ! hd_map_workflow_available; then
        echo "Skipped HD map postprocess because python_ws/map_tools is unavailable."
        return
      fi
      if prompt_yes_no "Run HD map postprocess from the VSLAM snapshot?"; then
        run_offline_postprocess "$map_dir"
      else
        echo "Skipped HD map postprocess."
      fi
      ;;
    *)
      die "ENABLE_HD_MAP_WORKFLOW must be true, false, or ask: $ENABLE_HD_MAP_WORKFLOW"
      ;;
  esac
}

run_offline_eval() {
  local bag_dir="$1"
  local map_dir="$2"
  local topic_config_file
  local snapshot_path
  local cuvslam_map_dir
  local offline_launch_pid=""
  local offline_ready=0
  local offline_attempt
  local offline_nodes
  local offline_resume_type
  local offline_player_missing=0
  local offline_launch_status=0
  local replay_additional_args
  local rviz_config_file

  topic_config_file="$(resolve_camera_topic_config_file)"
  snapshot_path="${map_dir}/vslam_reference_snapshot.json"
  cuvslam_map_dir="${map_dir}/cuvslam_map"
  rviz_config_file="$OFFLINE_RVIZ_CONFIG_FILE"
  if [[ ! -f "$rviz_config_file" ]]; then
    rviz_config_file="${ROS2_WS}/src/launch/jetpilot_system_launch/rviz/vslam_debug.rviz"
  fi
  replay_additional_args="$ROSBAG_PLAY_ADDITIONAL_ARGS"
  if [[ " $replay_additional_args " != *" --start-paused "* ]]; then
    replay_additional_args="--start-paused $replay_additional_args"
  fi

  [[ -d "${map_dir}/cuvgl_map" ]] || die "cuVGL map was not found: ${map_dir}/cuvgl_map"
  [[ -d "$cuvslam_map_dir" ]] || die "cuVSLAM map was not found for offline eval load: $cuvslam_map_dir"
  rm -f "$snapshot_path"
  echo "[stage] offline eval will load cuVSLAM map: $cuvslam_map_dir"
  echo "[stage] offline eval use_sim_time: true"
  echo "[stage] offline eval VSLAM visualization: true"

  cleanup_offline_graph() {
    echo "[stage] cleaning up any leftover ROS 2 offline nodes..."
    pkill -f "visual_slam_node" 2>/dev/null || true
    pkill -f "visual_global_localization_node" 2>/dev/null || true
    pkill -f "localization_manager" 2>/dev/null || true
    pkill -f "vslam_reference_snapshot_recorder" 2>/dev/null || true
    pkill -f "rosbag2_player" 2>/dev/null || true
    ros2 daemon stop 2>/dev/null || true
    local quiet_attempt=0
    local nodes
    local resume_type
    while (( quiet_attempt < 5 )); do
      nodes="$(ros2 node list 2>/dev/null || true)"
      resume_type="$(ros2 service type /rosbag2_player/resume 2>/dev/null || true)"
      if [[ "$nodes" != *visual_slam_node* ]] && [[ "$nodes" != *visual_global_localization_node* ]] && [[ "$nodes" != *localization_manager* ]] && [[ "$nodes" != *vslam_reference_snapshot_recorder* ]] && [[ "$nodes" != *rosbag2_interfaces/srv/Resume* ]]; then
        return 0
      fi
      pkill -9 -f "visual_slam_node|visual_global_localization_node|localization_manager|vslam_reference_snapshot_recorder|rosbag2_player" 2>/dev/null || true
      quiet_attempt=$((quiet_attempt + 1))
      sleep 1
    done
    return 0
  }
  offline_stop_launch() {
    local stop_signal="${1:-INT}"
    local timeout_s="${2:-20}"
    local waited_s=0
    local stop_status=0
    if [[ -n "$offline_launch_pid" ]]; then
      if kill -0 "$offline_launch_pid" 2>/dev/null; then
        kill -s "$stop_signal" "$offline_launch_pid" 2>/dev/null || true
      fi
      while kill -0 "$offline_launch_pid" 2>/dev/null; do
        if (( waited_s >= timeout_s )); then
          kill -9 "$offline_launch_pid" 2>/dev/null || true
          break
        fi
        sleep 1
        waited_s=$((waited_s + 1))
      done
      wait "$offline_launch_pid" 2>/dev/null || stop_status=$?
    fi
    offline_launch_pid=""
    cleanup_offline_graph || true
    return "$stop_status"
  }
  offline_topic_publishers() {
    ros2 topic info "$1" 2>/dev/null | awk '/Publisher count:/ {print $3; found=1} END {if (!found) print 0}'
  }
  trap 'offline_stop_launch TERM 5 || kill -KILL "$offline_launch_pid" 2>/dev/null || true' EXIT
  cleanup_offline_graph

  ros2 launch "$JETPILOT_LAUNCH_PACKAGE" bringup.launch.py \
    use_sim_time:=true \
    enable_rosbag_replay:=true \
    replay_additional_args:="$replay_additional_args" \
    rosbag_start_delay_s:=0.0 \
    rosbag_shutdown_on_exit:=false \
    enable_operation:=false \
    enable_control:=false \
    enable_vehicle:=false \
    publish_vehicle_description:=false \
    enable_sensor_kit:=false \
    enable_localization:=true \
    vslam_enable_slam:=true \
    vslam_enable_visualization:=true \
    vslam_localize_on_startup:=true \
    enable_vgl:=false \
    vgl_topic_config_file:="$topic_config_file" \
    vgl_model_dir:="$OUTPUT_MODEL_DIR" \
    enable_rviz:="$ENABLE_OFFLINE_RVIZ" \
    rviz_config_file:="$rviz_config_file" \
    enable_tool:=true \
    enable_bag_manager:=false \
    enable_joy:=false \
    enable_teleop:=false \
    enable_rc_serial:=false \
    enable_vslam_snapshot:=true \
    vslam_snapshot_output:="$snapshot_path" \
    vslam_snapshot_landmarks_topic:="$VSLAM_LANDMARKS_TOPIC" \
    vslam_snapshot_write_interval_s:="$VSLAM_SNAPSHOT_WRITE_INTERVAL_S" \
    rosbag:="$bag_dir" \
    map_dir:="$map_dir" &
  offline_launch_pid=$!

  for offline_attempt in $(seq 1 180); do
    if ! kill -0 "$offline_launch_pid" 2>/dev/null; then
      offline_stop_launch TERM || true
      die "offline eval launch exited before replay became ready"
    fi
    offline_nodes="$(ros2 node list 2>/dev/null || true)"
    offline_resume_type="$(ros2 service type /rosbag2_player/resume 2>/dev/null || true)"
    if [[ "$offline_nodes" == *visual_slam_node* ]] \
      && [[ "$offline_nodes" == *vslam_reference_snapshot_recorder* ]] \
      && [[ "$offline_resume_type" == *rosbag2_interfaces/srv/Resume* ]] \
      && (( $(offline_topic_publishers /visual_slam/tracking/slam_path) > 0 )) \
      && (( $(offline_topic_publishers /visual_slam/tracking/odometry) > 0 )) \
      && (( $(offline_topic_publishers "$VSLAM_LANDMARKS_TOPIC") > 0 )); then
      offline_ready=1
      break
    fi
    sleep 1
  done

  if [[ "$offline_ready" != "1" ]]; then
    offline_stop_launch TERM || true
    die "offline eval readiness timed out after 180 seconds; VSLAM publishers did not become available"
  fi

  echo "[stage] offline eval graph ready; preparing rosbag replay"
  if is_true_value "${ENABLE_ROSBAG_WARMUP_STEP:-true}"; then
    echo "[stage] starting 2-stage wait (warmup step)"
    sleep 5
    echo "[stage] advancing rosbag by ~1 image frame (0.15s) for lazy initialization"
    ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
    sleep 0.15
    ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause '{}' || true
    echo "[stage] waiting 5s for VGL/VSLAM initialization to settle during pause"
    sleep 5
    echo "[stage] resuming rosbag playback after warmup"
    ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
  else
    sleep 5
    ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
  fi

  for offline_attempt in $(seq 1 15); do
    if [[ -s "$snapshot_path" ]]; then
      break
    fi
    if ! kill -0 "$offline_launch_pid" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  if [[ ! -s "$snapshot_path" ]]; then
    offline_stop_launch TERM 5 || true
    die "offline eval produced no VSLAM snapshot messages after replay started; refusing to drain an empty run"
  fi

  while kill -0 "$offline_launch_pid" 2>/dev/null; do
    offline_resume_type="$(ros2 service type /rosbag2_player/resume 2>/dev/null || true)"
    if [[ "$offline_resume_type" != *rosbag2_interfaces/srv/Resume* ]]; then
      offline_player_missing=$((offline_player_missing + 1))
    else
      offline_player_missing=0
    fi
    if (( offline_player_missing >= 5 )); then
      break
    fi
    sleep 1
  done

  echo "[stage] rosbag replay finished; draining offline eval output"
  sleep 5
  if offline_stop_launch INT 20; then
    offline_launch_status=0
  else
    offline_launch_status=$?
  fi
  if (( offline_launch_status == 124 )); then
    echo "[stage] offline eval did not stop after SIGINT; escalating to SIGTERM"
    if offline_stop_launch TERM 10; then
      offline_launch_status=0
    else
      offline_launch_status=$?
    fi
  fi
  if (( offline_launch_status == 124 )); then
    echo "[stage] offline eval did not stop after SIGTERM; escalating to SIGKILL"
    kill -KILL "$offline_launch_pid" 2>/dev/null || true
    wait "$offline_launch_pid" 2>/dev/null || true
    offline_launch_pid=""
    offline_launch_status=0
  fi
  if (( offline_launch_status != 0 && offline_launch_status != 130 )); then
    die "offline eval launch exited with status $offline_launch_status"
  fi
  trap - EXIT

  [[ -f "$snapshot_path" ]] || die "VSLAM snapshot was not created: $snapshot_path"
  [[ -d "$cuvslam_map_dir" ]] || die "cuVSLAM map was not created: $cuvslam_map_dir"

  maybe_run_offline_postprocess "$map_dir"
}

main() {
  local mode="${1:-create}"
  local bag_dir
  local base_output_dir
  local output_dir
  local topic_config_file
  local create_map_steps=()

  case "$mode" in
    create) ;;
    --offline-eval) mode="offline_eval" ;;
    --offline-postprocess) mode="offline_postprocess" ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      print_usage >&2
      die "unknown option: $mode"
      ;;
  esac

  if (( $# > 1 )); then
    print_usage >&2
    die "too many arguments"
  fi

  ensure_workspace_overlay

  if [[ "$mode" == "offline_postprocess" ]]; then
    output_dir="$(select_existing_map)"

    echo
    echo "Existing map : $output_dir"
    echo

    run_offline_postprocess "$output_dir"
    offer_jetson_transfer "$output_dir"
    exit 0
  fi

  if [[ "$mode" == "offline_eval" ]]; then
    output_dir="$(select_existing_map)"
    bag_dir="$(select_rosbag)"

    echo
    echo "Selected rosbag : $bag_dir"
    echo "Existing map    : $output_dir"
    echo "VGL model dir   : $OUTPUT_MODEL_DIR"
    echo "ROS bag args    : $ROSBAG_PLAY_ADDITIONAL_ARGS"
    echo "HD postprocess  : $ENABLE_HD_MAP_WORKFLOW"
    echo "Offline RViz    : $ENABLE_OFFLINE_RVIZ"
    echo "RViz config     : $OFFLINE_RVIZ_CONFIG_FILE"
    echo

    run_offline_eval "$bag_dir" "$output_dir"
    offer_jetson_transfer "$output_dir"
    return
  fi

  bag_dir="$(select_rosbag)"
  base_output_dir="$(choose_output_dir "$bag_dir")"

  echo
  echo "Selected rosbag : $bag_dir"
  echo "Output folder   : $base_output_dir"
  echo "FS model res    : $FOUNDATIONSTEREO_MODEL_RES"
  echo "VGL model dir   : $OUTPUT_MODEL_DIR"
  echo "Mapping steps   : $CREATE_MAP_OFFLINE_STEPS"
  echo "ROS bag args    : $ROSBAG_PLAY_ADDITIONAL_ARGS"
  echo "HD postprocess  : $ENABLE_HD_MAP_WORKFLOW"
  echo "Offline RViz    : $ENABLE_OFFLINE_RVIZ"
  echo "RViz config     : $OFFLINE_RVIZ_CONFIG_FILE"
  echo

  if ! prompt_yes_no "Run create_map_offline.py?"; then
    echo "Canceled."
    exit 0
  fi

  export FOUNDATIONSTEREO_MODEL_RES
  topic_config_file="$(resolve_camera_topic_config_file)"
  validate_create_map_offline_steps
  read -r -a create_map_steps <<< "$CREATE_MAP_OFFLINE_STEPS"

  ros2 run isaac_mapping_ros create_map_offline.py \
    --sensor_data_bag="$bag_dir" \
    --base_output_folder="$base_output_dir" \
    --camera_topic_config="$topic_config_file" \
    --fs_model_res="${FOUNDATIONSTEREO_MODEL_RES}" \
    --steps_to_run "${create_map_steps[@]}"

  output_dir="$(resolve_generated_map_dir "$base_output_dir")"
  echo
  echo "Generated map   : $output_dir"

  echo
  if prompt_yes_no "Export VGL TensorRT engines? (normally needed once per environment)"; then
    run_vgl_tensorrt_export
  else
    echo "Skipped VGL TensorRT engine export."
  fi

  echo
  echo "Running offline evaluation to create the VSLAM snapshot and HD map artifacts."
  run_offline_eval "$bag_dir" "$output_dir"

  offer_jetson_transfer "$output_dir"
}

main "$@"
