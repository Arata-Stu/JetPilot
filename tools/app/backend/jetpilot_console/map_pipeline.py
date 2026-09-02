from __future__ import annotations

import math
import shlex
from pathlib import Path

from .config import ConsoleConfig


DEFAULT_RACELINE_VEHICLE_WIDTH_M = 0.25
DEFAULT_RACELINE_SAFETY_MARGIN_M = 0.05
DEFAULT_RACELINE_MAX_SPEED_MPS = 3.0
DEFAULT_RACELINE_MIN_SPEED_MPS = 0.8
DEFAULT_RACELINE_LATERAL_ACCEL_LIMIT_MPS2 = 2.5
DEFAULT_RACELINE_ACCEL_LIMIT_MPS2 = 1.5
DEFAULT_RACELINE_DECEL_LIMIT_MPS2 = 2.5
DEFAULT_RACELINE_DIRECTION = "forward"
VALID_RACELINE_DIRECTIONS = frozenset({"forward", "reverse"})
DEFAULT_HD_RASTER_AUTO_CROP_PERCENTILE = 99.0
DEFAULT_HD_RASTER_AUTO_CROP_MIN_RETAINED_RATIO = 0.75
DEFAULT_HD_RASTER_PATH_CROP_DISTANCE_M = 0.0
DEFAULT_HD_RASTER_PATH_CROP_MIN_RETAINED_RATIO = 0.2


def _q(value: str | Path) -> str:
    return shlex.quote(str(value))


def _nonnegative_finite(value: float, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite value greater than or equal to 0")
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{label} must be a finite value greater than or equal to 0")
    return parsed


def _positive_finite(value: float, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite value greater than 0")
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{label} must be a finite value greater than 0")
    return parsed


def _raceline_direction(value: str) -> str:
    if not isinstance(value, str):
        raise ValueError("direction must be forward or reverse")
    direction = value.strip().lower()
    if direction not in VALID_RACELINE_DIRECTIONS:
        raise ValueError("direction must be forward or reverse")
    return direction


def _bounded_finite(value: float, label: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite value in [{minimum}, {maximum}]")
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < minimum or parsed > maximum:
        raise ValueError(f"{label} must be a finite value in [{minimum}, {maximum}]")
    return parsed


def _source_ros_setup(config: ConsoleConfig) -> str:
    setup = config.ros2_ws / "install" / "setup.bash"
    return f'if [ -f {_q(setup)} ]; then set +u; source {_q(setup)}; set -u; fi'


def default_topic_config(config: ConsoleConfig) -> Path:
    return localization_config_dir(config) / "vgl_camera_topics.yaml"


def default_vslam_rviz_config(config: ConsoleConfig) -> Path:
    return config.ros2_ws / "install" / "jetpilot_system_launch" / "share" / "jetpilot_system_launch" / "rviz" / "vslam_debug.rviz"


def localization_config_dir(config: ConsoleConfig) -> Path:
    return (
        config.ros2_ws
        / "src"
        / "launch"
        / "jetpilot_system_launch"
        / "config"
        / "localization"
    )


def _path_relative_to(path: Path, base: Path) -> str:
    try:
        return str(path.relative_to(base))
    except ValueError:
        return str(path)


def _camera_topic_config_score(path: Path) -> int:
    name = path.name.lower()
    if name == "vgl_camera_topics.yaml":
        return 100
    score = 0
    if "camera" in name:
        score += 40
    if "topic" in name or "topics" in name:
        score += 40
    if "vgl" in name:
        score += 10
    return score


def _camera_topic_config_required_topics(path: Path) -> list[str]:
    topic_keys = {"left", "right", "left_camera_info", "right_camera_info"}
    topics: set[str] = set()
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []

    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].strip()
        key, separator, raw_value = line.partition(":")
        if not separator or key not in topic_keys:
            continue
        value = raw_value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
            value = value[1:-1]
        if value.startswith("/"):
            topics.add(value)
    return sorted(topics)


def scan_camera_topic_configs(config: ConsoleConfig) -> list[dict[str, object]]:
    config_dir = localization_config_dir(config)
    if not config_dir.exists():
        return []

    discovered = []
    for path in sorted([*config_dir.glob("*.yaml"), *config_dir.glob("*.yml")]):
        if not path.is_file():
            continue
        score = _camera_topic_config_score(path)
        discovered.append(
            {
                "name": path.name,
                "path": str(path),
                "relative_path": _path_relative_to(path, config.ros2_ws),
                "score": score,
                "recommended": path == default_topic_config(config) or score >= 80,
                "required_topics": _camera_topic_config_required_topics(path),
            }
        )

    confident = [item for item in discovered if int(item["score"]) >= 40]
    candidates = confident or discovered
    return sorted(
        candidates,
        key=lambda item: (
            not bool(item["recommended"]),
            -int(item["score"]),
            str(item["name"]),
        ),
    )


def build_vgl_vslam_script(
    config: ConsoleConfig,
    rosbag: str,
    map_dir: str,
    topic_config: str | None,
    steps: str,
    fs_model_res: str,
    output_model_dir: str,
    enable_rviz: bool,
    enable_warmup_step: bool = True,
) -> str:
    topic_config_path = topic_config or str(default_topic_config(config))
    create_steps = " ".join(shlex.quote(step) for step in steps.split())
    rviz_value = "true" if enable_rviz else "false"
    rviz_config_file = default_vslam_rviz_config(config)
    return f"""set -euo pipefail
{_source_ros_setup(config)}
mkdir -p {_q(map_dir)}
requested_map_dir={_q(map_dir)}
offline_launch_pid=""
offline_stop_launch() {{
  stop_signal="${{1:-INT}}"
  timeout_s="${{2:-20}}"
  waited_s=0
  stop_status=0
  if [ -n "$offline_launch_pid" ]; then
    if kill -0 "$offline_launch_pid" 2>/dev/null; then
      kill -s "$stop_signal" "-$offline_launch_pid" 2>/dev/null || kill -s "$stop_signal" "$offline_launch_pid" 2>/dev/null || true
    fi
    while kill -0 "$offline_launch_pid" 2>/dev/null; do
      if [ "$waited_s" -ge "$timeout_s" ]; then
        return 124
      fi
      sleep 1
      waited_s=$((waited_s + 1))
    done
    wait "$offline_launch_pid" 2>/dev/null || stop_status=$?
  fi
  offline_launch_pid=""
  return "$stop_status"
}}
offline_topic_publishers() {{
  ros2 topic info "$1" 2>/dev/null | awk '/Publisher count:/ {{print $3; found=1}} END {{if (!found) print 0}}'
}}
offline_topic_subscribers() {{
  ros2 topic info "$1" 2>/dev/null | awk '/Subscription count:/ {{print $3; found=1}} END {{if (!found) print 0}}'
}}
offline_log_topic_counts() {{
  echo "[debug] input topics: infra1 pubs=$(offline_topic_publishers /realsense/infra1/image_rect_raw), subs=$(offline_topic_subscribers /realsense/infra1/image_rect_raw); infra2 pubs=$(offline_topic_publishers /realsense/infra2/image_rect_raw), subs=$(offline_topic_subscribers /realsense/infra2/image_rect_raw); imu pubs=$(offline_topic_publishers /realsense/imu), subs=$(offline_topic_subscribers /realsense/imu)"
  echo "[debug] output topics: path pubs=$(offline_topic_publishers /visual_slam/tracking/slam_path), subs=$(offline_topic_subscribers /visual_slam/tracking/slam_path); odom pubs=$(offline_topic_publishers /visual_slam/tracking/odometry), subs=$(offline_topic_subscribers /visual_slam/tracking/odometry); landmarks pubs=$(offline_topic_publishers /visual_slam/vis/landmarks_cloud), subs=$(offline_topic_subscribers /visual_slam/vis/landmarks_cloud)"
}}
trap 'offline_stop_launch TERM 5 || kill -KILL "-$offline_launch_pid" 2>/dev/null || kill -KILL "$offline_launch_pid" 2>/dev/null || true' EXIT
export FOUNDATIONSTEREO_MODEL_RES={_q(fs_model_res)}
echo "[stage] create cuVGL map"
ros2 run isaac_mapping_ros create_map_offline.py \\
  --sensor_data_bag={_q(rosbag)} \\
  --base_output_folder="$requested_map_dir" \\
  --camera_topic_config={_q(topic_config_path)} \\
  --fs_model_res={_q(fs_model_res)} \\
  --steps_to_run {create_steps}
generated_map_dir="$requested_map_dir"
latest_candidate=""
while IFS= read -r -d '' candidate; do
  if [ -d "$candidate/cuvgl_map" ]; then
    if [ -z "$latest_candidate" ] || [ "$candidate" -nt "$latest_candidate" ]; then
      latest_candidate="$candidate"
    fi
  fi
done < <(find "$requested_map_dir" -mindepth 1 -maxdepth 1 -type d -print0 2>/dev/null || true)
if [ -n "$latest_candidate" ]; then
  generated_map_dir="$latest_candidate"
fi
echo "[stage] using map artifacts folder: $generated_map_dir"
snapshot="$generated_map_dir/vslam_reference_snapshot.json"
cuvslam_map="$generated_map_dir/cuvslam_map"
if [ ! -d "$cuvslam_map" ]; then
  echo "cuVSLAM map was not found for offline eval load: $cuvslam_map"
  exit 24
fi
rm -f "$snapshot"
echo "[stage] offline eval will load cuVSLAM map: $cuvslam_map"
echo "[stage] offline eval use_sim_time: true"
echo "[stage] offline eval VSLAM visualization: true"
echo "[stage] offline eval for cuVSLAM snapshot"
setsid ros2 launch {_q(config.launch_package)} bringup.launch.py \\
  use_sim_time:=true \\
  enable_rosbag_replay:=true \\
  replay_additional_args:='--clock --start-paused' \\
  rosbag_start_delay_s:=0.0 \\
  rosbag_shutdown_on_exit:=false \\
  enable_operation:=false \\
  enable_control:=false \\
  enable_vehicle:=false \\
  publish_vehicle_description:=false \\
  enable_sensor_kit:=false \\
  enable_localization:=true \\
  enable_localization_manager:=false \\
  vslam_enable_slam:=true \\
  vslam_enable_visualization:=true \\
  vslam_localize_on_startup:=true \\
  enable_vgl:=false \\
  vgl_topic_config_file:={_q(topic_config_path)} \\
{f"  vgl_model_dir:={_q(output_model_dir)} \\\n" if output_model_dir else ""}\
  enable_rviz:={rviz_value} \\
  rviz_config_file:={_q(rviz_config_file)} \\
  enable_tool:=true \\
  enable_bag_manager:=false \\
  enable_joy:=false \\
  enable_teleop:=false \\
  enable_rc_serial:=false \\
  enable_vslam_snapshot:=true \\
  vslam_snapshot_output:="$snapshot" \\
  vslam_snapshot_landmarks_topic:=/visual_slam/vis/landmarks_cloud \\
  vslam_snapshot_require_localized_map:=false \\
  vslam_snapshot_map_frame:=map \\
  vslam_snapshot_write_interval_s:=5.0 \\
  rosbag:={_q(rosbag)} \\
  map_dir:="$generated_map_dir" &
offline_launch_pid=$!
offline_ready=0
for offline_attempt in $(seq 1 120); do
  if ! kill -0 "$offline_launch_pid" 2>/dev/null; then
    echo "offline eval launch exited before graph became observable"
    offline_stop_launch TERM || true
    exit 22
  fi
  offline_nodes="$(ros2 node list 2>/dev/null || true)"
  if [[ "$offline_nodes" == *visual_slam_node* ]] && [[ "$offline_nodes" == *vslam_reference_snapshot_recorder* ]] && [[ "$offline_nodes" == *rosbag2_player* ]]; then
    offline_path_publishers="$(offline_topic_publishers /visual_slam/tracking/slam_path)"
    offline_odom_publishers="$(offline_topic_publishers /visual_slam/tracking/odometry)"
    if [ "${{offline_path_publishers:-0}}" -gt 0 ] && [ "${{offline_odom_publishers:-0}}" -gt 0 ]; then
      offline_ready=1
      break
    fi
  fi
  sleep 1
done
if [ "$offline_ready" -ne 1 ]; then
  echo "offline eval readiness timed out after 120 seconds; VSLAM publishers did not become available"
  echo "[debug] nodes: $(ros2 node list 2>/dev/null | tr '\n' ' ' || true)"
  offline_log_topic_counts
  offline_stop_launch TERM || true
  exit 23
fi
echo "[stage] offline eval graph ready; starting paused rosbag replay"
offline_log_topic_counts
echo "[stage] VGL is disabled; relying on VSLAM localize_on_startup identity pose"
if ! ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{{}}'; then
  offline_stop_launch TERM || true
  echo "offline eval could not resume the paused rosbag replay"
  exit 24
fi
offline_snapshot_seen=0
for offline_attempt in $(seq 1 300); do
  if [ -s "$snapshot" ]; then
    offline_snapshot_seen=1
    break
  fi
  if ! kill -0 "$offline_launch_pid" 2>/dev/null; then
    break
  fi
  if [ "$offline_attempt" = "30" ] || [ "$offline_attempt" = "120" ]; then
    echo "[debug] waiting for snapshot after $offline_attempt s"
    offline_log_topic_counts
  fi
  sleep 1
done
if [ "$offline_snapshot_seen" -ne 1 ]; then
  offline_log_topic_counts
  offline_stop_launch TERM 5 || true
  echo "offline eval produced no VSLAM snapshot messages after replay started"
  exit 25
fi
echo "[stage] VSLAM snapshot created; waiting for rosbag replay to drain"
offline_replay_finished=0
for offline_attempt in $(seq 1 600); do
  if ! kill -0 "$offline_launch_pid" 2>/dev/null; then
    break
  fi
  offline_nodes="$(ros2 node list 2>/dev/null || true)"
  if [[ "$offline_nodes" != *rosbag2_player* ]]; then
    offline_replay_finished=1
    break
  fi
  sleep 1
done
if [ "$offline_replay_finished" -ne 1 ]; then
  offline_stop_launch TERM 5 || true
  echo "offline eval rosbag replay did not finish within 600 seconds"
  exit 26
fi
if [ ! -s "$snapshot" ]; then
  offline_stop_launch TERM 5 || true
  echo "refusing to drain an empty run: VSLAM snapshot is missing"
  exit 25
fi
echo "[stage] rosbag replay finished; stopping offline eval to flush the final snapshot"
offline_stop_status=0
offline_stop_launch INT 20 || offline_stop_status=$?
if [ "$offline_stop_status" -ne 0 ] && [ "$offline_stop_status" -ne 130 ]; then
  echo "offline eval did not stop after SIGINT; forcing termination"
  kill -KILL "-$offline_launch_pid" 2>/dev/null || kill -KILL "$offline_launch_pid" 2>/dev/null || true
  offline_launch_pid=""
  exit 27
fi
trap - EXIT
"""


def prepare_hd_raster_script(
    config: ConsoleConfig,
    map_dir: str,
    *,
    auto_crop_percentile: float = DEFAULT_HD_RASTER_AUTO_CROP_PERCENTILE,
    auto_crop_min_retained_ratio: float = DEFAULT_HD_RASTER_AUTO_CROP_MIN_RETAINED_RATIO,
    path_crop_distance_m: float = DEFAULT_HD_RASTER_PATH_CROP_DISTANCE_M,
    path_crop_min_retained_ratio: float = DEFAULT_HD_RASTER_PATH_CROP_MIN_RETAINED_RATIO,
) -> str:
    map_path = Path(map_dir)
    crop_percentile = _bounded_finite(
        auto_crop_percentile,
        "auto_crop_percentile",
        0.001,
        100.0,
    )
    crop_min_retained = _bounded_finite(
        auto_crop_min_retained_ratio,
        "auto_crop_min_retained_ratio",
        0.0,
        1.0,
    )
    path_crop_distance = _bounded_finite(
        path_crop_distance_m,
        "path_crop_distance_m",
        0.0,
        1000.0,
    )
    path_crop_min_retained = _bounded_finite(
        path_crop_min_retained_ratio,
        "path_crop_min_retained_ratio",
        0.0,
        1.0,
    )
    return f"""set -euo pipefail
{_source_ros_setup(config)}
map_dir={_q(map_path)}
snapshot="$map_dir/vslam_reference_snapshot.json"
parent_snapshot="$(dirname "$map_dir")/vslam_reference_snapshot.json"
if [ ! -f "$snapshot" ] && [ -f "$parent_snapshot" ]; then
  echo "[stage] using parent VSLAM snapshot: $parent_snapshot"
  snapshot="$parent_snapshot"
fi
ros2 run vslam_map_tools export_aligned_landmarks_offline.py \\
  --snapshot "$snapshot" \\
  --output-image "$map_dir/vslam_landmarks.png" \\
  --output-yaml "$map_dir/vslam_landmarks.yaml" \\
  --no-path \\
  --auto-crop-percentile {crop_percentile:.9g} \\
  --auto-crop-min-retained-ratio {crop_min_retained:.9g} \\
  --path-crop-distance-m {path_crop_distance:.9g} \\
  --path-crop-min-retained-ratio {path_crop_min_retained:.9g} \\
  --require-landmarks
"""


def generate_raceline_script(
    config: ConsoleConfig,
    map_dir: str,
    *,
    vehicle_width_m: float = DEFAULT_RACELINE_VEHICLE_WIDTH_M,
    safety_margin_m: float = DEFAULT_RACELINE_SAFETY_MARGIN_M,
    max_speed_mps: float = DEFAULT_RACELINE_MAX_SPEED_MPS,
    min_speed_mps: float = DEFAULT_RACELINE_MIN_SPEED_MPS,
    lateral_accel_limit_mps2: float = DEFAULT_RACELINE_LATERAL_ACCEL_LIMIT_MPS2,
    accel_limit_mps2: float = DEFAULT_RACELINE_ACCEL_LIMIT_MPS2,
    decel_limit_mps2: float = DEFAULT_RACELINE_DECEL_LIMIT_MPS2,
    direction: str = DEFAULT_RACELINE_DIRECTION,
    preset: str | None = None,
) -> str:
    map_path = Path(map_dir)
    name = map_path.name
    vehicle_width = _nonnegative_finite(vehicle_width_m, "vehicle_width_m")
    safety_margin = _nonnegative_finite(safety_margin_m, "safety_margin_m")
    max_speed = _positive_finite(max_speed_mps, "max_speed_mps")
    min_speed = _nonnegative_finite(min_speed_mps, "min_speed_mps")
    if min_speed > max_speed:
        raise ValueError("min_speed_mps must be less than or equal to max_speed_mps")
    lateral_accel = _positive_finite(lateral_accel_limit_mps2, "lateral_accel_limit_mps2")
    accel_limit = _positive_finite(accel_limit_mps2, "accel_limit_mps2")
    decel_limit = _positive_finite(decel_limit_mps2, "decel_limit_mps2")
    selected_direction = _raceline_direction(direction)
    selected_preset = preset if preset else ("f110" if vehicle_width <= 0.6 else "race-stacks")
    return f"""set -euo pipefail
{_q(config.python_bin)} {_q(config.python_ws / "map_tools" / "generate_raceline.py")} \\
  --centerline {_q(map_path / f"{name}_hd_map_centerline.csv")} \\
  --output {_q(map_path / f"{name}_raceline.csv")} \\
  --vehicle-width-m {vehicle_width:.9g} \\
  --safety-margin-m {safety_margin:.9g} \\
  --max-speed {max_speed:.9g} \\
  --min-speed {min_speed:.9g} \\
  --lateral-accel-limit {lateral_accel:.9g} \\
  --accel-limit {accel_limit:.9g} \\
  --decel-limit {decel_limit:.9g} \\
  --direction {selected_direction} \\
  --preset {selected_preset} \\
  --show-progress
"""


def generate_preview_script(config: ConsoleConfig, map_dir: str) -> str:
    map_path = Path(map_dir)
    name = map_path.name
    return f"""set -euo pipefail
{_q(config.python_bin)} {_q(config.python_ws / "map_tools" / "visualize_race_lines.py")} \\
  --yaml {_q(map_path / "vslam_landmarks.yaml")} \\
  --hd-map {_q(map_path / f"{name}_hd_map.yaml")} \\
  --centerline {_q(map_path / f"{name}_hd_map_centerline.csv")} \\
  --raceline {_q(map_path / f"{name}_raceline.csv")} \\
  --output {_q(map_path / f"{name}_line_preview.png")}
"""
