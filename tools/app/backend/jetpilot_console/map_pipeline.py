from __future__ import annotations

import math
import shlex
from pathlib import Path

from .config import ConsoleConfig


DEFAULT_RACELINE_VEHICLE_WIDTH_M = 0.25
DEFAULT_RACELINE_SAFETY_MARGIN_M = 0.05


def _q(value: str | Path) -> str:
    return shlex.quote(str(value))


def _nonnegative_finite(value: float, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite value greater than or equal to 0")
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{label} must be a finite value greater than or equal to 0")
    return parsed


def _source_ros_setup(config: ConsoleConfig) -> str:
    setup = config.ros2_ws / "install" / "setup.bash"
    return f'if [ -f {_q(setup)} ]; then set +u; source {_q(setup)}; set -u; fi'


def default_topic_config(config: ConsoleConfig) -> Path:
    return localization_config_dir(config) / "vgl_camera_topics.yaml"


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
) -> str:
    topic_config_path = topic_config or str(default_topic_config(config))
    create_steps = " ".join(shlex.quote(step) for step in steps.split())
    # Automated offline replay is easier to shut down cleanly without RViz in the
    # launch graph. The UI can still visualize the generated artifacts afterward.
    rviz_value = "false"
    return f"""set -euo pipefail
{_source_ros_setup(config)}
mkdir -p {_q(map_dir)}
requested_map_dir={_q(map_dir)}
offline_launch_pid=""
offline_stop_launch() {{
  stop_signal="${{1:-INT}}"
  stop_status=0
  if [ -n "$offline_launch_pid" ]; then
    if kill -0 "$offline_launch_pid" 2>/dev/null; then
      kill -s "$stop_signal" "$offline_launch_pid" 2>/dev/null || true
    fi
    if wait "$offline_launch_pid" 2>/dev/null; then
      stop_status=0
    else
      stop_status=$?
    fi
  fi
  offline_launch_pid=""
  return "$stop_status"
}}
trap 'offline_stop_launch TERM || true' EXIT
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
echo "[stage] offline eval for cuVSLAM snapshot"
ros2 launch {_q(config.launch_package)} bringup.launch.py \\
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
  vslam_enable_slam:=true \\
  vslam_enable_visualization:=true \\
  vslam_save_map_folder_path:="$cuvslam_map" \\
  enable_vgl:=false \\
  vgl_topic_config_file:={_q(topic_config_path)} \\
  vgl_model_dir:={_q(output_model_dir)} \\
  enable_rviz:={rviz_value} \\
  enable_tool:=true \\
  enable_bag_manager:=false \\
  enable_joy:=false \\
  enable_teleop:=false \\
  enable_rc_serial:=false \\
  enable_vslam_snapshot:=true \\
  vslam_snapshot_output:="$snapshot" \\
  vslam_snapshot_landmarks_topic:=/visual_slam/vis/landmarks_cloud \\
  vslam_snapshot_write_interval_s:=5.0 \\
  rosbag:={_q(rosbag)} \\
  map_dir:="$generated_map_dir" &
offline_launch_pid=$!
offline_ready=0
for offline_attempt in $(seq 1 180); do
  if ! kill -0 "$offline_launch_pid" 2>/dev/null; then
    echo "offline eval launch exited before replay became ready"
    offline_stop_launch TERM || true
    exit 22
  fi
  offline_nodes="$(ros2 node list 2>/dev/null || true)"
  offline_resume_type="$(ros2 service type /rosbag2_player/resume 2>/dev/null || true)"
  if [[ "$offline_nodes" == *visual_slam_node* ]] && [[ "$offline_nodes" == *vslam_reference_snapshot_recorder* ]] && [[ "$offline_resume_type" == *rosbag2_interfaces/srv/Resume* ]]; then
    offline_ready=1
    break
  fi
  sleep 1
done
if [ "$offline_ready" -ne 1 ]; then
  echo "offline eval readiness timed out after 180 seconds"
  offline_stop_launch TERM || true
  exit 23
fi
echo "[stage] offline eval graph ready; starting paused rosbag replay"
sleep 5
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{{}}'
offline_player_missing=0
while kill -0 "$offline_launch_pid" 2>/dev/null; do
  offline_resume_type="$(ros2 service type /rosbag2_player/resume 2>/dev/null || true)"
  if [[ "$offline_resume_type" != *rosbag2_interfaces/srv/Resume* ]]; then
    offline_player_missing=$((offline_player_missing + 1))
  else
    offline_player_missing=0
  fi
  if [ "$offline_player_missing" -ge 5 ]; then
    break
  fi
  sleep 1
done
echo "[stage] rosbag replay finished; draining offline eval output"
sleep 5
if offline_stop_launch INT; then offline_launch_status=0; else offline_launch_status=$?; fi
if [ "$offline_launch_status" -ne 0 ] && [ "$offline_launch_status" -ne 130 ]; then
  echo "offline eval launch exited with status $offline_launch_status"
  exit "$offline_launch_status"
fi
trap - EXIT
"""


def prepare_hd_raster_script(config: ConsoleConfig, map_dir: str) -> str:
    map_path = Path(map_dir)
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
  --require-landmarks
"""


def generate_raceline_script(
    config: ConsoleConfig,
    map_dir: str,
    *,
    vehicle_width_m: float = DEFAULT_RACELINE_VEHICLE_WIDTH_M,
    safety_margin_m: float = DEFAULT_RACELINE_SAFETY_MARGIN_M,
) -> str:
    map_path = Path(map_dir)
    name = map_path.name
    vehicle_width = _nonnegative_finite(vehicle_width_m, "vehicle_width_m")
    safety_margin = _nonnegative_finite(safety_margin_m, "safety_margin_m")
    return f"""set -euo pipefail
{_q(config.python_bin)} {_q(config.python_ws / "map_tools" / "generate_raceline.py")} \\
  --centerline {_q(map_path / f"{name}_hd_map_centerline.csv")} \\
  --output {_q(map_path / f"{name}_raceline.csv")} \\
  --vehicle-width-m {vehicle_width:.9g} \\
  --safety-margin-m {safety_margin:.9g} \\
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
