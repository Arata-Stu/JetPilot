from __future__ import annotations

import shlex
from pathlib import Path

from .config import ConsoleConfig


def _q(value: str | Path) -> str:
    return shlex.quote(str(value))


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
    rviz_value = "true" if enable_rviz else "false"
    snapshot = Path(map_dir) / "vslam_reference_snapshot.json"
    cuvslam_map = Path(map_dir) / "cuvslam_map"
    return f"""set -euo pipefail
{_source_ros_setup(config)}
mkdir -p {_q(map_dir)}
export FOUNDATIONSTEREO_MODEL_RES={_q(fs_model_res)}
echo "[stage] create cuVGL map"
ros2 run isaac_mapping_ros create_map_offline.py \\
  --sensor_data_bag={_q(rosbag)} \\
  --base_output_folder={_q(map_dir)} \\
  --camera_topic_config={_q(topic_config_path)} \\
  --fs_model_res={_q(fs_model_res)} \\
  --steps_to_run {create_steps}
echo "[stage] offline eval for cuVSLAM snapshot"
ros2 launch {_q(config.launch_package)} bringup.launch.py \\
  use_sim_time:=true \\
  enable_rosbag_replay:=true \\
  rosbag_start_delay_s:=5.0 \\
  replay_additional_args:=--clock \\
  enable_operation:=false \\
  enable_control:=false \\
  enable_vehicle:=false \\
  enable_localization:=true \\
  vslam_enable_slam:=true \\
  vslam_enable_visualization:=true \\
  vslam_save_map_folder_path:={_q(cuvslam_map)} \\
  enable_vgl:=true \\
  vgl_topic_config_file:={_q(topic_config_path)} \\
  vgl_model_dir:={_q(output_model_dir)} \\
  enable_rviz:={rviz_value} \\
  enable_tool:=true \\
  enable_bag_manager:=false \\
  enable_joy:=false \\
  enable_teleop:=false \\
  enable_rc_serial:=false \\
  enable_vslam_snapshot:=true \\
  vslam_snapshot_output:={_q(snapshot)} \\
  vslam_snapshot_landmarks_topic:=/visual_slam/vis/landmarks_cloud \\
  vslam_snapshot_write_interval_s:=5.0 \\
  rosbag:={_q(rosbag)} \\
  map_dir:={_q(map_dir)}
"""


def prepare_hd_raster_script(config: ConsoleConfig, map_dir: str) -> str:
    map_path = Path(map_dir)
    return f"""set -euo pipefail
{_source_ros_setup(config)}
ros2 run vslam_map_tools export_aligned_landmarks_offline.py \\
  --snapshot {_q(map_path / "vslam_reference_snapshot.json")} \\
  --output-image {_q(map_path / "vslam_landmarks.png")} \\
  --output-yaml {_q(map_path / "vslam_landmarks.yaml")} \\
  --no-path \\
  --require-landmarks
"""


def generate_raceline_script(config: ConsoleConfig, map_dir: str) -> str:
    map_path = Path(map_dir)
    name = map_path.name
    return f"""set -euo pipefail
{_q(config.python_bin)} {_q(config.python_ws / "map_tools" / "generate_raceline.py")} \\
  --centerline {_q(map_path / f"{name}_hd_map_centerline.csv")} \\
  --output {_q(map_path / f"{name}_raceline.csv")} \\
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
