from __future__ import annotations

import os
import shlex
import subprocess
from pathlib import Path


def find_launcher() -> Path:
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "scripts" / "bringup.sh"
        if candidate.is_file():
            return candidate
    raise RuntimeError("scripts/bringup.sh was not found from the test source tree")


LAUNCHER = find_launcher()


def run_launcher(
    *arguments: str,
    check: bool = True,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(LAUNCHER), *arguments],
        check=check,
        capture_output=True,
        text=True,
        env=env,
    )


def test_presets_are_listed() -> None:
    output = run_launcher("--list-presets").stdout

    for preset in (
        "vehicle-pca",
        "vehicle-vesc",
        "localization",
        "localize-live",
        "replay-localization",
        "offline-vslam",
        "offline-vslam-map",
        "offline-localization",
        "drive-pca",
        "drive-vesc",
        "runtime-pca",
        "runtime-vesc",
        "custom",
    ):
        assert preset in output


def test_vehicle_presets_select_matching_driver_configuration() -> None:
    pca = run_launcher("vehicle-pca", "--dry-run").stdout
    vesc = run_launcher("vehicle-vesc", "--dry-run").stdout

    assert "enable_vehicle:=true" in pca
    assert "vehicle_interface_pkg:=pca9685_rc_driver" in pca
    assert "pca9685_rc_driver_node.param.yaml" in pca

    assert "enable_vehicle:=true" in vesc
    assert "vehicle_interface_pkg:=jetpilot_vesc_interface" in vesc
    assert "vehicle_interface_launch:=launch/vesc_interface.launch.xml" in vesc
    assert "vesc_interface.param.yaml" in vesc


def test_drive_presets_enable_live_sensor_teleop_and_vehicle() -> None:
    output = run_launcher("drive-vesc", "--dry-run").stdout

    assert "enable_sensor_kit:=true" in output
    assert "enable_tool:=true" in output
    assert "enable_bag_manager:=false" in output
    assert "enable_joy:=true" in output
    assert "enable_teleop:=true" in output
    assert "enable_operation:=true" in output
    assert "enable_vehicle:=true" in output
    assert "vehicle_interface_pkg:=jetpilot_vesc_interface" in output
    assert "enable_localization:=false" in output


def test_bag_manager_can_be_enabled_explicitly_for_drive_presets() -> None:
    output = run_launcher("drive-vesc", "--bag-manager", "--dry-run").stdout

    assert "enable_tool:=true" in output
    assert "enable_bag_manager:=true" in output


def test_bag_manager_can_be_disabled_explicitly_for_custom_components() -> None:
    output = run_launcher(
        "custom",
        "--components",
        "sensor,bag-manager,joy,teleop,operation,vehicle-vesc",
        "--no-bag-manager",
        "--dry-run",
    ).stdout

    assert "enable_tool:=true" in output
    assert "enable_bag_manager:=false" in output


def test_sensor_kit_launch_can_be_selected_explicitly() -> None:
    output = run_launcher(
        "drive-vesc",
        "--sensor-kit",
        "realsense-silky",
        "--dry-run",
    ).stdout

    assert "enable_sensor_kit:=true" in output
    assert "sensor_kit_interface_pkg:=jetpilot_system_launch" in output
    assert (
        "sensor_kit_interface_launch:=launch/sensors/realsense_silky_evcam.launch.py"
        in output
    )
    assert "sensor_kit_camera_name:=realsense" in output


def test_unknown_sensor_kit_is_rejected() -> None:
    result = run_launcher(
        "drive-vesc",
        "--sensor-kit",
        "unknown-camera",
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "sensor kit must be realsense or realsense-silky" in result.stderr


def test_empty_launch_arguments_are_not_emitted() -> None:
    output = run_launcher("vehicle-vesc", "--dry-run").stdout

    assert "rosbag:=" not in output


def test_custom_components_can_be_selected_in_one_argument() -> None:
    output = run_launcher(
        "custom",
        "--components",
        "sensor,joy,teleop,operation,vehicle-vesc",
        "--dry-run",
    ).stdout

    assert "enable_sensor_kit:=true" in output
    assert "enable_tool:=true" in output
    assert "enable_joy:=true" in output
    assert "enable_teleop:=true" in output
    assert "enable_operation:=true" in output
    assert "enable_vehicle:=true" in output
    assert "vehicle_interface_pkg:=jetpilot_vesc_interface" in output
    assert "enable_localization:=false" in output


def test_custom_control_enables_planning_and_pure_pursuit() -> None:
    output = run_launcher(
        "custom",
        "--components",
        "control",
        "--dry-run",
    ).stdout

    assert "enable_planning:=true" in output
    assert "enable_control:=true" in output
    assert "enable_operation:=true" in output


def test_custom_planning_can_run_without_controller() -> None:
    output = run_launcher(
        "custom",
        "--components",
        "planning",
        "--dry-run",
    ).stdout

    assert "enable_planning:=true" in output
    assert "enable_control:=false" in output


def test_raceline_option_enables_loader_and_matching_selector(tmp_path: Path) -> None:
    raceline = tmp_path / "course raceline.csv"
    raceline.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    output = run_launcher(
        "custom",
        "--components",
        "control",
        "--raceline",
        str(raceline),
        "--dry-run",
    ).stdout

    assert "enable_raceline_publisher:=true" in output
    assert "route_lane_selector.raceline.param.yaml" in output
    assert f"raceline_root:={tmp_path}" in output
    assert "raceline_csv:=course\\ raceline.csv" in output


def test_raceline_component_requires_csv() -> None:
    result = run_launcher(
        "custom", "--components", "raceline", "--dry-run", check=False
    )

    assert result.returncode != 0
    assert "requires --raceline" in result.stderr


def test_custom_components_reject_conflicting_input_sources() -> None:
    result = run_launcher(
        "custom",
        "--components",
        "sensor,replay",
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "either sensor or replay" in result.stderr


def test_custom_localization_requires_map_when_noninteractive() -> None:
    result = run_launcher(
        "custom",
        "--components",
        "sensor,localization,vehicle-vesc",
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "requires --map" in result.stderr


def test_custom_localization_accepts_map_argument(tmp_path: Path) -> None:
    output = run_launcher(
        "custom",
        "--components",
        "sensor,localization,vehicle-vesc",
        "--map",
        str(tmp_path),
        "--dry-run",
    ).stdout

    assert "enable_localization:=true" in output
    assert f"map_dir:={tmp_path}" in output


def test_live_localization_publishes_description_without_actuator(tmp_path: Path) -> None:
    output = run_launcher(
        "localize-live", "--map", str(tmp_path), "--dry-run"
    ).stdout

    assert "enable_sensor_kit:=true" in output
    assert "enable_localization:=true" in output
    assert "enable_vehicle:=false" in output
    assert "publish_vehicle_description:=true" in output


def test_replay_preset_is_safe_and_requires_inputs(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    map_dir = tmp_path / "map"
    bag.mkdir()
    map_dir.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")

    output = run_launcher(
        "replay-localization",
        "--bag",
        str(bag),
        "--map",
        str(map_dir),
        "--rate",
        "0.5",
        "--dry-run",
    ).stdout

    assert "enable_rosbag_replay:=true" in output
    assert "use_sim_time:=true" in output
    assert "enable_vehicle:=false" in output
    assert "allow_unsafe_replay_control_topics:=false" in output
    assert "replay_rate:=0.5" in output

    missing_map = run_launcher("localize-live", "--dry-run", check=False)
    assert missing_map.returncode != 0
    assert "requires --map" in missing_map.stderr


def test_offline_vslam_replays_bag_without_map(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")

    output = run_launcher(
        "offline-vslam",
        "--bag",
        str(bag),
        "--dry-run",
    ).stdout

    assert "enable_rosbag_replay:=true" in output
    assert "use_sim_time:=true" in output
    assert "enable_sensor_kit:=false" in output
    assert "enable_localization:=true" in output
    assert "enable_vslam:=true" in output
    assert "vslam_enable_slam:=true" in output
    assert "vslam_enable_visualization:=true" in output
    assert "enable_vgl:=false" in output
    assert "enable_vehicle:=false" in output
    assert "rviz_config_file:=" in output
    assert "vslam_debug.rviz" in output
    assert "map_dir:=" not in output


def test_rviz_config_can_be_selected_explicitly_for_rviz_presets(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")

    output = run_launcher(
        "offline-vslam",
        "--bag",
        str(bag),
        "--rviz-config",
        "default",
        "--dry-run",
    ).stdout

    assert "enable_rviz:=true" in output
    assert "rviz_config_file:=" in output
    assert "default.rviz" in output
    assert "vslam_debug.rviz" not in output

    invalid = run_launcher(
        "offline-vslam",
        "--bag",
        str(bag),
        "--rviz-config",
        "unknown",
        "--dry-run",
        check=False,
    )
    assert invalid.returncode != 0
    assert "rviz config must be default, vslam-debug, or an absolute path" in invalid.stderr


def test_offline_vslam_map_runs_mapping_debug_without_output_map(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    map_dir = tmp_path / "map"
    bag.mkdir()
    map_dir.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")

    output = run_launcher(
        "offline-vslam-map",
        "--bag",
        str(bag),
        "--map",
        str(map_dir),
        "--dry-run",
    ).stdout

    assert "enable_rosbag_replay:=true" in output
    assert "vslam_enable_slam:=true" in output
    assert "vslam_enable_visualization:=true" in output
    assert "enable_vslam_snapshot:=false" in output
    assert "vslam_save_map_folder_path:=" not in output
    assert "vslam_snapshot_output:=" not in output
    assert f"map_dir:={map_dir}" in output

    missing_map = run_launcher(
        "offline-vslam-map",
        "--bag",
        str(bag),
        "--dry-run",
        check=False,
    )
    assert missing_map.returncode != 0
    assert "requires --map" in missing_map.stderr


def test_offline_localization_uses_vgl_and_vslam_with_map(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    map_dir = tmp_path / "map"
    bag.mkdir()
    map_dir.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")

    output = run_launcher(
        "offline-localization",
        "--bag",
        str(bag),
        "--map",
        str(map_dir),
        "--dry-run",
    ).stdout

    assert "enable_rosbag_replay:=true" in output
    assert "enable_vslam:=true" in output
    assert "enable_vgl:=true" in output
    assert "vslam_localize_on_startup:=true" in output
    assert "enable_vehicle:=false" in output


def test_overrides_replace_instead_of_duplicate() -> None:
    output = run_launcher(
        "localize-live",
        "--map",
        "/tmp/map with space",
        "--dry-run",
        "--",
        "enable_rviz:=false",
    ).stdout

    assert output.count("enable_rviz:=false") == 1
    assert "enable_rviz:=true" not in output
    assert "map_dir:=/tmp/map\\ with\\ space" in output


def test_replay_vehicle_and_unsafe_overrides_are_rejected() -> None:
    replay_vehicle = run_launcher(
        "replay-localization",
        "--bag",
        "/tmp/bag",
        "--map",
        "/tmp/map",
        "--vehicle",
        "pca",
        "--dry-run",
        check=False,
    )
    assert replay_vehicle.returncode != 0
    assert "cannot be enabled together" in replay_vehicle.stderr

    unsafe = run_launcher(
        "sensor",
        "--dry-run",
        "--",
        "allow_unsafe_replay_control_topics:=true",
        check=False,
    )
    assert unsafe.returncode != 0
    assert "intentionally unsupported" in unsafe.stderr


def test_every_noninteractive_preset_emits_unique_launch_arguments(tmp_path: Path) -> None:
    bag = tmp_path / "bag"
    map_dir = tmp_path / "map"
    bag.mkdir()
    map_dir.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")

    presets = (
        "sensor",
        "localization-only",
        "localization",
        "localize-live",
        "replay-localization",
        "offline-vslam",
        "offline-vslam-map",
        "offline-localization",
        "vehicle-pca",
        "vehicle-vesc",
        "teleop-pca",
        "teleop-vesc",
        "drive-pca",
        "drive-vesc",
        "runtime-pca",
        "runtime-vesc",
    )
    for preset in presets:
        arguments = [preset, "--dry-run"]
        if preset in {
            "localization-only",
            "localization",
            "localize-live",
            "replay-localization",
            "offline-vslam-map",
            "offline-localization",
            "runtime-pca",
            "runtime-vesc",
        }:
            arguments += ["--map", str(map_dir)]
        if preset in {
            "replay-localization",
            "offline-vslam",
            "offline-vslam-map",
            "offline-localization",
        }:
            arguments += ["--bag", str(bag)]

        output = run_launcher(*arguments).stdout
        command_text = output.split("Command:\n  ", 1)[1].split("\n\n", 1)[0]
        command = shlex.split(command_text)
        launch_arguments = command[4:]
        names = [argument.split(":=", 1)[0] for argument in launch_arguments]

        assert len(names) == len(set(names)), preset
        assert "enable_control:=false" in launch_arguments


def test_exec_uses_an_argument_array_for_paths_with_spaces(tmp_path: Path) -> None:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    capture_path = tmp_path / "captured.bin"
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ \"$1\" == pkg && \"$2\" == prefix ]]; then exit 0; fi\n"
        "printf '%s\\0' \"$@\" > \"$ROS_CAPTURE\"\n"
    )
    fake_ros2.chmod(0o755)
    map_dir = tmp_path / "map with space"
    map_dir.mkdir()
    env = dict(os.environ)
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    env["ROS_CAPTURE"] = str(capture_path)

    run_launcher(
        "localization",
        "--map",
        str(map_dir),
        "--yes",
        env=env,
    )

    arguments = capture_path.read_bytes().split(b"\0")
    assert f"map_dir:={map_dir}".encode() in arguments
