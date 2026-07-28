from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
from pathlib import Path


def find_launcher() -> Path:
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "scripts" / "bringup.sh"
        if candidate.is_file():
            return candidate
    raise RuntimeError("scripts/bringup.sh was not found from the test source tree")


LAUNCHER = find_launcher()
PROJECT_ROOT = LAUNCHER.parents[1]
PROFILE_ROOT = (
    PROJECT_ROOT
    / "ros2_ws/src/launch/jetpilot_system_launch/config/bringup_profiles"
)


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
        "vehicle",
        "teleop",
        "drive",
        "runtime",
        "localization",
        "localize-live",
        "replay-localization",
        "offline-vslam",
        "offline-vslam-map",
        "offline-localization",
        "custom",
    ):
        assert preset in output


def test_vehicle_and_sensor_profiles_are_listed_dynamically() -> None:
    vehicles = run_launcher("--list-vehicles").stdout
    sensor_kits = run_launcher("--list-sensor-kits").stdout

    assert "pca" in vehicles
    assert "PCA9685 RC vehicle interface" in vehicles
    assert "vesc" in vehicles
    assert "VESC vehicle interface" in vehicles
    assert "realsense" in sensor_kits
    assert "flir" in sensor_kits
    assert "realsense-silky" in sensor_kits


def test_bringup_profiles_pass_schema_validation() -> None:
    output = run_launcher("--validate-profiles").stdout

    assert "vehicle: 2 profile(s)" in output
    assert "sensor_kit: 4 profile(s)" in output
    assert "validated: 6 profile(s)" in output


def test_new_manifest_is_available_without_editing_launcher(tmp_path: Path) -> None:
    profile_root = tmp_path / "bringup_profiles"
    shutil.copytree(PROFILE_ROOT, profile_root)
    mock_profile = {
        "schema_version": 1,
        "kind": "vehicle",
        "id": "mock",
        "label": "Mock vehicle interface",
        "order": 90,
        "aliases": [],
        "launch": {
            "package": "mock_vehicle_interface",
            "file": "launch/mock_vehicle_interface.launch.xml",
        },
        "driver_param": {
            "package": "mock_vehicle_interface",
            "path": "config/mock.param.yaml",
        },
        "arguments": {
            "publish_vehicle_description": True,
            "publish_vehicle_evs_description": False,
            "publish_vehicle_thremo_description": False,
        },
    }
    (profile_root / "vehicle/mock.json").write_text(
        json.dumps(mock_profile), encoding="utf-8"
    )
    mock_sensor_profile = {
        "schema_version": 1,
        "kind": "sensor_kit",
        "id": "mock-sensor",
        "label": "Mock sensor kit",
        "order": 90,
        "aliases": [],
        "launch": {
            "package": "mock_sensor_kit",
            "file": "launch/mock_sensor_kit.launch.py",
        },
        "arguments": {
            "sensor_kit_camera_name": "mock",
            "sensor_kit_rtp_image_topic": "/mock/image_raw",
        },
        "rtp_topics": ["/mock/image_raw"],
    }
    (profile_root / "sensor_kit/mock-sensor.json").write_text(
        json.dumps(mock_sensor_profile), encoding="utf-8"
    )
    env = dict(os.environ)
    env["BRINGUP_PROFILE_ROOT"] = str(profile_root)

    listed_vehicles = run_launcher("--list-vehicles", env=env).stdout
    listed_sensors = run_launcher("--list-sensor-kits", env=env).stdout
    vehicle_output = run_launcher(
        "vehicle", "--vehicle", "mock", "--dry-run", env=env
    ).stdout
    sensor_output = run_launcher(
        "sensor", "--sensor-kit", "mock-sensor", "--dry-run", env=env
    ).stdout

    assert "mock" in listed_vehicles
    assert "Mock vehicle interface" in listed_vehicles
    assert "vehicle      : mock" in vehicle_output
    assert "vehicle_interface_pkg:=mock_vehicle_interface" in vehicle_output
    assert (
        "vehicle_interface_launch:=launch/mock_vehicle_interface.launch.xml"
        in vehicle_output
    )
    assert "mock-sensor" in listed_sensors
    assert "Mock sensor kit" in listed_sensors
    assert "sensor_kit_interface_pkg:=mock_sensor_kit" in sensor_output
    assert "sensor_kit_interface_launch:=launch/mock_sensor_kit.launch.py" in (
        sensor_output
    )


def test_generic_vehicle_presets_accept_an_explicit_interface() -> None:
    pca = run_launcher("vehicle", "--vehicle", "pca", "--dry-run").stdout
    vesc = run_launcher("drive", "--vehicle", "vesc", "--dry-run").stdout

    assert "preset       : vehicle" in pca
    assert "vehicle      : pca" in pca
    assert "vehicle_interface_pkg:=pca9685_rc_driver" in pca

    assert "preset       : drive" in vesc
    assert "vehicle      : vesc" in vesc
    assert "enable_sensor_kit:=true" in vesc
    assert "enable_teleop:=true" in vesc
    assert "vehicle_interface_pkg:=jetpilot_vesc_interface" in vesc


def test_generic_vehicle_preset_requires_an_interface_noninteractively() -> None:
    result = run_launcher("drive", "--dry-run", check=False)

    assert result.returncode != 0
    assert "requires --vehicle PROFILE" in result.stderr


def test_vehicle_presets_select_matching_driver_configuration() -> None:
    # Legacy aliases remain supported for existing scripts.
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
    assert "enable_jetson_stats:=true" in output
    assert "enable_bag_manager:=false" in output
    assert "enable_joy:=true" in output
    assert "enable_teleop:=true" in output
    assert "enable_operation:=true" in output
    assert "enable_vehicle:=true" in output
    assert "vehicle_interface_pkg:=jetpilot_vesc_interface" in output
    assert "enable_localization:=false" in output


def test_jetson_stats_can_be_disabled_explicitly() -> None:
    output = run_launcher(
        "drive-vesc",
        "--set",
        "enable_jetson_stats:=false",
        "--dry-run",
    ).stdout

    assert "enable_jetson_stats:=false" in output


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


def test_openeb_raw_recording_follows_bag_manager_session() -> None:
    project_root = LAUNCHER.parents[1]
    bringup_source = (
        project_root
        / "ros2_ws/src/launch/jetpilot_system_launch/launch/bringup.launch.py"
    ).read_text(encoding="utf-8")
    bag_manager_source = (
        project_root
        / "ros2_ws/src/tool/jetpilot_bag_tools/jetpilot_bag_tools/bag_manager_node.py"
    ).read_text(encoding="utf-8")
    openeb_driver_source = (
        project_root
        / "ros2_ws/src/sensing/openeb_ros2/src/driver_component.cpp"
    ).read_text(encoding="utf-8")
    bag_manager_config = (
        project_root
        / "ros2_ws/src/launch/jetpilot_system_launch/config/tool/bag_manager.param.yaml"
    ).read_text(encoding="utf-8")

    assert (
        "args.add_arg('sensor_kit_silky_evcam_raw_recording_enabled', True"
        in bringup_source
    )
    assert (
        "args.add_arg('sensor_kit_silky_evcam_raw_recording_auto_start', False"
        in bringup_source
    )
    assert "self.wait_for_recording_directory()" in bag_manager_source
    assert "BagRequest.START,\n                    self.current_uri" in bag_manager_source
    assert "self.publish_raw_recording_request(BagRequest.STOP" in bag_manager_source
    assert '"--max-bag-duration"' in bag_manager_source
    assert "self.handle_scheduled_raw_split" in bag_manager_source
    assert "recording_split_duration_s: 0" in bag_manager_config
    assert "max_bag_duration:" not in bag_manager_config
    assert "sensor_kit_silky_evcam_raw_recording_split_duration_s" not in (
        bringup_source
    )
    assert "requested_path.is_absolute()" in openeb_driver_source
    assert "raw_recording_dir_ = requested_path.lexically_normal().string()" in (
        openeb_driver_source
    )


def test_flir_sensor_kit_launches_can_be_selected_explicitly() -> None:
    flir = run_launcher(
        "drive-vesc",
        "--sensor-kit",
        "flir",
        "--dry-run",
    ).stdout
    all_sensors = run_launcher(
        "drive-vesc",
        "--sensor-kit",
        "realsense-silky-flir",
        "--dry-run",
    ).stdout

    assert "sensor_kit_interface_launch:=launch/sensors/flir_boson.launch.py" in flir
    assert "sensor_kit_rtp_image_topic:=/flir/image_raw" in flir
    assert (
        "sensor_kit_interface_launch:=launch/sensors/realsense_silky_flir.launch.py"
        in all_sensors
    )
    assert "sensor_kit_flir_video_device:=/dev/video0" in all_sensors
    assert "sensor_kit_flir_pixel_format:=mono16" in all_sensors


def test_unknown_sensor_kit_is_rejected() -> None:
    result = run_launcher(
        "drive-vesc",
        "--sensor-kit",
        "unknown-camera",
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "unknown sensor_kit profile 'unknown-camera'" in result.stderr
    assert "available:" in result.stderr


def test_rtp_destination_and_topic_can_be_overridden() -> None:
    output = run_launcher(
        "sensor",
        "--dry-run",
        "--set",
        "sensor_kit_enable_rtp_stream:=true",
        "--set",
        "sensor_kit_rtp_host:=192.168.1.10",
        "--set",
        "sensor_kit_rtp_port:=5006",
        "--set",
        "sensor_kit_rtp_image_topic:=/realsense/infra1/image_rect_raw",
    ).stdout

    assert "sensor_kit_enable_rtp_stream:=true" in output
    assert "sensor_kit_rtp_host:=192.168.1.10" in output
    assert "sensor_kit_rtp_port:=5006" in output
    assert "sensor_kit_rtp_image_topic:=/realsense/infra1/image_rect_raw" in output
    assert "RTP receiver : 192.168.1.10:5006" in output


def test_rtp_requires_a_destination_host() -> None:
    result = run_launcher(
        "sensor",
        "--dry-run",
        "--set",
        "sensor_kit_enable_rtp_stream:=true",
        check=False,
    )

    assert result.returncode != 0
    assert "requires sensor_kit_rtp_host" in result.stderr


def test_rtp_rejects_invalid_destination_ports() -> None:
    for port in ("0", "65536", "not-a-port"):
        result = run_launcher(
            "sensor",
            "--dry-run",
            "--set",
            "sensor_kit_enable_rtp_stream:=true",
            "--set",
            "sensor_kit_rtp_host:=192.168.1.10",
            "--set",
            f"sensor_kit_rtp_port:={port}",
            check=False,
        )

        assert result.returncode != 0
        assert "sensor_kit_rtp_port must be an integer between 1 and 65535" in result.stderr


def test_interactive_sensor_configuration_includes_rtp_prompts() -> None:
    source = LAUNCHER.read_text(encoding="utf-8")

    assert "configure_rtp_interactively" in source
    assert "choose_one 'RTP stream'" in source
    assert "on   RTP送信 ON" in source
    assert "off  RTP送信 OFF" in source
    assert "RTP送信先IP / host" in source
    assert "RTP送信先UDP port" in source
    assert "UDP portは1〜65535の整数で入力してください。" in source
    assert "choose_one 'RTP image topic'" in source
    assert "トピックを手入力..." in source


def test_interactive_vehicle_configuration_has_one_profile_selector() -> None:
    source = LAUNCHER.read_text(encoding="utf-8")

    assert "configure_vehicle_interactively" in source
    assert "choose_one 'Vehicle interface'" in source
    assert "profile_list=\"$(list_profiles vehicle)\"" in source
    assert "profile_list=\"$(list_profiles sensor_kit)\"" in source
    assert "'vehicle            Vehicle interface (select next)'" in source
    assert "'vehicle-pca        PCA9685 vehicle interface'" not in source
    assert "'vehicle-vesc       VESC vehicle interface'" not in source


def test_flir_launch_can_load_the_rtp_component() -> None:
    flir_launch = (
        LAUNCHER.parents[1]
        / "ros2_ws"
        / "src"
        / "launch"
        / "jetpilot_system_launch"
        / "launch"
        / "sensors"
        / "flir_boson.launch.py"
    ).read_text(encoding="utf-8")

    assert "if lu.is_true(args.enable_rtp_stream):" in flir_launch
    assert "jetpilot_rtp_tools::ImageRtpSenderComponent" in flir_launch
    assert "'image_topic': args.rtp_image_topic" in flir_launch


def test_empty_launch_arguments_are_not_emitted() -> None:
    output = run_launcher("vehicle-vesc", "--dry-run").stdout

    assert "rosbag:=" not in output


def test_custom_components_can_be_selected_in_one_argument() -> None:
    output = run_launcher(
        "custom",
        "--components",
        "sensor,joy,teleop,operation,vehicle",
        "--vehicle",
        "vesc",
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
        "vehicle",
        "teleop",
        "drive",
        "runtime",
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
        if preset in {"vehicle", "teleop", "drive", "runtime"}:
            arguments += ["--vehicle", "vesc"]
        if preset in {
            "localization-only",
            "localization",
            "localize-live",
            "replay-localization",
            "offline-vslam-map",
            "offline-localization",
            "runtime-pca",
            "runtime-vesc",
            "runtime",
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
