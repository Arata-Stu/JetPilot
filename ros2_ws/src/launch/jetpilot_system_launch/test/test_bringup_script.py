from __future__ import annotations

import ast
import hashlib
import json
import os
import re
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


def test_launcher_resolves_project_root_when_scripts_are_mounted_separately(
    tmp_path: Path,
) -> None:
    detached_scripts = tmp_path / "scripts"
    detached_scripts.mkdir()
    detached_launcher = detached_scripts / "bringup.sh"
    detached_helper = detached_scripts / "launch_profiles.py"
    shutil.copy2(LAUNCHER, detached_launcher)
    shutil.copy2(LAUNCHER.with_name("launch_profiles.py"), detached_helper)
    env = dict(os.environ)
    env["ROS2_WS"] = str(PROJECT_ROOT / "ros2_ws")
    env.pop("JETPILOT_PROJECT_ROOT", None)
    env.pop("BRINGUP_PROFILE_ROOT", None)

    result = subprocess.run(
        ["bash", str(detached_launcher), "--list-sensor-kits"],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )

    assert "realsense" in result.stdout


def test_presets_are_listed() -> None:
    output = run_launcher("--list-presets").stdout

    for preset in (
        "vehicle",
        "teleop",
        "drive",
        "e2e",
        "runtime",
        "map-view",
        "competition",
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
    assert "jpbb" in vehicles
    assert "JPBB-01 USB/RC safety bridge" in vehicles
    assert "realsense" in sensor_kits
    assert "flir" in sensor_kits
    assert "realsense-silky" in sensor_kits


def test_bringup_profiles_pass_schema_validation() -> None:
    output = run_launcher("--validate-profiles").stdout

    assert "vehicle: 3 profile(s)" in output
    assert "sensor_kit: 4 profile(s)" in output
    assert "validated: 7 profile(s)" in output


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


def test_jpbb_vehicle_profile_resolves_without_changing_existing_backends() -> None:
    output = run_launcher("drive", "--vehicle", "jpbb", "--dry-run").stdout

    assert "vehicle      : jpbb" in output
    assert "vehicle_interface_pkg:=jetpilot_bridge_interface" in output
    assert (
        "vehicle_interface_launch:=launch/jetpilot_bridge_interface.launch.xml"
        in output
    )
    assert "jetpilot_bridge_interface_node.param.yaml" in output


def test_generic_vehicle_presets_require_an_interface_noninteractively() -> None:
    for preset in ("drive", "e2e"):
        result = run_launcher(preset, "--dry-run", check=False)

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


def test_drive_presets_enable_live_sensor_teleop_and_vehicle_on_jetson() -> None:
    env = dict(os.environ)
    env["ISAAC_ROS_PLATFORM"] = "arm64-jetpack"
    output = run_launcher("drive-vesc", "--dry-run", env=env).stdout

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


def test_e2e_preset_enables_direct_control_stack_without_rule_based_control() -> None:
    output = run_launcher("e2e", "--vehicle", "vesc", "--dry-run").stdout

    for argument in (
        "enable_sensor_kit:=true",
        "enable_tool:=true",
        "enable_joy:=true",
        "enable_teleop:=true",
        "enable_operation:=true",
        "enable_e2e_inference:=true",
        "enable_vehicle:=true",
    ):
        assert argument in output
    assert "enable_planning:=false" in output
    assert "enable_control:=false" in output
    assert "enable_localization:=false" in output
    assert "E2E inference: true" in output


def test_competition_preset_enables_complete_rule_based_stack() -> None:
    output = run_launcher(
        "competition",
        "--vehicle",
        "vesc",
        "--map",
        "/workspaces/map/course_a",
        "--dry-run",
    ).stdout

    for argument in (
        "enable_sensor_kit:=true",
        "enable_localization:=true",
        "enable_hd_map_publisher:=true",
        "enable_section_localizer:=true",
        "enable_competition_planning:=true",
        "enable_object_detection:=true",
        "object_detection_detections_topic:=/perception/signal/detections",
        "enable_control:=true",
        "enable_operation:=true",
        "enable_vehicle:=true",
        "competition_route_config_file:=/workspaces/map/course_a/competition_route.param.yaml",
    ):
        assert argument in output
    assert "enable_planning:=false" in output
    assert "enable_e2e_inference:=false" in output


def test_map_view_preset_enables_foxglove_hd_map_without_actuation() -> None:
    output = run_launcher(
        "map-view",
        "--map",
        "/workspaces/map/course_a",
        "--dry-run",
    ).stdout

    for argument in (
        "enable_sensor_kit:=true",
        "enable_localization:=true",
        "enable_foxglove:=true",
        "enable_vgl:=false",
        "enable_hd_map_publisher:=true",
        "enable_section_localizer:=true",
    ):
        assert argument in output
    assert "enable_competition_planning:=false" in output
    assert "enable_planning:=false" in output
    assert "enable_control:=false" in output
    assert "enable_vehicle:=false" in output
    assert "VSLAM init   : foxglove (/initialpose required; VGL off)" in output


def test_required_map_is_selected_before_vehicle_and_sensor_prompts() -> None:
    source = LAUNCHER.read_text(encoding="utf-8")
    selection = source.index(
        'if [[ "$REQUIRES_MAP" == \'true\' && -z "$MAP_DIR" '
        '&& "$INTERACTIVE" == \'true\' ]]; then\n  discover_map'
    )
    vehicle = source.index("configure_vehicle_interactively", selection)
    sensor = source.index("configure_sensor_kit_interactively", selection)

    assert selection < vehicle
    assert selection < sensor
    assert 'if ((${#options[@]} == 1)); then\n    MAP_DIR="${options[0]}"' not in source


def test_competition_and_standard_planning_cannot_start_together() -> None:
    result = run_launcher(
        "competition",
        "--vehicle",
        "vesc",
        "--map",
        "/workspaces/map/course_a",
        "--set",
        "enable_planning:=true",
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "cannot both be true" in result.stderr


def test_x86_disables_jetson_stats_by_default() -> None:
    env = dict(os.environ)
    env["ISAAC_ROS_PLATFORM"] = "amd64"
    output = run_launcher("drive-vesc", "--dry-run", env=env).stdout

    assert "enable_jetson_stats:=false" in output


def test_x86_rejects_explicit_jetson_stats_enable() -> None:
    env = dict(os.environ)
    env["ISAAC_ROS_PLATFORM"] = "amd64"
    result = run_launcher(
        "drive-vesc",
        "--set",
        "enable_jetson_stats:=true",
        "--dry-run",
        check=False,
        env=env,
    )

    assert result.returncode != 0
    assert "enable_jetson_stats is available only on Jetson" in result.stderr


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


def test_vslam_tracking_mode_can_switch_between_vo_and_vio() -> None:
    vo = run_launcher(
        "custom",
        "--components",
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--vslam-mode",
        "vo",
        "--dry-run",
    ).stdout
    vio = run_launcher(
        "custom",
        "--components",
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--vslam-mode",
        "vio",
        "--dry-run",
    ).stdout

    assert "VSLAM mode   : vo" in vo
    assert "vslam_mode:=vo" in vo
    assert "VSLAM mode   : vio" in vio
    assert "vslam_mode:=vio" in vio


def test_invalid_vslam_tracking_mode_is_rejected() -> None:
    result = run_launcher(
        "custom",
        "--components",
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--vslam-mode",
        "invalid",
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "VSLAM mode must be vo or vio" in result.stderr


def test_realsense_vio_topic_and_d455_mount_offset_are_configured() -> None:
    realsense_source = (
        PROJECT_ROOT
        / "ros2_ws/src/launch/jetpilot_system_launch/launch/sensors/realsense.launch.py"
    ).read_text(encoding="utf-8")
    bringup_source = (
        PROJECT_ROOT
        / "ros2_ws/src/launch/jetpilot_system_launch/launch/bringup.launch.py"
    ).read_text(encoding="utf-8")

    assert "'unite_imu_method': 2" in realsense_source
    assert "args.add_arg('vslam_imu_topic', '/realsense/imu'" in bringup_source
    assert "args.add_arg('vehicle_description_camera_y', '0.0115'" in bringup_source


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


def test_foxglove_custom_component_uses_hd_map_defaults() -> None:
    default_output = run_launcher("sensor", "--dry-run").stdout
    custom_without_foxglove = run_launcher(
        "custom",
        "--components",
        "sensor,localization,hd-map",
        "--map",
        "/workspaces/map/course_a",
        "--dry-run",
    ).stdout
    enabled_output = run_launcher(
        "custom",
        "--components",
        "sensor,hd-map,foxglove",
        "--map",
        "/workspaces/map/course_a",
        "--dry-run",
    ).stdout

    assert "enable_foxglove:=false" in default_output
    assert "Foxglove     : disabled" in default_output
    assert "enable_localization:=true" in custom_without_foxglove
    assert "enable_hd_map_publisher:=true" in custom_without_foxglove
    assert "enable_foxglove:=false" in custom_without_foxglove
    assert "Foxglove     : disabled" in custom_without_foxglove
    assert "enable_foxglove:=true" in enabled_output
    assert "enable_hd_map_publisher:=true" in enabled_output
    assert "foxglove_address:=0.0.0.0" in enabled_output
    assert "foxglove_port:=8767" in enabled_output
    assert "initialpose" in enabled_output
    assert "lane_markers" in enabled_output
    assert "diagnostics" in enabled_output
    assert "Foxglove     : bind 0.0.0.0:8767" in enabled_output


def test_live_localization_presets_start_foxglove_pose_fallback() -> None:
    cases = (
        ("localization-only", "--map", "/workspaces/map/course_a", "--dry-run"),
        ("localization", "--map", "/workspaces/map/course_a", "--dry-run"),
        ("localize-live", "--map", "/workspaces/map/course_a", "--dry-run"),
        (
            "runtime",
            "--vehicle",
            "vesc",
            "--map",
            "/workspaces/map/course_a",
            "--dry-run",
        ),
        ("runtime-pca", "--map", "/workspaces/map/course_a", "--dry-run"),
        ("runtime-vesc", "--map", "/workspaces/map/course_a", "--dry-run"),
    )

    for arguments in cases:
        output = run_launcher(*arguments).stdout

        assert "enable_localization:=true" in output
        assert "enable_localization_manager:=true" in output
        assert "vslam_localize_on_startup:=false" in output
        assert "enable_vgl:=true" in output
        assert "enable_foxglove:=true" in output
        assert "VSLAM init   : pose-hint" in output
        assert "Foxglove     : bind 0.0.0.0:8767" in output


def test_map_origin_initialization_disables_vgl_but_keeps_safety_status() -> None:
    output = run_launcher(
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--localization-init",
        "map-origin",
        "--set",
        "enable_vgl:=true",
        "--dry-run",
    ).stdout

    assert "vslam_localize_on_startup:=true" in output
    assert output.count("enable_vgl:=false") == 1
    assert "enable_vgl:=true" not in output
    assert "enable_localization_manager:=true" in output
    assert "enable_foxglove:=true" in output
    assert "VSLAM init   : map-origin (VGL off; restart with pose-hint on failure)" in output


def test_pose_hint_mode_can_use_foxglove_without_vgl() -> None:
    output = run_launcher(
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--pose-hint",
        "--set",
        "enable_vgl:=false",
        "--dry-run",
    ).stdout

    assert "vslam_localize_on_startup:=false" in output
    assert "enable_vgl:=false" in output
    assert "enable_localization_manager:=true" in output
    assert "enable_foxglove:=true" in output
    assert "VSLAM init   : pose-hint" in output


def test_foxglove_initialization_waits_for_manual_pose_without_vgl() -> None:
    output = run_launcher(
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--localization-init",
        "foxglove",
        "--set",
        "enable_vgl:=true",
        "--set",
        "enable_foxglove:=false",
        "--set",
        "vslam_localize_on_startup:=true",
        "--dry-run",
    ).stdout

    assert "vslam_localize_on_startup:=false" in output
    assert output.count("enable_vgl:=false") == 1
    assert "enable_vgl:=true" not in output
    assert output.count("enable_foxglove:=true") == 1
    assert "enable_foxglove:=false" not in output
    assert "enable_localization_manager:=true" in output
    assert "VSLAM init   : foxglove (/initialpose required; VGL off)" in output
    assert "Foxglove     : bind 0.0.0.0:8767" in output


def test_foxglove_initialization_survives_custom_component_selection() -> None:
    output = run_launcher(
        "custom",
        "--components",
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--localization-init",
        "foxglove",
        "--dry-run",
    ).stdout

    assert "enable_localization:=true" in output
    assert "enable_localization_manager:=true" in output
    assert "vslam_localize_on_startup:=false" in output
    assert "enable_vgl:=false" in output
    assert "enable_foxglove:=true" in output


def test_localization_init_rejects_invalid_mode() -> None:
    result = run_launcher(
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--localization-init",
        "unknown",
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "localization initialization must be pose-hint, foxglove, or map-origin" in (
        result.stderr
    )


def test_map_origin_requires_localization_manager() -> None:
    result = run_launcher(
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--no-pose-hint",
        "--set",
        "enable_localization_manager:=false",
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "requires the localization manager for safety status" in result.stderr


def test_map_origin_requires_vslam_localization_mode() -> None:
    result = run_launcher(
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--no-pose-hint",
        "--set",
        "vslam_enable_slam:=false",
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "requires vslam_enable_slam=true" in result.stderr


def test_external_localization_modes_reject_mapping_output() -> None:
    for mode in ("map-origin", "foxglove"):
        result = run_launcher(
            "localization",
            "--map",
            "/workspaces/map/course_a",
            "--localization-init",
            mode,
            "--set",
            "vslam_save_map_folder_path:=/workspaces/map/new_map/cuvslam_map",
            "--dry-run",
            check=False,
        )

        assert result.returncode != 0
        assert "cannot be combined with vslam_save_map_folder_path" in result.stderr


def test_map_origin_requires_saved_cuvslam_map(tmp_path: Path) -> None:
    missing_directory = run_launcher(
        "localization",
        "--map",
        str(tmp_path),
        "--no-pose-hint",
        check=False,
    )

    assert missing_directory.returncode != 0
    assert "map-origin initialization requires a saved cuVSLAM map" in (
        missing_directory.stderr
    )

    cuvslam_map = tmp_path / "cuvslam_map"
    cuvslam_map.mkdir()
    (cuvslam_map / "not_a_map.txt").write_text("invalid", encoding="utf-8")
    missing_database = run_launcher(
        "localization",
        "--map",
        str(tmp_path),
        "--no-pose-hint",
        check=False,
    )

    assert missing_database.returncode != 0
    assert "requires a cuVSLAM .mdb database" in missing_database.stderr


def test_foxglove_initialization_requires_saved_map(tmp_path: Path) -> None:
    missing_map = run_launcher(
        "localization",
        "--map",
        str(tmp_path),
        "--localization-init",
        "foxglove",
        check=False,
    )

    assert missing_map.returncode != 0
    assert "foxglove initialization requires a saved cuVSLAM map" in missing_map.stderr

    cuvslam_map = tmp_path / "cuvslam_map"
    cuvslam_map.mkdir()
    (cuvslam_map / "not_a_map.txt").write_text("invalid", encoding="utf-8")
    missing_database = run_launcher(
        "localization",
        "--map",
        str(tmp_path),
        "--localization-init",
        "foxglove",
        check=False,
    )

    assert missing_database.returncode != 0
    assert "requires a cuVSLAM .mdb database" in missing_database.stderr


def test_replay_enables_foxglove_only_when_mode_is_explicit() -> None:
    explicit_output = run_launcher(
        "replay-localization",
        "--bag",
        "/workspaces/record/run_01",
        "--map",
        "/workspaces/map/course_a",
        "--localization-init",
        "foxglove",
        "--dry-run",
    ).stdout

    assert "enable_rosbag_replay:=true" in explicit_output
    assert "vslam_localize_on_startup:=false" in explicit_output
    assert "enable_vgl:=false" in explicit_output
    assert "enable_foxglove:=true" in explicit_output
    assert "VSLAM init   : foxglove (/initialpose required; VGL off)" in explicit_output


def test_replay_and_offline_presets_keep_foxglove_disabled() -> None:
    cases = (
        (
            "replay-localization",
            "--bag",
            "/workspaces/record/run_01",
            "--map",
            "/workspaces/map/course_a",
            "--dry-run",
        ),
        (
            "offline-localization",
            "--bag",
            "/workspaces/record/run_01",
            "--map",
            "/workspaces/map/course_a",
            "--dry-run",
        ),
        ("offline-vslam", "--bag", "/workspaces/record/run_01", "--dry-run"),
        (
            "offline-vslam-map",
            "--bag",
            "/workspaces/record/run_01",
            "--map",
            "/workspaces/map/course_a",
            "--dry-run",
        ),
    )

    for arguments in cases:
        output = run_launcher(*arguments).stdout

        assert "enable_localization:=true" in output
        assert "enable_foxglove:=false" in output
        assert "Foxglove     : disabled" in output


def test_live_localization_can_disable_foxglove_explicitly() -> None:
    output = run_launcher(
        "localization",
        "--map",
        "/workspaces/map/course_a",
        "--set",
        "enable_foxglove:=false",
        "--dry-run",
    ).stdout

    assert output.count("enable_foxglove:=false") == 1
    assert "enable_foxglove:=true" not in output
    assert "Foxglove     : disabled" in output


def test_foxglove_rejects_invalid_ports() -> None:
    for port in ("0", "65536", "not-a-port"):
        result = run_launcher(
            "custom",
            "--components",
            "foxglove",
            "--set",
            f"foxglove_port:={port}",
            "--dry-run",
            check=False,
        )

        assert result.returncode != 0
        assert "foxglove_port must be an integer between 1 and 65535" in result.stderr


def test_foxglove_launch_contract_excludes_high_bandwidth_topics() -> None:
    launch_path = (
        PROJECT_ROOT
        / "ros2_ws/src/launch/jetpilot_system_launch/launch/bringup.launch.py"
    )
    launch_source = launch_path.read_text(encoding="utf-8")
    launcher_source = LAUNCHER.read_text(encoding="utf-8")
    package_xml = launch_path.parents[1].joinpath("package.xml").read_text(
        encoding="utf-8"
    )
    launch_tree = ast.parse(launch_source)
    default_whitelist = None
    for node in launch_tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if any(
            isinstance(target, ast.Name)
            and target.id == "_DEFAULT_FOXGLOVE_TOPIC_WHITELIST"
            for target in node.targets
        ):
            default_whitelist = ast.literal_eval(node.value)
            break

    assert default_whitelist is not None
    patterns = ast.literal_eval(default_whitelist)
    assert "^/map$" not in patterns
    assert "^/(.*/)?diagnostics$" in patterns
    assert "^/visual_slam/tracking/odometry$" in patterns
    assert (
        "^/localization/(pose_hint_required|pose_hint_state|current_section|"
        "current_section_marker)$"
    ) in patterns
    assert (
        "^/hd_map/(lane_markers|section_markers|primary_centerline_path)$"
        in patterns
    )
    assert all("image" not in pattern for pattern in patterns)
    assert all(
        "points" not in pattern and "landmarks" not in pattern
        for pattern in patterns
    )
    compiled_patterns = [re.compile(pattern) for pattern in patterns]
    for excluded_topic in (
        "/realsense/color/image_raw",
        "/visual_slam/vis/landmarks_cloud",
        "/localization/debug/image",
        "/hd_map/debug/points",
    ):
        assert not any(
            pattern.fullmatch(excluded_topic) for pattern in compiled_patterns
        )
    assert (
        f'FOXGLOVE_DEFAULT_TOPIC_WHITELIST="{default_whitelist}"'
        in launcher_source
    )
    assert (
        "'client_topic_whitelist': args.foxglove_client_topic_whitelist"
        in launch_source
    )
    assert "Foxglove 3.4.x parses but does not enforce this whitelist" in (
        launch_source
    )
    assert "'service_whitelist': \"['^$']\"" in launch_source
    assert "'param_whitelist': \"['^$']\"" in launch_source
    assert "'capabilities': '[clientPublish]'" in launch_source
    assert "'max_qos_depth': '25'" in launch_source
    assert "'sysinfo': 'false'" in launch_source
    assert "'foxglove_bridge'," in launch_source
    assert "<exec_depend>foxglove_bridge</exec_depend>" in package_xml


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


def test_custom_e2e_enables_inference_and_operation_without_sensor_or_vehicle() -> None:
    output = run_launcher(
        "custom",
        "--components",
        "e2e",
        "--dry-run",
    ).stdout

    assert "enable_e2e_inference:=true" in output
    assert "enable_operation:=true" in output
    assert "enable_sensor_kit:=false" in output
    assert "enable_vehicle:=false" in output
    assert "enable_planning:=false" in output
    assert "enable_control:=false" in output


def test_rule_based_control_and_e2e_are_mutually_exclusive() -> None:
    result = run_launcher(
        "custom",
        "--components",
        "control,e2e",
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "cannot be enabled together" in result.stderr
    assert "/auto/control_cmd" in result.stderr


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


def test_custom_line_option_enables_typed_loader_and_selector(tmp_path: Path) -> None:
    custom_line = tmp_path / "custom_lines" / "safe-main" / "trajectory.csv"
    custom_line.parent.mkdir(parents=True)
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;0.5;-0.2\n")
    trajectory_hash = hashlib.sha256(custom_line.read_bytes()).hexdigest()
    (custom_line.parent / "custom_line.json").write_text(
        json.dumps(
            {
                "format": "jetpilot_custom_line_v1",
                "id": "safe-main",
                "name": "Safe Main",
                "closed_loop": True,
                "revision": 2,
                "source_hash": "1" * 64,
                "trajectory_csv": "trajectory.csv",
                "trajectory_sha256": trajectory_hash,
            }
        )
    )

    output = run_launcher(
        "custom",
        "--components",
        "control",
        "--custom-line",
        str(custom_line),
        "--custom-line-open",
        "--dry-run",
    ).stdout

    assert "enable_custom_trajectory_publisher:=true" in output
    assert "route_lane_selector.custom.param.yaml" in output
    assert f"custom_root:={custom_line.parent}" in output
    assert "custom_csv:=trajectory.csv" in output
    assert "custom_line_id:=safe-main" in output
    assert "custom_line_name:=Safe\\ Main" in output
    assert f"custom_source_hash:={trajectory_hash}" in output
    assert "custom_closed:=false" in output


def test_canonical_custom_line_uses_validated_metadata(tmp_path: Path) -> None:
    custom_line = tmp_path / "course_a_custom_line.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;0.5;-0.2\n")
    trajectory_hash = hashlib.sha256(custom_line.read_bytes()).hexdigest()
    custom_line.with_suffix(".meta.json").write_text(
        json.dumps(
            {
                "format": "jetpilot_custom_line_v1",
                "id": "manual-attack",
                "name": "Manual Attack",
                "closed_loop": False,
                "revision": 4,
                "source_hash": "2" * 64,
                # Canonical bundles are relocatable from the Console host to Jetson.
                "trajectory_csv": "/Users/operator/maps/course_a/course_a_custom_line.csv",
                "trajectory_sha256": trajectory_hash,
            }
        )
    )

    output = run_launcher(
        "custom", "--components", "control", "--custom-line", str(custom_line), "--dry-run"
    ).stdout

    assert "custom_line_id:=manual-attack" in output
    assert "custom_line_name:=Manual\\ Attack" in output
    assert "custom_closed:=false" in output

    overridden = run_launcher(
        "custom",
        "--components",
        "control",
        "--custom-line",
        str(custom_line),
        "--custom-line-id",
        "alias",
        "--custom-line-name",
        "Alias Line",
        "--custom-line-closed",
        "--dry-run",
    ).stdout
    assert "custom_line_id:=alias" in overridden
    assert "custom_line_name:=Alias\\ Line" in overridden
    assert "custom_closed:=true" in overridden


def test_custom_line_rejects_metadata_hash_mismatch(tmp_path: Path) -> None:
    custom_line = tmp_path / "course_a_custom_line.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    custom_line.with_suffix(".meta.json").write_text(
        json.dumps(
            {
                "format": "jetpilot_custom_line_v1",
                "id": "manual",
                "name": "Manual",
                "closed_loop": True,
                "revision": 1,
                "source_hash": "3" * 64,
                "trajectory_csv": str(custom_line),
                "trajectory_sha256": "0" * 64,
            }
        )
    )

    result = run_launcher(
        "custom",
        "--components",
        "control",
        "--custom-line",
        str(custom_line),
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "metadata rejected" in result.stderr


def test_custom_line_bundle_requires_manifest(tmp_path: Path) -> None:
    custom_line = tmp_path / "custom_lines" / "manual" / "trajectory.csv"
    custom_line.parent.mkdir(parents=True)
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")

    result = run_launcher(
        "custom",
        "--components",
        "control",
        "--custom-line",
        str(custom_line),
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "metadata is required" in result.stderr


def test_custom_line_and_raceline_are_mutually_exclusive(tmp_path: Path) -> None:
    trajectory = tmp_path / "trajectory.csv"
    trajectory.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    result = run_launcher(
        "custom",
        "--components",
        "control",
        "--raceline",
        str(trajectory),
        "--custom-line",
        str(trajectory),
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "mutually exclusive" in result.stderr


def test_custom_line_component_requires_csv() -> None:
    result = run_launcher(
        "custom", "--components", "custom-line", "--dry-run", check=False
    )

    assert result.returncode != 0
    assert "requires --custom-line" in result.stderr


def test_custom_line_component_uses_active_line_from_selected_map(tmp_path: Path) -> None:
    map_dir = tmp_path / "course_a"
    map_dir.mkdir()
    custom_line = map_dir / "course_a_custom_line.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    trajectory_hash = hashlib.sha256(custom_line.read_bytes()).hexdigest()
    custom_line.with_suffix(".meta.json").write_text(
        json.dumps(
            {
                "format": "jetpilot_custom_line_v1",
                "id": "safe-main",
                "name": "Safe Main",
                "closed_loop": False,
                "revision": 2,
                "source_hash": "a" * 64,
                "trajectory_csv": str(custom_line),
                "trajectory_sha256": trajectory_hash,
            }
        )
    )

    output = run_launcher(
        "custom",
        "--components",
        "custom-line",
        "--map",
        str(map_dir),
        "--dry-run",
    ).stdout

    assert f"custom_root:={map_dir}" in output
    assert "custom_csv:=course_a_custom_line.csv" in output
    assert "custom_line_id:=safe-main" in output
    assert "custom_line_name:=Safe\\ Main" in output


def test_section_authored_canonical_custom_line_requires_matching_hd_map(
    tmp_path: Path,
) -> None:
    map_dir = tmp_path / "course_a"
    map_dir.mkdir()
    hd_map = map_dir / "course_a_hd_map.yaml"
    hd_map.write_text("format: tamiya_local_hd_map_v1\nsections: []\n", encoding="utf-8")
    hd_map_hash = hashlib.sha256(hd_map.read_bytes()).hexdigest()
    custom_line = map_dir / "course_a_custom_line.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    trajectory_hash = hashlib.sha256(custom_line.read_bytes()).hexdigest()

    for mode_field in ("speed_profile_mode", "speed_authoring"):
        custom_line.with_suffix(".meta.json").write_text(
            json.dumps(
                {
                    "format": "jetpilot_custom_line_v1",
                    "id": "safe-main",
                    "name": "Safe Main",
                    "closed_loop": False,
                    "revision": 2,
                    "source_hash": "a" * 64,
                    "trajectory_csv": custom_line.name,
                    "trajectory_sha256": trajectory_hash,
                    mode_field: "sections",
                    "hd_map_sha256": hd_map_hash,
                }
            ),
            encoding="utf-8",
        )

        output = run_launcher(
            "custom",
            "--components",
            "custom-line",
            "--map",
            str(map_dir),
            "--custom-line",
            str(custom_line),
            "--dry-run",
        ).stdout

        assert "enable_custom_trajectory_publisher:=true" in output
        assert "custom_line_id:=safe-main" in output


def test_section_authored_named_custom_line_accepts_matching_hd_map(
    tmp_path: Path,
) -> None:
    map_dir = tmp_path / "course_a"
    line_dir = map_dir / "custom_lines" / "safe-main"
    line_dir.mkdir(parents=True)
    hd_map = map_dir / "course_a_hd_map.yaml"
    hd_map.write_text("format: tamiya_local_hd_map_v1\nsections: []\n", encoding="utf-8")
    custom_line = line_dir / "trajectory.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    (line_dir / "custom_line.json").write_text(
        json.dumps(
            {
                "format": "jetpilot_custom_line_v1",
                "id": "safe-main",
                "name": "Safe Main",
                "closed_loop": False,
                "revision": 2,
                "source_hash": "b" * 64,
                "trajectory_csv": custom_line.name,
                "trajectory_sha256": hashlib.sha256(custom_line.read_bytes()).hexdigest(),
                "speed_authoring": "sections",
                "hd_map_sha256": hashlib.sha256(hd_map.read_bytes()).hexdigest(),
            }
        ),
        encoding="utf-8",
    )

    output = run_launcher(
        "custom",
        "--components",
        "control",
        "--map",
        str(map_dir),
        "--custom-line",
        str(custom_line),
        "--dry-run",
    ).stdout

    assert f"custom_root:={line_dir}" in output
    assert "custom_line_id:=safe-main" in output


def test_section_authored_custom_line_rejects_stale_hd_map(tmp_path: Path) -> None:
    map_dir = tmp_path / "course_a"
    map_dir.mkdir()
    hd_map = map_dir / "course_a_hd_map.yaml"
    hd_map.write_text("format: tamiya_local_hd_map_v1\nsections: []\n", encoding="utf-8")
    custom_line = map_dir / "course_a_custom_line.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    custom_line.with_suffix(".meta.json").write_text(
        json.dumps(
            {
                "format": "jetpilot_custom_line_v1",
                "id": "safe-main",
                "name": "Safe Main",
                "closed_loop": False,
                "revision": 2,
                "source_hash": "c" * 64,
                "trajectory_csv": custom_line.name,
                "trajectory_sha256": hashlib.sha256(custom_line.read_bytes()).hexdigest(),
                "speed_profile_mode": "sections",
                "hd_map_sha256": "0" * 64,
            }
        ),
        encoding="utf-8",
    )

    result = run_launcher(
        "custom",
        "--components",
        "custom-line",
        "--map",
        str(map_dir),
        "--custom-line",
        str(custom_line),
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "hd_map_sha256 does not match the selected map" in result.stderr


def test_section_authored_custom_line_requires_hd_map_hash(tmp_path: Path) -> None:
    map_dir = tmp_path / "course_a"
    map_dir.mkdir()
    hd_map = map_dir / "course_a_hd_map.yaml"
    hd_map.write_text("format: tamiya_local_hd_map_v1\nsections: []\n", encoding="utf-8")
    custom_line = map_dir / "course_a_custom_line.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    custom_line.with_suffix(".meta.json").write_text(
        json.dumps(
            {
                "format": "jetpilot_custom_line_v1",
                "id": "safe-main",
                "name": "Safe Main",
                "closed_loop": False,
                "revision": 2,
                "source_hash": "d" * 64,
                "trajectory_csv": custom_line.name,
                "trajectory_sha256": hashlib.sha256(custom_line.read_bytes()).hexdigest(),
                "speed_authoring": "sections",
            }
        ),
        encoding="utf-8",
    )

    result = run_launcher(
        "custom",
        "--components",
        "custom-line",
        "--map",
        str(map_dir),
        "--custom-line",
        str(custom_line),
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "section-authored custom line requires hd_map_sha256" in result.stderr


def test_section_authored_custom_line_requires_selected_map(tmp_path: Path) -> None:
    custom_line = tmp_path / "course_a_custom_line.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    custom_line.with_suffix(".meta.json").write_text(
        json.dumps(
            {
                "format": "jetpilot_custom_line_v1",
                "id": "safe-main",
                "name": "Safe Main",
                "closed_loop": False,
                "revision": 2,
                "source_hash": "e" * 64,
                "trajectory_csv": custom_line.name,
                "trajectory_sha256": hashlib.sha256(custom_line.read_bytes()).hexdigest(),
                "speed_profile_mode": "sections",
                "hd_map_sha256": "f" * 64,
            }
        ),
        encoding="utf-8",
    )

    result = run_launcher(
        "custom",
        "--components",
        "control",
        "--custom-line",
        str(custom_line),
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "section-authored custom line requires --map PATH" in result.stderr


def test_hashed_legacy_custom_line_requires_matching_selected_map(
    tmp_path: Path,
) -> None:
    map_dir = tmp_path / "course_a"
    map_dir.mkdir()
    hd_map = map_dir / "course_a_hd_map.yaml"
    hd_map.write_text("format: tamiya_local_hd_map_v1\nsections: []\n", encoding="utf-8")
    custom_line = map_dir / "course_a_custom_line.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    metadata = {
        "format": "jetpilot_custom_line_v1",
        "id": "legacy-main",
        "name": "Legacy Main",
        "closed_loop": False,
        "revision": 2,
        "source_hash": "e" * 64,
        "trajectory_csv": custom_line.name,
        "trajectory_sha256": hashlib.sha256(custom_line.read_bytes()).hexdigest(),
        "speed_profile_mode": "legacy_points",
        "hd_map_sha256": hashlib.sha256(hd_map.read_bytes()).hexdigest(),
    }
    custom_line.with_suffix(".meta.json").write_text(
        json.dumps(metadata),
        encoding="utf-8",
    )

    output = run_launcher(
        "custom",
        "--components",
        "control",
        "--map",
        str(map_dir),
        "--custom-line",
        str(custom_line),
        "--dry-run",
    ).stdout
    assert "custom_line_id:=legacy-main" in output

    hd_map.write_text("format: tamiya_local_hd_map_v1\nsections: [changed]\n", encoding="utf-8")
    stale = run_launcher(
        "custom",
        "--components",
        "control",
        "--map",
        str(map_dir),
        "--custom-line",
        str(custom_line),
        "--dry-run",
        check=False,
    )
    assert stale.returncode != 0
    assert "hd_map_sha256 does not match the selected map" in stale.stderr


def test_hashed_legacy_custom_line_requires_selected_map(tmp_path: Path) -> None:
    custom_line = tmp_path / "course_a_custom_line.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    custom_line.with_suffix(".meta.json").write_text(
        json.dumps(
            {
                "format": "jetpilot_custom_line_v1",
                "id": "legacy-main",
                "name": "Legacy Main",
                "closed_loop": False,
                "revision": 2,
                "source_hash": "e" * 64,
                "trajectory_csv": custom_line.name,
                "trajectory_sha256": hashlib.sha256(custom_line.read_bytes()).hexdigest(),
                "speed_authoring": "legacy_points",
                "hd_map_sha256": "f" * 64,
            }
        ),
        encoding="utf-8",
    )

    result = run_launcher(
        "custom",
        "--components",
        "control",
        "--custom-line",
        str(custom_line),
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "custom line metadata with hd_map_sha256 requires --map PATH" in result.stderr


def test_custom_line_component_requires_selected_map_for_legacy_bundle(
    tmp_path: Path,
) -> None:
    custom_line = tmp_path / "course_a_custom_line.csv"
    custom_line.write_text("0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n")
    custom_line.with_suffix(".meta.json").write_text(
        json.dumps(
            {
                "format": "jetpilot_custom_line_v1",
                "id": "safe-main",
                "name": "Safe Main",
                "closed_loop": False,
                "revision": 2,
                "source_hash": "f" * 64,
                "trajectory_csv": custom_line.name,
                "trajectory_sha256": hashlib.sha256(custom_line.read_bytes()).hexdigest(),
            }
        ),
        encoding="utf-8",
    )

    result = run_launcher(
        "custom",
        "--components",
        "custom-line",
        "--custom-line",
        str(custom_line),
        "--dry-run",
        check=False,
    )

    assert result.returncode != 0
    assert "custom-line component requires --map PATH" in result.stderr


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
    assert "vslam_enable_slam:=true" in output
    assert "vslam_localize_on_startup:=false" in output
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
        "e2e",
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
        if preset in {"vehicle", "teleop", "drive", "e2e", "runtime"}:
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
