from __future__ import annotations

import importlib.util
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LOCALIZATION_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "localization.launch.py"
VGL_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "localization" / "vgl.launch.py"
VSLAM_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "localization" / "vslam.launch.py"
TOOL_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "tool.launch.py"
VEHICLE_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "vehicle.launch.py"
BRINGUP_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "bringup.launch.py"
JOY_BUTTON_CONFIG_PATH = (
    PACKAGE_ROOT / "config" / "tool" / "joy_button_mapping.param.yaml"
)

SPEC = importlib.util.spec_from_file_location(
    "jetpilot_localization_launch", LOCALIZATION_LAUNCH_PATH
)
assert SPEC is not None and SPEC.loader is not None
LOCALIZATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LOCALIZATION)


def test_saved_map_availability_requires_both_directories(tmp_path: Path) -> None:
    assert LOCALIZATION.saved_localization_map_availability("") == (False, False)
    assert LOCALIZATION.saved_localization_map_availability(str(tmp_path)) == (
        False,
        False,
    )

    (tmp_path / "cuvgl_map").mkdir()
    assert LOCALIZATION.saved_localization_map_availability(str(tmp_path)) == (
        True,
        False,
    )

    (tmp_path / "cuvslam_map").mkdir()
    assert LOCALIZATION.saved_localization_map_availability(str(tmp_path)) == (
        True,
        True,
    )


def test_localization_manager_launch_contract_is_wired_statically() -> None:
    localization_source = LOCALIZATION_LAUNCH_PATH.read_text(encoding="utf-8")
    bringup_source = BRINGUP_LAUNCH_PATH.read_text(encoding="utf-8")

    assert "package='jetpilot_localization_manager'" in localization_source
    assert "executable='jetpilot_localization_manager_node'" in localization_source
    assert "mapping_mode = bool(args.vslam_save_map_folder_path)" in localization_source
    assert "if cuvslam_map_available and not mapping_mode" in localization_source
    assert "if enable_localization_manager and not mapping_mode" in localization_source
    assert "'autostart': cuvslam_map_available and not mapping_mode" in localization_source
    for parameter in (
        "'use_vgl': vgl_started",
        "'vslam_hint_request_topic': args.vslam_hint_request_topic",
        "'vslam_pose_hint_topic': args.vslam_pose_hint_topic",
        "'manual_pose_topic': args.manual_pose_topic",
        "'vgl_pose_topic': args.vgl_pose_topic",
        "'localization_trigger_topic': args.localization_trigger_topic",
        "'diagnostics_topic': args.vslam_diagnostics_topic",
        "'use_sim_time': use_sim_time",
    ):
        assert parameter in localization_source

    for source in (localization_source, bringup_source):
        assert "args.add_arg('enable_localization_manager', True, cli=True)" in source
        assert "'jetpilot_localization_manager'" in source
        assert "'config/localization_manager.param.yaml'" in source

    assert "'enable_localization_manager': args.enable_localization_manager" in bringup_source
    assert "'localization_manager_param': args.localization_manager_param" in bringup_source


def test_configurable_localization_endpoints_are_wired_to_producers() -> None:
    bringup_source = BRINGUP_LAUNCH_PATH.read_text(encoding="utf-8")
    localization_source = LOCALIZATION_LAUNCH_PATH.read_text(encoding="utf-8")
    vgl_source = VGL_LAUNCH_PATH.read_text(encoding="utf-8")
    vslam_source = VSLAM_LAUNCH_PATH.read_text(encoding="utf-8")
    tool_source = TOOL_LAUNCH_PATH.read_text(encoding="utf-8")

    assert "'localization_trigger_topic': args.localization_trigger_topic" in bringup_source
    assert "'localization_trigger_topic': args.localization_trigger_topic" in tool_source
    assert "'vgl_trigger_service': args.vgl_trigger_service" in localization_source
    assert "'vgl_pose_topic': args.vgl_pose_topic" in localization_source
    assert "'vgl_image_qos_profile': args.vgl_image_qos_profile" in localization_source
    assert "'image_qos_profile': args.vgl_image_qos_profile" in vgl_source
    assert "('visual_localization/trigger_localization', args.vgl_trigger_service)" in vgl_source
    assert "('visual_localization/pose', args.vgl_pose_topic)" in vgl_source
    assert "'vslam_diagnostics_topic': args.vslam_diagnostics_topic" in localization_source
    assert "('/diagnostics', args.vslam_diagnostics_topic)" in vslam_source
    assert "'vgl_diagnostics_topic': args.vgl_diagnostics_topic" in localization_source
    assert "('/diagnostics', args.vgl_diagnostics_topic)" in vgl_source
    assert "'vslam_enable_imu': lu.is_true(args.vslam_enable_imu)" in localization_source
    assert "'vslam_imu_topic': args.vslam_imu_topic" in localization_source
    assert "('visual_slam/imu', args.vslam_imu_topic)" in vslam_source
    assert "'tracking_mode': 1 if lu.is_true(args.vslam_enable_imu) else 0" in vslam_source


def test_jetson_stats_is_enabled_by_default() -> None:
    bringup_source = BRINGUP_LAUNCH_PATH.read_text(encoding="utf-8")
    tool_source = TOOL_LAUNCH_PATH.read_text(encoding="utf-8")

    for source in (bringup_source, tool_source):
        assert "args.add_arg('enable_jetson_stats', True, cli=True)" in source


def test_fallback_joy_profile_exposes_localization_trigger() -> None:
    joy_config = JOY_BUTTON_CONFIG_PATH.read_text(encoding="utf-8")

    assert "localization_trigger_button: 7" in joy_config


def test_vehicle_description_can_start_without_hardware_interface() -> None:
    bringup_source = BRINGUP_LAUNCH_PATH.read_text(encoding="utf-8")
    vehicle_source = VEHICLE_LAUNCH_PATH.read_text(encoding="utf-8")

    assert "vehicle_interface_enabled = lut.AndSubstitution" in bringup_source
    assert "vehicle_launch_enabled = OrSubstitution" in bringup_source
    assert "'enable_vehicle_interface': vehicle_interface_enabled" in bringup_source
    assert "condition=IfCondition(vehicle_launch_enabled)" in bringup_source
    assert "args.add_arg('enable_vehicle_interface', True, cli=True)" in vehicle_source
    assert "if lu.is_true(args.enable_vehicle_interface):" in vehicle_source


def test_vehicle_description_exposes_optional_evs_and_thremo_frames() -> None:
    bringup_source = BRINGUP_LAUNCH_PATH.read_text(encoding="utf-8")
    vehicle_source = VEHICLE_LAUNCH_PATH.read_text(encoding="utf-8")

    for frame in ("evs", "thremo"):
        assert f"vehicle_{frame}_static_transform_publisher" in vehicle_source
        assert f"args.vehicle_description_{frame}_frame" in vehicle_source
        assert f"args.add_arg('publish_vehicle_{frame}_description'" in vehicle_source
        assert f"'publish_vehicle_{frame}_description':" in bringup_source
        assert f"args.publish_vehicle_{frame}_description" in bringup_source


def test_vehicle_opaque_function_uses_python_conditionals() -> None:
    vehicle_source = VEHICLE_LAUNCH_PATH.read_text(encoding="utf-8")

    assert "AndSubstitution" not in vehicle_source
    assert "IfCondition" not in vehicle_source
    assert "if lu.is_true(args.enable_vehicle_interface):" in vehicle_source
