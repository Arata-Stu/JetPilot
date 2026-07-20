from __future__ import annotations

import csv
import json
import math
import os
import re
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from .config import ConsoleConfig
from .map_detail import load_yaml
from .map_pipeline import (
    DEFAULT_RACELINE_SAFETY_MARGIN_M,
    DEFAULT_RACELINE_VEHICLE_WIDTH_M,
    default_topic_config,
    localization_config_dir,
)
from .security import resolve_under_root


PASS = "pass"
WARNING = "warning"
BLOCKED = "blocked"
SUPPORTED_ACTIONS = frozenset(
    {
        "map-build",
        "prepare-hd-raster",
        "generate-raceline",
        "generate-preview",
        "analyze-rosbag",
    }
)

_MAP_BUILD_TOKEN = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.-]{0,63}$")
_PORTABLE_MAP_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")
_BASE64_PAYLOAD = re.compile(r"^[A-Za-z0-9+/]*={0,2}$")
_ROS_TOPIC = re.compile(r"^/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$")

DEFAULT_ANALYSIS_CONTROL_TOPIC = "/vehicle/control_cmd"
DEFAULT_ANALYSIS_MODE_TOPIC = "/operation_mode/state"
DEFAULT_ANALYSIS_POSE_TOPIC = "/visual_slam/tracking/odometry"


@dataclass(frozen=True)
class PreflightCheck:
    id: str
    label: str
    status: str
    message: str
    remediation: str | None = None
    details: dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "status": self.status,
            "message": self.message,
            "remediation": self.remediation,
            "details": self.details,
        }


class _Report:
    def __init__(self, action: str) -> None:
        self.action = action
        self.checks: list[PreflightCheck] = []
        self.resolved: dict[str, Any] = {}

    def add(
        self,
        check_id: str,
        label: str,
        status: str,
        message: str,
        *,
        remediation: str | None = None,
        details: Mapping[str, Any] | None = None,
    ) -> None:
        if status not in {PASS, WARNING, BLOCKED}:
            raise ValueError(f"unsupported preflight status: {status}")
        self.checks.append(
            PreflightCheck(
                id=check_id,
                label=label,
                status=status,
                message=message,
                remediation=remediation,
                details=dict(details or {}),
            )
        )

    def finish(self) -> dict[str, Any]:
        counts = {
            status: sum(check.status == status for check in self.checks)
            for status in (PASS, WARNING, BLOCKED)
        }
        ready = counts[BLOCKED] == 0
        if counts[BLOCKED]:
            status = BLOCKED
            message = f"{counts[BLOCKED]} required check(s) failed."
        elif counts[WARNING]:
            status = WARNING
            message = f"Ready with {counts[WARNING]} warning(s)."
        else:
            status = PASS
            message = "All required checks passed."
        return {
            "action": self.action,
            "ready": ready,
            "status": status,
            "summary": {
                **counts,
                "total": len(self.checks),
                "message": message,
            },
            "checks": [check.to_json() for check in self.checks],
            "resolved": self.resolved,
        }


def evaluate_preflight(
    config: ConsoleConfig,
    action: str,
    payload: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Evaluate whether a Console task can start.

    The return value contains JSON-native values only. A warning never disables
    an action; any blocked check makes ``ready`` false. Callers must still run
    their normal request validation immediately before starting a task because
    files can change after this inspection.
    """

    normalized_action = str(action).strip()
    if normalized_action not in SUPPORTED_ACTIONS:
        supported = ", ".join(sorted(SUPPORTED_ACTIONS))
        raise ValueError(f"unsupported preflight action {action!r}; expected one of: {supported}")
    values = dict(payload or {})
    report = _Report(normalized_action)

    if normalized_action == "map-build":
        _map_build_preflight(config, values, report)
    elif normalized_action == "analyze-rosbag":
        _analyze_rosbag_preflight(config, values, report)
    else:
        map_dir = _required_map_dir(config, values.get("map_dir"), report)
        if map_dir is not None:
            if normalized_action == "prepare-hd-raster":
                _prepare_hd_raster_preflight(config, map_dir, report)
            elif normalized_action == "generate-raceline":
                _generate_raceline_preflight(map_dir, values, report)
            elif normalized_action == "generate-preview":
                _generate_preview_preflight(config, map_dir, report)

    return report.finish()


def parse_rosbag_metadata(text: str) -> dict[str, Any]:
    """Parse the rosbag2 metadata subset used by Console APIs and preflight."""

    return _parse_rosbag_metadata(text)


def _map_build_preflight(
    config: ConsoleConfig,
    payload: Mapping[str, Any],
    report: _Report,
) -> None:
    bag_info = _inspect_rosbag(config, payload.get("rosbag"), report)
    camera_topics = _inspect_camera_config(config, payload.get("topic_config"), report)
    if bag_info is not None and camera_topics is not None:
        _check_mapping_topics(bag_info, camera_topics, report)
    _inspect_map_output(config, payload.get("map_dir"), report)
    _inspect_mapping_parameters(payload, report)
    _inspect_vgl_model(config, payload.get("output_model_dir"), report)


def _analysis_topic(
    topics: Mapping[str, Any],
    report: _Report,
    *,
    check_id: str,
    label: str,
    raw_value: Any,
    default: str = "",
    required: bool,
    expected_types: set[str] | None = None,
) -> str:
    explicit = raw_value is not None and bool(str(raw_value).strip())
    topic = str(raw_value).strip() if explicit else default
    if not topic:
        report.add(
            check_id,
            label,
            BLOCKED if required else WARNING,
            f"No {label.lower()} was selected.",
            remediation=(
                f"Select a {label.lower()} contained in the rosbag."
                if required
                else f"Select a {label.lower()} to include this signal in the analysis."
            ),
        )
        return ""
    if not _ROS_TOPIC.fullmatch(topic):
        report.add(
            check_id,
            label,
            BLOCKED if required or explicit else WARNING,
            f"{topic!r} is not a valid absolute ROS topic name.",
            remediation="Choose an absolute topic name from the rosbag topic list.",
            details={"topic": topic},
        )
        return ""

    raw_topic = topics.get(topic)
    if not isinstance(raw_topic, Mapping):
        report.add(
            check_id,
            label,
            BLOCKED if required else WARNING,
            f"The rosbag does not contain {topic}.",
            remediation=(
                f"Select another {label.lower()} or record {topic}."
                if required
                else f"Record {topic} or leave this optional signal unavailable."
            ),
            details={"topic": topic, "available_topics": sorted(topics)},
        )
        return ""

    message_count = raw_topic.get("message_count")
    if not isinstance(message_count, int) or isinstance(message_count, bool) or message_count <= 0:
        report.add(
            check_id,
            label,
            BLOCKED if required else WARNING,
            f"{topic} has no usable messages according to metadata.yaml.",
            remediation="Repair/reindex the rosbag or select a populated topic.",
            details={"topic": topic, "message_count": message_count},
        )
        return ""

    actual_type = str(raw_topic.get("type") or "")
    if expected_types and actual_type not in expected_types:
        report.add(
            check_id,
            label,
            BLOCKED if required else WARNING,
            f"{topic} has unsupported type {actual_type or '(unknown)' }.",
            remediation="Select a topic whose message type is supported by the analysis worker.",
            details={
                "topic": topic,
                "actual_type": actual_type,
                "expected_types": sorted(expected_types),
            },
        )
        return ""

    report.add(
        check_id,
        label,
        PASS,
        f"{topic} contains {message_count} message(s).",
        details={"topic": topic, "type": actual_type, "message_count": message_count},
    )
    return topic


def _analysis_map(
    config: ConsoleConfig,
    raw_value: Any,
    report: _Report,
    *,
    required: bool,
) -> Path | None:
    if raw_value is None or not str(raw_value).strip():
        report.add(
            "analysis.map",
            "Analysis map",
            BLOCKED if required else WARNING,
            "No map was selected for trajectory localization/overlay.",
            remediation=(
                "Select a map containing cuVGL and cuVSLAM artifacts."
                if required
                else "Select the map used for this run to enable the map overlay."
            ),
            details={"map_root": str(config.map_root)},
        )
        return None
    try:
        map_dir = resolve_under_root(
            str(raw_value),
            config.map_root,
            label="analysis map",
            require_exists=True,
            require_directory=True,
        )
    except ValueError as exc:
        report.add(
            "analysis.map",
            "Analysis map",
            BLOCKED,
            str(exc),
            remediation="Select an existing map folder inside MAP_ROOT.",
            details={"map_root": str(config.map_root)},
        )
        return None

    report.resolved["map_dir"] = str(map_dir)
    report.add(
        "analysis.map",
        "Analysis map",
        PASS,
        "The selected map is inside the configured map root.",
        details={"path": str(map_dir)},
    )
    overlay_files = [
        map_dir / f"{map_dir.name}_hd_map.yaml",
        map_dir / "vslam_landmarks.yaml",
    ]
    if not any(path.is_file() and path.stat().st_size > 0 for path in overlay_files):
        report.add(
            "analysis.map_overlay",
            "Map overlay",
            WARNING,
            "The selected map has no HD map or landmark raster metadata for the browser overlay.",
            remediation="Prepare the HD raster/HD map to display the trajectory on a map image.",
            details={"expected_any": [str(path) for path in overlay_files]},
        )
    else:
        report.add(
            "analysis.map_overlay",
            "Map overlay",
            PASS,
            "Map overlay metadata is available.",
        )
    return map_dir


def _offline_map_artifacts(
    map_dir: Path,
    report: _Report,
    *,
    localization_mode: str,
) -> tuple[bool, bool]:
    """Check saved localization maps without making VGL a VSLAM-only dependency."""

    availability: dict[str, bool] = {}
    for name in ("cuvgl_map", "cuvslam_map"):
        path = map_dir / name
        try:
            availability[name] = path.is_dir() and any(path.iterdir())
        except OSError:
            availability[name] = False

    vgl_available = availability["cuvgl_map"]
    vslam_available = availability["cuvslam_map"]
    missing = [
        str(map_dir / name)
        for name, available in availability.items()
        if not available
    ]
    if not vslam_available or (localization_mode == "vgl" and not vgl_available):
        required_names = ["cuvslam_map"]
        if localization_mode == "vgl":
            required_names.insert(0, "cuvgl_map")
        report.add(
            "analysis.localization_map",
            "Offline localization map",
            BLOCKED,
            "The selected offline localization mode is missing a required saved map.",
            remediation="Select the matching completed map or finish its localization map build first.",
            details={
                "mode": localization_mode,
                "required": required_names,
                "missing_or_empty": missing,
            },
        )
        return vgl_available, vslam_available

    if localization_mode == "auto" and not vgl_available:
        report.add(
            "analysis.localization_map",
            "Offline localization map",
            WARNING,
            "cuVGL map data is unavailable; Auto will use the saved cuVSLAM map with an identity pose hint.",
            remediation=(
                "Add the matching cuvgl_map to try global localization, or confirm that the bag starts "
                "near the saved cuVSLAM map origin."
            ),
            details={
                "mode": localization_mode,
                "fallback": "vslam",
                "missing_or_empty": missing,
            },
        )
        return vgl_available, vslam_available

    report.add(
        "analysis.localization_map",
        "Offline localization map",
        PASS,
        (
            "The saved cuVSLAM map is available for VSLAM-only localization."
            if localization_mode == "vslam"
            else "Populated cuVGL and cuVSLAM map folders are available."
        ),
        details={"mode": localization_mode},
    )
    return vgl_available, vslam_available


def _analysis_max_fps(payload: Mapping[str, Any], report: _Report) -> None:
    raw_value = payload.get("max_fps", 15.0)
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        value = math.nan
    if not math.isfinite(value) or value <= 0.0 or value > 240.0:
        report.add(
            "analysis.max_fps",
            "Frame extraction rate",
            BLOCKED,
            "max_fps must be a finite value greater than 0 and at most 240.",
            remediation="Choose a practical extraction rate such as 10, 15, or 30 fps.",
            details={"value": raw_value},
        )
        return
    report.resolved["max_fps"] = value
    report.add(
        "analysis.max_fps",
        "Frame extraction rate",
        PASS,
        f"Images will be extracted at up to {value:g} fps.",
        details={"max_fps": value},
    )


def _analyze_rosbag_preflight(
    config: ConsoleConfig,
    payload: Mapping[str, Any],
    report: _Report,
) -> None:
    _analysis_runtime(config, report)
    bag_info = _inspect_rosbag(config, payload.get("rosbag"), report)
    _analysis_max_fps(payload, report)
    if bag_info is None:
        return
    topics = bag_info.get("topics")
    if not isinstance(topics, Mapping):
        topics = {}

    image_topic = _analysis_topic(
        topics,
        report,
        check_id="analysis.image_topic",
        label="Image topic",
        raw_value=payload.get("image_topic"),
        required=True,
        expected_types={"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"},
    )
    if image_topic:
        report.resolved["image_topic"] = image_topic

    control_topic = _analysis_topic(
        topics,
        report,
        check_id="analysis.control_topic",
        label="Control command topic",
        raw_value=payload.get("control_topic"),
        default=DEFAULT_ANALYSIS_CONTROL_TOPIC,
        required=False,
        expected_types={"jetpilot_msgs/msg/ControlCommand"},
    )
    mode_topic = _analysis_topic(
        topics,
        report,
        check_id="analysis.mode_topic",
        label="Operation mode topic",
        raw_value=payload.get("mode_topic"),
        default=DEFAULT_ANALYSIS_MODE_TOPIC,
        required=False,
        expected_types={"jetpilot_msgs/msg/OperationModeState"},
    )
    if control_topic:
        report.resolved["control_topic"] = control_topic
    if mode_topic:
        report.resolved["mode_topic"] = mode_topic

    speed_topic = ""
    if payload.get("speed_topic") is not None and str(payload.get("speed_topic")).strip():
        speed_topic = _analysis_topic(
            topics,
            report,
            check_id="analysis.speed_topic",
            label="Speed topic",
            raw_value=payload.get("speed_topic"),
            required=False,
        )
        if speed_topic:
            report.resolved["speed_topic"] = speed_topic
            speed_type = str((topics.get(speed_topic) or {}).get("type") or "")
            supported_speed_type = bool(
                re.search(
                    r"/(?:Float32|Float64|Int8|Int16|Int32|Int64|UInt8|UInt16|UInt32|UInt64|Odometry|Twist|TwistStamped|TwistWithCovarianceStamped)$",
                    speed_type,
                )
            )
            if not supported_speed_type:
                report.add(
                    "analysis.speed_type",
                    "Speed topic decoding",
                    WARNING,
                    f"The selected type {speed_type or '<unknown>'} has no guaranteed speed-field contract.",
                    remediation=(
                        "Use Odometry, Twist, a numeric std_msgs topic, or leave this empty to derive "
                        "vehicle speed from the trajectory. Unknown samples are skipped, not converted to zero."
                    ),
                    details={"topic": speed_topic, "type": speed_type},
                )
            if speed_topic.startswith("/commands/"):
                report.resolved["speed_kind"] = "commanded"
                report.resolved["speed_label"] = "Commanded speed"
                report.add(
                    "analysis.speed_semantics",
                    "Speed interpretation",
                    WARNING,
                    "The selected /commands topic is an actuator command, not a measured vehicle speed.",
                    remediation=(
                        "Display it as Commanded speed. Use recorded VSLAM odometry or an offline "
                        "trajectory when actual vehicle speed is needed."
                    ),
                    details={"topic": speed_topic, "kind": "commanded"},
                )
            else:
                report.resolved["speed_kind"] = "topic"
                report.resolved["speed_label"] = "Speed topic"

    raw_mode = str(payload.get("trajectory_mode") or "auto").strip().lower()
    if raw_mode not in {"auto", "recorded", "offline", "none"}:
        report.add(
            "analysis.trajectory_mode",
            "Trajectory source",
            BLOCKED,
            "trajectory_mode must be auto, recorded, offline, or none.",
            remediation="Choose one of the trajectory source options shown in the UI.",
            details={"value": raw_mode},
        )
        return

    pose_topic_name = str(payload.get("pose_topic") or DEFAULT_ANALYSIS_POSE_TOPIC).strip()
    pose_topic = ""
    if raw_mode in {"auto", "recorded"}:
        pose_topic = _analysis_topic(
            topics,
            report,
            check_id="analysis.pose_topic",
            label="Recorded pose topic",
            raw_value=pose_topic_name,
            required=raw_mode == "recorded",
            expected_types={"nav_msgs/msg/Odometry"},
        )

    map_required = raw_mode == "offline" and str(payload.get("offline_localization_mode") or "auto").strip().lower() != "vslam_from_scratch"
    map_dir = _analysis_map(config, payload.get("map_dir"), report, required=map_required)
    if raw_mode == "recorded" and not pose_topic:
        resolved_mode = "recorded"
    elif raw_mode == "offline":
        resolved_mode = "offline"
    elif raw_mode == "none":
        resolved_mode = "none"
    elif pose_topic:
        resolved_mode = "recorded"
    elif map_dir is not None or str(payload.get("offline_localization_mode") or "auto").strip().lower() == "vslam_from_scratch":
        resolved_mode = "offline"
    else:
        resolved_mode = "none"

    report.resolved["trajectory_mode"] = resolved_mode
    if pose_topic and resolved_mode == "recorded":
        report.resolved["pose_topic"] = pose_topic
        report.add(
            "analysis.trajectory_source",
            "Trajectory source",
            PASS,
            "The recorded odometry topic will provide the trajectory.",
            details={"mode": "recorded", "topic": pose_topic},
        )
    elif resolved_mode == "none":
        report.add(
            "analysis.trajectory_source",
            "Trajectory source",
            WARNING,
            "No recorded pose is available and offline localization will not run.",
            remediation="Select the matching map and Auto/Offline to generate a trajectory.",
            details={"mode": "none"},
        )

    if resolved_mode == "offline":
        if map_dir is None:
            # _analysis_map already supplied the actionable blocked check for an
            # explicit offline request. Auto without a map degrades to `none`.
            if raw_mode == "offline":
                return
        else:
            offline_mode = str(
                payload.get("offline_localization_mode") or "auto"
            ).strip().lower()
            if offline_mode not in {"auto", "vgl", "vslam", "vslam_from_scratch"}:
                report.add(
                    "analysis.offline_localization_mode",
                    "Offline localization method",
                    BLOCKED,
                    "offline_localization_mode must be auto, vgl, vslam, or vslam_from_scratch.",
                    remediation="Select Auto, VGL + VSLAM, VSLAM only, or VSLAM from scratch.",
                    details={"value": offline_mode},
                )
                return
            report.resolved["offline_localization_mode_requested"] = offline_mode
            if offline_mode == "vslam_from_scratch":
                resolved_offline_mode = "vslam_from_scratch"
                report.resolved["offline_localization_mode"] = resolved_offline_mode
                report.add(
                    "analysis.offline_localization_mode",
                    "Offline localization method",
                    PASS,
                    "Offline localization will run VSLAM from scratch without loading an existing map.",
                    details={"requested": offline_mode, "resolved": resolved_offline_mode},
                )
                cameras = _inspect_camera_config(config, payload.get("topic_config"), report)
                if cameras is not None and bag_info is not None:
                    _check_mapping_topics(bag_info, cameras, report)
                report.add(
                    "analysis.trajectory_source",
                    "Trajectory source",
                    PASS,
                    "Offline localization will generate the trajectory from scratch using VSLAM before extraction.",
                    details={
                        "mode": "offline",
                        "localization_mode": resolved_offline_mode,
                    },
                )
            else:
                if map_dir is None:
                    return
                vgl_map_available, vslam_map_available = _offline_map_artifacts(
                    map_dir,
                    report,
                    localization_mode=offline_mode,
                )
                cameras = _inspect_camera_config(config, payload.get("topic_config"), report)
                if cameras is not None and bag_info is not None:
                    _check_mapping_topics(bag_info, cameras, report)
                vgl_model = None
                if offline_mode in {"auto", "vgl"} and vgl_map_available:
                    vgl_model = _inspect_vgl_model(
                        config,
                        payload.get("output_model_dir"),
                        report,
                        required=offline_mode == "vgl",
                    )
                resolved_offline_mode = offline_mode
                if offline_mode == "auto" and (not vgl_map_available or vgl_model is None):
                    resolved_offline_mode = "vslam"
                report.resolved["offline_localization_mode"] = resolved_offline_mode
                if resolved_offline_mode == "auto":
                    method_message = (
                        "Auto will try VGL first and restart from the beginning with a VSLAM "
                        "identity hint if needed."
                    )
                elif resolved_offline_mode == "vgl":
                    method_message = "VGL + VSLAM is required; runtime VGL failure will stop the job."
                else:
                    method_message = (
                        "Offline localization will load the saved cuVSLAM map and use an identity pose hint."
                    )
                report.add(
                    "analysis.offline_localization_mode",
                    "Offline localization method",
                    PASS if vslam_map_available else BLOCKED,
                    method_message,
                    details={
                        "requested": offline_mode,
                        "resolved": resolved_offline_mode,
                        "runtime_fallback": resolved_offline_mode == "auto",
                    },
                )
                report.add(
                    "analysis.trajectory_source",
                    "Trajectory source",
                    PASS,
                    "Offline localization will generate the trajectory before extraction.",
                    details={
                        "mode": "offline",
                        "localization_mode": resolved_offline_mode,
                        "map_dir": str(map_dir),
                    },
                )

    if not speed_topic and resolved_mode == "none":
        report.add(
            "analysis.speed_source",
            "Speed source",
            WARNING,
            "No speed topic or trajectory is available, so speed cannot be plotted.",
            remediation="Select a speed topic or enable recorded/offline trajectory generation.",
        )
    elif not speed_topic:
        report.resolved["speed_kind"] = "vehicle"
        report.resolved["speed_label"] = "Vehicle speed"
        report.add(
            "analysis.speed_source",
            "Speed source",
            PASS,
            "Speed can be read or derived from the selected trajectory source.",
        )


def _analysis_runtime(config: ConsoleConfig, report: _Report) -> None:
    setup = Path(config.ros2_ws) / "install" / "setup.bash"
    python_value = str(getattr(config, "python_bin", "python3") or "python3")
    python_path = Path(python_value).expanduser()
    python_available = (
        python_path.is_file()
        if python_path.is_absolute()
        else shutil.which(python_value) is not None
    )
    missing = []
    if not setup.is_file():
        missing.append(str(setup))
    if not python_available:
        missing.append(python_value)
    if missing:
        report.add(
            "analysis.runtime",
            "Linux/Docker analysis runtime",
            BLOCKED,
            "ROS workspace setup or the analysis Python interpreter is missing.",
            remediation=(
                "Run the Console in the built JetPilot Linux/Docker environment, source/build "
                "ros2_ws, and set PYTHON_BIN when using a custom Python environment."
            ),
            details={"missing": missing, "workspace_setup": str(setup), "python": python_value},
        )
        return
    report.add(
        "analysis.runtime",
        "Linux/Docker analysis runtime",
        PASS,
        "ROS workspace setup and the analysis Python interpreter are available.",
        details={"workspace_setup": str(setup), "python": python_value},
    )


def _inspect_rosbag(
    config: ConsoleConfig,
    raw_value: Any,
    report: _Report,
) -> dict[str, Any] | None:
    if raw_value is None or not str(raw_value).strip():
        report.add(
            "rosbag.path",
            "Rosbag folder",
            BLOCKED,
            "No rosbag was selected.",
            remediation="Select a rosbag under the configured record root.",
            details={"record_root": str(config.record_root)},
        )
        return None

    try:
        selected = Path(str(raw_value)).expanduser()
        bag_dir = resolve_under_root(
            selected,
            config.record_root,
            label="rosbag",
            require_exists=True,
            require_directory=True,
        )
    except ValueError as exc:
        report.add(
            "rosbag.path",
            "Rosbag folder",
            BLOCKED,
            str(exc),
            remediation="Select an existing rosbag folder inside RECORD_ROOT.",
            details={"record_root": str(config.record_root)},
        )
        return None

    report.resolved["rosbag"] = str(bag_dir)
    report.add(
        "rosbag.path",
        "Rosbag folder",
        PASS,
        "The rosbag folder is inside the configured record root.",
        details={"path": str(bag_dir)},
    )

    metadata_path = bag_dir / "metadata.yaml"
    try:
        metadata_text = metadata_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        report.add(
            "rosbag.metadata",
            "Rosbag metadata",
            BLOCKED,
            "metadata.yaml is missing or cannot be read.",
            remediation="Repair or re-copy the rosbag so metadata.yaml is present.",
            details={"path": str(metadata_path)},
        )
        return None

    if not metadata_text.strip() or "rosbag2_bagfile_information:" not in metadata_text:
        report.add(
            "rosbag.metadata",
            "Rosbag metadata",
            BLOCKED,
            "metadata.yaml does not contain rosbag2 bag information.",
            remediation="Regenerate rosbag metadata or select a complete rosbag directory.",
            details={"path": str(metadata_path)},
        )
        return None

    metadata = _parse_rosbag_metadata(metadata_text)
    topics = metadata["topics"]
    if topics:
        report.add(
            "rosbag.metadata",
            "Rosbag metadata",
            PASS,
            f"metadata.yaml is readable and describes {len(topics)} topic(s).",
            details={
                "path": str(metadata_path),
                "storage_identifier": metadata.get("storage_identifier") or "",
                "topic_count": len(topics),
            },
        )
    else:
        report.add(
            "rosbag.metadata",
            "Rosbag metadata",
            BLOCKED,
            "Topic metadata is empty or could not be parsed.",
            remediation="Regenerate metadata.yaml with `ros2 bag reindex` or select a complete bag.",
            details={"path": str(metadata_path), "topic_count": 0},
        )

    listed_paths = list(metadata["relative_file_paths"])
    storage_candidates: list[Path] = []
    unsafe_paths: list[str] = []
    if listed_paths:
        for relative_value in listed_paths:
            relative_path = Path(relative_value)
            candidate = (bag_dir / relative_path).resolve(strict=False)
            if relative_path.is_absolute() or not _is_relative_to(candidate, bag_dir):
                unsafe_paths.append(relative_value)
            else:
                storage_candidates.append(candidate)
    else:
        storage_candidates = sorted([*bag_dir.glob("*.mcap"), *bag_dir.glob("*.db3")])
        report.add(
            "rosbag.storage_index",
            "Rosbag storage index",
            WARNING,
            "metadata.yaml does not list storage files; files were inferred by extension.",
            remediation="Regenerate metadata.yaml with `ros2 bag reindex` when possible.",
            details={"inferred_files": [path.name for path in storage_candidates]},
        )

    if unsafe_paths:
        report.add(
            "rosbag.storage",
            "Rosbag storage",
            BLOCKED,
            "metadata.yaml references storage outside the rosbag folder.",
            remediation="Use an unmodified rosbag whose storage paths are relative and self-contained.",
            details={"unsafe_paths": unsafe_paths},
        )
        return None

    missing = [str(path) for path in storage_candidates if not path.is_file()]
    empty = [str(path) for path in storage_candidates if path.is_file() and path.stat().st_size <= 0]
    if not storage_candidates:
        report.add(
            "rosbag.storage",
            "Rosbag storage",
            BLOCKED,
            "No .mcap or .db3 storage file was found.",
            remediation="Select or copy the complete rosbag, including its storage files.",
            details={"path": str(bag_dir)},
        )
        return None
    if missing or empty:
        report.add(
            "rosbag.storage",
            "Rosbag storage",
            BLOCKED,
            "One or more rosbag storage files are missing or empty.",
            remediation="Re-copy or repair the rosbag before starting map generation.",
            details={"missing": missing, "empty": empty},
        )
        return None

    storage_identifier = str(metadata.get("storage_identifier") or "")
    extensions = sorted({path.suffix.lower() for path in storage_candidates})
    report.add(
        "rosbag.storage",
        "Rosbag storage",
        PASS,
        f"Found {len(storage_candidates)} non-empty storage file(s).",
        details={
            "files": [str(path) for path in storage_candidates],
            "extensions": extensions,
            "storage_identifier": storage_identifier,
        },
    )
    if storage_identifier and storage_identifier not in {"mcap", "sqlite3"}:
        report.add(
            "rosbag.storage_plugin",
            "Rosbag storage plugin",
            WARNING,
            f"Storage plugin {storage_identifier!r} is not one of the commonly tested formats.",
            remediation="Confirm that the corresponding rosbag2 storage plugin is installed.",
            details={"storage_identifier": storage_identifier},
        )

    return {
        "path": bag_dir,
        "metadata_path": metadata_path,
        "topics": topics,
        "storage_identifier": storage_identifier,
        "storage_files": storage_candidates,
    }


def _parse_rosbag_metadata(text: str) -> dict[str, Any]:
    storage_identifier = ""
    relative_paths: list[str] = []
    relative_paths_indent: int | None = None
    topics: dict[str, dict[str, Any]] = {}
    current_topic: dict[str, Any] | None = None
    topic_indent = -1
    qos_indent: int | None = None

    def flush_topic() -> None:
        nonlocal current_topic, qos_indent
        if current_topic and current_topic.get("name"):
            topics[str(current_topic["name"])] = dict(current_topic)
        current_topic = None
        qos_indent = None

    for raw_line in text.splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        if current_topic is not None and qos_indent is not None:
            if indent > qos_indent:
                previous = str(current_topic.get("offered_qos_profiles") or "")
                current_topic["offered_qos_profiles"] = "\n".join(
                    part for part in (previous, stripped) if part
                )
                continue
            qos_indent = None

        if stripped.startswith("storage_identifier:"):
            storage_identifier = _yaml_string(stripped.split(":", 1)[1])

        if stripped == "relative_file_paths:":
            relative_paths_indent = indent
            continue
        if relative_paths_indent is not None:
            if indent > relative_paths_indent and stripped.startswith("-"):
                value = _yaml_string(stripped[1:])
                if value:
                    relative_paths.append(value)
                continue
            if indent <= relative_paths_indent:
                relative_paths_indent = None

        if stripped in {"topic_metadata:", "- topic_metadata:"}:
            flush_topic()
            current_topic = {}
            topic_indent = indent
            continue
        if current_topic is not None:
            if indent <= topic_indent:
                flush_topic()
            else:
                key, separator, raw_value = stripped.partition(":")
                if separator and key in {"name", "type", "serialization_format"}:
                    current_topic[key] = _yaml_string(raw_value)
                elif separator and key == "offered_qos_profiles":
                    value = _yaml_string(raw_value)
                    current_topic[key] = "" if value in {"|", ">"} else value
                    qos_indent = indent
                elif separator and key == "message_count":
                    try:
                        current_topic[key] = int(_yaml_string(raw_value))
                    except ValueError:
                        current_topic[key] = None
    flush_topic()
    return {
        "storage_identifier": storage_identifier,
        "relative_file_paths": relative_paths,
        "topics": topics,
    }


def _yaml_string(value: str) -> str:
    text = value.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in {"'", '"'}:
        return text[1:-1]
    return text


def _inspect_camera_config(
    config: ConsoleConfig,
    raw_value: Any,
    report: _Report,
) -> list[dict[str, str]] | None:
    explicit = raw_value is not None and bool(str(raw_value).strip())
    selected = str(raw_value).strip() if explicit else str(default_topic_config(config))
    config_root = localization_config_dir(config)
    try:
        topic_path = resolve_under_root(
            selected,
            config_root,
            label="camera topic config",
            require_exists=True,
        )
        if not topic_path.is_file():
            raise ValueError(f"camera topic config must be a file: {topic_path}")
    except ValueError as exc:
        report.add(
            "camera.config",
            "Camera topic configuration",
            BLOCKED,
            str(exc),
            remediation=(
                "Choose a camera topic YAML inside the localization config directory."
                if explicit
                else "Add the default camera topic YAML or select a matching config before running."
            ),
            details={"config_root": str(config_root), "default_used": not explicit},
        )
        return None

    try:
        text = topic_path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        report.add(
            "camera.config",
            "Camera topic configuration",
            BLOCKED,
            f"Camera topic config cannot be read: {exc}",
            remediation="Fix the file permissions or choose another camera topic config.",
            details={"path": str(topic_path)},
        )
        return None

    cameras = _parse_camera_topics(text)
    if not cameras:
        report.add(
            "camera.config",
            "Camera topic configuration",
            BLOCKED,
            "No complete stereo camera entry was found in the selected config.",
            remediation="Define left/right image and CameraInfo topics under stereo_cameras.",
            details={"path": str(topic_path)},
        )
        return None

    report.resolved["topic_config"] = str(topic_path)
    report.add(
        "camera.config",
        "Camera topic configuration",
        PASS,
        f"Found {len(cameras)} complete stereo camera configuration(s).",
        details={"path": str(topic_path), "default_used": not explicit, "cameras": cameras},
    )
    return cameras


def _parse_camera_topics(text: str) -> list[dict[str, str]]:
    required = {"left", "right", "left_camera_info", "right_camera_info"}
    cameras: list[dict[str, str]] = []
    current: dict[str, str] = {}
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("- name:"):
            if required.issubset(current):
                cameras.append(current)
            current = {"name": _yaml_string(line.split(":", 1)[1])}
            continue
        key, separator, value = line.partition(":")
        if separator and key in required:
            current[key] = _yaml_string(value)
    if required.issubset(current):
        cameras.append(current)
    return cameras


def _check_mapping_topics(
    bag_info: Mapping[str, Any],
    cameras: list[dict[str, str]],
    report: _Report,
) -> None:
    topics = bag_info.get("topics")
    if not isinstance(topics, dict) or not topics:
        report.add(
            "rosbag.mapping_topics",
            "Mapping input topics",
            BLOCKED,
            "Topic metadata is unavailable, so required camera and TF inputs cannot be verified.",
            remediation="Regenerate bag metadata and confirm it with `ros2 bag info`.",
        )
        return

    required: dict[str, str] = {"/tf_static": "static transforms"}
    expected_types: dict[str, str] = {"/tf_static": "tf2_msgs/msg/TFMessage"}
    for camera in cameras:
        camera_name = camera.get("name") or "stereo camera"
        required[camera["left"]] = f"{camera_name} left image"
        required[camera["right"]] = f"{camera_name} right image"
        required[camera["left_camera_info"]] = f"{camera_name} left CameraInfo"
        required[camera["right_camera_info"]] = f"{camera_name} right CameraInfo"
        expected_types[camera["left"]] = "sensor_msgs/msg/Image"
        expected_types[camera["right"]] = "sensor_msgs/msg/Image"
        expected_types[camera["left_camera_info"]] = "sensor_msgs/msg/CameraInfo"
        expected_types[camera["right_camera_info"]] = "sensor_msgs/msg/CameraInfo"

    missing = [topic for topic in required if topic not in topics]
    empty = [
        topic
        for topic in required
        if topic in topics and topics[topic].get("message_count") == 0
    ]
    unknown_count = [
        topic
        for topic in required
        if topic in topics
        and (
            not isinstance(topics[topic].get("message_count"), int)
            or isinstance(topics[topic].get("message_count"), bool)
            or topics[topic].get("message_count") < 0
        )
    ]
    wrong_type = [
        {
            "topic": topic,
            "role": required[topic],
            "actual": str(topics[topic].get("type") or ""),
            "expected": expected_types[topic],
        }
        for topic in required
        if topic in topics and str(topics[topic].get("type") or "") != expected_types[topic]
    ]
    if missing or empty or unknown_count or wrong_type:
        report.add(
            "rosbag.mapping_topics",
            "Mapping input topics",
            BLOCKED,
            "The rosbag does not contain every topic required by the selected stereo config.",
            remediation="Record the missing topics or select the camera config that matches this bag.",
            details={
                "missing": [{"topic": topic, "role": required[topic]} for topic in missing],
                "empty": [{"topic": topic, "role": required[topic]} for topic in empty],
                "unknown_count": [
                    {"topic": topic, "role": required[topic]} for topic in unknown_count
                ],
                "wrong_type": wrong_type,
                "required": sorted(required),
            },
        )
        return

    report.add(
        "rosbag.mapping_topics",
        "Mapping input topics",
        PASS,
        "Stereo images, CameraInfo, and static transforms are present in the bag metadata.",
        details={"topics": sorted(required)},
    )
    _check_mapping_qos(topics, cameras, report)


def _check_mapping_qos(
    topics: Mapping[str, Any],
    cameras: list[dict[str, str]],
    report: _Report,
) -> None:
    camera_topics: dict[str, str] = {}
    for camera in cameras:
        camera_name = camera.get("name") or "stereo camera"
        camera_topics[camera["left"]] = f"{camera_name} left image"
        camera_topics[camera["right"]] = f"{camera_name} right image"
        camera_topics[camera["left_camera_info"]] = f"{camera_name} left CameraInfo"
        camera_topics[camera["right_camera_info"]] = f"{camera_name} right CameraInfo"

    unavailable: list[dict[str, str]] = []
    best_effort: list[dict[str, str]] = []
    for topic, role in camera_topics.items():
        info = topics.get(topic)
        if not isinstance(info, Mapping):
            continue
        reliabilities = _topic_qos_reliabilities(info)
        if not reliabilities:
            unavailable.append({"topic": topic, "role": role})
        elif "best_effort" in reliabilities and "reliable" not in reliabilities:
            best_effort.append({"topic": topic, "role": role, "reliability": "best_effort"})

    if best_effort:
        report.add(
            "rosbag.mapping_qos",
            "Mapping input QoS",
            WARNING,
            "Some selected camera topics are recorded as best-effort; VGL may request reliable QoS and receive no messages during replay.",
            remediation=(
                "Use a rosbag play QoS override for these topics or record the VGL input topics "
                "with reliable QoS."
            ),
            details={"best_effort": best_effort, "unavailable": unavailable},
        )
        return
    if unavailable:
        report.add(
            "rosbag.mapping_qos",
            "Mapping input QoS",
            WARNING,
            "metadata.yaml does not include QoS profiles for every selected camera topic.",
            remediation="Run a short offline replay check and watch for RELIABILITY_QOS_POLICY warnings.",
            details={"unavailable": unavailable},
        )
        return

    report.add(
        "rosbag.mapping_qos",
        "Mapping input QoS",
        PASS,
        "Selected camera topics include QoS metadata and are not best-effort only.",
        details={"topics": sorted(camera_topics)},
    )


def _topic_qos_reliabilities(topic_info: Mapping[str, Any]) -> set[str]:
    raw_profiles = topic_info.get("offered_qos_profiles")
    if raw_profiles is None:
        return set()
    text = str(raw_profiles).lower()
    values: set[str] = set()
    if "best_effort" in text or "besteffort" in text:
        values.add("best_effort")
    if "reliable" in text:
        values.add("reliable")
    return values


def _inspect_map_output(config: ConsoleConfig, raw_value: Any, report: _Report) -> None:
    if raw_value is None or not str(raw_value).strip():
        report.add(
            "map.output",
            "Map output folder",
            BLOCKED,
            "No map output folder was selected.",
            remediation="Choose a new map folder under MAP_ROOT.",
            details={"map_root": str(config.map_root)},
        )
        return

    map_root = Path(config.map_root).resolve(strict=False)
    if not map_root.exists() or not map_root.is_dir():
        report.add(
            "map.output",
            "Map output folder",
            BLOCKED,
            "The configured map root does not exist or is not a directory.",
            remediation="Mount or create MAP_ROOT before starting the map build.",
            details={"map_root": str(map_root)},
        )
        return
    try:
        map_dir = resolve_under_root(raw_value, map_root, label="map output directory")
    except ValueError as exc:
        report.add(
            "map.output",
            "Map output folder",
            BLOCKED,
            str(exc),
            remediation="Choose a map output folder inside MAP_ROOT.",
            details={"map_root": str(map_root)},
        )
        return

    if map_dir == map_root:
        report.add(
            "map.output",
            "Map output folder",
            BLOCKED,
            "MAP_ROOT itself cannot be used as one map bundle.",
            remediation="Create a named child folder for this map.",
            details={"path": str(map_dir)},
        )
        return
    if map_dir.exists() and not map_dir.is_dir():
        report.add(
            "map.output",
            "Map output folder",
            BLOCKED,
            "The selected output path exists and is not a directory.",
            remediation="Choose a different map name or remove the conflicting file.",
            details={"path": str(map_dir)},
        )
        return

    parent = map_dir if map_dir.exists() else _nearest_existing_parent(map_dir)
    if parent is None or not parent.is_dir() or not os.access(parent, os.W_OK):
        report.add(
            "map.output",
            "Map output folder",
            BLOCKED,
            "The map output folder cannot be created or written.",
            remediation="Fix the MAP_ROOT mount permissions or choose a writable folder.",
            details={"path": str(map_dir)},
        )
        return

    report.resolved["map_dir"] = str(map_dir)
    report.add(
        "map.output",
        "Map output folder",
        PASS,
        "The map output path is safe and writable.",
        details={"path": str(map_dir), "exists": map_dir.exists()},
    )
    if not _PORTABLE_MAP_NAME.fullmatch(map_dir.name):
        report.add(
            "map.output_name",
            "Map name compatibility",
            WARNING,
            "The map folder name contains characters that may not work with later transfer tools.",
            remediation="Use letters, numbers, dot, underscore, and hyphen for the map name.",
            details={"name": map_dir.name},
        )
    if map_dir.exists() and any(map_dir.iterdir()):
        report.add(
            "map.output_contents",
            "Existing map output",
            WARNING,
            "The selected map folder is not empty; generated artifacts may be replaced.",
            remediation="Use a new folder name if the existing artifacts must be preserved.",
            details={"path": str(map_dir)},
        )


def _nearest_existing_parent(path: Path) -> Path | None:
    candidate = path
    while not candidate.exists() and candidate != candidate.parent:
        candidate = candidate.parent
    return candidate if candidate.exists() else None


def _inspect_mapping_parameters(payload: Mapping[str, Any], report: _Report) -> None:
    raw_steps = payload.get("steps", "edex compute_poses cuvgl")
    fs_model_res = str(payload.get("fs_model_res") or "low_res").strip()
    if not isinstance(raw_steps, str):
        steps: list[str] = []
    else:
        steps = raw_steps.split()
    invalid_steps = [step for step in steps if not _MAP_BUILD_TOKEN.fullmatch(step)]
    if not steps or len(steps) > 16 or invalid_steps:
        report.add(
            "mapping.parameters",
            "Mapping parameters",
            BLOCKED,
            "Mapping steps are empty, too numerous, or contain unsupported characters.",
            remediation=(
                "Use up to 16 names of 1-64 characters, starting with a letter, number, or underscore."
            ),
            details={"steps": steps, "invalid_steps": invalid_steps},
        )
        return
    if not fs_model_res or not _MAP_BUILD_TOKEN.fullmatch(fs_model_res):
        report.add(
            "mapping.parameters",
            "Mapping parameters",
            BLOCKED,
            "FoundationStereo model resolution contains unsupported characters.",
            remediation="Choose a configured model resolution such as low_res.",
            details={"fs_model_res": fs_model_res},
        )
        return

    report.resolved["steps"] = " ".join(steps)
    report.resolved["fs_model_res"] = fs_model_res
    report.add(
        "mapping.parameters",
        "Mapping parameters",
        PASS,
        "Mapping step and model-resolution parameters are valid.",
        details={"steps": steps, "fs_model_res": fs_model_res},
    )


def _inspect_vgl_model(
    config: ConsoleConfig,
    raw_value: Any,
    report: _Report,
    *,
    required: bool = True,
) -> Path | None:
    model_root = Path(config.ros2_ws) / "isaac_ros_assets" / "models"
    selected = raw_value or model_root / "visual_global_localization"
    try:
        model_dir = resolve_under_root(
            selected,
            model_root,
            label="VGL model directory",
            require_exists=True,
            require_directory=True,
        )
    except ValueError as exc:
        report.add(
            "mapping.vgl_model",
            "VGL model assets",
            BLOCKED if required else WARNING,
            str(exc),
            remediation=(
                "Install/export the VGL model assets inside the ROS workspace model folder. "
                "Auto can otherwise continue with its VSLAM-only identity-hint fallback."
            ),
            details={"model_root": str(model_root), "fallback": not required},
        )
        return None

    report.resolved["output_model_dir"] = str(model_dir)
    if not any(model_dir.iterdir()):
        report.add(
            "mapping.vgl_model",
            "VGL model assets",
            BLOCKED if required else WARNING,
            "The selected VGL model directory is empty.",
            remediation=(
                "Export the required TensorRT/model files before the localization stage. "
                "Auto can otherwise continue with its VSLAM-only identity-hint fallback."
            ),
            details={"path": str(model_dir), "fallback": not required},
        )
        return None
    report.add(
        "mapping.vgl_model",
        "VGL model assets",
        PASS,
        "The selected VGL model directory exists and is not empty.",
        details={"path": str(model_dir)},
    )
    return model_dir


def _required_map_dir(
    config: ConsoleConfig,
    raw_value: Any,
    report: _Report,
) -> Path | None:
    if raw_value is None or not str(raw_value).strip():
        report.add(
            "map.directory",
            "Map folder",
            BLOCKED,
            "No map folder was selected.",
            remediation="Select an existing map bundle under MAP_ROOT.",
            details={"map_root": str(config.map_root)},
        )
        return None
    try:
        map_dir = resolve_under_root(
            raw_value,
            config.map_root,
            label="map directory",
            require_exists=True,
            require_directory=True,
        )
    except ValueError as exc:
        report.add(
            "map.directory",
            "Map folder",
            BLOCKED,
            str(exc),
            remediation="Select an existing map bundle inside MAP_ROOT.",
            details={"map_root": str(config.map_root)},
        )
        return None
    if map_dir == Path(config.map_root).resolve(strict=False):
        report.add(
            "map.directory",
            "Map folder",
            BLOCKED,
            "MAP_ROOT itself is not a map bundle.",
            remediation="Select a map bundle folder below MAP_ROOT.",
            details={"path": str(map_dir)},
        )
        return None
    if not os.access(map_dir, os.W_OK | os.X_OK):
        report.add(
            "map.directory",
            "Map folder",
            BLOCKED,
            "The selected map folder is not writable.",
            remediation="Fix the MAP_ROOT mount permissions before starting this stage.",
            details={"path": str(map_dir)},
        )
        return None
    report.resolved["map_dir"] = str(map_dir)
    report.add(
        "map.directory",
        "Map folder",
        PASS,
        "The selected map folder is inside MAP_ROOT and writable.",
        details={"path": str(map_dir)},
    )
    return map_dir


def _fixed_map_file_issue(path: Path, allowed_root: Path) -> str | None:
    """Return a safety issue for a fixed task file without following file symlinks."""

    if path.is_symlink():
        return "path is a symbolic link"
    try:
        resolved = path.resolve(strict=False)
    except OSError as exc:
        return f"path cannot be resolved: {exc}"
    if not _is_relative_to(resolved, allowed_root):
        return f"resolved path is outside {allowed_root}"
    if path.exists() and not path.is_file():
        return "existing path is not a regular file"
    return None


def _check_fixed_map_input(
    path: Path,
    allowed_root: Path,
    report: _Report,
    *,
    check_id: str,
    label: str,
    remediation: str,
) -> bool:
    issue = _fixed_map_file_issue(path, allowed_root)
    if issue is None:
        return True
    report.add(
        check_id,
        label,
        BLOCKED,
        f"Unsafe map artifact path: {issue}.",
        remediation=remediation,
        details={"path": str(path), "allowed_root": str(allowed_root)},
    )
    return False


def _inspect_stage_outputs(
    paths: list[Path],
    allowed_root: Path,
    report: _Report,
    *,
    check_id: str,
    label: str,
    remediation: str,
) -> bool:
    issues = []
    for path in paths:
        issue = _fixed_map_file_issue(path, allowed_root)
        if issue is None and path.exists() and not os.access(path, os.W_OK):
            issue = "existing output file is not writable"
        if issue is not None:
            issues.append({"path": str(path), "issue": issue})
    if issues:
        report.add(
            check_id,
            label,
            BLOCKED,
            "One or more output targets are symlinks, non-files, or resolve outside the map folder.",
            remediation=remediation,
            details={"issues": issues, "allowed_root": str(allowed_root)},
        )
        return False

    existing = [str(path) for path in paths if path.exists()]
    report.add(
        check_id,
        label,
        WARNING if existing else PASS,
        (
            "Existing regular output file(s) will be replaced."
            if existing
            else "Output targets are safe and available."
        ),
        remediation=("Back up existing outputs if they must be preserved." if existing else None),
        details={"targets": [str(path) for path in paths], "existing": existing},
    )
    return True


def _prepare_hd_raster_preflight(
    config: ConsoleConfig,
    map_dir: Path,
    report: _Report,
) -> None:
    local_snapshot = map_dir / "vslam_reference_snapshot.json"
    parent_snapshot = map_dir.parent / "vslam_reference_snapshot.json"
    snapshot = local_snapshot if local_snapshot.exists() or local_snapshot.is_symlink() else parent_snapshot
    if not _check_fixed_map_input(
        snapshot,
        Path(config.map_root).resolve(strict=False),
        report,
        check_id="raster.snapshot",
        label="VSLAM reference snapshot",
        remediation="Replace the snapshot with a regular file stored under MAP_ROOT.",
    ):
        return
    if not snapshot.is_file():
        report.add(
            "raster.snapshot",
            "VSLAM reference snapshot",
            BLOCKED,
            "No VSLAM reference snapshot was found in the map folder or its parent.",
            remediation="Run VGL/VSLAM map generation and save vslam_reference_snapshot.json first.",
            details={"searched": [str(local_snapshot), str(parent_snapshot)]},
        )
        return

    try:
        snapshot_data = json.loads(snapshot.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        report.add(
            "raster.snapshot",
            "VSLAM reference snapshot",
            BLOCKED,
            f"The VSLAM snapshot is not readable JSON: {exc}",
            remediation="Re-run the VSLAM snapshot recording stage.",
            details={"path": str(snapshot)},
        )
        return
    landmarks = snapshot_data.get("landmarks") if isinstance(snapshot_data, dict) else None
    landmark_issue = _landmark_snapshot_issue(landmarks)
    if landmark_issue:
        report.add(
            "raster.snapshot",
            "VSLAM reference snapshot",
            BLOCKED,
            landmark_issue,
            remediation="Re-run VSLAM with landmark visualization/recording enabled.",
            details={"path": str(snapshot)},
        )
        return

    report.resolved["snapshot"] = str(snapshot)
    report.add(
        "raster.snapshot",
        "VSLAM reference snapshot",
        PASS,
        "The VSLAM snapshot contains usable landmark data.",
        details={"path": str(snapshot), "source": "map" if snapshot == local_snapshot else "parent"},
    )
    outputs = [map_dir / "vslam_landmarks.yaml", map_dir / "vslam_landmarks.png"]
    _inspect_stage_outputs(
        outputs,
        map_dir,
        report,
        check_id="raster.outputs",
        label="Landmark raster outputs",
        remediation="Remove the unsafe output and let this stage create regular files.",
    )


def _landmark_snapshot_issue(value: Any) -> str | None:
    if not isinstance(value, dict) or not value:
        return "The VSLAM snapshot does not contain landmarks."
    data = value.get("data")
    if not isinstance(data, str) or not data:
        return "The landmark PointCloud2 payload is empty."
    try:
        point_step = int(value.get("point_step", 0))
    except (TypeError, ValueError):
        point_step = 0
    if point_step <= 0:
        return "The landmark PointCloud2 point_step is invalid."
    if len(data) % 4 != 0 or not _BASE64_PAYLOAD.fullmatch(data):
        return "The landmark PointCloud2 payload is not valid base64."
    padding = 2 if data.endswith("==") else (1 if data.endswith("=") else 0)
    decoded_size = (len(data) // 4) * 3 - padding
    if decoded_size < point_step:
        return "The landmark PointCloud2 payload does not contain one complete point."
    if decoded_size % point_step != 0:
        return "The landmark PointCloud2 payload size is not a multiple of point_step."
    fields = value.get("fields")
    field_names = {
        str(field.get("name"))
        for field in fields
        if isinstance(fields, list) and isinstance(field, dict)
    } if isinstance(fields, list) else set()
    if not {"x", "y", "z"}.issubset(field_names):
        return "The landmark PointCloud2 is missing x, y, or z fields."
    return None


def _generate_raceline_preflight(
    map_dir: Path,
    payload: Mapping[str, Any],
    report: _Report,
) -> None:
    name = map_dir.name
    centerline_path = map_dir / f"{name}_hd_map_centerline.csv"
    centerline = (
        _inspect_centerline(centerline_path, report)
        if _check_fixed_map_input(
            centerline_path,
            map_dir,
            report,
            check_id="raceline.centerline",
            label="Centerline and track widths",
            remediation="Save the centerline as a regular file inside the selected map folder.",
        )
        else None
    )
    clearance = _inspect_clearance_parameters(payload, report)
    if centerline is not None and clearance is not None:
        vehicle_width, safety_margin = clearance
        required_width = vehicle_width + 2.0 * safety_margin
        min_available = float(centerline["min_available_width_m"])
        if min_available + 1.0e-9 < required_width:
            report.add(
                "raceline.clearance",
                "Vehicle clearance",
                BLOCKED,
                "The track is too narrow for the requested vehicle envelope.",
                remediation="Reduce vehicle width/margin or correct the HD map lane boundaries.",
                details={
                    "minimum_available_width_m": min_available,
                    "required_envelope_width_m": required_width,
                    "vehicle_width_m": vehicle_width,
                    "safety_margin_m_per_side": safety_margin,
                    "narrowest_row": centerline["narrowest_row"],
                },
            )
        else:
            report.add(
                "raceline.clearance",
                "Vehicle clearance",
                PASS,
                "Every centerline point has enough left/right width for the requested envelope.",
                details={
                    "minimum_available_width_m": min_available,
                    "required_envelope_width_m": required_width,
                    "vehicle_width_m": vehicle_width,
                    "safety_margin_m_per_side": safety_margin,
                },
            )

    output = map_dir / f"{name}_raceline.csv"
    _inspect_stage_outputs(
        [output],
        map_dir,
        report,
        check_id="raceline.output",
        label="Raceline output",
        remediation="Remove the unsafe output and let the raceline stage create a regular CSV.",
    )


def _inspect_centerline(path: Path, report: _Report) -> dict[str, Any] | None:
    rows, issue = _numeric_csv_rows(path, delimiter=",", minimum_columns=4)
    if issue:
        report.add(
            "raceline.centerline",
            "Centerline and track widths",
            BLOCKED,
            issue,
            remediation="Save a valid HD map centerline CSV with x,y,w_right,w_left columns.",
            details={"path": str(path)},
        )
        return None
    if len(rows) < 8:
        report.add(
            "raceline.centerline",
            "Centerline and track widths",
            BLOCKED,
            f"The centerline has {len(rows)} point(s); raceline generation needs at least 8.",
            remediation="Add or regenerate centerline points in the HD map editor.",
            details={"path": str(path), "point_count": len(rows)},
        )
        return None

    invalid_width_rows = [
        index + 1
        for index, row in enumerate(rows)
        if row[2] < 0.0 or row[3] < 0.0 or row[2] + row[3] <= 0.0
    ]
    if invalid_width_rows:
        report.add(
            "raceline.centerline",
            "Centerline and track widths",
            BLOCKED,
            "Centerline track widths must be non-negative and have a positive total width.",
            remediation="Correct the left/right HD map bounds and save the centerline again.",
            details={"path": str(path), "invalid_rows": invalid_width_rows},
        )
        return None

    totals = [row[2] + row[3] for row in rows]
    narrowest_index = min(range(len(rows)), key=totals.__getitem__)
    result = {
        "point_count": len(rows),
        "min_available_width_m": totals[narrowest_index],
        "narrowest_row": narrowest_index + 1,
    }
    report.add(
        "raceline.centerline",
        "Centerline and track widths",
        PASS,
        "Centerline coordinates and left/right widths are finite and usable.",
        details={"path": str(path), **result},
    )
    return result


def _inspect_clearance_parameters(
    payload: Mapping[str, Any],
    report: _Report,
) -> tuple[float, float] | None:
    raw_vehicle = payload.get("vehicle_width_m", DEFAULT_RACELINE_VEHICLE_WIDTH_M)
    raw_margin = payload.get("safety_margin_m", DEFAULT_RACELINE_SAFETY_MARGIN_M)
    if raw_vehicle is None:
        raw_vehicle = DEFAULT_RACELINE_VEHICLE_WIDTH_M
    if raw_margin is None:
        raw_margin = DEFAULT_RACELINE_SAFETY_MARGIN_M
    try:
        if isinstance(raw_vehicle, bool) or isinstance(raw_margin, bool):
            raise ValueError
        vehicle_width = float(raw_vehicle)
        safety_margin = float(raw_margin)
        if (
            not math.isfinite(vehicle_width)
            or not math.isfinite(safety_margin)
            or vehicle_width < 0.0
            or safety_margin < 0.0
        ):
            raise ValueError
    except (TypeError, ValueError):
        report.add(
            "raceline.parameters",
            "Vehicle dimensions",
            BLOCKED,
            "Vehicle width and safety margin must be finite, non-negative numbers.",
            remediation="Enter the physical vehicle width and a per-side safety margin in metres.",
            details={"vehicle_width_m": raw_vehicle, "safety_margin_m": raw_margin},
        )
        return None

    report.resolved["vehicle_width_m"] = vehicle_width
    report.resolved["safety_margin_m"] = safety_margin
    status = WARNING if vehicle_width == 0.0 else PASS
    report.add(
        "raceline.parameters",
        "Vehicle dimensions",
        status,
        (
            "Vehicle width is zero, so the optimizer is not reserving a physical vehicle body."
            if status == WARNING
            else "Vehicle width and per-side safety margin are valid."
        ),
        remediation=("Enter the measured vehicle width for a safe raceline." if status == WARNING else None),
        details={
            "vehicle_width_m": vehicle_width,
            "safety_margin_m_per_side": safety_margin,
            "required_envelope_width_m": vehicle_width + 2.0 * safety_margin,
        },
    )
    return vehicle_width, safety_margin


def _numeric_csv_rows(
    path: Path,
    *,
    delimiter: str,
    minimum_columns: int,
) -> tuple[list[list[float]], str | None]:
    if not path.is_file():
        return [], f"Required CSV was not found: {path.name}"
    if path.stat().st_size <= 0:
        return [], f"Required CSV is empty: {path.name}"
    rows: list[list[float]] = []
    try:
        with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
            for line_number, row in enumerate(csv.reader(handle, delimiter=delimiter), start=1):
                if not row or row[0].strip().startswith("#"):
                    continue
                if len(row) < minimum_columns:
                    return rows, f"{path.name} row {line_number} has fewer than {minimum_columns} columns."
                try:
                    numeric = [float(value.strip()) for value in row[:minimum_columns]]
                except ValueError:
                    return rows, f"{path.name} row {line_number} contains a non-numeric value."
                if not all(math.isfinite(value) for value in numeric):
                    return rows, f"{path.name} row {line_number} contains a non-finite value."
                rows.append(numeric)
    except OSError as exc:
        return [], f"Could not read {path.name}: {exc}"
    if not rows:
        return [], f"{path.name} has no numeric data rows."
    return rows, None


def _generate_preview_preflight(
    config: ConsoleConfig,
    map_dir: Path,
    report: _Report,
) -> None:
    name = map_dir.name
    raster_path = map_dir / "vslam_landmarks.yaml"
    hd_map_path = map_dir / f"{name}_hd_map.yaml"
    centerline_path = map_dir / f"{name}_hd_map_centerline.csv"
    raceline_path = map_dir / f"{name}_raceline.csv"

    if _check_fixed_map_input(
        raster_path,
        map_dir,
        report,
        check_id="preview.raster",
        label="Landmark raster",
        remediation="Generate a regular vslam_landmarks.yaml inside the map folder.",
    ):
        _inspect_raster_yaml(config, raster_path, report)
    if _check_fixed_map_input(
        hd_map_path,
        map_dir,
        report,
        check_id="preview.hd_map",
        label="HD map",
        remediation="Save the HD map as a regular YAML file inside the map folder.",
    ):
        _inspect_hd_map(hd_map_path, report)
    if _check_fixed_map_input(
        centerline_path,
        map_dir,
        report,
        check_id="preview.centerline",
        label="Centerline",
        remediation="Save the centerline as a regular CSV inside the map folder.",
    ):
        _inspect_preview_centerline(centerline_path, report)
    if _check_fixed_map_input(
        raceline_path,
        map_dir,
        report,
        check_id="preview.raceline",
        label="Raceline",
        remediation="Generate the raceline as a regular CSV inside the map folder.",
    ):
        _inspect_raceline_csv(raceline_path, report)

    output = map_dir / f"{name}_line_preview.png"
    _inspect_stage_outputs(
        [output],
        map_dir,
        report,
        check_id="preview.output",
        label="Preview output",
        remediation="Remove the unsafe output and let the preview stage create a regular PNG.",
    )


def _inspect_raster_yaml(config: ConsoleConfig, path: Path, report: _Report) -> None:
    if not path.is_file() or path.stat().st_size <= 0:
        report.add(
            "preview.raster",
            "Landmark raster",
            BLOCKED,
            "vslam_landmarks.yaml is missing or empty.",
            remediation="Run Prepare HD raster before generating a preview.",
            details={"path": str(path)},
        )
        return
    try:
        data = load_yaml(path)
        resolution = float(data.get("resolution"))
        origin = data.get("origin")
        image_value = str(data.get("image") or "")
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if not isinstance(origin, list) or len(origin) < 2:
            raise ValueError("origin must contain x and y")
        origin_values = [float(value) for value in origin[:3]]
        if not all(math.isfinite(value) for value in origin_values):
            raise ValueError("origin must be finite")
        if not image_value:
            raise ValueError("image is missing")
        image_path = Path(image_value).expanduser()
        if not image_path.is_absolute():
            image_path = path.parent / image_path
        if image_path.is_symlink():
            raise ValueError("raster image must not be a symbolic link")
        image_path = image_path.resolve(strict=False)
        if not _is_relative_to(image_path, Path(config.map_root).resolve(strict=False)):
            raise ValueError("raster image is outside MAP_ROOT")
        if not image_path.is_file() or image_path.stat().st_size <= 0:
            raise ValueError("raster image is missing or empty")
    except (OSError, TypeError, ValueError) as exc:
        report.add(
            "preview.raster",
            "Landmark raster",
            BLOCKED,
            f"Landmark raster metadata is incomplete: {exc}",
            remediation="Re-run Prepare HD raster to regenerate YAML and image together.",
            details={"path": str(path)},
        )
        return
    report.add(
        "preview.raster",
        "Landmark raster",
        PASS,
        "Landmark raster metadata and image are usable.",
        details={"yaml_path": str(path), "image_path": str(image_path), "resolution": resolution},
    )


def _inspect_hd_map(path: Path, report: _Report) -> None:
    if not path.is_file() or path.stat().st_size <= 0:
        report.add(
            "preview.hd_map",
            "HD map",
            BLOCKED,
            "The HD map YAML is missing or empty.",
            remediation="Open the HD map editor and save lane boundaries and centerline.",
            details={"path": str(path)},
        )
        return
    try:
        data = load_yaml(path)
    except OSError as exc:
        data = {}
        issue = str(exc)
    else:
        issue = ""
    lanes = data.get("lanes") if isinstance(data, dict) else None
    primary_lane_id = str(data.get("primary_lane_id") or "") if isinstance(data, dict) else ""
    lane_by_id = {
        str(lane.get("id")): lane
        for lane in lanes
        if isinstance(lanes, list) and isinstance(lane, dict) and lane.get("id")
    } if isinstance(lanes, list) else {}
    primary = lane_by_id.get(primary_lane_id)
    if not lanes or not primary_lane_id or primary is None:
        report.add(
            "preview.hd_map",
            "HD map",
            BLOCKED,
            "HD map lane data or its primary lane is missing." + (f" ({issue})" if issue else ""),
            remediation="Save a primary lane with bounds and centerline in the HD map editor.",
            details={"path": str(path), "primary_lane_id": primary_lane_id},
        )
        return
    minimum_points = 3 if bool(primary.get("closed_loop", True)) else 2
    invalid_fields = []
    for field_name in ("left_bound", "right_bound", "centerline"):
        points = primary.get(field_name)
        if not isinstance(points, list) or len(points) < minimum_points or not _finite_points(points):
            invalid_fields.append(field_name)
    if invalid_fields:
        report.add(
            "preview.hd_map",
            "HD map",
            BLOCKED,
            "The primary lane has incomplete or invalid geometry.",
            remediation="Correct the primary lane bounds/centerline and save the HD map again.",
            details={"path": str(path), "invalid_fields": invalid_fields},
        )
        return
    report.add(
        "preview.hd_map",
        "HD map",
        PASS,
        "The HD map has a usable primary lane.",
        details={"path": str(path), "primary_lane_id": primary_lane_id, "lane_count": len(lane_by_id)},
    )


def _finite_points(value: list[Any]) -> bool:
    for row in value:
        if not isinstance(row, (list, tuple)) or len(row) < 2:
            return False
        try:
            x, y = float(row[0]), float(row[1])
        except (TypeError, ValueError):
            return False
        if not math.isfinite(x) or not math.isfinite(y):
            return False
    return True


def _inspect_preview_centerline(path: Path, report: _Report) -> None:
    rows, issue = _numeric_csv_rows(path, delimiter=",", minimum_columns=4)
    if issue or len(rows) < 2:
        report.add(
            "preview.centerline",
            "Centerline",
            BLOCKED,
            issue or "The centerline needs at least two points.",
            remediation="Save a valid centerline from the HD map editor.",
            details={"path": str(path), "point_count": len(rows)},
        )
        return
    invalid_widths = [index + 1 for index, row in enumerate(rows) if row[2] < 0 or row[3] < 0]
    if invalid_widths:
        report.add(
            "preview.centerline",
            "Centerline",
            BLOCKED,
            "The centerline contains negative track widths.",
            remediation="Correct the lane boundaries and save the centerline again.",
            details={"path": str(path), "invalid_rows": invalid_widths},
        )
        return
    report.add(
        "preview.centerline",
        "Centerline",
        PASS,
        "The centerline CSV is usable.",
        details={"path": str(path), "point_count": len(rows)},
    )


def _inspect_raceline_csv(path: Path, report: _Report) -> None:
    rows, issue = _numeric_csv_rows(path, delimiter=";", minimum_columns=3)
    if issue or len(rows) < 2:
        report.add(
            "preview.raceline",
            "Raceline",
            BLOCKED,
            issue or "The raceline needs at least two points.",
            remediation="Generate a valid raceline before creating the preview.",
            details={"path": str(path), "point_count": len(rows)},
        )
        return
    report.add(
        "preview.raceline",
        "Raceline",
        PASS,
        "The raceline CSV is usable.",
        details={"path": str(path), "point_count": len(rows)},
    )


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.resolve(strict=False).relative_to(root.resolve(strict=False))
        return True
    except ValueError:
        return False
