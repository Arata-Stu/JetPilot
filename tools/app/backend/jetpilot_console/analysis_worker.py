from __future__ import annotations

import argparse
import bisect
import fcntl
import hashlib
import json
import math
import os
import re
import struct
import sys
import traceback
import uuid
import zlib
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator, Mapping, Sequence


SCHEMA_VERSION = 1
MAX_EXTRACTED_FRAMES = 50_000
MODE_LABELS = {1: "AUTO", 2: "MANUAL", 3: "STOP", 4: "PROPO"}
JETSON_DIAGNOSTIC_TOPICS = (
    "/jetson/diagnostics",
    "/diagnostics",
)
OBJECT_DETECTION_TOPICS = (
    "/perception/detections",
)
OBJECT_DETECTION_OVERLAY_TOPIC = "/perception/detections_overlay"
JETSON_METRIC_ORDER = {
    "cpu": 10,
    "gpu": 20,
    "mem": 30,
    "temp": 40,
    "power": 50,
    "fan": 60,
    "engine": 70,
}
JETSON_VALUE_KEYS = {
    "cpu": {"message", "User", "System", "Freq"},
    "gpu": {"message", "Used", "Freq"},
    "mem": {"Use", "Total", "Bandwidth", "Freq"},
    "temp": {"message"},
    "power": {"Power", "Average"},
    "fan": {"PWM 0", "RPM 0"},
    "engine": None,
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@contextmanager
def _json_lock(path: Path) -> Iterator[None]:
    path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = path.with_name(f".{path.name}.lock")
    with lock_path.open("a+b") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _write_json_unlocked(path: Path, payload: object) -> None:
    temporary = path.with_name(
        f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp"
    )
    try:
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, payload: object) -> None:
    with _json_lock(path):
        _write_json_unlocked(path, payload)


def _update_json_object(
    path: Path,
    update: Callable[[dict[str, object]], Mapping[str, object]],
) -> dict[str, object]:
    """Read, merge, and atomically replace one JSON object under one lock."""

    with _json_lock(path):
        current: dict[str, object] = {}
        if path.is_file():
            try:
                loaded = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(loaded, dict):
                    current = loaded
            except (OSError, json.JSONDecodeError):
                pass
        updated = dict(update(dict(current)))
        _write_json_unlocked(path, updated)
        return updated


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def _nested(value: Any, *paths: str, default: Any = None) -> Any:
    for path in paths:
        current = value
        try:
            for part in path.split("."):
                current = getattr(current, part)
        except (AttributeError, TypeError):
            continue
        return current
    return default


def _stamp_ns(message: Any) -> int | None:
    stamp = _nested(message, "header.stamp")
    if stamp is None:
        return None
    value = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
        getattr(stamp, "nanosec", 0)
    )
    return value if value > 0 else None


def _quaternion_yaw(orientation: Any) -> float:
    x = _finite(getattr(orientation, "x", 0.0))
    y = _finite(getattr(orientation, "y", 0.0))
    z = _finite(getattr(orientation, "z", 0.0))
    w = _finite(getattr(orientation, "w", 1.0), 1.0)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _pose_payload(pose: Any) -> dict[str, float]:
    position = getattr(pose, "position", pose)
    orientation = getattr(pose, "orientation", None)
    return {
        "x": _finite(getattr(position, "x", 0.0)),
        "y": _finite(getattr(position, "y", 0.0)),
        "z": _finite(getattr(position, "z", 0.0)),
        "yaw": _quaternion_yaw(orientation) if orientation is not None else 0.0,
    }


def _imu_payload(message: Any) -> dict[str, float]:
    acceleration = getattr(message, "linear_acceleration", None)
    angular = getattr(message, "angular_velocity", None)
    return {
        "accel_x": _finite(getattr(acceleration, "x", 0.0)) / 9.80665,
        "accel_y": _finite(getattr(acceleration, "y", 0.0)) / 9.80665,
        "accel_z": _finite(getattr(acceleration, "z", 0.0)) / 9.80665,
        "gyro_x": _finite(getattr(angular, "x", 0.0)) / 5.0,
        "gyro_y": _finite(getattr(angular, "y", 0.0)) / 5.0,
        "gyro_z": _finite(getattr(angular, "z", 0.0)) / 5.0,
        "valid": 1.0,
    }


def _speed_from_twist(twist: Any) -> float | None:
    if twist is None:
        return None
    linear = getattr(twist, "linear", twist)
    values = [
        _finite(getattr(linear, "x", 0.0)),
        _finite(getattr(linear, "y", 0.0)),
        _finite(getattr(linear, "z", 0.0)),
    ]
    return math.sqrt(sum(value * value for value in values))


def _control_payload(message: Any) -> dict[str, object]:
    return {
        "steering": _finite(
            _nested(message, "steering", "steer", "steering_angle", "drive.steering_angle")
        ),
        "throttle": _finite(
            _nested(message, "throttle", "accel", "acceleration", "drive.speed")
        ),
        "brake": _finite(_nested(message, "brake")),
        "reverse": bool(_nested(message, "reverse", default=False)),
    }


def _mode_payload(message: Any) -> dict[str, object]:
    raw_mode = _nested(message, "mode", "data", default=0)
    try:
        mode = int(raw_mode)
    except (TypeError, ValueError):
        mode = 0
    label = MODE_LABELS.get(mode, str(raw_mode or "UNKNOWN").upper())
    return {
        "mode": mode,
        "label": label,
        "source": str(_nested(message, "source", default="") or ""),
    }


def _string_payload(message: Any) -> str:
    return str(_nested(message, "data", default="") or "")


def _e2e_diagnostic_payload(message: Any) -> dict[str, object] | None:
    statuses = getattr(message, "status", None)
    if not isinstance(statuses, (list, tuple)):
        return None
    for status in statuses:
        name = str(getattr(status, "name", "") or "").lower()
        hardware_id = str(getattr(status, "hardware_id", "") or "").lower()
        if "e2e" not in name and "e2e" not in hardware_id:
            continue
        payload: dict[str, object] = {
            "level": int(getattr(status, "level", 0) or 0),
            "message": str(getattr(status, "message", "") or ""),
        }
        for value in getattr(status, "values", []) or []:
            key = str(getattr(value, "key", "") or "").strip()
            if not key:
                continue
            parsed = _jetson_numeric_value(getattr(value, "value", ""))
            if parsed is not None:
                payload[key] = parsed[0]
            else:
                payload[key] = str(getattr(value, "value", "") or "")
        return payload
    return None


def _object_detection_payload(message: Any) -> list[dict[str, object]]:
    payload: list[dict[str, object]] = []
    for detection in getattr(message, "detections", []) or []:
        results = list(getattr(detection, "results", []) or [])
        if not results:
            continue
        best = max(
            results,
            key=lambda result: _finite(_nested(result, "hypothesis.score", default=0.0)),
        )
        center_x = _finite(
            _nested(detection, "bbox.center.position.x", "bbox.center.x", default=0.0)
        )
        center_y = _finite(
            _nested(detection, "bbox.center.position.y", "bbox.center.y", default=0.0)
        )
        size_x = max(0.0, _finite(_nested(detection, "bbox.size_x", default=0.0)))
        size_y = max(0.0, _finite(_nested(detection, "bbox.size_y", default=0.0)))
        if size_x <= 0.0 or size_y <= 0.0:
            continue
        payload.append(
            {
                "class_id": str(
                    _nested(best, "hypothesis.class_id", default="unknown") or "unknown"
                ),
                "score": round(
                    _finite(_nested(best, "hypothesis.score", default=0.0)), 6
                ),
                "x_min": round(center_x - size_x * 0.5, 3),
                "y_min": round(center_y - size_y * 0.5, 3),
                "x_max": round(center_x + size_x * 0.5, 3),
                "y_max": round(center_y + size_y * 0.5, 3),
            }
        )
    return payload


def _jetson_numeric_value(raw_value: Any) -> tuple[float, str] | None:
    text = str(raw_value or "").strip()
    if not text or text.lower() in {"offline", "disable", "disabled", "auto"}:
        return None
    match = re.search(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", text)
    if match is None:
        return None
    value = float(match.group(0))
    lower = text.lower()
    if "%" in text:
        return value, "%"
    if "rpm" in lower:
        return value, "rpm"
    if lower.endswith("c") or " c" in lower:
        return value, "C"
    if "ghz" in lower:
        return value * 1000.0, "MHz"
    if "mhz" in lower:
        return value, "MHz"
    if re.search(r"\bkhz\b", lower):
        return value / 1000.0, "MHz"
    if re.search(r"\bmw\b", lower):
        return value / 1000.0, "W"
    if re.search(r"\bw\b", lower):
        return value, "W"
    if lower.endswith("gib") or lower.endswith("gb") or lower.endswith("g"):
        return value, "GB"
    if lower.endswith("mib") or lower.endswith("mb") or lower.endswith("m"):
        return value / 1024.0, "GB"
    if lower.endswith("kib") or lower.endswith("kb") or lower.endswith("k"):
        return value / (1024.0 * 1024.0), "GB"
    return value, ""


def _jetson_series_label(category: str, leaf: str, key: str) -> str:
    if key == "message":
        if category == "cpu":
            return f"CPU {leaf} used"
        if category == "temp":
            return f"Temp {leaf}"
        return leaf
    if category == "engine":
        return f"{leaf} freq"
    if category == "mem" and leaf == "ram" and key in {"Use", "Total"}:
        return f"RAM {key.lower()}"
    if category == "gpu" and key == "Used":
        return "GPU used"
    return f"{leaf} {key}".strip()


def _jetson_metric_samples(message: Any, bag_timestamp_ns: int) -> list[dict[str, object]]:
    samples: list[dict[str, object]] = []
    cpu_used_values: list[float] = []
    for status in getattr(message, "status", ()):
        name = str(getattr(status, "name", "") or "")
        if not name.startswith("jetson_stats/"):
            continue
        parts = name.split("/")
        if len(parts) < 3:
            continue
        category = parts[1]
        leaf = "/".join(parts[2:])
        if category not in JETSON_VALUE_KEYS:
            continue
        if category == "board":
            continue

        values: dict[str, str] = {}
        for pair in getattr(status, "values", ()):
            key = str(getattr(pair, "key", "") or "")
            values[key] = str(getattr(pair, "value", "") or "")
        if category in {"cpu", "gpu", "temp"}:
            values.setdefault("message", str(getattr(status, "message", "") or ""))

        allowed = JETSON_VALUE_KEYS[category]
        for key, raw_value in values.items():
            if allowed is not None and key not in allowed:
                continue
            parsed = _jetson_numeric_value(raw_value)
            if parsed is None:
                continue
            value, unit = parsed
            if category == "cpu" and key == "message" and unit == "%":
                cpu_used_values.append(value)
            metric_id = f"{category}/{leaf}/{key}".replace(" ", "_")
            samples.append(
                {
                    "_timestamp_ns": int(bag_timestamp_ns),
                    "_header_timestamp_ns": None,
                    "id": metric_id,
                    "label": _jetson_series_label(category, leaf, key),
                    "group": category,
                    "unit": unit,
                    "value": round(value, 6),
                    "source": name,
                    "key": key,
                    "order": JETSON_METRIC_ORDER.get(category, 99),
                }
            )
    if cpu_used_values:
        samples.append(
            {
                "_timestamp_ns": int(bag_timestamp_ns),
                "_header_timestamp_ns": None,
                "id": "cpu/all/used",
                "label": "CPU avg used",
                "group": "cpu",
                "unit": "%",
                "value": round(sum(cpu_used_values) / len(cpu_used_values), 6),
                "source": "jetson_stats/cpu",
                "key": "used",
                "order": JETSON_METRIC_ORDER["cpu"] - 1,
            }
        )
    return samples


def _explicit_speed(message: Any) -> float | None:
    twist = _nested(message, "twist.twist", "twist")
    if twist is None and hasattr(message, "linear"):
        twist = message
    if twist is not None:
        return _speed_from_twist(twist)
    value = _nested(
        message,
        "speed_mps",
        "velocity_mps",
        "velocity",
        "speed",
        "data",
        default=None,
    )
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _pose_samples(message: Any, bag_timestamp_ns: int) -> Iterable[dict[str, object]]:
    frame_id = str(_nested(message, "header.frame_id", default="") or "")
    child_frame_id = str(getattr(message, "child_frame_id", "") or "")
    header_timestamp_ns = _stamp_ns(message)
    twist = _nested(message, "twist.twist", "twist")
    speed = _speed_from_twist(twist)

    poses = getattr(message, "poses", None)
    if poses is not None:
        for pose_stamped in poses:
            pose = _nested(pose_stamped, "pose", default=pose_stamped)
            timestamp_ns = _stamp_ns(pose_stamped) or header_timestamp_ns or bag_timestamp_ns
            sample = _pose_payload(pose)
            sample.update(
                {
                    "_timestamp_ns": int(timestamp_ns),
                    "_header_timestamp_ns": _stamp_ns(pose_stamped),
                    "frame_id": str(
                        _nested(pose_stamped, "header.frame_id", default=frame_id) or frame_id
                    ),
                    "child_frame_id": child_frame_id,
                    "speed_mps": speed,
                }
            )
            yield sample
        return

    pose = _nested(message, "pose.pose", "pose", default=None)
    if pose is None:
        return
    sample = _pose_payload(pose)
    sample.update(
        {
            "_timestamp_ns": int(bag_timestamp_ns),
            "_header_timestamp_ns": header_timestamp_ns,
            "frame_id": frame_id,
            "child_frame_id": child_frame_id,
            "speed_mps": speed,
        }
    )
    yield sample


def _normal_frame(value: Any) -> str:
    return str(value or "").strip().lstrip("/")


def _map_transform_samples(message: Any, bag_timestamp_ns: int) -> list[dict[str, object]]:
    """Extract planar map<-frame transforms from a TFMessage-like value."""

    samples: list[dict[str, object]] = []
    for stamped in getattr(message, "transforms", ()):
        parent = _normal_frame(_nested(stamped, "header.frame_id", default=""))
        child = _normal_frame(getattr(stamped, "child_frame_id", ""))
        transform = getattr(stamped, "transform", None)
        if transform is None or "map" not in {parent, child}:
            continue
        translation = getattr(transform, "translation", None)
        rotation = getattr(transform, "rotation", None)
        x = _finite(getattr(translation, "x", 0.0))
        y = _finite(getattr(translation, "y", 0.0))
        z = _finite(getattr(translation, "z", 0.0))
        yaw = _quaternion_yaw(rotation)
        source_frame = child
        if child == "map":
            # Invert frame<-map into map<-frame.
            inverse_yaw = -yaw
            cosine, sine = math.cos(inverse_yaw), math.sin(inverse_yaw)
            x, y = cosine * -x - sine * -y, sine * -x + cosine * -y
            z = -z
            yaw = inverse_yaw
            source_frame = parent
        if parent != "map" and child != "map":
            continue
        samples.append(
            {
                "_timestamp_ns": int(bag_timestamp_ns),
                "source_frame": source_frame,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
            }
        )
    return samples


def _transform_recorded_trajectory(
    trajectory: Sequence[dict[str, object]],
    transform_samples: Sequence[dict[str, object]],
) -> tuple[list[dict[str, object]], int, int]:
    """Apply the latest recorded map<-frame TF to each pose.

    Once a matching transform stream exists, pre-localization samples without a
    preceding map transform are intentionally discarded instead of being drawn as
    if their odom coordinates belonged to the selected map.
    """

    by_frame: dict[str, list[dict[str, object]]] = {}
    for transform in transform_samples:
        frame = _normal_frame(transform.get("source_frame"))
        if frame:
            by_frame.setdefault(frame, []).append(transform)
    for values in by_frame.values():
        values.sort(key=lambda sample: int(sample["_timestamp_ns"]))
    transform_times = {
        frame: [int(value["_timestamp_ns"]) for value in values]
        for frame, values in by_frame.items()
    }

    source_frames = {
        _normal_frame(sample.get("frame_id"))
        for sample in trajectory
        if _normal_frame(sample.get("frame_id")) != "map"
    }
    if not any(frame in by_frame for frame in source_frames):
        return list(trajectory), 0, 0

    transformed: list[dict[str, object]] = []
    transformed_count = 0
    dropped_count = 0
    for sample in trajectory:
        frame = _normal_frame(sample.get("frame_id"))
        if frame == "map":
            normalized = dict(sample)
            normalized["frame_id"] = "map"
            transformed.append(normalized)
            continue
        candidates = by_frame.get(frame, [])
        timestamps = transform_times.get(frame, [])
        index = bisect.bisect_right(timestamps, int(sample["_timestamp_ns"])) - 1
        if index < 0:
            dropped_count += 1
            continue
        transform = candidates[index]
        yaw = _finite(transform.get("yaw"))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        result = dict(sample)
        source_x = _finite(sample.get("x"))
        source_y = _finite(sample.get("y"))
        result.update(
            {
                "x": _finite(transform.get("x")) + cosine * source_x - sine * source_y,
                "y": _finite(transform.get("y")) + sine * source_x + cosine * source_y,
                "z": _finite(transform.get("z")) + _finite(sample.get("z")),
                "yaw": math.atan2(
                    math.sin(yaw + _finite(sample.get("yaw"))),
                    math.cos(yaw + _finite(sample.get("yaw"))),
                ),
                "source_frame_id": frame,
                "frame_id": "map",
            }
        )
        transformed.append(result)
        transformed_count += 1
    return transformed, transformed_count, dropped_count


def _detect_storage_id(bag_path: Path) -> str:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"ROS bag metadata was not found: {metadata_path}")
    match = re.search(
        r"^\s*storage_identifier:\s*['\"]?([^'\"\s]+)",
        metadata_path.read_text(encoding="utf-8", errors="replace"),
        flags=re.MULTILINE,
    )
    if match is None:
        raise RuntimeError(f"storage_identifier is missing from {metadata_path}")
    return match.group(1)


def _metadata_message_count(bag_path: Path) -> int | None:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        return None
    text = metadata_path.read_text(encoding="utf-8", errors="replace")
    match = re.search(r"^  message_count:\s*(\d+)\s*$", text, flags=re.MULTILINE)
    if not match:
        return None
    value = int(match.group(1))
    return value if value > 0 else None


def _metadata_duration_ns(bag_path: Path) -> int | None:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        return None
    text = metadata_path.read_text(encoding="utf-8", errors="replace")
    match = re.search(
        r"^\s*duration:\s*$\n\s*nanoseconds:\s*(\d+)\s*$",
        text,
        flags=re.MULTILINE,
    )
    if not match:
        return None
    value = int(match.group(1))
    return value if value > 0 else None


def _open_reader(bag_path: Path):
    try:
        import rosbag2_py
    except ImportError as error:
        raise RuntimeError(
            "rosbag2_py is required. Run this worker after sourcing the ROS 2 workspace."
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=_detect_storage_id(bag_path)),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    return reader, topic_types


def _deserializers():
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "rclpy and rosidl_runtime_py are required for analysis extraction."
        ) from error
    return deserialize_message, get_message


def _encoding_layout(encoding: str):
    import numpy as np

    layouts = {
        "mono8": (np.dtype(np.uint8), 1),
        "8uc1": (np.dtype(np.uint8), 1),
        "bgr8": (np.dtype(np.uint8), 3),
        "rgb8": (np.dtype(np.uint8), 3),
        "8uc3": (np.dtype(np.uint8), 3),
        "bgra8": (np.dtype(np.uint8), 4),
        "rgba8": (np.dtype(np.uint8), 4),
        "mono16": (np.dtype(np.uint16), 1),
        "16uc1": (np.dtype(np.uint16), 1),
        "bgr16": (np.dtype(np.uint16), 3),
        "rgb16": (np.dtype(np.uint16), 3),
        "bgra16": (np.dtype(np.uint16), 4),
        "rgba16": (np.dtype(np.uint16), 4),
        "bayer_rggb8": (np.dtype(np.uint8), 1),
        "bayer_bggr8": (np.dtype(np.uint8), 1),
        "bayer_gbrg8": (np.dtype(np.uint8), 1),
        "bayer_grbg8": (np.dtype(np.uint8), 1),
        "bayer_rggb16": (np.dtype(np.uint16), 1),
        "bayer_bggr16": (np.dtype(np.uint16), 1),
        "bayer_gbrg16": (np.dtype(np.uint16), 1),
        "bayer_grbg16": (np.dtype(np.uint16), 1),
        "yuv422": (np.dtype(np.uint8), 2),
        "yuv422_yuy2": (np.dtype(np.uint8), 2),
        "uyvy": (np.dtype(np.uint8), 2),
        "16sc1": (np.dtype(np.int16), 1),
        "32fc1": (np.dtype(np.float32), 1),
    }
    if encoding not in layouts:
        raise ValueError(f"Unsupported sensor_msgs/Image encoding: {encoding}")
    return layouts[encoding]


# Global state for thermal / 16-bit image contrast smoothing (topic -> (moving_low, moving_high, prev_gray_mean))
_THERMAL_SMOOTHING_STATE: dict[str, tuple[float, float, float]] = {}


def _decode_image(message: Any, topic: str = ""):
    try:
        import cv2
        import numpy as np
    except ImportError as error:
        raise RuntimeError("OpenCV and NumPy are required to decode image topics.") from error

    if hasattr(message, "format") and hasattr(message, "data"):
        raw_data = bytes(message.data)
        encoded = np.frombuffer(raw_data, dtype=np.uint8)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if decoded is None:
            offsets = [
                offset
                for signature in (b"\x89PNG\r\n\x1a\n", b"\xff\xd8\xff")
                if (offset := raw_data.find(signature)) > 0
            ]
            if offsets:
                decoded = cv2.imdecode(
                    np.frombuffer(raw_data[min(offsets) :], dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
        if decoded is None:
            raise RuntimeError("Failed to decode sensor_msgs/CompressedImage")
        return decoded

    encoding = str(message.encoding).lower()
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    dtype, channels = _encoding_layout(encoding)
    row_elements = step // dtype.itemsize
    array = np.frombuffer(message.data, dtype=dtype).reshape(height, row_elements)
    array = array[:, : width * channels]
    array = array.reshape(height, width, channels) if channels > 1 else array.reshape(height, width)
    if bool(getattr(message, "is_bigendian", False)) and dtype.itemsize > 1:
        array = array.byteswap()

    is_flir = "flir" in str(topic).lower() or "thermal" in str(topic).lower() or "infra" in str(topic).lower()
    topic_key = topic or "default"

    if array.dtype in {np.dtype(np.uint16), np.dtype(np.int16), np.dtype(np.float32)}:
        finite = np.isfinite(array)
        if not finite.any():
            if array.dtype == np.uint16:
                array = (array / 256.0).clip(0, 255).astype(np.uint8)
            else:
                array = np.full(array.shape, 128, dtype=np.uint8)
        else:
            valid = array[finite]
            raw_low, raw_high = np.percentile(valid, (0.5, 99.5))
            if raw_high <= raw_low:
                raw_high = raw_low + 1.0

            # Smooth contrast min/max over time using Exponential Moving Average (EMA)
            if topic_key in _THERMAL_SMOOTHING_STATE:
                prev_low, prev_high, prev_mean = _THERMAL_SMOOTHING_STATE[topic_key]
                alpha = 0.03  # Stronger smoothing (0.03) to eliminate flicker
                low = prev_low * (1.0 - alpha) + raw_low * alpha
                high = prev_high * (1.0 - alpha) + raw_high * alpha
            else:
                low, high, prev_mean = raw_low, raw_high, -1.0
            _THERMAL_SMOOTHING_STATE[topic_key] = (low, high, prev_mean)

            span = max(1.0e-3, high - low)
            array = np.clip((array - low) * (255.0 / span), 0, 255)
            array[~finite] = 0
            array = array.astype(np.uint8)

    bayer_codes = {
        "bayer_rggb8": cv2.COLOR_BAYER_RG2BGR,
        "bayer_bggr8": cv2.COLOR_BAYER_BG2BGR,
        "bayer_gbrg8": cv2.COLOR_BAYER_GB2BGR,
        "bayer_grbg8": cv2.COLOR_BAYER_GR2BGR,
        "bayer_rggb16": cv2.COLOR_BAYER_RG2BGR,
        "bayer_bggr16": cv2.COLOR_BAYER_BG2BGR,
        "bayer_gbrg16": cv2.COLOR_BAYER_GB2BGR,
        "bayer_grbg16": cv2.COLOR_BAYER_GR2BGR,
    }
    if encoding in bayer_codes:
        array = cv2.cvtColor(array, bayer_codes[encoding])
    elif encoding in {"yuv422", "yuv422_yuy2"}:
        array = cv2.cvtColor(array, cv2.COLOR_YUV2BGR_YUY2)
    elif encoding == "uyvy":
        array = cv2.cvtColor(array, cv2.COLOR_YUV2BGR_UYVY)
    elif encoding in {"rgb8", "rgb16"}:
        array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    elif encoding in {"rgba8", "rgba16"}:
        array = cv2.cvtColor(array, cv2.COLOR_RGBA2BGR)
    elif encoding in {"bgra8", "bgra16"}:
        array = cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
    elif len(array.shape) == 2 or channels == 1:
        # Monochrome / Thermal processing (Apply CLAHE and Luminance Stabilization for FLIR)
        if is_flir:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            array = clahe.apply(array)

            # Global luminance stabilization (Suppress single-frame auto-exposure / shutter spikes)
            current_mean = float(np.mean(array))
            if topic_key in _THERMAL_SMOOTHING_STATE:
                prev_low, prev_high, prev_mean = _THERMAL_SMOOTHING_STATE[topic_key]
                if prev_mean >= 0 and abs(current_mean - prev_mean) > 25.0:  # Sudden brightness jump
                    # Blend towards previous mean
                    target_ratio = prev_mean / max(1.0, current_mean)
                    array = np.clip(array.astype(np.float32) * target_ratio, 0, 255).astype(np.uint8)
                    current_mean = float(np.mean(array))
                new_mean = prev_mean * 0.8 + current_mean * 0.2 if prev_mean >= 0 else current_mean
                _THERMAL_SMOOTHING_STATE[topic_key] = (prev_low, prev_high, new_mean)
            else:
                _THERMAL_SMOOTHING_STATE[topic_key] = (0.0, 255.0, current_mean)

        array = cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)

    return array


def _write_jpeg(path: Path, image: Any, quality: int) -> tuple[int, int]:
    try:
        import cv2
    except ImportError as error:
        raise RuntimeError("OpenCV is required to write analysis frames.") from error
    height, width = image.shape[:2]
    if not cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, quality]):
        raise RuntimeError(f"Failed to write image frame: {path}")
    return int(width), int(height)


def _render_detection_overlays(
    analysis_dir: Path,
    frames: list[dict[str, object]],
    detection_samples: list[dict[str, object]],
    jpeg_quality: int,
    *,
    max_delta_ms: float = 250.0,
) -> int:
    if not frames or not detection_samples:
        return 0
    try:
        import cv2
    except ImportError as error:
        raise RuntimeError("OpenCV is required to render detection overlays.") from error

    def sync_timestamp_ns(sample: Mapping[str, object]) -> int:
        return int(sample.get("_header_timestamp_ns") or sample.get("_timestamp_ns") or 0)

    ordered = sorted(detection_samples, key=sync_timestamp_ns)
    timestamps = [sync_timestamp_ns(sample) for sample in ordered]
    overlay_dir = analysis_dir / "frames" / _topic_slug(OBJECT_DETECTION_OVERLAY_TOPIC)
    overlay_dir.mkdir(parents=True, exist_ok=True)
    palette = ((64, 220, 255), (255, 170, 64), (96, 232, 120), (224, 96, 224))
    rendered = 0

    for frame_index, frame in enumerate(frames):
        timestamp_ns = sync_timestamp_ns(frame)
        insert_at = bisect.bisect_left(timestamps, timestamp_ns)
        candidates = [index for index in (insert_at - 1, insert_at) if 0 <= index < len(ordered)]
        if not candidates:
            continue
        nearest_index = min(candidates, key=lambda index: abs(timestamps[index] - timestamp_ns))
        delta_ms = (timestamps[nearest_index] - timestamp_ns) / 1e6
        if abs(delta_ms) > max_delta_ms:
            continue

        source_path = analysis_dir / str(frame.get("path") or "")
        image = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        image_height, image_width = image.shape[:2]
        for item_index, detection in enumerate(ordered[nearest_index].get("detections", [])):
            if not isinstance(detection, dict):
                continue
            x_min = max(0, min(image_width - 1, round(_finite(detection.get("x_min")))))
            y_min = max(0, min(image_height - 1, round(_finite(detection.get("y_min")))))
            x_max = max(0, min(image_width - 1, round(_finite(detection.get("x_max")))))
            y_max = max(0, min(image_height - 1, round(_finite(detection.get("y_max")))))
            if x_max <= x_min or y_max <= y_min:
                continue
            color = palette[item_index % len(palette)]
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, 2)
            label = f"{detection.get('class_id', 'unknown')} {_finite(detection.get('score')):.2f}"
            text_y = max(14, y_min - 5)
            cv2.putText(
                image,
                label,
                (x_min, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )

        filename = f"frame_{frame_index:08d}.jpg"
        relative_path = f"frames/{_topic_slug(OBJECT_DETECTION_OVERLAY_TOPIC)}/{filename}"
        width, height = _write_jpeg(overlay_dir / filename, image, jpeg_quality)
        channels = frame.setdefault("channels", {})
        if isinstance(channels, dict):
            channels[OBJECT_DETECTION_OVERLAY_TOPIC] = {
                "path": relative_path,
                "width": width,
                "height": height,
                "delta_ms": round(delta_ms, 3),
            }
        rendered += 1
    return rendered


def _snapshot_samples(path: Path) -> list[dict[str, object]]:
    if not path.is_file():
        raise RuntimeError(f"Offline localization snapshot was not created: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError("Offline localization snapshot root must be an object")
    localization = data.get("localization")
    if not isinstance(localization, dict):
        raise RuntimeError("Offline localization snapshot is missing 'localization' metadata.")
    require_localized_map = bool(localization.get("required"))
    if require_localized_map:
        # Strict path: VGL/VSLAM map-localized mode must confirm localization.
        if not localization.get("confirmed"):
            raise RuntimeError(
                "VGL/VSLAM did not reach the confirmed localized state; no offline trajectory was accepted."
            )
        try:
            accepted_samples = int(localization.get("accepted_odometry_samples") or 0)
            minimum_samples = int(localization.get("minimum_required_odometry_samples") or 2)
        except (TypeError, ValueError):
            accepted_samples, minimum_samples = 0, 2
        if "accepted_odometry_samples" in localization and accepted_samples < minimum_samples:
            raise RuntimeError(
                f"Offline localization produced only {accepted_samples} accepted samples; "
                f"at least {minimum_samples} are required."
            )
        raw_samples = data.get("odometry_samples")
        if not isinstance(raw_samples, list) or not raw_samples:
            raise RuntimeError(
                "Offline localization was confirmed but produced no synchronized map-frame odometry samples."
            )
        samples: list[dict[str, object]] = []
        for raw in raw_samples or []:
            if not isinstance(raw, dict):
                continue
            try:
                header_timestamp_ns = int(raw.get("timestamp_ns"))
                received_timestamp_ns = int(raw.get("received_timestamp_ns") or 0)
                timestamp_ns = received_timestamp_ns or header_timestamp_ns
            except (TypeError, ValueError):
                continue
            if timestamp_ns <= 0:
                continue
            frame_id = str(raw.get("frame_id") or "").strip().lstrip("/")
            if frame_id != "map":
                raise RuntimeError(
                    f"Offline localization sample is in {frame_id or '<empty>'} frame, not map."
                )
            pose = raw.get("pose") or {}
            position = pose.get("position") or {}
            orientation = pose.get("orientation") or {}
            twist = raw.get("twist") or {}
            linear = twist.get("linear") or {}
            speed = math.sqrt(
                sum(_finite(linear.get(axis)) ** 2 for axis in ("x", "y", "z"))
            )
            sample = {
                "_timestamp_ns": timestamp_ns,
                "_header_timestamp_ns": header_timestamp_ns or None,
                "x": _finite(position.get("x")),
                "y": _finite(position.get("y")),
                "z": _finite(position.get("z")),
                "yaw": _quaternion_yaw_dict(orientation),
                "speed_mps": speed,
                "frame_id": frame_id,
                "child_frame_id": str(raw.get("child_frame_id") or ""),
            }
            samples.append(sample)
        if not samples:
            raise RuntimeError(
                "Offline localization produced no samples on the rosbag /clock time axis."
            )
        return samples
    else:
        # Relaxed path: vslam_from_scratch records in odom frame without map localization.
        # Accept any received odometry samples regardless of frame.
        raw_samples = data.get("odometry_samples")
        if not isinstance(raw_samples, list) or not raw_samples:
            raise RuntimeError(
                "VSLAM from scratch produced no odometry samples in the snapshot."
            )
        samples = []
        for raw in raw_samples:
            if not isinstance(raw, dict):
                continue
            try:
                header_timestamp_ns = int(raw.get("timestamp_ns"))
                received_timestamp_ns = int(raw.get("received_timestamp_ns") or 0)
                timestamp_ns = received_timestamp_ns or header_timestamp_ns
            except (TypeError, ValueError):
                continue
            if timestamp_ns <= 0:
                continue
            pose = raw.get("pose") or {}
            position = pose.get("position") or {}
            orientation = pose.get("orientation") or {}
            twist = raw.get("twist") or {}
            linear = twist.get("linear") or {}
            speed = math.sqrt(
                sum(_finite(linear.get(axis)) ** 2 for axis in ("x", "y", "z"))
            )
            frame_id = str(raw.get("frame_id") or "odom").strip().lstrip("/")
            sample = {
                "_timestamp_ns": timestamp_ns,
                "_header_timestamp_ns": header_timestamp_ns or None,
                "x": _finite(position.get("x")),
                "y": _finite(position.get("y")),
                "z": _finite(position.get("z")),
                "yaw": _quaternion_yaw_dict(orientation),
                "speed_mps": speed,
                "frame_id": frame_id,
                "child_frame_id": str(raw.get("child_frame_id") or ""),
            }
            samples.append(sample)
        if not samples:
            raise RuntimeError(
                "VSLAM from scratch produced no usable odometry samples in the snapshot."
            )
        return samples


def _quaternion_yaw_dict(orientation: Mapping[str, Any]) -> float:
    x = _finite(orientation.get("x"))
    y = _finite(orientation.get("y"))
    z = _finite(orientation.get("z"))
    w = _finite(orientation.get("w"), 1.0)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _deduplicate_trajectory(samples: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    by_timestamp: dict[int, dict[str, object]] = {}
    for sample in samples:
        timestamp_ns = int(sample["_timestamp_ns"])
        by_timestamp[timestamp_ns] = sample
    return [by_timestamp[key] for key in sorted(by_timestamp)]


def _fill_trajectory_speeds(samples: list[dict[str, object]]) -> None:
    previous: dict[str, object] | None = None
    for sample in samples:
        speed = sample.get("speed_mps")
        if speed is not None and math.isfinite(_finite(speed)):
            previous = sample
            continue
        if previous is None:
            sample["speed_mps"] = 0.0
        else:
            dt = (int(sample["_timestamp_ns"]) - int(previous["_timestamp_ns"])) / 1e9
            distance = math.hypot(
                _finite(sample.get("x")) - _finite(previous.get("x")),
                _finite(sample.get("y")) - _finite(previous.get("y")),
            )
            sample["speed_mps"] = distance / dt if dt > 1.0e-6 else 0.0
        previous = sample


def _normalise_samples(samples: list[dict[str, object]], start_ns: int) -> None:
    for sample in samples:
        timestamp_ns = int(sample.pop("_timestamp_ns"))
        header_timestamp_ns = sample.pop("_header_timestamp_ns", None)
        sample["timestamp_ns"] = str(timestamp_ns)
        sample["t"] = round((timestamp_ns - start_ns) / 1e9, 6)
        if header_timestamp_ns:
            sample["header_timestamp_ns"] = str(int(header_timestamp_ns))
            sample["header_delta_ms"] = round((int(header_timestamp_ns) - timestamp_ns) / 1e6, 3)


def _downsample(samples: list[dict[str, object]], maximum: int = 50_000) -> list[dict[str, object]]:
    if len(samples) <= maximum:
        return samples
    stride = math.ceil(len(samples) / maximum)
    result = samples[::stride]
    if result[-1] is not samples[-1]:
        result.append(samples[-1])
    return result


def _file_fingerprint(path: Path) -> str:
    digest = hashlib.sha256()
    if path.is_file():
        digest.update(path.read_bytes())
    elif path.is_dir():
        for child in sorted(item for item in path.rglob("*") if item.is_file()):
            relative = child.relative_to(path).as_posix()
            stat = child.stat()
            digest.update(relative.encode("utf-8"))
            digest.update(str(stat.st_size).encode("ascii"))
            digest.update(str(stat.st_mtime_ns).encode("ascii"))
    return digest.hexdigest()


def _map_geometry(map_dir: Path) -> dict[str, object] | None:
    yaml_path = map_dir / "vslam_landmarks.yaml"
    if not yaml_path.is_file():
        return None
    try:
        from .map_detail import _png_size, load_yaml

        data = load_yaml(yaml_path)
        image_value = str(data.get("image") or "")
        image_path = Path(image_value)
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        size = _png_size(image_path) if image_path.is_file() else None
        origin = data.get("origin") if isinstance(data.get("origin"), list) else [0, 0, 0]
        return {
            "yaml_path": str(yaml_path),
            "image_path": str(image_path),
            "resolution": _finite(data.get("resolution")),
            "origin": [
                _finite(origin[index] if len(origin) > index else 0.0) for index in range(3)
            ],
            "width": int(size[0]) if size else 0,
            "height": int(size[1]) if size else 0,
        }
    except (OSError, TypeError, ValueError):
        return None


def trajectory_map_consistency(
    samples: Sequence[Mapping[str, object]], geometry: Mapping[str, object] | None
) -> dict[str, object]:
    if not samples:
        return {
            "status": "unavailable",
            "message": "自己位置の軌跡がないためMap整合性を判定できません。",
            "inside_fraction": None,
        }
    frame_ids = sorted({str(sample.get("frame_id") or "") for sample in samples})
    overlay_ready = bool(frame_ids) and all(frame == "map" for frame in frame_ids)
    if not geometry:
        return {
            "status": "unknown",
            "message": "Mapのraster情報がないため範囲一致を判定できません。",
            "inside_fraction": None,
            "trajectory_frames": frame_ids,
            "overlay_ready": overlay_ready,
        }
    resolution = _finite(geometry.get("resolution"))
    width = int(geometry.get("width") or 0)
    height = int(geometry.get("height") or 0)
    origin = geometry.get("origin") or [0.0, 0.0, 0.0]
    if resolution <= 0 or width <= 0 or height <= 0:
        return {
            "status": "unknown",
            "message": "Mapの解像度または画像サイズが不足しています。",
            "inside_fraction": None,
            "trajectory_frames": frame_ids,
            "overlay_ready": overlay_ready,
        }
    ox, oy, yaw = (_finite(origin[index] if len(origin) > index else 0.0) for index in range(3))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    inside = 0
    for sample in samples:
        dx = _finite(sample.get("x")) - ox
        dy = _finite(sample.get("y")) - oy
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        if 0.0 <= local_x < width * resolution and 0.0 <= local_y < height * resolution:
            inside += 1
    fraction = inside / len(samples)
    if not overlay_ready:
        status = "warning"
        message = "軌跡frameがmapではないため、Map重畳にはTF変換が必要です。"
    elif fraction >= 0.8:
        status = "pass"
        message = "軌跡の大部分が選択Mapの範囲内です。"
    elif fraction >= 0.5:
        status = "warning"
        message = "軌跡の一部が選択Mapの範囲外です。Map選択と座標系を確認してください。"
    else:
        status = "mismatch"
        message = "軌跡の大部分が選択Mapの範囲外です。誤ったMapの可能性があります。"
    return {
        "status": status,
        "message": message,
        "inside_fraction": round(fraction, 4),
        "inside_count": inside,
        "sample_count": len(samples),
        "trajectory_frames": frame_ids,
        "overlay_ready": overlay_ready,
    }


@dataclass
class AnalysisOptions:
    rosbag: Path
    analysis_dir: Path
    image_topic: str = ""
    image_topics: list[str] = field(default_factory=list)
    primary_image_topic: str = ""
    control_topic: str = ""
    mode_topic: str = ""
    pose_topic: str = ""
    speed_topic: str = ""
    comparison_control_topic: str = ""
    section_topic: str = ""
    e2e_diagnostic_topic: str = ""
    detection_bag: Path | None = None
    detection_manifest: Path | None = None
    imu_topic: str = ""
    map_dir: Path | None = None
    trajectory_snapshot: Path | None = None
    status_file: Path | None = None
    max_fps: float = 10.0
    jpeg_quality: int = 85
    expected_map_fingerprint: str = ""


class Progress:
    def __init__(self, path: Path) -> None:
        self.path = path

    def update(
        self,
        stage: str,
        progress: float,
        message: str,
        *,
        status: str = "running",
        preserve_existing_failed: bool = False,
    ) -> None:
        def merge(previous: dict[str, object]) -> Mapping[str, object]:
            if preserve_existing_failed and previous.get("status") == "failed":
                return previous
            return {
                **previous,
                "schema_version": SCHEMA_VERSION,
                "status": status,
                "stage": stage,
                "phase": stage,
                "progress": max(0.0, min(1.0, float(progress))),
                "message": message,
                "updated_at": _utc_now(),
            }

        _update_json_object(
            self.path,
            merge,
        )


def _topic_slug(topic: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", topic.strip().lstrip("/")).strip("_")
    return slug or "camera"


def _read_detection_sidecar(path: Path) -> tuple[str, list[dict[str, object]]]:
    sidecar = path.expanduser().resolve()
    if not sidecar.is_dir():
        raise FileNotFoundError(f"Detection sidecar bag was not found: {sidecar}")
    reader, topic_types = _open_reader(sidecar)
    topic = next(
        (
            candidate
            for candidate in OBJECT_DETECTION_TOPICS
            if candidate in topic_types
            and topic_types[candidate].endswith("vision_msgs/msg/Detection2DArray")
        ),
        "",
    )
    if not topic:
        raise RuntimeError("Detection sidecar contains no supported Detection2DArray topic")
    deserialize_message, get_message = _deserializers()
    message_class = get_message(topic_types[topic])
    samples: list[dict[str, object]] = []
    while reader.has_next():
        current_topic, serialized, bag_timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_class)
        header_timestamp_ns = _stamp_ns(message)
        # Sidecar receive timestamps may use wall time. The detector preserves the
        # source image stamp, which is the stable clock for model-to-model comparison.
        timestamp_ns = header_timestamp_ns or int(bag_timestamp_ns)
        samples.append(
            {
                "_timestamp_ns": timestamp_ns,
                "_header_timestamp_ns": header_timestamp_ns,
                "detections": _object_detection_payload(message),
            }
        )
    if not samples:
        raise RuntimeError("Detection sidecar contains no decodable detection messages")
    return topic, samples


def extract_analysis(options: AnalysisOptions) -> dict[str, object]:
    options.rosbag = options.rosbag.expanduser().resolve()
    options.analysis_dir = options.analysis_dir.expanduser().resolve()
    if not options.rosbag.is_dir():
        raise FileNotFoundError(f"ROS bag folder was not found: {options.rosbag}")

    # Resolve image topics
    image_topics: list[str] = []
    for t in options.image_topics:
        for part in str(t).split(","):
            val = part.strip()
            if val and val not in image_topics:
                image_topics.append(val)
    if not image_topics and options.image_topic:
        for part in options.image_topic.split(","):
            val = part.strip()
            if val and val not in image_topics:
                image_topics.append(val)

    if not image_topics:
        raise ValueError("At least one image topic is required (--image-topic or --image-topics)")

    primary_image_topic = (
        options.primary_image_topic.strip()
        if options.primary_image_topic and options.primary_image_topic.strip() in image_topics
        else image_topics[0]
    )

    if not math.isfinite(options.max_fps) or options.max_fps <= 0:
        raise ValueError("--max-fps must be greater than zero")
    if options.jpeg_quality < 1 or options.jpeg_quality > 100:
        raise ValueError("--jpeg-quality must be between 1 and 100")
    if options.map_dir and options.expected_map_fingerprint:
        current_map_fingerprint = _file_fingerprint(options.map_dir)
        if current_map_fingerprint != options.expected_map_fingerprint:
            raise RuntimeError(
                "The selected Map changed after preflight. Re-run analysis with the current Map."
            )

    analysis_dir = options.analysis_dir
    frames_dir = analysis_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    topic_slugs = {topic: _topic_slug(topic) for topic in image_topics}
    for slug in topic_slugs.values():
        (frames_dir / slug).mkdir(parents=True, exist_ok=True)

    progress = Progress(
        options.status_file.expanduser().resolve()
        if options.status_file
        else analysis_dir / "status.json"
    )
    progress_floor = (
        0.66 if options.detection_bag else (0.42 if options.trajectory_snapshot else 0.0)
    )

    def task_progress(extraction_progress: float) -> float:
        return progress_floor + (1.0 - progress_floor) * extraction_progress

    progress.update("inspect", task_progress(0.02), "rosbagのtopic構成を確認しています。")

    offline_localization_method = ""
    if options.trajectory_snapshot:
        trajectory_snapshot = options.trajectory_snapshot.expanduser().resolve()
        method_path = trajectory_snapshot.parent / "method.txt"
        if method_path.is_file():
            try:
                candidate_method = method_path.read_text(
                    encoding="utf-8", errors="replace"
                ).strip()
            except OSError:
                candidate_method = ""
            if candidate_method in {
                "vgl", "vslam_identity", "vslam_identity_fallback", "vslam_from_scratch"
            }:
                offline_localization_method = candidate_method
        offline_snapshot_samples = _snapshot_samples(trajectory_snapshot)
    else:
        offline_snapshot_samples = None

    reader, topic_types = _open_reader(options.rosbag)
    missing_image_topics = [t for t in image_topics if t not in topic_types]
    if missing_image_topics:
        raise RuntimeError("Image topics were not found in bag: " + ", ".join(missing_image_topics))

    requested = set(image_topics)
    for topic in (
        options.control_topic,
        options.comparison_control_topic,
        options.mode_topic,
        options.pose_topic,
        options.speed_topic,
        options.section_topic,
        options.e2e_diagnostic_topic,
        options.imu_topic,
    ):
        if topic:
            requested.add(topic)
    jetson_diagnostic_topics = [
        topic
        for topic in JETSON_DIAGNOSTIC_TOPICS
        if topic in topic_types and topic_types[topic].endswith("DiagnosticArray")
    ]
    requested.update(jetson_diagnostic_topics)
    object_detection_topic = "" if options.detection_bag else next(
        (
            topic
            for topic in OBJECT_DETECTION_TOPICS
            if topic in topic_types
            and topic_types[topic].endswith("vision_msgs/msg/Detection2DArray")
        ),
        "",
    )
    if object_detection_topic:
        requested.add(object_detection_topic)

    recorded_tf_topic = "/tf" if options.pose_topic and "/tf" in topic_types else ""
    if recorded_tf_topic:
        requested.add(recorded_tf_topic)
    missing_topics = sorted(topic for topic in requested if topic not in topic_types)
    if missing_topics:
        raise RuntimeError("Selected topics were not found: " + ", ".join(missing_topics))

    deserialize_message, get_message = _deserializers()
    message_classes = {topic: get_message(topic_types[topic]) for topic in requested}
    frames: list[dict[str, object]] = []
    controls: list[dict[str, object]] = []
    comparison_controls: list[dict[str, object]] = []
    modes: list[dict[str, object]] = []
    sections: list[dict[str, object]] = []
    e2e_diagnostics: list[dict[str, object]] = []
    object_detections: list[dict[str, object]] = []
    imu_samples: list[dict[str, object]] = []
    speeds: list[dict[str, object]] = []
    trajectory: list[dict[str, object]] = []
    jetson_samples: list[dict[str, object]] = []
    recorded_map_transforms: list[dict[str, object]] = []
    timestamps: list[int] = []
    last_frame_timestamp_ns: int | None = None
    latest_decoded_images: dict[str, dict[str, object]] = {}
    bag_duration_ns = _metadata_duration_ns(options.rosbag)
    effective_max_fps = options.max_fps
    if bag_duration_ns:
        effective_max_fps = min(
            effective_max_fps,
            MAX_EXTRACTED_FRAMES / (bag_duration_ns / 1.0e9),
        )
    min_frame_interval_ns = max(1, int(1e9 / effective_max_fps))
    total_message_count = _metadata_message_count(options.rosbag)
    read_count = 0
    selected_count = 0
    image_decode_errors = 0
    image_decode_error_examples: list[str] = []
    frame_limit_reached = False
    progress.update("extract", task_progress(0.08), "画像と走行signalを抽出しています。")

    while reader.has_next():
        topic, serialized, bag_timestamp_ns = reader.read_next()
        read_count += 1
        if topic not in requested:
            continue
        selected_count += 1
        timestamp_ns = int(bag_timestamp_ns)
        message = deserialize_message(serialized, message_classes[topic])
        if recorded_tf_topic and topic == recorded_tf_topic:
            recorded_map_transforms.extend(_map_transform_samples(message, timestamp_ns))
            continue
        timestamps.append(int(bag_timestamp_ns))
        header_timestamp_ns = _stamp_ns(message)
        # Use bag_timestamp_ns as the unified master clock for cross-camera synchronization
        timestamp_ns = int(bag_timestamp_ns)

        common = {
            "_timestamp_ns": timestamp_ns,
            "_header_timestamp_ns": header_timestamp_ns,
        }

        if topic in image_topics:
            try:
                image = _decode_image(message, topic)
                latest_decoded_images[topic] = {
                    "image": image,
                    "timestamp_ns": timestamp_ns,
                    "header_timestamp_ns": header_timestamp_ns,
                }
            except Exception as exc:  # noqa: BLE001
                image_decode_errors += 1
                if len(image_decode_error_examples) < 3:
                    image_decode_error_examples.append(str(exc))
                continue

            if topic == primary_image_topic:
                if len(frames) >= MAX_EXTRACTED_FRAMES:
                    frame_limit_reached = True
                    continue

                if (
                    last_frame_timestamp_ns is not None
                    and timestamp_ns - last_frame_timestamp_ns < min_frame_interval_ns
                ):
                    continue

                last_frame_timestamp_ns = timestamp_ns

                frame_idx = len(frames)
                filename = f"frame_{frame_idx:08d}.jpg"
                channels_payload: dict[str, object] = {}
                primary_path = ""
                primary_width = 0
                primary_height = 0

                for img_topic in image_topics:
                    latest = latest_decoded_images.get(img_topic)
                    if latest is None:
                        continue
                    delta_ms = round((int(latest["timestamp_ns"]) - timestamp_ns) / 1e6, 3)

                    slug = topic_slugs[img_topic]
                    rel_path = f"frames/{slug}/{filename}"
                    out_path = frames_dir / slug / filename
                    w, h = _write_jpeg(out_path, latest["image"], options.jpeg_quality)
                    channels_payload[img_topic] = {
                        "path": rel_path,
                        "width": w,
                        "height": h,
                        "delta_ms": delta_ms,
                    }
                    if img_topic == primary_image_topic:
                        primary_path = rel_path
                        primary_width = w
                        primary_height = h

                if not primary_path:
                    primary_slug = topic_slugs[primary_image_topic]
                    primary_path = f"frames/{primary_slug}/{filename}"
                    w, h = _write_jpeg(
                        frames_dir / primary_slug / filename,
                        image,
                        options.jpeg_quality,
                    )
                    primary_width, primary_height = w, h

                frames.append(
                    {
                        **common,
                        "path": primary_path,
                        "width": primary_width,
                        "height": primary_height,
                        "channels": channels_payload,
                    }
                )
                last_frame_timestamp_ns = timestamp_ns
        if options.control_topic and topic == options.control_topic:
            controls.append({**common, **_control_payload(message)})
        if options.comparison_control_topic and topic == options.comparison_control_topic:
            comparison_controls.append({**common, **_control_payload(message)})
        if options.mode_topic and topic == options.mode_topic:
            modes.append({**common, **_mode_payload(message)})
        if options.section_topic and topic == options.section_topic:
            sections.append({**common, "section": _string_payload(message)})
        if options.e2e_diagnostic_topic and topic == options.e2e_diagnostic_topic:
            payload = _e2e_diagnostic_payload(message)
            if payload is not None:
                e2e_diagnostics.append({**common, **payload})
        if object_detection_topic and topic == object_detection_topic:
            object_detections.append(
                {**common, "detections": _object_detection_payload(message)}
            )
        if options.imu_topic and topic == options.imu_topic:
            imu_samples.append({**common, **_imu_payload(message)})
        if options.speed_topic and topic == options.speed_topic:
            value = _explicit_speed(message)
            if value is not None:
                speeds.append({**common, "value": value, "source": options.speed_topic})
        if options.pose_topic and topic == options.pose_topic:
            for sample in _pose_samples(message, timestamp_ns):
                trajectory.append(sample)
        if topic in jetson_diagnostic_topics:
            jetson_samples.extend(_jetson_metric_samples(message, timestamp_ns))

        if selected_count % 1000 == 0:
            phase = (
                min(0.8, 0.08 + 0.72 * read_count / total_message_count)
                if total_message_count
                else min(0.72, 0.08 + 0.08 * math.log10(max(10, selected_count)))
            )
            progress.update(
                "extract",
                task_progress(phase),
                (
                    f"rosbag {read_count:,}/{total_message_count:,}件を走査、"
                    f"対象{selected_count:,}件、画像{len(frames):,}枚を生成しました。"
                    if total_message_count
                    else f"対象{selected_count:,}件を処理、画像{len(frames):,}枚を生成しました。"
                ),
            )

    detection_source = "recorded" if object_detections else "none"
    detection_metadata: dict[str, object] = {}
    if options.detection_bag:
        object_detection_topic, object_detections = _read_detection_sidecar(
            options.detection_bag
        )
        detection_source = "offline_sidecar"
        if options.detection_manifest and options.detection_manifest.is_file():
            try:
                loaded_detection_metadata = json.loads(
                    options.detection_manifest.read_text(encoding="utf-8")
                )
            except (OSError, json.JSONDecodeError):
                loaded_detection_metadata = {}
            if isinstance(loaded_detection_metadata, dict):
                detection_metadata = loaded_detection_metadata

    if not frames:
        detail = f" Last decoder error: {image_decode_error_examples[-1]}" if image_decode_error_examples else ""
        raise RuntimeError(
            f"No decodable image messages were found on {options.image_topic}.{detail}"
        )

    detection_overlay_frames = _render_detection_overlays(
        analysis_dir,
        frames,
        object_detections,
        options.jpeg_quality,
    )

    recorded_transformed_count = 0
    recorded_dropped_count = 0
    if trajectory and recorded_map_transforms:
        trajectory, recorded_transformed_count, recorded_dropped_count = (
            _transform_recorded_trajectory(trajectory, recorded_map_transforms)
        )

    if offline_snapshot_samples is not None:
        trajectory = offline_snapshot_samples
        timestamps.extend(int(sample["_timestamp_ns"]) for sample in offline_snapshot_samples)
        trajectory_source = "offline_vslam"
    else:
        trajectory_source = "recorded" if trajectory else "none"

    trajectory = _deduplicate_trajectory(trajectory)
    _fill_trajectory_speeds(trajectory)
    if not speeds and trajectory:
        speeds = [
            {
                "_timestamp_ns": int(sample["_timestamp_ns"]),
                "_header_timestamp_ns": sample.get("_header_timestamp_ns"),
                "value": _finite(sample.get("speed_mps")),
                "source": f"{trajectory_source}_odometry",
            }
            for sample in trajectory
        ]

    all_groups = [
        frames,
        controls,
        comparison_controls,
        modes,
        sections,
        e2e_diagnostics,
        object_detections,
        imu_samples,
        speeds,
        trajectory,
        jetson_samples,
    ]
    timestamp_values = [
        int(sample["_timestamp_ns"])
        for group in all_groups
        for sample in group
        if "_timestamp_ns" in sample
    ]
    if not timestamp_values:
        raise RuntimeError("No timestamped analysis data was produced")
    start_ns, end_ns = min(timestamp_values), max(timestamp_values)
    for group in all_groups:
        _normalise_samples(group, start_ns)

    trajectory = _downsample(trajectory)
    controls = _downsample(controls)
    comparison_controls = _downsample(comparison_controls)
    modes = _downsample(modes)
    sections = _downsample(sections)
    e2e_diagnostics = _downsample(e2e_diagnostics)
    object_detections = _downsample(object_detections)
    imu_samples = _downsample(imu_samples)
    speeds = _downsample(speeds)
    jetson_series_by_id: dict[str, dict[str, object]] = {}
    for sample in jetson_samples:
        metric_id = str(sample.get("id") or "")
        if not metric_id:
            continue
        series = jetson_series_by_id.setdefault(
            metric_id,
            {
                "id": metric_id,
                "label": str(sample.get("label") or metric_id),
                "group": str(sample.get("group") or ""),
                "unit": str(sample.get("unit") or ""),
                "source": str(sample.get("source") or ""),
                "key": str(sample.get("key") or ""),
                "order": int(sample.get("order") or 99),
                "samples": [],
            },
        )
        series_samples = series["samples"]
        if isinstance(series_samples, list):
            series_samples.append({"t": sample["t"], "value": sample["value"]})
    jetson_series = []
    for series in jetson_series_by_id.values():
        series_samples = series.get("samples")
        if not isinstance(series_samples, list) or not series_samples:
            continue
        series_samples.sort(key=lambda item: float(item.get("t") or 0.0))
        values = [
            _finite(sample.get("value"))
            for sample in series_samples
            if math.isfinite(_finite(sample.get("value")))
        ]
        if not values:
            continue
        series["samples"] = _downsample(series_samples)
        series["min"] = round(min(values), 6)
        series["max"] = round(max(values), 6)
        jetson_series.append(series)
    jetson_series.sort(
        key=lambda item: (
            int(item.get("order") or 99),
            str(item.get("group") or ""),
            str(item.get("label") or ""),
        )
    )
    duration_s = max(0.0, (end_ns - start_ns) / 1e9)
    frame_id = str(trajectory[-1].get("frame_id") or "") if trajectory else ""
    geometry = _map_geometry(options.map_dir) if options.map_dir else None
    if options.map_dir and options.expected_map_fingerprint:
        current_map_fingerprint = _file_fingerprint(options.map_dir)
        if current_map_fingerprint != options.expected_map_fingerprint:
            raise RuntimeError(
                "The selected Map changed while analysis was running; the result was discarded."
            )
    consistency = trajectory_map_consistency(trajectory, geometry) if options.map_dir else {
        "status": "not_selected",
        "message": "Mapは選択されていません。",
        "inside_fraction": None,
        "overlay_ready": frame_id == "map",
    }
    map_payload: dict[str, object] = {
        "path": str(options.map_dir) if options.map_dir else "",
        "name": options.map_dir.name if options.map_dir else "",
        "fingerprint": _file_fingerprint(options.map_dir) if options.map_dir else "",
        "consistency": consistency,
    }
    if geometry:
        map_payload["raster"] = geometry

    warnings: list[str] = []
    if effective_max_fps + 1.0e-9 < options.max_fps:
        warnings.append(
            f"長時間bagの容量を抑えるため画像抽出を{effective_max_fps:.3g} fpsへ自動調整しました。"
        )
    if frame_limit_reached:
        warnings.append(
            f"画像は上限{MAX_EXTRACTED_FRAMES:,}枚まで抽出しました。より長い範囲には低いmax_fpsを指定してください。"
        )
    if not controls:
        warnings.append("Control topicがないためsteer/throttleは表示されません。")
    if image_decode_errors:
        warnings.append(
            f"破損または未対応encodingの画像 {image_decode_errors:,}件をskipしました: "
            + "; ".join(image_decode_error_examples)
        )
    if not modes:
        warnings.append("Operation mode topicがないためAUTO/MANUALは表示されません。")
    if not trajectory:
        warnings.append("自己位置topicまたはoffline localization結果がないため軌跡は表示されません。")
    if recorded_dropped_count:
        warnings.append(
            f"Map TF確立前のrecorded odometry {recorded_dropped_count:,}件を軌跡から除外しました。"
        )
    if consistency.get("status") in {"warning", "mismatch", "unknown"}:
        warnings.append(str(consistency.get("message") or "Map整合性を確認してください。"))
    if jetson_diagnostic_topics and not jetson_series:
        warnings.append("Jetson diagnostics topicはありますが、plot可能なjtop数値は抽出されませんでした。")
    if object_detection_topic and object_detections and not detection_overlay_frames:
        warnings.append("物体検出結果はありますが、画像と時刻同期できずoverlayを生成できませんでした。")
    if offline_localization_method in {"vslam_identity", "vslam_identity_fallback"}:
        warnings.append(
            "VGLを使わず、保存cuVSLAM Mapの原点をidentity初期姿勢として自己位置を生成しました。"
            "bag開始位置がMap原点付近であることを確認してください。"
        )

    progress.update(
        "package",
        task_progress(0.94),
        "同期timelineとMap整合性結果を保存しています。",
    )
    timeline = {
        "schema_version": SCHEMA_VERSION,
        "start_time_ns": str(start_ns),
        "end_time_ns": str(end_ns),
        "duration_s": round(duration_s, 6),
        "requested_max_fps": options.max_fps,
        "effective_max_fps": effective_max_fps,
        "frames": frames,
        "controls": controls,
        "comparison_controls": comparison_controls,
        "modes": modes,
        "sections": sections,
        "e2e_diagnostics": e2e_diagnostics,
        "object_detections": object_detections,
        "object_detection": {
            "source": detection_source,
            "topic": object_detection_topic,
            "metadata": detection_metadata,
        },
        "imu": imu_samples,
        "speeds": speeds,
        "trajectory": {
            "source": trajectory_source,
            "localization_method": offline_localization_method,
            "frame_id": frame_id,
            "map_transformed_samples": recorded_transformed_count,
            "prelocalization_samples_dropped": recorded_dropped_count,
            "samples": trajectory,
        },
        "jetson_stats": {
            "topics": jetson_diagnostic_topics,
            "series": jetson_series,
        },
        "map": map_payload,
    }
    _atomic_json(analysis_dir / "timeline.json", timeline)
    manifest_path = analysis_dir / "manifest.json"

    def build_manifest(previous: dict[str, object]) -> Mapping[str, object]:
        return {
            **previous,
            "schema_version": SCHEMA_VERSION,
            "id": analysis_dir.name,
            "analysis_id": str(previous.get("analysis_id") or analysis_dir.name),
            "status": "completed",
            "created_at": str(previous.get("created_at") or _utc_now()),
            "updated_at": _utc_now(),
            "rosbag": {
                "path": str(options.rosbag),
                "name": options.rosbag.name,
                "fingerprint": _file_fingerprint(options.rosbag / "metadata.yaml"),
            },
            "topics": {
                "image": options.image_topic,
                "image_topics": image_topics,
                "primary_image_topic": primary_image_topic,
                "control": options.control_topic,
                "comparison_control": options.comparison_control_topic,
                "mode": options.mode_topic,
                "section": options.section_topic,
                "e2e_diagnostics": options.e2e_diagnostic_topic,
                "object_detections": object_detection_topic,
                "object_detection_overlay": (
                    OBJECT_DETECTION_OVERLAY_TOPIC if detection_overlay_frames else ""
                ),
                "imu": options.imu_topic,
                "pose": options.pose_topic,
                "speed": options.speed_topic,
                "jetson_stats": jetson_diagnostic_topics,
            },
            "map": map_payload,
            "object_detection": {
                "source": detection_source,
                "topic": object_detection_topic,
                "sidecar_bag": str(options.detection_bag) if options.detection_bag else "",
                "metadata": detection_metadata,
            },
            "offline_localization": (
                {"method": offline_localization_method}
                if offline_snapshot_samples is not None
                else None
            ),
            "duration_s": round(duration_s, 6),
            "counts": {
                "frames": len(frames),
                "controls": len(controls),
                "comparison_controls": len(comparison_controls),
                "modes": len(modes),
                "sections": len(sections),
                "e2e_diagnostics": len(e2e_diagnostics),
                "object_detections": len(object_detections),
                "object_detection_overlay_frames": detection_overlay_frames,
                "imu": len(imu_samples),
                "speeds": len(speeds),
                "trajectory": len(trajectory),
                "jetson_stats": sum(
                    len(series.get("samples", []))
                    for series in jetson_series
                    if isinstance(series, dict)
                ),
            },
            "warnings": warnings,
            "timeline": "timeline.json",
        }

    manifest = _update_json_object(manifest_path, build_manifest)
    progress.update(
        "complete", 1.0, "分析用artifactの生成が完了しました。", status="completed"
    )
    return manifest


def write_demo_analysis(analysis_dir: Path) -> dict[str, object]:
    analysis_dir = analysis_dir.expanduser().resolve()
    frames_dir = analysis_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    frames: list[dict[str, object]] = []
    controls: list[dict[str, object]] = []
    modes: list[dict[str, object]] = []
    speeds: list[dict[str, object]] = []
    trajectory: list[dict[str, object]] = []
    duration_s = 20.0
    for index in range(100):
        t = index * duration_s / 99
        steering = 0.55 * math.sin(t * 0.8)
        throttle = 0.55 + 0.25 * math.sin(t * 0.35)
        speed = 2.4 + 1.1 * math.sin(t * 0.45)
        x = 12.0 * math.cos(t / duration_s * 2 * math.pi)
        y = 7.0 * math.sin(t / duration_s * 2 * math.pi)
        yaw = math.atan2(7.0 * math.cos(t / duration_s * 2 * math.pi), -12.0 * math.sin(t / duration_s * 2 * math.pi))
        frame_name = f"frame_{index:08d}.png"
        hue = int((index / 99) * 240)
        red = int(24 + 48 * (1.0 + math.sin(math.radians(hue))) / 2.0)
        green = int(38 + 80 * (1.0 + math.sin(math.radians(hue + 120))) / 2.0)
        blue = int(54 + 100 * (1.0 + math.sin(math.radians(hue + 240))) / 2.0)
        (frames_dir / frame_name).write_bytes(_solid_png(640, 360, (red, green, blue)))
        frames.append({"t": round(t, 6), "path": f"frames/{frame_name}", "width": 640, "height": 360})
        controls.append(
            {
                "t": round(t, 6),
                "steering": round(steering, 4),
                "throttle": round(throttle, 4),
                "brake": 0.0,
                "reverse": False,
            }
        )
        speeds.append({"t": round(t, 6), "value": round(speed, 4), "source": "demo_odometry"})
        trajectory.append(
            {
                "t": round(t, 6),
                "x": round(x, 5),
                "y": round(y, 5),
                "z": 0.0,
                "yaw": round(yaw, 5),
                "speed_mps": round(speed, 4),
                "frame_id": "map",
                "child_frame_id": "base_link",
            }
        )
    modes = [
        {"t": 0.0, "mode": 1, "label": "AUTO", "source": "demo"},
        {"t": 13.0, "mode": 2, "label": "MANUAL", "source": "demo"},
        {"t": 17.0, "mode": 1, "label": "AUTO", "source": "demo"},
    ]
    map_payload = {
        "path": "",
        "name": "Demo oval",
        "fingerprint": "demo",
        "consistency": {
            "status": "pass",
            "message": "Demo trajectory is aligned.",
            "inside_fraction": 1.0,
            "overlay_ready": True,
        },
    }
    timeline = {
        "schema_version": SCHEMA_VERSION,
        "start_time_ns": "0",
        "end_time_ns": str(int(duration_s * 1e9)),
        "duration_s": duration_s,
        "frames": frames,
        "controls": controls,
        "modes": modes,
        "speeds": speeds,
        "trajectory": {"source": "demo", "frame_id": "map", "samples": trajectory},
        "map": map_payload,
    }
    _atomic_json(analysis_dir / "timeline.json", timeline)
    def build_manifest(previous: dict[str, object]) -> Mapping[str, object]:
        return {
            **previous,
            "schema_version": SCHEMA_VERSION,
            "id": analysis_dir.name,
            "analysis_id": str(previous.get("analysis_id") or analysis_dir.name),
            "status": "completed",
            "created_at": str(previous.get("created_at") or _utc_now()),
            "updated_at": _utc_now(),
            "rosbag": {"path": "/demo/jetpilot.db3", "name": "Demo run", "fingerprint": "demo"},
            "topics": {
                "image": "/camera/image_raw",
                "image_topics": ["/camera/image_raw"],
                "primary_image_topic": "/camera/image_raw",
                "control": "/vehicle/control_cmd",
                "mode": "/operation_mode/state",
                "pose": "/visual_slam/tracking/odometry",
                "speed": "",
            },
            "map": map_payload,
            "duration_s": duration_s,
            "counts": {
                "frames": len(frames),
                "controls": len(controls),
                "modes": len(modes),
                "speeds": len(speeds),
                "trajectory": len(trajectory),
            },
            "warnings": [],
            "timeline": "timeline.json",
        }

    manifest = _update_json_object(analysis_dir / "manifest.json", build_manifest)
    Progress(analysis_dir / "status.json").update(
        "complete", 1.0, "Demo analysis is ready.", status="completed"
    )
    return manifest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Convert a ROS 2 bag into JetPilot analysis artifacts.")
    parser.add_argument("--rosbag", default="")
    parser.add_argument("--analysis-dir", required=True)
    parser.add_argument("--image-topic", default="")
    parser.add_argument("--image-topics", action="append", default=[])
    parser.add_argument("--primary-image-topic", default="")
    parser.add_argument("--control-topic", default="")
    parser.add_argument("--mode-topic", default="")
    parser.add_argument("--pose-topic", default="")
    parser.add_argument("--speed-topic", default="")
    parser.add_argument("--comparison-control-topic", default="")
    parser.add_argument("--section-topic", default="")
    parser.add_argument("--e2e-diagnostic-topic", default="")
    parser.add_argument("--detection-bag", default="")
    parser.add_argument("--detection-manifest", default="")
    parser.add_argument("--imu-topic", default="")
    parser.add_argument("--map-dir", default="")
    parser.add_argument("--trajectory-snapshot", default="")
    parser.add_argument("--status-file", default="")
    parser.add_argument("--max-fps", type=float, default=10.0)
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument("--expected-map-fingerprint", default="")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument(
        "--set-status",
        choices=("queued", "running", "completed", "failed", "stopped"),
        default="",
        help="Only update status.json; used by the task wrapper before extraction starts.",
    )
    parser.add_argument("--stage", default="")
    parser.add_argument("--progress", type=float, default=0.0)
    parser.add_argument("--message", default="")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    analysis_dir = Path(args.analysis_dir)
    status_path = (
        Path(args.status_file).expanduser().resolve()
        if args.status_file
        else analysis_dir / "status.json"
    )
    try:
        if args.set_status:
            Progress(status_path).update(
                args.stage or args.set_status,
                args.progress,
                args.message or args.set_status,
                status=args.set_status,
                preserve_existing_failed=args.set_status == "failed",
            )
        elif args.demo:
            write_demo_analysis(analysis_dir)
        else:
            if not args.rosbag:
                raise ValueError("--rosbag is required unless --demo is used")
            image_topics: list[str] = []
            for item in args.image_topics:
                for part in str(item).split(","):
                    val = part.strip()
                    if val and val not in image_topics:
                        image_topics.append(val)
            extract_analysis(
                AnalysisOptions(
                    rosbag=Path(args.rosbag),
                    analysis_dir=analysis_dir,
                    image_topic=args.image_topic,
                    image_topics=image_topics,
                    primary_image_topic=args.primary_image_topic,
                    control_topic=args.control_topic,
                    mode_topic=args.mode_topic,
                    pose_topic=args.pose_topic,
                    speed_topic=args.speed_topic,
                    comparison_control_topic=args.comparison_control_topic,
                    section_topic=args.section_topic,
                    e2e_diagnostic_topic=args.e2e_diagnostic_topic,
                    detection_bag=(
                        Path(args.detection_bag).expanduser().resolve()
                        if args.detection_bag
                        else None
                    ),
                    detection_manifest=(
                        Path(args.detection_manifest).expanduser().resolve()
                        if args.detection_manifest
                        else None
                    ),
                    imu_topic=args.imu_topic,
                    map_dir=Path(args.map_dir).expanduser().resolve() if args.map_dir else None,
                    trajectory_snapshot=(
                        Path(args.trajectory_snapshot).expanduser().resolve()
                        if args.trajectory_snapshot
                        else None
                    ),
                    status_file=(
                        Path(args.status_file).expanduser().resolve()
                        if args.status_file
                        else None
                    ),
                    max_fps=args.max_fps,
                    jpeg_quality=args.jpeg_quality,
                    expected_map_fingerprint=args.expected_map_fingerprint,
                )
            )
        return 0
    except Exception as error:  # noqa: BLE001 - task log must retain the full ROS failure.
        Progress(status_path).update(
            "failed", 1.0, str(error), status="failed"
        )
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
