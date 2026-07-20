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
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator, Mapping, Sequence


SCHEMA_VERSION = 1
MAX_EXTRACTED_FRAMES = 50_000
MODE_LABELS = {1: "AUTO", 2: "MANUAL", 3: "STOP", 4: "PROPO"}


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


def _decode_image(message: Any):
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
            # compressed_depth_image_transport prepends a small configuration
            # header before the PNG payload. Locate a known image signature so
            # the same viewer can still render a depth topic when selected.
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
    if array.dtype == np.uint16:
        array = (array / 256).astype(np.uint8)
    elif array.dtype in {np.dtype(np.int16), np.dtype(np.float32)}:
        finite = np.isfinite(array)
        if not finite.any():
            array = np.zeros(array.shape, dtype=np.uint8)
        else:
            valid = array[finite]
            low, high = np.percentile(valid, (1.0, 99.0))
            if high <= low:
                array = np.zeros(array.shape, dtype=np.uint8)
            else:
                array = np.clip((array - low) * (255.0 / (high - low)), 0, 255)
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
        return cv2.cvtColor(array, bayer_codes[encoding])
    if encoding in {"yuv422", "yuv422_yuy2"}:
        return cv2.cvtColor(array, cv2.COLOR_YUV2BGR_YUY2)
    if encoding == "uyvy":
        return cv2.cvtColor(array, cv2.COLOR_YUV2BGR_UYVY)
    if encoding in {"rgb8", "rgb16"}:
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if encoding in {"rgba8", "rgba16"}:
        return cv2.cvtColor(array, cv2.COLOR_RGBA2BGR)
    if encoding in {"bgra8", "bgra16"}:
        return cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
    if channels == 1:
        return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
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
    image_topic: str
    control_topic: str = ""
    mode_topic: str = ""
    pose_topic: str = ""
    speed_topic: str = ""
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
            # The shell EXIT trap runs after the worker's exception handler.
            # Keep the worker's more specific failure message when it won that
            # race, while still performing the decision under the JSON lock.
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


def extract_analysis(options: AnalysisOptions) -> dict[str, object]:
    options.rosbag = options.rosbag.expanduser().resolve()
    options.analysis_dir = options.analysis_dir.expanduser().resolve()
    if not options.rosbag.is_dir():
        raise FileNotFoundError(f"ROS bag folder was not found: {options.rosbag}")
    if not options.image_topic:
        raise ValueError("--image-topic is required")
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
    progress = Progress(
        options.status_file.expanduser().resolve()
        if options.status_file
        else analysis_dir / "status.json"
    )
    progress_floor = 0.42 if options.trajectory_snapshot else 0.0

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
    if options.image_topic not in topic_types:
        raise RuntimeError(f"Image topic was not found: {options.image_topic}")
    requested = {
        topic
        for topic in (
            options.image_topic,
            options.control_topic,
            options.mode_topic,
            options.pose_topic,
            options.speed_topic,
        )
        if topic
    }
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
    modes: list[dict[str, object]] = []
    speeds: list[dict[str, object]] = []
    trajectory: list[dict[str, object]] = []
    recorded_map_transforms: list[dict[str, object]] = []
    timestamps: list[int] = []
    last_frame_timestamp_ns: int | None = None
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
        timestamps.append(timestamp_ns)
        header_timestamp_ns = _stamp_ns(message)
        common = {
            "_timestamp_ns": timestamp_ns,
            "_header_timestamp_ns": header_timestamp_ns,
        }

        if topic == options.image_topic:
            if len(frames) >= MAX_EXTRACTED_FRAMES:
                frame_limit_reached = True
                continue
            if (
                last_frame_timestamp_ns is not None
                and timestamp_ns - last_frame_timestamp_ns < min_frame_interval_ns
            ):
                continue
            try:
                image = _decode_image(message)
                filename = f"frame_{len(frames):08d}.jpg"
                width, height = _write_jpeg(
                    frames_dir / filename, image, options.jpeg_quality
                )
            except Exception as exc:  # noqa: BLE001 - one corrupt frame must not abort a run.
                image_decode_errors += 1
                if len(image_decode_error_examples) < 3:
                    image_decode_error_examples.append(str(exc))
                continue
            frames.append(
                {
                    **common,
                    "path": f"frames/{filename}",
                    "width": width,
                    "height": height,
                }
            )
            last_frame_timestamp_ns = timestamp_ns
        if options.control_topic and topic == options.control_topic:
            controls.append({**common, **_control_payload(message)})
        if options.mode_topic and topic == options.mode_topic:
            modes.append({**common, **_mode_payload(message)})
        if options.speed_topic and topic == options.speed_topic:
            value = _explicit_speed(message)
            if value is not None:
                speeds.append({**common, "value": value, "source": options.speed_topic})
        if options.pose_topic and topic == options.pose_topic:
            for sample in _pose_samples(message, timestamp_ns):
                trajectory.append(sample)

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

    if not frames:
        detail = f" Last decoder error: {image_decode_error_examples[-1]}" if image_decode_error_examples else ""
        raise RuntimeError(
            f"No decodable image messages were found on {options.image_topic}.{detail}"
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

    all_groups = [frames, controls, modes, speeds, trajectory]
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
    modes = _downsample(modes)
    speeds = _downsample(speeds)
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
        "modes": modes,
        "speeds": speeds,
        "trajectory": {
            "source": trajectory_source,
            "localization_method": offline_localization_method,
            "frame_id": frame_id,
            "map_transformed_samples": recorded_transformed_count,
            "prelocalization_samples_dropped": recorded_dropped_count,
            "samples": trajectory,
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
                "control": options.control_topic,
                "mode": options.mode_topic,
                "pose": options.pose_topic,
                "speed": options.speed_topic,
            },
            "map": map_payload,
            "offline_localization": (
                {"method": offline_localization_method}
                if offline_snapshot_samples is not None
                else None
            ),
            "duration_s": round(duration_s, 6),
            "counts": {
                "frames": len(frames),
                "controls": len(controls),
                "modes": len(modes),
                "speeds": len(speeds),
                "trajectory": len(trajectory),
            "read_messages": read_count,
            "image_decode_errors": image_decode_errors,
        },
            "warnings": warnings,
            "timeline": "timeline.json",
        }

    manifest = _update_json_object(manifest_path, build_manifest)
    progress.update("complete", 1.0, "解析用データの準備が完了しました。", status="completed")
    return manifest


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))


def _solid_png(width: int, height: int, color: tuple[int, int, int]) -> bytes:
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    row = bytes(color) * width
    pixels = b"".join(b"\x00" + row for _ in range(height))
    return (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", zlib.compress(pixels, 6))
        + _png_chunk(b"IEND", b"")
    )


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
    parser.add_argument("--control-topic", default="")
    parser.add_argument("--mode-topic", default="")
    parser.add_argument("--pose-topic", default="")
    parser.add_argument("--speed-topic", default="")
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
            extract_analysis(
                AnalysisOptions(
                    rosbag=Path(args.rosbag),
                    analysis_dir=analysis_dir,
                    image_topic=args.image_topic,
                    control_topic=args.control_topic,
                    mode_topic=args.mode_topic,
                    pose_topic=args.pose_topic,
                    speed_topic=args.speed_topic,
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
