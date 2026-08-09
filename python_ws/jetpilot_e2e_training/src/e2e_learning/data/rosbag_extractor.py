from __future__ import annotations

import bisect
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from tqdm import tqdm

from e2e_learning.utils.io import ensure_dir, write_csv, write_yaml


@dataclass(frozen=True)
class ExtractConfig:
    bag_path: Path
    output_dir: Path
    image_topic: str
    control_topic: str
    input_width: int
    input_height: int
    max_control_dt_sec: float
    task: str = "control"
    odometry_topic: str = "/visual_slam/tracking/odometry"
    imu_topic: str = "/sensors/imu"
    max_odometry_dt_sec: float = 0.15
    trajectory_horizon_sec: float = 1.5
    trajectory_points: int = 10
    trajectory_scale_m: float = 5.0
    imu_window_sec: float = 0.5
    imu_samples: int = 10
    image_extension: str = "jpg"
    jpeg_quality: int = 92


def stamp_to_ns(msg: Any, fallback_ns: int) -> int:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return int(fallback_ns)
    value = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
        getattr(stamp, "nanosec", 0)
    )
    return value if value > 0 else int(fallback_ns)


def control_from_msg(msg: Any) -> dict[str, float]:
    return {
        "steering": float(getattr(msg, "steering")),
        "throttle": float(getattr(msg, "throttle")),
    }


def _quaternion_yaw(orientation: Any) -> float:
    x = float(getattr(orientation, "x", 0.0))
    y = float(getattr(orientation, "y", 0.0))
    z = float(getattr(orientation, "z", 0.0))
    w = float(getattr(orientation, "w", 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def odometry_from_msg(msg: Any) -> dict[str, float]:
    pose = getattr(getattr(msg, "pose", None), "pose", None)
    if pose is None:
        pose = getattr(msg, "pose", None)
    if pose is None:
        raise ValueError("odometry message does not contain a pose")
    position = getattr(pose, "position")
    return {
        "x": float(getattr(position, "x")),
        "y": float(getattr(position, "y")),
        "yaw": _quaternion_yaw(getattr(pose, "orientation")),
    }


def imu_from_msg(msg: Any) -> list[float]:
    acceleration = getattr(msg, "linear_acceleration", None)
    angular = getattr(msg, "angular_velocity", None)
    if acceleration is None or angular is None:
        raise ValueError("IMU message is missing acceleration or angular velocity")
    # Scaling keeps both sensor groups near O(1). The final channel is a validity mask.
    return [
        float(getattr(acceleration, "x", 0.0)) / 9.80665,
        float(getattr(acceleration, "y", 0.0)) / 9.80665,
        float(getattr(acceleration, "z", 0.0)) / 9.80665,
        float(getattr(angular, "x", 0.0)) / 5.0,
        float(getattr(angular, "y", 0.0)) / 5.0,
        float(getattr(angular, "z", 0.0)) / 5.0,
        1.0,
    ]


def image_msg_to_bgr(msg: Any) -> np.ndarray:
    encoding = str(getattr(msg, "encoding", "")).lower()
    width = int(getattr(msg, "width"))
    height = int(getattr(msg, "height"))
    step = int(getattr(msg, "step"))
    data = np.frombuffer(bytes(getattr(msg, "data")), dtype=np.uint8)

    if encoding in ("rgb8", "bgr8"):
        channels = 3
        row_bytes = width * channels
        if step < row_bytes:
            raise ValueError(f"image step {step} is smaller than expected {row_bytes}")
        image = data.reshape(height, step)[:, :row_bytes].reshape(height, width, channels)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else image

    if encoding in ("mono8", "8uc1"):
        if step < width:
            raise ValueError(f"image step {step} is smaller than expected {width}")
        mono = data.reshape(height, step)[:, :width].reshape(height, width)
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    if encoding in ("mono16", "16uc1"):
        raw = np.frombuffer(bytes(getattr(msg, "data")), dtype=np.uint16)
        pixels_per_row = step // 2
        mono16 = raw.reshape(height, pixels_per_row)[:, :width].reshape(height, width)
        mono8 = cv2.convertScaleAbs(mono16, alpha=255.0 / max(float(mono16.max()), 1.0))
        return cv2.cvtColor(mono8, cv2.COLOR_GRAY2BGR)

    raise ValueError(f"Unsupported image encoding: {encoding}")


def compressed_image_msg_to_bgr(msg: Any) -> np.ndarray:
    data = np.frombuffer(bytes(getattr(msg, "data")), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("Failed to decode compressed image")
    return image


def decode_image(msg: Any, msgtype: str) -> np.ndarray:
    if msgtype.endswith("/CompressedImage"):
        return compressed_image_msg_to_bgr(msg)
    return image_msg_to_bgr(msg)


def _nearest(
    records: list[tuple[int, Any]], timestamps: list[int], target_ns: int, max_dt_sec: float
) -> tuple[int, Any] | None:
    if not records:
        return None
    index = bisect.bisect_left(timestamps, target_ns)
    candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(records)]
    if not candidates:
        return None
    selected = min(candidates, key=lambda candidate: abs(timestamps[candidate] - target_ns))
    record = records[selected]
    return record if abs(record[0] - target_ns) <= max_dt_sec * 1_000_000_000 else None


def _trajectory_label(
    odometry: list[tuple[int, dict[str, float]]],
    timestamps: list[int],
    stamp_ns: int,
    config: ExtractConfig,
) -> list[list[float]] | None:
    origin_record = _nearest(odometry, timestamps, stamp_ns, config.max_odometry_dt_sec)
    if origin_record is None:
        return None
    origin = origin_record[1]
    cosine = math.cos(origin["yaw"])
    sine = math.sin(origin["yaw"])
    points: list[list[float]] = []
    for step in range(1, config.trajectory_points + 1):
        offset_sec = config.trajectory_horizon_sec * step / config.trajectory_points
        target = _nearest(
            odometry,
            timestamps,
            stamp_ns + int(offset_sec * 1_000_000_000),
            config.max_odometry_dt_sec,
        )
        if target is None:
            return None
        dx = target[1]["x"] - origin["x"]
        dy = target[1]["y"] - origin["y"]
        points.append([cosine * dx + sine * dy, -sine * dx + cosine * dy])
    return points


def _imu_window(
    imu: list[tuple[int, list[float]]], timestamps: list[int], stamp_ns: int, config: ExtractConfig
) -> list[list[float]]:
    if config.imu_samples <= 0:
        return []
    values: list[list[float]] = []
    maximum_gap = max(0.02, config.imu_window_sec / max(config.imu_samples - 1, 1) * 1.5)
    for index in range(config.imu_samples):
        fraction = index / max(config.imu_samples - 1, 1)
        target_ns = stamp_ns - int(config.imu_window_sec * (1.0 - fraction) * 1_000_000_000)
        sample = _nearest(imu, timestamps, target_ns, maximum_gap)
        values.append(list(sample[1]) if sample is not None else [0.0] * 7)
    return values


def _collect_signals(
    config: ExtractConfig,
) -> tuple[
    list[tuple[int, dict[str, float]]],
    list[tuple[int, dict[str, float]]],
    list[tuple[int, list[float]]],
]:
    controls: list[tuple[int, dict[str, float]]] = []
    odometry: list[tuple[int, dict[str, float]]] = []
    imu: list[tuple[int, list[float]]] = []
    requested = {config.control_topic, config.odometry_topic, config.imu_topic}
    with AnyReader([config.bag_path]) as reader:
        connections = [connection for connection in reader.connections if connection.topic in requested]
        for connection, timestamp, rawdata in tqdm(
            reader.messages(connections=connections), desc="index signals"
        ):
            msg = reader.deserialize(rawdata, connection.msgtype)
            stamp_ns = stamp_to_ns(msg, timestamp)
            try:
                if connection.topic == config.control_topic:
                    controls.append((stamp_ns, control_from_msg(msg)))
                elif connection.topic == config.odometry_topic:
                    odometry.append((stamp_ns, odometry_from_msg(msg)))
                elif connection.topic == config.imu_topic:
                    imu.append((stamp_ns, imu_from_msg(msg)))
            except (AttributeError, TypeError, ValueError):
                continue
    controls.sort(key=lambda item: item[0])
    odometry.sort(key=lambda item: item[0])
    imu.sort(key=lambda item: item[0])
    return controls, odometry, imu


def extract_dataset(config: ExtractConfig) -> dict[str, Any]:
    if config.task not in {"control", "trajectory"}:
        raise ValueError("task must be control or trajectory")
    if config.trajectory_points < 2:
        raise ValueError("trajectory_points must be at least 2")
    if config.trajectory_horizon_sec <= 0.0 or config.trajectory_scale_m <= 0.0:
        raise ValueError("trajectory horizon and scale must be positive")

    image_dir = ensure_dir(config.output_dir / "images")
    controls, odometry, imu = _collect_signals(config)
    control_times = [item[0] for item in controls]
    odometry_times = [item[0] for item in odometry]
    imu_times = [item[0] for item in imu]
    if config.task == "control" and not controls:
        raise RuntimeError(f"No control messages were found on {config.control_topic}")
    if config.task == "trajectory" and not odometry:
        raise RuntimeError(f"No odometry messages were found on {config.odometry_topic}")

    rows: list[dict[str, str | float]] = []
    dropped_without_control = 0
    dropped_without_trajectory = 0
    failed_images = 0
    with AnyReader([config.bag_path]) as reader:
        image_connections = [
            connection for connection in reader.connections if connection.topic == config.image_topic
        ]
        if not image_connections:
            raise RuntimeError(f"Image topic was not found in the bag: {config.image_topic}")
        message_iter: Iterable = reader.messages(connections=image_connections)
        for connection, timestamp, rawdata in tqdm(message_iter, desc="extract images"):
            msg = reader.deserialize(rawdata, connection.msgtype)
            stamp_ns = stamp_to_ns(msg, timestamp)
            control = _nearest(controls, control_times, stamp_ns, config.max_control_dt_sec)
            trajectory = _trajectory_label(odometry, odometry_times, stamp_ns, config)
            if config.task == "control" and control is None:
                dropped_without_control += 1
                continue
            if config.task == "trajectory" and trajectory is None:
                dropped_without_trajectory += 1
                continue
            try:
                image = decode_image(msg, connection.msgtype)
            except Exception:
                failed_images += 1
                continue

            resized = cv2.resize(
                image,
                (config.input_width, config.input_height),
                interpolation=cv2.INTER_AREA,
            )
            rel_path = Path("images") / f"{len(rows):08d}.{config.image_extension}"
            params = []
            if config.image_extension.lower() in ("jpg", "jpeg"):
                params = [int(cv2.IMWRITE_JPEG_QUALITY), int(config.jpeg_quality)]
            if not cv2.imwrite(str(config.output_dir / rel_path), resized, params):
                failed_images += 1
                continue
            control_value = control[1] if control is not None else {}
            rows.append(
                {
                    "sequence_id": config.bag_path.name,
                    "image_path": rel_path.as_posix(),
                    "stamp": stamp_ns,
                    "control_stamp": control[0] if control is not None else "",
                    "control_dt_sec": (
                        f"{abs(stamp_ns - control[0]) / 1_000_000_000.0:.6f}"
                        if control is not None
                        else ""
                    ),
                    "steering": f"{float(control_value.get('steering', 0.0)):.8f}",
                    "throttle": f"{float(control_value.get('throttle', 0.0)):.8f}",
                    "trajectory": json.dumps(trajectory or [], separators=(",", ":")),
                    "imu": json.dumps(
                        _imu_window(imu, imu_times, stamp_ns, config), separators=(",", ":")
                    ),
                    "image_topic": config.image_topic,
                    "control_topic": config.control_topic,
                    "odometry_topic": config.odometry_topic,
                    "imu_topic": config.imu_topic,
                }
            )

    samples_path = config.output_dir / "samples.csv"
    fields = [
        "sequence_id",
        "image_path",
        "stamp",
        "control_stamp",
        "control_dt_sec",
        "steering",
        "throttle",
        "trajectory",
        "imu",
        "image_topic",
        "control_topic",
        "odometry_topic",
        "imu_topic",
    ]
    count = write_csv(samples_path, rows, fields)
    metadata = {
        "bag_path": str(config.bag_path),
        "task": config.task,
        "image_topic": config.image_topic,
        "control_topic": config.control_topic,
        "odometry_topic": config.odometry_topic,
        "imu_topic": config.imu_topic,
        "input_width": config.input_width,
        "input_height": config.input_height,
        "sample_count": count,
        "max_control_dt_sec": config.max_control_dt_sec,
        "max_odometry_dt_sec": config.max_odometry_dt_sec,
        "trajectory_horizon_sec": config.trajectory_horizon_sec,
        "trajectory_points": config.trajectory_points,
        "trajectory_scale_m": config.trajectory_scale_m,
        "trajectory_label_source": "future_odometry",
        "imu_window_sec": config.imu_window_sec,
        "imu_samples": config.imu_samples,
        "imu_features": ["accel_x_g", "accel_y_g", "accel_z_g", "gyro_x_5radps", "gyro_y_5radps", "gyro_z_5radps", "valid"],
        "dropped_without_control": dropped_without_control,
        "dropped_without_trajectory": dropped_without_trajectory,
        "failed_images": failed_images,
    }
    write_yaml(config.output_dir / "metadata.yaml", metadata)
    if math.isclose(count, 0):
        raise RuntimeError(f"No aligned samples were extracted to {samples_path}")
    return metadata
