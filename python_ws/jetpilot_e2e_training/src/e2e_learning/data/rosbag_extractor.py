from __future__ import annotations

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
    image_extension: str = "jpg"
    jpeg_quality: int = 92


def stamp_to_ns(msg: Any, fallback_ns: int) -> int:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return int(fallback_ns)
    return int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))


def control_from_msg(msg: Any) -> dict[str, float]:
    return {
        "steering": float(getattr(msg, "steering")),
        "throttle": float(getattr(msg, "throttle")),
    }


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
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

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


def extract_dataset(config: ExtractConfig) -> dict[str, Any]:
    image_dir = ensure_dir(config.output_dir / "images")
    rows: list[dict[str, str | float]] = []
    latest_control: tuple[int, dict[str, float]] | None = None
    dropped_without_control = 0
    dropped_stale_control = 0
    failed_images = 0

    with AnyReader([config.bag_path]) as reader:
        topic_connections = [
            c for c in reader.connections if c.topic in (config.image_topic, config.control_topic)
        ]
        if not topic_connections:
            raise RuntimeError("No configured image/control topics were found in the bag")

        message_iter: Iterable = reader.messages(connections=topic_connections)
        for connection, timestamp, rawdata in tqdm(message_iter, desc="extract"):
            msg = reader.deserialize(rawdata, connection.msgtype)
            stamp_ns = stamp_to_ns(msg, timestamp)

            if connection.topic == config.control_topic:
                latest_control = (stamp_ns, control_from_msg(msg))
                continue

            if connection.topic != config.image_topic:
                continue
            if latest_control is None:
                dropped_without_control += 1
                continue

            control_ns, control = latest_control
            dt_sec = abs(stamp_ns - control_ns) / 1_000_000_000.0
            if dt_sec > config.max_control_dt_sec:
                dropped_stale_control += 1
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
            cv2.imwrite(str(config.output_dir / rel_path), resized, params)
            rows.append(
                {
                    "image_path": rel_path.as_posix(),
                    "stamp": stamp_ns,
                    "control_stamp": control_ns,
                    "dt_sec": f"{dt_sec:.6f}",
                    "steering": f"{control['steering']:.8f}",
                    "throttle": f"{control['throttle']:.8f}",
                    "image_topic": config.image_topic,
                    "control_topic": config.control_topic,
                }
            )

    samples_path = config.output_dir / "samples.csv"
    count = write_csv(
        samples_path,
        rows,
        [
            "image_path",
            "stamp",
            "control_stamp",
            "dt_sec",
            "steering",
            "throttle",
            "image_topic",
            "control_topic",
        ],
    )
    metadata = {
        "bag_path": str(config.bag_path),
        "image_topic": config.image_topic,
        "control_topic": config.control_topic,
        "input_width": config.input_width,
        "input_height": config.input_height,
        "sample_count": count,
        "max_control_dt_sec": config.max_control_dt_sec,
        "dropped_without_control": dropped_without_control,
        "dropped_stale_control": dropped_stale_control,
        "failed_images": failed_images,
    }
    write_yaml(config.output_dir / "metadata.yaml", metadata)
    if math.isclose(count, 0):
        raise RuntimeError(f"No aligned samples were extracted to {samples_path}")
    return metadata
