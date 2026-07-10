#!/usr/bin/env python3
"""Diagnose bags used by isaac_mapping_ros rosbag_to_mapping_data."""

from __future__ import annotations

import argparse
import math
from collections import deque
from pathlib import Path
from typing import Any

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception as exc:
        raise RuntimeError(f"Could not parse YAML: {path}") from exc


def bag_uri(path: Path) -> Path:
    if path.is_file() and (path.parent / "metadata.yaml").exists():
        return path.parent
    return path


def storage_id(path: Path) -> str:
    path = bag_uri(path)
    if path.is_file():
        if path.suffix == ".mcap":
            return "mcap"
        if path.suffix == ".db3":
            return "sqlite3"
    metadata_path = path / "metadata.yaml"
    if metadata_path.exists():
        metadata = load_yaml(metadata_path)
        info = metadata.get("rosbag2_bagfile_information", {})
        value = info.get("storage_identifier")
        if value:
            return str(value)
    if list(path.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def open_reader(path: Path) -> tuple[SequentialReader, dict[str, str]]:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_uri(path)), storage_id=storage_id(path)),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    return reader, topics


def stamp_to_int(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def msg_stamp(msg: Any, fallback: int) -> int:
    header = getattr(msg, "header", None)
    if header is not None and hasattr(header, "stamp"):
        return stamp_to_int(header.stamp)
    return fallback


def read_selected_messages(
    bag_path: Path,
    wanted_topics: set[str],
    max_per_topic: int = 1,
    scan_all_topics: set[str] | None = None,
) -> tuple[dict[str, list[Any]], dict[str, tuple[int | None, int | None, int]]]:
    scan_all_topics = scan_all_topics or set()
    reader, topics = open_reader(bag_path)
    counts = {topic: 0 for topic in wanted_topics}
    messages: dict[str, list[Any]] = {topic: [] for topic in wanted_topics}
    ranges: dict[str, tuple[int | None, int | None, int]] = {
        topic: (None, None, 0) for topic in wanted_topics
    }

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in wanted_topics:
            continue
        msg_type = get_message(topics[topic])
        msg = deserialize_message(data, msg_type)
        first, last, count = ranges[topic]
        stamp = msg_stamp(msg, timestamp)
        ranges[topic] = (stamp if first is None else first, stamp, count + 1)
        if topic in scan_all_topics or counts[topic] < max_per_topic:
            messages[topic].append(msg)
            counts[topic] += 1
        if all(
            counts[t] >= max_per_topic or t in scan_all_topics
            for t in wanted_topics
        ):
            if not scan_all_topics:
                break

    return messages, ranges


def q_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    n = math.sqrt(sum(v * v for v in q))
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / n for v in q)  # type: ignore[return-value]


def q_mul(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_inv(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = q_normalize(q)
    return (-x, -y, -z, w)


def q_rotate(
    q: tuple[float, float, float, float],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    vq = (v[0], v[1], v[2], 0.0)
    out = q_mul(q_mul(q, vq), q_inv(q))
    return (out[0], out[1], out[2])


def tf_compose(a: tuple[tuple[float, float, float], tuple[float, float, float, float]],
               b: tuple[tuple[float, float, float], tuple[float, float, float, float]]
               ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    at, aq = a
    bt, bq = b
    rbt = q_rotate(aq, bt)
    return ((at[0] + rbt[0], at[1] + rbt[1], at[2] + rbt[2]), q_normalize(q_mul(aq, bq)))


def tf_inverse(tf: tuple[tuple[float, float, float], tuple[float, float, float, float]]
               ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    t, q = tf
    iq = q_inv(q)
    it = q_rotate(iq, (-t[0], -t[1], -t[2]))
    return (it, iq)


def transform_from_msg(msg: Any) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    tr = msg.transform.translation
    rot = msg.transform.rotation
    return ((float(tr.x), float(tr.y), float(tr.z)), q_normalize((float(rot.x), float(rot.y), float(rot.z), float(rot.w))))


def relative_transform(
    transforms: list[Any],
    source: str,
    target: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None:
    graph: dict[str, list[tuple[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]]]] = {}
    for stamped in transforms:
        parent = str(stamped.header.frame_id).strip()
        child = str(stamped.child_frame_id).strip()
        if not parent or not child:
            continue
        tf = transform_from_msg(stamped)
        graph.setdefault(parent, []).append((child, tf))
        graph.setdefault(child, []).append((parent, tf_inverse(tf)))

    if source == target:
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    queue = deque([(source, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))])
    visited = {source}
    while queue:
        frame, tf_source_frame = queue.popleft()
        for next_frame, tf_frame_next in graph.get(frame, []):
            if next_frame in visited:
                continue
            tf_source_next = tf_compose(tf_source_frame, tf_frame_next)
            if next_frame == target:
                return tf_source_next
            visited.add(next_frame)
            queue.append((next_frame, tf_source_next))
    return None


def rotation_angle_deg(q: tuple[float, float, float, float]) -> float:
    q = q_normalize(q)
    w = max(-1.0, min(1.0, abs(q[3])))
    return math.degrees(2.0 * math.acos(w))


class Reporter:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def info(self, text: str) -> None:
        print(f"[INFO] {text}")

    def ok(self, text: str) -> None:
        print(f"[OK] {text}")

    def warn(self, text: str) -> None:
        self.warnings.append(text)
        print(f"[WARN] {text}")


def load_stereo_cameras(path: Path) -> list[dict[str, str]]:
    data = load_yaml(path)
    cameras = data.get("stereo_cameras", [])
    if not isinstance(cameras, list):
        return []
    out = []
    for camera in cameras:
        if isinstance(camera, dict):
            out.append({str(k): str(v) for k, v in camera.items()})
    return out


def summarize_stamp(ns: int | None) -> str:
    if ns is None:
        return "none"
    return f"{ns / 1e9:.9f}"


def run(args: argparse.Namespace) -> int:
    reporter = Reporter()
    sensor_bag = Path(args.sensor_bag).expanduser().resolve()
    pose_bag = Path(args.pose_bag).expanduser().resolve()
    camera_config = Path(args.camera_topic_config).expanduser().resolve()

    reporter.info(f"Sensor bag: {sensor_bag}")
    reporter.info(f"Pose bag:   {pose_bag}")
    reporter.info(f"Camera config: {camera_config}")

    sensor_reader, sensor_topics = open_reader(sensor_bag)
    del sensor_reader
    pose_reader, pose_topics = open_reader(pose_bag)
    del pose_reader

    cameras = load_stereo_cameras(camera_config)
    if not cameras:
        reporter.warn("No stereo_cameras entries were found in camera topic config.")

    wanted_sensor_topics = {"/tf_static"}
    wanted_sensor_read_topics = {"/tf_static"}
    for camera in cameras:
        for key in ("left", "right", "left_camera_info", "right_camera_info"):
            topic = camera.get(key)
            if topic:
                wanted_sensor_topics.add(topic)
                if key.endswith("camera_info") or key in ("left", "right"):
                    wanted_sensor_read_topics.add(topic)

    for topic in sorted(wanted_sensor_topics):
        if topic in sensor_topics:
            reporter.ok(f"sensor topic exists: {topic} ({sensor_topics[topic]})")
        else:
            reporter.warn(f"sensor topic missing: {topic}")

    sensor_messages, _ = read_selected_messages(
        sensor_bag,
        {topic for topic in wanted_sensor_read_topics if topic in sensor_topics},
        max_per_topic=1,
        scan_all_topics={"/tf_static"} if "/tf_static" in sensor_topics else set(),
    )

    tf_messages = sensor_messages.get("/tf_static", [])
    static_transforms = [tf for msg in tf_messages for tf in getattr(msg, "transforms", [])]
    if static_transforms:
        reporter.ok(f"/tf_static contains {len(static_transforms)} static transform(s)")
    else:
        reporter.warn("/tf_static had no transforms")

    for camera in cameras:
        name = camera.get("name", "(unnamed)")
        left_info = camera.get("left_camera_info", "")
        right_info = camera.get("right_camera_info", "")
        left_image = camera.get("left", "")
        right_image = camera.get("right", "")
        left_msgs = sensor_messages.get(left_info, [])
        right_msgs = sensor_messages.get(right_info, [])
        left_image_msgs = sensor_messages.get(left_image, [])
        right_image_msgs = sensor_messages.get(right_image, [])
        left_frame = getattr(getattr(left_msgs[0], "header", None), "frame_id", "") if left_msgs else ""
        right_frame = getattr(getattr(right_msgs[0], "header", None), "frame_id", "") if right_msgs else ""
        left_image_frame = (
            getattr(getattr(left_image_msgs[0], "header", None), "frame_id", "") if left_image_msgs else ""
        )
        right_image_frame = (
            getattr(getattr(right_image_msgs[0], "header", None), "frame_id", "") if right_image_msgs else ""
        )

        reporter.info(f"{name}: left camera_info frame_id={left_frame or '(missing)'}")
        reporter.info(f"{name}: right camera_info frame_id={right_frame or '(missing)'}")
        reporter.info(f"{name}: left image frame_id={left_image_frame or '(missing)'}")
        reporter.info(f"{name}: right image frame_id={right_image_frame or '(missing)'}")

        if left_image_frame and left_frame and left_image_frame != left_frame:
            reporter.warn(f"{name}: left image frame_id differs from left camera_info frame_id")
        if right_image_frame and right_frame and right_image_frame != right_frame:
            reporter.warn(f"{name}: right image frame_id differs from right camera_info frame_id")

        if not left_frame or not right_frame:
            reporter.warn(f"{name}: camera_info frame_id could not be read for both cameras")
            continue
        if left_frame == right_frame:
            reporter.warn(f"{name}: left/right camera_info frame_id are identical: {left_frame}")
            continue

        rel = relative_transform(static_transforms, left_frame, right_frame)
        if rel is None:
            reporter.warn(f"{name}: no /tf_static path between {left_frame} and {right_frame}")
            continue

        t, q = rel
        baseline = math.sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2])
        angle = rotation_angle_deg(q)
        reporter.info(
            f"{name}: {left_frame} -> {right_frame} translation="
            f"[{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}] m, norm={baseline:.6f} m, rotation={angle:.3f} deg"
        )
        if baseline < args.min_stereo_baseline_m:
            reporter.warn(
                f"{name}: stereo baseline is too small ({baseline:.6f} m). "
                "This usually causes rosbag_to_mapping_data invalid transform warnings."
            )
        else:
            reporter.ok(f"{name}: stereo baseline looks non-zero")

    pose_topic = args.pose_topic
    if pose_topic not in pose_topics:
        reporter.warn(f"pose topic missing: {pose_topic}")
    else:
        reporter.ok(f"pose topic exists: {pose_topic} ({pose_topics[pose_topic]})")
        pose_messages, pose_ranges = read_selected_messages(
            pose_bag,
            {pose_topic},
            max_per_topic=1,
            scan_all_topics={pose_topic},
        )
        first, last, count = pose_ranges.get(pose_topic, (None, None, 0))
        reporter.info(
            f"pose timestamps: first={summarize_stamp(first)}, last={summarize_stamp(last)}, count={count}"
        )
        if pose_messages.get(pose_topic):
            msg = pose_messages[pose_topic][0]
            header = getattr(msg, "header", None)
            frame_id = str(getattr(header, "frame_id", "")) if header is not None else ""
            child_frame_id = str(getattr(msg, "child_frame_id", ""))
            reporter.info(f"pose frame_id={frame_id or '(missing)'}, child_frame_id={child_frame_id or '(none)'}")
            if child_frame_id and child_frame_id != args.base_link_name:
                reporter.warn(
                    f"pose child_frame_id is {child_frame_id}, expected {args.base_link_name}"
                )

    if reporter.warnings:
        reporter.warn(f"diagnostic completed with {len(reporter.warnings)} warning(s)")
        return 2 if args.strict else 0
    reporter.ok("diagnostic completed without warnings")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor-bag", required=True, help="Sensor rosbag directory or storage file")
    parser.add_argument("--pose-bag", required=True, help="Pose rosbag directory or storage file")
    parser.add_argument("--camera-topic-config", required=True, help="VGL camera topic config YAML")
    parser.add_argument("--pose-topic", default="/visual_slam/vis/slam_odometry")
    parser.add_argument("--base-link-name", default="base_link")
    parser.add_argument("--min-stereo-baseline-m", type=float, default=0.01)
    parser.add_argument("--strict", action="store_true", help="Return non-zero when warnings are found")
    return parser


def main() -> None:
    raise SystemExit(run(build_arg_parser().parse_args()))


if __name__ == "__main__":
    main()
