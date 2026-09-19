#!/usr/bin/env python3
"""Export RGB/EVS image pairs from a JetPilot benchmark session.

RGB is read from rosbag2. OpenEB RAW is first converted to the repository's
monotonic EVSBIN format. The RAW recording START request in the bag is used as
the common wall-clock anchor; EVS timestamps remain relative sensor time.
"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import struct
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator, Sequence


EVSBIN_HEADER = struct.Struct("<8sIIIIQQ24s")
EVSBIN_MAGIC = b"EVSBENCH"


@dataclass(frozen=True)
class EvbinHeader:
    header_bytes: int
    width: int
    height: int
    event_count: int
    timestamp_unit_ns: int


@dataclass(frozen=True)
class RgbRecord:
    index: int
    bag_timestamp_ns: int
    header_timestamp_ns: int
    motion_score: float | None = None


@dataclass(frozen=True)
class RawRequestRecord:
    bag_timestamp_ns: int
    header_timestamp_ns: int
    command: int
    label: str


def read_evbin_header(path: Path) -> EvbinHeader:
    with path.open("rb") as stream:
        payload = stream.read(EVSBIN_HEADER.size)
    if len(payload) != EVSBIN_HEADER.size:
        raise ValueError(f"EVSBIN header is truncated: {path}")
    magic, version, header_bytes, width, height, count, unit_ns, _ = (
        EVSBIN_HEADER.unpack(payload)
    )
    if magic != EVSBIN_MAGIC or version != 1 or header_bytes != EVSBIN_HEADER.size:
        raise ValueError(f"Unsupported EVSBIN file: {path}")
    if width <= 0 or height <= 0 or unit_ns != 1000:
        raise ValueError(f"Invalid EVSBIN geometry or timestamp unit: {path}")
    expected_bytes = header_bytes + count * 16
    if path.stat().st_size < expected_bytes:
        raise ValueError(
            f"EVSBIN payload is truncated: expected at least {expected_bytes} bytes"
        )
    return EvbinHeader(header_bytes, width, height, count, unit_ns)


def map_bag_to_sensor_us(
    bag_timestamp_ns: int,
    raw_start_bag_ns: int,
    first_event_us: int,
    offset_ms: float = 0.0,
) -> int:
    """Map a rosbag receive timestamp to the RAW sensor-time domain."""
    elapsed_us = round((bag_timestamp_ns - raw_start_bag_ns) / 1000.0)
    return first_event_us + elapsed_us + round(offset_ms * 1000.0)


def _stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class BagBackend:
    """Small compatibility layer for standalone rosbags or sourced ROS 2."""

    def __init__(self, mode: str, api: tuple[Any, ...]):
        self.mode = mode
        self.api = api

    def available_topics(self, session: Path) -> set[str]:
        if self.mode == "rosbags":
            (any_reader,) = self.api
            with any_reader([session]) as reader:
                return {connection.topic for connection in reader.connections}
        sequential_reader, storage_options, converter_options, _, _ = self.api
        reader = sequential_reader()
        reader.open(
            storage_options(uri=str(session), storage_id=_storage_id(session)),
            converter_options(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        return {item.name for item in reader.get_all_topics_and_types()}

    def messages(
        self,
        session: Path,
        topics: set[str],
        deserialize_topics: set[str],
    ) -> Iterator[tuple[str, int, Any | None]]:
        if self.mode == "rosbags":
            (any_reader,) = self.api
            with any_reader([session]) as reader:
                connections = [
                    connection
                    for connection in reader.connections
                    if connection.topic in topics
                ]
                for connection, timestamp_ns, rawdata in reader.messages(
                    connections=connections
                ):
                    message = (
                        reader.deserialize(rawdata, connection.msgtype)
                        if connection.topic in deserialize_topics
                        else None
                    )
                    yield connection.topic, int(timestamp_ns), message
            return

        (
            sequential_reader,
            storage_options,
            converter_options,
            deserialize_message,
            get_message,
        ) = self.api
        reader = sequential_reader()
        reader.open(
            storage_options(uri=str(session), storage_id=_storage_id(session)),
            converter_options(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        message_types = {
            topic: get_message(topic_types[topic])
            for topic in deserialize_topics
            if topic in topic_types
        }
        while reader.has_next():
            topic, rawdata, timestamp_ns = reader.read_next()
            if topic not in topics:
                continue
            message = (
                deserialize_message(rawdata, message_types[topic])
                if topic in deserialize_topics
                else None
            )
            yield topic, int(timestamp_ns), message


def _storage_id(session: Path) -> str:
    if list(session.glob("*.mcap")):
        return "mcap"
    if list(session.glob("*.db3")):
        return "sqlite3"
    raise RuntimeError(f"No MCAP or SQLite rosbag storage file was found in {session}")


def _load_runtime_dependencies() -> tuple[Any, Any, BagBackend]:
    missing: list[str] = []
    try:
        import numpy as np
    except ImportError:
        np = None
        missing.append("numpy")
    try:
        import cv2
    except ImportError:
        cv2 = None
        missing.append("cv2/python3-opencv")
    if missing:
        raise RuntimeError(
            "Missing Python runtime module(s): "
            + ", ".join(missing)
            + ". Run this in the JetPilot container with system /usr/bin/python3."
        )

    try:
        from rosbags.highlevel import AnyReader

        return np, cv2, BagBackend("rosbags", (AnyReader,))
    except ImportError:
        pass
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message

        return np, cv2, BagBackend(
            "ros2",
            (
                SequentialReader,
                StorageOptions,
                ConverterOptions,
                deserialize_message,
                get_message,
            ),
        )
    except ImportError as error:
        raise RuntimeError(
            "No rosbag reader is available. Tried standalone 'rosbags' and the ROS 2 "
            "modules rosbag2_py/rclpy. Source /opt/ros/jazzy/setup.bash and the "
            "JetPilot workspace, then use /usr/bin/python3."
        ) from error


def _image_to_bgr(message: Any, np: Any, cv2: Any) -> Any:
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    encoding = str(message.encoding).lower()
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size != height * step:
        raise ValueError(
            f"Image payload size {raw.size} does not match height*step {height * step}"
        )
    rows = raw.reshape(height, step)
    if encoding in {"rgb8", "bgr8"}:
        image = rows[:, : width * 3].reshape(height, width, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else image.copy()
    if encoding in {"rgba8", "bgra8"}:
        image = rows[:, : width * 4].reshape(height, width, 4)
        code = cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(image, code)
    if encoding in {"mono8", "8uc1"}:
        image = rows[:, :width].reshape(height, width)
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if encoding in {"yuyv", "yuy2", "yuv422_yuy2"}:
        image = rows[:, : width * 2].reshape(height, width, 2)
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
    raise ValueError(f"Unsupported RGB image encoding: {message.encoding}")


def _find_raw(session: Path, explicit: Path | None) -> Path:
    if explicit is not None:
        if not explicit.is_file():
            raise FileNotFoundError(f"RAW file not found: {explicit}")
        return explicit
    candidates = sorted(session.glob("*.raw"))
    if len(candidates) != 1:
        raise RuntimeError(
            f"Expected exactly one *.raw in {session}, found {len(candidates)}. "
            "Select one with --raw (split RAW sessions require one export per segment)."
        )
    return candidates[0]


def _find_converter(explicit: Path | None) -> Path | None:
    if explicit is not None:
        if not explicit.is_file():
            raise FileNotFoundError(f"RAW converter not found: {explicit}")
        return explicit
    on_path = shutil.which("evs_raw_to_evbin")
    if on_path:
        return Path(on_path)
    tool_root = Path(__file__).resolve().parents[1]
    candidates = (
        tool_root / "build/evs_raw_to_evbin",
        tool_root / "build/Release/evs_raw_to_evbin",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def _convert_raw_with_metavision(raw_path: Path, evbin_path: Path, np: Any) -> None:
    try:
        from metavision_core.event_io import EventsIterator
    except ImportError as error:
        raise RuntimeError(
            "evs_raw_to_evbin and the Metavision Python binding are both unavailable. "
            "Build tools/evs_benchmark, or run with /usr/bin/python3 in the "
            "SilkyEvCam-enabled JetPilot container."
        ) from error

    iterator = EventsIterator(
        input_path=str(raw_path), delta_t=10_000, relative_timestamps=False
    )
    height, width = iterator.get_size()
    if width <= 0 or height <= 0:
        raise RuntimeError(f"Metavision reported invalid RAW geometry: {width}x{height}")
    dtype = np.dtype(
        [
            ("t", "<i8"),
            ("x", "<u2"),
            ("y", "<u2"),
            ("p", "u1"),
            ("padding", "u1", (3,)),
        ]
    )
    event_count = 0
    dropped_nonmonotonic = 0
    last_timestamp_us: int | None = None
    with evbin_path.open("wb") as stream:
        stream.write(
            EVSBIN_HEADER.pack(
                EVSBIN_MAGIC, 1, EVSBIN_HEADER.size, width, height, 0, 1000, b"\0" * 24
            )
        )
        for decoded in iterator:
            if len(decoded) == 0:
                continue
            timestamps = np.asarray(decoded["t"], dtype=np.int64)
            order = np.argsort(timestamps, kind="stable")
            decoded = decoded[order]
            timestamps = timestamps[order]
            if last_timestamp_us is not None:
                keep = timestamps >= last_timestamp_us
                dropped_nonmonotonic += int((~keep).sum())
                decoded = decoded[keep]
                timestamps = timestamps[keep]
            if len(decoded) == 0:
                continue
            converted = np.zeros(len(decoded), dtype=dtype)
            converted["t"] = timestamps
            converted["x"] = decoded["x"]
            converted["y"] = decoded["y"]
            converted["p"] = decoded["p"]
            stream.write(converted.tobytes())
            event_count += len(converted)
            last_timestamp_us = int(timestamps[-1])
        stream.seek(0)
        stream.write(
            EVSBIN_HEADER.pack(
                EVSBIN_MAGIC,
                1,
                EVSBIN_HEADER.size,
                width,
                height,
                event_count,
                1000,
                b"\0" * 24,
            )
        )
    if event_count == 0:
        evbin_path.unlink(missing_ok=True)
        raise RuntimeError(
            "Metavision decoded zero events from the RAW file. The incomplete EVSBIN was "
            "removed. Check that the RAW file is non-empty, then build and pass the native "
            "evs_raw_to_evbin converter."
        )
    stats_path = evbin_path.with_suffix(evbin_path.suffix + ".conversion.json")
    stats_path.write_text(
        json.dumps(
            {
                "input": str(raw_path),
                "output": str(evbin_path),
                "backend": "metavision_python",
                "width": width,
                "height": height,
                "written_events": event_count,
                "dropped_nonmonotonic_events": dropped_nonmonotonic,
                "timestamp_policy": "stable_sort_each_10ms_slice_then_drop_backward",
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def _convert_raw(
    raw_path: Path, evbin_path: Path, converter: Path | None, np: Any
) -> None:
    print(f"[1/4] Converting RAW to monotonic EVSBIN: {raw_path.name}")
    if converter is None:
        print("      evs_raw_to_evbin not found; using Metavision Python fallback")
        _convert_raw_with_metavision(raw_path, evbin_path, np)
        return
    stats_path = evbin_path.with_suffix(evbin_path.suffix + ".conversion.json")
    callbacks_path = evbin_path.with_suffix(evbin_path.suffix + ".callbacks.csv")
    command = [
        str(converter),
        str(raw_path),
        str(evbin_path),
        str(callbacks_path),
        "--timestamp-policy",
        "reorder",
        "--reorder-window-us",
        "10000",
        "--stats",
        str(stats_path),
    ]
    subprocess.run(command, check=True)


def _event_memmap(evbin_path: Path, header: EvbinHeader, np: Any) -> Any:
    dtype = np.dtype(
        [
            ("t", "<i8"),
            ("x", "<u2"),
            ("y", "<u2"),
            ("p", "u1"),
            ("padding", "u1", (3,)),
        ]
    )
    return np.memmap(
        evbin_path,
        dtype=dtype,
        mode="r",
        offset=header.header_bytes,
        shape=(header.event_count,),
    )


def _timestamps_are_monotonic(times: Any, np: Any, chunk_size: int = 4_000_000) -> bool:
    if len(times) < 2:
        return True
    previous = int(times[0])
    for start in range(1, len(times), chunk_size):
        chunk = times[start : min(len(times), start + chunk_size)]
        if int(chunk[0]) < previous or bool(np.any(chunk[1:] < chunk[:-1])):
            return False
        previous = int(chunk[-1])
    return True


def _scan_bag(
    session: Path,
    rgb_topic: str,
    request_topic: str,
    need_motion: bool,
    np: Any,
    cv2: Any,
    bag_backend: BagBackend,
) -> tuple[list[RawRequestRecord], list[RgbRecord]]:
    records: list[RgbRecord] = []
    requests: list[RawRequestRecord] = []
    previous_gray = None
    available = bag_backend.available_topics(session)
    if rgb_topic not in available:
        raise RuntimeError(f"RGB topic {rgb_topic!r} not found; available: {sorted(available)}")
    for topic, timestamp_ns, message in bag_backend.messages(
        session,
        {rgb_topic, request_topic},
        {rgb_topic, request_topic},
    ):
        if topic == request_topic:
            assert message is not None
            requests.append(
                RawRequestRecord(
                    bag_timestamp_ns=int(timestamp_ns),
                    header_timestamp_ns=_stamp_ns(message),
                    command=int(message.command),
                    label=str(message.label),
                )
            )
            continue
        assert message is not None
        motion_score = None
        if need_motion:
            bgr = _image_to_bgr(message, np, cv2)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            gray = cv2.resize(gray, (96, 72), interpolation=cv2.INTER_AREA)
            if previous_gray is not None:
                delta = cv2.absdiff(gray, previous_gray)
                motion_score = float(delta.mean())
            previous_gray = gray
        records.append(
            RgbRecord(
                index=len(records),
                bag_timestamp_ns=int(timestamp_ns),
                header_timestamp_ns=_stamp_ns(message),
                motion_score=motion_score,
            )
        )
    return requests, records


def _choose_raw_start_anchor(
    requests: Sequence[RawRequestRecord],
    first_event_us: int,
    last_event_us: int,
    explicit_anchor_ns: int | None,
) -> tuple[int, str]:
    if explicit_anchor_ns is not None:
        return explicit_anchor_ns, "explicit_raw_start_bag_ns"
    starts = [request for request in requests if request.command == 1]
    if starts:
        return starts[0].bag_timestamp_ns, "recorded_start_request"
    stops = [request for request in requests if request.command == 2]
    if stops:
        raw_duration_ns = max(0, last_event_us - first_event_us) * 1000
        return stops[-1].bag_timestamp_ns - raw_duration_ns, "inferred_from_stop_minus_raw_duration"
    raise RuntimeError(
        "Neither RAW START nor STOP request was found in the bag. "
        "Pass --raw-start-bag-ns explicitly."
    )


def _median_interval_ns(records: Sequence[RgbRecord]) -> int:
    if len(records) < 2:
        raise RuntimeError("At least two RGB frames are required")
    intervals = sorted(
        later.bag_timestamp_ns - earlier.bag_timestamp_ns
        for earlier, later in zip(records, records[1:])
        if later.bag_timestamp_ns > earlier.bag_timestamp_ns
    )
    if not intervals:
        raise RuntimeError("RGB timestamps are not increasing")
    return intervals[len(intervals) // 2]


def _event_count(times: Any, start_us: int, end_us: int, np: Any) -> int:
    begin = int(np.searchsorted(times, start_us, side="left"))
    end = int(np.searchsorted(times, end_us, side="right"))
    return max(0, end - begin)


def _estimate_offset_ms(
    records: Sequence[RgbRecord],
    times: Any,
    anchor_ns: int,
    first_event_us: int,
    search_ms: float,
    step_ms: float,
    base_offset_ms: float,
    np: Any,
) -> tuple[float, float, int]:
    usable = [record for record in records if record.motion_score is not None]
    if len(usable) < 10:
        raise RuntimeError("Auto alignment requires at least 10 RGB frame intervals")
    motion = np.asarray([record.motion_score for record in usable], dtype=np.float64)
    offsets = np.arange(-search_ms, search_ms + step_ms * 0.5, step_ms)
    best_offset = 0.0
    best_score = float("-inf")
    for offset in offsets:
        candidate_offset_ms = base_offset_ms + float(offset)
        activity = []
        for record in usable:
            previous = records[record.index - 1]
            start_us = map_bag_to_sensor_us(
                previous.bag_timestamp_ns,
                anchor_ns,
                first_event_us,
                candidate_offset_ms,
            )
            end_us = map_bag_to_sensor_us(
                record.bag_timestamp_ns,
                anchor_ns,
                first_event_us,
                candidate_offset_ms,
            )
            activity.append(_event_count(times, start_us, end_us, np))
        event_activity = np.log1p(np.asarray(activity, dtype=np.float64))
        if float(motion.std()) < 1.0e-9 or float(event_activity.std()) < 1.0e-9:
            continue
        score = float(np.corrcoef(motion, event_activity)[0, 1])
        if score > best_score:
            best_score = score
            best_offset = float(offset)
    if best_score == float("-inf"):
        raise RuntimeError("Auto alignment could not compute a usable correlation")
    return best_offset, best_score, len(usable)


def _render_event_image(
    events: Any,
    width: int,
    height: int,
    percentile: float,
    np: Any,
) -> Any:
    on = np.zeros((height, width), dtype=np.uint32)
    off = np.zeros((height, width), dtype=np.uint32)
    if len(events):
        valid = (events["x"] < width) & (events["y"] < height)
        selected = events[valid]
        on_events = selected[selected["p"] != 0]
        off_events = selected[selected["p"] == 0]
        np.add.at(on, (on_events["y"], on_events["x"]), 1)
        np.add.at(off, (off_events["y"], off_events["x"]), 1)
    total = on + off
    active = total[total > 0]
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    if active.size == 0:
        return image
    scale = max(1.0, float(np.percentile(active, percentile)))
    alpha = np.sqrt(np.clip(total.astype(np.float32) / scale, 0.0, 1.0))
    denominator = np.maximum(total, 1).astype(np.float32)
    on_fraction = on.astype(np.float32) / denominator
    off_fraction = off.astype(np.float32) / denominator
    # BGR palette: ON=blue, OFF=red, simultaneous polarity=magenta.
    image[:, :, 0] = np.rint(255.0 * (1.0 - alpha * off_fraction)).astype(np.uint8)
    image[:, :, 1] = np.rint(255.0 * (1.0 - alpha)).astype(np.uint8)
    image[:, :, 2] = np.rint(255.0 * (1.0 - alpha * on_fraction)).astype(np.uint8)
    return image


def _paired_image(rgb: Any, event: Any, cv2: Any) -> Any:
    if event.shape[0] != rgb.shape[0]:
        target_width = max(1, round(event.shape[1] * rgb.shape[0] / event.shape[0]))
        event = cv2.resize(event, (target_width, rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
    return cv2.hconcat([rgb, event])


def _write_image(path: Path, image: Any, cv2: Any) -> None:
    if not cv2.imwrite(str(path), image, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
        raise RuntimeError(f"Failed to write image: {path}")


def _selected_indices(
    records: Sequence[RgbRecord],
    anchor_ns: int,
    start_sec: float,
    duration_sec: float,
    every: int,
    max_frames: int,
) -> set[int]:
    start_ns = anchor_ns + round(start_sec * 1.0e9)
    end_ns = None if duration_sec <= 0 else start_ns + round(duration_sec * 1.0e9)
    eligible = [
        record.index
        for record in records
        if record.bag_timestamp_ns >= start_ns
        and (end_ns is None or record.bag_timestamp_ns < end_ns)
    ][::every]
    if max_frames > 0:
        eligible = eligible[:max_frames]
    return set(eligible)


def _export_frames(
    session: Path,
    output_dir: Path,
    rgb_topic: str,
    records: Sequence[RgbRecord],
    selected: set[int],
    events: Any,
    header: EvbinHeader,
    anchor_ns: int,
    first_event_us: int,
    offset_ms: float,
    window_ms: float | None,
    median_interval_ns: int,
    percentile: float,
    np: Any,
    cv2: Any,
    bag_backend: BagBackend,
) -> list[dict[str, Any]]:
    rgb_dir = output_dir / "rgb"
    event_dir = output_dir / "event"
    pair_dir = output_dir / "pair"
    for directory in (rgb_dir, event_dir, pair_dir):
        directory.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    frame_index = 0
    for _, timestamp_ns, message in bag_backend.messages(
        session, {rgb_topic}, {rgb_topic}
    ):
        if frame_index not in selected:
            frame_index += 1
            continue
        assert message is not None
        rgb = _image_to_bgr(message, np, cv2)
        end_us = map_bag_to_sensor_us(
            int(timestamp_ns), anchor_ns, first_event_us, offset_ms
        )
        if window_ms is not None:
            start_us = end_us - round(window_ms * 1000.0)
        elif frame_index > 0:
            start_us = map_bag_to_sensor_us(
                records[frame_index - 1].bag_timestamp_ns,
                anchor_ns,
                first_event_us,
                offset_ms,
            )
        else:
            start_us = end_us - round(median_interval_ns / 1000.0)
        begin = int(np.searchsorted(events["t"], start_us, side="left"))
        end = int(np.searchsorted(events["t"], end_us, side="right"))
        event_image = _render_event_image(
            events[begin:end], header.width, header.height, percentile, np
        )
        relative_sec = (int(timestamp_ns) - anchor_ns) / 1.0e9
        stem = f"{frame_index:06d}_t{relative_sec:010.6f}"
        rgb_rel = Path("rgb") / f"{stem}.png"
        event_rel = Path("event") / f"{stem}.png"
        pair_rel = Path("pair") / f"{stem}.png"
        _write_image(output_dir / rgb_rel, rgb, cv2)
        _write_image(output_dir / event_rel, event_image, cv2)
        _write_image(output_dir / pair_rel, _paired_image(rgb, event_image, cv2), cv2)
        rows.append(
            {
                "rgb_index": frame_index,
                "rgb_bag_timestamp_ns": int(timestamp_ns),
                "rgb_header_timestamp_ns": _stamp_ns(message),
                "relative_time_s": f"{relative_sec:.9f}",
                "event_window_start_sensor_us": start_us,
                "event_window_end_sensor_us": end_us,
                "event_window_ms": f"{(end_us - start_us) / 1000.0:.6f}",
                "event_count": max(0, end - begin),
                "sync_offset_ms": f"{offset_ms:.6f}",
                "rgb_path": rgb_rel.as_posix(),
                "event_path": event_rel.as_posix(),
                "pair_path": pair_rel.as_posix(),
            }
        )
        frame_index += 1
    return rows


def _write_manifest(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    rows = list(rows)
    if not rows:
        raise RuntimeError("No RGB frames matched the requested export range")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export publication-ready RGB and synchronized event images"
    )
    parser.add_argument("session", type=Path, help="rosbag2 session directory")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--raw", type=Path, help="OpenEB RAW file; auto-detected by default")
    parser.add_argument("--evbin", type=Path, help="Existing EVSBIN; skips RAW conversion")
    parser.add_argument("--converter", type=Path, help="Path to evs_raw_to_evbin")
    parser.add_argument("--rgb-topic", default="/realsense/color/image_raw")
    parser.add_argument(
        "--request-topic", default="/event_camera/raw_recording/request"
    )
    parser.add_argument(
        "--raw-start-bag-ns",
        type=int,
        help="Override RAW START rosbag timestamp when the request topic is unavailable",
    )
    parser.add_argument(
        "--offset-ms",
        type=float,
        default=0.0,
        help="Fixed residual offset; positive selects later EVS events",
    )
    parser.add_argument(
        "--auto-offset",
        action="store_true",
        help="Estimate residual offset from RGB motion and EVS activity",
    )
    parser.add_argument("--auto-search-ms", type=float, default=200.0)
    parser.add_argument("--auto-step-ms", type=float, default=2.0)
    parser.add_argument(
        "--auto-min-correlation",
        type=float,
        default=0.1,
        help="Reject automatic offset estimates below this correlation",
    )
    parser.add_argument(
        "--window-ms",
        type=float,
        help="Fixed EVS accumulation window; default is each actual RGB interval",
    )
    parser.add_argument("--start-sec", type=float, default=0.0)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--every", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--percentile", type=float, default=99.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    session = args.session.resolve()
    if not session.is_dir():
        raise FileNotFoundError(f"Session directory not found: {session}")
    if args.every <= 0 or args.max_frames < 0:
        raise ValueError("--every must be positive and --max-frames must be non-negative")
    if args.start_sec < 0 or args.duration_sec < 0:
        raise ValueError("--start-sec and --duration-sec must be non-negative")
    if args.window_ms is not None and args.window_ms <= 0:
        raise ValueError("--window-ms must be positive")
    if not 0 < args.percentile <= 100:
        raise ValueError("--percentile must be in (0, 100]")
    if args.auto_search_ms <= 0 or args.auto_step_ms <= 0:
        raise ValueError("auto-alignment search and step must be positive")
    if not -1.0 <= args.auto_min_correlation <= 1.0:
        raise ValueError("--auto-min-correlation must be within [-1, 1]")

    output_dir = (args.output_dir or session / "paper_rgb_event").resolve()
    resumable_names = {"rgb", "event", "pair"}
    existing_entries = list(output_dir.iterdir()) if output_dir.exists() else []
    unexpected_entries = [
        entry
        for entry in existing_entries
        if not (
            (entry.is_dir() and entry.name in resumable_names and not any(entry.iterdir()))
            or entry.name.endswith(".evbin")
            or entry.name.endswith(".evbin.conversion.json")
            or entry.name.endswith(".evbin.callbacks.csv")
        )
    ]
    if unexpected_entries:
        raise RuntimeError(
            f"Output directory is not empty: {output_dir}. Choose a new --output-dir."
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    np, cv2, bag_backend = _load_runtime_dependencies()

    raw_path = None
    if args.evbin is not None:
        evbin_path = args.evbin.resolve()
        if not evbin_path.is_file():
            raise FileNotFoundError(f"EVSBIN file not found: {evbin_path}")
    else:
        raw_path = _find_raw(session, args.raw.resolve() if args.raw else None)
        evbin_path = output_dir / f"{raw_path.stem}.evbin"
        if evbin_path.is_file():
            try:
                cached_header = read_evbin_header(evbin_path)
            except ValueError:
                cached_header = None
            if cached_header is not None and cached_header.event_count > 0:
                print(f"[1/4] Reusing existing EVSBIN: {evbin_path.name}")
            else:
                print(f"[1/4] Removing invalid EVSBIN cache: {evbin_path.name}")
                evbin_path.unlink()
                _convert_raw(raw_path, evbin_path, _find_converter(args.converter), np)
        else:
            _convert_raw(raw_path, evbin_path, _find_converter(args.converter), np)

    header = read_evbin_header(evbin_path)
    if header.event_count == 0:
        raise RuntimeError(f"EVSBIN contains no events: {evbin_path}")
    events = _event_memmap(evbin_path, header, np)
    times = events["t"]
    if not _timestamps_are_monotonic(times, np):
        raise RuntimeError("EVSBIN timestamps are not monotonic")
    first_event_us = int(times[0])

    print(f"[2/4] Reading RGB clock and RAW START anchor from {session.name}")
    requests, records = _scan_bag(
        session,
        args.rgb_topic,
        args.request_topic,
        args.auto_offset,
        np,
        cv2,
        bag_backend,
    )
    anchor_ns, anchor_method = _choose_raw_start_anchor(
        requests,
        first_event_us,
        int(times[-1]),
        args.raw_start_bag_ns,
    )
    print(f"      RAW start anchor: {anchor_method}")
    median_interval_ns = _median_interval_ns(records)
    offset_ms = float(args.offset_ms)
    auto_result = None
    if args.auto_offset:
        estimated_ms, score, samples = _estimate_offset_ms(
            records,
            times,
            anchor_ns,
            first_event_us,
            args.auto_search_ms,
            args.auto_step_ms,
            offset_ms,
            np,
        )
        accepted = score >= args.auto_min_correlation
        if accepted:
            offset_ms += estimated_ms
        auto_result = {
            "estimated_offset_ms": estimated_ms,
            "correlation": score,
            "sample_count": samples,
            "search_ms": args.auto_search_ms,
            "step_ms": args.auto_step_ms,
            "minimum_correlation": args.auto_min_correlation,
            "accepted": accepted,
        }
        if accepted:
            print(
                f"[3/4] Auto offset: {estimated_ms:+.3f} ms "
                f"(correlation={score:.3f}, final={offset_ms:+.3f} ms)"
            )
        else:
            print(
                f"[3/4] Auto offset rejected: correlation={score:.3f} < "
                f"{args.auto_min_correlation:.3f}; using {offset_ms:+.3f} ms"
            )
    else:
        print(f"[3/4] Fixed residual offset: {offset_ms:+.3f} ms")

    selected = _selected_indices(
        records,
        anchor_ns,
        args.start_sec,
        args.duration_sec,
        args.every,
        args.max_frames,
    )
    print(f"[4/4] Rendering {len(selected)} synchronized RGB/EVS pairs")
    rows = _export_frames(
        session,
        output_dir,
        args.rgb_topic,
        records,
        selected,
        events,
        header,
        anchor_ns,
        first_event_us,
        offset_ms,
        args.window_ms,
        median_interval_ns,
        args.percentile,
        np,
        cv2,
        bag_backend,
    )
    _write_manifest(output_dir / "frames.csv", rows)
    summary = {
        "session": str(session),
        "rgb_topic": args.rgb_topic,
        "raw_request_topic": args.request_topic,
        "raw_path": str(raw_path) if raw_path else None,
        "evbin_path": str(evbin_path),
        "output_frame_count": len(rows),
        "raw_start_bag_timestamp_ns": anchor_ns,
        "raw_start_anchor_method": anchor_method,
        "raw_request_messages": [
            {
                "bag_timestamp_ns": request.bag_timestamp_ns,
                "header_timestamp_ns": request.header_timestamp_ns,
                "command": request.command,
                "label": request.label,
            }
            for request in requests
        ],
        "raw_first_event_sensor_timestamp_us": first_event_us,
        "clock_mapping": (
            "event_sensor_us = raw_first_event_us + "
            "(rgb_bag_ns - raw_start_bag_ns) / 1000 + sync_offset_us"
        ),
        "sync_offset_ms": offset_ms,
        "manual_offset_ms": args.offset_ms,
        "auto_alignment": auto_result,
        "rgb_median_interval_ms": median_interval_ns / 1.0e6,
        "rgb_measured_hz": 1.0e9 / median_interval_ns,
        "event_window": (
            {"mode": "fixed", "window_ms": args.window_ms}
            if args.window_ms is not None
            else {"mode": "actual_rgb_interval"}
        ),
        "event_geometry": {"width": header.width, "height": header.height},
        "event_count_total": header.event_count,
        "bag_reader_backend": bag_backend.mode,
        "event_rendering": {
            "background": "white",
            "positive": "blue",
            "negative": "red",
            "normalization_percentile": args.percentile,
        },
        "timestamp_note": (
            "OpenEB RAW has no wall-clock timestamp. Synchronization is a relative-time "
            "estimate anchored at the rosbag receive time of the RAW START request."
        ),
    }
    (output_dir / "sync.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"Done: {output_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
