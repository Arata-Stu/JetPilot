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
from typing import Any, Iterable, Sequence


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


def _load_runtime_dependencies() -> tuple[Any, Any, Any]:
    try:
        import cv2
        import numpy as np
        from rosbags.highlevel import AnyReader
    except ImportError as error:
        raise RuntimeError(
            "This exporter requires numpy, opencv-python and rosbags. "
            "Run it in the JetPilot analysis/container environment."
        ) from error
    return np, cv2, AnyReader


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


def _find_converter(explicit: Path | None) -> Path:
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
    raise RuntimeError(
        "evs_raw_to_evbin was not found. Build tools/evs_benchmark first, "
        "or pass --converter /path/to/evs_raw_to_evbin."
    )


def _convert_raw(raw_path: Path, evbin_path: Path, converter: Path) -> None:
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
    print(f"[1/4] Converting RAW to monotonic EVSBIN: {raw_path.name}")
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
    AnyReader: Any,
) -> tuple[int | None, list[RgbRecord]]:
    records: list[RgbRecord] = []
    anchor_ns: int | None = None
    previous_gray = None
    with AnyReader([session]) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in {rgb_topic, request_topic}
        ]
        rgb_connections = [c for c in connections if c.topic == rgb_topic]
        if not rgb_connections:
            available = sorted({c.topic for c in reader.connections})
            raise RuntimeError(f"RGB topic {rgb_topic!r} not found; available: {available}")
        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            if connection.topic == request_topic:
                if anchor_ns is None:
                    anchor_ns = int(timestamp_ns)
                continue
            message = reader.deserialize(rawdata, connection.msgtype)
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
    return anchor_ns, records


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
    AnyReader: Any,
) -> list[dict[str, Any]]:
    rgb_dir = output_dir / "rgb"
    event_dir = output_dir / "event"
    pair_dir = output_dir / "pair"
    for directory in (rgb_dir, event_dir, pair_dir):
        directory.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    with AnyReader([session]) as reader:
        connections = [c for c in reader.connections if c.topic == rgb_topic]
        frame_index = 0
        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            if frame_index not in selected:
                frame_index += 1
                continue
            message = reader.deserialize(rawdata, connection.msgtype)
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

    output_dir = (args.output_dir or session / "paper_rgb_event").resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(
            f"Output directory is not empty: {output_dir}. Choose a new --output-dir."
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    np, cv2, AnyReader = _load_runtime_dependencies()

    raw_path = None
    if args.evbin is not None:
        evbin_path = args.evbin.resolve()
        if not evbin_path.is_file():
            raise FileNotFoundError(f"EVSBIN file not found: {evbin_path}")
    else:
        raw_path = _find_raw(session, args.raw.resolve() if args.raw else None)
        evbin_path = output_dir / f"{raw_path.stem}.evbin"
        _convert_raw(raw_path, evbin_path, _find_converter(args.converter))

    header = read_evbin_header(evbin_path)
    if header.event_count == 0:
        raise RuntimeError(f"EVSBIN contains no events: {evbin_path}")
    events = _event_memmap(evbin_path, header, np)
    times = events["t"]
    if not _timestamps_are_monotonic(times, np):
        raise RuntimeError("EVSBIN timestamps are not monotonic")
    first_event_us = int(times[0])

    print(f"[2/4] Reading RGB clock and RAW START anchor from {session.name}")
    detected_anchor_ns, records = _scan_bag(
        session,
        args.rgb_topic,
        args.request_topic,
        args.auto_offset,
        np,
        cv2,
        AnyReader,
    )
    anchor_ns = (
        args.raw_start_bag_ns
        if args.raw_start_bag_ns is not None
        else detected_anchor_ns
    )
    if anchor_ns is None:
        raise RuntimeError(
            f"No message was found on {args.request_topic}. "
            "Pass --raw-start-bag-ns explicitly."
        )
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
        offset_ms += estimated_ms
        auto_result = {
            "estimated_offset_ms": estimated_ms,
            "correlation": score,
            "sample_count": samples,
            "search_ms": args.auto_search_ms,
            "step_ms": args.auto_step_ms,
        }
        print(
            f"[3/4] Auto offset: {estimated_ms:+.3f} ms "
            f"(correlation={score:.3f}, final={offset_ms:+.3f} ms)"
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
        AnyReader,
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
