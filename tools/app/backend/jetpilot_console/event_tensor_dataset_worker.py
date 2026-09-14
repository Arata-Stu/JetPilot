from __future__ import annotations

import argparse
import bisect
import csv
import json
from pathlib import Path
import time
from typing import Any

import numpy as np

from e2e_learning.data.event_tensor import EventTensorAccumulator, EventTensorConfig
from .analysis_worker import (
    _atomic_json, _control_payload, _decode_image, _deserializers, _metadata_duration_ns,
    _open_reader, _stamp_ns, _write_jpeg,
)


class DatasetProgress:
    def __init__(self, path: Path, bag: Path, mode: str) -> None:
        self.path = path
        self.bag = bag
        self.mode = mode
        self.duration_ns = _metadata_duration_ns(bag)
        self.started = time.monotonic()
        self.first_bag_ns: int | None = None
        self.last_write = 0.0
        self.update(stage="starting", force=True)

    def update(
        self,
        *,
        stage: str = "reading",
        bag_ns: int | None = None,
        tensor_count: int = 0,
        rgb_count: int = 0,
        decoded_events: int = 0,
        force: bool = False,
        complete: bool = False,
    ) -> None:
        now = time.monotonic()
        if not force and now - self.last_write < 0.5:
            return
        if bag_ns is not None and self.first_bag_ns is None:
            self.first_bag_ns = int(bag_ns)
        processed_ns = (
            max(0, int(bag_ns) - self.first_bag_ns)
            if bag_ns is not None and self.first_bag_ns is not None else 0
        )
        bag_progress = (
            min(1.0, processed_ns / self.duration_ns)
            if self.duration_ns else 0.0
        )
        elapsed_s = max(0.0, now - self.started)
        speed = processed_ns / 1.0e9 / elapsed_s if elapsed_s > 0.0 else 0.0
        eta_s = (
            max(0.0, (self.duration_ns - processed_ns) / 1.0e9 / speed)
            if self.duration_ns and speed > 0.0 else None
        )
        if complete:
            progress = bag_progress = 1.0
            eta_s = 0.0
            status = "completed"
            message = "Dataset creation completed."
        elif stage == "aligning":
            progress = 0.97
            status = "running"
            message = (
                "Aligning tensors with causal teacher control."
                if self.mode == "event_tensor"
                else "Aligning tensors with RGB and teacher control."
            )
        else:
            progress = min(0.96, bag_progress * 0.96)
            status = "running"
            message = "Decoding events and writing tensors."
        _atomic_json(self.path, {
            "schema_version": 1,
            "status": status,
            "stage": stage,
            "progress": progress,
            "bag_progress": bag_progress,
            "message": message,
            "bag_path": str(self.bag),
            "dataset_mode": self.mode,
            "elapsed_s": elapsed_s,
            "eta_s": eta_s,
            "processed_bag_s": processed_ns / 1.0e9,
            "bag_duration_s": self.duration_ns / 1.0e9 if self.duration_ns else None,
            "processing_speed": speed,
            "event_tensor_count": tensor_count,
            "rgb_frame_count": rgb_count,
            "decoded_events": decoded_events,
        })
        self.last_write = now
        if force:
            eta_label = f"{eta_s:.1f}s" if eta_s is not None else "calculating"
            print(
                f"[dataset-progress] {progress * 100:.1f}% "
                f"tensors={tensor_count} RGB={rgb_count} ETA={eta_label}",
                flush=True,
            )


def _timestamp(message: Any, bag_ns: int, source: str) -> int:
    if source == "header":
        header_ns = _stamp_ns(message)
        if header_ns is not None:
            return int(header_ns)
    return int(bag_ns)


def _nearest_control(
    controls: list[tuple[int, dict[str, object]]], times: list[int], stamp_ns: int, max_dt_ns: int
) -> tuple[int, dict[str, object]] | None:
    position = bisect.bisect_left(times, stamp_ns)
    candidates = [candidate for candidate in (position - 1, position) if 0 <= candidate < len(controls)]
    if not candidates:
        return None
    selected = min(candidates, key=lambda candidate: abs(times[candidate] - stamp_ns))
    return controls[selected] if abs(times[selected] - stamp_ns) <= max_dt_ns else None


def _causal_control(
    controls: list[tuple[int, dict[str, object]]], times: list[int], stamp_ns: int, max_dt_ns: int
) -> tuple[int, dict[str, object]] | None:
    """Return the newest teacher command at or before the tensor timestamp."""
    position = bisect.bisect_right(times, stamp_ns) - 1
    if position < 0:
        return None
    control = controls[position]
    return control if 0 <= stamp_ns - control[0] <= max_dt_ns else None


def _limited_indices(count: int, maximum: int) -> list[int]:
    if count <= maximum:
        return list(range(count))
    if maximum == 1:
        return [count - 1]
    return sorted({round(index * (count - 1) / (maximum - 1)) for index in range(maximum)})


def _build_async_dataset(args: argparse.Namespace) -> dict[str, object]:
    import event_camera_py

    numpy_major = int(np.__version__.split(".", 1)[0])
    if numpy_major >= 2 and getattr(event_camera_py, "__file__", None):
        raise RuntimeError(f"event_camera_py requires NumPy 1.x; found {np.__version__}")

    bag = Path(args.rosbag).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"dataset output is not empty: {output}")
    image_dir, tensor_dir = output / "images", output / "tensors"
    image_dir.mkdir(parents=True, exist_ok=True)
    tensor_dir.mkdir(parents=True, exist_ok=True)
    progress_path = Path(args.progress_file) if args.progress_file else output / "progress.json"
    progress = DatasetProgress(progress_path, bag, args.dataset_mode)
    config = EventTensorConfig(
        bins=args.bins, window_ms=args.window_ms, stride_ms=args.stride_ms,
        width=args.width, height=args.height, polarity_layout=args.polarity_layout,
        temporal_interpolation=args.temporal_interpolation,
    )
    accumulator = EventTensorAccumulator(config)
    decoder = event_camera_py.Decoder()
    reader, topic_types = _open_reader(bag)
    for topic in (args.event_topic, args.reference_topic, args.control_topic):
        if topic not in topic_types:
            raise RuntimeError(f"required topic was not found: {topic}")
    deserialize, get_message = _deserializers()
    classes = {topic: get_message(topic_types[topic]) for topic in (
        args.event_topic, args.reference_topic, args.control_topic
    )}
    controls: list[tuple[int, dict[str, object]]] = []
    rgb_frames: list[dict[str, object]] = []
    events: list[dict[str, object]] = []
    decoded_events = 0
    next_rgb_bag_ns: int | None = None
    next_event_bag_ns: int | None = None
    last_event_end_ns: int | None = None
    rgb_interval_ns = max(1, round(1.0e9 / args.sample_hz))
    event_interval_ns = max(1, round(1.0e9 / args.event_sample_hz))
    # When the requested update period is the representation stride (250 Hz
    # for a 4 ms stride), every packet must reach snapshot(). Applying another
    # bag-time rate limiter at the same nominal frequency drops updates whenever
    # packet arrival jitter places one just before the next deadline. The
    # accumulator already de-duplicates packets that do not cross a new sensor
    # time boundary. Keep bag-time throttling only for an explicitly slower
    # requested output rate.
    throttle_event_packets = event_interval_ns > accumulator.stride_ns
    message_count = 0
    latest_bag_ns: int | None = None
    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        latest_bag_ns = int(bag_ns)
        message_count += 1
        if message_count == 1 or message_count % 128 == 0:
            progress.update(
                bag_ns=latest_bag_ns, tensor_count=len(events), rgb_count=len(rgb_frames),
                decoded_events=decoded_events, force=message_count == 1,
            )
        if topic not in classes:
            continue
        message = deserialize(serialized, classes[topic])
        if topic == args.control_topic:
            controls.append((_timestamp(message, bag_ns, args.timestamp_source), _control_payload(message)))
        elif topic == args.reference_topic:
            if next_rgb_bag_ns is not None and int(bag_ns) < next_rgb_bag_ns:
                continue
            relative = Path("images") / f"{len(rgb_frames):08d}.jpg"
            _write_jpeg(output / relative, _decode_image(message, topic), 92)
            rgb_frames.append({"stamp": _timestamp(message, bag_ns, args.timestamp_source), "path": relative.as_posix()})
            if next_rgb_bag_ns is None:
                next_rgb_bag_ns = int(bag_ns) + rgb_interval_ns
            else:
                while next_rgb_bag_ns <= int(bag_ns):
                    next_rgb_bag_ns += rgb_interval_ns
        else:
            decoded_events += accumulator.add_packet(decoder, message)
            if (
                throttle_event_packets
                and next_event_bag_ns is not None
                and int(bag_ns) < next_event_bag_ns
            ):
                continue
            snapshot = accumulator.snapshot()
            if snapshot is None:
                continue
            tensor, info = snapshot
            end_ns = int(info["window_end_sensor_ns"])
            if end_ns == last_event_end_ns:
                continue
            packet_stamp = _timestamp(message, bag_ns, args.timestamp_source)
            latest_sensor_ns = int(accumulator.latest_event_ns or end_ns)
            # Match EventTensorEncoderNode::ros_timestamp_ns(): anchor the
            # sensor-time window boundary to the packet's ROS/bag clock. This
            # keeps delta_t on exact 4 ms representation boundaries at 250 Hz.
            representation_stamp = packet_stamp - latest_sensor_ns + end_ns
            relative = Path("tensors") / f"{len(events):08d}.npz"
            np.savez_compressed(output / relative, tensor=tensor.astype(np.float16))
            events.append({
                "stamp": representation_stamp, "packet_stamp": packet_stamp,
                "sensor_stamp": end_ns,
                "path": relative.as_posix(), "events": int(info["events"]),
                "sum": tensor.sum(axis=(1, 2), dtype=np.float64),
                "square_sum": np.square(tensor, dtype=np.float64).sum(axis=(1, 2)),
            })
            if throttle_event_packets:
                if next_event_bag_ns is None:
                    next_event_bag_ns = int(bag_ns) + event_interval_ns
                else:
                    while next_event_bag_ns <= int(bag_ns):
                        next_event_bag_ns += event_interval_ns
            last_event_end_ns = end_ns

    progress.update(
        stage="aligning", bag_ns=latest_bag_ns, tensor_count=len(events),
        rgb_count=len(rgb_frames), decoded_events=decoded_events, force=True,
    )
    controls.sort(key=lambda item: item[0])
    rgb_frames.sort(key=lambda item: int(item["stamp"]))
    events.sort(key=lambda item: int(item["stamp"]))
    control_times = [value[0] for value in controls]
    event_times = [int(value["stamp"]) for value in events]
    max_control_dt_ns = round(args.max_control_dt_sec * 1.0e9)
    rows: list[dict[str, object]] = []
    sums = np.zeros(config.channels, dtype=np.float64)
    square_sums = np.zeros(config.channels, dtype=np.float64)
    value_count = 0
    for first, second in zip(rgb_frames, rgb_frames[1:]):
        start_stamp, end_stamp = int(first["stamp"]), int(second["stamp"])
        begin = bisect.bisect_right(event_times, start_stamp)
        end = bisect.bisect_right(event_times, end_stamp)
        interval_events = events[begin:end]
        selected = [interval_events[index] for index in _limited_indices(len(interval_events), args.rollout_steps)]
        aligned = []
        for event in selected:
            control = _nearest_control(controls, control_times, int(event["stamp"]), max_control_dt_ns)
            if control is not None:
                aligned.append((event, control))
        if not aligned:
            continue
        previous_stamp = start_stamp
        previous_sensor_stamp: int | None = None
        paths, deltas, labels, stamps = [], [], [], []
        for event, control in aligned:
            stamp = int(event["stamp"])
            sensor_stamp = int(event["sensor_stamp"])
            paths.append(event["path"])
            elapsed_ns = (
                stamp - previous_stamp if previous_sensor_stamp is None
                else sensor_stamp - previous_sensor_stamp
            )
            deltas.append(max(0.0, elapsed_ns / 1.0e9))
            labels.append([float(control[1].get("steering", 0.0)), float(control[1].get("throttle", 0.0))])
            stamps.append(stamp)
            previous_stamp = stamp
            previous_sensor_stamp = sensor_stamp
            sums += event["sum"]
            square_sums += event["square_sum"]
            value_count += args.width * args.height
        rows.append({
            "sequence_id": bag.name, "image_path": first["path"],
            "next_image_path": second["path"], "stamp": start_stamp,
            "next_stamp": end_stamp, "event_tensor_paths": json.dumps(paths),
            "event_stamps": json.dumps(stamps), "event_delta_t": json.dumps(deltas),
            "event_controls": json.dumps(labels), "event_steps": len(paths),
        })
    if not rows:
        raise RuntimeError("no RGB intervals with aligned EVS tensors and controls were found")
    with (output / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    mean = sums / max(value_count, 1)
    std = np.sqrt(np.maximum(square_sums / max(value_count, 1) - mean * mean, 0.0))
    std[std < 1.0e-6] = 1.0
    event_span_sec = (
        (int(events[-1]["sensor_stamp"]) - int(events[0]["sensor_stamp"])) / 1.0e9
        if len(events) > 1 else 0.0
    )
    effective_event_sample_hz = (
        (len(events) - 1) / event_span_sec if event_span_sec > 0.0 else 0.0
    )
    metadata = {
        "bag_path": str(bag), "task": "control", "modality": "rgb_event_async",
        "image_topic": args.reference_topic, "event_topic": args.event_topic,
        "control_topic": args.control_topic, "input_width": args.width,
        "input_height": args.height, "input_channels": 3, "event_channels": config.channels,
        "sample_count": len(rows), "sample_hz": args.sample_hz,
        "event_sample_hz": args.event_sample_hz, "rollout_steps": args.rollout_steps,
        "timestamp_source": args.timestamp_source, "event_bins": args.bins,
        "event_window_ms": args.window_ms, "event_stride_ms": args.stride_ms,
        "event_polarity_mode": "separate", "event_polarity_layout": args.polarity_layout,
        "event_temporal_interpolation": args.temporal_interpolation,
        "mean": [0.485, 0.456, 0.406], "std": [0.229, 0.224, 0.225],
        "event_mean": mean.tolist(), "event_std": std.tolist(),
        "decoded_events": decoded_events, "rgb_frame_count": len(rgb_frames),
        "event_tensor_count": len(events),
        "normalization_tensor_count": value_count // (args.width * args.height),
        "effective_event_sample_hz": effective_event_sample_hz,
        "timestamp_resets": accumulator.timestamp_resets,
    }
    (output / "metadata.yaml").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    progress.update(
        stage="complete", bag_ns=latest_bag_ns, tensor_count=len(events),
        rgb_count=len(rgb_frames), decoded_events=decoded_events, force=True, complete=True,
    )
    print(json.dumps(metadata, indent=2))
    return metadata


def build_dataset(args: argparse.Namespace) -> dict[str, object]:
    if args.dataset_mode == "rgb_event_async":
        return _build_async_dataset(args)
    import event_camera_py

    numpy_major = int(np.__version__.split(".", 1)[0])
    if numpy_major >= 2 and getattr(event_camera_py, "__file__", None):
        raise RuntimeError(f"event_camera_py requires NumPy 1.x; found {np.__version__}")
    bag = Path(args.rosbag).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"dataset output is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    tensor_dir = output / "tensors"
    tensor_dir.mkdir(parents=True, exist_ok=True)
    progress_path = Path(args.progress_file) if args.progress_file else output / "progress.json"
    progress = DatasetProgress(progress_path, bag, args.dataset_mode)
    config = EventTensorConfig(
        bins=args.bins,
        window_ms=args.window_ms,
        stride_ms=args.stride_ms,
        width=args.width,
        height=args.height,
        polarity_layout=args.polarity_layout,
        temporal_interpolation=args.temporal_interpolation,
    )
    accumulator = EventTensorAccumulator(config)
    decoder = event_camera_py.Decoder()
    reader, topic_types = _open_reader(bag)
    for topic in (args.event_topic, args.control_topic):
        if topic not in topic_types:
            raise RuntimeError(f"required topic was not found: {topic}")
    deserialize, get_message = _deserializers()
    selected_topics = [args.event_topic, args.control_topic]
    if args.reference_topic in topic_types:
        selected_topics.append(args.reference_topic)
    classes = {topic: get_message(topic_types[topic]) for topic in selected_topics}
    controls: list[tuple[int, dict[str, object]]] = []
    candidates: list[dict[str, object]] = []
    decoded_events = 0
    reference_messages = 0
    sample_interval_ns = max(1, int(round(1.0e9 / args.sample_hz)))
    native_interval_ns = accumulator.stride_ns
    if sample_interval_ns < native_interval_ns:
        raise ValueError(
            f"sample_hz={args.sample_hz:g} exceeds the event representation boundary "
            f"rate {1.0e9 / native_interval_ns:g} Hz"
        )
    next_boundary_sensor_ns: int | None = None
    next_sample_due_sensor_ns: int | None = None
    observed_timestamp_resets = 0
    message_count = 0
    latest_bag_ns: int | None = None
    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        latest_bag_ns = int(bag_ns)
        message_count += 1
        if message_count == 1 or message_count % 128 == 0:
            progress.update(
                bag_ns=latest_bag_ns, tensor_count=len(candidates),
                decoded_events=decoded_events, force=message_count == 1,
            )
        if topic not in classes:
            continue
        message = deserialize(serialized, classes[topic])
        if topic == args.control_topic:
            controls.append((_timestamp(message, bag_ns, args.timestamp_source), _control_payload(message)))
            continue
        if topic == args.event_topic:
            decoded_events += accumulator.add_packet(decoder, message)
            if accumulator.timestamp_resets != observed_timestamp_resets:
                next_boundary_sensor_ns = None
                next_sample_due_sensor_ns = None
                observed_timestamp_resets = accumulator.timestamp_resets
            if accumulator.first_event_ns is None or accumulator.latest_event_ns is None:
                continue
            available_end_ns = accumulator.first_event_ns + (
                (accumulator.latest_event_ns - accumulator.first_event_ns)
                // native_interval_ns
            ) * native_interval_ns
            if next_boundary_sensor_ns is None:
                minimum_steps = (
                    accumulator.window_ns + native_interval_ns - 1
                ) // native_interval_ns
                next_boundary_sensor_ns = (
                    accumulator.first_event_ns + minimum_steps * native_interval_ns
                )
                next_sample_due_sensor_ns = next_boundary_sensor_ns
            packet_stamp = _timestamp(message, bag_ns, args.timestamp_source)
            latest_sensor_ns = accumulator.latest_event_ns
            while next_boundary_sensor_ns <= available_end_ns:
                boundary_ns = next_boundary_sensor_ns
                next_boundary_sensor_ns += native_interval_ns
                if (
                    next_sample_due_sensor_ns is not None
                    and boundary_ns < next_sample_due_sensor_ns
                ):
                    continue
                snapshot = accumulator.snapshot_at(boundary_ns)
                if snapshot is None:
                    continue
                tensor, event_info = snapshot
                tensor_path = tensor_dir / f"{len(candidates):08d}.npy"
                np.save(tensor_path, tensor.astype(np.float16))
                representation_stamp = packet_stamp - latest_sensor_ns + boundary_ns
                candidates.append(
                    {
                        "tensor_path": tensor_path,
                        "stamp": representation_stamp,
                        "event_count": int(event_info["events"]),
                        "event_window_end_sensor_ns": boundary_ns,
                        "sum": tensor.sum(axis=(1, 2), dtype=np.float64),
                        "square_sum": np.square(tensor, dtype=np.float64).sum(axis=(1, 2)),
                    }
                )
                if next_sample_due_sensor_ns is not None:
                    while next_sample_due_sensor_ns <= boundary_ns:
                        next_sample_due_sensor_ns += sample_interval_ns
            continue
        reference_messages += 1

    progress.update(
        stage="aligning", bag_ns=latest_bag_ns, tensor_count=len(candidates),
        decoded_events=decoded_events, force=True,
    )
    controls.sort(key=lambda item: item[0])
    control_times = [item[0] for item in controls]
    max_control_dt_ns = int(round(args.max_control_dt_sec * 1.0e9))
    rows: list[dict[str, object]] = []
    sums = np.zeros(config.channels, dtype=np.float64)
    square_sums = np.zeros(config.channels, dtype=np.float64)
    values_per_channel = 0
    for candidate in candidates:
        stamp_ns = int(candidate["stamp"])
        control = _causal_control(controls, control_times, stamp_ns, max_control_dt_ns)
        if control is None:
            continue
        sums += candidate["sum"]
        square_sums += candidate["square_sum"]
        values_per_channel += args.width * args.height
        value = control[1]
        rows.append(
            {
                "sequence_id": bag.name,
                "tensor_path": Path(candidate["tensor_path"]).relative_to(output).as_posix(),
                "image_path": "",
                "stamp": stamp_ns,
                "control_stamp": control[0],
                "control_dt_sec": f"{(stamp_ns - control[0]) / 1.0e9:.6f}",
                "steering": f"{float(value.get('steering', 0.0)):.8f}",
                "throttle": f"{float(value.get('throttle', 0.0)):.8f}",
                "trajectory": "[]",
                "imu": "[]",
                "image_topic": args.reference_topic,
                "control_topic": args.control_topic,
                "odometry_topic": "",
                "imu_topic": "",
                "event_count": candidate["event_count"],
                "event_window_end_sensor_ns": candidate["event_window_end_sensor_ns"],
            }
        )
    if not rows:
        raise RuntimeError("no event tensors could be aligned with teacher control")
    fields = list(rows[0])
    with (output / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    mean = sums / max(values_per_channel, 1)
    variance = np.maximum(square_sums / max(values_per_channel, 1) - mean * mean, 0.0)
    std = np.sqrt(variance)
    std[std < 1.0e-6] = 1.0
    event_span_sec = (
        (
            int(candidates[-1]["event_window_end_sensor_ns"])
            - int(candidates[0]["event_window_end_sensor_ns"])
        ) / 1.0e9
        if len(candidates) > 1 else 0.0
    )
    effective_event_sample_hz = (
        (len(candidates) - 1) / event_span_sec if event_span_sec > 0.0 else 0.0
    )
    metadata = {
        "bag_path": str(bag),
        "task": "control",
        "modality": "event_tensor",
        "image_topic": "",
        "reference_image_topic": args.reference_topic,
        "reference_clock_topic": args.event_topic,
        "event_topic": args.event_topic,
        "control_topic": args.control_topic,
        "input_width": args.width,
        "input_height": args.height,
        "input_channels": config.channels,
        "sample_count": len(rows),
        "sample_hz": args.sample_hz,
        "event_sample_hz": args.sample_hz,
        "timestamp_source": args.timestamp_source,
        "event_bins": args.bins,
        "event_window_ms": args.window_ms,
        "event_stride_ms": args.stride_ms,
        "event_polarity_mode": "separate",
        "event_polarity_layout": args.polarity_layout,
        "event_temporal_interpolation": args.temporal_interpolation,
        "event_clock_source": "fixed_event_sensor_time",
        "tensor_dtype": "float16",
        "tensor_layout": "CHW",
        "mean": [float(value) for value in mean],
        "std": [float(value) for value in std],
        "decoded_events": decoded_events,
        "control_message_count": len(controls),
        "reference_message_count": reference_messages,
        "candidate_count": len(candidates),
        "effective_event_sample_hz": effective_event_sample_hz,
        "dropped_without_control": len(candidates) - len(rows),
        "timestamp_resets": accumulator.timestamp_resets,
    }
    # JSON is valid YAML and avoids adding PyYAML to the ROS system Python.
    (output / "metadata.yaml").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    progress.update(
        stage="complete", bag_ns=latest_bag_ns, tensor_count=len(candidates),
        decoded_events=decoded_events, force=True, complete=True,
    )
    print(json.dumps(metadata, indent=2, ensure_ascii=False))
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rosbag", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--reference-topic", required=True)
    parser.add_argument("--event-topic", required=True)
    parser.add_argument("--control-topic", required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--bins", type=int, required=True)
    parser.add_argument("--window-ms", type=float, required=True)
    parser.add_argument("--stride-ms", type=float, required=True)
    parser.add_argument("--polarity-layout", choices=("polarity_major", "time_major"), required=True)
    parser.add_argument("--temporal-interpolation", choices=("none", "linear"), required=True)
    parser.add_argument("--sample-hz", type=float, required=True)
    parser.add_argument("--max-control-dt-sec", type=float, required=True)
    parser.add_argument("--timestamp-source", choices=("bag", "header"), required=True)
    parser.add_argument("--dataset-mode", choices=("event_tensor", "rgb_event_async"), default="event_tensor")
    parser.add_argument("--event-sample-hz", type=float, default=100.0)
    parser.add_argument("--rollout-steps", type=int, default=8)
    parser.add_argument("--progress-file", default="")
    build_dataset(parser.parse_args())


if __name__ == "__main__":
    main()
