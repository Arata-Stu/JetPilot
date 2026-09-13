from __future__ import annotations

import argparse
import bisect
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np

from e2e_learning.data.event_tensor import EventTensorAccumulator, EventTensorConfig
from .analysis_worker import _control_payload, _deserializers, _open_reader, _stamp_ns


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


def build_dataset(args: argparse.Namespace) -> dict[str, object]:
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
    for topic in (args.event_topic, args.reference_topic, args.control_topic):
        if topic not in topic_types:
            raise RuntimeError(f"required topic was not found: {topic}")
    deserialize, get_message = _deserializers()
    classes = {
        topic: get_message(topic_types[topic])
        for topic in (args.event_topic, args.reference_topic, args.control_topic)
    }
    controls: list[tuple[int, dict[str, object]]] = []
    candidates: list[dict[str, object]] = []
    decoded_events = 0
    reference_messages = 0
    last_sample_bag_ns: int | None = None
    sample_interval_ns = max(1, int(round(1.0e9 / args.sample_hz)))
    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize(serialized, classes[topic])
        if topic == args.control_topic:
            controls.append((_timestamp(message, bag_ns, args.timestamp_source), _control_payload(message)))
            continue
        if topic == args.event_topic:
            decoded_events += accumulator.add_packet(decoder, message)
            continue
        reference_messages += 1
        if last_sample_bag_ns is not None and int(bag_ns) - last_sample_bag_ns < sample_interval_ns:
            continue
        snapshot = accumulator.snapshot()
        if snapshot is None:
            continue
        tensor, event_info = snapshot
        tensor_path = tensor_dir / f"{len(candidates):08d}.npy"
        np.save(tensor_path, tensor.astype(np.float16))
        candidates.append(
            {
                "tensor_path": tensor_path,
                "stamp": _timestamp(message, bag_ns, args.timestamp_source),
                "event_count": int(event_info["events"]),
                "event_window_end_sensor_ns": int(event_info["window_end_sensor_ns"]),
                "sum": tensor.sum(axis=(1, 2), dtype=np.float64),
                "square_sum": np.square(tensor, dtype=np.float64).sum(axis=(1, 2)),
            }
        )
        last_sample_bag_ns = int(bag_ns)

    controls.sort(key=lambda item: item[0])
    control_times = [item[0] for item in controls]
    max_control_dt_ns = int(round(args.max_control_dt_sec * 1.0e9))
    rows: list[dict[str, object]] = []
    sums = np.zeros(config.channels, dtype=np.float64)
    square_sums = np.zeros(config.channels, dtype=np.float64)
    values_per_channel = 0
    for candidate in candidates:
        stamp_ns = int(candidate["stamp"])
        control = _nearest_control(controls, control_times, stamp_ns, max_control_dt_ns)
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
                "control_dt_sec": f"{abs(stamp_ns - control[0]) / 1.0e9:.6f}",
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
    metadata = {
        "bag_path": str(bag),
        "task": "control",
        "modality": "event_tensor",
        "image_topic": args.reference_topic,
        "reference_clock_topic": args.reference_topic,
        "event_topic": args.event_topic,
        "control_topic": args.control_topic,
        "input_width": args.width,
        "input_height": args.height,
        "input_channels": config.channels,
        "sample_count": len(rows),
        "sample_hz": args.sample_hz,
        "timestamp_source": args.timestamp_source,
        "event_bins": args.bins,
        "event_window_ms": args.window_ms,
        "event_stride_ms": args.stride_ms,
        "event_polarity_mode": "separate",
        "event_polarity_layout": args.polarity_layout,
        "event_temporal_interpolation": args.temporal_interpolation,
        "event_clock_source": "sensor_time_latest_causal_at_reference",
        "tensor_dtype": "float16",
        "tensor_layout": "CHW",
        "mean": [float(value) for value in mean],
        "std": [float(value) for value in std],
        "decoded_events": decoded_events,
        "reference_message_count": reference_messages,
        "candidate_count": len(candidates),
        "dropped_without_control": len(candidates) - len(rows),
        "timestamp_resets": accumulator.timestamp_resets,
    }
    # JSON is valid YAML and avoids adding PyYAML to the ROS system Python.
    (output / "metadata.yaml").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
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
    build_dataset(parser.parse_args())


if __name__ == "__main__":
    main()
