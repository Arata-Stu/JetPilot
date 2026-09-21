#!/usr/bin/env python3
"""Summarize online EVS -> TensorRT benchmark diagnostics from ROS 2 bags."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


E2E_TOPIC = "/e2e/diagnostics"
EVENT_TOPIC = "/e2e/event_tensor/diagnostics"
JETSON_TOPIC = "/jetson/diagnostics"
CONTROL_TOPIC = "/benchmark/e2e/control_cmd"

COUNTER_KEYS = (
    "decode_errors",
    "publish_errors",
    "watchdog_timeouts",
    "fixed_rate_skipped_windows",
    "packet_queue_dropped_packets",
    "packet_queue_stale_packets",
    "decoded_queue_dropped_batches",
    "decoded_queue_dropped_events",
    "decoded_queue_stale_batches",
    "decoded_queue_stale_events",
    "memory_pool_exhaustions",
    "deadline_misses",
    "timestamp_resets",
    "input_sequence_missing_packets",
    "input_sequence_reorders",
)


def finite_number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def percentile(values: Iterable[float], quantile: float) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def numeric_summary(values: Iterable[float]) -> dict[str, float | int | None]:
    clean = [value for value in values if math.isfinite(value)]
    if not clean:
        return {"count": 0, "mean": None, "p50": None, "p95": None, "p99": None,
                "min": None, "max": None, "last": None}
    return {
        "count": len(clean),
        "mean": sum(clean) / len(clean),
        "p50": percentile(clean, 0.50),
        "p95": percentile(clean, 0.95),
        "p99": percentile(clean, 0.99),
        "min": min(clean),
        "max": max(clean),
        "last": clean[-1],
    }


def diagnostic_values(message: Any) -> Iterable[dict[str, str]]:
    for status in message.status:
        yield {str(item.key): str(item.value) for item in status.values}


def open_reader(path: Path) -> tuple[Any, Any, Any]:
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python modules are unavailable. Source /opt/ros/jazzy/setup.bash and "
            "/workspaces/ros2_ws/install/setup.bash, then run with /usr/bin/python3."
        ) from error

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(path), storage_id="mcap"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader, deserialize_message, get_message


def rate_hz(timestamps_ns: list[int]) -> float | None:
    if len(timestamps_ns) < 2:
        return None
    span_s = (timestamps_ns[-1] - timestamps_ns[0]) / 1.0e9
    return (len(timestamps_ns) - 1) / span_s if span_s > 0.0 else None


def parse_bag(label: str, bag: Path, deadline_ms: float) -> dict[str, Any]:
    if not bag.is_dir():
        raise FileNotFoundError(f"Bag directory not found: {bag}")

    reader, deserialize_message, get_message = open_reader(bag)
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    diagnostic_type = topic_types.get(E2E_TOPIC)
    if diagnostic_type is None:
        raise RuntimeError(f"{E2E_TOPIC} is missing from {bag}")
    diagnostic_message = get_message(diagnostic_type)

    topic_counts: dict[str, int] = defaultdict(int)
    topic_timestamps: dict[str, list[int]] = defaultdict(list)
    numeric: dict[str, dict[str, list[float]]] = {
        E2E_TOPIC: defaultdict(list),
        EVENT_TOPIC: defaultdict(list),
        JETSON_TOPIC: defaultdict(list),
    }
    text: dict[str, dict[str, list[str]]] = {
        E2E_TOPIC: defaultdict(list),
        EVENT_TOPIC: defaultdict(list),
        JETSON_TOPIC: defaultdict(list),
    }
    first_ns: int | None = None
    last_ns: int | None = None

    while reader.has_next():
        topic, payload, timestamp_ns = reader.read_next()
        first_ns = timestamp_ns if first_ns is None else min(first_ns, timestamp_ns)
        last_ns = timestamp_ns if last_ns is None else max(last_ns, timestamp_ns)
        topic_counts[topic] += 1
        if topic == CONTROL_TOPIC:
            topic_timestamps[topic].append(timestamp_ns)
        if topic not in numeric:
            continue
        message = deserialize_message(payload, diagnostic_message)
        for values in diagnostic_values(message):
            for key, raw_value in values.items():
                number = finite_number(raw_value)
                if number is None:
                    text[topic][key].append(raw_value)
                else:
                    numeric[topic][key].append(number)

    duration_s = (
        (last_ns - first_ns) / 1.0e9
        if first_ns is not None and last_ns is not None and last_ns >= first_ns
        else 0.0
    )
    capture = numeric[E2E_TOPIC].get("capture_to_command_ms", [])
    output_interval = numeric[E2E_TOPIC].get("output_interval_ms", [])
    recorded_misses = numeric[E2E_TOPIC].get("missed_deadline", [])
    recomputed_misses = sum(value > deadline_ms for value in capture)

    event_counters = {
        key: sum(numeric[EVENT_TOPIC].get(key, []))
        for key in COUNTER_KEYS
        if key in numeric[EVENT_TOPIC]
    }
    all_numeric = {
        topic: {key: numeric_summary(values) for key, values in fields.items()}
        for topic, fields in numeric.items()
    }
    all_text = {
        topic: {key: values[-1] for key, values in fields.items() if values}
        for topic, fields in text.items()
    }

    return {
        "label": label,
        "bag": str(bag),
        "bag_duration_s": duration_s,
        "topic_counts": dict(topic_counts),
        "control_output_hz": rate_hz(topic_timestamps[CONTROL_TOPIC]),
        "control_output_hz_over_bag": (
            topic_counts[CONTROL_TOPIC] / duration_s if duration_s > 0.0 else None
        ),
        "capture_to_command_ms": numeric_summary(capture),
        "decoder_callback_ms": numeric_summary(
            numeric[E2E_TOPIC].get("decoder_callback_ms", [])
        ),
        "output_interval_ms": numeric_summary(output_interval),
        "configured_deadline_ms": numeric_summary(
            numeric[E2E_TOPIC].get("deadline_ms", [])
        )["last"],
        "evaluated_deadline_ms": deadline_ms,
        "deadline_miss_count_recomputed": recomputed_misses,
        "deadline_miss_rate_recomputed": (
            recomputed_misses / len(capture) if capture else None
        ),
        "deadline_miss_count_recorded": int(round(sum(recorded_misses))),
        "stale_output_count": int(round(sum(numeric[E2E_TOPIC].get("stale_output", [])))),
        "event_counters": event_counters,
        "diagnostic_numeric": all_numeric,
        "diagnostic_text_last": all_text,
    }


def compact_row(result: dict[str, Any]) -> dict[str, Any]:
    latency = result["capture_to_command_ms"]
    interval = result["output_interval_ms"]
    decoder = result["decoder_callback_ms"]
    counters = result["event_counters"]
    pipeline = result.get("diagnostic_numeric", {}).get(E2E_TOPIC, {})
    sensor_to_input = pipeline.get("sensor_to_tensor_input_ms", {})
    input_to_output = pipeline.get("tensor_input_to_output_ms", {})
    sensor_to_output = pipeline.get("sensor_to_tensor_output_ms", {})
    matched = pipeline.get("tensor_input_matched", {})
    misses = result["deadline_miss_count_recomputed"]
    samples = latency["count"]
    return {
        "profile": result["label"],
        "duration_s": result["bag_duration_s"],
        "outputs": result["topic_counts"].get(CONTROL_TOPIC, 0),
        "output_hz": result["control_output_hz"],
        "latency_mean_ms": latency["mean"],
        "latency_p50_ms": latency["p50"],
        "latency_p95_ms": latency["p95"],
        "latency_p99_ms": latency["p99"],
        "latency_max_ms": latency["max"],
        "deadline_ms": result["evaluated_deadline_ms"],
        "deadline_misses": misses,
        "deadline_miss_pct": 100.0 * misses / samples if samples else None,
        "output_interval_p99_ms": interval["p99"],
        "output_interval_max_ms": interval["max"],
        "decoder_p99_ms": decoder["p99"],
        "decoder_max_ms": decoder["max"],
        "sensor_to_tensor_input_mean_ms": sensor_to_input.get("mean"),
        "sensor_to_tensor_input_p99_ms": sensor_to_input.get("p99"),
        "tensor_input_to_output_mean_ms": input_to_output.get("mean"),
        "tensor_input_to_output_p99_ms": input_to_output.get("p99"),
        "sensor_to_tensor_output_mean_ms": sensor_to_output.get("mean"),
        "sensor_to_tensor_output_p99_ms": sensor_to_output.get("p99"),
        "pipeline_latency_matched": int(round(
            (matched.get("mean") or 0.0) * (matched.get("count") or 0)
        )),
        "event_diagnostics": result["topic_counts"].get(EVENT_TOPIC, 0),
        "event_deadline_misses": counters.get("deadline_misses", 0.0),
        "event_skipped_windows": counters.get("fixed_rate_skipped_windows", 0.0),
        "event_other_error_drop_total": sum(
            value for key, value in counters.items()
            if key not in {"deadline_misses", "fixed_rate_skipped_windows"}
        ),
    }


def parse_run(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("Use LABEL=/absolute/path/to/bag")
    label, raw_path = value.split("=", 1)
    if not label or not raw_path:
        raise argparse.ArgumentTypeError("Use LABEL=/absolute/path/to/bag")
    return label, Path(raw_path).expanduser().resolve()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run", action="append", required=True, type=parse_run,
        metavar="LABEL=BAG", help="Profile label and ROS 2 bag directory; repeat per profile",
    )
    parser.add_argument("--deadline-ms", type=float, default=4.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.deadline_ms <= 0.0:
        parser.error("--deadline-ms must be positive")

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    results = [parse_bag(label, path, args.deadline_ms) for label, path in args.run]
    rows = [compact_row(result) for result in results]

    (output_dir / "online_tensorrt_diagnostics.json").write_text(
        json.dumps(results, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    csv_path = output_dir / "online_tensorrt_summary.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    terminal = csv.DictWriter(__import__("sys").stdout, fieldnames=list(rows[0]))
    terminal.writeheader()
    terminal.writerows(rows)
    print(f"\nCSV:  {csv_path}")
    print(f"JSON: {output_dir / 'online_tensorrt_diagnostics.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
