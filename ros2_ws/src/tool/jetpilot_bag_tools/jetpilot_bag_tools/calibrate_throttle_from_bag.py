#!/usr/bin/env python3
"""Build a throttle feedforward calibration from a ROS 2 bag."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
from typing import Any

from jetpilot_bag_tools.throttle_calibration import (
    AnalysisConfig,
    CalibrationSample,
    analyze_samples,
)


def bag_uri(path: Path) -> Path:
    if path.is_file() and path.suffix == ".db3" and (path.parent / "metadata.yaml").exists():
        return path.parent
    return path


def storage_id(path: Path) -> str:
    uri = bag_uri(path)
    if path.is_file() and path.suffix == ".mcap":
        return "mcap"
    if uri.is_dir() and list(uri.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def read_samples(
    path: Path, command_topic: str, odometry_topic: str, maximum_command_age_s: float
) -> list[CalibrationSample]:
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("Run this tool in a sourced ROS 2 environment") from exc

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_uri(path)), storage_id=storage_id(path)),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    for topic in (command_topic, odometry_topic):
        if topic not in topic_types:
            raise RuntimeError(f"required topic is missing from bag: {topic}")
    command_type = get_message(topic_types[command_topic])
    odometry_type = get_message(topic_types[odometry_topic])
    latest_command: tuple[int, Any] | None = None
    samples: list[CalibrationSample] = []

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic == command_topic:
            latest_command = (timestamp_ns, deserialize_message(data, command_type))
        elif topic == odometry_topic and latest_command is not None:
            command_time_ns, command = latest_command
            age_s = (timestamp_ns - command_time_ns) * 1.0e-9
            if age_s < 0.0 or age_s > maximum_command_age_s:
                continue
            odometry = deserialize_message(data, odometry_type)
            linear = odometry.twist.twist.linear
            samples.append(
                CalibrationSample(
                    time_s=timestamp_ns * 1.0e-9,
                    throttle=float(command.throttle),
                    speed_mps=math.hypot(float(linear.x), float(linear.y)),
                    steering=float(command.steering),
                    brake=float(command.brake),
                    reverse=float(command.reverse),
                )
            )
    return samples


def validate_monotonic(points: list[Any]) -> None:
    for previous, current in zip(points, points[1:]):
        if current.steady_speed_mps <= previous.steady_speed_mps:
            raise RuntimeError(
                "steady speed is not monotonic with throttle; repeat the affected runs: "
                f"{previous.throttle:.3f}, {current.throttle:.3f}"
            )


def write_outputs(
    output_dir: Path, bag: Path, result: Any, minimum_steady_speed_mps: float
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "source_bag": str(bag.resolve()),
        "points": [asdict(point) for point in result.points],
        "segments": [asdict(segment) for segment in result.segments],
    }
    (output_dir / "throttle_speed_calibration.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    csv_path = output_dir / "throttle_speed_calibration.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=list(asdict(result.points[0]).keys()))
        writer.writeheader()
        writer.writerows(asdict(point) for point in result.points)

    moving_points = [
        point for point in result.points
        if point.steady_speed_mps >= minimum_steady_speed_mps
    ]
    if not moving_points:
        raise RuntimeError(
            "no calibration point reached the minimum steady moving speed"
        )
    validate_monotonic(moving_points)
    speeds = [round(point.steady_speed_mps, 6) for point in moving_points]
    commands = [round(point.throttle, 6) for point in moving_points]
    minimum_command = min(commands)
    yaml_text = "\n".join(
        [
            "path_tracking_controller_node:",
            "  ros__parameters:",
            f"    throttle_feedforward_speeds_mps: {json.dumps(speeds)}",
            f"    throttle_feedforward_commands: {json.dumps(commands)}",
            f"    minimum_moving_throttle_command: {minimum_command:.6f}",
            "",
        ]
    )
    controller_path = output_dir / "controller_throttle_calibration.param.yaml"
    controller_path.write_text(yaml_text, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--command-topic", default="/vehicle/control_cmd")
    parser.add_argument("--odometry-topic", default="/visual_slam/tracking/odometry")
    parser.add_argument("--minimum-throttle", type=float, default=0.05)
    parser.add_argument("--throttle-tolerance", type=float, default=0.005)
    parser.add_argument("--maximum-steering", type=float, default=0.15)
    parser.add_argument("--minimum-segment-s", type=float, default=4.0)
    parser.add_argument("--settling-time-s", type=float, default=2.0)
    parser.add_argument("--steady-window-s", type=float, default=2.0)
    parser.add_argument("--maximum-speed-slope-mps2", type=float, default=0.10)
    parser.add_argument("--minimum-steady-speed-mps", type=float, default=0.05)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = AnalysisConfig(
        minimum_throttle=args.minimum_throttle,
        throttle_tolerance=args.throttle_tolerance,
        maximum_steering=args.maximum_steering,
        minimum_segment_s=args.minimum_segment_s,
        settling_time_s=args.settling_time_s,
        steady_window_s=args.steady_window_s,
        maximum_speed_slope_mps2=args.maximum_speed_slope_mps2,
    )
    try:
        samples = read_samples(
            args.bag,
            args.command_topic,
            args.odometry_topic,
            config.maximum_command_age_s,
        )
        result = analyze_samples(samples, config)
        if not result.points:
            raise RuntimeError("no stable fixed-throttle straight segments were found")
        default_output = args.bag.resolve().parent / f"{args.bag.stem}_throttle_calibration"
        output_dir = args.output_dir or default_output
        write_outputs(
            output_dir, args.bag, result, args.minimum_steady_speed_mps
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    accepted = sum(1 for segment in result.segments if segment.accepted)
    rejected = len(result.segments) - accepted
    print(
        f"Calibration points: {len(result.points)}; accepted runs: {accepted}; "
        f"rejected runs: {rejected}"
    )
    for point in result.points:
        print(
            f"  throttle {point.throttle:.3f} -> {point.steady_speed_mps:.3f} m/s "
            f"(MAD {point.speed_mad_mps:.3f}, runs {point.run_count})"
        )
    print(f"Output: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
