#!/usr/bin/env python3
"""Summarize timestamp anomalies emitted by evs_raw_decode_bench."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from statistics import fmean


EVT3_WRAP_PERIOD_US = 1 << 24


def percentile(sorted_values: list[int], percentile_value: float) -> float | None:
    if not sorted_values:
        return None
    position = (len(sorted_values) - 1) * percentile_value
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(sorted_values[lower])
    fraction = position - lower
    return sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction


def load_summary(path: Path | None) -> dict[str, str]:
    if path is None:
        return {}
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 1:
        raise ValueError(f"summary CSV must contain exactly one trial: {path}")
    return rows[0]


def load_anomalies(path: Path) -> list[dict[str, int | str]]:
    required = {
        "kind", "callback_index", "event_index", "at_callback_boundary",
        "previous_timestamp_us", "current_timestamp_us", "delta_us",
        "distance_to_evt3_wrap_us",
    }
    records: list[dict[str, int | str]] = []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"anomaly CSV is missing columns: {sorted(missing)}")
        for row in reader:
            records.append({
                "kind": row["kind"],
                "callback_index": int(row["callback_index"]),
                "event_index": int(row["event_index"]),
                "at_callback_boundary": int(row["at_callback_boundary"]),
                "previous_timestamp_us": int(row["previous_timestamp_us"]),
                "current_timestamp_us": int(row["current_timestamp_us"]),
                "delta_us": int(row["delta_us"]),
                "distance_to_evt3_wrap_us": int(row["distance_to_evt3_wrap_us"]),
            })
    return records


def magnitude_buckets(magnitudes: list[int]) -> dict[str, int]:
    limits = [16, 64, 256, 1000, 4000, 4096, 8000, 8192]
    labels = [
        "0_16", "17_64", "65_256", "257_1000", "1001_4000",
        "4001_4096", "4097_8000", "8001_8192", "over_8192",
    ]
    counts = {label: 0 for label in labels}
    for magnitude in magnitudes:
        for index, limit in enumerate(limits):
            if magnitude <= limit:
                counts[labels[index]] += 1
                break
        else:
            counts[labels[-1]] += 1
    return counts


def build_clusters(
    backward: list[dict[str, int | str]], first_timestamp_us: int, cluster_gap_us: int,
    wrap_proximity_us: int,
) -> list[dict[str, int | float]]:
    clusters: list[dict[str, int | float]] = []
    active: list[dict[str, int | str]] = []
    previous_time: int | None = None

    def finish() -> None:
        nonlocal active
        if not active:
            return
        times = [
            max(int(row["previous_timestamp_us"]), int(row["current_timestamp_us"]))
            for row in active
        ]
        magnitudes = [-int(row["delta_us"]) for row in active]
        clusters.append({
            "cluster_id": len(clusters),
            "start_elapsed_us": min(times) - first_timestamp_us,
            "end_elapsed_us": max(times) - first_timestamp_us,
            "duration_us": max(times) - min(times),
            "start_event_index": int(active[0]["event_index"]),
            "end_event_index": int(active[-1]["event_index"]),
            "backward_events": len(active),
            "affected_callbacks": len({int(row["callback_index"]) for row in active}),
            "max_backward_us": max(magnitudes),
            "mean_backward_us": fmean(magnitudes),
            "over_4000_us": sum(value > 4000 for value in magnitudes),
            "near_evt3_wrap": sum(
                int(row["distance_to_evt3_wrap_us"]) <= wrap_proximity_us for row in active
            ),
        })
        active = []

    for row in sorted(backward, key=lambda item: int(item["event_index"])):
        anomaly_time = max(
            int(row["previous_timestamp_us"]), int(row["current_timestamp_us"])
        )
        if previous_time is not None and (
            anomaly_time < previous_time - cluster_gap_us or
            anomaly_time > previous_time + cluster_gap_us
        ):
            finish()
        active.append(row)
        previous_time = anomaly_time
    finish()
    return clusters


def estimate_snapshot_exposure(
    backward: list[dict[str, int | str]], first_timestamp_us: int, last_timestamp_us: int,
    window_us: int, stride_us: int,
) -> dict[str, dict[str, float | int]]:
    total_snapshots = max(0, (last_timestamp_us - first_timestamp_us) // stride_us + 1)
    thresholds = {"all": 0, "over_1000_us": 1000, "over_4000_us": 4000,
                  "over_8000_us": 8000}
    result: dict[str, dict[str, float | int]] = {}
    for name, threshold in thresholds.items():
        affected: set[int] = set()
        for row in backward:
            if -int(row["delta_us"]) <= threshold and name != "all":
                continue
            anomaly_time = max(
                int(row["previous_timestamp_us"]), int(row["current_timestamp_us"])
            )
            relative = anomaly_time - first_timestamp_us
            first_index = max(0, math.ceil(relative / stride_us))
            last_index = min(
                total_snapshots - 1, math.floor((relative + window_us) / stride_us)
            )
            if last_index >= first_index:
                affected.update(range(first_index, last_index + 1))
        result[name] = {
            "affected_snapshots": len(affected),
            "total_snapshots": total_snapshots,
            "affected_fraction": len(affected) / total_snapshots if total_snapshots else 0.0,
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--anomalies", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--hal-log", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--wrap-proximity-us", type=int, default=10000)
    parser.add_argument("--cluster-gap-us", type=int, default=1000)
    parser.add_argument("--timeline-bin-us", type=int, default=100000)
    parser.add_argument("--window-us", type=int, default=40000)
    parser.add_argument("--stride-us", type=int, default=4000)
    args = parser.parse_args()

    for name in ("wrap_proximity_us", "cluster_gap_us", "timeline_bin_us", "window_us", "stride_us"):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")

    records = load_anomalies(args.anomalies)
    summary_row = load_summary(args.summary)
    backward = [row for row in records if row["kind"] == "backward"]
    forward_gaps = [row for row in records if row["kind"] == "forward_gap"]
    magnitudes = sorted(-int(row["delta_us"]) for row in backward)

    if summary_row:
        first_timestamp_us = int(summary_row["first_timestamp_us"])
        last_timestamp_us = int(summary_row["last_timestamp_us"])
    elif records:
        timestamps = [
            value for row in records for value in (
                int(row["previous_timestamp_us"]), int(row["current_timestamp_us"])
            )
        ]
        first_timestamp_us = min(timestamps)
        last_timestamp_us = max(timestamps)
    else:
        first_timestamp_us = 0
        last_timestamp_us = 0

    callback_counts = Counter(int(row["callback_index"]) for row in backward)
    near_wrap = sum(
        int(row["distance_to_evt3_wrap_us"]) <= args.wrap_proximity_us
        for row in backward
    )
    hal_violations = None
    if args.hal_log:
        hal_violations = args.hal_log.read_bytes().count(b"NonMonotonicTimeHigh")

    clusters = build_clusters(
        backward, first_timestamp_us, args.cluster_gap_us, args.wrap_proximity_us
    )
    histogram = Counter(magnitudes)
    timeline: dict[int, dict[str, int]] = defaultdict(
        lambda: {"backward_events": 0, "over_4000_us": 0, "near_evt3_wrap": 0,
                 "max_backward_us": 0}
    )
    for row in backward:
        anomaly_time = max(
            int(row["previous_timestamp_us"]), int(row["current_timestamp_us"])
        )
        bin_index = max(0, (anomaly_time - first_timestamp_us) // args.timeline_bin_us)
        magnitude = -int(row["delta_us"])
        item = timeline[bin_index]
        item["backward_events"] += 1
        item["over_4000_us"] += magnitude > 4000
        item["near_evt3_wrap"] += (
            int(row["distance_to_evt3_wrap_us"]) <= args.wrap_proximity_us
        )
        item["max_backward_us"] = max(item["max_backward_us"], magnitude)

    report = {
        "input": {
            "anomalies": str(args.anomalies),
            "summary": str(args.summary) if args.summary else None,
            "hal_log": str(args.hal_log) if args.hal_log else None,
        },
        "configuration": {
            "evt3_wrap_period_us": EVT3_WRAP_PERIOD_US,
            "wrap_proximity_us": args.wrap_proximity_us,
            "cluster_gap_us": args.cluster_gap_us,
            "timeline_bin_us": args.timeline_bin_us,
            "window_us": args.window_us,
            "stride_us": args.stride_us,
        },
        "timestamp_range": {
            "first_timestamp_us": first_timestamp_us,
            "last_timestamp_us": last_timestamp_us,
            "span_us": max(0, last_timestamp_us - first_timestamp_us),
        },
        "backward": {
            "events": len(backward),
            "affected_callbacks": len(callback_counts),
            "mean_per_affected_callback": (
                len(backward) / len(callback_counts) if callback_counts else 0.0
            ),
            "max_per_affected_callback": max(callback_counts.values(), default=0),
            "at_callback_boundary": sum(int(row["at_callback_boundary"]) for row in backward),
            "within_callback": sum(not int(row["at_callback_boundary"]) for row in backward),
            "near_evt3_wrap": near_wrap,
            "near_evt3_wrap_fraction": near_wrap / len(backward) if backward else 0.0,
            "over_1000_us": sum(value > 1000 for value in magnitudes),
            "over_4000_us": sum(value > 4000 for value in magnitudes),
            "over_4096_us": sum(value > 4096 for value in magnitudes),
            "over_8000_us": sum(value > 8000 for value in magnitudes),
            "magnitude_us": {
                "min": min(magnitudes, default=None),
                "mean": fmean(magnitudes) if magnitudes else None,
                "p50": percentile(magnitudes, 0.50),
                "p90": percentile(magnitudes, 0.90),
                "p95": percentile(magnitudes, 0.95),
                "p99": percentile(magnitudes, 0.99),
                "p999": percentile(magnitudes, 0.999),
                "max": max(magnitudes, default=None),
            },
            "buckets": magnitude_buckets(magnitudes),
        },
        "forward_gaps": {"events": len(forward_gaps)},
        "clusters": {
            "count": len(clusters),
            "largest_backward_events": max(
                (int(cluster["backward_events"]) for cluster in clusters), default=0
            ),
            "longest_duration_us": max(
                (int(cluster["duration_us"]) for cluster in clusters), default=0
            ),
        },
        "hal_non_monotonic_time_high": hal_violations,
        "estimated_snapshot_exposure": estimate_snapshot_exposure(
            backward, first_timestamp_us, last_timestamp_us, args.window_us, args.stride_us
        ),
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "timestamp_anomaly_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n"
    )
    with (args.output_dir / "backward_magnitude_histogram.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["backward_us", "count"])
        writer.writerows(sorted(histogram.items()))
    with (args.output_dir / "timestamp_anomaly_timeline.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "bin_index", "start_elapsed_us", "end_elapsed_us", "backward_events",
            "over_4000_us", "near_evt3_wrap", "max_backward_us",
        ])
        for bin_index, item in sorted(timeline.items()):
            writer.writerow([
                bin_index, bin_index * args.timeline_bin_us,
                (bin_index + 1) * args.timeline_bin_us, item["backward_events"],
                item["over_4000_us"], item["near_evt3_wrap"], item["max_backward_us"],
            ])
    with (args.output_dir / "timestamp_anomaly_clusters.csv").open("w", newline="") as stream:
        fieldnames = list(clusters[0]) if clusters else [
            "cluster_id", "start_elapsed_us", "end_elapsed_us", "duration_us",
            "start_event_index", "end_event_index", "backward_events", "affected_callbacks",
            "max_backward_us", "mean_backward_us", "over_4000_us", "near_evt3_wrap",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(clusters)

    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
