#!/usr/bin/env python3
"""Summarize evs_bench CSV files using only the Python standard library."""

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


METRICS = (
    "wall_ms",
    "cpu_ms",
    "cpu_util_pct",
    "host_staging_ms",
    "h2d_ms",
    "update_ms",
    "snapshot_ms",
    "gpu_total_ms",
    "wall_us_per_snapshot",
    "host_staging_us_per_snapshot",
    "h2d_us_per_snapshot",
    "update_us_per_snapshot",
    "snapshot_us_per_snapshot",
    "gpu_total_us_per_snapshot",
    "effective_snapshot_hz",
    "effective_mev_s",
)


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("summary.csv"))
    args = parser.parse_args()
    groups: dict[tuple[str, str], dict[str, list[float]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for path in args.inputs:
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                key = (row["backend"], row["algorithm"])
                snapshots = float(row["snapshots"])
                wall_ms = float(row["wall_ms"])
                row["wall_us_per_snapshot"] = str(1000.0 * wall_ms / snapshots)
                for source in ("host_staging", "h2d", "update", "snapshot", "gpu_total"):
                    row[f"{source}_us_per_snapshot"] = str(
                        1000.0 * float(row[f"{source}_ms"]) / snapshots
                    )
                row["effective_snapshot_hz"] = str(1000.0 * snapshots / wall_ms)
                row["effective_mev_s"] = str(float(row["events"]) / wall_ms / 1000.0)
                for metric in METRICS:
                    groups[key][metric].append(float(row[metric]))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("backend", "algorithm", "metric", "n", "mean", "stdev", "p50", "p95", "p99"))
        for (backend, algorithm), values_by_metric in sorted(groups.items()):
            for metric in METRICS:
                values = values_by_metric[metric]
                writer.writerow(
                    (
                        backend,
                        algorithm,
                        metric,
                        len(values),
                        statistics.fmean(values),
                        statistics.stdev(values) if len(values) > 1 else 0.0,
                        percentile(values, 0.50),
                        percentile(values, 0.95),
                        percentile(values, 0.99),
                    )
                )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
