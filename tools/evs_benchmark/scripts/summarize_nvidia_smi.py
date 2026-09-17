#!/usr/bin/env python3
"""Summarize the nvidia-smi CSV emitted by run_condition.sh."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from pathlib import Path


def number(value: str) -> float | None:
    match = re.search(r"[-+]?[0-9]*\.?[0-9]+", value)
    return float(match.group(0)) if match else None


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summarize(values: list[float]) -> dict[str, float | None]:
    return {
        "mean": statistics.fmean(values) if values else None,
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values) if values else None,
    }


def processed_snapshots(metadata_path: Path | None, results_path: Path | None) -> int | None:
    if not metadata_path or not results_path or not metadata_path.exists() or not results_path.exists():
        return None
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    with results_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return None
    return int(rows[0]["snapshots"]) * (int(metadata.get("warmup_trials", 0)) + len(rows))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--interval-ms", type=float, default=100.0)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--results", type=Path)
    args = parser.parse_args()

    columns = {
        "utilization.gpu [%]": "gpu_util_pct",
        "utilization.memory [%]": "memory_util_pct",
        "power.draw [W]": "gpu_power_w",
        "clocks.current.sm [MHz]": "sm_clock_mhz",
        "temperature.gpu": "gpu_temp_c",
    }
    values: dict[str, list[float]] = {target: [] for target in columns.values()}
    with args.input.open(newline="", encoding="utf-8", errors="replace") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise SystemExit(f"empty nvidia-smi CSV: {args.input}")
        normalized = {name.strip(): name for name in reader.fieldnames}
        for row in reader:
            for source, target in columns.items():
                actual = normalized.get(source)
                if actual is None:
                    continue
                parsed = number(row.get(actual, ""))
                if parsed is not None:
                    values[target].append(parsed)

    samples = max((len(metric) for metric in values.values()), default=0)
    if samples == 0:
        raise SystemExit(f"no nvidia-smi samples found in {args.input}")
    power = values["gpu_power_w"]
    run_energy_j = sum(power) * args.interval_ms / 1000.0
    snapshots = processed_snapshots(args.metadata, args.results)
    report = {
        "input": str(args.input),
        "samples": samples,
        "sample_interval_ms": args.interval_ms,
        "sampled_duration_s": samples * args.interval_ms / 1000.0,
        "metrics": {key: summarize(metric) for key, metric in values.items()},
        "gpu_run_energy_j": run_energy_j,
        "processed_snapshots_including_warmup": snapshots,
        "gpu_energy_per_processed_snapshot_j": (
            run_energy_j / snapshots if snapshots and snapshots > 0 else None
        ),
        "energy_note": (
            "GPU board power only; unlike Jetson VDD_IN this excludes host system power. "
            "Integration covers setup and warm-up."
        ),
    }
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
