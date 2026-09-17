#!/usr/bin/env python3
"""Convert tegrastats text into a timeline CSV and a compact JSON summary."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from pathlib import Path


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


def match_float(pattern: str, line: str) -> float | None:
    match = re.search(pattern, line)
    return float(match.group(1)) if match else None


def parse_line(line: str, sample_index: int, interval_ms: float) -> dict[str, object] | None:
    if "RAM " not in line:
        return None
    timestamp_match = re.match(r"(\d{2}-\d{2}-\d{4}\s+\d{2}:\d{2}:\d{2})", line)
    cpu_match = re.search(r"CPU \[([^]]*)\]", line)
    cpu_values: list[float] = []
    cpu_frequencies: list[float] = []
    if cpu_match:
        for utilization, frequency in re.findall(r"(\d+)%@(\d+)", cpu_match.group(1)):
            cpu_values.append(float(utilization))
            cpu_frequencies.append(float(frequency))
    return {
        "sample_index": sample_index,
        "relative_time_s": sample_index * interval_ms / 1000.0,
        "timestamp": timestamp_match.group(1) if timestamp_match else "",
        "ram_used_mb": match_float(r"RAM\s+(\d+)/\d+MB", line),
        "cpu_mean_pct": statistics.fmean(cpu_values) if cpu_values else None,
        "cpu_max_pct": max(cpu_values) if cpu_values else None,
        "cpu_mean_mhz": statistics.fmean(cpu_frequencies) if cpu_frequencies else None,
        "gr3d_pct": match_float(r"GR3D_FREQ\s+(\d+)%", line),
        "cpu_temp_c": match_float(r"\bcpu@([0-9.]+)C", line),
        "gpu_temp_c": match_float(r"\bgpu@([0-9.]+)C", line),
        "vdd_in_mw": match_float(r"VDD_IN\s+(\d+)mW", line),
        "vdd_cpu_gpu_cv_mw": match_float(r"VDD_CPU_GPU_CV\s+(\d+)mW", line),
        "vdd_soc_mw": match_float(r"VDD_SOC\s+(\d+)mW", line),
    }


def metric_summary(rows: list[dict[str, object]], key: str) -> dict[str, float | None]:
    values = [float(row[key]) for row in rows if row.get(key) is not None]
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
    snapshots_per_trial = int(rows[0]["snapshots"])
    return snapshots_per_trial * (int(metadata.get("warmup_trials", 0)) + len(rows))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--timeline", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--interval-ms", type=float, default=100.0)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--results", type=Path)
    args = parser.parse_args()
    if args.interval_ms <= 0:
        parser.error("--interval-ms must be positive")

    rows = []
    for line in args.input.read_text(encoding="utf-8", errors="replace").splitlines():
        parsed = parse_line(line, len(rows), args.interval_ms)
        if parsed is not None:
            rows.append(parsed)
    if not rows:
        raise SystemExit(f"no tegrastats samples found in {args.input}")

    args.timeline.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0])
    with args.timeline.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    power_values = [float(row["vdd_in_mw"]) for row in rows if row.get("vdd_in_mw") is not None]
    run_energy_j = sum(power_values) * args.interval_ms / 1_000_000.0
    snapshots = processed_snapshots(args.metadata, args.results)
    metrics = (
        "ram_used_mb", "cpu_mean_pct", "cpu_max_pct", "cpu_mean_mhz", "gr3d_pct",
        "cpu_temp_c", "gpu_temp_c", "vdd_in_mw", "vdd_cpu_gpu_cv_mw", "vdd_soc_mw",
    )
    summary = {
        "input": str(args.input),
        "samples": len(rows),
        "sample_interval_ms": args.interval_ms,
        "sampled_duration_s": len(rows) * args.interval_ms / 1000.0,
        "metrics": {metric: metric_summary(rows, metric) for metric in metrics},
        "run_energy_j": run_energy_j,
        "processed_snapshots_including_warmup": snapshots,
        "energy_per_processed_snapshot_j": (
            run_energy_j / snapshots if snapshots and snapshots > 0 else None
        ),
        "energy_note": (
            "tegrastats interval integration over the complete process lifetime; includes setup "
            "and warm-up and is an end-to-end estimate"
        ),
    }
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
