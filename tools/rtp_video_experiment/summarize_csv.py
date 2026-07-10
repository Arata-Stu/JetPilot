#!/usr/bin/env python3
"""Summarize numeric CSV columns."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * q
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--column", required=True)
    args = parser.parse_args()

    values: list[float] = []
    with args.csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if args.column not in (reader.fieldnames or []):
            raise SystemExit(f"column was not found: {args.column}")
        for row in reader:
            text = row.get(args.column, "")
            if text == "":
                continue
            try:
                values.append(float(text))
            except ValueError:
                continue

    if not values:
        raise SystemExit(f"no numeric values in column: {args.column}")

    stats = {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "stddev": statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
    }
    for key, value in stats.items():
        print(f"{key},{value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

