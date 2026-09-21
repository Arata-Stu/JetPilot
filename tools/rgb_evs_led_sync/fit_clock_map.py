#!/usr/bin/env python3
"""Fit an affine RGB-to-EVS clock map from manually paired LED edges."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean, median


@dataclass(frozen=True)
class Edge:
    marker: str
    edge: int
    state: str
    rgb_before_s: float
    rgb_after_s: float
    evs_time_s: float

    @property
    def rgb_mid_s(self) -> float:
        return 0.5 * (self.rgb_before_s + self.rgb_after_s)

    @property
    def rgb_half_interval_s(self) -> float:
        return 0.5 * (self.rgb_after_s - self.rgb_before_s)


def load_edges(path: Path) -> list[Edge]:
    required = {
        "marker",
        "edge",
        "state",
        "rgb_before_s",
        "rgb_after_s",
        "evs_time_s",
    }
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f"missing CSV column(s): {', '.join(sorted(missing))}")
        edges = [
            Edge(
                marker=row["marker"].strip().lower(),
                edge=int(row["edge"]),
                state=row["state"].strip().lower(),
                rgb_before_s=float(row["rgb_before_s"]),
                rgb_after_s=float(row["rgb_after_s"]),
                evs_time_s=float(row["evs_time_s"]),
            )
            for row in reader
        ]

    if len(edges) < 2:
        raise ValueError("at least two paired edges are required")
    for item in edges:
        if item.marker not in {"start", "end"}:
            raise ValueError(f"marker must be start or end: {item.marker!r}")
        if item.state not in {"on", "off"}:
            raise ValueError(f"state must be on or off: {item.state!r}")
        values = (
            item.rgb_before_s,
            item.rgb_after_s,
            item.evs_time_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("timestamps must be finite")
        if item.rgb_after_s <= item.rgb_before_s:
            raise ValueError(
                f"rgb_after_s must be later than rgb_before_s at "
                f"{item.marker} edge {item.edge}"
            )
    return edges


def fit_affine(edges: list[Edge]) -> dict[str, object]:
    x_values = [item.rgb_mid_s for item in edges]
    y_values = [item.evs_time_s for item in edges]
    x_mean = fmean(x_values)
    y_mean = fmean(y_values)
    denominator = sum((value - x_mean) ** 2 for value in x_values)
    if denominator <= 0.0:
        raise ValueError("RGB edge times must span more than one timestamp")

    scale = sum(
        (x_value - x_mean) * (y_value - y_mean)
        for x_value, y_value in zip(x_values, y_values)
    ) / denominator
    offset_s = y_mean - scale * x_mean
    residuals_s = [
        y_value - (scale * x_value + offset_s)
        for x_value, y_value in zip(x_values, y_values)
    ]
    rmse_s = math.sqrt(fmean(value * value for value in residuals_s))
    max_abs_s = max(abs(value) for value in residuals_s)

    marker_offsets_ms: dict[str, float | None] = {}
    marker_counts: dict[str, int] = {}
    for marker in ("start", "end"):
        selected = [item for item in edges if item.marker == marker]
        marker_counts[marker] = len(selected)
        marker_offsets_ms[marker] = (
            1000.0 * median(item.evs_time_s - item.rgb_mid_s for item in selected)
            if selected
            else None
        )

    max_half_interval_s = max(item.rgb_half_interval_s for item in edges)
    warnings: list[str] = []
    if marker_counts["start"] < 4 or marker_counts["end"] < 4:
        warnings.append(
            "Use at least four matched edges at both start and end before claiming drift."
        )
    if max_abs_s > 2.0 * max_half_interval_s:
        warnings.append(
            "Maximum residual exceeds one full RGB transition interval; re-check edge pairing."
        )
    if abs((scale - 1.0) * 1_000_000.0) > 1000.0:
        warnings.append(
            "Clock drift exceeds 1000 ppm; inspect timestamp domains, drops, and restarts."
        )

    return {
        "schema_version": 1,
        "model": "evs_time_s = scale * rgb_time_s + offset_s",
        "edge_count": len(edges),
        "marker_counts": marker_counts,
        "scale": scale,
        "offset_s": offset_s,
        "drift_ppm": (scale - 1.0) * 1_000_000.0,
        "rmse_ms": rmse_s * 1000.0,
        "max_abs_residual_ms": max_abs_s * 1000.0,
        "max_rgb_half_interval_ms": max_half_interval_s * 1000.0,
        "start_median_raw_offset_ms": marker_offsets_ms["start"],
        "end_median_raw_offset_ms": marker_offsets_ms["end"],
        "warnings": warnings,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--edges", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = fit_affine(load_edges(args.edges))
    payload = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
