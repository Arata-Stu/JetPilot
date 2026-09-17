#!/usr/bin/env python3
"""Summarize snapshot traces and create a dependency-free SVG timeline."""

from __future__ import annotations

import argparse
import csv
import html
import math
import statistics
from collections import defaultdict
from pathlib import Path


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def points(rows: list[dict[str, str]], x_key: str, y_key: str, box: tuple[float, ...]) -> str:
    left, top, width, height, x_min, x_max, y_max = box
    coordinates = []
    for row in rows:
        x_value = float(row[x_key])
        y_value = float(row[y_key])
        x = left if x_max == x_min else left + width * (x_value - x_min) / (x_max - x_min)
        y = top + height if y_max <= 0 else top + height * (1.0 - min(y_value, y_max) / y_max)
        coordinates.append(f"{x:.2f},{y:.2f}")
    return " ".join(coordinates)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--svg", type=Path, required=True)
    parser.add_argument("--deadline-ms", type=float, default=4.0)
    args = parser.parse_args()

    groups: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for path in args.inputs:
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                groups[(row["backend"], row["algorithm"])].append(row)
    if not groups:
        raise SystemExit("no trace rows")

    args.summary.parent.mkdir(parents=True, exist_ok=True)
    with args.summary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ("backend", "algorithm", "snapshots", "metric", "mean", "p50", "p95", "p99", "max", "deadline_misses")
        )
        for (backend, algorithm), rows in sorted(groups.items()):
            for metric in ("wall_ms", "host_staging_ms", "h2d_ms", "update_ms", "snapshot_ms", "event_rate_mev_s"):
                values = [float(row[metric]) for row in rows]
                writer.writerow(
                    (
                        backend, algorithm, len(rows), metric, statistics.fmean(values),
                        percentile(values, 0.50), percentile(values, 0.95),
                        percentile(values, 0.99), max(values),
                        sum(value > args.deadline_ms for value in values) if metric == "wall_ms" else "",
                    )
                )

    all_rows = [row for rows in groups.values() for row in rows]
    x_min = min(float(row["relative_time_ms"]) for row in all_rows)
    x_max = max(float(row["relative_time_ms"]) for row in all_rows)
    rate_max = max(float(row["event_rate_mev_s"]) for row in all_rows) * 1.05
    latency_max = max(
        args.deadline_ms * 1.1,
        percentile([float(row["wall_ms"]) for row in all_rows], 0.99) * 1.15,
    )
    width, height = 1200, 720
    left, plot_width = 90, 1060
    colors = ["#35a7ff", "#ff6b6b", "#8ac926", "#ffca3a"]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#0d1117"/>',
        '<style>text{font-family:monospace;fill:#d8dee9}.axis{stroke:#6e7681;stroke-width:1}.grid{stroke:#30363d;stroke-width:1}.deadline{stroke:#ffca3a;stroke-dasharray:7 5}</style>',
        '<text x="30" y="35" font-size="22">EVS snapshot timeline</text>',
    ]
    panels = [
        (70.0, 245.0, rate_max, "Event rate (Mev/s)"),
        (390.0, 245.0, latency_max, "Snapshot latency (ms)"),
    ]
    for top, panel_height, y_max, label in panels:
        svg.append(f'<text x="30" y="{top - 12}" font-size="16">{html.escape(label)}</text>')
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = top + panel_height * (1.0 - fraction)
            svg.append(f'<line class="grid" x1="{left}" y1="{y}" x2="{left + plot_width}" y2="{y}"/>')
            svg.append(f'<text x="15" y="{y + 5}" font-size="12">{y_max * fraction:.2f}</text>')
        svg.append(f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top + panel_height}"/>')
        svg.append(f'<line class="axis" x1="{left}" y1="{top + panel_height}" x2="{left + plot_width}" y2="{top + panel_height}"/>')
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        x = left + plot_width * fraction
        time_s = (x_min + (x_max - x_min) * fraction) / 1000.0
        svg.append(f'<line class="grid" x1="{x}" y1="70" x2="{x}" y2="635"/>')
        svg.append(f'<text x="{x - 22}" y="655" font-size="12">{time_s:.2f}</text>')

    representative_rows = next(iter(groups.values()))
    rate_box = (left, 70.0, plot_width, 245.0, x_min, x_max, rate_max)
    svg.append(f'<polyline fill="none" stroke="#9b5de5" stroke-width="1" points="{points(representative_rows, "relative_time_ms", "event_rate_mev_s", rate_box)}"/>')
    latency_box = (left, 390.0, plot_width, 245.0, x_min, x_max, latency_max)
    deadline_y = 390.0 + 245.0 * (1.0 - min(args.deadline_ms, latency_max) / latency_max)
    svg.append(f'<line class="deadline" x1="{left}" y1="{deadline_y}" x2="{left + plot_width}" y2="{deadline_y}"/>')
    svg.append(f'<text x="{left + plot_width - 145}" y="{deadline_y - 7}" font-size="12">deadline {args.deadline_ms:.1f} ms</text>')
    for index, ((backend, algorithm), rows) in enumerate(sorted(groups.items())):
        color = colors[index % len(colors)]
        label = f"{backend}/{algorithm}"
        svg.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.2" points="{points(rows, "relative_time_ms", "wall_ms", latency_box)}"/>')
        svg.append(f'<line x1="{left + index * 230}" y1="680" x2="{left + 30 + index * 230}" y2="680" stroke="{color}" stroke-width="3"/>')
        svg.append(f'<text x="{left + 38 + index * 230}" y="685" font-size="13">{html.escape(label)}</text>')
    svg.append(f'<text x="{left + plot_width / 2 - 60}" y="710" font-size="14">sensor time (s)</text>')
    svg.append('</svg>')
    args.svg.parent.mkdir(parents=True, exist_ok=True)
    args.svg.write_text("\n".join(svg) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
