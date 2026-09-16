#!/usr/bin/env python3
"""Select reproducible low/median/high-density windows from a sorted EVSBIN file."""

from __future__ import annotations

import argparse
import csv
import json
import os
import struct
from functools import lru_cache
from pathlib import Path


HEADER = struct.Struct("<8sIIIIQQ24s")
TIMESTAMP = struct.Struct("<q")
MAGIC = b"EVSBENCH"
EVENT_BYTES = 16


class EvbinIndex:
    def __init__(self, path: Path):
        self.path = path
        self.stream = path.open("rb")
        raw_header = self.stream.read(HEADER.size)
        if len(raw_header) != HEADER.size:
            raise ValueError(f"truncated EVSBIN header: {path}")
        (
            magic, version, header_bytes, self.width, self.height,
            self.event_count, timestamp_unit_ns, _reserved,
        ) = HEADER.unpack(raw_header)
        if magic != MAGIC or version != 1 or header_bytes != HEADER.size:
            raise ValueError(f"invalid or unsupported EVSBIN header: {path}")
        if timestamp_unit_ns != 1000:
            raise ValueError(f"unsupported timestamp unit {timestamp_unit_ns} ns")
        expected_bytes = header_bytes + self.event_count * EVENT_BYTES
        actual_bytes = os.fstat(self.stream.fileno()).st_size
        if actual_bytes < expected_bytes:
            raise ValueError(
                f"truncated EVSBIN payload: expected {expected_bytes}, found {actual_bytes}"
            )
        self.header_bytes = header_bytes

    def close(self) -> None:
        self.stream.close()

    @lru_cache(maxsize=65536)
    def timestamp(self, index: int) -> int:
        if index < 0 or index >= self.event_count:
            raise IndexError(index)
        self.stream.seek(self.header_bytes + index * EVENT_BYTES)
        data = self.stream.read(TIMESTAMP.size)
        if len(data) != TIMESTAMP.size:
            raise ValueError(f"failed reading event {index} from {self.path}")
        return TIMESTAMP.unpack(data)[0]

    def lower_bound(self, target_us: int) -> int:
        left = 0
        right = self.event_count
        while left < right:
            middle = left + (right - left) // 2
            if self.timestamp(middle) < target_us:
                left = middle + 1
            else:
                right = middle
        return left


def percentile_index(length: int, fraction: float) -> int:
    if length <= 1:
        return 0
    return round((length - 1) * fraction)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--window-us", type=int, default=2_000_000)
    parser.add_argument("--step-us", type=int, default=100_000)
    parser.add_argument("--top", type=int, default=10)
    args = parser.parse_args()
    if args.window_us <= 0 or args.step_us <= 0 or args.top <= 0:
        parser.error("window, step and top must be positive")

    index = EvbinIndex(args.input)
    try:
        if index.event_count == 0:
            raise ValueError("EVSBIN contains no events")
        first_timestamp_us = index.timestamp(0)
        last_timestamp_us = index.timestamp(index.event_count - 1)
        span_us = last_timestamp_us - first_timestamp_us
        if span_us < args.window_us:
            raise ValueError(
                f"recording span {span_us} us is shorter than window {args.window_us} us"
            )

        windows: list[dict[str, int | float]] = []
        start_offset_us = 0
        while start_offset_us + args.window_us <= span_us:
            start_timestamp_us = first_timestamp_us + start_offset_us
            end_timestamp_us = start_timestamp_us + args.window_us
            begin_index = index.lower_bound(start_timestamp_us)
            end_index = index.lower_bound(end_timestamp_us)
            event_count = end_index - begin_index
            windows.append({
                "window_index": len(windows),
                "start_offset_us": start_offset_us,
                "start_timestamp_us": start_timestamp_us,
                "end_timestamp_us": end_timestamp_us,
                "begin_event_index": begin_index,
                "end_event_index": end_index,
                "event_count": event_count,
                "event_rate_meps": event_count / args.window_us,
            })
            start_offset_us += args.step_us
    finally:
        index.close()

    by_density = sorted(windows, key=lambda row: (int(row["event_count"]), int(row["window_index"])))
    low = by_density[percentile_index(len(by_density), 0.10)]
    median = by_density[percentile_index(len(by_density), 0.50)]
    high = by_density[-1]
    top_windows = list(reversed(by_density[-args.top:]))
    selection = {
        "input": str(args.input),
        "width": index.width,
        "height": index.height,
        "event_count": index.event_count,
        "first_timestamp_us": first_timestamp_us,
        "last_timestamp_us": last_timestamp_us,
        "span_us": span_us,
        "window_us": args.window_us,
        "step_us": args.step_us,
        "candidate_windows": len(windows),
        "selected": {"low_p10": low, "median_p50": median, "high_max": high},
        "top_high_density": top_windows,
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "density_windows.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(windows[0]))
        writer.writeheader()
        writer.writerows(windows)
    (args.output_dir / "density_selection.json").write_text(
        json.dumps(selection, indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps(selection["selected"], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
