#!/usr/bin/env python3
"""Create and update a dependency-free RC pop-out experiment plan."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import os
import random
import tempfile
from datetime import datetime
from pathlib import Path


FIELDS = (
    "run_id",
    "block_id",
    "speed",
    "direction",
    "obstacle_position",
    "camera_position",
    "repetition",
    "status",
    "recorded_at",
    "note",
)
VALID_STATUSES = {"pending", "completed", "retry", "skipped"}


def split_levels(value: str, name: str) -> list[str]:
    levels = [item.strip() for item in value.split(",") if item.strip()]
    if not levels:
        raise ValueError(f"{name} must contain at least one value")
    if len(levels) != len(set(levels)):
        raise ValueError(f"{name} contains duplicate values")
    return levels


def read_plan(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream, delimiter="\t")
        if tuple(reader.fieldnames or ()) != FIELDS:
            raise ValueError(f"unexpected plan columns: {reader.fieldnames}")
        return list(reader)


def write_plan(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS, delimiter="\t")
            writer.writeheader()
            writer.writerows(rows)
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def initialize(args: argparse.Namespace) -> None:
    plan_path = Path(args.plan)
    if plan_path.exists() and not args.force:
        raise FileExistsError(f"plan already exists: {plan_path}")

    speeds = split_levels(args.speeds, "speeds")
    directions = split_levels(args.directions, "directions")
    obstacle_positions = split_levels(args.obstacle_positions, "obstacle positions")
    camera_positions = split_levels(args.camera_positions, "camera positions")
    if args.repetitions < 1:
        raise ValueError("repetitions must be at least 1")

    rng = random.Random(args.seed)
    rows: list[dict[str, str]] = []
    run_number = 1
    block_number = 1
    # Camera and obstacle placement are outer blocks, reducing physical setup changes.
    for camera_position, obstacle_position in itertools.product(
        camera_positions, obstacle_positions
    ):
        trials = list(itertools.product(speeds, directions, range(1, args.repetitions + 1)))
        rng.shuffle(trials)
        for speed, direction, repetition in trials:
            rows.append(
                {
                    "run_id": f"r{run_number:03d}",
                    "block_id": f"b{block_number:02d}",
                    "speed": speed,
                    "direction": direction,
                    "obstacle_position": obstacle_position,
                    "camera_position": camera_position,
                    "repetition": str(repetition),
                    "status": "pending",
                    "recorded_at": "",
                    "note": "",
                }
            )
            run_number += 1
        block_number += 1

    write_plan(plan_path, rows)
    metadata = {
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "plan_file": str(plan_path),
        "ordering": "camera_position/obstacle_position blocks; randomized within each block",
        "seed": args.seed,
        "speeds": speeds,
        "directions": directions,
        "obstacle_positions": obstacle_positions,
        "camera_positions": camera_positions,
        "repetitions": args.repetitions,
        "total_runs": len(rows),
        "sensor_profile": {
            "bringup_preset": "rc-popout",
            "rgb": "848x480@60",
            "infra": False,
            "depth": False,
            "imu": False,
            "evs_bias_file": "",
            "evs_recording": "OpenEB native RAW",
        },
    }
    metadata_path = plan_path.with_name("experiment_metadata.json")
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"runs": len(rows), "plan": str(plan_path)}, ensure_ascii=False))


def next_run(args: argparse.Namespace) -> None:
    for row in read_plan(Path(args.plan)):
        if row["status"] in {"pending", "retry"}:
            print(json.dumps(row, ensure_ascii=False))
            return
    raise SystemExit(3)


def update(args: argparse.Namespace) -> None:
    if args.status not in VALID_STATUSES:
        raise ValueError(f"invalid status: {args.status}")
    plan_path = Path(args.plan)
    rows = read_plan(plan_path)
    matches = [row for row in rows if row["run_id"] == args.run_id]
    if len(matches) != 1:
        raise ValueError(f"run_id must match exactly one row: {args.run_id}")
    row = matches[0]
    row["status"] = args.status
    row["recorded_at"] = (
        datetime.now().astimezone().isoformat(timespec="seconds")
        if args.status == "completed"
        else ""
    )
    row["note"] = args.note
    write_plan(plan_path, rows)


def status(args: argparse.Namespace) -> None:
    rows = read_plan(Path(args.plan))
    counts = {key: 0 for key in VALID_STATUSES}
    for row in rows:
        counts[row["status"]] = counts.get(row["status"], 0) + 1
    result = {"total": len(rows), **counts}
    print(json.dumps(result, ensure_ascii=False))


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser()
    subcommands = root.add_subparsers(dest="command", required=True)

    init = subcommands.add_parser("init")
    init.add_argument("--plan", required=True)
    init.add_argument("--speeds", required=True)
    init.add_argument("--directions", required=True)
    init.add_argument("--obstacle-positions", required=True)
    init.add_argument("--camera-positions", required=True)
    init.add_argument("--repetitions", required=True, type=int)
    init.add_argument("--seed", required=True, type=int)
    init.add_argument("--force", action="store_true")
    init.set_defaults(handler=initialize)

    next_parser = subcommands.add_parser("next")
    next_parser.add_argument("--plan", required=True)
    next_parser.set_defaults(handler=next_run)

    update_parser = subcommands.add_parser("update")
    update_parser.add_argument("--plan", required=True)
    update_parser.add_argument("--run-id", required=True)
    update_parser.add_argument("--status", required=True)
    update_parser.add_argument("--note", default="")
    update_parser.set_defaults(handler=update)

    status_parser = subcommands.add_parser("status")
    status_parser.add_argument("--plan", required=True)
    status_parser.set_defaults(handler=status)
    return root


def main() -> None:
    args = parser().parse_args()
    args.handler(args)


if __name__ == "__main__":
    main()
