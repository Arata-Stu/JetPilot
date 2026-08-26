from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Sequence

from object_detection_learning.contract import dataset_root, load_dataset


def _split_path(
    dataset_path: Path, dataset: dict[object, object], split: str
) -> Path | None:
    raw = str(dataset.get(split) or "").strip()
    if not raw:
        return None
    candidate = Path(raw).expanduser()
    if not candidate.is_absolute():
        candidate = dataset_root(dataset_path, dataset) / candidate
    return candidate.resolve()


def _label_directory(image_directory: Path) -> Path:
    parts = list(image_directory.parts)
    for index in range(len(parts) - 1, -1, -1):
        if parts[index] == "images":
            parts[index] = "labels"
            return Path(*parts)
    return image_directory.parent / "labels"


def _validate_label(path: Path, class_count: int) -> int:
    annotations = 0
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8", errors="replace").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_number}: expected 5 YOLO fields")
        try:
            class_id = int(fields[0])
            coordinates = [float(value) for value in fields[1:]]
        except ValueError as error:
            raise ValueError(f"{path}:{line_number}: invalid numeric value") from error
        if class_id < 0 or class_id >= class_count:
            raise ValueError(f"{path}:{line_number}: class id {class_id} is out of range")
        if any(not math.isfinite(value) or value < 0.0 or value > 1.0 for value in coordinates):
            raise ValueError(
                f"{path}:{line_number}: normalized box values must be finite and in [0, 1]"
            )
        if coordinates[2] <= 0.0 or coordinates[3] <= 0.0:
            raise ValueError(f"{path}:{line_number}: box width and height must be positive")
        annotations += 1
    return annotations


def validate(path: Path, *, scan_labels: bool = True) -> dict[str, object]:
    dataset_path, dataset, classes = load_dataset(path)
    split_summary: dict[str, object] = {}
    for split in ("train", "val", "test"):
        image_directory = _split_path(dataset_path, dataset, split)
        if image_directory is None:
            if split in {"train", "val"}:
                raise ValueError(f"Dataset YAML is missing the required {split} split")
            continue
        if not image_directory.is_dir():
            raise FileNotFoundError(f"{split} image directory was not found: {image_directory}")
        images = [
            item
            for item in image_directory.rglob("*")
            if item.is_file() and item.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
        ]
        labels = 0
        annotations = 0
        label_directory = _label_directory(image_directory)
        if scan_labels:
            if not label_directory.is_dir():
                raise FileNotFoundError(f"{split} label directory was not found: {label_directory}")
            for label in label_directory.rglob("*.txt"):
                labels += 1
                annotations += _validate_label(label, len(classes))
        split_summary[split] = {
            "image_directory": str(image_directory),
            "label_directory": str(label_directory),
            "images": len(images),
            "label_files": labels,
            "annotations": annotations,
        }
    return {
        "dataset_yaml": str(dataset_path),
        "dataset_root": str(dataset_root(dataset_path, dataset)),
        "classes": classes,
        "splits": split_summary,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate the JetPilot YOLOv8 dataset contract")
    parser.add_argument("--data", type=Path, required=True, help="Roboflow YOLOv8 data.yaml")
    parser.add_argument(
        "--skip-label-scan",
        action="store_true",
        help="Validate paths and class order without scanning every label file",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        summary = validate(args.data, scan_labels=not args.skip_label_scan)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
