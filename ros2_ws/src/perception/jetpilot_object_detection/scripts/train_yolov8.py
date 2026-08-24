#!/usr/bin/env python3
"""Train and export the small JetPilot YOLOv8 model on the x86 training image."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path


EXPECTED_CLASSES = ["vehicle", "barrier"]


def _classes(dataset: dict[object, object]) -> list[str]:
    names = dataset.get("names")
    if isinstance(names, list):
        return [str(item) for item in names]
    if isinstance(names, dict):
        indexed: list[tuple[int, str]] = []
        for key, value in names.items():
            try:
                index = int(key)
            except (TypeError, ValueError) as error:
                raise ValueError(f"Dataset class id is not an integer: {key}") from error
            indexed.append((index, str(value)))
        indexed.sort()
        if [index for index, _ in indexed] != list(range(len(indexed))):
            raise ValueError("Dataset class ids must be contiguous and start at 0")
        return [name for _, name in indexed]
    raise ValueError("Dataset YAML must contain a names list or mapping")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True, help="Roboflow YOLOv8 data.yaml")
    parser.add_argument("--model", default="yolov8n.pt", help="Initial Ultralytics weights")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--project", type=Path, default=Path("runs/jetpilot_detection"))
    parser.add_argument("--name", default="yolov8n_224")
    parser.add_argument(
        "--classes",
        nargs="+",
        default=EXPECTED_CLASSES,
        help="Expected labels in exact output-channel order",
    )
    parser.add_argument("--no-export", action="store_true")
    args = parser.parse_args()

    if args.epochs <= 0 or args.batch <= 0 or args.workers < 0:
        parser.error("epochs/batch must be positive and workers must be non-negative")
    data_path = args.data.expanduser().resolve()
    if not data_path.is_file():
        parser.error(f"Dataset YAML was not found: {data_path}")

    try:
        import yaml
        from ultralytics import YOLO
    except ImportError as error:
        parser.error(
            "Activate /opt/env in the x86_64 training container; PyYAML/Ultralytics is missing: "
            f"{error}"
        )
    dataset = yaml.safe_load(data_path.read_text(encoding="utf-8"))
    if not isinstance(dataset, dict):
        parser.error("Dataset YAML root must be a mapping")
    actual_classes = _classes(dataset)
    if actual_classes != args.classes:
        parser.error(
            "Dataset class order does not match --classes: "
            f"dataset={actual_classes}, expected={args.classes}"
        )

    model = YOLO(args.model)
    result = model.train(
        data=str(data_path),
        imgsz=224,
        epochs=args.epochs,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(args.project),
        name=args.name,
        cache=False,
        plots=True,
    )
    weights = Path(result.save_dir) / "weights" / "best.pt"
    exported = ""
    if not args.no_export:
        trained = YOLO(str(weights))
        exported = str(
            trained.export(
                format="onnx",
                imgsz=224,
                batch=1,
                dynamic=False,
                simplify=True,
                opset=17,
                nms=False,
            )
        )

    manifest = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "dataset_yaml": str(data_path),
        "classes": actual_classes,
        "image_size": [224, 224],
        "initial_model": args.model,
        "best_weights": str(weights),
        "onnx": exported,
    }
    manifest_path = Path(result.save_dir) / "jetpilot_model_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
