from __future__ import annotations

import argparse
import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

from object_detection_learning.cli.export_onnx import export_model
from object_detection_learning.contract import NETWORK_WIDTH, load_dataset


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


def _atomic_json(path: Path, payload: dict[str, object]) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Train, retrain, or resume the JetPilot 224x224 YOLOv8 detector"
    )
    parser.add_argument("--data", type=Path, required=True, help="Roboflow YOLOv8 data.yaml")
    parser.add_argument(
        "--model",
        default="yolov8n.pt",
        help="Initial weights; use an existing best.pt for fine-tuning",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Resume the exact interrupted Ultralytics run stored in --model",
    )
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--freeze", type=int, default=0, help="Freeze the first N model layers")
    parser.add_argument(
        "--project", type=Path, default=WORKSPACE_ROOT / "outputs/yolov8"
    )
    parser.add_argument("--name", default="yolov8n_224")
    parser.add_argument("--no-export", action="store_true")
    parser.add_argument("--opset", type=int, default=17)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.epochs <= 0 or args.batch <= 0 or args.workers < 0:
        raise SystemExit("error: epochs/batch must be positive and workers must be non-negative")
    if args.patience < 0 or args.freeze < 0 or args.opset <= 0:
        raise SystemExit("error: patience/freeze must be non-negative and opset must be positive")
    try:
        dataset_path, _dataset, classes = load_dataset(args.data)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error

    requested_model_path = Path(args.model).expanduser()
    local_checkpoint = requested_model_path.is_file()
    if args.resume and not local_checkpoint:
        raise SystemExit("error: --resume requires --model to point to an existing checkpoint")
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit(
            "error: activate the training environment; Ultralytics is not installed"
        ) from error

    model_reference = str(requested_model_path.resolve()) if local_checkpoint else args.model
    model = YOLO(model_reference)
    if args.resume:
        result = model.train(resume=True)
    else:
        result = model.train(
            data=str(dataset_path),
            imgsz=NETWORK_WIDTH,
            epochs=args.epochs,
            batch=args.batch,
            device=args.device,
            workers=args.workers,
            patience=args.patience,
            seed=args.seed,
            freeze=args.freeze or None,
            project=str(args.project.expanduser().resolve()),
            name=args.name,
            cache=False,
            plots=True,
        )

    run_directory = Path(result.save_dir).resolve()
    weights = run_directory / "weights/best.pt"
    if not weights.is_file():
        raise SystemExit(f"error: training completed without best.pt: {weights}")

    export_directory = run_directory / "export"
    onnx_path = ""
    metadata_path = ""
    if not args.no_export:
        try:
            exported_onnx, exported_metadata = export_model(
                weights,
                output_dir=export_directory,
                dataset_yaml=dataset_path,
                opset=args.opset,
            )
        except (FileNotFoundError, RuntimeError, ValueError) as error:
            raise SystemExit(f"error: ONNX export failed: {error}") from error
        onnx_path = str(exported_onnx)
        metadata_path = str(exported_metadata)

    manifest = {
        "format_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "mode": "resume" if args.resume else ("fine_tune" if local_checkpoint else "train"),
        "dataset_yaml": str(dataset_path),
        "classes": classes,
        "image_size": [NETWORK_WIDTH, NETWORK_WIDTH],
        "initial_model": model_reference,
        "best_weights": str(weights),
        "onnx": onnx_path,
        "metadata": metadata_path,
        "run_directory": str(run_directory),
    }
    manifest_path = run_directory / "jetpilot_training_manifest.json"
    _atomic_json(manifest_path, manifest)
    print(f"Training manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
