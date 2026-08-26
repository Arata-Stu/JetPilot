from __future__ import annotations

import argparse
import json
import os
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

from object_detection_learning.contract import (
    EXPECTED_CLASSES,
    INPUT_BINDING_NAME,
    NETWORK_HEIGHT,
    NETWORK_WIDTH,
    OUTPUT_BINDING_NAME,
    classes_from_dataset,
    load_dataset,
    sha256_file,
)


def _atomic_json(path: Path, payload: dict[str, object]) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def export_model(
    weights: Path,
    *,
    output_dir: Path,
    dataset_yaml: Path | None = None,
    opset: int = 17,
    simplify: bool = True,
) -> tuple[Path, Path]:
    weights_path = weights.expanduser().resolve()
    if not weights_path.is_file():
        raise FileNotFoundError(f"Weights were not found: {weights_path}")
    if dataset_yaml is not None:
        dataset_path, _dataset, classes = load_dataset(dataset_yaml)
    else:
        dataset_path = None
        classes = list(EXPECTED_CLASSES)

    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise RuntimeError("Ultralytics is required for ONNX export") from error

    model = YOLO(str(weights_path))
    model_classes = classes_from_dataset({"names": model.names})
    if model_classes != classes:
        raise ValueError(
            "Checkpoint class order does not match the deployment contract: "
            f"checkpoint={model_classes}, expected={classes}"
        )

    output_directory = output_dir.expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    exported = Path(
        model.export(
            format="onnx",
            imgsz=NETWORK_WIDTH,
            batch=1,
            dynamic=False,
            simplify=simplify,
            opset=opset,
            nms=False,
        )
    ).resolve()
    if not exported.is_file():
        raise RuntimeError(f"Ultralytics did not create the expected ONNX file: {exported}")

    onnx_path = output_directory / "model.onnx"
    if exported != onnx_path:
        temporary_onnx = output_directory / f".model.onnx.{os.getpid()}.tmp"
        shutil.copy2(exported, temporary_onnx)
        os.replace(temporary_onnx, onnx_path)

    metadata = {
        "format_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "task": "object_detection",
        "architecture": "yolov8",
        "classes": classes,
        "weights": {
            "path": str(weights_path),
            "sha256": sha256_file(weights_path),
        },
        "dataset_yaml": str(dataset_path) if dataset_path else "",
        "input": {
            "binding_name": INPUT_BINDING_NAME,
            "shape": [1, 3, NETWORK_HEIGHT, NETWORK_WIDTH],
            "layout": "NCHW",
            "color_order": "rgb",
            "value_range": [0.0, 1.0],
            "resize_mode": "letterbox",
        },
        "output": {
            "binding_name": OUTPUT_BINDING_NAME,
            "shape": [1, 4 + len(classes), 1029],
            "layout": "channel_major",
            "nms_included": False,
        },
        "onnx": {
            "filename": onnx_path.name,
            "sha256": sha256_file(onnx_path),
            "opset": opset,
            "dynamic": False,
        },
        "tensorrt": {
            "engine_filename": "model.plan",
            "build_on_target": True,
            "enable_fp16": True,
        },
        "ros_decoder": {
            "package": "jetpilot_object_detection",
            "parameter_file": "config/yolov8.param.yaml",
        },
    }
    metadata_path = output_directory / "metadata.json"
    _atomic_json(metadata_path, metadata)
    return onnx_path, metadata_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export JetPilot YOLOv8 weights to ONNX")
    parser.add_argument("--weights", type=Path, required=True, help="best.pt or another YOLO checkpoint")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--data", type=Path, help="Dataset YAML used to verify class order")
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--no-simplify", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.opset <= 0:
        raise SystemExit("error: --opset must be positive")
    try:
        onnx_path, metadata_path = export_model(
            args.weights,
            output_dir=args.output_dir,
            dataset_yaml=args.data,
            opset=args.opset,
            simplify=not args.no_simplify,
        )
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error
    print(f"Exported ONNX: {onnx_path}")
    print(f"Metadata     : {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
