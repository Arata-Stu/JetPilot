from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

from .security import resolve_under_root


def object_detection_model_roots(config: Any) -> list[Path]:
    candidates = [Path(config.ros2_ws) / "models" / "yolov8"]
    configured_python_ws = getattr(config, "python_ws", None)
    if configured_python_ws:
        candidates.append(
            Path(configured_python_ws)
            / "jetpilot_object_detection_training"
            / "outputs"
            / "yolov8"
        )
    configured = os.environ.get("JETPILOT_OBJECT_DETECTION_MODEL_ROOTS", "")
    candidates.extend(
        Path(value).expanduser() for value in configured.split(os.pathsep) if value
    )
    roots: list[Path] = []
    for candidate in candidates:
        resolved = candidate.expanduser().resolve(strict=False)
        if resolved not in roots:
            roots.append(resolved)
    return roots


def scan_object_detection_models(config: Any) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    seen: set[Path] = set()
    for index, root in enumerate(object_detection_model_roots(config)):
        if not root.is_dir() or root.is_symlink():
            continue
        resolved_root = root.resolve(strict=True)
        for onnx in root.rglob("model.onnx"):
            if not onnx.is_file() or onnx.is_symlink():
                continue
            try:
                resolved_onnx = onnx.resolve(strict=True)
                resolved_onnx.relative_to(resolved_root)
                directory = resolved_onnx.parent
                relative = directory.relative_to(resolved_root)
            except (OSError, ValueError):
                continue
            if directory in seen:
                continue
            seen.add(directory)
            metadata_path = directory / "metadata.json"
            metadata: dict[str, Any] = {}
            if metadata_path.is_file() and not metadata_path.is_symlink():
                try:
                    value = json.loads(metadata_path.read_text(encoding="utf-8"))
                    if isinstance(value, dict):
                        metadata = value
                except (OSError, json.JSONDecodeError):
                    pass
            display_name = directory.parent.name if directory.name == "export" else directory.name
            records.append(
                {
                    "name": display_name,
                    "path": str(directory),
                    "relative_path": str(relative),
                    "source": "deployment" if index == 0 else "training_run",
                    "onnx_path": str(resolved_onnx),
                    "metadata_path": str(metadata_path) if metadata_path.is_file() else "",
                    "engine_path": str(directory / "model.plan") if (directory / "model.plan").is_file() else "",
                    "classes": metadata.get("classes") if isinstance(metadata.get("classes"), list) else [],
                    "modified_at_ns": str(resolved_onnx.stat().st_mtime_ns),
                }
            )
    records.sort(key=lambda item: int(item["modified_at_ns"]), reverse=True)
    return records


def resolve_object_detection_model_root(config: Any, value: object) -> Path | None:
    raw = str(value or "").strip()
    if not raw:
        return None
    requested = Path(raw).expanduser()
    for root in object_detection_model_roots(config):
        try:
            candidate = resolve_under_root(
                requested if requested.is_absolute() else root / requested,
                root,
                label="YOLOv8 model directory",
                require_exists=True,
                require_directory=True,
            )
        except (FileNotFoundError, ValueError):
            continue
        if (candidate / "model.onnx").is_file() and not (candidate / "model.onnx").is_symlink():
            return candidate
    raise ValueError(
        "YOLOv8 model directory must contain model.onnx under an allowed model root: "
        + ", ".join(str(root) for root in object_detection_model_roots(config))
    )
