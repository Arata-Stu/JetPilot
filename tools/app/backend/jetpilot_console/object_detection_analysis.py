from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from .security import resolve_under_root


def object_detection_model_roots(config: Any) -> list[Path]:
    candidates = [Path(config.ros2_ws) / "models" / "yolov8"]
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
