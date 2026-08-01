from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, Iterable, Mapping

from .security import resolve_under_root


MODEL_FILENAMES = ("model.onnx",)
METADATA_FILENAME = "metadata.json"


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def e2e_model_roots(config: Any) -> list[Path]:
    candidates = [
        Path(config.repo_root) / "outputs",
        Path(config.python_ws) / "jetpilot_e2e_training" / "outputs",
        Path(config.python_ws) / "outputs",
    ]
    configured = os.environ.get("JETPILOT_E2E_MODEL_ROOTS", "")
    candidates.extend(Path(value).expanduser() for value in configured.split(os.pathsep) if value)
    deployed = Path("/opt/jetpilot/models/e2e")
    if deployed.is_dir():
        candidates.append(deployed)

    roots: list[Path] = []
    for candidate in candidates:
        resolved = candidate.expanduser().resolve(strict=False)
        if resolved not in roots:
            roots.append(resolved)
    return roots


def _read_metadata(path: Path) -> dict[str, Any]:
    metadata_path = path.parent / METADATA_FILENAME
    if not metadata_path.is_file() or metadata_path.is_symlink():
        return {}
    try:
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _model_record(path: Path, root: Path) -> dict[str, Any]:
    metadata = _read_metadata(path)
    model_input = metadata.get("input") if isinstance(metadata.get("input"), dict) else {}
    model_output = metadata.get("output") if isinstance(metadata.get("output"), dict) else {}
    stat = path.stat()
    return {
        "path": str(path),
        "name": str(metadata.get("model_name") or path.parent.name),
        "kind": str(metadata.get("model_kind") or "e2e_control"),
        "root": str(root),
        "relative_path": str(path.relative_to(root)),
        "metadata_path": str(path.parent / METADATA_FILENAME) if metadata else "",
        "input": model_input,
        "output": model_output,
        "size_bytes": stat.st_size,
        "modified_at_ns": str(stat.st_mtime_ns),
        "has_tensorrt_engine": (path.parent / "model.plan").is_file(),
    }


def scan_e2e_models(config: Any) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    seen: set[Path] = set()
    for root in e2e_model_roots(config):
        if not root.is_dir() or root.is_symlink():
            continue
        for filename in MODEL_FILENAMES:
            for candidate in root.glob(f"**/{filename}"):
                try:
                    resolved = candidate.resolve(strict=True)
                except OSError:
                    continue
                if resolved in seen or not resolved.is_file() or resolved.is_symlink():
                    continue
                if not _is_relative_to(resolved, root):
                    continue
                seen.add(resolved)
                records.append(_model_record(resolved, root))
    records.sort(key=lambda item: int(item["modified_at_ns"]), reverse=True)
    return records


def resolve_e2e_model(config: Any, value: object) -> tuple[Path | None, dict[str, Any]]:
    raw = str(value or "").strip()
    if not raw:
        return None, {}
    requested = Path(raw).expanduser()
    allowed_roots = e2e_model_roots(config)
    resolved: Path | None = None
    for root in allowed_roots:
        try:
            candidate = resolve_under_root(
                requested if requested.is_absolute() else root / requested,
                root,
                label="E2E model",
                require_exists=True,
            )
        except (FileNotFoundError, ValueError):
            continue
        if candidate.is_file() and not candidate.is_symlink() and candidate.name.endswith(".onnx"):
            resolved = candidate
            break
    if resolved is None:
        raise ValueError("E2E model must be an existing ONNX file under an allowed model root")
    return resolved, _read_metadata(resolved)


def finite_summary(values: Iterable[object]) -> dict[str, float | int | None]:
    numbers: list[float] = []
    for value in values:
        try:
            number = float(value)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            numbers.append(number)
    numbers.sort()
    if not numbers:
        return {"count": 0, "mean": None, "p50": None, "p95": None, "p99": None, "max": None}

    def percentile(fraction: float) -> float:
        position = fraction * (len(numbers) - 1)
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        if lower == upper:
            return numbers[lower]
        ratio = position - lower
        return numbers[lower] * (1.0 - ratio) + numbers[upper] * ratio

    return {
        "count": len(numbers),
        "mean": round(sum(numbers) / len(numbers), 6),
        "p50": round(percentile(0.50), 6),
        "p95": round(percentile(0.95), 6),
        "p99": round(percentile(0.99), 6),
        "max": round(numbers[-1], 6),
    }


def control_error_summary(samples: Iterable[Mapping[str, Any]], field: str) -> dict[str, Any]:
    errors: list[float] = []
    for sample in samples:
        try:
            value = float(sample.get(f"{field}_error"))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            errors.append(value)
    if not errors:
        return {"count": 0, "mae": None, "rmse": None, "max_abs": None, "signed": finite_summary([])}
    absolute = [abs(value) for value in errors]
    return {
        "count": len(errors),
        "mae": round(sum(absolute) / len(absolute), 8),
        "rmse": round(math.sqrt(sum(value * value for value in errors) / len(errors)), 8),
        "max_abs": round(max(absolute), 8),
        "absolute": finite_summary(absolute),
        "signed": finite_summary(errors),
    }
