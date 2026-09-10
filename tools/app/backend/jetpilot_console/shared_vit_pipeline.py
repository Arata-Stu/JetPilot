from __future__ import annotations

import json
import os
import re
from datetime import datetime
from pathlib import Path, PurePosixPath
from typing import Any

from .e2e_pipeline import PipelineTaskSpec, occupied_output_names, scan_runs
from .object_detection_pipeline import (
    _load_deploy_profiles,
    _validated_dataset_yaml,
    scan_datasets,
)
from .security import (
    resolve_under_root,
    validate_remote_absolute_path,
    validate_ssh_target,
)


NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
DEVICE_PATTERN = re.compile(r"^(?:cpu|mps|cuda(?::[0-9]+)?|[0-9]+)$")
WEIGHT_SUFFIXES = frozenset({".pt", ".pth"})


def e2e_root(config: Any) -> Path:
    return (Path(config.python_ws) / "jetpilot_e2e_training").resolve(strict=False)


def detection_training_root(config: Any) -> Path:
    return (Path(config.python_ws) / "jetpilot_object_detection_training").resolve(strict=False)


def weight_root(config: Any) -> Path:
    return e2e_root(config) / "weights"


def detection_run_root(config: Any) -> Path:
    configured = os.environ.get("JETPILOT_SHARED_VIT_DETECTION_OUTPUT_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve(strict=False)
    return detection_training_root(config) / "outputs" / "shared_vit_detection"


def model_root(config: Any) -> Path:
    configured = os.environ.get("JETPILOT_SHARED_VIT_MODEL_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve(strict=False)
    return e2e_root(config) / "outputs" / "shared_vit"


def _name(value: object, *, label: str) -> str:
    result = str(value or "").strip()
    if not NAME_PATTERN.fullmatch(result) or result in {".", ".."}:
        raise ValueError(f"{label} must use 1-64 letters, numbers, '.', '_' or '-'")
    return result


def _integer(value: object, *, label: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError):
        raise ValueError(f"{label} must be an integer") from None
    if result < minimum or result > maximum:
        raise ValueError(f"{label} must be between {minimum} and {maximum}")
    return result


def _number(value: object, *, label: str, minimum: float, maximum: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{label} must be a number") from None
    if result < minimum or result > maximum:
        raise ValueError(f"{label} must be between {minimum} and {maximum}")
    return result


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _regular_file(path: Path) -> bool:
    return path.is_file() and not path.is_symlink()


def _available_name(base: str, root: Path) -> str:
    if not (root / base).exists():
        return base
    version = 1
    while True:
        suffix = f"_v{version}"
        candidate = f"{base[:64 - len(suffix)]}{suffix}"
        if not (root / candidate).exists():
            return candidate
        version += 1


def _python_command(config: Any, module: str, arguments: list[str]) -> list[str]:
    sources = [detection_training_root(config) / "src", e2e_root(config) / "src"]
    inherited = os.environ.get("PYTHONPATH", "")
    python_path = os.pathsep.join(str(path) for path in sources)
    if inherited:
        python_path += os.pathsep + inherited
    return ["env", f"PYTHONPATH={python_path}", str(config.python_bin), "-m", module, *arguments]


def _normalization(value: object, *, label: str, default: tuple[float, float, float]) -> list[float]:
    raw = value if isinstance(value, list) else str(value or "").split(",")
    if not raw or raw == [""]:
        return list(default)
    if len(raw) != 3:
        raise ValueError(f"{label} must contain exactly three values")
    return [_number(item, label=label, minimum=-100.0, maximum=100.0) for item in raw]


def scan_backbone_weights(config: Any) -> list[dict[str, str]]:
    root = weight_root(config)
    if not root.is_dir() or root.is_symlink():
        return []
    records: list[dict[str, str]] = []
    for path in root.rglob("*"):
        if not _regular_file(path) or path.suffix.lower() not in WEIGHT_SUFFIXES:
            continue
        relative = path.relative_to(root)
        top = relative.parts[0].lower() if relative.parts else ""
        modality = "event_image" if top in {"eventstate", "gep"} else "rgb"
        records.append({
            "name": relative.as_posix(),
            "path": str(path.resolve(strict=False)),
            "modality": modality,
        })
    records.sort(key=lambda item: item["name"])
    return records


def scan_detection_runs(config: Any) -> list[dict[str, Any]]:
    root = detection_run_root(config)
    if not root.is_dir() or root.is_symlink():
        return []
    records: list[dict[str, Any]] = []
    for directory in root.iterdir():
        if not directory.is_dir() or directory.is_symlink():
            continue
        best = directory / "weights" / "best.pt"
        last = directory / "weights" / "last.pt"
        manifest_path = directory / "jetpilot_training_manifest.json"
        metrics_path = directory / "metrics.json"
        if not any(_regular_file(path) for path in (best, last, manifest_path, metrics_path)):
            continue
        manifest = _read_json(manifest_path)
        stat_path = max(
            (path for path in (best, last, manifest_path, metrics_path) if _regular_file(path)),
            key=lambda path: path.stat().st_mtime_ns,
        )
        records.append({
            "name": directory.name,
            "path": str(directory.resolve(strict=False)),
            "best_checkpoint": str(best) if _regular_file(best) else "",
            "last_checkpoint": str(last) if _regular_file(last) else "",
            "manifest_path": str(manifest_path) if _regular_file(manifest_path) else "",
            "metrics_path": str(metrics_path) if _regular_file(metrics_path) else "",
            "dataset_yaml": str(manifest.get("dataset_yaml") or ""),
            "classes": manifest.get("classes") if isinstance(manifest.get("classes"), list) else [],
            "modality": str(manifest.get("modality") or ""),
            "mean": manifest.get("mean") if isinstance(manifest.get("mean"), list) else [],
            "std": manifest.get("std") if isinstance(manifest.get("std"), list) else [],
            "backbone_weights": str(manifest.get("backbone_weights") or ""),
            "backbone_fingerprint": str(manifest.get("backbone_fingerprint") or ""),
            "modified_at_ns": str(stat_path.stat().st_mtime_ns),
        })
    records.sort(key=lambda item: int(item["modified_at_ns"]), reverse=True)
    return records


def scan_models(config: Any) -> list[dict[str, Any]]:
    root = model_root(config)
    if not root.is_dir() or root.is_symlink():
        return []
    records: list[dict[str, Any]] = []
    for directory in root.iterdir():
        if not directory.is_dir() or directory.is_symlink():
            continue
        onnx = directory / "model.onnx"
        metadata_path = directory / "metadata.json"
        if not _regular_file(onnx):
            continue
        metadata = _read_json(metadata_path)
        records.append({
            "name": directory.name,
            "path": str(directory.resolve(strict=False)),
            "onnx_path": str(onnx),
            "metadata_path": str(metadata_path) if _regular_file(metadata_path) else "",
            "modality": str(metadata.get("modality") or ""),
            "outputs": metadata.get("outputs") if isinstance(metadata.get("outputs"), list) else [],
            "backbone_fingerprint": str(metadata.get("backbone_fingerprint") or ""),
            "modified_at_ns": str(onnx.stat().st_mtime_ns),
        })
    records.sort(key=lambda item: int(item["modified_at_ns"]), reverse=True)
    return records


def pipeline_snapshot(config: Any) -> dict[str, Any]:
    default_profile, profiles = _load_deploy_profiles(config)
    now = datetime.now().astimezone().strftime("%m%d-%H%M")
    detection_base = f"vit-dinov3-vits16-rgb-det-head_{now}"
    model_base = f"vit-dinov3-vits16-rgb-shared_{now}"
    control_runs = [run for run in scan_runs(config) if run.get("model") == "dinov3_vits16"]
    selected_profile = next((item for item in profiles if item.get("id") == default_profile), {})
    remote = str(selected_profile.get("remote_root") or "/workspaces/ros2_ws/models/e2e")
    remote_path = PurePosixPath(remote)
    if remote_path.name != "shared_vit":
        remote = str(remote_path / "shared_vit")
    return {
        "weight_root": str(weight_root(config)),
        "detection_run_root": str(detection_run_root(config)),
        "model_root": str(model_root(config)),
        "weights": scan_backbone_weights(config),
        "datasets": scan_datasets(config),
        "control_runs": control_runs,
        "detection_runs": scan_detection_runs(config),
        "models": scan_models(config),
        "occupied_detection_names": occupied_output_names(detection_run_root(config)),
        "occupied_model_names": occupied_output_names(model_root(config)),
        "suggested_detection_name": _available_name(detection_base, detection_run_root(config)),
        "suggested_model_name": _available_name(model_base, model_root(config)),
        "deploy_profiles": profiles,
        "default_deploy_profile": default_profile,
        "defaults": {
            "remote_root": remote,
            "deploy_user": str(selected_profile.get("user") or config.jetson_user),
            "deploy_host": "" if selected_profile.get("host") == "__manual__" else str(selected_profile.get("host") or ""),
        },
    }


def build_detection_train_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    dataset = _validated_dataset_yaml(config, body.get("dataset_yaml"))
    backbone = resolve_under_root(
        str(body.get("backbone_weights") or ""), weight_root(config),
        label="backbone weights", require_exists=True,
    )
    if not _regular_file(backbone) or backbone.suffix.lower() not in WEIGHT_SUFFIXES:
        raise ValueError("backbone weights must be a regular .pt or .pth file")
    name = _name(body.get("run_name"), label="run name")
    output = resolve_under_root(name, detection_run_root(config), label="detection run")
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"detection output already exists and is not empty: {output}")
    modality = str(body.get("modality") or "rgb")
    if modality not in {"rgb", "event_image"}:
        raise ValueError("modality must be rgb or event_image")
    device = str(body.get("device") or "cuda").strip()
    if not DEVICE_PATTERN.fullmatch(device):
        raise ValueError("device must be cpu, mps, cuda, cuda:N or a CUDA index")
    mean = _normalization(body.get("mean"), label="mean", default=(0.485, 0.456, 0.406))
    std = _normalization(body.get("std"), label="std", default=(0.229, 0.224, 0.225))
    if any(value <= 0 for value in std):
        raise ValueError("std values must be greater than zero")
    command = _python_command(config, "object_detection_learning.cli.train_shared_vit_head", [
        "--data", str(dataset), "--backbone-weights", str(backbone),
        "--project", str(detection_run_root(config)), "--name", name,
        "--epochs", str(_integer(body.get("epochs", 100), label="epochs", minimum=1, maximum=10000)),
        "--batch", str(_integer(body.get("batch", 16), label="batch", minimum=1, maximum=1024)),
        "--workers", str(_integer(body.get("workers", 4), label="workers", minimum=0, maximum=64)),
        "--device", device,
        "--lr", str(_number(body.get("learning_rate", 0.001), label="learning rate", minimum=1e-8, maximum=10.0)),
        "--weight-decay", str(_number(body.get("weight_decay", 0.0001), label="weight decay", minimum=0.0, maximum=10.0)),
        "--input-width", "212", "--input-height", "120",
        "--mean", *(str(value) for value in mean),
        "--std", *(str(value) for value in std),
        "--modality", modality,
    ])
    return PipelineTaskSpec(
        kind="shared-vit-detection-train",
        title=f"Train shared ViT detection head: {name}",
        command=command,
        cwd=str(detection_training_root(config)),
        artifacts=[
            {"name": "run", "path": str(output)},
            {"name": "best checkpoint", "path": str(output / "weights" / "best.pt")},
        ],
        resource_keys=[f"shared-vit-backbone:{backbone}", f"shared-vit-detection-run:{output}"],
    )


def build_export_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    control = resolve_under_root(
        str(body.get("control_checkpoint") or ""), e2e_root(config) / "outputs" / "e2e",
        label="control checkpoint", require_exists=True,
    )
    detection = resolve_under_root(
        str(body.get("detection_checkpoint") or ""), detection_run_root(config),
        label="detection checkpoint", require_exists=True,
    )
    if not _regular_file(control) or control.name != "best.pt":
        raise ValueError("control checkpoint must be an E2E checkpoints/best.pt file")
    if not _regular_file(detection) or detection.name != "best.pt":
        raise ValueError("detection checkpoint must be a shared ViT weights/best.pt file")
    name = _name(body.get("model_name"), label="model name")
    output = resolve_under_root(name, model_root(config), label="shared ViT model")
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"shared ViT model output already exists and is not empty: {output}")
    command = _python_command(config, "e2e_learning.cli.export_multitask_onnx", [
        "--control-checkpoint", str(control),
        "--detection-checkpoint", str(detection),
        "--output-dir", str(output),
        "--opset", str(_integer(body.get("opset", 17), label="opset", minimum=11, maximum=20)),
    ])
    return PipelineTaskSpec(
        kind="shared-vit-export",
        title=f"Assemble shared ViT ONNX: {name}",
        command=command,
        cwd=str(e2e_root(config)),
        artifacts=[{"name": "ONNX", "path": str(output / "model.onnx")}, {"name": "metadata", "path": str(output / "metadata.json")}],
        resource_keys=[f"e2e-run:{control.parent.parent}", f"shared-vit-detection-run:{detection.parent.parent}", f"shared-vit-model:{output}"],
    )


def build_deploy_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    onnx = resolve_under_root(
        str(body.get("model_path") or ""), model_root(config),
        label="shared ViT ONNX", require_exists=True,
    )
    if not _regular_file(onnx) or onnx.name != "model.onnx":
        raise ValueError("model must be an exported shared ViT model.onnx")
    default_profile, profiles = _load_deploy_profiles(config)
    profile_id = str(body.get("profile") or default_profile)
    profile = next((item for item in profiles if str(item.get("id")) == profile_id), None)
    if profile is None:
        raise ValueError(f"unknown deploy profile: {profile_id}")
    user = str(body.get("user") or profile.get("user") or config.jetson_user)
    host = str(body.get("host") or profile.get("host") or "")
    if host == "__manual__":
        raise ValueError("host is required for the manual deploy profile")
    target = validate_ssh_target(user, host)
    remote_root = validate_remote_absolute_path(
        str(body.get("remote_root") or "/workspaces/ros2_ws/models/e2e/shared_vit"),
        label="remote shared ViT model root",
    )
    if remote_root in {"/", "/home", "/root", "/tmp", "/usr", "/var", "/workspaces"}:
        raise ValueError("remote shared ViT model root is too broad")
    name = _name(body.get("deploy_name") or onnx.parent.name, label="deploy name")
    command = [
        "bash", str(e2e_root(config) / "scripts" / "deploy_shared_vit_model.sh"),
        "--onnx", str(onnx), "--user", user, "--host", host,
        "--remote-root", remote_root, "--name", name,
    ]
    return PipelineTaskSpec(
        kind="shared-vit-deploy",
        title=f"Deploy shared ViT model to {target}",
        command=command,
        cwd=str(e2e_root(config)),
        artifacts=[{"name": "source ONNX", "path": str(onnx)}],
        resource_keys=[f"shared-vit-model:{onnx.parent}", f"shared-vit-deploy:{target}:{remote_root}/{name}"],
    )
