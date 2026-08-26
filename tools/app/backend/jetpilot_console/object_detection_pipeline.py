from __future__ import annotations

import csv
import json
import os
import re
from pathlib import Path, PurePosixPath
from typing import Any

from .e2e_pipeline import PipelineTaskSpec
from .map_detail import load_yaml
from .object_detection_analysis import scan_object_detection_models
from .security import (
    resolve_under_root,
    validate_remote_absolute_path,
    validate_ssh_target,
)


EXPECTED_CLASSES = ("vehicle", "barrier")
NETWORK_SIZE = 224
BASE_MODELS = (
    {
        "id": "yolov8n.pt",
        "label": "YOLOv8 Nano",
        "description": "Jetson Orin Nano向けの推奨軽量モデル",
        "recommended": True,
    },
    {
        "id": "yolov8s.pt",
        "label": "YOLOv8 Small",
        "description": "精度を優先する比較用モデル",
        "recommended": False,
    },
)
NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
DEVICE_PATTERN = re.compile(r"^(?:cpu|mps|[0-9]+(?:,[0-9]+)*)$")
REMOTE_PATH_PATTERN = re.compile(r"^/[A-Za-z0-9._/-]+$")
IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".webp"})


def training_root(config: Any) -> Path:
    return (Path(config.python_ws) / "jetpilot_object_detection_training").resolve(
        strict=False
    )


def dataset_root(config: Any) -> Path:
    configured = os.environ.get("JETPILOT_OBJECT_DETECTION_DATASET_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve(strict=False)
    return training_root(config) / "datasets"


def run_root(config: Any) -> Path:
    configured = os.environ.get("JETPILOT_OBJECT_DETECTION_OUTPUT_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve(strict=False)
    return training_root(config) / "outputs" / "yolov8"


def model_root(config: Any) -> Path:
    configured = os.environ.get("JETPILOT_YOLO_MODEL_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve(strict=False)
    return (Path(config.ros2_ws) / "models" / "yolov8").resolve(strict=False)


def _name(value: object, *, label: str) -> str:
    normalized = str(value or "").strip()
    if not NAME_PATTERN.fullmatch(normalized) or normalized in {".", ".."}:
        raise ValueError(
            f"{label} must use 1-64 letters, numbers, '.', '_' or '-'"
        )
    return normalized


def _integer(
    value: object, *, label: str, minimum: int, maximum: int
) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be an integer")
    if isinstance(value, float) and not value.is_integer():
        raise ValueError(f"{label} must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError):
        raise ValueError(f"{label} must be an integer") from None
    if result < minimum or result > maximum:
        raise ValueError(f"{label} must be between {minimum} and {maximum}")
    return result


def _boolean(value: object, *, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off", ""}:
        return False
    raise ValueError("boolean value must be true or false")


def _safe_integer(value: object, *, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _python_command(config: Any, module: str, arguments: list[str]) -> list[str]:
    source_root = training_root(config) / "src"
    inherited = os.environ.get("PYTHONPATH", "")
    python_path = str(source_root) + (
        os.pathsep + inherited if inherited else ""
    )
    return [
        "env",
        f"PYTHONPATH={python_path}",
        str(config.python_bin),
        "-m",
        module,
        *arguments,
    ]


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _regular_file(path: Path) -> bool:
    return path.is_file() and not path.is_symlink()


def _under(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _has_symlink_component(path: Path, root: Path) -> bool:
    try:
        relative = path.relative_to(root)
    except ValueError:
        return True
    current = root
    if current.is_symlink():
        return True
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            return True
    return False


def _classes_from_yaml(data: dict[str, Any]) -> list[str]:
    names = data.get("names")
    if isinstance(names, list):
        return [str(item) for item in names]
    # The Console's lightweight YAML reader intentionally has no PyYAML
    # dependency. Roboflow commonly renders ``names: [vehicle, barrier]``;
    # accept that simple scalar form while leaving full validation to the
    # training workspace CLI (which uses PyYAML).
    if isinstance(names, str) and names.startswith("[") and names.endswith("]"):
        return [
            item.strip().strip("'\"")
            for item in names[1:-1].split(",")
            if item.strip()
        ]
    if isinstance(names, dict):
        indexed: list[tuple[int, str]] = []
        for key, value in names.items():
            try:
                index = int(key)
            except (TypeError, ValueError):
                raise ValueError(f"class id is not an integer: {key}") from None
            indexed.append((index, str(value)))
        indexed.sort()
        if [index for index, _ in indexed] != list(range(len(indexed))):
            raise ValueError("class ids must be contiguous and start at 0")
        return [value for _, value in indexed]
    raise ValueError("names must be a list or mapping")


def _resolve_dataset_directory(
    yaml_path: Path, data: dict[str, Any], allowed_root: Path
) -> Path:
    raw = str(data.get("path") or ".").strip()
    candidate = Path(raw).expanduser()
    if not candidate.is_absolute():
        candidate = yaml_path.parent / candidate
    resolved = candidate.resolve(strict=False)
    if not _under(resolved, allowed_root):
        raise ValueError(f"dataset path must be under {allowed_root}")
    if not resolved.is_dir() or _has_symlink_component(resolved, allowed_root):
        raise ValueError(f"dataset directory does not exist or uses a symlink: {resolved}")
    return resolved


def _resolve_split_directory(
    value: object,
    *,
    dataset_directory: Path,
    allowed_root: Path,
    split: str,
) -> Path:
    raw = str(value or "").strip()
    if not raw:
        raise ValueError(f"dataset YAML is missing the required {split} split")
    candidate = Path(raw).expanduser()
    if not candidate.is_absolute():
        candidate = dataset_directory / candidate
    resolved = candidate.resolve(strict=False)
    if not _under(resolved, allowed_root):
        raise ValueError(f"{split} split must be under {allowed_root}")
    if not resolved.is_dir() or _has_symlink_component(resolved, allowed_root):
        raise ValueError(f"{split} image directory was not found: {resolved}")
    return resolved


def _label_directory(image_directory: Path) -> Path:
    parts = list(image_directory.parts)
    for index in range(len(parts) - 1, -1, -1):
        if parts[index] == "images":
            parts[index] = "labels"
            return Path(*parts)
    return image_directory.parent / "labels"


def _count_files(path: Path, suffixes: frozenset[str]) -> int:
    if not path.is_dir() or path.is_symlink():
        return 0
    return sum(
        1
        for item in path.rglob("*")
        if _regular_file(item) and item.suffix.lower() in suffixes
    )


def _count_annotations(path: Path) -> tuple[int, int]:
    if not path.is_dir() or path.is_symlink():
        return 0, 0
    files = 0
    annotations = 0
    for label in path.rglob("*.txt"):
        if not _regular_file(label):
            continue
        files += 1
        try:
            annotations += sum(
                1
                for line in label.read_text(
                    encoding="utf-8", errors="replace"
                ).splitlines()
                if line.strip()
            )
        except OSError:
            continue
    return files, annotations


def _dataset_summary(path: Path, root: Path) -> dict[str, Any]:
    issues: list[str] = []
    classes: list[str] = []
    split_summary: dict[str, Any] = {}
    dataset_directory = path.parent.resolve(strict=False)
    try:
        data = load_yaml(path)
        if not data:
            raise ValueError("dataset YAML is empty or invalid")
        classes = _classes_from_yaml(data)
        if tuple(classes) != EXPECTED_CLASSES:
            raise ValueError(
                f"class order must be {list(EXPECTED_CLASSES)}, got {classes}"
            )
        dataset_directory = _resolve_dataset_directory(path, data, root)
        for split in ("train", "val", "test"):
            if split == "test" and not str(data.get(split) or "").strip():
                continue
            image_directory = _resolve_split_directory(
                data.get(split),
                dataset_directory=dataset_directory,
                allowed_root=root,
                split=split,
            )
            label_directory = _label_directory(image_directory)
            if not _under(label_directory.resolve(strict=False), root):
                raise ValueError(f"{split} label directory must be under {root}")
            images = _count_files(image_directory, IMAGE_SUFFIXES)
            label_files, annotations = _count_annotations(label_directory)
            split_summary[split] = {
                "image_directory": str(image_directory),
                "label_directory": str(label_directory),
                "images": images,
                "label_files": label_files,
                "annotations": annotations,
            }
            if images <= 0:
                issues.append(f"{split} split has no images")
            if not label_directory.is_dir():
                issues.append(f"{split} label directory was not found")
    except (OSError, TypeError, ValueError) as error:
        issues.append(str(error))

    stat = path.stat()
    return {
        "name": path.parent.name,
        "path": str(path.resolve(strict=False)),
        "data_yaml": str(path.resolve(strict=False)),
        "directory": str(dataset_directory),
        "relative_path": str(path.resolve(strict=False).relative_to(root)),
        "classes": classes,
        "splits": split_summary,
        "image_count": sum(int(item["images"]) for item in split_summary.values()),
        "label_count": sum(int(item["label_files"]) for item in split_summary.values()),
        "annotation_count": sum(
            int(item["annotations"]) for item in split_summary.values()
        ),
        "valid": not issues,
        "issues": issues,
        "error": "; ".join(issues),
        "modified_at_ns": str(stat.st_mtime_ns),
    }


def scan_datasets(config: Any) -> list[dict[str, Any]]:
    root = dataset_root(config)
    if not root.is_dir() or root.is_symlink():
        return []
    resolved_root = root.resolve(strict=True)
    records: list[dict[str, Any]] = []
    for path in root.rglob("data.yaml"):
        if not _regular_file(path):
            continue
        resolved = path.resolve(strict=True)
        if not _under(resolved, resolved_root) or _has_symlink_component(
            resolved, resolved_root
        ):
            continue
        try:
            records.append(_dataset_summary(resolved, resolved_root))
        except (OSError, ValueError):
            continue
    records.sort(key=lambda item: int(item["modified_at_ns"]), reverse=True)
    return records


def _resolve_dataset_yaml(config: Any, value: object) -> Path:
    root = dataset_root(config)
    path = resolve_under_root(
        str(value or ""), root, label="dataset YAML", require_exists=True
    )
    if path.name != "data.yaml" or not _regular_file(path):
        raise ValueError(f"dataset must be a regular data.yaml file: {path}")
    resolved_root = root.resolve(strict=False)
    if _has_symlink_component(path, resolved_root):
        raise ValueError("dataset YAML must not use symlinks")
    return path


def _validated_dataset_yaml(config: Any, value: object) -> Path:
    path = _resolve_dataset_yaml(config, value)
    summary = _dataset_summary(path, dataset_root(config).resolve(strict=False))
    if not summary["valid"]:
        raise ValueError(
            "dataset contract is invalid: " + "; ".join(summary["issues"])
        )
    return path


def _last_metrics(path: Path) -> dict[str, Any]:
    if not _regular_file(path):
        return {}
    last: dict[str, str] = {}
    try:
        with path.open(newline="", encoding="utf-8", errors="replace") as handle:
            for row in csv.DictReader(handle):
                if isinstance(row, dict) and any(str(value or "").strip() for value in row.values()):
                    last = {str(key).strip(): str(value or "").strip() for key, value in row.items()}
    except OSError:
        return {}
    if not last:
        return {}

    def number(*keys: str) -> float | int | None:
        for key in keys:
            if key not in last or not last[key]:
                continue
            try:
                value = float(last[key])
            except ValueError:
                continue
            return int(value) if value.is_integer() else value
        return None

    return {
        "epoch": number("epoch"),
        "precision": number("metrics/precision(B)", "metrics/precision"),
        "recall": number("metrics/recall(B)", "metrics/recall"),
        "map50": number("metrics/mAP50(B)", "metrics/mAP50"),
        "map50_95": number("metrics/mAP50-95(B)", "metrics/mAP50-95"),
        "train_box_loss": number("train/box_loss"),
        "val_box_loss": number("val/box_loss"),
        "raw": last,
    }


def scan_runs(config: Any) -> list[dict[str, Any]]:
    root = run_root(config)
    if not root.is_dir() or root.is_symlink():
        return []
    resolved_root = root.resolve(strict=True)
    records: list[dict[str, Any]] = []
    for directory in root.iterdir():
        if not directory.is_dir() or directory.is_symlink():
            continue
        resolved = directory.resolve(strict=True)
        if resolved.parent != resolved_root:
            continue
        best = resolved / "weights" / "best.pt"
        last = resolved / "weights" / "last.pt"
        results = resolved / "results.csv"
        args_path = resolved / "args.yaml"
        manifest_path = resolved / "jetpilot_training_manifest.json"
        onnx = resolved / "export" / "model.onnx"
        metadata_path = resolved / "export" / "metadata.json"
        candidates = (onnx, best, last, results, args_path, manifest_path)
        existing = [path for path in candidates if _regular_file(path)]
        if not existing:
            continue
        manifest = _read_json(manifest_path)
        metadata = _read_json(metadata_path)
        args = load_yaml(args_path) if _regular_file(args_path) else {}
        metrics = _last_metrics(results)
        stat_path = max(existing, key=lambda path: path.stat().st_mtime_ns)
        epochs = _safe_integer(args.get("epochs"), default=0)
        current_epoch = metrics.get("epoch")
        status = "exported" if _regular_file(onnx) else (
            "trained" if _regular_file(best) else "incomplete"
        )
        classes_value = metadata.get("classes") or manifest.get("classes") or []
        classes = (
            [str(item) for item in classes_value]
            if isinstance(classes_value, list)
            else []
        )
        records.append(
            {
                "name": resolved.name,
                "path": str(resolved),
                "status": status,
                "mode": str(manifest.get("mode") or ""),
                "dataset_yaml": str(
                    manifest.get("dataset_yaml") or args.get("data") or ""
                ),
                "base_model": str(
                    manifest.get("initial_model") or args.get("model") or ""
                ),
                "classes": classes,
                "image_size": _safe_integer(
                    args.get("imgsz"), default=NETWORK_SIZE
                ),
                "epochs": epochs,
                "best_checkpoint": str(best) if _regular_file(best) else "",
                "last_checkpoint": str(last) if _regular_file(last) else "",
                "onnx_path": str(onnx) if _regular_file(onnx) else "",
                "model_root": str(onnx.parent) if _regular_file(onnx) else "",
                "metadata_path": (
                    str(metadata_path) if _regular_file(metadata_path) else ""
                ),
                "manifest_path": (
                    str(manifest_path) if _regular_file(manifest_path) else ""
                ),
                "metrics_path": str(results) if _regular_file(results) else "",
                "metrics": metrics,
                "progress": {
                    "epoch": current_epoch,
                    "epochs": epochs,
                    "fraction": (
                        min(1.0, (float(current_epoch) + 1.0) / epochs)
                        if current_epoch is not None and epochs > 0
                        else 0.0
                    ),
                },
                "modified_at_ns": str(stat_path.stat().st_mtime_ns),
            }
        )
    records.sort(key=lambda item: int(item["modified_at_ns"]), reverse=True)
    return records


def _load_deploy_profiles(config: Any) -> tuple[str, list[dict[str, Any]]]:
    configured = os.environ.get("JETPILOT_OBJECT_DETECTION_DEPLOY_PROFILES", "").strip()
    if not configured:
        configured = os.environ.get("E2E_DEPLOY_PROFILES", "").strip()
    path = (
        Path(configured).expanduser().resolve(strict=False)
        if configured
        else Path(config.python_ws)
        / "jetpilot_e2e_training/src/e2e_learning/conf/deploy_profiles.json"
    )
    payload = _read_json(path)
    values = payload.get("profiles")
    source = values if isinstance(values, list) else []
    profiles: list[dict[str, Any]] = []
    for raw in source:
        if not isinstance(raw, dict):
            continue
        try:
            profile_id = _name(raw.get("id"), label="deploy profile id")
            user = str(raw.get("user") or config.jetson_user).strip()
            host = str(raw.get("host") or "").strip()
            if host != "__manual__":
                validate_ssh_target(user, host)
            remote = _object_detection_remote_root(str(raw.get("remote_root") or ""))
        except ValueError:
            continue
        profiles.append(
            {
                **raw,
                "id": profile_id,
                "user": user,
                "host": host,
                "remote_root": remote,
            }
        )
    default = str(payload.get("default") or "")
    if not any(item["id"] == default for item in profiles):
        default = str(profiles[0]["id"]) if profiles else ""
    return default, profiles


def _object_detection_remote_root(value: str) -> str:
    raw = value.strip()
    if not raw:
        raw = "/workspaces/ros2_ws/models/yolov8"
    validated = validate_remote_absolute_path(raw, label="remote YOLOv8 model root")
    path = PurePosixPath(validated)
    if path.name == "e2e":
        path = path.with_name("yolov8")
    rendered = str(path)
    if rendered in {"/", "/home", "/root", "/tmp", "/usr", "/var", "/workspaces"}:
        raise ValueError("remote YOLOv8 model root is too broad")
    if not REMOTE_PATH_PATTERN.fullmatch(rendered) or "//" in rendered:
        raise ValueError(
            "remote YOLOv8 model root contains unsupported characters"
        )
    return rendered


def pipeline_snapshot(config: Any) -> dict[str, Any]:
    default_profile, profiles = _load_deploy_profiles(config)
    selected_profile = next(
        (item for item in profiles if item["id"] == default_profile), {}
    )
    return {
        "training_root": str(training_root(config)),
        "dataset_root": str(dataset_root(config)),
        "run_root": str(run_root(config)),
        "model_root": str(model_root(config)),
        "datasets": scan_datasets(config),
        "runs": scan_runs(config),
        "models": scan_object_detection_models(config),
        "base_models": [dict(item) for item in BASE_MODELS],
        "classes": list(EXPECTED_CLASSES),
        "input_size": [NETWORK_SIZE, NETWORK_SIZE],
        "deploy_profiles": profiles,
        "default_deploy_profile": default_profile,
        "default_base_model": "yolov8n.pt",
        "defaults": {
            "epochs": 100,
            "batch": 32,
            "device": "0",
            "workers": 8,
            "patience": 30,
            "seed": 42,
            "freeze": 0,
            "opset": 17,
            "export_after_training": True,
            "build_engine": True,
            "deploy_user": str(selected_profile.get("user") or config.jetson_user),
            "deploy_host": (
                ""
                if selected_profile.get("host") == "__manual__"
                else str(selected_profile.get("host") or "")
            ),
            "remote_root": str(selected_profile.get("remote_root") or ""),
        },
    }


# Keep naming parallel to the older E2E backend while the frontend migrates to
# the more explicit snapshot terminology.
pipeline_catalog = pipeline_snapshot


def build_validate_dataset_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    dataset = _resolve_dataset_yaml(
        config,
        body.get("dataset_yaml") or body.get("data_yaml") or body.get("data"),
    )
    command = _python_command(
        config,
        "object_detection_learning.cli.validate_dataset",
        ["--data", str(dataset)],
    )
    if _boolean(body.get("skip_label_scan"), default=False):
        command.append("--skip-label-scan")
    return PipelineTaskSpec(
        kind="object-detection-validate-dataset",
        title=f"Validate object-detection dataset: {dataset.parent.name}",
        command=command,
        cwd=str(training_root(config)),
        artifacts=[{"name": "dataset YAML", "path": str(dataset)}],
        resource_keys=[f"object-detection-dataset:{dataset}"],
    )


def _resolve_checkpoint(config: Any, value: object) -> tuple[Path, Path]:
    root = run_root(config)
    checkpoint = resolve_under_root(
        str(value or ""), root, label="checkpoint", require_exists=True
    )
    if not _regular_file(checkpoint) or checkpoint.suffix != ".pt":
        raise ValueError("checkpoint must be a regular .pt file from a training run")
    resolved_root = root.resolve(strict=False)
    if _has_symlink_component(checkpoint, resolved_root):
        raise ValueError("checkpoint must not use symlinks")
    if checkpoint.parent.name != "weights" or checkpoint.parent.parent.parent != resolved_root:
        raise ValueError("checkpoint must be in <run>/weights under the output root")
    return checkpoint, checkpoint.parent.parent


def _run_dataset_reference(directory: Path) -> str:
    manifest = _read_json(directory / "jetpilot_training_manifest.json")
    if manifest.get("dataset_yaml"):
        return str(manifest["dataset_yaml"])
    args_path = directory / "args.yaml"
    args = load_yaml(args_path) if _regular_file(args_path) else {}
    return str(args.get("data") or "")


def build_train_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    raw_mode = str(body.get("mode") or "train").strip().lower().replace("-", "_")
    mode = "fine_tune" if raw_mode == "finetune" else raw_mode
    if mode not in {"train", "fine_tune", "resume"}:
        raise ValueError("training mode must be train, fine_tune or resume")

    checkpoint: Path | None = None
    checkpoint_run: Path | None = None
    if mode in {"fine_tune", "resume"}:
        checkpoint_value = (
            body.get("checkpoint")
            or body.get("checkpoint_path")
            or body.get("model")
        )
        if not checkpoint_value and body.get("source_run"):
            source_run = _resolve_run(config, body.get("source_run"))
            checkpoint_value = source_run / "weights" / (
                "last.pt" if mode == "resume" else "best.pt"
            )
        checkpoint, checkpoint_run = _resolve_checkpoint(
            config,
            checkpoint_value,
        )
        if mode == "resume" and checkpoint.name != "last.pt":
            raise ValueError("resume requires the selected run's weights/last.pt")
        if mode == "resume":
            requested_name = str(body.get("run_name") or "").strip()
            if requested_name and _name(requested_name, label="run name") != checkpoint_run.name:
                raise ValueError("resume run name must match the selected checkpoint run")
            output = checkpoint_run
            run_name = checkpoint_run.name
        else:
            run_name = _name(body.get("run_name"), label="run name")
            output = resolve_under_root(run_name, run_root(config), label="training output")
    else:
        run_name = _name(body.get("run_name"), label="run name")
        output = resolve_under_root(run_name, run_root(config), label="training output")

    requested_dataset = (
        body.get("dataset_yaml") or body.get("data_yaml") or body.get("data")
    )
    existing_dataset = (
        _run_dataset_reference(checkpoint_run)
        if mode == "resume" and checkpoint_run is not None
        else ""
    )
    dataset = _validated_dataset_yaml(
        config, existing_dataset or requested_dataset
    )
    if existing_dataset and requested_dataset:
        requested_path = _resolve_dataset_yaml(config, requested_dataset)
        if requested_path.resolve(strict=False) != dataset.resolve(strict=False):
            raise ValueError("resume dataset must match the original training run")

    if mode != "resume" and output.exists() and any(output.iterdir()):
        raise ValueError(f"training output already exists and is not empty: {output}")

    base_model = str(body.get("base_model") or body.get("model") or "yolov8n.pt").strip()
    supported = {str(item["id"]) for item in BASE_MODELS}
    if mode == "train" and base_model not in supported:
        raise ValueError(f"base model must be one of: {', '.join(sorted(supported))}")
    model_reference = str(checkpoint) if checkpoint is not None else base_model
    epochs = _integer(body.get("epochs", 100), label="epochs", minimum=1, maximum=10000)
    batch = _integer(body.get("batch", 32), label="batch", minimum=1, maximum=4096)
    workers = _integer(body.get("workers", 8), label="workers", minimum=0, maximum=64)
    patience = _integer(body.get("patience", 30), label="patience", minimum=0, maximum=10000)
    seed = _integer(body.get("seed", 42), label="seed", minimum=0, maximum=2_147_483_647)
    freeze = _integer(body.get("freeze", 0), label="freeze", minimum=0, maximum=1000)
    opset = _integer(body.get("opset", 17), label="opset", minimum=7, maximum=21)
    device = str(body.get("device", "0")).strip().lower()
    if not DEVICE_PATTERN.fullmatch(device):
        raise ValueError("device must be cpu, mps, a GPU index, or comma-separated GPU indexes")

    arguments = ["--data", str(dataset), "--model", model_reference]
    if mode == "resume":
        arguments.append("--resume")
    else:
        arguments.extend(
            [
                "--epochs",
                str(epochs),
                "--batch",
                str(batch),
                "--device",
                device,
                "--workers",
                str(workers),
                "--patience",
                str(patience),
                "--seed",
                str(seed),
                "--freeze",
                str(freeze),
                "--project",
                str(run_root(config)),
                "--name",
                run_name,
                "--opset",
                str(opset),
            ]
        )
        if not _boolean(body.get("export_after_training"), default=True):
            arguments.append("--no-export")
    command = _python_command(
        config, "object_detection_learning.cli.train", arguments
    )
    device_resource_keys = (
        [f"ml-device:{item}" for item in device.split(",")]
        if device not in {"cpu", "mps"}
        else [f"ml-device:{device}"]
    )
    resource_keys = [
        f"object-detection-dataset:{dataset}",
        f"object-detection-run:{output}",
    ]
    if checkpoint_run is not None and checkpoint_run != output:
        resource_keys.append(f"object-detection-run:{checkpoint_run}")
    resource_keys.extend(device_resource_keys)
    return PipelineTaskSpec(
        kind="object-detection-train",
        title=f"{mode.replace('_', ' ').title()} YOLOv8 model: {run_name}",
        command=command,
        cwd=str(training_root(config)),
        artifacts=[
            {"name": "training run", "path": str(output)},
            {"name": "best checkpoint", "path": str(output / "weights/best.pt")},
            {"name": "last checkpoint", "path": str(output / "weights/last.pt")},
            {"name": "metrics", "path": str(output / "results.csv")},
            {"name": "ONNX", "path": str(output / "export/model.onnx")},
            {"name": "metadata", "path": str(output / "export/metadata.json")},
            {
                "name": "training manifest",
                "path": str(output / "jetpilot_training_manifest.json"),
            },
        ],
        resource_keys=resource_keys,
    )


def _resolve_run(config: Any, value: object) -> Path:
    directory = resolve_under_root(
        str(value or ""),
        run_root(config),
        label="training run",
        require_exists=True,
        require_directory=True,
    )
    root = run_root(config).resolve(strict=False)
    if directory.parent != root or directory.is_symlink():
        raise ValueError("training run must be a direct directory under the output root")
    return directory


def build_export_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    directory = _resolve_run(config, body.get("run_dir"))
    requested_weights = body.get("weights_path") or directory / "weights/best.pt"
    weights, weights_run = _resolve_checkpoint(config, requested_weights)
    if weights_run != directory:
        raise ValueError("weights must belong to the selected training run")
    dataset_value = (
        body.get("dataset_yaml")
        or body.get("data_yaml")
        or _run_dataset_reference(directory)
    )
    if not dataset_value:
        raise ValueError("dataset YAML is required to verify the checkpoint class order")
    dataset = _validated_dataset_yaml(config, dataset_value)
    opset = _integer(body.get("opset", 17), label="opset", minimum=7, maximum=21)
    output = directory / "export"
    if output.is_symlink():
        raise ValueError("export directory must not be a symlink")
    arguments = [
        "--weights",
        str(weights),
        "--data",
        str(dataset),
        "--output-dir",
        str(output),
        "--opset",
        str(opset),
    ]
    if not _boolean(body.get("simplify"), default=True):
        arguments.append("--no-simplify")
    return PipelineTaskSpec(
        kind="object-detection-export-onnx",
        title=f"Export YOLOv8 ONNX: {directory.name}",
        command=_python_command(
            config, "object_detection_learning.cli.export_onnx", arguments
        ),
        cwd=str(training_root(config)),
        artifacts=[
            {"name": "ONNX", "path": str(output / "model.onnx")},
            {"name": "metadata", "path": str(output / "metadata.json")},
        ],
        resource_keys=[
            f"object-detection-dataset:{dataset}",
            f"object-detection-run:{directory}",
        ],
    )


def _exported_model(config: Any, value: object) -> tuple[Path, Path, Path]:
    requested = Path(str(value or "")).expanduser().resolve(strict=False)
    for run in scan_runs(config):
        candidate = str(run.get("onnx_path") or "")
        if candidate and Path(candidate).resolve(strict=False) == requested:
            onnx = Path(candidate)
            metadata = Path(str(run.get("metadata_path") or ""))
            directory = Path(str(run["path"]))
            if _regular_file(onnx) and _regular_file(metadata):
                metadata_value = _read_json(metadata)
                classes = metadata_value.get("classes")
                if classes != list(EXPECTED_CLASSES):
                    raise ValueError("export metadata class order does not match the ROS decoder")
                return onnx, metadata, directory
    raise ValueError("model must be an exported model.onnx from a training run")


def build_deploy_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    onnx, metadata, directory = _exported_model(
        config, body.get("model_path") or body.get("onnx_path")
    )
    default_profile, profiles = _load_deploy_profiles(config)
    profile_id = str(body.get("profile") or default_profile)
    profile = next(
        (item for item in profiles if str(item.get("id")) == profile_id), None
    )
    if profile is None:
        raise ValueError(f"unknown deploy profile: {profile_id}")
    user = str(body.get("user") or profile.get("user") or config.jetson_user).strip()
    host = str(body.get("host") or profile.get("host") or "").strip()
    if host == "__manual__":
        raise ValueError("host is required for the manual deploy profile")
    target = validate_ssh_target(user, host)
    remote_root = _object_detection_remote_root(
        str(body.get("remote_root") or profile.get("remote_root") or "")
    )
    deployment_name = _name(
        body.get("model_name") or directory.name, label="deployment model name"
    )
    script = training_root(config) / "scripts" / "deploy_model.sh"
    if not _regular_file(script):
        raise ValueError(f"deployment script was not found: {script}")
    command = [
        str(script),
        str(onnx),
        "--metadata",
        str(metadata),
        "--remote-root",
        remote_root,
        "--name",
        deployment_name,
        "--user",
        user,
        "--host",
        host,
        "--yes",
    ]
    build_engine = _boolean(body.get("build_engine"), default=True)
    if build_engine:
        command.append("--build-engine")
    resource_keys = [
        f"object-detection-run:{directory}",
        f"object-detection-deploy:{target}:{remote_root}/{deployment_name}",
    ]
    if build_engine:
        resource_keys.append(f"jetson-trtexec:{target}")
    return PipelineTaskSpec(
        kind="object-detection-deploy",
        title=f"Deploy YOLOv8 model to {target}",
        command=command,
        cwd=str(training_root(config)),
        artifacts=[
            {"name": "source ONNX", "path": str(onnx)},
            {"name": "source metadata", "path": str(metadata)},
        ],
        resource_keys=resource_keys,
    )
