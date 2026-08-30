from __future__ import annotations

import csv
import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .map_detail import load_yaml
from .security import (
    resolve_under_root,
    validate_remote_absolute_path,
    validate_ssh_target,
)


NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
EXPERIMENTS = {
    "pilotnet_scratch": {"label": "Control · PilotNet", "stages": 1, "task": "control"},
    "control_pilotnet_fusion": {"label": "Control · PilotNet (fusion baseline)", "stages": 1, "task": "control", "model": "fusion"},
    "control_pilotnet_gru": {"label": "Control · PilotNet + GRU", "stages": 1, "task": "control", "model": "fusion"},
    "control_pilotnet_imu": {"label": "Control · PilotNet + IMU", "stages": 1, "task": "control", "model": "fusion"},
    "control_pilotnet_gru_imu": {"label": "Control · PilotNet + GRU + IMU", "stages": 1, "task": "control", "model": "fusion"},
    "trajectory_pilotnet": {"label": "Trajectory · PilotNet", "stages": 1, "task": "trajectory", "model": "fusion"},
    "trajectory_pilotnet_gru": {"label": "Trajectory · PilotNet + GRU", "stages": 1, "task": "trajectory", "model": "fusion"},
    "trajectory_pilotnet_imu": {"label": "Trajectory · PilotNet + IMU", "stages": 1, "task": "trajectory", "model": "fusion"},
    "trajectory_pilotnet_gru_imu": {"label": "Trajectory · PilotNet + GRU + IMU", "stages": 1, "task": "trajectory", "model": "fusion"},
    "mobilenet_frozen_head": {"label": "Control · MobileNetV3 / frozen head", "stages": 1, "task": "control"},
    "mobilenet_head_then_finetune": {
        "label": "Control · MobileNetV3 / head then fine-tune",
        "stages": 2,
        "task": "control",
    },
}


@dataclass(frozen=True)
class PipelineTaskSpec:
    kind: str
    title: str
    command: list[str]
    cwd: str
    artifacts: list[dict[str, str]]
    resource_keys: list[str]


def training_root(config: Any) -> Path:
    return (Path(config.python_ws) / "jetpilot_e2e_training").resolve(strict=False)


def dataset_root(config: Any) -> Path:
    configured = os.environ.get("JETPILOT_E2E_DATASET_ROOT", "")
    return Path(configured).expanduser().resolve(strict=False) if configured else training_root(config) / "datasets"


def run_root(config: Any) -> Path:
    configured = os.environ.get("JETPILOT_E2E_OUTPUT_ROOT", "")
    return Path(configured).expanduser().resolve(strict=False) if configured else training_root(config) / "outputs" / "e2e"


def _name(value: object, *, label: str) -> str:
    normalized = str(value or "").strip()
    if not NAME_PATTERN.fullmatch(normalized):
        raise ValueError(f"{label} must use 1-64 letters, numbers, '.', '_' or '-'")
    return normalized


def _integer(value: object, *, label: str, minimum: int, maximum: int) -> int:
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


def _topic(value: object, *, label: str) -> str:
    topic = str(value or "").strip()
    if not topic.startswith("/") or any(char.isspace() for char in topic):
        raise ValueError(f"{label} must be an absolute ROS topic")
    return topic


def _python_command(config: Any, module: str, overrides: list[str]) -> list[str]:
    source_root = training_root(config) / "src"
    inherited = os.environ.get("PYTHONPATH", "")
    python_path = str(source_root) + (os.pathsep + inherited if inherited else "")
    return [
        "env",
        f"PYTHONPATH={python_path}",
        str(config.python_bin),
        "-m",
        module,
        *overrides,
    ]


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _sample_count(path: Path) -> int:
    try:
        with path.open(newline="", encoding="utf-8", errors="replace") as handle:
            return max(0, sum(1 for _ in csv.reader(handle)) - 1)
    except OSError:
        return 0


def scan_datasets(config: Any) -> list[dict[str, Any]]:
    root = dataset_root(config)
    if not root.is_dir() or root.is_symlink():
        return []
    records: list[dict[str, Any]] = []
    for samples in root.glob("**/samples.csv"):
        directory = samples.parent
        try:
            resolved = directory.resolve(strict=True)
            resolved.relative_to(root.resolve(strict=False))
        except (OSError, ValueError):
            continue
        if directory.is_symlink() or samples.is_symlink():
            continue
        metadata = load_yaml(directory / "metadata.yaml") if (directory / "metadata.yaml").is_file() else {}
        stat = samples.stat()
        records.append(
            {
                "name": directory.name,
                "path": str(resolved),
                "relative_path": str(resolved.relative_to(root.resolve(strict=False))),
                "sample_count": int(metadata.get("sample_count") or _sample_count(samples)),
                "bag_path": str(metadata.get("bag_path") or ""),
                "image_topic": str(metadata.get("image_topic") or ""),
                "control_topic": str(metadata.get("control_topic") or ""),
                "odometry_topic": str(metadata.get("odometry_topic") or ""),
                "imu_topic": str(metadata.get("imu_topic") or ""),
                "task": str(metadata.get("task") or "control"),
                "trajectory_points": int(metadata.get("trajectory_points") or 0),
                "trajectory_horizon_sec": float(metadata.get("trajectory_horizon_sec") or 0.0),
                "input_width": int(metadata.get("input_width") or 0),
                "input_height": int(metadata.get("input_height") or 0),
                "modified_at_ns": str(stat.st_mtime_ns),
            }
        )
    records.sort(key=lambda item: int(item["modified_at_ns"]), reverse=True)
    return records


def scan_runs(config: Any) -> list[dict[str, Any]]:
    root = run_root(config)
    if not root.is_dir() or root.is_symlink():
        return []
    records: list[dict[str, Any]] = []
    for directory in root.iterdir():
        if not directory.is_dir() or directory.is_symlink():
            continue
        run_yaml = directory / "run.yaml"
        progress_path = directory / "progress.json"
        metrics_path = directory / "metrics.json"
        best_path = directory / "checkpoints" / "best.pt"
        onnx_path = directory / "model.onnx"
        if not any(path.exists() for path in (run_yaml, progress_path, metrics_path, best_path, onnx_path)):
            continue
        config_data = load_yaml(run_yaml) if run_yaml.is_file() else {}
        progress = _read_json(progress_path)
        metrics = _read_json(metrics_path)
        model_data = config_data.get("model") if isinstance(config_data.get("model"), dict) else {}
        data = config_data.get("data") if isinstance(config_data.get("data"), dict) else {}
        stat_path = max(
            (path for path in (onnx_path, best_path, progress_path, run_yaml) if path.exists()),
            key=lambda path: path.stat().st_mtime_ns,
        )
        records.append(
            {
                "name": directory.name,
                "path": str(directory.resolve(strict=False)),
                "status": str(progress.get("status") or ("trained" if best_path.is_file() else "incomplete")),
                "model": str(model_data.get("name") or metrics.get("model") or ""),
                "task": str(model_data.get("task") or metrics.get("task") or "control"),
                "architecture": metrics.get("architecture") if isinstance(metrics.get("architecture"), dict) else {},
                "dataset_dir": str(data.get("dataset_dir") or metrics.get("dataset_dir") or ""),
                "input_width": int(data.get("input_width") or 0),
                "input_height": int(data.get("input_height") or 0),
                "best_checkpoint": str(best_path) if best_path.is_file() else "",
                "onnx_path": str(onnx_path) if onnx_path.is_file() else "",
                "metadata_path": str(directory / "metadata.json") if (directory / "metadata.json").is_file() else "",
                "metrics": metrics,
                "progress": progress,
                "modified_at_ns": str(stat_path.stat().st_mtime_ns),
            }
        )
    records.sort(key=lambda item: int(item["modified_at_ns"]), reverse=True)
    return records


def _load_collection(path: Path, key: str) -> tuple[str, list[dict[str, Any]]]:
    payload = _read_json(path)
    values = payload.get(key)
    entries = [dict(item) for item in values if isinstance(item, dict)] if isinstance(values, list) else []
    return str(payload.get("default") or ""), entries


def pipeline_catalog(config: Any) -> dict[str, Any]:
    root = training_root(config)
    profile_default, profiles = _load_collection(
        root / "src/e2e_learning/conf/deploy_profiles.json", "profiles"
    )
    preset_default, presets = _load_collection(
        root / "src/e2e_learning/conf/deploy_model_presets.json", "presets"
    )
    return {
        "dataset_root": str(dataset_root(config)),
        "run_root": str(run_root(config)),
        "datasets": scan_datasets(config),
        "runs": scan_runs(config),
        "experiments": [
            {"id": key, **value} for key, value in EXPERIMENTS.items()
        ],
        "deploy_profiles": profiles,
        "default_deploy_profile": profile_default,
        "deploy_presets": presets,
        "default_deploy_preset": preset_default,
    }


def build_preprocess_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    bag = resolve_under_root(
        str(body.get("rosbag") or ""),
        Path(config.record_root),
        label="rosbag",
        require_exists=True,
        require_directory=True,
    )
    name = _name(body.get("dataset_name"), label="dataset name")
    output = resolve_under_root(name, dataset_root(config), label="dataset output")
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"dataset output already exists and is not empty: {output}")
    width = _integer(body.get("input_width", 212), label="input width", minimum=32, maximum=4096)
    height = _integer(body.get("input_height", 120), label="input height", minimum=32, maximum=4096)
    max_dt = _number(body.get("max_control_dt_sec", 0.1), label="max control dt", minimum=0.001, maximum=5.0)
    max_odom_dt = _number(body.get("max_odometry_dt_sec", 0.15), label="max odometry dt", minimum=0.001, maximum=5.0)
    task = str(body.get("task") or "control")
    if task not in {"control", "trajectory"}:
        raise ValueError("task must be control or trajectory")
    trajectory_points = _integer(body.get("trajectory_points", 10), label="trajectory points", minimum=2, maximum=100)
    trajectory_horizon = _number(body.get("trajectory_horizon_sec", 1.5), label="trajectory horizon", minimum=0.1, maximum=20.0)
    trajectory_scale = _number(body.get("trajectory_scale_m", 5.0), label="trajectory scale", minimum=0.1, maximum=100.0)
    imu_samples = _integer(body.get("imu_samples", 10), label="IMU samples", minimum=1, maximum=500)
    imu_window = _number(body.get("imu_window_sec", 0.5), label="IMU window", minimum=0.01, maximum=10.0)
    jpeg_quality = _integer(body.get("jpeg_quality", 92), label="JPEG quality", minimum=1, maximum=100)
    command = _python_command(
        config,
        "e2e_learning.cli.preprocess_bag",
        [
            f"data.bag_path={bag}",
            f"data.output_dir={output}",
            f"data.image_topic={_topic(body.get('image_topic'), label='image topic')}",
            f"data.control_topic={_topic(body.get('control_topic') or '/teleop/control_cmd', label='control topic')}",
            f"data.odometry_topic={_topic(body.get('odometry_topic') or '/visual_slam/tracking/odometry', label='odometry topic')}",
            f"data.imu_topic={_topic(body.get('imu_topic') or '/realsense/imu', label='IMU topic')}",
            f"data.task={task}",
            f"data.input_width={width}",
            f"data.input_height={height}",
            f"data.max_control_dt_sec={max_dt}",
            f"data.max_odometry_dt_sec={max_odom_dt}",
            f"data.trajectory_points={trajectory_points}",
            f"data.trajectory_horizon_sec={trajectory_horizon}",
            f"data.trajectory_scale_m={trajectory_scale}",
            f"data.imu_samples={imu_samples}",
            f"data.imu_window_sec={imu_window}",
            f"data.jpeg_quality={jpeg_quality}",
        ],
    )
    return PipelineTaskSpec(
        kind="e2e-preprocess",
        title=f"Create E2E dataset: {name}",
        command=command,
        cwd=str(training_root(config)),
        artifacts=[
            {"name": "dataset", "path": str(output)},
            {"name": "samples", "path": str(output / "samples.csv")},
            {"name": "metadata", "path": str(output / "metadata.yaml")},
        ],
        resource_keys=[f"analysis-bag:{bag}", f"e2e-dataset:{output}"],
    )


def build_train_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    dataset = resolve_under_root(
        str(body.get("dataset_dir") or ""),
        dataset_root(config),
        label="dataset",
        require_exists=True,
        require_directory=True,
    )
    if not (dataset / "samples.csv").is_file():
        raise ValueError(f"dataset has no samples.csv: {dataset}")
    run_name = _name(body.get("run_name"), label="run name")
    experiment = str(body.get("experiment") or "pilotnet_scratch")
    if experiment not in EXPERIMENTS:
        raise ValueError(f"unsupported experiment: {experiment}")
    output = resolve_under_root(run_name, run_root(config), label="training output")
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"training output already exists and is not empty: {output}")

    batch_size = _integer(body.get("batch_size", 64), label="batch size", minimum=1, maximum=4096)
    workers = _integer(body.get("num_workers", 4), label="worker count", minimum=0, maximum=64)
    seed = _integer(body.get("seed", 42), label="seed", minimum=0, maximum=2_147_483_647)
    fraction = _number(body.get("fraction", 1.0), label="data fraction", minimum=0.001, maximum=1.0)
    val_fraction = _number(body.get("val_fraction", 0.2), label="validation fraction", minimum=0.01, maximum=0.9)
    weight_decay = _number(body.get("weight_decay", 0.0001), label="weight decay", minimum=0.0, maximum=1.0)
    epochs = _integer(body.get("epochs", 30), label="epochs", minimum=1, maximum=10000)
    learning_rate = _number(body.get("learning_rate", 0.001), label="learning rate", minimum=1e-8, maximum=10.0)
    device = str(body.get("device") or "").strip().lower()
    if device not in {"", "cpu", "cuda", "mps"}:
        raise ValueError("device must be auto, cpu, cuda or mps")
    dataset_metadata = load_yaml(dataset / "metadata.yaml") if (dataset / "metadata.yaml").is_file() else {}
    dataset_task = str(dataset_metadata.get("task") or "control")
    experiment_task = str(EXPERIMENTS[experiment].get("task") or "control")
    if dataset_task != experiment_task:
        raise ValueError(
            f"{experiment} requires a {experiment_task} dataset, but {dataset.name} is {dataset_task}"
        )
    width = int(dataset_metadata.get("input_width") or body.get("input_width") or 212)
    height = int(dataset_metadata.get("input_height") or body.get("input_height") or 120)
    overrides = [
        f"experiment={experiment}",
        f"data.dataset_dir={dataset}",
        f"data.input_width={width}",
        f"data.input_height={height}",
        f"data.fraction={fraction}",
        f"run.name={run_name}",
        f"run.output_root={run_root(config)}",
        f"train.batch_size={batch_size}",
        f"train.num_workers={workers}",
        f"train.seed={seed}",
        f"train.val_fraction={val_fraction}",
        f"train.weight_decay={weight_decay}",
        f"train.device={device}",
        f"train.stages.0.epochs={epochs}",
        f"train.stages.0.lr={learning_rate}",
    ]
    for key in (
        "trajectory_horizon_sec",
        "trajectory_points",
        "trajectory_scale_m",
        "imu_window_sec",
        "imu_samples",
    ):
        if dataset_metadata.get(key) is not None:
            overrides.append(f"data.{key}={dataset_metadata[key]}")
    if str(EXPERIMENTS[experiment].get("model") or "") == "fusion":
        for key in ("trajectory_points", "trajectory_scale_m", "imu_samples"):
            if dataset_metadata.get(key) is not None:
                overrides.append(f"model.{key}={dataset_metadata[key]}")
    if int(EXPERIMENTS[experiment]["stages"]) == 2:
        finetune_epochs = _integer(body.get("finetune_epochs", epochs), label="fine-tune epochs", minimum=1, maximum=10000)
        finetune_lr = _number(body.get("finetune_learning_rate", 0.0001), label="fine-tune learning rate", minimum=1e-8, maximum=10.0)
        overrides.extend(
            [
                f"train.stages.1.epochs={finetune_epochs}",
                f"train.stages.1.lr={finetune_lr}",
            ]
        )
    command = _python_command(config, "e2e_learning.cli.train", overrides)
    return PipelineTaskSpec(
        kind="e2e-train",
        title=f"Train E2E model: {run_name}",
        command=command,
        cwd=str(training_root(config)),
        artifacts=[
            {"name": "run", "path": str(output)},
            {"name": "best checkpoint", "path": str(output / "checkpoints/best.pt")},
            {"name": "metrics", "path": str(output / "metrics.json")},
            {"name": "progress", "path": str(output / "progress.json")},
        ],
        resource_keys=[f"e2e-dataset:{dataset}", f"e2e-run:{output}"],
    )


def _resolve_run(config: Any, value: object) -> tuple[Path, dict[str, Any]]:
    directory = resolve_under_root(
        str(value or ""),
        run_root(config),
        label="training run",
        require_exists=True,
        require_directory=True,
    )
    run_config = load_yaml(directory / "run.yaml") if (directory / "run.yaml").is_file() else {}
    return directory, run_config


def build_export_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    directory, _ = _resolve_run(config, body.get("run_dir"))
    checkpoint = directory / "checkpoints" / "best.pt"
    if not checkpoint.is_file() or checkpoint.is_symlink():
        raise ValueError(f"best checkpoint was not found: {checkpoint}")
    overrides = [
        f"checkpoint={checkpoint}",
        f"export.output_dir={directory}",
    ]
    command = _python_command(config, "e2e_learning.cli.export_onnx", overrides)
    return PipelineTaskSpec(
        kind="e2e-export-onnx",
        title=f"Export E2E ONNX: {directory.name}",
        command=command,
        cwd=str(training_root(config)),
        artifacts=[
            {"name": "ONNX", "path": str(directory / "model.onnx")},
            {"name": "metadata", "path": str(directory / "metadata.json")},
        ],
        resource_keys=[f"e2e-run:{directory}"],
    )


def build_deploy_task(config: Any, body: dict[str, Any]) -> PipelineTaskSpec:
    model_path = Path(str(body.get("model_path") or "")).expanduser()
    allowed = None
    for model in scan_runs(config):
        if model.get("onnx_path") and Path(str(model["onnx_path"])).resolve(strict=False) == model_path.resolve(strict=False):
            allowed = Path(str(model["onnx_path"]))
            break
    if allowed is None or not allowed.is_file() or allowed.is_symlink():
        raise ValueError("model must be an exported model.onnx from an E2E training run")

    root = training_root(config)
    default_profile, profiles = _load_collection(root / "src/e2e_learning/conf/deploy_profiles.json", "profiles")
    default_preset, presets = _load_collection(root / "src/e2e_learning/conf/deploy_model_presets.json", "presets")
    profile_id = str(body.get("profile") or default_profile)
    preset_id = str(body.get("preset") or default_preset)
    profile = next((item for item in profiles if str(item.get("id")) == profile_id), None)
    if profile is None:
        raise ValueError(f"unknown deploy profile: {profile_id}")
    if not any(str(item.get("id")) == preset_id for item in presets):
        raise ValueError(f"unknown model preset: {preset_id}")
    user = str(body.get("user") or profile.get("user") or config.jetson_user)
    host = str(body.get("host") or profile.get("host") or "")
    if host == "__manual__":
        raise ValueError("host is required for the manual deploy profile")
    target = validate_ssh_target(user, host)
    remote_root = validate_remote_absolute_path(
        str(body.get("remote_root") or profile.get("remote_root") or "/workspaces/ros2_ws/models/e2e"),
        label="remote E2E model root",
    )
    command = [
        str(root / "scripts/deploy_model.sh"),
        "--onnx",
        str(allowed),
        "--profile",
        profile_id,
        "--preset",
        preset_id,
        "--user",
        user,
        "--host",
        host,
        "--remote-root",
        remote_root,
        "--yes",
    ]
    if bool(body.get("build_engine", True)):
        command.append("--build-engine")
    return PipelineTaskSpec(
        kind="e2e-deploy",
        title=f"Deploy E2E model to {target}",
        command=command,
        cwd=str(root),
        artifacts=[{"name": "source ONNX", "path": str(allowed)}],
        resource_keys=[f"e2e-run:{allowed.parent}", f"e2e-deploy:{target}:{remote_root}"],
    )
