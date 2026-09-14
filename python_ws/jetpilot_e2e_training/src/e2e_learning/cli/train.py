from __future__ import annotations

import csv
import hashlib
import json
import random
from pathlib import Path
from typing import Any

import cv2
import hydra
import numpy as np
import torch
from omegaconf import DictConfig, OmegaConf
from torch import nn
from torch.utils.data import ConcatDataset, DataLoader, Dataset, Subset
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm

from e2e_learning.data.dataset import AsyncRgbEvsDataset, E2EDataset
from e2e_learning.data.transforms import ImageTransform
from e2e_learning.models.factory import build_model
from e2e_learning.utils.io import ensure_dir, write_json, write_yaml

ASYNC_RGB_EVS_MODELS = {"async_rgb_evs_control", "async_rgb_evs_dinov3_control"}


def prepare_dinov3_rgb_feature_cache(
    cfg: DictConfig, model: nn.Module, device: torch.device, dataset_dir: Path
) -> Path:
    """Cache frozen DINOv3 CLS features without changing the deployed graph."""
    samples_path = dataset_dir / "samples.csv"
    with samples_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    relative_paths = sorted({
        str(row[key])
        for row in rows
        for key in ("image_path", "next_image_path")
        if str(row.get(key) or "").strip()
    })
    weights_path = Path(str(getattr(cfg.model, "weights_path", ""))).expanduser()
    weights_identity: dict[str, Any] = {"path": str(weights_path)}
    if weights_path.is_file():
        stat = weights_path.stat()
        weights_identity.update({"size": stat.st_size, "mtime_ns": stat.st_mtime_ns})
    backbone_fields = (
        "image_size", "input_channels", "patch_size", "embed_dim", "depth",
        "num_heads", "ffn_ratio", "storage_tokens", "layer_scale", "rope_base",
        "rope_rescale_coords",
    )
    resolved_model = OmegaConf.to_container(cfg.model, resolve=True)
    signature = {
        "format": 1,
        "backbone": {
            key: resolved_model.get(key)
            for key in backbone_fields
        },
        "input_width": int(cfg.data.input_width),
        "input_height": int(cfg.data.input_height),
        "mean": [float(value) for value in cfg.data.mean],
        "std": [float(value) for value in cfg.data.std],
        "weights": weights_identity,
    }
    digest = hashlib.sha256(
        json.dumps(signature, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()[:16]
    cache_dir = dataset_dir / ".feature_cache" / f"dinov3-vits16-{digest}"
    cache_dir.mkdir(parents=True, exist_ok=True)
    transform = ImageTransform(
        int(cfg.data.input_width), int(cfg.data.input_height),
        tuple(float(value) for value in cfg.data.mean),
        tuple(float(value) for value in cfg.data.std),
    )
    batch_size = max(1, int(cfg.train.batch_size))
    missing = [
        relative for relative in relative_paths
        if not (cache_dir / relative).with_suffix(".npy").is_file()
    ]
    print(
        f"[rgb-feature-cache] path={cache_dir} images={len(relative_paths)} "
        f"missing={len(missing)}"
    )
    model.rgb_backbone.eval()
    with torch.inference_mode():
        for offset in tqdm(range(0, len(missing), batch_size), desc="cache RGB", leave=False):
            paths = missing[offset:offset + batch_size]
            images = []
            for relative in paths:
                image = cv2.imread(str(dataset_dir / relative), cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError(f"Failed to read RGB image for cache: {relative}")
                images.append(torch.from_numpy(transform(image)))
            inputs = torch.stack(images).to(device)
            features = model.rgb_backbone.forward_features(inputs)["x_norm_clstoken"]
            values = features.detach().to(dtype=torch.float32, device="cpu").numpy()
            for relative, value in zip(paths, values):
                destination = (cache_dir / relative).with_suffix(".npy")
                destination.parent.mkdir(parents=True, exist_ok=True)
                temporary = destination.with_suffix(".npy.tmp")
                with temporary.open("wb") as handle:
                    np.save(handle, value, allow_pickle=False)
                temporary.replace(destination)
    manifest = {**signature, "images": len(relative_paths), "dtype": "float32"}
    (cache_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return cache_dir


def combine_datasets(datasets: list[Dataset]) -> Dataset:
    if not datasets:
        raise RuntimeError("At least one dataset is required")
    return datasets[0] if len(datasets) == 1 else ConcatDataset(datasets)


def apply_combined_event_normalization(cfg: DictConfig, dataset_dirs: list[Path]) -> None:
    """Use training bags only to derive one normalization for every split."""
    if len(dataset_dirs) < 2 or str(getattr(cfg.data, "modality", "image")) not in {
        "event_tensor", "rgb_event_async",
    }:
        return
    weighted_sum: np.ndarray | None = None
    weighted_square_sum: np.ndarray | None = None
    total_weight = 0.0
    for directory in dataset_dirs:
        metadata = OmegaConf.load(directory / "metadata.yaml")
        mean = np.asarray(getattr(metadata, "event_mean", []), dtype=np.float64)
        std = np.asarray(getattr(metadata, "event_std", []), dtype=np.float64)
        if mean.size == 0 or mean.shape != std.shape:
            raise RuntimeError(f"Dataset has invalid event normalization: {directory}")
        weight = float(
            getattr(metadata, "normalization_tensor_count", 0)
            or getattr(metadata, "event_tensor_count", 0)
            or getattr(metadata, "sample_count", 1)
        )
        weighted_sum = mean * weight if weighted_sum is None else weighted_sum + mean * weight
        second_moment = std.square() + mean.square()
        weighted_square_sum = (
            second_moment * weight
            if weighted_square_sum is None
            else weighted_square_sum + second_moment * weight
        )
        total_weight += weight
    combined_mean = weighted_sum / max(total_weight, 1.0)
    combined_variance = np.maximum(
        weighted_square_sum / max(total_weight, 1.0) - combined_mean.square(), 0.0
    )
    combined_std = np.sqrt(combined_variance)
    combined_std[combined_std < 1.0e-6] = 1.0
    cfg.data.event_mean = combined_mean.tolist()
    cfg.data.event_std = combined_std.tolist()


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def apply_dataset_metadata(cfg: DictConfig) -> None:
    """Keep label geometry and sensor windows aligned with the extracted dataset."""
    metadata_path = Path(str(cfg.data.dataset_dir)) / "metadata.yaml"
    if not metadata_path.is_file():
        return
    metadata = OmegaConf.load(metadata_path)
    dataset_task = str(getattr(metadata, "task", "control"))
    model_task = str(getattr(cfg.model, "task", "control"))
    if dataset_task != model_task:
        raise RuntimeError(
            f"Dataset task is {dataset_task}, but model task is {model_task}: {metadata_path}"
        )
    fixed_input_width = getattr(cfg.model, "input_width", None)
    fixed_input_height = getattr(cfg.model, "input_height", None)
    has_fixed_geometry = fixed_input_width is not None and fixed_input_height is not None
    for key in (
        "image_topic",
        "modality",
        "event_topic",
        "event_bins",
        "event_window_ms",
        "event_stride_ms",
        "event_polarity_mode",
        "event_polarity_layout",
        "event_temporal_interpolation",
        "event_sample_hz",
        "rollout_steps",
        "event_mean",
        "event_std",
        "input_channels",
        "mean",
        "std",
        "input_width",
        "input_height",
        "trajectory_horizon_sec",
        "trajectory_points",
        "trajectory_scale_m",
        "imu_window_sec",
        "imu_samples",
    ):
        value = getattr(metadata, key, None)
        if value is not None and not (
            has_fixed_geometry and key in {"input_width", "input_height"}
        ):
            cfg.data[key] = value
    if has_fixed_geometry:
        cfg.data.input_width = int(fixed_input_width)
        cfg.data.input_height = int(fixed_input_height)
    if str(getattr(metadata, "modality", "image")) == "event_tensor":
        cfg.model.input_channels = int(getattr(metadata, "input_channels", 20))
    if str(getattr(metadata, "modality", "image")) == "rgb_event_async":
        cfg.model.event_channels = int(getattr(metadata, "event_channels", 20))
        cfg.model.rollout_steps = int(getattr(metadata, "rollout_steps", 8))
    if str(cfg.model.name) == "fusion":
        if getattr(metadata, "trajectory_points", None) is not None:
            cfg.model.trajectory_points = int(metadata.trajectory_points)
        if getattr(metadata, "trajectory_scale_m", None) is not None:
            cfg.model.trajectory_scale_m = float(metadata.trajectory_scale_m)
        if getattr(metadata, "imu_samples", None) is not None:
            cfg.model.imu_samples = int(metadata.imu_samples)


def split_dataset(dataset: E2EDataset, val_fraction: float, seed: int):
    del seed  # Temporal data must not be randomly interleaved across train and validation.
    val_size = max(1, int(len(dataset) * val_fraction))
    val_start = len(dataset) - val_size
    future_gap = dataset.future_horizon * dataset.future_stride
    train_size = val_start - future_gap
    if train_size <= 0:
        raise RuntimeError("Dataset is too small for the requested validation split")
    return Subset(dataset, range(train_size)), Subset(dataset, range(val_start, len(dataset)))


def set_encoder_trainable(model: nn.Module, trainable: bool) -> None:
    if hasattr(model, "set_encoder_trainable"):
        model.set_encoder_trainable(trainable)


def _predict(model, images, imu, model_name: str, use_imu: bool):
    if model_name == "fusion":
        return model(images, imu if use_imu else None)
    return model(images[:, -1])


class SteeringLoss(nn.Module):
    def forward(self, prediction, target):
        return nn.functional.mse_loss(prediction[:, 0:1], target[:, 0:1])


def train_epoch(model, loader, optimizer, loss_fn, device, model_name, use_imu) -> dict[str, float]:
    model.train()
    total_loss = 0.0
    count = 0
    for images, imu, y in tqdm(loader, desc="train", leave=False):
        images = images.to(device)
        imu = imu.to(device)
        y = y.to(device)
        optimizer.zero_grad(set_to_none=True)
        pred = _predict(model, images, imu, model_name, use_imu)
        loss = loss_fn(pred, y)
        loss.backward()
        optimizer.step()
        batch = images.shape[0]
        total_loss += float(loss.detach().cpu()) * batch
        count += batch
    return {"loss": total_loss / max(count, 1)}


def _wam_losses(model, batch, device, model_cfg) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
    images = batch["images"].to(device)
    history_actions = batch["history_actions"].to(device)
    control = batch["control"].to(device)
    future_images = batch["future_images"].to(device)
    future_actions = batch["future_actions"].to(device)
    mask = batch["future_mask"].to(device)
    outputs = model.forward_train(
        images,
        history_actions,
        control,
        future_actions,
    )
    target_latents = model.encode_target(future_images)
    learned_slice = slice(0, 1) if model.steering_only else slice(0, 2)
    current_action_loss = nn.functional.mse_loss(
        outputs["control"][:, learned_slice], control[:, learned_slice]
    )
    future_action_error = (
        outputs["future_controls"][:, :, learned_slice]
        - future_actions[:, :, learned_slice]
    ).square().mean(dim=-1)
    future_latent_error = 1.0 - nn.functional.cosine_similarity(
        outputs["future_latents"], target_latents, dim=-1
    )
    denominator = mask.sum().clamp_min(1.0)
    future_action_loss = (future_action_error * mask).sum() / denominator
    future_latent_loss = (future_latent_error * mask).sum() / denominator
    loss = (
        float(getattr(model_cfg, "current_action_loss_weight", 1.0)) * current_action_loss
        + float(getattr(model_cfg, "future_action_loss_weight", 0.5)) * future_action_loss
        + float(getattr(model_cfg, "future_latent_loss_weight", 1.0)) * future_latent_loss
    )
    return loss, {
        "current_action_loss": current_action_loss,
        "future_action_loss": future_action_loss,
        "future_latent_loss": future_latent_loss,
        "steering_abs": (outputs["control"][:, 0] - control[:, 0]).abs().sum(),
        "throttle_abs": (outputs["control"][:, 1] - control[:, 1]).abs().sum(),
    }


def train_wam_epoch(model, loader, optimizer, device, model_cfg) -> dict[str, float]:
    model.train()
    totals = {key: 0.0 for key in ("loss", "current_action_loss", "future_action_loss", "future_latent_loss")}
    count = 0
    for batch in tqdm(loader, desc="train", leave=False):
        optimizer.zero_grad(set_to_none=True)
        loss, parts = _wam_losses(model, batch, device, model_cfg)
        loss.backward()
        optimizer.step()
        batch_size = int(batch["images"].shape[0])
        totals["loss"] += float(loss.detach().cpu()) * batch_size
        for key in ("current_action_loss", "future_action_loss", "future_latent_loss"):
            totals[key] += float(parts[key].detach().cpu()) * batch_size
        count += batch_size
    return {key: value / max(count, 1) for key, value in totals.items()}


def _async_rgb_evs_losses(model, batch, device, model_cfg):
    rgb = batch["rgb"].to(device)
    next_rgb = batch["next_rgb"].to(device)
    events = batch["events"].to(device)
    delta_t = batch["delta_t"].to(device)
    mask = batch["mask"].to(device)
    targets = batch["controls"].to(device)
    controls, final_state = model.rollout(rgb, events, delta_t, mask)
    with torch.no_grad():
        target_state = model.encode_rgb(next_rgb)
    learned = slice(0, 1) if model.steering_only else slice(0, 2)
    error = (controls[:, :, learned] - targets[:, :, learned]).square().mean(dim=-1)
    denominator = mask.sum().clamp_min(1.0)
    control_loss = (error * mask).sum() / denominator
    latent_loss = (1.0 - nn.functional.cosine_similarity(final_state, target_state, dim=-1)).mean()
    if controls.shape[1] > 1:
        pair_mask = mask[:, 1:] * mask[:, :-1]
        smooth_error = (controls[:, 1:, learned] - controls[:, :-1, learned]).square().mean(dim=-1)
        smooth_loss = (smooth_error * pair_mask).sum() / pair_mask.sum().clamp_min(1.0)
    else:
        smooth_loss = control_loss.new_zeros(())
    loss = (
        float(getattr(model_cfg, "control_loss_weight", 1.0)) * control_loss
        + float(getattr(model_cfg, "latent_loss_weight", 0.25)) * latent_loss
        + float(getattr(model_cfg, "smooth_loss_weight", 0.02)) * smooth_loss
    )
    absolute = (controls - targets).abs() * mask[:, :, None]
    return loss, {
        "control_loss": control_loss,
        "latent_loss": latent_loss,
        "smooth_loss": smooth_loss,
        "steering_abs": absolute[:, :, 0].sum(),
        "throttle_abs": absolute[:, :, 1].sum(),
        "valid_steps": mask.sum(),
    }


def run_async_rgb_evs_epoch(model, loader, device, model_cfg, optimizer=None) -> dict[str, float]:
    model.train(optimizer is not None)
    totals = {key: 0.0 for key in ("loss", "control_loss", "latent_loss", "smooth_loss")}
    steering_abs = throttle_abs = valid_steps = 0.0
    for batch in tqdm(loader, desc="train" if optimizer else "validate", leave=False):
        if optimizer is not None:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(optimizer is not None):
            loss, parts = _async_rgb_evs_losses(model, batch, device, model_cfg)
            if optimizer is not None:
                loss.backward()
                optimizer.step()
        batch_size = int(batch["rgb"].shape[0])
        totals["loss"] += float(loss.detach().cpu()) * batch_size
        for key in ("control_loss", "latent_loss", "smooth_loss"):
            totals[key] += float(parts[key].detach().cpu()) * batch_size
        steering_abs += float(parts["steering_abs"].detach().cpu())
        throttle_abs += float(parts["throttle_abs"].detach().cpu())
        valid_steps += float(parts["valid_steps"].detach().cpu())
    samples = max(1, len(loader.dataset))
    result = {
        **{key: value / samples for key, value in totals.items()},
        "steering_mae": steering_abs / max(valid_steps, 1.0),
    }
    if not model.steering_only:
        result["throttle_mae"] = throttle_abs / max(valid_steps, 1.0)
    return result


@torch.no_grad()
def evaluate_wam(model, loader, device, model_cfg) -> dict[str, float]:
    model.eval()
    totals = {key: 0.0 for key in ("loss", "current_action_loss", "future_action_loss", "future_latent_loss")}
    steering_abs = 0.0
    throttle_abs = 0.0
    count = 0
    for batch in tqdm(loader, desc="val", leave=False):
        loss, parts = _wam_losses(model, batch, device, model_cfg)
        batch_size = int(batch["images"].shape[0])
        totals["loss"] += float(loss.cpu()) * batch_size
        for key in ("current_action_loss", "future_action_loss", "future_latent_loss"):
            totals[key] += float(parts[key].cpu()) * batch_size
        steering_abs += float(parts["steering_abs"].cpu())
        throttle_abs += float(parts["throttle_abs"].cpu())
        count += batch_size
    result = {key: value / max(count, 1) for key, value in totals.items()}
    result["steering_mae"] = steering_abs / max(count, 1)
    if not model.steering_only:
        result["throttle_mae"] = throttle_abs / max(count, 1)
    return result


@torch.no_grad()
def evaluate(model, loader, loss_fn, device, model_name, use_imu, task, trajectory_scale_m) -> dict[str, float]:
    model.eval()
    total_loss = 0.0
    total_abs = torch.zeros(2)
    total_sq = torch.zeros(2)
    trajectory_distance_sum = 0.0
    trajectory_final_sum = 0.0
    trajectory_lateral_sum = 0.0
    trajectory_point_count = 0
    count = 0
    for images, imu, y in tqdm(loader, desc="val", leave=False):
        images = images.to(device)
        imu = imu.to(device)
        y = y.to(device)
        pred = _predict(model, images, imu, model_name, use_imu)
        loss = loss_fn(pred, y)
        diff = (pred - y).detach().cpu()
        batch = images.shape[0]
        total_loss += float(loss.detach().cpu()) * batch
        if task == "control":
            total_abs += diff.abs().sum(dim=0)
            total_sq += (diff * diff).sum(dim=0)
        else:
            distance = torch.linalg.vector_norm(diff * trajectory_scale_m, dim=-1)
            trajectory_distance_sum += float(distance.sum())
            trajectory_final_sum += float(distance[:, -1].sum())
            trajectory_lateral_sum += float((diff[..., 1] * trajectory_scale_m).abs().sum())
            trajectory_point_count += int(distance.numel())
        count += batch
    denom = max(count, 1)
    if task == "trajectory":
        return {
            "loss": total_loss / denom,
            "trajectory_ade_m": trajectory_distance_sum / max(trajectory_point_count, 1),
            "trajectory_fde_m": trajectory_final_sum / denom,
            "trajectory_lateral_mae_m": trajectory_lateral_sum / max(trajectory_point_count, 1),
        }
    rmse = torch.sqrt(total_sq / denom)
    mae = total_abs / denom
    if getattr(model, "steering_only", False):
        return {
            "loss": total_loss / denom,
            "steering_mae": float(mae[0]),
            "steering_rmse": float(rmse[0]),
        }
    return {
        "loss": total_loss / denom,
        "steering_mae": float(mae[0]),
        "throttle_mae": float(mae[1]),
        "steering_rmse": float(rmse[0]),
        "throttle_rmse": float(rmse[1]),
    }


def train_stage(
    cfg: DictConfig,
    stage: Any,
    model: nn.Module,
    train_loader,
    val_loader,
    output_dir: Path,
    writer: SummaryWriter,
    device: torch.device,
    best_loss: float,
    model_name: str,
    use_imu: bool,
    task: str,
    trajectory_scale_m: float,
    history: list[dict[str, Any]],
) -> tuple[float, dict[str, float]]:
    set_encoder_trainable(model, not bool(stage.freeze_encoder))
    optimizer = torch.optim.AdamW(
        [p for p in model.parameters() if p.requires_grad],
        lr=float(stage.lr),
        weight_decay=float(getattr(stage, "weight_decay", cfg.train.weight_decay)),
    )
    loss_fn = nn.SmoothL1Loss() if task == "trajectory" else nn.MSELoss()
    if getattr(model, "steering_only", False):
        loss_fn = SteeringLoss()
    best_metrics: dict[str, float] = {}
    for epoch in range(1, int(stage.epochs) + 1):
        if model_name in ASYNC_RGB_EVS_MODELS:
            train_metrics = run_async_rgb_evs_epoch(model, train_loader, device, cfg.model, optimizer)
            val_metrics = run_async_rgb_evs_epoch(model, val_loader, device, cfg.model)
        elif model_name == "wam_dinov3_vits16":
            train_metrics = train_wam_epoch(model, train_loader, optimizer, device, cfg.model)
            val_metrics = evaluate_wam(model, val_loader, device, cfg.model)
        else:
            train_metrics = train_epoch(
                model, train_loader, optimizer, loss_fn, device, model_name, use_imu
            )
            val_metrics = evaluate(
                model,
                val_loader,
                loss_fn,
                device,
                model_name,
                use_imu,
                task,
                trajectory_scale_m,
            )
        global_step = int(writer.get_logdir().split("_")[-1]) if False else epoch
        writer.add_scalar(f"{stage.name}/train_loss", train_metrics["loss"], global_step)
        for key, value in val_metrics.items():
            writer.add_scalar(f"{stage.name}/{key}", value, global_step)
        detail = (
            f"ADE={val_metrics['trajectory_ade_m']:.4f}m FDE={val_metrics['trajectory_fde_m']:.4f}m"
            if task == "trajectory"
            else f"steer_mae={val_metrics['steering_mae']:.6f}"
            + (f" throttle_mae={val_metrics['throttle_mae']:.6f}" if "throttle_mae" in val_metrics else "")
        )
        print(
            f"[{stage.name}] epoch={epoch} train_loss={train_metrics['loss']:.6f} "
            f"val_loss={val_metrics['loss']:.6f} {detail}"
        )
        history.append(
            {
                "stage": str(stage.name),
                "epoch": epoch,
                "train_loss": float(train_metrics["loss"]),
                "validation_loss": float(val_metrics["loss"]),
                "validation": {key: float(value) for key, value in val_metrics.items()},
            }
        )

        checkpoint = {
            "model_state": model.state_dict(),
            "cfg": OmegaConf.to_container(cfg, resolve=True),
            "stage": str(stage.name),
            "epoch": epoch,
            "metrics": val_metrics,
        }
        write_json(
            output_dir / "progress.json",
            {
                "status": "running",
                "run_name": str(cfg.run.name),
                "stage": str(stage.name),
                "epoch": epoch,
                "epochs_in_stage": int(stage.epochs),
                "train": train_metrics,
                "validation": val_metrics,
                "history": history,
            },
        )
        torch.save(checkpoint, output_dir / "checkpoints" / "last.pt")
        if val_metrics["loss"] < best_loss:
            best_loss = val_metrics["loss"]
            best_metrics = val_metrics
            torch.save(checkpoint, output_dir / "checkpoints" / "best.pt")
    return best_loss, best_metrics


@hydra.main(config_path="../conf", config_name="config", version_base=None)
def main(cfg: DictConfig) -> None:
    set_seed(int(cfg.train.seed))
    apply_dataset_metadata(cfg)
    output_dir = ensure_dir(Path(cfg.run.output_root) / str(cfg.run.name))
    ensure_dir(output_dir / "checkpoints")
    write_yaml(output_dir / "run.yaml", cfg)

    task = str(getattr(cfg.model, "task", "control"))
    model_name = str(cfg.model.name)
    use_imu = bool(getattr(cfg.model, "use_imu", False))
    trajectory_points = int(getattr(cfg.model, "trajectory_points", getattr(cfg.data, "trajectory_points", 10)))
    trajectory_scale_m = float(getattr(cfg.model, "trajectory_scale_m", getattr(cfg.data, "trajectory_scale_m", 5.0)))
    device = torch.device(
        str(cfg.train.device)
        if str(cfg.train.device)
        else "cuda" if torch.cuda.is_available() else "cpu"
    )
    model = build_model(cfg.model).to(device)
    train_dataset_dirs = [
        Path(str(value)) for value in getattr(cfg.data, "dataset_dirs", []) if str(value)
    ] or [Path(str(cfg.data.dataset_dir))]
    validation_dataset_dirs = [
        Path(str(value))
        for value in getattr(cfg.data, "validation_dataset_dirs", []) if str(value)
    ]
    split_mode = str(getattr(cfg.data, "split_mode", "temporal"))
    if split_mode not in {"temporal", "explicit"}:
        raise RuntimeError("data.split_mode must be temporal or explicit")
    if split_mode == "explicit" and not validation_dataset_dirs:
        raise RuntimeError("explicit split requires validation_dataset_dirs")
    if set(path.resolve() for path in train_dataset_dirs).intersection(
        path.resolve() for path in validation_dataset_dirs
    ):
        raise RuntimeError("training and validation datasets must be different")
    apply_combined_event_normalization(cfg, train_dataset_dirs)
    # Persist the resolved multi-dataset split and training-only normalization.
    write_yaml(output_dir / "run.yaml", cfg)
    if model_name in ASYNC_RGB_EVS_MODELS:
        timing_augmentation = bool(getattr(cfg.data, "timing_augmentation", True))
        timing_max_hz = float(getattr(cfg.data, "timing_max_hz", 250.0))
        source_event_hz = float(getattr(cfg.data, "event_sample_hz", 0.0))
        if timing_augmentation and source_event_hz + 1.0e-6 < timing_max_hz:
            raise RuntimeError(
                f"250 Hz timing augmentation requires a dataset extracted at >= "
                f"{timing_max_hz:g} Hz; this dataset is {source_event_hz:g} Hz"
            )
        use_rgb_feature_cache = bool(getattr(cfg.data, "rgb_feature_cache", False))
        if use_rgb_feature_cache and model_name != "async_rgb_evs_dinov3_control":
            raise RuntimeError(
                "RGB feature cache is only supported by async_rgb_evs_dinov3_control"
            )
        cache_dirs = {
            path: prepare_dinov3_rgb_feature_cache(cfg, model, device, path)
            for path in [*train_dataset_dirs, *validation_dataset_dirs]
        } if use_rgb_feature_cache else {}
        async_dataset_args = {
            "input_width": int(cfg.data.input_width),
            "input_height": int(cfg.data.input_height),
            "mean": tuple(float(v) for v in cfg.data.mean),
            "std": tuple(float(v) for v in cfg.data.std),
            "rollout_steps": int(getattr(cfg.model, "rollout_steps", 32)),
            "event_mean": tuple(float(v) for v in getattr(cfg.data, "event_mean", [0.0])),
            "event_std": tuple(float(v) for v in getattr(cfg.data, "event_std", [1.0])),
            "timing_min_hz": float(getattr(cfg.data, "timing_min_hz", 100.0)),
            "timing_max_hz": timing_max_hz,
            "timing_jitter_fraction": float(
                getattr(cfg.data, "timing_jitter_fraction", 0.15)
            ),
            "event_update_drop_probability": float(
                getattr(cfg.data, "event_update_drop_probability", 0.05)
            ),
            "rgb_update_drop_probability": float(
                getattr(cfg.data, "rgb_update_drop_probability", 0.10)
            ),
            "max_rgb_interval_multiplier": int(
                getattr(cfg.data, "max_rgb_interval_multiplier", 2)
            ),
        }
        def make_async(path: Path, *, augment: bool, fraction: float) -> AsyncRgbEvsDataset:
            return AsyncRgbEvsDataset(
                **async_dataset_args, dataset_dir=path, data_fraction=fraction,
                augment_timing=augment, rgb_feature_cache_dir=cache_dirs.get(path),
            )

        if split_mode == "explicit":
            train_set = combine_datasets([
                make_async(path, augment=timing_augmentation, fraction=float(cfg.data.fraction))
                for path in train_dataset_dirs
            ])
            val_set = combine_datasets([
                make_async(path, augment=False, fraction=1.0)
                for path in validation_dataset_dirs
            ])
        else:
            train_parts: list[Dataset] = []
            val_parts: list[Dataset] = []
            for path in train_dataset_dirs:
                train_dataset = make_async(
                    path, augment=timing_augmentation, fraction=float(cfg.data.fraction)
                )
                validation_dataset = make_async(
                    path, augment=False, fraction=float(cfg.data.fraction)
                )
                train_part, _ = split_dataset(
                    train_dataset, float(cfg.train.val_fraction), int(cfg.train.seed)
                )
                _, val_part = split_dataset(
                    validation_dataset, float(cfg.train.val_fraction), int(cfg.train.seed)
                )
                train_parts.append(train_part)
                val_parts.append(val_part)
            train_set = combine_datasets(train_parts)
            val_set = combine_datasets(val_parts)
    else:
        def make_standard(path: Path, fraction: float) -> E2EDataset:
            return E2EDataset(
                dataset_dir=path,
                input_width=int(cfg.data.input_width),
                input_height=int(cfg.data.input_height),
                mean=tuple(float(v) for v in cfg.data.mean),
                std=tuple(float(v) for v in cfg.data.std),
                task=task,
                sequence_length=int(getattr(cfg.model, "sequence_length", 1)),
                frame_stride=int(getattr(cfg.model, "frame_stride", 1)),
                trajectory_points=trajectory_points,
                trajectory_scale_m=trajectory_scale_m,
                imu_samples=int(getattr(cfg.model, "imu_samples", getattr(cfg.data, "imu_samples", 10))),
                imu_features=int(getattr(cfg.model, "imu_features", 7)),
                data_fraction=fraction,
                future_horizon=(
                    int(getattr(cfg.model, "future_horizon", 0))
                    if model_name == "wam_dinov3_vits16" else 0
                ),
                future_stride=int(getattr(cfg.model, "future_stride", 1)),
            )

        if split_mode == "explicit":
            train_set = combine_datasets([
                make_standard(path, float(cfg.data.fraction)) for path in train_dataset_dirs
            ])
            val_set = combine_datasets([
                make_standard(path, 1.0) for path in validation_dataset_dirs
            ])
        else:
            train_parts = []
            val_parts = []
            for path in train_dataset_dirs:
                dataset = make_standard(path, float(cfg.data.fraction))
                train_part, val_part = split_dataset(
                    dataset, float(cfg.train.val_fraction), int(cfg.train.seed)
                )
                train_parts.append(train_part)
                val_parts.append(val_part)
            train_set = combine_datasets(train_parts)
            val_set = combine_datasets(val_parts)
    train_loader = DataLoader(
        train_set,
        batch_size=int(cfg.train.batch_size),
        shuffle=True,
        num_workers=int(cfg.train.num_workers),
        pin_memory=torch.cuda.is_available(),
    )
    val_loader = DataLoader(
        val_set,
        batch_size=int(cfg.train.batch_size),
        shuffle=False,
        num_workers=int(cfg.train.num_workers),
        pin_memory=torch.cuda.is_available(),
    )

    writer = SummaryWriter(log_dir=str(output_dir / "tensorboard"))
    best_loss = float("inf")
    best_metrics: dict[str, float] = {}
    history: list[dict[str, Any]] = []

    for stage in cfg.train.stages:
        best_loss, stage_best_metrics = train_stage(
            cfg,
            stage,
            model,
            train_loader,
            val_loader,
            output_dir,
            writer,
            device,
            best_loss,
            model_name,
            use_imu,
            task,
            trajectory_scale_m,
            history,
        )
        if stage_best_metrics:
            best_metrics = stage_best_metrics
    writer.close()

    payload = {
        "run_name": str(cfg.run.name),
        "model": str(cfg.model.name),
        "task": task,
        "architecture": {
            "backbone": str(getattr(cfg.model, "backbone", cfg.model.name)),
            "temporal": str(getattr(cfg.model, "temporal", "none")),
            "use_imu": use_imu,
            "sequence_length": int(getattr(cfg.model, "sequence_length", 1)),
            "future_horizon": int(getattr(cfg.model, "future_horizon", 0)),
            "future_stride": int(getattr(cfg.model, "future_stride", 1)),
            "stateful_step": model_name == "wam_dinov3_vits16",
        },
        "dataset_dir": str(cfg.data.dataset_dir),
        "dataset_dirs": [str(value) for value in train_dataset_dirs],
        "validation_dataset_dirs": [str(value) for value in validation_dataset_dirs],
        "split_mode": split_mode,
        "data_fraction": float(cfg.data.fraction),
        "best": best_metrics,
        "history": history,
    }
    write_json(output_dir / "metrics.json", payload)
    write_json(
        output_dir / "progress.json",
        {
            "status": "complete",
            "run_name": str(cfg.run.name),
            "best": best_metrics,
            "history": history,
        },
    )
    print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
