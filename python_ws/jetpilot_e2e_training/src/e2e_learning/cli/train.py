from __future__ import annotations

import json
import random
from pathlib import Path
from typing import Any

import hydra
import numpy as np
import torch
from omegaconf import DictConfig, OmegaConf
from torch import nn
from torch.utils.data import DataLoader, Subset
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm

from e2e_learning.data.dataset import E2EDataset
from e2e_learning.models.factory import build_model
from e2e_learning.utils.io import ensure_dir, write_json, write_yaml


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
    for key in (
        "input_width",
        "input_height",
        "trajectory_horizon_sec",
        "trajectory_points",
        "trajectory_scale_m",
        "imu_window_sec",
        "imu_samples",
    ):
        value = getattr(metadata, key, None)
        if value is not None:
            cfg.data[key] = value
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
    train_size = len(dataset) - val_size
    if train_size <= 0:
        raise RuntimeError("Dataset is too small for the requested validation split")
    return Subset(dataset, range(train_size)), Subset(dataset, range(train_size, len(dataset)))


def set_encoder_trainable(model: nn.Module, trainable: bool) -> None:
    if hasattr(model, "set_encoder_trainable"):
        model.set_encoder_trainable(trainable)


def _predict(model, images, imu, model_name: str, use_imu: bool):
    if model_name == "fusion":
        return model(images, imu if use_imu else None)
    return model(images[:, -1])


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
) -> tuple[float, dict[str, float]]:
    set_encoder_trainable(model, not bool(stage.freeze_encoder))
    optimizer = torch.optim.AdamW(
        [p for p in model.parameters() if p.requires_grad],
        lr=float(stage.lr),
        weight_decay=float(getattr(stage, "weight_decay", cfg.train.weight_decay)),
    )
    loss_fn = nn.SmoothL1Loss() if task == "trajectory" else nn.MSELoss()
    best_metrics: dict[str, float] = {}
    for epoch in range(1, int(stage.epochs) + 1):
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
            else f"steer_mae={val_metrics['steering_mae']:.6f} throttle_mae={val_metrics['throttle_mae']:.6f}"
        )
        print(
            f"[{stage.name}] epoch={epoch} train_loss={train_metrics['loss']:.6f} "
            f"val_loss={val_metrics['loss']:.6f} {detail}"
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
    dataset = E2EDataset(
        dataset_dir=cfg.data.dataset_dir,
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
        data_fraction=float(cfg.data.fraction),
    )
    train_set, val_set = split_dataset(dataset, float(cfg.train.val_fraction), int(cfg.train.seed))
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

    device = torch.device(str(cfg.train.device) if str(cfg.train.device) else "cuda" if torch.cuda.is_available() else "cpu")
    model = build_model(cfg.model).to(device)
    writer = SummaryWriter(log_dir=str(output_dir / "tensorboard"))
    best_loss = float("inf")
    best_metrics: dict[str, float] = {}

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
        },
        "dataset_dir": str(cfg.data.dataset_dir),
        "data_fraction": float(cfg.data.fraction),
        "best": best_metrics,
    }
    write_json(output_dir / "metrics.json", payload)
    write_json(
        output_dir / "progress.json",
        {
            "status": "complete",
            "run_name": str(cfg.run.name),
            "best": best_metrics,
        },
    )
    print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
