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
from torch.utils.data import DataLoader, random_split
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm

from e2e_learning.data.dataset import ControlImageDataset
from e2e_learning.models.factory import build_model
from e2e_learning.utils.io import ensure_dir, write_json, write_yaml


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def split_dataset(dataset: ControlImageDataset, val_fraction: float, seed: int):
    val_size = max(1, int(len(dataset) * val_fraction))
    train_size = len(dataset) - val_size
    if train_size <= 0:
        raise RuntimeError("Dataset is too small for the requested validation split")
    generator = torch.Generator().manual_seed(seed)
    return random_split(dataset, [train_size, val_size], generator=generator)


def set_encoder_trainable(model: nn.Module, trainable: bool) -> None:
    if hasattr(model, "set_encoder_trainable"):
        model.set_encoder_trainable(trainable)


def train_epoch(model, loader, optimizer, loss_fn, device) -> dict[str, float]:
    model.train()
    total_loss = 0.0
    count = 0
    for x, y in tqdm(loader, desc="train", leave=False):
        x = x.to(device)
        y = y.to(device)
        optimizer.zero_grad(set_to_none=True)
        pred = model(x)
        loss = loss_fn(pred, y)
        loss.backward()
        optimizer.step()
        batch = x.shape[0]
        total_loss += float(loss.detach().cpu()) * batch
        count += batch
    return {"loss": total_loss / max(count, 1)}


@torch.no_grad()
def evaluate(model, loader, loss_fn, device) -> dict[str, float]:
    model.eval()
    total_loss = 0.0
    total_abs = torch.zeros(2)
    total_sq = torch.zeros(2)
    count = 0
    for x, y in tqdm(loader, desc="val", leave=False):
        x = x.to(device)
        y = y.to(device)
        pred = model(x)
        loss = loss_fn(pred, y)
        diff = (pred - y).detach().cpu()
        batch = x.shape[0]
        total_loss += float(loss.detach().cpu()) * batch
        total_abs += diff.abs().sum(dim=0)
        total_sq += (diff * diff).sum(dim=0)
        count += batch
    denom = max(count, 1)
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
) -> tuple[float, dict[str, float]]:
    set_encoder_trainable(model, not bool(stage.freeze_encoder))
    optimizer = torch.optim.AdamW(
        [p for p in model.parameters() if p.requires_grad],
        lr=float(stage.lr),
        weight_decay=float(getattr(stage, "weight_decay", cfg.train.weight_decay)),
    )
    loss_fn = nn.MSELoss()
    best_metrics: dict[str, float] = {}
    for epoch in range(1, int(stage.epochs) + 1):
        train_metrics = train_epoch(model, train_loader, optimizer, loss_fn, device)
        val_metrics = evaluate(model, val_loader, loss_fn, device)
        global_step = int(writer.get_logdir().split("_")[-1]) if False else epoch
        writer.add_scalar(f"{stage.name}/train_loss", train_metrics["loss"], global_step)
        for key, value in val_metrics.items():
            writer.add_scalar(f"{stage.name}/{key}", value, global_step)
        print(
            f"[{stage.name}] epoch={epoch} "
            f"train_loss={train_metrics['loss']:.6f} val_loss={val_metrics['loss']:.6f} "
            f"steer_mae={val_metrics['steering_mae']:.6f} throttle_mae={val_metrics['throttle_mae']:.6f}"
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
    output_dir = ensure_dir(Path(cfg.run.output_root) / str(cfg.run.name))
    ensure_dir(output_dir / "checkpoints")
    write_yaml(output_dir / "run.yaml", cfg)

    dataset = ControlImageDataset(
        dataset_dir=cfg.data.dataset_dir,
        input_width=int(cfg.data.input_width),
        input_height=int(cfg.data.input_height),
        mean=tuple(float(v) for v in cfg.data.mean),
        std=tuple(float(v) for v in cfg.data.std),
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
        best_loss, best_metrics = train_stage(
            cfg, stage, model, train_loader, val_loader, output_dir, writer, device, best_loss
        )
    writer.close()

    payload = {
        "run_name": str(cfg.run.name),
        "model": str(cfg.model.name),
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
