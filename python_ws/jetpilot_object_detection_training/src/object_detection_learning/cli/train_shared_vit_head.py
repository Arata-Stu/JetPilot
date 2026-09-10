from __future__ import annotations

import argparse
import json
import random
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

import numpy as np
import torch
import torch.nn.functional as functional
from PIL import Image
from torch import Tensor
from torch.utils.data import DataLoader, Dataset

from e2e_learning.models.dinov3_multitask import (
    DinoV3ViTSmallMultiTask,
    backbone_fingerprint,
    detection_state_dict,
)
from e2e_learning.models.dinov3_vit import load_dinov3_backbone_weights
from object_detection_learning.contract import dataset_root, load_dataset, sha256_file


IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".webp"})
MEAN = (0.485, 0.456, 0.406)
STD = (0.229, 0.224, 0.225)


class YoloDataset(Dataset):
    def __init__(
        self, yaml_path: Path, split: str, width: int, height: int,
        augment: bool, mean: Sequence[float], std: Sequence[float]
    ) -> None:
        _, config, _ = load_dataset(yaml_path)
        root = dataset_root(yaml_path, config)
        split_value = config.get(split)
        if not isinstance(split_value, str) or not split_value.strip():
            raise ValueError(f"dataset has no {split} split")
        image_root = Path(split_value).expanduser()
        if not image_root.is_absolute():
            image_root = root / image_root
        self.images = sorted(
            path for path in image_root.resolve().rglob("*")
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        )
        if not self.images:
            raise ValueError(f"no images found in {image_root}")
        self.width = width
        self.height = height
        self.augment = augment
        self.mean = tuple(float(value) for value in mean)
        self.std = tuple(float(value) for value in std)

    @staticmethod
    def _label_path(image_path: Path) -> Path:
        parts = list(image_path.parts)
        for index in range(len(parts) - 1, -1, -1):
            if parts[index] == "images":
                parts[index] = "labels"
                return Path(*parts).with_suffix(".txt")
        return image_path.parent.parent / "labels" / image_path.with_suffix(".txt").name

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, index: int) -> tuple[Tensor, Tensor]:
        image_path = self.images[index]
        with Image.open(image_path) as source:
            image = source.convert("RGB").resize((self.width, self.height), Image.Resampling.BILINEAR)
            array = np.asarray(image, dtype=np.float32) / 255.0
        labels: list[list[float]] = []
        label_path = self._label_path(image_path)
        if label_path.is_file():
            for line in label_path.read_text(encoding="utf-8", errors="replace").splitlines():
                fields = line.split()
                if len(fields) != 5:
                    continue
                class_id, center_x, center_y, width, height = (float(value) for value in fields)
                labels.append([class_id, center_x, center_y, width, height])
        if self.augment and random.random() < 0.5:
            array = np.ascontiguousarray(array[:, ::-1])
            for label in labels:
                label[1] = 1.0 - label[1]
        tensor = torch.from_numpy(array).permute(2, 0, 1)
        mean = torch.tensor(self.mean, dtype=tensor.dtype).view(3, 1, 1)
        std = torch.tensor(self.std, dtype=tensor.dtype).view(3, 1, 1)
        target = torch.tensor(labels, dtype=torch.float32).reshape(-1, 5)
        return (tensor - mean) / std, target


def collate_batch(batch: list[tuple[Tensor, Tensor]]) -> tuple[Tensor, list[Tensor]]:
    return torch.stack([item[0] for item in batch]), [item[1] for item in batch]


def _xywh_to_xyxy(boxes: Tensor) -> Tensor:
    center_x, center_y, width, height = boxes.unbind(dim=-1)
    return torch.stack(
        (center_x - width * 0.5, center_y - height * 0.5,
         center_x + width * 0.5, center_y + height * 0.5), dim=-1
    )


def _aligned_iou(prediction: Tensor, target: Tensor) -> Tensor:
    prediction = _xywh_to_xyxy(prediction)
    target = _xywh_to_xyxy(target)
    top_left = torch.maximum(prediction[:, :2], target[:, :2])
    bottom_right = torch.minimum(prediction[:, 2:], target[:, 2:])
    intersection = (bottom_right - top_left).clamp(min=0).prod(dim=1)
    pred_area = (prediction[:, 2:] - prediction[:, :2]).clamp(min=0).prod(dim=1)
    target_area = (target[:, 2:] - target[:, :2]).clamp(min=0).prod(dim=1)
    return intersection / (pred_area + target_area - intersection + 1.0e-6)


class FrozenBackboneDetectionLoss:
    def __init__(self, model: DinoV3ViTSmallMultiTask) -> None:
        self.model = model

    def __call__(self, raw_levels: Sequence[Tensor], targets: list[Tensor]) -> tuple[Tensor, dict[str, float]]:
        decoded = self.model.detection_head.decode(raw_levels)
        boxes = decoded[:, :4].transpose(1, 2)
        class_logits = torch.cat(
            [raw[:, 4:].flatten(2).transpose(1, 2) for raw in raw_levels], dim=1
        )
        class_targets = torch.zeros_like(class_logits)
        box_targets = torch.zeros_like(boxes)
        positive = torch.zeros(boxes.shape[:2], dtype=torch.bool, device=boxes.device)
        level_offsets: list[int] = []
        offset = 0
        for raw in raw_levels:
            level_offsets.append(offset)
            offset += raw.shape[-2] * raw.shape[-1]
        pad_left = (self.model.backbone.padded_width - self.model.backbone.input_width) // 2
        pad_top = (self.model.backbone.padded_height - self.model.backbone.input_height) // 2
        for batch_index, labels in enumerate(targets):
            for label in labels.to(boxes.device):
                class_id = int(label[0].item())
                center_x = label[1] * self.model.backbone.input_width + pad_left
                center_y = label[2] * self.model.backbone.input_height + pad_top
                width = label[3] * self.model.backbone.input_width
                height = label[4] * self.model.backbone.input_height
                size = float(torch.maximum(width, height).detach().cpu())
                level = 0 if size <= 32.0 else 1 if size <= 64.0 else 2
                stride = self.model.detection_head.strides[level]
                grid_width = raw_levels[level].shape[-1]
                grid_height = raw_levels[level].shape[-2]
                cell_x = min(grid_width - 1, max(0, int(float(center_x) / stride)))
                cell_y = min(grid_height - 1, max(0, int(float(center_y) / stride)))
                candidate = level_offsets[level] + cell_y * grid_width + cell_x
                positive[batch_index, candidate] = True
                class_targets[batch_index, candidate, class_id] = 1.0
                box_targets[batch_index, candidate] = torch.stack((center_x, center_y, width, height))
        probability = class_logits.sigmoid()
        cross_entropy = functional.binary_cross_entropy_with_logits(
            class_logits, class_targets, reduction="none"
        )
        focal_weight = (class_targets - probability).abs().pow(2.0)
        class_loss = (cross_entropy * focal_weight).sum() / max(int(positive.sum()), 1)
        if positive.any():
            predicted_positive = boxes[positive]
            target_positive = box_targets[positive]
            iou_loss = (1.0 - _aligned_iou(predicted_positive, target_positive)).mean()
            l1_loss = functional.smooth_l1_loss(
                predicted_positive / 32.0, target_positive / 32.0
            )
        else:
            iou_loss = boxes.sum() * 0.0
            l1_loss = boxes.sum() * 0.0
        total = class_loss + 5.0 * iou_loss + l1_loss
        return total, {
            "loss": float(total.detach().cpu()),
            "class_loss": float(class_loss.detach().cpu()),
            "iou_loss": float(iou_loss.detach().cpu()),
            "box_l1_loss": float(l1_loss.detach().cpu()),
            "positives": float(positive.sum().detach().cpu()),
        }


def _run_epoch(model, loader, loss_function, device, optimizer=None) -> dict[str, float]:
    training = optimizer is not None
    model.train(training)
    totals: dict[str, float] = {}
    batches = 0
    context = torch.enable_grad() if training else torch.no_grad()
    with context:
        for images, targets in loader:
            images = images.to(device)
            if training:
                optimizer.zero_grad(set_to_none=True)
            raw = model.forward_detection_raw(images)
            loss, metrics = loss_function(raw, targets)
            if training:
                loss.backward()
                optimizer.step()
            for key, value in metrics.items():
                totals[key] = totals.get(key, 0.0) + value
            batches += 1
    return {key: value / max(batches, 1) for key, value in totals.items()}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Train only the detection neck/head on a frozen DINOv3 ViT-S/16")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--backbone-weights", type=Path, required=True)
    parser.add_argument("--project", type=Path, default=Path("outputs/shared_vit_detection"))
    parser.add_argument("--name", default="vit-dinov3-vits16-det-head")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--lr", type=float, default=1.0e-3)
    parser.add_argument("--weight-decay", type=float, default=1.0e-4)
    parser.add_argument("--input-width", type=int, default=212)
    parser.add_argument("--input-height", type=int, default=120)
    parser.add_argument("--mean", type=float, nargs=3, default=MEAN)
    parser.add_argument("--std", type=float, nargs=3, default=STD)
    parser.add_argument("--modality", choices=("rgb", "event_image"), default="rgb")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    dataset_path, _, classes = load_dataset(args.data)
    backbone_path = args.backbone_weights.expanduser().resolve()
    if not backbone_path.is_file():
        raise SystemExit(f"error: backbone weights not found: {backbone_path}")
    device = torch.device(args.device if args.device != "cuda" or torch.cuda.is_available() else "cpu")
    model = DinoV3ViTSmallMultiTask(
        input_width=args.input_width,
        input_height=args.input_height,
        num_classes=len(classes),
    )
    loaded, total = load_dinov3_backbone_weights(backbone_path, model.backbone)
    for parameter in model.parameters():
        parameter.requires_grad = False
    for module in (model.detection_neck, model.detection_head):
        for parameter in module.parameters():
            parameter.requires_grad = True
    model.to(device)
    train_data = YoloDataset(
        dataset_path, "train", args.input_width, args.input_height, True, args.mean, args.std
    )
    val_data = YoloDataset(
        dataset_path, "val", args.input_width, args.input_height, False, args.mean, args.std
    )
    train_loader = DataLoader(train_data, batch_size=args.batch, shuffle=True, num_workers=args.workers, collate_fn=collate_batch)
    val_loader = DataLoader(val_data, batch_size=args.batch, shuffle=False, num_workers=args.workers, collate_fn=collate_batch)
    optimizer = torch.optim.AdamW(
        [parameter for parameter in model.parameters() if parameter.requires_grad],
        lr=args.lr,
        weight_decay=args.weight_decay,
    )
    loss_function = FrozenBackboneDetectionLoss(model)
    output = (args.project.expanduser() / args.name).resolve()
    weights = output / "weights"
    weights.mkdir(parents=True, exist_ok=True)
    best_loss = float("inf")
    history: list[dict[str, object]] = []
    backbone_sha256 = sha256_file(backbone_path)
    frozen_backbone_fingerprint = backbone_fingerprint(model.backbone)
    for epoch in range(1, args.epochs + 1):
        train_metrics = _run_epoch(model, train_loader, loss_function, device, optimizer)
        validation_metrics = _run_epoch(model, val_loader, loss_function, device)
        record = {"epoch": epoch, "train": train_metrics, "validation": validation_metrics}
        history.append(record)
        checkpoint = {
            "format_version": 1,
            "kind": "dinov3_vits16_detection_head",
            "detection_state": detection_state_dict(model),
            "backbone": {"path": str(backbone_path), "sha256": backbone_sha256, "fingerprint": frozen_backbone_fingerprint, "loaded_tensors": loaded, "total_tensors": total},
            "model": {
                "input_width": args.input_width,
                "input_height": args.input_height,
                "num_classes": len(classes),
                "classes": classes,
                "mean": list(args.mean),
                "std": list(args.std),
                "modality": args.modality,
            },
            "dataset_yaml": str(dataset_path),
            "epoch": epoch,
            "metrics": validation_metrics,
        }
        torch.save(checkpoint, weights / "last.pt")
        if validation_metrics["loss"] < best_loss:
            best_loss = validation_metrics["loss"]
            torch.save(checkpoint, weights / "best.pt")
        (output / "metrics.json").write_text(json.dumps(history, indent=2) + "\n", encoding="utf-8")
        print(f"epoch={epoch} train={train_metrics['loss']:.6f} val={validation_metrics['loss']:.6f}")
    manifest = {
        "format_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "mode": "frozen_backbone_detection_head",
        "dataset_yaml": str(dataset_path),
        "classes": classes,
        "backbone_weights": str(backbone_path),
        "backbone_sha256": backbone_sha256,
        "modality": args.modality,
        "mean": list(args.mean),
        "std": list(args.std),
        "best_weights": str(weights / "best.pt"),
        "run_directory": str(output),
    }
    (output / "jetpilot_training_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
