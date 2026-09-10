from __future__ import annotations

import hashlib
from collections.abc import Sequence
from typing import Any

import torch
from torch import Tensor, nn

from e2e_learning.models.dinov3_vit import DinoV3ViTSmallBackbone


class ConvBlock(nn.Sequential):
    def __init__(self, in_channels: int, out_channels: int, stride: int = 1) -> None:
        super().__init__(
            nn.Conv2d(in_channels, out_channels, 3, stride=stride, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.SiLU(inplace=True),
        )


class ViTFeaturePyramid(nn.Module):
    """Turn same-resolution DINO patch tokens into stride 8/16/32 maps."""

    def __init__(self, embed_dim: int = 384, channels: int = 128, levels: int = 4) -> None:
        super().__init__()
        self.projections = nn.ModuleList(
            nn.Conv2d(embed_dim, channels, 1, bias=False) for _ in range(levels)
        )
        self.fuse = ConvBlock(channels, channels)
        self.p3 = nn.Sequential(
            nn.ConvTranspose2d(channels, channels, 2, stride=2, bias=False),
            nn.BatchNorm2d(channels),
            nn.SiLU(inplace=True),
            ConvBlock(channels, channels),
        )
        self.p4 = ConvBlock(channels, channels)
        self.p5 = ConvBlock(channels, channels, stride=2)

    def forward(
        self, tokens: Sequence[Tensor], grid_height: int, grid_width: int
    ) -> tuple[Tensor, Tensor, Tensor]:
        if len(tokens) != len(self.projections):
            raise ValueError("feature count does not match pyramid projections")
        maps = [
            projection(value.transpose(1, 2).reshape(
                value.shape[0], value.shape[2], grid_height, grid_width
            ))
            for projection, value in zip(self.projections, tokens, strict=True)
        ]
        fused = self.fuse(torch.stack(maps, dim=0).mean(dim=0))
        return self.p3(fused), self.p4(fused), self.p5(fused)


class YoloCompatibleHead(nn.Module):
    """Anchor-free head exporting YOLOv8 decoder-compatible xywh/class tensors."""

    def __init__(self, channels: int, num_classes: int, strides: Sequence[int] = (8, 16, 32)) -> None:
        super().__init__()
        self.num_classes = int(num_classes)
        self.strides = tuple(int(value) for value in strides)
        self.towers = nn.ModuleList(
            nn.Sequential(
                ConvBlock(channels, channels),
                ConvBlock(channels, channels),
                nn.Conv2d(channels, 4 + self.num_classes, 1),
            )
            for _ in self.strides
        )

    def forward_raw(self, features: Sequence[Tensor]) -> tuple[Tensor, ...]:
        return tuple(tower(feature) for tower, feature in zip(self.towers, features, strict=True))

    def decode(
        self,
        raw_levels: Sequence[Tensor],
        *,
        pad_left: int = 0,
        pad_top: int = 0,
        output_width: int | None = None,
        output_height: int | None = None,
    ) -> Tensor:
        outputs: list[Tensor] = []
        for raw, stride in zip(raw_levels, self.strides, strict=True):
            batch, _, height, width = raw.shape
            dtype, device = raw.dtype, raw.device
            grid_y, grid_x = torch.meshgrid(
                torch.arange(height, dtype=dtype, device=device),
                torch.arange(width, dtype=dtype, device=device),
                indexing="ij",
            )
            center_x = (raw[:, 0].sigmoid() + grid_x) * stride
            center_y = (raw[:, 1].sigmoid() + grid_y) * stride
            box_width = raw[:, 2].clamp(-6.0, 6.0).exp() * stride
            box_height = raw[:, 3].clamp(-6.0, 6.0).exp() * stride
            left = center_x - box_width * 0.5 - pad_left
            top = center_y - box_height * 0.5 - pad_top
            right = center_x + box_width * 0.5 - pad_left
            bottom = center_y + box_height * 0.5 - pad_top
            if output_width is not None and output_height is not None:
                left = left.clamp(0.0, float(output_width))
                right = right.clamp(0.0, float(output_width))
                top = top.clamp(0.0, float(output_height))
                bottom = bottom.clamp(0.0, float(output_height))
            xywh = torch.stack(
                ((left + right) * 0.5, (top + bottom) * 0.5, right - left, bottom - top),
                dim=1,
            )
            scores = raw[:, 4:].sigmoid()
            outputs.append(torch.cat((xywh, scores), dim=1).reshape(batch, 4 + self.num_classes, -1))
        return torch.cat(outputs, dim=2)

    def forward(self, features: Sequence[Tensor], **decode_options: Any) -> Tensor:
        return self.decode(self.forward_raw(features), **decode_options)


class DinoV3ViTSmallMultiTask(nn.Module):
    """One frozen DINOv3 backbone shared by independent control and detection heads."""

    intermediate_indices = (2, 5, 8, 11)

    def __init__(
        self,
        *,
        input_height: int = 120,
        input_width: int = 212,
        num_classes: int = 2,
        detection_channels: int = 128,
        steering_only: bool = False,
        **backbone_kwargs: Any,
    ) -> None:
        super().__init__()
        self.steering_only = steering_only
        self.backbone = DinoV3ViTSmallBackbone(
            input_height=input_height,
            input_width=input_width,
            **backbone_kwargs,
        )
        self.control_head = nn.Sequential(
            nn.Linear(self.backbone.embed_dim, 128),
            nn.GELU(),
            nn.Dropout(0.1),
            nn.Linear(128, 1 if steering_only else 2),
        )
        self.detection_neck = ViTFeaturePyramid(
            self.backbone.embed_dim, detection_channels, len(self.intermediate_indices)
        )
        self.detection_head = YoloCompatibleHead(detection_channels, num_classes)
        self.freeze_backbone()

    def freeze_backbone(self) -> None:
        for parameter in self.backbone.parameters():
            parameter.requires_grad = False
        self.backbone.eval()

    def train(self, mode: bool = True) -> "DinoV3ViTSmallMultiTask":
        super().train(mode)
        # Frozen DINO features must remain deterministic. In particular, this
        # disables training-time random RoPE coordinate rescaling.
        self.backbone.eval()
        return self

    def shared_features(self, image: Tensor) -> tuple[Tensor, tuple[Tensor, ...]]:
        features, intermediate = self.backbone.forward_features_with_intermediates(
            image, self.intermediate_indices
        )
        return features["x_norm_clstoken"], intermediate

    def forward_control_from_cls(self, cls_token: Tensor) -> Tensor:
        raw = self.control_head(cls_token)
        steering = torch.tanh(raw[:, 0:1])
        throttle = torch.zeros_like(steering) if self.steering_only else torch.sigmoid(raw[:, 1:2])
        return torch.cat((steering, throttle), dim=1)

    def detection_features(self, intermediate: Sequence[Tensor]) -> tuple[Tensor, Tensor, Tensor]:
        return self.detection_neck(
            intermediate,
            self.backbone.padded_height // self.backbone.patch_size,
            self.backbone.padded_width // self.backbone.patch_size,
        )

    def forward_detection_raw(self, image: Tensor) -> tuple[Tensor, ...]:
        _, intermediate = self.shared_features(image)
        return self.detection_head.forward_raw(self.detection_features(intermediate))

    def forward(self, image: Tensor) -> tuple[Tensor, Tensor]:
        cls_token, intermediate = self.shared_features(image)
        control = self.forward_control_from_cls(cls_token)
        pad_left = (self.backbone.padded_width - self.backbone.input_width) // 2
        pad_top = (self.backbone.padded_height - self.backbone.input_height) // 2
        detections = self.detection_head(
            self.detection_features(intermediate),
            pad_left=pad_left,
            pad_top=pad_top,
            output_width=self.backbone.input_width,
            output_height=self.backbone.input_height,
        )
        return control, detections


def detection_state_dict(model: DinoV3ViTSmallMultiTask) -> dict[str, Tensor]:
    state: dict[str, Tensor] = {}
    for prefix in ("detection_neck", "detection_head"):
        module = getattr(model, prefix)
        state.update({f"{prefix}.{name}": value for name, value in module.state_dict().items()})
    return state


def backbone_fingerprint(backbone: nn.Module) -> str:
    digest = hashlib.sha256()
    for name, value in sorted(backbone.state_dict().items()):
        tensor = value.detach().cpu().contiguous()
        digest.update(name.encode("utf-8"))
        digest.update(str(tuple(tensor.shape)).encode("ascii"))
        digest.update(str(tensor.dtype).encode("ascii"))
        digest.update(tensor.numpy().tobytes())
    return digest.hexdigest()


def load_detection_state_dict(model: DinoV3ViTSmallMultiTask, state: dict[str, Tensor]) -> None:
    missing, unexpected = model.load_state_dict(state, strict=False)
    required_missing = [
        name for name in missing if name.startswith(("detection_neck.", "detection_head."))
    ]
    if required_missing or unexpected:
        raise RuntimeError(
            f"Invalid detection head checkpoint: missing={required_missing}, unexpected={unexpected}"
        )
