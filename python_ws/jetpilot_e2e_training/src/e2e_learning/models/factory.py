from pathlib import Path
from typing import Any

import torch
from torch import nn
from torchvision import models

from e2e_learning.models.pilotnet import PilotNet
from e2e_learning.models.fusion import FusionE2EModel
from e2e_learning.models.dinov3_vit import (
    DinoV3ViTSmallControl,
    load_dinov3_backbone_weights,
)
from e2e_learning.models.wam import DinoV3TinyWAM


class TorchvisionEncoderHead(nn.Module):
    def __init__(
        self,
        backbone_name: str,
        output_dim: int = 2,
        steering_only: bool = False,
        pretrained: bool = False,
        weights_path: str = "",
    ) -> None:
        super().__init__()
        if backbone_name != "mobilenet_v3_small":
            raise ValueError(f"Unsupported pretrained encoder: {backbone_name}")
        self.steering_only = steering_only

        weights = models.MobileNet_V3_Small_Weights.DEFAULT if pretrained and not weights_path else None
        backbone = models.mobilenet_v3_small(weights=weights)
        if weights_path:
            state = torch.load(weights_path, map_location="cpu", weights_only=True)
            backbone.load_state_dict(state)

        self.encoder = backbone.features
        self.pool = nn.AdaptiveAvgPool2d((1, 1))
        in_features = backbone.classifier[0].in_features
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(in_features, 128),
            nn.Hardswish(inplace=True),
            nn.Dropout(p=0.1),
            nn.Linear(128, 1 if steering_only else output_dim),
        )

    def set_encoder_trainable(self, trainable: bool) -> None:
        for parameter in self.encoder.parameters():
            parameter.requires_grad = trainable

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        raw = self.head(self.pool(self.encoder(x)))
        steering = torch.tanh(raw[:, 0:1])
        throttle = torch.zeros_like(steering) if self.steering_only else torch.sigmoid(raw[:, 1:2])
        return torch.cat([steering, throttle], dim=1)


def build_model(config: Any) -> nn.Module:
    name = str(config.name)
    output_dim = int(getattr(config, "output_dim", 2))
    if name == "pilotnet":
        return PilotNet(
            input_channels=int(getattr(config, "input_channels", 3)),
            output_dim=output_dim,
            steering_only=bool(getattr(config, "steering_only", False)),
        )
    if name == "mobilenet_v3_small":
        return TorchvisionEncoderHead(
            backbone_name=name,
            output_dim=output_dim,
            steering_only=bool(getattr(config, "steering_only", False)),
            pretrained=bool(getattr(config, "pretrained", False)),
            weights_path=str(getattr(config, "weights_path", "")),
        )
    if name == "dinov3_vits16":
        model = DinoV3ViTSmallControl(
            image_size=int(getattr(config, "image_size", 224)),
            input_height=int(getattr(config, "input_height", 120)),
            input_width=int(getattr(config, "input_width", 212)),
            output_dim=output_dim,
            steering_only=bool(getattr(config, "steering_only", False)),
            in_channels=int(getattr(config, "input_channels", 3)),
            patch_size=int(getattr(config, "patch_size", 16)),
            embed_dim=int(getattr(config, "embed_dim", 384)),
            depth=int(getattr(config, "depth", 12)),
            num_heads=int(getattr(config, "num_heads", 6)),
            ffn_ratio=float(getattr(config, "ffn_ratio", 4.0)),
            storage_tokens=int(getattr(config, "storage_tokens", 4)),
            layer_scale=float(getattr(config, "layer_scale", 1e-5)),
            rope_base=float(getattr(config, "rope_base", 100.0)),
            rope_rescale_coords=float(getattr(config, "rope_rescale_coords", 2.0)),
        )
        weights_path = str(getattr(config, "weights_path", ""))
        if weights_path:
            load_dinov3_backbone_weights(
                weights_path,
                model.backbone,
                trusted_checkpoint=bool(getattr(config, "trusted_checkpoint", False)),
            )
        elif bool(getattr(config, "require_weights", False)):
            raise RuntimeError(
                "model.weights_path is required for this DINOv3 fine-tuning preset"
            )
        return model
    if name == "wam_dinov3_vits16":
        model = DinoV3TinyWAM(
            image_size=int(getattr(config, "image_size", 224)),
            input_height=int(getattr(config, "input_height", 120)),
            input_width=int(getattr(config, "input_width", 212)),
            latent_dim=int(getattr(config, "latent_dim", 256)),
            hidden_dim=int(getattr(config, "hidden_dim", 256)),
            future_horizon=int(getattr(config, "future_horizon", 6)),
            steering_only=bool(getattr(config, "steering_only", False)),
            in_channels=int(getattr(config, "input_channels", 3)),
            patch_size=int(getattr(config, "patch_size", 16)),
            embed_dim=int(getattr(config, "embed_dim", 384)),
            depth=int(getattr(config, "depth", 12)),
            num_heads=int(getattr(config, "num_heads", 6)),
            ffn_ratio=float(getattr(config, "ffn_ratio", 4.0)),
            storage_tokens=int(getattr(config, "storage_tokens", 4)),
            layer_scale=float(getattr(config, "layer_scale", 1e-5)),
            rope_base=float(getattr(config, "rope_base", 100.0)),
            rope_rescale_coords=float(getattr(config, "rope_rescale_coords", 2.0)),
        )
        weights_path = str(getattr(config, "weights_path", ""))
        if weights_path:
            load_dinov3_backbone_weights(
                weights_path,
                model.backbone,
                trusted_checkpoint=bool(getattr(config, "trusted_checkpoint", False)),
            )
        elif bool(getattr(config, "require_weights", False)):
            raise RuntimeError("model.weights_path is required for this WAM preset")
        return model
    if name == "fusion":
        return FusionE2EModel(config)
    raise ValueError(f"Unsupported model: {name}")


def load_checkpoint(path: str | Path, model: nn.Module) -> dict[str, Any]:
    # Training checkpoints are produced by this project and include more than
    # tensors (resolved config and optimizer/runtime state).
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    state = checkpoint["model_state"] if "model_state" in checkpoint else checkpoint
    model.load_state_dict(state)
    return checkpoint
