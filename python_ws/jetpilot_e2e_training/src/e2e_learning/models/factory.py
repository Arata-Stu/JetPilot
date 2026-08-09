from pathlib import Path
from typing import Any

import torch
from torch import nn
from torchvision import models

from e2e_learning.models.pilotnet import PilotNet
from e2e_learning.models.fusion import FusionE2EModel


class TorchvisionEncoderHead(nn.Module):
    def __init__(
        self,
        backbone_name: str,
        output_dim: int = 2,
        pretrained: bool = False,
        weights_path: str = "",
    ) -> None:
        super().__init__()
        if backbone_name != "mobilenet_v3_small":
            raise ValueError(f"Unsupported pretrained encoder: {backbone_name}")

        weights = models.MobileNet_V3_Small_Weights.DEFAULT if pretrained and not weights_path else None
        backbone = models.mobilenet_v3_small(weights=weights)
        if weights_path:
            state = torch.load(weights_path, map_location="cpu")
            backbone.load_state_dict(state)

        self.encoder = backbone.features
        self.pool = nn.AdaptiveAvgPool2d((1, 1))
        in_features = backbone.classifier[0].in_features
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(in_features, 128),
            nn.Hardswish(inplace=True),
            nn.Dropout(p=0.1),
            nn.Linear(128, output_dim),
        )

    def set_encoder_trainable(self, trainable: bool) -> None:
        for parameter in self.encoder.parameters():
            parameter.requires_grad = trainable

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        raw = self.head(self.pool(self.encoder(x)))
        steering = torch.tanh(raw[:, 0:1])
        throttle = torch.sigmoid(raw[:, 1:2])
        return torch.cat([steering, throttle], dim=1)


def build_model(config: Any) -> nn.Module:
    name = str(config.name)
    output_dim = int(getattr(config, "output_dim", 2))
    if name == "pilotnet":
        return PilotNet(input_channels=int(getattr(config, "input_channels", 3)), output_dim=output_dim)
    if name == "mobilenet_v3_small":
        return TorchvisionEncoderHead(
            backbone_name=name,
            output_dim=output_dim,
            pretrained=bool(getattr(config, "pretrained", False)),
            weights_path=str(getattr(config, "weights_path", "")),
        )
    if name == "fusion":
        return FusionE2EModel(config)
    raise ValueError(f"Unsupported model: {name}")


def load_checkpoint(path: str | Path, model: nn.Module) -> dict[str, Any]:
    checkpoint = torch.load(path, map_location="cpu")
    state = checkpoint["model_state"] if "model_state" in checkpoint else checkpoint
    model.load_state_dict(state)
    return checkpoint
