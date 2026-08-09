from __future__ import annotations

from typing import Any

import torch
from torch import nn
from torchvision import models


class PilotNetEncoder(nn.Module):
    def __init__(self, input_channels: int = 3, feature_dim: int = 128) -> None:
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(input_channels, 24, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=3),
            nn.ELU(inplace=True),
            nn.Conv2d(64, 64, kernel_size=3),
            nn.ELU(inplace=True),
            nn.AdaptiveAvgPool2d((2, 4)),
            nn.Flatten(),
            nn.Linear(64 * 2 * 4, feature_dim),
            nn.ELU(inplace=True),
        )
        self.output_dim = feature_dim

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.features(image)


class MobileNetEncoder(nn.Module):
    def __init__(self, feature_dim: int = 128, pretrained: bool = False) -> None:
        super().__init__()
        weights = models.MobileNet_V3_Small_Weights.DEFAULT if pretrained else None
        backbone = models.mobilenet_v3_small(weights=weights)
        self.features = backbone.features
        self.pool = nn.AdaptiveAvgPool2d((1, 1))
        self.projection = nn.Sequential(
            nn.Flatten(),
            nn.Linear(backbone.classifier[0].in_features, feature_dim),
            nn.Hardswish(inplace=True),
        )
        self.output_dim = feature_dim

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.projection(self.pool(self.features(image)))


class FusionE2EModel(nn.Module):
    """CNN with optional visual GRU and causal IMU-window fusion."""

    def __init__(self, config: Any) -> None:
        super().__init__()
        self.task = str(getattr(config, "task", "trajectory"))
        self.temporal = str(getattr(config, "temporal", "none"))
        self.use_imu = bool(getattr(config, "use_imu", False))
        self.trajectory_points = int(getattr(config, "trajectory_points", 10))
        feature_dim = int(getattr(config, "feature_dim", 128))
        hidden_dim = int(getattr(config, "hidden_dim", 128))
        backbone = str(getattr(config, "backbone", "pilotnet"))
        if backbone == "pilotnet":
            self.image_encoder = PilotNetEncoder(
                input_channels=int(getattr(config, "input_channels", 3)),
                feature_dim=feature_dim,
            )
        elif backbone == "mobilenet_v3_small":
            self.image_encoder = MobileNetEncoder(
                feature_dim=feature_dim,
                pretrained=bool(getattr(config, "pretrained", False)),
            )
        else:
            raise ValueError(f"Unsupported fusion backbone: {backbone}")

        if self.temporal == "gru":
            self.visual_temporal = nn.GRU(feature_dim, hidden_dim, batch_first=True)
            visual_dim = hidden_dim
        elif self.temporal == "none":
            self.visual_temporal = None
            visual_dim = feature_dim
        else:
            raise ValueError("temporal must be none or gru")

        imu_hidden_dim = int(getattr(config, "imu_hidden_dim", 32))
        if self.use_imu:
            self.imu_encoder = nn.GRU(
                int(getattr(config, "imu_features", 7)), imu_hidden_dim, batch_first=True
            )
        else:
            self.imu_encoder = None
            imu_hidden_dim = 0

        output_dim = 2 if self.task == "control" else self.trajectory_points * 2
        self.head = nn.Sequential(
            nn.Linear(visual_dim + imu_hidden_dim, hidden_dim),
            nn.ELU(inplace=True),
            nn.Dropout(float(getattr(config, "dropout", 0.1))),
            nn.Linear(hidden_dim, output_dim),
        )

    def set_encoder_trainable(self, trainable: bool) -> None:
        for parameter in self.image_encoder.parameters():
            parameter.requires_grad = trainable

    def forward(self, images: torch.Tensor, imu: torch.Tensor | None = None) -> torch.Tensor:
        if images.ndim == 4:
            images = images.unsqueeze(1)
        if images.ndim != 5:
            raise ValueError("images must have shape [B,T,C,H,W] or [B,C,H,W]")
        batch, sequence, channels, height, width = images.shape
        encoded = self.image_encoder(images.reshape(batch * sequence, channels, height, width))
        encoded = encoded.reshape(batch, sequence, -1)
        if self.visual_temporal is not None:
            _, visual_state = self.visual_temporal(encoded)
            visual = visual_state[-1]
        else:
            visual = encoded[:, -1]

        features = [visual]
        if self.imu_encoder is not None:
            if imu is None:
                raise ValueError("IMU input is required by this model")
            _, imu_state = self.imu_encoder(imu)
            features.append(imu_state[-1])
        raw = self.head(torch.cat(features, dim=1))
        if self.task == "control":
            return torch.cat([torch.tanh(raw[:, 0:1]), torch.sigmoid(raw[:, 1:2])], dim=1)
        return torch.tanh(raw).reshape(batch, self.trajectory_points, 2)
