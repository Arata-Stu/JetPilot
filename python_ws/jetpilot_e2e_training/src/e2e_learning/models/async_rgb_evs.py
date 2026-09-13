from __future__ import annotations

import torch
from torch import nn

from e2e_learning.models.fusion import PilotNetEncoder


class AsyncRgbEvsControl(nn.Module):
    """Compact RGB-initialized state updated by a sequence of 20ch EVS tensors."""

    def __init__(
        self,
        event_channels: int = 20,
        state_dim: int = 128,
        event_feature_dim: int = 64,
        steering_only: bool = False,
    ) -> None:
        super().__init__()
        self.steering_only = steering_only
        self.rgb_encoder = PilotNetEncoder(input_channels=3, feature_dim=state_dim)
        self.event_encoder = nn.Sequential(
            nn.Conv2d(event_channels, 24, 5, stride=2, padding=2), nn.ELU(inplace=True),
            nn.Conv2d(24, 32, 3, stride=2, padding=1), nn.ELU(inplace=True),
            nn.Conv2d(32, 48, 3, stride=2, padding=1), nn.ELU(inplace=True),
            nn.AdaptiveAvgPool2d((2, 4)), nn.Flatten(),
            nn.Linear(48 * 2 * 4, event_feature_dim), nn.ELU(inplace=True),
        )
        self.updater = nn.GRUCell(event_feature_dim + 1, state_dim)
        self.control_head = nn.Sequential(
            nn.Linear(state_dim, 64), nn.ELU(inplace=True),
            nn.Linear(64, 1 if steering_only else 2),
        )

    def set_encoder_trainable(self, trainable: bool) -> None:
        for parameter in self.rgb_encoder.parameters():
            parameter.requires_grad = trainable

    def encode_rgb(self, rgb: torch.Tensor) -> torch.Tensor:
        return self.rgb_encoder(rgb)

    def decode_control(self, state: torch.Tensor) -> torch.Tensor:
        raw = self.control_head(state)
        steering = torch.tanh(raw[:, :1])
        throttle = torch.zeros_like(steering) if self.steering_only else torch.sigmoid(raw[:, 1:2])
        return torch.cat((steering, throttle), dim=1)

    def rollout(
        self,
        rgb: torch.Tensor,
        event_tensors: torch.Tensor,
        delta_t: torch.Tensor,
        event_mask: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        state = self.encode_rgb(rgb)
        controls = []
        for index in range(event_tensors.shape[1]):
            event_feature = self.event_encoder(event_tensors[:, index])
            candidate = self.updater(
                torch.cat((event_feature, delta_t[:, index:index + 1]), dim=1), state
            )
            mask = event_mask[:, index:index + 1]
            state = candidate * mask + state * (1.0 - mask)
            controls.append(self.decode_control(state))
        return torch.stack(controls, dim=1), state

    def forward(
        self,
        rgb: torch.Tensor,
        event_tensors: torch.Tensor,
        delta_t: torch.Tensor,
        event_mask: torch.Tensor,
    ) -> torch.Tensor:
        controls, _ = self.rollout(rgb, event_tensors, delta_t, event_mask)
        return controls

