import torch
from torch import nn


class PilotNet(nn.Module):
    def __init__(self, input_channels: int = 3, output_dim: int = 2) -> None:
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
            nn.AdaptiveAvgPool2d((4, 8)),
            nn.Flatten(),
        )
        self.head = nn.Sequential(
            nn.Linear(64 * 4 * 8, 100),
            nn.ELU(inplace=True),
            nn.Linear(100, 50),
            nn.ELU(inplace=True),
            nn.Linear(50, 10),
            nn.ELU(inplace=True),
            nn.Linear(10, output_dim),
        )

    def set_encoder_trainable(self, trainable: bool) -> None:
        for parameter in self.features.parameters():
            parameter.requires_grad = trainable

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        raw = self.head(self.features(x))
        steering = torch.tanh(raw[:, 0:1])
        throttle = torch.sigmoid(raw[:, 1:2])
        return torch.cat([steering, throttle], dim=1)
