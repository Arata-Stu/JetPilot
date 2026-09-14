from __future__ import annotations

import torch
from torch import nn

from e2e_learning.models.dinov3_vit import DinoV3ViTSmallBackbone
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


class AsyncDinoRgbEvsControl(nn.Module):
    """Frozen DINOv3 RGB anchor with a high-rate event-driven latent updater."""

    def __init__(
        self,
        input_height: int = 120,
        input_width: int = 212,
        event_channels: int = 20,
        state_dim: int = 128,
        event_feature_dim: int = 64,
        steering_only: bool = False,
        nominal_delta_t_s: float = 0.004,
        max_delta_t_s: float = 0.04,
        image_size: int = 224,
        input_channels: int = 3,
        patch_size: int = 16,
        embed_dim: int = 384,
        depth: int = 12,
        num_heads: int = 6,
        ffn_ratio: float = 4.0,
        storage_tokens: int = 4,
        layer_scale: float = 1e-5,
        rope_base: float = 100.0,
        rope_rescale_coords: float = 2.0,
    ) -> None:
        super().__init__()
        if nominal_delta_t_s <= 0.0 or max_delta_t_s < nominal_delta_t_s:
            raise ValueError("delta_t limits must be positive and ordered")
        self.steering_only = steering_only
        self.nominal_delta_t_s = float(nominal_delta_t_s)
        self.max_delta_t_s = float(max_delta_t_s)
        self.rgb_feature_dim = int(embed_dim)
        self.rgb_backbone = DinoV3ViTSmallBackbone(
            image_size=image_size,
            input_height=input_height,
            input_width=input_width,
            in_channels=input_channels,
            patch_size=patch_size,
            embed_dim=embed_dim,
            depth=depth,
            num_heads=num_heads,
            ffn_ratio=ffn_ratio,
            storage_tokens=storage_tokens,
            layer_scale=layer_scale,
            rope_base=rope_base,
            rope_rescale_coords=rope_rescale_coords,
        )
        # Only this projection learns how general DINO features map into the
        # control-specific recurrent state. The foundation backbone stays fixed.
        self.rgb_adapter = nn.Sequential(
            nn.LayerNorm(embed_dim),
            nn.Linear(embed_dim, state_dim),
            nn.GELU(),
            nn.LayerNorm(state_dim),
        )
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
        self._backbone_trainable = False
        self.set_encoder_trainable(False)

    def set_encoder_trainable(self, trainable: bool) -> None:
        # The DINO variant intentionally ignores attempts to unfreeze from a
        # generic training stage. A separate explicit experiment is required
        # before foundation weights may be changed.
        del trainable
        self._backbone_trainable = False
        for parameter in self.rgb_backbone.parameters():
            parameter.requires_grad = False
        self.rgb_backbone.eval()

    def train(self, mode: bool = True) -> "AsyncDinoRgbEvsControl":
        super().train(mode)
        self.rgb_backbone.eval()
        return self

    def encode_rgb(self, rgb: torch.Tensor) -> torch.Tensor:
        # Cached training supplies the frozen backbone's CLS feature directly.
        # Runtime/export still supplies NCHW RGB and therefore follows the
        # original backbone path.
        if rgb.ndim == 2:
            if rgb.shape[1] != self.rgb_feature_dim:
                raise ValueError(
                    f"cached RGB feature has {rgb.shape[1]} values; "
                    f"expected {self.rgb_feature_dim}"
                )
            return self.rgb_adapter(rgb)
        with torch.no_grad():
            feature = self.rgb_backbone.forward_features(rgb)["x_norm_clstoken"]
        return self.rgb_adapter(feature)

    def encode_delta_t(self, delta_t: torch.Tensor) -> torch.Tensor:
        clipped = torch.clamp(delta_t, min=0.0, max=self.max_delta_t_s)
        return clipped / self.nominal_delta_t_s

    def decode_control(self, state: torch.Tensor) -> torch.Tensor:
        raw = self.control_head(state)
        steering = torch.tanh(raw[:, :1])
        throttle = torch.zeros_like(steering) if self.steering_only else torch.sigmoid(raw[:, 1:2])
        return torch.cat((steering, throttle), dim=1)

    def update_state(
        self,
        state: torch.Tensor,
        event_tensor: torch.Tensor,
        delta_t: torch.Tensor,
    ) -> torch.Tensor:
        event_feature = self.event_encoder(event_tensor)
        normalized_delta_t = self.encode_delta_t(delta_t.reshape(state.shape[0], 1))
        return self.updater(torch.cat((event_feature, normalized_delta_t), dim=1), state)

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
            candidate = self.update_state(state, event_tensors[:, index], delta_t[:, index])
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


class AsyncRgbEncoderExport(nn.Module):
    """TensorRT export view for the low-rate RGB state initializer."""

    def __init__(self, model: nn.Module) -> None:
        super().__init__()
        self.model = model

    def forward(self, rgb: torch.Tensor) -> torch.Tensor:
        return self.model.encode_rgb(rgb)


class AsyncEventUpdaterExport(nn.Module):
    """TensorRT export view for one high-rate EVS state update."""

    def __init__(self, model: nn.Module) -> None:
        super().__init__()
        self.model = model

    def forward(
        self,
        state_in: torch.Tensor,
        event_tensor: torch.Tensor,
        delta_t: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        if hasattr(self.model, "update_state"):
            state_out = self.model.update_state(state_in, event_tensor, delta_t)
        else:
            event_feature = self.model.event_encoder(event_tensor)
            delta_t_column = delta_t.reshape(state_in.shape[0], 1)
            state_out = self.model.updater(
                torch.cat((event_feature, delta_t_column), dim=1), state_in
            )
        return state_out, self.model.decode_control(state_out)
