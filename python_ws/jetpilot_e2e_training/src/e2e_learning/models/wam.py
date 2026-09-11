from __future__ import annotations

from typing import Any

import torch
import torch.nn.functional as functional
from torch import Tensor, nn

from e2e_learning.models.dinov3_vit import DinoV3ViTSmallBackbone


class TensorRTGRUCell(nn.Module):
    """GRU cell expressed with TensorRT-friendly primitive operations."""

    def __init__(self, input_dim: int, hidden_dim: int) -> None:
        super().__init__()
        self.input_linear = nn.Linear(input_dim, hidden_dim * 3)
        self.hidden_linear = nn.Linear(hidden_dim, hidden_dim * 3)

    def forward(self, value: Tensor, hidden: Tensor) -> Tensor:
        input_reset, input_update, input_new = self.input_linear(value).chunk(3, dim=-1)
        hidden_reset, hidden_update, hidden_new = self.hidden_linear(hidden).chunk(3, dim=-1)
        reset = torch.sigmoid(input_reset + hidden_reset)
        update = torch.sigmoid(input_update + hidden_update)
        candidate = torch.tanh(input_new + reset * hidden_new)
        return update * hidden + (1.0 - update) * candidate


class DinoV3TinyWAM(nn.Module):
    """Deterministic World Action Model with a DINOv3-compatible visual encoder."""

    def __init__(
        self,
        image_size: int = 224,
        input_height: int = 120,
        input_width: int = 212,
        latent_dim: int = 256,
        hidden_dim: int = 256,
        future_horizon: int = 6,
        steering_only: bool = False,
        **backbone_kwargs: Any,
    ) -> None:
        super().__init__()
        if future_horizon < 1:
            raise ValueError("future_horizon must be positive")
        self.steering_only = steering_only
        self.future_horizon = int(future_horizon)
        self.hidden_dim = int(hidden_dim)
        self.backbone = DinoV3ViTSmallBackbone(
            image_size=image_size,
            input_height=input_height,
            input_width=input_width,
            **backbone_kwargs,
        )
        self.visual_projection = nn.Sequential(
            nn.Linear(self.backbone.embed_dim, latent_dim),
            nn.LayerNorm(latent_dim),
            nn.GELU(),
        )
        self.history_cell = TensorRTGRUCell(latent_dim + 2, hidden_dim)
        self.dynamics_cell = TensorRTGRUCell(2, hidden_dim)
        self.future_latent_head = nn.Linear(hidden_dim, self.backbone.embed_dim)
        self.action_head = nn.Sequential(
            nn.Linear(hidden_dim, 128),
            nn.GELU(),
            nn.Linear(128, 1 if steering_only else 2),
        )
        self._encoder_trainable = True

    def set_encoder_trainable(self, trainable: bool) -> None:
        self._encoder_trainable = trainable
        for parameter in self.backbone.parameters():
            parameter.requires_grad = trainable
        if not trainable:
            self.backbone.eval()

    def train(self, mode: bool = True) -> "DinoV3TinyWAM":
        super().train(mode)
        if not self._encoder_trainable:
            self.backbone.eval()
        return self

    def _backbone_features(self, images: Tensor) -> Tensor:
        if images.ndim == 4:
            return self.backbone.forward_features(images)["x_norm_clstoken"]
        if images.ndim != 5:
            raise ValueError("images must have shape [B,C,H,W] or [B,T,C,H,W]")
        batch, sequence, channels, height, width = images.shape
        encoded = self._backbone_features(
            images.reshape(batch * sequence, channels, height, width)
        )
        return encoded.reshape(batch, sequence, -1)

    def encode(self, images: Tensor) -> Tensor:
        return self.visual_projection(self._backbone_features(images))

    def encode_target(self, images: Tensor) -> Tensor:
        with torch.no_grad():
            return functional.normalize(self._backbone_features(images), dim=-1)

    def _action(self, hidden: Tensor) -> Tensor:
        raw = self.action_head(hidden)
        steering = torch.tanh(raw[:, 0:1])
        throttle = (
            torch.zeros_like(steering)
            if self.steering_only
            else torch.sigmoid(raw[:, 1:2])
        )
        return torch.cat((steering, throttle), dim=-1)

    def initial_state(self, batch: int, reference: Tensor) -> Tensor:
        return torch.zeros(batch, self.hidden_dim, dtype=reference.dtype, device=reference.device)

    def observe(
        self,
        images: Tensor,
        actions: Tensor | None = None,
        hidden: Tensor | None = None,
    ) -> Tensor:
        encoded = self.encode(images)
        if encoded.ndim == 2:
            encoded = encoded.unsqueeze(1)
        batch, sequence, _ = encoded.shape
        if actions is None:
            actions = torch.zeros(batch, sequence, 2, dtype=encoded.dtype, device=encoded.device)
        if actions.shape != (batch, sequence, 2):
            raise ValueError("actions must have shape [B,T,2]")
        state = self.initial_state(batch, encoded) if hidden is None else hidden
        previous_actions = torch.cat((torch.zeros_like(actions[:, :1]), actions[:, :-1]), dim=1)
        for index in range(sequence):
            state = self.history_cell(
                torch.cat((encoded[:, index], previous_actions[:, index]), dim=-1),
                state,
            )
        return state

    def rollout(
        self,
        hidden: Tensor,
        first_action: Tensor,
        teacher_actions: Tensor | None = None,
    ) -> tuple[Tensor, Tensor, Tensor]:
        predicted_latents: list[Tensor] = []
        predicted_actions: list[Tensor] = []
        state = hidden
        action = first_action
        for step in range(self.future_horizon):
            state = self.dynamics_cell(action, state)
            predicted_latents.append(functional.normalize(self.future_latent_head(state), dim=-1))
            predicted_action = self._action(state)
            predicted_actions.append(predicted_action)
            if teacher_actions is not None and step < self.future_horizon - 1:
                action = teacher_actions[:, step]
            else:
                action = predicted_action
        return (
            torch.stack(predicted_latents, dim=1),
            torch.stack(predicted_actions, dim=1),
            state,
        )

    def forward_train(
        self,
        images: Tensor,
        history_actions: Tensor,
        current_action: Tensor,
        future_actions: Tensor,
    ) -> dict[str, Tensor]:
        hidden = self.observe(images, history_actions)
        control = self._action(hidden)
        future_latents, future_controls, next_hidden = self.rollout(
            hidden,
            current_action,
            future_actions,
        )
        return {
            "control": control,
            "future_latents": future_latents,
            "future_controls": future_controls,
            "next_hidden": next_hidden,
        }

    def forward(self, images: Tensor, history_actions: Tensor | None = None) -> Tensor:
        return self._action(self.observe(images, history_actions))

    def forward_step(
        self,
        image: Tensor,
        hidden_state: Tensor,
        previous_action: Tensor,
    ) -> tuple[Tensor, Tensor, Tensor, Tensor]:
        """One online step with explicit recurrent state for ONNX/TensorRT."""
        encoded = self.encode(image)
        hidden = self.history_cell(torch.cat((encoded, previous_action), dim=-1), hidden_state)
        control = self._action(hidden)
        future_latents, future_controls, _ = self.rollout(hidden, control)
        return control, hidden, future_controls, future_latents


class WAMTensorRTWrapper(nn.Module):
    """Stable TensorRT export boundary for the stateful online WAM step."""

    def __init__(self, model: DinoV3TinyWAM) -> None:
        super().__init__()
        self.model = model

    def forward(
        self,
        image: Tensor,
        hidden_state: Tensor,
        previous_action: Tensor,
    ) -> tuple[Tensor, Tensor, Tensor, Tensor]:
        return self.model.forward_step(image, hidden_state, previous_action)
