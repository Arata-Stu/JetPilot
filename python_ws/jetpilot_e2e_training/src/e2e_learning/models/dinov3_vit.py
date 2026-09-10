from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Sequence

import torch
import torch.nn.functional as functional
from torch import Tensor, nn


class LayerScale(nn.Module):
    def __init__(self, dim: int, init_value: float) -> None:
        super().__init__()
        self.gamma = nn.Parameter(torch.full((dim,), init_value))

    def forward(self, x: Tensor) -> Tensor:
        return x * self.gamma


class PatchEmbed(nn.Module):
    def __init__(
        self,
        image_size: tuple[int, int],
        patch_size: int,
        in_channels: int,
        embed_dim: int,
    ) -> None:
        super().__init__()
        if any(size % patch_size for size in image_size):
            raise ValueError("padded image dimensions must be divisible by patch_size")
        self.img_size = image_size
        self.patch_size = (patch_size, patch_size)
        self.patches_resolution = tuple(size // patch_size for size in image_size)
        self.num_patches = self.patches_resolution[0] * self.patches_resolution[1]
        self.proj = nn.Conv2d(
            in_channels, embed_dim, kernel_size=patch_size, stride=patch_size
        )
        self.norm = nn.Identity()

    def forward(self, image: Tensor) -> Tensor:
        patches = self.proj(image).permute(0, 2, 3, 1)
        return self.norm(patches)


class RopePositionEmbedding(nn.Module):
    """DINOv3-style axial 2-D RoPE, with no learned positional embedding."""

    def __init__(
        self,
        head_dim: int,
        grid_size: tuple[int, int],
        base: float = 100.0,
        rescale_coords: float | None = 2.0,
    ) -> None:
        super().__init__()
        if head_dim % 4:
            raise ValueError("attention head dimension must be divisible by four")
        exponents = 2 * torch.arange(head_dim // 4, dtype=torch.float32) / (head_dim // 2)
        self.register_buffer("periods", base**exponents, persistent=True)
        self.head_dim = head_dim
        self.grid_size = grid_size
        self.rescale_coords = rescale_coords
        sin, cos = self._make_embedding(*grid_size, random_rescale=False)
        # Fixed inference constants keep Sin/Cos out of the exported TensorRT graph.
        self.register_buffer("inference_sin", sin, persistent=False)
        self.register_buffer("inference_cos", cos, persistent=False)

    def _make_embedding(
        self, height: int, width: int, *, random_rescale: bool
    ) -> tuple[Tensor, Tensor]:
        dtype = self.periods.dtype
        device = self.periods.device
        coords_h = (torch.arange(height, dtype=dtype, device=device) + 0.5) / height
        coords_w = (torch.arange(width, dtype=dtype, device=device) + 0.5) / width
        grid_h, grid_w = torch.meshgrid(coords_h, coords_w, indexing="ij")
        coords = torch.stack((grid_h, grid_w), dim=-1).reshape(-1, 2)
        coords = coords * 2.0 - 1.0
        if random_rescale and self.rescale_coords is not None:
            bound = math.log(self.rescale_coords)
            scale = torch.empty((), dtype=dtype, device=device).uniform_(-bound, bound).exp()
            coords = coords * scale
        angles = 2.0 * math.pi * coords.unsqueeze(-1) / self.periods.view(1, 1, -1)
        angles = angles.flatten(1).repeat(1, 2)
        return torch.sin(angles), torch.cos(angles)

    def forward(self, height: int, width: int) -> tuple[Tensor, Tensor]:
        if not self.training and (height, width) == self.grid_size:
            return self.inference_sin, self.inference_cos
        return self._make_embedding(height, width, random_rescale=self.training)


def _rotate_half(x: Tensor) -> Tensor:
    first, second = x.chunk(2, dim=-1)
    return torch.cat((-second, first), dim=-1)


class MaskedKeyBiasLinear(nn.Linear):
    """QKV projection whose key bias is fixed to zero, as in DINOv3 ViT-S/16."""

    def __init__(self, dim: int) -> None:
        super().__init__(dim, dim * 3, bias=True)
        mask = torch.ones(dim * 3)
        mask[dim : 2 * dim] = 0
        self.register_buffer("bias_mask", mask, persistent=True)

    def forward(self, x: Tensor) -> Tensor:
        return functional.linear(x, self.weight, self.bias * self.bias_mask)


class SelfAttention(nn.Module):
    def __init__(self, dim: int, num_heads: int) -> None:
        super().__init__()
        if dim % num_heads:
            raise ValueError("embed_dim must be divisible by num_heads")
        self.num_heads = num_heads
        self.head_dim = dim // num_heads
        self.scale = self.head_dim**-0.5
        self.qkv = MaskedKeyBiasLinear(dim)
        self.attn_drop = nn.Dropout(0.0)
        self.proj = nn.Linear(dim, dim, bias=True)
        self.proj_drop = nn.Dropout(0.0)

    @staticmethod
    def _apply_rope(x: Tensor, sin: Tensor, cos: Tensor) -> Tensor:
        patch_count = sin.shape[0]
        prefix_count = x.shape[-2] - patch_count
        patch_tokens = x[:, :, prefix_count:, :]
        sin = sin.view(1, 1, patch_count, -1).to(dtype=x.dtype)
        cos = cos.view(1, 1, patch_count, -1).to(dtype=x.dtype)
        rotated = patch_tokens * cos + _rotate_half(patch_tokens) * sin
        return torch.cat((x[:, :, :prefix_count, :], rotated), dim=-2)

    def forward(self, x: Tensor, rope: tuple[Tensor, Tensor]) -> Tensor:
        batch, tokens, channels = x.shape
        qkv = self.qkv(x).reshape(
            batch, tokens, 3, self.num_heads, self.head_dim
        ).permute(2, 0, 3, 1, 4)
        query, key, value = qkv.unbind(0)
        sin, cos = rope
        query = self._apply_rope(query, sin, cos)
        key = self._apply_rope(key, sin, cos)

        # Keep the graph in primitive operations for ONNX/TensorRT parsing.
        attention = torch.matmul(query * self.scale, key.transpose(-2, -1))
        attention = self.attn_drop(torch.softmax(attention, dim=-1))
        output = torch.matmul(attention, value)
        output = output.transpose(1, 2).reshape(batch, tokens, channels)
        return self.proj_drop(self.proj(output))


class Mlp(nn.Module):
    def __init__(self, dim: int, hidden_dim: int) -> None:
        super().__init__()
        self.fc1 = nn.Linear(dim, hidden_dim, bias=True)
        self.act = nn.GELU()
        self.drop1 = nn.Dropout(0.0)
        self.fc2 = nn.Linear(hidden_dim, dim, bias=True)
        self.drop2 = nn.Dropout(0.0)

    def forward(self, x: Tensor) -> Tensor:
        return self.drop2(self.fc2(self.drop1(self.act(self.fc1(x)))))


class SelfAttentionBlock(nn.Module):
    def __init__(self, dim: int, num_heads: int, ffn_ratio: float, layer_scale: float) -> None:
        super().__init__()
        self.norm1 = nn.LayerNorm(dim, eps=1e-5)
        self.attn = SelfAttention(dim, num_heads)
        self.ls1 = LayerScale(dim, layer_scale)
        self.norm2 = nn.LayerNorm(dim, eps=1e-5)
        self.mlp = Mlp(dim, int(dim * ffn_ratio))
        self.ls2 = LayerScale(dim, layer_scale)

    def forward(self, x: Tensor, rope: tuple[Tensor, Tensor]) -> Tensor:
        x = x + self.ls1(self.attn(self.norm1(x), rope))
        return x + self.ls2(self.mlp(self.norm2(x)))


class DinoV3ViTSmallBackbone(nn.Module):
    """DINOv3 ViT-S/16-compatible encoder boundary for later event/RGB alignment."""

    def __init__(
        self,
        image_size: int = 224,
        input_height: int | None = None,
        input_width: int | None = None,
        in_channels: int = 3,
        patch_size: int = 16,
        embed_dim: int = 384,
        depth: int = 12,
        num_heads: int = 6,
        ffn_ratio: float = 4.0,
        storage_tokens: int = 4,
        layer_scale: float = 1e-5,
        rope_base: float = 100.0,
        rope_rescale_coords: float | None = 2.0,
    ) -> None:
        super().__init__()
        self.input_height = int(input_height if input_height is not None else image_size)
        self.input_width = int(input_width if input_width is not None else image_size)
        self.patch_size = patch_size
        self.embed_dim = embed_dim
        self.num_features = embed_dim
        self.n_storage_tokens = storage_tokens
        self.padded_height = math.ceil(self.input_height / patch_size) * patch_size
        self.padded_width = math.ceil(self.input_width / patch_size) * patch_size
        self.patch_embed = PatchEmbed(
            (self.padded_height, self.padded_width),
            patch_size,
            in_channels,
            embed_dim,
        )
        self.cls_token = nn.Parameter(torch.empty(1, 1, embed_dim))
        self.storage_tokens = nn.Parameter(torch.empty(1, storage_tokens, embed_dim))
        self.mask_token = nn.Parameter(torch.empty(1, embed_dim))
        self.rope_embed = RopePositionEmbedding(
            embed_dim // num_heads,
            grid_size=(self.padded_height // patch_size, self.padded_width // patch_size),
            base=rope_base,
            rescale_coords=rope_rescale_coords,
        )
        self.blocks = nn.ModuleList(
            SelfAttentionBlock(embed_dim, num_heads, ffn_ratio, layer_scale)
            for _ in range(depth)
        )
        self.norm = nn.LayerNorm(embed_dim, eps=1e-5)
        self.head = nn.Identity()
        self._init_weights()

    def _init_weights(self) -> None:
        nn.init.normal_(self.cls_token, std=0.02)
        nn.init.normal_(self.storage_tokens, std=0.02)
        nn.init.zeros_(self.mask_token)
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.trunc_normal_(module.weight, std=0.02)
                if module.bias is not None:
                    nn.init.zeros_(module.bias)

    def _prepare_tokens(self, image: Tensor) -> tuple[Tensor, int, int]:
        if image.shape[-2:] != (self.input_height, self.input_width):
            raise ValueError(
                "DINOv3 ViT-S/16 requires "
                f"{self.input_width}x{self.input_height} input"
            )
        pad_height = self.padded_height - self.input_height
        pad_width = self.padded_width - self.input_width
        image = functional.pad(
            image,
            (
                pad_width // 2,
                pad_width - pad_width // 2,
                pad_height // 2,
                pad_height - pad_height // 2,
            ),
            value=0.0,
        )
        patches = self.patch_embed(image)
        batch, height, width, _ = patches.shape
        cls_token = self.cls_token + self.mask_token.view(1, 1, -1) * 0.0
        prefix = torch.cat((cls_token, self.storage_tokens), dim=1)
        tokens = torch.cat((prefix.expand(batch, -1, -1), patches.flatten(1, 2)), dim=1)
        return tokens, height, width

    def forward_features(self, image: Tensor) -> dict[str, Tensor | None]:
        features, _ = self.forward_features_with_intermediates(image, indices=())
        return features

    def forward_features_with_intermediates(
        self,
        image: Tensor,
        indices: Sequence[int] = (2, 5, 8, 11),
    ) -> tuple[dict[str, Tensor | None], tuple[Tensor, ...]]:
        """Return control and detection features from one transformer pass."""
        tokens, height, width = self._prepare_tokens(image)
        rope = self.rope_embed(height, width)
        wanted = set(int(index) for index in indices)
        intermediate: dict[int, Tensor] = {}
        for index, block in enumerate(self.blocks):
            tokens = block(tokens, rope)
            if index in wanted:
                intermediate[index] = self.norm(tokens)[:, self.n_storage_tokens + 1 :]
        normalized = self.norm(tokens)
        prefix_end = self.n_storage_tokens + 1
        features: dict[str, Tensor | None] = {
            "x_norm_clstoken": normalized[:, 0],
            "x_storage_tokens": normalized[:, 1:prefix_end],
            "x_norm_patchtokens": normalized[:, prefix_end:],
            "x_prenorm": tokens,
            "masks": None,
        }
        if len(intermediate) != len(wanted):
            raise ValueError(f"intermediate layer indices out of range: {sorted(wanted)}")
        return features, tuple(intermediate[index] for index in indices)

    def get_intermediate_layers(
        self, image: Tensor, indices: Sequence[int] = (2, 5, 8, 11), norm: bool = True
    ) -> tuple[Tensor, ...]:
        if norm:
            _, outputs = self.forward_features_with_intermediates(image, indices)
            return outputs
        tokens, height, width = self._prepare_tokens(image)
        rope = self.rope_embed(height, width)
        wanted = set(int(index) for index in indices)
        outputs: list[Tensor] = []
        for index, block in enumerate(self.blocks):
            tokens = block(tokens, rope)
            if index in wanted:
                value = self.norm(tokens) if norm else tokens
                outputs.append(value[:, self.n_storage_tokens + 1 :])
        if len(outputs) != len(wanted):
            raise ValueError(f"intermediate layer indices out of range: {sorted(wanted)}")
        return tuple(outputs)

    def forward(self, image: Tensor) -> Tensor:
        return self.head(self.forward_features(image)["x_norm_clstoken"])


class DinoV3ViTSmallControl(nn.Module):
    def __init__(
        self,
        image_size: int = 224,
        input_height: int | None = None,
        input_width: int | None = None,
        output_dim: int = 2,
        steering_only: bool = False,
        **backbone_kwargs: Any,
    ) -> None:
        super().__init__()
        self.steering_only = steering_only
        self.backbone = DinoV3ViTSmallBackbone(
            image_size=image_size,
            input_height=input_height,
            input_width=input_width,
            **backbone_kwargs,
        )
        self.control_head = nn.Sequential(
            nn.Linear(self.backbone.embed_dim, 128),
            nn.GELU(),
            nn.Dropout(0.1),
            nn.Linear(128, 1 if steering_only else output_dim),
        )
        self._encoder_trainable = True

    def set_encoder_trainable(self, trainable: bool) -> None:
        self._encoder_trainable = trainable
        for parameter in self.backbone.parameters():
            parameter.requires_grad = trainable
        if not trainable:
            self.backbone.eval()

    def train(self, mode: bool = True) -> "DinoV3ViTSmallControl":
        super().train(mode)
        if not self._encoder_trainable:
            self.backbone.eval()
        return self

    def forward_features(self, image: Tensor) -> dict[str, Tensor | None]:
        return self.backbone.forward_features(image)

    def get_intermediate_layers(self, image: Tensor, **kwargs: Any) -> tuple[Tensor, ...]:
        return self.backbone.get_intermediate_layers(image, **kwargs)

    def forward(self, image: Tensor) -> Tensor:
        features = self.backbone.forward_features(image)
        raw = self.control_head(features["x_norm_clstoken"])
        steering = torch.tanh(raw[:, 0:1])
        throttle = torch.zeros_like(steering) if self.steering_only else torch.sigmoid(raw[:, 1:2])
        return torch.cat((steering, throttle), dim=1)


def load_dinov3_backbone_weights(path: str | Path, backbone: nn.Module) -> tuple[int, int]:
    """Load a raw DINOv3 or GEP event-encoder checkpoint into the compatible backbone."""
    checkpoint = torch.load(path, map_location="cpu")
    state: Any = checkpoint
    if isinstance(checkpoint, dict):
        for key in ("event_encoder", "teacher", "backbone", "model_state", "model"):
            candidate = checkpoint.get(key)
            if isinstance(candidate, dict):
                state = candidate
                break
    if not isinstance(state, dict):
        raise RuntimeError("DINOv3 checkpoint does not contain a state dictionary")

    target = backbone.state_dict()
    compatible: dict[str, Tensor] = {}
    for original_name, value in state.items():
        if not isinstance(value, Tensor):
            continue
        name = str(original_name)
        for prefix in (
            "module.",
            "student.",
            "teacher.",
            "event_encoder.",
            "encoder.",
            "backbone.",
        ):
            if name.startswith(prefix):
                name = name[len(prefix) :]
        if name in target and target[name].shape == value.shape:
            compatible[name] = value
    if not compatible:
        raise RuntimeError(f"No compatible DINOv3 backbone tensors were found in {path}")
    if "rope_embed.periods" not in compatible or "storage_tokens" not in compatible:
        raise RuntimeError(
            "Checkpoint is not a DINOv3 ViT-S/16 backbone: RoPE or storage tokens are missing"
        )
    backbone.load_state_dict(compatible, strict=False)
    return len(compatible), len(target)
