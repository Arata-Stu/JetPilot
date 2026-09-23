"""One frozen DINOv3 backbone; arbitrary steering heads, stable output order."""
import torch
from torch import nn

from .dinov3_vit import DinoV3ViTSmallBackbone


class SectionMultiheadControl(nn.Module):
    def __init__(self, head_names, input_width=212, input_height=120):
        super().__init__()
        self.head_names = list(head_names)
        self.backbone = DinoV3ViTSmallBackbone(input_width=input_width, input_height=input_height)
        self.heads = nn.ModuleDict({name: nn.Sequential(
            nn.Linear(self.backbone.embed_dim, 128), nn.GELU(), nn.Dropout(0.1),
            nn.Linear(128, 1), nn.Tanh(),
        ) for name in self.head_names})
        self.backbone.requires_grad_(False)
        self.backbone.eval()

    def train(self, mode=True):
        super().train(mode)
        self.backbone.eval()
        return self

    def features(self, image):
        return self.backbone.forward_features(image)["x_norm_clstoken"]

    def forward(self, image):
        features = self.features(image)
        return torch.cat([self.heads[name](features) for name in self.head_names], dim=1)
