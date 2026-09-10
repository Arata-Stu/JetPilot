from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

import torch

from e2e_learning.models.dinov3_multitask import (
    DinoV3ViTSmallMultiTask,
    backbone_fingerprint,
    load_detection_state_dict,
)


def _checkpoint(path: Path) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"checkpoint not found: {resolved}")
    value = torch.load(resolved, map_location="cpu")
    if not isinstance(value, dict):
        raise ValueError(f"checkpoint must be a mapping: {resolved}")
    return value


def _control_model_config(checkpoint: dict[str, Any]) -> dict[str, Any]:
    config = checkpoint.get("cfg")
    if not isinstance(config, dict):
        return {}
    model = config.get("model")
    return model if isinstance(model, dict) else {}


def _control_data_config(checkpoint: dict[str, Any]) -> dict[str, Any]:
    config = checkpoint.get("cfg")
    if not isinstance(config, dict):
        return {}
    data = config.get("data")
    return data if isinstance(data, dict) else {}


def _load_control_state(model: DinoV3ViTSmallMultiTask, checkpoint: dict[str, Any]) -> None:
    raw = checkpoint.get("model_state")
    if not isinstance(raw, dict):
        raise ValueError("control checkpoint has no model_state")
    state = {
        str(name): value
        for name, value in raw.items()
        if str(name).startswith(("backbone.", "control_head."))
    }
    missing, unexpected = model.load_state_dict(state, strict=False)
    required_missing = [
        name for name in missing
        if name.startswith(("backbone.", "control_head."))
    ]
    if required_missing or unexpected:
        raise ValueError(
            f"control checkpoint is incompatible: missing={required_missing}, unexpected={unexpected}"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Assemble separately trained control/detection heads and export one shared-backbone ONNX"
    )
    parser.add_argument("--control-checkpoint", type=Path, required=True)
    parser.add_argument("--detection-checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--opset", type=int, default=17)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    control_checkpoint = _checkpoint(args.control_checkpoint)
    detection_checkpoint = _checkpoint(args.detection_checkpoint)
    detection_config = detection_checkpoint.get("model")
    if not isinstance(detection_config, dict):
        raise SystemExit("error: detection checkpoint has no model configuration")
    control_model = _control_model_config(control_checkpoint)
    control_data = _control_data_config(control_checkpoint)
    input_width = int(detection_config.get("input_width", 212))
    input_height = int(detection_config.get("input_height", 120))
    if int(control_data.get("input_width", input_width)) != input_width or int(
        control_data.get("input_height", input_height)
    ) != input_height:
        raise SystemExit("error: control and detection checkpoints use different input geometry")
    control_mean = [float(value) for value in control_data.get("mean", [0.485, 0.456, 0.406])]
    control_std = [float(value) for value in control_data.get("std", [0.229, 0.224, 0.225])]
    detection_mean = [float(value) for value in detection_config.get("mean", control_mean)]
    detection_std = [float(value) for value in detection_config.get("std", control_std)]
    if control_mean != detection_mean or control_std != detection_std:
        raise SystemExit("error: control and detection heads use different image normalization")
    control_modality = (
        "event_image"
        if str(control_data.get("image_topic") or "").endswith("/event_image")
        else "rgb"
    )
    detection_modality = str(detection_config.get("modality") or "rgb")
    if control_modality != detection_modality:
        raise SystemExit("error: control and detection heads use different image modalities")
    classes = detection_config.get("classes")
    if not isinstance(classes, list) or not classes:
        raise SystemExit("error: detection checkpoint has no class list")
    model = DinoV3ViTSmallMultiTask(
        input_width=input_width,
        input_height=input_height,
        num_classes=len(classes),
        steering_only=bool(control_model.get("steering_only", False)),
        in_channels=int(control_model.get("input_channels", 3)),
        patch_size=int(control_model.get("patch_size", 16)),
        embed_dim=int(control_model.get("embed_dim", 384)),
        depth=int(control_model.get("depth", 12)),
        num_heads=int(control_model.get("num_heads", 6)),
        ffn_ratio=float(control_model.get("ffn_ratio", 4.0)),
        storage_tokens=int(control_model.get("storage_tokens", 4)),
        layer_scale=float(control_model.get("layer_scale", 1.0e-5)),
        rope_base=float(control_model.get("rope_base", 100.0)),
        rope_rescale_coords=float(control_model.get("rope_rescale_coords", 2.0)),
    )
    _load_control_state(model, control_checkpoint)
    expected_fingerprint = str(
        (detection_checkpoint.get("backbone") or {}).get("fingerprint") or ""
    )
    actual_fingerprint = backbone_fingerprint(model.backbone)
    if not expected_fingerprint:
        raise SystemExit("error: detection checkpoint has no frozen-backbone fingerprint")
    if actual_fingerprint != expected_fingerprint:
        raise SystemExit(
            "error: control and detection heads were trained with different backbone weights"
        )
    detection_state = detection_checkpoint.get("detection_state")
    if not isinstance(detection_state, dict):
        raise SystemExit("error: detection checkpoint has no detection_state")
    load_detection_state_dict(model, detection_state)
    model.eval()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    onnx_path = output_dir / "model.onnx"
    dummy = torch.zeros(1, 3, input_height, input_width)
    torch.onnx.export(
        model,
        dummy,
        onnx_path,
        input_names=["image"],
        output_names=["control", "detections"],
        opset_version=args.opset,
        dynamic_axes=None,
        external_data=False,
    )
    patch_height = model.backbone.padded_height // model.backbone.patch_size
    patch_width = model.backbone.padded_width // model.backbone.patch_size
    candidate_count = (patch_height * 2) * (patch_width * 2) + patch_height * patch_width + ((patch_height + 1) // 2) * ((patch_width + 1) // 2)
    metadata = {
        "format_version": 3,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "model_kind": "dinov3_vits16_shared_control_detection",
        "modality": control_modality,
        "backbone_frozen": True,
        "backbone_fingerprint": actual_fingerprint,
        "input": {
            "binding_name": "image",
            "shape": [1, 3, input_height, input_width],
            "layout": "NCHW",
            "color_order": "rgb",
            "mean": control_mean,
            "std": control_std,
        },
        "outputs": [
            {"binding_name": "control", "tensor_name": "control_output", "shape": [1, 2], "fields": ["steering", "throttle"]},
            {"binding_name": "detections", "tensor_name": "detection_output", "shape": [1, 4 + len(classes), candidate_count], "layout": "channel_major", "classes": classes, "nms_included": False},
        ],
        "checkpoints": {
            "control": str(args.control_checkpoint.expanduser().resolve()),
            "detection": str(args.detection_checkpoint.expanduser().resolve()),
        },
        "tensorrt": {
            "engine_filename": "model.plan",
            "input_tensor_names": ["input_tensor"],
            "input_binding_names": ["image"],
            "output_tensor_names": ["control_output", "detection_output"],
            "output_binding_names": ["control", "detections"],
            "enable_fp16": True,
            "build_on_target": True,
        },
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"Exported ONNX: {onnx_path}")
    print(f"Metadata     : {output_dir / 'metadata.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
