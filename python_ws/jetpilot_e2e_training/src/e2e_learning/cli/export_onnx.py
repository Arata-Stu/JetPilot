from __future__ import annotations

from pathlib import Path
from typing import Any

import hydra
import torch
from omegaconf import DictConfig, OmegaConf

from e2e_learning.models.factory import build_model
from e2e_learning.utils.io import ensure_dir, write_json


def _stored_config(checkpoint: dict[str, Any], fallback: DictConfig) -> DictConfig:
    stored = checkpoint.get("cfg")
    return OmegaConf.create(stored) if isinstance(stored, dict) else fallback


@hydra.main(config_path="../conf", config_name="config", version_base=None)
def main(cfg: DictConfig) -> None:
    checkpoint_path = Path(str(cfg.checkpoint)).expanduser()
    if not checkpoint_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    if not isinstance(checkpoint, dict):
        checkpoint = {"model_state": checkpoint}
    run_cfg = _stored_config(checkpoint, cfg)
    model = build_model(run_cfg.model)
    model.load_state_dict(checkpoint.get("model_state", checkpoint))
    model.eval()

    output_dir = ensure_dir(
        Path(cfg.export.output_dir) if cfg.export.output_dir else checkpoint_path.parent.parent
    )
    model_name = str(run_cfg.model.name)
    task = str(getattr(run_cfg.model, "task", "control"))
    sequence_length = int(getattr(run_cfg.model, "sequence_length", 1))
    use_imu = bool(getattr(run_cfg.model, "use_imu", False))
    input_height = int(run_cfg.data.input_height)
    input_width = int(run_cfg.data.input_width)
    image_input_name = str(cfg.export.input_name)
    if model_name == "fusion" and sequence_length > 1:
        dummy_images = torch.randn(1, sequence_length, 3, input_height, input_width)
    else:
        dummy_images = torch.randn(1, 3, input_height, input_width)

    input_names = [image_input_name]
    inputs: tuple[torch.Tensor, ...] | torch.Tensor = dummy_images
    input_metadata = [
        {
            "name": image_input_name,
            "kind": "image_sequence" if dummy_images.ndim == 5 else "image",
            "shape": list(dummy_images.shape),
            "layout": "NTCHW" if dummy_images.ndim == 5 else "NCHW",
            "color_order": "rgb",
            "mean": [float(value) for value in run_cfg.data.mean],
            "std": [float(value) for value in run_cfg.data.std],
        }
    ]
    if use_imu:
        imu_samples = int(getattr(run_cfg.model, "imu_samples", 10))
        imu_features = int(getattr(run_cfg.model, "imu_features", 7))
        dummy_imu = torch.zeros(1, imu_samples, imu_features)
        inputs = (dummy_images, dummy_imu)
        input_names.append("imu")
        input_metadata.append(
            {
                "name": "imu",
                "kind": "imu_sequence",
                "shape": list(dummy_imu.shape),
                "layout": "NTF",
                "causal": True,
            }
        )

    output_name = "trajectory" if task == "trajectory" else str(cfg.export.output_name)
    onnx_path = output_dir / str(cfg.export.onnx_filename)
    torch.onnx.export(
        model,
        inputs,
        onnx_path,
        input_names=input_names,
        output_names=[output_name],
        opset_version=int(cfg.export.opset_version),
        dynamic_axes=None,
        external_data=False,
    )

    if task == "trajectory":
        trajectory_points = int(getattr(run_cfg.model, "trajectory_points", 10))
        trajectory_scale_m = float(getattr(run_cfg.model, "trajectory_scale_m", 5.0))
        output_metadata = {
            "name": output_name,
            "task": "trajectory",
            "shape": [1, trajectory_points, 2],
            "fields": ["x", "y"],
            "activation": "tanh",
            "points": trajectory_points,
            "scale_m": trajectory_scale_m,
            "horizon_sec": float(getattr(run_cfg.data, "trajectory_horizon_sec", 1.5)),
            "frame_id": "base_link",
            "prepend_origin": True,
        }
    else:
        output_metadata = {
            "name": output_name,
            "task": "control",
            "shape": [1, 2],
            "fields": ["steering", "throttle"],
            "activations": ["tanh", "sigmoid"],
        }

    metadata = {
        "format_version": 2,
        "model_name": str(run_cfg.run.name),
        "model_kind": model_name,
        "task": task,
        "architecture": {
            "backbone": str(getattr(run_cfg.model, "backbone", model_name)),
            "temporal": str(getattr(run_cfg.model, "temporal", "none")),
            "use_imu": use_imu,
            "sequence_length": sequence_length,
        },
        "checkpoint": str(checkpoint_path),
        "stage": checkpoint.get("stage", ""),
        "epoch": checkpoint.get("epoch", 0),
        "input": input_metadata[0],
        "inputs": input_metadata,
        "output": output_metadata,
        "tensorrt": {
            "engine_filename": "model.plan",
            "enable_fp16": bool(cfg.export.tensorrt.enable_fp16),
            "input_tensor_names": input_names,
            "output_tensor_name": str(cfg.export.tensorrt.output_tensor_name),
            "input_tensor_format": "nitros_tensor_list_nchw_rgb_f32",
            "output_tensor_format": "nitros_tensor_list_nchw_rgb_f32",
        },
        "config": OmegaConf.to_container(run_cfg, resolve=True),
    }
    write_json(output_dir / str(cfg.export.metadata_filename), metadata)
    print(f"Exported ONNX: {onnx_path}")
    print(f"Metadata     : {output_dir / str(cfg.export.metadata_filename)}")


if __name__ == "__main__":
    main()
