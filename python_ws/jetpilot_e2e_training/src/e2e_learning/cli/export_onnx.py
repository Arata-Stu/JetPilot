from pathlib import Path

import hydra
import torch
from omegaconf import DictConfig, OmegaConf

from e2e_learning.models.factory import build_model, load_checkpoint
from e2e_learning.utils.io import ensure_dir, write_json


@hydra.main(config_path="../conf", config_name="config", version_base=None)
def main(cfg: DictConfig) -> None:
    checkpoint_path = Path(str(cfg.checkpoint)).expanduser()
    if not checkpoint_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    output_dir = ensure_dir(Path(cfg.export.output_dir) if cfg.export.output_dir else checkpoint_path.parent.parent)
    model = build_model(cfg.model)
    checkpoint = load_checkpoint(checkpoint_path, model)
    model.eval()

    input_name = str(cfg.export.input_name)
    output_name = str(cfg.export.output_name)
    input_height = int(cfg.data.input_height)
    input_width = int(cfg.data.input_width)
    dummy = torch.randn(1, 3, input_height, input_width)
    onnx_path = output_dir / str(cfg.export.onnx_filename)
    torch.onnx.export(
        model,
        dummy,
        onnx_path,
        input_names=[input_name],
        output_names=[output_name],
        opset_version=int(cfg.export.opset_version),
        dynamic_axes=None,
        # Deployment copies model.onnx as one atomic artifact, so keep weights
        # embedded instead of creating a separate model.onnx.data file.
        external_data=False,
    )

    metadata = {
        "format_version": 1,
        "model_name": str(cfg.run.name),
        "model_kind": str(cfg.model.name),
        "checkpoint": str(checkpoint_path),
        "stage": checkpoint.get("stage", ""),
        "epoch": checkpoint.get("epoch", 0),
        "input": {
            "name": input_name,
            "shape": [1, 3, input_height, input_width],
            "layout": "NCHW",
            "color_order": "rgb",
            "mean": [float(v) for v in cfg.data.mean],
            "std": [float(v) for v in cfg.data.std],
        },
        "output": {
            "name": output_name,
            "shape": [1, 2],
            "fields": ["steering", "throttle"],
            "activations": ["tanh", "sigmoid"],
        },
        "tensorrt": {
            "engine_filename": "model.plan",
            "enable_fp16": bool(cfg.export.tensorrt.enable_fp16),
            "input_tensor_name": str(cfg.export.tensorrt.input_tensor_name),
            "output_tensor_name": str(cfg.export.tensorrt.output_tensor_name),
            "input_tensor_format": "nitros_tensor_list_nchw_rgb_f32",
            "output_tensor_format": "nitros_tensor_list_nchw_rgb_f32",
        },
        "config": OmegaConf.to_container(cfg, resolve=True),
    }
    write_json(output_dir / str(cfg.export.metadata_filename), metadata)
    print(f"Exported ONNX: {onnx_path}")
    print(f"Metadata     : {output_dir / str(cfg.export.metadata_filename)}")


if __name__ == "__main__":
    main()
