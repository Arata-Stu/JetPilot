from __future__ import annotations

from pathlib import Path
from typing import Any

import hydra
import torch
from omegaconf import DictConfig, OmegaConf

from e2e_learning.models.factory import build_model
from e2e_learning.models.wam import WAMTensorRTWrapper
from e2e_learning.utils.io import ensure_dir, write_json


def _stored_config(checkpoint: dict[str, Any], fallback: DictConfig) -> DictConfig:
    stored = checkpoint.get("cfg")
    return OmegaConf.create(stored) if isinstance(stored, dict) else fallback


@hydra.main(config_path="../conf", config_name="config", version_base=None)
def main(cfg: DictConfig) -> None:
    checkpoint_path = Path(str(cfg.checkpoint)).expanduser()
    if not checkpoint_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    # This is a JetPilot-generated training checkpoint containing its config.
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    if not isinstance(checkpoint, dict):
        checkpoint = {"model_state": checkpoint}
    run_cfg = _stored_config(checkpoint, cfg)
    # The trained checkpoint already contains the complete model. Do not require
    # the original foundation-model file (or download weights) during export.
    export_model_cfg = OmegaConf.create(OmegaConf.to_container(run_cfg.model, resolve=True))
    if "weights_path" in export_model_cfg:
        export_model_cfg.weights_path = ""
    if "require_weights" in export_model_cfg:
        export_model_cfg.require_weights = False
    if "pretrained" in export_model_cfg:
        export_model_cfg.pretrained = False
    model = build_model(export_model_cfg)
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
    input_channels = int(getattr(run_cfg.model, "input_channels", 3))
    modality = str(getattr(run_cfg.data, "modality", "image"))
    image_input_name = str(cfg.export.input_name)
    is_wam = model_name == "wam_dinov3_vits16"
    is_async_rgb_evs = model_name == "async_rgb_evs_control"
    if model_name == "fusion" and sequence_length > 1:
        dummy_images = torch.randn(1, sequence_length, input_channels, input_height, input_width)
    else:
        dummy_images = torch.randn(1, input_channels, input_height, input_width)

    input_names = [image_input_name]
    inputs: tuple[torch.Tensor, ...] | torch.Tensor = dummy_images
    input_metadata = [
        {
            "name": image_input_name,
            "kind": (
                "event_tensor_sequence" if modality == "event_tensor" and dummy_images.ndim == 5
                else "event_tensor" if modality == "event_tensor"
                else "image_sequence" if dummy_images.ndim == 5 else "image"
            ),
            "shape": list(dummy_images.shape),
            "layout": "NTCHW" if dummy_images.ndim == 5 else "NCHW",
            "color_order": "none" if modality == "event_tensor" else "rgb",
            "mean": [float(value) for value in run_cfg.data.mean],
            "std": [float(value) for value in run_cfg.data.std],
        }
    ]
    export_model = model
    output_names = ["trajectory" if task == "trajectory" else str(cfg.export.output_name)]
    if is_async_rgb_evs:
        rollout_steps = int(getattr(run_cfg.model, "rollout_steps", 8))
        event_channels = int(getattr(run_cfg.model, "event_channels", 20))
        dummy_events = torch.zeros(1, rollout_steps, event_channels, input_height, input_width)
        dummy_delta_t = torch.full((1, rollout_steps), float(getattr(run_cfg.data, "event_stride_ms", 4.0)) / 1000.0)
        dummy_mask = torch.ones(1, rollout_steps)
        inputs = (dummy_images, dummy_events, dummy_delta_t, dummy_mask)
        input_names = ["rgb", "event_tensors", "delta_t", "event_mask"]
        input_metadata = [
            {"name": "rgb", "kind": "image", "shape": list(dummy_images.shape), "layout": "NCHW",
             "color_order": "rgb", "mean": [float(v) for v in run_cfg.data.mean], "std": [float(v) for v in run_cfg.data.std]},
            {"name": "event_tensors", "kind": "event_tensor_sequence", "shape": list(dummy_events.shape),
             "layout": "NTCHW", "mean": [float(v) for v in run_cfg.data.event_mean],
             "std": [float(v) for v in run_cfg.data.event_std]},
            {"name": "delta_t", "kind": "time_delta_sequence", "shape": list(dummy_delta_t.shape), "layout": "NT"},
            {"name": "event_mask", "kind": "sequence_mask", "shape": list(dummy_mask.shape), "layout": "NT"},
        ]
        output_names = ["control_sequence"]
    elif is_wam:
        hidden_dim = int(getattr(run_cfg.model, "hidden_dim", 256))
        dummy_hidden = torch.zeros(1, hidden_dim)
        dummy_action = torch.zeros(1, 2)
        inputs = (dummy_images, dummy_hidden, dummy_action)
        input_names.extend(("hidden_state", "previous_action"))
        input_metadata.extend(
            (
                {
                    "name": "hidden_state",
                    "kind": "recurrent_state",
                    "shape": [1, hidden_dim],
                    "layout": "NC",
                    "initial_value": 0.0,
                },
                {
                    "name": "previous_action",
                    "kind": "control_state",
                    "shape": [1, 2],
                    "layout": "NC",
                    "fields": ["steering", "throttle"],
                    "initial_value": 0.0,
                },
            )
        )
        output_names = ["control", "next_hidden_state", "future_controls", "future_latents"]
        export_model = WAMTensorRTWrapper(model)
    elif use_imu:
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

    output_name = output_names[0]
    onnx_path = output_dir / str(cfg.export.onnx_filename)
    requested_opset = int(cfg.export.opset_version)
    # Current PyTorch lowers GRUCell gate chunking to Split with the
    # num_outputs attribute. That attribute is part of Split-18, and an
    # opset-17 graph is rejected by ONNX Runtime as INVALID_GRAPH.
    opset_version = max(requested_opset, 18) if is_async_rgb_evs else requested_opset
    torch.onnx.export(
        export_model,
        inputs,
        onnx_path,
        input_names=input_names,
        output_names=output_names,
        opset_version=opset_version,
        dynamic_axes=None,
        external_data=False,
    )
    if is_async_rgb_evs:
        # Fail the export task immediately if the recurrent graph cannot be
        # consumed by the same runtime used by Console Offline Analysis.
        import onnxruntime as ort

        ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])

    if is_async_rgb_evs:
        output_metadata = {
            "name": output_name, "task": "control", "shape": [1, int(run_cfg.model.rollout_steps), 2],
            "fields": ["steering", "throttle"], "sequence": True,
            "activations": ["tanh", "sigmoid"],
        }
    elif task == "trajectory":
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

    if is_wam:
        future_horizon = int(getattr(run_cfg.model, "future_horizon", 6))
        hidden_dim = int(getattr(run_cfg.model, "hidden_dim", 256))
        output_metadata["world_model"] = {
            "future_horizon": future_horizon,
            "future_stride": int(getattr(run_cfg.model, "future_stride", 1)),
            "outputs": [
                {"name": "next_hidden_state", "shape": [1, hidden_dim]},
                {"name": "future_controls", "shape": [1, future_horizon, 2]},
                {
                    "name": "future_latents",
                    "shape": [1, future_horizon, int(getattr(run_cfg.model, "embed_dim", 384))],
                },
            ],
        }

    if bool(getattr(run_cfg.model, "steering_only", False)):
        output_metadata["activations"] = ["tanh", "constant_zero"]
        output_metadata["learned_fields"] = ["steering"]
        output_metadata["requires_fixed_throttle_mode"] = True

    metadata = {
        "format_version": 2,
        "opset_version": opset_version,
        "model_name": str(run_cfg.run.name),
        "image_topic": str(run_cfg.data.image_topic),
        "modality": (
            "rgb_event_async" if modality == "rgb_event_async"
            else "event_tensor" if modality == "event_tensor"
            else "event_image" if str(run_cfg.data.image_topic).endswith("/event_image")
            else "rgb"
        ),
        "model_kind": model_name,
        "task": task,
        "steering_only": bool(getattr(run_cfg.model, "steering_only", False)),
        "architecture": {
            "backbone": str(getattr(run_cfg.model, "backbone", model_name)),
            "temporal": str(getattr(run_cfg.model, "temporal", "none")),
            "use_imu": use_imu,
            "sequence_length": sequence_length,
            "future_horizon": int(getattr(run_cfg.model, "future_horizon", 0)),
            "future_stride": int(getattr(run_cfg.model, "future_stride", 1)),
            "stateful_step": is_wam,
            "async_rgb_evs": is_async_rgb_evs,
            "rollout_steps": int(getattr(run_cfg.model, "rollout_steps", 1)),
        },
        "checkpoint": str(checkpoint_path),
        "source_dataset": str(run_cfg.data.dataset_dir),
        "stage": checkpoint.get("stage", ""),
        "epoch": checkpoint.get("epoch", 0),
        "input": input_metadata[0],
        "inputs": input_metadata,
        "output": output_metadata,
        "tensorrt": {
            "engine_filename": "model.plan",
            "enable_fp16": bool(cfg.export.tensorrt.enable_fp16),
            "input_tensor_names": input_names,
            "input_binding_names": input_names,
            "output_tensor_name": str(cfg.export.tensorrt.output_tensor_name),
            "output_tensor_names": output_names,
            "output_binding_names": output_names,
            "input_tensor_format": "nitros_tensor_list_nchw_rgb_f32",
            "output_tensor_format": "nitros_tensor_list_nchw_rgb_f32",
            "requires_state_adapter": is_wam,
        },
        "config": OmegaConf.to_container(run_cfg, resolve=True),
    }
    if modality in {"event_tensor", "rgb_event_async"}:
        metadata["event_representation"] = {
            "event_topic": str(getattr(run_cfg.data, "event_topic", "/event_camera/events")),
            "bins": int(getattr(run_cfg.data, "event_bins", input_channels // 2)),
            "channels": int(getattr(run_cfg.model, "event_channels", input_channels)),
            "window_ms": float(getattr(run_cfg.data, "event_window_ms", 40.0)),
            "stride_ms": float(getattr(run_cfg.data, "event_stride_ms", 4.0)),
            "polarity_mode": str(getattr(run_cfg.data, "event_polarity_mode", "separate")),
            "polarity_layout": str(
                getattr(run_cfg.data, "event_polarity_layout", "polarity_major")
            ),
            "temporal_interpolation": str(
                getattr(run_cfg.data, "event_temporal_interpolation", "none")
            ),
            "clock_source": "event_sensor_time",
        }
    write_json(output_dir / str(cfg.export.metadata_filename), metadata)
    print(f"Exported ONNX: {onnx_path}")
    print(f"Metadata     : {output_dir / str(cfg.export.metadata_filename)}")


if __name__ == "__main__":
    main()
