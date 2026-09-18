#!/usr/bin/env python3
"""Generate a benchmark-only 20-channel event-tensor ONNX model.

The graph deliberately depends on the complete input tensor but performs only a
global reduction and two scalar activations. It is a lower-bound model for
measuring the TensorRT node and surrounding ROS/NITROS pipeline before a trained
event model exists. It is not a driving model.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def metadata(*, name: str, channels: int, height: int, width: int) -> dict:
    if channels <= 0 or channels % 2:
        raise ValueError("channels must be a positive even integer")
    bins = channels // 2
    return {
        "format_version": 2,
        "opset_version": 17,
        "model_name": name,
        "modality": "event_tensor",
        "model_kind": "dummy_event_tensor_reduce",
        "task": "control",
        "steering_only": False,
        "benchmark_only": True,
        "trained": False,
        "purpose": "TensorRT and ROS/NITROS pipeline lower-bound benchmark",
        "architecture": {
            "backbone": "global_mean_reduce",
            "temporal": "none",
            "use_imu": False,
            "sequence_length": 1,
        },
        "input": {
            "name": "image",
            "kind": "event_tensor",
            "shape": [1, channels, height, width],
            "layout": "NCHW",
            "color_order": "none",
            "mean": [0.0] * channels,
            "std": [1.0] * channels,
        },
        "inputs": [
            {
                "name": "image",
                "kind": "event_tensor",
                "shape": [1, channels, height, width],
                "layout": "NCHW",
                "color_order": "none",
                "mean": [0.0] * channels,
                "std": [1.0] * channels,
            }
        ],
        "output": {
            "name": "control",
            "task": "control",
            "shape": [1, 2],
            "fields": ["steering", "throttle"],
            "activations": ["tanh", "sigmoid"],
        },
        "event_representation": {
            "event_topic": "/event_camera/events",
            "bins": bins,
            "channels": channels,
            "window_ms": 40.0,
            "stride_ms": 4.0,
            "polarity_mode": "separate",
            "polarity_layout": "polarity_major",
            "temporal_interpolation": "none",
            "clock_source": "event_sensor_time",
        },
        "tensorrt": {
            "engine_filename": "model.plan",
            "enable_fp16": True,
            "input_tensor_names": ["input_tensor"],
            "input_binding_names": ["image"],
            "output_tensor_name": "output_tensor",
            "output_tensor_names": ["output_tensor"],
            "output_binding_names": ["control"],
            "input_tensor_format": "nitros_tensor_list_nchw_rgb_f32",
            "output_tensor_format": "nitros_tensor_list_nchw_rgb_f32",
            "requires_state_adapter": False,
        },
        "config": {
            "data": {
                "modality": "event_tensor",
                "event_topic": "/event_camera/events",
                "event_bins": bins,
                "event_window_ms": 40.0,
                "event_stride_ms": 4.0,
                "event_polarity_mode": "separate",
                "event_polarity_layout": "polarity_major",
                "event_temporal_interpolation": "none",
                "input_height": height,
                "input_width": width,
            }
        },
    }


def generate_onnx(path: Path, *, channels: int, height: int, width: int) -> None:
    try:
        import onnx
        from onnx import TensorProto, helper
    except ImportError as error:
        raise SystemExit(
            "The 'onnx' package is required. Run this in the JetPilot E2E "
            "training environment."
        ) from error

    graph = helper.make_graph(
        [
            helper.make_node(
                "ReduceMean", ["image"], ["mean_nchw"],
                axes=[1, 2, 3], keepdims=1,
            ),
            helper.make_node("Flatten", ["mean_nchw"], ["mean"], axis=1),
            helper.make_node("Tanh", ["mean"], ["steering"]),
            helper.make_node("Sigmoid", ["mean"], ["throttle"]),
            helper.make_node(
                "Concat", ["steering", "throttle"], ["control"], axis=1,
            ),
        ],
        "jetpilot_dummy_event_tensor_reduce",
        [helper.make_tensor_value_info(
            "image", TensorProto.FLOAT, [1, channels, height, width]
        )],
        [helper.make_tensor_value_info("control", TensorProto.FLOAT, [1, 2])],
    )
    model = helper.make_model(
        graph,
        producer_name="JetPilot dummy event benchmark generator",
        opset_imports=[helper.make_opsetid("", 17)],
    )
    # IR v8 is supported by the TensorRT versions used by the Jetson images in
    # this project and is sufficient for a fixed-shape opset-17 graph.
    model.ir_version = 8
    onnx.checker.check_model(model)
    onnx.save(model, str(path))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--name", default="dummy-event-tensor-20ch")
    parser.add_argument("--channels", type=int, default=20)
    parser.add_argument("--height", type=int, default=120)
    parser.add_argument("--width", type=int, default=212)
    args = parser.parse_args()

    if args.channels <= 0 or args.channels % 2:
        parser.error("--channels must be a positive even integer")
    if args.height <= 0 or args.width <= 0:
        parser.error("--height and --width must be positive")

    output_dir = args.output_dir.expanduser().resolve()
    onnx_path = output_dir / "model.onnx"
    metadata_path = output_dir / "metadata.json"
    engine_path = output_dir / "model.plan"
    existing = [path for path in (onnx_path, metadata_path, engine_path) if path.exists()]
    if existing:
        parser.error(
            "refusing to overwrite existing model artifacts: "
            + ", ".join(str(path) for path in existing)
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    generate_onnx(
        onnx_path, channels=args.channels, height=args.height, width=args.width
    )
    metadata_path.write_text(
        json.dumps(
            metadata(
                name=args.name,
                channels=args.channels,
                height=args.height,
                width=args.width,
            ),
            indent=2,
            ensure_ascii=False,
        ) + "\n",
        encoding="utf-8",
    )
    print(f"ONNX:     {onnx_path}")
    print(f"Metadata: {metadata_path}")
    print("Benchmark-only model generated; do not use it for vehicle control.")


if __name__ == "__main__":
    main()
