#!/usr/bin/env python3
"""Inspect deployed E2E models without importing ROS, Torch, or TensorRT."""
import argparse
import json
from pathlib import Path


def inspect_model(
    path: Path,
    sensor: str,
    steering_only: bool | None,
    event_tensor_channels: int | None = None,
) -> dict:
    path = path.expanduser().resolve()
    metadata = json.loads((path / "metadata.json").read_text())
    config = metadata.get("config", {})
    data = config.get("data", {})
    output = metadata.get("output", {})
    topic = str(metadata.get("image_topic") or data.get("image_topic") or "")
    modality = metadata.get("modality")
    actual_sensor = (
        "rgb-event" if modality == "rgb_event_async"
        else "event" if modality in ("event_image", "event_tensor") or topic.endswith("/event_image")
        else "rgb" if modality == "rgb" or topic
        else "unknown"
    )
    actual_steering = metadata.get("steering_only") is True or output.get("learned_fields") == ["steering"] or output.get("requires_fixed_throttle_mode") is True
    if actual_sensor != sensor:
        raise ValueError(f"sensor mismatch: selected={sensor}, model={actual_sensor}")
    if str(metadata.get("task") or output.get("task") or "control") != "control":
        raise ValueError("this bringup requires a control model, not a trajectory model")
    if steering_only is not None and actual_steering != steering_only:
        raise ValueError("model learning target does not match e2e/e2e-steering")
    shape = metadata.get("input", {}).get("shape", [])
    architecture = metadata.get("architecture", {})
    if modality == "rgb_event_async":
        inputs = metadata.get("inputs", [])
        rgb_input = next((item for item in inputs if item.get("name") == "rgb"), {})
        event_input = next(
            (item for item in inputs if item.get("name") == "event_tensors"), {}
        )
        rgb_shape = rgb_input.get("shape", [])
        event_shape = event_input.get("shape", [])
        if (
            len(rgb_shape) != 4 or rgb_shape[1] != 3
            or len(event_shape) != 5
            or not all(isinstance(size, int) and size > 0 for size in rgb_shape[2:])
            or event_shape[-2:] != rgb_shape[-2:]
        ):
            raise ValueError("async RGB-EVS model requires fixed RGB NCHW and event NTCHW inputs")
        if event_tensor_channels is not None and event_shape[2] != event_tensor_channels:
            raise ValueError(
                "async event tensor channels mismatch: "
                f"requested={event_tensor_channels}, model={event_shape[2]}"
            )
        for filename in ("rgb_encoder.onnx", "event_updater.onnx"):
            if not (path / filename).is_file():
                raise ValueError(f"{filename} is missing")
        representation = metadata.get("event_representation", {})
        bins = int(representation.get("bins") or event_shape[2] // 2)
        event = {
            "bins": bins,
            "window_ms": float(representation.get("window_ms") or 40.0),
            "stride_ms": float(representation.get("stride_ms") or 4.0),
            "polarity_mode": str(representation.get("polarity_mode") or "separate"),
            "polarity_layout": str(
                representation.get("polarity_layout") or "polarity_major"
            ),
            "temporal_interpolation": str(
                representation.get("temporal_interpolation") or "none"
            ),
            "event_topic": str(
                representation.get("event_topic") or "/event_camera/events"
            ),
            "mean": event_input.get("mean") or [0.0],
            "std": event_input.get("std") or [1.0],
        }
        window_us = int(round(event["window_ms"] * 1000.0))
        stride_us = int(round(event["stride_ms"] * 1000.0))
        bin_width_us = window_us // bins if bins > 0 and window_us % bins == 0 else 0
        event["backend"] = (
            "cuda"
            if event["temporal_interpolation"] == "none"
            and bin_width_us > 0
            and stride_us % bin_width_us == 0
            else "cpu"
        )
        event["output_rate_hz"] = float(
            data.get("event_sample_hz") or (1000.0 / event["stride_ms"])
        )
        return {
            "path": str(path),
            "name": str(metadata.get("model_name") or path.name),
            "width": rgb_shape[3],
            "height": rgb_shape[2],
            "steering_only": actual_steering,
            "engine": all(
                (path / filename).is_file()
                for filename in ("rgb_encoder.plan", "event_updater.plan")
            ),
            "event_representation": event,
            "rgb": {
                "image_topic": topic or "/realsense/color/image_raw",
                "mean": rgb_input.get("mean") or [0.485, 0.456, 0.406],
                "std": rgb_input.get("std") or [0.229, 0.224, 0.225],
            },
            "async_rgb_evs": True,
        }
    if len(shape) != 4 or architecture.get("use_imu") or len(metadata.get("inputs", [])) > 1:
        if event_tensor_channels is None:
            raise ValueError("online inference requires a single RGB-format image and no IMU input")
        raise ValueError("online event tensor inference requires one fixed NCHW input and no IMU input")
    if event_tensor_channels is None:
        if shape[1] != 3 or modality == "event_tensor":
            raise ValueError("online image inference requires a single RGB-format image")
    elif modality != "event_tensor" or shape[1] != event_tensor_channels:
        raise ValueError(
            "event tensor model mismatch: metadata modality must be event_tensor "
            f"and input channels must be {event_tensor_channels}"
        )
    if not (path / "model.onnx").is_file():
        raise ValueError("model.onnx is missing")
    if not all(isinstance(size, int) and size > 0 for size in shape[2:]):
        raise ValueError("model image dimensions must be fixed positive integers")
    record = {
        "path": str(path), "name": str(metadata.get("model_name") or path.name),
        "width": shape[3], "height": shape[2], "steering_only": actual_steering,
        "engine": (path / "model.plan").is_file(),
    }
    if modality == "event_tensor":
        representation = metadata.get("event_representation", {})
        model_input = metadata.get("input", {})
        bins = int(representation.get("bins") or shape[1] // 2)
        if event_tensor_channels is not None and shape[1] != event_tensor_channels:
            raise ValueError(
                f"event tensor channels mismatch: requested={event_tensor_channels}, model={shape[1]}"
            )
        record["event_representation"] = {
            "bins": bins,
            "window_ms": float(representation.get("window_ms") or 40.0),
            "stride_ms": float(representation.get("stride_ms") or 4.0),
            "polarity_mode": str(representation.get("polarity_mode") or "separate"),
            "polarity_layout": str(representation.get("polarity_layout") or "polarity_major"),
            "temporal_interpolation": str(
                representation.get("temporal_interpolation") or "none"
            ),
            "mean": model_input.get("mean") or [0.0],
            "std": model_input.get("std") or [1.0],
        }
        event = record["event_representation"]
        window_us = int(round(event["window_ms"] * 1000.0))
        stride_us = int(round(event["stride_ms"] * 1000.0))
        bin_width_us = window_us // bins if bins > 0 and window_us % bins == 0 else 0
        event["backend"] = (
            "cuda"
            if event["temporal_interpolation"] == "none"
            and bin_width_us > 0
            and stride_us % bin_width_us == 0
            else "cpu"
        )
    return record


def list_models(
    root: Path,
    sensor: str,
    steering_only: bool | None,
    event_tensor_channels: int | None = None,
) -> list[dict]:
    records = []
    seen = set()
    if not root.is_dir():
        return records
    for path in sorted(root.iterdir()):
        if not path.is_dir() or path.resolve() in seen:
            continue
        seen.add(path.resolve())
        try:
            records.append(inspect_model(path, sensor, steering_only, event_tensor_channels))
        except (OSError, ValueError, TypeError, AttributeError):
            continue
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["list", "validate"])
    parser.add_argument("path", type=Path)
    parser.add_argument("--sensor", choices=["rgb", "event", "rgb-event"], required=True)
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--steering-only", action="store_true")
    target.add_argument("--auto-throttle", action="store_true")
    parser.add_argument("--event-tensor-channels", type=int)
    args = parser.parse_args()
    steering_only = None if args.auto_throttle else args.steering_only
    try:
        if args.action == "list":
            for record in list_models(
                args.path, args.sensor, steering_only, args.event_tensor_channels
            ):
                mode = "固定スロットル" if record["steering_only"] else "モデルがスロットルを予測"
                label = f"{record['name']} ({mode}, {record['width']}x{record['height']}, {'TRT ready' if record['engine'] else 'build needed'})"
                if not any(char in record["path"] + label for char in "\t\r\n"):
                    print(f"{record['path']}\t{label}")
        else:
            record = inspect_model(
                args.path, args.sensor, steering_only, args.event_tensor_channels
            )
            if args.auto_throttle:
                print(f"e2e_fixed_throttle_mode\t{str(record['steering_only']).lower()}")
            print(f"e2e_network_image_width\t{record['width']}")
            print(f"e2e_network_image_height\t{record['height']}")
            event = record.get("event_representation")
            if event:
                if record.get("async_rgb_evs"):
                    print("e2e_async_rgb_evs_mode\ttrue")
                    print("e2e_event_tensor_mode\tfalse")
                    print(f"e2e_image_topic\t{record['rgb']['image_topic']}")
                    print(
                        "e2e_async_rgb_mean\t"
                        + json.dumps(record["rgb"]["mean"], separators=(",", ":"))
                    )
                    print(
                        "e2e_async_rgb_stddev\t"
                        + json.dumps(record["rgb"]["std"], separators=(",", ":"))
                    )
                    print(f"e2e_async_event_output_rate_hz\t{event['output_rate_hz']}")
                else:
                    print("e2e_event_tensor_mode\ttrue")
                print(f"e2e_event_bins\t{event['bins']}")
                print(f"e2e_event_window_ms\t{event['window_ms']}")
                print(f"e2e_event_stride_ms\t{event['stride_ms']}")
                print(f"e2e_event_polarity_mode\t{event['polarity_mode']}")
                print(f"e2e_event_polarity_layout\t{event['polarity_layout']}")
                print(f"e2e_event_temporal_interpolation\t{event['temporal_interpolation']}")
                print(f"e2e_event_representation_backend\t{event['backend']}")
                if event.get("event_topic"):
                    print(f"e2e_event_topic\t{event['event_topic']}")
                print(
                    "e2e_event_tensor_mean\t"
                    + json.dumps(event["mean"], separators=(",", ":"))
                )
                print(
                    "e2e_event_tensor_stddev\t"
                    + json.dumps(event["std"], separators=(",", ":"))
                )
    except (OSError, ValueError, TypeError, AttributeError) as error:
        parser.exit(1, f"E2E model error: {error}\n")


if __name__ == "__main__":
    main()
