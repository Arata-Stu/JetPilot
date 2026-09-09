#!/usr/bin/env python3
"""Inspect deployed E2E models without importing ROS, Torch, or TensorRT."""
import argparse
import json
from pathlib import Path


def inspect_model(path: Path, sensor: str, steering_only: bool | None) -> dict:
    path = path.expanduser().resolve()
    metadata = json.loads((path / "metadata.json").read_text())
    config = metadata.get("config", {})
    data = config.get("data", {})
    output = metadata.get("output", {})
    topic = str(metadata.get("image_topic") or data.get("image_topic") or "")
    modality = metadata.get("modality")
    actual_sensor = "event" if modality == "event_image" or topic.endswith("/event_image") else "rgb" if modality == "rgb" or topic else "unknown"
    actual_steering = metadata.get("steering_only") is True or output.get("learned_fields") == ["steering"] or output.get("requires_fixed_throttle_mode") is True
    if actual_sensor != sensor:
        raise ValueError(f"sensor mismatch: selected={sensor}, model={actual_sensor}")
    if str(metadata.get("task") or output.get("task") or "control") != "control":
        raise ValueError("this bringup requires a control model, not a trajectory model")
    if steering_only is not None and actual_steering != steering_only:
        raise ValueError("model learning target does not match e2e/e2e-steering")
    shape = metadata.get("input", {}).get("shape", [])
    architecture = metadata.get("architecture", {})
    if len(shape) != 4 or shape[1] != 3 or architecture.get("use_imu") or len(metadata.get("inputs", [])) > 1:
        raise ValueError("online inference requires a single RGB-format image and no IMU input")
    if not (path / "model.onnx").is_file():
        raise ValueError("model.onnx is missing")
    if not all(isinstance(size, int) and size > 0 for size in shape[2:]):
        raise ValueError("model image dimensions must be fixed positive integers")
    return {
        "path": str(path), "name": str(metadata.get("model_name") or path.name),
        "width": shape[3], "height": shape[2], "steering_only": actual_steering,
        "engine": (path / "model.plan").is_file(),
    }


def list_models(root: Path, sensor: str, steering_only: bool | None) -> list[dict]:
    records = []
    seen = set()
    if not root.is_dir():
        return records
    for path in sorted(root.iterdir()):
        if not path.is_dir() or path.resolve() in seen:
            continue
        seen.add(path.resolve())
        try:
            records.append(inspect_model(path, sensor, steering_only))
        except (OSError, ValueError, TypeError, AttributeError):
            continue
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["list", "validate"])
    parser.add_argument("path", type=Path)
    parser.add_argument("--sensor", choices=["rgb", "event"], required=True)
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--steering-only", action="store_true")
    target.add_argument("--auto-throttle", action="store_true")
    args = parser.parse_args()
    steering_only = None if args.auto_throttle else args.steering_only
    try:
        if args.action == "list":
            for record in list_models(args.path, args.sensor, steering_only):
                mode = "固定スロットル" if record["steering_only"] else "モデルがスロットルを予測"
                label = f"{record['name']} ({mode}, {record['width']}x{record['height']}, {'TRT ready' if record['engine'] else 'build needed'})"
                if not any(char in record["path"] + label for char in "\t\r\n"):
                    print(f"{record['path']}\t{label}")
        else:
            record = inspect_model(args.path, args.sensor, steering_only)
            if args.auto_throttle:
                print(f"e2e_fixed_throttle_mode\t{str(record['steering_only']).lower()}")
            print(f"e2e_network_image_width\t{record['width']}")
            print(f"e2e_network_image_height\t{record['height']}")
    except (OSError, ValueError, TypeError, AttributeError) as error:
        parser.exit(1, f"E2E model error: {error}\n")


if __name__ == "__main__":
    main()
