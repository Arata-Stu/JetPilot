#!/usr/bin/env python3

from __future__ import annotations

import math
import pickle
import time
from pathlib import Path
from typing import Any

import rclpy
import torch
import torch.nn.functional as functional
from jetpilot_msgs.msg import ControlCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from torch import nn


class PilotNet(nn.Module):
    """Deployment copy of the training-side PilotNet architecture."""

    def __init__(self, input_channels: int = 3, output_dim: int = 2) -> None:
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(input_channels, 24, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=3),
            nn.ELU(inplace=True),
            nn.Conv2d(64, 64, kernel_size=3),
            nn.ELU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 8)),
            nn.Flatten(),
        )
        self.head = nn.Sequential(
            nn.Linear(64 * 4 * 8, 100),
            nn.ELU(inplace=True),
            nn.Linear(100, 50),
            nn.ELU(inplace=True),
            nn.Linear(50, 10),
            nn.ELU(inplace=True),
            nn.Linear(10, output_dim),
        )

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        raw = self.head(self.features(image))
        steering = torch.tanh(raw[:, 0:1])
        throttle = torch.sigmoid(raw[:, 1:2])
        return torch.cat([steering, throttle], dim=1)


class TorchvisionEncoderHead(nn.Module):
    """Deployment copy of the MobileNetV3-small training architecture."""

    def __init__(self, output_dim: int = 2) -> None:
        super().__init__()
        try:
            from torchvision import models
        except ImportError as exc:
            raise RuntimeError(
                "mobilenet_v3_small checkpoints require torchvision"
            ) from exc

        backbone = models.mobilenet_v3_small(weights=None)
        self.encoder = backbone.features
        self.pool = nn.AdaptiveAvgPool2d((1, 1))
        in_features = backbone.classifier[0].in_features
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(in_features, 128),
            nn.Hardswish(inplace=True),
            nn.Dropout(p=0.1),
            nn.Linear(128, output_dim),
        )

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        raw = self.head(self.pool(self.encoder(image)))
        steering = torch.tanh(raw[:, 0:1])
        throttle = torch.sigmoid(raw[:, 1:2])
        return torch.cat([steering, throttle], dim=1)


def _nested_value(mapping: Any, *keys: str, default: Any = None) -> Any:
    current = mapping
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def _load_checkpoint(path: Path) -> Any:
    try:
        return torch.load(path, map_location="cpu", weights_only=True)
    except TypeError:
        # weights_only was added after the first Jetson PyTorch releases.
        return torch.load(path, map_location="cpu")
    except pickle.UnpicklingError:
        # JetPilot checkpoints are trusted local deployment artifacts and older
        # files may contain objects outside the weights-only allowlist.
        return torch.load(path, map_location="cpu", weights_only=False)


def _build_checkpoint_model(model_kind: str, state: dict[str, Any]) -> nn.Module:
    if model_kind == "pilotnet":
        model = PilotNet()
    elif model_kind == "mobilenet_v3_small":
        model = TorchvisionEncoderHead()
    else:
        raise RuntimeError(f"Unsupported checkpoint model: {model_kind}")
    model.load_state_dict(state)
    return model


class E2EPyTorchInferenceNode(Node):
    def __init__(self) -> None:
        super().__init__("e2e_pytorch_inference")

        self.model_file_path = Path(
            str(self.declare_parameter("model_file_path", "").value)
        ).expanduser()
        self.model_format = str(
            self.declare_parameter("model_format", "auto").value
        ).lower()
        self.model_kind = str(
            self.declare_parameter("model_kind", "pilotnet").value
        )
        self.use_checkpoint_config = bool(
            self.declare_parameter("use_checkpoint_config", True).value
        )
        self.input_width = int(self.declare_parameter("input_width", 212).value)
        self.input_height = int(self.declare_parameter("input_height", 120).value)
        self.image_mean = tuple(
            float(value)
            for value in self.declare_parameter(
                "image_mean", [0.485, 0.456, 0.406]
            ).value
        )
        self.image_std = tuple(
            float(value)
            for value in self.declare_parameter(
                "image_std", [0.229, 0.224, 0.225]
            ).value
        )
        self.steering_min = float(
            self.declare_parameter("steering_min", -1.0).value
        )
        self.steering_max = float(
            self.declare_parameter("steering_max", 1.0).value
        )
        self.throttle_min = float(
            self.declare_parameter("throttle_min", 0.0).value
        )
        self.throttle_max = float(
            self.declare_parameter("throttle_max", 1.0).value
        )
        self.max_inference_rate_hz = float(
            self.declare_parameter("max_inference_rate_hz", 0.0).value
        )
        self.log_interval = max(
            0, int(self.declare_parameter("log_interval", 100).value)
        )
        self.use_half = bool(self.declare_parameter("use_half", False).value)
        self.device = self._resolve_device(
            str(self.declare_parameter("device", "cpu").value).lower()
        )
        cpu_threads = int(self.declare_parameter("cpu_threads", 0).value)
        if cpu_threads > 0:
            torch.set_num_threads(cpu_threads)

        self.model, checkpoint_config = self._load_model()
        self._apply_checkpoint_config(checkpoint_config)
        if len(self.image_mean) != 3 or len(self.image_std) != 3:
            raise RuntimeError("image_mean and image_std must contain three values")
        if any(value <= 0.0 for value in self.image_std):
            raise RuntimeError("image_std values must be positive")

        self.tensor_dtype = (
            torch.float16 if self.use_half and self.device.type == "cuda" else torch.float32
        )
        if self.use_half and self.device.type != "cuda":
            self.get_logger().warning("use_half is ignored on a non-CUDA device")
        self.model = self.model.to(device=self.device, dtype=self.tensor_dtype)
        self.model.eval()
        self.mean_tensor = torch.tensor(
            self.image_mean, device=self.device, dtype=self.tensor_dtype
        ).view(1, 3, 1, 1)
        self.std_tensor = torch.tensor(
            self.image_std, device=self.device, dtype=self.tensor_dtype
        ).view(1, 3, 1, 1)

        self.last_inference_monotonic = 0.0
        self.last_error_log_monotonic = 0.0
        self.inference_count = 0

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = self.create_subscription(
            Image, "image", self._image_callback, image_qos
        )
        self.command_pub = self.create_publisher(ControlCommand, "control_cmd", 10)

        self._warm_up()
        self.get_logger().info(
            f"PyTorch E2E inference ready: model={self.model_file_path}, "
            f"device={self.device}, input={self.input_width}x{self.input_height}, "
            f"dtype={self.tensor_dtype}"
        )

    def _resolve_device(self, requested: str) -> torch.device:
        if requested == "auto":
            requested = "cuda" if torch.cuda.is_available() else "cpu"
        if requested == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("device=cuda was requested but CUDA is unavailable")
        if requested not in {"cpu", "cuda"}:
            raise RuntimeError("device must be one of: cpu, cuda, auto")
        return torch.device(requested)

    def _load_model(self) -> tuple[nn.Module, dict[str, Any]]:
        if not self.model_file_path.is_file():
            raise RuntimeError(f"Model file was not found: {self.model_file_path}")
        if self.model_format not in {"auto", "checkpoint", "torchscript"}:
            raise RuntimeError("model_format must be one of: auto, checkpoint, torchscript")

        if self.model_format in {"auto", "torchscript"}:
            try:
                return torch.jit.load(str(self.model_file_path), map_location="cpu"), {}
            except (RuntimeError, ValueError):
                if self.model_format == "torchscript":
                    raise

        checkpoint = _load_checkpoint(self.model_file_path)
        checkpoint_config = checkpoint.get("cfg", {}) if isinstance(checkpoint, dict) else {}
        state = (
            checkpoint.get("model_state", checkpoint)
            if isinstance(checkpoint, dict)
            else checkpoint
        )
        if not isinstance(state, dict):
            raise RuntimeError("Checkpoint does not contain a model state dictionary")
        model_kind = str(
            _nested_value(checkpoint_config, "model", "name", default=self.model_kind)
        )
        self.model_kind = model_kind
        return _build_checkpoint_model(model_kind, state), checkpoint_config

    def _apply_checkpoint_config(self, config: dict[str, Any]) -> None:
        if not self.use_checkpoint_config or not config:
            return
        self.input_width = int(
            _nested_value(config, "data", "input_width", default=self.input_width)
        )
        self.input_height = int(
            _nested_value(config, "data", "input_height", default=self.input_height)
        )
        mean = _nested_value(config, "data", "mean", default=self.image_mean)
        std = _nested_value(config, "data", "std", default=self.image_std)
        self.image_mean = tuple(float(value) for value in mean)
        self.image_std = tuple(float(value) for value in std)

    def _warm_up(self) -> None:
        dummy = torch.zeros(
            (1, 3, self.input_height, self.input_width),
            device=self.device,
            dtype=self.tensor_dtype,
        )
        with torch.inference_mode():
            output = self.model(dummy)
        if not isinstance(output, torch.Tensor) or output.numel() < 2:
            raise RuntimeError("Model output must be a tensor containing steering and throttle")
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)

    @staticmethod
    def _image_channels(encoding: str) -> tuple[int, bool, bool]:
        normalized = encoding.lower()
        if normalized == "rgb8":
            return 3, False, False
        if normalized == "bgr8":
            return 3, True, False
        if normalized == "rgba8":
            return 4, False, False
        if normalized == "bgra8":
            return 4, True, False
        if normalized in {"mono8", "8uc1"}:
            return 1, False, True
        raise ValueError(f"Unsupported image encoding: {encoding}")

    def _prepare_image(self, msg: Image) -> torch.Tensor:
        channels, is_bgr, is_mono = self._image_channels(msg.encoding)
        row_bytes = int(msg.width) * channels
        if msg.height <= 0 or msg.width <= 0 or msg.step < row_bytes:
            raise ValueError(
                f"Invalid image dimensions or step: {msg.width}x{msg.height}, step={msg.step}"
            )
        required_bytes = int(msg.height) * int(msg.step)
        if len(msg.data) < required_bytes:
            raise ValueError(
                f"Image data is truncated: {len(msg.data)} < {required_bytes} bytes"
            )

        image = torch.frombuffer(msg.data, dtype=torch.uint8, count=required_bytes)
        image = image.reshape(int(msg.height), int(msg.step))[:, :row_bytes]
        image = image.reshape(int(msg.height), int(msg.width), channels)
        if is_mono:
            image = image.expand(-1, -1, 3)
        else:
            image = image[:, :, :3]
            if is_bgr:
                image = image.flip(-1)
        image = image.permute(2, 0, 1).contiguous().unsqueeze(0)
        image = image.to(device=self.device, dtype=self.tensor_dtype)
        image = image / 255.0
        if image.shape[-2:] != (self.input_height, self.input_width):
            image = functional.interpolate(
                image,
                size=(self.input_height, self.input_width),
                mode="area",
            )
        return (image - self.mean_tensor) / self.std_tensor

    def _rate_limited(self, now: float) -> bool:
        if self.max_inference_rate_hz <= 0.0:
            return False
        minimum_interval = 1.0 / self.max_inference_rate_hz
        return now - self.last_inference_monotonic < minimum_interval

    def _log_callback_error(self, exc: Exception) -> None:
        now = time.monotonic()
        if now - self.last_error_log_monotonic >= 1.0:
            self.get_logger().error(f"E2E inference failed: {exc}")
            self.last_error_log_monotonic = now

    def _image_callback(self, msg: Image) -> None:
        callback_started = time.perf_counter_ns()
        now = time.monotonic()
        if self._rate_limited(now):
            return
        self.last_inference_monotonic = now

        try:
            image = self._prepare_image(msg)
            with torch.inference_mode():
                output = self.model(image).reshape(-1)
            if output.numel() < 2:
                raise ValueError("Model output contains fewer than two values")
            steering = float(output[0].item())
            throttle = float(output[1].item())
            if not math.isfinite(steering) or not math.isfinite(throttle):
                raise ValueError("Model output contains a non-finite value")

            command = ControlCommand()
            command.header = msg.header
            command.header.frame_id = command.header.frame_id or "base_link"
            command.steering = min(max(steering, self.steering_min), self.steering_max)
            command.throttle = min(max(throttle, self.throttle_min), self.throttle_max)
            command.brake = 0.0
            command.reverse = 0.0
            self.command_pub.publish(command)

            self.inference_count += 1
            if self.log_interval > 0 and self.inference_count % self.log_interval == 0:
                callback_ms = (time.perf_counter_ns() - callback_started) / 1.0e6
                self.get_logger().info(
                    f"inference_count={self.inference_count}, callback_ms={callback_ms:.3f}"
                )
        except (RuntimeError, ValueError, TypeError) as exc:
            self._log_callback_error(exc)


def main() -> None:
    rclpy.init()
    node = E2EPyTorchInferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
