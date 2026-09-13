from __future__ import annotations

import csv
import json
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

from e2e_learning.data.transforms import ImageTransform


class E2EDataset(Dataset):
    def __init__(
        self,
        dataset_dir: str | Path,
        input_width: int,
        input_height: int,
        mean: tuple[float, ...],
        std: tuple[float, ...],
        task: str = "control",
        sequence_length: int = 1,
        frame_stride: int = 1,
        trajectory_points: int = 10,
        trajectory_scale_m: float = 5.0,
        imu_samples: int = 10,
        imu_features: int = 7,
        data_fraction: float = 1.0,
        future_horizon: int = 0,
        future_stride: int = 1,
    ) -> None:
        self.dataset_dir = Path(dataset_dir)
        self.samples_path = self.dataset_dir / "samples.csv"
        self.transform = ImageTransform(input_width, input_height, mean, std)
        self.input_width = int(input_width)
        self.input_height = int(input_height)
        self.mean = np.asarray(mean, dtype=np.float32)
        self.std = np.asarray(std, dtype=np.float32)
        self.task = str(task)
        self.sequence_length = int(sequence_length)
        self.frame_stride = int(frame_stride)
        self.trajectory_points = int(trajectory_points)
        self.trajectory_scale_m = float(trajectory_scale_m)
        self.imu_samples = int(imu_samples)
        self.imu_features = int(imu_features)
        self.future_horizon = int(future_horizon)
        self.future_stride = int(future_stride)
        if self.task not in {"control", "trajectory"}:
            raise ValueError("task must be control or trajectory")
        if self.sequence_length < 1 or self.frame_stride < 1:
            raise ValueError("sequence_length and frame_stride must be positive")
        if self.future_horizon < 0 or self.future_stride < 1:
            raise ValueError("future_horizon must be non-negative and future_stride positive")
        if self.trajectory_points < 2 or self.trajectory_scale_m <= 0.0:
            raise ValueError("trajectory geometry configuration is invalid")
        self.rows = self._read_rows()
        if not self.rows:
            raise RuntimeError(f"No training samples found in {self.samples_path}")
        if data_fraction <= 0.0 or data_fraction > 1.0:
            raise ValueError("data_fraction must be in (0.0, 1.0]")
        keep = max(1, int(len(self.rows) * data_fraction))
        self.rows = self.rows[:keep]

    def _read_rows(self) -> list[dict[str, str]]:
        with self.samples_path.open(newline="") as fp:
            return list(csv.DictReader(fp))

    def __len__(self) -> int:
        return len(self.rows)

    def _sequence_indices(self, index: int) -> list[int]:
        sequence_id = self.rows[index].get("sequence_id", "")
        values: list[int] = []
        for offset in reversed(range(self.sequence_length)):
            candidate = max(0, index - offset * self.frame_stride)
            while candidate < index and self.rows[candidate].get("sequence_id", "") != sequence_id:
                candidate += 1
            if self.rows[candidate].get("sequence_id", "") != sequence_id:
                candidate = index
            values.append(candidate)
        return values

    def _image(self, row: dict[str, str]) -> torch.Tensor:
        tensor_path_value = str(row.get("tensor_path") or "").strip()
        if tensor_path_value:
            tensor_path = self.dataset_dir / tensor_path_value
            tensor = np.load(tensor_path, allow_pickle=False).astype(np.float32, copy=False)
            if tensor.ndim != 3 or tensor.shape[1:] != (self.input_height, self.input_width):
                raise RuntimeError(
                    f"Expected event tensor CHW (*,{self.input_height},{self.input_width}), "
                    f"got {tensor.shape}: {tensor_path}"
                )
            channels = int(tensor.shape[0])
            mean = self.mean if self.mean.size == channels else np.zeros(channels, dtype=np.float32)
            std = self.std if self.std.size == channels else np.ones(channels, dtype=np.float32)
            if self.mean.size not in {1, channels} or self.std.size not in {1, channels}:
                raise RuntimeError(
                    f"Event tensor normalization must contain 1 or {channels} values"
                )
            if self.mean.size == 1:
                mean = np.repeat(self.mean, channels)
            if self.std.size == 1:
                std = np.repeat(self.std, channels)
            normalized = (tensor - mean[:, None, None]) / std[:, None, None]
            return torch.from_numpy(np.ascontiguousarray(normalized, dtype=np.float32))
        image_path = self.dataset_dir / row["image_path"]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Failed to read image: {image_path}")
        return torch.from_numpy(self.transform(image))

    def _imu(self, row: dict[str, str]) -> torch.Tensor:
        try:
            values = json.loads(row.get("imu") or "[]")
        except json.JSONDecodeError:
            values = []
        array = np.zeros((self.imu_samples, self.imu_features), dtype=np.float32)
        if isinstance(values, list):
            for index, sample in enumerate(values[-self.imu_samples :]):
                if not isinstance(sample, list):
                    continue
                usable = min(len(sample), self.imu_features)
                array[self.imu_samples - min(len(values), self.imu_samples) + index, :usable] = np.asarray(
                    sample[:usable], dtype=np.float32
                )
        return torch.from_numpy(array)

    def _target(self, row: dict[str, str]) -> torch.Tensor:
        if self.task == "control":
            return torch.tensor(
                [float(row["steering"]), float(row["throttle"])], dtype=torch.float32
            )
        try:
            values = json.loads(row.get("trajectory") or "[]")
        except json.JSONDecodeError as error:
            raise RuntimeError("Invalid trajectory JSON in samples.csv") from error
        array = np.asarray(values, dtype=np.float32)
        if array.shape != (self.trajectory_points, 2):
            raise RuntimeError(
                f"Expected trajectory shape {(self.trajectory_points, 2)}, got {array.shape}"
            )
        return torch.from_numpy(np.clip(array / self.trajectory_scale_m, -1.0, 1.0))

    @staticmethod
    def _control(row: dict[str, str]) -> torch.Tensor:
        return torch.tensor(
            [float(row["steering"]), float(row["throttle"])], dtype=torch.float32
        )

    def _future_indices(self, index: int) -> tuple[list[int], list[bool]]:
        sequence_id = self.rows[index].get("sequence_id", "")
        indices: list[int] = []
        valid: list[bool] = []
        for offset in range(1, self.future_horizon + 1):
            candidate = index + offset * self.future_stride
            is_valid = (
                candidate < len(self.rows)
                and self.rows[candidate].get("sequence_id", "") == sequence_id
            )
            indices.append(candidate if is_valid else index)
            valid.append(is_valid)
        return indices, valid

    def __getitem__(self, index: int):
        row = self.rows[index]
        history_indices = self._sequence_indices(index)
        images = torch.stack([self._image(self.rows[item]) for item in history_indices])
        if self.future_horizon == 0:
            return images, self._imu(row), self._target(row)

        if self.task != "control":
            raise RuntimeError("future prediction currently requires a control dataset")
        future_indices, future_valid = self._future_indices(index)
        return {
            "images": images,
            "history_actions": torch.stack(
                [self._control(self.rows[item]) for item in history_indices]
            ),
            "control": self._control(row),
            "future_images": torch.stack(
                [self._image(self.rows[item]) for item in future_indices]
            ),
            "future_actions": torch.stack(
                [self._control(self.rows[item]) for item in future_indices]
            ),
            "future_mask": torch.tensor(future_valid, dtype=torch.float32),
        }


# Kept for external imports that used the first control-only dataset name.
ControlImageDataset = E2EDataset


class AsyncRgbEvsDataset(Dataset):
    """One RGB interval with a padded sequence of causal EVS updates."""

    def __init__(self, dataset_dir: str | Path, input_width: int, input_height: int,
                 mean: tuple[float, ...], std: tuple[float, ...], rollout_steps: int,
                 event_mean: tuple[float, ...], event_std: tuple[float, ...],
                 data_fraction: float = 1.0) -> None:
        self.dataset_dir = Path(dataset_dir)
        self.transform = ImageTransform(input_width, input_height, mean, std)
        self.rollout_steps = int(rollout_steps)
        self.event_mean = np.asarray(event_mean, dtype=np.float32)
        self.event_std = np.asarray(event_std, dtype=np.float32)
        # Adjacent intervals share their boundary RGB frame; leave a one-row
        # gap between temporal training and validation partitions.
        self.future_horizon = 1
        self.future_stride = 1
        with (self.dataset_dir / "samples.csv").open(newline="") as handle:
            self.rows = list(csv.DictReader(handle))
        keep = max(1, int(len(self.rows) * data_fraction))
        self.rows = self.rows[:keep]
        if not self.rows:
            raise RuntimeError("No asynchronous RGB-EVS samples were found")

    def __len__(self) -> int:
        return len(self.rows)

    def _rgb(self, relative: str) -> torch.Tensor:
        image = cv2.imread(str(self.dataset_dir / relative), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Failed to read RGB image: {relative}")
        return torch.from_numpy(self.transform(image))

    def _event(self, relative: str) -> torch.Tensor:
        loaded = np.load(self.dataset_dir / relative, allow_pickle=False)
        tensor = (loaded["tensor"] if isinstance(loaded, np.lib.npyio.NpzFile) else loaded).astype(np.float32)
        if isinstance(loaded, np.lib.npyio.NpzFile):
            loaded.close()
        if tensor.ndim != 3:
            raise RuntimeError(f"Event tensor must be CHW, got {tensor.shape}: {relative}")
        channels = tensor.shape[0]
        if self.event_mean.size not in {1, channels} or self.event_std.size not in {1, channels}:
            raise RuntimeError(
                f"Event normalization does not match {channels} channels: {relative}"
            )
        mean = self.event_mean if self.event_mean.size == channels else np.repeat(self.event_mean, channels)
        std = self.event_std if self.event_std.size == channels else np.repeat(self.event_std, channels)
        if np.any(std == 0):
            raise RuntimeError(f"Event normalization std contains zero: {relative}")
        return torch.from_numpy(np.ascontiguousarray((tensor - mean[:, None, None]) / std[:, None, None]))

    def __getitem__(self, index: int):
        row = self.rows[index]
        paths = json.loads(row["event_tensor_paths"])
        delta_t = [float(value) for value in json.loads(row["event_delta_t"])]
        controls = json.loads(row["event_controls"])
        tensors = [self._event(path) for path in paths[:self.rollout_steps]]
        if not tensors:
            raise RuntimeError("RGB-EVS interval has no event tensors")
        event_shape = tensors[0].shape
        mask = [1.0] * len(tensors)
        while len(tensors) < self.rollout_steps:
            tensors.append(torch.zeros(event_shape, dtype=torch.float32))
            delta_t.append(0.0)
            controls.append(controls[-1])
            mask.append(0.0)
        return {
            "rgb": self._rgb(row["image_path"]),
            "next_rgb": self._rgb(row["next_image_path"]),
            "events": torch.stack(tensors),
            "delta_t": torch.tensor(delta_t[:self.rollout_steps], dtype=torch.float32),
            "mask": torch.tensor(mask[:self.rollout_steps], dtype=torch.float32),
            "controls": torch.tensor(controls[:self.rollout_steps], dtype=torch.float32),
        }
