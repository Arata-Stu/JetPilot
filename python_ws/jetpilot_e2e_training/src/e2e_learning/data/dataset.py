from __future__ import annotations

import csv
import json
import random
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
                 data_fraction: float = 1.0, augment_timing: bool = False,
                 timing_min_hz: float = 100.0, timing_max_hz: float = 250.0,
                 timing_jitter_fraction: float = 0.15,
                 event_update_drop_probability: float = 0.05,
                 rgb_update_drop_probability: float = 0.10,
                 max_rgb_interval_multiplier: int = 2,
                 rgb_feature_cache_dir: str | Path | None = None) -> None:
        self.dataset_dir = Path(dataset_dir)
        self.transform = ImageTransform(input_width, input_height, mean, std)
        self.rollout_steps = int(rollout_steps)
        self.event_mean = np.asarray(event_mean, dtype=np.float32)
        self.event_std = np.asarray(event_std, dtype=np.float32)
        self.augment_timing = bool(augment_timing)
        self.timing_min_hz = float(timing_min_hz)
        self.timing_max_hz = float(timing_max_hz)
        self.timing_jitter_fraction = float(timing_jitter_fraction)
        self.event_update_drop_probability = float(event_update_drop_probability)
        self.rgb_update_drop_probability = float(rgb_update_drop_probability)
        self.max_rgb_interval_multiplier = int(max_rgb_interval_multiplier)
        self.rgb_feature_cache_dir = (
            Path(rgb_feature_cache_dir) if rgb_feature_cache_dir is not None else None
        )
        if self.rollout_steps < 1:
            raise ValueError("rollout_steps must be positive")
        if self.timing_min_hz <= 0.0 or self.timing_max_hz < self.timing_min_hz:
            raise ValueError("timing Hz limits must be positive and ordered")
        if not 0.0 <= self.timing_jitter_fraction < 1.0:
            raise ValueError("timing_jitter_fraction must be in [0, 1)")
        if not 0.0 <= self.event_update_drop_probability < 1.0:
            raise ValueError("event_update_drop_probability must be in [0, 1)")
        if not 0.0 <= self.rgb_update_drop_probability < 1.0:
            raise ValueError("rgb_update_drop_probability must be in [0, 1)")
        if self.max_rgb_interval_multiplier < 1:
            raise ValueError("max_rgb_interval_multiplier must be positive")
        # Adjacent intervals share boundary RGB frames. Leave enough gap that
        # RGB-drop augmentation cannot read from the validation partition.
        self.future_horizon = self.max_rgb_interval_multiplier
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
        if self.rgb_feature_cache_dir is not None:
            feature_path = (self.rgb_feature_cache_dir / relative).with_suffix(".npy")
            if not feature_path.is_file():
                raise RuntimeError(f"Cached RGB feature was not found: {feature_path}")
            feature = np.load(feature_path, allow_pickle=False).astype(np.float32, copy=False)
            if feature.ndim != 1:
                raise RuntimeError(
                    f"Cached RGB feature must be one-dimensional, got {feature.shape}: "
                    f"{feature_path}"
                )
            return torch.from_numpy(np.ascontiguousarray(feature))
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

    def _interval_rows(self, index: int) -> list[dict[str, str]]:
        rows = [self.rows[index]]
        if not self.augment_timing:
            return rows
        while (
            len(rows) < self.max_rgb_interval_multiplier
            and random.random() < self.rgb_update_drop_probability
        ):
            candidate_index = index + len(rows)
            if candidate_index >= len(self.rows):
                break
            candidate = self.rows[candidate_index]
            if candidate.get("sequence_id", "") != rows[0].get("sequence_id", ""):
                break
            if candidate.get("image_path") != rows[-1].get("next_image_path"):
                break
            rows.append(candidate)
        return rows

    @staticmethod
    def _row_sequence(row: dict[str, str]) -> tuple[list[str], list[int], list[float], list[list[float]]]:
        paths = [str(value) for value in json.loads(row["event_tensor_paths"])]
        deltas = [float(value) for value in json.loads(row["event_delta_t"])]
        controls = [[float(item[0]), float(item[1])] for item in json.loads(row["event_controls"])]
        # event_delta_t is derived from sensor-time representation boundaries;
        # reconstruct a jitter-free local timeline even when packet transport
        # timestamps fluctuate.
        stamp = int(row.get("stamp") or 0)
        stamps = []
        for delta in deltas:
            stamp += round(max(0.0, delta) * 1.0e9)
            stamps.append(stamp)
        count = min(len(paths), len(stamps), len(deltas), len(controls))
        return paths[:count], stamps[:count], deltas[:count], controls[:count]

    def _timing_indices(self, stamps: list[int], start_stamp: int) -> list[int]:
        if not self.augment_timing or len(stamps) <= 1:
            return list(range(len(stamps)))

        # Preserve dense 250 Hz examples in part of every epoch. Other samples
        # draw a rate in [100, 250] Hz and remain on real recorded boundaries.
        dense_sample = random.random() < 0.25
        if dense_sample:
            selected = list(range(len(stamps)))
            if len(selected) > self.rollout_steps:
                selected = selected[: self.rollout_steps - 1] + [selected[-1]]
            return selected
        target_hz = random.uniform(self.timing_min_hz, self.timing_max_hz)
        jitter = self.timing_jitter_fraction

        def next_period_ns() -> int:
            scale = random.uniform(1.0 - jitter, 1.0 + jitter)
            return max(1, round(1.0e9 * scale / target_hz))

        selected: list[int] = []
        next_due = start_stamp + next_period_ns()
        for source_index, stamp in enumerate(stamps):
            if stamp < next_due:
                continue
            next_due = stamp + next_period_ns()
            if random.random() < self.event_update_drop_probability:
                continue
            selected.append(source_index)
        # The endpoint must correspond to next_rgb for the latent teacher loss.
        if not selected or selected[-1] != len(stamps) - 1:
            selected.append(len(stamps) - 1)
        if len(selected) > self.rollout_steps:
            selected = selected[: self.rollout_steps - 1] + [selected[-1]]
        return selected

    def __getitem__(self, index: int):
        interval_rows = self._interval_rows(index)
        row = interval_rows[0]
        paths: list[str] = []
        stamps: list[int] = []
        controls: list[list[float]] = []
        for interval in interval_rows:
            interval_paths, interval_stamps, _, interval_controls = self._row_sequence(interval)
            paths.extend(interval_paths)
            stamps.extend(interval_stamps)
            controls.extend(interval_controls)
        indices = self._timing_indices(stamps, int(row.get("stamp") or 0))
        paths = [paths[item] for item in indices]
        stamps = [stamps[item] for item in indices]
        controls = [controls[item] for item in indices]
        previous_stamp = int(row.get("stamp") or 0)
        delta_t = []
        for stamp in stamps:
            delta_t.append(max(0.0, (stamp - previous_stamp) / 1.0e9))
            previous_stamp = stamp
        tensors = [self._event(path) for path in paths]
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
            "next_rgb": self._rgb(interval_rows[-1]["next_image_path"]),
            "events": torch.stack(tensors),
            "delta_t": torch.tensor(delta_t[:self.rollout_steps], dtype=torch.float32),
            "mask": torch.tensor(mask[:self.rollout_steps], dtype=torch.float32),
            "controls": torch.tensor(controls[:self.rollout_steps], dtype=torch.float32),
        }
