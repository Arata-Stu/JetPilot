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
        mean: tuple[float, float, float],
        std: tuple[float, float, float],
        task: str = "control",
        sequence_length: int = 1,
        frame_stride: int = 1,
        trajectory_points: int = 10,
        trajectory_scale_m: float = 5.0,
        imu_samples: int = 10,
        imu_features: int = 7,
        data_fraction: float = 1.0,
    ) -> None:
        self.dataset_dir = Path(dataset_dir)
        self.samples_path = self.dataset_dir / "samples.csv"
        self.transform = ImageTransform(input_width, input_height, mean, std)
        self.task = str(task)
        self.sequence_length = int(sequence_length)
        self.frame_stride = int(frame_stride)
        self.trajectory_points = int(trajectory_points)
        self.trajectory_scale_m = float(trajectory_scale_m)
        self.imu_samples = int(imu_samples)
        self.imu_features = int(imu_features)
        if self.task not in {"control", "trajectory"}:
            raise ValueError("task must be control or trajectory")
        if self.sequence_length < 1 or self.frame_stride < 1:
            raise ValueError("sequence_length and frame_stride must be positive")
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

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        row = self.rows[index]
        images = torch.stack([self._image(self.rows[item]) for item in self._sequence_indices(index)])
        return images, self._imu(row), self._target(row)


# Kept for external imports that used the first control-only dataset name.
ControlImageDataset = E2EDataset
