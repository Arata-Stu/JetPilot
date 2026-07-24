import csv
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

from e2e_learning.data.transforms import ImageTransform


class ControlImageDataset(Dataset):
    def __init__(
        self,
        dataset_dir: str | Path,
        input_width: int,
        input_height: int,
        mean: tuple[float, float, float],
        std: tuple[float, float, float],
        data_fraction: float = 1.0,
    ) -> None:
        self.dataset_dir = Path(dataset_dir)
        self.samples_path = self.dataset_dir / "samples.csv"
        self.transform = ImageTransform(input_width, input_height, mean, std)
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

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        row = self.rows[index]
        image_path = self.dataset_dir / row["image_path"]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Failed to read image: {image_path}")
        x = torch.from_numpy(self.transform(image))
        y = torch.tensor(
            [float(row["steering"]), float(row["throttle"])],
            dtype=torch.float32,
        )
        return x, y
