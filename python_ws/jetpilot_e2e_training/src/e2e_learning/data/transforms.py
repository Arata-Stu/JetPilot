from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class ImageTransform:
    width: int
    height: int
    mean: tuple[float, float, float] = (0.485, 0.456, 0.406)
    std: tuple[float, float, float] = (0.229, 0.224, 0.225)

    def __call__(self, image_bgr: np.ndarray) -> np.ndarray:
        image = cv2.resize(image_bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        mean = np.asarray(self.mean, dtype=np.float32).reshape(1, 1, 3)
        std = np.asarray(self.std, dtype=np.float32).reshape(1, 1, 3)
        image = (image - mean) / std
        return np.transpose(image, (2, 0, 1)).astype(np.float32)
