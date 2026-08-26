from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Mapping


EXPECTED_CLASSES = ("vehicle", "barrier")
NETWORK_WIDTH = 224
NETWORK_HEIGHT = 224
INPUT_BINDING_NAME = "images"
OUTPUT_BINDING_NAME = "output0"


def classes_from_dataset(dataset: Mapping[object, object]) -> list[str]:
    names = dataset.get("names")
    if isinstance(names, list):
        return [str(item) for item in names]
    if isinstance(names, Mapping):
        indexed: list[tuple[int, str]] = []
        for key, value in names.items():
            try:
                index = int(key)
            except (TypeError, ValueError) as error:
                raise ValueError(f"Dataset class id is not an integer: {key}") from error
            indexed.append((index, str(value)))
        indexed.sort()
        if [index for index, _ in indexed] != list(range(len(indexed))):
            raise ValueError("Dataset class ids must be contiguous and start at 0")
        return [name for _, name in indexed]
    raise ValueError("Dataset YAML must contain a names list or mapping")


def load_dataset(path: Path) -> tuple[Path, dict[object, object], list[str]]:
    try:
        import yaml
    except ImportError as error:
        raise RuntimeError("PyYAML is required to read the dataset contract") from error

    dataset_path = path.expanduser().resolve()
    if not dataset_path.is_file():
        raise FileNotFoundError(f"Dataset YAML was not found: {dataset_path}")
    loaded = yaml.safe_load(dataset_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError("Dataset YAML root must be a mapping")
    classes = classes_from_dataset(loaded)
    if tuple(classes) != EXPECTED_CLASSES:
        raise ValueError(
            "Dataset class order does not match the ROS decoder contract: "
            f"dataset={classes}, expected={list(EXPECTED_CLASSES)}"
        )
    return dataset_path, loaded, classes


def dataset_root(dataset_path: Path, dataset: Mapping[object, object]) -> Path:
    raw = str(dataset.get("path") or ".").strip()
    candidate = Path(raw).expanduser()
    if not candidate.is_absolute():
        candidate = dataset_path.parent / candidate
    return candidate.resolve()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
