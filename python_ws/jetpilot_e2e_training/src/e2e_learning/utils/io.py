import csv
import json
from pathlib import Path
from typing import Iterable, Mapping

import yaml
from omegaconf import DictConfig, OmegaConf


def ensure_dir(path: str | Path) -> Path:
    directory = Path(path)
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def write_json(path: str | Path, payload: Mapping) -> None:
    path = Path(path)
    ensure_dir(path.parent)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def write_yaml(path: str | Path, payload: Mapping | DictConfig) -> None:
    path = Path(path)
    ensure_dir(path.parent)
    if isinstance(payload, DictConfig):
        text = OmegaConf.to_yaml(payload, resolve=True)
    else:
        text = yaml.safe_dump(dict(payload), sort_keys=False)
    path.write_text(text)


def read_json(path: str | Path) -> dict:
    return json.loads(Path(path).read_text())


def write_csv(path: str | Path, rows: Iterable[Mapping], fieldnames: list[str]) -> int:
    path = Path(path)
    ensure_dir(path.parent)
    count = 0
    with path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
            count += 1
    return count
