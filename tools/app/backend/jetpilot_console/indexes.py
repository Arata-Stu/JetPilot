from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable


def _dir_size(path: Path) -> int:
    total = 0
    if not path.exists():
        return total
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += (Path(root) / name).stat().st_size
            except OSError:
                continue
    return total


def _iso_mtime(path: Path) -> float:
    try:
        return path.stat().st_mtime
    except OSError:
        return 0.0


def scan_rosbags(record_root: Path) -> list[dict[str, object]]:
    if not record_root.exists():
        return []

    bags: list[dict[str, object]] = []
    for metadata in sorted(record_root.rglob("metadata.yaml")):
        bag_dir = metadata.parent
        topic_count = 0
        try:
            text = metadata.read_text(encoding="utf-8", errors="replace")
            topic_count = text.count("topic_metadata:")
        except OSError:
            text = ""

        bags.append(
            {
                "name": bag_dir.name,
                "path": str(bag_dir),
                "metadata_path": str(metadata),
                "size_bytes": _dir_size(bag_dir),
                "modified_at": _iso_mtime(bag_dir),
                "topic_count_hint": topic_count,
            }
        )
    return bags


def _artifact(path: Path) -> dict[str, object]:
    exists = path.exists()
    return {
        "path": str(path),
        "exists": exists,
        "size_bytes": _dir_size(path) if exists and path.is_dir() else (path.stat().st_size if exists else 0),
        "modified_at": _iso_mtime(path) if exists else 0.0,
    }


def _looks_like_map_dir(path: Path) -> bool:
    if (path / "cuvgl_map").exists() or (path / "cuvslam_map").exists():
        return True
    if (path / "vslam_reference_snapshot.json").exists():
        return True
    if (path / "vslam_landmarks.yaml").exists() or (path / "vslam_landmarks.png").exists():
        return True
    return any(path.glob("*_hd_map.yaml")) or any(path.glob("*_raceline.csv"))


def _candidate_map_dirs(map_root: Path, max_depth: int = 3) -> Iterable[Path]:
    if not map_root.exists():
        return []
    candidates: list[Path] = []
    for root, dirs, _ in os.walk(map_root):
        path = Path(root)
        try:
            depth = len(path.relative_to(map_root).parts)
        except ValueError:
            depth = 0
        dirs[:] = [name for name in dirs if not name.startswith(".")]
        if depth >= max_depth:
            dirs[:] = []
        if depth > 0 and _looks_like_map_dir(path):
            candidates.append(path)
    return sorted(candidates)


def scan_maps(map_root: Path) -> list[dict[str, object]]:
    maps: list[dict[str, object]] = []
    for map_dir in _candidate_map_dirs(map_root):
        name = map_dir.name
        artifacts = {
            "cuvgl_map": _artifact(map_dir / "cuvgl_map"),
            "cuvslam_map": _artifact(map_dir / "cuvslam_map"),
            "snapshot": _artifact(map_dir / "vslam_reference_snapshot.json"),
            "landmark_yaml": _artifact(map_dir / "vslam_landmarks.yaml"),
            "landmark_image": _artifact(map_dir / "vslam_landmarks.png"),
            "hd_map": _artifact(map_dir / f"{name}_hd_map.yaml"),
            "centerline_csv": _artifact(map_dir / f"{name}_hd_map_centerline.csv"),
            "raceline_csv": _artifact(map_dir / f"{name}_raceline.csv"),
            "line_preview": _artifact(map_dir / f"{name}_line_preview.png"),
        }
        complete_runtime = all(
            artifacts[key]["exists"]
            for key in ("cuvgl_map", "cuvslam_map", "hd_map", "raceline_csv")
        )
        maps.append(
            {
                "name": name,
                "path": str(map_dir),
                "size_bytes": _dir_size(map_dir),
                "modified_at": _iso_mtime(map_dir),
                "artifacts": artifacts,
                "complete_runtime_bundle": complete_runtime,
            }
        )
    return maps
