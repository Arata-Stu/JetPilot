from __future__ import annotations

import os
import json
from pathlib import Path
from typing import Iterable


ROSBAG_METADATA_FILENAME = ".jetpilot_console.json"
ROSBAG_TRASH_DIRNAME = ".jetpilot_trash"


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


def _read_rosbag_metadata(bag_dir: Path) -> dict[str, object]:
    metadata_path = bag_dir / ROSBAG_METADATA_FILENAME
    try:
        value = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _write_rosbag_metadata(bag_dir: Path, metadata: dict[str, object]) -> None:
    path = bag_dir / ROSBAG_METADATA_FILENAME
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(metadata, indent=2, ensure_ascii=True), encoding="utf-8")
    tmp.replace(path)


def update_rosbag_metadata(bag_dir: Path, **updates: object) -> dict[str, object]:
    metadata = _read_rosbag_metadata(bag_dir)
    for key, value in updates.items():
        if value is None:
            metadata.pop(key, None)
        else:
            metadata[key] = value
    _write_rosbag_metadata(bag_dir, metadata)
    return metadata


def _rosbag_record(record_root: Path, metadata: Path, *, trashed: bool = False) -> dict[str, object]:
    bag_dir = metadata.parent
    topic_count = 0
    try:
        text = metadata.read_text(encoding="utf-8", errors="replace")
        topic_count = text.count("topic_metadata:")
    except OSError:
        text = ""
    console_metadata = _read_rosbag_metadata(bag_dir)
    display_name = str(console_metadata.get("display_name") or bag_dir.name)
    return {
        "name": bag_dir.name,
        "display_name": display_name,
        "path": str(bag_dir),
        "metadata_path": str(metadata),
        "size_bytes": _dir_size(bag_dir),
        "modified_at": _iso_mtime(bag_dir),
        "topic_count_hint": topic_count,
        "favorite": bool(console_metadata.get("favorite")),
        "trashed": trashed,
        "original_path": str(console_metadata.get("original_path") or ""),
        "trashed_at": str(console_metadata.get("trashed_at") or ""),
    }


def scan_rosbags(record_root: Path, *, include_trash: bool = False) -> list[dict[str, object]]:
    if not record_root.exists():
        return []

    bags: list[dict[str, object]] = []
    for metadata in sorted(record_root.rglob("metadata.yaml")):
        try:
            relative_parts = metadata.relative_to(record_root).parts
        except ValueError:
            continue
        if not include_trash and relative_parts[:1] == (ROSBAG_TRASH_DIRNAME,):
            continue
        if any(part.startswith(".") for part in relative_parts):
            continue
        bags.append(_rosbag_record(record_root, metadata))
    return bags


def scan_trashed_rosbags(record_root: Path) -> list[dict[str, object]]:
    trash_root = record_root / ROSBAG_TRASH_DIRNAME
    if not trash_root.exists():
        return []

    bags: list[dict[str, object]] = []
    for metadata in sorted(trash_root.rglob("metadata.yaml")):
        try:
            relative_parts = metadata.relative_to(trash_root).parts
        except ValueError:
            continue
        if any(part.startswith(".") for part in relative_parts):
            continue
        bags.append(_rosbag_record(record_root, metadata, trashed=True))
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
    return (
        any(path.glob("*_hd_map.yaml"))
        or any(path.glob("*_raceline.csv"))
        or any(path.glob("*_custom_line.csv"))
    )


def _map_dir_score(path: Path) -> int:
    score = 0
    if (path / "cuvgl_map").exists():
        score += 10
    if (path / "cuvslam_map").exists():
        score += 10
    if (path / "vslam_reference_snapshot.json").exists():
        score += 5
    if (path / "vslam_landmarks.yaml").exists():
        score += 4
    if (path / "vslam_landmarks.png").exists():
        score += 4
    if any(path.glob("*_hd_map.yaml")):
        score += 20
    if any(path.glob("*_hd_map_centerline.csv")):
        score += 10
    if any(path.glob("*_raceline.csv")):
        score += 10
    if any(path.glob("*_custom_line.csv")):
        score += 10
    if any(path.glob("*_line_preview.png")):
        score += 5
    return score


def _display_name(map_root: Path, map_dir: Path) -> str:
    try:
        parts = map_dir.relative_to(map_root).parts
    except ValueError:
        parts = ()
    return parts[0] if len(parts) > 1 else map_dir.name


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


def _collapse_map_dirs(map_root: Path, candidates: Iterable[Path]) -> list[Path]:
    grouped: dict[Path, list[Path]] = {}
    for candidate in candidates:
        try:
            relative = candidate.relative_to(map_root)
        except ValueError:
            grouped.setdefault(candidate, []).append(candidate)
            continue
        top = map_root / relative.parts[0] if relative.parts else candidate
        grouped.setdefault(top, []).append(candidate)

    collapsed = []
    for group in grouped.values():
        collapsed.append(
            max(
                group,
                key=lambda path: (
                    _map_dir_score(path),
                    _iso_mtime(path),
                    len(path.parts),
                ),
            )
        )
    return sorted(collapsed, key=lambda path: (_display_name(map_root, path), str(path)))


def scan_maps(map_root: Path) -> list[dict[str, object]]:
    # Imported lazily because map_detail reuses the low-level artifact helpers
    # in this module. The summary only reads HD-map junction metadata and the
    # optional competition route config; it does not build the full map detail.
    from .map_detail import competition_route_summary

    maps: list[dict[str, object]] = []
    for map_dir in _collapse_map_dirs(map_root, _candidate_map_dirs(map_root)):
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
            "custom_line_csv": _artifact(map_dir / f"{name}_custom_line.csv"),
            "custom_line_meta": _artifact(map_dir / f"{name}_custom_line.meta.json"),
            "line_preview": _artifact(map_dir / f"{name}_line_preview.png"),
        }
        localization_ready = all(
            artifacts[key]["exists"] for key in ("cuvgl_map", "cuvslam_map", "hd_map")
        )
        custom_line_ready = bool(
            artifacts["custom_line_csv"]["exists"] and artifacts["custom_line_meta"]["exists"]
        )
        driving_line_ready = bool(artifacts["raceline_csv"]["exists"] or custom_line_ready)
        competition_routes = competition_route_summary(map_dir)
        competition_route_status = str(competition_routes["status"])
        competition_routes_ready = bool(competition_routes["ready"])
        complete_runtime = localization_ready and driving_line_ready and competition_routes_ready
        maps.append(
            {
                "name": name,
                "display_name": _display_name(map_root, map_dir),
                "path": str(map_dir),
                "size_bytes": _dir_size(map_dir),
                "modified_at": _iso_mtime(map_dir),
                "artifacts": artifacts,
                "competition_route_status": competition_route_status,
                "competition_routes_ready": competition_routes_ready,
                "complete_runtime_bundle": complete_runtime,
            }
        )
    return maps
