#!/usr/bin/env python3
"""
Interactive local HD map editor for VSLAM landmark raster backgrounds.

The editor stores lane boundaries in world coordinates and can derive
centerlines from the left/right bounds. A selected primary lane is also exported
to the F1TENTH-style centerline CSV consumed by data_analysis/generate_raceline.py.
"""

from __future__ import annotations

import argparse
import json
import math
from ast import literal_eval
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    import cv2
    import numpy as np

    _IMPORT_ERROR = None
except ModuleNotFoundError as exc:
    cv2 = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]
    _IMPORT_ERROR = exc


PointPx = Tuple[int, int]
PointM = Tuple[float, float]

POLYLINE_FIELDS = ("centerline", "left_bound", "right_bound")
FIELD_LABELS = {
    "centerline": "centerline",
    "left_bound": "left bound",
    "right_bound": "right bound",
}
FIELD_COLORS = {
    "centerline": (0, 255, 255),
    "left_bound": (80, 220, 80),
    "right_bound": (230, 100, 230),
}
VSLAM_PATH_COLOR = (255, 96, 0)
VSLAM_PATH_THICKNESS_PX = 2
AUTO_CENTERLINE_SPACING_M = 0.10
AUTO_CENTERLINE_MIN_POINTS = 8
AUTO_CENTERLINE_MAX_POINTS = 2000
CURVE_ASSIST_SPACING_M = 0.10
CURVE_ASSIST_ITERATIONS = 1
CURVE_ASSIST_MAX_POINTS = 2000
HISTORY_LIMIT = 100


@dataclass
class RasterGeometry:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    map_yaml_path: Path
    image_path: Path

    def pixel_to_world(self, point: PointPx) -> PointM:
        grid_x = float(point[0]) * self.resolution
        grid_y = float((self.height - 1) - point[1]) * self.resolution
        cos_t = math.cos(self.origin_yaw)
        sin_t = math.sin(self.origin_yaw)
        return (
            self.origin_x + cos_t * grid_x - sin_t * grid_y,
            self.origin_y + sin_t * grid_x + cos_t * grid_y,
        )

    def world_to_pixel(self, point: Sequence[float]) -> PointPx:
        dx = float(point[0]) - self.origin_x
        dy = float(point[1]) - self.origin_y
        cos_t = math.cos(self.origin_yaw)
        sin_t = math.sin(self.origin_yaw)
        grid_x = (cos_t * dx + sin_t * dy) / self.resolution
        grid_y = (-sin_t * dx + cos_t * dy) / self.resolution
        u = int(round(grid_x))
        v = int(round((self.height - 1) - grid_y))
        return (
            max(0, min(self.width - 1, u)),
            max(0, min(self.height - 1, v)),
        )


@dataclass
class LaneDraft:
    lane_id: str
    closed_loop: bool = True
    centerline: List[PointPx] = field(default_factory=list)
    left_bound: List[PointPx] = field(default_factory=list)
    right_bound: List[PointPx] = field(default_factory=list)

    def points(self, field_name: str) -> List[PointPx]:
        return getattr(self, field_name)


EditorState = Tuple[List[LaneDraft], str, int, str, Dict[str, str]]


def _strip_yaml_scalar(value: object) -> object:
    if not isinstance(value, str):
        return value
    stripped = value.strip()
    if len(stripped) >= 2 and stripped[0] == stripped[-1] and stripped[0] in ("'", '"'):
        return stripped[1:-1]
    return stripped


def _parse_flat_yaml(path: Path) -> Dict[str, object]:
    data: Dict[str, object] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = _strip_yaml_scalar(value)
    return data


def load_yaml(path: Path, *, allow_flat_fallback: bool) -> Dict[str, Any]:
    try:
        import yaml  # type: ignore

        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file)
        if isinstance(data, dict):
            return data
    except Exception:
        pass

    try:
        from omegaconf import OmegaConf

        data = OmegaConf.to_container(OmegaConf.load(path), resolve=True)
        if isinstance(data, dict):
            return data
    except Exception:
        if not allow_flat_fallback:
            raise RuntimeError(
                f"Could not parse YAML {path}. Install PyYAML or omegaconf to load nested HD map YAML."
            )

    if allow_flat_fallback:
        return _parse_flat_yaml(path)
    raise RuntimeError(f"YAML root must be a mapping: {path}")


def parse_origin(value: object) -> Tuple[float, float, float]:
    if isinstance(value, str):
        value = literal_eval(value)
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) < 2:
        raise RuntimeError("map YAML origin must be [x, y, yaw]")
    yaw = float(value[2]) if len(value) >= 3 else 0.0
    return float(value[0]), float(value[1]), yaw


def resolve_image_path(map_yaml_path: Path, image_value: object) -> Path:
    raw = str(_strip_yaml_scalar(image_value) or "").strip()
    if not raw:
        raise RuntimeError(f"Map YAML has no image entry: {map_yaml_path}")
    image_path = Path(raw).expanduser()
    return image_path.resolve() if image_path.is_absolute() else (map_yaml_path.parent / image_path).resolve()


def load_raster_geometry(map_yaml_path: Path) -> Tuple[RasterGeometry, np.ndarray]:
    map_yaml_path = map_yaml_path.expanduser().resolve()
    data = load_yaml(map_yaml_path, allow_flat_fallback=True)
    if "resolution" not in data or "origin" not in data:
        raise RuntimeError(f"Map YAML needs resolution and origin: {map_yaml_path}")
    image_path = resolve_image_path(map_yaml_path, data.get("image"))
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"Could not read map image: {image_path}")
    origin_x, origin_y, origin_yaw = parse_origin(data["origin"])
    resolution = float(data["resolution"])
    if resolution <= 0.0:
        raise RuntimeError(f"Map YAML resolution must be positive: {map_yaml_path}")
    height, width = image.shape[:2]
    return (
        RasterGeometry(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
            map_yaml_path=map_yaml_path,
            image_path=image_path,
        ),
        image,
    )


def sanitize_lane_id(value: str, fallback: str) -> str:
    allowed = []
    for char in value.strip():
        if char.isalnum() or char in ("_", "-", "."):
            allowed.append(char)
    result = "".join(allowed)
    return result if result else fallback


def next_lane_id(lanes: Sequence[LaneDraft]) -> str:
    existing = {lane.lane_id for lane in lanes}
    index = 1
    while True:
        candidate = f"lane_{index:03d}"
        if candidate not in existing:
            return candidate
        index += 1


def _point_rows_to_pixels(rows: object, geometry: RasterGeometry) -> List[PointPx]:
    points: List[PointPx] = []
    if not isinstance(rows, Sequence) or isinstance(rows, (str, bytes)):
        return points
    for row in rows:
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)) or len(row) < 2:
            continue
        points.append(geometry.world_to_pixel((float(row[0]), float(row[1]))))
    return points


def load_hd_map_lanes(path: Path, geometry: RasterGeometry) -> Tuple[List[LaneDraft], str]:
    data = load_yaml(path, allow_flat_fallback=False)
    raw_lanes = data.get("lanes", [])
    if not isinstance(raw_lanes, Sequence) or isinstance(raw_lanes, (str, bytes)):
        raise RuntimeError(f"HD map lanes must be a list: {path}")

    lanes: List[LaneDraft] = []
    for index, raw_lane in enumerate(raw_lanes, start=1):
        if not isinstance(raw_lane, dict):
            continue
        lane_id = sanitize_lane_id(str(raw_lane.get("id", "")), f"lane_{index:03d}")
        lanes.append(
            LaneDraft(
                lane_id=lane_id,
                closed_loop=bool(raw_lane.get("closed_loop", True)),
                centerline=_point_rows_to_pixels(raw_lane.get("centerline", []), geometry),
                left_bound=_point_rows_to_pixels(raw_lane.get("left_bound", []), geometry),
                right_bound=_point_rows_to_pixels(raw_lane.get("right_bound", []), geometry),
            )
        )

    if not lanes:
        lanes = [LaneDraft(lane_id="lane_001")]
    primary_lane_id = sanitize_lane_id(str(data.get("primary_lane_id", "")), lanes[0].lane_id)
    if primary_lane_id not in {lane.lane_id for lane in lanes}:
        primary_lane_id = lanes[0].lane_id
    return lanes, primary_lane_id


def _fmt_float(value: float) -> str:
    normalized = 0.0 if abs(value) < 5.0e-13 else float(value)
    return f"{normalized:.9g}"


def _quote_yaml_string(value: str) -> str:
    return json.dumps(value, ensure_ascii=True)


def _append_world_polyline(
    lines: List[str],
    field_name: str,
    pixel_points: Sequence[PointPx],
    geometry: RasterGeometry,
) -> None:
    lines.append(f"    {field_name}:")
    if not pixel_points:
        lines[-1] = f"    {field_name}: []"
        return
    for point in pixel_points:
        x, y = geometry.pixel_to_world(point)
        lines.append(f"      - [{_fmt_float(x)}, {_fmt_float(y)}, 0.0]")


def write_hd_map_yaml(
    output_path: Path,
    geometry: RasterGeometry,
    lanes: Sequence[LaneDraft],
    primary_lane_id: str,
    centerline_csv_path: Optional[Path],
) -> None:
    lines = [
        "format: tamiya_local_hd_map_v1",
        "frame_id: map",
        "units: meter",
        f"primary_lane_id: {_quote_yaml_string(primary_lane_id)}",
        "source_raster:",
        f"  map_yaml: {_quote_yaml_string(str(geometry.map_yaml_path))}",
        f"  image: {_quote_yaml_string(str(geometry.image_path))}",
        f"  resolution_m_per_px: {_fmt_float(geometry.resolution)}",
        (
            "  origin_xy_yaw: "
            f"[{_fmt_float(geometry.origin_x)}, {_fmt_float(geometry.origin_y)}, {_fmt_float(geometry.origin_yaw)}]"
        ),
        f"  image_size_px: [{geometry.width}, {geometry.height}]",
    ]
    if centerline_csv_path is not None:
        lines.extend(
            [
                "exports:",
                f"  primary_centerline_csv: {_quote_yaml_string(str(centerline_csv_path))}",
            ]
        )
    lines.append("lanes:")

    for lane in lanes:
        lines.extend(
            [
                f"  - id: {_quote_yaml_string(lane.lane_id)}",
                f"    closed_loop: {'true' if lane.closed_loop else 'false'}",
            ]
        )
        _append_world_polyline(lines, "left_bound", lane.left_bound, geometry)
        _append_world_polyline(lines, "right_bound", lane.right_bound, geometry)
        _append_world_polyline(lines, "centerline", lane.centerline, geometry)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _world_xy(points: Sequence[PointPx], geometry: RasterGeometry) -> np.ndarray:
    if not points:
        return np.empty((0, 2), dtype=np.float64)
    return np.asarray([geometry.pixel_to_world(point) for point in points], dtype=np.float64)


def _clean_polyline_array(points: Sequence[PointPx], closed_loop: bool) -> np.ndarray:
    cleaned: List[Tuple[float, float]] = []
    for point in points:
        candidate = (float(point[0]), float(point[1]))
        if cleaned and math.hypot(candidate[0] - cleaned[-1][0], candidate[1] - cleaned[-1][1]) <= 1.0e-9:
            continue
        cleaned.append(candidate)

    if closed_loop and len(cleaned) >= 2:
        first = cleaned[0]
        last = cleaned[-1]
        if math.hypot(first[0] - last[0], first[1] - last[1]) <= 1.0e-9:
            cleaned.pop()

    if not cleaned:
        return np.empty((0, 2), dtype=np.float64)
    return np.asarray(cleaned, dtype=np.float64)


def _polyline_total_length_px(points: np.ndarray, closed_loop: bool) -> float:
    if len(points) < 2:
        return 0.0
    if closed_loop and len(points) >= 3:
        deltas = np.roll(points, -1, axis=0) - points
    else:
        deltas = np.diff(points, axis=0)
    return float(np.sum(np.linalg.norm(deltas, axis=1)))


def _sample_polyline_array(points: np.ndarray, fractions: np.ndarray, closed_loop: bool) -> np.ndarray:
    if len(points) == 0:
        return np.empty((len(fractions), 2), dtype=np.float64)
    if len(points) == 1:
        return np.repeat(points[:1], len(fractions), axis=0)

    if closed_loop and len(points) >= 3:
        starts = points
        ends = np.roll(points, -1, axis=0)
        segment_lengths = np.linalg.norm(ends - starts, axis=1)
        total_length = float(np.sum(segment_lengths))
        if total_length <= 1.0e-9:
            return np.repeat(points[:1], len(fractions), axis=0)
        distances = np.mod(fractions, 1.0) * total_length
    else:
        starts = points[:-1]
        ends = points[1:]
        segment_lengths = np.linalg.norm(ends - starts, axis=1)
        total_length = float(np.sum(segment_lengths))
        if total_length <= 1.0e-9:
            return np.repeat(points[:1], len(fractions), axis=0)
        distances = np.clip(fractions, 0.0, 1.0) * total_length

    cumulative = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    segment_indices = np.searchsorted(cumulative, distances, side="right") - 1
    segment_indices = np.clip(segment_indices, 0, len(segment_lengths) - 1)
    local_distances = distances - cumulative[segment_indices]
    selected_lengths = segment_lengths[segment_indices]
    ratios = np.divide(
        local_distances,
        selected_lengths,
        out=np.zeros_like(local_distances),
        where=selected_lengths > 1.0e-9,
    )
    return starts[segment_indices] + (ends[segment_indices] - starts[segment_indices]) * ratios[:, None]


def _best_closed_bound_alignment(left: np.ndarray, right: np.ndarray) -> Tuple[np.ndarray, float]:
    probe_count = min(256, max(32, len(left), len(right)))
    fractions = np.arange(probe_count, dtype=np.float64) / float(probe_count)
    left_probe = _sample_polyline_array(left, fractions, closed_loop=True)

    best_score = float("inf")
    best_right = right
    best_offset = 0.0
    for candidate in (right, right[::-1].copy()):
        right_probe = _sample_polyline_array(candidate, fractions, closed_loop=True)
        for shift in range(probe_count):
            shifted = np.roll(right_probe, -shift, axis=0)
            delta = left_probe - shifted
            score = float(np.mean(np.sum(delta * delta, axis=1)))
            if score < best_score:
                best_score = score
                best_right = candidate
                best_offset = float(shift) / float(probe_count)
    return best_right, best_offset


def _best_open_bound_alignment(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    probe_count = min(256, max(16, len(left), len(right)))
    fractions = np.linspace(0.0, 1.0, probe_count, dtype=np.float64)
    left_probe = _sample_polyline_array(left, fractions, closed_loop=False)

    best_score = float("inf")
    best_right = right
    for candidate in (right, right[::-1].copy()):
        right_probe = _sample_polyline_array(candidate, fractions, closed_loop=False)
        delta = left_probe - right_probe
        score = float(np.mean(np.sum(delta * delta, axis=1)))
        if score < best_score:
            best_score = score
            best_right = candidate
    return best_right


def _centerline_sample_count(
    left: np.ndarray,
    right: np.ndarray,
    geometry: RasterGeometry,
    closed_loop: bool,
    spacing_m: float,
) -> int:
    left_length_px = _polyline_total_length_px(left, closed_loop)
    right_length_px = _polyline_total_length_px(right, closed_loop)
    average_length_m = 0.5 * (left_length_px + right_length_px) * geometry.resolution
    minimum = AUTO_CENTERLINE_MIN_POINTS if closed_loop else 2
    sample_count = max(minimum, len(left), len(right))
    if spacing_m > 1.0e-9 and average_length_m > 1.0e-9:
        spacing_count = int(math.ceil(average_length_m / spacing_m))
        if not closed_loop:
            spacing_count += 1
        sample_count = max(sample_count, spacing_count)
    return max(2, min(AUTO_CENTERLINE_MAX_POINTS, sample_count))


def _polyline_pixels_from_samples(samples: np.ndarray, geometry: RasterGeometry, closed_loop: bool) -> List[PointPx]:
    points: List[PointPx] = []
    for sample in samples:
        point = (
            max(0, min(geometry.width - 1, int(round(float(sample[0]))))),
            max(0, min(geometry.height - 1, int(round(float(sample[1]))))),
        )
        if points and points[-1] == point:
            continue
        points.append(point)

    if closed_loop and len(points) >= 2 and points[0] == points[-1]:
        points.pop()
    return points


def _centerline_pixels_from_samples(samples: np.ndarray, geometry: RasterGeometry, closed_loop: bool) -> List[PointPx]:
    return _polyline_pixels_from_samples(samples, geometry, closed_loop)


def _chaikin_smooth(points: np.ndarray, closed_loop: bool) -> np.ndarray:
    if len(points) < 2:
        return points.copy()

    if closed_loop and len(points) >= 3:
        smoothed: List[np.ndarray] = []
        for index, point in enumerate(points):
            next_point = points[(index + 1) % len(points)]
            smoothed.append(0.75 * point + 0.25 * next_point)
            smoothed.append(0.25 * point + 0.75 * next_point)
        return np.asarray(smoothed, dtype=np.float64)

    smoothed = [points[0]]
    for index in range(len(points) - 1):
        point = points[index]
        next_point = points[index + 1]
        smoothed.append(0.75 * point + 0.25 * next_point)
        smoothed.append(0.25 * point + 0.75 * next_point)
    smoothed.append(points[-1])
    return np.asarray(smoothed, dtype=np.float64)


def _curve_assist_sample_count(
    points: np.ndarray,
    geometry: RasterGeometry,
    closed_loop: bool,
    spacing_m: float,
) -> int:
    length_m = _polyline_total_length_px(points, closed_loop) * geometry.resolution
    minimum = 3 if closed_loop else 2
    sample_count = max(minimum, len(points))
    if spacing_m > 1.0e-9 and length_m > 1.0e-9:
        spacing_count = int(math.ceil(length_m / spacing_m))
        if not closed_loop:
            spacing_count += 1
        sample_count = max(sample_count, spacing_count)
    return max(minimum, min(CURVE_ASSIST_MAX_POINTS, sample_count))


def smooth_polyline_points(
    points: Sequence[PointPx],
    geometry: RasterGeometry,
    closed_loop: bool,
    iterations: int = CURVE_ASSIST_ITERATIONS,
    spacing_m: float = CURVE_ASSIST_SPACING_M,
) -> List[PointPx]:
    cleaned = _clean_polyline_array(points, closed_loop)
    required = 3 if closed_loop else 2
    if len(cleaned) < required:
        raise RuntimeError(f"curve assist needs at least {required} points")

    smoothed = cleaned
    for _ in range(max(0, int(iterations))):
        smoothed = _chaikin_smooth(smoothed, closed_loop)

    sample_count = _curve_assist_sample_count(smoothed, geometry, closed_loop, max(0.0, float(spacing_m)))
    if closed_loop:
        fractions = np.arange(sample_count, dtype=np.float64) / float(sample_count)
    else:
        fractions = np.linspace(0.0, 1.0, sample_count, dtype=np.float64)

    samples = _sample_polyline_array(smoothed, fractions, closed_loop)
    result = _polyline_pixels_from_samples(samples, geometry, closed_loop)
    if len(result) < required:
        raise RuntimeError(f"curve assist generated fewer than {required} points")
    return result


def generate_centerline_from_bounds(
    lane: LaneDraft,
    geometry: RasterGeometry,
    spacing_m: float = AUTO_CENTERLINE_SPACING_M,
) -> List[PointPx]:
    left = _clean_polyline_array(lane.left_bound, lane.closed_loop)
    right = _clean_polyline_array(lane.right_bound, lane.closed_loop)
    required = 3 if lane.closed_loop else 2
    if len(left) < required:
        raise RuntimeError(f"left bound needs at least {required} points")
    if len(right) < required:
        raise RuntimeError(f"right bound needs at least {required} points")

    if lane.closed_loop:
        aligned_right, right_offset = _best_closed_bound_alignment(left, right)
        sample_count = _centerline_sample_count(left, aligned_right, geometry, True, spacing_m)
        fractions = np.arange(sample_count, dtype=np.float64) / float(sample_count)
        left_samples = _sample_polyline_array(left, fractions, closed_loop=True)
        right_samples = _sample_polyline_array(aligned_right, fractions + right_offset, closed_loop=True)
    else:
        aligned_right = _best_open_bound_alignment(left, right)
        sample_count = _centerline_sample_count(left, aligned_right, geometry, False, spacing_m)
        fractions = np.linspace(0.0, 1.0, sample_count, dtype=np.float64)
        left_samples = _sample_polyline_array(left, fractions, closed_loop=False)
        right_samples = _sample_polyline_array(aligned_right, fractions, closed_loop=False)

    center_samples = 0.5 * (left_samples + right_samples)
    centerline = _centerline_pixels_from_samples(center_samples, geometry, lane.closed_loop)
    minimum_centerline_points = AUTO_CENTERLINE_MIN_POINTS if lane.closed_loop else 2
    if len(centerline) < minimum_centerline_points:
        raise RuntimeError(f"generated centerline needs at least {minimum_centerline_points} points")
    return centerline


def update_auto_centerline(
    lane: LaneDraft,
    geometry: RasterGeometry,
    spacing_m: float = AUTO_CENTERLINE_SPACING_M,
) -> Optional[str]:
    try:
        lane.centerline = generate_centerline_from_bounds(lane, geometry, spacing_m)
    except RuntimeError as exc:
        return str(exc)
    return None


def _nearest_distances(points: np.ndarray, polyline: np.ndarray, closed_loop: bool) -> np.ndarray:
    if len(polyline) == 0:
        return np.zeros(len(points), dtype=np.float64)
    if len(polyline) == 1:
        deltas = points - polyline[0]
        return np.sqrt(np.sum(deltas * deltas, axis=1))

    starts = polyline if closed_loop else polyline[:-1]
    ends = np.roll(polyline, -1, axis=0) if closed_loop else polyline[1:]
    segments = ends - starts
    segment_len_sq = np.sum(segments * segments, axis=1)
    segment_len_sq[segment_len_sq < 1.0e-12] = 1.0

    rel = points[:, None, :] - starts[None, :, :]
    t = np.sum(rel * segments[None, :, :], axis=2) / segment_len_sq[None, :]
    t = np.clip(t, 0.0, 1.0)
    closest = starts[None, :, :] + t[:, :, None] * segments[None, :, :]
    deltas = points[:, None, :] - closest
    return np.sqrt(np.min(np.sum(deltas * deltas, axis=2), axis=1))


def lane_export_issue(lane: LaneDraft) -> Optional[str]:
    bound_points = 3 if lane.closed_loop else 2
    if len(lane.left_bound) < bound_points:
        return f"left bound needs at least {bound_points} points"
    if len(lane.right_bound) < bound_points:
        return f"right bound needs at least {bound_points} points"
    centerline_points = 3 if lane.closed_loop else 2
    if len(lane.centerline) < centerline_points:
        return f"centerline needs at least {centerline_points} points"
    return None


def export_centerline_csv(output_path: Path, lane: LaneDraft, geometry: RasterGeometry) -> None:
    issue = lane_export_issue(lane)
    if issue is not None:
        raise RuntimeError(f"Lane {lane.lane_id} cannot export centerline CSV: {issue}.")

    centerline = _world_xy(lane.centerline, geometry)
    left_bound = _world_xy(lane.left_bound, geometry)
    right_bound = _world_xy(lane.right_bound, geometry)
    rows = np.column_stack(
        [
            centerline[:, 0],
            centerline[:, 1],
            _nearest_distances(centerline, right_bound, lane.closed_loop),
            _nearest_distances(centerline, left_bound, lane.closed_loop),
        ]
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("wb") as file:
        np.savetxt(
            file,
            rows,
            fmt="%.6f",
            delimiter=",",
            header="x_m,y_m,w_tr_right_m,w_tr_left_m",
        )


def _find_alignment_params(data: object) -> Dict[str, object]:
    if not isinstance(data, dict):
        return {}
    ros_params = data.get("ros__parameters")
    if isinstance(ros_params, dict):
        found = _find_alignment_params(ros_params)
        if found:
            return found
    if any(key in data for key in ("x", "y", "z", "roll_rad", "pitch_rad", "yaw_rad")):
        return data
    for value in data.values():
        found = _find_alignment_params(value)
        if found:
            return found
    return {}


def _rotation_translation_from_alignment(alignment_path: Optional[Path]) -> Tuple[np.ndarray, np.ndarray]:
    params: Dict[str, object] = {}
    if alignment_path is not None:
        params = _find_alignment_params(load_yaml(alignment_path, allow_flat_fallback=True))

    tx = float(params.get("x", 0.0) or 0.0)
    ty = float(params.get("y", 0.0) or 0.0)
    tz = float(params.get("z", 0.0) or 0.0)
    roll = float(params.get("roll_rad", 0.0) or 0.0)
    pitch = float(params.get("pitch_rad", 0.0) or 0.0)
    yaw = float(params.get("yaw_rad", 0.0) or 0.0)

    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    return rotation, np.array([tx, ty, tz], dtype=np.float64)


def _transform_points(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points
    transformed = points @ rotation.T
    transformed[:, 0] += translation[0]
    transformed[:, 1] += translation[1]
    transformed[:, 2] += translation[2]
    return transformed


def _snapshot_path_points(snapshot: Dict[str, Any]) -> np.ndarray:
    for key in ("full_vslam_path", "path"):
        path_data = snapshot.get(key)
        if not isinstance(path_data, dict):
            continue
        poses = path_data.get("poses", [])
        if not isinstance(poses, Sequence) or isinstance(poses, (str, bytes)):
            continue
        points: List[List[float]] = []
        for pose in poses:
            if not isinstance(pose, dict):
                continue
            position = pose.get("position")
            if not isinstance(position, dict):
                nested_pose = pose.get("pose")
                if isinstance(nested_pose, dict):
                    position = nested_pose.get("position")
            if not isinstance(position, dict):
                continue
            try:
                points.append(
                    [
                        float(position.get("x", 0.0)),
                        float(position.get("y", 0.0)),
                        float(position.get("z", 0.0)),
                    ]
                )
            except (TypeError, ValueError):
                continue
        if points:
            return np.asarray(points, dtype=np.float64)
    return np.empty((0, 3), dtype=np.float64)


def _world_xy_to_pixels(points_xy: np.ndarray, geometry: RasterGeometry) -> List[PointPx]:
    if points_xy.size == 0:
        return []
    dx = points_xy[:, 0] - geometry.origin_x
    dy = points_xy[:, 1] - geometry.origin_y
    cos_t = math.cos(geometry.origin_yaw)
    sin_t = math.sin(geometry.origin_yaw)
    grid_x = (cos_t * dx + sin_t * dy) / geometry.resolution
    grid_y = (-sin_t * dx + cos_t * dy) / geometry.resolution
    pixels = np.column_stack(
        [
            np.round(grid_x).astype(np.int32),
            np.round((geometry.height - 1) - grid_y).astype(np.int32),
        ]
    )
    mask = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] < geometry.width)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < geometry.height)
    )
    return [(int(px), int(py)) for px, py in pixels[mask]]


def load_vslam_path_pixels(
    snapshot_path: Path,
    alignment_path: Optional[Path],
    geometry: RasterGeometry,
) -> List[PointPx]:
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    path_points = _snapshot_path_points(snapshot)
    if path_points.size == 0:
        return []
    rotation, translation = _rotation_translation_from_alignment(alignment_path)
    path_points = _transform_points(path_points, rotation, translation)
    return _world_xy_to_pixels(path_points[:, :2], geometry)


class HdMapEditor:
    def __init__(
        self,
        background: np.ndarray,
        geometry: RasterGeometry,
        output_path: Path,
        centerline_output_path: Path,
        lanes: Sequence[LaneDraft],
        primary_lane_id: str,
        window_width: int,
        window_height: int,
        scale: float,
        vslam_path_points: Sequence[PointPx] = (),
        show_vslam_path: bool = True,
        show_centerlines: bool = False,
        auto_centerline: bool = True,
        centerline_spacing_m: float = AUTO_CENTERLINE_SPACING_M,
        curve_assist_iterations: int = CURVE_ASSIST_ITERATIONS,
        curve_assist_spacing_m: float = CURVE_ASSIST_SPACING_M,
    ) -> None:
        self.background = background.copy()
        self.geometry = geometry
        self.output_path = output_path
        self.centerline_output_path = centerline_output_path
        self.lanes = list(lanes) if lanes else [LaneDraft(lane_id="lane_001")]
        self.primary_lane_id = primary_lane_id
        self.active_lane_index = 0
        self.auto_centerline = bool(auto_centerline)
        self.centerline_spacing_m = max(0.0, float(centerline_spacing_m))
        self.curve_assist_iterations = max(0, int(curve_assist_iterations))
        self.curve_assist_spacing_m = max(0.0, float(curve_assist_spacing_m))
        self.auto_centerline_issues: Dict[str, str] = {}
        self.active_field = "left_bound" if self.auto_centerline else "centerline"
        self.undo_stack: List[EditorState] = []
        self.redo_stack: List[EditorState] = []

        self.window_name = "Local HD Map Editor"
        self.window_width = max(480, int(window_width))
        self.window_height = max(320, int(window_height))
        self.min_scale = 0.1
        self.max_scale = 24.0
        self.scale = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        self.is_panning = False
        self.pan_start_mouse = (0, 0)
        self.pan_start_offset = (0, 0)
        self.dragging_index: Optional[int] = None
        self.dragging_original_state: Optional[EditorState] = None
        self.has_unsaved_changes = False
        self.show_help = True
        self.vslam_path_points = list(vslam_path_points)
        self.show_vslam_path = bool(show_vslam_path and len(self.vslam_path_points) >= 2)
        self.show_centerlines = bool(show_centerlines or not self.auto_centerline)

        if scale > 0.0:
            self.scale = max(self.min_scale, min(self.max_scale, float(scale)))
            self._center_view()
        else:
            self._reset_view()

        if self.auto_centerline:
            updated_count, failed_count = self._regenerate_all_auto_centerlines(mark_dirty=False)
            if updated_count > 0:
                self.has_unsaved_changes = True
                print(f"[INFO] Auto-updated centerline for {updated_count} lane(s). Press s to save YAML.")
            if failed_count > 0:
                print(f"[WARN] Auto centerline is pending for {failed_count} lane(s). Draw left/right bounds first.")

    @property
    def active_lane(self) -> LaneDraft:
        return self.lanes[self.active_lane_index]

    @property
    def active_points(self) -> List[PointPx]:
        return self.active_lane.points(self.active_field)

    def _capture_state(self) -> EditorState:
        return (
            deepcopy(self.lanes),
            self.primary_lane_id,
            self.active_lane_index,
            self.active_field,
            deepcopy(self.auto_centerline_issues),
        )

    def _remember_state(self, state: Optional[EditorState] = None) -> None:
        self.undo_stack.append(state if state is not None else self._capture_state())
        if len(self.undo_stack) > HISTORY_LIMIT:
            self.undo_stack.pop(0)
        self.redo_stack.clear()

    def _restore_state(self, state: EditorState) -> None:
        lanes, primary_lane_id, active_lane_index, active_field, auto_centerline_issues = state
        self.lanes = deepcopy(lanes)
        self.primary_lane_id = primary_lane_id
        self.active_lane_index = max(0, min(active_lane_index, len(self.lanes) - 1))
        self.active_field = active_field
        self.auto_centerline_issues = deepcopy(auto_centerline_issues)
        self.dragging_index = None
        self.dragging_original_state = None
        self.has_unsaved_changes = True

    def _undo_edit(self) -> None:
        if not self.undo_stack:
            print("[INFO] Nothing to undo.")
            return
        self.redo_stack.append(self._capture_state())
        if len(self.redo_stack) > HISTORY_LIMIT:
            self.redo_stack.pop(0)
        self._restore_state(self.undo_stack.pop())
        print("[INFO] Undid the last edit.")

    def _redo_edit(self) -> None:
        if not self.redo_stack:
            print("[INFO] Nothing to redo.")
            return
        self.undo_stack.append(self._capture_state())
        if len(self.undo_stack) > HISTORY_LIMIT:
            self.undo_stack.pop(0)
        self._restore_state(self.redo_stack.pop())
        print("[INFO] Redid the last edit.")

    def _try_update_auto_centerline(self, lane: LaneDraft, mark_dirty: bool) -> bool:
        if not self.auto_centerline:
            return False

        previous = list(lane.centerline)
        issue = update_auto_centerline(lane, self.geometry, self.centerline_spacing_m)
        if issue is not None:
            self.auto_centerline_issues[lane.lane_id] = issue
            if lane.centerline:
                lane.centerline = []
                if mark_dirty:
                    self.has_unsaved_changes = True
                return True
            return False

        self.auto_centerline_issues.pop(lane.lane_id, None)
        changed = lane.centerline != previous
        if changed and mark_dirty:
            self.has_unsaved_changes = True
        return changed

    def _regenerate_all_auto_centerlines(self, mark_dirty: bool) -> Tuple[int, int]:
        if not self.auto_centerline:
            return 0, 0

        updated_count = 0
        failed_count = 0
        for lane in self.lanes:
            changed = self._try_update_auto_centerline(lane, mark_dirty=mark_dirty)
            if changed:
                updated_count += 1
            if lane.lane_id in self.auto_centerline_issues:
                failed_count += 1
        return updated_count, failed_count

    def _refresh_active_auto_centerline(self) -> None:
        if self.auto_centerline and self.active_field in ("left_bound", "right_bound"):
            self._try_update_auto_centerline(self.active_lane, mark_dirty=True)

    def _scaled_size(self) -> Tuple[int, int]:
        return (
            max(1, int(round(self.geometry.width * self.scale))),
            max(1, int(round(self.geometry.height * self.scale))),
        )

    def _clamp_pan(self) -> None:
        scaled_width, scaled_height = self._scaled_size()
        self.pan_x = max(0, min(self.pan_x, max(0, scaled_width - self.window_width)))
        self.pan_y = max(0, min(self.pan_y, max(0, scaled_height - self.window_height)))

    def _reset_view(self) -> None:
        self.scale = max(
            self.min_scale,
            min(
                self.max_scale,
                min(
                    self.window_width / float(max(1, self.geometry.width)),
                    self.window_height / float(max(1, self.geometry.height)),
                ),
            ),
        )
        self.pan_x = 0
        self.pan_y = 0

    def _center_view(self) -> None:
        scaled_width, scaled_height = self._scaled_size()
        self.pan_x = max(0, int(round((scaled_width - self.window_width) / 2.0)))
        self.pan_y = max(0, int(round((scaled_height - self.window_height) / 2.0)))
        self._clamp_pan()

    def _zoom_at(self, factor: float, x: int, y: int) -> None:
        old_scale = self.scale
        new_scale = max(self.min_scale, min(self.max_scale, old_scale * factor))
        if abs(new_scale - old_scale) < 1.0e-9:
            return
        u = (self.pan_x + x) / old_scale
        v = (self.pan_y + y) / old_scale
        self.scale = new_scale
        self.pan_x = int(round(u * new_scale - x))
        self.pan_y = int(round(v * new_scale - y))
        self._clamp_pan()

    def _to_map_pixel(self, x: int, y: int) -> PointPx:
        u = int(round((self.pan_x + x) / self.scale))
        v = int(round((self.pan_y + y) / self.scale))
        return (
            max(0, min(self.geometry.width - 1, u)),
            max(0, min(self.geometry.height - 1, v)),
        )

    def _inside_map(self, x: int, y: int) -> bool:
        scaled_width, scaled_height = self._scaled_size()
        sx = self.pan_x + x
        sy = self.pan_y + y
        return 0 <= sx < scaled_width and 0 <= sy < scaled_height

    def _nearest_active_point_index(self, point: PointPx) -> Optional[int]:
        if not self.active_points:
            return None
        threshold = max(0.5, 5.0 / max(self.scale, 1.0e-6))
        best_index: Optional[int] = None
        best_distance = float("inf")
        for index, candidate in enumerate(self.active_points):
            distance = math.hypot(candidate[0] - point[0], candidate[1] - point[1])
            if distance <= threshold and distance < best_distance:
                best_index = index
                best_distance = distance
        return best_index

    def _draw_text(self, frame: np.ndarray, text: str, origin: PointPx, scale: float, color: Tuple[int, int, int]) -> None:
        cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)

    def _draw_panel(self, frame: np.ndarray, height: int) -> None:
        overlay = frame.copy()
        cv2.rectangle(overlay, (8, 8), (min(self.window_width - 8, 1050), min(self.window_height - 8, height)), (18, 18, 18), -1)
        cv2.addWeighted(overlay, 0.74, frame, 0.26, 0.0, dst=frame)
        cv2.rectangle(frame, (8, 8), (min(self.window_width - 8, 1050), min(self.window_height - 8, height)), (220, 220, 220), 1, cv2.LINE_AA)

    def _draw_polyline(
        self,
        canvas: np.ndarray,
        points: Sequence[PointPx],
        color: Tuple[int, int, int],
        closed: bool,
        point_radius: int,
        thickness: int,
        highlight: bool = False,
    ) -> None:
        clean_points: List[PointPx] = []
        for point in points:
            if len(point) < 2:
                continue
            x = float(point[0])
            y = float(point[1])
            if not math.isfinite(x) or not math.isfinite(y):
                continue
            clean_points.append(
                (
                    max(0, min(self.geometry.width - 1, int(round(x)))),
                    max(0, min(self.geometry.height - 1, int(round(y)))),
                )
            )
        if not clean_points:
            return
        pts = np.asarray(clean_points, dtype=np.int32).reshape((-1, 1, 2))
        if highlight:
            cv2.polylines(
                canvas,
                [pts],
                bool(closed and len(clean_points) >= 3),
                (0, 0, 0),
                thickness + 4,
                cv2.LINE_AA,
            )
        cv2.polylines(canvas, [pts], bool(closed and len(clean_points) >= 3), color, thickness, cv2.LINE_AA)
        if point_radius > 0:
            for point in clean_points:
                if highlight:
                    cv2.circle(canvas, point, point_radius + 2, (0, 0, 0), -1, cv2.LINE_AA)
                cv2.circle(canvas, point, point_radius, color, -1, cv2.LINE_AA)

    def _draw_vslam_path(self, canvas: np.ndarray) -> None:
        if not self.show_vslam_path or len(self.vslam_path_points) < 2:
            return
        pts = np.asarray(self.vslam_path_points, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(
            canvas,
            [pts],
            isClosed=False,
            color=VSLAM_PATH_COLOR,
            thickness=max(1, VSLAM_PATH_THICKNESS_PX),
            lineType=cv2.LINE_AA,
        )

    def _vslam_path_control_text(self) -> str:
        if len(self.vslam_path_points) < 2:
            return "path:none"
        return f"v:path {'on' if self.show_vslam_path else 'off'}"

    def _centerline_control_text(self) -> str:
        return f"m:center {'on' if self.show_centerlines else 'off'}"

    def _draw_map(self) -> np.ndarray:
        canvas = self.background.copy()
        shade = np.full_like(canvas, 235)
        canvas = cv2.addWeighted(canvas, 0.82, shade, 0.18, 0.0)
        self._draw_vslam_path(canvas)

        for lane_index, lane in enumerate(self.lanes):
            active_lane = lane_index == self.active_lane_index
            for field_name in POLYLINE_FIELDS:
                if field_name == "centerline" and not self.show_centerlines:
                    continue
                base_color = FIELD_COLORS[field_name]
                if active_lane:
                    color = base_color
                    if self.auto_centerline and field_name == "centerline":
                        radius = 0
                        thickness = 3
                    else:
                        radius = 4 if field_name == self.active_field else 2
                        thickness = 3 if field_name == self.active_field else 2
                else:
                    color = tuple(int(channel * 0.55) for channel in base_color)
                    radius = 0
                    thickness = 1
                self._draw_polyline(
                    canvas,
                    lane.points(field_name),
                    color,
                    lane.closed_loop,
                    radius,
                    thickness,
                    highlight=active_lane and field_name == "centerline",
                )

        scaled_width, scaled_height = self._scaled_size()
        interpolation = cv2.INTER_NEAREST if self.scale >= 1.0 else cv2.INTER_AREA
        scaled = cv2.resize(canvas, (scaled_width, scaled_height), interpolation=interpolation)
        self._clamp_pan()
        cropped = scaled[
            self.pan_y : min(scaled_height, self.pan_y + self.window_height),
            self.pan_x : min(scaled_width, self.pan_x + self.window_width),
        ]
        frame = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8)
        frame[: cropped.shape[0], : cropped.shape[1]] = cropped
        self._draw_hud(frame)
        return frame

    def _draw_hud(self, frame: np.ndarray) -> None:
        if not self.show_help:
            self._draw_panel(frame, 72)
            centerline_mode = "auto-center" if self.auto_centerline else "manual-center"
            self._draw_text(
                frame,
                (
                    f"{self.active_lane.lane_id}  {FIELD_LABELS[self.active_field]}  "
                    f"{centerline_mode}  {self._centerline_control_text()}  "
                    f"{self._vslam_path_control_text()}  i:help  s:save"
                ),
                (22, 48),
                0.75,
                (255, 255, 255),
            )
            return

        cursor_px = self._to_map_pixel(self.last_mouse_x, self.last_mouse_y)
        cursor_m = self.geometry.pixel_to_world(cursor_px)
        issue = lane_export_issue(self._lane_by_id(self.primary_lane_id))
        export_state = "ready" if issue is None else issue
        auto_issue = self.auto_centerline_issues.get(self.primary_lane_id)
        if self.auto_centerline and auto_issue is not None:
            export_state = f"auto centerline: {auto_issue}"
            issue = auto_issue
        dirty = "unsaved" if self.has_unsaved_changes else "saved"
        loop_state = "closed" if self.active_lane.closed_loop else "open"
        centerline_mode = "auto-center" if self.auto_centerline else "manual-center"
        primary_suffix = " primary" if self.active_lane.lane_id == self.primary_lane_id else ""
        self._draw_panel(frame, 220)
        self._draw_text(
            frame,
            (
                f"HD map: {self.active_lane.lane_id}{primary_suffix} "
                f"({self.active_lane_index + 1}/{len(self.lanes)}, {loop_state}, {centerline_mode}, {dirty})"
            ),
            (22, 46),
            0.78,
            (255, 255, 255),
        )
        self._draw_text(
            frame,
            (
                f"Editing {FIELD_LABELS[self.active_field]}: {len(self.active_points)} points   "
                f"zoom {self.scale:.2f}x   map ({cursor_m[0]:.3f}, {cursor_m[1]:.3f}) m"
            ),
            (22, 80),
            0.66,
            FIELD_COLORS[self.active_field],
        )
        edit_help = (
            "L-click:add/drag  d:delete point  u:remove last  2:left  3:right  c:smooth  a:regen"
            if self.auto_centerline
            else "L-click:add/drag  d:delete point  u:remove last  1:center  2:left  3:right  c:smooth"
        )
        self._draw_text(
            frame,
            edit_help,
            (22, 114),
            0.62,
            (235, 235, 235),
        )
        self._draw_text(
            frame,
            "n:new lane  x:delete lane  [/]:switch  p:primary  o:closed/open  z/y:undo/redo  s:save",
            (22, 146),
            0.62,
            (235, 235, 235),
        )
        self._draw_text(
            frame,
            (
                "Wheel +/-:zoom  Right-drag/HJKL:pan  0:fit  "
                f"{self._centerline_control_text()}  {self._vslam_path_control_text()}  "
                "i:help  q/Esc:quit"
            ),
            (22, 178),
            0.62,
            (235, 235, 235),
        )
        self._draw_text(
            frame,
            f"Primary CSV export: {export_state}",
            (22, 208),
            0.60,
            (130, 235, 130) if issue is None else (120, 190, 255),
        )

    def _mouse_callback(self, event: int, x: int, y: int, flags: int, _userdata: object) -> None:
        self.last_mouse_x = max(0, min(self.window_width - 1, int(x)))
        self.last_mouse_y = max(0, min(self.window_height - 1, int(y)))

        if event == cv2.EVENT_RBUTTONDOWN:
            self.is_panning = True
            self.pan_start_mouse = (x, y)
            self.pan_start_offset = (self.pan_x, self.pan_y)
            return
        if event == cv2.EVENT_RBUTTONUP:
            self.is_panning = False
            return
        if event == cv2.EVENT_MOUSEWHEEL:
            self._zoom_at(1.15 if flags > 0 else 1.0 / 1.15, x, y)
            return
        if event == cv2.EVENT_MOUSEMOVE and self.is_panning:
            self.pan_x = self.pan_start_offset[0] - (x - self.pan_start_mouse[0])
            self.pan_y = self.pan_start_offset[1] - (y - self.pan_start_mouse[1])
            self._clamp_pan()
            return

        if event == cv2.EVENT_LBUTTONDOWN and self._inside_map(x, y):
            point = self._to_map_pixel(x, y)
            near_index = self._nearest_active_point_index(point)
            if near_index is not None:
                self.dragging_index = near_index
                self.dragging_original_state = self._capture_state()
                return
            self._remember_state()
            self.active_points.append(point)
            self.has_unsaved_changes = True
            self.dragging_index = len(self.active_points) - 1
            return

        if event == cv2.EVENT_MOUSEMOVE and self.dragging_index is not None and self._inside_map(x, y):
            point = self._to_map_pixel(x, y)
            if point == self.active_points[self.dragging_index]:
                return
            if self.dragging_original_state is not None:
                self._remember_state(self.dragging_original_state)
                self.dragging_original_state = None
            self.active_points[self.dragging_index] = point
            self.has_unsaved_changes = True
            return

        if event == cv2.EVENT_LBUTTONUP:
            was_dragging = self.dragging_index is not None
            self.dragging_index = None
            self.dragging_original_state = None
            if was_dragging:
                self._refresh_active_auto_centerline()

    def _lane_by_id(self, lane_id: str) -> LaneDraft:
        for lane in self.lanes:
            if lane.lane_id == lane_id:
                return lane
        return self.lanes[0]

    def _set_field(self, field_name: str) -> None:
        if field_name in POLYLINE_FIELDS:
            if self.auto_centerline and field_name == "centerline":
                print("[INFO] Centerline is auto-generated from left/right bounds. Edit left/right bounds instead.")
                return
            self.active_field = field_name

    def _delete_point(self) -> None:
        if not self.active_points:
            print("[INFO] Active polyline has no points.")
            return
        self._remember_state()
        cursor = self._to_map_pixel(self.last_mouse_x, self.last_mouse_y)
        index = self._nearest_active_point_index(cursor)
        if index is None:
            index = len(self.active_points) - 1
        removed = self.active_points.pop(index)
        self.has_unsaved_changes = True
        self._refresh_active_auto_centerline()
        print(f"[INFO] Removed {FIELD_LABELS[self.active_field]} point {removed}.")

    def _smooth_active_polyline(self) -> None:
        if self.auto_centerline and self.active_field == "centerline":
            print("[INFO] Centerline is auto-generated from left/right bounds. Smooth the bounds instead.")
            return
        try:
            smoothed = smooth_polyline_points(
                self.active_points,
                self.geometry,
                self.active_lane.closed_loop,
                self.curve_assist_iterations,
                self.curve_assist_spacing_m,
            )
        except RuntimeError as exc:
            print(f"[WARN] Curve assist skipped {FIELD_LABELS[self.active_field]}: {exc}.")
            return

        if smoothed == self.active_points:
            print(f"[INFO] Curve assist made no change to {FIELD_LABELS[self.active_field]}.")
            return

        self._remember_state()
        old_count = len(self.active_points)
        setattr(self.active_lane, self.active_field, smoothed)
        self.has_unsaved_changes = True
        self._refresh_active_auto_centerline()
        print(
            f"[INFO] Smoothed {FIELD_LABELS[self.active_field]} "
            f"from {old_count} to {len(smoothed)} points."
        )

    def _new_lane(self) -> None:
        self._remember_state()
        lane = LaneDraft(lane_id=next_lane_id(self.lanes))
        self.lanes.append(lane)
        self.active_lane_index = len(self.lanes) - 1
        self.active_field = "left_bound" if self.auto_centerline else "centerline"
        self.has_unsaved_changes = True
        print(f"[INFO] Added lane {lane.lane_id}.")

    def _delete_active_lane(self) -> None:
        self._remember_state()
        removed_lane = self.active_lane
        self.auto_centerline_issues.pop(removed_lane.lane_id, None)

        if len(self.lanes) == 1:
            self.lanes[0] = LaneDraft(lane_id=removed_lane.lane_id)
            self.active_lane_index = 0
            print(f"[INFO] Cleared the only lane {removed_lane.lane_id}.")
        else:
            self.lanes.pop(self.active_lane_index)
            self.active_lane_index = min(self.active_lane_index, len(self.lanes) - 1)
            print(f"[INFO] Deleted lane {removed_lane.lane_id}.")

        if removed_lane.lane_id == self.primary_lane_id:
            self.primary_lane_id = self.active_lane.lane_id
            print(f"[INFO] Primary lane: {self.primary_lane_id}.")
        self.active_field = "left_bound" if self.auto_centerline else "centerline"
        self.has_unsaved_changes = True

    def _switch_lane(self, delta: int) -> None:
        self.active_lane_index = (self.active_lane_index + delta) % len(self.lanes)
        print(f"[INFO] Active lane: {self.active_lane.lane_id}.")

    def _save(self) -> None:
        updated_count, failed_count = self._regenerate_all_auto_centerlines(mark_dirty=False)
        if updated_count > 0:
            print(f"[INFO] Auto-updated centerline for {updated_count} lane(s).")
        if failed_count > 0:
            print(f"[WARN] Auto centerline is pending for {failed_count} lane(s). Draw left/right bounds first.")
        write_hd_map_yaml(
            output_path=self.output_path,
            geometry=self.geometry,
            lanes=self.lanes,
            primary_lane_id=self.primary_lane_id,
            centerline_csv_path=self.centerline_output_path,
        )
        print(f"[INFO] Saved HD map YAML: {self.output_path}")
        primary_lane = self._lane_by_id(self.primary_lane_id)
        issue = self.auto_centerline_issues.get(primary_lane.lane_id) if self.auto_centerline else None
        if issue is None:
            issue = lane_export_issue(primary_lane)
        if issue is None:
            export_centerline_csv(self.centerline_output_path, primary_lane, self.geometry)
            print(f"[INFO] Exported primary centerline CSV: {self.centerline_output_path}")
        else:
            if self.centerline_output_path.exists():
                self.centerline_output_path.unlink()
                print(f"[INFO] Removed stale centerline CSV: {self.centerline_output_path}")
            print(
                f"[WARN] Saved YAML without centerline CSV for primary lane "
                f"{primary_lane.lane_id}: {issue}. Select the intended lane and press p if needed."
            )
        self.has_unsaved_changes = False

    def _pan_by_key(self, dx: int, dy: int) -> None:
        self.pan_x += dx
        self.pan_y += dy
        self._clamp_pan()

    def _handle_key(self, key: int) -> bool:
        low = key & 0xFF
        if low in (27, ord("q")):
            if self.has_unsaved_changes:
                print("[WARN] Quit with unsaved HD map edits.")
            return False
        if low == ord("1"):
            self._set_field("centerline")
        elif low == ord("2"):
            self._set_field("left_bound")
        elif low == ord("3"):
            self._set_field("right_bound")
        elif low == ord("a"):
            previous_state = self._capture_state()
            updated_count, failed_count = self._regenerate_all_auto_centerlines(mark_dirty=True)
            if updated_count > 0:
                self._remember_state(previous_state)
            if self.auto_centerline:
                print(f"[INFO] Auto centerline updated: updated={updated_count}, pending={failed_count}.")
            else:
                print("[INFO] Auto centerline is disabled in manual-centerline mode.")
        elif low == ord("c"):
            self._smooth_active_polyline()
        elif low == ord("n"):
            self._new_lane()
        elif low == ord("x"):
            self._delete_active_lane()
        elif low in (ord("["), ord(",")):
            self._switch_lane(-1)
        elif low in (ord("]"), ord(".")):
            self._switch_lane(1)
        elif low == ord("p"):
            if self.primary_lane_id != self.active_lane.lane_id:
                self._remember_state()
                self.primary_lane_id = self.active_lane.lane_id
                self.has_unsaved_changes = True
                print(f"[INFO] Primary lane: {self.primary_lane_id}.")
        elif low == ord("o"):
            self._remember_state()
            self.active_lane.closed_loop = not self.active_lane.closed_loop
            self.has_unsaved_changes = True
            self._try_update_auto_centerline(self.active_lane, mark_dirty=True)
        elif low == ord("s"):
            self._save()
        elif low in (ord("d"), 8, 127):
            self._delete_point()
        elif low == ord("u"):
            if self.active_points:
                self._remember_state()
                self.active_points.pop()
                self.has_unsaved_changes = True
                self._refresh_active_auto_centerline()
            else:
                print("[INFO] Active polyline has no points.")
        elif low == ord("z"):
            self._undo_edit()
        elif low == ord("y"):
            self._redo_edit()
        elif low == ord("i"):
            self.show_help = not self.show_help
        elif low == ord("v"):
            if len(self.vslam_path_points) >= 2:
                self.show_vslam_path = not self.show_vslam_path
                print(f"[INFO] VSLAM path overlay: {'on' if self.show_vslam_path else 'off'}.")
            else:
                print("[INFO] No VSLAM path overlay was loaded.")
        elif low == ord("m"):
            self.show_centerlines = not self.show_centerlines
            print(f"[INFO] Centerline overlay: {'on' if self.show_centerlines else 'off'}.")
        elif low == ord("0"):
            self._reset_view()
        elif low in (ord("+"), ord("=")):
            self._zoom_at(1.15, self.window_width // 2, self.window_height // 2)
        elif low in (ord("-"), ord("_")):
            self._zoom_at(1.0 / 1.15, self.window_width // 2, self.window_height // 2)
        elif low in (ord("h"), 81):
            self._pan_by_key(-60, 0)
        elif low in (ord("l"), 83):
            self._pan_by_key(60, 0)
        elif low in (ord("k"), 82):
            self._pan_by_key(0, -60)
        elif low in (ord("j"), 84):
            self._pan_by_key(0, 60)
        return True

    def run(self) -> None:
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)
        print("[INFO] HD map editor started. Press i for help and s to save.")
        keep_running = True
        while keep_running:
            try:
                frame = self._draw_map()
            except Exception as exc:
                frame = self.background.copy()
                frame = cv2.resize(frame, (self.window_width, self.window_height), interpolation=cv2.INTER_AREA)
                self._draw_panel(frame, 86)
                self._draw_text(frame, f"Draw failed: {exc}", (22, 48), 0.64, (120, 190, 255))
                print(f"[WARN] Draw failed: {exc}")
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKeyEx(20)
            if key >= 0:
                keep_running = self._handle_key(key)
        cv2.destroyWindow(self.window_name)


def _default_centerline_output(output_path: Path) -> Path:
    return output_path.with_name(f"{output_path.stem}_centerline.csv")


def load_or_create_lanes(output_path: Path, geometry: RasterGeometry) -> Tuple[List[LaneDraft], str]:
    if output_path.exists():
        lanes, primary_lane_id = load_hd_map_lanes(output_path, geometry)
        print(f"[INFO] Loaded HD map YAML: {output_path}")
        return lanes, primary_lane_id
    return [LaneDraft(lane_id="lane_001")], "lane_001"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Draw local HD map lane bounds on a map YAML raster and derive centerlines."
    )
    parser.add_argument("--map-yaml", required=True, help="Landmark raster or occupancy map YAML used as the editor background.")
    parser.add_argument("--output", required=True, help="Editable HD map YAML output path.")
    parser.add_argument(
        "--centerline-output",
        default="",
        help="Primary lane centerline CSV. Default: <output_stem>_centerline.csv.",
    )
    parser.add_argument("--window-width", type=int, default=1600)
    parser.add_argument("--window-height", type=int, default=1000)
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Initial image scale. Default 1.0 opens at native raster zoom; 0 fits the whole raster.",
    )
    parser.add_argument(
        "--export-only",
        action="store_true",
        help="Reload the HD map YAML and export the primary centerline CSV without opening the GUI.",
    )
    parser.add_argument(
        "--manual-centerline",
        action="store_true",
        help="Disable automatic centerline generation and edit centerline points by hand.",
    )
    parser.add_argument(
        "--show-centerlines",
        action="store_true",
        help="Start with generated centerline overlays visible. Press m to toggle them.",
    )
    parser.add_argument(
        "--auto-centerline-spacing",
        type=float,
        default=AUTO_CENTERLINE_SPACING_M,
        help=(
            "Approximate spacing for generated centerline points in meters. "
            f"Default: {AUTO_CENTERLINE_SPACING_M:.2f}."
        ),
    )
    parser.add_argument(
        "--curve-assist-iterations",
        type=int,
        default=CURVE_ASSIST_ITERATIONS,
        help=(
            "Chaikin smoothing iterations applied when pressing c. "
            f"Default: {CURVE_ASSIST_ITERATIONS}."
        ),
    )
    parser.add_argument(
        "--curve-assist-spacing",
        type=float,
        default=CURVE_ASSIST_SPACING_M,
        help=(
            "Approximate point spacing after curve assist in meters. "
            f"Default: {CURVE_ASSIST_SPACING_M:.2f}."
        ),
    )
    parser.add_argument(
        "--vslam-snapshot",
        default="",
        help="Optional VSLAM reference snapshot JSON used for a toggleable path overlay.",
    )
    parser.add_argument(
        "--vslam-alignment",
        default="",
        help="Optional map alignment YAML applied to the VSLAM snapshot path.",
    )
    parser.add_argument(
        "--hide-vslam-path",
        action="store_true",
        help="Start with the VSLAM path overlay hidden when --vslam-snapshot is provided.",
    )
    return parser


def main() -> int:
    if _IMPORT_ERROR is not None:
        raise SystemExit(f"hd_map_editor.py requires numpy and opencv-python: {_IMPORT_ERROR}")

    args = build_arg_parser().parse_args()
    output_path = Path(args.output).expanduser().resolve()
    centerline_output_path = (
        Path(args.centerline_output).expanduser().resolve()
        if args.centerline_output
        else _default_centerline_output(output_path)
    )
    geometry, background = load_raster_geometry(Path(args.map_yaml))
    lanes, primary_lane_id = load_or_create_lanes(output_path, geometry)
    auto_centerline = not args.manual_centerline
    centerline_spacing_m = max(0.0, float(args.auto_centerline_spacing))
    curve_assist_iterations = max(0, int(args.curve_assist_iterations))
    curve_assist_spacing_m = max(0.0, float(args.curve_assist_spacing))
    vslam_path_points: List[PointPx] = []
    if args.vslam_snapshot:
        snapshot_path = Path(args.vslam_snapshot).expanduser().resolve()
        alignment_path = Path(args.vslam_alignment).expanduser().resolve() if args.vslam_alignment else None
        try:
            vslam_path_points = load_vslam_path_pixels(snapshot_path, alignment_path, geometry)
            if vslam_path_points:
                print(f"[INFO] Loaded VSLAM path overlay: {len(vslam_path_points)} points.")
            else:
                print(f"[WARN] VSLAM snapshot had no path points: {snapshot_path}")
        except Exception as exc:
            print(f"[WARN] Could not load VSLAM path overlay from {snapshot_path}: {exc}")

    if args.export_only:
        primary_lane = next((lane for lane in lanes if lane.lane_id == primary_lane_id), lanes[0])
        if auto_centerline:
            issue = update_auto_centerline(primary_lane, geometry, centerline_spacing_m)
            if issue is not None:
                raise RuntimeError(f"Lane {primary_lane.lane_id} cannot auto-generate centerline: {issue}.")
            print(f"[INFO] Auto-generated primary centerline from bounds: {primary_lane.lane_id}")
        export_centerline_csv(centerline_output_path, primary_lane, geometry)
        print(f"[INFO] Exported primary centerline CSV: {centerline_output_path}")
        return 0

    editor = HdMapEditor(
        background=background,
        geometry=geometry,
        output_path=output_path,
        centerline_output_path=centerline_output_path,
        lanes=lanes,
        primary_lane_id=primary_lane_id,
        window_width=args.window_width,
        window_height=args.window_height,
        scale=args.scale,
        vslam_path_points=vslam_path_points,
        show_vslam_path=not args.hide_vslam_path,
        show_centerlines=args.show_centerlines,
        auto_centerline=auto_centerline,
        centerline_spacing_m=centerline_spacing_m,
        curve_assist_iterations=curve_assist_iterations,
        curve_assist_spacing_m=curve_assist_spacing_m,
    )
    editor.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
