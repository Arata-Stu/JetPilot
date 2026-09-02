from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import shutil
import struct
import tempfile
from ast import literal_eval
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.parse import quote

from .config import ConsoleConfig
from .indexes import _artifact, _dir_size, _iso_mtime


HD_MAP_VERSION_DIR = "hd_map_versions"
HD_MAP_VERSION_ACTIVE_FILE = "active.json"
CUSTOM_LINE_DIR = "custom_lines"
CUSTOM_LINE_ACTIVE_FILE = "active.json"
CUSTOM_LINE_MANIFEST_FILE = "custom_line.json"
CUSTOM_LINE_TRAJECTORY_FILE = "trajectory.csv"
CUSTOM_LINE_FORMAT = "jetpilot_custom_line_v1"
CUSTOM_LINE_DEFAULT_SPEED_MPS = 1.0
CUSTOM_LINE_MIN_SECTION_SPEED_MPS = 0.1
CUSTOM_LINE_POINT_EPSILON_M = 1.0e-9
CUSTOM_LINE_MAX_POINTS = 20_000
CUSTOM_LINE_SPEED_SAMPLE_STEP_M = 0.10
CUSTOM_LINE_MAX_COMPILED_POINTS = 100_000
CUSTOM_LINE_CONTAINMENT_SAMPLE_STEP_M = 0.05
CUSTOM_LINE_MAX_CONTAINMENT_SAMPLES = 100_000
CUSTOM_LINE_DEFAULT_MAX_SPEED_MPS = 3.0
CUSTOM_LINE_DEFAULT_LATERAL_ACCEL_LIMIT_MPS2 = 2.5
CUSTOM_LINE_DEFAULT_ACCEL_LIMIT_MPS2 = 1.5
CUSTOM_LINE_DEFAULT_DECEL_LIMIT_MPS2 = 2.5
COMPETITION_ROUTE_CONFIG_FILE = "competition_route.param.yaml"


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def resolve_allowed_path(config: ConsoleConfig, value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = config.map_root / path
    resolved = path.resolve()
    allowed_roots = [
        config.map_root.resolve(),
    ]
    if not any(_is_relative_to(resolved, root) for root in allowed_roots):
        raise ValueError("path is outside the JetPilot workspace")
    return resolved


def _file_url(path: Path) -> str:
    return f"/api/files?path={quote(str(path))}"


def _strip_comment(line: str) -> str:
    in_quote = ""
    for index, char in enumerate(line):
        if char in ("'", '"'):
            if in_quote == char:
                in_quote = ""
            elif not in_quote:
                in_quote = char
        if char == "#" and not in_quote:
            return line[:index]
    return line


def _split_inline_yaml_values(value: str) -> list[str]:
    values: list[str] = []
    current: list[str] = []
    quote = ""
    escaped = False
    depth = 0
    for char in value:
        if escaped:
            current.append(char)
            escaped = False
            continue
        if char == "\\" and quote:
            current.append(char)
            escaped = True
            continue
        if char in {"'", '"'}:
            if quote == char:
                quote = ""
            elif not quote:
                quote = char
            current.append(char)
            continue
        if not quote:
            if char in "[({":
                depth += 1
            elif char in "])}" and depth > 0:
                depth -= 1
            elif char == "," and depth == 0:
                values.append("".join(current).strip())
                current = []
                continue
        current.append(char)
    values.append("".join(current).strip())
    return values


def _clean_scalar(value: str) -> Any:
    text = value.strip()
    if text == "":
        return ""
    lowered = text.lower()
    if lowered in {"null", "none", "~"}:
        return None
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if len(text) >= 2 and text[0] == text[-1] and text[0] in {"'", '"'}:
        if text[0] == '"':
            try:
                return json.loads(text)
            except (json.JSONDecodeError, TypeError):
                pass
        return text[1:-1]
    if text.startswith("[") and text.endswith("]"):
        try:
            return literal_eval(text)
        except (SyntaxError, ValueError):
            inner = text[1:-1].strip()
            if not inner:
                return []
            return [_clean_scalar(item) for item in _split_inline_yaml_values(inner)]
    try:
        if any(char in text for char in ".eE"):
            return float(text)
        return int(text)
    except ValueError:
        return text


def _yaml_lines(path: Path) -> list[tuple[int, str]]:
    lines = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        cleaned = _strip_comment(raw).rstrip()
        if not cleaned.strip():
            continue
        indent = len(cleaned) - len(cleaned.lstrip(" "))
        lines.append((indent, cleaned.strip()))
    return lines


def _next_indent(lines: list[tuple[int, str]], index: int, fallback: int) -> int:
    if index < len(lines):
        return lines[index][0]
    return fallback


def _parse_yaml_block(lines: list[tuple[int, str]], index: int, indent: int) -> tuple[Any, int]:
    if index >= len(lines):
        return {}, index
    if lines[index][0] < indent:
        return {}, index
    if lines[index][1].startswith("- "):
        return _parse_yaml_list(lines, index, lines[index][0])
    return _parse_yaml_map(lines, index, indent)


def _parse_yaml_map(lines: list[tuple[int, str]], index: int, indent: int) -> tuple[dict[str, Any], int]:
    data: dict[str, Any] = {}
    while index < len(lines):
        line_indent, text = lines[index]
        if line_indent < indent:
            break
        if line_indent > indent:
            index += 1
            continue
        if text.startswith("- ") or ":" not in text:
            break
        key, value = text.split(":", 1)
        key = key.strip()
        value = value.strip()
        index += 1
        if value:
            data[key] = _clean_scalar(value)
        else:
            child_indent = _next_indent(lines, index, line_indent + 2)
            data[key], index = _parse_yaml_block(lines, index, child_indent)
    return data, index


def _parse_yaml_list(lines: list[tuple[int, str]], index: int, indent: int) -> tuple[list[Any], int]:
    items: list[Any] = []
    while index < len(lines):
        line_indent, text = lines[index]
        if line_indent < indent:
            break
        if line_indent != indent or not text.startswith("- "):
            break

        item_text = text[2:].strip()
        index += 1

        if item_text.startswith("- "):
            nested = [_clean_scalar(item_text[2:].strip())]
            while index < len(lines) and lines[index][0] > indent and lines[index][1].startswith("- "):
                nested.append(_clean_scalar(lines[index][1][2:].strip()))
                index += 1
            items.append(nested)
            continue

        if not item_text:
            child_indent = _next_indent(lines, index, indent + 2)
            child, index = _parse_yaml_block(lines, index, child_indent)
            items.append(child)
            continue

        if ":" in item_text and not item_text.startswith("["):
            item: dict[str, Any] = {}
            key, value = item_text.split(":", 1)
            item[key.strip()] = _clean_scalar(value.strip()) if value.strip() else {}
            while index < len(lines):
                next_indent, next_text = lines[index]
                if next_indent <= indent:
                    break
                if next_text.startswith("- ") and next_indent == indent:
                    break
                if ":" not in next_text:
                    index += 1
                    continue
                child_key, child_value = next_text.split(":", 1)
                child_key = child_key.strip()
                child_value = child_value.strip()
                index += 1
                if child_value:
                    item[child_key] = _clean_scalar(child_value)
                else:
                    child_indent = _next_indent(lines, index, next_indent + 2)
                    item[child_key], index = _parse_yaml_block(lines, index, child_indent)
            items.append(item)
            continue

        items.append(_clean_scalar(item_text))
    return items, index


def load_yaml(path: Path) -> dict[str, Any]:
    lines = _yaml_lines(path)
    data, _ = _parse_yaml_block(lines, 0, lines[0][0] if lines else 0)
    return data if isinstance(data, dict) else {}


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_point(row: Any) -> list[float] | None:
    if isinstance(row, (list, tuple)) and len(row) >= 2:
        return [_as_float(row[0]), _as_float(row[1])]
    return None


def _as_points(rows: Any) -> list[list[float]]:
    points = []
    if not isinstance(rows, list):
        return points
    for row in rows:
        point = _as_point(row)
        if point is not None:
            points.append(point)
    return points


def _polyline_length(points: list[list[float]], closed: bool = False) -> float:
    if len(points) < 2:
        return 0.0
    total = 0.0
    for index in range(1, len(points)):
        total += math.hypot(points[index][0] - points[index - 1][0], points[index][1] - points[index - 1][1])
    if closed and len(points) >= 3:
        total += math.hypot(points[0][0] - points[-1][0], points[0][1] - points[-1][1])
    return total


def _fmt_float(value: float) -> str:
    normalized = 0.0 if abs(value) < 5.0e-13 else float(value)
    return f"{normalized:.9g}"


def _quote_yaml_string(value: str) -> str:
    return json.dumps(value, ensure_ascii=True)


def _sanitize_id(value: str, fallback: str) -> str:
    allowed = [char for char in value.strip() if char.isalnum() or char in ("_", "-", ".")]
    result = "".join(allowed)
    return result or fallback


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
    except OSError:
        return ""
    return digest.hexdigest()


def _file_fingerprint(paths: list[Path]) -> str:
    digest = hashlib.sha256()
    for path in paths:
        digest.update(_sha256_file(path).encode("ascii"))
    return digest.hexdigest()


def _payload_points(value: Any) -> list[list[float]]:
    points: list[list[float]] = []
    if not isinstance(value, list):
        return points
    for row in value:
        if not isinstance(row, (list, tuple)) or len(row) < 2:
            continue
        try:
            x = float(row[0])
            y = float(row[1])
        except (TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append([x, y])
    return points


def _segment_distance(point: list[float], start: list[float], end: list[float]) -> float:
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    denom = vx * vx + vy * vy
    if denom <= 1.0e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    t = ((point[0] - start[0]) * vx + (point[1] - start[1]) * vy) / denom
    t = max(0.0, min(1.0, t))
    closest_x = start[0] + vx * t
    closest_y = start[1] + vy * t
    return math.hypot(point[0] - closest_x, point[1] - closest_y)


def _nearest_distance(point: list[float], polyline: list[list[float]], closed_loop: bool) -> float:
    if not polyline:
        return 0.0
    if len(polyline) == 1:
        return math.hypot(point[0] - polyline[0][0], point[1] - polyline[0][1])
    segment_count = len(polyline) if closed_loop and len(polyline) >= 3 else len(polyline) - 1
    return min(
        _segment_distance(point, polyline[index], polyline[(index + 1) % len(polyline)])
        for index in range(segment_count)
    )


def _lane_export_issue(lane: dict[str, Any]) -> str | None:
    closed_loop = bool(lane.get("closed_loop", True))
    bound_points = 3 if closed_loop else 2
    centerline_points = 3 if closed_loop else 2
    if len(lane.get("left_bound", [])) < bound_points:
        return f"left bound needs at least {bound_points} points"
    if len(lane.get("right_bound", [])) < bound_points:
        return f"right bound needs at least {bound_points} points"
    if len(lane.get("centerline", [])) < centerline_points:
        return f"centerline needs at least {centerline_points} points"
    return None


def _append_world_points(lines: list[str], field_name: str, points: list[list[float]]) -> None:
    if not points:
        lines.append(f"    {field_name}: []")
        return
    lines.append(f"    {field_name}:")
    for point in points:
        lines.append(f"      - [{_fmt_float(point[0])}, {_fmt_float(point[1])}, 0.0]")


def _yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return _fmt_float(float(value))
    return _quote_yaml_string(str(value))


def _append_preserved_section_gates(lines: list[str], gates: Any) -> None:
    if not isinstance(gates, list) or not gates:
        return
    lines.append("section_gates:")
    for index, gate in enumerate(gates, start=1):
        if not isinstance(gate, dict):
            continue
        gate_id = _sanitize_id(str(gate.get("id") or ""), f"gate_{index:03d}")
        lane_id = _sanitize_id(str(gate.get("lane_id") or ""), "lane_001")
        lines.append(f"  - id: {_quote_yaml_string(gate_id)}")
        lines.append(f"    lane_id: {_quote_yaml_string(lane_id)}")
        lines.append(f"    s_m: {_fmt_float(_as_float(gate.get('s_m'), 0.0))}")
        line = _as_points(gate.get("line", []))[:2]
        if line:
            lines.append("    line:")
            for point in line:
                lines.append(f"      - [{_fmt_float(point[0])}, {_fmt_float(point[1])}, 0.0]")
        else:
            lines.append("    line: []")


def _append_preserved_sections(lines: list[str], sections: Any) -> None:
    if not isinstance(sections, list) or not sections:
        return
    keys = (
        "id",
        "lane_id",
        "start_gate_id",
        "end_gate_id",
        "start_s_m",
        "end_s_m",
        "wrap",
        "lane_length_m",
        "speed_override_mps",
        "speed_scale",
        "class",
        "policy",
        "allow_overtake",
        "note",
    )
    lines.append("sections:")
    for index, section in enumerate(sections, start=1):
        if not isinstance(section, dict):
            continue
        first = True
        emitted = False
        for key in keys:
            if key not in section or section[key] is None:
                continue
            prefix = "  - " if first else "    "
            lines.append(f"{prefix}{key}: {_yaml_scalar(section[key])}")
            first = False
            emitted = True
        if not emitted:
            lines.append(f"  - id: {_quote_yaml_string(f'section_{index:03d}')}")


def _append_preserved_junctions(lines: list[str], junctions: Any) -> None:
    if not isinstance(junctions, list) or not junctions:
        return
    lines.append("junctions:")
    for index, junction in enumerate(junctions, start=1):
        if not isinstance(junction, dict):
            continue
        junction_id = _sanitize_id(str(junction.get("id") or ""), f"junction_{index:03d}")
        signal_id = _sanitize_id(str(junction.get("signal_id") or ""), f"signal_{index:03d}")
        sections = junction.get("activation_section_ids", [])
        if not isinstance(sections, list):
            sections = []
        release_sections = junction.get("release_section_ids", [])
        if not isinstance(release_sections, list):
            release_sections = []
        branches = junction.get("branches", {})
        if not isinstance(branches, dict):
            branches = {}
        lines.append(f"  - id: {_quote_yaml_string(junction_id)}")
        lines.append(f"    signal_id: {_quote_yaml_string(signal_id)}")
        position = _as_point(junction.get("position")) or [0.0, 0.0]
        lines.append(
            f"    position: [{_fmt_float(position[0])}, {_fmt_float(position[1])}, 0.0]"
        )
        section_values = ", ".join(_quote_yaml_string(str(value)) for value in sections if str(value))
        lines.append(f"    activation_section_ids: [{section_values}]")
        release_values = ", ".join(
            _quote_yaml_string(str(value)) for value in release_sections if str(value)
        )
        lines.append(f"    release_section_ids: [{release_values}]")
        lines.append("    branches:")
        lines.append(f"      left: {_quote_yaml_string(str(branches.get('left') or ''))}")
        lines.append(f"      straight: {_quote_yaml_string(str(branches.get('straight') or ''))}")
        lines.append(f"      right: {_quote_yaml_string(str(branches.get('right') or ''))}")


def _write_hd_map_yaml(
    output_path: Path,
    raster: dict[str, Any],
    lanes: list[dict[str, Any]],
    primary_lane_id: str,
    centerline_csv_path: Path,
    previous_data: dict[str, Any],
) -> None:
    output_mode = output_path.stat().st_mode & 0o777 if output_path.exists() else 0o644
    origin = raster.get("origin_xy_yaw") or [0.0, 0.0, 0.0]
    lines = [
        "format: tamiya_local_hd_map_v1",
        "frame_id: map",
        "units: meter",
        f"primary_lane_id: {_quote_yaml_string(primary_lane_id)}",
        "source_raster:",
        f"  map_yaml: {_quote_yaml_string(str(raster.get('map_yaml_path') or ''))}",
        f"  image: {_quote_yaml_string(str(raster.get('image_path') or ''))}",
        f"  resolution_m_per_px: {_fmt_float(_as_float(raster.get('resolution_m_per_px'), 0.0))}",
        (
            "  origin_xy_yaw: "
            f"[{_fmt_float(_as_float(origin[0] if len(origin) > 0 else 0.0))}, "
            f"{_fmt_float(_as_float(origin[1] if len(origin) > 1 else 0.0))}, "
            f"{_fmt_float(_as_float(origin[2] if len(origin) > 2 else 0.0))}]"
        ),
        f"  image_size_px: [{int(raster.get('width') or 0)}, {int(raster.get('height') or 0)}]",
        "exports:",
        f"  primary_centerline_csv: {_quote_yaml_string(str(centerline_csv_path))}",
        "lanes:",
    ]
    for lane in lanes:
        lane_id = str(lane["id"])
        lines.extend(
            [
                f"  - id: {_quote_yaml_string(lane_id)}",
                f"    closed_loop: {'true' if lane.get('closed_loop', True) else 'false'}",
            ]
        )
        _append_world_points(lines, "left_bound", lane["left_bound"])
        _append_world_points(lines, "right_bound", lane["right_bound"])
        _append_world_points(lines, "centerline", lane["centerline"])
    _append_preserved_section_gates(lines, previous_data.get("section_gates"))
    _append_preserved_sections(lines, previous_data.get("sections"))
    _append_preserved_junctions(lines, previous_data.get("junctions"))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output_path.parent,
            prefix=f".{output_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            handle.write("\n".join(lines) + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary_path, output_mode)
        os.replace(temporary_path, output_path)
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass


def _write_centerline_csv(path: Path, lane: dict[str, Any]) -> None:
    issue = _lane_export_issue(lane)
    if issue is not None:
        raise ValueError(f"Lane {lane['id']} cannot export centerline CSV: {issue}.")
    centerline = lane["centerline"]
    left_bound = lane["left_bound"]
    right_bound = lane["right_bound"]
    closed_loop = bool(lane.get("closed_loop", True))
    with path.open("w", encoding="utf-8", newline="") as handle:
        handle.write("# x_m,y_m,w_tr_right_m,w_tr_left_m\n")
        writer = csv.writer(handle)
        for point in centerline:
            writer.writerow(
                [
                    f"{point[0]:.6f}",
                    f"{point[1]:.6f}",
                    f"{_nearest_distance(point, right_bound, closed_loop):.6f}",
                    f"{_nearest_distance(point, left_bound, closed_loop):.6f}",
                ]
            )


def _lanes_from_hd_data(data: dict[str, Any]) -> list[dict[str, Any]]:
    lanes: list[dict[str, Any]] = []
    raw_lanes = data.get("lanes")
    if not isinstance(raw_lanes, list):
        return lanes
    for index, raw_lane in enumerate(raw_lanes, start=1):
        if not isinstance(raw_lane, dict):
            continue
        lane_id = _sanitize_id(str(raw_lane.get("id") or ""), f"lane_{index:03d}")
        lanes.append(
            {
                "id": lane_id,
                "closed_loop": bool(raw_lane.get("closed_loop", True)),
                "left_bound": _as_points(raw_lane.get("left_bound", [])),
                "right_bound": _as_points(raw_lane.get("right_bound", [])),
                "centerline": _as_points(raw_lane.get("centerline", [])),
            }
        )
    return lanes


def _payload_section_gates(value: Any, lane_ids: set[str]) -> list[dict[str, Any]]:
    gates: list[dict[str, Any]] = []
    if not isinstance(value, list):
        return gates
    fallback_lane_id = sorted(lane_ids)[0] if lane_ids else "lane_001"
    used_ids: set[str] = set()
    for index, raw_gate in enumerate(value, start=1):
        if not isinstance(raw_gate, dict):
            continue
        gate_id = _sanitize_id(str(raw_gate.get("id") or ""), f"gate_{index:03d}")
        if gate_id in used_ids:
            base_id = f"gate_{index:03d}"
            gate_id = base_id
            suffix = 2
            while gate_id in used_ids:
                gate_id = f"{base_id[:55]}_{suffix}"
                suffix += 1
        used_ids.add(gate_id)
        lane_id = _sanitize_id(str(raw_gate.get("lane_id") or ""), fallback_lane_id)
        if lane_ids and lane_id not in lane_ids:
            lane_id = fallback_lane_id
        line = _payload_points(raw_gate.get("line"))[:2]
        if len(line) < 2:
            continue
        gates.append(
            {
                "id": gate_id,
                "lane_id": lane_id,
                "s_m": _as_float(raw_gate.get("s_m"), 0.0),
                "line": line,
            }
        )
    return sorted(gates, key=lambda gate: (str(gate["lane_id"]), float(gate["s_m"]), str(gate["id"])))


def _string_id_list(value: Any, field_name: str) -> list[str]:
    if not isinstance(value, list):
        raise ValueError(f"{field_name} must be a list")
    output: list[str] = []
    seen: set[str] = set()
    for item in value:
        raw = str(item or "").strip()
        if not raw:
            continue
        normalized = _sanitize_id(raw, "")
        if not normalized:
            raise ValueError(f"{field_name} contains an invalid ID")
        if normalized not in seen:
            seen.add(normalized)
            output.append(normalized)
    return output


def _junction_default_position(
    activation_section_ids: list[str],
    sections: list[dict[str, Any]],
    gates: list[dict[str, Any]],
    lanes: list[dict[str, Any]],
) -> list[float]:
    section_by_id = {str(section.get("id") or ""): section for section in sections}
    gate_by_id = {str(gate.get("id") or ""): gate for gate in gates}
    for section_id in activation_section_ids:
        section = section_by_id.get(section_id)
        gate = gate_by_id.get(str(section.get("end_gate_id") or "")) if section else None
        line = _as_points(gate.get("line"))[:2] if gate else []
        if len(line) == 2:
            return [0.5 * (line[0][0] + line[1][0]), 0.5 * (line[0][1] + line[1][1])]
    for lane in lanes:
        centerline = _as_points(lane.get("centerline"))
        if centerline:
            return list(centerline[0])
    return [0.0, 0.0]


def _payload_junctions(
    value: Any,
    sections: list[dict[str, Any]],
    gates: list[dict[str, Any]],
    lanes: list[dict[str, Any]],
    known_route_ids: set[str],
) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        raise ValueError("junctions must be a list")
    section_ids = {str(section.get("id") or "") for section in sections}
    used_junction_ids: set[str] = set()
    used_signal_ids: set[str] = set()
    activation_owners: dict[str, str] = {}
    junctions: list[dict[str, Any]] = []
    for index, raw_junction in enumerate(value, start=1):
        if not isinstance(raw_junction, dict):
            raise ValueError(f"junctions[{index - 1}] must be an object")
        junction_id = _sanitize_id(str(raw_junction.get("id") or ""), "")
        signal_id = _sanitize_id(str(raw_junction.get("signal_id") or ""), "")
        if not junction_id:
            raise ValueError(f"junctions[{index - 1}].id is required")
        if not signal_id:
            raise ValueError(f"junction {junction_id} requires signal_id")
        if junction_id in used_junction_ids:
            raise ValueError(f"duplicate junction ID: {junction_id}")
        if signal_id in used_signal_ids:
            raise ValueError(f"duplicate signal ID: {signal_id}")
        used_junction_ids.add(junction_id)
        used_signal_ids.add(signal_id)

        activation = _string_id_list(
            raw_junction.get("activation_section_ids"),
            f"junction {junction_id} activation_section_ids",
        )
        release = _string_id_list(
            raw_junction.get("release_section_ids", []),
            f"junction {junction_id} release_section_ids",
        )
        if not activation:
            raise ValueError(f"junction {junction_id} requires at least one activation section")
        if not release:
            raise ValueError(f"junction {junction_id} requires at least one release section")
        unknown_sections = sorted((set(activation) | set(release)) - section_ids)
        if unknown_sections:
            raise ValueError(
                f"junction {junction_id} references unknown section(s): {', '.join(unknown_sections)}"
            )
        overlap = sorted(set(activation) & set(release))
        if overlap:
            raise ValueError(
                f"junction {junction_id} uses section(s) for both activation and release: {', '.join(overlap)}"
            )
        for section_id in activation:
            owner = activation_owners.get(section_id)
            if owner:
                raise ValueError(
                    f"activation section {section_id} is already assigned to junction {owner}"
                )
            activation_owners[section_id] = junction_id

        branches = raw_junction.get("branches")
        if not isinstance(branches, dict):
            raise ValueError(f"junction {junction_id} branches must be an object")
        normalized_branches: dict[str, str] = {}
        for direction in ("left", "straight", "right"):
            route_id = _sanitize_id(str(branches.get(direction) or ""), "")
            if not route_id:
                raise ValueError(f"junction {junction_id} requires a {direction} route")
            if route_id not in known_route_ids:
                raise ValueError(
                    f"junction {junction_id} references unknown {direction} route: {route_id}"
                )
            normalized_branches[direction] = route_id

        raw_position = raw_junction.get("position")
        if raw_position is None:
            position = _junction_default_position(activation, sections, gates, lanes)
        else:
            parsed_positions = _payload_points([raw_position])
            if not parsed_positions:
                raise ValueError(f"junction {junction_id} position must contain finite x/y values")
            position = parsed_positions[0]

        junctions.append(
            {
                "id": junction_id,
                "signal_id": signal_id,
                "position": position,
                "activation_section_ids": activation,
                "release_section_ids": release,
                "branches": normalized_branches,
            }
        )
    return junctions


def _custom_line_junction_route_issue(item: dict[str, Any]) -> str:
    if not bool(item.get("valid")):
        detail = str(item.get("issue") or "").strip()
        return detail or "custom line is invalid"
    if item.get("source_stale") is True:
        return "custom line source has changed; update the line before using it as a branch"
    if bool(item.get("section_layout_stale")):
        return "custom line section layout is stale; update the line before using it as a branch"
    return ""


def _junction_route_catalog(
    map_dir: Path,
    lane_ids: set[str],
    custom_line_catalog: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = [
        {"id": lane_id, "kind": "lane", "eligible": True, "issue": ""}
        for lane_id in sorted(lane_ids)
        if lane_id
    ]
    if "primary" not in lane_ids:
        entries.append({"id": "primary", "kind": "lane_alias", "eligible": True, "issue": ""})
    if (map_dir / f"{map_dir.name}_raceline.csv").is_file():
        entries.append({"id": "raceline", "kind": "raceline", "eligible": True, "issue": ""})

    catalog = custom_line_catalog if custom_line_catalog is not None else _read_custom_lines(map_dir)
    for item in catalog.get("items", []):
        if not isinstance(item, dict):
            continue
        route_id = str(item.get("id") or "")
        if not route_id:
            continue
        issue = _custom_line_junction_route_issue(item)
        entries.append(
            {
                "id": route_id,
                "kind": "custom_line",
                "eligible": not issue,
                "issue": issue,
            }
        )
    return entries


def _known_junction_route_ids(
    map_dir: Path,
    lane_ids: set[str],
    custom_line_catalog: dict[str, Any] | None = None,
) -> set[str]:
    return {
        str(item["id"])
        for item in _junction_route_catalog(map_dir, lane_ids, custom_line_catalog)
        if item.get("eligible") and str(item.get("id") or "")
    }


def _validated_hd_topology(
    map_dir: Path,
    previous_data: dict[str, Any],
    lanes: list[dict[str, Any]],
) -> dict[str, Any]:
    lane_ids = {str(lane.get("id") or "") for lane in lanes}
    gates = [gate for gate in previous_data.get("section_gates", []) if isinstance(gate, dict)]
    gate_ids = {str(gate.get("id") or "") for gate in gates}
    for gate in gates:
        lane_id = str(gate.get("lane_id") or "")
        if lane_id and lane_id not in lane_ids:
            raise ValueError(
                f"section gate {gate.get('id') or '<unnamed>'} references unknown lane: {lane_id}"
            )

    sections = [
        section for section in previous_data.get("sections", []) if isinstance(section, dict)
    ]
    for section in sections:
        section_id = str(section.get("id") or "<unnamed>")
        lane_id = str(section.get("lane_id") or "")
        if lane_id and lane_id not in lane_ids:
            raise ValueError(f"section {section_id} references unknown lane: {lane_id}")
        for field_name in ("start_gate_id", "end_gate_id"):
            gate_id = str(section.get(field_name) or "")
            if gate_id and gate_id not in gate_ids:
                raise ValueError(
                    f"section {section_id} references unknown {field_name}: {gate_id}"
                )

    validated = dict(previous_data)
    validated["junctions"] = _payload_junctions(
        previous_data.get("junctions", []),
        sections,
        gates,
        lanes,
        _known_junction_route_ids(map_dir, lane_ids),
    )
    return validated


def _section_preserve_key(section: dict[str, Any]) -> tuple[str, str, str]:
    return (
        str(section.get("lane_id") or ""),
        str(section.get("start_gate_id") or ""),
        str(section.get("end_gate_id") or ""),
    )


def _build_sections_for_gates(
    previous_sections: Any,
    gates: list[dict[str, Any]],
    lanes: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    preserved: dict[tuple[str, str, str], dict[str, Any]] = {}
    if isinstance(previous_sections, list):
        for section in previous_sections:
            if isinstance(section, dict):
                preserved[_section_preserve_key(section)] = section

    lane_by_id = {str(lane["id"]): lane for lane in lanes}
    gates_by_lane: dict[str, list[dict[str, Any]]] = {}
    for gate in gates:
        gates_by_lane.setdefault(str(gate["lane_id"]), []).append(gate)

    sections: list[dict[str, Any]] = []
    used_section_ids: set[str] = set()
    section_index = 1
    for lane_id, lane_gates in sorted(gates_by_lane.items()):
        lane = lane_by_id.get(lane_id)
        if lane is None or len(lane_gates) < 2:
            continue
        sorted_gates = sorted(lane_gates, key=lambda gate: float(gate["s_m"]))
        closed_loop = bool(lane.get("closed_loop", True))
        lane_length = _polyline_length(lane.get("centerline", []), closed_loop)
        pair_count = len(sorted_gates) if closed_loop else len(sorted_gates) - 1
        for index in range(pair_count):
            start_gate = sorted_gates[index]
            end_gate = sorted_gates[(index + 1) % len(sorted_gates)]
            if not closed_loop and index + 1 >= len(sorted_gates):
                continue
            key = (lane_id, str(start_gate["id"]), str(end_gate["id"]))
            previous = preserved.get(key, {})
            requested_id = _sanitize_id(str(previous.get("id") or ""), f"section_{section_index:03d}")
            section_id = requested_id
            suffix = 2
            while section_id in used_section_ids:
                section_id = f"{requested_id[:55]}_{suffix}"
                suffix += 1
            used_section_ids.add(section_id)
            section: dict[str, Any] = {
                "id": section_id,
                "lane_id": lane_id,
                "start_gate_id": str(start_gate["id"]),
                "end_gate_id": str(end_gate["id"]),
                "start_s_m": float(start_gate["s_m"]),
                "end_s_m": float(end_gate["s_m"]),
            }
            if closed_loop:
                section["wrap"] = float(start_gate["s_m"]) > float(end_gate["s_m"])
                section["lane_length_m"] = lane_length
            for key_name in (
                "speed_override_mps",
                "speed_scale",
                "class",
                "policy",
                "allow_overtake",
                "note",
            ):
                if key_name in previous:
                    section[key_name] = previous.get(key_name)
            sections.append(section)
            section_index += 1
    return sections


def _resolve_embedded_path(value: Any, base: Path) -> Path | None:
    if not value:
        return None
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def _png_size(path: Path) -> tuple[int, int] | None:
    try:
        with path.open("rb") as handle:
            header = handle.read(24)
    except OSError:
        return None
    if header.startswith(b"\x89PNG\r\n\x1a\n") and len(header) >= 24:
        width, height = struct.unpack(">II", header[16:24])
        return int(width), int(height)
    return None


def _raster_from_hd_map(hd_map_path: Path, hd_data: dict[str, Any]) -> dict[str, Any] | None:
    source = hd_data.get("source_raster")
    if not isinstance(source, dict):
        return None
    image_path = _resolve_embedded_path(source.get("image"), hd_map_path.parent)
    map_yaml_path = _resolve_embedded_path(source.get("map_yaml"), hd_map_path.parent)
    image_size = source.get("image_size_px")
    width = int(image_size[0]) if isinstance(image_size, list) and len(image_size) >= 2 else 0
    height = int(image_size[1]) if isinstance(image_size, list) and len(image_size) >= 2 else 0
    if image_path and (not width or not height):
        size = _png_size(image_path)
        if size:
            width, height = size
    origin = source.get("origin_xy_yaw") if isinstance(source.get("origin_xy_yaw"), list) else [0.0, 0.0, 0.0]
    return {
        "map_yaml_path": str(map_yaml_path) if map_yaml_path else "",
        "image_path": str(image_path) if image_path else "",
        "image_url": _file_url(image_path) if image_path and image_path.exists() else "",
        "resolution_m_per_px": _as_float(source.get("resolution_m_per_px"), 0.0),
        "origin_xy_yaw": [_as_float(origin[0]), _as_float(origin[1]), _as_float(origin[2] if len(origin) > 2 else 0.0)],
        "width": width,
        "height": height,
    }


def _raster_from_map_yaml(map_yaml_path: Path) -> dict[str, Any] | None:
    if not map_yaml_path.exists():
        return None
    data = load_yaml(map_yaml_path)
    image_path = _resolve_embedded_path(data.get("image"), map_yaml_path.parent)
    origin = data.get("origin") if isinstance(data.get("origin"), list) else [0.0, 0.0, 0.0]
    width = height = 0
    if image_path:
        size = _png_size(image_path)
        if size:
            width, height = size
    return {
        "map_yaml_path": str(map_yaml_path),
        "image_path": str(image_path) if image_path else "",
        "image_url": _file_url(image_path) if image_path and image_path.exists() else "",
        "resolution_m_per_px": _as_float(data.get("resolution"), 0.0),
        "origin_xy_yaw": [_as_float(origin[0]), _as_float(origin[1]), _as_float(origin[2] if len(origin) > 2 else 0.0)],
        "width": width,
        "height": height,
    }


def _read_hd_map(path: Path) -> tuple[dict[str, Any], dict[str, Any] | None]:
    if not path.exists():
        return {"exists": False, "lanes": [], "section_gates": [], "sections": [], "junctions": []}, None
    data = load_yaml(path)
    primary_lane_id = str(data.get("primary_lane_id") or "")
    lanes = []
    for lane in data.get("lanes", []) or []:
        if not isinstance(lane, dict):
            continue
        lane_id = str(lane.get("id") or f"lane_{len(lanes) + 1:03d}")
        closed_loop = bool(lane.get("closed_loop", True))
        centerline = _as_points(lane.get("centerline", []))
        lanes.append(
            {
                "id": lane_id,
                "primary": lane_id == primary_lane_id,
                "closed_loop": closed_loop,
                "left_bound": _as_points(lane.get("left_bound", [])),
                "right_bound": _as_points(lane.get("right_bound", [])),
                "centerline": centerline,
                "centerline_length_m": _polyline_length(centerline, closed_loop),
            }
        )

    gates = []
    for raw_gate in data.get("section_gates", []) or []:
        if not isinstance(raw_gate, dict):
            continue
        line = _as_points(raw_gate.get("line", []))
        gates.append(
            {
                "id": str(raw_gate.get("id") or f"gate_{len(gates) + 1:03d}"),
                "lane_id": str(raw_gate.get("lane_id") or ""),
                "s_m": _as_float(raw_gate.get("s_m")),
                "line": line[:2],
            }
        )

    sections = []
    for raw_section in data.get("sections", []) or []:
        if not isinstance(raw_section, dict):
            continue
        sections.append(
            {
                "id": str(raw_section.get("id") or f"section_{len(sections) + 1:03d}"),
                "lane_id": str(raw_section.get("lane_id") or ""),
                "start_gate_id": str(raw_section.get("start_gate_id") or ""),
                "end_gate_id": str(raw_section.get("end_gate_id") or ""),
                "start_s_m": _as_float(raw_section.get("start_s_m")),
                "end_s_m": _as_float(raw_section.get("end_s_m")),
                "wrap": bool(raw_section.get("wrap", False)),
                "lane_length_m": raw_section.get("lane_length_m"),
                "speed_override_mps": raw_section.get("speed_override_mps"),
                "speed_scale": raw_section.get("speed_scale"),
                "class": raw_section.get("class"),
                "policy": raw_section.get("policy"),
            }
        )

    junctions = []
    for raw_junction in data.get("junctions", []) or []:
        if not isinstance(raw_junction, dict):
            continue
        branches = raw_junction.get("branches", {})
        activation_sections = raw_junction.get("activation_section_ids", [])
        if not isinstance(activation_sections, list):
            activation_sections = []
        release_sections = raw_junction.get("release_section_ids", [])
        if not isinstance(release_sections, list):
            release_sections = []
        activation_section_ids = [str(value) for value in activation_sections]
        raw_position = raw_junction.get("position")
        position = _as_point(raw_position) if raw_position is not None else None
        if position is None:
            position = _junction_default_position(
                activation_section_ids,
                sections,
                gates,
                lanes,
            )
        junctions.append(
            {
                "id": str(raw_junction.get("id") or f"junction_{len(junctions) + 1:03d}"),
                "signal_id": str(raw_junction.get("signal_id") or ""),
                "position": position,
                "activation_section_ids": activation_section_ids,
                "release_section_ids": [str(value) for value in release_sections],
                "branches": dict(branches) if isinstance(branches, dict) else {},
            }
        )

    return (
        {
            "exists": True,
            "path": str(path),
            "primary_lane_id": primary_lane_id,
            "lanes": lanes,
            "section_gates": gates,
            "sections": sections,
            "junctions": junctions,
        },
        _raster_from_hd_map(path, data),
    )


def _read_xy_csv(path: Path, delimiter: str, x_index: int, y_index: int) -> dict[str, Any]:
    if not path.exists():
        return {"exists": False, "path": str(path), "points": [], "count": 0, "length_m": 0.0}
    points: list[list[float]] = []
    try:
        with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
            reader = csv.reader(handle, delimiter=delimiter)
            for row in reader:
                if not row or row[0].strip().startswith("#"):
                    continue
                try:
                    points.append([float(row[x_index]), float(row[y_index])])
                except (IndexError, ValueError):
                    continue
    except OSError:
        points = []
    return {
        "exists": True,
        "path": str(path),
        "points": points,
        "count": len(points),
        "length_m": _polyline_length(points, False),
    }


def _sample_pose_point(sample: Any) -> list[float] | None:
    if not isinstance(sample, dict):
        return None
    pose = sample.get("pose") if isinstance(sample.get("pose"), dict) else sample
    position = pose.get("position") if isinstance(pose, dict) and isinstance(pose.get("position"), dict) else None
    if not isinstance(position, dict):
        return None
    try:
        x = float(position.get("x"))
        y = float(position.get("y"))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return [x, y]


def _read_snapshot_odometry(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"exists": False, "path": str(path), "points": [], "count": 0, "length_m": 0.0}
    try:
        data = json.loads(path.read_text(encoding="utf-8", errors="replace"))
    except (OSError, json.JSONDecodeError):
        return {"exists": True, "path": str(path), "points": [], "count": 0, "length_m": 0.0, "error": "snapshot could not be parsed"}

    samples = data.get("odometry_samples")
    source = "odometry_samples"
    if not isinstance(samples, list) or not samples:
        full_path = data.get("full_vslam_path")
        samples = full_path.get("poses") if isinstance(full_path, dict) else []
        source = "full_vslam_path"
    if not isinstance(samples, list):
        samples = []
    points = []
    if isinstance(samples, list):
        for sample in samples:
            point = _sample_pose_point(sample)
            if point is not None:
                points.append(point)
    localization = data.get("localization") if isinstance(data.get("localization"), dict) else {}
    try:
        history_stride = int(localization.get("history_stride") or 1)
    except (TypeError, ValueError):
        history_stride = 1
    return {
        "exists": True,
        "path": str(path),
        "source": source,
        "frame_id": str((samples[-1].get("frame_id") if samples and isinstance(samples[-1], dict) else "") or localization.get("map_frame") or ""),
        "points": points,
        "count": len(points),
        "length_m": _polyline_length(points, False),
        "localized": bool(localization.get("confirmed") or localization.get("confirmed_once")),
        "history_stride": history_stride,
    }


def _display_name(map_root: Path, map_dir: Path) -> str:
    try:
        parts = map_dir.relative_to(map_root).parts
    except ValueError:
        parts = ()
    return parts[0] if len(parts) > 1 else map_dir.name


def directory_fingerprint(path: Path) -> str:
    """Match the stat-based fingerprint stored by the analysis worker."""

    digest = hashlib.sha256()
    for child in sorted(item for item in path.rglob("*") if item.is_file()):
        relative = child.relative_to(path).as_posix()
        stat = child.stat()
        digest.update(relative.encode("utf-8"))
        digest.update(str(stat.st_size).encode("ascii"))
        digest.update(str(stat.st_mtime_ns).encode("ascii"))
    return digest.hexdigest()


def _active_hd_artifact_paths(map_dir: Path) -> dict[str, Path]:
    name = map_dir.name
    return {
        "hd_map": map_dir / f"{name}_hd_map.yaml",
        "centerline_csv": map_dir / f"{name}_hd_map_centerline.csv",
        "raceline_csv": map_dir / f"{name}_raceline.csv",
        "raceline_meta": map_dir / f"{name}_raceline.meta.json",
        "line_preview": map_dir / f"{name}_line_preview.png",
    }


def _version_root(map_dir: Path) -> Path:
    return map_dir / HD_MAP_VERSION_DIR


def _version_active_path(map_dir: Path) -> Path:
    return _version_root(map_dir) / HD_MAP_VERSION_ACTIVE_FILE


def _read_json_file(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8", errors="replace"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def _write_json_file(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            handle.write(json.dumps(data, ensure_ascii=True, indent=2, sort_keys=True) + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass


def _version_artifact_paths(version_dir: Path) -> dict[str, Path]:
    return {
        "hd_map": version_dir / "hd_map.yaml",
        "centerline_csv": version_dir / "centerline.csv",
        "raceline_csv": version_dir / "raceline.csv",
        "raceline_meta": version_dir / "raceline.meta.json",
        "line_preview": version_dir / "line_preview.png",
    }


def _next_hd_map_version_id(root: Path) -> str:
    highest = 0
    if root.exists():
        for child in root.iterdir():
            if not child.is_dir() or not child.name.startswith("ver_"):
                continue
            try:
                highest = max(highest, int(child.name[4:]))
            except ValueError:
                continue
    return f"ver_{highest + 1:03d}"


def _active_hd_fingerprint(map_dir: Path) -> str:
    artifacts = _active_hd_artifact_paths(map_dir)
    paths = [artifacts["hd_map"], artifacts["centerline_csv"]]
    return _file_fingerprint([path for path in paths if path.exists()])


def _version_fingerprint(version_dir: Path) -> str:
    artifacts = _version_artifact_paths(version_dir)
    paths = [artifacts["hd_map"], artifacts["centerline_csv"]]
    return _file_fingerprint([path for path in paths if path.exists()])


def _read_hd_map_versions(map_dir: Path) -> dict[str, Any]:
    root = _version_root(map_dir)
    active = _read_json_file(_version_active_path(map_dir))
    active_id = str(active.get("id") or "")
    active_fingerprint = _active_hd_fingerprint(map_dir)
    versions: list[dict[str, Any]] = []
    if root.exists():
        for version_dir in sorted(child for child in root.iterdir() if child.is_dir()):
            if not version_dir.name.startswith("ver_"):
                continue
            manifest_path = version_dir / "manifest.json"
            manifest = _read_json_file(manifest_path)
            version_id = str(manifest.get("id") or version_dir.name)
            artifacts = _version_artifact_paths(version_dir)
            version_fingerprint = str(manifest.get("hd_fingerprint") or _version_fingerprint(version_dir))
            versions.append(
                {
                    "id": version_id,
                    "label": str(manifest.get("label") or version_id),
                    "created_at": str(manifest.get("created_at") or _iso_mtime(version_dir)),
                    "active": version_id == active_id,
                    "matches_active_files": bool(version_fingerprint and version_fingerprint == active_fingerprint),
                    "artifacts": {
                        "hd_map": _artifact(artifacts["hd_map"]),
                        "centerline_csv": _artifact(artifacts["centerline_csv"]),
                        "raceline_csv": _artifact(artifacts["raceline_csv"]),
                        "raceline_meta": _artifact(artifacts["raceline_meta"]),
                        "line_preview": _artifact(artifacts["line_preview"]),
                    },
                }
            )
    versions.sort(key=lambda item: str(item.get("created_at") or item.get("id") or ""), reverse=True)
    return {
        "active_id": active_id,
        "working_copy_dirty": bool(active_id and not any(item["active"] and item["matches_active_files"] for item in versions)),
        "versions": versions,
    }


def _custom_line_root(map_dir: Path) -> Path:
    return map_dir / CUSTOM_LINE_DIR


def _checked_custom_line_root(map_dir: Path, *, create: bool = False) -> Path:
    root = _custom_line_root(map_dir)
    if root.is_symlink():
        raise ValueError("custom_lines must be a real folder, not a symbolic link")
    if create:
        root.mkdir(parents=True, exist_ok=True)
    if root.is_symlink():
        raise ValueError("custom_lines must be a real folder, not a symbolic link")
    if root.exists() and not root.is_dir():
        raise ValueError("custom_lines exists but is not a directory")
    resolved_map_dir = map_dir.resolve()
    resolved_root = root.resolve()
    if not _is_relative_to(resolved_root, resolved_map_dir):
        raise ValueError("custom_lines resolves outside the map folder")
    return root


def _custom_line_active_path(map_dir: Path) -> Path:
    return _checked_custom_line_root(map_dir) / CUSTOM_LINE_ACTIVE_FILE


def _canonical_custom_line_paths(map_dir: Path) -> dict[str, Path]:
    return {
        "trajectory": map_dir / f"{map_dir.name}_custom_line.csv",
        "meta": map_dir / f"{map_dir.name}_custom_line.meta.json",
    }


def _canonical_custom_line_active(map_dir: Path) -> tuple[str, str]:
    canonical = _canonical_custom_line_paths(map_dir)
    meta = _read_json_file(canonical["meta"])
    if not meta:
        cached = _read_json_file(_custom_line_active_path(map_dir))
        cached_issue = str(cached.get("issue") or "")
        return "", cached_issue
    if str(meta.get("format") or "") != CUSTOM_LINE_FORMAT:
        return "", "active custom line metadata has an unsupported format"
    custom_line_id = str(meta.get("id") or "")
    if not _is_safe_custom_line_id(custom_line_id):
        return "", "active custom line metadata has an invalid id"
    expected_hash = str(meta.get("trajectory_sha256") or "").lower()
    if len(expected_hash) != 64 or any(char not in "0123456789abcdef" for char in expected_hash):
        return "", "active custom line metadata has an invalid trajectory hash"
    if canonical["trajectory"].is_symlink() or not canonical["trajectory"].is_file():
        return "", "active custom line trajectory is missing or not a regular file"
    if _sha256_file(canonical["trajectory"]).lower() != expected_hash:
        return "", "active custom line trajectory does not match its metadata"
    profile_mode = str(meta.get("speed_profile_mode") or "")
    speed_authoring = str(meta.get("speed_authoring") or "")
    section_authored = profile_mode == "sections" or speed_authoring == "sections"
    expected_layout = str(
        meta.get("section_layout_fingerprint") or meta.get("section_layout_hash") or ""
    ).lower()
    expected_hd_map_hash = str(meta.get("hd_map_sha256") or "").lower()
    if section_authored or expected_layout or expected_hd_map_hash:
        try:
            layout = _custom_line_hd_layout(map_dir)
        except (OSError, TypeError, ValueError, OverflowError) as exc:
            return "", f"active custom line HD section layout is invalid: {exc}"
        if not expected_layout or expected_layout != str(layout["fingerprint"]).lower():
            return "", "active custom line section layout is stale"
        if not expected_hd_map_hash or expected_hd_map_hash != str(layout["hd_map_sha256"]).lower():
            return "", "active custom line HD map hash is stale"
    return custom_line_id, ""


def _is_safe_custom_line_id(value: str) -> bool:
    if not value or len(value) > 64 or not value[0].isascii() or not value[0].isalnum():
        return False
    return all(char.isascii() and (char.isalnum() or char in ("_", "-")) for char in value)


def _require_custom_line_id(payload: dict[str, Any]) -> str:
    value = str(payload.get("id") or payload.get("custom_line_id") or "").strip()
    if not _is_safe_custom_line_id(value):
        raise ValueError("custom line id must use 1-64 ASCII letters, digits, '_' or '-', starting with a letter or digit")
    return value


def _custom_line_path(map_dir: Path, custom_line_id: str) -> Path:
    if not _is_safe_custom_line_id(custom_line_id):
        raise ValueError("invalid custom line id")
    raw_root = _checked_custom_line_root(map_dir)
    root = raw_root.resolve()
    candidate = raw_root / custom_line_id
    if candidate.is_symlink():
        raise ValueError("custom line folder must not be a symbolic link")
    path = candidate.resolve()
    if not _is_relative_to(path, root):
        raise ValueError("custom line path is outside the map folder")
    return path


def _custom_line_id_from_name(root: Path, name: str) -> str:
    slug_chars: list[str] = []
    previous_separator = False
    for char in name.strip().lower():
        if char.isascii() and char.isalnum():
            slug_chars.append(char)
            previous_separator = False
        elif not previous_separator and slug_chars:
            slug_chars.append("-")
            previous_separator = True
    base = "".join(slug_chars).strip("-")[:48] or "custom-line"
    if base == "active":
        base = "custom-line"
    candidate = base
    suffix = 2
    while (root / candidate).exists():
        candidate = f"{base[:55]}-{suffix}"
        suffix += 1
    return candidate


def _custom_line_name(value: Any) -> str:
    name = str(value or "").strip()
    if not name:
        raise ValueError("custom line name is required")
    if len(name) > 120:
        raise ValueError("custom line name must be 120 characters or fewer")
    if any(ord(char) < 32 or ord(char) == 127 for char in name):
        raise ValueError("custom line name must not contain control characters")
    return name


def _custom_line_source_type(payload: dict[str, Any]) -> str:
    value = str(payload.get("source_type") or payload.get("base") or "").strip().lower()
    aliases = {
        "centerline": "centerline",
        "center-line": "centerline",
        "center_line": "centerline",
        "raceline": "raceline",
        "race-line": "raceline",
        "race_line": "raceline",
    }
    normalized = aliases.get(value)
    if normalized is None:
        raise ValueError("source_type/base must be 'centerline' or 'raceline'")
    return normalized


def _nonnegative_finite_float(value: Any, field_name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field_name} must be a finite number")
    if number < 0.0:
        raise ValueError(f"{field_name} must be greater than or equal to 0")
    return number


def _positive_finite_float(value: Any, field_name: str) -> float:
    number = _nonnegative_finite_float(value, field_name)
    if number <= 0.0:
        raise ValueError(f"{field_name} must be greater than 0")
    return number


def _custom_line_constraints(
    payload: dict[str, Any],
    defaults: dict[str, Any] | None = None,
) -> dict[str, float]:
    fallback = {
        "max_speed_mps": CUSTOM_LINE_DEFAULT_MAX_SPEED_MPS,
        "lateral_accel_limit_mps2": CUSTOM_LINE_DEFAULT_LATERAL_ACCEL_LIMIT_MPS2,
        "accel_limit_mps2": CUSTOM_LINE_DEFAULT_ACCEL_LIMIT_MPS2,
        "decel_limit_mps2": CUSTOM_LINE_DEFAULT_DECEL_LIMIT_MPS2,
    }
    if isinstance(defaults, dict):
        fallback.update({key: defaults[key] for key in fallback if key in defaults})
    nested = payload.get("constraints") if isinstance(payload.get("constraints"), dict) else {}
    result: dict[str, float] = {}
    for field_name, default in fallback.items():
        value = payload[field_name] if field_name in payload else nested.get(field_name, default)
        result[field_name] = _positive_finite_float(value, field_name)
    return result


def _custom_line_points(value: Any, closed_loop: bool) -> list[dict[str, float]]:
    if not isinstance(value, list):
        raise ValueError("points must be an array")
    if len(value) > CUSTOM_LINE_MAX_POINTS:
        raise ValueError(f"custom line supports at most {CUSTOM_LINE_MAX_POINTS} points")
    minimum = 3 if closed_loop else 2
    if len(value) < minimum:
        raise ValueError(f"custom line needs at least {minimum} points")

    points: list[dict[str, float]] = []
    for index, row in enumerate(value):
        if not isinstance(row, dict):
            raise ValueError(f"points[{index}] must be an object")
        try:
            x_m = float(row.get("x_m"))
            y_m = float(row.get("y_m"))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"points[{index}] x_m and y_m must be finite numbers") from exc
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            raise ValueError(f"points[{index}] x_m and y_m must be finite numbers")
        point = {"x_m": x_m, "y_m": y_m}
        # speed_mps was the authoring format before section profiles were
        # introduced.  Keep accepting it so old manifests remain readable,
        # but new manifests only persist x/y here.
        if row.get("speed_mps") is not None:
            point["speed_mps"] = _nonnegative_finite_float(
                row.get("speed_mps"),
                f"points[{index}].speed_mps",
            )
        points.append(point)

    buckets: dict[tuple[int, int], list[tuple[int, dict[str, float]]]] = {}
    for index, point in enumerate(points):
        try:
            bucket_x = math.floor(point["x_m"] / CUSTOM_LINE_POINT_EPSILON_M)
            bucket_y = math.floor(point["y_m"] / CUSTOM_LINE_POINT_EPSILON_M)
        except OverflowError as exc:
            raise ValueError(f"points[{index}] coordinates are outside the supported range") from exc
        for offset_x in (-1, 0, 1):
            for offset_y in (-1, 0, 1):
                for previous_index, previous in buckets.get((bucket_x + offset_x, bucket_y + offset_y), []):
                    distance = math.hypot(
                        point["x_m"] - previous["x_m"],
                        point["y_m"] - previous["y_m"],
                    )
                    if not math.isfinite(distance):
                        raise ValueError("custom line coordinates produce a non-finite segment length")
                    if distance <= CUSTOM_LINE_POINT_EPSILON_M:
                        raise ValueError(f"points[{index}] duplicates points[{previous_index}]")
        buckets.setdefault((bucket_x, bucket_y), []).append((index, point))
    if closed_loop:
        closing_distance = math.hypot(
            points[0]["x_m"] - points[-1]["x_m"],
            points[0]["y_m"] - points[-1]["y_m"],
        )
        if not math.isfinite(closing_distance) or closing_distance <= CUSTOM_LINE_POINT_EPSILON_M:
            raise ValueError("closed custom line has a zero-length closing segment")
    return points


def _primary_lane_geometry(map_dir: Path) -> dict[str, Any] | None:
    hd_map_path = map_dir / f"{map_dir.name}_hd_map.yaml"
    hd_map, _ = _read_hd_map(hd_map_path)
    primary_lane_id = str(hd_map.get("primary_lane_id") or "")
    lanes = hd_map.get("lanes") if isinstance(hd_map.get("lanes"), list) else []
    primary = next(
        (
            lane
            for lane in lanes
            if isinstance(lane, dict) and (str(lane.get("id") or "") == primary_lane_id or lane.get("primary"))
        ),
        None,
    )
    return primary if isinstance(primary, dict) else None


def _point_inside_polygon(point: list[float], polygon: list[list[float]]) -> bool:
    if len(polygon) < 3:
        return False
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        if _segment_distance(point, start, end) <= CUSTOM_LINE_POINT_EPSILON_M:
            return True

    inside = False
    x, y = point
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < intersection_x:
                inside = not inside
        previous = current
    return inside


def _custom_line_geometry_validation(
    map_dir: Path,
    points: list[dict[str, float]],
    closed_loop: bool,
) -> dict[str, Any]:
    lane = _primary_lane_geometry(map_dir)
    if lane is None:
        return {
            "valid": False,
            "issue": "primary lane geometry is required to validate a custom line",
            "min_clearance_m": None,
            "containment_checked": False,
        }
    left_bound = _as_points(lane.get("left_bound", []))
    right_bound = _as_points(lane.get("right_bound", []))
    polygon = left_bound + list(reversed(right_bound))
    if len(left_bound) < 2 or len(right_bound) < 2 or len(polygon) < 3:
        return {
            "valid": False,
            "issue": "primary lane needs usable left and right bounds",
            "min_clearance_m": None,
            "containment_checked": False,
        }

    area_twice = sum(
        polygon[index][0] * polygon[(index + 1) % len(polygon)][1]
        - polygon[(index + 1) % len(polygon)][0] * polygon[index][1]
        for index in range(len(polygon))
    )
    if not math.isfinite(area_twice) or abs(area_twice) <= CUSTOM_LINE_POINT_EPSILON_M:
        return {
            "valid": False,
            "issue": "primary lane bounds do not form a usable corridor",
            "min_clearance_m": None,
            "containment_checked": False,
        }

    closed_lane = bool(lane.get("closed_loop", True))
    min_clearance = math.inf
    def validate_sample(xy: list[float], label: str) -> dict[str, Any] | None:
        nonlocal min_clearance
        if not _point_inside_polygon(xy, polygon):
            return {
                "valid": False,
                "issue": f"{label} is outside the primary lane bounds",
                "min_clearance_m": None if min_clearance == math.inf else min_clearance,
                "containment_checked": True,
            }
        clearance = min(
            _nearest_distance(xy, left_bound, closed_lane),
            _nearest_distance(xy, right_bound, closed_lane),
        )
        min_clearance = min(min_clearance, clearance)
        return None

    for index, point in enumerate(points):
        issue = validate_sample([point["x_m"], point["y_m"]], f"points[{index}]")
        if issue is not None:
            return issue

    segment_count = len(points) if closed_loop else len(points) - 1
    sample_count = 0
    for index in range(segment_count):
        start = points[index]
        end = points[(index + 1) % len(points)]
        distance = math.hypot(end["x_m"] - start["x_m"], end["y_m"] - start["y_m"])
        steps = max(1, math.ceil(distance / CUSTOM_LINE_CONTAINMENT_SAMPLE_STEP_M))
        sample_count += max(0, steps - 1)
        if sample_count > CUSTOM_LINE_MAX_CONTAINMENT_SAMPLES:
            return {
                "valid": False,
                "issue": "custom line requires too many lane-containment samples",
                "min_clearance_m": None if min_clearance == math.inf else min_clearance,
                "containment_checked": True,
            }
        for sample_index in range(1, steps):
            ratio = sample_index / steps
            xy = [
                start["x_m"] + (end["x_m"] - start["x_m"]) * ratio,
                start["y_m"] + (end["y_m"] - start["y_m"]) * ratio,
            ]
            issue = validate_sample(xy, f"segment[{index}] sample[{sample_index}]")
            if issue is not None:
                return issue
    return {
        "valid": True,
        "issue": "",
        "min_clearance_m": min_clearance if math.isfinite(min_clearance) else None,
        "containment_checked": True,
    }


def _custom_line_content_hash(
    points: list[dict[str, float]],
    closed_loop: bool,
    default_speed_mps: float | None = None,
    section_speeds_mps: dict[str, float] | None = None,
    constraints: dict[str, float] | None = None,
) -> str:
    geometry = [{"x_m": point["x_m"], "y_m": point["y_m"]} for point in points]
    content: dict[str, Any] = {"closed_loop": closed_loop, "points": geometry}
    if default_speed_mps is not None:
        content["default_speed_mps"] = default_speed_mps
        content["section_speeds_mps"] = dict(sorted((section_speeds_mps or {}).items()))
    if constraints is not None:
        content["constraints"] = constraints
    serialized = json.dumps(
        content,
        ensure_ascii=True,
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(serialized.encode("utf-8")).hexdigest()


def _custom_line_hd_layout(map_dir: Path) -> dict[str, Any]:
    hd_map_path = map_dir / f"{map_dir.name}_hd_map.yaml"
    hd_map, _ = _read_hd_map(hd_map_path)
    primary_lane_id = str(hd_map.get("primary_lane_id") or "")
    lanes = hd_map.get("lanes") if isinstance(hd_map.get("lanes"), list) else []
    lane = next(
        (
            item
            for item in lanes
            if isinstance(item, dict)
            and (str(item.get("id") or "") == primary_lane_id or item.get("primary"))
        ),
        None,
    )
    if not isinstance(lane, dict):
        raise ValueError("primary lane geometry is required for custom line section speeds")
    primary_lane_id = str(lane.get("id") or primary_lane_id)
    centerline = _as_points(lane.get("centerline", []))
    closed_loop = bool(lane.get("closed_loop", True))
    minimum = 3 if closed_loop else 2
    if len(centerline) < minimum:
        raise ValueError("primary lane centerline is incomplete")

    gate_by_id: dict[str, dict[str, Any]] = {}
    for raw_gate in hd_map.get("section_gates", []) or []:
        if not isinstance(raw_gate, dict) or str(raw_gate.get("lane_id") or "") != primary_lane_id:
            continue
        gate_id = str(raw_gate.get("id") or "")
        if not gate_id:
            raise ValueError("primary lane section gate has no id")
        if gate_id in gate_by_id:
            raise ValueError(f"duplicate primary lane section gate id: {gate_id}")
        line = _as_points(raw_gate.get("line", []))[:2]
        if len(line) != 2 or math.hypot(line[1][0] - line[0][0], line[1][1] - line[0][1]) <= CUSTOM_LINE_POINT_EPSILON_M:
            raise ValueError(f"section gate {gate_id} needs a usable two-point line")
        gate_by_id[gate_id] = {
            "id": gate_id,
            "lane_id": primary_lane_id,
            "s_m": _as_float(raw_gate.get("s_m"), 0.0),
            "line": line,
        }

    sections: list[dict[str, Any]] = []
    section_ids: set[str] = set()
    section_pairs: set[tuple[str, str]] = set()
    for raw_section in hd_map.get("sections", []) or []:
        if not isinstance(raw_section, dict) or str(raw_section.get("lane_id") or "") != primary_lane_id:
            continue
        section_id = str(raw_section.get("id") or "")
        if not section_id:
            raise ValueError("primary lane section has no id")
        if section_id in section_ids:
            raise ValueError(f"duplicate primary lane section id: {section_id}")
        section_ids.add(section_id)
        start_gate_id = str(raw_section.get("start_gate_id") or "")
        end_gate_id = str(raw_section.get("end_gate_id") or "")
        if start_gate_id == end_gate_id or start_gate_id not in gate_by_id or end_gate_id not in gate_by_id:
            raise ValueError(f"section {section_id} references missing or identical gates")
        section_pair = (start_gate_id, end_gate_id)
        if section_pair in section_pairs:
            raise ValueError(
                f"duplicate primary lane section gate pair: {start_gate_id} -> {end_gate_id}"
            )
        section_pairs.add(section_pair)
        sections.append(
            {
                "id": section_id,
                "lane_id": primary_lane_id,
                "start_gate_id": start_gate_id,
                "end_gate_id": end_gate_id,
                "start_s_m": _as_float(raw_section.get("start_s_m"), gate_by_id[start_gate_id]["s_m"]),
                "end_s_m": _as_float(raw_section.get("end_s_m"), gate_by_id[end_gate_id]["s_m"]),
                "wrap": bool(raw_section.get("wrap", False)),
                "speed_override_mps": raw_section.get("speed_override_mps"),
            }
        )

    layout_source = {
        "primary_lane_id": primary_lane_id,
        "closed_loop": closed_loop,
        "centerline": centerline,
        "gates": [gate_by_id[key] for key in sorted(gate_by_id)],
        "sections": [
            {
                key: section[key]
                for key in ("id", "lane_id", "start_gate_id", "end_gate_id", "start_s_m", "end_s_m", "wrap")
            }
            for section in sorted(sections, key=lambda item: str(item["id"]))
        ],
    }
    fingerprint = hashlib.sha256(
        json.dumps(layout_source, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return {
        "primary_lane_id": primary_lane_id,
        "closed_loop": closed_loop,
        "centerline": centerline,
        "gates": gate_by_id,
        "sections": sections,
        "section_ids": section_ids,
        "fingerprint": fingerprint,
        "hd_map_sha256": _sha256_file(hd_map_path),
    }


def _custom_line_section_speeds(
    value: Any,
    layout: dict[str, Any],
    *,
    allow_legacy_zero: bool = False,
) -> dict[str, float]:
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError("section_speeds_mps must be an object keyed by section id")
    known_ids = set(layout.get("section_ids") or set())
    result: dict[str, float] = {}
    for raw_id, raw_speed in value.items():
        section_id = str(raw_id)
        if section_id not in known_ids:
            raise ValueError(f"section_speeds_mps contains unknown primary lane section id: {section_id}")
        speed = _nonnegative_finite_float(raw_speed, f"section_speeds_mps.{section_id}")
        if not allow_legacy_zero and speed < CUSTOM_LINE_MIN_SECTION_SPEED_MPS:
            raise ValueError(
                f"section_speeds_mps.{section_id} must be at least "
                f"{CUSTOM_LINE_MIN_SECTION_SPEED_MPS:g} m/s"
            )
        result[section_id] = speed
    return dict(sorted(result.items()))


def _custom_line_default_speed(value: Any, *, allow_legacy_zero: bool = False) -> float:
    speed = _nonnegative_finite_float(value, "default_speed_mps")
    if not allow_legacy_zero and speed < CUSTOM_LINE_MIN_SECTION_SPEED_MPS:
        raise ValueError(f"default_speed_mps must be at least {CUSTOM_LINE_MIN_SECTION_SPEED_MPS:g} m/s")
    return speed


def _custom_polyline_stations(
    points: list[dict[str, float]],
    closed_loop: bool,
) -> tuple[list[float], list[float], float]:
    stations = [0.0]
    segment_count = len(points) if closed_loop else len(points) - 1
    segment_lengths: list[float] = []
    for index in range(segment_count):
        following = (index + 1) % len(points)
        distance = math.hypot(
            points[following]["x_m"] - points[index]["x_m"],
            points[following]["y_m"] - points[index]["y_m"],
        )
        if not math.isfinite(distance) or distance <= CUSTOM_LINE_POINT_EPSILON_M:
            raise ValueError(f"custom line segment {index} has a zero or non-finite length")
        segment_lengths.append(distance)
        if following != 0:
            stations.append(stations[-1] + distance)
    total_length = sum(segment_lengths)
    if not math.isfinite(total_length) or total_length <= CUSTOM_LINE_POINT_EPSILON_M:
        raise ValueError("custom line length is invalid")
    return stations, segment_lengths, total_length


def _segment_intersection_parameter(
    start: dict[str, float],
    end: dict[str, float],
    gate_start: list[float],
    gate_end: list[float],
) -> float | None:
    rx = end["x_m"] - start["x_m"]
    ry = end["y_m"] - start["y_m"]
    sx = gate_end[0] - gate_start[0]
    sy = gate_end[1] - gate_start[1]
    qpx = gate_start[0] - start["x_m"]
    qpy = gate_start[1] - start["y_m"]
    denominator = rx * sy - ry * sx
    scale = max(1.0, math.hypot(rx, ry) * math.hypot(sx, sy))
    tolerance = 1.0e-10 * scale
    if abs(denominator) <= tolerance:
        # A path running along a gate has no unique section transition.
        if abs(qpx * ry - qpy * rx) <= tolerance:
            raise ValueError("custom line overlaps a section gate")
        return None
    t = (qpx * sy - qpy * sx) / denominator
    u = (qpx * ry - qpy * rx) / denominator
    bound_tolerance = 1.0e-9
    if -bound_tolerance <= t <= 1.0 + bound_tolerance and -bound_tolerance <= u <= 1.0 + bound_tolerance:
        return max(0.0, min(1.0, t))
    return None


def _custom_line_gate_stations(
    points: list[dict[str, float]],
    closed_loop: bool,
    layout: dict[str, Any],
) -> tuple[dict[str, float], list[float], list[float], float]:
    author_stations, segment_lengths, total_length = _custom_polyline_stations(points, closed_loop)
    gate_stations: dict[str, float] = {}
    segment_count = len(segment_lengths)
    for gate_id, gate in layout["gates"].items():
        raw_stations: list[float] = []
        for index in range(segment_count):
            following = (index + 1) % len(points)
            parameter = _segment_intersection_parameter(
                points[index],
                points[following],
                gate["line"][0],
                gate["line"][1],
            )
            if parameter is None:
                continue
            station = author_stations[index] + segment_lengths[index] * parameter
            if closed_loop and (station >= total_length - 1.0e-8 or station <= 1.0e-8):
                station = 0.0
            if not any(abs(station - previous) <= 1.0e-7 for previous in raw_stations):
                raw_stations.append(station)
        if len(raw_stations) != 1:
            raise ValueError(
                f"section gate {gate_id} must intersect the custom line exactly once; found {len(raw_stations)}"
            )
        gate_stations[gate_id] = raw_stations[0]

    if layout["sections"]:
        ordered_gates = sorted(gate_stations, key=lambda gate_id: gate_stations[gate_id])
        actual_pairs = {
            (str(section["start_gate_id"]), str(section["end_gate_id"]))
            for section in layout["sections"]
        }
        if closed_loop:
            expected_pairs = {
                (ordered_gates[index], ordered_gates[(index + 1) % len(ordered_gates)])
                for index in range(len(ordered_gates))
            }
        else:
            expected_pairs = {
                (ordered_gates[index], ordered_gates[index + 1])
                for index in range(max(0, len(ordered_gates) - 1))
            }
        if actual_pairs != expected_pairs:
            raise ValueError("section gate order is inconsistent with the custom line direction")
    return gate_stations, author_stations, segment_lengths, total_length


def _signed_area_twice_xy(points: list[list[float]]) -> float:
    return sum(
        points[index][0] * points[(index + 1) % len(points)][1]
        - points[(index + 1) % len(points)][0] * points[index][1]
        for index in range(len(points))
    )


def _validate_two_gate_closed_direction(
    points: list[dict[str, float]],
    layout: dict[str, Any],
) -> None:
    custom_xy = [[point["x_m"], point["y_m"]] for point in points]
    custom_area = _signed_area_twice_xy(custom_xy)
    center_area = _signed_area_twice_xy(layout["centerline"])
    coordinate_scale = max(
        1.0,
        *(abs(value) for point in [*custom_xy, *layout["centerline"]] for value in point[:2]),
    )
    area_tolerance = 1.0e-12 * coordinate_scale * coordinate_scale * max(
        len(custom_xy),
        len(layout["centerline"]),
    )
    if (
        not math.isfinite(custom_area)
        or not math.isfinite(center_area)
        or abs(custom_area) <= area_tolerance
        or abs(center_area) <= area_tolerance
    ):
        raise ValueError("custom line direction is ambiguous relative to the primary lane")
    if (custom_area < 0.0) != (center_area < 0.0):
        raise ValueError("custom line direction is opposite relative to the primary lane")


def _custom_line_speed_context(
    points: list[dict[str, float]],
    closed_loop: bool,
    layout: dict[str, Any],
) -> dict[str, Any]:
    if layout["sections"] and bool(layout["closed_loop"]) != closed_loop:
        raise ValueError("custom line closed_loop must match the primary lane topology")
    if layout["sections"] and closed_loop and len(layout["gates"]) == 2:
        _validate_two_gate_closed_direction(points, layout)
    if not layout["sections"]:
        stations, segment_lengths, total_length = _custom_polyline_stations(points, closed_loop)
        return {
            "gate_stations": {},
            "author_stations": stations,
            "segment_lengths": segment_lengths,
            "total_length": total_length,
            "sections": [],
        }
    gate_stations, author_stations, segment_lengths, total_length = _custom_line_gate_stations(
        points,
        closed_loop,
        layout,
    )
    sections: list[dict[str, Any]] = []
    for section in layout["sections"]:
        start_station = gate_stations[str(section["start_gate_id"])]
        end_station = gate_stations[str(section["end_gate_id"])]
        if not closed_loop and end_station <= start_station + CUSTOM_LINE_POINT_EPSILON_M:
            raise ValueError(f"section {section['id']} has reversed or empty custom-line bounds")
        if closed_loop and abs(end_station - start_station) <= CUSTOM_LINE_POINT_EPSILON_M:
            raise ValueError(f"section {section['id']} has empty custom-line bounds")
        sections.append({**section, "custom_start_s_m": start_station, "custom_end_s_m": end_station})
    return {
        "gate_stations": gate_stations,
        "author_stations": author_stations,
        "segment_lengths": segment_lengths,
        "total_length": total_length,
        "sections": sections,
    }


def _section_for_custom_station(station: float, context: dict[str, Any], closed_loop: bool) -> str | None:
    total_length = float(context["total_length"])
    if closed_loop:
        station %= total_length
    for section in context["sections"]:
        start = float(section["custom_start_s_m"])
        end = float(section["custom_end_s_m"])
        if closed_loop and end < start:
            inside = station >= start or station < end
        else:
            inside = start <= station < end
            if not closed_loop and abs(station - end) <= CUSTOM_LINE_POINT_EPSILON_M and abs(end - total_length) <= 1.0e-8:
                inside = True
        if inside:
            return str(section["id"])
    return None


def _legacy_custom_line_speed_profile(
    points: list[dict[str, float]],
    context: dict[str, Any],
    closed_loop: bool,
) -> tuple[float, dict[str, float]]:
    legacy_speeds: list[float] = []
    for index, point in enumerate(points):
        if "speed_mps" not in point:
            raise ValueError("legacy custom line points must include speed_mps")
        legacy_speeds.append(_nonnegative_finite_float(point["speed_mps"], f"points[{index}].speed_mps"))
    default_speed = min(legacy_speeds)
    grouped: dict[str, list[float]] = {}
    for station, speed in zip(context["author_stations"], legacy_speeds):
        section_id = _section_for_custom_station(station, context, closed_loop)
        if section_id is not None:
            grouped.setdefault(section_id, []).append(speed)
    section_speeds = {
        str(section["id"]): min(grouped.get(str(section["id"]), legacy_speeds))
        for section in context["sections"]
    }
    return default_speed, dict(sorted(section_speeds.items()))


def _augment_custom_line_at_gates(
    points: list[dict[str, float]],
    closed_loop: bool,
    context: dict[str, Any],
) -> list[dict[str, Any]]:
    augmented: list[dict[str, Any]] = [
        {
            "x_m": point["x_m"],
            "y_m": point["y_m"],
            "custom_s_m": context["author_stations"][index],
            "author_index": index,
            "gate_ids": [],
        }
        for index, point in enumerate(points)
    ]
    total_length = float(context["total_length"])
    for gate_id, station in context["gate_stations"].items():
        existing = next((item for item in augmented if abs(float(item["custom_s_m"]) - station) <= 1.0e-7), None)
        if existing is not None:
            existing["gate_ids"].append(gate_id)
            continue
        segment_index = 0
        for index, start_station in enumerate(context["author_stations"]):
            end_station = (
                context["author_stations"][index + 1]
                if index + 1 < len(points)
                else total_length
            )
            if start_station < station < end_station:
                segment_index = index
                break
        following = (segment_index + 1) % len(points)
        start_station = context["author_stations"][segment_index]
        ratio = (station - start_station) / context["segment_lengths"][segment_index]
        augmented.append(
            {
                "x_m": points[segment_index]["x_m"] + (points[following]["x_m"] - points[segment_index]["x_m"]) * ratio,
                "y_m": points[segment_index]["y_m"] + (points[following]["y_m"] - points[segment_index]["y_m"]) * ratio,
                "custom_s_m": station,
                "author_index": None,
                "gate_ids": [gate_id],
            }
        )
    augmented.sort(key=lambda item: float(item["custom_s_m"]))
    densified: list[dict[str, Any]] = []
    interval_count = len(augmented) if closed_loop else len(augmented) - 1
    for index in range(interval_count):
        start = augmented[index]
        end = augmented[(index + 1) % len(augmented)]
        densified.append(start)
        distance = math.hypot(end["x_m"] - start["x_m"], end["y_m"] - start["y_m"])
        steps = max(1, math.ceil(distance / CUSTOM_LINE_SPEED_SAMPLE_STEP_M))
        if len(densified) + steps - 1 > CUSTOM_LINE_MAX_COMPILED_POINTS:
            raise ValueError(
                f"compiled custom line supports at most {CUSTOM_LINE_MAX_COMPILED_POINTS} points"
            )
        start_station = float(start["custom_s_m"])
        end_station = float(end["custom_s_m"])
        if closed_loop and index == len(augmented) - 1:
            end_station = float(context["total_length"])
        for step in range(1, steps):
            ratio = step / steps
            densified.append(
                {
                    "x_m": start["x_m"] + (end["x_m"] - start["x_m"]) * ratio,
                    "y_m": start["y_m"] + (end["y_m"] - start["y_m"]) * ratio,
                    "custom_s_m": start_station + (end_station - start_station) * ratio,
                    "author_index": None,
                    "gate_ids": [],
                }
            )
    if not closed_loop:
        densified.append(augmented[-1])
    if len(densified) > CUSTOM_LINE_MAX_COMPILED_POINTS:
        raise ValueError(f"compiled custom line supports at most {CUSTOM_LINE_MAX_COMPILED_POINTS} points")
    return densified


def _requested_section_speed(
    station: float,
    context: dict[str, Any],
    closed_loop: bool,
    default_speed_mps: float,
    section_speeds_mps: dict[str, float],
) -> float:
    section_id = _section_for_custom_station(station, context, closed_loop)
    return section_speeds_mps.get(section_id, default_speed_mps) if section_id is not None else default_speed_mps


def _apply_custom_speed_envelope(
    trajectory: list[dict[str, float]],
    closed_loop: bool,
    constraints: dict[str, float],
) -> list[float]:
    speeds: list[float] = []
    for row in trajectory:
        curvature = abs(row["kappa_radpm"])
        curvature_cap = math.inf
        if curvature > 1.0e-12:
            curvature_cap = math.sqrt(constraints["lateral_accel_limit_mps2"] / curvature)
        speeds.append(min(row["vx_mps"], constraints["max_speed_mps"], curvature_cap))

    edges = [(index, index + 1) for index in range(len(speeds) - 1)]
    distances = [trajectory[index + 1]["s_m"] - trajectory[index]["s_m"] for index in range(len(speeds) - 1)]
    if closed_loop:
        closing_distance = math.hypot(
            trajectory[0]["x_m"] - trajectory[-1]["x_m"],
            trajectory[0]["y_m"] - trajectory[-1]["y_m"],
        )
        edges.append((len(speeds) - 1, 0))
        distances.append(closing_distance)

    tolerance = 1.0e-12
    for _ in range(max(2, len(speeds) * 2 + 2)):
        changed = False
        for (start, end), distance in zip(edges, distances):
            cap = math.sqrt(max(0.0, speeds[start] * speeds[start] + 2.0 * constraints["accel_limit_mps2"] * distance))
            if speeds[end] > cap + tolerance:
                speeds[end] = cap
                changed = True
        for (start, end), distance in reversed(list(zip(edges, distances))):
            cap = math.sqrt(max(0.0, speeds[end] * speeds[end] + 2.0 * constraints["decel_limit_mps2"] * distance))
            if speeds[start] > cap + tolerance:
                speeds[start] = cap
                changed = True
        if not changed:
            break
    return speeds


def _compile_custom_line(
    map_dir: Path,
    points: list[dict[str, float]],
    closed_loop: bool,
    default_speed_mps: float,
    section_speeds_mps: dict[str, float],
    constraints: dict[str, float],
) -> tuple[list[dict[str, float]], list[dict[str, float]], dict[str, Any], dict[str, Any]]:
    layout = _custom_line_hd_layout(map_dir)
    default_speed_mps = _custom_line_default_speed(default_speed_mps, allow_legacy_zero=True)
    section_speeds_mps = _custom_line_section_speeds(
        section_speeds_mps,
        layout,
        allow_legacy_zero=True,
    )
    context = _custom_line_speed_context(points, closed_loop, layout)
    augmented = _augment_custom_line_at_gates(points, closed_loop, context)
    target_by_section = {
        str(section["id"]): section_speeds_mps.get(str(section["id"]), default_speed_mps)
        for section in context["sections"]
    }
    requested: list[dict[str, float]] = []
    for item in augmented:
        target = _requested_section_speed(
            float(item["custom_s_m"]),
            context,
            closed_loop,
            default_speed_mps,
            section_speeds_mps,
        )
        if item["gate_ids"]:
            adjacent: list[float] = []
            for section in context["sections"]:
                if section["start_gate_id"] in item["gate_ids"] or section["end_gate_id"] in item["gate_ids"]:
                    adjacent.append(target_by_section[str(section["id"])])
            if not closed_loop and len(adjacent) < 2:
                adjacent.append(default_speed_mps)
            target = min([target, *adjacent])
        requested.append({"x_m": item["x_m"], "y_m": item["y_m"], "speed_mps": target})

    geometry_seed = [
        {"x_m": point["x_m"], "y_m": point["y_m"], "speed_mps": 0.0}
        for point in requested
    ]
    initial_trajectory = _derive_custom_trajectory(geometry_seed, closed_loop)
    for row, point in zip(initial_trajectory, requested):
        row["vx_mps"] = point["speed_mps"]
    safe_speeds = _apply_custom_speed_envelope(initial_trajectory, closed_loop, constraints)
    if any(not math.isfinite(speed * speed) for speed in safe_speeds):
        raise ValueError("compiled custom line speed is outside the supported numeric range")
    compiled_points = [
        {"x_m": point["x_m"], "y_m": point["y_m"], "speed_mps": safe_speeds[index]}
        for index, point in enumerate(requested)
    ]
    trajectory = _derive_custom_trajectory(compiled_points, closed_loop)
    geometry = _custom_line_geometry_validation(map_dir, points, closed_loop)
    if not geometry["valid"]:
        return trajectory, [], {**geometry, "speed_adjusted": False}, {**layout, "context": context}
    feasibility = _custom_line_feasibility_validation(trajectory, constraints)
    validation = {
        **geometry,
        **feasibility,
        "speed_adjusted": any(safe + 1.0e-9 < requested[index]["speed_mps"] for index, safe in enumerate(safe_speeds)),
        "requested_max_speed_mps": max(point["speed_mps"] for point in requested),
    }
    author_points: list[dict[str, float] | None] = [None] * len(points)
    for index, item in enumerate(augmented):
        author_index = item["author_index"]
        if author_index is not None:
            author_points[author_index] = {
                "x_m": points[author_index]["x_m"],
                "y_m": points[author_index]["y_m"],
                "speed_mps": safe_speeds[index],
            }
    if any(point is None for point in author_points):
        raise ValueError("failed to map compiled speeds back to editable points")
    return trajectory, [point for point in author_points if point is not None], validation, {**layout, "context": context}


def _derive_custom_trajectory(
    points: list[dict[str, float]],
    closed_loop: bool,
) -> list[dict[str, float]]:
    count = len(points)
    s_values = [0.0]
    for index in range(1, count):
        distance = math.hypot(
            points[index]["x_m"] - points[index - 1]["x_m"],
            points[index]["y_m"] - points[index - 1]["y_m"],
        )
        if not math.isfinite(distance) or distance <= CUSTOM_LINE_POINT_EPSILON_M:
            raise ValueError(f"points[{index - 1}] to points[{index}] has a zero or non-finite length")
        next_s = s_values[-1] + distance
        if not math.isfinite(next_s) or next_s <= s_values[-1]:
            raise ValueError("derived s values are not finite and strictly increasing")
        s_values.append(next_s)

    if closed_loop:
        closing_distance = math.hypot(
            points[0]["x_m"] - points[-1]["x_m"],
            points[0]["y_m"] - points[-1]["y_m"],
        )
        if not math.isfinite(closing_distance) or closing_distance <= CUSTOM_LINE_POINT_EPSILON_M:
            raise ValueError("closed custom line has a zero or non-finite closing segment")

    psi_values: list[float] = []
    kappa_values: list[float] = []
    for index in range(count):
        if closed_loop:
            previous = points[(index - 1) % count]
            following = points[(index + 1) % count]
        elif index == 0:
            previous = points[0]
            following = points[1]
        elif index == count - 1:
            previous = points[count - 2]
            following = points[count - 1]
        else:
            previous = points[index - 1]
            following = points[index + 1]

        tangent_x = following["x_m"] - previous["x_m"]
        tangent_y = following["y_m"] - previous["y_m"]
        tangent_length = math.hypot(tangent_x, tangent_y)
        if not math.isfinite(tangent_length) or tangent_length <= CUSTOM_LINE_POINT_EPSILON_M:
            raise ValueError(f"points[{index}] does not have a valid tangent")
        psi_values.append(math.atan2(tangent_y, tangent_x))

        if not closed_loop and index in (0, count - 1):
            kappa_values.append(0.0)
            continue
        current = points[index]
        a = math.hypot(current["x_m"] - previous["x_m"], current["y_m"] - previous["y_m"])
        b = math.hypot(following["x_m"] - current["x_m"], following["y_m"] - current["y_m"])
        c = math.hypot(following["x_m"] - previous["x_m"], following["y_m"] - previous["y_m"])
        denominator = a * b * c
        if not math.isfinite(denominator) or denominator <= CUSTOM_LINE_POINT_EPSILON_M:
            raise ValueError(f"points[{index}] does not have a valid curvature neighborhood")
        cross = (
            (current["x_m"] - previous["x_m"]) * (following["y_m"] - current["y_m"])
            - (current["y_m"] - previous["y_m"]) * (following["x_m"] - current["x_m"])
        )
        kappa_values.append(2.0 * cross / denominator)

    acceleration_values: list[float] = []
    for index in range(count):
        if index + 1 < count:
            following_index = index + 1
            distance = s_values[following_index] - s_values[index]
        elif closed_loop:
            following_index = 0
            distance = math.hypot(
                points[0]["x_m"] - points[-1]["x_m"],
                points[0]["y_m"] - points[-1]["y_m"],
            )
        else:
            acceleration_values.append(0.0)
            continue
        acceleration = (
            points[following_index]["speed_mps"] ** 2 - points[index]["speed_mps"] ** 2
        ) / (2.0 * distance)
        if not math.isfinite(acceleration):
            raise ValueError("custom speeds produce a non-finite longitudinal acceleration")
        acceleration_values.append(acceleration)

    trajectory: list[dict[str, float]] = []
    for index, point in enumerate(points):
        row = {
            "s_m": s_values[index],
            "x_m": point["x_m"],
            "y_m": point["y_m"],
            "psi_rad": psi_values[index],
            "kappa_radpm": kappa_values[index],
            "vx_mps": point["speed_mps"],
            "ax_mps2": acceleration_values[index],
        }
        if not all(math.isfinite(value) for value in row.values()):
            raise ValueError("derived trajectory contains a non-finite value")
        trajectory.append(row)
    return trajectory


def _custom_line_feasibility_validation(
    trajectory: list[dict[str, float]],
    constraints: dict[str, float],
) -> dict[str, Any]:
    max_speed = max(row["vx_mps"] for row in trajectory)
    max_lateral_accel = max(abs(row["vx_mps"] ** 2 * row["kappa_radpm"]) for row in trajectory)
    max_accel = max(max(0.0, row["ax_mps2"]) for row in trajectory)
    max_decel = max(max(0.0, -row["ax_mps2"]) for row in trajectory)
    metrics = {
        "max_speed_mps": max_speed,
        "max_lateral_accel_mps2": max_lateral_accel,
        "max_accel_mps2": max_accel,
        "max_decel_mps2": max_decel,
    }
    checks = (
        (max_speed, constraints["max_speed_mps"], "custom speed exceeds max_speed_mps"),
        (
            max_lateral_accel,
            constraints["lateral_accel_limit_mps2"],
            "custom speed and curvature exceed lateral_accel_limit_mps2",
        ),
        (max_accel, constraints["accel_limit_mps2"], "custom speed profile exceeds accel_limit_mps2"),
        (max_decel, constraints["decel_limit_mps2"], "custom speed profile exceeds decel_limit_mps2"),
    )
    for actual, limit, issue in checks:
        if not math.isfinite(actual):
            return {"valid": False, "issue": "custom speed feasibility is non-finite", **metrics}
        if actual > limit + 1.0e-9:
            return {"valid": False, "issue": f"{issue}: {actual:.6g} > {limit:.6g}", **metrics}
    return {"valid": True, "issue": "", **metrics}


def _validate_custom_line(
    map_dir: Path,
    points: list[dict[str, float]],
    closed_loop: bool,
    constraints: dict[str, float],
) -> tuple[list[dict[str, float]], dict[str, Any]]:
    trajectory = _derive_custom_trajectory(points, closed_loop)
    geometry = _custom_line_geometry_validation(map_dir, points, closed_loop)
    if not geometry["valid"]:
        return trajectory, geometry
    feasibility = _custom_line_feasibility_validation(trajectory, constraints)
    return trajectory, {**geometry, **feasibility}


def _write_custom_trajectory(path: Path, trajectory: list[dict[str, float]]) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            handle.write("# s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2\n")
            writer = csv.writer(handle, delimiter=";", lineterminator="\n")
            for row in trajectory:
                writer.writerow(
                    [
                        _fmt_float(row["s_m"]),
                        _fmt_float(row["x_m"]),
                        _fmt_float(row["y_m"]),
                        _fmt_float(row["psi_rad"]),
                        _fmt_float(row["kappa_radpm"]),
                        _fmt_float(row["vx_mps"]),
                        _fmt_float(row["ax_mps2"]),
                    ]
                )
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass
    trajectory_hash = _sha256_file(path)
    if not trajectory_hash:
        raise OSError(f"failed to hash trajectory: {path}")
    return trajectory_hash


def _source_closed_loop(map_dir: Path) -> bool:
    lane = _primary_lane_geometry(map_dir)
    return bool(lane.get("closed_loop", True)) if lane is not None else True


def _read_custom_line_source(
    map_dir: Path,
    source_type: str,
    default_speed_mps: float,
) -> tuple[Path, list[dict[str, float]]]:
    if source_type == "centerline":
        source_path = map_dir / f"{map_dir.name}_hd_map_centerline.csv"
        delimiter = ","
        x_index, y_index = 0, 1
    else:
        source_path = map_dir / f"{map_dir.name}_raceline.csv"
        delimiter = ";"
        x_index, y_index = 1, 2
    if not source_path.exists() or not source_path.is_file():
        raise FileNotFoundError(f"{source_type} source not found: {source_path}")

    points: list[dict[str, float]] = []
    with source_path.open("r", encoding="utf-8", errors="strict", newline="") as handle:
        reader = csv.reader(handle, delimiter=delimiter)
        for line_number, row in enumerate(reader, start=1):
            if not row or not any(value.strip() for value in row) or row[0].strip().startswith("#"):
                continue
            try:
                x_m = float(row[x_index])
                y_m = float(row[y_index])
            except (IndexError, ValueError) as exc:
                raise ValueError(f"invalid {source_type} row {line_number}") from exc
            if not math.isfinite(x_m) or not math.isfinite(y_m):
                raise ValueError(f"invalid non-finite coordinate in {source_type} row {line_number}")
            points.append({"x_m": x_m, "y_m": y_m, "speed_mps": default_speed_mps})
    if not points:
        raise ValueError(f"{source_type} source contains no valid points")
    return source_path, points


def _resolve_custom_line_map(config: ConsoleConfig, payload: dict[str, Any]) -> Path:
    map_dir_value = str(payload.get("map_dir") or "")
    if not map_dir_value:
        raise ValueError("map_dir is required")
    map_dir = resolve_allowed_path(config, map_dir_value)
    if not map_dir.exists() or not map_dir.is_dir():
        raise FileNotFoundError(f"map folder not found: {map_dir}")
    return map_dir


def _manifest_closed_loop(manifest: dict[str, Any]) -> bool:
    value = manifest.get("closed_loop", True)
    if not isinstance(value, bool):
        raise ValueError("custom line closed_loop must be a boolean")
    return value


def _read_custom_line_manifest(line_dir: Path) -> dict[str, Any]:
    manifest_path = line_dir / CUSTOM_LINE_MANIFEST_FILE
    manifest = _read_json_file(manifest_path)
    if not manifest:
        raise ValueError(f"custom line manifest is missing or invalid: {manifest_path}")
    if str(manifest.get("format") or "") != CUSTOM_LINE_FORMAT:
        raise ValueError(f"unsupported custom line format: {manifest.get('format')}")
    return manifest


def _write_custom_line_bundle(
    map_dir: Path,
    line_dir: Path,
    manifest: dict[str, Any],
    points: list[dict[str, float]],
    closed_loop: bool,
    default_speed_mps: float,
    section_speeds_mps: dict[str, float],
    constraints: dict[str, float],
) -> dict[str, Any]:
    trajectory, _, validation, layout = _compile_custom_line(
        map_dir,
        points,
        closed_loop,
        default_speed_mps,
        section_speeds_mps,
        constraints,
    )
    if not validation["valid"]:
        raise ValueError(str(validation["issue"]))
    trajectory_path = line_dir / CUSTOM_LINE_TRAJECTORY_FILE
    trajectory_hash = _write_custom_trajectory(trajectory_path, trajectory)
    geometry = [{"x_m": point["x_m"], "y_m": point["y_m"]} for point in points]
    updated = dict(manifest)
    updated["format"] = CUSTOM_LINE_FORMAT
    updated["closed_loop"] = closed_loop
    updated["points"] = geometry
    updated["point_count"] = len(points)
    updated["trajectory_point_count"] = len(trajectory)
    updated["default_speed_mps"] = default_speed_mps
    updated["section_speeds_mps"] = dict(sorted(section_speeds_mps.items()))
    updated["speed_profile_mode"] = "sections"
    updated["speed_authoring"] = "sections"
    updated["section_layout_fingerprint"] = layout["fingerprint"]
    updated["section_layout_hash"] = layout["fingerprint"]
    updated["hd_map_sha256"] = layout["hd_map_sha256"]
    updated["validation"] = validation
    updated["content_hash"] = _custom_line_content_hash(
        geometry,
        closed_loop,
        default_speed_mps,
        section_speeds_mps,
        constraints,
    )
    updated["trajectory_csv"] = CUSTOM_LINE_TRAJECTORY_FILE
    updated["trajectory_sha256"] = trajectory_hash
    _write_json_file(line_dir / CUSTOM_LINE_MANIFEST_FILE, updated)
    return updated


def _custom_line_manifest_speed_profile(
    map_dir: Path,
    manifest: dict[str, Any],
    points: list[dict[str, float]],
    closed_loop: bool,
) -> tuple[float, dict[str, float], str, dict[str, Any]]:
    layout = _custom_line_hd_layout(map_dir)
    context = _custom_line_speed_context(points, closed_loop, layout)
    has_section_authoring = "default_speed_mps" in manifest or "section_speeds_mps" in manifest
    if has_section_authoring:
        if "default_speed_mps" not in manifest:
            raise ValueError("section speed profile is missing default_speed_mps")
        default_speed = _custom_line_default_speed(manifest.get("default_speed_mps"))
        section_speeds = _custom_line_section_speeds(manifest.get("section_speeds_mps"), layout)
        return default_speed, section_speeds, "sections", {**layout, "context": context}
    default_speed, section_speeds = _legacy_custom_line_speed_profile(points, context, closed_loop)
    return default_speed, section_speeds, "legacy_points", {**layout, "context": context}


def _activate_custom_line_bundle(map_dir: Path, manifest: dict[str, Any]) -> None:
    custom_line_id = str(manifest.get("id") or "")
    if not _is_safe_custom_line_id(custom_line_id):
        raise ValueError("custom line manifest has an invalid id")
    closed_loop = _manifest_closed_loop(manifest)
    points = _custom_line_points(manifest.get("points"), closed_loop)
    constraints = _custom_line_constraints(
        {},
        manifest.get("constraints") if isinstance(manifest.get("constraints"), dict) else None,
    )
    default_speed, section_speeds, speed_mode, _ = _custom_line_manifest_speed_profile(
        map_dir,
        manifest,
        points,
        closed_loop,
    )
    if speed_mode == "legacy_points" and (
        default_speed < CUSTOM_LINE_MIN_SECTION_SPEED_MPS
        or any(speed < CUSTOM_LINE_MIN_SECTION_SPEED_MPS for speed in section_speeds.values())
    ):
        raise ValueError(
            "legacy custom line speed is below 0.1 m/s; save it as a section speed profile before activation"
        )
    trajectory, _, validation, compiled_layout = _compile_custom_line(
        map_dir,
        points,
        closed_loop,
        default_speed,
        section_speeds,
        constraints,
    )
    if not validation["valid"]:
        raise ValueError(str(validation["issue"]))
    canonical = _canonical_custom_line_paths(map_dir)
    trajectory_hash = _write_custom_trajectory(canonical["trajectory"], trajectory)
    source_hash = str(manifest.get("source_hash") or manifest.get("source_sha256") or "")
    meta = {
        "format": CUSTOM_LINE_FORMAT,
        "id": custom_line_id,
        "name": str(manifest.get("name") or custom_line_id),
        "closed_loop": closed_loop,
        "source_hash": source_hash,
        "revision": int(manifest.get("revision") or 1),
        "trajectory_csv": canonical["trajectory"].name,
        "trajectory_sha256": trajectory_hash,
        "constraints": constraints,
        "default_speed_mps": default_speed,
        "section_speeds_mps": section_speeds,
        "speed_profile_mode": speed_mode,
        "speed_authoring": speed_mode,
        "section_layout_fingerprint": compiled_layout["fingerprint"],
        "section_layout_hash": compiled_layout["fingerprint"],
        "hd_map_sha256": compiled_layout["hd_map_sha256"],
        "activated_at": _now_iso(),
    }
    _write_json_file(canonical["meta"], meta)
    try:
        _write_json_file(
            _custom_line_active_path(map_dir),
            {
                "id": custom_line_id,
                "activated_at": meta["activated_at"],
                "revision": meta["revision"],
                "trajectory_sha256": trajectory_hash,
            },
        )
    except OSError:
        # The canonical CSV + metadata pair is the atomic source of truth.
        # active.json is only a human-readable cache and must not turn a
        # completed activation into a reported failure.
        pass


def _custom_line_source_stale(map_dir: Path, manifest: dict[str, Any]) -> bool | None:
    source_type = str(manifest.get("source_type") or "")
    source_hash = str(manifest.get("source_hash") or manifest.get("source_sha256") or "")
    if not source_type or not source_hash:
        return None
    if source_type == "centerline":
        source_path = map_dir / f"{map_dir.name}_hd_map_centerline.csv"
    elif source_type == "raceline":
        source_path = map_dir / f"{map_dir.name}_raceline.csv"
    else:
        return None
    return _sha256_file(source_path) != source_hash


def _custom_line_item(
    map_dir: Path,
    line_dir: Path,
    active_id: str,
) -> dict[str, Any]:
    manifest_path = line_dir / CUSTOM_LINE_MANIFEST_FILE
    trajectory_path = line_dir / CUSTOM_LINE_TRAJECTORY_FILE
    manifest = _read_json_file(manifest_path)
    item: dict[str, Any] = {
        "id": line_dir.name,
        "name": str(manifest.get("name") or line_dir.name),
        "closed_loop": bool(manifest.get("closed_loop", True)),
        "source_type": str(manifest.get("source_type") or ""),
        "source_hash": str(manifest.get("source_hash") or manifest.get("source_sha256") or ""),
        "base_hash": str(manifest.get("base_hash") or ""),
        "content_hash": str(manifest.get("content_hash") or ""),
        "revision": int(manifest.get("revision") or 0) if str(manifest.get("revision") or "").lstrip("-").isdigit() else 0,
        "created_at": str(manifest.get("created_at") or ""),
        "updated_at": str(manifest.get("updated_at") or ""),
        "active": line_dir.name == active_id,
        "points": [],
        "point_count": 0,
        "length_m": 0.0,
        "source_stale": _custom_line_source_stale(map_dir, manifest),
        "section_layout_stale": False,
        "default_speed_mps": CUSTOM_LINE_DEFAULT_SPEED_MPS,
        "section_speeds_mps": {},
        "speed_profile_mode": str(manifest.get("speed_profile_mode") or "legacy_points"),
        "speed_sections": [],
        "repairable": False,
        "constraints": {
            "max_speed_mps": CUSTOM_LINE_DEFAULT_MAX_SPEED_MPS,
            "lateral_accel_limit_mps2": CUSTOM_LINE_DEFAULT_LATERAL_ACCEL_LIMIT_MPS2,
            "accel_limit_mps2": CUSTOM_LINE_DEFAULT_ACCEL_LIMIT_MPS2,
            "decel_limit_mps2": CUSTOM_LINE_DEFAULT_DECEL_LIMIT_MPS2,
        },
        "valid": False,
        "issue": "custom line manifest is invalid",
        "min_clearance_m": None,
        "validation": {
            "valid": False,
            "issue": "custom line manifest is invalid",
            "min_clearance_m": None,
            "containment_checked": False,
        },
        "manifest": _artifact(manifest_path),
        "trajectory": _artifact(trajectory_path),
        "trajectory_sha256": str(manifest.get("trajectory_sha256") or ""),
    }
    try:
        if str(manifest.get("format") or "") != CUSTOM_LINE_FORMAT:
            raise ValueError("custom line manifest has an unsupported format")
        if str(manifest.get("id") or "") != line_dir.name:
            raise ValueError("custom line manifest id does not match its folder")
        closed_loop = _manifest_closed_loop(manifest)
        points = _custom_line_points(manifest.get("points"), closed_loop)
        fallback_speed = manifest.get("default_speed_mps", CUSTOM_LINE_DEFAULT_SPEED_MPS)
        try:
            fallback_speed = _nonnegative_finite_float(fallback_speed, "default_speed_mps")
        except ValueError:
            fallback_speed = CUSTOM_LINE_DEFAULT_SPEED_MPS
        item["points"] = [
            {
                "x_m": point["x_m"],
                "y_m": point["y_m"],
                "speed_mps": point.get("speed_mps", fallback_speed),
            }
            for point in points
        ]
        item["point_count"] = len(points)
        item["repairable"] = True
        item["default_speed_mps"] = fallback_speed
        if isinstance(manifest.get("section_speeds_mps"), dict):
            item["section_speeds_mps"] = dict(manifest["section_speeds_mps"])
        try:
            current_layout = _custom_line_hd_layout(map_dir)
            raw_section_speeds = (
                manifest.get("section_speeds_mps")
                if isinstance(manifest.get("section_speeds_mps"), dict)
                else {}
            )
            item["speed_sections"] = [
                {
                    "id": str(section["id"]),
                    "lane_id": str(section["lane_id"]),
                    "start_gate_id": str(section["start_gate_id"]),
                    "end_gate_id": str(section["end_gate_id"]),
                    "start_s_m": float(section["start_s_m"]),
                    "end_s_m": float(section["end_s_m"]),
                    "wrap": bool(section.get("wrap", False)),
                    "speed_mps": raw_section_speeds.get(str(section["id"]), fallback_speed),
                    "configured": str(section["id"]) in raw_section_speeds,
                }
                for section in current_layout["sections"]
            ]
        except (OSError, TypeError, ValueError, OverflowError):
            pass
        constraints = _custom_line_constraints(
            {},
            manifest.get("constraints") if isinstance(manifest.get("constraints"), dict) else None,
        )
        default_speed, section_speeds, speed_mode, layout = _custom_line_manifest_speed_profile(
            map_dir,
            manifest,
            points,
            closed_loop,
        )
        trajectory, author_points, validation, compiled_layout = _compile_custom_line(
            map_dir,
            points,
            closed_loop,
            default_speed,
            section_speeds,
            constraints,
        )
        if not validation["valid"]:
            raise ValueError(str(validation["issue"]))
        expected_hash = str(manifest.get("trajectory_sha256") or "")
        actual_hash = _sha256_file(trajectory_path)
        if not trajectory_path.exists() or not expected_hash or actual_hash != expected_hash:
            raise ValueError("trajectory.csv is missing or does not match its manifest")
        stored_layout = str(
            manifest.get("section_layout_fingerprint") or manifest.get("section_layout_hash") or ""
        )
        section_layout_stale = bool(speed_mode == "sections" and stored_layout != layout["fingerprint"])
        if speed_mode == "sections":
            expected_content_hash = _custom_line_content_hash(
                points,
                closed_loop,
                default_speed,
                section_speeds,
                constraints,
            )
            if str(manifest.get("content_hash") or "") != expected_content_hash:
                raise ValueError("custom line authoring content does not match its manifest hash")
        length_m = trajectory[-1]["s_m"]
        if closed_loop:
            length_m += math.hypot(
                trajectory[0]["x_m"] - trajectory[-1]["x_m"],
                trajectory[0]["y_m"] - trajectory[-1]["y_m"],
            )
        item.update(
            {
                "closed_loop": closed_loop,
                "points": author_points,
                "point_count": len(points),
                "trajectory_point_count": len(trajectory),
                "length_m": length_m,
                "constraints": constraints,
                "default_speed_mps": default_speed,
                "section_speeds_mps": section_speeds,
                "speed_profile_mode": speed_mode,
                "section_layout_stale": section_layout_stale,
                "speed_sections": [
                    {
                        "id": str(section["id"]),
                        "lane_id": str(section["lane_id"]),
                        "start_gate_id": str(section["start_gate_id"]),
                        "end_gate_id": str(section["end_gate_id"]),
                        "start_s_m": float(section["start_s_m"]),
                        "end_s_m": float(section["end_s_m"]),
                        "custom_start_s_m": float(section["custom_start_s_m"]),
                        "custom_end_s_m": float(section["custom_end_s_m"]),
                        "wrap": bool(section.get("wrap", False)),
                        "speed_mps": section_speeds.get(str(section["id"]), default_speed),
                        "configured": str(section["id"]) in section_speeds,
                    }
                    for section in compiled_layout["context"]["sections"]
                ],
                "valid": True,
                "issue": "",
                "min_clearance_m": validation["min_clearance_m"],
                "validation": validation,
            }
        )
    except (OSError, TypeError, ValueError, OverflowError) as exc:
        item["issue"] = str(exc)
        item["validation"] = {
            "valid": False,
            "issue": str(exc),
            "min_clearance_m": item.get("min_clearance_m"),
            "containment_checked": False,
        }
    return item


def _read_custom_lines(map_dir: Path) -> dict[str, Any]:
    root = _checked_custom_line_root(map_dir)
    active_id, active_issue = _canonical_custom_line_active(map_dir)
    items: list[dict[str, Any]] = []
    if root.exists():
        resolved_root = root.resolve()
        for line_dir in sorted(root.iterdir()):
            if line_dir.name == CUSTOM_LINE_ACTIVE_FILE or not line_dir.is_dir() or line_dir.is_symlink():
                continue
            if not _is_safe_custom_line_id(line_dir.name):
                continue
            if not _is_relative_to(line_dir.resolve(), resolved_root):
                continue
            items.append(_custom_line_item(map_dir, line_dir, active_id))
    items.sort(key=lambda item: (str(item.get("name") or "").casefold(), str(item.get("id") or "")))
    canonical = _canonical_custom_line_paths(map_dir)
    return {
        "active_id": active_id,
        "active_issue": active_issue,
        "active_missing": bool(active_id and not any(item["id"] == active_id for item in items)),
        "items": items,
        "active_trajectory": _artifact(canonical["trajectory"]),
        "active_meta": _artifact(canonical["meta"]),
    }


def create_custom_line(config: ConsoleConfig, payload: dict[str, Any]) -> dict[str, Any]:
    map_dir = _resolve_custom_line_map(config, payload)
    name = _custom_line_name(payload.get("name"))
    source_type = _custom_line_source_type(payload)
    default_speed_mps = _custom_line_default_speed(
        payload.get("default_speed_mps", CUSTOM_LINE_DEFAULT_SPEED_MPS),
    )
    closed_loop_value = payload.get("closed_loop", _source_closed_loop(map_dir))
    if not isinstance(closed_loop_value, bool):
        raise ValueError("closed_loop must be a boolean")
    constraints = _custom_line_constraints(payload)
    source_path, raw_points = _read_custom_line_source(map_dir, source_type, default_speed_mps)
    points = _custom_line_points(raw_points, closed_loop_value)
    points = [{"x_m": point["x_m"], "y_m": point["y_m"]} for point in points]
    layout = _custom_line_hd_layout(map_dir)
    supplied_section_speeds = _custom_line_section_speeds(payload.get("section_speeds_mps"), layout)
    section_speeds: dict[str, float] = dict(supplied_section_speeds)
    for section in layout["sections"]:
        section_id = str(section["id"])
        if section_id in section_speeds:
            continue
        raw_override = section.get("speed_override_mps")
        if raw_override is not None:
            try:
                override = _custom_line_default_speed(raw_override)
            except ValueError:
                continue
            section_speeds[section_id] = override

    root = _checked_custom_line_root(map_dir, create=True)
    custom_line_id = _custom_line_id_from_name(root, name)
    line_dir = _custom_line_path(map_dir, custom_line_id)
    line_dir.mkdir(parents=False, exist_ok=False)
    timestamp = _now_iso()
    source_hash = _sha256_file(source_path)
    manifest = {
        "format": CUSTOM_LINE_FORMAT,
        "id": custom_line_id,
        "name": name,
        "closed_loop": closed_loop_value,
        "source_type": source_type,
        "source_path": source_path.name,
        "source_hash": source_hash,
        "source_sha256": source_hash,
        "base_hash": _custom_line_content_hash(points, closed_loop_value),
        "revision": 1,
        "created_at": timestamp,
        "updated_at": timestamp,
        "constraints": constraints,
    }
    try:
        _write_custom_line_bundle(
            map_dir,
            line_dir,
            manifest,
            points,
            closed_loop_value,
            default_speed_mps,
            section_speeds,
            constraints,
        )
    except Exception:
        shutil.rmtree(line_dir, ignore_errors=True)
        raise
    return build_map_detail(config, str(map_dir))


def update_custom_line(config: ConsoleConfig, payload: dict[str, Any]) -> dict[str, Any]:
    map_dir = _resolve_custom_line_map(config, payload)
    custom_line_id = _require_custom_line_id(payload)
    line_dir = _custom_line_path(map_dir, custom_line_id)
    if not line_dir.exists() or not line_dir.is_dir():
        raise FileNotFoundError(f"custom line not found: {custom_line_id}")
    manifest = _read_custom_line_manifest(line_dir)
    if str(manifest.get("id") or "") != custom_line_id:
        raise ValueError("custom line manifest id does not match its folder")

    name = _custom_line_name(payload.get("name", manifest.get("name")))
    closed_loop = payload.get("closed_loop", manifest.get("closed_loop", True))
    if not isinstance(closed_loop, bool):
        raise ValueError("closed_loop must be a boolean")
    constraints = _custom_line_constraints(
        payload,
        manifest.get("constraints") if isinstance(manifest.get("constraints"), dict) else None,
    )
    layout = _custom_line_hd_layout(map_dir)
    explicit_complete_profile = "default_speed_mps" in payload and "section_speeds_mps" in payload
    if explicit_complete_profile:
        default_speed_mps = _custom_line_default_speed(payload["default_speed_mps"])
        section_speeds = _custom_line_section_speeds(payload["section_speeds_mps"], layout)
    elif "default_speed_mps" in manifest:
        default_speed_mps = _custom_line_default_speed(
            payload.get("default_speed_mps", manifest.get("default_speed_mps")),
        )
        section_speeds = _custom_line_section_speeds(
            payload.get("section_speeds_mps", manifest.get("section_speeds_mps")),
            layout,
        )
    else:
        existing_closed_loop = _manifest_closed_loop(manifest)
        existing_points = _custom_line_points(manifest.get("points"), existing_closed_loop)
        existing_default, existing_section_speeds, _, _ = _custom_line_manifest_speed_profile(
            map_dir,
            manifest,
            existing_points,
            existing_closed_loop,
        )
        default_speed_mps = _custom_line_default_speed(
            payload.get("default_speed_mps", existing_default),
        )
        section_speeds = _custom_line_section_speeds(
            payload.get("section_speeds_mps", existing_section_speeds),
            layout,
        )
    point_payload = payload.get("points", manifest.get("points"))
    points = _custom_line_points(point_payload, closed_loop)
    points = [{"x_m": point["x_m"], "y_m": point["y_m"]} for point in points]

    try:
        previous_revision = int(manifest.get("revision") or 0)
    except (TypeError, ValueError) as exc:
        raise ValueError("custom line revision is invalid") from exc
    updated = dict(manifest)
    updated.update(
        {
            "id": custom_line_id,
            "name": name,
            "closed_loop": closed_loop,
            "revision": previous_revision + 1,
            "updated_at": _now_iso(),
            "constraints": constraints,
        }
    )
    updated = _write_custom_line_bundle(
        map_dir,
        line_dir,
        updated,
        points,
        closed_loop,
        default_speed_mps,
        section_speeds,
        constraints,
    )
    canonical_meta = _read_json_file(_canonical_custom_line_paths(map_dir)["meta"])
    if str(canonical_meta.get("id") or "") == custom_line_id:
        _activate_custom_line_bundle(map_dir, updated)
    return build_map_detail(config, str(map_dir))


def activate_custom_line(config: ConsoleConfig, payload: dict[str, Any]) -> dict[str, Any]:
    map_dir = _resolve_custom_line_map(config, payload)
    custom_line_id = _require_custom_line_id(payload)
    line_dir = _custom_line_path(map_dir, custom_line_id)
    if not line_dir.exists() or not line_dir.is_dir():
        raise FileNotFoundError(f"custom line not found: {custom_line_id}")
    manifest = _read_custom_line_manifest(line_dir)
    if str(manifest.get("id") or "") != custom_line_id:
        raise ValueError("custom line manifest id does not match its folder")
    closed_loop = _manifest_closed_loop(manifest)
    points = _custom_line_points(manifest.get("points"), closed_loop)
    constraints = _custom_line_constraints(
        {},
        manifest.get("constraints") if isinstance(manifest.get("constraints"), dict) else None,
    )
    default_speed, section_speeds, speed_mode, _ = _custom_line_manifest_speed_profile(
        map_dir,
        manifest,
        points,
        closed_loop,
    )
    _, _, validation, _ = _compile_custom_line(
        map_dir,
        points,
        closed_loop,
        default_speed,
        section_speeds,
        constraints,
    )
    if not validation["valid"]:
        raise ValueError(str(validation["issue"]))
    if speed_mode == "sections":
        manifest = _write_custom_line_bundle(
            map_dir,
            line_dir,
            manifest,
            points,
            closed_loop,
            default_speed,
            section_speeds,
            constraints,
        )
    _activate_custom_line_bundle(map_dir, manifest)
    return build_map_detail(config, str(map_dir))


def delete_custom_line(config: ConsoleConfig, payload: dict[str, Any]) -> dict[str, Any]:
    map_dir = _resolve_custom_line_map(config, payload)
    custom_line_id = _require_custom_line_id(payload)
    line_dir = _custom_line_path(map_dir, custom_line_id)
    if not line_dir.exists() or not line_dir.is_dir():
        raise FileNotFoundError(f"custom line not found: {custom_line_id}")
    hd_map_path = map_dir / f"{map_dir.name}_hd_map.yaml"
    if hd_map_path.is_file():
        hd_data = load_yaml(hd_map_path)
        for junction in hd_data.get("junctions", []) or []:
            if not isinstance(junction, dict):
                continue
            branches = junction.get("branches", {})
            if isinstance(branches, dict) and custom_line_id in {
                str(branches.get(direction) or "")
                for direction in ("left", "straight", "right")
            }:
                raise ValueError(
                    f"custom line {custom_line_id} is referenced by junction "
                    f"{junction.get('id') or '<unnamed>'}"
                )
    active_id, _ = _canonical_custom_line_active(map_dir)
    canonical = _canonical_custom_line_paths(map_dir)
    canonical_meta = _read_json_file(canonical["meta"])
    canonical_claimed_id = str(canonical_meta.get("id") or "")
    active_cache = _read_json_file(_custom_line_active_path(map_dir))
    cached_failed_id = str(active_cache.get("failed_id") or "")
    if active_id == custom_line_id or canonical_claimed_id == custom_line_id or cached_failed_id == custom_line_id:
        timestamp = _now_iso()
        for path in (canonical["meta"], canonical["trajectory"]):
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        _write_json_file(_custom_line_active_path(map_dir), {"id": "", "deactivated_at": timestamp})
    shutil.rmtree(line_dir)
    return build_map_detail(config, str(map_dir))


def _clear_active_custom_line_after_hd_change(
    map_dir: Path,
    issue: str,
    custom_line_id: str = "",
) -> None:
    canonical = _canonical_custom_line_paths(map_dir)
    for path in (canonical["meta"], canonical["trajectory"]):
        try:
            path.unlink()
        except FileNotFoundError:
            pass
    try:
        _write_json_file(
            _custom_line_active_path(map_dir),
            {
                "id": "",
                "failed_id": custom_line_id,
                "deactivated_at": _now_iso(),
                "issue": issue,
            },
        )
    except OSError:
        pass


def _refresh_active_custom_line_after_hd_change(map_dir: Path) -> None:
    canonical_meta = _read_json_file(_canonical_custom_line_paths(map_dir)["meta"])
    custom_line_id = str(canonical_meta.get("id") or "")
    if not custom_line_id:
        return
    try:
        line_dir = _custom_line_path(map_dir, custom_line_id)
        manifest = _read_custom_line_manifest(line_dir)
        if str(manifest.get("id") or "") != custom_line_id:
            raise ValueError("custom line manifest id does not match its folder")
        closed_loop = _manifest_closed_loop(manifest)
        points = _custom_line_points(manifest.get("points"), closed_loop)
        constraints = _custom_line_constraints(
            {},
            manifest.get("constraints") if isinstance(manifest.get("constraints"), dict) else None,
        )
        default_speed, section_speeds, speed_mode, _ = _custom_line_manifest_speed_profile(
            map_dir,
            manifest,
            points,
            closed_loop,
        )
        if speed_mode == "sections":
            manifest = _write_custom_line_bundle(
                map_dir,
                line_dir,
                manifest,
                points,
                closed_loop,
                default_speed,
                section_speeds,
                constraints,
            )
        _activate_custom_line_bundle(map_dir, manifest)
    except (OSError, TypeError, ValueError, OverflowError) as exc:
        _clear_active_custom_line_after_hd_change(
            map_dir,
            f"HD map changed: {exc}",
            custom_line_id,
        )


def _map_record(
    map_dir: Path,
    map_root: Path | None = None,
    competition_routes: dict[str, Any] | None = None,
) -> dict[str, Any]:
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
        "raceline_meta": _artifact(map_dir / f"{name}_raceline.meta.json"),
        "custom_line_csv": _artifact(map_dir / f"{name}_custom_line.csv"),
        "custom_line_meta": _artifact(map_dir / f"{name}_custom_line.meta.json"),
        "line_preview": _artifact(map_dir / f"{name}_line_preview.png"),
    }
    localization_ready = all(
        artifacts[key]["exists"] for key in ("cuvgl_map", "cuvslam_map", "hd_map")
    )
    active_custom_line_id, _ = _canonical_custom_line_active(map_dir)
    custom_line_ready = bool(active_custom_line_id)
    driving_line_ready = bool(artifacts["raceline_csv"]["exists"] or custom_line_ready)
    route_summary = competition_routes or competition_route_summary(map_dir)
    competition_route_status = str(route_summary["status"])
    competition_routes_ready = bool(route_summary["ready"])
    complete_runtime = localization_ready and driving_line_ready and competition_routes_ready
    return {
        "name": name,
        "display_name": _display_name(map_root, map_dir) if map_root is not None else name,
        "path": str(map_dir),
        "size_bytes": _dir_size(map_dir),
        "modified_at": _iso_mtime(map_dir),
        "fingerprint": directory_fingerprint(map_dir),
        "artifacts": artifacts,
        "competition_route_status": competition_route_status,
        "competition_routes_ready": competition_routes_ready,
        "complete_runtime_bundle": complete_runtime,
    }


def _junction_branch_ids(hd_map: dict[str, Any]) -> set[str]:
    branch_ids: set[str] = set()
    for junction in hd_map.get("junctions", []) or []:
        if not isinstance(junction, dict):
            continue
        branches = junction.get("branches", {})
        if not isinstance(branches, dict):
            continue
        for direction in ("left", "straight", "right"):
            route_id = str(branches.get(direction) or "").strip()
            if route_id:
                branch_ids.add(route_id)
    return branch_ids


def _route_string_array(
    parameters: dict[str, Any],
    field_name: str,
    issues: list[str],
) -> list[str]:
    raw_values = parameters.get(field_name)
    if not isinstance(raw_values, list):
        issues.append(f"{field_name} must be an array")
        return []
    values: list[str] = []
    for index, raw_value in enumerate(raw_values):
        if not isinstance(raw_value, str):
            issues.append(f"{field_name}[{index}] must be a string")
            values.append("")
            continue
        values.append(raw_value.strip())
    return values


def _runtime_routes(map_dir: Path, hd_map: dict[str, Any]) -> dict[str, Any]:
    config_path = map_dir / COMPETITION_ROUTE_CONFIG_FILE
    branch_ids = _junction_branch_ids(hd_map)
    result: dict[str, Any] = {
        "status": "unconfigured",
        "configured_lane_ids": [],
        "routes": [],
        "missing_branch_ids": sorted(branch_ids),
        "issues": [],
        "config_path": str(config_path),
    }
    if config_path.is_symlink():
        result["status"] = "invalid"
        result["issues"] = ["competition route config must be a regular file"]
        return result
    if not config_path.exists():
        return result
    if not config_path.is_file():
        result["status"] = "invalid"
        result["issues"] = ["competition route config must be a regular file"]
        return result

    try:
        data = load_yaml(config_path)
    except OSError as exc:
        result["status"] = "invalid"
        result["issues"] = [f"competition route config could not be read: {exc}"]
        return result

    issues: list[str] = []
    wildcard = data.get("/**")
    if not isinstance(wildcard, dict):
        issues.append("competition route config requires a '/**' mapping")
        parameters: dict[str, Any] = {}
    else:
        raw_parameters = wildcard.get("ros__parameters")
        if not isinstance(raw_parameters, dict):
            issues.append("competition route config requires a ros__parameters mapping")
            parameters = {}
        else:
            parameters = raw_parameters

    lane_ids = _route_string_array(parameters, "lane_ids", issues)
    lane_path_topics = _route_string_array(parameters, "lane_path_topics", issues)
    lane_trajectory_topics = _route_string_array(
        parameters,
        "lane_trajectory_topics",
        issues,
    )
    raw_speeds = parameters.get("lane_target_speeds_mps")
    if not isinstance(raw_speeds, list):
        issues.append("lane_target_speeds_mps must be an array")
        speeds: list[float | None] = []
    else:
        speeds = []
        for index, raw_speed in enumerate(raw_speeds):
            if isinstance(raw_speed, bool) or not isinstance(raw_speed, (int, float)):
                issues.append(f"lane_target_speeds_mps[{index}] must be a finite non-negative number")
                speeds.append(None)
                continue
            speed = float(raw_speed)
            if not math.isfinite(speed) or speed < 0.0:
                issues.append(f"lane_target_speeds_mps[{index}] must be a finite non-negative number")
                speeds.append(None)
            else:
                speeds.append(speed)

    configured_lane_ids = [lane_id for lane_id in lane_ids if lane_id]
    result["configured_lane_ids"] = configured_lane_ids
    route_count = max(
        len(lane_ids),
        len(lane_path_topics),
        len(lane_trajectory_topics),
        len(speeds),
    )
    result["routes"] = [
        {
            "id": lane_ids[index] if index < len(lane_ids) else "",
            "path_topic": lane_path_topics[index] if index < len(lane_path_topics) else "",
            "trajectory_topic": (
                lane_trajectory_topics[index]
                if index < len(lane_trajectory_topics)
                else ""
            ),
            "target_speed_mps": speeds[index] if index < len(speeds) else None,
        }
        for index in range(route_count)
    ]
    result["default_lane_id"] = str(parameters.get("default_lane_id") or "")
    result["requested_lane_timeout_sec"] = parameters.get(
        "requested_lane_timeout_sec"
    )
    result["current_section_timeout_sec"] = parameters.get(
        "current_section_timeout_sec"
    )
    if not lane_ids:
        issues.append("lane_ids must not be empty")
    if any(not lane_id for lane_id in lane_ids):
        issues.append("lane_ids must not contain empty values")
    duplicate_lane_ids = sorted(
        {lane_id for lane_id in configured_lane_ids if configured_lane_ids.count(lane_id) > 1}
    )
    if duplicate_lane_ids:
        issues.append(f"lane_ids contains duplicates: {', '.join(duplicate_lane_ids)}")

    array_lengths = {
        len(lane_ids),
        len(lane_path_topics),
        len(lane_trajectory_topics),
        len(speeds),
    }
    if len(array_lengths) != 1:
        issues.append(
            "lane_ids, lane_path_topics, lane_trajectory_topics, and "
            "lane_target_speeds_mps must have equal lengths"
        )

    pair_count = min(len(lane_ids), len(lane_path_topics), len(lane_trajectory_topics))
    for index in range(pair_count):
        if not lane_path_topics[index] and not lane_trajectory_topics[index]:
            lane_label = lane_ids[index] or str(index)
            issues.append(f"lane {lane_label} requires a path or trajectory topic")

    for field_name, topics in (
        ("lane_path_topics", lane_path_topics),
        ("lane_trajectory_topics", lane_trajectory_topics),
    ):
        nonempty = [topic for topic in topics if topic]
        duplicates = sorted({topic for topic in nonempty if nonempty.count(topic) > 1})
        if duplicates:
            issues.append(f"{field_name} contains duplicates: {', '.join(duplicates)}")

    default_lane_id = parameters.get("default_lane_id")
    if not isinstance(default_lane_id, str) or not default_lane_id.strip():
        issues.append("default_lane_id must be a non-empty string")
    elif default_lane_id.strip() not in set(configured_lane_ids):
        issues.append(f"default_lane_id is not present in lane_ids: {default_lane_id.strip()}")
    elif default_lane_id.strip() != "primary":
        issues.append("default_lane_id must be primary for the competition planning manager")

    competition_topics = {
        "requested_lane_topic": "/planning/requested_lane",
        "current_section_topic": "/localization/current_section",
        "output_trajectory_topic": "/planning/route/trajectory",
        "output_profile_topic": "/planning/route/trajectory_profile",
        "target_speed_topic": "/planning/route/target_speed",
        "selected_lane_topic": "/planning/route/selected_lane",
        "ready_topic": "/planning/route/ready",
        "diagnostics_topic": "/planning/route/diagnostics",
    }
    resolved_topics: dict[str, str] = {}
    for field_name, expected_topic in competition_topics.items():
        raw_topic = parameters.get(field_name)
        if not isinstance(raw_topic, str) or not raw_topic.strip():
            issues.append(f"{field_name} must be a non-empty string")
            resolved_topics[field_name] = ""
            continue
        topic = raw_topic.strip()
        resolved_topics[field_name] = topic
        if topic != expected_topic:
            issues.append(
                f"{field_name} must be {expected_topic} for the competition planning manager"
            )

    output_trajectory_topic = resolved_topics["output_trajectory_topic"]
    output_profile_topic = resolved_topics["output_profile_topic"]
    if output_trajectory_topic and output_trajectory_topic in lane_path_topics:
        issues.append("a lane path input must not equal output_trajectory_topic")
    if output_profile_topic and output_profile_topic in lane_trajectory_topics:
        issues.append("a lane trajectory input must not equal output_profile_topic")

    if parameters.get("require_requested_lane_heartbeat") is not True:
        issues.append("require_requested_lane_heartbeat must be true for competition driving")
    if "section_lane_rules" in parameters:
        section_lane_rules = parameters.get("section_lane_rules")
        if section_lane_rules == []:
            issues.append(
                "empty section_lane_rules must be omitted because ROS 2 cannot infer its array type"
            )
        elif not isinstance(section_lane_rules, list) or any(
            not isinstance(rule, str) for rule in section_lane_rules
        ):
            issues.append("section_lane_rules must be an array of strings")
    for field_name in ("requested_lane_timeout_sec", "current_section_timeout_sec"):
        raw_timeout = parameters.get(field_name)
        if (
            isinstance(raw_timeout, bool)
            # These parameters are declared as doubles by the ROS 2 node.
            # An integer YAML scalar is not implicitly converted by rclcpp.
            or not isinstance(raw_timeout, float)
            or not math.isfinite(float(raw_timeout))
            or float(raw_timeout) <= 0.0
        ):
            issues.append(f"{field_name} must be a finite positive YAML float")

    missing_branch_ids = sorted(branch_ids - set(configured_lane_ids))
    result["missing_branch_ids"] = missing_branch_ids
    if missing_branch_ids:
        issues.append(
            "junction branch routes are not registered in lane_ids: "
            + ", ".join(missing_branch_ids)
        )
    result["issues"] = issues
    result["status"] = "invalid" if any(
        not issue.startswith("junction branch routes are not registered") for issue in issues
    ) else ("warning" if missing_branch_ids else "ready")
    return result


def competition_route_summary(map_dir: Path) -> dict[str, Any]:
    """Return the lightweight competition-route readiness used by map indexes and detail."""
    hd_map_path = map_dir / f"{map_dir.name}_hd_map.yaml"
    if not hd_map_path.is_file():
        return {"status": "not_required", "ready": True}
    try:
        hd_map, _ = _read_hd_map(hd_map_path)
    except (OSError, TypeError, ValueError, OverflowError):
        return {"status": "invalid", "ready": False}
    if not hd_map.get("junctions"):
        return {"status": "not_required", "ready": True}
    try:
        status = str(_runtime_routes(map_dir, hd_map).get("status") or "invalid")
    except (OSError, TypeError, ValueError, OverflowError):
        status = "invalid"
    return {"status": status, "ready": status == "ready"}


def _competition_route_topic(value: Any, field_name: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field_name} must be a string")
    topic = value.strip()
    if topic and (not topic.startswith("/") or any(char.isspace() for char in topic)):
        raise ValueError(f"{field_name} must be an absolute ROS topic without whitespace")
    return topic


def _competition_route_number(
    value: Any,
    field_name: str,
    *,
    allow_zero: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a finite number")
    number = float(value)
    if not math.isfinite(number) or number < 0.0 or (not allow_zero and number == 0.0):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{field_name} must be a finite {qualifier} number")
    return number


def _write_competition_route_yaml(
    path: Path,
    routes: list[dict[str, Any]],
    requested_lane_timeout_sec: float,
    current_section_timeout_sec: float,
) -> None:
    def yaml_array(values: list[Any]) -> str:
        return json.dumps(values, ensure_ascii=True, separators=(", ", ": "))

    def yaml_float(value: float) -> str:
        # json.dumps preserves the decimal point for integral float values,
        # keeping the scalar compatible with rclcpp double parameters.
        return json.dumps(float(value), allow_nan=False)

    content = "\n".join(
        [
            "/**:",
            "  ros__parameters:",
            f"    lane_ids: {yaml_array([route['id'] for route in routes])}",
            f"    lane_path_topics: {yaml_array([route['path_topic'] for route in routes])}",
            f"    lane_trajectory_topics: {yaml_array([route['trajectory_topic'] for route in routes])}",
            f"    lane_target_speeds_mps: {yaml_array([route['target_speed_mps'] for route in routes])}",
            '    default_lane_id: "primary"',
            # ROS 2 cannot infer a parameter type from an empty YAML sequence.
            # Omit optional rules so the node's typed empty-array default is used.
            "    fallback_to_default_lane: false",
            "",
            '    requested_lane_topic: "/planning/requested_lane"',
            '    current_section_topic: "/localization/current_section"',
            '    output_trajectory_topic: "/planning/route/trajectory"',
            '    output_profile_topic: "/planning/route/trajectory_profile"',
            '    target_speed_topic: "/planning/route/target_speed"',
            '    selected_lane_topic: "/planning/route/selected_lane"',
            '    ready_topic: "/planning/route/ready"',
            '    diagnostics_topic: "/planning/route/diagnostics"',
            "",
            "    publish_rate_hz: 10.0",
            "    path_timeout_sec: 0.0",
            f"    requested_lane_timeout_sec: {yaml_float(requested_lane_timeout_sec)}",
            "    require_requested_lane_heartbeat: true",
            f"    current_section_timeout_sec: {yaml_float(current_section_timeout_sec)}",
            "    min_path_poses: 2",
            "    min_path_length_m: 0.10",
            "",
        ]
    )
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass


def save_competition_routes(
    config: ConsoleConfig,
    payload: dict[str, Any],
) -> dict[str, Any]:
    map_dir_value = str(payload.get("map_dir") or "")
    if not map_dir_value:
        raise ValueError("map_dir is required")
    map_dir = resolve_allowed_path(config, map_dir_value)
    if not map_dir.is_dir():
        raise FileNotFoundError(f"map folder not found: {map_dir}")

    hd_map_path = map_dir / f"{map_dir.name}_hd_map.yaml"
    if not hd_map_path.is_file():
        raise FileNotFoundError("HD map YAML is required before configuring competition routes")
    hd_map, _ = _read_hd_map(hd_map_path)
    lane_ids = {str(lane.get("id") or "") for lane in hd_map.get("lanes", [])}
    eligible_ids = {
        str(item["id"])
        for item in _junction_route_catalog(map_dir, lane_ids, _read_custom_lines(map_dir))
        if item.get("eligible")
    }

    raw_routes = payload.get("routes")
    if not isinstance(raw_routes, list) or not raw_routes:
        raise ValueError("routes must be a non-empty array")
    if len(raw_routes) > 128:
        raise ValueError("routes must contain at most 128 entries")
    routes: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    for index, raw_route in enumerate(raw_routes):
        if not isinstance(raw_route, dict):
            raise ValueError(f"routes[{index}] must be an object")
        route_id = str(raw_route.get("id") or "").strip()
        if not route_id:
            raise ValueError(f"routes[{index}].id is required")
        if route_id in seen_ids:
            raise ValueError(f"duplicate route ID: {route_id}")
        if route_id not in eligible_ids:
            raise ValueError(f"route is not available in this map: {route_id}")
        seen_ids.add(route_id)
        path_topic = _competition_route_topic(
            raw_route.get("path_topic", ""), f"routes[{index}].path_topic"
        )
        trajectory_topic = _competition_route_topic(
            raw_route.get("trajectory_topic", ""),
            f"routes[{index}].trajectory_topic",
        )
        if not path_topic and not trajectory_topic:
            raise ValueError(f"route {route_id} requires a Path or Trajectory topic")
        routes.append(
            {
                "id": route_id,
                "path_topic": path_topic,
                "trajectory_topic": trajectory_topic,
                "target_speed_mps": _competition_route_number(
                    raw_route.get("target_speed_mps"),
                    f"routes[{index}].target_speed_mps",
                    allow_zero=True,
                ),
            }
        )

    if "primary" not in seen_ids:
        raise ValueError("primary route is required")
    missing_branch_ids = sorted(_junction_branch_ids(hd_map) - seen_ids)
    if missing_branch_ids:
        raise ValueError(
            "junction branch routes are missing: " + ", ".join(missing_branch_ids)
        )
    for field_name in ("path_topic", "trajectory_topic"):
        topics = [str(route[field_name]) for route in routes if route[field_name]]
        duplicates = sorted({topic for topic in topics if topics.count(topic) > 1})
        if duplicates:
            raise ValueError(f"{field_name} contains duplicate topics: {', '.join(duplicates)}")

    requested_timeout = _competition_route_number(
        payload.get("requested_lane_timeout_sec", 0.5),
        "requested_lane_timeout_sec",
    )
    section_timeout = _competition_route_number(
        payload.get("current_section_timeout_sec", 1.0),
        "current_section_timeout_sec",
    )
    config_path = map_dir / COMPETITION_ROUTE_CONFIG_FILE
    if config_path.is_symlink():
        raise ValueError("competition route config must be a regular file")
    _write_competition_route_yaml(config_path, routes, requested_timeout, section_timeout)
    detail = build_map_detail(config, str(map_dir))
    if detail.get("runtime_routes", {}).get("status") != "ready":
        raise RuntimeError("saved competition route config did not pass runtime validation")
    return detail


def build_map_detail(config: ConsoleConfig, map_dir_value: str) -> dict[str, Any]:
    map_dir = resolve_allowed_path(config, map_dir_value)
    if not map_dir.exists() or not map_dir.is_dir():
        raise FileNotFoundError(f"map folder not found: {map_dir}")

    name = map_dir.name
    hd_map_path = map_dir / f"{name}_hd_map.yaml"
    landmark_yaml_path = map_dir / "vslam_landmarks.yaml"
    centerline_path = map_dir / f"{name}_hd_map_centerline.csv"
    raceline_path = map_dir / f"{name}_raceline.csv"
    raceline_meta_path = map_dir / f"{name}_raceline.meta.json"
    line_preview_path = map_dir / f"{name}_line_preview.png"
    snapshot_path = map_dir / "vslam_reference_snapshot.json"

    hd_map, raster = _read_hd_map(hd_map_path)
    if raster is None:
        raster = _raster_from_map_yaml(landmark_yaml_path)

    preview_image = ""
    if line_preview_path.exists():
        preview_image = _file_url(line_preview_path)
    elif raster and raster.get("image_url"):
        preview_image = str(raster["image_url"])

    centerline = _read_xy_csv(centerline_path, ",", 0, 1)
    raceline = _read_xy_csv(raceline_path, ";", 1, 2)
    odometry = _read_snapshot_odometry(snapshot_path)
    speed_override_count = sum(
        1 for section in hd_map.get("sections", []) if section.get("speed_override_mps") is not None
    )
    primary_lane = next((lane for lane in hd_map.get("lanes", []) if lane.get("primary")), None)
    custom_line_catalog = _read_custom_lines(map_dir)
    junction_route_catalog = _junction_route_catalog(
        map_dir,
        {str(lane.get("id") or "") for lane in hd_map.get("lanes", [])},
        custom_line_catalog,
    )
    runtime_routes = _runtime_routes(map_dir, hd_map)
    competition_routes = (
        {
            "status": str(runtime_routes.get("status") or "invalid"),
            "ready": runtime_routes.get("status") == "ready",
        }
        if hd_map.get("junctions")
        else {"status": "not_required", "ready": True}
    )

    return {
        "map": _map_record(map_dir, config.map_root, competition_routes),
        "raster": raster,
        "preview_image_url": preview_image,
        "hd_map": hd_map,
        "runtime_routes": runtime_routes,
        "hd_map_versions": _read_hd_map_versions(map_dir),
        "custom_lines": custom_line_catalog["items"],
        "custom_line_catalog": custom_line_catalog,
        "active_custom_line_id": custom_line_catalog["active_id"],
        "junction_route_catalog": junction_route_catalog,
        "junction_route_ids": [
            str(item["id"]) for item in junction_route_catalog if item.get("eligible")
        ],
        "centerline_csv": centerline,
        "raceline_csv": raceline,
        "raceline_metadata": _read_json_file(raceline_meta_path),
        "odometry": odometry,
        "stats": {
            "lane_count": len(hd_map.get("lanes", [])),
            "primary_lane_id": hd_map.get("primary_lane_id") or "",
            "primary_centerline_points": len(primary_lane.get("centerline", [])) if primary_lane else 0,
            "primary_centerline_length_m": primary_lane.get("centerline_length_m", 0.0) if primary_lane else 0.0,
            "section_gate_count": len(hd_map.get("section_gates", [])),
            "section_count": len(hd_map.get("sections", [])),
            "junction_count": len(hd_map.get("junctions", [])),
            "speed_override_count": speed_override_count,
            "raceline_points": raceline["count"],
            "custom_line_count": len(custom_line_catalog["items"]),
            "odometry_points": odometry["count"],
        },
    }


def save_hd_map(config: ConsoleConfig, payload: dict[str, Any]) -> dict[str, Any]:
    map_dir_value = str(payload.get("map_dir") or "")
    if not map_dir_value:
        raise ValueError("map_dir is required")
    map_dir = resolve_allowed_path(config, map_dir_value)
    if not map_dir.exists() or not map_dir.is_dir():
        raise FileNotFoundError(f"map folder not found: {map_dir}")

    name = map_dir.name
    hd_map_path = map_dir / f"{name}_hd_map.yaml"
    landmark_yaml_path = map_dir / "vslam_landmarks.yaml"
    centerline_path = map_dir / f"{name}_hd_map_centerline.csv"

    previous_data = load_yaml(hd_map_path) if hd_map_path.exists() else {}
    raster = _raster_from_map_yaml(landmark_yaml_path)
    if raster is None and hd_map_path.exists():
        raster = _raster_from_hd_map(hd_map_path, previous_data)
    if not raster:
        raise FileNotFoundError("vslam_landmarks.yaml is required before editing the HD map. Run Raster first.")
    if not raster.get("resolution_m_per_px") or not raster.get("width") or not raster.get("height"):
        raise ValueError("raster metadata is incomplete. Re-run Raster before saving the HD map.")

    raw_lanes = payload.get("lanes")
    if not isinstance(raw_lanes, list) or not raw_lanes:
        raise ValueError("at least one lane is required")

    lanes: list[dict[str, Any]] = []
    used_ids: set[str] = set()
    for index, raw_lane in enumerate(raw_lanes, start=1):
        if not isinstance(raw_lane, dict):
            continue
        fallback_id = f"lane_{index:03d}"
        lane_id = _sanitize_id(str(raw_lane.get("id") or ""), fallback_id)
        if lane_id in used_ids:
            lane_id = fallback_id
        used_ids.add(lane_id)
        lanes.append(
            {
                "id": lane_id,
                "closed_loop": bool(raw_lane.get("closed_loop", True)),
                "left_bound": _payload_points(raw_lane.get("left_bound")),
                "right_bound": _payload_points(raw_lane.get("right_bound")),
                "centerline": _payload_points(raw_lane.get("centerline")),
            }
        )

    if not lanes:
        raise ValueError("at least one valid lane is required")

    primary_lane_id = _sanitize_id(str(payload.get("primary_lane_id") or ""), lanes[0]["id"])
    lane_by_id = {str(lane["id"]): lane for lane in lanes}
    if primary_lane_id not in lane_by_id:
        primary_lane_id = lanes[0]["id"]
    primary_lane = lane_by_id[primary_lane_id]

    output_issue = _lane_export_issue(primary_lane)
    if output_issue is not None:
        raise ValueError(f"Lane {primary_lane_id} cannot be saved for raceline export: {output_issue}.")

    validated_previous_data = _validated_hd_topology(map_dir, previous_data, lanes)
    hd_map_path.parent.mkdir(parents=True, exist_ok=True)
    _write_hd_map_yaml(
        hd_map_path,
        raster,
        lanes,
        primary_lane_id,
        centerline_path,
        validated_previous_data,
    )
    _write_centerline_csv(centerline_path, primary_lane)
    _refresh_active_custom_line_after_hd_change(map_dir)
    return build_map_detail(config, str(map_dir))


def save_hd_map_version(config: ConsoleConfig, payload: dict[str, Any]) -> dict[str, Any]:
    map_dir_value = str(payload.get("map_dir") or "")
    if not map_dir_value:
        raise ValueError("map_dir is required")
    map_dir = resolve_allowed_path(config, map_dir_value)
    if not map_dir.exists() or not map_dir.is_dir():
        raise FileNotFoundError(f"map folder not found: {map_dir}")

    active_artifacts = _active_hd_artifact_paths(map_dir)
    if not active_artifacts["hd_map"].exists() or not active_artifacts["centerline_csv"].exists():
        raise FileNotFoundError("active HD map and centerline are required before saving a version")

    root = _version_root(map_dir)
    version_id = _next_hd_map_version_id(root)
    version_dir = root / version_id
    version_dir.mkdir(parents=True, exist_ok=False)
    version_artifacts = _version_artifact_paths(version_dir)

    copied: dict[str, str] = {}
    for key, source_path in active_artifacts.items():
        if not source_path.exists():
            continue
        target_path = version_artifacts[key]
        shutil.copy2(source_path, target_path)
        copied[key] = str(target_path)

    label = str(payload.get("label") or "").strip()
    if not label:
        label = version_id
    manifest = {
        "format": "jetpilot_hd_map_version_v1",
        "id": version_id,
        "label": label,
        "created_at": _now_iso(),
        "map_dir": str(map_dir),
        "hd_fingerprint": _version_fingerprint(version_dir),
        "artifacts": copied,
    }
    _write_json_file(version_dir / "manifest.json", manifest)
    _write_json_file(_version_active_path(map_dir), {"id": version_id, "activated_at": _now_iso()})
    return build_map_detail(config, str(map_dir))


def _archive_active_artifact(map_dir: Path, path: Path, archive_id: str) -> None:
    if not path.exists():
        return
    archive_dir = _version_root(map_dir) / "activation_archive" / archive_id
    archive_dir.mkdir(parents=True, exist_ok=True)
    shutil.move(str(path), str(archive_dir / path.name))


def activate_hd_map_version(config: ConsoleConfig, payload: dict[str, Any]) -> dict[str, Any]:
    map_dir_value = str(payload.get("map_dir") or "")
    version_id = _sanitize_id(str(payload.get("version_id") or ""), "")
    if not map_dir_value:
        raise ValueError("map_dir is required")
    if not version_id:
        raise ValueError("version_id is required")
    map_dir = resolve_allowed_path(config, map_dir_value)
    if not map_dir.exists() or not map_dir.is_dir():
        raise FileNotFoundError(f"map folder not found: {map_dir}")

    root = _version_root(map_dir).resolve()
    version_dir = (root / version_id).resolve()
    if not _is_relative_to(version_dir, root) or not version_dir.exists() or not version_dir.is_dir():
        raise FileNotFoundError(f"HD map version not found: {version_id}")

    version_artifacts = _version_artifact_paths(version_dir)
    if not version_artifacts["hd_map"].exists() or not version_artifacts["centerline_csv"].exists():
        raise FileNotFoundError(f"HD map version {version_id} is incomplete")

    active_artifacts = _active_hd_artifact_paths(map_dir)
    for key in ("hd_map", "centerline_csv"):
        shutil.copy2(version_artifacts[key], active_artifacts[key])

    archive_id = datetime.now(timezone.utc).astimezone().strftime("%Y%m%d_%H%M%S")
    for key in ("raceline_csv", "raceline_meta", "line_preview"):
        if version_artifacts[key].exists():
            shutil.copy2(version_artifacts[key], active_artifacts[key])
        else:
            _archive_active_artifact(map_dir, active_artifacts[key], archive_id)

    _write_json_file(_version_active_path(map_dir), {"id": version_id, "activated_at": _now_iso()})
    _refresh_active_custom_line_after_hd_change(map_dir)
    return build_map_detail(config, str(map_dir))


def save_section_gates(config: ConsoleConfig, payload: dict[str, Any]) -> dict[str, Any]:
    map_dir_value = str(payload.get("map_dir") or "")
    if not map_dir_value:
        raise ValueError("map_dir is required")
    map_dir = resolve_allowed_path(config, map_dir_value)
    if not map_dir.exists() or not map_dir.is_dir():
        raise FileNotFoundError(f"map folder not found: {map_dir}")

    name = map_dir.name
    hd_map_path = map_dir / f"{name}_hd_map.yaml"
    if not hd_map_path.exists():
        raise FileNotFoundError("HD map YAML is required before editing section gates. Save the HD map first.")

    data = load_yaml(hd_map_path)
    raster = _raster_from_hd_map(hd_map_path, data)
    if not raster:
        raise ValueError("HD map source_raster metadata is missing. Re-save the HD map before editing section gates.")

    lanes = _lanes_from_hd_data(data)
    if not lanes:
        raise ValueError("HD map has no lanes")
    primary_lane_id = _sanitize_id(str(data.get("primary_lane_id") or ""), lanes[0]["id"])
    lane_ids = {str(lane["id"]) for lane in lanes}
    if primary_lane_id not in lane_ids:
        primary_lane_id = lanes[0]["id"]

    gates = _payload_section_gates(payload.get("section_gates"), lane_ids)
    updated_data = dict(data)
    updated_data["section_gates"] = gates
    updated_data["sections"] = _build_sections_for_gates(data.get("sections"), gates, lanes)

    updated_data["junctions"] = _payload_junctions(
        data.get("junctions", []),
        updated_data["sections"],
        gates,
        lanes,
        _known_junction_route_ids(map_dir, lane_ids),
    )

    exports = data.get("exports")
    centerline_path = None
    if isinstance(exports, dict):
        centerline_path = _resolve_embedded_path(exports.get("primary_centerline_csv"), hd_map_path.parent)
    if centerline_path is None:
        centerline_path = map_dir / f"{name}_hd_map_centerline.csv"

    _write_hd_map_yaml(hd_map_path, raster, lanes, primary_lane_id, centerline_path, updated_data)
    _refresh_active_custom_line_after_hd_change(map_dir)
    return build_map_detail(config, str(map_dir))


def save_junctions(config: ConsoleConfig, payload: dict[str, Any]) -> dict[str, Any]:
    map_dir_value = str(payload.get("map_dir") or "")
    if not map_dir_value:
        raise ValueError("map_dir is required")
    map_dir = resolve_allowed_path(config, map_dir_value)
    if not map_dir.exists() or not map_dir.is_dir():
        raise FileNotFoundError(f"map folder not found: {map_dir}")

    name = map_dir.name
    hd_map_path = map_dir / f"{name}_hd_map.yaml"
    if not hd_map_path.exists():
        raise FileNotFoundError(
            "HD map YAML is required before editing junctions. Save the HD map first."
        )
    data = load_yaml(hd_map_path)
    raster = _raster_from_hd_map(hd_map_path, data)
    if not raster:
        raise ValueError(
            "HD map source_raster metadata is missing. Re-save the HD map before editing junctions."
        )
    lanes = _lanes_from_hd_data(data)
    if not lanes:
        raise ValueError("HD map has no lanes")
    lane_ids = {str(lane["id"]) for lane in lanes}
    sections = [section for section in data.get("sections", []) if isinstance(section, dict)]
    gates = [gate for gate in data.get("section_gates", []) if isinstance(gate, dict)]
    junctions = _payload_junctions(
        payload.get("junctions"),
        sections,
        gates,
        lanes,
        _known_junction_route_ids(map_dir, lane_ids),
    )
    updated_data = dict(data)
    updated_data["junctions"] = junctions

    primary_lane_id = _sanitize_id(str(data.get("primary_lane_id") or ""), lanes[0]["id"])
    if primary_lane_id not in lane_ids:
        primary_lane_id = lanes[0]["id"]
    exports = data.get("exports")
    centerline_path = None
    if isinstance(exports, dict):
        centerline_path = _resolve_embedded_path(
            exports.get("primary_centerline_csv"), hd_map_path.parent
        )
    if centerline_path is None:
        centerline_path = map_dir / f"{name}_hd_map_centerline.csv"

    _write_hd_map_yaml(
        hd_map_path, raster, lanes, primary_lane_id, centerline_path, updated_data
    )
    _refresh_active_custom_line_after_hd_change(map_dir)
    return build_map_detail(config, str(map_dir))
