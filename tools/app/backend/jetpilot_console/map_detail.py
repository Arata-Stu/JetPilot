from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
import struct
from ast import literal_eval
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.parse import quote

from .config import ConsoleConfig
from .indexes import _artifact, _dir_size, _iso_mtime


HD_MAP_VERSION_DIR = "hd_map_versions"
HD_MAP_VERSION_ACTIVE_FILE = "active.json"


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
        return text[1:-1]
    if text.startswith("[") and text.endswith("]"):
        try:
            return literal_eval(text)
        except (SyntaxError, ValueError):
            return text
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


def _write_hd_map_yaml(
    output_path: Path,
    raster: dict[str, Any],
    lanes: list[dict[str, Any]],
    primary_lane_id: str,
    centerline_csv_path: Path,
    previous_data: dict[str, Any],
) -> None:
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
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


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
            gate_id = f"gate_{index:03d}"
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
            section: dict[str, Any] = {
                "id": str(previous.get("id") or f"section_{section_index:03d}"),
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
        return {"exists": False, "lanes": [], "section_gates": [], "sections": []}, None
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
                "speed_override_mps": raw_section.get("speed_override_mps"),
                "speed_scale": raw_section.get("speed_scale"),
                "class": raw_section.get("class"),
                "policy": raw_section.get("policy"),
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
    temporary_path = path.with_suffix(f"{path.suffix}.tmp")
    temporary_path.write_text(json.dumps(data, ensure_ascii=True, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary_path.replace(path)


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


def _map_record(map_dir: Path, map_root: Path | None = None) -> dict[str, Any]:
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
    return {
        "name": name,
        "display_name": _display_name(map_root, map_dir) if map_root is not None else name,
        "path": str(map_dir),
        "size_bytes": _dir_size(map_dir),
        "modified_at": _iso_mtime(map_dir),
        "fingerprint": directory_fingerprint(map_dir),
        "artifacts": artifacts,
        "complete_runtime_bundle": complete_runtime,
    }


def build_map_detail(config: ConsoleConfig, map_dir_value: str) -> dict[str, Any]:
    map_dir = resolve_allowed_path(config, map_dir_value)
    if not map_dir.exists() or not map_dir.is_dir():
        raise FileNotFoundError(f"map folder not found: {map_dir}")

    name = map_dir.name
    hd_map_path = map_dir / f"{name}_hd_map.yaml"
    landmark_yaml_path = map_dir / "vslam_landmarks.yaml"
    centerline_path = map_dir / f"{name}_hd_map_centerline.csv"
    raceline_path = map_dir / f"{name}_raceline.csv"
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

    return {
        "map": _map_record(map_dir, config.map_root),
        "raster": raster,
        "preview_image_url": preview_image,
        "hd_map": hd_map,
        "hd_map_versions": _read_hd_map_versions(map_dir),
        "centerline_csv": centerline,
        "raceline_csv": raceline,
        "odometry": odometry,
        "stats": {
            "lane_count": len(hd_map.get("lanes", [])),
            "primary_lane_id": hd_map.get("primary_lane_id") or "",
            "primary_centerline_points": len(primary_lane.get("centerline", [])) if primary_lane else 0,
            "primary_centerline_length_m": primary_lane.get("centerline_length_m", 0.0) if primary_lane else 0.0,
            "section_gate_count": len(hd_map.get("section_gates", [])),
            "section_count": len(hd_map.get("sections", [])),
            "speed_override_count": speed_override_count,
            "raceline_points": raceline["count"],
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

    hd_map_path.parent.mkdir(parents=True, exist_ok=True)
    _write_hd_map_yaml(hd_map_path, raster, lanes, primary_lane_id, centerline_path, previous_data)
    _write_centerline_csv(centerline_path, primary_lane)
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

    exports = data.get("exports")
    centerline_path = None
    if isinstance(exports, dict):
        centerline_path = _resolve_embedded_path(exports.get("primary_centerline_csv"), hd_map_path.parent)
    if centerline_path is None:
        centerline_path = map_dir / f"{name}_hd_map_centerline.csv"

    _write_hd_map_yaml(hd_map_path, raster, lanes, primary_lane_id, centerline_path, updated_data)
    return build_map_detail(config, str(map_dir))
