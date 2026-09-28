"""Reuse authored driving geometry across venue maps with a planar rigid transform.

The VSLAM map and its TF remain untouched. Geometry is materialized in the new
map frame so all existing runtime consumers keep the same coordinate contract.
"""
from __future__ import annotations

import copy
import csv
import hashlib
import io
import json
import math
import os
import shutil
import tempfile
from pathlib import Path

from . import map_detail as maps

REGISTRATION = "hd_map_registration.json"
LANE_GEOMETRY = ("left_bound", "right_bound", "centerline", "drivable_left_bound",
                 "drivable_right_bound", "network_raceline")


def transform_parameters(payload):
    result = {}
    for key in ("x_m", "y_m", "yaw_deg"):
        try:
            value = float(payload.get(key, 0))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{key} must be a finite number") from exc
        if not math.isfinite(value) or abs(value) > 1e6:
            raise ValueError(f"{key} must be finite and within +/- 1000000")
        result[key] = value
    return result


def transform_point(point, transform):
    if not isinstance(point, (list, tuple)) or len(point) < 2:
        raise ValueError("geometry needs XY coordinates")
    x, y = float(point[0]), float(point[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("geometry coordinates must be finite")
    angle = math.radians(transform["yaw_deg"])
    c, s = math.cos(angle), math.sin(angle)
    return [c*x - s*y + transform["x_m"], s*x + c*y + transform["y_m"], *point[2:]]


def transform_hd_map(data, transform):
    result = copy.deepcopy(data)
    lanes = result.get("lanes", [])
    if not lanes or result.get("frame_id", "map") != "map":
        raise ValueError("source must contain lanes in the map frame")
    fingerprint = maps.network.source_hash(lanes, result.get("obstacles", []) or [])
    for lane in lanes:
        if lane.get("network_raceline") and lane.get("network_source_hash") != fingerprint:
            raise ValueError("source network raceline is stale; regenerate it first")
        for key in LANE_GEOMETRY:
            if key in lane:
                lane[key] = [transform_point(p, transform) for p in lane[key]]
    for gate in result.get("section_gates", []) or []:
        gate["line"] = [transform_point(p, transform) for p in gate["line"]]
    for junction in result.get("junctions", []) or []:
        if junction.get("position") is not None:
            junction["position"] = transform_point(junction["position"], transform)
    for obstacle in result.get("obstacles", []) or []:
        obstacle["polygon"] = [transform_point(p, transform) for p in obstacle["polygon"]]
    fingerprint = maps.network.source_hash(lanes, result.get("obstacles", []) or [])
    for lane in lanes:
        if lane.get("network_raceline"):
            lane["network_source_hash"] = fingerprint
    return result


def transform_csv(raw, transform, *, centerline=False):
    """Rotate headings; keep station, widths, curvature, speeds and acceleration."""
    delimiter = "," if centerline else ";"
    output = io.StringIO()
    writer = csv.writer(output, delimiter=delimiter, lineterminator="\n")
    count = 0
    for line in raw.splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            output.write(line + "\n")
            continue
        row = next(csv.reader([line], delimiter=delimiter))
        if len(row) != (4 if centerline else 7):
            raise ValueError("unsupported centerline/trajectory CSV columns")
        try:
            if not all(math.isfinite(float(v)) for v in row):
                raise ValueError("non-finite trajectory value")
            ix = 0 if centerline else 1
            xy = transform_point([float(row[ix]), float(row[ix+1])], transform)
            row[ix:ix+2] = [format(v, ".12g") for v in xy]
            if not centerline:
                yaw = float(row[3]) + math.radians(transform["yaw_deg"])
                row[3] = format(math.atan2(math.sin(yaw), math.cos(yaw)), ".12g")
        except (ValueError, TypeError) as exc:
            raise ValueError("invalid trajectory CSV value") from exc
        writer.writerow(row)
        count += 1
    if not count:
        raise ValueError("empty trajectory CSV")
    return output.getvalue()


def _files(folder):
    names = [f"{folder.name}{suffix}" for suffix in (
        "_hd_map.yaml", "_hd_map_centerline.csv", "_raceline.csv", "_raceline.meta.json",
        "_custom_line.csv", "_custom_line.meta.json")]
    names.append(maps.COMPETITION_ROUTE_CONFIG_FILE)
    paths = [folder / name for name in names]
    custom = folder / maps.CUSTOM_LINE_DIR
    if custom.is_symlink():
        raise ValueError("custom_lines must not be a symlink")
    if custom.exists():
        paths.extend(custom.rglob("*"))
    result = {}
    for path in sorted(paths):
        if path.is_symlink():
            raise ValueError(f"map artifact must not be a symlink: {path}")
        if path.is_file():
            result[path.relative_to(folder).as_posix()] = path.read_bytes()
    return result


def _fingerprint(files):
    digest = hashlib.sha256()
    for name, data in sorted(files.items()):
        digest.update(name.encode())
        digest.update(b"\0")
        digest.update(hashlib.sha256(data).digest())
    return digest.hexdigest()


def _prepare(config, payload):
    if not payload.get("source_map_dir") or not payload.get("map_dir"):
        raise ValueError("source_map_dir and map_dir are required")
    source = maps.resolve_allowed_path(config, str(payload["source_map_dir"]))
    target = maps.resolve_allowed_path(config, str(payload["map_dir"]))
    if source == target or source in target.parents or target in source.parents:
        raise ValueError("source and destination must be separate map folders")
    if not source.is_dir() or not target.is_dir():
        raise FileNotFoundError("source and destination map folders must exist")
    transform = transform_parameters(payload)
    files = _files(source)
    for suffix in ("_hd_map.yaml", "_hd_map_centerline.csv"):
        if source.name + suffix not in files:
            raise ValueError("source HD map and centerline are required")
    raster = maps._raster_from_map_yaml(target / "vslam_landmarks.yaml")
    if not raster or not all(raster.get(k) for k in ("width", "height", "resolution_m_per_px")):
        raise ValueError("generate the destination VSLAM landmark raster first")
    data = transform_hd_map(maps.load_yaml(source / f"{source.name}_hd_map.yaml"), transform)
    data["frame_id"] = "map"
    data["source_raster"] = {
        "map_yaml": str(target / "vslam_landmarks.yaml"), "image": raster["image_path"],
        "resolution_m_per_px": raster["resolution_m_per_px"],
        "origin_xy_yaw": raster["origin_xy_yaw"], "image_size_px": [raster["width"], raster["height"]],
    }
    data["exports"] = {"primary_centerline_csv": str(target / f"{target.name}_hd_map_centerline.csv")}
    maps.network.validate(data["lanes"])
    maps._require_primary_sections(data, data["lanes"], data["primary_lane_id"])
    maps.normalize_obstacles(data.get("obstacles", []))
    target_files = _files(target)
    for name in (REGISTRATION, "vslam_landmarks.yaml", f"{target.name}_line_preview.png",
                 "hd_map_versions/active.json"):
        path = target / name
        if path.is_symlink():
            raise ValueError("destination artifacts must not be symlinks")
        if path.is_file():
            target_files[name] = path.read_bytes()
    # Bind approval to both bundles, raster and exact transform (not just names).
    token = _fingerprint({"source": _fingerprint(files).encode(),
                          "target": _fingerprint(target_files).encode(),
                          "transform": json.dumps(transform, sort_keys=True).encode(),
                          "paths": json.dumps([str(source), str(target)]).encode()})
    return source, target, transform, files, data, token


def preview_registration(config, payload):
    source, target, transform, files, data, token = _prepare(config, payload)
    trajectories = []
    for name, raw in files.items():
        if name.endswith(".csv"):
            converted = transform_csv(raw.decode(), transform, centerline=name.endswith("_hd_map_centerline.csv"))
            if not name.endswith("_hd_map_centerline.csv"):
                trajectories.append({"name": name, "points": [
                    [float(row[1]), float(row[2])] for row in csv.reader(converted.splitlines(), delimiter=";")
                    if row and not row[0].lstrip().startswith("#")]})
    return {"source_map_dir": str(source), "map_dir": str(target), "transform": transform,
            "preview_token": token, "hd_map": data, "trajectories": trajectories,
            "replaces_existing": (target / f"{target.name}_hd_map.yaml").exists()}


def _remap_paths(value, source, target):
    if isinstance(value, dict):
        return {k: _remap_paths(v, source, target) for k, v in value.items()}
    if isinstance(value, list):
        return [_remap_paths(v, source, target) for v in value]
    if isinstance(value, str):
        if value.startswith(str(source) + "/"):
            relative = value[len(str(source))+1:]
            return str(target / _renamed(relative, source.name, target.name))
        if value.startswith(source.name + "_") and value.endswith((".csv", ".yaml", ".json")):
            return target.name + value[len(source.name):]
    return value


def _renamed(name, source, target):
    return target + name[len(source):] if name.startswith(source + "_") else name


def _stage_bundle(stage, source, target, files, data, transform):
    # Never make an already-invalid optimized line look current by rebinding hashes.
    source_lines = maps._read_custom_lines(source)
    if source_lines["active_issue"]:
        raise ValueError(f"source active custom line: {source_lines['active_issue']}")
    for item in source_lines["items"]:
        if not item["valid"]:
            raise ValueError(f"source custom line {item['id']}: {item['issue']}")
    for name, raw in files.items():
        dest = stage / _renamed(name, source.name, target.name)
        dest.parent.mkdir(parents=True, exist_ok=True)
        if name == f"{source.name}_hd_map.yaml":
            maps._write_json_file(dest, data)  # JSON is YAML; retain all authored attributes.
        elif name.endswith(".csv"):
            dest.write_text(transform_csv(raw.decode(), transform,
                centerline=name.endswith("_hd_map_centerline.csv")), encoding="utf-8")
        elif name.endswith((".json", ".yaml")):
            content = maps.load_yaml(source / name) if name.endswith(".yaml") else json.loads(raw)
            maps._write_json_file(dest, _remap_paths(content, source, target))
        else:
            raise ValueError(f"unsupported custom line artifact: {name}")
    # Rebind custom-line integrity metadata without regenerating optimized speeds.
    manifests = sorted((stage / maps.CUSTOM_LINE_DIR).glob("*/custom_line.json"))
    canonical = stage / f"{stage.name}_custom_line.meta.json"
    if canonical.exists():
        manifests.append(canonical)
    if manifests:
        layout = maps._custom_line_hd_layout(stage)
        for path in manifests:
            content = json.loads(path.read_text())
            trajectory = (path.parent / "trajectory.csv" if path != canonical
                          else stage / f"{stage.name}_custom_line.csv")
            old_trajectory = (source / path.relative_to(stage) if path != canonical
                              else source / f"{source.name}_custom_line.meta.json")
            old_meta = json.loads(old_trajectory.read_text())
            original_csv = (old_trajectory.parent / "trajectory.csv" if path != canonical
                            else source / f"{source.name}_custom_line.csv")
            if maps._sha256_file(original_csv) != old_meta.get("trajectory_sha256"):
                raise ValueError("source custom line trajectory hash mismatch")
            if "points" in content:
                for p in content["points"]:
                    p["x_m"], p["y_m"] = transform_point([p["x_m"], p["y_m"]], transform)
                content["content_hash"] = maps._custom_line_content_hash(
                    content["points"], content["closed_loop"], content.get("default_speed_mps"),
                    content.get("section_speeds_mps"), content.get("constraints"))
                content["base_hash"] = maps._custom_line_content_hash(content["points"], content["closed_loop"])
            content.update(trajectory_sha256=maps._sha256_file(trajectory),
                           hd_map_sha256=layout["hd_map_sha256"],
                           section_layout_fingerprint=layout["fingerprint"], section_layout_hash=layout["fingerprint"])
            source_type = content.get("source_type")
            if source_type in ("centerline", "raceline"):
                suffix = "_hd_map_centerline.csv" if source_type == "centerline" else "_raceline.csv"
                content["source_path"] = stage.name + suffix
                content["source_hash"] = content["source_sha256"] = maps._sha256_file(stage / (stage.name + suffix))
            maps._write_json_file(path, content)
        active = stage / maps.CUSTOM_LINE_DIR / "active.json"
        if active.exists() and canonical.exists():
            cache = json.loads(active.read_text())
            cache["trajectory_sha256"] = json.loads(canonical.read_text())["trajectory_sha256"]
            maps._write_json_file(active, cache)
        for item in maps._read_custom_lines(stage)["items"]:
            if not item["valid"]:
                raise ValueError(f"custom line {item['id']}: {item['issue']}")


def apply_registration(config, payload):
    source, target, transform, files, data, token = _prepare(config, payload)
    if payload.get("preview_token") != token:
        raise ValueError("map or transform changed; preview the alignment again before applying")
    with tempfile.TemporaryDirectory(prefix=".hd-registration-", dir=target.parent) as temporary:
        stage = Path(temporary) / target.name
        stage.mkdir()
        _stage_bundle(stage, source, target, files, data, transform)
        maps.build_map_detail(config, str(stage))  # Validate every reader before touching destination.
        maps._write_json_file(stage / REGISTRATION, {
            "format": "jetpilot_hd_map_registration_v1", "source_map_dir": str(source),
            "source_fingerprint": _fingerprint(files), "transform": transform,
            "equation": "p_target = R(yaw) * p_source + [x_m, y_m]",
            "created_at": maps._now_iso(),
        })
        destinations = set(_files(target)) | {p.relative_to(stage).as_posix() for p in stage.rglob("*") if p.is_file()}
        destinations.update({REGISTRATION, f"{target.name}_line_preview.png", "hd_map_versions/active.json"})
        for name in destinations:
            path = target / name
            if path.is_symlink() or any(p.is_symlink() for p in path.parents if p != target.parent):
                raise ValueError("destination artifacts must not traverse symlinks")
        backup = Path(tempfile.mkdtemp(prefix=".hd-registration-backup-", dir=target))
        originals = {}
        for name in destinations:
            path = target / name
            if path.is_file():
                saved = backup / name
                saved.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, saved)
                originals[name] = saved
        # Publish HD YAML last; it is the runtime reload trigger. Apply offline.
        ordered = sorted(destinations, key=lambda n: n == f"{target.name}_hd_map.yaml")
        changed = []
        try:
            for name in ordered:
                dest, staged = target / name, stage / name
                dest.parent.mkdir(parents=True, exist_ok=True)
                changed.append(name)
                if staged.is_file():
                    os.replace(staged, dest)
                elif dest.is_file():
                    dest.unlink()
        except Exception:
            for name in reversed(changed):
                if name in originals:
                    shutil.copy2(originals[name], target / name)
                else:
                    (target / name).unlink(missing_ok=True)
            raise
        custom_root = target / maps.CUSTOM_LINE_DIR
        if custom_root.is_dir():
            for directory in sorted(custom_root.rglob("*"), reverse=True):
                if directory.is_dir() and not any(directory.iterdir()):
                    directory.rmdir()
    result = maps.build_map_detail(config, str(target))
    result["registration_backup"] = str(backup)
    return result
