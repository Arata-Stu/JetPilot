from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

from .analysis_worker import Progress, _atomic_json, _update_json_object, _utc_now
from .e2e_analysis import control_error_summary, finite_summary
from .map_detail import load_yaml


AGGRESSIVENESS_THRESHOLDS = {
    "steering_rate_per_s": 1.5,
    "throttle_rate_per_s": 1.5,
    "longitudinal_accel_mps2": 2.5,
    "jerk_mps3": 6.0,
    "lateral_accel_mps2": 3.0,
    "steering_saturation": 0.95,
    "throttle_saturation": 0.95,
}
AGGRESSIVENESS_WEIGHTS = {
    "steering_rate_per_s": 0.28,
    "throttle_rate_per_s": 0.12,
    "longitudinal_accel_mps2": 0.18,
    "jerk_mps3": 0.16,
    "lateral_accel_mps2": 0.20,
    "saturation": 0.06,
}


def _timed(records: object) -> list[dict[str, Any]]:
    if not isinstance(records, list):
        return []
    values = [dict(item) for item in records if isinstance(item, dict) and _finite(item.get("t")) is not None]
    values.sort(key=lambda item: float(item["t"]))
    return values


def _finite(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _nearest(records: list[dict[str, Any]], times: list[float], target: float, max_dt: float) -> dict[str, Any] | None:
    if not records:
        return None
    index = bisect.bisect_left(times, target)
    candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(records)]
    if not candidates:
        return None
    best = min(candidates, key=lambda candidate: abs(times[candidate] - target))
    return records[best] if abs(times[best] - target) <= max_dt else None


def _mode_name(record: Mapping[str, Any] | None) -> str:
    if not record:
        return ""
    raw = record.get("label") or record.get("name") or record.get("mode")
    aliases = {1: "AUTO", 2: "MANUAL", 3: "STOP", 4: "PROPO"}
    try:
        numeric = int(raw)
    except (TypeError, ValueError):
        return str(raw or "").upper()
    return aliases.get(numeric, str(numeric))


def _metadata(model_path: Path) -> dict[str, Any]:
    path = model_path.parent / "metadata.json"
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _preprocess(image: Any, model_input: Mapping[str, Any]) -> Any:
    import cv2
    import numpy as np

    shape = model_input.get("shape") if isinstance(model_input.get("shape"), list) else [1, 3, 120, 212]
    height = int(shape[-2]) if len(shape) >= 4 else 120
    width = int(shape[-1]) if len(shape) >= 4 else 212
    mean = model_input.get("mean") if isinstance(model_input.get("mean"), list) else [0.485, 0.456, 0.406]
    std = model_input.get("std") if isinstance(model_input.get("std"), list) else [0.229, 0.224, 0.225]
    resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    rgb = (rgb - np.asarray(mean, dtype=np.float32).reshape(1, 1, 3)) / np.asarray(
        std, dtype=np.float32
    ).reshape(1, 1, 3)
    return np.transpose(rgb, (2, 0, 1))[None, ...].astype(np.float32)


def _provider(requested: str, available: Sequence[str]) -> list[str]:
    normalized = requested.strip().lower()
    if normalized == "cuda" and "CUDAExecutionProvider" in available:
        return ["CUDAExecutionProvider", "CPUExecutionProvider"]
    if normalized in {"auto", ""} and "CUDAExecutionProvider" in available:
        return ["CUDAExecutionProvider", "CPUExecutionProvider"]
    return ["CPUExecutionProvider"]


def _supervised_predictions(
    analysis_dir: Path,
    timeline: dict[str, Any],
    model_path: Path,
    provider_name: str,
    max_control_dt_s: float,
    manual_only: bool,
    deadline_ms: float,
    progress: Progress,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    import cv2
    import numpy as np
    import onnxruntime as ort

    frames = _timed(timeline.get("frames"))
    teachers = _timed(timeline.get("controls"))
    modes = _timed(timeline.get("modes"))
    teacher_times = [float(item["t"]) for item in teachers]
    mode_times = [float(item["t"]) for item in modes]
    metadata = _metadata(model_path)
    model_input = metadata.get("input") if isinstance(metadata.get("input"), dict) else {}
    output_fields = (
        metadata.get("output", {}).get("fields")
        if isinstance(metadata.get("output"), dict)
        else None
    )
    fields = [str(item) for item in output_fields] if isinstance(output_fields, list) else ["steering", "throttle"]
    providers = _provider(provider_name, ort.get_available_providers())
    session_started = time.perf_counter_ns()
    session = ort.InferenceSession(str(model_path), providers=providers)
    session_init_ms = (time.perf_counter_ns() - session_started) / 1.0e6
    input_name = session.get_inputs()[0].name

    warmup_frame = next((analysis_dir / str(frame.get("path")) for frame in frames if frame.get("path")), None)
    if warmup_frame and warmup_frame.is_file():
        warmup_image = cv2.imread(str(warmup_frame), cv2.IMREAD_COLOR)
        if warmup_image is not None:
            warmup_tensor = _preprocess(warmup_image, model_input)
            for _ in range(5):
                session.run(None, {input_name: warmup_tensor})

    samples: list[dict[str, Any]] = []
    excluded_mode = 0
    missing_teacher = 0
    for index, frame in enumerate(frames):
        t = float(frame["t"])
        mode = _nearest(modes, mode_times, t, max_control_dt_s * 2.0)
        if manual_only and modes and _mode_name(mode) != "MANUAL":
            excluded_mode += 1
            continue
        frame_path = (analysis_dir / str(frame.get("path") or "")).resolve(strict=False)
        try:
            frame_path.relative_to(analysis_dir.resolve())
        except ValueError:
            continue
        image = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        started = time.perf_counter_ns()
        tensor = _preprocess(image, model_input)
        preprocessed = time.perf_counter_ns()
        outputs = session.run(None, {input_name: tensor})
        finished = time.perf_counter_ns()
        flat = np.asarray(outputs[0], dtype=np.float32).reshape(-1)
        decoded = {field: float(flat[field_index]) for field_index, field in enumerate(fields) if field_index < flat.size}
        teacher = _nearest(teachers, teacher_times, t, max_control_dt_s)
        if teacher is None:
            missing_teacher += 1
        steering_gt = _finite(teacher.get("steering")) if teacher else None
        throttle_gt = _finite(teacher.get("throttle")) if teacher else None
        steering_pred = _finite(decoded.get("steering"))
        throttle_pred = _finite(decoded.get("throttle"))
        preprocess_ms = (preprocessed - started) / 1.0e6
        inference_ms = (finished - preprocessed) / 1.0e6
        total_ms = (finished - started) / 1.0e6
        samples.append(
            {
                "t": round(t, 9),
                "stamp": str(frame.get("_timestamp_ns") or frame.get("stamp") or ""),
                "frame_path": str(frame.get("path") or ""),
                "mode": _mode_name(mode),
                "steering_gt": steering_gt,
                "steering_pred": steering_pred,
                "steering_error": (
                    round(steering_pred - steering_gt, 9)
                    if steering_pred is not None and steering_gt is not None
                    else None
                ),
                "throttle_gt": throttle_gt,
                "throttle_pred": throttle_pred,
                "throttle_error": (
                    round(throttle_pred - throttle_gt, 9)
                    if throttle_pred is not None and throttle_gt is not None
                    else None
                ),
                "teacher_dt_ms": (
                    round(abs(float(teacher["t"]) - t) * 1000.0, 6) if teacher else None
                ),
                "preprocess_ms": round(preprocess_ms, 6),
                "inference_ms": round(inference_ms, 6),
                "total_ms": round(total_ms, 6),
                "missed_deadline": total_ms > deadline_ms,
            }
        )
        if index and index % 100 == 0:
            progress.update(
                "e2e_inference",
                0.82 + 0.15 * index / max(1, len(frames)),
                f"E2E推論 {index:,}/{len(frames):,} frames",
            )

    metrics = {
        "sample_count": len(samples),
        "frame_count": len(frames),
        "excluded_non_manual": excluded_mode,
        "missing_teacher": missing_teacher,
        "steering": control_error_summary(samples, "steering"),
        "throttle": control_error_summary(samples, "throttle"),
        "preprocess_ms": finite_summary(item["preprocess_ms"] for item in samples),
        "inference_ms": finite_summary(item["inference_ms"] for item in samples),
        "total_ms": finite_summary(item["total_ms"] for item in samples),
        "deadline_ms": deadline_ms,
        "deadline_miss_count": sum(bool(item["missed_deadline"]) for item in samples),
        "deadline_miss_rate": round(
            sum(bool(item["missed_deadline"]) for item in samples) / max(1, len(samples)), 8
        ),
        "session_init_ms": round(session_init_ms, 6),
        "provider": session.get_providers(),
    }
    return samples, metrics


def _points(value: object) -> list[tuple[float, float]]:
    if not isinstance(value, list):
        return []
    points: list[tuple[float, float]] = []
    for item in value:
        if not isinstance(item, (list, tuple)) or len(item) < 2:
            continue
        x, y = _finite(item[0]), _finite(item[1])
        if x is not None and y is not None:
            points.append((x, y))
    return points


def _cumulative(points: list[tuple[float, float]], closed: bool) -> tuple[list[float], float]:
    values = [0.0]
    for first, second in zip(points, points[1:]):
        values.append(values[-1] + math.hypot(second[0] - first[0], second[1] - first[1]))
    total = values[-1] if values else 0.0
    if closed and len(points) > 2:
        total += math.hypot(points[0][0] - points[-1][0], points[0][1] - points[-1][1])
    return values, total


def _project(point: tuple[float, float], centerline: list[tuple[float, float]], closed: bool) -> tuple[float, float]:
    cumulative, _ = _cumulative(centerline, closed)
    count = len(centerline) if closed and len(centerline) > 2 else len(centerline) - 1
    best_s, best_distance = 0.0, float("inf")
    for index in range(max(0, count)):
        next_index = (index + 1) % len(centerline)
        ax, ay = centerline[index]
        bx, by = centerline[next_index]
        vx, vy = bx - ax, by - ay
        denominator = vx * vx + vy * vy
        if denominator <= 1.0e-12:
            continue
        ratio = max(0.0, min(1.0, ((point[0] - ax) * vx + (point[1] - ay) * vy) / denominator))
        cx, cy = ax + ratio * vx, ay + ratio * vy
        distance = math.hypot(point[0] - cx, point[1] - cy)
        if distance < best_distance:
            best_s = cumulative[index] + math.sqrt(denominator) * ratio
            best_distance = distance
    return best_s, best_distance


def _map_sections(map_dir: Path | None) -> dict[str, Any]:
    if map_dir is None:
        return {}
    path = map_dir / f"{map_dir.name}_hd_map.yaml"
    if not path.is_file():
        candidates = sorted(map_dir.glob("*_hd_map.yaml"))
        path = candidates[0] if candidates else path
    if not path.is_file():
        return {}
    data = load_yaml(path)
    lanes = data.get("lanes") if isinstance(data, dict) else None
    primary_id = str(data.get("primary_lane_id") or "") if isinstance(data, dict) else ""
    primary = next(
        (lane for lane in lanes or [] if isinstance(lane, dict) and str(lane.get("id")) == primary_id),
        None,
    )
    if not isinstance(primary, dict):
        return {}
    centerline = _points(primary.get("centerline"))
    if len(centerline) < 2:
        return {}
    return {
        "path": str(path),
        "lane_id": primary_id,
        "centerline": centerline,
        "closed_loop": bool(primary.get("closed_loop", True)),
        "sections": [item for item in data.get("sections", []) if isinstance(item, dict)],
    }


def _section_for_s(sections: list[dict[str, Any]], s_value: float, total: float, closed: bool) -> str:
    if not sections:
        return "unknown"
    s = s_value % total if closed and total > 0 else s_value
    for item in sections:
        start = _finite(item.get("start_s_m"))
        end = _finite(item.get("end_s_m"))
        if start is None or end is None:
            continue
        if closed and total > 0:
            start, end = start % total, end % total
            inside = (s >= start or s < end) if start > end else start <= s < end
        else:
            inside = start <= s < end
        if inside:
            return str(item.get("id") or "unknown")
    return "unknown"


def _enrich_trajectory(timeline: dict[str, Any], map_dir: Path | None) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    trajectory = timeline.get("trajectory") if isinstance(timeline.get("trajectory"), dict) else {}
    samples = _timed(trajectory.get("samples"))
    geometry = _map_sections(map_dir)
    if not samples or not geometry:
        return samples, geometry
    centerline = geometry["centerline"]
    closed = bool(geometry["closed_loop"])
    _, total = _cumulative(centerline, closed)
    previous_s: float | None = None
    lap = 0
    for sample in samples:
        x, y = _finite(sample.get("x")), _finite(sample.get("y"))
        if x is None or y is None:
            continue
        s_value, distance = _project((x, y), centerline, closed)
        if closed and previous_s is not None and total > 0 and previous_s - s_value > total * 0.5:
            lap += 1
        previous_s = s_value
        sample["course_s_m"] = round(s_value, 6)
        sample["cross_track_error_m"] = round(distance, 6)
        sample["section"] = _section_for_s(geometry["sections"], s_value, total, closed)
        sample["lap"] = lap
    trajectory["samples"] = samples
    timeline["trajectory"] = trajectory
    geometry["length_m"] = round(total, 6)
    geometry.pop("centerline", None)
    return samples, geometry


def _attach_pose_context(records: list[dict[str, Any]], trajectory: list[dict[str, Any]]) -> None:
    times = [float(item["t"]) for item in trajectory]
    for record in records:
        pose = _nearest(trajectory, times, float(record["t"]), 0.5)
        if pose:
            record["section"] = str(pose.get("section") or "unknown")
            record["lap"] = int(pose.get("lap") or 0)
            record["course_s_m"] = pose.get("course_s_m")
            record["cross_track_error_m"] = pose.get("cross_track_error_m")
            for key in (
                "speed_mps",
                "longitudinal_accel_mps2",
                "jerk_mps3",
                "yaw_rate_radps",
                "lateral_accel_mps2",
            ):
                record[key] = pose.get(key)


def _attach_recorded_sections(
    records: list[dict[str, Any]], section_records: list[dict[str, Any]]
) -> None:
    times = [float(item["t"]) for item in section_records]
    for record in records:
        if str(record.get("section") or "").strip() not in {"", "unknown"}:
            continue
        section = _nearest(section_records, times, float(record["t"]), 1.0)
        if section and section.get("section"):
            record["section"] = str(section["section"])


def _attach_aggressiveness_to_trajectory(
    trajectory: list[dict[str, Any]], records: list[dict[str, Any]]
) -> None:
    times = [float(item["t"]) for item in records]
    for sample in trajectory:
        control = _nearest(records, times, float(sample["t"]), 0.25)
        if control is None:
            continue
        vehicle_score = _finite(sample.get("aggressiveness_score")) or 0.0
        control_score = _finite(control.get("aggressiveness_score")) or 0.0
        sample["aggressiveness_score"] = round(max(vehicle_score, control_score), 3)


def _attach_applied_controls(
    records: list[dict[str, Any]], applied_controls: list[dict[str, Any]]
) -> None:
    times = [float(item["t"]) for item in applied_controls]
    for record in records:
        applied = _nearest(applied_controls, times, float(record["t"]), 0.1)
        if not applied:
            continue
        steering_pred = _finite(record.get("steering_pred"))
        throttle_pred = _finite(record.get("throttle_pred"))
        steering_applied = _finite(applied.get("steering"))
        throttle_applied = _finite(applied.get("throttle"))
        record["steering_applied"] = steering_applied
        record["throttle_applied"] = throttle_applied
        record["steering_applied_error"] = (
            round(steering_pred - steering_applied, 9)
            if steering_pred is not None and steering_applied is not None
            else None
        )
        record["throttle_applied_error"] = (
            round(throttle_pred - throttle_applied, 9)
            if throttle_pred is not None and throttle_applied is not None
            else None
        )


def _angle_delta(current: float, previous: float) -> float:
    return (current - previous + math.pi) % (2.0 * math.pi) - math.pi


def _enrich_trajectory_dynamics(trajectory: list[dict[str, Any]]) -> None:
    previous: dict[str, Any] | None = None
    previous_accel: float | None = None
    for sample in trajectory:
        speed = _finite(sample.get("speed_mps"))
        if speed is None and previous is not None:
            dt = float(sample["t"]) - float(previous["t"])
            x, y = _finite(sample.get("x")), _finite(sample.get("y"))
            px, py = _finite(previous.get("x")), _finite(previous.get("y"))
            if 0.001 <= dt <= 1.0 and None not in {x, y, px, py}:
                speed = math.hypot(float(x) - float(px), float(y) - float(py)) / dt
                sample["speed_mps"] = round(speed, 6)
        if previous is not None:
            dt = float(sample["t"]) - float(previous["t"])
            previous_speed = _finite(previous.get("speed_mps"))
            if 0.001 <= dt <= 1.0 and speed is not None and previous_speed is not None:
                accel = (speed - previous_speed) / dt
                sample["longitudinal_accel_mps2"] = round(accel, 6)
                if previous_accel is not None:
                    sample["jerk_mps3"] = round((accel - previous_accel) / dt, 6)
                previous_accel = accel
            yaw = _finite(sample.get("yaw"))
            previous_yaw = _finite(previous.get("yaw"))
            if 0.001 <= dt <= 1.0 and yaw is not None and previous_yaw is not None:
                yaw_rate = _angle_delta(yaw, previous_yaw) / dt
                sample["yaw_rate_radps"] = round(yaw_rate, 6)
                if speed is not None:
                    sample["lateral_accel_mps2"] = round(speed * yaw_rate, 6)
        previous = sample


def _enrich_control_dynamics(records: list[dict[str, Any]]) -> int:
    previous: dict[str, Any] | None = None
    previous_rate_sign = 0
    oscillations = 0
    for record in records:
        steering = _finite(
            record.get("steering_pred")
            if record.get("steering_pred") is not None
            else record.get("steering")
        )
        throttle = _finite(
            record.get("throttle_pred")
            if record.get("throttle_pred") is not None
            else record.get("throttle")
        )
        record["steering_abs"] = abs(steering) if steering is not None else None
        record["steering_saturated"] = (
            abs(steering) >= AGGRESSIVENESS_THRESHOLDS["steering_saturation"]
            if steering is not None
            else False
        )
        record["throttle_saturated"] = (
            throttle >= AGGRESSIVENESS_THRESHOLDS["throttle_saturation"]
            if throttle is not None
            else False
        )
        if previous is not None:
            dt = float(record["t"]) - float(previous["t"])
            previous_steering = _finite(previous.get("steering_pred"))
            previous_throttle = _finite(previous.get("throttle_pred"))
            if 0.001 <= dt <= 1.0:
                if steering is not None and previous_steering is not None:
                    steering_rate = (steering - previous_steering) / dt
                    record["steering_rate_per_s"] = round(steering_rate, 6)
                    rate_sign = 1 if steering_rate > 0.1 else -1 if steering_rate < -0.1 else 0
                    if rate_sign and previous_rate_sign and rate_sign != previous_rate_sign:
                        oscillations += 1
                    if rate_sign:
                        previous_rate_sign = rate_sign
                if throttle is not None and previous_throttle is not None:
                    record["throttle_rate_per_s"] = round(
                        (throttle - previous_throttle) / dt, 6
                    )
        previous = record
    return oscillations


def _aggressiveness_components(record: Mapping[str, Any]) -> dict[str, float]:
    saturation = 1.0 if record.get("steering_saturated") or record.get("throttle_saturated") else 0.0
    values = {
        "steering_rate_per_s": abs(_finite(record.get("steering_rate_per_s")) or 0.0),
        "throttle_rate_per_s": abs(_finite(record.get("throttle_rate_per_s")) or 0.0),
        "longitudinal_accel_mps2": abs(_finite(record.get("longitudinal_accel_mps2")) or 0.0),
        "jerk_mps3": abs(_finite(record.get("jerk_mps3")) or 0.0),
        "lateral_accel_mps2": abs(_finite(record.get("lateral_accel_mps2")) or 0.0),
        "saturation": saturation,
    }
    return {
        key: min(1.0, value / AGGRESSIVENESS_THRESHOLDS[key])
        if key != "saturation"
        else value
        for key, value in values.items()
    }


def _aggressiveness_score(record: Mapping[str, Any]) -> float:
    components = _aggressiveness_components(record)
    available = {
        "steering_rate_per_s": record.get("steering_rate_per_s") is not None,
        "throttle_rate_per_s": record.get("throttle_rate_per_s") is not None,
        "longitudinal_accel_mps2": record.get("longitudinal_accel_mps2") is not None,
        "jerk_mps3": record.get("jerk_mps3") is not None,
        "lateral_accel_mps2": record.get("lateral_accel_mps2") is not None,
        "saturation": record.get("steering_abs") is not None or record.get("steering_saturated") is not None,
    }
    denominator = sum(
        AGGRESSIVENESS_WEIGHTS[key] for key, is_available in available.items() if is_available
    )
    if denominator <= 0.0:
        return 0.0
    return round(
        100.0
        * sum(
            components[key] * AGGRESSIVENESS_WEIGHTS[key]
            for key, is_available in available.items()
            if is_available
        )
        / denominator,
        3,
    )


def _teacher_free_metrics(
    records: list[dict[str, Any]], trajectory: list[dict[str, Any]], oscillations: int = 0
) -> dict[str, Any]:
    duration = max(
        0.0,
        max((float(item["t"]) for item in [*records, *trajectory]), default=0.0)
        - min((float(item["t"]) for item in [*records, *trajectory]), default=0.0),
    )
    scores = [item.get("aggressiveness_score") for item in records]
    score_summary = finite_summary(scores)
    score = _finite(score_summary.get("p95")) or 0.0
    label = "cautious" if score < 25 else "balanced" if score < 50 else "aggressive" if score < 75 else "extreme"
    return {
        "score": round(score, 3),
        "classification": label,
        "score_distribution": score_summary,
        "aggressive_sample_rate": round(
            sum((_finite(item.get("aggressiveness_score")) or 0.0) >= 60.0 for item in records)
            / max(1, len(records)),
            8,
        ),
        "steering_abs": finite_summary(item.get("steering_abs") for item in records),
        "steering_rate_abs_per_s": finite_summary(abs(_finite(item.get("steering_rate_per_s")) or 0.0) for item in records if item.get("steering_rate_per_s") is not None),
        "throttle_rate_abs_per_s": finite_summary(abs(_finite(item.get("throttle_rate_per_s")) or 0.0) for item in records if item.get("throttle_rate_per_s") is not None),
        "steering_saturation_rate": round(sum(bool(item.get("steering_saturated")) for item in records) / max(1, len(records)), 8),
        "throttle_saturation_rate": round(sum(bool(item.get("throttle_saturated")) for item in records) / max(1, len(records)), 8),
        "steering_oscillation_count": oscillations,
        "steering_oscillations_per_min": round(oscillations * 60.0 / max(1.0, duration), 6),
        "speed_mps": finite_summary(item.get("speed_mps") for item in trajectory),
        "longitudinal_accel_abs_mps2": finite_summary(abs(_finite(item.get("longitudinal_accel_mps2")) or 0.0) for item in trajectory if item.get("longitudinal_accel_mps2") is not None),
        "jerk_abs_mps3": finite_summary(abs(_finite(item.get("jerk_mps3")) or 0.0) for item in trajectory if item.get("jerk_mps3") is not None),
        "yaw_rate_abs_radps": finite_summary(abs(_finite(item.get("yaw_rate_radps")) or 0.0) for item in trajectory if item.get("yaw_rate_radps") is not None),
        "lateral_accel_abs_mps2": finite_summary(abs(_finite(item.get("lateral_accel_mps2")) or 0.0) for item in trajectory if item.get("lateral_accel_mps2") is not None),
        "cross_track_error_m": finite_summary(item.get("cross_track_error_m") for item in trajectory),
        "thresholds": AGGRESSIVENESS_THRESHOLDS,
        "weights": AGGRESSIVENESS_WEIGHTS,
    }


def _aggressive_events(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    labels = {
        "steering_rate_per_s": "steering rate",
        "throttle_rate_per_s": "throttle rate",
        "longitudinal_accel_mps2": "longitudinal acceleration",
        "jerk_mps3": "jerk",
        "lateral_accel_mps2": "lateral acceleration",
        "saturation": "control saturation",
    }

    def reasons(record: Mapping[str, Any]) -> list[str]:
        components = _aggressiveness_components(record)
        return [labels[key] for key, value in components.items() if value >= 0.7]

    selected = [item for item in records if (_finite(item.get("aggressiveness_score")) or 0.0) >= 60.0]
    events: list[dict[str, Any]] = []
    for record in selected:
        if events and float(record["t"]) - float(events[-1]["t_end"]) <= 0.5:
            event = events[-1]
            event["t_end"] = float(record["t"])
            if float(record["aggressiveness_score"]) > float(event["peak_score"]):
                event["peak_score"] = record["aggressiveness_score"]
                event["peak_t"] = record["t"]
                event["reasons"] = reasons(record)
        else:
            events.append(
                {
                    "t_start": float(record["t"]),
                    "t_end": float(record["t"]),
                    "peak_t": float(record["t"]),
                    "peak_score": record["aggressiveness_score"],
                    "section": str(record.get("section") or "unknown"),
                    "reasons": reasons(record),
                }
            )
    for event in events:
        event["duration_s"] = round(float(event["t_end"]) - float(event["t_start"]), 6)
    return events


def _section_metrics(records: list[dict[str, Any]], trajectory: list[dict[str, Any]]) -> list[dict[str, Any]]:
    sections = sorted({str(item.get("section")) for item in [*records, *trajectory] if item.get("section")})
    output: list[dict[str, Any]] = []
    for section in sections:
        control_rows = [item for item in records if str(item.get("section")) == section]
        pose_rows = [item for item in trajectory if str(item.get("section")) == section]
        output.append(
            {
                "section": section,
                "t_start": min(
                    (float(item["t"]) for item in [*control_rows, *pose_rows]),
                    default=None,
                ),
                "t_end": max(
                    (float(item["t"]) for item in [*control_rows, *pose_rows]),
                    default=None,
                ),
                "sample_count": len(control_rows),
                "pose_count": len(pose_rows),
                "steering": control_error_summary(control_rows, "steering"),
                "throttle": control_error_summary(control_rows, "throttle"),
                "steering_applied": control_error_summary(
                    control_rows, "steering_applied"
                ),
                "throttle_applied": control_error_summary(
                    control_rows, "throttle_applied"
                ),
                "inference_ms": finite_summary(item.get("inference_ms") for item in control_rows),
                "pipeline_latency_ms": finite_summary(item.get("pipeline_latency_ms") for item in control_rows),
                "cross_track_error_m": finite_summary(item.get("cross_track_error_m") for item in pose_rows),
                "speed_mps": finite_summary(item.get("speed_mps") for item in pose_rows),
                "teacher_free": _teacher_free_metrics(control_rows, pose_rows),
                "aggressive_event_count": sum(
                    (_finite(item.get("aggressiveness_score")) or 0.0) >= 60.0
                    for item in control_rows
                ),
                "laps": sorted({int(item.get("lap") or 0) for item in [*control_rows, *pose_rows]}),
            }
        )
    return output


def _write_predictions(path: Path, records: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "t", "stamp", "frame_path", "mode", "section", "lap", "course_s_m",
        "steering_gt", "steering_pred", "steering_error", "throttle_gt",
        "steering_applied", "steering_applied_error", "throttle_pred", "throttle_error",
        "throttle_applied", "throttle_applied_error", "teacher_dt_ms", "preprocess_ms",
        "inference_ms", "total_ms", "missed_deadline", "steering_abs",
        "steering_rate_per_s", "throttle_rate_per_s", "speed_mps",
        "longitudinal_accel_mps2", "jerk_mps3", "yaw_rate_radps",
        "lateral_accel_mps2", "aggressiveness_score", "aggressive",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(records)


def run(args: argparse.Namespace) -> dict[str, Any]:
    analysis_dir = Path(args.analysis_dir).expanduser().resolve()
    timeline_path = analysis_dir / "timeline.json"
    timeline = json.loads(timeline_path.read_text(encoding="utf-8"))
    if not isinstance(timeline, dict):
        raise ValueError("timeline.json must contain an object")
    progress = Progress(analysis_dir / "status.json")
    progress.update("e2e_prepare", 0.81, "E2E評価を準備しています。")
    map_dir = Path(args.map_dir).expanduser().resolve() if args.map_dir else None
    trajectory, course = _enrich_trajectory(timeline, map_dir)
    _enrich_trajectory_dynamics(trajectory)
    for sample in trajectory:
        sample["aggressiveness_score"] = _aggressiveness_score(sample)

    if args.mode == "supervised":
        if not args.model:
            raise ValueError("supervised E2E analysis requires --model")
        records, metrics = _supervised_predictions(
            analysis_dir,
            timeline,
            Path(args.model).expanduser().resolve(),
            args.provider,
            args.max_control_dt_s,
            args.manual_only,
            args.deadline_ms,
            progress,
        )
    else:
        diagnostics = _timed(timeline.get("e2e_diagnostics"))
        diagnostic_times = [float(item["t"]) for item in diagnostics]
        records = []
        for item in _timed(timeline.get("controls")):
            diagnostic = _nearest(diagnostics, diagnostic_times, float(item["t"]), 0.25)
            records.append(
                {
                    **item,
                    "steering_pred": _finite(item.get("steering")),
                    "throttle_pred": _finite(item.get("throttle")),
                    "inference_ms": (
                        _finite(
                            diagnostic.get("inference_ms")
                            if diagnostic.get("inference_ms") is not None
                            else diagnostic.get("decoder_callback_ms")
                        )
                        if diagnostic
                        else None
                    ),
                    "pipeline_latency_ms": (
                        _finite(diagnostic.get("capture_to_command_ms")) if diagnostic else None
                    ),
                    "missed_deadline": bool(diagnostic.get("missed_deadline")) if diagnostic else False,
                }
            )
        metrics = {
            "sample_count": len(records),
            "pipeline_latency_ms": finite_summary(
                item.get("capture_to_command_ms") for item in diagnostics
            ),
            "decoder_callback_ms": finite_summary(
                item.get("decoder_callback_ms") for item in diagnostics
            ),
            "deadline_miss_count": sum(bool(item.get("missed_deadline")) for item in diagnostics),
        }
    applied_controls = _timed(timeline.get("comparison_controls"))
    _attach_applied_controls(records, applied_controls)
    metrics["steering_applied"] = control_error_summary(
        records, "steering_applied"
    )
    metrics["throttle_applied"] = control_error_summary(
        records, "throttle_applied"
    )
    _attach_pose_context(records, trajectory)
    recorded_sections = _timed(timeline.get("sections"))
    _attach_recorded_sections(trajectory, recorded_sections)
    _attach_recorded_sections(records, recorded_sections)
    oscillations = _enrich_control_dynamics(records)
    for record in records:
        record["aggressiveness_score"] = _aggressiveness_score(record)
        record["aggressive"] = record["aggressiveness_score"] >= 60.0
    _attach_aggressiveness_to_trajectory(trajectory, records)
    teacher_free = _teacher_free_metrics(records, trajectory, oscillations)
    events = _aggressive_events(records)
    teacher_free["event_count"] = len(events)
    metrics["teacher_free"] = teacher_free
    sections = _section_metrics(records, trajectory)
    e2e_payload = {
        "schema_version": 2,
        "mode": args.mode,
        "model": str(Path(args.model).resolve()) if args.model else "",
        "provider": args.provider,
        "teacher_topic": args.teacher_topic,
        "prediction_topic": args.prediction_topic,
        "applied_control_topic": args.applied_control_topic,
        "predictions": records,
        "recorded_applied_controls": applied_controls,
        "latency": _timed(timeline.get("e2e_diagnostics")),
        "metrics": metrics,
        "sections": sections,
        "events": events,
        "course": course,
    }
    timeline["e2e"] = e2e_payload
    (analysis_dir / "e2e").mkdir(parents=True, exist_ok=True)
    _atomic_json(timeline_path, timeline)
    _atomic_json(
        analysis_dir / "e2e" / "metrics.json",
        {**metrics, "sections": sections, "events": events, "course": course},
    )
    _write_predictions(analysis_dir / "e2e" / "predictions.csv", records)

    def update_manifest(previous: dict[str, object]) -> Mapping[str, object]:
        return {
            **previous,
            "analysis_kind": "e2e",
            "e2e": {
                "mode": args.mode,
                "model": str(Path(args.model).resolve()) if args.model else "",
                "metrics": metrics,
                "section_count": len(sections),
            },
            "updated_at": _utc_now(),
        }

    manifest = _update_json_object(analysis_dir / "manifest.json", update_manifest)
    progress.update("complete", 1.0, "E2E解析が完了しました。", status="completed")
    return manifest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Add E2E evaluation data to a JetPilot analysis timeline.")
    parser.add_argument("--analysis-dir", required=True)
    parser.add_argument("--mode", choices=("supervised", "offline_localization", "recorded_localization"), required=True)
    parser.add_argument("--model", default="")
    parser.add_argument("--provider", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--map-dir", default="")
    parser.add_argument("--teacher-topic", default="")
    parser.add_argument("--prediction-topic", default="")
    parser.add_argument("--applied-control-topic", default="")
    parser.add_argument("--max-control-dt-s", type=float, default=0.1)
    parser.add_argument("--deadline-ms", type=float, default=33.3)
    parser.add_argument("--manual-only", action=argparse.BooleanOptionalAction, default=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        run(args)
        return 0
    except Exception as error:  # noqa: BLE001
        analysis_dir = Path(args.analysis_dir).expanduser().resolve()
        Progress(analysis_dir / "status.json").update("failed", 1.0, str(error), status="failed")
        raise


if __name__ == "__main__":
    raise SystemExit(main())
