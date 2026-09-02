"""Pure-Python analysis core for throttle-to-speed calibration."""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics


@dataclass(frozen=True)
class CalibrationSample:
    time_s: float
    throttle: float
    speed_mps: float
    steering: float = 0.0
    brake: float = 0.0
    reverse: float = 0.0


@dataclass(frozen=True)
class CalibrationPoint:
    throttle: float
    steady_speed_mps: float
    speed_mad_mps: float
    speed_slope_mps2: float
    sample_count: int
    run_count: int


@dataclass(frozen=True)
class SegmentSummary:
    accepted: bool
    reason: str
    throttle: float
    start_s: float
    end_s: float
    sample_count: int
    steady_speed_mps: float = 0.0
    speed_mad_mps: float = 0.0
    speed_slope_mps2: float = 0.0


@dataclass(frozen=True)
class AnalysisConfig:
    minimum_throttle: float = 0.05
    throttle_tolerance: float = 0.005
    maximum_steering: float = 0.15
    maximum_command_age_s: float = 0.2
    maximum_sample_gap_s: float = 0.25
    minimum_segment_s: float = 4.0
    settling_time_s: float = 2.0
    steady_window_s: float = 2.0
    minimum_steady_samples: int = 20
    maximum_speed_slope_mps2: float = 0.10


@dataclass(frozen=True)
class AnalysisResult:
    points: list[CalibrationPoint]
    segments: list[SegmentSummary]


def _slope(samples: list[CalibrationSample]) -> float:
    if len(samples) < 2:
        return 0.0
    origin = samples[0].time_s
    xs = [sample.time_s - origin for sample in samples]
    ys = [sample.speed_mps for sample in samples]
    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(ys)
    denominator = sum((value - x_mean) ** 2 for value in xs)
    if denominator <= 1.0e-12:
        return 0.0
    return sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denominator


def _summarize_segment(
    samples: list[CalibrationSample], config: AnalysisConfig
) -> tuple[SegmentSummary, list[CalibrationSample]]:
    start = samples[0].time_s
    end = samples[-1].time_s
    throttle = statistics.median(sample.throttle for sample in samples)
    duration = end - start
    if duration < config.minimum_segment_s:
        return SegmentSummary(False, "segment_too_short", throttle, start, end, len(samples)), []

    window_start = max(start + config.settling_time_s, end - config.steady_window_s)
    steady = [sample for sample in samples if sample.time_s >= window_start]
    if len(steady) < config.minimum_steady_samples:
        return SegmentSummary(
            False, "too_few_steady_samples", throttle, start, end, len(steady)
        ), []

    speeds = [sample.speed_mps for sample in steady]
    median_speed = statistics.median(speeds)
    mad = statistics.median(abs(speed - median_speed) for speed in speeds)
    slope = _slope(steady)
    if abs(slope) > config.maximum_speed_slope_mps2:
        return SegmentSummary(
            False,
            "speed_not_settled",
            throttle,
            start,
            end,
            len(steady),
            median_speed,
            mad,
            slope,
        ), []
    return SegmentSummary(
        True, "accepted", throttle, start, end, len(steady), median_speed, mad, slope
    ), steady


def analyze_samples(
    samples: list[CalibrationSample], config: AnalysisConfig = AnalysisConfig()
) -> AnalysisResult:
    """Extract constant-throttle straight segments and aggregate steady speeds."""
    ordered = sorted(samples, key=lambda sample: sample.time_s)
    raw_segments: list[list[CalibrationSample]] = []
    current: list[CalibrationSample] = []

    def close_segment() -> None:
        nonlocal current
        if current:
            raw_segments.append(current)
            current = []

    for sample in ordered:
        valid = (
            math.isfinite(sample.time_s)
            and math.isfinite(sample.throttle)
            and math.isfinite(sample.speed_mps)
            and sample.throttle >= config.minimum_throttle
            and sample.speed_mps >= 0.0
            and abs(sample.steering) <= config.maximum_steering
            and sample.brake <= 1.0e-6
            and sample.reverse <= 1.0e-6
        )
        if not valid:
            close_segment()
            continue
        if current:
            gap = sample.time_s - current[-1].time_s
            reference_throttle = statistics.median(item.throttle for item in current)
            throttle_changed = (
                abs(sample.throttle - reference_throttle) > config.throttle_tolerance
            )
            if gap > config.maximum_sample_gap_s or throttle_changed:
                close_segment()
        current.append(sample)
    close_segment()

    summaries: list[SegmentSummary] = []
    accepted_windows: list[tuple[SegmentSummary, list[CalibrationSample]]] = []
    for segment in raw_segments:
        summary, steady = _summarize_segment(segment, config)
        summaries.append(summary)
        if summary.accepted:
            accepted_windows.append((summary, steady))

    grouped: dict[int, list[tuple[SegmentSummary, list[CalibrationSample]]]] = {}
    tolerance = max(config.throttle_tolerance, 1.0e-6)
    for summary, steady in accepted_windows:
        key = round(summary.throttle / tolerance)
        grouped.setdefault(key, []).append((summary, steady))

    points: list[CalibrationPoint] = []
    for entries in grouped.values():
        all_samples = [sample for _, window in entries for sample in window]
        speeds = [sample.speed_mps for sample in all_samples]
        median_speed = statistics.median(speeds)
        points.append(
            CalibrationPoint(
                throttle=statistics.median(summary.throttle for summary, _ in entries),
                steady_speed_mps=median_speed,
                speed_mad_mps=statistics.median(abs(speed - median_speed) for speed in speeds),
                speed_slope_mps2=statistics.median(
                    summary.speed_slope_mps2 for summary, _ in entries
                ),
                sample_count=len(all_samples),
                run_count=len(entries),
            )
        )
    points.sort(key=lambda point: point.throttle)
    return AnalysisResult(points=points, segments=summaries)
