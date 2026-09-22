"""Pure trajectory and speed-envelope calculations, independent of file I/O."""
from __future__ import annotations
import math
from typing import Any

CUSTOM_LINE_POINT_EPSILON_M = 1.0e-9

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
        # Check lengths in meters, not their cubic product. A short but
        # distinct edge (e.g. next to a Section Gate) is still valid geometry.
        if any(not math.isfinite(length) or length <= CUSTOM_LINE_POINT_EPSILON_M for length in (a, b, c)):
            raise ValueError(f"points[{index}] does not have a valid curvature neighborhood")
        turn_sine = (
            ((current["x_m"] - previous["x_m"]) / a) * ((following["y_m"] - current["y_m"]) / b)
            - ((current["y_m"] - previous["y_m"]) / a) * ((following["x_m"] - current["x_m"]) / b)
        )
        kappa_values.append(2.0 * turn_sine / c)

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


