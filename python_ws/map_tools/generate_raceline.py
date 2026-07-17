#!/usr/bin/env python3
"""
Raceline generation from a centerline CSV using global_racetrajectory_optimization.

The output follows the common F1TENTH raceline layout:
  s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
"""

from __future__ import annotations

import argparse
import inspect
import json
import math
import os
import sys
import time
from typing import Tuple

import numpy as np
from scipy import interpolate, spatial


VALID_DIRECTIONS = ("forward", "reverse", "both")
VALID_PRESETS = ("default", "race-stacks")
DEFAULT_VEHICLE_WIDTH_M = 0.25
DEFAULT_SAFETY_MARGIN_M = 0.05


def nonnegative_finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("must be a finite value greater than or equal to 0")
    return parsed


def effective_vehicle_envelope_width(vehicle_width: float, safety_margin: float) -> float:
    """Return the width reserved by the optimizer.

    ``safety_margin`` is the desired boundary clearance on each side of the
    vehicle, so the optimizer footprint is the physical width plus two margins.
    The centerline track widths are intentionally left unchanged; the upstream
    optimizer already applies ``w_veh / 2`` to both track boundaries.
    """

    values = {
        "vehicle width": vehicle_width,
        "safety margin": safety_margin,
    }
    for label, value in values.items():
        if not math.isfinite(value) or value < 0.0:
            raise RuntimeError(f"{label} must be a finite value greater than or equal to 0 m.")

    return vehicle_width + 2.0 * safety_margin


def validate_track_clearance(
    widths: np.ndarray,
    vehicle_width: float,
    safety_margin: float,
    *,
    source: str,
) -> float:
    """Validate that the centerline widths can contain the requested envelope."""

    if widths.ndim != 2 or widths.shape[1] != 2:
        raise RuntimeError(f"{source} widths must have right and left columns.")
    if not np.all(np.isfinite(widths)):
        index = int(np.argwhere(~np.isfinite(widths))[0][0])
        raise RuntimeError(f"{source} contains a non-finite track width at point {index}.")
    if np.any(widths < 0.0):
        index = int(np.argwhere(widths < 0.0)[0][0])
        right, left = widths[index]
        raise RuntimeError(
            f"{source} contains a negative track width at point {index} "
            f"(right={right:.4g} m, left={left:.4g} m)."
        )

    required_width = effective_vehicle_envelope_width(vehicle_width, safety_margin)
    total_width = widths[:, 0] + widths[:, 1]
    if np.any(total_width <= 0.0):
        index = int(np.argwhere(total_width <= 0.0)[0][0])
        raise RuntimeError(f"{source} has no usable track width at point {index}.")

    # A tiny tolerance avoids rejecting values that only differ after floating
    # point interpolation. No boundary is moved or inflated automatically.
    too_narrow = total_width + 1e-9 < required_width
    if np.any(too_narrow):
        narrow_indices = np.flatnonzero(too_narrow)
        index = int(narrow_indices[np.argmin(total_width[narrow_indices])])
        available = float(total_width[index])
        raise RuntimeError(
            f"{source} is too narrow for the requested vehicle envelope at point {index}: "
            f"available={available:.4g} m, required={required_width:.4g} m "
            f"(vehicle={vehicle_width:.4g} m + 2 x margin={safety_margin:.4g} m). "
            "Reduce --vehicle-width-m/--safety-margin-m or correct the HD map bounds; "
            "track widths are not expanded automatically."
        )

    return required_width


def remove_duplicate_points(points: np.ndarray, *extra_columns: np.ndarray) -> Tuple[np.ndarray, ...]:
    if len(points) < 2:
        return (points, *extra_columns)

    keep = np.ones(len(points), dtype=bool)
    keep[1:] = np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9
    if np.linalg.norm(points[0] - points[-1]) <= 1e-9:
        keep[-1] = False

    return (points[keep], *(col[keep] for col in extra_columns))


def cumulative_s(points: np.ndarray, closed: bool = True) -> Tuple[np.ndarray, float]:
    pts = points
    if closed:
        pts = np.vstack([points, points[0]])

    seg_len = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg_len)])
    return s[:-1] if closed else s, float(s[-1])


def load_centerline(path: str) -> Tuple[np.ndarray, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", comments="#", dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 4:
        raise RuntimeError("Centerline CSV must contain x, y, w_tr_right, w_tr_left columns.")

    points = data[:, :2].astype(np.float64)
    widths = data[:, 2:4].astype(np.float64)

    if len(points) < 8:
        raise RuntimeError("Centerline needs at least 8 points to create a raceline.")
    if not np.all(np.isfinite(points)):
        index = int(np.argwhere(~np.isfinite(points))[0][0])
        raise RuntimeError(f"Centerline contains a non-finite coordinate at point {index}.")

    return points, widths


def reverse_centerline(centerline: np.ndarray, widths: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return centerline[::-1].copy(), widths[::-1][:, [1, 0]].copy()


def output_path_for_direction(base_output_path: str, direction: str) -> str:
    if direction == "forward":
        return base_output_path

    root, ext = os.path.splitext(base_output_path)
    return f"{root}_{direction}{ext}"


def metadata_path_for_output(output_path: str) -> str:
    root, _ = os.path.splitext(output_path)
    return f"{root}.meta.json"


def write_raceline_metadata(
    *,
    output_path: str,
    centerline_path: str,
    direction: str,
    backend: str,
    preset: str,
    opt_type: str,
    vehicle_width: float,
    safety_margin: float,
    widths: np.ndarray,
    point_count: int,
    track_length: float,
) -> str:
    metadata_path = metadata_path_for_output(output_path)
    metadata = {
        "format": "jetpilot_raceline_metadata_v1",
        "raceline_csv": os.path.abspath(output_path),
        "source_centerline_csv": os.path.abspath(centerline_path),
        "direction": direction,
        "backend": backend,
        "preset": preset,
        "opt_type": opt_type,
        "vehicle_clearance": {
            "vehicle_width_m": float(vehicle_width),
            "safety_margin_m_per_side": float(safety_margin),
            "effective_envelope_width_m": float(
                effective_vehicle_envelope_width(vehicle_width, safety_margin)
            ),
            "min_available_track_width_m": float(np.min(np.sum(widths, axis=1))),
        },
        "result": {
            "point_count": int(point_count),
            "closed_track_length_m": float(track_length),
        },
    }

    temporary_path = f"{metadata_path}.tmp"
    with open(temporary_path, "w", encoding="utf-8") as handle:
        json.dump(metadata, handle, ensure_ascii=True, indent=2, sort_keys=True)
        handle.write("\n")
    os.replace(temporary_path, metadata_path)
    return metadata_path


def default_optimizer_root() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))

    candidates = [
        os.path.abspath(os.path.join(script_dir, "..", "global_racetrajectory_optimization")),
        os.path.abspath(os.path.join(os.getcwd(), "tmp", "global_racetrajectory_optimization")),
        "/workspaces/python_ws/global_racetrajectory_optimization",
    ]

    for candidate in candidates:
        if os.path.isdir(candidate):
            return candidate

    return candidates[0]


def preset_defaults(preset: str) -> dict:
    if preset == "race-stacks":
        return {
            "opt_type": "mincurv",
            "curvature_limit": 1.0,
            "global_opt_stepsize_prep": 0.10,
            "global_opt_stepsize_reg": 0.50,
            "global_opt_stepsize_after_opt": 0.10,
            "global_opt_spline_smoothing": 10.0,
        }

    return {
        "opt_type": "mincurv",
        "curvature_limit": 3.0,
        "global_opt_stepsize_prep": 0.10,
        "global_opt_stepsize_reg": 0.30,
        "global_opt_stepsize_after_opt": 0.10,
        "global_opt_spline_smoothing": 10.0,
    }


class ProgressReporter:
    def __init__(self, enabled: bool) -> None:
        self.enabled = enabled
        self.start_time = time.perf_counter()

    def step(self, current: int, total: int, message: str) -> None:
        if not self.enabled:
            return

        elapsed = time.perf_counter() - self.start_time
        print(f"[{current}/{total}] {message} ({elapsed:.1f}s)", file=sys.stderr, flush=True)


def velocity_profile_from_curvature(
    points: np.ndarray,
    kappa: np.ndarray,
    max_speed: float,
    min_speed: float,
    lateral_accel_limit: float,
    accel_limit: float,
    decel_limit: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    s, total = cumulative_s(points, closed=True)

    vx_curve = np.sqrt(np.maximum(lateral_accel_limit / np.maximum(np.abs(kappa), 1e-6), 0.0))
    vx = np.clip(vx_curve, min_speed, max_speed)

    ds = np.diff(np.concatenate([s, [total]]))

    for i in range(len(vx) - 2, -1, -1):
        vx[i] = min(vx[i], np.sqrt(max(vx[i + 1] ** 2 + 2.0 * decel_limit * ds[i], 0.0)))

    for i in range(1, len(vx)):
        vx[i] = min(vx[i], np.sqrt(max(vx[i - 1] ** 2 + 2.0 * accel_limit * ds[i - 1], 0.0)))

    ax = np.zeros(len(vx), dtype=np.float64)
    valid_ds = ds > 1e-9
    ax[valid_ds] = (np.roll(vx, -1)[valid_ds] ** 2 - vx[valid_ds] ** 2) / (2.0 * ds[valid_ds])
    ax = np.clip(ax, -decel_limit, accel_limit)

    return s, vx, ax


def patch_tph_spline_approximation_compat(tph: object) -> None:
    """Patch older trajectory_planning_helpers for newer NumPy/SciPy behavior."""

    def dist_to_p_compat(t_glob: np.ndarray, path: list, p: np.ndarray) -> float:
        t_scalar = float(np.asarray(t_glob).reshape(-1)[0])
        s = np.asarray(interpolate.splev(t_scalar, path), dtype=np.float64).reshape(-1)
        p_vec = np.asarray(p, dtype=np.float64).reshape(-1)
        return float(spatial.distance.euclidean(p_vec, s))

    module = tph.spline_approximation
    module.dist_to_p = dist_to_p_compat

    original_spline_approximation = module.spline_approximation
    if getattr(original_spline_approximation, "_numpy_scalar_compat", False):
        return

    original_fmin = module.optimize.fmin

    def spline_approximation_compat(*args: object, **kwargs: object) -> np.ndarray:
        def fmin_compat(*fmin_args: object, **fmin_kwargs: object) -> object:
            result = original_fmin(*fmin_args, **fmin_kwargs)
            values = np.asarray(result)
            if values.size == 1:
                return float(values.reshape(-1)[0])
            return result

        previous_fmin = module.optimize.fmin
        module.optimize.fmin = fmin_compat
        try:
            return original_spline_approximation(*args, **kwargs)
        finally:
            module.optimize.fmin = previous_fmin

    spline_approximation_compat._numpy_scalar_compat = True  # type: ignore[attr-defined]
    module.spline_approximation = spline_approximation_compat


def build_reftrack_interp_with_retry(
    tph: object,
    reftrack: np.ndarray,
    stepsize_prep: float,
    stepsize_reg: float,
    spline_smoothing: float,
    debug: bool,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    smoothing = max(float(spline_smoothing), 1e-6)
    attempted = []

    for _ in range(6):
        attempted.append(smoothing)

        if debug:
            print(
                f"[global-opt] spline approximation with smoothing={smoothing:g}",
                file=sys.stderr,
                flush=True,
            )

        reftrack_interp = tph.spline_approximation.spline_approximation(
            track=reftrack,
            k_reg=3,
            s_reg=smoothing,
            stepsize_prep=stepsize_prep,
            stepsize_reg=stepsize_reg,
            debug=debug,
        )

        refpath_interp_cl = np.vstack((reftrack_interp[:, :2], reftrack_interp[0, :2]))
        coeffs_x, coeffs_y, a_interp, normvec = tph.calc_splines.calc_splines(path=refpath_interp_cl)

        if not tph.check_normals_crossing.check_normals_crossing(
            track=reftrack_interp,
            normvec_normalized=normvec,
            horizon=10,
        ):
            return reftrack_interp, a_interp, normvec, coeffs_x, coeffs_y, smoothing

        smoothing *= 2.0

    attempted_str = ", ".join(f"{value:g}" for value in attempted)

    raise RuntimeError(
        "Spline normals cross even after retrying higher --global-opt-spline-smoothing values: "
        f"{attempted_str}. Try a smoother centerline or a larger initial smoothing value."
    )


def call_iqp_handler_compat(
    tph: object,
    reftrack_interp: np.ndarray,
    normvec: np.ndarray,
    a_interp: np.ndarray,
    coeffs_x: np.ndarray,
    coeffs_y: np.ndarray,
    curvature_limit: float,
    width_opt: float,
    debug: bool,
    stepsize_reg: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    iqp_handler_fn = tph.iqp_handler.iqp_handler

    kwargs = dict(
        reftrack=reftrack_interp,
        normvectors=normvec,
        A=a_interp,
        kappa_bound=curvature_limit,
        w_veh=width_opt,
        print_debug=debug,
        plot_debug=False,
        stepsize_interp=stepsize_reg,
        iters_min=3,
        curv_error_allowed=0.01,
    )

    param_names = set(inspect.signature(iqp_handler_fn).parameters.keys())

    if {"spline_len", "psi", "kappa", "dkappa"} & param_names:
        spline_len = np.asarray(
            tph.calc_spline_lengths.calc_spline_lengths(coeffs_x=coeffs_x, coeffs_y=coeffs_y),
            dtype=np.float64,
        ).reshape(-1)

        ind_spls = np.arange(len(spline_len), dtype=np.int32)
        t_spls = np.zeros(len(spline_len), dtype=np.float64)

        psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(
            coeffs_x=coeffs_x,
            coeffs_y=coeffs_y,
            ind_spls=ind_spls,
            t_spls=t_spls,
        )

        psi = np.asarray(psi, dtype=np.float64).reshape(-1)
        kappa = np.asarray(kappa, dtype=np.float64).reshape(-1)

        ds_prev = np.roll(spline_len, 1)
        ds_next = spline_len
        ds_sum = ds_prev + ds_next

        dkappa = np.zeros_like(kappa)
        valid = ds_sum > 1e-9
        dkappa[valid] = (np.roll(kappa, -1)[valid] - np.roll(kappa, 1)[valid]) / ds_sum[valid]

        extra_kwargs = {
            "spline_len": spline_len,
            "psi": psi,
            "kappa": kappa,
            "dkappa": dkappa,
        }

        for name, value in extra_kwargs.items():
            if name in param_names:
                kwargs[name] = value

    return iqp_handler_fn(**kwargs)


def generate_global_opt_raceline(
    centerline: np.ndarray,
    widths: np.ndarray,
    optimizer_root: str,
    opt_type: str,
    vehicle_width: float,
    safety_margin: float,
    curvature_limit: float,
    stepsize_prep: float,
    stepsize_reg: float,
    stepsize_after_opt: float,
    spline_smoothing: float,
    max_speed: float,
    min_speed: float,
    lateral_accel_limit: float,
    accel_limit: float,
    decel_limit: float,
    debug: bool,
    progress: ProgressReporter | None = None,
) -> np.ndarray:
    if opt_type not in {"shortest_path", "mincurv", "mincurv_iqp"}:
        raise RuntimeError("global-opt backend supports shortest_path, mincurv, and mincurv_iqp.")

    width_opt = validate_track_clearance(
        widths,
        vehicle_width,
        safety_margin,
        source="Centerline",
    )

    optimizer_root_abs = os.path.abspath(optimizer_root)

    if not os.path.isdir(optimizer_root_abs):
        raise RuntimeError(f"optimizer root not found: {optimizer_root_abs}")

    if optimizer_root_abs not in sys.path:
        sys.path.insert(0, optimizer_root_abs)

    try:
        import trajectory_planning_helpers as tph
    except ImportError as exc:
        message = (
            "global-opt backend requires trajectory-planning-helpers and quadprog. "
            "Install the Python 3.10 compatible optimizer dependencies first."
        )

        if "quadprog" in str(exc) and "undefined symbol" in str(exc):
            message += (
                " Detected a broken quadprog native extension at import time "
                "(undefined symbol). Reinstall quadprog for the current Python/OS image, "
                "or try the known workaround of using quadprog 0.1.6 with "
                "trajectory-planning-helpers installed via --no-deps."
            )

        raise RuntimeError(message) from exc

    patch_tph_spline_approximation_compat(tph)

    reftrack = np.column_stack([centerline, widths]).astype(np.float64)
    reftrack, = remove_duplicate_points(reftrack)

    if progress is not None:
        progress.step(1, 5, "Smoothing reference track")

    reftrack_interp, a_interp, normvec, coeffs_x, coeffs_y, smoothing_used = build_reftrack_interp_with_retry(
        tph=tph,
        reftrack=reftrack,
        stepsize_prep=stepsize_prep,
        stepsize_reg=stepsize_reg,
        spline_smoothing=spline_smoothing,
        debug=debug,
    )

    validate_track_clearance(
        reftrack_interp[:, 2:4],
        vehicle_width,
        safety_margin,
        source="Interpolated centerline",
    )

    if debug and smoothing_used != spline_smoothing:
        print(
            "Adjusted global-opt spline smoothing from "
            f"{spline_smoothing:g} to {smoothing_used:g} to avoid crossing normals.",
            file=sys.stderr,
            flush=True,
        )

    if opt_type == "shortest_path":
        if progress is not None:
            progress.step(2, 5, "Running shortest-path optimizer")

        alpha = tph.opt_shortest_path.opt_shortest_path(
            reftrack=reftrack_interp,
            normvectors=normvec,
            w_veh=width_opt,
            print_debug=debug,
        )

    elif opt_type == "mincurv":
        if progress is not None:
            progress.step(2, 5, "Running minimum-curvature optimizer")

        alpha = tph.opt_min_curv.opt_min_curv(
            reftrack=reftrack_interp,
            normvectors=normvec,
            A=a_interp,
            kappa_bound=curvature_limit,
            w_veh=width_opt,
            print_debug=debug,
            plot_debug=False,
        )[0]

    else:
        try:
            if progress is not None:
                progress.step(2, 5, "Running iterative minimum-curvature optimizer")

            alpha, reftrack_interp, normvec = call_iqp_handler_compat(
                tph=tph,
                reftrack_interp=reftrack_interp,
                normvec=normvec,
                a_interp=a_interp,
                coeffs_x=coeffs_x,
                coeffs_y=coeffs_y,
                curvature_limit=curvature_limit,
                width_opt=width_opt,
                debug=debug,
                stepsize_reg=stepsize_reg,
            )

        except TypeError:
            if progress is not None:
                progress.step(2, 5, "Falling back to minimum-curvature optimizer")

            alpha = tph.opt_min_curv.opt_min_curv(
                reftrack=reftrack_interp,
                normvectors=normvec,
                A=a_interp,
                kappa_bound=curvature_limit,
                w_veh=width_opt,
                print_debug=debug,
                plot_debug=False,
            )[0]

    if progress is not None:
        progress.step(3, 5, "Interpolating optimized raceline")

    (
        raceline_xy,
        _,
        coeffs_x_opt,
        coeffs_y_opt,
        spline_inds,
        t_vals,
        s_points,
        _,
        _,
    ) = tph.create_raceline.create_raceline(
        refline=reftrack_interp[:, :2],
        normvectors=normvec,
        alpha=alpha,
        stepsize_interp=stepsize_after_opt,
    )

    if progress is not None:
        progress.step(4, 5, "Computing curvature and velocity profile")

    psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=coeffs_x_opt,
        coeffs_y=coeffs_y_opt,
        ind_spls=spline_inds,
        t_spls=t_vals,
    )

    psi = np.mod(psi, 2.0 * np.pi)

    s = np.asarray(s_points, dtype=np.float64)
    kappa = np.asarray(kappa, dtype=np.float64)

    if len(s) != len(raceline_xy):
        s, _, _ = velocity_profile_from_curvature(
            raceline_xy,
            kappa,
            max_speed=max_speed,
            min_speed=min_speed,
            lateral_accel_limit=lateral_accel_limit,
            accel_limit=accel_limit,
            decel_limit=decel_limit,
        )

    _, vx, ax = velocity_profile_from_curvature(
        raceline_xy,
        kappa,
        max_speed=max_speed,
        min_speed=min_speed,
        lateral_accel_limit=lateral_accel_limit,
        accel_limit=accel_limit,
        decel_limit=decel_limit,
    )

    if progress is not None:
        progress.step(5, 5, "Assembling output")

    return np.column_stack([s, raceline_xy[:, 0], raceline_xy[:, 1], psi, kappa, vx, ax])


def generate_with_selected_backend(
    args: argparse.Namespace,
    centerline: np.ndarray,
    widths: np.ndarray,
) -> Tuple[np.ndarray, str]:
    progress = ProgressReporter(enabled=args.show_progress)

    raceline = generate_global_opt_raceline(
        centerline=centerline,
        widths=widths,
        optimizer_root=args.optimizer_root or default_optimizer_root(),
        opt_type=args.opt_type,
        vehicle_width=args.vehicle_width,
        safety_margin=args.safety_margin,
        curvature_limit=args.curvature_limit,
        stepsize_prep=args.global_opt_stepsize_prep,
        stepsize_reg=args.global_opt_stepsize_reg,
        stepsize_after_opt=args.global_opt_stepsize_after_opt,
        spline_smoothing=args.global_opt_spline_smoothing,
        max_speed=args.max_speed,
        min_speed=args.min_speed,
        lateral_accel_limit=args.lateral_accel_limit,
        accel_limit=args.accel_limit,
        decel_limit=args.decel_limit,
        debug=args.global_opt_debug,
        progress=progress,
    )

    return raceline, "global-opt"


def run(args: argparse.Namespace) -> None:
    centerline_forward, widths_forward = load_centerline(args.centerline)

    out_path = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    header = "s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2"

    direction_plan = [("forward", centerline_forward, widths_forward, out_path)]

    if args.direction == "reverse":
        centerline_reverse, widths_reverse = reverse_centerline(centerline_forward, widths_forward)
        direction_plan = [("reverse", centerline_reverse, widths_reverse, out_path)]

    elif args.direction == "both":
        centerline_reverse, widths_reverse = reverse_centerline(centerline_forward, widths_forward)
        direction_plan.append(
            ("reverse", centerline_reverse, widths_reverse, output_path_for_direction(out_path, "reverse"))
        )

    print(f"Input centerline: {args.centerline}")
    print(
        "Vehicle envelope: "
        f"{effective_vehicle_envelope_width(args.vehicle_width, args.safety_margin):.3f} m "
        f"(vehicle {args.vehicle_width:.3f} m + {args.safety_margin:.3f} m margin per side)"
    )

    for direction, centerline, widths, direction_out_path in direction_plan:
        raceline, backend_used = generate_with_selected_backend(args, centerline, widths)

        np.savetxt(
            direction_out_path,
            raceline,
            fmt="%.7f",
            delimiter=";",
            header=header,
        )

        print(f"Direction: {direction}")
        print(f"Backend: {backend_used}")
        print(f"Output points: {len(raceline)}")

        closed_length = raceline[-1, 0] + float(np.linalg.norm(raceline[0, 1:3] - raceline[-1, 1:3]))
        metadata_path = write_raceline_metadata(
            output_path=direction_out_path,
            centerline_path=args.centerline,
            direction=direction,
            backend=backend_used,
            preset=args.preset,
            opt_type=args.opt_type,
            vehicle_width=args.vehicle_width,
            safety_margin=args.safety_margin,
            widths=widths,
            point_count=len(raceline),
            track_length=closed_length,
        )
        print(f"Track length: {closed_length:.3f} m")
        print(f"Wrote raceline CSV: {direction_out_path}")
        print(f"Wrote raceline metadata: {metadata_path}")


def build_arg_parser(preset: str = "race-stacks") -> argparse.ArgumentParser:
    defaults = preset_defaults(preset)

    p = argparse.ArgumentParser(
        description="Generate a raceline from a centerline CSV using global_racetrajectory_optimization."
    )

    p.add_argument("--centerline", required=True, help="Path to centerline CSV: x,y,w_tr_right,w_tr_left")
    p.add_argument("--output", required=True, help="Path to output raceline CSV")

    p.add_argument(
        "--preset",
        choices=VALID_PRESETS,
        default=preset,
        help="Parameter preset. Defaults to 'race-stacks'.",
    )

    p.add_argument(
        "--show-progress",
        action="store_true",
        help="Print coarse-grained progress updates to stderr.",
    )

    p.add_argument(
        "--optimizer-root",
        default=None,
        help="Path to global_racetrajectory_optimization checkout.",
    )

    p.add_argument(
        "--opt-type",
        choices=["shortest_path", "mincurv", "mincurv_iqp"],
        default=defaults["opt_type"],
    )

    p.add_argument(
        "--vehicle-width-m",
        "--vehicle-width",
        dest="vehicle_width",
        type=nonnegative_finite_float,
        default=DEFAULT_VEHICLE_WIDTH_M,
        help=(
            "Physical vehicle width in metres. The legacy --vehicle-width name remains supported "
            f"(default: {DEFAULT_VEHICLE_WIDTH_M:g})."
        ),
    )
    p.add_argument(
        "--safety-margin-m",
        "--safety-margin",
        dest="safety_margin",
        type=nonnegative_finite_float,
        default=DEFAULT_SAFETY_MARGIN_M,
        help=(
            "Extra clearance in metres between the vehicle body and each track boundary. "
            "The optimizer reserves vehicle_width + 2 * safety_margin; the legacy "
            f"--safety-margin name remains supported (default: {DEFAULT_SAFETY_MARGIN_M:g})."
        ),
    )
    p.add_argument("--curvature-limit", type=float, default=defaults["curvature_limit"])

    p.add_argument("--global-opt-stepsize-prep", type=float, default=defaults["global_opt_stepsize_prep"])
    p.add_argument("--global-opt-stepsize-reg", type=float, default=defaults["global_opt_stepsize_reg"])

    p.add_argument(
        "--global-opt-stepsize-after-opt",
        type=float,
        default=defaults["global_opt_stepsize_after_opt"],
    )

    p.add_argument(
        "--global-opt-spline-smoothing",
        type=float,
        default=defaults["global_opt_spline_smoothing"],
    )

    p.add_argument("--global-opt-debug", action="store_true")

    p.add_argument(
        "--direction",
        choices=VALID_DIRECTIONS,
        default="forward",
        help="Generate raceline for forward, reverse, or both directions.",
    )

    p.add_argument("--max-speed", type=float, default=3.0)
    p.add_argument("--min-speed", type=float, default=0.8)
    p.add_argument("--lateral-accel-limit", type=float, default=2.5)
    p.add_argument("--accel-limit", type=float, default=1.5)
    p.add_argument("--decel-limit", type=float, default=2.5)

    return p


def main() -> None:
    preset_parser = argparse.ArgumentParser(add_help=False)
    preset_parser.add_argument("--preset", choices=VALID_PRESETS, default="race-stacks")
    preset_args, _ = preset_parser.parse_known_args()

    parser = build_arg_parser(preset=preset_args.preset)
    args = parser.parse_args()

    run(args)


if __name__ == "__main__":
    main()
