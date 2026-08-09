from __future__ import annotations

import math
import unittest

from jetpilot_console.e2e_analysis import control_error_summary, finite_summary
from jetpilot_console.e2e_analysis_worker import (
    _aggressive_events,
    _aggressiveness_score,
    _attach_pose_context,
    _enrich_control_dynamics,
    _enrich_trajectory_dynamics,
    _relative_future_trajectory,
    _teacher_free_metrics,
    trajectory_error_summary,
)


class E2EAnalysisMetricTests(unittest.TestCase):
    def test_finite_summary_ignores_invalid_values_and_interpolates_percentiles(self) -> None:
        summary = finite_summary([1, 2, 3, float("nan"), None, "bad"])

        self.assertEqual(summary["count"], 3)
        self.assertEqual(summary["mean"], 2.0)
        self.assertEqual(summary["p50"], 2.0)
        self.assertTrue(math.isclose(float(summary["p95"]), 2.9))

    def test_control_error_summary_reports_mae_rmse_and_signed_distribution(self) -> None:
        summary = control_error_summary(
            [
                {"steering_error": -0.2},
                {"steering_error": 0.1},
                {"steering_error": None},
            ],
            "steering",
        )

        self.assertEqual(summary["count"], 2)
        self.assertEqual(summary["mae"], 0.15)
        self.assertAlmostEqual(summary["rmse"], math.sqrt(0.025), places=7)
        self.assertEqual(summary["max_abs"], 0.2)

    def test_teacher_free_metrics_detect_aggressive_control_and_vehicle_dynamics(self) -> None:
        trajectory = [
            {"t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "speed_mps": 0.0},
            {"t": 0.1, "x": 0.1, "y": 0.0, "yaw": 0.2, "speed_mps": 1.0},
            {"t": 0.2, "x": 0.3, "y": 0.1, "yaw": 0.6, "speed_mps": 2.0},
        ]
        records = [
            {"t": 0.0, "steering_pred": 0.0, "throttle_pred": 0.1},
            {"t": 0.1, "steering_pred": 0.9, "throttle_pred": 1.0},
            {"t": 0.2, "steering_pred": -0.9, "throttle_pred": 0.0},
        ]

        _enrich_trajectory_dynamics(trajectory)
        _attach_pose_context(records, trajectory)
        oscillations = _enrich_control_dynamics(records)
        for record in records:
            record["aggressiveness_score"] = _aggressiveness_score(record)
        metrics = _teacher_free_metrics(records, trajectory, oscillations)
        events = _aggressive_events(records)

        self.assertGreater(metrics["score"], 60.0)
        self.assertIn(metrics["classification"], {"aggressive", "extreme"})
        self.assertGreater(metrics["steering_rate_abs_per_s"]["p95"], 1.5)
        self.assertGreater(metrics["lateral_accel_abs_mps2"]["p95"], 3.0)
        self.assertGreaterEqual(oscillations, 1)
        self.assertTrue(events)
        self.assertIn("steering rate", events[0]["reasons"])

    def test_relative_future_trajectory_is_expressed_in_vehicle_frame(self) -> None:
        trajectory = [
            {"t": 0.0, "x": 10.0, "y": 5.0, "yaw": math.pi / 2.0},
            {"t": 0.5, "x": 10.0, "y": 6.0, "yaw": math.pi / 2.0},
            {"t": 1.0, "x": 9.0, "y": 7.0, "yaw": math.pi / 2.0},
        ]

        local = _relative_future_trajectory(
            trajectory,
            [0.0, 0.5, 1.0],
            stamp=0.0,
            points=2,
            horizon_sec=1.0,
            max_dt_sec=0.01,
        )

        self.assertIsNotNone(local)
        self.assertAlmostEqual(local[0][0], 1.0)
        self.assertAlmostEqual(local[0][1], 0.0)
        self.assertAlmostEqual(local[1][0], 2.0)
        self.assertAlmostEqual(local[1][1], 1.0)

    def test_trajectory_error_summary_reports_ade_and_fde(self) -> None:
        summary = trajectory_error_summary(
            [
                {
                    "trajectory_point_errors_m": [0.1, 0.3],
                    "trajectory_fde_m": 0.3,
                },
                {
                    "trajectory_point_errors_m": [0.2, 0.4],
                    "trajectory_fde_m": 0.4,
                },
            ]
        )

        self.assertEqual(summary["sample_count"], 2)
        self.assertEqual(summary["point_count"], 4)
        self.assertEqual(summary["ade_m"], 0.25)
        self.assertEqual(summary["fde_m"], 0.35)


if __name__ == "__main__":
    unittest.main()
