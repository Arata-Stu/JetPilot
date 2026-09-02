import unittest

from jetpilot_bag_tools.throttle_calibration import (
    AnalysisConfig,
    CalibrationSample,
    analyze_samples,
)


def fixed_run(start, throttle, speed, duration=6.0, slope=0.0, steering=0.0):
    return [
        CalibrationSample(
            time_s=start + index * 0.05,
            throttle=throttle,
            speed_mps=speed + slope * index * 0.05,
            steering=steering,
        )
        for index in range(int(duration / 0.05) + 1)
    ]


class ThrottleCalibrationTest(unittest.TestCase):
    def test_extracts_and_orders_fixed_throttle_runs(self):
        samples = fixed_run(0.0, 0.2, 0.3)
        samples.append(CalibrationSample(6.1, 0.0, 0.0))
        samples.extend(fixed_run(7.0, 0.25, 0.6))
        result = analyze_samples(samples)
        self.assertEqual(len(result.points), 2)
        self.assertAlmostEqual(result.points[0].throttle, 0.2)
        self.assertAlmostEqual(result.points[0].steady_speed_mps, 0.3)
        self.assertAlmostEqual(result.points[1].steady_speed_mps, 0.6)

    def test_rejects_turning_and_unsettled_runs(self):
        samples = fixed_run(0.0, 0.2, 0.3, steering=0.4)
        samples.extend(fixed_run(7.0, 0.25, 0.2, slope=0.2))
        result = analyze_samples(
            samples, AnalysisConfig(maximum_speed_slope_mps2=0.1)
        )
        self.assertEqual(result.points, [])
        self.assertTrue(any(segment.reason == "speed_not_settled" for segment in result.segments))


if __name__ == "__main__":
    unittest.main()
