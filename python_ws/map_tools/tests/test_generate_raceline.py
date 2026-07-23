from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


MAP_TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MAP_TOOLS_DIR))

try:
    import numpy as np

    import generate_raceline as raceline
except ModuleNotFoundError as exc:  # The host may not have Docker's optimizer dependencies.
    np = None
    raceline = None
    IMPORT_ERROR = str(exc)
else:
    IMPORT_ERROR = ""


@unittest.skipIf(raceline is None, f"raceline dependencies unavailable: {IMPORT_ERROR}")
class RacelineClearanceTest(unittest.TestCase):
    def test_metadata_path_keeps_direction_suffix(self) -> None:
        self.assertEqual(
            raceline.metadata_path_for_output("/maps/course_raceline_reverse.csv"),
            "/maps/course_raceline_reverse.meta.json",
        )

    def test_metadata_records_vehicle_clearance_without_changing_csv(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / "course_raceline.csv")
            metadata_path = raceline.write_raceline_metadata(
                output_path=output,
                centerline_path=str(Path(directory) / "course_centerline.csv"),
                direction="forward",
                backend="global-opt",
                preset="race-stacks",
                opt_type="mincurv",
                vehicle_width=0.25,
                safety_margin=0.05,
                widths=np.asarray([[0.2, 0.2], [0.18, 0.22]], dtype=np.float64),
                point_count=42,
                track_length=12.5,
            )

            with open(metadata_path, "r", encoding="utf-8") as handle:
                metadata = json.load(handle)

            self.assertEqual(metadata["format"], "jetpilot_raceline_metadata_v1")
            self.assertAlmostEqual(
                metadata["vehicle_clearance"]["effective_envelope_width_m"],
                0.35,
            )
            self.assertAlmostEqual(
                metadata["vehicle_clearance"]["min_available_track_width_m"],
                0.4,
            )
            self.assertAlmostEqual(metadata["speed_profile"]["max_speed_mps"], 3.0)
            self.assertAlmostEqual(metadata["speed_profile"]["min_speed_mps"], 0.8)

    def test_effective_width_adds_margin_on_both_sides(self) -> None:
        self.assertAlmostEqual(raceline.effective_vehicle_envelope_width(0.25, 0.05), 0.35)

    def test_speed_profile_rejects_invalid_limits(self) -> None:
        cases = (
            {"max_speed": 0.0},
            {"min_speed": -0.1},
            {"min_speed": 4.0, "max_speed": 3.0},
            {"lateral_accel_limit": 0.0},
            {"accel_limit": 0.0},
            {"decel_limit": 0.0},
        )
        defaults = {
            "max_speed": 3.0,
            "min_speed": 0.8,
            "lateral_accel_limit": 2.5,
            "accel_limit": 1.5,
            "decel_limit": 2.5,
        }
        for override in cases:
            with self.subTest(override=override):
                with self.assertRaises(RuntimeError):
                    raceline.validate_speed_profile(**{**defaults, **override})

    def test_validation_does_not_modify_track_widths(self) -> None:
        widths = np.asarray([[0.20, 0.20], [0.18, 0.22]], dtype=np.float64)
        original = widths.copy()

        required = raceline.validate_track_clearance(
            widths,
            0.25,
            0.05,
            source="test centerline",
        )

        self.assertAlmostEqual(required, 0.35)
        np.testing.assert_array_equal(widths, original)

    def test_narrow_track_fails_instead_of_expanding_bounds(self) -> None:
        widths = np.asarray([[0.20, 0.20], [0.10, 0.20]], dtype=np.float64)
        original = widths.copy()

        with self.assertRaisesRegex(RuntimeError, r"point 1.*available=0.3 m.*required=0.35 m"):
            raceline.validate_track_clearance(
                widths,
                0.25,
                0.05,
                source="test centerline",
            )

        np.testing.assert_array_equal(widths, original)

    def test_new_and_legacy_cli_names_share_the_same_destination(self) -> None:
        parser = raceline.build_arg_parser()
        common = ["--centerline", "input.csv", "--output", "output.csv"]

        current = parser.parse_args(
            [*common, "--vehicle-width-m", "0.19", "--safety-margin-m", "0.02"]
        )
        legacy = parser.parse_args(
            [*common, "--vehicle-width", "0.19", "--safety-margin", "0.02"]
        )

        self.assertEqual(current.vehicle_width, legacy.vehicle_width)
        self.assertEqual(current.safety_margin, legacy.safety_margin)


if __name__ == "__main__":
    unittest.main()
