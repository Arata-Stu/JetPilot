from __future__ import annotations

import math
import unittest
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.map_pipeline import generate_raceline_script


class GenerateRacelineScriptTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SimpleNamespace(
            python_bin="/opt/env/bin/python",
            python_ws=Path("/workspaces/python_ws"),
        )

    def test_passes_default_vehicle_clearance_explicitly(self) -> None:
        script = generate_raceline_script(self.config, "/workspaces/map/course_a")

        self.assertIn("--vehicle-width-m 0.25", script)
        self.assertIn("--safety-margin-m 0.05", script)

    def test_passes_custom_vehicle_clearance(self) -> None:
        script = generate_raceline_script(
            self.config,
            "/workspaces/map/course_a",
            vehicle_width_m=0.187,
            safety_margin_m=0.02,
        )

        self.assertIn("--vehicle-width-m 0.187", script)
        self.assertIn("--safety-margin-m 0.02", script)

    def test_rejects_invalid_vehicle_clearance(self) -> None:
        for value in (-0.01, math.inf, math.nan, True):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "finite value"):
                    generate_raceline_script(
                        self.config,
                        "/workspaces/map/course_a",
                        vehicle_width_m=value,
                    )


if __name__ == "__main__":
    unittest.main()
