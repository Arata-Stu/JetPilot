from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.map_detail import build_map_detail


class MapDetailOdometryOverlayTest(unittest.TestCase):
    def test_reads_odometry_samples_from_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_root = Path(temporary_directory)
            map_dir = map_root / "course_a"
            map_dir.mkdir()
            (map_dir / "vslam_reference_snapshot.json").write_text(
                json.dumps(
                    {
                        "localization": {
                            "confirmed": True,
                            "map_frame": "map",
                            "history_stride": 2,
                        },
                        "odometry_samples": [
                            {
                                "frame_id": "map",
                                "pose": {"position": {"x": 1.0, "y": 2.0, "z": 0.0}},
                            },
                            {
                                "frame_id": "map",
                                "pose": {"position": {"x": 2.5, "y": 3.5, "z": 0.0}},
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )

            detail = build_map_detail(SimpleNamespace(map_root=map_root), str(map_dir))

        self.assertEqual(detail["odometry"]["count"], 2)
        self.assertEqual(detail["odometry"]["points"], [[1.0, 2.0], [2.5, 3.5]])
        self.assertEqual(detail["odometry"]["frame_id"], "map")
        self.assertTrue(detail["odometry"]["localized"])
        self.assertEqual(detail["odometry"]["history_stride"], 2)
        self.assertEqual(detail["stats"]["odometry_points"], 2)


if __name__ == "__main__":
    unittest.main()
