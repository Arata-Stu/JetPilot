from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.map_detail import activate_hd_map_version, build_map_detail, save_hd_map_version


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


class HdMapVersionTest(unittest.TestCase):
    def test_saves_and_activates_hd_map_versions_without_rebuilding_vslam(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_root = Path(temporary_directory)
            map_dir = map_root / "course_a"
            map_dir.mkdir()
            config = SimpleNamespace(map_root=map_root)
            hd_map_path = map_dir / "course_a_hd_map.yaml"
            centerline_path = map_dir / "course_a_hd_map_centerline.csv"
            raceline_path = map_dir / "course_a_raceline.csv"

            hd_map_v1 = """format: tamiya_local_hd_map_v1
primary_lane_id: "lane_001"
lanes:
  - id: "lane_001"
    closed_loop: true
    left_bound:
      - [0, 1, 0]
      - [1, 1, 0]
      - [1, 0, 0]
    right_bound:
      - [0, -1, 0]
      - [1, -1, 0]
      - [1, 0, 0]
    centerline:
      - [0, 0, 0]
      - [1, 0, 0]
      - [1, 0.5, 0]
"""
            hd_map_v2 = hd_map_v1.replace("[1, 0.5, 0]", "[2, 0.5, 0]")
            hd_map_path.write_text(hd_map_v1, encoding="utf-8")
            centerline_path.write_text("# x_m,y_m,w_tr_right_m,w_tr_left_m\n0,0,1,1\n", encoding="utf-8")
            raceline_path.write_text("# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n0;0;0;0;0;1;0\n", encoding="utf-8")

            detail = save_hd_map_version(config, {"map_dir": str(map_dir), "label": "base"})
            self.assertEqual(detail["hd_map_versions"]["active_id"], "ver_001")
            self.assertFalse(detail["hd_map_versions"]["working_copy_dirty"])

            hd_map_path.write_text(hd_map_v2, encoding="utf-8")
            centerline_path.write_text("# x_m,y_m,w_tr_right_m,w_tr_left_m\n2,0,1,1\n", encoding="utf-8")
            raceline_path.write_text("# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2\n0;2;0;0;0;1;0\n", encoding="utf-8")
            detail = save_hd_map_version(config, {"map_dir": str(map_dir), "label": "variant"})
            self.assertEqual(detail["hd_map_versions"]["active_id"], "ver_002")

            detail = activate_hd_map_version(config, {"map_dir": str(map_dir), "version_id": "ver_001"})

            self.assertEqual(detail["hd_map_versions"]["active_id"], "ver_001")
            self.assertIn("[1, 0.5, 0]", hd_map_path.read_text(encoding="utf-8"))
            self.assertIn("0,0,1,1", centerline_path.read_text(encoding="utf-8"))
            self.assertIn("0;0;0;0;0;1;0", raceline_path.read_text(encoding="utf-8"))
            self.assertFalse(detail["hd_map_versions"]["working_copy_dirty"])


if __name__ == "__main__":
    unittest.main()
