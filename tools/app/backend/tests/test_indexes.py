from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from jetpilot_console.indexes import scan_maps


class MapIndexTest(unittest.TestCase):
    def _make_localized_map(self, root: Path, name: str) -> Path:
        map_dir = root / name
        map_dir.mkdir()
        (map_dir / "cuvgl_map").mkdir()
        (map_dir / "cuvslam_map").mkdir()
        (map_dir / f"{name}_hd_map.yaml").write_text("format: test\n", encoding="utf-8")
        return map_dir

    def test_active_custom_line_is_a_runtime_driving_line(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_root = Path(temporary_directory)
            map_dir = self._make_localized_map(map_root, "course_a")
            (map_dir / "course_a_custom_line.csv").write_text(
                "# s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2\n"
                "0;0;0;0;0;1;0\n1;1;0;0;0;1;0\n",
                encoding="utf-8",
            )
            (map_dir / "course_a_custom_line.meta.json").write_text(
                '{"format":"jetpilot_custom_line_v1","id":"safe"}\n',
                encoding="utf-8",
            )

            maps = scan_maps(map_root)

        self.assertEqual(len(maps), 1)
        self.assertTrue(maps[0]["artifacts"]["custom_line_csv"]["exists"])
        self.assertTrue(maps[0]["complete_runtime_bundle"])

    def test_custom_line_without_metadata_is_incomplete(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_root = Path(temporary_directory)
            map_dir = self._make_localized_map(map_root, "course_a")
            (map_dir / "course_a_custom_line.csv").write_text(
                "# incomplete custom bundle\n",
                encoding="utf-8",
            )

            maps = scan_maps(map_root)

        self.assertEqual(len(maps), 1)
        self.assertFalse(maps[0]["complete_runtime_bundle"])

    def test_map_without_raceline_or_active_custom_line_is_incomplete(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_root = Path(temporary_directory)
            self._make_localized_map(map_root, "course_a")

            maps = scan_maps(map_root)

        self.assertEqual(len(maps), 1)
        self.assertFalse(maps[0]["complete_runtime_bundle"])

    def test_junction_route_readiness_is_included_without_building_map_detail(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_root = Path(temporary_directory)
            map_dir = self._make_localized_map(map_root, "course_a")
            (map_dir / "course_a_raceline.csv").write_text("# route\n", encoding="utf-8")
            (map_dir / "course_a_hd_map.yaml").write_text(
                """format: tamiya_local_hd_map_v1
junctions:
  - id: "junction_001"
    signal_id: "signal_001"
    branches:
      left: "route_left"
      straight: "primary"
      right: "route_right"
""",
                encoding="utf-8",
            )

            unconfigured = scan_maps(map_root)[0]
            self.assertEqual(unconfigured["competition_route_status"], "unconfigured")
            self.assertFalse(unconfigured["competition_routes_ready"])
            self.assertFalse(unconfigured["complete_runtime_bundle"])

            config_path = map_dir / "competition_route.param.yaml"
            config_path.write_text("invalid: true\n", encoding="utf-8")
            invalid = scan_maps(map_root)[0]
            self.assertEqual(invalid["competition_route_status"], "invalid")
            self.assertFalse(invalid["complete_runtime_bundle"])

            config_path.write_text(
                """/**:
  ros__parameters:
    lane_ids: [primary, route_left]
    lane_path_topics: [/hd_map/primary, ""]
    lane_trajectory_topics: ["", /planning/left]
    lane_target_speeds_mps: [1.0, 0.8]
    default_lane_id: "primary"
    requested_lane_topic: /planning/requested_lane
    current_section_topic: /localization/current_section
    output_trajectory_topic: /planning/route/trajectory
    output_profile_topic: /planning/route/trajectory_profile
    target_speed_topic: /planning/route/target_speed
    selected_lane_topic: /planning/route/selected_lane
    ready_topic: /planning/route/ready
    diagnostics_topic: /planning/route/diagnostics
    require_requested_lane_heartbeat: true
    requested_lane_timeout_sec: 0.5
    current_section_timeout_sec: 1.0
""",
                encoding="utf-8",
            )
            warning = scan_maps(map_root)[0]
            self.assertEqual(warning["competition_route_status"], "warning")
            self.assertFalse(warning["complete_runtime_bundle"])

            config_path.write_text(
                """/**:
  ros__parameters:
    lane_ids: [primary, route_left, route_right]
    lane_path_topics: [/hd_map/primary, "", ""]
    lane_trajectory_topics: ["", /planning/left, /planning/right]
    lane_target_speeds_mps: [1.0, 0.8, 0.8]
    default_lane_id: "primary"
    requested_lane_topic: /planning/requested_lane
    current_section_topic: /localization/current_section
    output_trajectory_topic: /planning/route/trajectory
    output_profile_topic: /planning/route/trajectory_profile
    target_speed_topic: /planning/route/target_speed
    selected_lane_topic: /planning/route/selected_lane
    ready_topic: /planning/route/ready
    diagnostics_topic: /planning/route/diagnostics
    require_requested_lane_heartbeat: true
    requested_lane_timeout_sec: 0.5
    current_section_timeout_sec: 1.0
""",
                encoding="utf-8",
            )
            ready = scan_maps(map_root)[0]
            self.assertEqual(ready["competition_route_status"], "ready")
            self.assertTrue(ready["competition_routes_ready"])
            self.assertTrue(ready["complete_runtime_bundle"])


if __name__ == "__main__":
    unittest.main()
