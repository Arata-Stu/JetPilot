from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.map_detail import (
    CUSTOM_LINE_MAX_POINTS,
    activate_custom_line,
    activate_hd_map_version,
    build_map_detail,
    create_custom_line,
    delete_custom_line,
    save_hd_map_version,
    save_section_gates,
    update_custom_line,
)


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

    def test_exposes_saved_raceline_direction(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_root = Path(temporary_directory)
            map_dir = map_root / "course_a"
            map_dir.mkdir()
            (map_dir / "course_a_raceline.meta.json").write_text(
                json.dumps({"direction": "reverse"}),
                encoding="utf-8",
            )

            detail = build_map_detail(SimpleNamespace(map_root=map_root), str(map_dir))

        self.assertEqual(detail["raceline_metadata"]["direction"], "reverse")
        self.assertTrue(detail["map"]["artifacts"]["raceline_meta"]["exists"])


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


class CustomLineTest(unittest.TestCase):
    def _make_map(self, root: Path) -> tuple[SimpleNamespace, Path]:
        map_dir = root / "course_a"
        map_dir.mkdir()
        (map_dir / "course_a_hd_map.yaml").write_text(
            """format: tamiya_local_hd_map_v1
primary_lane_id: "lane_001"
lanes:
  - id: "lane_001"
    closed_loop: false
    left_bound:
      - [0, 1, 0]
      - [1, 1, 0]
      - [2, 1, 0]
    right_bound:
      - [0, -1, 0]
      - [1, -1, 0]
      - [2, -1, 0]
    centerline:
      - [0, 0, 0]
      - [1, 0, 0]
      - [2, 0, 0]
""",
            encoding="utf-8",
        )
        (map_dir / "course_a_hd_map_centerline.csv").write_text(
            "# x_m,y_m,w_tr_right_m,w_tr_left_m\n0,0,1,1\n1,0,1,1\n2,0,1,1\n",
            encoding="utf-8",
        )
        (map_dir / "course_a_raceline.csv").write_text(
            (
                "# s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2\n"
                "0;0;0;0;0;1.0;0\n"
                "1;1;0.2;0;0;1.5;0\n"
                "2;2;0;0;0;1.0;0\n"
            ),
            encoding="utf-8",
        )
        return SimpleNamespace(map_root=root), map_dir

    def _make_section_map(self, root: Path) -> tuple[SimpleNamespace, Path]:
        map_dir = root / "section_course"
        map_dir.mkdir()
        (map_dir / "section_course_hd_map.yaml").write_text(
            """format: tamiya_local_hd_map_v1
primary_lane_id: "lane_001"
source_raster:
  map_yaml: "vslam_landmarks.yaml"
  image: "vslam_landmarks.png"
  resolution_m_per_px: 0.05
  origin_xy_yaw: [0, 0, 0]
  image_size_px: [200, 40]
lanes:
  - id: "lane_001"
    closed_loop: false
    left_bound:
      - [0, 1, 0]
      - [5, 1, 0]
      - [10, 1, 0]
    right_bound:
      - [0, -1, 0]
      - [5, -1, 0]
      - [10, -1, 0]
    centerline:
      - [0, 0, 0]
      - [5, 0, 0]
      - [10, 0, 0]
section_gates:
  - id: "gate_001"
    lane_id: "lane_001"
    s_m: 2
    line:
      - [2, -1, 0]
      - [2, 1, 0]
  - id: "gate_002"
    lane_id: "lane_001"
    s_m: 5
    line:
      - [5, -1, 0]
      - [5, 1, 0]
  - id: "gate_003"
    lane_id: "lane_001"
    s_m: 8
    line:
      - [8, -1, 0]
      - [8, 1, 0]
sections:
  - id: "section_a"
    lane_id: "lane_001"
    start_gate_id: "gate_001"
    end_gate_id: "gate_002"
    start_s_m: 2
    end_s_m: 5
    speed_override_mps: 1.8
  - id: "section_b"
    lane_id: "lane_001"
    start_gate_id: "gate_002"
    end_gate_id: "gate_003"
    start_s_m: 5
    end_s_m: 8
""",
            encoding="utf-8",
        )
        (map_dir / "section_course_hd_map_centerline.csv").write_text(
            "# x_m,y_m,w_tr_right_m,w_tr_left_m\n0,0,1,1\n5,0,1,1\n10,0,1,1\n",
            encoding="utf-8",
        )
        (map_dir / "section_course_raceline.csv").write_text(
            (
                "# s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2\n"
                "0;0;0;0;0;2.8;0\n"
                "5;5;0;0;0;2.8;0\n"
                "10;10;0;0;0;2.8;0\n"
            ),
            encoding="utf-8",
        )
        return SimpleNamespace(map_root=root), map_dir

    def _make_closed_two_gate_map(self, root: Path) -> tuple[SimpleNamespace, Path]:
        map_dir = root / "closed_course"
        map_dir.mkdir()
        (map_dir / "closed_course_hd_map.yaml").write_text(
            """format: tamiya_local_hd_map_v1
primary_lane_id: "lane_001"
lanes:
  - id: "lane_001"
    closed_loop: true
    left_bound:
      - [-2, 12, 0]
      - [5, 12, 0]
      - [12, 12, 0]
    right_bound:
      - [-2, -2, 0]
      - [5, -2, 0]
      - [12, -2, 0]
    centerline:
      - [0, 0, 0]
      - [10, 0, 0]
      - [10, 10, 0]
      - [0, 10, 0]
section_gates:
  - id: "gate_a"
    lane_id: "lane_001"
    s_m: 5
    line:
      - [5, -1, 0]
      - [5, 1, 0]
  - id: "gate_b"
    lane_id: "lane_001"
    s_m: 25
    line:
      - [5, 9, 0]
      - [5, 11, 0]
sections:
  - id: "section_a"
    lane_id: "lane_001"
    start_gate_id: "gate_a"
    end_gate_id: "gate_b"
    start_s_m: 5
    end_s_m: 25
  - id: "section_b"
    lane_id: "lane_001"
    start_gate_id: "gate_b"
    end_gate_id: "gate_a"
    start_s_m: 25
    end_s_m: 5
    wrap: true
""",
            encoding="utf-8",
        )
        (map_dir / "closed_course_hd_map_centerline.csv").write_text(
            "# x_m,y_m,w_tr_right_m,w_tr_left_m\n0,0,1,1\n10,0,1,1\n10,10,1,1\n0,10,1,1\n",
            encoding="utf-8",
        )
        return SimpleNamespace(map_root=root), map_dir

    def test_closed_two_gate_profile_rejects_reverse_direction(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_closed_two_gate_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Clockwise contract", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]
            with self.assertRaisesRegex(ValueError, "direction is opposite"):
                update_custom_line(
                    config,
                    {
                        "map_dir": str(map_dir),
                        "id": line_id,
                        "points": [
                            {"x_m": 0.0, "y_m": 0.0},
                            {"x_m": 0.0, "y_m": 10.0},
                            {"x_m": 10.0, "y_m": 10.0},
                            {"x_m": 10.0, "y_m": 0.0},
                        ],
                        "default_speed_mps": 1.0,
                        "section_speeds_mps": {},
                    },
                )

    def test_compiles_named_section_targets_with_gate_points_and_safety_envelope(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_section_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {
                    "map_dir": str(map_dir),
                    "name": "Section profile",
                    "base": "raceline",
                    "default_speed_mps": 1.0,
                },
            )
            line = detail["custom_lines"][0]
            self.assertEqual(line["default_speed_mps"], 1.0)
            self.assertEqual(line["section_speeds_mps"], {"section_a": 1.8})
            self.assertEqual(
                [section["configured"] for section in line["speed_sections"]],
                [True, False],
            )

            detail = update_custom_line(
                config,
                {
                    "map_dir": str(map_dir),
                    "id": line["id"],
                    "points": [{"x_m": 0.0, "y_m": 0.0}, {"x_m": 5.0, "y_m": 0.0}, {"x_m": 10.0, "y_m": 0.0}],
                    "default_speed_mps": 1.2,
                    "section_speeds_mps": {"section_a": 2.5, "section_b": 0.5},
                },
            )
            line = detail["custom_lines"][0]
            self.assertEqual(line["point_count"], 3)
            self.assertGreater(line["trajectory_point_count"], line["point_count"])
            self.assertEqual(line["section_speeds_mps"], {"section_a": 2.5, "section_b": 0.5})
            self.assertLessEqual(line["validation"]["max_speed_mps"], 3.0)
            self.assertLessEqual(line["validation"]["max_accel_mps2"], 1.5 + 1.0e-8)
            self.assertLessEqual(line["validation"]["max_decel_mps2"], 2.5 + 1.0e-8)

            line_dir = map_dir / "custom_lines" / line["id"]
            manifest = json.loads((line_dir / "custom_line.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["speed_profile_mode"], "sections")
            self.assertTrue(manifest["hd_map_sha256"])
            self.assertTrue(manifest["section_layout_fingerprint"])
            self.assertTrue(all("speed_mps" not in point for point in manifest["points"]))

            rows = [
                [float(value) for value in row.split(";")]
                for row in (line_dir / "trajectory.csv").read_text(encoding="utf-8").splitlines()
                if row and not row.startswith("#")
            ]
            by_x = {round(row[1], 6): row for row in rows}
            self.assertAlmostEqual(by_x[5.0][5], 0.5, places=6)
            self.assertAlmostEqual(by_x[8.0][5], 0.5, places=6)
            self.assertGreater(by_x[3.0][5], 1.2)
            self.assertEqual({len(row) for row in rows}, {7})

            activate_custom_line(config, {"map_dir": str(map_dir), "id": line["id"]})
            hd_map_path = map_dir / "section_course_hd_map.yaml"
            hd_map_path.write_text(
                hd_map_path.read_text(encoding="utf-8").replace("[2, -1, 0]", "[2.2, -1, 0]"),
                encoding="utf-8",
            )
            stale_detail = build_map_detail(config, str(map_dir))
            self.assertEqual(stale_detail["active_custom_line_id"], "")
            self.assertTrue(stale_detail["custom_lines"][0]["section_layout_stale"])

    def test_world_gate_intersections_define_sections_when_custom_station_differs_from_hd_station(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_section_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Wavy stations", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]
            detail = update_custom_line(
                config,
                {
                    "map_dir": str(map_dir),
                    "id": line_id,
                    "points": [
                        {"x_m": 0.0, "y_m": 0.0},
                        {"x_m": 4.0, "y_m": 0.5},
                        {"x_m": 7.0, "y_m": -0.5},
                        {"x_m": 10.0, "y_m": 0.0},
                    ],
                    "default_speed_mps": 1.2,
                    "section_speeds_mps": {"section_a": 1.8, "section_b": 0.5},
                },
            )
            line = detail["custom_lines"][0]
            compiled_sections = {section["id"]: section for section in line["speed_sections"]}
            self.assertNotAlmostEqual(compiled_sections["section_a"]["custom_start_s_m"], 2.0, places=4)
            self.assertNotAlmostEqual(compiled_sections["section_a"]["custom_end_s_m"], 5.0, places=4)
            self.assertNotAlmostEqual(compiled_sections["section_b"]["custom_end_s_m"], 8.0, places=4)

            trajectory_path = map_dir / "custom_lines" / line_id / "trajectory.csv"
            rows = [
                [float(value) for value in row.split(";")]
                for row in trajectory_path.read_text(encoding="utf-8").splitlines()
                if row and not row.startswith("#")
            ]
            for gate_x in (2.0, 5.0, 8.0):
                self.assertEqual(sum(abs(row[1] - gate_x) <= 1.0e-8 for row in rows), 1)
            gate_5 = next(row for row in rows if abs(row[1] - 5.0) <= 1.0e-8)
            gate_8 = next(row for row in rows if abs(row[1] - 8.0) <= 1.0e-8)
            self.assertAlmostEqual(gate_5[5], 0.5, places=6)
            self.assertAlmostEqual(gate_8[5], 0.5, places=6)

    def test_rejects_unknown_or_stopping_section_targets_and_ambiguous_gate_crossings(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_section_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Strict sections", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]
            with self.assertRaisesRegex(ValueError, "unknown primary lane section"):
                update_custom_line(
                    config,
                    {
                        "map_dir": str(map_dir),
                        "id": line_id,
                        "default_speed_mps": 1.0,
                        "section_speeds_mps": {"missing": 1.0},
                    },
                )
            with self.assertRaisesRegex(ValueError, "at least 0.1"):
                update_custom_line(
                    config,
                    {
                        "map_dir": str(map_dir),
                        "id": line_id,
                        "default_speed_mps": 1.0,
                        "section_speeds_mps": {"section_a": 0.0},
                    },
                )
            with self.assertRaisesRegex(ValueError, "exactly once"):
                update_custom_line(
                    config,
                    {
                        "map_dir": str(map_dir),
                        "id": line_id,
                        "points": [
                            {"x_m": 0.0, "y_m": 0.0},
                            {"x_m": 4.0, "y_m": 0.2},
                            {"x_m": 1.0, "y_m": -0.2},
                            {"x_m": 10.0, "y_m": 0.0},
                        ],
                        "default_speed_mps": 1.0,
                        "section_speeds_mps": {},
                    },
                )
            with self.assertRaisesRegex(ValueError, "gate order is inconsistent"):
                update_custom_line(
                    config,
                    {
                        "map_dir": str(map_dir),
                        "id": line_id,
                        "points": [
                            {"x_m": 10.0, "y_m": 0.0},
                            {"x_m": 5.0, "y_m": 0.0},
                            {"x_m": 0.0, "y_m": 0.0},
                        ],
                        "default_speed_mps": 1.0,
                        "section_speeds_mps": {},
                    },
                )

    def test_can_repair_renamed_sections_from_complete_new_authoring_payload(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_section_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Repairable", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]
            hd_map_path = map_dir / "section_course_hd_map.yaml"
            hd_map_path.write_text(
                hd_map_path.read_text(encoding="utf-8").replace(
                    'id: "section_a"',
                    'id: "section_new"',
                ),
                encoding="utf-8",
            )

            stale = build_map_detail(config, str(map_dir))["custom_lines"][0]
            self.assertFalse(stale["valid"])
            self.assertTrue(stale["repairable"])
            self.assertEqual(len(stale["points"]), 3)
            self.assertEqual([section["id"] for section in stale["speed_sections"]], ["section_new", "section_b"])

            detail = update_custom_line(
                config,
                {
                    "map_dir": str(map_dir),
                    "id": line_id,
                    "points": [{"x_m": point["x_m"], "y_m": point["y_m"]} for point in stale["points"]],
                    "default_speed_mps": 1.0,
                    "section_speeds_mps": {"section_new": 1.4},
                },
            )
            repaired = detail["custom_lines"][0]
            self.assertTrue(repaired["valid"])
            self.assertEqual(repaired["section_speeds_mps"], {"section_new": 1.4})

    def test_gate_save_deactivates_with_visible_issue_when_active_line_cannot_recompile(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_section_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Active safety", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]
            activate_custom_line(config, {"map_dir": str(map_dir), "id": line_id})

            detail = save_section_gates(
                config,
                {
                    "map_dir": str(map_dir),
                    "section_gates": [
                        {"id": "gate_001", "lane_id": "lane_001", "s_m": 2, "line": [[2, 2], [2, 3]]},
                        {"id": "gate_002", "lane_id": "lane_001", "s_m": 5, "line": [[5, -1], [5, 1]]},
                        {"id": "gate_003", "lane_id": "lane_001", "s_m": 8, "line": [[8, -1], [8, 1]]},
                    ],
                },
            )
            self.assertEqual(detail["active_custom_line_id"], "")
            self.assertIn("exactly once", detail["custom_line_catalog"]["active_issue"])
            self.assertFalse((map_dir / "section_course_custom_line.csv").exists())

    def test_gate_save_makes_duplicate_fallback_ids_unique(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_section_map(Path(temporary_directory))
            detail = save_section_gates(
                config,
                {
                    "map_dir": str(map_dir),
                    "section_gates": [
                        {"id": "gate_002", "lane_id": "lane_001", "s_m": 2, "line": [[2, -1], [2, 1]]},
                        {"id": "gate_002", "lane_id": "lane_001", "s_m": 5, "line": [[5, -1], [5, 1]]},
                        {"id": "gate_003", "lane_id": "lane_001", "s_m": 8, "line": [[8, -1], [8, 1]]},
                    ],
                },
            )
            gate_ids = [gate["id"] for gate in detail["hd_map"]["section_gates"]]
            self.assertEqual(gate_ids, ["gate_002", "gate_002_2", "gate_003"])
            self.assertEqual(len(gate_ids), len(set(gate_ids)))

    def test_reads_legacy_point_speeds_and_invalidates_active_profile_after_external_gate_change(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_section_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Legacy compatible", "base": "centerline"},
            )
            line = detail["custom_lines"][0]
            line_dir = map_dir / "custom_lines" / line["id"]
            manifest_path = line_dir / "custom_line.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["points"] = [
                {"x_m": 0.0, "y_m": 0.0, "speed_mps": 1.0},
                {"x_m": 5.0, "y_m": 0.0, "speed_mps": 1.5},
                {"x_m": 10.0, "y_m": 0.0, "speed_mps": 1.0},
            ]
            for key in (
                "default_speed_mps",
                "section_speeds_mps",
                "speed_profile_mode",
                "speed_authoring",
                "section_layout_fingerprint",
                "section_layout_hash",
                "hd_map_sha256",
            ):
                manifest.pop(key, None)
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

            detail = build_map_detail(config, str(map_dir))
            legacy = detail["custom_lines"][0]
            self.assertTrue(legacy["valid"])
            self.assertEqual(legacy["speed_profile_mode"], "legacy_points")
            self.assertEqual(legacy["default_speed_mps"], 1.0)

            activate_custom_line(config, {"map_dir": str(map_dir), "id": line["id"]})
            active_meta_path = map_dir / "section_course_custom_line.meta.json"
            active_meta = json.loads(active_meta_path.read_text(encoding="utf-8"))
            for key in (
                "speed_profile_mode",
                "speed_authoring",
                "section_layout_fingerprint",
                "section_layout_hash",
                "hd_map_sha256",
            ):
                active_meta.pop(key, None)
            active_meta_path.write_text(json.dumps(active_meta), encoding="utf-8")
            self.assertEqual(build_map_detail(config, str(map_dir))["active_custom_line_id"], line["id"])

            hd_map_path = map_dir / "section_course_hd_map.yaml"
            hd_map_path.write_text(
                hd_map_path.read_text(encoding="utf-8").replace("[2, -1, 0]", "[2.2, -1, 0]"),
                encoding="utf-8",
            )
            # Legacy canonical metadata deliberately has no provenance fields,
            # so it remains readable for backward compatibility.
            self.assertEqual(build_map_detail(config, str(map_dir))["active_custom_line_id"], line["id"])

    def test_legacy_zero_speed_profile_is_readable_but_cannot_be_activated(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Legacy stop", "base": "centerline"},
            )
            line = detail["custom_lines"][0]
            manifest_path = map_dir / "custom_lines" / line["id"] / "custom_line.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["points"] = [
                {"x_m": 0.0, "y_m": 0.0, "speed_mps": 0.0},
                {"x_m": 1.0, "y_m": 0.0, "speed_mps": 1.0},
                {"x_m": 2.0, "y_m": 0.0, "speed_mps": 1.0},
            ]
            for key in (
                "default_speed_mps",
                "section_speeds_mps",
                "speed_profile_mode",
                "speed_authoring",
                "section_layout_fingerprint",
                "section_layout_hash",
                "hd_map_sha256",
            ):
                manifest.pop(key, None)
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

            legacy = build_map_detail(config, str(map_dir))["custom_lines"][0]
            self.assertTrue(legacy["valid"])
            self.assertTrue(legacy["repairable"])
            self.assertEqual(legacy["default_speed_mps"], 0.0)
            with self.assertRaisesRegex(ValueError, "below 0.1 m/s"):
                activate_custom_line(config, {"map_dir": str(map_dir), "id": line["id"]})
            self.assertFalse((map_dir / "course_a_custom_line.csv").exists())

    def test_creates_multiple_named_lines_updates_and_activates(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_map(Path(temporary_directory))

            detail = create_custom_line(
                config,
                {
                    "map_dir": str(map_dir),
                    "name": "Safe manual",
                    "base": "centerline",
                    "default_speed_mps": 1.25,
                },
            )
            self.assertEqual(len(detail["custom_lines"]), 1)
            safe = detail["custom_lines"][0]
            self.assertEqual(safe["id"], "safe-manual")
            self.assertEqual(safe["source_type"], "centerline")
            self.assertTrue(safe["valid"])
            self.assertTrue(safe["validation"]["containment_checked"])
            self.assertAlmostEqual(safe["min_clearance_m"], 1.0)
            self.assertEqual([point["speed_mps"] for point in safe["points"]], [1.25, 1.25, 1.25])

            detail = create_custom_line(
                config,
                {
                    "map_dir": str(map_dir),
                    "name": "Attack manual",
                    "source_type": "raceline",
                },
            )
            self.assertEqual(len(detail["custom_lines"]), 2)
            attack = next(item for item in detail["custom_lines"] if item["id"] == "attack-manual")
            self.assertEqual(attack["default_speed_mps"], 1.0)
            self.assertEqual(attack["section_speeds_mps"], {})
            self.assertTrue(all(point["speed_mps"] <= 1.0 for point in attack["points"]))

            detail = update_custom_line(
                config,
                {
                    "map_dir": str(map_dir),
                    "id": safe["id"],
                    "name": "Safe manual v2",
                    "closed_loop": False,
                    "points": [
                        {"x_m": 0.0, "y_m": 0.0, "speed_mps": 1.0},
                        {"x_m": 1.0, "y_m": 0.1, "speed_mps": 2.0},
                        {"x_m": 2.0, "y_m": 0.0, "speed_mps": 1.0},
                    ],
                },
            )
            safe = next(item for item in detail["custom_lines"] if item["id"] == "safe-manual")
            self.assertEqual(safe["name"], "Safe manual v2")
            self.assertEqual(safe["revision"], 2)
            trajectory_path = map_dir / "custom_lines" / safe["id"] / "trajectory.csv"
            rows = [line for line in trajectory_path.read_text(encoding="utf-8").splitlines() if not line.startswith("#")]
            s_values = [float(line.split(";")[0]) for line in rows]
            self.assertEqual(s_values[0], 0.0)
            self.assertTrue(all(current > previous for previous, current in zip(s_values, s_values[1:])))
            for line in rows:
                self.assertEqual(len(line.split(";")), 7)

            detail = activate_custom_line(config, {"map_dir": str(map_dir), "id": safe["id"]})
            self.assertEqual(detail["active_custom_line_id"], safe["id"])
            self.assertTrue((map_dir / "course_a_custom_line.csv").exists())
            active_meta = json.loads((map_dir / "course_a_custom_line.meta.json").read_text(encoding="utf-8"))
            self.assertEqual(active_meta["id"], safe["id"])
            self.assertEqual(active_meta["revision"], 2)
            self.assertEqual(active_meta["format"], "jetpilot_custom_line_v1")
            self.assertEqual(active_meta["trajectory_csv"], "course_a_custom_line.csv")

            detail = delete_custom_line(config, {"map_dir": str(map_dir), "id": attack["id"]})
            self.assertEqual([item["id"] for item in detail["custom_lines"]], [safe["id"]])

    def test_rejects_invalid_points_and_marks_stale_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Editable", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]

            invalid_point_sets = [
                [
                    {"x_m": 0.0, "y_m": 0.0, "speed_mps": 1.0},
                    {"x_m": 0.0, "y_m": 0.0, "speed_mps": 1.0},
                    {"x_m": 2.0, "y_m": 0.0, "speed_mps": 1.0},
                ],
                [
                    {"x_m": 0.0, "y_m": 0.0, "speed_mps": 1.0},
                    {"x_m": 1.0, "y_m": 2.0, "speed_mps": 1.0},
                    {"x_m": 2.0, "y_m": 0.0, "speed_mps": 1.0},
                ],
                [
                    {"x_m": 0.0, "y_m": 0.0, "speed_mps": 1.0},
                    {"x_m": 1.0, "y_m": 0.0, "speed_mps": -0.1},
                    {"x_m": 2.0, "y_m": 0.0, "speed_mps": 1.0},
                ],
            ]
            for points in invalid_point_sets:
                with self.subTest(points=points), self.assertRaises(ValueError):
                    update_custom_line(
                        config,
                        {"map_dir": str(map_dir), "id": line_id, "closed_loop": False, "points": points},
                    )

            (map_dir / "course_a_hd_map_centerline.csv").write_text(
                "# x_m,y_m,w_tr_right_m,w_tr_left_m\n0,0.1,1,1\n1,0.1,1,1\n2,0.1,1,1\n",
                encoding="utf-8",
            )
            detail = build_map_detail(config, str(map_dir))
            self.assertTrue(detail["custom_lines"][0]["source_stale"])

            with self.assertRaises(ValueError):
                activate_custom_line(config, {"map_dir": str(map_dir), "id": "../escape"})

    def test_deleting_active_line_clears_canonical_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Temporary", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]
            activate_custom_line(config, {"map_dir": str(map_dir), "id": line_id})

            detail = delete_custom_line(config, {"map_dir": str(map_dir), "id": line_id})

            self.assertEqual(detail["active_custom_line_id"], "")
            self.assertEqual(detail["custom_lines"], [])
            self.assertFalse((map_dir / "course_a_custom_line.csv").exists())
            self.assertFalse((map_dir / "course_a_custom_line.meta.json").exists())

    def test_canonical_pair_is_the_authoritative_active_selection(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Authoritative", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]
            detail = activate_custom_line(config, {"map_dir": str(map_dir), "id": line_id})
            self.assertEqual(detail["active_custom_line_id"], line_id)

            (map_dir / "custom_lines" / "active.json").write_text(
                json.dumps({"id": "stale-pointer"}),
                encoding="utf-8",
            )
            detail = build_map_detail(config, str(map_dir))
            self.assertEqual(detail["active_custom_line_id"], line_id)

            with (map_dir / "course_a_custom_line.csv").open("a", encoding="utf-8") as handle:
                handle.write("# tampered\n")
            detail = build_map_detail(config, str(map_dir))
            self.assertEqual(detail["active_custom_line_id"], "")
            self.assertIn("does not match", detail["custom_line_catalog"]["active_issue"])

    def test_rejects_symlinked_root_and_ignores_predictable_temp_symlinks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            config, map_dir = self._make_map(root)
            outside = root / "outside"
            outside.mkdir()
            (map_dir / "custom_lines").symlink_to(outside, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, "symbolic link"):
                create_custom_line(
                    config,
                    {"map_dir": str(map_dir), "name": "Blocked", "base": "centerline"},
                )

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            config, map_dir = self._make_map(root)
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Atomic", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]
            line_dir = map_dir / "custom_lines" / line_id
            sentinel = root / "sentinel.txt"
            sentinel.write_text("unchanged", encoding="utf-8")
            (line_dir / "trajectory.csv.tmp").symlink_to(sentinel)
            (line_dir / "custom_line.json.tmp").symlink_to(sentinel)

            update_custom_line(
                config,
                {"map_dir": str(map_dir), "id": line_id, "name": "Atomic updated"},
            )

            self.assertEqual(sentinel.read_text(encoding="utf-8"), "unchanged")
            self.assertTrue((line_dir / "trajectory.csv.tmp").is_symlink())
            self.assertTrue((line_dir / "custom_line.json.tmp").is_symlink())

    def test_enforces_point_limit_and_speed_feasibility_constraints(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_map(Path(temporary_directory))
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Limits", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]
            too_many = [
                {"x_m": index * 0.001, "y_m": 0.0, "speed_mps": 1.0}
                for index in range(CUSTOM_LINE_MAX_POINTS + 1)
            ]
            with self.assertRaisesRegex(ValueError, "at most"):
                update_custom_line(
                    config,
                    {"map_dir": str(map_dir), "id": line_id, "closed_loop": False, "points": too_many},
                )

            with self.assertRaisesRegex(ValueError, "at least 0.1"):
                update_custom_line(
                    config,
                    {"map_dir": str(map_dir), "id": line_id, "default_speed_mps": 0.0},
                )

            detail = update_custom_line(
                config,
                {
                    "map_dir": str(map_dir),
                    "id": line_id,
                    "closed_loop": False,
                    "default_speed_mps": 4.0,
                },
            )
            line = detail["custom_lines"][0]
            self.assertEqual(line["default_speed_mps"], 4.0)
            self.assertLessEqual(line["validation"]["max_speed_mps"], 3.0)
            self.assertTrue(line["validation"]["speed_adjusted"])

    def test_rejects_segment_that_leaves_concave_lane_between_vertices(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_map(Path(temporary_directory))
            (map_dir / "course_a_hd_map.yaml").write_text(
                """format: tamiya_local_hd_map_v1
primary_lane_id: "lane_001"
lanes:
  - id: "lane_001"
    closed_loop: false
    left_bound:
      - [0, 0, 0]
      - [4, 0, 0]
      - [4, 4, 0]
      - [3, 4, 0]
    right_bound:
      - [0, 4, 0]
      - [1, 4, 0]
      - [1, 1, 0]
      - [3, 1, 0]
    centerline:
      - [0.5, 0.5, 0]
      - [2, 0.5, 0]
      - [3.5, 0.5, 0]
""",
                encoding="utf-8",
            )
            (map_dir / "course_a_hd_map_centerline.csv").write_text(
                "# x_m,y_m,w_tr_right_m,w_tr_left_m\n0.5,0.5,1,1\n2,0.5,1,1\n3.5,0.5,1,1\n",
                encoding="utf-8",
            )
            detail = create_custom_line(
                config,
                {"map_dir": str(map_dir), "name": "Concave", "base": "centerline"},
            )
            line_id = detail["custom_lines"][0]["id"]

            with self.assertRaisesRegex(ValueError, "segment"):
                update_custom_line(
                    config,
                    {
                        "map_dir": str(map_dir),
                        "id": line_id,
                        "closed_loop": False,
                        "points": [
                            {"x_m": 0.5, "y_m": 2.0, "speed_mps": 1.0},
                            {"x_m": 3.5, "y_m": 2.0, "speed_mps": 1.0},
                        ],
                    },
                )

    def test_requires_primary_lane_bounds_for_custom_line_validation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            config, map_dir = self._make_map(Path(temporary_directory))
            (map_dir / "course_a_hd_map.yaml").unlink()

            with self.assertRaisesRegex(ValueError, "primary lane geometry"):
                create_custom_line(
                    config,
                    {"map_dir": str(map_dir), "name": "Unbounded", "base": "centerline"},
                )


if __name__ == "__main__":
    unittest.main()
