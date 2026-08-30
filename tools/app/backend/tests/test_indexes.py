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


if __name__ == "__main__":
    unittest.main()
