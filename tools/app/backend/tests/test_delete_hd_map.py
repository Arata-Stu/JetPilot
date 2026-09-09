import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from jetpilot_console.map_detail import delete_hd_map


class DeleteHdMapTest(unittest.TestCase):
    def test_only_working_hd_artifacts_are_archived(self):
        with tempfile.TemporaryDirectory() as root:
            config = SimpleNamespace(map_root=Path(root))
            folder = Path(root) / "course"
            removed = ["course_hd_map.yaml", "course_hd_map_centerline.csv", "course_raceline.csv",
                       "course_raceline.meta.json", "course_line_preview.png", "course_custom_line.csv",
                       "course_custom_line.meta.json", "hd_map_versions/active.json", "custom_lines/active.json"]
            kept = ["vslam_landmarks.yaml", "vslam_landmarks.png", "cuvslam_map/data",
                    "cuvgl_map/data", "vslam_reference_snapshot.json", "custom_lines/line_001/trajectory.csv",
                    "hd_map_versions/ver_001/hd_map.yaml", "unrelated.csv"]
            for name in removed + kept:
                path = folder / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(name)
            with patch("jetpilot_console.map_detail.build_map_detail", return_value={"hd_map": {"exists": False}}):
                result = delete_hd_map(config, {"map_dir": str(folder)})
            backup = Path(result["deleted_hd_map_backup"])
            for name in removed:
                self.assertFalse((folder / name).exists())
                self.assertEqual((backup / name).read_text(), name)
            for name in kept:
                self.assertEqual((folder / name).read_text(), name)

    def test_failure_rolls_back_and_links_and_root_are_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            config = SimpleNamespace(map_root=Path(root))
            folder = Path(root) / "course"
            folder.mkdir()
            hd = folder / "course_hd_map.yaml"
            hd.write_text("original")
            with patch("jetpilot_console.map_detail.build_map_detail", side_effect=ValueError("bad detail")):
                with self.assertRaises(ValueError):
                    delete_hd_map(config, {"map_dir": str(folder)})
            self.assertEqual(hd.read_text(), "original")
            self.assertEqual(list(folder.glob(".deleted-hd-map-*")), [])
            (folder / "custom_lines").symlink_to(Path(root), target_is_directory=True)
            with self.assertRaises(ValueError):
                delete_hd_map(config, {"map_dir": str(folder)})
            self.assertEqual(hd.read_text(), "original")
            for value in ["", root, str(Path(root).parent)]:
                with self.assertRaises(ValueError):
                    delete_hd_map(config, {"map_dir": value})
