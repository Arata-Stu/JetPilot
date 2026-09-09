import json
import subprocess
import tempfile
import unittest
from pathlib import Path

from e2e_models import inspect_model, list_models


ROOT = Path(__file__).resolve().parents[1]


class E2EModelSelectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for sensor in ("camera", "event"):
            for target in ("control", "steering"):
                path = self.root / f"{sensor}_{target}"
                path.mkdir()
                (path / "model.onnx").write_bytes(b"test")
                (path / "metadata.json").write_text(json.dumps({
                    "model_name": path.name, "task": "control",
                    "steering_only": target == "steering",
                    "input": {"shape": [1, 3, 120, 212]},
                    "config": {"data": {"image_topic": "/event_camera/event_image" if sensor == "event" else "/realsense/color/image_raw"}},
                }))

    def test_candidates_match_both_sensor_and_learning_target(self):
        (self.root / "latest").symlink_to("camera_control")
        for sensor, prefix in (("rgb", "camera"), ("event", "event")):
            for steering in (False, True):
                records = list_models(self.root, sensor, steering)
                self.assertEqual(len(records), 1)
                self.assertEqual(records[0]["name"], f"{prefix}_{'steering' if steering else 'control'}")

    def test_explicit_incompatible_model_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "sensor mismatch"):
            inspect_model(self.root / "camera_control", "event", False)
        with self.assertRaisesRegex(ValueError, "learning target"):
            inspect_model(self.root / "event_control", "event", True)

    def test_bringup_four_default_destinations_and_explicit_override(self):
        for preset in ("e2e", "e2e-steering"):
            for sensor, prefix in (("realsense", "camera"), ("event-camera", "event")):
                expected = f"{prefix}_{'steering' if preset == 'e2e-steering' else 'control'}"
                args = ["bash", str(ROOT / "scripts/bringup.sh"), preset, "--vehicle", "jpbb", "--sensor-kit", sensor, "--dry-run"]
                result = subprocess.run(args, text=True, capture_output=True, check=True)
                self.assertIn(f"e2e_model_root:=/workspaces/ros2_ws/models/e2e/{expected}", result.stdout)
                result = subprocess.run(args + ["--e2e-model", str(self.root / expected)], text=True, capture_output=True, check=True)
                self.assertIn(f"e2e_model_root:={self.root / expected}", result.stdout)
                self.assertIn("e2e_network_image_width:=212", result.stdout)
        bad = subprocess.run(args + ["--e2e-model", str(self.root / "camera_control")], text=True, capture_output=True)
        self.assertNotEqual(bad.returncode, 0)

    def test_online_models_require_single_image(self):
        path = self.root / "camera_control" / "metadata.json"
        metadata = json.loads(path.read_text())
        metadata["input"]["shape"] = [1, 4, 3, 120, 212]
        path.write_text(json.dumps(metadata))
        with self.assertRaisesRegex(ValueError, "single RGB-format image"):
            inspect_model(path.parent, "rgb", False)


if __name__ == "__main__":
    unittest.main()
