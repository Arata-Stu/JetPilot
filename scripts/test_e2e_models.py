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

    def test_bringup_requires_explicit_model_and_accepts_compatible_override(self):
        for preset in ("e2e", "e2e-steering"):
            for sensor, prefix in (("realsense", "camera"), ("event-camera", "event")):
                expected = f"{prefix}_{'steering' if preset == 'e2e-steering' else 'control'}"
                args = ["bash", str(ROOT / "scripts/bringup.sh"), preset, "--vehicle", "jpbb", "--sensor-kit", sensor, "--dry-run"]
                result = subprocess.run(args, text=True, capture_output=True)
                self.assertNotEqual(result.returncode, 0)
                result = subprocess.run(args + ["--e2e-model", str(self.root / expected)], text=True, capture_output=True, check=True)
                self.assertIn(f"e2e_model_root:={self.root / expected}", result.stdout)
                self.assertIn("e2e_network_image_width:=212", result.stdout)
        bad = subprocess.run(args + ["--e2e-model", str(self.root / "camera_control")], text=True, capture_output=True)
        self.assertNotEqual(bad.returncode, 0)

    def test_auto_throttle_lists_both_targets(self):
        records = list_models(self.root, "rgb", None)
        self.assertEqual({r["steering_only"] for r in records}, {True, False})

    def test_e2e_adapts_to_selected_model_and_preserves_fixed_value(self):
        for target in ("control", "steering"):
            result = subprocess.run([
                "bash", str(ROOT / "scripts/bringup.sh"), "e2e", "--dry-run",
                "--e2e-model", str(self.root / f"camera_{target}"),
                "--set", "fixed_throttle:=0.25",
            ], text=True, capture_output=True, check=True)
            self.assertIn(f"e2e_fixed_throttle_mode:={'true' if target == 'steering' else 'false'}", result.stdout)
            self.assertIn("fixed_throttle:=0.25", result.stdout)

    def test_explicit_throttle_conflict_and_rgb_off_are_rejected(self):
        args = ["bash", str(ROOT / "scripts/bringup.sh"), "e2e", "--dry-run",
                "--e2e-model", str(self.root / "camera_steering")]
        for override in ("e2e_fixed_throttle_mode:=false", "sensor_kit_rgb_fps:=0"):
            result = subprocess.run(args + ["--set", override], text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
        result = subprocess.run(args + ["--set", "sensor_kit_infra_fps:=0"], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_required_fixed_throttle_metadata_is_recognized(self):
        path = self.root / "camera_steering" / "metadata.json"
        metadata = json.loads(path.read_text())
        del metadata["steering_only"]
        metadata["output"] = {"requires_fixed_throttle_mode": True}
        path.write_text(json.dumps(metadata))
        self.assertTrue(inspect_model(path.parent, "rgb", None)["steering_only"])

    def test_online_models_require_single_image(self):
        path = self.root / "camera_control" / "metadata.json"
        metadata = json.loads(path.read_text())
        metadata["input"]["shape"] = [1, 4, 3, 120, 212]
        path.write_text(json.dumps(metadata))
        with self.assertRaisesRegex(ValueError, "single RGB-format image"):
            inspect_model(path.parent, "rgb", False)

    def test_tensorrt_build_requires_explicit_model(self):
        result = subprocess.run(
            ["bash", str(ROOT / "scripts/e2e_trt.sh")],
            text=True,
            capture_output=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Specify the model directory", result.stderr)

    def test_deploy_and_tui_do_not_update_latest_alias(self):
        deploy = (
            ROOT / "python_ws/jetpilot_e2e_training/scripts/deploy_model.sh"
        ).read_text()
        tui = (
            ROOT
            / "ros2_ws/src/perception/jetpilot_e2e_inference/scripts/deploy_tensorrt_tui.sh"
        ).read_text()
        self.assertIn("--name", deploy)
        self.assertNotIn("ln -sfn", deploy)
        self.assertNotIn("ln -sfn", tui)


if __name__ == "__main__":
    unittest.main()
