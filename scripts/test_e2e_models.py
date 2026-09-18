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
        tensor_path = self.root / "event_tensor_control"
        tensor_path.mkdir()
        (tensor_path / "model.onnx").write_bytes(b"test")
        (tensor_path / "metadata.json").write_text(json.dumps({
            "model_name": tensor_path.name,
            "task": "control",
            "steering_only": False,
            "modality": "event_tensor",
            "input": {
                "shape": [1, 20, 120, 212],
                "mean": [0.1] * 20,
                "std": [0.2] * 20,
            },
            "event_representation": {
                "bins": 10,
                "window_ms": 40.0,
                "stride_ms": 4.0,
                "polarity_mode": "separate",
                "polarity_layout": "polarity_major",
                "temporal_interpolation": "none",
            },
        }))
        async_path = self.root / "rgb_event_async_control"
        async_path.mkdir()
        for filename in ("model.onnx", "rgb_encoder.onnx", "event_updater.onnx"):
            (async_path / filename).write_bytes(b"test")
        (async_path / "metadata.json").write_text(json.dumps({
            "model_name": async_path.name,
            "task": "control",
            "steering_only": False,
            "modality": "rgb_event_async",
            "image_topic": "/realsense/color/image_raw",
            "inputs": [
                {
                    "name": "rgb", "shape": [1, 3, 120, 212],
                    "mean": [0.485, 0.456, 0.406],
                    "std": [0.229, 0.224, 0.225],
                },
                {
                    "name": "event_tensors", "shape": [1, 32, 20, 120, 212],
                    "mean": [0.1] * 20, "std": [0.2] * 20,
                },
            ],
            "event_representation": {
                "event_topic": "/event_camera/events",
                "bins": 10,
                "window_ms": 40.0,
                "stride_ms": 4.0,
                "polarity_mode": "separate",
                "polarity_layout": "polarity_major",
                "temporal_interpolation": "none",
            },
            "config": {"data": {"event_sample_hz": 250.0}},
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

    def test_event_tensor_model_requires_explicit_modality_and_channel_match(self):
        path = self.root / "event_tensor_control"
        record = inspect_model(path, "event", False, event_tensor_channels=20)
        self.assertEqual(record["width"], 212)
        self.assertEqual(record["event_representation"]["backend"], "cuda")
        with self.assertRaisesRegex(ValueError, "event tensor model mismatch"):
            inspect_model(path, "event", False, event_tensor_channels=10)
        with self.assertRaisesRegex(ValueError, "RGB-format"):
            inspect_model(path, "event", False)

    def test_benchmark_only_model_requires_explicit_permission(self):
        path = self.root / "event_tensor_dummy"
        path.mkdir()
        (path / "model.onnx").write_bytes(b"test")
        source = json.loads(
            (self.root / "event_tensor_control" / "metadata.json").read_text()
        )
        source["model_name"] = "event_tensor_dummy"
        source["benchmark_only"] = True
        source["trained"] = False
        (path / "metadata.json").write_text(json.dumps(source))

        with self.assertRaisesRegex(ValueError, "benchmark-only"):
            inspect_model(path, "event", False, event_tensor_channels=20)
        record = inspect_model(
            path,
            "event",
            False,
            event_tensor_channels=20,
            allow_benchmark_only=True,
        )
        self.assertEqual(record["name"], "event_tensor_dummy")

    def test_event_tensor_bringup_accepts_matching_twenty_channel_model(self):
        result = subprocess.run([
            "bash", str(ROOT / "scripts/bringup.sh"), "e2e", "--dry-run",
            "--e2e-model", str(self.root / "event_tensor_control"),
            "--set", "e2e_event_tensor_mode:=true",
            "--set", "e2e_event_bins:=10",
            "--set", "e2e_event_polarity_mode:=separate",
        ], text=True, capture_output=True, check=True)
        self.assertIn("e2e_event_tensor_mode:=true", result.stdout)
        self.assertIn("e2e_event_window_ms:=40.0", result.stdout)
        self.assertIn("e2e_event_stride_ms:=4.0", result.stdout)
        self.assertIn("e2e_event_representation_backend:=cuda", result.stdout)
        self.assertIn("e2e_event_tensor_mean:=", result.stdout)
        launch_source = (
            ROOT
            / "ros2_ws/src/perception/jetpilot_e2e_inference/launch/e2e_tensor_rt.launch.py"
        ).read_text()
        self.assertIn('"event_representation_backend", default_value="cuda"', launch_source)
        self.assertIn('"event_inference_policy", default_value="periodic"', launch_source)
        self.assertIn('"event_cuda_events_per_transfer", default_value="8192"', launch_source)

    def test_event_image_bringup_does_not_read_unset_event_channels(self):
        result = subprocess.run([
            "bash", str(ROOT / "scripts/bringup.sh"), "e2e", "--dry-run",
            "--e2e-model", str(self.root / "event_control"),
            "--set", "e2e_event_image_mode:=true",
        ], text=True, capture_output=True, check=True)
        self.assertIn("e2e_event_image_mode:=true", result.stdout)
        self.assertNotIn("unbound variable", result.stderr)

    def test_tui_offers_raw_event_tensor_separately_from_async_rgb_evs(self):
        source = (ROOT / "scripts/bringup.sh").read_text()
        self.assertIn("event_tensor  Raw EVS 20ch", source)
        self.assertIn("rgb_event_async  RGB + Raw EVS", source)
        self.assertIn("Raw EVS preprocess", source)
        self.assertIn("async   単一node非同期", source)

    def test_event_tensor_bringup_accepts_async_preprocessor(self):
        result = subprocess.run([
            "bash", str(ROOT / "scripts/bringup.sh"), "e2e", "--dry-run",
            "--e2e-model", str(self.root / "event_tensor_control"),
            "--set", "e2e_event_tensor_mode:=true",
            "--set", "e2e_event_preprocessor_mode:=async",
            "--set", "e2e_event_async_gpu_chunk_events:=16384",
            "--set", "e2e_event_cuda_events_per_transfer:=16384",
        ], text=True, capture_output=True, check=True)
        self.assertIn("e2e_event_preprocessor_mode:=async", result.stdout)
        self.assertIn("e2e_event_async_gpu_chunk_events:=16384", result.stdout)
        self.assertIn("e2e_event_cuda_events_per_transfer:=16384", result.stdout)
        self.assertIn("EVS preprocess: async", result.stdout)

    def test_async_rgb_evs_bringup_selects_cuda_pipeline(self):
        path = self.root / "rgb_event_async_control"
        record = inspect_model(path, "rgb-event", False, event_tensor_channels=20)
        self.assertEqual(record["event_representation"]["output_rate_hz"], 250.0)
        self.assertEqual(record["event_representation"]["backend"], "cuda")
        self.assertFalse(record["engine"])

        result = subprocess.run([
            "bash", str(ROOT / "scripts/bringup.sh"), "e2e",
            "--vehicle", "jpbb", "--sensor-kit", "realsense-silky", "--dry-run",
            "--e2e-model", str(path),
            "--set", "e2e_async_rgb_evs_mode:=true",
        ], text=True, capture_output=True, check=True)
        self.assertIn("e2e_async_rgb_evs_mode:=true", result.stdout)
        self.assertIn("e2e_async_event_output_rate_hz:=250.0", result.stdout)
        self.assertIn("e2e_event_representation_backend:=cuda", result.stdout)
        self.assertIn("sensor_kit_silky_evcam_event_image_enabled:=false", result.stdout)

    def test_tensorrt_build_requires_explicit_model_without_tty(self):
        result = subprocess.run(
            ["bash", str(ROOT / "scripts/e2e_trt.sh")],
            text=True,
            capture_output=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue(
            "E2E model root was not found" in result.stderr
            or "No deployed E2E ONNX model was found" in result.stderr
            or "interactive terminal is unavailable" in result.stderr
        )

    def test_async_rgb_evs_launch_preserves_state_feedback_contract(self):
        source = (
            ROOT
            / "ros2_ws/src/perception/jetpilot_e2e_inference/launch/async_rgb_evs_latent.launch.py"
        ).read_text()
        self.assertIn("jetpilot_e2e_inference::LatentStateManagerNode", source)
        self.assertIn("['state_in', 'event_tensor', 'delta_t']", source)
        self.assertIn("['state_out', 'control']", source)
        self.assertIn('(\"tensor_feedback\", LaunchConfiguration(\"event_feedback_topic\"))', source)
        self.assertIn('(\"updater_output\", LaunchConfiguration(\"updater_output_topic\"))', source)

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
