from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.shared_vit_pipeline import (
    build_deploy_task,
    build_detection_train_task,
    build_export_task,
    pipeline_snapshot,
)


class SharedVitPipelineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.python_ws = self.root / "python_ws"
        self.e2e = self.python_ws / "jetpilot_e2e_training"
        self.detection = self.python_ws / "jetpilot_object_detection_training"
        self.config = SimpleNamespace(
            python_ws=self.python_ws,
            python_bin="/opt/env/bin/python",
            jetson_user="tamiya",
        )
        config_dir = self.e2e / "src" / "e2e_learning" / "conf"
        config_dir.mkdir(parents=True)
        (config_dir / "deploy_profiles.json").write_text(json.dumps({
            "default": "jetson",
            "profiles": [{
                "id": "jetson", "label": "Jetson", "user": "tamiya",
                "host": "10.42.0.1",
                "remote_root": "/home/tamiya/JetPilot/ros2_ws/models/e2e",
            }],
        }), encoding="utf-8")
        self.backbone = self.e2e / "weights" / "dinov3" / "dinov3_vits16.pth"
        self.backbone.parent.mkdir(parents=True)
        self.backbone.write_bytes(b"weights")
        self.dataset_yaml = self._make_dataset()
        self.control_run = self._make_control_run()
        self.detection_run = self._make_detection_run()

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def _make_dataset(self) -> Path:
        dataset = self.detection / "datasets" / "track"
        for split in ("train", "valid"):
            (dataset / split / "images").mkdir(parents=True)
            (dataset / split / "labels").mkdir(parents=True)
            (dataset / split / "images" / "sample.jpg").write_bytes(b"image")
            (dataset / split / "labels" / "sample.txt").write_text(
                "0 0.5 0.5 0.2 0.2\n", encoding="utf-8"
            )
        path = dataset / "data.yaml"
        path.write_text(
            "path: .\ntrain: train/images\nval: valid/images\nnames: [vehicle, barrier]\n",
            encoding="utf-8",
        )
        return path

    def _make_control_run(self) -> Path:
        run = self.e2e / "outputs" / "e2e" / "vit-control"
        (run / "checkpoints").mkdir(parents=True)
        (run / "checkpoints" / "best.pt").write_bytes(b"control")
        (run / "run.yaml").write_text(
            "model:\n  name: dinov3_vits16\n"
            "data:\n  input_width: 212\n  input_height: 120\n  image_topic: /realsense/color/image_raw\n",
            encoding="utf-8",
        )
        return run

    def _make_detection_run(self) -> Path:
        run = self.detection / "outputs" / "shared_vit_detection" / "vit-detection"
        (run / "weights").mkdir(parents=True)
        (run / "weights" / "best.pt").write_bytes(b"detection")
        (run / "jetpilot_training_manifest.json").write_text(json.dumps({
            "modality": "rgb", "classes": ["vehicle", "barrier"],
            "mean": [0.485, 0.456, 0.406], "std": [0.229, 0.224, 0.225],
            "backbone_weights": str(self.backbone), "backbone_fingerprint": "abc123",
        }), encoding="utf-8")
        return run

    def test_snapshot_lists_shared_assets(self) -> None:
        snapshot = pipeline_snapshot(self.config)
        self.assertEqual(snapshot["weights"][0]["modality"], "rgb")
        self.assertEqual(snapshot["control_runs"][0]["name"], "vit-control")
        self.assertEqual(snapshot["detection_runs"][0]["backbone_fingerprint"], "abc123")
        self.assertIn("shared_vit", snapshot["defaults"]["remote_root"])

    def test_detection_training_uses_both_python_sources(self) -> None:
        spec = build_detection_train_task(self.config, {
            "dataset_yaml": str(self.dataset_yaml),
            "backbone_weights": str(self.backbone),
            "run_name": "vit-det-new",
            "modality": "rgb",
            "mean": "0.485,0.456,0.406",
            "std": "0.229,0.224,0.225",
        })
        self.assertEqual(spec.kind, "shared-vit-detection-train")
        self.assertIn("object_detection_learning.cli.train_shared_vit_head", spec.command)
        self.assertIn(str(self.e2e / "src"), spec.command[1])
        self.assertIn("--input-width", spec.command)
        self.assertIn("212", spec.command)

    def test_export_and_deploy_use_shared_artifact_root(self) -> None:
        export = build_export_task(self.config, {
            "control_checkpoint": str(self.control_run / "checkpoints" / "best.pt"),
            "detection_checkpoint": str(self.detection_run / "weights" / "best.pt"),
            "model_name": "vit-shared",
        })
        self.assertEqual(export.kind, "shared-vit-export")
        self.assertIn("e2e_learning.cli.export_multitask_onnx", export.command)

        model = self.e2e / "outputs" / "shared_vit" / "vit-shared"
        model.mkdir(parents=True)
        (model / "model.onnx").write_bytes(b"onnx")
        deploy = build_deploy_task(self.config, {
            "model_path": str(model / "model.onnx"),
            "profile": "jetson",
            "remote_root": "/home/tamiya/JetPilot/ros2_ws/models/e2e/shared_vit",
            "deploy_name": "vit-shared",
            "build_engine": True,
        })
        self.assertEqual(deploy.kind, "shared-vit-deploy")
        self.assertIn("--build-engine", deploy.command)
        self.assertIn("deploy_shared_vit_model.sh", " ".join(deploy.command))


if __name__ == "__main__":
    unittest.main()
