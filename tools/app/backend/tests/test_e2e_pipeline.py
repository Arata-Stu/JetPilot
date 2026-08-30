from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.e2e_pipeline import (
    build_deploy_task,
    build_export_task,
    build_preprocess_task,
    build_train_task,
    pipeline_catalog,
)


class E2EPipelineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.training = self.root / "python_ws" / "jetpilot_e2e_training"
        self.conf = self.training / "src" / "e2e_learning" / "conf"
        self.conf.mkdir(parents=True)
        self.record_root = self.root / "record"
        self.bag = self.record_root / "bag-a"
        self.bag.mkdir(parents=True)
        (self.bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  version: 8\n")
        self.config = SimpleNamespace(
            python_ws=self.root / "python_ws",
            record_root=self.record_root,
            python_bin="/opt/env/bin/python",
            jetson_user="tamiya",
        )
        self._write_deploy_config()

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def _write_deploy_config(self) -> None:
        (self.conf / "deploy_profiles.json").write_text(
            json.dumps(
                {
                    "default": "jetson",
                    "profiles": [
                        {
                            "id": "jetson",
                            "label": "Jetson",
                            "user": "tamiya",
                            "host": "10.42.0.1",
                            "remote_root": "/home/tamiya/JetPilot/ros2_ws/models/e2e",
                        }
                    ],
                }
            )
        )
        (self.conf / "deploy_model_presets.json").write_text(
            json.dumps(
                {
                    "default": "camera_control",
                    "presets": [
                        {"id": "camera_control", "label": "Camera Control"}
                    ],
                }
            )
        )

    def _dataset(self) -> Path:
        dataset = self.training / "datasets" / "dataset-a"
        dataset.mkdir(parents=True)
        (dataset / "samples.csv").write_text(
            "image_path,steering,throttle\nimages/000.jpg,0.1,0.2\n"
        )
        (dataset / "metadata.yaml").write_text(
            f"bag_path: {self.bag}\ninput_width: 212\ninput_height: 120\nsample_count: 1\n"
        )
        return dataset

    def _run(self, *, onnx: bool = False) -> Path:
        run = self.training / "outputs" / "e2e" / "run-a"
        (run / "checkpoints").mkdir(parents=True)
        (run / "checkpoints" / "best.pt").write_bytes(b"checkpoint")
        (run / "run.yaml").write_text(
            "model:\n"
            "  name: pilotnet\n"
            "data:\n"
            "  dataset_dir: /datasets/dataset-a\n"
            "  input_width: 212\n"
            "  input_height: 120\n"
        )
        if onnx:
            (run / "model.onnx").write_bytes(b"onnx")
            (run / "metadata.json").write_text("{}")
        return run

    def test_preprocess_task_uses_selected_bag_topics_and_safe_output(self) -> None:
        spec = build_preprocess_task(
            self.config,
            {
                "rosbag": str(self.bag),
                "dataset_name": "dataset-a",
                "image_topic": "/camera/image",
                "control_topic": "/teleop/control_cmd",
                "input_width": 212,
                "input_height": 120,
            },
        )

        self.assertEqual(spec.kind, "e2e-preprocess")
        self.assertIn(f"data.bag_path={self.bag.resolve()}", spec.command)
        self.assertIn("data.image_topic=/camera/image", spec.command)
        self.assertIn("data.imu_topic=/realsense/imu", spec.command)
        self.assertTrue(spec.artifacts[0]["path"].endswith("datasets/dataset-a"))

        with self.assertRaises(ValueError):
            build_preprocess_task(
                self.config,
                {
                    "rosbag": str(self.bag),
                    "dataset_name": "../escape",
                    "image_topic": "/camera/image",
                    "control_topic": "/teleop/control_cmd",
                },
            )

    def test_train_and_export_tasks_preserve_selected_configuration(self) -> None:
        dataset = self._dataset()
        train = build_train_task(
            self.config,
            {
                "dataset_dir": str(dataset),
                "run_name": "run-a",
                "experiment": "mobilenet_head_then_finetune",
                "epochs": 12,
                "learning_rate": 0.002,
                "finetune_epochs": 4,
                "finetune_learning_rate": 0.0002,
            },
        )
        self.assertEqual(train.kind, "e2e-train")
        self.assertIn("experiment=mobilenet_head_then_finetune", train.command)
        self.assertIn("train.stages.0.epochs=12", train.command)
        self.assertIn("train.stages.1.epochs=4", train.command)

        run = self._run()
        export = build_export_task(self.config, {"run_dir": str(run)})
        self.assertEqual(export.kind, "e2e-export-onnx")
        self.assertIn(f"checkpoint={run.resolve() / 'checkpoints/best.pt'}", export.command)

    def test_trajectory_training_uses_dataset_geometry(self) -> None:
        dataset = self.training / "datasets" / "trajectory-a"
        dataset.mkdir(parents=True)
        (dataset / "samples.csv").write_text(
            "image_path,trajectory,imu\nimages/000.jpg,[],[]\n"
        )
        (dataset / "metadata.yaml").write_text(
            "task: trajectory\n"
            "input_width: 212\n"
            "input_height: 120\n"
            "trajectory_horizon_sec: 2.0\n"
            "trajectory_points: 12\n"
            "trajectory_scale_m: 7.5\n"
            "imu_window_sec: 0.8\n"
            "imu_samples: 16\n"
        )

        train = build_train_task(
            self.config,
            {
                "dataset_dir": str(dataset),
                "run_name": "trajectory-run",
                "experiment": "trajectory_pilotnet_gru_imu",
            },
        )

        self.assertIn("data.trajectory_points=12", train.command)
        self.assertIn("model.trajectory_points=12", train.command)
        self.assertIn("model.trajectory_scale_m=7.5", train.command)
        self.assertIn("model.imu_samples=16", train.command)

    def test_catalog_and_deploy_task_only_accept_exported_runs(self) -> None:
        dataset = self._dataset()
        run = self._run(onnx=True)
        catalog = pipeline_catalog(self.config)

        self.assertEqual(catalog["datasets"][0]["path"], str(dataset.resolve()))
        self.assertEqual(catalog["runs"][0]["onnx_path"], str(run.resolve() / "model.onnx"))

        deploy = build_deploy_task(
            self.config,
            {
                "model_path": str(run / "model.onnx"),
                "profile": "jetson",
                "preset": "camera_control",
                "build_engine": True,
            },
        )
        self.assertEqual(deploy.kind, "e2e-deploy")
        self.assertIn("--build-engine", deploy.command)
        self.assertIn("10.42.0.1", deploy.command)

        outside = self.root / "outside.onnx"
        outside.write_bytes(b"onnx")
        with self.assertRaises(ValueError):
            build_deploy_task(self.config, {"model_path": str(outside)})


if __name__ == "__main__":
    unittest.main()
