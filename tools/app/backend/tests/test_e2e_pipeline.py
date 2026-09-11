from __future__ import annotations

import json
import tempfile
import unittest
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.e2e_pipeline import (
    build_deploy_task,
    build_export_task,
    build_preprocess_task,
    build_train_task,
    pipeline_catalog,
    suggest_dataset_name,
    suggest_run_name,
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
        self.assertIn("data.timestamp_source=bag", spec.command)
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

    def test_suggested_names_include_time_architecture_and_target(self) -> None:
        now = datetime(2026, 9, 10, 18, 0)

        self.assertEqual(suggest_dataset_name(now), "e2e_dataset_0910-1800")
        self.assertEqual(
            suggest_run_name("pilotnet_steering", now),
            "cnn-pilotnet-steer-only_0910-1800",
        )
        self.assertEqual(
            suggest_run_name("dinov3_vits16_scratch", now),
            "vit-dinov3-vits16-control-scratch_0910-1800",
        )
        self.assertEqual(
            suggest_run_name("dinov3_vits16_scratch", now, "steer"),
            "vit-dinov3-vits16-steer-only-scratch_0910-1800",
        )
        self.assertEqual(
            suggest_run_name("wam_dinov3_vits16_frozen", now),
            "wam-dinov3-vits16-gru-control-frozen_0910-1800",
        )

    def test_preprocess_timestamp_source_override_and_validation(self) -> None:
        body = {"rosbag": str(self.bag), "dataset_name": "clock-test", "image_topic": "/event_camera/event_image"}
        spec = build_preprocess_task(self.config, {**body, "timestamp_source": "header"})
        self.assertIn("data.timestamp_source=header", spec.command)
        with self.assertRaisesRegex(ValueError, "timestamp_source"):
            build_preprocess_task(self.config, {**body, "timestamp_source": "unknown"})

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

    def test_dinov3_vits16_training_uses_cnn_input_geometry(self) -> None:
        dataset = self._dataset()
        train = build_train_task(
            self.config,
            {
                "dataset_dir": str(dataset),
                "run_name": "vit-run",
                "experiment": "dinov3_vits16_scratch",
            },
        )

        self.assertIn("data.input_width=212", train.command)
        self.assertIn("data.input_height=120", train.command)

    def test_wam_is_available_as_an_independent_control_model(self) -> None:
        dataset = self._dataset()
        catalog = pipeline_catalog(self.config)
        experiments = {item["id"]: item for item in catalog["experiments"]}

        self.assertEqual(experiments["wam_dinov3_vits16_frozen"]["family"], "wam")
        train = build_train_task(
            self.config,
            {
                "dataset_dir": str(dataset),
                "run_name": "wam-run",
                "experiment": "wam_dinov3_vits16_frozen",
                "wam_context_length": 5,
                "wam_future_horizon": 8,
                "wam_future_stride": 2,
            },
        )
        self.assertIn("experiment=wam_dinov3_vits16_frozen", train.command)
        self.assertIn("model.steering_only=false", train.command)
        self.assertIn("model.sequence_length=5", train.command)
        self.assertIn("model.future_horizon=8", train.command)
        self.assertIn("model.future_stride=2", train.command)

    def test_vit_and_mobilenet_support_control_or_steer_only_output(self) -> None:
        dataset = self._dataset()
        for index, experiment in enumerate((
            "dinov3_vits16_frozen_head",
            "mobilenet_frozen_head",
        )):
            steer = build_train_task(
                self.config,
                {
                    "dataset_dir": str(dataset),
                    "run_name": f"steer-{index}",
                    "experiment": experiment,
                    "output_target": "steer",
                },
            )
            self.assertIn("model.steering_only=true", steer.command)

            control = build_train_task(
                self.config,
                {
                    "dataset_dir": str(dataset),
                    "run_name": f"control-{index}",
                    "experiment": experiment,
                    "output_target": "control",
                },
            )
            self.assertIn("model.steering_only=false", control.command)

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

    def test_event_steering_deploy_is_selected_and_rgb_destination_rejected(self) -> None:
        run = self._run(onnx=True)
        (run / "metadata.json").write_text(json.dumps({
            "task": "control", "steering_only": True,
            "image_topic": "/event_camera/event_image", "modality": "event_image",
        }))
        presets_path = self.conf / "deploy_model_presets.json"
        presets = json.loads(presets_path.read_text())
        presets["presets"].append({"id": "event_steering", "model_name": "event_steering"})
        presets_path.write_text(json.dumps(presets))
        spec = build_deploy_task(self.config, {"model_path": str(run / "model.onnx")})
        self.assertIn("event_steering", spec.command)
        with self.assertRaisesRegex(ValueError, "does not match"):
            build_deploy_task(self.config, {"model_path": str(run / "model.onnx"), "preset": "camera_control"})

    def test_catalog_and_deploy_task_only_accept_exported_runs(self) -> None:
        dataset = self._dataset()
        run = self._run(onnx=True)
        catalog = pipeline_catalog(self.config)

        self.assertEqual(catalog["datasets"][0]["path"], str(dataset.resolve()))
        self.assertEqual(catalog["runs"][0]["onnx_path"], str(run.resolve() / "model.onnx"))
        experiment_ids = {item["id"] for item in catalog["experiments"]}
        self.assertNotIn("pilotnet_steering", experiment_ids)
        mobilenet = next(
            item for item in catalog["experiments"]
            if item["id"] == "mobilenet_frozen_head"
        )
        self.assertEqual(mobilenet["output_targets"], ["control", "steer"])
        self.assertIn("steer", mobilenet["suggested_run_names"])

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
        self.assertEqual(deploy.command[deploy.command.index("--name") + 1], "run-a")
        self.assertIn("10.42.0.1", deploy.command)

        outside = self.root / "outside.onnx"
        outside.write_bytes(b"onnx")
        with self.assertRaises(ValueError):
            build_deploy_task(self.config, {"model_path": str(outside)})

    def test_stateful_wam_requires_runtime_adapter_before_deploy(self) -> None:
        run = self._run(onnx=True)
        (run / "metadata.json").write_text(json.dumps({
            "task": "control",
            "architecture": {"stateful_step": True},
        }))

        with self.assertRaisesRegex(ValueError, "recurrent-state adapter"):
            build_deploy_task(self.config, {"model_path": str(run / "model.onnx")})


if __name__ == "__main__":
    unittest.main()
