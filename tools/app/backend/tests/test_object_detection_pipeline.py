from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.object_detection_pipeline import (
    build_deploy_task,
    build_export_task,
    build_train_task,
    build_validate_dataset_task,
    pipeline_snapshot,
    scan_datasets,
    scan_runs,
)


class ObjectDetectionPipelineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.python_ws = self.root / "python_ws"
        self.training = self.python_ws / "jetpilot_object_detection_training"
        (self.training / "datasets").mkdir(parents=True)
        (self.training / "outputs/yolov8").mkdir(parents=True)
        scripts = self.training / "scripts"
        scripts.mkdir(parents=True)
        deploy_script = scripts / "deploy_model.sh"
        deploy_script.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
        deploy_script.chmod(0o755)

        profile_directory = (
            self.python_ws
            / "jetpilot_e2e_training/src/e2e_learning/conf"
        )
        profile_directory.mkdir(parents=True)
        (profile_directory / "deploy_profiles.json").write_text(
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
                        },
                        {
                            "id": "manual",
                            "label": "Manual",
                            "user": "tamiya",
                            "host": "__manual__",
                            "remote_root": "/home/tamiya/JetPilot/ros2_ws/models/e2e",
                        },
                    ],
                }
            ),
            encoding="utf-8",
        )
        self.config = SimpleNamespace(
            python_ws=self.python_ws,
            ros2_ws=self.root / "ros2_ws",
            python_bin="/opt/env/bin/python",
            jetson_user="tamiya",
        )

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def _dataset(self, name: str = "objects-v1") -> Path:
        directory = self.training / "datasets" / name
        for split in ("train", "valid", "test"):
            images = directory / split / "images"
            labels = directory / split / "labels"
            images.mkdir(parents=True)
            labels.mkdir(parents=True)
            (images / "frame.jpg").write_bytes(b"jpeg")
            (labels / "frame.txt").write_text(
                "0 0.5 0.5 0.2 0.2\n1 0.2 0.2 0.1 0.1\n",
                encoding="utf-8",
            )
        data = directory / "data.yaml"
        data.write_text(
            "path: .\n"
            "train: train/images\n"
            "val: valid/images\n"
            "test: test/images\n"
            "names: [vehicle, barrier]\n"
            "nc: 2\n",
            encoding="utf-8",
        )
        return data

    def _run(self, dataset: Path, *, exported: bool = True) -> Path:
        run = self.training / "outputs/yolov8" / "run-a"
        weights = run / "weights"
        weights.mkdir(parents=True)
        (weights / "best.pt").write_bytes(b"best")
        (weights / "last.pt").write_bytes(b"last")
        (run / "args.yaml").write_text(
            f"data: {dataset}\nepochs: 100\nimgsz: 224\nmodel: yolov8n.pt\n",
            encoding="utf-8",
        )
        (run / "results.csv").write_text(
            "epoch,metrics/precision(B),metrics/recall(B),metrics/mAP50(B),metrics/mAP50-95(B),train/box_loss,val/box_loss\n"
            "4,0.8,0.7,0.75,0.5,0.4,0.45\n",
            encoding="utf-8",
        )
        (run / "jetpilot_training_manifest.json").write_text(
            json.dumps(
                {
                    "mode": "train",
                    "dataset_yaml": str(dataset),
                    "classes": ["vehicle", "barrier"],
                    "initial_model": "yolov8n.pt",
                }
            ),
            encoding="utf-8",
        )
        if exported:
            export = run / "export"
            export.mkdir()
            (export / "model.onnx").write_bytes(b"onnx")
            (export / "metadata.json").write_text(
                json.dumps({"classes": ["vehicle", "barrier"]}),
                encoding="utf-8",
            )
        return run

    def test_snapshot_scans_dataset_contract_counts_and_profiles(self) -> None:
        dataset = self._dataset()

        snapshot = pipeline_snapshot(self.config)

        self.assertEqual(snapshot["classes"], ["vehicle", "barrier"])
        self.assertEqual(snapshot["input_size"], [224, 224])
        self.assertEqual(snapshot["default_base_model"], "yolov8n.pt")
        self.assertEqual(snapshot["datasets"][0]["path"], str(dataset.resolve()))
        self.assertTrue(snapshot["datasets"][0]["valid"])
        self.assertEqual(snapshot["datasets"][0]["image_count"], 3)
        self.assertEqual(snapshot["datasets"][0]["annotation_count"], 6)
        self.assertEqual(snapshot["default_deploy_profile"], "jetson")
        self.assertTrue(
            snapshot["deploy_profiles"][0]["remote_root"].endswith(
                "/models/yolov8"
            )
        )

    def test_dataset_scan_reports_class_contract_and_validate_task_is_safe(self) -> None:
        valid = self._dataset()
        invalid = self._dataset("wrong-classes")
        invalid.write_text(
            invalid.read_text(encoding="utf-8").replace(
                "[vehicle, barrier]", "[barrier, vehicle]"
            ),
            encoding="utf-8",
        )

        datasets = scan_datasets(self.config)
        records = {record["name"]: record for record in datasets}
        self.assertTrue(records["objects-v1"]["valid"])
        self.assertFalse(records["wrong-classes"]["valid"])

        validate = build_validate_dataset_task(
            self.config, {"dataset_yaml": str(valid)}
        )
        self.assertEqual(validate.kind, "object-detection-validate-dataset")
        self.assertIn(
            "object_detection_learning.cli.validate_dataset", validate.command
        )
        self.assertIn(str(valid.resolve()), validate.command)

        outside = self.root / "data.yaml"
        outside.write_text("names: [vehicle, barrier]\n", encoding="utf-8")
        with self.assertRaises(ValueError):
            build_validate_dataset_task(
                self.config, {"dataset_yaml": str(outside)}
            )

    def test_train_supports_new_fine_tune_and_resume_modes(self) -> None:
        dataset = self._dataset()
        new = build_train_task(
            self.config,
            {
                "dataset_yaml": str(dataset),
                "mode": "train",
                "run_name": "new-run",
                "base_model": "yolov8n.pt",
                "epochs": 12,
                "batch": 8,
                "device": "mps",
            },
        )
        self.assertEqual(new.kind, "object-detection-train")
        self.assertIn("--epochs", new.command)
        self.assertIn("12", new.command)
        self.assertIn("mps", new.command)
        self.assertIn("ml-device:mps", new.resource_keys)

        run = self._run(dataset, exported=False)
        fine_tune = build_train_task(
            self.config,
            {
                "dataset_yaml": str(dataset),
                "mode": "fine_tune",
                "run_name": "fine-tuned",
                "checkpoint": str(run / "weights/best.pt"),
            },
        )
        self.assertIn(str((run / "weights/best.pt").resolve()), fine_tune.command)
        self.assertNotIn("--resume", fine_tune.command)
        self.assertIn(
            f"object-detection-run:{run.resolve()}", fine_tune.resource_keys
        )
        self.assertTrue(
            any(
                artifact["name"] == "training manifest"
                for artifact in fine_tune.artifacts
            )
        )

        resume = build_train_task(
            self.config,
            {
                "dataset_yaml": str(dataset),
                "mode": "resume",
                "checkpoint": str(run / "weights/last.pt"),
            },
        )
        self.assertIn("--resume", resume.command)
        self.assertEqual(resume.artifacts[0]["path"], str(run.resolve()))

        with self.assertRaises(ValueError):
            build_train_task(
                self.config,
                {
                    "dataset_yaml": str(dataset),
                    "mode": "resume",
                    "checkpoint": str(run / "weights/best.pt"),
                },
            )

        with self.assertRaisesRegex(ValueError, "batch must be an integer"):
            build_train_task(
                self.config,
                {
                    "dataset_yaml": str(dataset),
                    "mode": "train",
                    "run_name": "boolean-batch",
                    "batch": True,
                },
            )

    def test_run_scan_export_and_deploy_only_use_registered_artifacts(self) -> None:
        dataset = self._dataset()
        run = self._run(dataset)

        runs = scan_runs(self.config)
        self.assertEqual(runs[0]["status"], "exported")
        self.assertEqual(runs[0]["metrics"]["map50"], 0.75)
        self.assertEqual(runs[0]["progress"]["epoch"], 4)

        export = build_export_task(
            self.config,
            {"run_dir": str(run), "dataset_yaml": str(dataset), "opset": 17},
        )
        self.assertEqual(export.kind, "object-detection-export-onnx")
        self.assertIn(str((run / "weights/best.pt").resolve()), export.command)
        self.assertIn(str((run / "export").resolve()), export.command)

        deploy = build_deploy_task(
            self.config,
            {
                "model_path": str(run / "export/model.onnx"),
                "profile": "jetson",
                "model_name": "objects-v1",
                "build_engine": True,
            },
        )
        self.assertEqual(deploy.kind, "object-detection-deploy")
        self.assertIn("--user", deploy.command)
        self.assertIn("--host", deploy.command)
        self.assertIn("10.42.0.1", deploy.command)
        self.assertIn("--build-engine", deploy.command)
        self.assertIn("/models/yolov8", " ".join(deploy.command))
        self.assertIn("jetson-trtexec:tamiya@10.42.0.1", deploy.resource_keys)

        manual = build_deploy_task(
            self.config,
            {
                "model_path": str(run / "export/model.onnx"),
                "profile": "manual",
                "host": "192.168.55.1",
            },
        )
        self.assertIn("192.168.55.1", manual.command)

        outside = self.root / "outside.onnx"
        outside.write_bytes(b"onnx")
        with self.assertRaises(ValueError):
            build_deploy_task(
                self.config, {"model_path": str(outside), "profile": "jetson"}
            )
        with self.assertRaisesRegex(ValueError, "too broad"):
            build_deploy_task(
                self.config,
                {
                    "model_path": str(run / "export/model.onnx"),
                    "profile": "jetson",
                    "remote_root": "/",
                },
            )


if __name__ == "__main__":
    unittest.main()
