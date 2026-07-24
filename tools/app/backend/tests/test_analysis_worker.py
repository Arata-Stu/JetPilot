from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from jetpilot_console.analysis_worker import (
    AnalysisOptions,
    _file_fingerprint,
    _decode_image,
    _explicit_speed,
    _jetson_metric_samples,
    Progress,
    _snapshot_samples,
    _THERMAL_SMOOTHING_STATE,
    _transform_recorded_trajectory,
    extract_analysis,
    trajectory_map_consistency,
    write_demo_analysis,
)
from jetpilot_console.map_detail import directory_fingerprint


class SnapshotTrajectoryTests(unittest.TestCase):
    def test_extracts_plot_friendly_jetson_metrics(self) -> None:
        message = SimpleNamespace(
            status=[
                SimpleNamespace(
                    name="jetson_stats/cpu/0",
                    message=" 25.00%",
                    values=[
                        SimpleNamespace(key="Idle", value="75.00%"),
                        SimpleNamespace(key="Freq", value="729MHz"),
                    ],
                ),
                SimpleNamespace(
                    name="jetson_stats/cpu/1",
                    message=" 50.00%",
                    values=[SimpleNamespace(key="Freq", value="1.2GHz")],
                ),
                SimpleNamespace(
                    name="jetson_stats/gpu/gpu",
                    message=" 10.00%",
                    values=[SimpleNamespace(key="Used", value=" 10.00%")],
                ),
                SimpleNamespace(
                    name="jetson_stats/board/Config",
                    message="Jetpack 7.2 GA",
                    values=[SimpleNamespace(key="Jetpack", value="7.2 GA")],
                ),
                SimpleNamespace(
                    name="jetson_stats/board/Disk",
                    message="272G/914G",
                    values=[SimpleNamespace(key="Used", value="272G")],
                ),
            ]
        )

        samples = _jetson_metric_samples(message, 123)
        by_id = {sample["id"]: sample for sample in samples}

        self.assertEqual(by_id["cpu/all/used"]["value"], 37.5)
        self.assertEqual(by_id["cpu/1/Freq"]["value"], 1200.0)
        self.assertEqual(by_id["gpu/gpu/Used"]["unit"], "%")
        self.assertNotIn("board/Config/Jetpack", by_id)
        self.assertNotIn("board/Disk/Used", by_id)

    def test_invalid_speed_is_skipped_and_twist_is_supported(self) -> None:
        self.assertIsNone(_explicit_speed(SimpleNamespace(data="not-a-number")))
        twist = SimpleNamespace(
            linear=SimpleNamespace(x=3.0, y=4.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        )
        self.assertEqual(_explicit_speed(twist), 5.0)

    def test_flir_mono16_decodes_to_visible_stable_8bit_image(self) -> None:
        try:
            import cv2  # noqa: F401
            import numpy as np
        except ImportError:
            self.skipTest("OpenCV or NumPy is unavailable")

        _THERMAL_SMOOTHING_STATE.clear()
        values = np.array(
            [[10000, 12000, 14000, 16000], [18000, 20000, 22000, 24000]],
            dtype=np.uint16,
        )
        message = SimpleNamespace(
            encoding="mono16",
            height=2,
            width=4,
            step=8,
            is_bigendian=False,
            data=values.tobytes(),
        )

        decoded = _decode_image(message, "/flir/image_raw")

        self.assertEqual(decoded.shape, (2, 4, 3))
        self.assertGreater(int(decoded.max()), int(decoded.min()))
        self.assertGreater(int(decoded.mean()), 5)

    def test_reads_timed_odometry_samples(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "snapshot.json"
            path.write_text(
                json.dumps(
                    {
                        "localization": {"required": True, "confirmed": True},
                        "odometry_samples": [
                            {
                                "timestamp_ns": "1234567890123456789",
                                "frame_id": "map",
                                "child_frame_id": "base_link",
                                "pose": {
                                    "position": {"x": 1, "y": 2, "z": 0},
                                    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                                },
                                "twist": {
                                    "linear": {"x": 3, "y": 4, "z": 0},
                                    "angular": {"x": 0, "y": 0, "z": 0},
                                },
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )

            samples = _snapshot_samples(path)

            self.assertEqual(len(samples), 1)
            self.assertEqual(samples[0]["_timestamp_ns"], 1234567890123456789)
            self.assertEqual(samples[0]["frame_id"], "map")
            self.assertEqual(samples[0]["speed_mps"], 5.0)

    def test_uses_simulated_receive_time_when_header_stamp_is_zero(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "snapshot.json"
            path.write_text(
                json.dumps(
                    {
                        "localization": {"required": True, "confirmed": True},
                        "odometry_samples": [
                            {
                                "timestamp_ns": "0",
                                "received_timestamp_ns": "9876543210",
                                "frame_id": "map",
                                "pose": {
                                    "position": {"x": 0, "y": 0, "z": 0},
                                    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                                },
                                "twist": {"linear": {"x": 0, "y": 0, "z": 0}},
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )

            samples = _snapshot_samples(path)

            self.assertEqual(samples[0]["_timestamp_ns"], 9_876_543_210)
            self.assertIsNone(samples[0]["_header_timestamp_ns"])

    def test_prefers_simulated_receive_time_for_viewer_synchronization(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "snapshot.json"
            path.write_text(
                json.dumps(
                    {
                        "localization": {"required": True, "confirmed": True},
                        "odometry_samples": [
                            {
                                "timestamp_ns": "111",
                                "received_timestamp_ns": "222",
                                "frame_id": "map",
                                "pose": {
                                    "position": {"x": 0, "y": 0, "z": 0},
                                    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                                },
                                "twist": {"linear": {"x": 0, "y": 0, "z": 0}},
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            samples = _snapshot_samples(path)

            self.assertEqual(samples[0]["_timestamp_ns"], 222)
            self.assertEqual(samples[0]["_header_timestamp_ns"], 111)

    def test_rejects_unconfirmed_or_non_map_offline_results(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "snapshot.json"
            path.write_text(
                json.dumps(
                    {
                        "localization": {"required": True, "confirmed": False},
                        "odometry_samples": [],
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "did not reach"):
                _snapshot_samples(path)

            path.write_text(
                json.dumps(
                    {
                        "localization": {"required": True, "confirmed": True},
                        "odometry_samples": [
                            {
                                "timestamp_ns": "1",
                                "received_timestamp_ns": "1",
                                "frame_id": "odom",
                                "pose": {"position": {}, "orientation": {}},
                                "twist": {"linear": {}},
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "not map"):
                _snapshot_samples(path)

    def test_recorded_odom_uses_latest_map_transform_and_drops_early_pose(self) -> None:
        trajectory = [
            {"_timestamp_ns": 5, "frame_id": "odom", "x": 1.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            {"_timestamp_ns": 10, "frame_id": "odom", "x": 1.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        ]
        transforms = [
            {"_timestamp_ns": 10, "source_frame": "odom", "x": 10.0, "y": 20.0, "z": 0.0, "yaw": 3.141592653589793 / 2.0}
        ]

        result, transformed_count, dropped_count = _transform_recorded_trajectory(
            trajectory, transforms
        )

        self.assertEqual(transformed_count, 1)
        self.assertEqual(dropped_count, 1)
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0]["x"], 10.0)
        self.assertAlmostEqual(result[0]["y"], 21.0)
        self.assertEqual(result[0]["frame_id"], "map")

    def test_map_consistency_detects_aligned_and_mismatched_paths(self) -> None:
        geometry = {
            "resolution": 0.1,
            "origin": [0.0, 0.0, 0.0],
            "width": 100,
            "height": 100,
        }
        aligned = [
            {"x": float(index), "y": 1.0, "frame_id": "map"} for index in range(5)
        ]
        mismatch = [
            {"x": 100.0 + index, "y": 100.0, "frame_id": "map"}
            for index in range(5)
        ]

        self.assertEqual(trajectory_map_consistency(aligned, geometry)["status"], "pass")
        self.assertEqual(
            trajectory_map_consistency(mismatch, geometry)["status"], "mismatch"
        )


class DemoAnalysisTests(unittest.TestCase):
    def test_changed_map_is_rejected_before_rosbag_reader_starts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            bag = root / "bag"
            map_dir = root / "map"
            bag.mkdir()
            map_dir.mkdir()
            (map_dir / "map.bin").write_bytes(b"current")

            with self.assertRaisesRegex(RuntimeError, "changed after preflight"):
                extract_analysis(
                    AnalysisOptions(
                        rosbag=bag,
                        analysis_dir=root / "analysis",
                        image_topic="/camera",
                        map_dir=map_dir,
                        expected_map_fingerprint="stale",
                    )
                )

    def test_analysis_and_live_map_fingerprints_use_same_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_dir = Path(temporary_directory) / "map"
            map_dir.mkdir()
            (map_dir / "vslam_landmarks.yaml").write_text("resolution: 0.1\n")

            self.assertEqual(
                _file_fingerprint(map_dir), directory_fingerprint(map_dir)
            )

    def test_demo_writes_browser_ready_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "demo-analysis"
            manifest = write_demo_analysis(output)
            timeline = json.loads((output / "timeline.json").read_text(encoding="utf-8"))
            status = json.loads((output / "status.json").read_text(encoding="utf-8"))

            self.assertEqual(manifest["status"], "completed")
            self.assertEqual(status["status"], "completed")
            self.assertEqual(len(timeline["frames"]), 100)
            self.assertEqual(len(timeline["trajectory"]["samples"]), 100)
            self.assertTrue((output / timeline["frames"][0]["path"]).is_file())
            self.assertIsInstance(timeline["start_time_ns"], str)

    def test_demo_preserves_task_metadata_attached_before_worker_finishes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "demo-analysis"
            output.mkdir()
            (output / "manifest.json").write_text(
                json.dumps(
                    {
                        "analysis_id": "analysis-race",
                        "task_id": "task-race",
                        "created_at": "2026-01-01T00:00:00+00:00",
                    }
                ),
                encoding="utf-8",
            )

            manifest = write_demo_analysis(output)

            self.assertEqual(manifest["analysis_id"], "analysis-race")
            self.assertEqual(manifest["task_id"], "task-race")
            self.assertEqual(manifest["created_at"], "2026-01-01T00:00:00+00:00")

    def test_exit_trap_does_not_replace_existing_worker_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            status_path = Path(temporary_directory) / "status.json"
            progress = Progress(status_path)
            progress.update(
                "failed",
                1.0,
                "specific decoder failure",
                status="failed",
            )

            progress.update(
                "failed",
                1.0,
                "generic shell failure",
                status="failed",
                preserve_existing_failed=True,
            )

            status = json.loads(status_path.read_text(encoding="utf-8"))
            self.assertEqual(status["message"], "specific decoder failure")


if __name__ == "__main__":
    unittest.main()
