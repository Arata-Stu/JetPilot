from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from jetpilot_console.preflight import BLOCKED, PASS, evaluate_preflight


CAMERA_TOPICS = (
    "/realsense/infra1/image_rect_raw",
    "/realsense/infra1/camera_info",
    "/realsense/infra2/image_rect_raw",
    "/realsense/infra2/camera_info",
    "/tf_static",
)


class PreflightTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        root = Path(self.temporary_directory.name)
        self.record_root = root / "record"
        self.map_root = root / "map"
        self.ros2_ws = root / "ros2_ws"
        self.record_root.mkdir()
        self.map_root.mkdir()
        self.config = SimpleNamespace(
            record_root=self.record_root,
            map_root=self.map_root,
            ros2_ws=self.ros2_ws,
        )
        self.model_dir = (
            self.ros2_ws / "isaac_ros_assets/models/visual_global_localization"
        )
        self.model_dir.mkdir(parents=True)
        (self.model_dir / "model.plan").write_bytes(b"model")

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def camera_config(self) -> Path:
        path = (
            self.ros2_ws
            / "src/launch/jetpilot_system_launch/config/localization/vgl_camera_topics.yaml"
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            """stereo_cameras:
  - name: realsense_front
    left: /realsense/infra1/image_rect_raw
    left_camera_info: /realsense/infra1/camera_info
    right: /realsense/infra2/image_rect_raw
    right_camera_info: /realsense/infra2/camera_info
""",
            encoding="utf-8",
        )
        return path

    def bag(
        self,
        *,
        topics: tuple[str, ...] = CAMERA_TOPICS,
        storage: bool = True,
        reliability: str | None = "reliable",
    ) -> Path:
        bag = self.record_root / "run_001"
        bag.mkdir(exist_ok=True)
        if storage:
            (bag / "run_001_0.mcap").write_bytes(b"bag-data")
        topic_lines = []
        for topic in topics:
            message_type = "tf2_msgs/msg/TFMessage" if topic == "/tf_static" else (
                "sensor_msgs/msg/CameraInfo" if topic.endswith("camera_info") else "sensor_msgs/msg/Image"
            )
            topic_lines.extend(
                [
                    "    - topic_metadata:",
                    f"        name: {topic}",
                    f"        type: {message_type}",
                    "        serialization_format: cdr",
                ]
            )
            if reliability:
                topic_lines.extend(
                    [
                        "        offered_qos_profiles: |",
                        "          - history: keep_last",
                        "            depth: 5",
                        f"            reliability: {reliability}",
                        "            durability: volatile",
                    ]
                )
            topic_lines.append("      message_count: 10")
        (bag / "metadata.yaml").write_text(
            "\n".join(
                [
                    "rosbag2_bagfile_information:",
                    "  storage_identifier: mcap",
                    "  relative_file_paths:",
                    "    - run_001_0.mcap",
                    "  topics_with_message_count:",
                    *topic_lines,
                    "",
                ]
            ),
            encoding="utf-8",
        )
        return bag

    def map_dir(self, name: str = "course_a") -> Path:
        path = self.map_root / name
        path.mkdir(exist_ok=True)
        return path

    def ready_map(self) -> Path:
        path = self.map_dir()
        name = path.name
        snapshot = {
            "landmarks": {
                # Twelve zero bytes: one complete point for point_step=12.
                "data": "AAAAAAAAAAAAAAAA",
                "point_step": 12,
                "fields": [
                    {"name": "x", "offset": 0},
                    {"name": "y", "offset": 4},
                    {"name": "z", "offset": 8},
                ],
            }
        }
        (path / "vslam_reference_snapshot.json").write_text(json.dumps(snapshot))
        (path / "vslam_landmarks.png").write_bytes(b"not-empty-test-image")
        (path / "vslam_landmarks.yaml").write_text(
            'image: "vslam_landmarks.png"\n'
            "resolution: 0.05\n"
            "origin: [0.0, 0.0, 0.0]\n",
            encoding="utf-8",
        )
        points = "\n".join(f"      - [{index * 0.1}, 0.0, 0.0]" for index in range(8))
        left = "\n".join(f"      - [{index * 0.1}, 0.3, 0.0]" for index in range(8))
        right = "\n".join(f"      - [{index * 0.1}, -0.3, 0.0]" for index in range(8))
        (path / f"{name}_hd_map.yaml").write_text(
            f"""format: jetpilot_hd_map_v1
primary_lane_id: lane_main
lanes:
  - id: lane_main
    closed_loop: true
    left_bound:
{left}
    right_bound:
{right}
    centerline:
{points}
""",
            encoding="utf-8",
        )
        (path / f"{name}_hd_map_centerline.csv").write_text(
            "# x,y,w_right,w_left\n"
            + "".join(f"{index * 0.1},0.0,0.3,0.3\n" for index in range(8)),
            encoding="utf-8",
        )
        (path / f"{name}_raceline.csv").write_text(
            "# s;x;y;psi;kappa;vx;ax\n"
            + "".join(f"{index * 0.1};{index * 0.1};0.0;0;0;1;0\n" for index in range(8)),
            encoding="utf-8",
        )
        return path

    @staticmethod
    def check(result: dict[str, object], check_id: str) -> dict[str, object]:
        return next(check for check in result["checks"] if check["id"] == check_id)

    def test_map_build_is_ready_and_json_serializable(self) -> None:
        bag = self.bag()
        self.camera_config()
        result = evaluate_preflight(
            self.config,
            "map-build",
            {"rosbag": str(bag), "map_dir": str(self.map_root / "new_map")},
        )

        self.assertTrue(result["ready"])
        self.assertEqual(result["status"], PASS)
        self.assertEqual(self.check(result, "rosbag.mapping_topics")["status"], PASS)
        self.assertEqual(self.check(result, "rosbag.mapping_qos")["status"], PASS)
        self.assertEqual(result["resolved"]["rosbag"], str(bag.resolve()))
        json.dumps(result)

    def test_map_build_blocks_incomplete_or_unsafe_bag(self) -> None:
        self.camera_config()
        bag = self.bag(storage=False)
        result = evaluate_preflight(
            self.config,
            "map-build",
            {"rosbag": str(bag), "map_dir": str(self.map_root / "new_map")},
        )
        self.assertFalse(result["ready"])
        self.assertEqual(self.check(result, "rosbag.storage")["status"], BLOCKED)

        outside = Path(self.temporary_directory.name).parent
        result = evaluate_preflight(
            self.config,
            "map-build",
            {"rosbag": str(outside), "map_dir": str(self.map_root / "new_map")},
        )
        self.assertEqual(self.check(result, "rosbag.path")["status"], BLOCKED)

    def test_map_build_blocks_missing_required_camera_topic(self) -> None:
        self.camera_config()
        bag = self.bag(topics=CAMERA_TOPICS[:-1])
        result = evaluate_preflight(
            self.config,
            "map-build",
            {"rosbag": str(bag), "map_dir": str(self.map_root / "new_map")},
        )

        topic_check = self.check(result, "rosbag.mapping_topics")
        self.assertEqual(topic_check["status"], BLOCKED)
        self.assertEqual(topic_check["details"]["missing"][0]["topic"], "/tf_static")

    def test_map_build_warns_for_best_effort_camera_qos(self) -> None:
        self.camera_config()
        bag = self.bag(reliability="best_effort")
        result = evaluate_preflight(
            self.config,
            "map-build",
            {"rosbag": str(bag), "map_dir": str(self.map_root / "new_map")},
        )

        qos_check = self.check(result, "rosbag.mapping_qos")
        self.assertTrue(result["ready"])
        self.assertEqual(qos_check["status"], "warning")
        self.assertEqual(
            qos_check["details"]["best_effort"][0]["topic"],
            "/realsense/infra1/image_rect_raw",
        )

    def test_map_build_warns_when_camera_qos_metadata_is_missing(self) -> None:
        self.camera_config()
        bag = self.bag(reliability=None)
        result = evaluate_preflight(
            self.config,
            "map-build",
            {"rosbag": str(bag), "map_dir": str(self.map_root / "new_map")},
        )

        qos_check = self.check(result, "rosbag.mapping_qos")
        self.assertTrue(result["ready"])
        self.assertEqual(qos_check["status"], "warning")
        self.assertEqual(
            qos_check["details"]["unavailable"][0]["topic"],
            "/realsense/infra1/image_rect_raw",
        )

    def test_map_build_blocks_required_topic_with_wrong_message_type(self) -> None:
        self.camera_config()
        bag = self.bag()
        metadata = bag / "metadata.yaml"
        metadata.write_text(
            metadata.read_text().replace(
                "name: /tf_static\n        type: tf2_msgs/msg/TFMessage",
                "name: /tf_static\n        type: std_msgs/msg/String",
            )
        )

        result = evaluate_preflight(
            self.config,
            "map-build",
            {"rosbag": str(bag), "map_dir": str(self.map_root / "new_map")},
        )

        topic_check = self.check(result, "rosbag.mapping_topics")
        self.assertEqual(topic_check["status"], BLOCKED)
        self.assertEqual(topic_check["details"]["wrong_type"][0]["topic"], "/tf_static")

    def test_map_build_blocks_empty_topic_metadata(self) -> None:
        self.camera_config()
        bag = self.bag(topics=())

        result = evaluate_preflight(
            self.config,
            "map-build",
            {"rosbag": str(bag), "map_dir": str(self.map_root / "new_map")},
        )

        self.assertFalse(result["ready"])
        self.assertEqual(self.check(result, "rosbag.metadata")["status"], BLOCKED)
        self.assertEqual(self.check(result, "rosbag.mapping_topics")["status"], BLOCKED)

    def test_map_build_blocks_missing_or_invalid_required_message_count(self) -> None:
        self.camera_config()
        bag = self.bag()
        metadata = bag / "metadata.yaml"
        original = metadata.read_text()
        topic_block = (
            "        name: /tf_static\n"
            "        type: tf2_msgs/msg/TFMessage\n"
            "        serialization_format: cdr\n"
            "        offered_qos_profiles: |\n"
            "          - history: keep_last\n"
            "            depth: 5\n"
            "            reliability: reliable\n"
            "            durability: volatile\n"
            "      message_count: 10"
        )
        for label, replacement in (
            (
                "missing",
                "        name: /tf_static\n"
                "        type: tf2_msgs/msg/TFMessage\n"
                "        serialization_format: cdr\n"
                "        offered_qos_profiles: |\n"
                "          - history: keep_last\n"
                "            depth: 5\n"
                "            reliability: reliable\n"
                "            durability: volatile",
            ),
            ("invalid", topic_block.replace("message_count: 10", "message_count: unknown")),
        ):
            with self.subTest(label=label):
                metadata.write_text(original.replace(topic_block, replacement))
                result = evaluate_preflight(
                    self.config,
                    "map-build",
                    {"rosbag": str(bag), "map_dir": str(self.map_root / "new_map")},
                )
                topic_check = self.check(result, "rosbag.mapping_topics")
                self.assertEqual(topic_check["status"], BLOCKED)
                self.assertEqual(topic_check["details"]["unknown_count"][0]["topic"], "/tf_static")

    def test_map_build_requires_bag_directory_not_metadata_file(self) -> None:
        self.camera_config()
        bag = self.bag()
        result = evaluate_preflight(
            self.config,
            "map-build",
            {
                "rosbag": str(bag / "metadata.yaml"),
                "map_dir": str(self.map_root / "new_map"),
            },
        )

        self.assertFalse(result["ready"])
        self.assertEqual(self.check(result, "rosbag.path")["status"], BLOCKED)

    def test_missing_default_camera_config_blocks_execution(self) -> None:
        bag = self.bag()
        result = evaluate_preflight(
            self.config,
            "map-build",
            {"rosbag": str(bag), "map_dir": str(self.map_root / "new_map")},
        )

        self.assertFalse(result["ready"])
        self.assertEqual(result["status"], BLOCKED)
        self.assertEqual(self.check(result, "camera.config")["status"], BLOCKED)

    def test_map_build_blocks_unsafe_output_and_parameters(self) -> None:
        bag = self.bag()
        self.camera_config()
        outside = Path(self.temporary_directory.name).parent / "outside-map"
        result = evaluate_preflight(
            self.config,
            "map-build",
            {"rosbag": str(bag), "map_dir": str(outside), "steps": "edex --bad/value"},
        )

        self.assertEqual(self.check(result, "map.output")["status"], BLOCKED)
        self.assertEqual(self.check(result, "mapping.parameters")["status"], BLOCKED)

    def test_map_build_tokens_match_execution_validation_and_model_must_not_be_empty(self) -> None:
        bag = self.bag()
        self.camera_config()
        payload = {"rosbag": str(bag), "map_dir": str(self.map_root / "new_map")}
        for token in (".starts-with-dot", "a" * 65):
            with self.subTest(token=token):
                result = evaluate_preflight(
                    self.config, "map-build", {**payload, "steps": token}
                )
                self.assertEqual(self.check(result, "mapping.parameters")["status"], BLOCKED)

        result = evaluate_preflight(
            self.config, "map-build", {**payload, "steps": "_" + "a" * 63}
        )
        self.assertEqual(self.check(result, "mapping.parameters")["status"], PASS)

        (self.model_dir / "model.plan").unlink()
        result = evaluate_preflight(self.config, "map-build", payload)
        self.assertFalse(result["ready"])
        self.assertEqual(self.check(result, "mapping.vgl_model")["status"], BLOCKED)

    def test_prepare_raster_accepts_snapshot_in_map_or_parent(self) -> None:
        path = self.ready_map()
        result = evaluate_preflight(self.config, "prepare-hd-raster", {"map_dir": str(path)})
        self.assertTrue(result["ready"])
        self.assertEqual(self.check(result, "raster.snapshot")["details"]["source"], "map")

        (path / "vslam_reference_snapshot.json").replace(path.parent / "vslam_reference_snapshot.json")
        result = evaluate_preflight(self.config, "prepare-hd-raster", {"map_dir": str(path)})
        self.assertTrue(result["ready"])
        self.assertEqual(self.check(result, "raster.snapshot")["details"]["source"], "parent")

    def test_prepare_raster_blocks_snapshot_without_landmarks(self) -> None:
        path = self.map_dir()
        (path / "vslam_reference_snapshot.json").write_text('{"landmarks": null}')
        result = evaluate_preflight(self.config, "prepare-hd-raster", {"map_dir": str(path)})
        self.assertFalse(result["ready"])
        self.assertEqual(self.check(result, "raster.snapshot")["status"], BLOCKED)

    def test_prepare_raster_blocks_invalid_or_incomplete_landmark_payload(self) -> None:
        path = self.map_dir()
        base_snapshot = {
            "landmarks": {
                "data": "AAAAAAAAAAAAAAAA",
                "point_step": 12,
                "fields": [{"name": name} for name in ("x", "y", "z")],
            }
        }
        snapshot_path = path / "vslam_reference_snapshot.json"
        for label, payload in (
            ("invalid_base64", "not base64!"),
            ("short_payload", "AAAA"),
            ("partial_point", "AAAAAAAAAAAAAAAAAAAA"),
        ):
            with self.subTest(label=label):
                snapshot = json.loads(json.dumps(base_snapshot))
                snapshot["landmarks"]["data"] = payload
                snapshot_path.write_text(json.dumps(snapshot))
                result = evaluate_preflight(
                    self.config, "prepare-hd-raster", {"map_dir": str(path)}
                )
                self.assertFalse(result["ready"])
                self.assertEqual(self.check(result, "raster.snapshot")["status"], BLOCKED)

    def test_prepare_raster_checks_large_payload_without_decoding_copy(self) -> None:
        path = self.ready_map()
        snapshot_path = path / "vslam_reference_snapshot.json"
        snapshot = json.loads(snapshot_path.read_text())
        # 1,000,000 base64 characters describe 750,000 bytes, divisible by point_step=12.
        snapshot["landmarks"]["data"] = "A" * 1_000_000
        snapshot_path.write_text(json.dumps(snapshot))

        result = evaluate_preflight(self.config, "prepare-hd-raster", {"map_dir": str(path)})

        self.assertTrue(result["ready"])
        self.assertEqual(self.check(result, "raster.snapshot")["status"], PASS)

    def test_raceline_validates_widths_and_requested_envelope(self) -> None:
        path = self.ready_map()
        result = evaluate_preflight(
            self.config,
            "generate-raceline",
            {"map_dir": str(path), "vehicle_width_m": 0.4, "safety_margin_m": 0.05},
        )
        self.assertTrue(result["ready"])
        self.assertEqual(self.check(result, "raceline.clearance")["status"], PASS)

        result = evaluate_preflight(
            self.config,
            "generate-raceline",
            {"map_dir": str(path), "vehicle_width_m": 0.55, "safety_margin_m": 0.05},
        )
        clearance = self.check(result, "raceline.clearance")
        self.assertEqual(clearance["status"], BLOCKED)
        self.assertAlmostEqual(clearance["details"]["required_envelope_width_m"], 0.65)

    def test_raceline_blocks_bad_centerline_semantics(self) -> None:
        path = self.ready_map()
        centerline = path / f"{path.name}_hd_map_centerline.csv"
        cases = {
            "too_few": "0,0,0.3,0.3\n" * 7,
            "non_finite": ("0,0,0.3,0.3\n" * 7) + "1,nan,0.3,0.3\n",
            "negative_width": ("0,0,0.3,0.3\n" * 7) + "1,0,-0.1,0.3\n",
            "missing_width": "0,0,0.3\n" * 8,
        }
        for label, content in cases.items():
            with self.subTest(label=label):
                centerline.write_text(content)
                result = evaluate_preflight(
                    self.config, "generate-raceline", {"map_dir": str(path)}
                )
                self.assertEqual(
                    self.check(result, "raceline.centerline")["status"], BLOCKED
                )

    def test_raceline_blocks_invalid_dimensions(self) -> None:
        path = self.ready_map()
        for value in (-0.1, math.inf, math.nan, True, "bad"):
            with self.subTest(value=value):
                result = evaluate_preflight(
                    self.config,
                    "generate-raceline",
                    {"map_dir": str(path), "vehicle_width_m": value},
                )
                self.assertEqual(
                    self.check(result, "raceline.parameters")["status"], BLOCKED
                )

    def test_raceline_rejects_symlink_input_and_non_regular_output(self) -> None:
        path = self.ready_map()
        outside = Path(self.temporary_directory.name) / "outside.csv"
        outside.write_text("outside\n")
        centerline = path / f"{path.name}_hd_map_centerline.csv"
        centerline_content = centerline.read_bytes()
        centerline.unlink()
        centerline.symlink_to(outside)

        result = evaluate_preflight(
            self.config, "generate-raceline", {"map_dir": str(path)}
        )
        self.assertEqual(self.check(result, "raceline.centerline")["status"], BLOCKED)

        centerline.unlink()
        centerline.write_bytes(centerline_content)
        output = path / f"{path.name}_raceline.csv"
        output.unlink()
        output.symlink_to(outside)
        result = evaluate_preflight(
            self.config, "generate-raceline", {"map_dir": str(path)}
        )
        self.assertEqual(self.check(result, "raceline.output")["status"], BLOCKED)

        output.unlink()
        output.mkdir()
        result = evaluate_preflight(
            self.config, "generate-raceline", {"map_dir": str(path)}
        )
        self.assertEqual(self.check(result, "raceline.output")["status"], BLOCKED)

    def test_raster_rejects_symlink_output(self) -> None:
        path = self.ready_map()
        outside = Path(self.temporary_directory.name) / "outside.png"
        outside.write_bytes(b"outside")
        output = path / "vslam_landmarks.png"
        output.unlink()
        output.symlink_to(outside)

        result = evaluate_preflight(
            self.config, "prepare-hd-raster", {"map_dir": str(path)}
        )

        self.assertFalse(result["ready"])
        self.assertEqual(self.check(result, "raster.outputs")["status"], BLOCKED)

    def test_preview_rejects_symlink_inputs_embedded_image_and_output(self) -> None:
        path = self.ready_map()
        outside_yaml = Path(self.temporary_directory.name) / "outside.yaml"
        outside_yaml.write_text("lanes: []\n")
        hd_map = path / f"{path.name}_hd_map.yaml"
        hd_content = hd_map.read_bytes()
        hd_map.unlink()
        hd_map.symlink_to(outside_yaml)

        result = evaluate_preflight(self.config, "generate-preview", {"map_dir": str(path)})
        self.assertEqual(self.check(result, "preview.hd_map")["status"], BLOCKED)

        hd_map.unlink()
        hd_map.write_bytes(hd_content)
        outside_image = Path(self.temporary_directory.name) / "outside.png"
        outside_image.write_bytes(b"outside")
        raster_image = path / "vslam_landmarks.png"
        raster_image.unlink()
        raster_image.symlink_to(outside_image)
        result = evaluate_preflight(self.config, "generate-preview", {"map_dir": str(path)})
        self.assertEqual(self.check(result, "preview.raster")["status"], BLOCKED)

        raster_image.unlink()
        raster_image.write_bytes(b"restored")
        preview = path / f"{path.name}_line_preview.png"
        preview.symlink_to(outside_image)
        result = evaluate_preflight(self.config, "generate-preview", {"map_dir": str(path)})
        self.assertEqual(self.check(result, "preview.output")["status"], BLOCKED)

    def test_map_stage_requires_writable_map_directory(self) -> None:
        path = self.ready_map()
        with patch("jetpilot_console.preflight.os.access", return_value=False):
            result = evaluate_preflight(
                self.config, "generate-raceline", {"map_dir": str(path)}
            )

        self.assertFalse(result["ready"])
        self.assertEqual(self.check(result, "map.directory")["status"], BLOCKED)

    def test_preview_requires_all_four_semantic_artifacts(self) -> None:
        path = self.ready_map()
        result = evaluate_preflight(self.config, "generate-preview", {"map_dir": str(path)})
        self.assertTrue(result["ready"])
        for check_id in (
            "preview.raster",
            "preview.hd_map",
            "preview.centerline",
            "preview.raceline",
        ):
            self.assertEqual(self.check(result, check_id)["status"], PASS)

        files_and_checks = (
            (path / "vslam_landmarks.yaml", "preview.raster"),
            (path / f"{path.name}_hd_map.yaml", "preview.hd_map"),
            (path / f"{path.name}_hd_map_centerline.csv", "preview.centerline"),
            (path / f"{path.name}_raceline.csv", "preview.raceline"),
        )
        for artifact, check_id in files_and_checks:
            with self.subTest(artifact=artifact.name):
                original = artifact.read_bytes()
                artifact.unlink()
                result = evaluate_preflight(
                    self.config, "generate-preview", {"map_dir": str(path)}
                )
                self.assertFalse(result["ready"])
                self.assertEqual(self.check(result, check_id)["status"], BLOCKED)
                artifact.write_bytes(original)

    def test_map_stage_rejects_map_outside_root_and_unknown_action(self) -> None:
        outside = Path(self.temporary_directory.name).parent
        result = evaluate_preflight(
            self.config, "generate-preview", {"map_dir": str(outside)}
        )
        self.assertFalse(result["ready"])
        self.assertEqual(self.check(result, "map.directory")["status"], BLOCKED)
        with self.assertRaises(ValueError):
            evaluate_preflight(self.config, "not-an-action", {})


if __name__ == "__main__":
    unittest.main()
