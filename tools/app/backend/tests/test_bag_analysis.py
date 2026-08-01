from __future__ import annotations

import io
import json
import os
import subprocess
import tempfile
import unittest
from email.message import Message
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import quote

from jetpilot_console.bag_analysis import (
    AnalysisRepository,
    build_analysis_script,
    rosbag_detail,
)
from jetpilot_console.analysis_worker import Progress
from jetpilot_console.indexes import scan_rosbags
from jetpilot_console.main import Handler
from jetpilot_console.preflight import BLOCKED, PASS, WARNING, evaluate_preflight
from jetpilot_console.tasks import TaskResourceConflict


def write_bag(bag_dir: Path, topics: dict[str, tuple[str, int]]) -> Path:
    bag_dir.mkdir(parents=True, exist_ok=True)
    (bag_dir / "data_0.mcap").write_bytes(b"bag-data")
    topic_lines = []
    total = 0
    for name, (message_type, count) in topics.items():
        total += count
        topic_lines.extend(
            [
                "    - topic_metadata:",
                f"        name: {name}",
                f"        type: {message_type}",
                "        serialization_format: cdr",
                f"      message_count: {count}",
            ]
        )
    (bag_dir / "metadata.yaml").write_text(
        "\n".join(
            [
                "rosbag2_bagfile_information:",
                "  storage_identifier: mcap",
                "  duration:",
                "    nanoseconds: 12500000000",
                "  starting_time:",
                "    nanoseconds_since_epoch: 1720000000123456789",
                f"  message_count: {total}",
                "  relative_file_paths:",
                "    - data_0.mcap",
                "  topics_with_message_count:",
                *topic_lines,
                "",
            ]
        ),
        encoding="utf-8",
    )
    return bag_dir


class RosbagDetailTests(unittest.TestCase):
    def test_metadata_detail_exposes_topics_and_string_epoch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            record_root = Path(temporary_directory) / "record"
            bag = write_bag(
                record_root / "run-a",
                {
                    "/camera": ("sensor_msgs/msg/Image", 20),
                    "/vehicle/control_cmd": ("jetpilot_msgs/msg/ControlCommand", 100),
                },
            )
            detail = rosbag_detail(SimpleNamespace(record_root=record_root), bag)

            self.assertEqual(detail["duration_seconds"], 12.5)
            self.assertEqual(detail["starting_time_ns"], "1720000000123456789")
            self.assertEqual(detail["message_count"], 120)
            self.assertEqual(detail["topic_count"], 2)
            self.assertEqual(detail["topics"][0]["name"], "/camera")
            json.dumps(detail)

    def test_detail_rejects_storage_escape_and_scanner_skips_analysis_root(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            record_root = Path(temporary_directory) / "record"
            bag = write_bag(record_root / "run-a", {"/camera": ("sensor_msgs/msg/Image", 1)})
            metadata = bag / "metadata.yaml"
            metadata.write_text(metadata.read_text().replace("data_0.mcap", "../outside.mcap"))
            with self.assertRaises(ValueError):
                rosbag_detail(SimpleNamespace(record_root=record_root), bag)

            write_bag(
                record_root / ".jetpilot_analysis" / "fake",
                {"/camera": ("sensor_msgs/msg/Image", 1)},
            )
            scanned = scan_rosbags(record_root)
            self.assertEqual(len(scanned), 1)
            self.assertEqual(scanned[0]["name"], "run-a")
            self.assertNotIn(".jetpilot_analysis", scanned[0]["path"])


class AnalysisRepositoryTests(unittest.TestCase):
    def test_create_list_timeline_and_safe_frame_access(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "analyses"
            repository = AnalysisRepository(root)
            created = repository.create(
                label="session A",
                request={"image_topic": "/camera"},
                preflight={"resolved": {"trajectory_mode": "none"}},
                initial_phase="extracting",
            )
            analysis_id = created["analysis_id"]
            analysis_dir = Path(created["path"])
            (analysis_dir / "timeline.json").write_text(
                json.dumps({"frames": [], "controls": [], "modes": [], "speeds": [], "trajectory": {"samples": []}}),
                encoding="utf-8",
            )
            frames = analysis_dir / "frames"
            frames.mkdir()
            frame = frames / "frame_0001.jpg"
            frame.write_bytes(b"jpeg")

            self.assertEqual(repository.timeline(analysis_id)["frames"], [])
            self.assertEqual(repository.frame(analysis_id, "frames/frame_0001.jpg")[0], frame)
            self.assertEqual(repository.list()[0]["analysis_id"], analysis_id)
            with self.assertRaises(ValueError):
                repository.frame(analysis_id, "../manifest.json")
            with self.assertRaises(ValueError):
                repository.detail("../escape")

    def test_frame_symlink_cannot_escape_analysis(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            parent = Path(temporary_directory)
            repository = AnalysisRepository(parent / "analyses")
            created = repository.create(
                label="safe",
                request={},
                preflight={"resolved": {}},
                initial_phase="extracting",
            )
            frames = Path(created["path"]) / "frames"
            frames.mkdir()
            outside = parent / "outside.jpg"
            outside.write_bytes(b"secret")
            (frames / "leak.jpg").symlink_to(outside)
            with self.assertRaises(ValueError):
                repository.frame(created["analysis_id"], "leak.jpg")

    def test_task_attachment_and_worker_progress_preserve_each_others_fields(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            repository = AnalysisRepository(Path(temporary_directory) / "analyses")
            created = repository.create(
                label="race",
                request={},
                preflight={"resolved": {}},
                initial_phase="extracting",
            )
            analysis_id = created["analysis_id"]
            status_path = Path(created["path"]) / "status.json"

            # The worker can start before TaskManager returns its task id.
            Progress(status_path).update(
                "extract", 0.25, "worker started", status="running"
            )
            repository.attach_task(
                analysis_id,
                {"task_id": "task-after-worker", "status": "queued"},
            )
            status = repository.detail(analysis_id)["status"]
            self.assertEqual(status["task_id"], "task-after-worker")
            self.assertEqual(status["status"], "running")
            self.assertEqual(status["phase"], "extract")
            self.assertEqual(status["message"], "worker started")

            # Later worker updates must retain the task id attached by the API.
            Progress(status_path).update(
                "complete", 1.0, "worker complete", status="completed"
            )
            status = repository.detail(analysis_id)["status"]
            self.assertEqual(status["task_id"], "task-after-worker")
            self.assertEqual(status["status"], "completed")
            self.assertEqual(status["phase"], "complete")


class AnalysisPreflightTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        root = Path(self.temporary_directory.name)
        self.record_root = root / "record"
        self.map_root = root / "map"
        self.ros2_ws = root / "ros2_ws"
        self.record_root.mkdir()
        self.map_root.mkdir()
        (self.ros2_ws / "install").mkdir(parents=True)
        (self.ros2_ws / "install/setup.bash").write_text("", encoding="utf-8")
        self.config = SimpleNamespace(
            record_root=self.record_root,
            map_root=self.map_root,
            ros2_ws=self.ros2_ws,
            python_bin="python3",
        )

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    @staticmethod
    def check(result: dict[str, object], check_id: str) -> dict[str, object]:
        return next(check for check in result["checks"] if check["id"] == check_id)

    def test_recorded_analysis_requires_image_but_allows_missing_commands(self) -> None:
        bag = write_bag(
            self.record_root / "run",
            {
                "/camera": ("sensor_msgs/msg/Image", 30),
                "/visual_slam/tracking/odometry": ("nav_msgs/msg/Odometry", 30),
            },
        )
        result = evaluate_preflight(
            self.config,
            "analyze-rosbag",
            {"rosbag": str(bag), "image_topic": "/camera", "trajectory_mode": "auto"},
        )

        self.assertTrue(result["ready"])
        self.assertEqual(result["resolved"]["trajectory_mode"], "recorded")
        self.assertEqual(self.check(result, "analysis.image_topic")["status"], PASS)
        self.assertEqual(self.check(result, "analysis.control_topic")["status"], WARNING)
        self.assertEqual(self.check(result, "analysis.mode_topic")["status"], WARNING)

        blocked = evaluate_preflight(
            self.config,
            "analyze-rosbag",
            {"rosbag": str(bag), "trajectory_mode": "none"},
        )
        self.assertFalse(blocked["ready"])
        self.assertEqual(self.check(blocked, "analysis.image_topic")["status"], BLOCKED)

    def test_analysis_runtime_is_reported_before_start(self) -> None:
        bag = write_bag(
            self.record_root / "runtime-check",
            {"/camera": ("sensor_msgs/msg/Image", 1)},
        )
        (self.ros2_ws / "install/setup.bash").unlink()

        result = evaluate_preflight(
            self.config,
            "analyze-rosbag",
            {"rosbag": str(bag), "image_topic": "/camera", "trajectory_mode": "none"},
        )

        self.assertFalse(result["ready"])
        self.assertEqual(self.check(result, "analysis.runtime")["status"], BLOCKED)

    def test_offline_analysis_checks_map_stereo_topics_and_model(self) -> None:
        stereo_topics = {
            "/camera": ("sensor_msgs/msg/Image", 20),
            "/realsense/infra1/image_rect_raw": ("sensor_msgs/msg/Image", 20),
            "/realsense/infra1/camera_info": ("sensor_msgs/msg/CameraInfo", 20),
            "/realsense/infra2/image_rect_raw": ("sensor_msgs/msg/Image", 20),
            "/realsense/infra2/camera_info": ("sensor_msgs/msg/CameraInfo", 20),
            "/tf_static": ("tf2_msgs/msg/TFMessage", 1),
        }
        bag = write_bag(self.record_root / "run", stereo_topics)
        map_dir = self.map_root / "course"
        (map_dir / "cuvgl_map").mkdir(parents=True)
        (map_dir / "cuvslam_map").mkdir()
        (map_dir / "cuvgl_map/index").write_bytes(b"map")
        (map_dir / "cuvslam_map/index").write_bytes(b"map")
        config_path = (
            self.ros2_ws
            / "src/launch/jetpilot_system_launch/config/localization/vgl_camera_topics.yaml"
        )
        config_path.parent.mkdir(parents=True)
        config_path.write_text(
            """stereo_cameras:
  - name: front
    left: /realsense/infra1/image_rect_raw
    left_camera_info: /realsense/infra1/camera_info
    right: /realsense/infra2/image_rect_raw
    right_camera_info: /realsense/infra2/camera_info
""",
            encoding="utf-8",
        )
        model = self.ros2_ws / "isaac_ros_assets/models/visual_global_localization"
        model.mkdir(parents=True)
        (model / "model.plan").write_bytes(b"model")

        result = evaluate_preflight(
            self.config,
            "analyze-rosbag",
            {
                "rosbag": str(bag),
                "image_topic": "/camera",
                "map_dir": str(map_dir),
                "trajectory_mode": "offline",
            },
        )
        self.assertTrue(result["ready"])
        self.assertEqual(result["resolved"]["trajectory_mode"], "offline")
        self.assertEqual(self.check(result, "analysis.localization_map")["status"], PASS)
        self.assertEqual(self.check(result, "rosbag.mapping_topics")["status"], PASS)

        no_map = evaluate_preflight(
            self.config,
            "analyze-rosbag",
            {"rosbag": str(bag), "image_topic": "/camera", "trajectory_mode": "offline"},
        )
        self.assertFalse(no_map["ready"])
        self.assertEqual(self.check(no_map, "analysis.map")["status"], BLOCKED)

        vslam_only_map = self.map_root / "course-vslam-only"
        (vslam_only_map / "cuvslam_map").mkdir(parents=True)
        (vslam_only_map / "cuvslam_map/index").write_bytes(b"map")
        vslam_only = evaluate_preflight(
            self.config,
            "analyze-rosbag",
            {
                "rosbag": str(bag),
                "image_topic": "/camera",
                "map_dir": str(vslam_only_map),
                "trajectory_mode": "offline",
                "offline_localization_mode": "vslam",
            },
        )
        self.assertTrue(vslam_only["ready"])
        self.assertEqual(
            vslam_only["resolved"]["offline_localization_mode"], "vslam"
        )
        self.assertEqual(
            self.check(vslam_only, "analysis.localization_map")["status"], PASS
        )
        self.assertNotIn("output_model_dir", vslam_only["resolved"])

        auto_without_vgl = evaluate_preflight(
            self.config,
            "analyze-rosbag",
            {
                "rosbag": str(bag),
                "image_topic": "/camera",
                "map_dir": str(vslam_only_map),
                "trajectory_mode": "offline",
                "offline_localization_mode": "auto",
            },
        )
        self.assertTrue(auto_without_vgl["ready"])
        self.assertEqual(
            auto_without_vgl["resolved"]["offline_localization_mode"], "vslam"
        )
        self.assertEqual(
            self.check(auto_without_vgl, "analysis.localization_map")["status"],
            WARNING,
        )

        vgl_without_vgl_map = evaluate_preflight(
            self.config,
            "analyze-rosbag",
            {
                "rosbag": str(bag),
                "image_topic": "/camera",
                "map_dir": str(vslam_only_map),
                "trajectory_mode": "offline",
                "offline_localization_mode": "vgl",
            },
        )
        self.assertFalse(vgl_without_vgl_map["ready"])
        self.assertEqual(
            self.check(vgl_without_vgl_map, "analysis.localization_map")["status"],
            BLOCKED,
        )

    def test_command_speed_remains_runnable_but_is_not_vehicle_speed(self) -> None:
        bag = write_bag(
            self.record_root / "run-command-speed",
            {
                "/camera": ("sensor_msgs/msg/Image", 30),
                "/visual_slam/tracking/odometry": ("nav_msgs/msg/Odometry", 30),
                "/commands/vehicle_speed": ("std_msgs/msg/Float32", 30),
            },
        )
        result = evaluate_preflight(
            self.config,
            "analyze-rosbag",
            {
                "rosbag": str(bag),
                "image_topic": "/camera",
                "speed_topic": "/commands/vehicle_speed",
                "trajectory_mode": "recorded",
            },
        )

        self.assertTrue(result["ready"])
        self.assertEqual(self.check(result, "analysis.speed_semantics")["status"], WARNING)
        self.assertEqual(result["resolved"]["speed_topic"], "/commands/vehicle_speed")
        self.assertEqual(result["resolved"]["speed_kind"], "commanded")
        self.assertEqual(result["resolved"]["speed_label"], "Commanded speed")


class AnalysisScriptTests(unittest.TestCase):
    def test_worker_contract_and_offline_safety_flags(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            config = SimpleNamespace(
                ros2_ws=root / "ros2_ws",
                python_bin="/opt/env/bin/python",
                launch_package="jetpilot_system_launch",
                analysis_ros_domain_id=92,
            )
            recorded = build_analysis_script(
                config,
                analysis_dir=root / "analysis",
                rosbag=root / "record/run",
                image_topic="/camera",
                control_topic="/vehicle/control_cmd",
                mode_topic="/operation_mode/state",
                pose_topic="/visual_slam/tracking/odometry",
                speed_topic="",
                map_dir=None,
                trajectory_mode="recorded",
                max_fps=15.0,
            )
            self.assertIn("jetpilot_console.analysis_worker", recorded)
            self.assertIn("--status-file", recorded)
            self.assertIn("--pose-topic /visual_slam/tracking/odometry", recorded)
            self.assertIn("trap analysis_exit EXIT", recorded)
            self.assertIn("--set-status failed --stage failed", recorded)
            self.assertNotIn("ros2 launch", recorded)

            offline = build_analysis_script(
                config,
                analysis_dir=root / "analysis",
                rosbag=root / "record/run",
                image_topic="/camera",
                control_topic="",
                mode_topic="",
                pose_topic="",
                speed_topic="",
                map_dir=root / "map/course",
                trajectory_mode="offline",
                max_fps=10.0,
                topic_config=root / "topics.yaml",
                model_dir=root / "models",
            )
            self.assertIn("export ROS_DOMAIN_ID=92", offline)
            self.assertIn("enable_vehicle:=false", offline)
            self.assertIn("publish_vehicle_description:=false", offline)
            self.assertIn("replay_additional_args:='--clock --start-paused'", offline)
            self.assertIn("/rosbag2_player/resume", offline)
            self.assertIn("localization readiness timed out", offline)
            self.assertIn("rosbag_shutdown_on_exit:=false", offline)
            self.assertIn("--stage offline_drain", offline)
            self.assertIn("vslam_snapshot_require_localized_map:=true", offline)
            self.assertIn("vslam_snapshot_tf_topic:=/tf", offline)
            self.assertIn("vslam_snapshot_write_interval_s:=0.0", offline)
            self.assertIn("--set-status running --stage offline_localization", offline)
            self.assertIn("--trajectory-snapshot", offline)
            self.assertIn("run_offline_localization_attempt vgl 0.15 0.40", offline)
            self.assertIn("--stage offline_fallback", offline)
            self.assertIn("run_offline_localization_attempt vslam 0.41 0.415", offline)
            self.assertIn("ros2 topic pub --once /initialpose", offline)
            syntax = subprocess.run(
                ["bash", "-n"],
                input=offline,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(syntax.returncode, 0, syntax.stderr)

            vslam_only = build_analysis_script(
                config,
                analysis_dir=root / "analysis-vslam",
                rosbag=root / "record/run",
                image_topic="/camera",
                control_topic="",
                mode_topic="",
                pose_topic="",
                speed_topic="",
                map_dir=root / "map/course",
                trajectory_mode="offline",
                offline_localization_mode="vslam",
                max_fps=10.0,
                topic_config=root / "topics.yaml",
                model_dir=None,
            )
            self.assertIn("run_offline_localization_attempt vslam 0.15 0.40", vslam_only)
            self.assertIn("vslam_identity", vslam_only)
            self.assertNotIn("--stage offline_fallback", vslam_only)
            vslam_syntax = subprocess.run(
                ["bash", "-n"],
                input=vslam_only,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(vslam_syntax.returncode, 0, vslam_syntax.stderr)

    def test_auto_offline_runtime_restarts_with_identity_hint_after_vgl_exit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            ros2_ws = root / "ros2_ws"
            (ros2_ws / "install").mkdir(parents=True)
            (ros2_ws / "install/setup.bash").write_text("", encoding="utf-8")
            fake_bin = root / "bin"
            fake_bin.mkdir()
            state_file = root / "fake-ros-active"
            service_counter = root / "fake-service-counter"
            fake_ros_log = root / "fake-ros.log"

            fake_ros2 = fake_bin / "ros2"
            fake_ros2.write_text(
                """#!/bin/bash
set -u
printf '%s\n' "$*" >> "$FAKE_ROS_LOG"
command_name="${1:-}"
subcommand="${2:-}"
if [ "$command_name" = "launch" ]; then
  enable_vgl=true
  snapshot=""
  for argument in "$@"; do
    case "$argument" in
      enable_vgl:=*) enable_vgl="${argument#enable_vgl:=}" ;;
      vslam_snapshot_output:=*) snapshot="${argument#vslam_snapshot_output:=}" ;;
    esac
  done
  if [ "$enable_vgl" = "true" ]; then
    exit 42
  fi
  printf '%s\n' active > "$FAKE_ROS_STATE"
  rm -f "$FAKE_ROS_SERVICE_COUNTER"
  cleanup() {
    printf '%s\n' '{"localization":{"confirmed":true}}' > "$snapshot"
    rm -f "$FAKE_ROS_STATE"
    exit 0
  }
  trap cleanup INT TERM
  while true; do
    if [ -f "$FAKE_ROS_SERVICE_COUNTER" ]; then
      count="$(<"$FAKE_ROS_SERVICE_COUNTER")"
      if [ "$count" -ge 4 ]; then cleanup; fi
    fi
    /bin/sleep 0.1
  done
fi
if [ "$command_name" = "node" ] && [ "$subcommand" = "list" ]; then
  if [ -f "$FAKE_ROS_STATE" ]; then
    printf '%s\n' /visual_slam_node /localization_manager /vslam_reference_snapshot_recorder
  fi
  exit 0
fi
if [ "$command_name" = "service" ] && [ "$subcommand" = "type" ]; then
  if [ -f "$FAKE_ROS_STATE" ]; then
    count=0
    if [ -f "$FAKE_ROS_SERVICE_COUNTER" ]; then count="$(<"$FAKE_ROS_SERVICE_COUNTER")"; fi
    count=$((count + 1))
    printf '%s\n' "$count" > "$FAKE_ROS_SERVICE_COUNTER"
    if [ "$count" -le 3 ]; then
      printf '%s\n' rosbag2_interfaces/srv/Resume
    fi
  fi
  exit 0
fi
if [ "$command_name" = "service" ] && [ "$subcommand" = "call" ]; then exit 0; fi
if [ "$command_name" = "topic" ] && [ "$subcommand" = "pub" ]; then exit 0; fi
exit 1
""",
                encoding="utf-8",
            )
            fake_ros2.chmod(0o755)
            fake_sleep = fake_bin / "sleep"
            fake_sleep.write_text("#!/bin/bash\n/bin/sleep 0.01\n", encoding="utf-8")
            fake_sleep.chmod(0o755)

            analysis_dir = root / "analysis"
            script = build_analysis_script(
                SimpleNamespace(
                    ros2_ws=ros2_ws,
                    python_bin="/usr/bin/true",
                    launch_package="jetpilot_system_launch",
                    analysis_ros_domain_id=92,
                ),
                analysis_dir=analysis_dir,
                rosbag=root / "record/run",
                image_topic="/camera",
                control_topic="",
                mode_topic="",
                pose_topic="",
                speed_topic="",
                map_dir=root / "map/course",
                trajectory_mode="offline",
                offline_localization_mode="auto",
                max_fps=10.0,
                topic_config=root / "topics.yaml",
                model_dir=root / "models",
            )
            completed = subprocess.run(
                ["bash"],
                input=script,
                text=True,
                capture_output=True,
                check=False,
                timeout=10,
                env={
                    **os.environ,
                    "PATH": f"{fake_bin}:{os.environ.get('PATH', '')}",
                    "FAKE_ROS_STATE": str(state_file),
                    "FAKE_ROS_SERVICE_COUNTER": str(service_counter),
                    "FAKE_ROS_LOG": str(fake_ros_log),
                },
            )

            fake_log = fake_ros_log.read_text(encoding="utf-8") if fake_ros_log.is_file() else ""
            self.assertEqual(
                completed.returncode,
                0,
                completed.stderr + completed.stdout + "\nFAKE ROS LOG:\n" + fake_log,
            )
            self.assertIn("VGL offline localization failed", completed.stdout)
            self.assertEqual(
                (analysis_dir / "localization/method.txt").read_text(encoding="utf-8").strip(),
                "vslam_identity_fallback",
            )
            self.assertTrue((analysis_dir / "localization/vslam_snapshot.json").is_file())

    def test_worker_accepts_multiple_image_topics_and_primary_topic(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            config = SimpleNamespace(
                ros2_ws=root / "ros2_ws",
                python_bin="/opt/env/bin/python",
                launch_package="jetpilot_system_launch",
                analysis_ros_domain_id=92,
            )

            script = build_analysis_script(
                config,
                analysis_dir=root / "analysis",
                rosbag=root / "record/run",
                image_topic="/realsense/color/image_raw",
                image_topics=[
                    "/realsense/color/image_raw",
                    "/realsense/infra1/image_rect_raw",
                    "/flir/image_raw",
                ],
                primary_image_topic="/flir/image_raw",
                control_topic="",
                mode_topic="",
                pose_topic="",
                speed_topic="",
                map_dir=None,
                trajectory_mode="recorded",
                max_fps=15.0,
            )

            self.assertIn("--image-topics /realsense/color/image_raw", script)
            self.assertIn("--image-topics /realsense/infra1/image_rect_raw", script)
            self.assertIn("--image-topics /flir/image_raw", script)
            self.assertIn("--primary-image-topic /flir/image_raw", script)


class _StartedTask:
    def __init__(self, task_id: str = "analysis-test") -> None:
        self.task_id = task_id
        self.title = "Analyze rosbag"

    def to_json(self) -> dict[str, object]:
        return {
            "task_id": self.task_id,
            "kind": "analyze-rosbag",
            "title": "Analyze rosbag",
            "status": "queued",
        }


class _RecordingTasks:
    def __init__(self, failure: Exception | None = None) -> None:
        self.calls: list[dict[str, object]] = []
        self.failure = failure

    def start(self, **kwargs: object) -> _StartedTask:
        self.calls.append(kwargs)
        if self.failure is not None:
            raise self.failure
        return _StartedTask()


class AnalysisRouteTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        root = Path(self.temporary_directory.name)
        self.record_root = root / "record"
        self.map_root = root / "map"
        self.ros2_ws = root / "ros2_ws"
        self.repo_root = root / "repo"
        for path in (self.record_root, self.map_root, self.ros2_ws, self.repo_root):
            path.mkdir()
        (self.ros2_ws / "install").mkdir()
        (self.ros2_ws / "install/setup.bash").write_text("", encoding="utf-8")
        self.config = SimpleNamespace(
            record_root=self.record_root,
            map_root=self.map_root,
            ros2_ws=self.ros2_ws,
            repo_root=self.repo_root,
            python_ws=self.repo_root / "python_ws",
            python_bin="python3",
            launch_package="jetpilot_system_launch",
            analysis_ros_domain_id=92,
        )
        self.analyses = AnalysisRepository(self.record_root / ".jetpilot_analysis")
        self.tasks = _RecordingTasks()

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def request(
        self,
        method: str,
        path: str,
        body: dict[str, object] | None = None,
        *,
        tasks: _RecordingTasks | None = None,
    ) -> tuple[int, str, bytes]:
        encoded = json.dumps(body or {}).encode("utf-8") if method == "POST" else b""
        headers = Message()
        headers["Host"] = "127.0.0.1:8765"
        if method == "POST":
            headers["Content-Type"] = "application/json"
            headers["Content-Length"] = str(len(encoded))

        handler = Handler.__new__(Handler)
        handler.path = path
        handler.command = method
        handler.request_version = "HTTP/1.1"
        handler.requestline = f"{method} {path} HTTP/1.1"
        handler.client_address = ("127.0.0.1", 12345)
        handler.close_connection = True
        handler.headers = headers
        handler.rfile = io.BytesIO(encoded)
        handler.wfile = io.BytesIO()
        handler.server = SimpleNamespace(
            state=SimpleNamespace(
                config=self.config,
                joy_only=False,
                loopback_only=True,
                tasks=tasks or self.tasks,
                analyses=self.analyses,
            )
        )
        if method == "POST":
            handler.do_POST()
        else:
            handler.do_GET()
        raw_headers, response_body = handler.wfile.getvalue().split(b"\r\n\r\n", 1)
        status = int(raw_headers.splitlines()[0].split()[1])
        return status, raw_headers.decode("iso-8859-1"), response_body

    @staticmethod
    def json_body(response_body: bytes) -> dict[str, object]:
        value = json.loads(response_body.decode("utf-8"))
        if not isinstance(value, dict):
            raise AssertionError("expected an object response")
        return value

    def recorded_payload(self, bag: Path) -> dict[str, object]:
        return {
            "rosbag": str(bag),
            "image_topic": "/camera",
            "pose_topic": "/visual_slam/tracking/odometry",
            "trajectory_mode": "recorded",
            "max_fps": 12,
        }

    def test_start_and_read_analysis_artifacts_through_http(self) -> None:
        bag = write_bag(
            self.record_root / "route-run",
            {
                "/camera": ("sensor_msgs/msg/Image", 30),
                "/visual_slam/tracking/odometry": ("nav_msgs/msg/Odometry", 30),
            },
        )
        status, _, raw = self.request(
            "POST", "/api/analyses/start", self.recorded_payload(bag)
        )
        payload = self.json_body(raw)
        self.assertEqual(status, 201)
        self.assertTrue(payload["preflight"]["ready"])
        analysis_id = str(payload["analysis"]["analysis_id"])
        self.assertEqual(self.tasks.calls[0]["resource_key"], f"analysis-bag:{bag.resolve()}")
        self.assertEqual(
            self.tasks.calls[0]["resource_keys"],
            [f"analysis-bag:{bag.resolve()}"],
        )
        self.assertIn("trap analysis_exit EXIT", self.tasks.calls[0]["command"][2])

        analysis_dir = Path(str(payload["analysis"]["path"]))
        timeline = {
            "frames": [{"t": 0.0, "path": "frames/frame_0001.jpg"}],
            "controls": [],
            "modes": [],
            "speeds": [],
            "trajectory": {"samples": []},
        }
        (analysis_dir / "timeline.json").write_text(json.dumps(timeline), encoding="utf-8")
        (analysis_dir / "frames").mkdir()
        (analysis_dir / "frames/frame_0001.jpg").write_bytes(b"jpeg-frame")

        list_status, _, list_raw = self.request("GET", "/api/analyses")
        self.assertEqual(list_status, 200)
        self.assertEqual(self.json_body(list_raw)["analyses"][0]["analysis_id"], analysis_id)

        detail_status, _, detail_raw = self.request(
            "GET", f"/api/analyses/{analysis_id}"
        )
        self.assertEqual(detail_status, 200)
        self.assertTrue(self.json_body(detail_raw)["analysis"]["timeline_available"])

        timeline_status, _, timeline_raw = self.request(
            "GET", f"/api/analyses/{analysis_id}/timeline"
        )
        self.assertEqual(timeline_status, 200)
        self.assertEqual(self.json_body(timeline_raw)["frames"], timeline["frames"])

        frame_status, frame_headers, frame_raw = self.request(
            "GET", f"/api/analyses/{analysis_id}/frames/frame_0001.jpg"
        )
        self.assertEqual(frame_status, 200)
        self.assertIn("Content-Type: image/jpeg", frame_headers)
        self.assertEqual(frame_raw, b"jpeg-frame")

        traversal_status, _, _ = self.request(
            "GET", f"/api/analyses/{analysis_id}/frames/..%2Fmanifest.json"
        )
        self.assertEqual(traversal_status, 400)

        bag_status, _, bag_raw = self.request(
            "GET", f"/api/rosbags/detail?path={quote(str(bag), safe='')}"
        )
        self.assertEqual(bag_status, 200)
        self.assertEqual(self.json_body(bag_raw)["rosbag"]["topic_count"], 2)

    def test_e2e_recorded_analysis_route_and_model_index(self) -> None:
        bag = write_bag(
            self.record_root / "e2e-run",
            {
                "/camera": ("sensor_msgs/msg/Image", 30),
                "/auto/control_cmd": ("jetpilot_msgs/msg/ControlCommand", 30),
                "/vehicle/control_cmd": ("jetpilot_msgs/msg/ControlCommand", 30),
                "/visual_slam/tracking/odometry": ("nav_msgs/msg/Odometry", 30),
                "/e2e/diagnostics": ("diagnostic_msgs/msg/DiagnosticArray", 30),
            },
        )
        model_dir = self.repo_root / "outputs" / "run-a"
        model_dir.mkdir(parents=True)
        (model_dir / "model.onnx").write_bytes(b"onnx")
        (model_dir / "metadata.json").write_text(
            json.dumps({"model_name": "run-a"}), encoding="utf-8"
        )

        model_status, _, model_raw = self.request("GET", "/api/e2e/models")
        self.assertEqual(model_status, 200)
        self.assertEqual(self.json_body(model_raw)["models"][0]["name"], "run-a")

        request = {
            "analysis_kind": "e2e",
            "e2e_mode": "recorded_localization",
            "rosbag": str(bag),
            "image_topic": "/camera",
            "prediction_control_topic": "/auto/control_cmd",
            "applied_control_topic": "/vehicle/control_cmd",
            "pose_topic": "/visual_slam/tracking/odometry",
            "e2e_diagnostic_topic": "/e2e/diagnostics",
            "trajectory_mode": "recorded",
            "max_fps": 15,
        }
        status, _, raw = self.request("POST", "/api/e2e-analyses", request)
        payload = self.json_body(raw)

        self.assertEqual(status, 201, payload)
        self.assertTrue(payload["preflight"]["ready"])
        self.assertEqual(self.tasks.calls[0]["kind"], "analyze-e2e")
        script = str(self.tasks.calls[0]["command"][2])
        self.assertIn("jetpilot_console.e2e_analysis_worker", script)
        self.assertIn("--mode recorded_localization", script)
        self.assertIn("--e2e-diagnostic-topic /e2e/diagnostics", script)

    def test_analysis_claims_selected_map_as_a_second_resource(self) -> None:
        bag = write_bag(
            self.record_root / "mapped-run",
            {
                "/camera": ("sensor_msgs/msg/Image", 30),
                "/visual_slam/tracking/odometry": ("nav_msgs/msg/Odometry", 30),
            },
        )
        map_dir = self.map_root / "course"
        map_dir.mkdir()
        request = self.recorded_payload(bag)
        request["map_dir"] = str(map_dir)

        status, _, raw = self.request("POST", "/api/analyses/start", request)

        self.assertEqual(status, 201, self.json_body(raw))
        call = self.tasks.calls[0]
        self.assertEqual(call["resource_key"], f"analysis-bag:{bag.resolve()}")
        self.assertEqual(
            call["resource_keys"],
            [
                f"analysis-bag:{bag.resolve()}",
                f"map-dir:{map_dir.resolve()}",
            ],
        )

    def test_map_resource_conflict_marks_analysis_blocked(self) -> None:
        bag = write_bag(
            self.record_root / "map-conflict-run",
            {
                "/camera": ("sensor_msgs/msg/Image", 30),
                "/visual_slam/tracking/odometry": ("nav_msgs/msg/Odometry", 30),
            },
        )
        map_dir = self.map_root / "course"
        map_dir.mkdir()
        request = self.recorded_payload(bag)
        request["map_dir"] = str(map_dir)
        active = _StartedTask("active-map-build")
        tasks = _RecordingTasks(
            TaskResourceConflict(f"map-dir:{map_dir.resolve()}", active)
        )

        status, _, raw = self.request(
            "POST", "/api/analyses/start", request, tasks=tasks
        )

        payload = self.json_body(raw)
        self.assertEqual(status, 409)
        self.assertIn("selected map folder", str(payload["error"]).lower())
        self.assertEqual(payload["active_task"]["task_id"], "active-map-build")
        self.assertEqual(payload["analysis"]["status"]["phase"], "blocked")

    def test_task_conflict_marks_new_analysis_terminal(self) -> None:
        bag = write_bag(
            self.record_root / "conflict-run",
            {
                "/camera": ("sensor_msgs/msg/Image", 30),
                "/visual_slam/tracking/odometry": ("nav_msgs/msg/Odometry", 30),
            },
        )
        active = _StartedTask("active-analysis")
        tasks = _RecordingTasks(TaskResourceConflict(f"analysis-bag:{bag.resolve()}", active))
        status, _, raw = self.request(
            "POST", "/api/analyses", self.recorded_payload(bag), tasks=tasks
        )

        payload = self.json_body(raw)
        self.assertEqual(status, 409)
        self.assertEqual(payload["analysis"]["status"]["status"], "failed")
        self.assertEqual(payload["analysis"]["status"]["phase"], "blocked")
        self.assertEqual(payload["active_task"]["task_id"], "active-analysis")

    def test_task_start_failure_marks_new_analysis_failed(self) -> None:
        bag = write_bag(
            self.record_root / "failed-start-run",
            {
                "/camera": ("sensor_msgs/msg/Image", 30),
                "/visual_slam/tracking/odometry": ("nav_msgs/msg/Odometry", 30),
            },
        )
        status, _, raw = self.request(
            "POST",
            "/api/analyses/start",
            self.recorded_payload(bag),
            tasks=_RecordingTasks(RuntimeError("thread unavailable")),
        )

        payload = self.json_body(raw)
        self.assertEqual(status, 500)
        self.assertEqual(payload["analysis"]["status"]["status"], "failed")
        self.assertEqual(payload["analysis"]["status"]["phase"], "failed")
        self.assertIn("thread unavailable", payload["analysis"]["status"]["message"])


if __name__ == "__main__":
    unittest.main()
