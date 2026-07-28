#!/usr/bin/env python3

import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from jetpilot_msgs.msg import BagRequest, BagStatus
from rclpy.node import Node
from rclpy.parameter import Parameter


DEFAULT_TOPICS = [
    # Control and operation state.
    "/joy",
    "/rc/channels",
    "/teleop/control_cmd",
    "/propo/control_cmd",
    "/auto/control_cmd",
    "/vehicle/control_cmd",
    "/operation_mode/request",
    "/operation_mode/state",
    "/bag/request",
    "/bag/status",
    # Vehicle interface feedback.
    "/commands/motor/speed",
    "/commands/motor/brake",
    "/commands/servo/position",
    "/sensors/core",
    "/sensors/imu",
    "/sensors/imu/raw",
    "/sensors/servo_position_command",
    # TF and robot description.
    "/tf",
    "/tf_static",
    "/robot_description",
    "/joint_states",
    # Localization output/state retained for online and offline drive analysis.
    "/visual_slam/tracking/odometry",
    "/localization/pose_hint_state",
    "/localization/pose_hint_required",
    "/localization/diagnostics",
    "/planning/diagnostics",
    "/controller/diagnostics",
    "/jetson/diagnostics",
    # SilkyEvCam event camera.
    "/event_camera/camera_info",
    "/event_camera/events",
    "/event_camera/events_raw",
    "/event_camera/event_image",
    "/event_camera/raw_recording/request",
    "/event_camera/diagnostics",
    # FLIR Boson thermal camera. Keep raw mono16 for offline analysis.
    "/flir/camera_info",
    "/flir/image_raw",
    # RealSense RGB candidates. CameraInfo is required for offline VSLAM.
    "/realsense/color/camera_info",
    "/realsense/color/image_raw",
    "/realsense/color/image_raw/compressed",
    "/realsense/color/metadata",
    # RealSense IMU.
    "/realsense/accel/imu_info",
    "/realsense/accel/metadata",
    "/realsense/accel/sample",
    "/realsense/gyro/imu_info",
    "/realsense/gyro/metadata",
    "/realsense/gyro/sample",
    "/realsense/extrinsics/depth_to_accel",
    "/realsense/extrinsics/depth_to_gyro",
    # RealSense stereo infrared candidates. CameraInfo is required for offline VSLAM.
    "/realsense/infra1/camera_info",
    "/realsense/infra1/image_rect_raw",
    "/realsense/infra1/image_rect_raw/compressed",
    "/realsense/infra1/image_rect_raw/compressedDepth",
    "/realsense/infra1/image_rect_raw/zstd",
    "/realsense/infra1/metadata",
    "/realsense/infra2/camera_info",
    "/realsense/infra2/image_rect_raw",
    "/realsense/infra2/image_rect_raw/compressed",
    "/realsense/infra2/image_rect_raw/compressedDepth",
    "/realsense/infra2/image_rect_raw/zstd",
    "/realsense/infra2/metadata",
    "/realsense/extrinsics/depth_to_infra1",
    "/realsense/extrinsics/depth_to_infra2",
]


def declare_string_array_parameter(
    node: Node, name: str, default_value: Optional[List[str]] = None
) -> List[str]:
    parameter = node.declare_parameter(
        name,
        default_value if default_value is not None else Parameter.Type.STRING_ARRAY,
    )
    if parameter.value is None:
        return []
    return [str(item) for item in parameter.value]


class BagManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("bag_manager_node")
        self.output_dir = Path(self.declare_parameter("output_dir", "/tmp/jetpilot_bags").value)
        self.record_all = bool(self.declare_parameter("record_all", True).value)
        self.topics = declare_string_array_parameter(self, "topics", DEFAULT_TOPICS)
        self.exclude_topics = declare_string_array_parameter(self, "exclude_topics")
        self.storage_id = str(self.declare_parameter("storage_id", "mcap").value)
        self.serialization_format = str(self.declare_parameter("serialization_format", "").value)
        self.max_bag_size = int(self.declare_parameter("max_bag_size", 0).value)
        self.recording_split_duration_s = int(
            self.declare_parameter("recording_split_duration_s", 0).value
        )
        legacy_max_bag_duration = int(
            self.declare_parameter("max_bag_duration", 0).value
        )
        self.max_cache_size = int(self.declare_parameter("max_cache_size", 0).value)
        self.compression_mode = str(self.declare_parameter("compression_mode", "").value)
        self.compression_format = str(self.declare_parameter("compression_format", "").value)
        self.compression_queue_size = int(
            self.declare_parameter("compression_queue_size", 0).value
        )
        self.compression_threads = int(self.declare_parameter("compression_threads", 0).value)
        self.qos_profile_overrides_path = str(
            self.declare_parameter("qos_profile_overrides_path", "").value
        )
        self.include_hidden_topics = bool(
            self.declare_parameter("include_hidden_topics", False).value
        )
        self.no_discovery = bool(self.declare_parameter("no_discovery", False).value)
        self.snapshot_mode = bool(self.declare_parameter("snapshot_mode", False).value)
        self.start_paused = bool(self.declare_parameter("start_paused", False).value)
        self.extra_args = declare_string_array_parameter(self, "extra_args")
        self.status_period_s = float(self.declare_parameter("status_period_s", 1.0).value)
        self.raw_recording_request_topic = str(
            self.declare_parameter(
                "raw_recording_request_topic",
                "/event_camera/raw_recording/request",
            ).value
        )
        self.recording_start_timeout_s = float(
            self.declare_parameter("recording_start_timeout_s", 5.0).value
        )
        if self.recording_split_duration_s < 0:
            raise ValueError("recording_split_duration_s must be non-negative")
        if legacy_max_bag_duration != 0:
            raise ValueError(
                "max_bag_duration is no longer configurable; use "
                "recording_split_duration_s for coordinated bag/RAW splitting"
            )
        if any(
            argument == "--max-bag-duration"
            or argument == "-d"
            or argument.startswith("--max-bag-duration=")
            for argument in self.extra_args
        ):
            raise ValueError(
                "extra_args must not override --max-bag-duration; use "
                "recording_split_duration_s"
            )

        self.process: Optional[subprocess.Popen] = None
        self.current_uri = ""
        self.last_event = "idle"
        self.raw_recording_requested = False
        self.raw_split_timer = None

        self.request_sub = self.create_subscription(
            BagRequest, "/bag/request", self.handle_request, 10
        )
        self.status_pub = self.create_publisher(BagStatus, "/bag/status", 10)
        self.raw_recording_request_pub = (
            self.create_publisher(
                BagRequest,
                self.raw_recording_request_topic,
                10,
            )
            if self.raw_recording_request_topic
            else None
        )
        self.timer = self.create_timer(self.status_period_s, self.publish_status)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    @property
    def recording(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def handle_request(self, request: BagRequest) -> None:
        if request.command == BagRequest.START:
            already_recording = self.recording
            if self.start_recording(request.label):
                self.publish_raw_recording_request(
                    BagRequest.START,
                    self.current_uri,
                )
                if not already_recording:
                    self.start_raw_split_timer()
        elif request.command == BagRequest.STOP:
            self.stop_raw_split_timer()
            self.publish_raw_recording_request(BagRequest.STOP, request.label)
            self.stop_recording("stop requested")
        elif request.command == BagRequest.SPLIT:
            self.last_event = (
                "manual split ignored: automatic bag/raw splitting is controlled by "
                "recording_split_duration_s"
            )
            self.get_logger().warn(self.last_event)
        elif request.command == BagRequest.MARK:
            self.last_event = f"mark: {request.label}"
            self.get_logger().info(self.last_event)
            self.publish_raw_recording_request(BagRequest.MARK, request.label)
        else:
            self.last_event = f"unknown request: {request.command}"
            self.get_logger().warn(self.last_event)
        self.publish_status()

    def start_recording(self, label: str) -> bool:
        if self.recording:
            self.last_event = "start ignored: already recording"
            self.get_logger().warn(self.last_event)
            return True

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in label).strip("_")
        name = f"{stamp}_{safe_label}" if safe_label else stamp
        output_path = self.output_dir / name
        suffix = 1
        while output_path.exists():
            output_path = self.output_dir / f"{name}_{suffix:02d}"
            suffix += 1
        self.current_uri = str(output_path)

        command = self.build_record_command(self.current_uri)

        if not self.record_all and not self.topics:
            self.last_event = "start ignored: no topics configured"
            self.get_logger().error(self.last_event)
            return False

        self.process = subprocess.Popen(command, preexec_fn=os.setsid)
        if not self.wait_for_recording_directory():
            self.last_event = (
                "recording failed: rosbag output directory was not created: "
                f"{self.current_uri}"
            )
            self.get_logger().error(self.last_event)
            self.stop_recording_process()
            return False

        self.last_event = "recording started"
        self.get_logger().info(f"Started rosbag recording: {self.current_uri}")
        return True

    def wait_for_recording_directory(self) -> bool:
        deadline = time.monotonic() + max(0.0, self.recording_start_timeout_s)
        output_path = Path(self.current_uri)

        while self.recording:
            if output_path.is_dir():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)
        return False

    def publish_raw_recording_request(self, command: int, label: str) -> None:
        if self.raw_recording_request_pub is None:
            return

        request = BagRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "bag_manager"
        request.command = command
        request.label = label
        self.raw_recording_request_pub.publish(request)
        if command == BagRequest.START:
            self.raw_recording_requested = True
        elif command == BagRequest.STOP:
            self.raw_recording_requested = False

    def start_raw_split_timer(self) -> None:
        self.stop_raw_split_timer()
        if self.recording_split_duration_s <= 0:
            return
        self.raw_split_timer = self.create_timer(
            float(self.recording_split_duration_s),
            self.handle_scheduled_raw_split,
        )

    def stop_raw_split_timer(self) -> None:
        if self.raw_split_timer is None:
            return
        self.raw_split_timer.cancel()
        self.destroy_timer(self.raw_split_timer)
        self.raw_split_timer = None

    def handle_scheduled_raw_split(self) -> None:
        if not self.recording or not self.raw_recording_requested:
            self.stop_raw_split_timer()
            return
        self.publish_raw_recording_request(
            BagRequest.SPLIT,
            "scheduled_split",
        )

    def stop_recording_process(self) -> None:
        if not self.recording:
            self.process = None
            return

        assert self.process is not None
        os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        try:
            self.process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=5.0)
        self.process = None

    def build_record_command(self, output_uri: str) -> List[str]:
        command = ["ros2", "bag", "record", "-o", output_uri]
        if self.storage_id:
            command.extend(["--storage", self.storage_id])
        if self.serialization_format:
            command.extend(["--serialization-format", self.serialization_format])
        if self.max_bag_size > 0:
            command.extend(["--max-bag-size", str(self.max_bag_size)])
        if self.recording_split_duration_s > 0:
            command.extend(
                [
                    "--max-bag-duration",
                    str(self.recording_split_duration_s),
                ]
            )
        if self.max_cache_size > 0:
            command.extend(["--max-cache-size", str(self.max_cache_size)])
        if self.compression_mode:
            command.extend(["--compression-mode", self.compression_mode])
        if self.compression_format:
            command.extend(["--compression-format", self.compression_format])
        if self.compression_queue_size > 0:
            command.extend(["--compression-queue-size", str(self.compression_queue_size)])
        if self.compression_threads > 0:
            command.extend(["--compression-threads", str(self.compression_threads)])
        if self.qos_profile_overrides_path:
            command.extend(["--qos-profile-overrides-path", self.qos_profile_overrides_path])
        if self.include_hidden_topics:
            command.append("--include-hidden-topics")
        if self.no_discovery:
            command.append("--no-discovery")
        if self.snapshot_mode:
            command.append("--snapshot-mode")
        if self.start_paused:
            command.append("--start-paused")
        for topic in self.exclude_topics:
            command.extend(["--exclude", str(topic)])
        command.extend(self.extra_args)
        if self.record_all:
            command.append("-a")
        else:
            command.extend(self.topics)
        return command

    def stop_recording(self, reason: str) -> None:
        if not self.recording:
            self.process = None
            self.last_event = "stop ignored: not recording"
            self.get_logger().warn(self.last_event)
            return

        self.stop_recording_process()
        self.last_event = reason
        self.get_logger().info(f"Stopped rosbag recording: {self.current_uri}")

    def publish_status(self) -> None:
        if self.raw_recording_requested and not self.recording:
            self.stop_raw_split_timer()
            self.publish_raw_recording_request(
                BagRequest.STOP,
                "rosbag_process_stopped",
            )
            self.process = None
            self.last_event = "rosbag process stopped unexpectedly"
            self.get_logger().error(self.last_event)

        msg = BagStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "bag_manager"
        msg.recording = self.recording
        msg.current_uri = self.current_uri
        msg.last_event = self.last_event
        msg.message = "recording" if msg.recording else "idle"
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self.stop_raw_split_timer()
        if self.raw_recording_requested:
            self.publish_raw_recording_request(BagRequest.STOP, "node_shutdown")
        if self.recording:
            self.stop_recording("node shutdown")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BagManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
