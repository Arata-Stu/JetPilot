#!/usr/bin/env python3

import os
import signal
import subprocess
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
from jetpilot_msgs.msg import BagRequest, BagStatus
from rclpy.node import Node
from rclpy.parameter import Parameter


DEFAULT_TOPICS = [
    "/joy",
    "/rc/channels",
    "/teleop/control_cmd",
    "/propo/control_cmd",
    "/auto/control_cmd",
    "/operation_mode/request",
    "/operation_mode/state",
    "/vehicle/control_cmd",
    "/bag/request",
    "/bag/status",
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
        self.max_bag_duration = int(self.declare_parameter("max_bag_duration", 0).value)
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

        self.process: Optional[subprocess.Popen] = None
        self.current_uri = ""
        self.last_event = "idle"

        self.request_sub = self.create_subscription(
            BagRequest, "/bag/request", self.handle_request, 10
        )
        self.status_pub = self.create_publisher(BagStatus, "/bag/status", 10)
        self.timer = self.create_timer(self.status_period_s, self.publish_status)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    @property
    def recording(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def handle_request(self, request: BagRequest) -> None:
        if request.command == BagRequest.START:
            self.start_recording(request.label)
        elif request.command == BagRequest.STOP:
            self.stop_recording("stop requested")
        elif request.command == BagRequest.SPLIT:
            self.last_event = "split requested but not implemented"
            self.get_logger().warn(self.last_event)
        elif request.command == BagRequest.MARK:
            self.last_event = f"mark: {request.label}"
            self.get_logger().info(self.last_event)
        else:
            self.last_event = f"unknown request: {request.command}"
            self.get_logger().warn(self.last_event)
        self.publish_status()

    def start_recording(self, label: str) -> None:
        if self.recording:
            self.last_event = "start ignored: already recording"
            self.get_logger().warn(self.last_event)
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in label).strip("_")
        name = f"{stamp}_{safe_label}" if safe_label else stamp
        self.current_uri = str(self.output_dir / name)

        command = self.build_record_command(self.current_uri)

        if not self.record_all and not self.topics:
            self.last_event = "start ignored: no topics configured"
            self.get_logger().error(self.last_event)
            return

        self.process = subprocess.Popen(command, preexec_fn=os.setsid)
        self.last_event = "recording started"
        self.get_logger().info(f"Started rosbag recording: {self.current_uri}")

    def build_record_command(self, output_uri: str) -> List[str]:
        command = ["ros2", "bag", "record", "-o", output_uri]
        if self.storage_id:
            command.extend(["--storage", self.storage_id])
        if self.serialization_format:
            command.extend(["--serialization-format", self.serialization_format])
        if self.max_bag_size > 0:
            command.extend(["--max-bag-size", str(self.max_bag_size)])
        if self.max_bag_duration > 0:
            command.extend(["--max-bag-duration", str(self.max_bag_duration)])
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

        assert self.process is not None
        os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        try:
            self.process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=5.0)
        self.process = None
        self.last_event = reason
        self.get_logger().info(f"Stopped rosbag recording: {self.current_uri}")

    def publish_status(self) -> None:
        msg = BagStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "bag_manager"
        msg.recording = self.recording
        msg.current_uri = self.current_uri
        msg.last_event = self.last_event
        msg.message = "recording" if msg.recording else "idle"
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self.recording:
            self.stop_recording("node shutdown")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BagManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
