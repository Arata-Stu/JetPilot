#!/usr/bin/env python3

from __future__ import annotations

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from isaac_ros_tensor_list_interfaces.msg import TensorList
from jetpilot_msgs.msg import ControlCommand
from rclpy.node import Node


class E2EControlDecoderNode(Node):
    def __init__(self) -> None:
        super().__init__("e2e_control_decoder")
        self.output_tensor_name = str(
            self.declare_parameter("output_tensor_name", "output_tensor").value
        )
        self.output_fields = [
            str(value) for value in self.declare_parameter(
                "output_fields", ["steering", "throttle"]
            ).value
        ]
        self.steering_min = float(self.declare_parameter("steering_min", -1.0).value)
        self.steering_max = float(self.declare_parameter("steering_max", 1.0).value)
        self.throttle_min = float(self.declare_parameter("throttle_min", 0.0).value)
        self.throttle_max = float(self.declare_parameter("throttle_max", 1.0).value)
        self.stale_timeout_sec = float(self.declare_parameter("stale_timeout_sec", 0.2).value)
        self.deadline_ms = float(self.declare_parameter("deadline_ms", 33.3).value)
        self.diagnostics_topic = str(
            self.declare_parameter("diagnostics_topic", "/e2e/diagnostics").value
        )
        self.last_publish_monotonic = 0.0
        self.sequence = 0

        self.tensor_sub = self.create_subscription(TensorList, "tensor_sub", self.tensor_callback, 10)
        self.command_pub = self.create_publisher(ControlCommand, "control_cmd", 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.diagnostics_topic, 10
        )

    @staticmethod
    def _diagnostic_value(key: str, value: object) -> KeyValue:
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

    def _publish_diagnostics(
        self,
        msg: TensorList,
        *,
        callback_ms: float,
        output_interval_ms: float | None,
    ) -> None:
        now = self.get_clock().now()
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        capture_to_command_ms = (
            max(0.0, (now.nanoseconds - stamp_ns) / 1.0e6) if stamp_ns > 0 else None
        )
        deadline_value = capture_to_command_ms if capture_to_command_ms is not None else callback_ms
        missed_deadline = deadline_value > self.deadline_ms

        status = DiagnosticStatus()
        status.name = "jetpilot_e2e_inference/pipeline"
        status.hardware_id = "jetpilot-e2e"
        status.level = DiagnosticStatus.WARN if missed_deadline else DiagnosticStatus.OK
        status.message = "deadline missed" if missed_deadline else "ok"
        status.values = [
            self._diagnostic_value("capture_to_command_ms", capture_to_command_ms if capture_to_command_ms is not None else ""),
            self._diagnostic_value("decoder_callback_ms", round(callback_ms, 6)),
            self._diagnostic_value("output_interval_ms", round(output_interval_ms, 6) if output_interval_ms is not None else ""),
            self._diagnostic_value("deadline_ms", self.deadline_ms),
            self._diagnostic_value("missed_deadline", int(missed_deadline)),
            self._diagnostic_value("sequence", self.sequence),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = now.to_msg()
        diagnostics.status = [status]
        self.diagnostics_pub.publish(diagnostics)

    def tensor_callback(self, msg: TensorList) -> None:
        callback_started = time.perf_counter_ns()
        publish_monotonic = time.monotonic()
        output_interval_ms = (
            (publish_monotonic - self.last_publish_monotonic) * 1000.0
            if self.last_publish_monotonic > 0.0
            else None
        )
        tensor = next((item for item in msg.tensors if item.name == self.output_tensor_name), None)
        if tensor is None:
            self.get_logger().warn(f"Tensor not found: {self.output_tensor_name}")
            return

        try:
            values = memoryview(tensor.data).cast("f")
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"Tensor data is not a contiguous float32 buffer: {exc}")
            return

        if len(values) < len(self.output_fields):
            self.get_logger().warn(
                f"Tensor has {len(values)} values, expected {len(self.output_fields)}"
            )
            return

        decoded = {field: float(values[index]) for index, field in enumerate(self.output_fields)}
        if not all(math.isfinite(value) for value in decoded.values()):
            self.get_logger().warn("Tensor contains a non-finite control value")
            return

        cmd = ControlCommand()
        cmd.header = msg.header
        cmd.header.frame_id = cmd.header.frame_id or "base_link"
        cmd.steering = min(
            max(decoded.get("steering", 0.0), self.steering_min),
            self.steering_max,
        )
        cmd.throttle = min(
            max(decoded.get("throttle", 0.0), self.throttle_min),
            self.throttle_max,
        )
        cmd.brake = 0.0
        cmd.reverse = 0.0
        self.command_pub.publish(cmd)
        callback_ms = (time.perf_counter_ns() - callback_started) / 1.0e6
        self.sequence += 1
        self._publish_diagnostics(
            msg,
            callback_ms=callback_ms,
            output_interval_ms=output_interval_ms,
        )
        self.last_publish_monotonic = publish_monotonic


def main() -> None:
    rclpy.init()
    node = E2EControlDecoderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
