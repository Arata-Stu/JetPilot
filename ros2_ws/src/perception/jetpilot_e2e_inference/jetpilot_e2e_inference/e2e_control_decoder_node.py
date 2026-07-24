#!/usr/bin/env python3

from __future__ import annotations

import time

import numpy as np
import rclpy
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
        self.last_publish_monotonic = 0.0

        self.tensor_sub = self.create_subscription(TensorList, "tensor_sub", self.tensor_callback, 10)
        self.command_pub = self.create_publisher(ControlCommand, "control_cmd", 10)

    def tensor_callback(self, msg: TensorList) -> None:
        tensor = next((item for item in msg.tensors if item.name == self.output_tensor_name), None)
        if tensor is None:
            self.get_logger().warn(f"Tensor not found: {self.output_tensor_name}")
            return

        values = np.frombuffer(bytes(tensor.data), dtype=np.float32).reshape(-1)
        if values.size < len(self.output_fields):
            self.get_logger().warn(
                f"Tensor has {values.size} values, expected {len(self.output_fields)}"
            )
            return

        decoded = {field: float(values[index]) for index, field in enumerate(self.output_fields)}
        cmd = ControlCommand()
        cmd.header = msg.header
        cmd.header.frame_id = cmd.header.frame_id or "base_link"
        cmd.steering = float(
            np.clip(decoded.get("steering", 0.0), self.steering_min, self.steering_max)
        )
        cmd.throttle = float(
            np.clip(decoded.get("throttle", 0.0), self.throttle_min, self.throttle_max)
        )
        cmd.brake = 0.0
        cmd.reverse = 0.0
        self.command_pub.publish(cmd)
        self.last_publish_monotonic = time.monotonic()


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
