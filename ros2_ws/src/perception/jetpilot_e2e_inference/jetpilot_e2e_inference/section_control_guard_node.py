#!/usr/bin/env python3
"""Gate section heads on localization health, fresh TF and stable recovery."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from jetpilot_e2e_inference.section_guard import SectionGuard


class SectionControlGuard(Node):
    def __init__(self):
        super().__init__("section_control_guard")
        self.frame = self.declare_parameter("map_frame", "map").value
        self.base = self.declare_parameter("base_frame", "base_link").value
        self.timeout = float(self.declare_parameter("section_timeout_sec", .3).value)
        self.health_timeout = float(self.declare_parameter("health_timeout_sec", 2.).value)
        self.guard = SectionGuard(
            float(self.declare_parameter("recovery_stable_sec", 1.).value),
            float(self.declare_parameter("section_switch_sec", .15).value),
            float(self.declare_parameter("max_plausible_speed_mps", 8.).value),
            float(self.declare_parameter("jump_margin_m", .5).value))
        self.section, self.localized = "unknown", False
        self.section_time = self.health_time = 0.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/localization/current_section", self.on_section, qos)
        self.create_subscription(String, "/localization/pose_hint_state", self.on_health, qos)
        self.publisher = self.create_publisher(String, "/e2e/validated_section", 1)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(.05, self.tick)

    def on_section(self, msg):
        self.section, self.section_time = msg.data, time.monotonic()

    def on_health(self, msg):
        try:
            self.localized = json.loads(msg.data).get("state") == "localized"
        except (ValueError, AttributeError):
            self.localized = False
        self.health_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        valid = self.localized and now-self.health_time < self.health_timeout and now-self.section_time < self.timeout
        pose, stamp = None, 0.
        try:
            tf = self.buffer.lookup_transform(self.frame, self.base, Time())
            stamp = tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
            age = self.get_clock().now().nanoseconds/1e9-stamp
            valid = valid and -.05 <= age <= self.timeout
            t = tf.transform.translation
            pose = (t.x,t.y,t.z)
        except Exception:
            valid = False
        message = String()
        message.data = self.guard.update(now, self.section, valid, pose, stamp)
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = SectionControlGuard()
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
