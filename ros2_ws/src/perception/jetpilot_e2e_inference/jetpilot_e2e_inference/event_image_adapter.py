#!/usr/bin/env python3
"""Prepare event frames for the same RGB preprocessing used during training."""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class EventImageAdapter(Node):
    def __init__(self):
        super().__init__("e2e_event_image_adapter")
        self.width = int(self.declare_parameter("width", 212).value)
        self.height = int(self.declare_parameter("height", 120).value)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be positive")
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "image", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "camera_info", qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, "event_image", self.on_image, qos_profile_sensor_data
        )

    def on_image(self, message):
        try:
            # Match rosbag extraction: resize BGR with INTER_AREA, then convert to RGB.
            bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            resized = cv2.resize(bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            image = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
            image.header = message.header
            # The encoder only uses this message for exact timestamp synchronization.
            # K remains zero: this is explicitly uncalibrated, not camera calibration.
            info = CameraInfo()
            info.header = message.header
            info.width = self.width
            info.height = self.height
            self.info_pub.publish(info)
            self.image_pub.publish(image)
        except Exception as error:
            self.get_logger().error(f"Event image conversion failed: {error}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = EventImageAdapter()
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
