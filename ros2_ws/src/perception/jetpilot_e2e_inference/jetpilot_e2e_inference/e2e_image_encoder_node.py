#!/usr/bin/env python3

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from isaac_ros_tensor_list_interfaces.msg import Tensor, TensorList, TensorShape
from rclpy.node import Node
from sensor_msgs.msg import Image


FP32_DATA_TYPE = 9
FLOAT32_BYTES = 4


class E2EImageEncoderNode(Node):
    def __init__(self) -> None:
        super().__init__("e2e_image_encoder")
        self.input_width = int(self.declare_parameter("input_width", 212).value)
        self.input_height = int(self.declare_parameter("input_height", 120).value)
        self.input_tensor_name = str(self.declare_parameter("input_tensor_name", "input_tensor").value)
        self.mean = np.asarray(
            self.declare_parameter("mean", [0.485, 0.456, 0.406]).value,
            dtype=np.float32,
        ).reshape(1, 1, 3)
        self.std = np.asarray(
            self.declare_parameter("std", [0.229, 0.224, 0.225]).value,
            dtype=np.float32,
        ).reshape(1, 1, 3)

        self.image_sub = self.create_subscription(Image, "image", self.image_callback, 10)
        self.tensor_pub = self.create_publisher(TensorList, "tensor_pub", 10)

    def image_to_bgr(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if encoding in ("rgb8", "bgr8"):
            channels = 3
            row_bytes = msg.width * channels
            image = data.reshape(msg.height, msg.step)[:, :row_bytes].reshape(
                msg.height, msg.width, channels
            )
            if encoding == "rgb8":
                return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image
        if encoding in ("mono8", "8uc1"):
            mono = data.reshape(msg.height, msg.step)[:, : msg.width].reshape(msg.height, msg.width)
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        if encoding in ("mono16", "16uc1"):
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint16)
            pixels_per_row = msg.step // 2
            mono16 = raw.reshape(msg.height, pixels_per_row)[:, : msg.width].reshape(
                msg.height, msg.width
            )
            mono8 = cv2.convertScaleAbs(mono16, alpha=255.0 / max(float(mono16.max()), 1.0))
            return cv2.cvtColor(mono8, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    def image_callback(self, msg: Image) -> None:
        try:
            image_bgr = self.image_to_bgr(msg)
            resized = cv2.resize(
                image_bgr,
                (self.input_width, self.input_height),
                interpolation=cv2.INTER_AREA,
            )
            image_rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            normalized = (image_rgb - self.mean) / self.std
            nchw = np.transpose(normalized, (2, 0, 1))[None, ...].astype(np.float32)
        except Exception as exc:
            self.get_logger().warn(f"Failed to encode image: {exc}")
            return

        tensor = Tensor()
        tensor.name = self.input_tensor_name
        tensor.shape = TensorShape()
        tensor.shape.rank = 4
        tensor.shape.dims = [1, 3, self.input_height, self.input_width]
        tensor.data_type = FP32_DATA_TYPE
        tensor.strides = [
            3 * self.input_height * self.input_width * FLOAT32_BYTES,
            self.input_height * self.input_width * FLOAT32_BYTES,
            self.input_width * FLOAT32_BYTES,
            FLOAT32_BYTES,
        ]
        tensor.data = nchw.tobytes()

        tensor_list = TensorList()
        tensor_list.header = msg.header
        tensor_list.tensors = [tensor]
        self.tensor_pub.publish(tensor_list)


def main() -> None:
    rclpy.init()
    node = E2EImageEncoderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
