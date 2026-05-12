#!/usr/bin/env python3
import sys

sys.path = [path for path in sys.path if "/.local/lib/python" not in path]

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class DebugFeedRepublisher(Node):
    def __init__(self):
        super().__init__("debug_feed_republisher")
        self.publisher = self.create_publisher(Image, "/vision/debug_feed_view", 10)
        self.create_subscription(
            CompressedImage,
            "/vision/debug_feed/compressed",
            self._callback,
            10,
        )
        self.get_logger().info(
            "Republishing /vision/debug_feed/compressed to /vision/debug_feed_view"
        )

    def _callback(self, msg: CompressedImage):
        try:
            encoded = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if cv_image is None:
                raise ValueError("JPEG decode returned no image")

            image_msg = Image()
            image_msg.header = msg.header
            image_msg.height = int(cv_image.shape[0])
            image_msg.width = int(cv_image.shape[1])
            image_msg.encoding = "bgr8"
            image_msg.is_bigendian = 0
            image_msg.step = int(cv_image.shape[1] * 3)
            image_msg.data = np.ascontiguousarray(cv_image).tobytes()
            self.publisher.publish(image_msg)
        except Exception as exc:
            self.get_logger().warning(f"Failed to republish debug feed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = DebugFeedRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
