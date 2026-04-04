#!/usr/bin/env python3
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class DebugFeedRepublisher(Node):
    def __init__(self):
        super().__init__("debug_feed_republisher")
        self.bridge = CvBridge()
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
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            image_msg.header = msg.header
            self.publisher.publish(image_msg)
        except Exception as exc:
            self.get_logger().warning(f"Failed to republish debug feed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = DebugFeedRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
