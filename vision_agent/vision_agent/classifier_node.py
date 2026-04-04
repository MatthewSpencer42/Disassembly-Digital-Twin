#!/usr/bin/env python3
import json
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from vision_agent.common import PATH_REFEREE, PERF_LOG_INTERVAL_SEC, PROCESSING_RATE_HZ, REFEREE_EVERY_N_FRAMES
from vision_agent.agents.referee import RefereeAgent


class ClassifierVisionNode(Node):
    def __init__(self):
        super().__init__("vision_classifier_node")
        self.get_logger().info("Starting classifier vision node")

        self.referee = RefereeAgent(PATH_REFEREE)
        self.bridge = CvBridge()
        self.frame_local = None
        self.frame_counter = 0
        self.last_status = {"state": "unknown", "confidence": 0.0}
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.referee_future = None
        self._perf_time = time.time()
        self._perf_stats = {"referee_ms": [0.0, 0]}

        self.create_subscription(
            CompressedImage,
            "/tool_cam/image_raw/compressed",
            self.cb_local,
            10,
        )
        self.state_pub = self.create_publisher(String, "/vision/assembly_state", 10)
        self.timer = self.create_timer(1.0 / PROCESSING_RATE_HZ, self.processing_loop)

    def cb_local(self, msg):
        try:
            self.frame_local = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _record_stage_time(self, elapsed_s):
        stat = self._perf_stats["referee_ms"]
        stat[0] += elapsed_s * 1000.0
        stat[1] += 1

    def _maybe_log_perf(self):
        now = time.time()
        if now - self._perf_time < PERF_LOG_INTERVAL_SEC:
            return
        total_ms, count = self._perf_stats["referee_ms"]
        avg_ms = (total_ms / count) if count else 0.0
        self.get_logger().info(f"Classifier perf: referee={avg_ms:.1f}ms")
        self._perf_stats["referee_ms"] = [0.0, 0]
        self._perf_time = now

    def _run_referee(self, frame):
        t0 = time.perf_counter()
        result = self.referee.inspect(frame)
        return result, time.perf_counter() - t0

    def _poll_future(self):
        if self.referee_future and self.referee_future.done():
            try:
                self.last_status, elapsed = self.referee_future.result()
                self._record_stage_time(elapsed)
            except Exception as e:
                self.get_logger().error(f"Referee Error: {e}")
            self.referee_future = None

    def processing_loop(self):
        self._poll_future()
        self._maybe_log_perf()
        if self.frame_local is None:
            return

        if self.frame_counter % REFEREE_EVERY_N_FRAMES == 0 and self.referee_future is None:
            self.referee_future = self.pool.submit(self._run_referee, self.frame_local.copy())

        packet = dict(self.last_status)
        packet["timestamp"] = self.get_clock().now().nanoseconds
        self.state_pub.publish(String(data=json.dumps(packet)))
        self.frame_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ClassifierVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
