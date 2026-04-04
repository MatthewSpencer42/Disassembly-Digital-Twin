#!/usr/bin/env python3
import json
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from vision_agent.common import (
    LOCAL_CROSSHAIR_OFFSET_X,
    LOCAL_CROSSHAIR_OFFSET_Y,
    LOCAL_INFERENCE_EVERY_N_FRAMES,
    PATH_SNIPER,
    PERF_LOG_INTERVAL_SEC,
    PROCESSING_RATE_HZ,
)
from vision_agent.agents.sniper import SniperAgent


class LocalVisionNode(Node):
    def __init__(self):
        super().__init__("vision_local_node")
        self.get_logger().info("Starting local vision node")

        self.sniper = SniperAgent(PATH_SNIPER)
        self.bridge = CvBridge()
        self.frame_local = None
        self.frame_counter = 0
        self.last_sniper_data = {"screw_heads": [], "tool_tips": [], "holes": [], "crosshair": []}
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.sniper_future = None
        self._perf_time = time.time()
        self._perf_stats = {"sniper_ms": [0.0, 0]}

        self.create_subscription(
            CompressedImage,
            "/tool_cam/image_raw/compressed",
            self.cb_local,
            10,
        )
        self.state_pub = self.create_publisher(String, "/vision/local_state", 10)
        self.timer = self.create_timer(1.0 / PROCESSING_RATE_HZ, self.processing_loop)

    def cb_local(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            if img.shape[1] > 640:
                img = cv2.resize(img, (640, 480))
            self.frame_local = img
        except Exception:
            pass

    def _record_stage_time(self, elapsed_s):
        stat = self._perf_stats["sniper_ms"]
        stat[0] += elapsed_s * 1000.0
        stat[1] += 1

    def _maybe_log_perf(self):
        now = time.time()
        if now - self._perf_time < PERF_LOG_INTERVAL_SEC:
            return
        total_ms, count = self._perf_stats["sniper_ms"]
        avg_ms = (total_ms / count) if count else 0.0
        self.get_logger().info(f"Local perf: sniper={avg_ms:.1f}ms")
        self._perf_stats["sniper_ms"] = [0.0, 0]
        self._perf_time = now

    def _run_sniper(self, frame):
        t0 = time.perf_counter()
        result = self.sniper.target(frame)
        return result, time.perf_counter() - t0

    def _poll_future(self):
        if self.sniper_future and self.sniper_future.done():
            try:
                self.last_sniper_data, elapsed = self.sniper_future.result()
                self._record_stage_time(elapsed)
            except Exception as e:
                self.get_logger().error(f"Sniper Error: {e}")
            self.sniper_future = None

    def processing_loop(self):
        self._poll_future()
        self._maybe_log_perf()
        if self.frame_local is None:
            return

        if self.frame_counter % LOCAL_INFERENCE_EVERY_N_FRAMES == 0 and self.sniper_future is None:
            self.sniper_future = self.pool.submit(self._run_sniper, self.frame_local.copy())

        packet = dict(self.last_sniper_data)
        h_loc, w_loc = self.frame_local.shape[:2]
        cross_x = (w_loc // 2) + LOCAL_CROSSHAIR_OFFSET_X
        cross_y = (h_loc // 2) + LOCAL_CROSSHAIR_OFFSET_Y
        packet["crosshair"] = [int(cross_x), int(cross_y)]
        packet["timestamp"] = self.get_clock().now().nanoseconds
        self.state_pub.publish(String(data=json.dumps(packet)))
        self.frame_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = LocalVisionNode()
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
