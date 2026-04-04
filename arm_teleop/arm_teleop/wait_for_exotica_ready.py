#!/usr/bin/env python3
from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


class ExoticaReadyWaiter(Node):
    def __init__(self):
        super().__init__("wait_for_exotica_ready")
        self.declare_parameter("timeout_sec", 90.0)
        self._timeout_sec = max(float(self.get_parameter("timeout_sec").value), 1.0)
        self._ready = threading.Event()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/exotica/ready", self._ready_cb, qos)

    def _ready_cb(self, msg: Bool):
        if bool(msg.data):
            self._ready.set()

    def wait(self) -> bool:
        self.get_logger().info(
            f"Waiting for /exotica/ready for up to {self._timeout_sec:.0f}s before starting teleop..."
        )
        deadline = self.get_clock().now().nanoseconds / 1e9 + self._timeout_sec
        while rclpy.ok() and not self._ready.is_set():
            if self.get_clock().now().nanoseconds / 1e9 >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._ready.is_set()


def main(args=None):
    rclpy.init(args=args)
    node = ExoticaReadyWaiter()
    exit_code = 0
    try:
        if not node.wait():
            node.get_logger().error("Timed out waiting for /exotica/ready.")
            exit_code = 1
        else:
            node.get_logger().info("/exotica/ready received. Teleop launch may continue.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
