#!/usr/bin/env python3
"""Isaac Sim state relay for digital-twin (real+isaac) mode.

Subscribes to /robot_joint_states (real hardware, URDF coordinate system)
and republishes to /isaac_joint_commands so Isaac Sim mirrors the real robot
in real time.

Coordinate convention:
  uf_slide_joint URDF range: 0.054 – 0.75 m
  uf_slide_joint Isaac range: 0.0   – 0.70 m
  isaac_val = rviz_val - SLIDE_OFFSET  (SLIDE_OFFSET = 0.054)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

SLIDE_OFFSET: float = 0.054


class IsaacStateRelay(Node):
    def __init__(self) -> None:
        super().__init__('isaac_state_relay')
        self._pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)
        self._sub = self.create_subscription(
            JointState, '/robot_joint_states', self._cb, 10)
        self.get_logger().info(
            'Isaac state relay active — mirroring real robot to Isaac Sim.')

    def _cb(self, msg: JointState) -> None:
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position) if msg.position else []
        out.velocity = list(msg.velocity) if msg.velocity else []
        out.effort = list(msg.effort) if msg.effort else []
        # Convert uf_slide_joint from URDF coords to Isaac coords.
        if out.position:
            for i, name in enumerate(out.name):
                if name == 'uf_slide_joint' and i < len(out.position):
                    out.position[i] -= SLIDE_OFFSET
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = IsaacStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
