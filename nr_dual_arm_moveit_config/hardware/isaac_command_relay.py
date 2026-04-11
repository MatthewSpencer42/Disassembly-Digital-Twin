#!/usr/bin/env python3
"""Isaac Sim command relay for pure Isaac mode.

Subscribes to a URDF-coordinate command topic and republishes to the raw Isaac
command topic after converting uf_slide_joint into Isaac coordinates.

Coordinate convention:
  uf_slide_joint URDF range:  0.054 – 0.75 m
  uf_slide_joint Isaac range: 0.0   – 0.70 m
  isaac_val = urdf_val - SLIDE_OFFSET
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

SLIDE_OFFSET: float = 0.054
SLIDE_MIN: float = 0.0
SLIDE_MAX: float = 0.7


class IsaacCommandRelay(Node):
    def __init__(self) -> None:
        super().__init__("isaac_command_relay")
        self.declare_parameter("input_topic", "/isaac_joint_commands_urdf")
        self.declare_parameter("output_topic", "/isaac_joint_commands")
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(JointState, output_topic, 10)
        self._sub = self.create_subscription(JointState, input_topic, self._cb, 10)
        self.get_logger().info(
            f"Isaac command relay active — {input_topic} -> {output_topic}"
        )

    def _cb(self, msg: JointState) -> None:
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position) if msg.position else []
        out.velocity = list(msg.velocity) if msg.velocity else []
        out.effort = list(msg.effort) if msg.effort else []
        if out.position:
            for i, name in enumerate(out.name):
                if name == "uf_slide_joint" and i < len(out.position):
                    out.position[i] = max(
                        SLIDE_MIN,
                        min(SLIDE_MAX, out.position[i] - SLIDE_OFFSET),
                    )
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = IsaacCommandRelay()
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
