#!/usr/bin/env python3
"""Joint state filter for Isaac mode.

Subscribes to /joint_states, strips any joint names not in the robot model,
and republishes to /filtered_joint_states. In Isaac mode, raw USD names and
coordinates are normalized before ros2_control by isaac_joint_adapter.py.
This final filter protects MoveIt from any unexpected joint names.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# All joints registered in ros2_control.xacro + URDF for dual_arm_world.
VALID_JOINTS: frozenset = frozenset([
    'uf_slide_joint',
    'uf850_joint1', 'uf850_joint2', 'uf850_joint3',
    'uf850_joint4', 'uf850_joint5', 'uf850_joint6',
    'xarm5_joint1', 'xarm5_joint2', 'xarm5_joint3',
    'xarm5_joint4', 'xarm5_joint5',
    'rg6_right_drive_joint', 'rg6_right_inner_joint',
    'rg6_right_finger_joint', 'rg6_right_ignore_joint',
    'rg6_left_drive_joint', 'rg6_left_inner_joint',
    'rg6_left_finger_joint', 'rg6_left_ignore_joint',
])


class JointStateFilter(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_filter')
        self._pub = self.create_publisher(JointState, 'filtered_joint_states', 10)
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._cb, 10)
        self.get_logger().info('Joint state filter active — stripping unknown joints.')

    def _cb(self, msg: JointState) -> None:
        indices = [i for i, n in enumerate(msg.name) if n in VALID_JOINTS]
        if not indices:
            return
        out = JointState()
        out.header = msg.header
        out.name = [msg.name[i] for i in indices]
        if len(msg.position) >= len(msg.name):
            out.position = [msg.position[i] for i in indices]
        if len(msg.velocity) >= len(msg.name):
            out.velocity = [msg.velocity[i] for i in indices]
        if len(msg.effort) >= len(msg.name):
            out.effort = [msg.effort[i] for i in indices]
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = JointStateFilter()
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
