#!/usr/bin/env python3
"""Bridge legacy Isaac USD joint topics to the active ros2_control model."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from dual_arm_moveit_config.isaac_joint_mapping import (
    ACTIVE_TO_ISAAC_TRANSFORMS,
    ISAAC_TO_ACTIVE_TRANSFORMS,
    translate_joint_sample,
)


class IsaacJointAdapter(Node):
    def __init__(self) -> None:
        super().__init__("isaac_joint_adapter")

        self.declare_parameter("isaac_state_topic", "/isaac_joint_states")
        self.declare_parameter("isaac_command_topic", "/isaac_joint_commands")
        self.declare_parameter(
            "ros2_control_state_topic",
            "/isaac_joint_states_mapped",
        )
        self.declare_parameter(
            "ros2_control_command_topic",
            "/isaac_joint_commands_mapped",
        )
        self.declare_parameter("publish_clock", True)

        isaac_state_topic = self.get_parameter("isaac_state_topic").value
        isaac_command_topic = self.get_parameter("isaac_command_topic").value
        control_state_topic = self.get_parameter("ros2_control_state_topic").value
        control_command_topic = self.get_parameter("ros2_control_command_topic").value
        self._publish_clock = bool(self.get_parameter("publish_clock").value)

        self._state_pub = self.create_publisher(JointState, control_state_topic, 10)
        self._command_pub = self.create_publisher(JointState, isaac_command_topic, 10)
        self._clock_pub = (
            self.create_publisher(Clock, "/clock", 10)
            if self._publish_clock
            else None
        )
        self.create_subscription(
            JointState,
            isaac_state_topic,
            self._handle_isaac_state,
            10,
        )
        self.create_subscription(
            JointState,
            control_command_topic,
            self._handle_control_command,
            10,
        )

        self._warned_empty_state = False
        self._warned_empty_command = False
        self.get_logger().info(
            "Isaac joint adapter active: "
            f"{isaac_state_topic} -> {control_state_topic}, "
            f"{control_command_topic} -> {isaac_command_topic}, "
            f"publish_clock={self._publish_clock}"
        )

    @staticmethod
    def _translated_message(
        source: JointState,
        transforms,
    ) -> JointState | None:
        translated = translate_joint_sample(
            source.name,
            source.position,
            source.velocity,
            source.effort,
            transforms,
        )
        if not translated.names:
            return None

        output = JointState()
        output.header = source.header
        output.name = list(translated.names)
        output.position = list(translated.positions)
        output.velocity = list(translated.velocities)
        output.effort = list(translated.efforts)
        return output

    def _handle_isaac_state(self, msg: JointState) -> None:
        if self._clock_pub is not None:
            clock = Clock()
            clock.clock = msg.header.stamp
            self._clock_pub.publish(clock)

        output = self._translated_message(msg, ISAAC_TO_ACTIVE_TRANSFORMS)
        if output is None:
            if not self._warned_empty_state:
                self.get_logger().error(
                    "Isaac joint state contained none of the configured USD joint names"
                )
                self._warned_empty_state = True
            return
        self._state_pub.publish(output)

    def _handle_control_command(self, msg: JointState) -> None:
        output = self._translated_message(msg, ACTIVE_TO_ISAAC_TRANSFORMS)
        if output is None:
            if not self._warned_empty_command:
                self.get_logger().error(
                    "ros2_control command contained none of the active model joint names"
                )
                self._warned_empty_command = True
            return
        self._command_pub.publish(output)


def main() -> None:
    rclpy.init()
    node = IsaacJointAdapter()
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
