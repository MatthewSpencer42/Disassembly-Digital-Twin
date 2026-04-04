#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from tf2_geometry_msgs import do_transform_pose_stamped

from dual_arm_moveit_config.motion_backend import MotionBackend as MoveItMotionBackend


class MotionBackend(MoveItMotionBackend):
    """Compatibility wrapper for the updated dual-arm MoveIt backend."""

    def stop_immediately(self):
        self._publish_zero_twist()
        if self.servo_pub is not None:
            msg = TwistStamped()
            msg.header.frame_id = "base_link"
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.servo_pub.publish(msg)

    def get_transformed_pose(self, source_pose, source_frame, target_frame, z_offset=0.0):
        if not isinstance(source_pose, Pose):
            return None
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = source_frame
        pose_stamped.header.stamp = rclpy.time.Time().to_msg()
        pose_stamped.pose = source_pose
        pose_stamped.pose.position.z += float(z_offset)
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
            return do_transform_pose_stamped(pose_stamped, transform)
        except Exception as exc:
            self.node.get_logger().warning(
                f"Failed to transform pose from {source_frame} to {target_frame}: {exc}"
            )
            return None


def main():
    raise SystemExit("disassembly_skill.motion_backend is a library module, not a standalone node")
