#!/usr/bin/env python3
import json
import math
import os
import threading
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TwistStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dual_arm_moveit_config.exotica_planner import ExoticaDualArmPlanner, ExoticaSingleArmPosePlanner, RemoteExoticaIKClient


class MotionBackend:
    _RG6_RAD_OPEN = -0.625
    _RG6_RAD_CLOSE = 0.625
    _RG6_MM_OPEN = 160.0
    _RG6_MM_CLOSE = 0.0

    _MIN_TCP_Z_LIMITS = {
        "rg6_tcp": 0.92962,
        "screwdriver_tcp": 0.91775,
    }
    _TCP_X_LIMITS = {
        "rg6_tcp": (0.647599, 1.24581),
        "screwdriver_tcp": (0.637258, 1.16441),
    }

    def __init__(self, node: Node, group_name: str):
        self.node = node
        self.group_name = group_name
        group_name_lower = group_name.lower()
        self.is_dual_arms = "dual_arms" in group_name_lower
        self.is_xarm5 = "xarm" in group_name_lower
        self.is_gripper = "rg6" in group_name_lower or "gripper" in group_name_lower
        self.is_uf850 = not self.is_dual_arms and not self.is_xarm5 and not self.is_gripper

        if self.is_dual_arms:
            self.backend_kind = "dual_arms"
            self.controller_name = None
            self.servo_namespace = None
            self.default_ik_link = None
            self.joint_prefixes = ("uf_slide_", "uf850_", "xarm5_", "xarm5_")
        elif self.is_xarm5:
            self.backend_kind = "xarm5"
            self.controller_name = "xarm5_controller"
            self.servo_namespace = "/xarm_servo_node"
            self.default_ik_link = "screwdriver_tcp"
            self.joint_prefixes = ("xarm5_",)
        elif self.is_gripper:
            self.backend_kind = "rg6_gripper"
            self.controller_name = "rg6_controller"
            self.servo_namespace = None
            self.default_ik_link = None
            self.joint_prefixes = ("rg6_",)
        else:
            self.backend_kind = "uf850"
            self.controller_name = "uf850_controller"
            self.servo_namespace = "/uf_servo_node"
            self.default_ik_link = "rg6_tcp"
            self.joint_prefixes = ("uf850_",)
        self.min_tcp_z = self._MIN_TCP_Z_LIMITS.get(self.default_ik_link)
        x_limits = self._TCP_X_LIMITS.get(self.default_ik_link)
        self.min_tcp_x = x_limits[0] if x_limits is not None else None
        self.max_tcp_x = x_limits[1] if x_limits is not None else None

        self._service_cb_group = ReentrantCallbackGroup()
        self._move_group_client = ActionClient(self.node, MoveGroup, "move_action")
        self._execute_client = ActionClient(self.node, ExecuteTrajectory, "execute_trajectory")
        self._ik_client = self.node.create_client(
            GetPositionIK, "compute_ik", callback_group=self._service_cb_group
        )
        self._cartesian_client = self.node.create_client(
            GetCartesianPath, "compute_cartesian_path", callback_group=self._service_cb_group
        )
        self._joint_command_pub = self.node.create_publisher(
            JointState, "/robot_joint_commands", 10
        )
        # Publisher for streaming direct joint commands via the JTC topic.
        # Arm streaming uses this path exclusively. RG6 commands also publish
        # here to overwrite any stale rg6_controller goal that could otherwise
        # fight the raw hardware command.
        self._joint_traj_stream_pub = None
        if (self.is_uf850 or self.is_xarm5 or self.is_gripper) and self.controller_name:
            self._joint_traj_stream_pub = self.node.create_publisher(
                JointTrajectory, f"/{self.controller_name}/joint_trajectory", 10
            )
        self._controller_switch_client = self.node.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
            callback_group=self._service_cb_group,
        )
        self._controller_list_client = self.node.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
            callback_group=self._service_cb_group,
        )

        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.current_joint_efforts = {}
        self.state_received = threading.Event()
        self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
            callback_group=self._service_cb_group,
        )
        self.node.create_subscription(
            JointState,
            "/robot_joint_states",
            self._joint_state_callback,
            10,
            callback_group=self._service_cb_group,
        )

        self.current_gripper_state = {}
        self.current_gripper_force_n = 40.0
        self._gripper_force_pub = None
        if self.is_gripper:
            self._gripper_force_pub = self.node.create_publisher(Float32, "/rg6/force_command", 10)
            self.node.create_subscription(String, "/rg6/state", self._gripper_state_callback, 10)

        self.servo_pub = None
        self._servo_start_client = None
        self._servo_stop_client = None
        if self.servo_namespace is not None:
            self.servo_pub = self.node.create_publisher(
                TwistStamped, f"{self.servo_namespace}/delta_twist_cmds", 10
            )
            self._servo_start_client = self.node.create_client(
                Trigger,
                f"{self.servo_namespace}/start_servo",
                callback_group=self._service_cb_group,
            )
            self._servo_stop_client = self.node.create_client(
                Trigger,
                f"{self.servo_namespace}/stop_servo",
                callback_group=self._service_cb_group,
            )

        if not hasattr(self.node, "_shared_tf_buffer"):
            self.node._shared_tf_buffer = Buffer()
            self.node._shared_tf_listener = TransformListener(self.node._shared_tf_buffer, self.node)
        self.tf_buffer = self.node._shared_tf_buffer
        if not hasattr(self.node, "_display_trajectory_pub"):
            self.node._display_trajectory_pub = self.node.create_publisher(
                DisplayTrajectory, "/display_planned_path", 10
            )
        self._display_trajectory_pub = self.node._display_trajectory_pub

        self._in_servo_mode = False
        self._exotica_planner = None
        self._single_arm_exotica_planner = None
        hardware_type = "fake"
        try:
            hardware_type = self.node.get_parameter("hardware_type").value
        except Exception:
            pass
        # True for real/twin hardware — enables direct trajectory streaming that
        # bypasses the JTC→TopicBasedSystem→100Hz chain (which causes timing jitter
        # in the Python hardware bridge).
        self._is_real_hardware = str(hardware_type) in ("real", "twin")
        if self.is_dual_arms:
            self._exotica_planner = ExoticaDualArmPlanner(self.node, hardware_type=str(hardware_type))
            if not self._exotica_planner.available:
                self.node.get_logger().warning(
                    f"EXOTica dual-arm planner unavailable, falling back to MoveIt OMPL: {self._exotica_planner.last_error}"
                )
        elif self.is_xarm5 or self.is_uf850:
            exotica_group = "xarm5_arm_no_slide" if self.is_xarm5 else "uf850_arm"
            # Try to use the pre-warmed EXOTica IK server (started with MoveIt launch)
            # to avoid 25-30s local initialization cost per node.
            if self.node.count_publishers('/exotica/ready') > 0:
                self.node.get_logger().info(
                    f"[{self.backend_kind}] EXOTica IK server detected — using RemoteExoticaIKClient"
                )
                self._single_arm_exotica_planner = RemoteExoticaIKClient(
                    self.node, exotica_group, hardware_type=str(hardware_type)
                )
            else:
                self.node.get_logger().info(
                    f"[{self.backend_kind}] No EXOTica IK server found — initializing local planner"
                )
                self._single_arm_exotica_planner = ExoticaSingleArmPosePlanner(
                    self.node, exotica_group, hardware_type=str(hardware_type)
                )
            if not self._single_arm_exotica_planner.available:
                self.node.get_logger().warning(
                    f"[{self.backend_kind}] EXOTica single-arm planner unavailable: "
                    f"{self._single_arm_exotica_planner.last_error}"
                )

    def _clamp_target_z(self, requested_z: float, context: str) -> float:
        if self.min_tcp_z is None:
            return float(requested_z)
        requested_z = float(requested_z)
        if requested_z < self.min_tcp_z:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] {context}: requested z={requested_z:.5f} is below "
                f"table limit z={self.min_tcp_z:.5f} for {self.default_ik_link}. Clamping."
            )
            return float(self.min_tcp_z)
        return requested_z

    def _clamp_target_x(self, requested_x: float, context: str) -> float:
        requested_x = float(requested_x)
        if self.min_tcp_x is not None and requested_x < self.min_tcp_x:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] {context}: requested x={requested_x:.5f} is below "
                f"backward limit x={self.min_tcp_x:.5f} for {self.default_ik_link}. Clamping."
            )
            return float(self.min_tcp_x)
        if self.max_tcp_x is not None and requested_x > self.max_tcp_x:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] {context}: requested x={requested_x:.5f} is above "
                f"forward limit x={self.max_tcp_x:.5f} for {self.default_ik_link}. Clamping."
            )
            return float(self.max_tcp_x)
        return requested_x

    def _joint_state_callback(self, msg: JointState):
        for index, name in enumerate(msg.name):
            if len(msg.position) > index:
                self.current_joint_positions[name] = msg.position[index]
            if len(msg.velocity) > index:
                self.current_joint_velocities[name] = msg.velocity[index]
            if len(msg.effort) > index:
                self.current_joint_efforts[name] = msg.effort[index]
        self.state_received.set()

    def _gripper_state_callback(self, msg: String):
        try:
            self.current_gripper_state = json.loads(msg.data)
        except Exception:
            self.current_gripper_state = {}

    def _servo_supported(self) -> bool:
        return self.servo_pub is not None and self._servo_start_client is not None and self._servo_stop_client is not None

    def _publish_zero_twist(self):
        if self.servo_pub is None:
            return
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.servo_pub.publish(msg)

    def _call_trigger_sync(self, client, timeout_sec: float, label: str) -> bool:
        if client is None:
            return False
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning(f"{label} service unavailable")
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            self.node.get_logger().warning(f"{label} timed out")
            return False
        response = future.result()
        return bool(response and response.success)

    def start_servo(self, timeout_sec: float = 5.0) -> bool:
        return self._ensure_servo_mode(timeout_sec=timeout_sec)

    def stop_servo(self, timeout_sec: float = 5.0) -> bool:
        return self._ensure_trajectory_mode(timeout_sec=timeout_sec)

    def _ensure_servo_mode(self, timeout_sec: float = 5.0) -> bool:
        if not self._servo_supported():
            return False
        if self._in_servo_mode:
            return True
        # MoveIt Servo is configured to publish JointTrajectory commands into the
        # normal trajectory controllers. Do not switch to the separate
        # JointGroupPositionController path; that path drops motion on the real
        # TopicBasedSystem bridge.
        if self._servo_stop_client is not None:
            self._call_trigger_sync(self._servo_stop_client, 1.0, "stop_servo")
            time.sleep(0.1)
        if not self._call_trigger_sync(self._servo_start_client, timeout_sec, "start_servo"):
            return False
        self._in_servo_mode = True
        for _ in range(10):
            self._publish_zero_twist()
            time.sleep(0.03)
        return True

    def _ensure_trajectory_mode(self, timeout_sec: float = 5.0) -> bool:
        self._publish_zero_twist()
        if not self._in_servo_mode:
            return True
        if self._servo_stop_client is not None:
            self._call_trigger_sync(self._servo_stop_client, timeout_sec, "stop_servo")
        self._in_servo_mode = False
        return True

    def _switch_controller_mode(self, target_mode: str, timeout_sec: float = 5.0) -> bool:
        if self.is_gripper or self.is_dual_arms or self.controller_name is None:
            return True
        servo_controller = None
        if self.is_uf850:
            servo_controller = "uf850_servo_controller"
        elif self.is_xarm5:
            servo_controller = "xarm5_servo_controller"
        if servo_controller is None:
            return True

        if not self._controller_list_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning("controller_manager/list_controllers unavailable")
            return False
        if not self._controller_switch_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning("controller_manager/switch_controller unavailable")
            return False

        list_future = self._controller_list_client.call_async(ListControllers.Request())
        if not self._wait_for_future(list_future, timeout_sec):
            self.node.get_logger().warning("Timed out waiting for list_controllers")
            return False
        response = list_future.result()
        if response is None:
            self.node.get_logger().warning("list_controllers returned no response")
            return False

        controller_states = {controller.name: controller.state for controller in response.controller}
        if target_mode == "servo":
            # MoveIt Servo is configured to output JointTrajectory commands to
            # the normal trajectory controllers. Keep that controller active and
            # shut down the auxiliary JointGroupPositionController if it was
            # manually activated.
            activate = [self.controller_name] if controller_states.get(self.controller_name) == "inactive" else []
            deactivate = [servo_controller] if controller_states.get(servo_controller) == "active" else []
        else:
            activate = [self.controller_name] if controller_states.get(self.controller_name) == "inactive" else []
            deactivate = [servo_controller] if controller_states.get(servo_controller) == "active" else []

        if not activate and not deactivate:
            return True

        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        switch_future = self._controller_switch_client.call_async(request)
        if not self._wait_for_future(switch_future, timeout_sec):
            self.node.get_logger().warning("Timed out waiting for switch_controller")
            return False
        result = switch_future.result()
        return bool(result and result.ok)

    def set_gripper_force(self, force_n: float) -> bool:
        if self._gripper_force_pub is None:
            return False
        msg = Float32()
        msg.data = float(force_n)
        self.current_gripper_force_n = float(force_n)
        self._gripper_force_pub.publish(msg)
        return True

    def move_gripper(self, joint_position_rad: float, force_n: float = None, velocity: float = 0.2) -> bool:
        if force_n is not None:
            self.set_gripper_force(force_n)
        return self.move_to_joint_positions(
            {"rg6_right_drive_joint": float(joint_position_rad)},
            velocity=velocity,
        )

    @classmethod
    def _rg6_rad_to_width_mm(cls, value_rad: float) -> float:
        value = max(cls._RG6_RAD_OPEN, min(cls._RG6_RAD_CLOSE, float(value_rad)))
        normalized = (value - cls._RG6_RAD_CLOSE) / (cls._RG6_RAD_OPEN - cls._RG6_RAD_CLOSE)
        return cls._RG6_MM_CLOSE + normalized * (cls._RG6_MM_OPEN - cls._RG6_MM_CLOSE)

    def _rg6_target_width_acknowledged(self, target_width_mm: float, tolerance_mm: float = 2.0) -> bool:
        try:
            reported = float(self.current_gripper_state.get("target_width_mm"))
        except Exception:
            return False
        return abs(reported - float(target_width_mm)) <= float(tolerance_mm)

    def _publish_gripper_target_until_ack(self, joint_name: str, target: float, timeout_s: float = 1.2) -> bool:
        target_width_mm = self._rg6_rad_to_width_mm(target)
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            self._publish_gripper_command(joint_name, target)
            time.sleep(0.08)
            if self._rg6_target_width_acknowledged(target_width_mm):
                return True
        return self._rg6_target_width_acknowledged(target_width_mm)

    def _publish_gripper_command(self, joint_name: str, target: float, lookahead_s: float = 0.25):
        """Publish a single final RG6 target on both command paths.

        The real hardware bridge listens to /robot_joint_commands, while the
        active rg6_controller can retain the previous trajectory command. A
        one-point JTC command replaces that controller target without sending a
        multi-point gripper trajectory.
        """
        target = float(target)
        self._publish_raw_joint_command({joint_name: target})
        if self._joint_traj_stream_pub is None:
            return

        traj = JointTrajectory()
        traj.header.stamp = self.node.get_clock().now().to_msg()
        traj.joint_names = [joint_name]
        pt = JointTrajectoryPoint()
        pt.positions = [target]
        pt.velocities = [0.0]
        pt.accelerations = [0.0]
        _la = max(0.05, float(lookahead_s))
        pt.time_from_start = Duration(
            sec=int(_la),
            nanosec=int((_la - int(_la)) * 1_000_000_000),
        )
        traj.points = [pt]
        self._joint_traj_stream_pub.publish(traj)

    def _execute_gripper_direct(
        self,
        target_joints: dict[str, float],
        velocity: float = 0.2,
        gripper_force_n: float = None,
    ) -> bool:
        """Command RG6 once through the hardware bridge.

        Sending RG6 as a FollowJointTrajectory creates many intermediate width
        targets. The Modbus bridge intentionally does not send a new width while
        the gripper is moving, so those intermediate targets become visible chunks.
        A single final width command lets the RG6 internal controller move smoothly.
        """
        joint_name = "rg6_right_drive_joint"
        if joint_name not in target_joints:
            self.node.get_logger().error(
                f"[{self.backend_kind}] gripper command missing {joint_name}: {target_joints}"
            )
            return False
        if gripper_force_n is not None:
            self.set_gripper_force(gripper_force_n)

        target = float(target_joints[joint_name])
        current = float(self.current_joint_positions.get(joint_name, target))
        start_current = current
        is_closing = target > current
        target_width_mm = self._rg6_rad_to_width_mm(target)
        self.node.get_logger().info(
            f"[{self.backend_kind}] Direct RG6 command: target={target:.3f}rad "
            f"width={target_width_mm:.1f}mm force={self.current_gripper_force_n:.1f}N"
        )
        acked = self._publish_gripper_target_until_ack(joint_name, target, timeout_s=0.8)
        if not acked and is_closing:
            nudge = target + (0.03 if is_closing else -0.03)
            nudge = max(self._RG6_RAD_OPEN, min(self._RG6_RAD_CLOSE, nudge))
            self.node.get_logger().warning(
                f"[{self.backend_kind}] RG6 bridge did not acknowledge target width "
                f"{target_width_mm:.1f}mm; nudging command path before retry"
            )
            self._publish_gripper_command(joint_name, nudge)
            time.sleep(0.12)
            acked = self._publish_gripper_target_until_ack(joint_name, target, timeout_s=1.2)
        elif not acked:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] RG6 bridge did not acknowledge open target width "
                f"{target_width_mm:.1f}mm; retrying exact open command without nudge"
            )
            acked = self._publish_gripper_target_until_ack(joint_name, target, timeout_s=1.2)
        if not acked:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] RG6 bridge target width still not acknowledged; "
                f"continuing to monitor physical motion. state={self.current_gripper_state}"
            )
            if not is_closing:
                return False

        if not self.state_received.wait(timeout=1.0):
            return True

        tolerance = 0.08
        deadline = time.time() + max(4.0, min(12.0, abs(target - current) / max(float(velocity), 0.05) + 2.0))
        last_resend = time.time()
        while rclpy.ok() and time.time() < deadline:
            current = float(self.current_joint_positions.get(joint_name, current))
            if abs(current - target) <= tolerance:
                return True
            if is_closing and bool(self.current_gripper_state.get("object_detected", False)):
                return True
            if time.time() - last_resend >= 0.2:
                if not self._rg6_target_width_acknowledged(target_width_mm):
                    self._publish_gripper_target_until_ack(joint_name, target, timeout_s=0.25)
                else:
                    self._publish_gripper_command(joint_name, target)
                last_resend = time.time()
            time.sleep(0.05)
        self.node.get_logger().warning(
            f"[{self.backend_kind}] Direct RG6 command timed out before target; "
            f"current={current:.3f}rad target={target:.3f}rad "
            f"state={self.current_gripper_state}"
        )
        moved_toward_target = (current - start_current) if is_closing else (start_current - current)
        if is_closing and moved_toward_target >= 0.08:
            self.node.get_logger().info(
                f"[{self.backend_kind}] Direct RG6 command accepted after partial motion "
                f"toward target: moved={moved_toward_target:.3f}rad"
            )
            return True
        return False

    def _get_full_robot_state(self) -> RobotState:
        state = RobotState()
        state.joint_state.name = list(self.current_joint_positions.keys())
        state.joint_state.position = [self.current_joint_positions[name] for name in state.joint_state.name]
        return state

    def _publish_direct_joint_command(self, target_joints: dict[str, float], lookahead_s: float = 0.1):
        # JTC topic: single-point trajectory with adaptive lookahead for smooth streaming.
        # lookahead_s is set by the caller to slightly exceed the actual IK cycle time so
        # the controller always has an active trajectory — preventing stop-and-lurch gaps.
        # NOTE: we do NOT also publish to /robot_joint_commands here because that would send
        # competing commands to the real_hardware_bridge simultaneously with the JTC, causing
        # unpredictable motion on real hardware.
        if self._joint_traj_stream_pub is not None:
            traj = JointTrajectory()
            traj.header.stamp = self.node.get_clock().now().to_msg()
            traj.joint_names = list(target_joints.keys())
            pt = JointTrajectoryPoint()
            pt.positions = [float(target_joints[name]) for name in traj.joint_names]
            pt.velocities = [0.0] * len(traj.joint_names)
            pt.accelerations = [0.0] * len(traj.joint_names)
            _la = max(0.04, float(lookahead_s))
            pt.time_from_start = Duration(
                sec=int(_la),
                nanosec=int((_la - int(_la)) * 1_000_000_000),
            )
            traj.points = [pt]
            self._joint_traj_stream_pub.publish(traj)

    def _publish_raw_joint_command(self, target_joints: dict[str, float]):
        """Publish directly to the topic-based hardware bridge command topic.

        This bypasses MoveIt's execute_trajectory action. Use only for verified,
        single-arm, small-step streaming where the caller is not also executing a
        JTC trajectory.
        """
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(target_joints.keys())
        msg.position = [float(target_joints[name]) for name in msg.name]
        self._joint_command_pub.publish(msg)

    def _hold_current_arm_position(self):
        hold_joints = {
            name: float(self.current_joint_positions[name])
            for name in self.current_joint_positions
            if name.startswith(self.joint_prefixes)
        }
        if hold_joints:
            self._publish_direct_joint_command(hold_joints)

    def _execute_raw_joint_interpolation(
        self,
        target_joints: dict[str, float],
        velocity: float = 0.15,
        rate_hz: float = 30.0,
        max_joint_step_rad: float = 0.025,
        settle_tolerance_rad: float = 0.025,
    ) -> bool:
        """Stream a joint target directly to /robot_joint_commands in safe increments."""
        controlled = [
            name for name in target_joints
            if name in self.current_joint_positions and name.startswith(self.joint_prefixes)
        ]
        if not controlled:
            self.node.get_logger().error(
                f"[{self.backend_kind}] Raw joint interpolation has no controlled joints "
                f"(target={list(target_joints.keys())}, prefixes={self.joint_prefixes})"
            )
            return False

        start_joints = {name: float(self.current_joint_positions[name]) for name in controlled}
        target = {name: float(target_joints[name]) for name in controlled}
        max_delta = max(abs(target[name] - start_joints[name]) for name in controlled)
        if max_delta < 1e-5:
            self._publish_raw_joint_command(target)
            return True

        rad_per_sec = max(0.05, min(0.30, float(velocity)))
        dt = 1.0 / max(float(rate_hz), 1.0)
        step_from_speed = rad_per_sec * dt
        step = max(0.002, min(abs(max_joint_step_rad), step_from_speed))
        steps = max(1, int(math.ceil(max_delta / step)))

        self.node.get_logger().info(
            f"[{self.backend_kind}] Raw joint interpolation: {len(controlled)} joints, "
            f"max_delta={max_delta:.3f}rad, steps={steps}, rate={rate_hz:.1f}Hz"
        )
        for i in range(1, steps + 1):
            ratio = i / float(steps)
            command = {
                name: start_joints[name] + (target[name] - start_joints[name]) * ratio
                for name in controlled
            }
            self._publish_raw_joint_command(command)
            time.sleep(dt)

        for _ in range(8):
            self._publish_raw_joint_command(target)
            time.sleep(dt)

        deadline = time.time() + max(2.0, (max_delta / rad_per_sec) + 2.0)
        while rclpy.ok() and time.time() < deadline:
            joint_err = max(
                abs(float(self.current_joint_positions.get(name, start_joints[name])) - target[name])
                for name in controlled
            )
            if joint_err <= settle_tolerance_rad:
                self.node.get_logger().info(
                    f"[{self.backend_kind}] Raw joint interpolation settled, max_joint_err={joint_err:.4f}rad"
                )
                return True
            self._publish_raw_joint_command(target)
            time.sleep(0.05)

        joint_err = max(
            abs(float(self.current_joint_positions.get(name, start_joints[name])) - target[name])
            for name in controlled
        )
        self.node.get_logger().error(
            f"[{self.backend_kind}] Raw joint interpolation did not settle; "
            f"max_joint_err={joint_err:.4f}rad"
        )
        return False

    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )

    def _quaternion_to_rpy(self, x: float, y: float, z: float, w: float):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _configure_move_group_request(self, goal: MoveGroup.Goal, velocity: float):
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = velocity
        goal.request.max_acceleration_scaling_factor = velocity

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.01)
        return future.done()

    def _execute_joint_goal(self, joint_state: JointState, velocity: float) -> bool:
        goal = MoveGroup.Goal()
        self._configure_move_group_request(goal, velocity)
        constraints = Constraints()
        for name, position in zip(joint_state.name, joint_state.position):
            if name.startswith(self.joint_prefixes):
                constraint = JointConstraint()
                constraint.joint_name = name
                constraint.position = float(position)
                constraint.tolerance_above = 0.01
                constraint.tolerance_below = 0.01
                constraint.weight = 1.0
                constraints.joint_constraints.append(constraint)
        if not constraints.joint_constraints:
            return False
        goal.request.goal_constraints.append(constraints)
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            return False
        future = self._move_group_client.send_goal_async(goal)
        if not self._wait_for_future(future, 30.0):
            return False
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, 180.0):
            return False
        result = result_future.result()
        return bool(result and result.result.error_code.val == 1)

    def _execute_robot_trajectory(self, trajectory) -> bool:
        pts = len(trajectory.joint_trajectory.points)
        joints = trajectory.joint_trajectory.joint_names
        if pts > 0:
            duration = trajectory.joint_trajectory.points[-1].time_from_start
            dur_s = duration.sec + duration.nanosec * 1e-9
        else:
            dur_s = 0.0
        self.node.get_logger().info(
            f"[{self.backend_kind}] Executing trajectory: {pts} points, "
            f"{len(joints)} joints [{', '.join(joints)}], duration={dur_s:.2f}s"
        )

        if os.environ.get("DISASSEMBLY_PUBLISH_DISPLAY_TRAJECTORY", "0") == "1":
            display_msg = DisplayTrajectory()
            display_msg.model_id = "dual_arm_world"
            display_msg.trajectory_start = self._get_full_robot_state()
            display_msg.trajectory = [trajectory]
            self._display_trajectory_pub.publish(display_msg)
            time.sleep(0.1)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        if not self._execute_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(
                f"[{self.backend_kind}] execute_trajectory action server not available after 10s"
            )
            return False
        send_future = self._execute_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 20.0):
            self.node.get_logger().error(
                f"[{self.backend_kind}] Timed out waiting for execute_trajectory goal acceptance (20s)"
            )
            return False
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.node.get_logger().error(
                f"[{self.backend_kind}] execute_trajectory goal was rejected by the action server"
            )
            return False
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, 120.0):
            self.node.get_logger().error(
                f"[{self.backend_kind}] Timed out waiting for trajectory execution to complete (120s)"
            )
            return False
        exec_result = result_future.result()
        success = bool(exec_result and exec_result.result.error_code.val == 1)
        if success:
            self.node.get_logger().info(
                f"[{self.backend_kind}] Trajectory executed successfully"
            )
        else:
            err_val = exec_result.result.error_code.val if exec_result else "no result"
            self.node.get_logger().error(
                f"[{self.backend_kind}] Trajectory execution FAILED: error_code={err_val}"
            )
        return success

    def _execute_trajectory_direct(self, trajectory) -> bool:
        """Stream a pre-planned trajectory to /robot_joint_commands at the trajectory's own rate.

        Replaces the JTC execute_trajectory action for single-arm real-hardware moves.
        The JTC path feeds TopicBasedSystem → real_hardware.py at 100Hz; Python timing
        jitter in set_servo_angle_j TCP calls means commands arrive in bursts → jerky arm.
        Here we replay each quintic waypoint at its intended wall-clock time (~25Hz for a
        2s trajectory) so the Python bridge always has ample time per command.
        """
        pts = trajectory.joint_trajectory.points
        names = list(trajectory.joint_trajectory.joint_names)
        if not pts or not names:
            return self._execute_robot_trajectory(trajectory)

        arm_names = [n for n in names if n.startswith(self.joint_prefixes)]
        if not arm_names:
            return self._execute_robot_trajectory(trajectory)
        arm_idx = [names.index(n) for n in arm_names]

        dur_s = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9
        self.node.get_logger().info(
            f"[{self.backend_kind}] Direct trajectory stream: {len(pts)} pts, "
            f"{len(arm_names)} joints, duration={dur_s:.2f}s"
        )
        t0 = time.time()
        for pt in pts:
            t_target = t0 + pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            wait_s = t_target - time.time()
            if wait_s > 0.001:
                time.sleep(wait_s)
            self._publish_raw_joint_command(
                {arm_names[i]: float(pt.positions[arm_idx[i]]) for i in range(len(arm_names))}
            )

        # Settle: keep sending the final target until the arm arrives.
        target = {arm_names[i]: float(pts[-1].positions[arm_idx[i]]) for i in range(len(arm_names))}
        deadline = time.time() + 5.0
        while rclpy.ok() and time.time() < deadline:
            err = max(
                abs(float(self.current_joint_positions.get(n, target[n])) - target[n])
                for n in arm_names
            )
            if err <= 0.03:
                self.node.get_logger().info(
                    f"[{self.backend_kind}] Direct trajectory settled (err={err:.4f}rad)"
                )
                return True
            self._publish_raw_joint_command(target)
            time.sleep(0.05)

        self.node.get_logger().warning(
            f"[{self.backend_kind}] Direct trajectory settling timeout (non-fatal)"
        )
        return True

    def move_to_joint_positions(self, target_joints, velocity: float = 0.2, gripper_force_n: float = None) -> bool:
        self.node.get_logger().info(
            f"[{self.backend_kind}] move_to_joint_positions: {len(target_joints)} joints, velocity={velocity}"
        )
        if self.is_gripper:
            return self._execute_gripper_direct(target_joints, velocity, gripper_force_n)
        if not self._ensure_trajectory_mode():
            self.node.get_logger().error(f"[{self.backend_kind}] move_to_joint_positions: failed to enter trajectory mode")
            return False
        if not self.state_received.wait(timeout=2.0):
            self.node.get_logger().error(f"[{self.backend_kind}] move_to_joint_positions: timed out waiting for joint states")
            return False
        if self.is_gripper and gripper_force_n is not None:
            self.set_gripper_force(gripper_force_n)
        if self.is_dual_arms and self._exotica_planner is not None and self._exotica_planner.available:
            self.node.get_logger().info(f"[{self.backend_kind}] Planning with EXOTica RRT-Connect...")
            t0 = time.time()
            trajectory = self._exotica_planner.plan_joint_trajectory(
                self.current_joint_positions,
                target_joints,
                velocity_scaling=velocity,
            )
            if trajectory is not None:
                pts = len(trajectory.joint_trajectory.points)
                self.node.get_logger().info(
                    f"[{self.backend_kind}] EXOTica RRT-Connect produced {pts}-point trajectory in {time.time()-t0:.2f}s"
                )
                return self._execute_robot_trajectory(trajectory)
            self.node.get_logger().warning(
                f"[{self.backend_kind}] EXOTica dual-arm planning failed after {time.time()-t0:.2f}s, "
                f"falling back to MoveIt OMPL: {self._exotica_planner.last_error}"
            )
        self.node.get_logger().info(f"[{self.backend_kind}] Planning joint goal with MoveIt MoveGroup...")
        goal = MoveGroup.Goal()
        self._configure_move_group_request(goal, velocity)
        constraints = Constraints()
        for name, position in target_joints.items():
            if name.startswith(self.joint_prefixes):
                constraint = JointConstraint()
                constraint.joint_name = name
                constraint.position = float(position)
                constraint.tolerance_above = 0.01
                constraint.tolerance_below = 0.01
                constraint.weight = 1.0
                constraints.joint_constraints.append(constraint)
        if not constraints.joint_constraints:
            self.node.get_logger().error(
                f"[{self.backend_kind}] move_to_joint_positions: no matching joint constraints built "
                f"(target_joints keys={list(target_joints.keys())}, prefixes={self.joint_prefixes})"
            )
            return False
        goal.request.goal_constraints.append(constraints)
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(f"[{self.backend_kind}] MoveGroup action server not available after 10s")
            return False
        future = self._move_group_client.send_goal_async(goal)
        if not self._wait_for_future(future, 30.0):
            self.node.get_logger().error(f"[{self.backend_kind}] MoveGroup goal send timed out (30s)")
            return False
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.node.get_logger().error(f"[{self.backend_kind}] MoveGroup joint goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, 60.0):
            self.node.get_logger().error(f"[{self.backend_kind}] MoveGroup joint goal execution timed out (60s)")
            return False
        result = result_future.result()
        success = bool(result and result.result.error_code.val == 1)
        if not success:
            err_val = result.result.error_code.val if result else "no result"
            self.node.get_logger().error(
                f"[{self.backend_kind}] MoveGroup joint goal FAILED: error_code={err_val}"
            )
        return success

    def move_to_pose_robust(
        self,
        x: float,
        y: float,
        z: float,
        q_dict=None,
        velocity: float = 0.1,
        frame_id: str = "base_link",
    ) -> bool:
        self.node.get_logger().info(
            f"[{self.backend_kind}] move_to_pose_robust: target=({x:.3f},{y:.3f},{z:.3f}) "
            f"frame={frame_id} velocity={velocity}"
        )
        if frame_id == "base_link":
            x = self._clamp_target_x(x, "move_to_pose_robust")
            z = self._clamp_target_z(z, "move_to_pose_robust")
        if not self._ensure_trajectory_mode():
            self.node.get_logger().error(f"[{self.backend_kind}] move_to_pose_robust: failed to enter trajectory mode")
            return False
        if not self._ik_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error(f"[{self.backend_kind}] move_to_pose_robust: /compute_ik service not available")
            return False
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.avoid_collisions = False
        request.ik_request.ik_link_name = self.default_ik_link or ""
        request.ik_request.robot_state = self._get_full_robot_state()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        if q_dict:
            pose_stamped.pose.orientation = Quaternion(
                x=q_dict["qx"], y=q_dict["qy"], z=q_dict["qz"], w=q_dict["qw"]
            )
        else:
            default_roll = 0.0 if self.is_xarm5 else math.pi
            pose_stamped.pose.orientation = self._rpy_to_quaternion(default_roll, 0.0, 0.0)
        request.ik_request.pose_stamped = pose_stamped

        future = self._ik_client.call_async(request)
        if not self._wait_for_future(future, 15.0):
            self.node.get_logger().error(f"[{self.backend_kind}] move_to_pose_robust: /compute_ik timed out (15s)")
            return False
        result = future.result()
        if result and result.error_code.val == 1:
            self.node.get_logger().info(f"[{self.backend_kind}] move_to_pose_robust: IK solved, executing trajectory")
            return self._execute_joint_goal(result.solution.joint_state, velocity)

        ik_err = result.error_code.val if result else "no response"
        self.node.get_logger().warning(
            f"[{self.backend_kind}] move_to_pose_robust: IK failed (error_code={ik_err})"
            + (" — trying yaw perturbations for 5-DOF xArm5" if self.is_xarm5 else "")
        )
        if self.is_xarm5:
            for yaw_deg in (15, -15, 30, -30, 45, -45, 90, -90, 180):
                pose_stamped.pose.orientation = self._rpy_to_quaternion(0.0, 0.0, math.radians(yaw_deg))
                request.ik_request.pose_stamped = pose_stamped
                request.ik_request.robot_state = self._get_full_robot_state()
                retry_future = self._ik_client.call_async(request)
                if not self._wait_for_future(retry_future, 5.0):
                    continue
                retry_result = retry_future.result()
                if retry_result and retry_result.error_code.val == 1:
                    self.node.get_logger().info(
                        f"[{self.backend_kind}] move_to_pose_robust: IK succeeded with yaw_offset={yaw_deg}°"
                    )
                    return self._execute_joint_goal(retry_result.solution.joint_state, velocity)
        self.node.get_logger().error(
            f"[{self.backend_kind}] move_to_pose_robust: all IK attempts FAILED for "
            f"({x:.3f},{y:.3f},{z:.3f}). "
            + ("5-DOF xArm5 cannot achieve this orientation." if self.is_xarm5 else "Check reachability.")
        )
        return False

    def move_to_pose_exotica(
        self,
        x: float,
        y: float,
        z: float,
        q_dict=None,
        velocity: float = 0.1,
        frame_id: str = "base_link",
    ) -> bool:
        self.node.get_logger().info(
            f"[{self.backend_kind}] move_to_pose_exotica: target=({x:.3f},{y:.3f},{z:.3f}) "
            f"frame={frame_id} velocity={velocity}"
        )
        if frame_id == "base_link":
            x = self._clamp_target_x(x, "move_to_pose_exotica")
            z = self._clamp_target_z(z, "move_to_pose_exotica")
        if not self._ensure_trajectory_mode():
            self.node.get_logger().error(f"[{self.backend_kind}] move_to_pose_exotica: failed to enter trajectory mode")
            return False
        if self._single_arm_exotica_planner is None or not self._single_arm_exotica_planner.available:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] move_to_pose_exotica: EXOTica planner unavailable "
                f"({getattr(self._single_arm_exotica_planner, 'last_error', 'not initialized')}). "
                "Falling back to MoveIt IK — motion will not use smooth quintic trajectories."
            )
            return self.move_to_pose_robust(x, y, z, q_dict=q_dict, velocity=velocity, frame_id=frame_id)
        if frame_id != "base_link":
            self.node.get_logger().warning(
                f"[{self.backend_kind}] move_to_pose_exotica: frame_id={frame_id} is not base_link. "
                "EXOTica expects base_link targets. Falling back to MoveIt IK."
            )
            return self.move_to_pose_robust(x, y, z, q_dict=q_dict, velocity=velocity, frame_id=frame_id)
        if not self.state_received.wait(timeout=2.0):
            self.node.get_logger().error(
                f"[{self.backend_kind}] move_to_pose_exotica: timed out waiting for joint states"
            )
            return False

        if q_dict:
            roll, pitch, yaw = self._quaternion_to_rpy(
                float(q_dict["qx"]),
                float(q_dict["qy"]),
                float(q_dict["qz"]),
                float(q_dict["qw"]),
            )
        else:
            # xArm5 screwdriver_tcp: HOME=roll=0 has R_zz=+1 (tip physically DOWN).
            # roll=π would flip the frame (R_zz=-1, tip UP) which is wrong for screwing.
            # UF850 rg6_tcp uses a different frame convention where roll=π is correct.
            roll, pitch, yaw = (0.0, 0.0, 0.0) if self.is_xarm5 else (math.pi, 0.0, 0.0)

        self.node.get_logger().info(
            f"[{self.backend_kind}] EXOTica IK solving for "
            f"[{x:.3f},{y:.3f},{z:.3f}, r={math.degrees(roll):.1f}°, p={math.degrees(pitch):.1f}°, y={math.degrees(yaw):.1f}°]"
            + (" (5-DOF: orientation may have residual error)" if self.is_xarm5 else "")
        )
        t0 = time.time()
        trajectory = self._single_arm_exotica_planner.plan_pose_trajectory(
            self.current_joint_positions,
            [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)],
            velocity_scaling=velocity,
        )
        if trajectory is not None:
            pts = len(trajectory.joint_trajectory.points)
            self.node.get_logger().info(
                f"[{self.backend_kind}] EXOTica IK solved in {time.time()-t0:.3f}s, "
                f"generated {pts}-point quintic trajectory"
            )
            return self._execute_robot_trajectory(trajectory)

        self.node.get_logger().warning(
            f"[{self.backend_kind}] move_to_pose_exotica: EXOTica IK FAILED after {time.time()-t0:.3f}s "
            f"({self._single_arm_exotica_planner.last_error})."
            + (" Trying yaw perturbations (5-DOF xArm5)." if self.is_xarm5 else " Falling back to MoveIt IK.")
        )
        if self.is_xarm5:
            for yaw_deg in (15, -15, 30, -30, 45, -45, 90, -90, 180):
                yaw_p = math.radians(yaw_deg)
                t1 = time.time()
                traj_p = self._single_arm_exotica_planner.plan_pose_trajectory(
                    self.current_joint_positions,
                    [float(x), float(y), float(z), float(roll), float(pitch), yaw_p],
                    velocity_scaling=velocity,
                )
                if traj_p is not None:
                    pts = len(traj_p.joint_trajectory.points)
                    self.node.get_logger().info(
                        f"[{self.backend_kind}] EXOTica IK solved with yaw_offset={yaw_deg}° "
                        f"in {time.time()-t1:.3f}s, generated {pts}-point quintic trajectory"
                    )
                    return self._execute_robot_trajectory(traj_p)
            self.node.get_logger().warning(
                f"[{self.backend_kind}] move_to_pose_exotica: all yaw perturbations failed. "
                "Falling back to MoveIt IK."
            )
        return self.move_to_pose_robust(x, y, z, q_dict=q_dict, velocity=velocity, frame_id=frame_id)

    def move_to_pose_direct_exotica(
        self,
        x: float,
        y: float,
        z: float,
        q_dict=None,
        velocity: float = 0.15,
        frame_id: str = "base_link",
    ) -> bool:
        """Solve pose IK with EXOTica, then stream joints directly to hardware bridge.

        This is intended for real hardware cases where MoveIt's trajectory action
        can report success while the physical arm did not move. Callers must still
        verify the final TCP pose after this method returns.
        """
        self.node.get_logger().info(
            f"[{self.backend_kind}] move_to_pose_direct_exotica: target=({x:.3f},{y:.3f},{z:.3f}) "
            f"frame={frame_id} velocity={velocity}"
        )
        if frame_id != "base_link":
            self.node.get_logger().error(
                f"[{self.backend_kind}] direct EXOTica pose streaming requires base_link targets"
            )
            return False
        if self._single_arm_exotica_planner is None or not self._single_arm_exotica_planner.available:
            self.node.get_logger().error(
                f"[{self.backend_kind}] direct EXOTica pose streaming unavailable: "
                f"{getattr(self._single_arm_exotica_planner, 'last_error', 'planner missing')}"
            )
            return False
        if not self.state_received.wait(timeout=2.0):
            self.node.get_logger().error(
                f"[{self.backend_kind}] direct EXOTica pose streaming timed out waiting for joint states"
            )
            return False

        x = self._clamp_target_x(x, "move_to_pose_direct_exotica")
        z = self._clamp_target_z(z, "move_to_pose_direct_exotica")
        if q_dict:
            roll, pitch, yaw = self._quaternion_to_rpy(
                float(q_dict["qx"]),
                float(q_dict["qy"]),
                float(q_dict["qz"]),
                float(q_dict["qw"]),
            )
        else:
            roll, pitch, yaw = (0.0, 0.0, 0.0) if self.is_xarm5 else (math.pi, 0.0, 0.0)

        target_joints = self._single_arm_exotica_planner.solve_pose_goal_joint_positions(
            self.current_joint_positions,
            [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)],
            max_retries=10,
        )
        if not target_joints:
            self.node.get_logger().error(
                f"[{self.backend_kind}] direct EXOTica IK failed: "
                f"{self._single_arm_exotica_planner.last_error}"
            )
            return False

        return self._execute_raw_joint_interpolation(
            target_joints,
            velocity=velocity,
            rate_hz=30.0,
            max_joint_step_rad=0.018 if self.is_xarm5 else 0.025,
            settle_tolerance_rad=0.03 if self.is_xarm5 else 0.025,
        )

    def move_cartesian_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        q_dict=None,
        velocity: float = 0.1,
        frame_id: str = "base_link",
    ) -> bool:
        if frame_id == "base_link":
            x = self._clamp_target_x(x, "move_cartesian_to_pose")
            z = self._clamp_target_z(z, "move_cartesian_to_pose")
        if not self._ensure_trajectory_mode():
            return False
        if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
            return False
        request = GetCartesianPath.Request()
        request.header.frame_id = frame_id
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.start_state = self._get_full_robot_state()
        request.group_name = self.group_name
        request.link_name = self.default_ik_link or ""
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        if q_dict:
            target.orientation = Quaternion(
                x=q_dict["qx"], y=q_dict["qy"], z=q_dict["qz"], w=q_dict["qw"]
            )
        else:
            default_roll = 0.0 if self.is_xarm5 else math.pi
            target.orientation = self._rpy_to_quaternion(default_roll, 0.0, 0.0)
        request.waypoints = [target]

        future = self._cartesian_client.call_async(request)
        if not self._wait_for_future(future, 20.0):
            return False
        result = future.result()
        if not result or result.fraction < 0.85:
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = result.solution
        if not self._execute_client.wait_for_server(timeout_sec=10.0):
            return False
        send_future = self._execute_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 20.0):
            return False
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, 60.0):
            return False
        exec_result = result_future.result()
        return bool(exec_result and exec_result.result.error_code.val == 1)

    def publish_servo_velocity(self, dx: float, dy: float, dz: float) -> bool:
        """Publish one servo velocity command (non-blocking, no zero-stop after)."""
        if not self._ensure_servo_mode():
            return False
        twist = TwistStamped()
        twist.header.frame_id = "base_link"
        twist.header.stamp = self.node.get_clock().now().to_msg()
        twist.twist.linear.x = dx
        twist.twist.linear.y = dy
        twist.twist.linear.z = dz
        self.servo_pub.publish(twist)
        return True

    def jog_cartesian_servo(self, dx: float, dy: float, dz: float, duration: float = 1.0) -> bool:
        if not self._ensure_servo_mode():
            return False
        twist = TwistStamped()
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = dx
        twist.twist.linear.y = dy
        twist.twist.linear.z = dz
        deadline = time.time() + duration
        while rclpy.ok() and time.time() < deadline:
            twist.header.stamp = self.node.get_clock().now().to_msg()
            self.servo_pub.publish(twist)
            time.sleep(0.033)
        self._publish_zero_twist()
        return True

    def move_linear_z_with_torque_stop(
        self, speed_mps: float, threshold_nm: float, joint_index: int = 2, timeout: float = 30.0
    ) -> bool:
        if not self._ensure_servo_mode():
            self.node.get_logger().warning("Servo tactile descent could not start because servo mode was unavailable.")
            return False
        joint_prefix = "xarm5_joint" if self.is_xarm5 else "uf850_joint"
        joint_name = f"{joint_prefix}{joint_index + 1}"
        if joint_name not in self.current_joint_efforts:
            self.node.get_logger().warning(
                f"Servo tactile descent has no effort feedback for {joint_name}. "
                "Check that /robot_joint_states is being received."
            )
            return False
        samples = []
        sample_deadline = time.time() + 0.2
        while time.time() < sample_deadline:
            samples.append(self.current_joint_efforts.get(joint_name, 0.0))
            time.sleep(0.01)
        baseline = sum(samples) / len(samples) if samples else 0.0
        try:
            start_tf = self.tf_buffer.lookup_transform("base_link", self.default_ik_link, rclpy.time.Time())
            start_z = float(start_tf.transform.translation.z)
        except Exception:
            start_z = None
        twist = TwistStamped()
        twist.header.frame_id = "base_link"
        twist.twist.linear.z = -abs(speed_mps)
        end_time = time.time() + timeout
        movement_detected = False
        movement_check_deadline = time.time() + min(1.0, timeout)
        while rclpy.ok() and time.time() < end_time:
            effort = self.current_joint_efforts.get(joint_name, baseline)
            if abs(effort - baseline) > threshold_nm:
                self._publish_zero_twist()
                return True
            twist.header.stamp = self.node.get_clock().now().to_msg()
            self.servo_pub.publish(twist)
            if start_z is not None and not movement_detected:
                try:
                    current_tf = self.tf_buffer.lookup_transform("base_link", self.default_ik_link, rclpy.time.Time())
                    current_z = float(current_tf.transform.translation.z)
                    if self.min_tcp_z is not None and current_z <= self.min_tcp_z + 1e-3:
                        self.node.get_logger().warning(
                            f"[{self.backend_kind}] Servo tactile descent reached table limit "
                            f"z={self.min_tcp_z:.5f} for {self.default_ik_link}. Stopping."
                        )
                        self._publish_zero_twist()
                        return False
                    if abs(current_z - start_z) > 0.003:
                        movement_detected = True
                except Exception:
                    pass
                if time.time() >= movement_check_deadline and not movement_detected:
                    self.node.get_logger().warning(
                        "Servo tactile descent is publishing commands but TCP Z is not changing. "
                        "Servo may not be executing the requested downward motion."
                    )
                    movement_detected = True
            time.sleep(0.01)
        self._publish_zero_twist()
        return False

    def move_linear_z_with_effort_stop_exotica(
        self,
        descent_distance_m: float,
        step_m: float,
        threshold_nm: float,
        joint_index: int = 4,
        q_dict=None,
        rate_hz: float = 50.0,
        settling_cycles: int = 3,
        command_alpha: float = 0.5,
        max_joint_step_rad: float = 0.03,
        max_solver_failures: int = 8,
        target_x: float | None = None,
        target_y: float | None = None,
    ) -> bool:
        self.node.get_logger().info(
            f"[{self.backend_kind}] move_linear_z_with_effort_stop_exotica: "
            f"descent={descent_distance_m*1000:.1f}mm step={step_m*1000:.2f}mm "
            f"threshold={threshold_nm:.2f}Nm joint_index={joint_index} "
            f"rate={rate_hz}Hz alpha={command_alpha}"
        )
        if self._single_arm_exotica_planner is None or not self._single_arm_exotica_planner.available:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] EXOTica stepped tactile descent unavailable "
                f"({getattr(self._single_arm_exotica_planner, 'last_error', 'not initialized')}). "
                "Falling back to servo torque stop."
            )
            speed_mps = max(step_m * rate_hz, 0.002)
            return self.move_linear_z_with_torque_stop(
                speed_mps=speed_mps,
                threshold_nm=threshold_nm,
                joint_index=joint_index,
                timeout=max(descent_distance_m / max(speed_mps, 1e-3), 10.0),
            )

        if not self.state_received.wait(timeout=2.0):
            self.node.get_logger().error(
                f"[{self.backend_kind}] move_linear_z_with_effort_stop_exotica: "
                "timed out waiting for /joint_states"
            )
            return False

        joint_prefix = "xarm5_joint" if self.is_xarm5 else "uf850_joint"
        joint_name = f"{joint_prefix}{joint_index + 1}"
        effort_available = joint_name in self.current_joint_efforts
        if not effort_available:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] move_linear_z_with_effort_stop_exotica: "
                f"no effort data for '{joint_name}' "
                f"(available: {list(self.current_joint_efforts.keys())}). "
                "Contact detection DISABLED — descent will traverse full distance."
            )

        try:
            start_tf = self.tf_buffer.lookup_transform("base_link", self.default_ik_link, rclpy.time.Time())
        except Exception as exc:
            self.node.get_logger().error(f"Unable to read start pose for EXOTica tactile descent: {exc}")
            return False

        if q_dict:
            roll, pitch, yaw = self._quaternion_to_rpy(
                float(q_dict["qx"]),
                float(q_dict["qy"]),
                float(q_dict["qz"]),
                float(q_dict["qw"]),
            )
        else:
            q = start_tf.transform.rotation
            roll, pitch, yaw = self._quaternion_to_rpy(q.x, q.y, q.z, q.w)

        samples = []
        sample_deadline = time.time() + 0.15
        while time.time() < sample_deadline:
            samples.append(self.current_joint_efforts.get(joint_name, 0.0))
            time.sleep(0.01)
        baseline = (sum(samples) / len(samples)) if samples else 0.0

        actual_start_x = float(start_tf.transform.translation.x)
        actual_start_y = float(start_tf.transform.translation.y)
        start_x = float(target_x) if target_x is not None else actual_start_x
        start_y = float(target_y) if target_y is not None else actual_start_y
        start_z = float(start_tf.transform.translation.z)
        target_depth = abs(float(descent_distance_m))
        if self.min_tcp_z is not None:
            allowed_depth = max(0.0, start_z - self.min_tcp_z)
            if target_depth > allowed_depth:
                self.node.get_logger().warning(
                    f"[{self.backend_kind}] EXOTica tactile descent requested "
                    f"{target_depth*1000:.1f}mm from z={start_z:.5f}, but table limit "
                    f"for {self.default_ik_link} only allows {allowed_depth*1000:.1f}mm. Clamping."
                )
                target_depth = allowed_depth
        step_m = max(abs(float(step_m)), 0.00025)
        loop_dt = 1.0 / max(float(rate_hz), 1.0)
        self.node.get_logger().info(
            f"[{self.backend_kind}] Tactile descent start: actual_EE=({actual_start_x:.3f},{actual_start_y:.3f},{start_z:.3f}), "
            f"command_xy=({start_x:.3f},{start_y:.3f}), "
            f"target_depth={target_depth*1000:.1f}mm, baseline_effort={baseline:.3f}Nm on {joint_name}"
        )
        commanded_positions = {
            name: float(self.current_joint_positions[name])
            for name in self.current_joint_positions
            if name in self._single_arm_exotica_planner.controlled_joint_names
        }
        seed_positions = dict(commanded_positions)
        commanded_z = start_z
        descent_started_at = time.time()
        consecutive_solver_failures = 0
        _last_progress_log = time.time()

        stable_contact_cycles = 0
        while rclpy.ok():
            effort = self.current_joint_efforts.get(joint_name, baseline) if effort_available else baseline
            spike = abs(effort - baseline)
            if effort_available and spike > threshold_nm:
                stable_contact_cycles += 1
                self.node.get_logger().info(
                    f"[{self.backend_kind}] Contact spike detected: {spike:.3f}Nm > {threshold_nm}Nm "
                    f"(cycle {stable_contact_cycles}/{settling_cycles})"
                )
                if stable_contact_cycles >= max(1, int(settling_cycles)):
                    self._hold_current_arm_position()
                    travelled_mm = (start_z - commanded_z) * 1000
                    self.node.get_logger().info(
                        f"[{self.backend_kind}] Tactile descent CONTACT confirmed on {joint_name}: "
                        f"spike={spike:.3f}Nm, travelled={travelled_mm:.1f}mm, "
                        f"time={time.time()-descent_started_at:.2f}s"
                    )
                    return True
            else:
                stable_contact_cycles = 0

            # Periodic progress log every 0.5s
            now = time.time()
            if now - _last_progress_log >= 0.5:
                travelled_mm = (start_z - commanded_z) * 1000
                self.node.get_logger().info(
                    f"[{self.backend_kind}] Descending: travelled={travelled_mm:.1f}/{target_depth*1000:.1f}mm "
                    f"effort={effort:.3f}Nm baseline={baseline:.3f}Nm spike={spike:.3f}Nm"
                )
                _last_progress_log = now

            travelled = start_z - commanded_z
            if travelled >= target_depth - 1e-4:
                break

            commanded_z = max(start_z - target_depth, commanded_z - step_m)
            commanded_z = self._clamp_target_z(commanded_z, "move_linear_z_with_effort_stop_exotica")
            target_joints = self._single_arm_exotica_planner.solve_pose_goal_joint_positions(
                seed_positions,
                [start_x, start_y, commanded_z, roll, pitch, yaw],
            )
            if not target_joints:
                consecutive_solver_failures += 1
                if consecutive_solver_failures >= max(1, int(max_solver_failures)):
                    self.node.get_logger().warning(
                        "EXOTica streaming tactile descent failed repeatedly: "
                        f"{self._single_arm_exotica_planner.last_error}"
                    )
                    self._hold_current_arm_position()
                    return False
                time.sleep(loop_dt)
                continue

            consecutive_solver_failures = 0
            filtered_command = {}
            for joint_name_key, solved_position in target_joints.items():
                previous = float(commanded_positions.get(joint_name_key, solved_position))
                delta = float(solved_position) - previous
                delta *= float(command_alpha)
                delta = max(-abs(max_joint_step_rad), min(abs(max_joint_step_rad), delta))
                filtered_command[joint_name_key] = previous + delta

            self._publish_direct_joint_command(filtered_command)
            commanded_positions = dict(filtered_command)
            seed_positions = dict(filtered_command)
            time.sleep(loop_dt)

        self._hold_current_arm_position()
        elapsed = time.time() - descent_started_at
        self.node.get_logger().warning(
            f"EXOTica tactile descent finished max depth without contact after {elapsed:.2f}s."
        )
        # Closed-loop: report actual final EE position
        try:
            end_tf = self.tf_buffer.lookup_transform(
                "base_link", self.default_ik_link, rclpy.time.Time()
            )
            actual_z = end_tf.transform.translation.z
            actual_x = end_tf.transform.translation.x
            actual_y = end_tf.transform.translation.y
            descended = start_z - actual_z
            self.node.get_logger().info(
                f"[{self.backend_kind}] Descent closed-loop: "
                f"EE=({actual_x:.4f},{actual_y:.4f},{actual_z:.4f}), "
                f"descended={descended*1000:.1f}mm / {target_depth*1000:.1f}mm target"
            )
        except Exception:
            pass
        return False

    def move_linear_axis_with_effort_stop_exotica(
        self,
        axis: str,
        distance_m: float,
        step_m: float,
        threshold_nm: float,
        joint_index: int = 4,
        q_dict=None,
        rate_hz: float = 50.0,
        settling_cycles: int = 3,
        command_alpha: float = 0.5,
        max_joint_step_rad: float = 0.03,
        max_solver_failures: int = 8,
    ) -> bool:
        """Tactile approach along any Cartesian axis with EXOTica IK streaming.

        axis     : 'x', 'y', or 'z'
        distance_m: signed travel distance (positive = +axis, negative = -axis)
        All other params mirror move_linear_z_with_effort_stop_exotica.
        """
        axis = axis.lower().strip('+-')
        if axis not in ('x', 'y', 'z'):
            self.node.get_logger().error(
                f"[{self.backend_kind}] move_linear_axis_with_effort_stop_exotica: "
                f"invalid axis '{axis}'"
            )
            return False
        sign = 1.0 if float(distance_m) >= 0.0 else -1.0
        target_depth = abs(float(distance_m))
        step_m = max(abs(float(step_m)), 0.00025)

        self.node.get_logger().info(
            f"[{self.backend_kind}] move_linear_axis_with_effort_stop_exotica: "
            f"axis={axis} dist={distance_m*1000:.1f}mm step={step_m*1000:.2f}mm "
            f"threshold={threshold_nm:.2f}Nm joint_index={joint_index}"
        )

        if self._single_arm_exotica_planner is None or not self._single_arm_exotica_planner.available:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] EXOTica lateral approach unavailable — no fallback for axis motion."
            )
            return False

        if not self.state_received.wait(timeout=2.0):
            self.node.get_logger().error(
                f"[{self.backend_kind}] move_linear_axis_with_effort_stop_exotica: "
                "timed out waiting for /joint_states"
            )
            return False

        joint_prefix = "xarm5_joint" if self.is_xarm5 else "uf850_joint"
        joint_name = f"{joint_prefix}{joint_index + 1}"
        effort_available = joint_name in self.current_joint_efforts
        if not effort_available:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] No effort data for '{joint_name}'. "
                "Contact detection DISABLED — will traverse full distance."
            )

        try:
            start_tf = self.tf_buffer.lookup_transform("base_link", self.default_ik_link, rclpy.time.Time())
        except Exception as exc:
            self.node.get_logger().error(
                f"Unable to read start pose for lateral approach: {exc}"
            )
            return False

        if q_dict:
            roll, pitch, yaw = self._quaternion_to_rpy(
                float(q_dict["qx"]), float(q_dict["qy"]),
                float(q_dict["qz"]), float(q_dict["qw"]),
            )
        else:
            q = start_tf.transform.rotation
            roll, pitch, yaw = self._quaternion_to_rpy(q.x, q.y, q.z, q.w)

        # Sample baseline effort
        samples = []
        sample_deadline = time.time() + 0.15
        while time.time() < sample_deadline:
            samples.append(self.current_joint_efforts.get(joint_name, 0.0))
            time.sleep(0.01)
        baseline = (sum(samples) / len(samples)) if samples else 0.0

        start_x = float(start_tf.transform.translation.x)
        start_y = float(start_tf.transform.translation.y)
        start_z = float(start_tf.transform.translation.z)

        loop_dt = 1.0 / max(float(rate_hz), 1.0)
        self.node.get_logger().info(
            f"[{self.backend_kind}] Lateral approach start: "
            f"EE=({start_x:.3f},{start_y:.3f},{start_z:.3f}), "
            f"axis={axis} travel={target_depth*1000:.1f}mm sign={sign:+.0f}, "
            f"baseline_effort={baseline:.3f}Nm on {joint_name}"
        )

        commanded_positions = {
            name: float(self.current_joint_positions[name])
            for name in self.current_joint_positions
            if name in self._single_arm_exotica_planner.controlled_joint_names
        }
        seed_positions = dict(commanded_positions)
        commanded_x, commanded_y, commanded_z = start_x, start_y, start_z
        travelled = 0.0
        consecutive_solver_failures = 0
        stable_contact_cycles = 0
        started_at = time.time()
        _last_progress_log = time.time()

        while rclpy.ok():
            effort = self.current_joint_efforts.get(joint_name, baseline) if effort_available else baseline
            spike = abs(effort - baseline)
            if effort_available and spike > threshold_nm:
                stable_contact_cycles += 1
                self.node.get_logger().info(
                    f"[{self.backend_kind}] Lateral contact spike: {spike:.3f}Nm > {threshold_nm}Nm "
                    f"(cycle {stable_contact_cycles}/{settling_cycles})"
                )
                if stable_contact_cycles >= max(1, int(settling_cycles)):
                    self._hold_current_arm_position()
                    self.node.get_logger().info(
                        f"[{self.backend_kind}] Lateral CONTACT confirmed: "
                        f"spike={spike:.3f}Nm, travelled={travelled*1000:.1f}mm, "
                        f"time={time.time()-started_at:.2f}s"
                    )
                    return True
            else:
                stable_contact_cycles = 0

            now = time.time()
            if now - _last_progress_log >= 0.5:
                self.node.get_logger().info(
                    f"[{self.backend_kind}] Lateral approach: "
                    f"travelled={travelled*1000:.1f}/{target_depth*1000:.1f}mm "
                    f"effort={effort:.3f}Nm baseline={baseline:.3f}Nm spike={spike:.3f}Nm"
                )
                _last_progress_log = now

            if travelled >= target_depth - 1e-4:
                break

            next_step = min(step_m, target_depth - travelled)
            if axis == 'x':
                commanded_x += sign * next_step
            elif axis == 'y':
                commanded_y += sign * next_step
            else:
                commanded_z += sign * next_step

            travelled += next_step

            target_joints = self._single_arm_exotica_planner.solve_pose_goal_joint_positions(
                seed_positions,
                [commanded_x, commanded_y, commanded_z, roll, pitch, yaw],
            )
            if not target_joints:
                consecutive_solver_failures += 1
                if consecutive_solver_failures >= max(1, int(max_solver_failures)):
                    self.node.get_logger().warning(
                        f"EXOTica lateral approach failed repeatedly: "
                        f"{self._single_arm_exotica_planner.last_error}"
                    )
                    self._hold_current_arm_position()
                    return False
                time.sleep(loop_dt)
                continue

            consecutive_solver_failures = 0
            filtered_command = {}
            for jname, solved_pos in target_joints.items():
                prev = float(commanded_positions.get(jname, solved_pos))
                delta = float(solved_pos) - prev
                delta *= float(command_alpha)
                delta = max(-abs(max_joint_step_rad), min(abs(max_joint_step_rad), delta))
                filtered_command[jname] = prev + delta

            self._publish_direct_joint_command(filtered_command)
            commanded_positions = dict(filtered_command)
            seed_positions = dict(filtered_command)
            time.sleep(loop_dt)

        self._hold_current_arm_position()
        elapsed = time.time() - started_at
        self.node.get_logger().warning(
            f"[{self.backend_kind}] Lateral approach finished max travel without contact after {elapsed:.2f}s."
        )
        return False

    def move_cartesian_realtime_exotica(
        self,
        target_fn,
        stop_fn=None,
        rate_hz: float = 50.0,
        max_step_m: float = 0.003,
        joint_smooth_alpha: float = 0.7,
        timeout_s: float = 60.0,
        max_joint_delta_rad: float = None,
        max_joint_step_rad: float = None,
    ) -> str:
        """Stream EXOTica IK in a real-time loop (TouchLab/teleoperation style).

        target_fn() is called each cycle. Returns (x, y, z, roll, pitch, yaw) in
        base_link frame, or None to stop cleanly ("DONE").
        stop_fn(), if provided, returns True to stop immediately ("STOPPED").

        Smoothness techniques matching TouchLab / Meta Quest teleoperation demos:
          1. Warm-start: each solve seeds from the previous solution — small
             incremental targets converge in 1-2 gradient iterations.
          2. Joint-space smoothing: q_cmd = q_prev*(1-alpha) + q_ik*alpha
             — prevents sudden joint jumps if IK hops between local minima.
          3. Step clamping: target clamped to max_step_m from current EE each cycle.
          4. max_joint_delta_rad: if set, rejects IK solutions that move any single
             joint by more than this threshold from the previous command — prevents
             runaway from wrong IK branch solutions.
          5. max_joint_step_rad: if set, clamps each published joint command step.
             This mirrors arm_teleop's final safety filter and prevents a small
             Cartesian target update from producing a large joint-space jump.

        Returns: "DONE" | "STOPPED" | "TIMEOUT" | "IK_FAIL"
        """
        if self._single_arm_exotica_planner is None or not self._single_arm_exotica_planner.available:
            self.node.get_logger().error(
                f"[{self.backend_kind}] move_cartesian_realtime_exotica: EXOTica unavailable"
            )
            return "IK_FAIL"

        import numpy as _np

        planner = self._single_arm_exotica_planner
        joint_names = planner.controlled_joint_names
        dt = 1.0 / max(float(rate_hz), 1.0)
        t_end = time.time() + float(timeout_s)
        ee_link = self.default_ik_link or ""

        seed = {n: float(self.current_joint_positions.get(n, 0.0)) for n in joint_names}
        prev_q = _np.array([seed[n] for n in joint_names], dtype=float)
        consecutive_ik_failures = 0

        while rclpy.ok() and time.time() < t_end:
            loop_start = time.time()

            if stop_fn is not None and stop_fn():
                return "STOPPED"

            target = target_fn()
            if target is None:
                return "DONE"

            tx, ty, tz, tr, tp, tyaw = (float(v) for v in target)
            tx = self._clamp_target_x(tx, "move_cartesian_realtime_exotica")
            tz = self._clamp_target_z(tz, "move_cartesian_realtime_exotica")

            # Step clamping: interpolate target so EE never jumps more than max_step_m
            if ee_link:
                try:
                    tf = self.tf_buffer.lookup_transform("base_link", ee_link, rclpy.time.Time())
                    ex = tf.transform.translation.x
                    ey = tf.transform.translation.y
                    ez = tf.transform.translation.z
                    dist = ((tx - ex) ** 2 + (ty - ey) ** 2 + (tz - ez) ** 2) ** 0.5
                    if dist > max_step_m and dist > 1e-6:
                        ratio = max_step_m / dist
                        tx = ex + (tx - ex) * ratio
                        ty = ey + (ty - ey) * ratio
                        tz = ez + (tz - ez) * ratio
                except Exception:
                    pass

            # Match arm_teleop's IK seeding: keep the full live joint state for
            # passive/non-controlled joints, then overlay the filtered command seed
            # for this arm. This keeps EXOTica's scene consistent when the linear
            # slide or other passive joints are not part of controlled_joint_names.
            full_seed = dict(self.current_joint_positions)
            full_seed.update(seed)
            ik_start = time.time()
            result = planner.solve_pose_goal_joint_positions(
                full_seed, [tx, ty, tz, tr, tp, tyaw], max_retries=3
            )
            ik_elapsed = time.time() - ik_start

            if result is None:
                consecutive_ik_failures += 1
                if consecutive_ik_failures > 10:
                    self.node.get_logger().error(
                        f"[{self.backend_kind}] move_cartesian_realtime_exotica: "
                        "10 consecutive IK failures — aborting"
                    )
                    return "IK_FAIL"
                elapsed = time.time() - loop_start
                if dt - elapsed > 0:
                    time.sleep(dt - elapsed)
                continue

            consecutive_ik_failures = 0

            # Joint-space smoothing with time-proportional alpha.
            # Scale alpha so filtering strength is consistent regardless of IK latency:
            # at 50 Hz alpha=0.7 ≈ same bandwidth as at 20 Hz alpha=0.92.
            q_new = _np.array([result[n] for n in joint_names], dtype=float)
            actual_dt = max(1e-3, time.time() - loop_start)
            alpha_t = 1.0 - (1.0 - float(joint_smooth_alpha)) ** (actual_dt / dt)

            # Wrong-branch rejection: if IK solution requires any joint to move more
            # than max_joint_delta_rad from the previous command, it has likely found
            # a different IK branch. Skip and hold position instead.
            if max_joint_delta_rad is not None:
                max_delta = float(_np.max(_np.abs(q_new - prev_q)))
                if max_delta > float(max_joint_delta_rad):
                    self.node.get_logger().warning(
                        f"[{self.backend_kind}] move_cartesian_realtime_exotica: "
                        f"IK joint jump {max_delta:.3f}rad > {max_joint_delta_rad:.3f}rad "
                        "— wrong branch detected, holding position"
                    )
                    elapsed = time.time() - loop_start
                    if dt - elapsed > 0:
                        time.sleep(dt - elapsed)
                    continue

            q_target = prev_q + alpha_t * (q_new - prev_q)
            if max_joint_step_rad is not None:
                max_step = abs(float(max_joint_step_rad))
                q_cmd = prev_q + _np.clip(q_target - prev_q, -max_step, max_step)
            else:
                q_cmd = q_target
            smoothed = {n: float(q_cmd[i]) for i, n in enumerate(joint_names)}

            # Adaptive lookahead: always slightly longer than the measured IK round-trip
            # so the JTC always has a pending target — prevents stop-and-lurch gaps.
            lookahead = max(0.12, ik_elapsed * 1.5)
            self._publish_direct_joint_command(smoothed, lookahead_s=lookahead)
            seed = smoothed
            prev_q = q_cmd

            elapsed = time.time() - loop_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

        return "TIMEOUT"

    def retract_z_exotica(
        self,
        distance_m: float,
        speed_mps: float = 0.05,
        rate_hz: float = 50.0,
    ) -> bool:
        """Move EE straight up by distance_m at speed_mps using EXOTica IK streaming.

        Pure Z-axis motion in base_link frame, orientation held constant.
        Returns True on completion, False on IK failure.
        """
        if self._single_arm_exotica_planner is None or not self._single_arm_exotica_planner.available:
            self.node.get_logger().warning(
                f"[{self.backend_kind}] retract_z_exotica: EXOTica unavailable — "
                "falling back to retract_relative_z"
            )
            return self.retract_relative_z(distance_m, velocity=float(speed_mps))

        dt = 1.0 / max(float(rate_hz), 1.0)
        step_m = float(speed_mps) * dt
        remaining = [float(distance_m)]
        ee_link = self.default_ik_link or ""

        def _target_fn():
            if remaining[0] <= 0.0:
                return None
            try:
                tf = self.tf_buffer.lookup_transform("base_link", ee_link, rclpy.time.Time())
                ex = tf.transform.translation.x
                ey = tf.transform.translation.y
                ez = tf.transform.translation.z
                q = tf.transform.rotation
                roll, pitch, yaw = self._quaternion_to_rpy(q.x, q.y, q.z, q.w)
            except Exception as exc:
                self.node.get_logger().warning(
                    f"[{self.backend_kind}] retract_z_exotica: TF lookup failed: {exc}"
                )
                return None
            this_step = min(step_m, remaining[0])
            remaining[0] -= this_step
            return (ex, ey, ez + this_step, roll, pitch, yaw)

        timeout = float(distance_m) / max(float(speed_mps), 1e-3) * 3.0 + 2.0
        result = self.move_cartesian_realtime_exotica(
            _target_fn,
            rate_hz=rate_hz,
            max_step_m=step_m * 2.0,
            joint_smooth_alpha=0.8,
            timeout_s=timeout,
        )
        return result in ("DONE", "STOPPED")

    def retract_relative_z(self, distance: float, velocity: float = 0.05) -> bool:
        try:
            transform = self.tf_buffer.lookup_transform("base_link", self.default_ik_link, rclpy.time.Time())
        except Exception:
            return False
        q = transform.transform.rotation
        return self.move_to_pose_exotica(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z + distance,
            {"qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w},
            velocity=velocity,
        )

    def retract_servo_z_closed_loop(
        self, distance: float, speed_mps: float = 0.03, timeout: float = None
    ) -> bool:
        if not self._ensure_servo_mode():
            return False
        if timeout is None:
            timeout = max(abs(distance) / 0.002, 5.0)
        try:
            start = self.tf_buffer.lookup_transform("base_link", self.default_ik_link, rclpy.time.Time())
        except Exception:
            return False
        start_z = start.transform.translation.z
        twist = TwistStamped()
        twist.header.frame_id = "base_link"
        twist.twist.linear.z = abs(speed_mps) if distance > 0 else -abs(speed_mps)
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            try:
                current = self.tf_buffer.lookup_transform("base_link", self.default_ik_link, rclpy.time.Time())
                if distance < 0.0 and self.min_tcp_z is not None:
                    if float(current.transform.translation.z) <= self.min_tcp_z + 1e-3:
                        self.node.get_logger().warning(
                            f"[{self.backend_kind}] Servo Z motion reached table limit "
                            f"z={self.min_tcp_z:.5f} for {self.default_ik_link}. Stopping."
                        )
                        self._publish_zero_twist()
                        return False
                travelled = abs(current.transform.translation.z - start_z)
                if travelled >= abs(distance) - 0.002:
                    self._publish_zero_twist()
                    return True
            except Exception:
                pass
            twist.header.stamp = self.node.get_clock().now().to_msg()
            self.servo_pub.publish(twist)
            time.sleep(0.033)
        self._publish_zero_twist()
        return False

    def move_servo_xy_closed_loop(
        self,
        dx: float,
        dy: float,
        speed_mps: float = 0.03,
        timeout: float = 30.0,
        stop_check=None,
        z_lock_m: float = None,
        z_gain: float = 1.2,
        z_speed_cap: float = 0.003,
    ):
        if not self._ensure_servo_mode():
            return False
        target_distance = math.hypot(dx, dy)
        if target_distance == 0.0:
            return True
        try:
            start = self.tf_buffer.lookup_transform("base_link", self.default_ik_link, rclpy.time.Time())
        except Exception:
            return False
        start_x = start.transform.translation.x
        start_y = start.transform.translation.y
        last_travelled = 0.0
        twist = TwistStamped()
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = dx / target_distance * speed_mps
        twist.twist.linear.y = dy / target_distance * speed_mps
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            if stop_check and stop_check():
                self._publish_zero_twist()
                return "STOPPED"
            try:
                current = self.tf_buffer.lookup_transform("base_link", self.default_ik_link, rclpy.time.Time())
                if z_lock_m is not None:
                    z_error = float(z_lock_m) - float(current.transform.translation.z)
                    twist.twist.linear.z = max(min(z_error * z_gain, z_speed_cap), -z_speed_cap)
                travelled = math.hypot(
                    current.transform.translation.x - start_x,
                    current.transform.translation.y - start_y,
                )
                last_travelled = travelled
                if travelled >= target_distance - 0.002:
                    self._publish_zero_twist()
                    return {"status": "DONE", "travelled_m": travelled, "target_m": target_distance}
            except Exception:
                pass
            twist.header.stamp = self.node.get_clock().now().to_msg()
            self.servo_pub.publish(twist)
            time.sleep(0.033)
        self._publish_zero_twist()
        return {"status": "TIMEOUT", "travelled_m": last_travelled, "target_m": target_distance}
