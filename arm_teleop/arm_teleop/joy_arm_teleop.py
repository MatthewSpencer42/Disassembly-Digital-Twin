#!/usr/bin/env python3
"""Joystick (gamepad) teleoperation for dual-arm setup via EXOTica IK.

Controls (Xbox-style gamepad via joy_node):
  LS horizontal (axes[0]) : Y translation  left (+) / right (-)
  LS vertical   (axes[1]) : X translation  forward (+) / backward (-)
  RS horizontal (axes[3]) : Yaw rotation   CCW (+) / CW (-)
  RS vertical   (axes[4]) : Z translation  up (+) / down (-)
  LB (buttons[4])         : Toggle active arm (xarm5 ↔ uf850, default xarm5)
  X  (buttons[2])         : Toggle selected arm enabled / disabled
  B  (buttons[1])         : Toggle both arms enabled / disabled
  RB (buttons[5])         : Toggle gripper open/close

Launch:
  ros2 launch arm_teleop joy_exotica_teleop.launch.py hardware_type:=real
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from arm_teleop.hand_math import clamp, quat_to_rpy
from nr_dual_arm_moveit_config.motion_backend import MotionBackend
from nr_dual_arm_moveit_config.exotica_planner import (
    ExoticaSingleArmPosePlanner,
    RemoteExoticaIKClient,
)

# ── Axes and button indices (Xbox gamepad via joy_node on Linux) ──────────────
_AX_LS_X = 0   # LS horizontal: left +1, right -1  → robot Y
_AX_LS_Y = 1   # LS vertical:   fwd  +1, back  -1  → robot X
_AX_RS_X = 3   # RS horizontal: left +1, right -1  → yaw
_AX_RS_Y = 4   # RS vertical:   fwd  +1, back  -1  → robot Z

_BTN_B = 1     # toggle both arms
_BTN_X = 2     # toggle selected arm enabled / disabled
_BTN_LB = 4    # toggle active arm
_BTN_RB = 5    # toggle gripper

_ARM_HOME_JOINTS: dict[str, dict[str, float]] = {
    "xarm5": {
        "xarm5_joint1": 0.0,
        "xarm5_joint2": 0.0,
        "xarm5_joint3": -math.pi / 2,
        "xarm5_joint4": math.pi / 2,
        "xarm5_joint5": 0.0,
    },
    "uf850": {
        "uf850_joint1": 0.0,
        "uf850_joint2": 0.0,
        "uf850_joint3": -math.pi / 2,
        "uf850_joint4": 0.0,
        "uf850_joint5": -math.pi / 2,
        "uf850_joint6": 0.0,
    },
}

_CONTROLLER_TIMEOUT_SEC = 0.5


class JoyArmTeleop(Node):
    def __init__(self):
        super().__init__("joy_arm_teleop")

        self.declare_parameter("hardware_type", "fake")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("linear_speed_mps", 0.05)
        self.declare_parameter("yaw_speed_rps", 0.3)
        self.declare_parameter("deadband", 0.05)
        self.declare_parameter("enable_xarm5", False)
        self.declare_parameter("enable_uf850", False)
        self.declare_parameter("workspace_min", [-0.70, -0.85, 0.00])
        self.declare_parameter("workspace_max", [1.25, 0.85, 1.30])
        self.declare_parameter("planner_init_delay_sec", 8.0)
        self.declare_parameter("planner_retry_sec", 2.0)
        self.declare_parameter("max_joint_step_rad", 0.08)
        self.declare_parameter("joint_command_gain", 0.35)
        self.declare_parameter("max_joint_velocity_rad_s", 0.8)

        self._hardware_type = str(self.get_parameter("hardware_type").value)
        self._rate_hz = max(float(self.get_parameter("control_rate_hz").value), 1.0)
        self._dt = 1.0 / self._rate_hz
        self._linear_speed = float(self.get_parameter("linear_speed_mps").value)
        self._yaw_speed = float(self.get_parameter("yaw_speed_rps").value)
        self._deadband = float(self.get_parameter("deadband").value)
        self._ws_min = [float(v) for v in self.get_parameter("workspace_min").value]
        self._ws_max = [float(v) for v in self.get_parameter("workspace_max").value]
        self._planner_init_delay = float(self.get_parameter("planner_init_delay_sec").value)
        self._planner_retry_sec = float(self.get_parameter("planner_retry_sec").value)
        self._max_joint_step = float(self.get_parameter("max_joint_step_rad").value)
        self._joint_alpha = float(self.get_parameter("joint_command_gain").value)
        self._max_joint_velocity = float(self.get_parameter("max_joint_velocity_rad_s").value)
        self._node_started_at = time.monotonic()

        self._cb_group = ReentrantCallbackGroup()

        xarm5_enabled = bool(self.get_parameter("enable_xarm5").value)
        uf850_enabled = bool(self.get_parameter("enable_uf850").value)

        self._arms: dict[str, dict] = {
            "xarm5": {
                "robot_name": "xarm5",
                "planner_group": "xarm5_arm_no_slide",
                "backend": MotionBackend(self, "xarm5_arm_no_slide", defer_exotica_init=True),
                "gripper_backend": MotionBackend(self, "xarm_gripper"),
                "gripper_joint_name": "xarm_gripper_right_drive_joint",
                "gripper_open": 0.0,
                "gripper_closed_pos": 0.854,
                "gripper_is_closed": False,
                "enabled": xarm5_enabled,
                "target_pos": None,
                "target_rpy": None,
                "seed_joints": None,
                "_last_filtered_joints": None,
                "_ik_lock": threading.Lock(),
                "last_planner_retry_time": 0.0,
                "last_planner_error_time": 0.0,
                "last_cmd_log_time": 0.0,
            },
            "uf850": {
                "robot_name": "uf850",
                "planner_group": "uf850_arm",
                "backend": MotionBackend(self, "uf850_arm", defer_exotica_init=True),
                "gripper_backend": MotionBackend(self, "rg6_gripper"),
                "gripper_joint_name": None,
                "gripper_open": -0.625,
                "gripper_closed_pos": 0.625,
                "gripper_is_closed": False,
                "enabled": uf850_enabled,
                "target_pos": None,
                "target_rpy": None,
                "seed_joints": None,
                "_last_filtered_joints": None,
                "_ik_lock": threading.Lock(),
                "last_planner_retry_time": 0.0,
                "last_planner_error_time": 0.0,
                "last_cmd_log_time": 0.0,
            },
        }

        self._active_arm_name: str = "xarm5"

        # EXOTica ready gate — planners are not created until the server signals
        # readiness via /exotica/ready.  This prevents both the "initialization
        # in progress" spurious error and the follow-up 30-second IK timeout.
        self._exotica_ready = False
        ready_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, "/exotica/ready", self._exotica_ready_cb, ready_qos,
            callback_group=self._cb_group,
        )

        # Joy input
        self._joy_msg: Joy | None = None
        self._joy_last_rx: float = 0.0
        self._joy_lock = threading.Lock()
        self._b_was_pressed = False
        self._x_was_pressed = False
        self._lb_was_pressed = False
        self._rb_was_pressed = False

        # Status publishers
        self._pub_xarm5_enabled = self.create_publisher(Bool, "/joy_teleop_status/xarm5_enabled", 10)
        self._pub_uf850_enabled = self.create_publisher(Bool, "/joy_teleop_status/uf850_enabled", 10)
        self._pub_active_arm = self.create_publisher(String, "/joy_teleop_status/active_arm", 10)
        self._pub_controller = self.create_publisher(Bool, "/joy_teleop_status/controller_connected", 10)
        self._pub_gripper_closed = self.create_publisher(Bool, "/joy_teleop_status/gripper_closed", 10)
        self._pub_exotica_ready = self.create_publisher(Bool, "/joy_teleop_status/exotica_ready", 10)

        # Services (mirroring exotica_arm_teleop pattern, keyed by arm name)
        self.create_service(
            SetBool, "~/set_xarm5_enabled",
            self._handle_set_xarm5_enabled, callback_group=self._cb_group,
        )
        self.create_service(
            SetBool, "~/set_uf850_enabled",
            self._handle_set_uf850_enabled, callback_group=self._cb_group,
        )
        self.create_service(
            SetBool, "~/set_all_arms_enabled",
            self._handle_set_all_arms_enabled, callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "~/go_home",
            self._handle_go_home, callback_group=self._cb_group,
        )

        self.create_subscription(
            Joy, "/joy", self._joy_cb, 10, callback_group=self._cb_group
        )
        self.create_timer(
            1.0 / self._rate_hz, self._tick, callback_group=self._cb_group
        )
        self.create_timer(
            0.2, self._publish_status, callback_group=self._cb_group
        )

        self.get_logger().info(
            f"JoyArmTeleop ready. Active arm: {self._active_arm_name}. "
            "Waiting for EXOTica IK server (/exotica/ready)... "
            "LS: XY | RS-vert: Z | RS-horiz: Yaw | X: toggle arm | "
            "B: toggle both | LB: switch arm | RB: gripper."
        )

    # ── EXOTica ready gate ─────────────────────────────────────────────────────

    def _exotica_ready_cb(self, msg: Bool):
        was_ready = self._exotica_ready
        self._exotica_ready = bool(msg.data)
        if self._exotica_ready and not was_ready:
            self.get_logger().info(
                "EXOTica IK server ready — joystick control is now active."
            )

    # ── Joy input ──────────────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        with self._joy_lock:
            self._joy_msg = msg
            self._joy_last_rx = time.monotonic()

    def _apply_deadband(self, value: float) -> float:
        if abs(value) < self._deadband:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self._deadband) / (1.0 - self._deadband)

    # ── Services ───────────────────────────────────────────────────────────────

    def _set_arm_enabled(self, arm_name: str, enabled: bool) -> tuple[bool, str]:
        arm = self._arms.get(arm_name)
        if arm is None:
            return False, f"Unknown arm: {arm_name}"
        arm["enabled"] = enabled
        arm["target_pos"] = None
        arm["target_rpy"] = None
        arm["seed_joints"] = None
        arm["_last_filtered_joints"] = None
        if enabled and self._active_arm_name != arm_name:
            # If the newly enabled arm is the only enabled one, switch to it.
            other = "uf850" if arm_name == "xarm5" else "xarm5"
            if not self._arms[other]["enabled"]:
                self._switch_to(arm_name)
        elif not enabled and self._active_arm_name == arm_name:
            # Active arm was disabled; switch to the other if available.
            other = "uf850" if arm_name == "xarm5" else "xarm5"
            if self._arms[other]["enabled"]:
                self._switch_to(other)
        state = "enabled" if enabled else "disabled"
        self.get_logger().info(f"[{arm_name}] {state} via service.")
        return True, f"{arm_name} {state}."

    def _handle_set_xarm5_enabled(self, request, response):
        response.success, response.message = self._set_arm_enabled("xarm5", bool(request.data))
        return response

    def _handle_set_uf850_enabled(self, request, response):
        response.success, response.message = self._set_arm_enabled("uf850", bool(request.data))
        return response

    def _handle_set_all_arms_enabled(self, request, response):
        enabled = bool(request.data)
        msgs = []
        for name in ("xarm5", "uf850"):
            ok, msg = self._set_arm_enabled(name, enabled)
            msgs.append(msg)
        response.success = True
        response.message = " ".join(msgs)
        return response

    def _handle_go_home(self, _request, response):
        arms_to_home = [
            (name, arm) for name, arm in self._arms.items() if arm["enabled"]
        ]
        for _, arm in self._arms.items():
            arm["enabled"] = False
            arm["target_pos"] = None
            arm["target_rpy"] = None
            arm["seed_joints"] = None
            arm["_last_filtered_joints"] = None

        def _do_home():
            for name, arm in arms_to_home:
                home = _ARM_HOME_JOINTS.get(name)
                if home:
                    self.get_logger().info(f"[{name}] Moving to home...")
                    ok = arm["backend"].move_to_joint_positions_direct(home, duration_sec=8.0)
                    self.get_logger().info(
                        f"[{name}] Home {'reached' if ok else 'FAILED'}."
                    )

        threading.Thread(target=_do_home, daemon=True).start()
        response.success = True
        response.message = "Enabled arms moving to home pose. All arms disabled."
        return response

    # ── Status publishing ──────────────────────────────────────────────────────

    def _publish_status(self):
        now = time.monotonic()
        with self._joy_lock:
            last_rx = self._joy_last_rx

        _b = Bool()
        _s = String()

        _b.data = (now - last_rx) < _CONTROLLER_TIMEOUT_SEC
        self._pub_controller.publish(_b)

        _b.data = self._arms["xarm5"]["enabled"]
        self._pub_xarm5_enabled.publish(_b)

        _b.data = self._arms["uf850"]["enabled"]
        self._pub_uf850_enabled.publish(_b)

        _s.data = self._active_arm_name
        self._pub_active_arm.publish(_s)

        _b.data = self._arms[self._active_arm_name]["gripper_is_closed"]
        self._pub_gripper_closed.publish(_b)

        _b.data = self._exotica_ready
        self._pub_exotica_ready.publish(_b)

    # ── Gripper ────────────────────────────────────────────────────────────────

    def _command_gripper(self, arm: dict, close: bool):
        target = arm["gripper_closed_pos"] if close else arm["gripper_open"]
        if arm["gripper_joint_name"] is not None:
            arm["gripper_backend"].move_to_joint_positions(
                {arm["gripper_joint_name"]: float(target)}, velocity=1.0
            )
        else:
            arm["gripper_backend"].move_gripper(target, velocity=1.0)
        arm["gripper_is_closed"] = close
        self.get_logger().info(
            f"[{arm['robot_name']}] Gripper {'closed' if close else 'open'}."
        )

    # ── EXOTica planner management ─────────────────────────────────────────────

    def _recover_planner(self, arm: dict) -> bool:
        backend = arm["backend"]
        planner = backend._single_arm_exotica_planner
        if planner is not None and planner.available:
            return True

        # Gate: do not attempt to connect until the EXOTica server signals it
        # is fully initialised for all groups.  This eliminates the race between
        # "no planner available / initialisation in progress" and the follow-up
        # 30-second IK timeout that occurs when the client queues a request while
        # the server is still loading a group.
        if not self._exotica_ready:
            return False

        now = time.monotonic()
        if now - self._node_started_at < self._planner_init_delay:
            return False
        if now - arm["last_planner_retry_time"] < self._planner_retry_sec:
            return False
        arm["last_planner_retry_time"] = now

        # Server is confirmed ready — use remote client (faster than local).
        backend._single_arm_exotica_planner = RemoteExoticaIKClient(
            self,
            arm["planner_group"],
            hardware_type=self._hardware_type,
            skip_ready_wait=True,  # safe: we already waited for /exotica/ready
        )
        planner = backend._single_arm_exotica_planner
        return planner is not None and planner.available

    # ── Target initialisation from TF ─────────────────────────────────────────

    def _init_target_from_tf(self, arm: dict) -> bool:
        backend = arm["backend"]
        if not backend.state_received.wait(timeout=0.0):
            return False
        try:
            t = backend.tf_buffer.lookup_transform(
                "base_link", backend.default_ik_link, rclpy.time.Time()
            )
        except Exception:
            return False
        arm["target_pos"] = [
            float(t.transform.translation.x),
            float(t.transform.translation.y),
            float(t.transform.translation.z),
        ]
        quat = [
            float(t.transform.rotation.x),
            float(t.transform.rotation.y),
            float(t.transform.rotation.z),
            float(t.transform.rotation.w),
        ]
        roll, pitch, yaw = quat_to_rpy(quat)
        arm["target_rpy"] = [roll, pitch, yaw]
        pos = arm["target_pos"]
        self.get_logger().info(
            f"[{arm['robot_name']}] Target init: "
            f"pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})  "
            f"yaw={math.degrees(yaw):.1f}°"
        )
        return True

    # ── IK solve and joint publish ─────────────────────────────────────────────

    def _solve_and_publish(self, arm: dict):
        backend = arm["backend"]
        if arm["target_pos"] is None or arm["target_rpy"] is None:
            return
        if not self._recover_planner(arm):
            now = time.monotonic()
            if now - arm["last_planner_error_time"] > 2.0:
                msg = (
                    "Waiting for EXOTica IK server..."
                    if not self._exotica_ready
                    else "Waiting for EXOTica planner to initialise..."
                )
                self.get_logger().info(f"[{arm['robot_name']}] {msg}")
                arm["last_planner_error_time"] = now
            return
        if not backend.state_received.wait(timeout=0.0):
            return

        lock = arm["_ik_lock"]
        if not lock.acquire(blocking=False):
            last = arm["_last_filtered_joints"]
            if last is not None:
                backend._publish_direct_joint_command(last)
            return

        try:
            planner = backend._single_arm_exotica_planner
            if planner is None or not planner.available:
                return

            pos = arm["target_pos"]
            roll, pitch, yaw = arm["target_rpy"]

            seed_joints = arm["seed_joints"]
            if seed_joints is None:
                seed_joints = {
                    name: float(backend.current_joint_positions.get(name, 0.0))
                    for name in planner.controlled_joint_names
                }
                arm["seed_joints"] = dict(seed_joints)

            full_positions = dict(backend.current_joint_positions)
            full_positions.update(seed_joints)

            result = planner.solve_pose_goal_joint_positions(
                full_positions,
                [pos[0], pos[1], pos[2], roll, pitch, yaw],
                max_retries=3,
            )
            if result is None:
                now = time.monotonic()
                if now - arm["last_cmd_log_time"] > 2.0:
                    self.get_logger().warning(
                        f"[{arm['robot_name']}] IK failed (will retry): {planner.last_error}"
                    )
                    arm["last_cmd_log_time"] = now
                return

            max_step = min(self._max_joint_step, self._max_joint_velocity / self._rate_hz)
            filtered = {}
            for name in planner.controlled_joint_names:
                prev = float(seed_joints.get(name, backend.current_joint_positions.get(name, 0.0)))
                solved = float(result[name])
                target_val = prev + self._joint_alpha * (solved - prev)
                delta = clamp(target_val - prev, -max_step, max_step)
                filtered[name] = prev + delta

            backend._publish_direct_joint_command(filtered)
            arm["seed_joints"] = dict(filtered)
            arm["_last_filtered_joints"] = dict(filtered)
        finally:
            lock.release()

    # ── Arm switching ──────────────────────────────────────────────────────────

    def _switch_to(self, arm_name: str):
        arm = self._arms[arm_name]
        self._active_arm_name = arm_name
        arm["target_pos"] = None
        arm["target_rpy"] = None
        arm["seed_joints"] = None
        arm["_last_filtered_joints"] = None
        self.get_logger().info(f"Active arm → {self._active_arm_name}")

    def _switch_arm(self):
        other = "uf850" if self._active_arm_name == "xarm5" else "xarm5"
        self._switch_to(other)

    def _toggle_selected_arm_enabled(self):
        arm_name = self._active_arm_name
        enabled = not self._arms[arm_name]["enabled"]
        self._set_arm_enabled(arm_name, enabled)
        if enabled:
            self._switch_to(arm_name)

    def _toggle_all_arms_enabled(self):
        enable_all = not all(self._arms[name]["enabled"] for name in ("xarm5", "uf850"))
        for name in ("xarm5", "uf850"):
            self._arms[name]["enabled"] = enable_all
        if enable_all:
            self._switch_to(self._active_arm_name)
        self.get_logger().info(
            "All arms %s." % ("enabled" if enable_all else "disabled")
        )

    # ── Main timer tick ────────────────────────────────────────────────────────

    def _tick(self):
        with self._joy_lock:
            joy = self._joy_msg

        # Button edge detection
        if joy is not None and len(joy.buttons) > max(_BTN_B, _BTN_X, _BTN_LB, _BTN_RB):
            b_now = bool(joy.buttons[_BTN_B])
            x_now = bool(joy.buttons[_BTN_X])
            lb_now = bool(joy.buttons[_BTN_LB])
            rb_now = bool(joy.buttons[_BTN_RB])

            if b_now and not self._b_was_pressed:
                self._toggle_all_arms_enabled()
            self._b_was_pressed = b_now

            if x_now and not self._x_was_pressed:
                self._toggle_selected_arm_enabled()
            self._x_was_pressed = x_now

            if lb_now and not self._lb_was_pressed:
                self._switch_arm()
            self._lb_was_pressed = lb_now

            arm = self._arms[self._active_arm_name]
            if rb_now and not self._rb_was_pressed:
                if arm["enabled"]:
                    threading.Thread(
                        target=self._command_gripper,
                        args=(arm, not arm["gripper_is_closed"]),
                        daemon=True,
                    ).start()
            self._rb_was_pressed = rb_now

        arm = self._arms[self._active_arm_name]
        if not arm["enabled"]:
            return

        # Initialise target pose from TF on first tick for this arm
        if arm["target_pos"] is None:
            if not self._init_target_from_tf(arm):
                return

        # Integrate joystick axes into target
        if joy is not None and len(joy.axes) > max(_AX_LS_X, _AX_LS_Y, _AX_RS_X, _AX_RS_Y):
            ax_ls_x = self._apply_deadband(float(joy.axes[_AX_LS_X]))
            ax_ls_y = self._apply_deadband(float(joy.axes[_AX_LS_Y]))
            ax_rs_x = self._apply_deadband(float(joy.axes[_AX_RS_X]))
            ax_rs_y = self._apply_deadband(float(joy.axes[_AX_RS_Y]))

            speed = self._linear_speed * self._dt
            dx = ax_ls_y * speed     # LS forward  → +X
            dy = ax_ls_x * speed     # LS left     → +Y
            dz = ax_rs_y * speed     # RS forward  → +Z
            dyaw = ax_rs_x * self._yaw_speed * self._dt

            pos = arm["target_pos"]
            arm["target_pos"] = [
                clamp(pos[0] + dx, self._ws_min[0], self._ws_max[0]),
                clamp(pos[1] + dy, self._ws_min[1], self._ws_max[1]),
                clamp(pos[2] + dz, self._ws_min[2], self._ws_max[2]),
            ]
            arm["target_rpy"][2] += dyaw

        self._solve_and_publish(arm)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = JoyArmTeleop()
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
