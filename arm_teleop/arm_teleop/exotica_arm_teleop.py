from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool, Trigger

from arm_teleop.hand_math import (
    clamp,
    mat_vec_mul,
    quat_conjugate,
    quat_multiply,
    quat_to_rpy,
    quaternion_slerp,
    rpy_to_quat,
    vec_add,
    vec_scale,
    vec_sub,
)
from nr_dual_arm_moveit_config.motion_backend import MotionBackend
from nr_dual_arm_moveit_config.exotica_planner import (
    ExoticaSingleArmPosePlanner,
    RemoteExoticaIKClient,
)


_UF850_HOME_JOINTS = {
    "uf850_joint1": 0.0,
    "uf850_joint2": 0.0,
    "uf850_joint3": -math.pi / 2,
    "uf850_joint4": 0.0,
    "uf850_joint5": -math.pi / 2,
    "uf850_joint6": 0.0,
}
_XARM5_HOME_JOINTS = {
    "xarm5_joint1": 0.0,
    "xarm5_joint2": 0.0,
    "xarm5_joint3": -math.pi / 2,
    "xarm5_joint4": math.pi / 2,
    "xarm5_joint5": 0.0,
}
_ARM_HOME_JOINTS = {
    "uf850": _UF850_HOME_JOINTS,
    "xarm5": _XARM5_HOME_JOINTS,
}


class ExoticaArmTeleop(Node):
    def __init__(self):
        super().__init__("exotica_arm_teleop")

        self.declare_parameter("control_rate_hz", 15.0)
        self.declare_parameter("hardware_type", "fake")
        self.declare_parameter("tracking_timeout_sec", 0.35)
        self.declare_parameter("translation_scale", 1.0)
        self.declare_parameter("translation_scale_xyz", [1.4, 1.4, 1.8])
        self.declare_parameter("position_filter_alpha", 0.25)
        self.declare_parameter("orientation_filter_alpha", 0.2)
        self.declare_parameter("joint_filter_alpha", 0.35)
        self.declare_parameter("joint_command_gain", 0.35)
        self.declare_parameter("max_joint_velocity_rad_s", 0.8)
        self.declare_parameter("max_joint_step_rad", 0.08)
        self.declare_parameter("max_target_step_m", 0.03)
        self.declare_parameter("planner_retry_sec", 2.0)
        self.declare_parameter("planner_init_delay_sec", 8.0)
        self.declare_parameter("enable_uf850", True)
        self.declare_parameter("enable_xarm5", True)
        self.declare_parameter("uf850.hand", "right")
        self.declare_parameter("xarm5.hand", "left")
        self.declare_parameter("workspace_min", [-0.70, -0.85, 0.00])
        self.declare_parameter("workspace_max", [1.25, 0.85, 1.30])
        self.declare_parameter("uf850.min_tcp_x", 0.647599)
        self.declare_parameter("uf850.max_tcp_x", 1.24581)
        self.declare_parameter("uf850.min_tcp_z", 0.92962)
        self.declare_parameter("uf850.max_tcp_z", 1.30)
        self.declare_parameter("xarm5.min_tcp_x", 0.637258)
        self.declare_parameter("xarm5.max_tcp_x", 1.16441)
        self.declare_parameter("xarm5.min_tcp_z", 0.92706)
        self.declare_parameter("xarm5.max_tcp_z", 1.28452)
        self.declare_parameter("camera_to_base_rotation", [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0])
        self.declare_parameter("uf850.track_orientation", True)
        self.declare_parameter("xarm5.track_orientation", False)
        # xArm5 is 5-DOF (joints: Z-Y-Y-X-Y).  Roll is kinematically coupled and cannot
        # be commanded independently (no terminal wrist-roll joint).  Yaw and pitch can be
        # enabled individually; each zeroes the other unused delta components before passing
        # the reconstructed quaternion to EXOTica.
        self.declare_parameter("xarm5.track_yaw", True)
        self.declare_parameter("xarm5.track_pitch", False)
        self.declare_parameter("gripper_command_alpha", 0.45)
        self.declare_parameter("gripper_command_deadband", 0.005)
        self.declare_parameter("gripper_max_step", 0.06)
        self.declare_parameter("gripper_aperture_alpha", 0.12)
        self.declare_parameter("gripper_aperture_hysteresis", 0.06)
        # Remap aperture so physical finger-touch (aperture_min) = fully closed
        # and fully spread (aperture_max) = fully open.  Tune these to match your
        # hand tracking's actual output range.
        self.declare_parameter("gripper_aperture_min", 0.10)
        self.declare_parameter("gripper_aperture_max", 0.90)

        self._hardware_type = str(self.get_parameter("hardware_type").value)
        self._rate_hz = max(float(self.get_parameter("control_rate_hz").value), 1.0)
        self._tracking_timeout = float(self.get_parameter("tracking_timeout_sec").value)
        self._translation_scale = float(self.get_parameter("translation_scale").value)
        self._translation_scale_xyz = [float(v) for v in self.get_parameter("translation_scale_xyz").value]
        self._position_alpha = clamp(float(self.get_parameter("position_filter_alpha").value), 0.01, 1.0)
        self._orientation_alpha = clamp(float(self.get_parameter("orientation_filter_alpha").value), 0.01, 1.0)
        self._joint_alpha = clamp(float(self.get_parameter("joint_command_gain").value), 0.01, 1.0)
        self._max_joint_velocity = max(float(self.get_parameter("max_joint_velocity_rad_s").value), 0.01)
        self._max_joint_step = max(float(self.get_parameter("max_joint_step_rad").value), 0.001)
        self._max_target_step = max(float(self.get_parameter("max_target_step_m").value), 0.002)
        self._planner_retry_sec = max(float(self.get_parameter("planner_retry_sec").value), 0.5)
        self._planner_init_delay_sec = max(float(self.get_parameter("planner_init_delay_sec").value), 0.0)
        self._workspace_min = [float(v) for v in self.get_parameter("workspace_min").value]
        self._workspace_max = [float(v) for v in self.get_parameter("workspace_max").value]
        self._node_started_at = time.monotonic()
        self._gripper_alpha = clamp(float(self.get_parameter("gripper_command_alpha").value), 0.01, 1.0)
        self._gripper_deadband = max(0.0, float(self.get_parameter("gripper_command_deadband").value))
        self._gripper_max_step = max(float(self.get_parameter("gripper_max_step").value), 0.001)
        self._gripper_aperture_alpha = clamp(float(self.get_parameter("gripper_aperture_alpha").value), 0.01, 1.0)
        self._gripper_aperture_hysteresis = max(0.0, float(self.get_parameter("gripper_aperture_hysteresis").value))
        self._gripper_aperture_min = float(self.get_parameter("gripper_aperture_min").value)
        self._gripper_aperture_max = float(self.get_parameter("gripper_aperture_max").value)

        rotation_raw = [float(v) for v in self.get_parameter("camera_to_base_rotation").value]
        self._camera_to_base = [
            rotation_raw[0:3],
            rotation_raw[3:6],
            rotation_raw[6:9],
        ]

        self._hand_state = {
            "right": {"msg": None, "stamp": 0.0, "stable_since": 0.0},
            "left": {"msg": None, "stamp": 0.0, "stable_since": 0.0},
        }
        self._gripper_aperture_state = {
            "right": {"value": 0.0, "stamp": 0.0},
            "left": {"value": 0.0, "stamp": 0.0},
        }
        # Per-hand low-pass filter state and hysteresis tracking.
        self._gripper_aperture_filter: dict[str, float | None] = {"right": None, "left": None}
        self._gripper_last_committed_aperture: dict[str, float | None] = {"right": None, "left": None}
        # Minimum seconds of continuous hand tracking required before calibration.
        # Prevents calibrating on the first noisy frame when the hand enters view.
        self._calibration_stability_sec = 0.5
        self._cb_group = ReentrantCallbackGroup()
        self._fist_state = {"right": False, "left": False}
        self._pinky_pinch_state = {"right": False, "left": False}
        self._status_pub = {
            "right": self.create_publisher(Bool, "/teleop_status/right_arm_enabled", 10),
            "left": self.create_publisher(Bool, "/teleop_status/left_arm_enabled", 10),
        }
        uf850_enabled = bool(self.get_parameter("enable_uf850").value)
        xarm5_enabled = bool(self.get_parameter("enable_xarm5").value)
        uf850_hand = self._normalize_hand_name(str(self.get_parameter("uf850.hand").value), "uf850", "right")
        xarm5_hand = self._normalize_hand_name(str(self.get_parameter("xarm5.hand").value), "xarm5", "left")
        if uf850_enabled and xarm5_enabled and uf850_hand == xarm5_hand:
            fallback = "left" if uf850_hand == "right" else "right"
            self.get_logger().warning(
                f"uf850.hand and xarm5.hand both resolved to '{uf850_hand}'. "
                f"Forcing xarm5.hand to '{fallback}'."
            )
            xarm5_hand = fallback

        uf850_arm = {
            "robot_name": "uf850",
            "label": f"{uf850_hand}/uf850",
            "planner_group": "uf850_arm",
            "backend": MotionBackend(self, "uf850_arm", defer_exotica_init=True),
            "min_tcp_x": float(self.get_parameter("uf850.min_tcp_x").value),
            "max_tcp_x": float(self.get_parameter("uf850.max_tcp_x").value),
            "min_tcp_z": float(self.get_parameter("uf850.min_tcp_z").value),
            "max_tcp_z": float(self.get_parameter("uf850.max_tcp_z").value),
            "track_orientation": bool(self.get_parameter("uf850.track_orientation").value),
            "origin_hand_pos": None,
            "origin_hand_quat": None,
            "origin_robot_pos": None,
            "origin_robot_quat": None,
            "target_pose": None,
            "seed_joints": None,
            "last_command_time": 0.0,
            "last_planner_retry_time": 0.0,
            "last_planner_error_time": 0.0,
            "last_limit_log_time": 0.0,
            "teleop_allowed": uf850_enabled,
            "enabled": False,
            "calibrated": False,
            "gripper_backend": MotionBackend(self, "rg6_gripper"),
            "gripper_open_position": -0.625,
            "gripper_closed_position": 0.625,
            "gripper_closed": False,
            "gripper_follow_hand": False,
            "last_gripper_command": None,
            "_ik_lock": threading.Lock(),
            "_last_filtered_joints": None,
        }
        xarm5_arm = {
            "robot_name": "xarm5",
            "label": f"{xarm5_hand}/xarm5",
            "planner_group": "xarm5_arm_no_slide",
            "backend": MotionBackend(self, "xarm5_arm_no_slide", defer_exotica_init=True),
            "min_tcp_x": float(self.get_parameter("xarm5.min_tcp_x").value),
            "max_tcp_x": float(self.get_parameter("xarm5.max_tcp_x").value),
            "min_tcp_z": float(self.get_parameter("xarm5.min_tcp_z").value),
            "max_tcp_z": float(self.get_parameter("xarm5.max_tcp_z").value),
            "track_orientation": False,
            "track_yaw": bool(self.get_parameter("xarm5.track_yaw").value),
            "track_pitch": bool(self.get_parameter("xarm5.track_pitch").value),
            "origin_hand_pos": None,
            "origin_hand_quat": None,
            "origin_robot_pos": None,
            "origin_robot_quat": None,
            "target_pose": None,
            "seed_joints": None,
            "last_command_time": 0.0,
            "last_planner_retry_time": 0.0,
            "last_planner_error_time": 0.0,
            "last_limit_log_time": 0.0,
            "teleop_allowed": xarm5_enabled,
            "enabled": False,
            "calibrated": False,
            "gripper_backend": MotionBackend(self, "xarm_gripper"),
            "gripper_joint_name": "xarm_gripper_right_drive_joint",
            "gripper_open_position": 0.0,
            "gripper_closed_position": 0.854,
            "gripper_closed": False,
            "gripper_follow_hand": True,
            "last_gripper_command": None,
            "_ik_lock": threading.Lock(),
            "_last_filtered_joints": None,
        }
        self._arms = {"right": None, "left": None}
        for hand_name, arm in sorted(
            [(uf850_hand, uf850_arm), (xarm5_hand, xarm5_arm)],
            key=lambda item: bool(item[1]["teleop_allowed"]),
        ):
            self._arms[hand_name] = arm

        self.create_subscription(
            PoseStamped,
            "/teleop_hand_tracking/right/wrist",
            self._right_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            PoseStamped,
            "/teleop_hand_tracking/left/wrist",
            self._left_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Bool,
            "/teleop_hand_tracking/right/fist",
            self._right_fist_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Bool,
            "/teleop_hand_tracking/left/fist",
            self._left_fist_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Bool,
            "/teleop_hand_tracking/right/pinky_pinch",
            self._right_pinky_pinch_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Bool,
            "/teleop_hand_tracking/left/pinky_pinch",
            self._left_pinky_pinch_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Float32,
            "/teleop_hand_tracking/right/gripper_aperture",
            self._right_gripper_aperture_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Float32,
            "/teleop_hand_tracking/left/gripper_aperture",
            self._left_gripper_aperture_cb,
            10,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger,
            "~/recalibrate",
            self._handle_recalibrate,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger,
            "~/go_home",
            self._handle_go_home,
            callback_group=self._cb_group,
        )
        self.create_service(
            SetBool,
            "~/set_right_arm_enabled",
            self._handle_set_right_arm_enabled,
            callback_group=self._cb_group,
        )
        self.create_service(
            SetBool,
            "~/set_left_arm_enabled",
            self._handle_set_left_arm_enabled,
            callback_group=self._cb_group,
        )
        self.create_service(
            SetBool,
            "~/set_all_arms_enabled",
            self._handle_set_all_arms_enabled,
            callback_group=self._cb_group,
        )

        self._last_status_log = 0.0
        self.timer = self.create_timer(
            1.0 / self._rate_hz,
            self._tick,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"EXOTica arm teleop node started. uf850 is on {uf850_hand}, xarm5 is on {xarm5_hand}. "
            "Arm teleop is controlled from the UI/services. xarm5 gripper follows hand aperture when available; "
            "pinky pinch remains a fallback toggle. "
            "xarm5 orientation: yaw=" + str(bool(self.get_parameter("xarm5.track_yaw").value))
            + " pitch=" + str(bool(self.get_parameter("xarm5.track_pitch").value))
            + " roll=False (5-DOF kinematic limit)."
        )
        self._publish_arm_enabled_status()

    def _normalize_hand_name(self, value: str, robot_name: str, default: str) -> str:
        hand = value.strip().lower()
        if hand in ("left", "right"):
            return hand
        self.get_logger().warning(
            f"Invalid {robot_name}.hand='{value}'. Falling back to '{default}'."
        )
        return default

    def _right_cb(self, msg: PoseStamped):
        self._store_hand_state("right", msg)

    def _left_cb(self, msg: PoseStamped):
        self._store_hand_state("left", msg)

    def _right_fist_cb(self, msg: Bool):
        self._fist_state["right"] = bool(msg.data)

    def _left_fist_cb(self, msg: Bool):
        self._fist_state["left"] = bool(msg.data)

    def _right_pinky_pinch_cb(self, msg: Bool):
        self._handle_pinky_pinch("right", bool(msg.data))

    def _left_pinky_pinch_cb(self, msg: Bool):
        self._handle_pinky_pinch("left", bool(msg.data))

    def _right_gripper_aperture_cb(self, msg: Float32):
        self._store_gripper_aperture("right", float(msg.data))

    def _left_gripper_aperture_cb(self, msg: Float32):
        self._store_gripper_aperture("left", float(msg.data))

    def _store_hand_state(self, hand: str, msg: PoseStamped):
        now = time.monotonic()
        prev_stamp = self._hand_state[hand]["stamp"]
        # If tracking was lost (gap > 2× timeout), restart the stability clock so
        # we don't calibrate on the first noisy frame after hand re-detection.
        if now - prev_stamp > 2.0 * self._tracking_timeout:
            self._hand_state[hand]["stable_since"] = now
        self._hand_state[hand]["msg"] = msg
        self._hand_state[hand]["stamp"] = now

    def _store_gripper_aperture(self, hand: str, value: float):
        raw = clamp(float(value), 0.0, 1.0)
        # Low-pass filter the raw measurement to reduce sensor noise.
        prev = self._gripper_aperture_filter[hand]
        filtered = raw if prev is None else prev + self._gripper_aperture_alpha * (raw - prev)
        self._gripper_aperture_filter[hand] = filtered
        # Remap so that physical finger-touch (aperture_min) → 0.0 and fully spread
        # (aperture_max) → 1.0.  This ensures the gripper can reach fully closed even
        # when the tracker's minimum output is non-zero.
        span = self._gripper_aperture_max - self._gripper_aperture_min
        if span > 1e-6:
            remapped = clamp((filtered - self._gripper_aperture_min) / span, 0.0, 1.0)
        else:
            remapped = filtered
        self._gripper_aperture_state[hand]["value"] = remapped
        self._gripper_aperture_state[hand]["stamp"] = time.monotonic()

    def _set_arm_enabled(self, hand: str, enabled: bool) -> tuple[bool, str]:
        arm = self._arms.get(hand)
        if arm is None:
            return False, f"No arm is assigned to the {hand} hand."
        if not arm["teleop_allowed"]:
            return False, f"[{arm['label']}] Teleoperation is disabled by launch configuration."
        if arm["enabled"] == enabled:
            state = "enabled" if enabled else "disabled"
            return True, f"[{arm['label']}] Teleoperation already {state}."
        arm["enabled"] = enabled
        arm["calibrated"] = False
        arm["target_pose"] = None
        arm["_last_filtered_joints"] = None
        arm["last_gripper_command"] = None
        self._gripper_aperture_filter[hand] = None
        self._gripper_last_committed_aperture[hand] = None
        state = "ENABLED" if enabled else "DISABLED"
        self.get_logger().info(f"[{arm['label']}] Teleoperation {state} via UI/service command.")
        self._publish_arm_enabled_status()
        return True, f"[{arm['label']}] Teleoperation {state}."

    def _set_all_arms_enabled(self, enabled: bool) -> tuple[bool, str]:
        messages = []
        changed = False
        for hand in ("right", "left"):
            arm = self._arms.get(hand)
            if arm is None:
                continue
            ok, msg = self._set_arm_enabled(hand, enabled)
            if ok:
                changed = True
            messages.append(msg)
        if not messages:
            return False, "No teleop arms are configured."
        return changed or enabled is False, " ".join(messages)

    def _handle_pinky_pinch(self, hand: str, current: bool):
        previous = self._pinky_pinch_state[hand]
        self._pinky_pinch_state[hand] = current
        if current and not previous:
            arm = self._arms[hand]
            if arm is None:
                return
            if not arm["teleop_allowed"]:
                return
            if arm.get("gripper_follow_hand", False):
                return
            arm["gripper_closed"] = not arm["gripper_closed"]
            closed = arm["gripper_closed"]
            state = "closed" if closed else "open"
            if self._command_gripper(arm, closed):
                self.get_logger().info(f"[{arm['label']}] Gripper toggled {state} via pinky pinch.")
            else:
                self.get_logger().warning(f"[{arm['label']}] Failed to toggle gripper {state}.")

    def _publish_arm_enabled_status(self):
        for hand_name, arm in self._arms.items():
            if arm is None:
                continue
            msg = Bool()
            msg.data = bool(arm["enabled"] and arm["teleop_allowed"])
            self._status_pub[hand_name].publish(msg)

    def _handle_set_right_arm_enabled(self, request, response):
        response.success, response.message = self._set_arm_enabled("right", bool(request.data))
        return response

    def _handle_set_left_arm_enabled(self, request, response):
        response.success, response.message = self._set_arm_enabled("left", bool(request.data))
        return response

    def _handle_set_all_arms_enabled(self, request, response):
        response.success, response.message = self._set_all_arms_enabled(bool(request.data))
        return response

    def _reset_input_state(self):
        for hand_name in self._hand_state:
            self._fist_state[hand_name] = False
            self._pinky_pinch_state[hand_name] = False
            self._hand_state[hand_name]["msg"] = None
            self._hand_state[hand_name]["stamp"] = 0.0
            self._hand_state[hand_name]["stable_since"] = 0.0
            self._gripper_aperture_state[hand_name]["value"] = 0.0
            self._gripper_aperture_state[hand_name]["stamp"] = 0.0
            self._gripper_aperture_filter[hand_name] = None
            self._gripper_last_committed_aperture[hand_name] = None

    def _command_gripper(self, arm: dict, closed: bool) -> bool:
        target = arm["gripper_closed_position"] if closed else arm["gripper_open_position"]
        backend = arm["gripper_backend"]
        if arm["label"] == "right/uf850":
            return backend.move_gripper(target, velocity=0.5)
        return backend.move_to_joint_positions(
            {arm["gripper_joint_name"]: float(target)},
            velocity=0.5,
        )

    def _current_gripper_aperture(self, hand: str):
        entry = self._gripper_aperture_state[hand]
        if time.monotonic() - entry["stamp"] > self._tracking_timeout:
            return None
        return float(entry["value"])

    def _update_gripper_follow(self, hand_name: str, arm: dict):
        if not arm.get("gripper_follow_hand", False):
            return
        aperture = self._current_gripper_aperture(hand_name)
        if aperture is None:
            return

        # Hysteresis gate: only send a command when the (already low-pass filtered)
        # aperture moves more than gripper_aperture_hysteresis from the last sent value.
        # The strong alpha=0.12 filter absorbs sensor noise; the hysteresis keeps the
        # gripper still when the hand is steady despite residual filter wobble.
        last_committed = self._gripper_last_committed_aperture[hand_name]
        if last_committed is not None and abs(aperture - last_committed) < self._gripper_aperture_hysteresis:
            return

        self._gripper_last_committed_aperture[hand_name] = aperture

        target = clamp(
            arm["gripper_closed_position"]
            + aperture * (arm["gripper_open_position"] - arm["gripper_closed_position"]),
            min(arm["gripper_open_position"], arm["gripper_closed_position"]),
            max(arm["gripper_open_position"], arm["gripper_closed_position"]),
        )
        arm["gripper_backend"]._publish_direct_joint_command({arm["gripper_joint_name"]: float(target)})
        arm["last_gripper_command"] = float(target)

    def _handle_recalibrate(self, _request, response):
        for arm in self._arms.values():
            if arm is None:
                continue
            arm["calibrated"] = False
            arm["origin_hand_pos"] = None
            arm["origin_hand_quat"] = None
            arm["origin_robot_pos"] = None
            arm["origin_robot_quat"] = None
            arm["target_pose"] = None
            arm["seed_joints"] = None
            arm["_last_filtered_joints"] = None
            arm["last_gripper_command"] = None
        self._reset_input_state()
        response.success = True
        response.message = "Teleoperation calibration cleared. Hold both hands in the new neutral pose."
        self.get_logger().info(response.message)
        return response

    def _handle_go_home(self, _request, response):
        """Disable teleop, move arms to home pose in background, clear calibration."""
        arms_to_home = []
        for arm in self._arms.values():
            if arm is None:
                continue
            arm["enabled"] = False
            arm["calibrated"] = False
            arm["origin_hand_pos"] = None
            arm["origin_hand_quat"] = None
            arm["origin_robot_pos"] = None
            arm["origin_robot_quat"] = None
            arm["target_pose"] = None
            arm["seed_joints"] = None
            arm["_last_filtered_joints"] = None
            arm["last_gripper_command"] = None
            if arm["teleop_allowed"]:
                arms_to_home.append(arm)
        self._reset_input_state()
        self._publish_arm_enabled_status()

        def _do_home():
            for arm in arms_to_home:
                home_joints = _ARM_HOME_JOINTS.get(arm["robot_name"])
                if home_joints is None:
                    continue
                self.get_logger().info(f"[{arm['label']}] Moving to home pose...")
                ok = arm["backend"].move_to_joint_positions(home_joints, velocity=0.3)
                if ok:
                    self.get_logger().info(f"[{arm['label']}] Home pose reached.")
                else:
                    self.get_logger().warning(f"[{arm['label']}] Failed to reach home pose.")

        threading.Thread(target=_do_home, daemon=True).start()
        response.success = True
        response.message = "Arms disabled, moving to home. Teleop will recalibrate on next hand detection."
        self.get_logger().info(response.message)
        return response

    def _hand_pose(self, hand: str):
        entry = self._hand_state[hand]
        if entry["msg"] is None:
            return None
        if time.monotonic() - entry["stamp"] > self._tracking_timeout:
            return None
        msg = entry["msg"]
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        return pos, quat

    def _lookup_robot_pose(self, backend: MotionBackend):
        try:
            transform = backend.tf_buffer.lookup_transform("base_link", backend.default_ik_link, rclpy.time.Time())
        except Exception:
            return None
        pos = [
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            float(transform.transform.translation.z),
        ]
        quat = [
            float(transform.transform.rotation.x),
            float(transform.transform.rotation.y),
            float(transform.transform.rotation.z),
            float(transform.transform.rotation.w),
        ]
        return pos, quat

    def _recover_planner_if_needed(self, arm: dict) -> bool:
        backend = arm["backend"]
        planner = backend._single_arm_exotica_planner
        if planner is not None and planner.available:
            return True

        now = time.monotonic()
        if now - self._node_started_at < self._planner_init_delay_sec:
            return False
        if now - arm["last_planner_retry_time"] < self._planner_retry_sec:
            return False
        arm["last_planner_retry_time"] = now

        if self.count_publishers("/exotica/ready") > 0:
            self.get_logger().info(
                f"[{arm['label']}] Connecting to remote EXOTica IK server for {arm['planner_group']}."
            )
            backend._single_arm_exotica_planner = RemoteExoticaIKClient(
                self,
                arm["planner_group"],
                hardware_type=self._hardware_type,
                skip_ready_wait=True,
            )
        else:
            self.get_logger().info(
                f"[{arm['label']}] Initializing local EXOTica planner for {arm['planner_group']}."
            )
            backend._single_arm_exotica_planner = ExoticaSingleArmPosePlanner(
                self,
                arm["planner_group"],
                hardware_type=self._hardware_type,
            )
        return bool(
            backend._single_arm_exotica_planner is not None
            and backend._single_arm_exotica_planner.available
        )

    def _calibrate_arm(self, hand_name: str, arm: dict) -> bool:
        hand_pose = self._hand_pose(hand_name)
        if hand_pose is None:
            return False

        # Wait until tracking has been stable long enough to avoid calibrating on
        # the first noisy frame when the hand enters the camera view.
        stable_since = self._hand_state[hand_name]["stable_since"]
        tracking_age = time.monotonic() - stable_since
        if tracking_age < self._calibration_stability_sec:
            return False

        if not arm["backend"].state_received.wait(timeout=0.01):
            return False
        robot_pose = self._lookup_robot_pose(arm["backend"])
        if robot_pose is None:
            return False
        if not self._recover_planner_if_needed(arm):
            planner = arm["backend"]._single_arm_exotica_planner
            now = time.monotonic()
            if now - arm["last_planner_error_time"] > 2.0:
                self.get_logger().info(
                    f"[{arm['label']}] Waiting for EXOTica planner availability: "
                    f"{getattr(planner, 'last_error', 'not initialized')}"
                )
                arm["last_planner_error_time"] = now
            return False
        arm["origin_hand_pos"], arm["origin_hand_quat"] = hand_pose
        arm["origin_robot_pos"], arm["origin_robot_quat"] = robot_pose
        arm["target_pose"] = robot_pose
        planner = arm["backend"]._single_arm_exotica_planner
        arm["seed_joints"] = {
            name: float(arm["backend"].current_joint_positions.get(name, 0.0))
            for name in planner.controlled_joint_names
        }
        arm["calibrated"] = True
        hp = arm["origin_hand_pos"]
        rp = arm["origin_robot_pos"]
        self.get_logger().info(
            f"[{arm['label']}] Calibrated. "
            f"Hand origin: ({hp[0]:.3f}, {hp[1]:.3f}, {hp[2]:.3f})  "
            f"TCP origin:  ({rp[0]:.3f}, {rp[1]:.3f}, {rp[2]:.3f})"
        )
        return True

    def _clamp_position(self, arm: dict, position):
        min_x = float(arm.get("min_tcp_x", self._workspace_min[0]))
        max_x = float(arm.get("max_tcp_x", self._workspace_max[0]))
        min_z = max(
            float(self._workspace_min[2]),
            float(arm.get("min_tcp_z", arm["backend"].min_tcp_z or self._workspace_min[2])),
        )
        max_z = min(float(self._workspace_max[2]), float(arm.get("max_tcp_z", self._workspace_max[2])))
        clamped_min_x = max(self._workspace_min[0], min_x)
        clamped_max_x = min(self._workspace_max[0], max_x)
        clamped_x = clamp(position[0], clamped_min_x, clamped_max_x)
        clamped_y = position[1]  # Y (left/right) is unrestricted — let IK/joint limits handle it
        clamped_z = clamp(position[2], min_z, max_z)

        now = time.monotonic()
        if now - arm["last_limit_log_time"] > 1.0:
            if clamped_x != position[0]:
                direction = "backward" if position[0] < clamped_min_x else "forward"
                limit_x = clamped_min_x if direction == "backward" else clamped_max_x
                self.get_logger().warning(
                    f"[{arm['label']}] TCP {direction} X limit reached: requested x={position[0]:.5f}, "
                    f"clamped to x={limit_x:.5f}."
                )
                arm["last_limit_log_time"] = now
            elif clamped_z != position[2] and position[2] < min_z:
                self.get_logger().warning(
                    f"[{arm['label']}] TCP lower Z limit reached: requested z={position[2]:.5f}, "
                    f"clamped to z={min_z:.5f}."
                )
                arm["last_limit_log_time"] = now
            elif clamped_z != position[2] and position[2] > max_z:
                self.get_logger().warning(
                    f"[{arm['label']}] TCP upper Z limit reached: requested z={position[2]:.5f}, "
                    f"clamped to z={max_z:.5f}."
                )
                arm["last_limit_log_time"] = now

        return [clamped_x, clamped_y, clamped_z]

    def _target_from_hand(self, hand_name: str, arm: dict):
        hand_pose = self._hand_pose(hand_name)
        if hand_pose is None:
            return None
        current_hand_pos, current_hand_quat = hand_pose
        hand_delta = vec_sub(current_hand_pos, arm["origin_hand_pos"])
        scaled_hand_delta = [
            hand_delta[0] * self._translation_scale * self._translation_scale_xyz[0],
            hand_delta[1] * self._translation_scale * self._translation_scale_xyz[1],
            hand_delta[2] * self._translation_scale * self._translation_scale_xyz[2],
        ]
        base_delta = mat_vec_mul(self._camera_to_base, scaled_hand_delta)
        unclamped_position = vec_add(arm["origin_robot_pos"], base_delta)
        if arm["target_pose"] is not None:
            previous_position = arm["target_pose"][0]
            step = vec_sub(unclamped_position, previous_position)
            step_mag = math.sqrt(sum(component * component for component in step))
            if step_mag > self._max_target_step:
                unclamped_position = vec_add(
                    previous_position,
                    vec_scale(step, self._max_target_step / step_mag),
                )
        target_position = self._clamp_position(arm, unclamped_position)

        if arm["track_orientation"]:
            # Full 6-DOF orientation tracking (UF850 / 6-DOF arms).
            hand_delta_quat = quat_multiply(current_hand_quat, quat_conjugate(arm["origin_hand_quat"]))
            target_quat = quat_multiply(hand_delta_quat, arm["origin_robot_quat"])
        elif arm.get("track_yaw", False) or arm.get("track_pitch", False):
            # Partial orientation tracking for 5-DOF arms (xArm5: joints Z-Y-Y-X-Y).
            # Roll is kinematically coupled (no terminal wrist-roll joint) and is always
            # kept at the calibration-moment value.  Yaw and pitch are independently
            # selectable via the track_yaw / track_pitch parameters.
            hand_delta_quat = quat_multiply(current_hand_quat, quat_conjugate(arm["origin_hand_quat"]))
            _, delta_pitch, delta_yaw = quat_to_rpy(hand_delta_quat)
            active_pitch = delta_pitch if arm.get("track_pitch", False) else 0.0
            active_yaw   = delta_yaw   if arm.get("track_yaw",   False) else 0.0
            partial_delta_quat = rpy_to_quat(0.0, active_pitch, active_yaw)
            target_quat = quat_multiply(partial_delta_quat, arm["origin_robot_quat"])
        else:
            target_quat = arm["origin_robot_quat"]

        if arm["target_pose"] is not None:
            smoothed_pos = vec_add(
                vec_scale(arm["target_pose"][0], 1.0 - self._position_alpha),
                vec_scale(target_position, self._position_alpha),
            )
            smoothed_quat = quaternion_slerp(arm["target_pose"][1], target_quat, self._orientation_alpha)
            return smoothed_pos, smoothed_quat
        return target_position, target_quat

    def _solve_and_publish(self, arm: dict):
        backend = arm["backend"]
        if not arm["enabled"] or not arm["teleop_allowed"] or not arm["calibrated"]:
            return
        self._recover_planner_if_needed(arm)
        planner = backend._single_arm_exotica_planner
        if planner is None or not planner.available:
            return
        target_pose = arm["target_pose"]
        if target_pose is None:
            return
        if not backend.state_received.wait(timeout=0.0):
            return

        # Non-blocking trylock: if IK is already running in another thread, re-apply
        # the last good joint command and return immediately.  Without this, every tick
        # (15 Hz) would block a thread for ~1.5 s on the IK call.  With 4 executor
        # threads all stalled on IK, fist/gesture callbacks have no thread to run on
        # and the node appears frozen.
        lock = arm["_ik_lock"]
        if not lock.acquire(blocking=False):
            last = arm["_last_filtered_joints"]
            if last is not None:
                backend._publish_direct_joint_command(last)
            return

        try:
            if not arm["enabled"] or not arm["teleop_allowed"] or not arm["calibrated"]:
                return
            pos, quat = target_pose
            roll, pitch, yaw = backend._quaternion_to_rpy(quat[0], quat[1], quat[2], quat[3])
            seed_joints = arm["seed_joints"]
            if seed_joints is None:
                seed_joints = {
                    name: float(backend.current_joint_positions.get(name, 0.0))
                    for name in planner.controlled_joint_names
                }
                arm["seed_joints"] = dict(seed_joints)
            # Merge full joint state (for non-controlled joints like linear_slide_joint)
            # with our smoothed seed values for the controlled joints.  The planner uses
            # this to update the EXOTica scene's passive joints so FK reflects the real
            # slide position instead of always assuming 0.
            full_positions = dict(backend.current_joint_positions)
            full_positions.update(seed_joints)
            result = planner.solve_pose_goal_joint_positions(
                full_positions,
                [pos[0], pos[1], pos[2], roll, pitch, yaw],
                max_retries=3,
            )
            if result is None:
                now = time.monotonic()
                if now - arm["last_command_time"] > 1.0:
                    self.get_logger().warning(
                        f"[{arm['label']}] EXOTica IK failed: {planner.last_error}"
                    )
                    arm["last_command_time"] = now
                return

            if not arm["enabled"] or not arm["teleop_allowed"] or not arm["calibrated"]:
                return
            seed_joints = arm["seed_joints"]
            if seed_joints is None:
                return
            filtered = {}
            max_step_from_velocity = self._max_joint_velocity / self._rate_hz
            max_joint_step = min(self._max_joint_step, max_step_from_velocity)
            for name in planner.controlled_joint_names:
                previous = float(seed_joints.get(name, backend.current_joint_positions.get(name, 0.0)))
                solved = float(result[name])
                target_val = previous + self._joint_alpha * (solved - previous)
                delta = clamp(target_val - previous, -max_joint_step, max_joint_step)
                filtered[name] = previous + delta

            if not arm["enabled"] or not arm["teleop_allowed"] or not arm["calibrated"]:
                return
            backend._publish_direct_joint_command(filtered)
            arm["seed_joints"] = dict(filtered)
            arm["_last_filtered_joints"] = dict(filtered)
            arm["last_command_time"] = time.monotonic()
        finally:
            lock.release()

    def _tick(self):
        enabled_arms = [
            arm
            for arm in self._arms.values()
            if arm is not None and arm["enabled"] and arm["teleop_allowed"]
        ]
        if not enabled_arms:
            return

        active_hands = 0
        for hand_name, arm in self._arms.items():
            if arm is None:
                continue
            if not arm["enabled"] or not arm["teleop_allowed"]:
                continue
            if not arm["calibrated"]:
                if not self._calibrate_arm(hand_name, arm):
                    continue
            target_pose = self._target_from_hand(hand_name, arm)
            if target_pose is None:
                continue
            arm["target_pose"] = target_pose
            self._solve_and_publish(arm)
            self._update_gripper_follow(hand_name, arm)
            active_hands += 1

        if active_hands == 0:
            return


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = ExoticaArmTeleop()
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
