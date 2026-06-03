#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
import threading, json, math, time, os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from disassembly_skill.device_config import DeviceConfig
from disassembly_skill.motion_backend import MotionBackend

def default_device_config_path():
    env_path = os.environ.get("DISASSEMBLY_DEVICE_CONFIG")
    if env_path:
        return Path(env_path).expanduser()

    installed_path = (
        Path(get_package_share_directory('disassembly_skill'))
        / 'config' / 'device_configs' / 'hdd.yaml'
    )
    source_candidates = [
        Path.cwd() / 'src' / 'agentic_disassembly' / 'disassembly_skill' / 'config' / 'device_configs' / 'hdd.yaml',
        Path.home() / 'workspace' / 'disassembly_ws' / 'src' / 'agentic_disassembly' / 'disassembly_skill' / 'config' / 'device_configs' / 'hdd.yaml',
    ]
    for candidate in source_candidates:
        if candidate.exists():
            return candidate
    return installed_path

class ObjectHoldSkill(Node):
    def __init__(self, device_cfg=None):
        super().__init__('object_hold_skill_node')
        self.device_cfg = device_cfg

        self.uf850 = MotionBackend(self, "uf850_arm")
        self.gripper = MotionBackend(self, "rg6_gripper")

        # --- Thread Safety & Vision State ---
        self.data_lock = threading.Lock()
        self.latest_targets = []
        self.create_subscription(String, '/vision/agent_state', self.vision_callback, 10)

        # --- Central State Managers ---
        self.hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hold_status_pub = self.create_publisher(Bool, '/object_hold_state/is_held', self.hold_qos)
        self.state_update_pub = self.create_publisher(String, '/robot_state/manip_arm/update', 10)

        # Track whether we (or any other skill) currently consider the object held.
        # Subscription uses transient-local so we pick up the last published value on connect.
        self.is_holding_object = False
        self.create_subscription(Bool, '/object_hold_state/is_held', self._hold_status_cb, self.hold_qos)

        # Labels searched in vision when the primary target label is not found.
        # Overridden per step via holdable_labels in the device config.
        self.HOLDABLE_LABELS = ["case", "chassis", "device", "hdd_holder", "holder", "lid"]

        # Configuration
        self.CAMERA_FRAME = 'camera_color_optical_frame'
        self.PLANNING_FRAME = "base_link"
        self.ROBOT_EE_LINK = "rg6_tcp"
        self.TOOL_LENGTH = 0.28
        self.HOVER_Z_OFFSET = 0.05
        self.JOINT_GRIPPER = "rg6_right_drive_joint"
        self.OPEN_DEG = -35.0
        self.CLOSE_DEG = 35.0
        self.GRIPPER_OPEN_FORCE_N = 40.0
        self.GRIPPER_CLOSE_FORCE_N = 100.0
        self.APPROACH_VELOCITY = 0.08
        self.HOVER_VELOCITY = 0.08
        self.GRIP_VELOCITY = 0.15
        self.TORQUE_THRESHOLD = 3.0
        self.DESCENT_SPEED_MPS = 0.02
        self.DESCENT_STEP_M = 0.0005
        self.DESCENT_DISTANCE_M = 0.06
        self.DESCENT_RATE_HZ = 30.0
        self.MIN_CONTACT_DESCENT_M = 0.030
        self.USE_TACTILE_DESCENT = True
        self.RETRACT_VELOCITY = 0.05
        self.CONTACT_RETRACT_M = 0.005
        self.CONTACT_RETRACT_MIN_SUCCESS_M = 0.0025
        self.CONTACT_RETRACT_VELOCITY = 0.02
        self.POST_GRASP_RETRACT_SPEED = 0.1
        self.STRATEGY = "fixture_press"
        self.HOVER_X_OFFSET = 0.0
        self.HOVER_Y_OFFSET = 0.0
        self.FINAL_Z_OFFSET_M = None  # None → grip at hover Z (no separate lowering move)
        # Lateral-clamp specific fields
        self.APPROACH_AXIS = "+z"
        self.TILT_DEG = 0.0
        self.GRIP_WIDTH_MM = 0.0
        self.GRIP_CONTACT_MARGIN_MM = 20.0
        self.APPROACH_STANDOFF_M = 0.12
        self.FLIP_APPROACH = False    # add π to v_rad → approach from opposite Y side
        self.APPROACH_RPY_RAD = None  # None → use default per strategy

        if device_cfg is not None:
            self._apply_hold_config(device_cfg)

        self.get_logger().info("Object Hold Skill: Tactile Hold Mode Active.")
        self.publish_state("IDLE")

    @staticmethod
    def _rpy_to_quat_dict(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return {
            'qx': sr * cp * cy - cr * sp * sy,
            'qy': cr * sp * cy + sr * cp * sy,
            'qz': cr * cp * sy - sr * sp * cy,
            'qw': cr * cp * cy + sr * sp * sy,
        }

    @staticmethod
    def _norm_label(value):
        return str(value or "").strip().lower()

    # ── Hold-state helpers ────────────────────────────────────────────────────

    def _hold_status_cb(self, msg: Bool):
        """Receive hold state published by this node or any other skill (e.g. pickup)."""
        self.is_holding_object = bool(msg.data)

    def _gripper_has_object(self) -> bool:
        """Return True if the RG6 bridge reports object_detected."""
        try:
            return bool(self.gripper.current_gripper_state.get("object_detected", False))
        except Exception:
            return False

    def _get_target_by_any_label(self, labels):
        """Search vision snapshot for the first object matching any label in *labels*."""
        with self.data_lock:
            for lbl in labels:
                result = next(
                    (t for t in self.latest_targets
                     if lbl.lower() in t.get("label", "").lower()),
                    None,
                )
                if result:
                    return result
        return None

    def _select_hold_step(self, cfg, target_label=None):
        hold_steps = [s for s in cfg.disassembly_sequence if s.action == 'hold']
        if not hold_steps:
            return None
        target_norm = self._norm_label(target_label)
        if target_norm:
            for step in hold_steps:
                if self._norm_label(step.target) == target_norm:
                    return step
            for step in hold_steps:
                if target_norm in self._norm_label(step.label):
                    return step
        return hold_steps[0]

    def _apply_hold_config(self, cfg, target_label=None, hold_step=None):
        step = hold_step if hold_step is not None else self._select_hold_step(cfg, target_label)
        if step is None:
            return
        p = step.parameters
        self.STRATEGY = p.get('strategy', self.STRATEGY)
        self.TORQUE_THRESHOLD = p.get('torque_threshold_nm', self.TORQUE_THRESHOLD)
        self.DESCENT_SPEED_MPS = p.get('descent_speed_mps', self.DESCENT_SPEED_MPS)
        self.HOVER_Z_OFFSET = p.get('hover_z_offset', self.HOVER_Z_OFFSET)
        self.HOVER_X_OFFSET = p.get('hover_x_offset_m', 0.0)
        self.HOVER_Y_OFFSET = p.get('hover_y_offset_m', 0.0)
        self.GRIPPER_CLOSE_FORCE_N = p.get('gripper_close_force_n', self.GRIPPER_CLOSE_FORCE_N)
        # Hold skill sign convention: OPEN is negative, CLOSE is positive
        # Config stores: gripper_open_deg=35.0 (positive = open), gripper_close_deg=-35.0
        # Map to hold skill convention (negate both)
        open_deg = p.get('gripper_open_deg', 35.0)
        close_deg = p.get('gripper_close_deg', -35.0)
        self.OPEN_DEG = -abs(open_deg)
        self.CLOSE_DEG = abs(close_deg)
        # Lateral-clamp params
        self.APPROACH_AXIS = p.get('approach_axis', '+z')
        self.TILT_DEG = p.get('tilt_deg', 0.0)
        self.GRIP_WIDTH_MM = p.get('grip_width_mm', 0.0)
        self.GRIP_CONTACT_MARGIN_MM = p.get('grip_contact_margin_mm', self.GRIP_CONTACT_MARGIN_MM)
        self.APPROACH_STANDOFF_M = p.get('approach_standoff_m', 0.12)
        # flip_approach: add π to v_rad so arm approaches from the opposite side
        self.FLIP_APPROACH = bool(p.get('flip_approach', False))
        rpy_deg = p.get('approach_rpy_deg', None)
        if rpy_deg is not None:
            self.APPROACH_RPY_RAD = [math.radians(d) for d in rpy_deg]
        else:
            self.APPROACH_RPY_RAD = None
        self.DESCENT_DISTANCE_M = p.get('descent_distance_m', self.DESCENT_DISTANCE_M)
        self.MIN_CONTACT_DESCENT_M = p.get('min_contact_descent_m', self.MIN_CONTACT_DESCENT_M)
        self.DESCENT_RATE_HZ = p.get('descent_rate_hz', self.DESCENT_RATE_HZ)
        self.DESCENT_STEP_M = p.get(
            'descent_step_m',
            max(self.DESCENT_SPEED_MPS / max(float(self.DESCENT_RATE_HZ), 1.0), 0.00025),
        )
        self.USE_TACTILE_DESCENT = bool(p.get('use_tactile_descent', self.USE_TACTILE_DESCENT))
        self.FINAL_Z_OFFSET_M = p.get('final_z_offset_m', None)
        self.APPROACH_VELOCITY = p.get('approach_velocity', self.APPROACH_VELOCITY)
        self.HOVER_VELOCITY = p.get('hover_velocity', self.APPROACH_VELOCITY)
        self.GRIP_VELOCITY = p.get('grip_velocity', self.GRIP_VELOCITY)
        self.CONTACT_RETRACT_M = p.get('contact_retract_m', self.CONTACT_RETRACT_M)
        self.CONTACT_RETRACT_MIN_SUCCESS_M = p.get(
            'contact_retract_min_success_m',
            self.CONTACT_RETRACT_MIN_SUCCESS_M,
        )
        self.CONTACT_RETRACT_VELOCITY = p.get('contact_retract_velocity', self.CONTACT_RETRACT_VELOCITY)
        # Labels accepted as valid hold targets (used as vision-search fallback list)
        holdable_labels = p.get('holdable_labels', None)
        if holdable_labels:
            self.HOLDABLE_LABELS = [str(l).lower() for l in holdable_labels]
        source = getattr(cfg, "source_path", None)
        self.get_logger().info(
            f"[hold] Config applied from {source}: step={step.step} target='{step.target}' "
            f"offsets=({self.HOVER_X_OFFSET*1000:.1f}, {self.HOVER_Y_OFFSET*1000:.1f}, "
            f"{self.HOVER_Z_OFFSET*1000:.1f})mm strategy={self.STRATEGY} "
            f"hover_velocity={float(self.HOVER_VELOCITY):.2f}m/s "
            f"descent={float(self.DESCENT_SPEED_MPS)*1000:.1f}mm/s@{float(self.DESCENT_RATE_HZ):.0f}Hz "
            f"max_step={float(self.DESCENT_STEP_M)*1000:.2f}mm"
        )

    def _log_pose_diagnostics(self, target_data, world_xyz, hover_xyz, quaternion_dict):
        raw_xyz = target_data.get("xyz", [None, None, None])
        raw_px = target_data.get("px", [None, None])
        print(f"[DIAG] Detection pixel in color image: px=({raw_px[0]}, {raw_px[1]})")
        reach_radius = math.hypot(float(hover_xyz[0]), float(hover_xyz[1]))
        uf_base_msg = "unavailable"
        try:
            tf = self.uf850.tf_buffer.lookup_transform(
                self.PLANNING_FRAME,
                "uf850_base_link",
                rclpy.time.Time(),
            )
            rel_x = float(hover_xyz[0]) - float(tf.transform.translation.x)
            rel_y = float(hover_xyz[1]) - float(tf.transform.translation.y)
            rel_z = float(hover_xyz[2]) - float(tf.transform.translation.z)
            uf_reach = math.hypot(rel_x, rel_y)
            uf_base_msg = (
                f"uf850_base_rel=({rel_x:.3f}, {rel_y:.3f}, {rel_z:.3f}) | "
                f"uf850_planar_reach={uf_reach:.3f} m"
            )
        except Exception:
            pass
        self.get_logger().info(
            "Hold diagnostics | raw_camera_xyz=(%.3f, %.3f, %.3f) | "
            "base_xyz=(%.3f, %.3f, %.3f) | hover_xyz=(%.3f, %.3f, %.3f) | "
            "base_planar_reach=%.3f m | %s | quat=(%.4f, %.4f, %.4f, %.4f)"
            % (
                float(raw_xyz[0]),
                float(raw_xyz[1]),
                float(raw_xyz[2]),
                float(world_xyz[0]),
                float(world_xyz[1]),
                float(world_xyz[2]),
                float(hover_xyz[0]),
                float(hover_xyz[1]),
                float(hover_xyz[2]),
                float(reach_radius),
                uf_base_msg,
                float(quaternion_dict["qx"]),
                float(quaternion_dict["qy"]),
                float(quaternion_dict["qz"]),
                float(quaternion_dict["qw"]),
            )
        )
        uf_reach = None
        if "uf850_planar_reach=" in uf_base_msg:
            try:
                uf_reach = float(uf_base_msg.split("uf850_planar_reach=")[1].split(" m")[0])
            except Exception:
                uf_reach = None
        if uf_reach is not None and uf_reach > 0.82:
            self.get_logger().warning(
                f"Computed hover target is near or beyond UF850 practical reach: {uf_reach:.3f} m"
            )

    def _current_tcp_xyz(self):
        pose = self._current_tcp_pose()
        if pose is None:
            return None
        return pose[:3]

    def _current_tcp_pose(self):
        try:
            tf_msg = self.uf850.tf_buffer.lookup_transform(
                self.PLANNING_FRAME,
                self.ROBOT_EE_LINK,
                rclpy.time.Time(),
            )
            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            roll, pitch, yaw = self.uf850._quaternion_to_rpy(q.x, q.y, q.z, q.w)
            return float(t.x), float(t.y), float(t.z), float(roll), float(pitch), float(yaw)
        except Exception as exc:
            self.get_logger().warning(f"[hold] Unable to read current TCP pose: {exc}")
            return None

    @staticmethod
    def _angle_delta(target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    def _move_to_hover_pose(self, hover_x, hover_y, hover_z, qd):
        """Move to hold hover as one low-speed continuous EXOTica trajectory."""
        velocity = min(max(float(self.HOVER_VELOCITY), 0.03), 0.60)
        current = self._current_tcp_xyz()
        self.uf850.stop_servo(timeout_sec=2.0)
        distance_msg = ""
        if current is not None:
            sx, sy, sz = current
            distance = math.sqrt((hover_x - sx) ** 2 + (hover_y - sy) ** 2 + (hover_z - sz) ** 2)
            distance_msg = f" distance={distance*1000:.1f}mm"
        self.get_logger().info(
            f"[hold] EXOTica continuous hover trajectory:{distance_msg} "
            f"target=({hover_x:.3f},{hover_y:.3f},{hover_z:.3f}) velocity={velocity:.2f}"
        )
        ok = self.uf850.move_to_pose_exotica(
            hover_x,
            hover_y,
            hover_z,
            qd,
            velocity=velocity,
        )
        if not ok:
            self.uf850._hold_current_arm_position()
        return ok

    @staticmethod
    def _gripper_width_mm_to_rad(width_mm):
        rad_open = -0.625
        rad_close = 0.625
        mm_open = 160.0
        mm_close = 0.0
        width = max(mm_close, min(mm_open, float(width_mm)))
        normalized = (width - mm_close) / (mm_open - mm_close)
        return rad_close + normalized * (rad_open - rad_close)

    @staticmethod
    def _gripper_rad_to_width_mm(value_rad):
        rad_open = -0.625
        rad_close = 0.625
        mm_open = 160.0
        mm_close = 0.0
        value = max(rad_open, min(rad_close, float(value_rad)))
        normalized = (value - rad_close) / (rad_open - rad_close)
        return mm_close + normalized * (mm_open - mm_close)

    def _hold_close_target_width_mm(self):
        if float(self.GRIP_WIDTH_MM) > 0.0:
            return max(0.0, float(self.GRIP_WIDTH_MM) - float(self.GRIP_CONTACT_MARGIN_MM))
        return self._gripper_rad_to_width_mm(math.radians(self.CLOSE_DEG))

    def _hold_close_target_rad(self):
        return self._gripper_width_mm_to_rad(self._hold_close_target_width_mm())

    def _wait_for_gripper_position(self, target_rad, is_closing, start_rad=None, timeout=5.0):
        start_t = time.time()
        last_pos = 999.0
        stall_timer = 0.0
        position_tolerance = 0.08
        motion_acceptance_rad = 0.08

        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = self.gripper.current_joint_positions.get(self.JOINT_GRIPPER, 999)
            if curr == 999:
                time.sleep(0.1)
                continue
            if abs(curr - target_rad) < position_tolerance:
                return True
            if is_closing and bool(self.gripper.current_gripper_state.get("object_detected", False)):
                self.get_logger().info(f"Grasp confirmed (RG6 object_detected) at {curr:.3f} rad.")
                return True
            if abs(curr - last_pos) < 0.002:
                stall_timer += 0.1
                if stall_timer >= 0.8:
                    if is_closing:
                        if start_rad is not None:
                            moved_toward_close = float(curr) - float(start_rad)
                            if moved_toward_close >= motion_acceptance_rad:
                                self.get_logger().info(
                                    f"Grasp confirmed by closing stall at {curr:.3f} rad "
                                    f"(moved {moved_toward_close:.3f} rad)."
                                )
                                return True
                        self.get_logger().warning(
                            f"Gripper close stalled without meaningful closing motion: "
                            f"current={curr:.3f} target={target_rad:.3f}"
                        )
                        return False
                    self.get_logger().info(
                        f"Gripper open accepted at mechanical limit {curr:.3f} rad "
                        f"(target {target_rad:.3f} rad)."
                    )
                    return True
            else:
                stall_timer = 0.0
            last_pos = curr
            time.sleep(0.1)
        return False

    def _close_gripper_for_hold(self):
        target_rad = self._hold_close_target_rad()
        target_width_mm = self._hold_close_target_width_mm()
        current = self.gripper.current_joint_positions.get(self.JOINT_GRIPPER, None)
        is_closing = True if current is None else target_rad > float(current)
        self.get_logger().info(
            f"[hold] Closing gripper to target={target_rad:.3f}rad "
            f"(target_width={target_width_mm:.1f}mm, configured_width={float(self.GRIP_WIDTH_MM):.1f}mm, "
            f"force={self.GRIPPER_CLOSE_FORCE_N:.1f}N)"
        )
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: target_rad},
            gripper_force_n=self.GRIPPER_CLOSE_FORCE_N,
        ):
            return False
        return self._wait_for_gripper_position(
            target_rad,
            is_closing=is_closing,
            start_rad=None if current is None else float(current),
        )

    def _retract_after_contact(self, distance_m=None, q_dict=None, target_x=None, target_y=None):
        """Lift the arm above contact with TF-verified EXOTica streaming and Servo fallback."""
        distance_m = abs(float(self.CONTACT_RETRACT_M if distance_m is None else distance_m))
        if distance_m <= 0.0:
            return True

        self.uf850.stop_servo(timeout_sec=0.5)
        for _ in range(3):
            self.uf850._hold_current_arm_position()
            time.sleep(0.08)

        try:
            start_tf = self.uf850.tf_buffer.lookup_transform(
                self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time()
            )
            sx = float(start_tf.transform.translation.x)
            sy = float(start_tf.transform.translation.y)
            sz = float(start_tf.transform.translation.z)
        except Exception as exc:
            self.get_logger().error(f"[hold] Contact retract TF lookup failed: {exc}")
            return False

        velocity = max(0.010, min(float(self.CONTACT_RETRACT_VELOCITY), 0.05))
        max_lateral_drift = 0.006
        min_expected = max(
            0.0015,
            min(float(self.CONTACT_RETRACT_MIN_SUCCESS_M), distance_m - 0.001),
        )
        hard_down_limit = -0.015

        def _read_retract_tf():
            tf = self.uf850.tf_buffer.lookup_transform(
                self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time()
            )
            return (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                float(tf.transform.translation.z),
                tf.transform.rotation,
            )

        def _wait_for_retract_settle(timeout_s=2.0, window_s=0.35, tolerance_m=0.0012):
            samples = []
            deadline = time.time() + float(timeout_s)
            while rclpy.ok() and time.time() < deadline:
                try:
                    ex, ey, ez, q = _read_retract_tf()
                except Exception:
                    time.sleep(0.05)
                    continue
                now = time.time()
                samples.append((now, ex, ey, ez, q))
                samples = [s for s in samples if now - s[0] <= window_s]
                if len(samples) >= 4:
                    z_values = [s[3] for s in samples]
                    xy_drift = max(math.hypot(s[1] - sx, s[2] - sy) for s in samples)
                    if (max(z_values) - min(z_values)) <= tolerance_m and xy_drift <= max_lateral_drift:
                        return samples[-1][1], samples[-1][2], samples[-1][3], samples[-1][4]
                self.uf850._hold_current_arm_position()
                time.sleep(0.05)
            return _read_retract_tf()

        def _verify_retract(label):
            try:
                ex, ey, ez, _ = _read_retract_tf()
            except Exception as exc:
                self.get_logger().warning(f"[hold] Contact retract verify TF failed after {label}: {exc}")
                return False, 0.0
            actual_dz = ez - sz
            drift = math.hypot(ex - sx, ey - sy)
            if actual_dz >= distance_m - 0.001:
                self.get_logger().info(
                    f"[hold] Contact retract done by {label}: actual dz={actual_dz*1000:.1f}mm "
                    f"target={distance_m*1000:.1f}mm drift={drift*1000:.1f}mm"
                )
                return True, actual_dz
            if drift > max_lateral_drift:
                self.get_logger().error(
                    f"[hold] Contact retract {label} lateral drift {drift*1000:.1f}mm exceeded "
                    f"{max_lateral_drift*1000:.1f}mm."
                )
                return False, actual_dz
            self.get_logger().warning(
                f"[hold] Contact retract {label} incomplete: actual dz={actual_dz*1000:.1f}mm "
                f"target={distance_m*1000:.1f}mm drift={drift*1000:.1f}mm"
            )
            return False, actual_dz

        def _attempt_exotica_step_lift(start_x, start_y, start_z, start_q, attempt_label):
            target_z = start_z + distance_m
            command_x = start_x
            command_y = start_y
            if target_x is not None and abs(float(target_x) - start_x) <= 0.010:
                command_x = float(target_x)
            if target_y is not None and abs(float(target_y) - start_y) <= 0.010:
                command_y = float(target_y)

            planner = getattr(self.uf850, "_single_arm_exotica_planner", None)
            if planner is None or not planner.available:
                return False, 0.0, "NO_PLANNER"
            if not self.uf850.state_received.wait(timeout=2.0):
                return False, 0.0, "NO_JOINT_STATES"

            if q_dict:
                roll, pitch, yaw = self.uf850._quaternion_to_rpy(
                    float(q_dict["qx"]), float(q_dict["qy"]),
                    float(q_dict["qz"]), float(q_dict["qw"]),
                )
            else:
                roll, pitch, yaw = self.uf850._quaternion_to_rpy(
                    start_q.x, start_q.y, start_q.z, start_q.w
                )

            step_m = max(0.00025, min(velocity / 50.0, 0.0010))
            loop_dt = 1.0 / 50.0
            command_alpha = 0.45
            max_joint_step_rad = 0.012
            max_solver_failures = 8
            no_progress_deadline = time.time() + 0.90
            hard_timeout = time.time() + max(1.5, (distance_m / max(velocity, 1e-3)) * 6.0 + 0.5)
            commanded_positions = {
                name: float(self.uf850.current_joint_positions[name])
                for name in self.uf850.current_joint_positions
                if name in planner.controlled_joint_names
            }
            seed_positions = dict(commanded_positions)
            commanded_z = start_z
            best_dz = 0.0
            consecutive_solver_failures = 0
            last_log_t = 0.0
            self.get_logger().info(
                f"[hold] Contact retract EXOTica stepped lift {attempt_label}: "
                f"start_z={start_z:.4f} target_z={target_z:.4f} step={step_m*1000:.2f}mm"
            )

            while rclpy.ok() and time.time() < hard_timeout:
                try:
                    ex, ey, ez, _ = _read_retract_tf()
                except Exception as exc:
                    self.get_logger().error(f"[hold] Contact retract EXOTica TF lost: {exc}")
                    return False, best_dz, "TF_LOST"

                actual_dz = ez - start_z
                best_dz = max(best_dz, actual_dz)
                drift = math.hypot(ex - start_x, ey - start_y)
                if actual_dz >= distance_m - 0.001:
                    self.uf850._hold_current_arm_position()
                    self.get_logger().info(
                        f"[hold] Contact retract done by EXOTica stepped lift: "
                        f"actual dz={actual_dz*1000:.1f}mm target={distance_m*1000:.1f}mm "
                        f"drift={drift*1000:.1f}mm"
                    )
                    return True, actual_dz, "DONE"
                if actual_dz < hard_down_limit:
                    self.uf850._hold_current_arm_position()
                    self.get_logger().warning(
                        f"[hold] Contact retract EXOTica moved downward {actual_dz*1000:.1f}mm; "
                        "will re-settle/retry from current pose."
                    )
                    return False, best_dz, "MOVED_DOWN"
                if drift > max_lateral_drift:
                    self.uf850._hold_current_arm_position()
                    self.get_logger().error(
                        f"[hold] Contact retract EXOTica lateral drift {drift*1000:.1f}mm exceeded "
                        f"{max_lateral_drift*1000:.1f}mm."
                    )
                    return False, best_dz, "DRIFT"
                if best_dz < 0.001 and time.time() > no_progress_deadline:
                    self.uf850._hold_current_arm_position()
                    self.get_logger().warning(
                        "[hold] Contact retract EXOTica made no upward TF progress; will re-settle/retry."
                    )
                    return False, best_dz, "NO_PROGRESS"

                commanded_z = min(target_z, commanded_z + step_m)
                target_joints = planner.solve_pose_goal_joint_positions(
                    seed_positions,
                    [command_x, command_y, commanded_z, roll, pitch, yaw],
                )
                if not target_joints:
                    consecutive_solver_failures += 1
                    if consecutive_solver_failures >= max_solver_failures:
                        self.uf850._hold_current_arm_position()
                        self.get_logger().warning(
                            f"[hold] Contact retract EXOTica IK failed repeatedly: {planner.last_error}. "
                            "Will re-settle/retry."
                        )
                        return False, best_dz, "IK_FAIL"
                    time.sleep(loop_dt)
                    continue

                consecutive_solver_failures = 0
                filtered_command = {}
                for joint_name, solved_position in target_joints.items():
                    previous = float(commanded_positions.get(joint_name, solved_position))
                    delta = (float(solved_position) - previous) * command_alpha
                    delta = max(-max_joint_step_rad, min(max_joint_step_rad, delta))
                    filtered_command[joint_name] = previous + delta

                self.uf850._publish_direct_joint_command(filtered_command, lookahead_s=0.12)
                commanded_positions = dict(filtered_command)
                seed_positions = dict(filtered_command)

                now = time.time()
                if now - last_log_t >= 0.5:
                    self.get_logger().info(
                        f"[hold] Contact retract EXOTica lifting: actual dz={actual_dz*1000:.1f}/"
                        f"{distance_m*1000:.1f}mm commanded_z_delta={(commanded_z-start_z)*1000:.1f}mm "
                        f"drift={drift*1000:.1f}mm"
                    )
                    last_log_t = now
                time.sleep(loop_dt)

            return False, best_dz, "TIMEOUT"

        def _rebase_to_settled_pose(retry_index):
            try:
                nx, ny, nz, nq = _wait_for_retract_settle()
            except Exception as exc:
                self.get_logger().error(f"[hold] Contact retract settle TF failed: {exc}")
                return None
            dropped = nz - sz
            if dropped < -0.030:
                self.get_logger().error(
                    f"[hold] Contact retract settled {dropped*1000:.1f}mm below initial contact; aborting."
                )
                return None
            self.get_logger().warning(
                f"[hold] Contact retract retry {retry_index}: rebasing lift from settled "
                f"z={nz:.4f} (delta from initial {dropped*1000:.1f}mm)."
            )
            return nx, ny, nz, nq

        # Contact can continue settling after the effort spike because trajectory
        # commands and joint-state feedback are not perfectly synchronous. Always
        # rebase to a settled TF pose and retry the 5 mm lift from there.
        settled = _rebase_to_settled_pose(1)
        if settled is None:
            return False
        sx, sy, sz, start_q = settled
        target_z = sz + distance_m
        self.get_logger().info(
            f"[hold] Contact retract live Z lift: {self.ROBOT_EE_LINK} z={sz:.4f} → {target_z:.4f} "
            f"(+{distance_m*1000:.1f}mm) at {velocity*1000:.0f}mm/s"
        )

        best_dz = 0.0
        for retry_index in range(1, 4):
            ok, actual_dz, reason = _attempt_exotica_step_lift(sx, sy, sz, start_q, f"{retry_index}/3")
            best_dz = max(best_dz, actual_dz)
            if ok:
                return True
            if reason == "DRIFT":
                return False
            if best_dz >= min_expected:
                self.get_logger().warning(
                    f"[hold] Contact retract reached acceptable dz={best_dz*1000:.1f}mm "
                    f"(target {distance_m*1000:.1f}mm)."
                )
                return True
            if retry_index < 3:
                settled = _rebase_to_settled_pose(retry_index + 1)
                if settled is None:
                    return False
                sx, sy, sz, start_q = settled
                target_z = sz + distance_m
                continue
            self.get_logger().warning(
                f"[hold] Contact retract EXOTica retries exhausted; best dz={best_dz*1000:.1f}mm. "
                "Trying Servo fallback from current settled pose."
            )

        exotica_best_dz = best_dz
        try:
            sx, sy, sz, start_q = _wait_for_retract_settle(timeout_s=1.0)
            target_z = sz + distance_m
        except Exception:
            pass

        if not self.uf850.start_servo(timeout_sec=8.0):
            self.get_logger().error("[hold] Contact retract failed to start Servo.")
            return False

        rate_hz = 50.0
        dt = 1.0 / rate_hz
        timeout = max(3.0, (distance_m / max(velocity, 1e-3)) * 6.0)
        deadline = time.time() + timeout
        switch_down_limit = -0.004
        best_dz = exotica_best_dz
        # Servo Z sign has varied with controller state on this stack. Probe
        # the expected sign first, then automatically reverse if TF moves down.
        z_signs = (1.0, -1.0)

        try:
            for sign_index, z_sign in enumerate(z_signs, start=1):
                if time.time() >= deadline:
                    break
                cmd_z = z_sign * velocity
                sign_start_t = time.time()
                last_log_t = 0.0
                attempt_deadline = min(
                    deadline,
                    sign_start_t + max(1.5, (distance_m / max(velocity, 1e-3)) * 3.0),
                )
                self.get_logger().info(
                    f"[hold] Contact retract attempt {sign_index}/{len(z_signs)}: "
                    f"cmd_z={cmd_z*1000:.1f}mm/s"
                )

                while rclpy.ok() and time.time() < attempt_deadline:
                    try:
                        tf = self.uf850.tf_buffer.lookup_transform(
                            self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time()
                        )
                        x = float(tf.transform.translation.x)
                        y = float(tf.transform.translation.y)
                        z = float(tf.transform.translation.z)
                    except Exception as exc:
                        self.get_logger().error(f"[hold] Contact retract TF lost during lift: {exc}")
                        return False

                    actual_dz = z - sz
                    best_dz = max(best_dz, actual_dz)
                    drift = math.hypot(x - sx, y - sy)
                    if actual_dz >= distance_m - 0.001:
                        self.uf850._publish_zero_twist()
                        self.get_logger().info(
                            f"[hold] Contact retract done: actual dz={actual_dz*1000:.1f}mm "
                            f"target={distance_m*1000:.1f}mm drift={drift*1000:.1f}mm "
                            f"cmd_z={cmd_z*1000:.1f}mm/s"
                        )
                        return True
                    if actual_dz >= min_expected:
                        self.uf850._publish_zero_twist()
                        self.get_logger().warning(
                            f"[hold] Contact retract accepted Servo relief: "
                            f"actual dz={actual_dz*1000:.1f}mm "
                            f"(target {distance_m*1000:.1f}mm, minimum {min_expected*1000:.1f}mm)."
                        )
                        return True

                    if actual_dz < hard_down_limit:
                        self.uf850._publish_zero_twist()
                        self.get_logger().error(
                            f"[hold] Contact retract moved downward {actual_dz*1000:.1f}mm; aborting."
                        )
                        return False

                    if drift > max_lateral_drift:
                        self.uf850._publish_zero_twist()
                        self.get_logger().error(
                            f"[hold] Contact retract lateral drift {drift*1000:.1f}mm exceeded "
                            f"{max_lateral_drift*1000:.1f}mm."
                        )
                        return False

                    if (
                        actual_dz < switch_down_limit
                        and sign_index < len(z_signs)
                        and time.time() - sign_start_t > 0.35
                    ):
                        self.uf850._publish_zero_twist()
                        self.get_logger().warning(
                            f"[hold] Contact retract cmd_z={cmd_z*1000:.1f}mm/s moved down "
                            f"{actual_dz*1000:.1f}mm; reversing Servo Z direction."
                        )
                        time.sleep(0.15)
                        break

                    if not self.uf850.publish_servo_velocity(0.0, 0.0, cmd_z):
                        self.get_logger().error("[hold] Contact retract Servo command failed.")
                        return False

                    now = time.time()
                    if now - last_log_t >= 0.5:
                        self.get_logger().info(
                            f"[hold] Contact retract lifting: dz={actual_dz*1000:.1f}/"
                            f"{distance_m*1000:.1f}mm cmd_z={cmd_z*1000:.1f}mm/s "
                            f"drift={drift*1000:.1f}mm"
                        )
                        last_log_t = now
                    time.sleep(dt)

                self.uf850._publish_zero_twist()
                time.sleep(0.10)

            try:
                end_tf = self.uf850.tf_buffer.lookup_transform(
                    self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time()
                )
                actual_dz = float(end_tf.transform.translation.z) - sz
            except Exception as exc:
                self.get_logger().warning(f"[hold] Could not verify contact retract TF after timeout: {exc}")
                return False
            best_dz = max(best_dz, actual_dz)
            if actual_dz >= min_expected:
                self.get_logger().warning(
                    f"[hold] Contact retract timed out but reached acceptable dz="
                    f"{actual_dz*1000:.1f}mm."
                )
                return True
            self.get_logger().error(
                f"[hold] Contact retract incomplete: actual dz={actual_dz*1000:.1f}mm "
                f"best dz={best_dz*1000:.1f}mm (minimum {min_expected*1000:.1f}mm)."
            )
            return False
        finally:
            try:
                self.uf850._publish_zero_twist()
                self.uf850.stop_servo(timeout_sec=2.0)
            except Exception as exc:
                self.get_logger().warning(f"[hold] Contact retract Servo stop failed: {exc}")

    def _tactile_descent_to_contact(self, q_dict=None, joint_index=4, target_x=None, target_y=None):
        """Filtered EXOTica Z descent with effort stop.

        This keeps the previous working tactile descent path, but uses the
        smaller 0.35 mm increments and higher update rate from config. The
        fully realtime target-function stream was too conservative on hardware
        and held position at 0 mm travelled.
        """
        self.get_logger().info(
            f"[hold] Filtered tactile descent: distance={self.DESCENT_DISTANCE_M*1000:.1f}mm "
            f"step={self.DESCENT_STEP_M*1000:.2f}mm rate={self.DESCENT_RATE_HZ:.1f}Hz "
            f"threshold={self.TORQUE_THRESHOLD:.2f}Nm min_contact={self.MIN_CONTACT_DESCENT_M*1000:.1f}mm"
        )
        try:
            start_tf = self.uf850.tf_buffer.lookup_transform(
                self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time()
            )
            start_z = float(start_tf.transform.translation.z)
        except Exception as exc:
            self.get_logger().error(f"[hold] Could not read TCP before tactile descent: {exc}")
            return False

        ok = self.uf850.move_linear_z_with_effort_stop_exotica(
            descent_distance_m=self.DESCENT_DISTANCE_M,
            step_m=self.DESCENT_STEP_M,
            threshold_nm=self.TORQUE_THRESHOLD,
            joint_index=joint_index,
            q_dict=q_dict,
            rate_hz=self.DESCENT_RATE_HZ,
            command_alpha=0.45,
            max_joint_step_rad=0.012,
            target_x=target_x,
            target_y=target_y,
            settling_cycles=3,
        )
        if not ok:
            return False

        try:
            end_tf = self.uf850.tf_buffer.lookup_transform(
                self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time()
            )
            actual_descended = start_z - float(end_tf.transform.translation.z)
        except Exception as exc:
            self.get_logger().warning(f"[hold] Could not verify tactile descent distance: {exc}")
            return True

        if actual_descended < float(self.MIN_CONTACT_DESCENT_M):
            self.get_logger().error(
                f"[hold] Rejecting early torque spike: descended={actual_descended*1000:.1f}mm "
                f"minimum={self.MIN_CONTACT_DESCENT_M*1000:.1f}mm. "
                "This is likely settling/IK load, not object contact."
            )
            return False
        return True

    def publish_state(self, s):
        self.state_update_pub.publish(String(data=s))

    def publish_hold_status(self, h):
        self.is_holding_object = bool(h)   # synchronous local update (no callback delay)
        self.hold_status_pub.publish(Bool(data=bool(h)))

    def vision_callback(self, msg):
        try:
            raw_data = msg.data.strip().strip("'").strip('"')
            data = json.loads(raw_data)
            with self.data_lock:
                self.latest_targets = data.get("global_view", {}).get("objects", [])
        except Exception as exc:
            self.get_logger().warning(f"Failed to parse /vision/agent_state payload: {exc}")

    @staticmethod
    def _has_valid_xyz(target):
        xyz = target.get("xyz")
        return isinstance(xyz, (list, tuple)) and len(xyz) >= 3 and all(v is not None for v in xyz[:3])

    @staticmethod
    def _label_contains(target, keywords):
        label = str(target.get("label", "")).lower()
        return any(k in label for k in keywords)

    def _select_hold_target(self):
        """Pick the object to hold from the current global vision snapshot."""
        hold_keywords = ("case", "chassis", "device", "hdd_holder", "holder", "lid")
        with self.data_lock:
            valid = [t for t in self.latest_targets if self._has_valid_xyz(t)]
        preferred = [t for t in valid if self._label_contains(t, hold_keywords)]
        return (preferred or valid)[0] if (preferred or valid) else None

    def _get_target_by_id(self, part_id):
        with self.data_lock:
            return next((t for t in self.latest_targets if t.get('id') == part_id), None)

    def _current_uf_joint_positions(self):
        return {
            name: pos
            for name, pos in self.uf850.current_joint_positions.items()
            if name.startswith("uf850_")
        }

    def wait_for_arm_settled(self, timeout=20.0):
        print("Waiting for arm to physically settle...")
        time.sleep(0.2)
        start_t = time.time()
        settle_timer = 0.0
        last_positions = {}
        NOISE_TOLERANCE = 0.006

        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr_positions = self._current_uf_joint_positions()
            if not curr_positions:
                time.sleep(0.1)
                continue
            if last_positions:
                max_delta = 0.0
                for j_name, j_pos in curr_positions.items():
                    if j_name in last_positions:
                        delta = abs(j_pos - last_positions[j_name])
                        if delta > max_delta:
                            max_delta = delta
                if max_delta <= NOISE_TOLERANCE:
                    settle_timer += 0.1
                    if settle_timer >= 0.4:
                        return True
                else:
                    settle_timer = 0.0
            last_positions = curr_positions
            time.sleep(0.1)
        print("Warning: Arm settle timeout reached.")
        return False

    def wait_for_gripper(self, target_deg, timeout=5.0):
        target_rad = math.radians(target_deg)
        target_width_mm = self._gripper_rad_to_width_mm(target_rad)
        start_t = time.time()
        last_pos = 999.0
        stall_timer = 0.0
        is_closing = target_deg > 0
        position_tolerance = 0.08

        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = self.gripper.current_joint_positions.get(self.JOINT_GRIPPER, 999)
            if curr == 999:
                time.sleep(0.1)
                continue
            state = self.gripper.current_gripper_state
            try:
                bridge_target_width = float(state.get("target_width_mm"))
            except Exception:
                bridge_target_width = None
            target_ack = (
                bridge_target_width is not None
                and abs(bridge_target_width - target_width_mm) <= 5.0
            )
            if abs(curr - target_rad) < position_tolerance:
                return True
            if not is_closing and not target_ack:
                if time.time() - start_t > 1.0 and not bool(state.get("is_moving", False)):
                    self.get_logger().warning(
                        f"Gripper open command not acknowledged: bridge_target={bridge_target_width}mm "
                        f"expected={target_width_mm:.1f}mm"
                    )
                    return False
                time.sleep(0.1)
                continue
            if abs(curr - last_pos) < 0.002:
                stall_timer += 0.1
                if stall_timer >= 0.8:
                    if is_closing:
                        self.get_logger().info(f"Grasp confirmed (Force reached) at {curr:.3f} rad.")
                        return True
                    else:
                        if not target_ack:
                            self.get_logger().warning(
                                f"Refusing to accept gripper-open stall while bridge target is stale: "
                                f"bridge_target={bridge_target_width}mm expected={target_width_mm:.1f}mm"
                            )
                            return False
                        self.get_logger().info(
                            f"Gripper open accepted at mechanical limit {curr:.3f} rad "
                            f"(target {target_rad:.3f} rad)."
                        )
                        return True
            else:
                stall_timer = 0.0
            last_pos = curr
            time.sleep(0.1)
        return False

    def _open_gripper_verified(self, attempts=3):
        target_rad = math.radians(self.OPEN_DEG)
        target_width_mm = self._gripper_rad_to_width_mm(target_rad)
        for attempt in range(1, attempts + 1):
            self.get_logger().info(
                f"[hold] Opening gripper attempt {attempt}/{attempts}: target={target_rad:.3f}rad "
                f"width={target_width_mm:.1f}mm"
            )
            self.gripper.set_gripper_force(self.GRIPPER_OPEN_FORCE_N)
            self.gripper._publish_gripper_command(self.JOINT_GRIPPER, target_rad)
            if self._wait_for_gripper_open_physical(target_rad, timeout=7.0):
                return True
            time.sleep(0.5)
        self.get_logger().error("Gripper failed to verify open state; aborting before arm motion.")
        return False

    def _wait_for_gripper_open_physical(self, target_rad, timeout=7.0):
        target_width_mm = self._gripper_rad_to_width_mm(target_rad)
        min_open_width_mm = max(145.0, target_width_mm - 10.0)
        start_t = time.time()
        stable_since = None
        last_republish = 0.0
        last_width = None

        while rclpy.ok() and (time.time() - start_t) < timeout:
            state = self.gripper.current_gripper_state
            try:
                width_mm = float(state.get("width_mm"))
            except Exception:
                width_mm = None
            is_moving = bool(state.get("is_moving", False))

            now = time.time()
            if now - last_republish >= 0.35:
                self.gripper._publish_gripper_command(self.JOINT_GRIPPER, target_rad)
                last_republish = now

            if width_mm is not None and last_width is not None:
                if last_width - width_mm > 3.0:
                    self.get_logger().warning(
                        f"Gripper started closing during open verification "
                        f"({last_width:.1f}mm -> {width_mm:.1f}mm); reasserting open command."
                    )
                    self.gripper._publish_gripper_command(self.JOINT_GRIPPER, target_rad)
                    stable_since = None

            if width_mm is not None and width_mm >= min_open_width_mm:
                if not is_moving:
                    if stable_since is None:
                        stable_since = now
                    elif now - stable_since >= 0.4:
                        self.get_logger().info(
                            f"Gripper physically open: width={width_mm:.1f}mm "
                            f"(target {target_width_mm:.1f}mm)."
                        )
                        return True
                else:
                    stable_since = None
            else:
                stable_since = None

            last_width = width_mm
            time.sleep(0.1)
        return False

    def _get_target_by_label(self, label):
        with self.data_lock:
            return next((t for t in self.latest_targets if label.lower() in t.get('label', '').lower()), None)

    def _run_hold_sequence(self, part_id, target_label, interactive, target_data_override=None):
        # Caller-provided snapshot bypasses stale-ID issues (e.g. master_agent).
        target_data = target_data_override if target_data_override and target_data_override.get('xyz') else None
        if target_data is None:
            target_data = self._get_target_by_id(part_id)

        # Fallback: ID may be stale — match by label in the current live vision snapshot.
        if not target_data or 'xyz' not in target_data:
            print(f"⚠️ ID {part_id} not found. Searching by label '{target_label}'...")
            target_data = self._get_target_by_label(target_label)

        # Second fallback: try any of the configured holdable_labels (e.g. 'lid' after a flip)
        if not target_data or 'xyz' not in target_data:
            fallback_labels = [l for l in self.HOLDABLE_LABELS if l != self._norm_label(target_label)]
            if fallback_labels:
                print(
                    f"⚠️ '{target_label}' not found in vision. "
                    f"Trying holdable_labels fallback: {fallback_labels}"
                )
                target_data = self._get_target_by_any_label(fallback_labels)
                if target_data:
                    print(f"[hold] Fallback target found: label='{target_data.get('label')}' id={target_data.get('id')}")

        if not target_data or 'xyz' not in target_data:
            print(f"ABORT: Vision data for '{target_label}' (ID {part_id}) is missing.")
            return False

        # ── Camera-chain sanity check ──────────────────────────────────────────
        try:
            cam_tf = self.uf850.tf_buffer.lookup_transform(
                self.PLANNING_FRAME, self.CAMERA_FRAME, rclpy.time.Time()
            )
            cx = cam_tf.transform.translation.x
            cy = cam_tf.transform.translation.y
            cz = cam_tf.transform.translation.z
            print(f"[DIAG] {self.CAMERA_FRAME} origin in base_link: ({cx:.4f}, {cy:.4f}, {cz:.4f})")
            print(f"[DIAG] Expected from calib: (~1.058, ~0.074, ~1.480)")
            q = cam_tf.transform.rotation
            print(f"[DIAG] Camera orientation quat: x={q.x:.3f} y={q.y:.3f} z={q.z:.3f} w={q.w:.3f}")
        except Exception as e:
            print(f"[DIAG] Camera TF lookup FAILED: {e} — check handeye publisher and camera driver TF")

        p = Pose()
        p.position.x, p.position.y, p.position.z = target_data["xyz"]
        p.orientation.w = 1.0

        t_pose = self.uf850.get_transformed_pose(p, self.CAMERA_FRAME, self.PLANNING_FRAME)
        if not t_pose:
            return False

        wx = t_pose.pose.position.x
        wy = t_pose.pose.position.y
        wz = t_pose.pose.position.z

        if self.STRATEGY == 'lateral_clamp':
            return self._run_lateral_clamp(wx, wy, wz, target_data, target_label, interactive)
        else:
            return self._run_top_down(wx, wy, wz, target_data, target_label, interactive)

    def _run_lateral_clamp(self, wx, wy, wz, target_data, target_label, interactive):
        """Side-clamp the object.

        Direct mode (approach_rpy_deg set): move to the configured hover pose,
          then descend with joint-5 effort-stop unless explicitly disabled.
        Legacy mode: warm-start pre-position above HDD, computed v_rad hover with
          EXOTica+Cartesian fallbacks, then tactile Z-descent for contact detection.
        """
        tilt_rad = math.radians(self.TILT_DEG) if self.TILT_DEG else 0.0

        try:
            target_angle_deg = float(target_data.get("angle", 0.0))
        except (TypeError, ValueError):
            target_angle_deg = 0.0
        v_rad_base = math.radians(-target_angle_deg) + (math.pi / 2.0)
        if self.FLIP_APPROACH:
            v_rad_base += math.pi

        roll = math.pi
        pitch = -math.pi / 2.0 + tilt_rad
        off = self.TOOL_LENGTH * math.cos(tilt_rad)
        hover_z = wz + self.HOVER_Z_OFFSET + self.TOOL_LENGTH * math.sin(tilt_rad)

        self.publish_state("MOVING")
        if interactive:
            input(f"STEP 1: Open gripper and move to lateral hover for {target_label}? [Enter]")

        print("Ensuring gripper is open...")
        if not self._open_gripper_verified():
            return False

        # ── Phase 1: pre-position above HDD (warm-start for legacy v_rad mode only) ──
        # Direct approach mode skips this: EXOTica solves the configured pose quickly
        # without a warm-start, so adding a top-down pre-position only wastes ~31s.
        if self.APPROACH_RPY_RAD is None:
            try:
                current_tf = self.uf850.tf_buffer.lookup_transform(
                    self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time()
                )
                qd_current = {
                    'qx': float(current_tf.transform.rotation.x),
                    'qy': float(current_tf.transform.rotation.y),
                    'qz': float(current_tf.transform.rotation.z),
                    'qw': float(current_tf.transform.rotation.w),
                }
            except Exception:
                qd_current = {'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}

            pre_z = wz + 0.15
            self.get_logger().info(
                f"[lateral_clamp] Phase 1 pre-position above HDD: "
                f"({wx:.3f},{wy:.3f},{pre_z:.3f}) with current TCP orientation"
            )
            pre_ok = self.uf850.move_to_pose_exotica(
                wx, wy, pre_z, qd_current, velocity=self.APPROACH_VELOCITY
            )
            if not pre_ok:
                pre_ok = self.uf850.move_cartesian_to_pose(
                    wx, wy, pre_z, qd_current, velocity=self.APPROACH_VELOCITY
                )
            if pre_ok:
                self.wait_for_arm_settled()
            else:
                self.get_logger().warning(
                    "[lateral_clamp] Phase 1 pre-position failed; "
                    "attempting lateral hover without warm-start."
                )

        # ── Phase 2: lateral hover — EXOTica seeds from warm-started joints ────────
        if self.APPROACH_RPY_RAD is not None:
            # Direct approach mode: use the configured RPY and XY offsets directly.
            # hover_x/y_offset_m are deltas from HDD centre in the world frame.
            qd = self._rpy_to_quat_dict(*self.APPROACH_RPY_RAD)
            hover_x = wx + self.HOVER_X_OFFSET
            hover_y = wy + self.HOVER_Y_OFFSET
            hover_z = wz + self.HOVER_Z_OFFSET
            self.get_logger().info(
                f"[lateral_clamp] Direct approach: hover=({hover_x:.3f},{hover_y:.3f},{hover_z:.3f}) "
                f"rpy=({math.degrees(self.APPROACH_RPY_RAD[0]):.1f}°,"
                f"{math.degrees(self.APPROACH_RPY_RAD[1]):.1f}°,"
                f"{math.degrees(self.APPROACH_RPY_RAD[2]):.1f}°)"
            )
            self._log_pose_diagnostics(
                target_data=target_data,
                world_xyz=(wx, wy, wz),
                hover_xyz=(hover_x, hover_y, hover_z),
                quaternion_dict=qd,
            )
            if not self._move_to_hover_pose(hover_x, hover_y, hover_z, qd):
                print("[ERROR] Failed to reach direct approach hover. Aborting hold.")
                return False
            print("Hover reached (direct approach).")
        else:
            # Legacy computed approach: try ±Y from v_rad_base
            for attempt, v_rad in enumerate([v_rad_base, v_rad_base + math.pi]):
                yaw = v_rad
                qd = self._rpy_to_quat_dict(roll, pitch, yaw)
                hover_x = wx - off * math.cos(v_rad) + self.HOVER_X_OFFSET
                hover_y = wy - off * math.sin(v_rad) + self.HOVER_Y_OFFSET

                side_label = "primary" if attempt == 0 else "opposite-side fallback"
                self.get_logger().info(
                    f"lateral_clamp {side_label}: hover=({hover_x:.3f},{hover_y:.3f},{hover_z:.3f}) "
                    f"pitch={math.degrees(pitch):.1f}° yaw={math.degrees(yaw):.1f}°"
                )
                self._log_pose_diagnostics(
                    target_data=target_data,
                    world_xyz=(wx, wy, wz),
                    hover_xyz=(hover_x, hover_y, hover_z),
                    quaternion_dict=qd,
                )
                hover_ok = self._move_to_hover_pose(hover_x, hover_y, hover_z, qd)
                if not hover_ok:
                    print(f"Hover planner failed for {side_label}. Trying Cartesian...")
                    hover_ok = self.uf850.move_cartesian_to_pose(
                        hover_x, hover_y, hover_z, qd, velocity=self.APPROACH_VELOCITY
                    )
                if hover_ok:
                    print(f"Hover reached ({side_label}).")
                    break
                print(f"Both planners failed for {side_label}.")
            else:
                print("[ERROR] All approach sides failed IK. Aborting hold.")
                return False

        if not self.wait_for_arm_settled():
            print("[ERROR] Arm did not settle after lateral hover.")
            return False

        try:
            tf_check = self.uf850.tf_buffer.lookup_transform(
                self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time()
            )
            ax = tf_check.transform.translation.x
            ay = tf_check.transform.translation.y
            az = tf_check.transform.translation.z
            hover_err = math.sqrt((ax - hover_x) ** 2 + (ay - hover_y) ** 2 + (az - hover_z) ** 2)
            print(f"[DIAG] {self.ROBOT_EE_LINK} ACTUAL:    ({ax:.4f}, {ay:.4f}, {az:.4f})")
            print(f"[DIAG] {self.ROBOT_EE_LINK} COMMANDED: ({hover_x:.4f}, {hover_y:.4f}, {hover_z:.4f})")
            print(f"[DIAG] TCP error: dx={ax-hover_x:.4f} dy={ay-hover_y:.4f} dz={az-hover_z:.4f} m | norm={hover_err:.4f} m")
            print(f"[DIAG] Object in base_link: ({wx:.4f}, {wy:.4f}, {wz:.4f})")
            _MAX_HOVER_CORRECTIONS = 5
            _HOVER_CORRECTION_VEL = 0.12   # m/s — fast enough to generate a real trajectory
            _HOVER_ABORT_THRESHOLD = 0.010  # require verified hover before tactile descent
            if hover_err > 0.018:
                for _corr_i in range(_MAX_HOVER_CORRECTIONS):
                    self.get_logger().warning(
                        f"[lateral_clamp] Hover TCP error {hover_err*1000:.1f}mm; "
                        f"correction attempt {_corr_i + 1}/{_MAX_HOVER_CORRECTIONS} "
                        f"at {_HOVER_CORRECTION_VEL*1000:.0f}mm/s."
                    )
                    if not self.uf850.move_to_pose_exotica(
                        hover_x, hover_y, hover_z, qd, velocity=_HOVER_CORRECTION_VEL
                    ):
                        print(f"[ERROR] Hover correction {_corr_i + 1} move failed. Aborting hold.")
                        return False
                    time.sleep(0.3)   # let TF propagate before checking
                    if not self.wait_for_arm_settled():
                        print("[ERROR] Arm did not settle after hover correction.")
                        return False
                    tf_check = self.uf850.tf_buffer.lookup_transform(
                        self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time()
                    )
                    ax = tf_check.transform.translation.x
                    ay = tf_check.transform.translation.y
                    az = tf_check.transform.translation.z
                    hover_err = math.sqrt((ax - hover_x) ** 2 + (ay - hover_y) ** 2 + (az - hover_z) ** 2)
                    print(
                        f"[DIAG] corrected {self.ROBOT_EE_LINK} (attempt {_corr_i + 1}): "
                        f"({ax:.4f}, {ay:.4f}, {az:.4f}) | err={hover_err*1000:.1f}mm"
                    )
                    if hover_err <= _HOVER_ABORT_THRESHOLD:
                        break
                if hover_err > _HOVER_ABORT_THRESHOLD:
                    print(
                        f"[ERROR] Hover still {hover_err*1000:.1f}mm from target after "
                        f"{_MAX_HOVER_CORRECTIONS} corrections. Aborting before descent."
                    )
                    return False
        except Exception as e:
            print(f"[DIAG] TCP TF lookup failed: {e}")

        if self.APPROACH_RPY_RAD is not None:
            if self.USE_TACTILE_DESCENT:
                if interactive:
                    input(f"STEP 2: Tactile descent to joint-5 effort spike for {target_label}? [Enter]")
                self.get_logger().info(
                    f"[lateral_clamp] Tactile descent until joint-5 effort spike: "
                    f"distance={self.DESCENT_DISTANCE_M*1000:.1f}mm "
                    f"step={self.DESCENT_STEP_M*1000:.2f}mm threshold={self.TORQUE_THRESHOLD:.2f}Nm"
                )
                if not self._tactile_descent_to_contact(
                    q_dict=qd,
                    joint_index=4,
                    target_x=hover_x,
                    target_y=hover_y,
                ):
                    print("[ERROR] Tactile descent ended without joint-5 contact spike. Aborting hold.")
                    return False
                print("[lateral_clamp] Contact confirmed — retracting 5mm before gripper close...")
                if not self._retract_after_contact(q_dict=qd, target_x=hover_x, target_y=hover_y):
                    print("[ERROR] Contact retract failed or did not move upward. Aborting hold.")
                    return False
            else:
                final_z = wz + (self.FINAL_Z_OFFSET_M if self.FINAL_Z_OFFSET_M is not None else self.HOVER_Z_OFFSET)
                if abs(final_z - hover_z) <= 0.002:
                    final_z = None
            if not self.USE_TACTILE_DESCENT and final_z is not None:
                self.get_logger().info(
                    f"[lateral_clamp] Lowering to final grip Z={final_z:.3f} "
                    f"(delta={hover_z - final_z:.3f}m)"
                )
                if interactive:
                    input(f"STEP 2: Lower to final grip Z for {target_label}? [Enter]")
                if not self.uf850.move_to_pose_exotica(
                    hover_x, hover_y, final_z, qd, velocity=self.GRIP_VELOCITY
                ):
                    print("[ERROR] EXOTica failed for final grip Z. Aborting hold.")
                    return False
                if not self.wait_for_arm_settled():
                    print("[ERROR] Arm did not settle at final grip position.")
                    return False
        else:
            # Legacy mode: tactile Z-descent with effort-spike contact detection.
            if interactive:
                input(f"STEP 2: Tactile Z-descent toward {target_label} side? [Enter]")
            print(f"Starting tactile Z-descent to contact HDD side face (threshold={self.TORQUE_THRESHOLD}Nm)...")
            if not self._tactile_descent_to_contact(joint_index=4):
                print("[ERROR] Tactile Z-descent failed before side contact.")
                return False
            print("[lateral_clamp] Contact confirmed — retracting 5mm before gripper close...")
            if not self._retract_after_contact(q_dict=qd):
                print("[ERROR] Contact retract failed or did not move upward. Aborting hold.")
                return False

        if interactive:
            input(f"STEP 3: Close gripper on {target_label}? [Enter]")
        print("Closing gripper...")
        if not self._close_gripper_for_hold():
            return False

        self.publish_state("HOLDING")
        return True

    def _run_top_down(self, wx, wy, wz, target_data, target_label, interactive):
        """Standard top-down or fixture-press hold: arm descends in Z with tool pointing down."""
        # Explicit tool-down orientation (roll=π) instead of reading current TCP quaternion
        qd = self._rpy_to_quat_dict(math.pi, 0.0, 0.0)

        hover_x = wx + self.HOVER_X_OFFSET
        hover_y = wy + self.HOVER_Y_OFFSET
        hover_z = wz + self.HOVER_Z_OFFSET

        self._log_pose_diagnostics(
            target_data=target_data,
            world_xyz=(wx, wy, wz),
            hover_xyz=(hover_x, hover_y, hover_z),
            quaternion_dict=qd,
        )

        self.publish_state("MOVING")
        if interactive:
            input(f"STEP 1: Hover over {target_label}? [Enter]")

        print("Ensuring gripper is open...")
        if not self._open_gripper_verified():
            return False

        print(f"Moving to hover pose at X:{hover_x:.3f}, Y:{hover_y:.3f}, Z:{hover_z:.3f}...")
        if not self._move_to_hover_pose(hover_x, hover_y, hover_z, qd):
            print("Hover pose planning failed. Attempting Cartesian fallback...")
            if not self.uf850.move_cartesian_to_pose(hover_x, hover_y, hover_z, qd, velocity=self.APPROACH_VELOCITY):
                print("[ERROR] Hover planner and Cartesian fallback failed to reach hover pose. Aborting.")
                return False
        if not self.wait_for_arm_settled():
            print("[ERROR] Arm did not settle after hover move.")
            return False

        print("Starting filtered EXOTica tactile Z-descent with effort spike stop...")
        if not self._tactile_descent_to_contact(q_dict=qd, joint_index=4):
            print("[ERROR] Tactile descent failed before contact.")
            return False
        if self.STRATEGY == "top_down_clamp":
            print("[top_down_clamp] Contact confirmed — retracting 5mm before gripper close...")
            if not self._retract_after_contact(q_dict=qd, target_x=hover_x, target_y=hover_y):
                print("[ERROR] Contact retract failed or did not move upward. Aborting hold.")
                return False
            print("Closing gripper for top_down_clamp...")
            if not self._close_gripper_for_hold():
                return False
        else:
            print(f"Strategy '{self.STRATEGY}': arm pressure applied, gripper stays open.")

        self.publish_state("HOLDING")
        return True

    def execute_hold(self, part_id, target_label, interactive=True, hold_step=None, target_data_override=None):
        print(f"\n[START] {target_label} Hold Sequence on ID: {part_id}")
        if self.device_cfg is not None:
            self._apply_hold_config(self.device_cfg, target_label=target_label, hold_step=hold_step)

        # ── Idempotency: gripper already confirmed holding → treat as success ──
        # Covers the case where the arm is still gripping after a flip or a
        # previous hold step that was not released in between.
        if self.is_holding_object and self._gripper_has_object():
            print(
                "[hold] Object already gripped (is_held=True + gripper object_detected). "
                "Skipping full hold sequence — treating step as success."
            )
            self.publish_hold_status(True)
            self.publish_state("HOLDING")
            return True

        self.publish_hold_status(False)
        self.publish_state("MOVING")
        success = False
        try:
            success = self._run_hold_sequence(part_id, target_label, interactive,
                                              target_data_override=target_data_override)
        except Exception as e:
            self.get_logger().error(f"Crashed: {e}")
        finally:
            # Always ensure servo is stopped so next skill can use trajectory mode
            self.uf850.stop_servo()
            self.publish_hold_status(success)
            self.publish_state("HOLDING" if success else "IDLE")
        return success

def main(args=None):
    rclpy.init(args=args)
    try:
        cfg_path = default_device_config_path()
        print(f"[hold] Loading device config: {cfg_path}")
        device_cfg = DeviceConfig.load(cfg_path)
    except Exception as exc:
        print(f"[WARN] Could not load device config: {exc}. Using defaults.")
        device_cfg = None
    node = ObjectHoldSkill(device_cfg=device_cfg)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(2.0)
    print("Single-Shot Mode: Waiting for live vision data on /vision/agent_state...")

    is_held = False
    try:
        target_id = None
        target_label_to_pass = ""

        last_status_t = 0.0
        while rclpy.ok() and target_id is None:
            target = node._select_hold_target()
            if target:
                target_id = target.get('id')
                target_label_to_pass = target.get('label', 'case')
            if target_id is None:
                now = time.time()
                if now - last_status_t >= 5.0:
                    with node.data_lock:
                        labels = [t.get('label', '?') for t in node.latest_targets[:10]]
                    print(f"Waiting for hold target. Current vision labels: {labels}")
                    last_status_t = now
                time.sleep(0.5)

        if target_id is not None:
            print(f"Vision data received! Target ID: {target_id}. Executing sequence...")
            is_held = node.execute_hold(target_id, target_label=target_label_to_pass, interactive=False)

            print(f"Single-shot execution complete. Status: {is_held}. Broadcasting state. Press Ctrl+C to exit.")
            while rclpy.ok():
                node.publish_hold_status(is_held)
                time.sleep(1.0)

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
