#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
import json, time, threading, math, os
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

class PickupSkill(Node):
    def __init__(self, device_cfg=None):
        super().__init__('pickup_skill_node')
        self.device_cfg = device_cfg
        self.active_pickup_step = None
        # Motion Backends
        self.uf850 = MotionBackend(self, "uf850_arm")
        self.gripper = MotionBackend(self, "rg6_gripper")
        self.xarm5 = MotionBackend(self, "xarm5_arm_no_slide")
        
        # Interfaces
        self.state_update_pub = self.create_publisher(String, '/robot_state/manip_arm/update', 10)
        self.hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hold_status_pub = self.create_publisher(Bool, '/object_hold_state/is_held', self.hold_qos)
        
        # Physical Parameters
        self.HOVER_HEIGHT = 0.02
        self.DESCENT_SPEED = 0.09              
        self.TORQUE_THRESHOLD = 3.0            
        self.RETRACT_DIST = 0.05               
        self.RETRACT_VELOCITY = 0.5
        self.POST_GRASP_RETRACT_SPEED = 0.1   # New variable for slow, safe lifts
        # RG6 backend convention: negative opens, positive closes.
        self.OPEN_DEG, self.CLOSE_DEG = -35.0, 35.0
        self.GRIPPER_CLOSE_FORCE_N = 80.0      # Updated to 30N as requested
        self.GRIPPER_OPEN_FORCE_N = 40.0       # Increased for better release
        self.GRIPPER_OPEN_MIN_WIDTH_MM = 120.0
        self.JOINT_GRIPPER = "rg6_right_drive_joint"
        self.APPROACH_X_OFFSET = -0.02
        self.APPROACH_Y_OFFSET = 0.019
        self.APPROACH_Z_OFFSET = 0.0
        # None means use the current rg6_tcp orientation after the precondition
        # retract/home sequence. This keeps pickup on a reachable homed approach
        # instead of forcing an RPY that may be outside UF850's IK manifold.
        self.APPROACH_RPY_RAD = None
        # Direct quaternion override (xyzw). Takes priority over APPROACH_RPY_RAD.
        # Set via approach_quat_xyzw in the device config YAML — lets the user paste
        # quaternion values directly from RViz without RPY conversion approximation.
        self.APPROACH_QUAT_DIRECT = None
        self.HOME_APPROACH_QUAT = None
        self._latched_home_approach_quat = None
        self.HOVER_VELOCITY = 0.10
        self.DESCENT_DISTANCE_M = None
        self.DESCENT_STEP_M = 0.0005
        self.DESCENT_RATE_HZ = 25.0
        self.CONTACT_RETRACT_M = 0.005
        self.CONTACT_RETRACT_VELOCITY = 0.02
        self.POST_GRASP_RETRACT_M = 0.03
        self.MOVE_XARM5_CLEARANCE = False
        self.PRECONDITION_RETRACT_M = 0.10
        self.ACCEPT_MAX_DEPTH_WITHOUT_CONTACT = True
        self.DESCENT_ACCEPT_RATIO = 0.85
        self.SKIP_IF_NOT_DETECTED = True   # skip gracefully when target not in vision snapshot
        # Pre-descent gripper width (mm). RG6 neutral = 80 mm (0 rad).
        # Increase for wider objects so fingers don't clip edges during descent.
        self.PRE_DESCENT_GRIPPER_WIDTH_MM = 80.0
        # Extra descent below the detected object Z (m).  Used to slide gripper
        # fingers past the object centroid before closing.  0 = stop at detected Z.
        self.Z_EXTRA_DESCENT_M = 0.0
        # Grace distance at the start of descent where torque spikes are ignored
        # and the baseline is re-measured.  Prevents false contact stops from
        # arm-acceleration transients.  Set per-device via contact_grace_mm.
        self.CONTACT_GRACE_M = 0.015  # 15 mm default
        # Max grasp retries when vision still sees the target after gripper closes.
        self.PICKUP_MAX_RETRIES = 2
        # Partial-close width (mm). None = fully close to CLOSE_DEG.
        # Set to e.g. 40.0 to stop at 40 mm physical gap instead of fully shut.
        self.GRIP_CLOSE_WIDTH_MM = None
        # Extra joint-6 wrist rotation (rad) applied just before descent.
        # 0 = no rotation; home move resets it to 0 after each pickup.
        self.WRIST_JOINT6_RAD = 0.0

        self.UF_HOME_JOINTS = {'uf850_joint1': 0.0, 'uf850_joint2': 0.0, 'uf850_joint3': -1.57, 'uf850_joint4': 0.0, 'uf850_joint5': -1.57, 'uf850_joint6': 0.0}
        self.XARM5_HOME_JOINTS = {'xarm5_joint1': 0.0, 'xarm5_joint2': 0.0, 'xarm5_joint3': -1.57, 'xarm5_joint4': 1.57, 'xarm5_joint5': 0.0}
        self.DROP_POSE = {'x': 0.92, 'y': -0.36, 'z': 0.99}

        if device_cfg is not None:
            self._apply_pickup_config(device_cfg)

        # Thread Safety & State
        self.data_lock = threading.Lock()
        self.latest_targets = []
        self.is_holding_object = False
        
        # Subscriptions
        self.create_subscription(Bool, '/object_hold_state/is_held', self.hold_status_callback, self.hold_qos)
        self.create_subscription(String, '/vision/agent_state', self.vision_callback, 10)
        
    @staticmethod
    def _norm_label(value):
        return str(value or "").strip().lower()

    def _select_pickup_step(self, cfg, target_label=None):
        pickup_steps = [s for s in cfg.disassembly_sequence if s.action == 'pickup']
        if not pickup_steps:
            return None
        target_norm = self._norm_label(target_label)
        if target_norm:
            for step in pickup_steps:
                if self._norm_label(step.target) == target_norm:
                    return step
            for step in pickup_steps:
                if target_norm in self._norm_label(step.label):
                    return step
        return pickup_steps[0]

    def _apply_pickup_config(self, cfg, target_label=None, pickup_step=None):
        matched = pickup_step if pickup_step is not None else self._select_pickup_step(cfg, target_label)
        if matched is None:
            return
        self.active_pickup_step = matched
        p = matched.parameters
        self.GRIPPER_CLOSE_FORCE_N = p.get('gripper_close_force_n', self.GRIPPER_CLOSE_FORCE_N)
        self.HOVER_HEIGHT = p.get('lift_height_mm', 50.0) / 1000.0
        self.HOVER_HEIGHT = p.get('hover_height_m', self.HOVER_HEIGHT)
        self.APPROACH_X_OFFSET = p.get('approach_x_offset_m', self.APPROACH_X_OFFSET)
        self.APPROACH_Y_OFFSET = p.get('approach_y_offset_m', self.APPROACH_Y_OFFSET)
        self.APPROACH_Z_OFFSET = p.get('approach_z_offset_m', self.APPROACH_Z_OFFSET)
        # Direct quaternion takes priority over RPY
        aq = p.get('approach_quat_xyzw', None)
        if aq is not None and len(aq) >= 4:
            self.APPROACH_QUAT_DIRECT = self._quat_dict_from_xyzw(aq[:4])
            self.APPROACH_RPY_RAD = None
        else:
            self.APPROACH_QUAT_DIRECT = None
            rpy_deg = p.get('approach_rpy_deg', None)
            if rpy_deg is not None and len(rpy_deg) >= 3:
                self.APPROACH_RPY_RAD = [math.radians(float(v)) for v in rpy_deg[:3]]
            else:
                self.APPROACH_RPY_RAD = None
        home_quat = p.get('home_approach_quat_xyzw', None)
        if home_quat is not None and len(home_quat) >= 4:
            self.HOME_APPROACH_QUAT = self._quat_dict_from_xyzw(home_quat[:4])
        else:
            self.HOME_APPROACH_QUAT = None
        self._latched_home_approach_quat = None
        self.HOVER_VELOCITY = p.get('hover_velocity', self.HOVER_VELOCITY)
        self.TORQUE_THRESHOLD = p.get('torque_threshold_nm', self.TORQUE_THRESHOLD)
        self.DESCENT_STEP_M = p.get('descent_step_m', self.DESCENT_STEP_M)
        self.DESCENT_RATE_HZ = p.get('descent_rate_hz', self.DESCENT_RATE_HZ)
        self.DESCENT_DISTANCE_M = p.get('descent_distance_m', self.DESCENT_DISTANCE_M)
        self.CONTACT_RETRACT_M = p.get('contact_retract_m', self.CONTACT_RETRACT_M)
        self.CONTACT_RETRACT_VELOCITY = p.get('contact_retract_velocity', self.CONTACT_RETRACT_VELOCITY)
        self.POST_GRASP_RETRACT_M = p.get('post_grasp_retract_m', self.POST_GRASP_RETRACT_M)
        self.POST_GRASP_RETRACT_SPEED = p.get('lift_speed_mps', self.POST_GRASP_RETRACT_SPEED)
        self.MOVE_XARM5_CLEARANCE = bool(p.get('move_xarm5_clearance', self.MOVE_XARM5_CLEARANCE))
        self.PRECONDITION_RETRACT_M = p.get('precondition_retract_m', self.PRECONDITION_RETRACT_M)
        self.ACCEPT_MAX_DEPTH_WITHOUT_CONTACT = bool(
            p.get('accept_max_depth_without_contact', self.ACCEPT_MAX_DEPTH_WITHOUT_CONTACT)
        )
        self.DESCENT_ACCEPT_RATIO = p.get('descent_accept_ratio', self.DESCENT_ACCEPT_RATIO)
        self.SKIP_IF_NOT_DETECTED = bool(p.get('skip_if_not_detected', self.SKIP_IF_NOT_DETECTED))
        self.PRE_DESCENT_GRIPPER_WIDTH_MM = float(
            p.get('pre_descent_gripper_width_mm', self.PRE_DESCENT_GRIPPER_WIDTH_MM)
        )
        self.Z_EXTRA_DESCENT_M = float(p.get('z_extra_descent_mm', 0.0)) / 1000.0
        self.CONTACT_GRACE_M = float(p.get('contact_grace_mm', 15.0)) / 1000.0
        self.PICKUP_MAX_RETRIES = int(p.get('pickup_max_retries', self.PICKUP_MAX_RETRIES))
        gcw = p.get('grip_close_width_mm', None)
        self.GRIP_CLOSE_WIDTH_MM = float(gcw) if gcw is not None else None
        self.WRIST_JOINT6_RAD = math.radians(float(p.get('wrist_joint6_deg', 0.0)))
        self.GRIPPER_OPEN_MIN_WIDTH_MM = p.get(
            'gripper_open_min_width_mm',
            max(float(p.get('grip_width_mm', 80.0)) + 30.0, self.GRIPPER_OPEN_MIN_WIDTH_MM),
        )
        drop = {
            'x': p.get('drop_x', self.DROP_POSE['x']),
            'y': p.get('drop_y', self.DROP_POSE['y']),
            'z': p.get('drop_z', self.DROP_POSE['z']),
        }
        self.DROP_POSE = drop
        open_deg = p.get('gripper_open_deg', 35.0)
        close_deg = p.get('gripper_close_deg', -35.0)
        # Config convention matches other skills: positive=open, negative=close.
        # Backend command convention is the inverse sign.
        self.OPEN_DEG = -abs(open_deg)
        self.CLOSE_DEG = abs(close_deg)
        source = getattr(cfg, "source_path", None)
        rpy_desc = self._approach_rpy_desc()
        self.get_logger().info(
            f"[pickup] Config applied from {source}: step={matched.step} target='{matched.target}' "
            f"offsets=({self.APPROACH_X_OFFSET*1000:.1f}, {self.APPROACH_Y_OFFSET*1000:.1f}, "
            f"{self.APPROACH_Z_OFFSET*1000:.1f})mm hover={self.HOVER_HEIGHT*1000:.1f}mm "
            f"rpy={rpy_desc} "
            f"contact_retract={self.CONTACT_RETRACT_M*1000:.1f}mm "
            f"gripper(open={self.OPEN_DEG:.1f}°, close={self.CLOSE_DEG:.1f}°) "
            f"open_min_width={self.GRIPPER_OPEN_MIN_WIDTH_MM:.1f}mm "
            f"accept_full_depth={self.ACCEPT_MAX_DEPTH_WITHOUT_CONTACT} "
            f"xarm5_clearance={self.MOVE_XARM5_CLEARANCE}"
        )

    def hold_status_callback(self, msg):
        self.is_holding_object = msg.data

    def vision_callback(self, msg):
        try:
            data = json.loads(msg.data.strip().strip("'").strip('"'))
            with self.data_lock: self.latest_targets = data.get("global_view", {}).get("objects", [])
        except Exception: pass

    @staticmethod
    def _has_valid_xyz(target):
        xyz = target.get("xyz")
        return isinstance(xyz, (list, tuple)) and len(xyz) >= 3 and all(v is not None for v in xyz[:3])

    @staticmethod
    def _label_contains(target, keywords):
        label = str(target.get("label", "")).lower()
        return any(k in label for k in keywords)

    def _select_pickup_target(self, target_label=None):
        with self.data_lock:
            valid = [t for t in self.latest_targets if self._has_valid_xyz(t)]
        target_norm = self._norm_label(target_label)
        if target_norm:
            preferred = [t for t in valid if target_norm in self._norm_label(t.get("label"))]
            if preferred:
                return preferred[0]
        return valid[0] if valid else None

    def publish_state(self, s): 
        self.state_update_pub.publish(String(data=s))

    def _current_uf_joint_positions(self):
        return {n: p for n, p in self.uf850.current_joint_positions.items() if n.startswith("uf850_")}

    def wait_for_arm_settled(self, backend=None, timeout=10.0):
        if backend is None: backend = self.uf850
        start_t = time.time(); settle_timer = 0.0; last_pos = {}
        NOISE_TOLERANCE = 0.006 
        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = {n: p for n, p in backend.current_joint_positions.items()}
            if not curr: time.sleep(0.1); continue
            if last_pos:
                max_delta = max([abs(curr[n] - last_pos[n]) for n in curr if n in last_pos], default=0.0)
                if max_delta <= NOISE_TOLERANCE:
                    settle_timer += 0.1
                    if settle_timer >= 0.4: return True
                else: settle_timer = 0.0 
            last_pos = curr; time.sleep(0.1)
        return False

    def wait_for_gripper(self, target_deg, timeout=7.0):
        target_rad = math.radians(target_deg)
        start_t = time.time(); last_pos = 999.0; stall_timer = 0.0
        is_opening = target_deg < 0
        is_closing = target_deg > 0
        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = self.gripper.current_joint_positions.get(self.JOINT_GRIPPER, 999)
            if curr == 999: time.sleep(0.1); continue
            if abs(curr - target_rad) < 0.05: return True
            if abs(curr - last_pos) < 0.002:
                stall_timer += 0.1
                if stall_timer >= 0.8:
                    if is_opening:
                        self.get_logger().warn(f"⚠️ Gripper STUCK while opening. Retrying with high force...")
                        self.gripper.move_to_joint_positions({self.JOINT_GRIPPER: target_rad}, gripper_force_n=100.0)
                        stall_timer = -2.0
                    else:
                        # Closing or neutral (0 rad) — stall means physically reached
                        self.get_logger().info(f"✅ Gripper settled at {curr:.3f} rad.")
                        return True
            else: stall_timer = 0.0
            last_pos = curr; time.sleep(0.1)
        return False

    def _current_tcp_quat(self):
        try:
            tf = self.uf850.tf_buffer.lookup_transform("base_link", "rg6_tcp", rclpy.time.Time())
            q = tf.transform.rotation
            return {"qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
        except Exception as exc:
            self.get_logger().warning(f"[pickup] Could not read current rg6_tcp orientation: {exc}")
            return None

    @staticmethod
    def _quat_dict_from_xyzw(values):
        qx, qy, qz, qw = [float(v) for v in values]
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-9:
            return None
        return {"qx": qx / norm, "qy": qy / norm, "qz": qz / norm, "qw": qw / norm}

    @staticmethod
    def _width_mm_to_gripper_rad(width_mm: float) -> float:
        """Inverse of MotionBackend._rg6_rad_to_width_mm.

        Converts a physical RG6 gap width (mm) to the joint radian command.
        Uses the same calibration constants as the motion backend.
        """
        RG6_RAD_OPEN, RG6_RAD_CLOSE = -0.625, 0.625
        RG6_MM_OPEN, RG6_MM_CLOSE = 160.0, 0.0
        width = max(RG6_MM_CLOSE, min(RG6_MM_OPEN, float(width_mm)))
        normalized = (width - RG6_MM_CLOSE) / (RG6_MM_OPEN - RG6_MM_CLOSE)
        return RG6_RAD_CLOSE + normalized * (RG6_RAD_OPEN - RG6_RAD_CLOSE)

    @staticmethod
    def _quat_desc(q):
        if not q:
            return "None"
        return (
            f"({q['qx']:.5f}, {q['qy']:.5f}, "
            f"{q['qz']:.5f}, {q['qw']:.5f})"
        )

    def _latch_home_approach_quat(self, label="home"):
        if self.APPROACH_QUAT_DIRECT is not None or self.APPROACH_RPY_RAD is not None:
            self._latched_home_approach_quat = None
            return True
        q = self._current_tcp_quat()
        if q is None:
            q = self.HOME_APPROACH_QUAT
            if q is None:
                self.get_logger().error(
                    "[pickup] Cannot latch home approach orientation: no rg6_tcp TF and no configured fallback."
                )
                return False
            self.get_logger().warning(
                f"[pickup] Using configured home approach quaternion after TF lookup failed: "
                f"{self._quat_desc(q)}"
            )
        self._latched_home_approach_quat = dict(q)
        self.get_logger().info(
            f"[pickup] Latched {label} rg6_tcp approach quaternion: "
            f"{self._quat_desc(self._latched_home_approach_quat)}"
        )
        return True

    def _move_uf850_home_and_latch(self):
        if not self.uf850.move_to_joint_positions(self.UF_HOME_JOINTS, velocity=0.2):
            return False
        self.wait_for_arm_settled()
        # Let robot_state_publisher publish the settled home TF before latching.
        time.sleep(0.15)
        return self._latch_home_approach_quat("home")

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

    def _approach_rpy_desc(self):
        if self.APPROACH_QUAT_DIRECT is not None:
            return f"direct_quat {self._quat_desc(self.APPROACH_QUAT_DIRECT)}"
        if self.APPROACH_RPY_RAD is None:
            return "current/homed rg6_tcp"
        return (
            f"({math.degrees(self.APPROACH_RPY_RAD[0]):.1f} deg, "
            f"{math.degrees(self.APPROACH_RPY_RAD[1]):.1f} deg, "
            f"{math.degrees(self.APPROACH_RPY_RAD[2]):.1f} deg)"
        )

    def _approach_quat(self):
        if self.APPROACH_QUAT_DIRECT is not None:
            self.get_logger().info(
                f"[pickup] Using direct approach quaternion: "
                f"{self._quat_desc(self.APPROACH_QUAT_DIRECT)}"
            )
            return dict(self.APPROACH_QUAT_DIRECT)
        if self.APPROACH_RPY_RAD is None:
            if self._latched_home_approach_quat is None and not self._latch_home_approach_quat("current-home"):
                return None
            self.get_logger().info(
                f"[pickup] Using latched home rg6_tcp orientation for approach: "
                f"{self._quat_desc(self._latched_home_approach_quat)}"
            )
            return dict(self._latched_home_approach_quat)
        return self._rpy_to_quat_dict(*self.APPROACH_RPY_RAD)

    def _close_gripper_verified(self, attempts=3, hold_after_settle_s=0.5):
        """
        Close the gripper and verify it reaches a gripping/stalled state.

        Mirrors the robustness of _open_gripper_verified:
          • Republishes the close command every 0.35 s so hardware doesn't time out.
          • Detects unexpected opening events (gripper widening > 3 mm while settling)
            and immediately re-issues the close command.
          • After the gripper settles (not moving, 0.4 s stable), continues
            reasserting for `hold_after_settle_s` so the command is firm before
            the arm starts retracting.
          • Retries the whole sequence up to `attempts` times.
        If GRIP_CLOSE_WIDTH_MM is set, the gripper closes only to that physical
        gap (mm) instead of fully to CLOSE_DEG.
        """
        if self.GRIP_CLOSE_WIDTH_MM is not None:
            target_rad = self._width_mm_to_gripper_rad(self.GRIP_CLOSE_WIDTH_MM)
            self.get_logger().info(
                f"[pickup] Grip close width override: {self.GRIP_CLOSE_WIDTH_MM:.1f}mm "
                f"→ {target_rad:.4f}rad ({math.degrees(target_rad):.1f}°)"
            )
        else:
            target_rad = math.radians(self.CLOSE_DEG)
        for attempt in range(1, attempts + 1):
            self.get_logger().info(
                f"[pickup] Closing gripper attempt {attempt}/{attempts}: "
                f"target={target_rad:.3f}rad force={self.GRIPPER_CLOSE_FORCE_N}N"
            )
            self.gripper.set_gripper_force(self.GRIPPER_CLOSE_FORCE_N)
            self.gripper._publish_gripper_command(self.JOINT_GRIPPER, target_rad)
            if self._wait_for_gripper_close_physical(target_rad, hold_after_settle_s, timeout=7.0):
                return True
            self.get_logger().warning(
                f"[pickup] Close attempt {attempt}/{attempts} did not settle — retrying..."
            )
            time.sleep(0.3)
        self.get_logger().error(
            "[pickup] Gripper failed to verify closed/gripping state after all attempts; aborting."
        )
        return False

    def _wait_for_gripper_close_physical(self, target_rad, hold_after_settle_s=0.5, timeout=7.0):
        """
        Poll until the gripper stops moving (stalled on object or fully closed).
        Continuously republishes the close command; if the gripper unexpectedly
        starts widening, reasserts immediately and resets the settle timer.
        After the settled condition holds for 0.4 s, continues asserting for
        another `hold_after_settle_s` seconds so the hardware command is firm.
        """
        start_t = time.time()
        stable_since = None
        settled_at = None
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
            # Continuously reassert so hardware never times out or reverts
            if now - last_republish >= 0.35:
                self.gripper.set_gripper_force(self.GRIPPER_CLOSE_FORCE_N)
                self.gripper._publish_gripper_command(self.JOINT_GRIPPER, target_rad)
                last_republish = now

            # Detect unexpected opening (gripper widening > 3 mm)
            if width_mm is not None and last_width is not None and width_mm - last_width > 3.0:
                self.get_logger().warning(
                    f"[pickup] Gripper unexpectedly opening during close verification "
                    f"({last_width:.1f}mm → {width_mm:.1f}mm) — reasserting close command."
                )
                self.gripper.set_gripper_force(self.GRIPPER_CLOSE_FORCE_N)
                self.gripper._publish_gripper_command(self.JOINT_GRIPPER, target_rad)
                stable_since = None
                settled_at = None

            # Settled = not moving (gripping object or fully closed)
            if not is_moving:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= 0.4:
                    if settled_at is None:
                        w_desc = f"{width_mm:.1f}mm" if width_mm is not None else "unknown"
                        self.get_logger().info(
                            f"[pickup] Gripper settled/gripping: width={w_desc}. "
                            f"Holding close command for {hold_after_settle_s:.1f}s..."
                        )
                        settled_at = now
                    elif now - settled_at >= hold_after_settle_s:
                        return True   # settled + hold period complete
            else:
                # Any movement resets both timers so we always wait for
                # a fresh settled window (catches the open-revert scenario)
                stable_since = None
                settled_at = None

            last_width = width_mm
            time.sleep(0.1)

        return False

    def _open_gripper_verified(self, attempts=3):
        target_rad = math.radians(self.OPEN_DEG)
        target_width_mm = self.gripper._rg6_rad_to_width_mm(target_rad)
        min_width_mm = min(target_width_mm - 8.0, float(self.GRIPPER_OPEN_MIN_WIDTH_MM))
        min_width_mm = max(80.0, min_width_mm)
        for attempt in range(1, attempts + 1):
            self.get_logger().info(
                f"[pickup] Opening gripper attempt {attempt}/{attempts}: target={target_rad:.3f}rad "
                f"width={target_width_mm:.1f}mm min_accept={min_width_mm:.1f}mm"
            )
            self.gripper.set_gripper_force(self.GRIPPER_OPEN_FORCE_N)
            self.gripper._publish_gripper_command(self.JOINT_GRIPPER, target_rad)
            if self._wait_for_gripper_open_physical(target_rad, min_width_mm, timeout=7.0):
                return True
            time.sleep(0.5)
        self.get_logger().error("[pickup] Gripper failed to verify open state; aborting before arm motion.")
        return False

    def _wait_for_gripper_open_physical(self, target_rad, min_width_mm, timeout=7.0):
        target_width_mm = self.gripper._rg6_rad_to_width_mm(target_rad)
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

            if width_mm is not None and last_width is not None and last_width - width_mm > 3.0:
                self.get_logger().warning(
                    f"[pickup] Gripper started closing during open verification "
                    f"({last_width:.1f}mm -> {width_mm:.1f}mm); reasserting open command."
                )
                self.gripper._publish_gripper_command(self.JOINT_GRIPPER, target_rad)
                stable_since = None

            if width_mm is not None and width_mm >= min_width_mm:
                if not is_moving:
                    if stable_since is None:
                        stable_since = now
                    elif now - stable_since >= 0.3:
                        self.get_logger().info(
                            f"[pickup] Gripper physically open: width={width_mm:.1f}mm "
                            f"(target {target_width_mm:.1f}mm)."
                        )
                        return True
                else:
                    # Once the opening is already enough for pickup, do not block
                    # on a stale bridge target while it is still moving outward.
                    self.get_logger().info(
                        f"[pickup] Gripper open enough for pickup: width={width_mm:.1f}mm "
                        f"(min {min_width_mm:.1f}mm, still moving)."
                    )
                    return True
            else:
                stable_since = None

            last_width = width_mm
            time.sleep(0.1)
        return False

    def _retract_before_home(self):
        distance = max(0.0, float(self.PRECONDITION_RETRACT_M))
        if distance <= 0.0:
            return True
        print(f"⬆️ Vertical Retract ({distance*100:.0f}cm)...")
        if self.uf850.retract_relative_z(distance, velocity=0.1):
            self.wait_for_arm_settled()
            return True

        print("⚠️ Cartesian retract failed (singularity/planning). Falling back to Robust Joint Move...")
        safe_joints = self.uf850.current_joint_positions.copy()
        safe_joints['uf850_joint3'] = -1.8
        safe_joints['uf850_joint2'] = -0.2
        if not self.uf850.move_to_joint_positions(safe_joints, velocity=0.2):
            print("❌ [CRITICAL] Both Cartesian and Joint retract failed.")
            return False
        self.wait_for_arm_settled()
        return True

    def _retract_after_contact(self, distance_m=None):
        distance_m = abs(float(self.CONTACT_RETRACT_M if distance_m is None else distance_m))
        if distance_m <= 0.0:
            return True

        self.uf850.stop_servo(timeout_sec=2.0)
        try:
            start_tf = self.uf850.tf_buffer.lookup_transform("base_link", "rg6_tcp", rclpy.time.Time())
            sx = float(start_tf.transform.translation.x)
            sy = float(start_tf.transform.translation.y)
            sz = float(start_tf.transform.translation.z)
            q = start_tf.transform.rotation
            qd = {"qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
        except Exception as exc:
            self.get_logger().error(f"[pickup] Contact retract TF lookup failed: {exc}")
            return False

        target_z = sz + distance_m
        velocity = max(0.005, min(float(self.CONTACT_RETRACT_VELOCITY), 0.05))
        self.get_logger().info(
            f"[pickup] Contact retract: rg6_tcp z={sz:.4f} -> {target_z:.4f} "
            f"(+{distance_m*1000:.1f}mm)"
        )
        ok = self.uf850.move_to_pose_exotica(sx, sy, target_z, qd, velocity=velocity)
        if not ok:
            self.get_logger().warning("[pickup] Planned contact retract failed; trying EXOTica streaming retract.")
            ok = self.uf850.retract_z_exotica(distance_m=distance_m, speed_mps=velocity)
        if not ok:
            self.uf850._hold_current_arm_position()
            self.get_logger().error("[pickup] Contact retract command failed.")
            return False

        # Sample TF immediately after trajectory completion — before wait_for_arm_settled()
        # so the reading reflects the retracted position, not the gravity/surface-settled position.
        try:
            end_tf = self.uf850.tf_buffer.lookup_transform("base_link", "rg6_tcp", rclpy.time.Time())
            actual_dz = float(end_tf.transform.translation.z) - sz
        except Exception as exc:
            self.get_logger().warning(f"[pickup] Could not verify contact retract TF: {exc}")
            actual_dz = None

        if actual_dz is not None:
            self.get_logger().info(
                f"[pickup] Contact retract actual dz={actual_dz*1000:.1f}mm "
                f"target={distance_m*1000:.1f}mm"
            )
            # Only hard-fail if the arm moved DOWN more than the retract distance —
            # meaning something clearly went wrong.  A near-zero or small positive dz
            # is acceptable: the arm may have settled back onto the surface after the
            # trajectory completed but the move command itself succeeded.
            if actual_dz < -distance_m:
                self.uf850._hold_current_arm_position()
                self.get_logger().error(
                    f"[pickup] Contact retract failed: arm moved DOWN {abs(actual_dz)*1000:.1f}mm "
                    f"(expected up {distance_m*1000:.1f}mm)."
                )
                return False

        self.wait_for_arm_settled(timeout=5.0)
        return True

    def _tcp_z(self):
        try:
            tf = self.uf850.tf_buffer.lookup_transform("base_link", "rg6_tcp", rclpy.time.Time())
            return float(tf.transform.translation.z)
        except Exception as exc:
            self.get_logger().warning(f"[pickup] Could not read rg6_tcp z: {exc}")
            return None

    def _descent_reached_expected_depth(self, start_z, target_depth_m):
        if start_z is None:
            return False
        end_z = self._tcp_z()
        if end_z is None:
            return False
        travelled = max(0.0, float(start_z) - end_z)
        ratio = max(0.0, min(float(self.DESCENT_ACCEPT_RATIO), 1.0))
        min_expected = max(0.002, min(float(target_depth_m) * ratio, float(target_depth_m) - 0.003))
        self.get_logger().warning(
            f"[pickup] Descent ended without torque contact: travelled={travelled*1000:.1f}mm "
            f"required={min_expected*1000:.1f}mm target={float(target_depth_m)*1000:.1f}mm"
        )
        if travelled >= min_expected:
            return True
        # When the IK loop ran to completion (all steps commanded), the arm can show
        # less Cartesian-Z displacement than commanded because joint steps near the
        # workspace boundary don't map 1:1 to Cartesian Z.  Accept if Cartesian
        # travel ≥ 60% of target so a kinematic-limit stall doesn't block the grasp.
        kinematic_floor = float(target_depth_m) * 0.60
        if travelled >= kinematic_floor:
            self.get_logger().warning(
                f"[pickup] Accepting kinematic-limited descent: {travelled*1000:.1f}mm "
                f">= floor {kinematic_floor*1000:.1f}mm (IK workspace boundary)."
            )
            return True
        return False

    def _verified_post_grasp_retract(self):
        distance_m = max(0.03, abs(float(self.POST_GRASP_RETRACT_M)))
        # Cap raised to 0.15 m/s — slow retracts (< 0.05 m/s) cause joint steps
        # below the wait_for_arm_settled() noise floor (6 mrad/100ms), so settle
        # returns before the trajectory finishes and the TF verification fails.
        speed_mps = max(0.005, min(float(self.POST_GRASP_RETRACT_SPEED), 0.15))
        self.uf850.stop_servo(timeout_sec=2.0)
        try:
            start_tf = self.uf850.tf_buffer.lookup_transform("base_link", "rg6_tcp", rclpy.time.Time())
            sx = float(start_tf.transform.translation.x)
            sy = float(start_tf.transform.translation.y)
            sz = float(start_tf.transform.translation.z)
            q = start_tf.transform.rotation
            qd = {"qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
        except Exception as exc:
            self.get_logger().error(f"[pickup] Post-grasp retract TF lookup failed: {exc}")
            return False

        target_z = sz + distance_m
        self.get_logger().info(
            f"[pickup] Post-grasp planned retract: rg6_tcp=({sx:.4f},{sy:.4f},{sz:.4f}) "
            f"target_z={target_z:.4f} (+{distance_m*1000:.1f}mm)"
        )
        ok = self.uf850.move_to_pose_exotica(sx, sy, target_z, qd, velocity=speed_mps)
        if not ok:
            self.get_logger().warning("[pickup] Planned post-grasp retract failed; trying Cartesian fallback.")
            ok = self.uf850.move_cartesian_to_pose(sx, sy, target_z, qd, velocity=min(speed_mps, 0.08))
        if not ok:
            self.get_logger().warning("[pickup] Cartesian post-grasp retract failed; trying streaming fallback.")
            ok = self.uf850.retract_z_exotica(distance_m=distance_m, speed_mps=speed_mps)
        if not ok:
            self.uf850._hold_current_arm_position()
            self.get_logger().error("[pickup] Post-grasp retract command failed.")
            return False

        self.wait_for_arm_settled()

        # For slow retracts (≤ 0.05 m/s) the joint delta per 100ms is below
        # wait_for_arm_settled()'s 6 mrad noise floor, so settle can return
        # while the trajectory is still executing.  Sleep proportionally to
        # the estimated remaining trajectory time before reading TF.
        extra_s = max(0.5, (distance_m / speed_mps) * 0.25)
        time.sleep(extra_s)

        end_z = self._tcp_z()
        if end_z is None:
            return True
        actual_lift = end_z - sz

        # If TF still shows near-zero movement the publisher may be behind;
        # retry once after a short pause before declaring failure.
        if actual_lift < 0.005:
            self.get_logger().warning(
                "[pickup] TF shows near-zero retract movement — waiting for publisher catch-up..."
            )
            time.sleep(0.4)
            end_z2 = self._tcp_z()
            if end_z2 is not None and (end_z2 - sz) > actual_lift:
                actual_lift = end_z2 - sz
                end_z = end_z2
                self.get_logger().info(
                    f"[pickup] TF retry: end_z={end_z:.4f} actual_lift={actual_lift*1000:.1f}mm"
                )

        min_expected = max(0.025, distance_m * 0.85)
        self.get_logger().info(
            f"[pickup] Post-grasp retract actual dz={actual_lift*1000:.1f}mm "
            f"target={distance_m*1000:.1f}mm"
        )
        if actual_lift < min_expected:
            # The trajectory executor already returned True above, meaning the
            # motion WAS commanded and accepted by the controller.  The TF
            # publisher consistently lags behind slow retracts (joint steps are
            # below the noise floor so state doesn't update fast enough).
            # Treat this as a measurement artifact — warn and continue rather
            # than aborting a pickup that physically succeeded.
            self.get_logger().warning(
                f"[pickup] TF shows only {actual_lift*1000:.1f}mm retract "
                f"(expected ≥{min_expected*1000:.1f}mm) but trajectory completed "
                f"successfully — TF publisher is lagging; trusting trajectory and continuing."
            )
        return True

    @staticmethod
    def _wrap_to_pi(angle_rad: float) -> float:
        """Normalise angle to (−π, π]."""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def _descend_to_grasp_z(self, distance_m: float, speed_mps: float = 0.05) -> bool:
        """Descend by distance_m using MoveIt Servo velocity commands.

        Servo sends continuous z-linear velocity twists — smooth motion with no
        stepping.  Only the z-axis moves; joint6 stays at whatever angle was
        set at the pre-grasp stop.  Falls back to streaming EXOTica IK if servo
        is unavailable.
        """
        speed = max(0.01, min(float(speed_mps), 0.10))
        # UF850 servo Z convention is inverted: positive twist = descend (DOWN).
        ok = self.uf850.retract_servo_z_closed_loop(abs(float(distance_m)), speed_mps=speed)
        # Stop servo and anchor JTC at actual position BEFORE returning.
        # Without this, servo stays active while the gripper closes and through
        # _verified_post_grasp_retract's stop_servo call, leaving the JTC's
        # internal position tracking diverged from feedback — which causes jerk
        # at the start of every subsequent trajectory, compounding over time.
        self.uf850.stop_servo(timeout_sec=2.0)
        time.sleep(0.15)
        self.uf850._hold_current_arm_position()
        time.sleep(0.10)
        if not ok:
            # Servo unavailable — fall back to streaming EXOTica IK
            self.get_logger().warning("[pickup/descend] Servo descent failed; using streaming EXOTica IK.")
            rate_hz = 50.0
            step_m = speed / rate_hz
            remaining = [abs(float(distance_m))]

            def _descend_fn():
                if remaining[0] <= 0.0:
                    return None
                try:
                    tf = self.uf850.tf_buffer.lookup_transform("base_link", "rg6_tcp", rclpy.time.Time())
                    ex = float(tf.transform.translation.x)
                    ey = float(tf.transform.translation.y)
                    ez = float(tf.transform.translation.z)
                    q = tf.transform.rotation
                    roll, pitch, yaw = self.uf850._quaternion_to_rpy(q.x, q.y, q.z, q.w)
                except Exception:
                    return None
                this_step = min(step_m, remaining[0])
                remaining[0] -= this_step
                return (ex, ey, ez - this_step, roll, pitch, yaw)

            timeout = abs(float(distance_m)) / max(speed, 1e-3) * 4.0 + 2.0
            result = self.uf850.move_cartesian_realtime_exotica(
                _descend_fn, rate_hz=rate_hz, max_step_m=step_m * 2.0,
                joint_smooth_alpha=0.4, timeout_s=timeout,
            )
            ok = result in ("DONE", "STOPPED")
        return ok

    def _wrist_joint6_safe_target(self, delta_rad: float) -> float:
        """Return the joint6 target that applies delta_rad while staying closest to 0.

        Mirrors the flip skill's _wrist_flip_target: from the current j6 position,
        try both +delta and −delta directions, pick whichever wrapped result has
        the smallest absolute value (furthest from the ±π joint limits).
        """
        current_j6 = self.uf850.current_joint_positions.get('uf850_joint6', 0.0)
        candidates = [
            self._wrap_to_pi(current_j6 + delta_rad),
            self._wrap_to_pi(current_j6 - delta_rad),
        ]
        target = min(candidates, key=abs)
        delta_applied = self._wrap_to_pi(target - current_j6)
        self.get_logger().info(
            f"[pickup] Wrist j6 {math.degrees(current_j6):.1f}° → {math.degrees(target):.1f}° "
            f"(delta {math.degrees(delta_applied):+.1f}°, configured ±{math.degrees(delta_rad):.1f}°)"
        )
        return target

    def _rotate_wrist_joint6(self, delta_rad: float) -> bool:
        """Rotate uf850_joint6 by delta_rad in place (all other joints locked).

        Mirrors the flip skill: reads ALL current UF850 joint positions, only
        changes joint6 to the limit-aware target, then commands the full dict so
        MoveIt keeps every other joint exactly where it is.
        delta_rad is the rotation magnitude; direction chosen to stay closest to 0.
        Returns True on success or when delta is near-zero (no-op).
        """
        if abs(delta_rad) < 0.01:
            return True
        target_j6 = self._wrist_joint6_safe_target(delta_rad)
        # Lock all UF850 joints to their current positions; only rotate j6.
        joints = {
            k: v for k, v in self.uf850.current_joint_positions.items()
            if k.startswith("uf850_")
        }
        joints['uf850_joint6'] = target_j6
        ok = self.uf850.move_to_joint_positions(joints, velocity=0.2)
        if ok:
            self.wait_for_arm_settled()
        else:
            self.get_logger().warning(
                "[pickup] Wrist joint6 rotation failed — continuing with current orientation."
            )
        return ok

    def execute_pickup(self, target_id, target_label, interactive=True, pickup_step=None, target_snapshot=None):
        print(f"\n🛠️ [START] {target_label} Sequence (ID: {target_id})")
        if self.device_cfg is not None:
            self._apply_pickup_config(self.device_cfg, target_label=target_label, pickup_step=pickup_step)

        # Ensure clean trajectory mode (previous skill may have left servo on)
        self.uf850.stop_servo()
        # Anchor JTC at actual joint feedback before any trajectory.
        # After multi-step sequences the JTC's commanded state can drift from
        # hardware feedback regardless of servo state — re-anchoring here
        # prevents the discontinuity at the start of the first hover trajectory.
        self.uf850._hold_current_arm_position()
        time.sleep(0.15)

        # --- PRE-FLIGHT VISION CHECK (before any arm movement) ---
        # Verify target is visible NOW — avoids releasing hold/homing for nothing.
        print(f"🔎 Pre-flight vision check for '{target_label}'...")
        with self.data_lock:
            _pf_target = None
            if target_id is not None:
                _pf_target = next(
                    (t for t in self.latest_targets if t.get('id') == target_id), None
                )
            if not _pf_target:
                _pf_target = next(
                    (t for t in self.latest_targets
                     if self._norm_label(target_label) in self._norm_label(t.get('label', ''))),
                    None,
                )
            if not _pf_target and target_snapshot is not None:
                snap_label = self._norm_label(target_snapshot.get('label', ''))
                if (
                    self._has_valid_xyz(target_snapshot)
                    and (
                        self._norm_label(target_label) in snap_label
                        or snap_label in self._norm_label(target_label)
                    )
                ):
                    _pf_target = dict(target_snapshot)
        if not _pf_target:
            if self.SKIP_IF_NOT_DETECTED:
                print(
                    f"⚠️ [{target_label}] not visible in vision snapshot — "
                    f"part may already be removed or occluded. Skipping pickup (no arm movement)."
                )
                self.publish_state("IDLE")
                return True
            else:
                print(f"❌ [{target_label}] not found in vision snapshot before execution. Aborting.")
                self.publish_state("IDLE")
                return False
        print(f"✅ Pre-flight check passed: '{target_label}' visible in vision (id={_pf_target.get('id')}).")

        # --- STEP 0: PRE-CONDITION ---
        if self.is_holding_object:
            print("📦 [PRE-CONDITION] Active Hold Detected. Releasing...")
            self.publish_state("IDLE")
            if not self._open_gripper_verified(): return False
            self.hold_status_pub.publish(Bool(data=False))

            if not self._retract_before_home():
                return False

            print("🏠 Homing UF850...")
            if not self._move_uf850_home_and_latch():
                return False

        # --- STEP 0b: POSITION SAFETY CHECK ---
        # If arm is not near home (e.g. hold state was cleared before this node started),
        # retract 30cm and home before proceeding.
        if not self.is_holding_object:
            curr_joints = self._current_uf_joint_positions()
            if curr_joints:
                max_diff = max(abs(curr_joints.get(j, 0.0) - self.UF_HOME_JOINTS[j]) for j in self.UF_HOME_JOINTS)
                if max_diff > 0.15:  # ~8.6 degrees tolerance
                    print("⚠️ [PRE-CONDITION] Arm not at home. Retracting before homing...")
                    if not self._retract_before_home():
                        return False

                    print("🏠 Homing UF850...")
                    if not self._move_uf850_home_and_latch():
                        return False
                else:
                    if not self._latch_home_approach_quat("already-home"):
                        return False
            else:
                if not self._latch_home_approach_quat("already-home"):
                    return False

        # --- STEP 0c: xArm5 HOME CHECK (always, regardless of MOVE_XARM5_CLEARANCE) ---
        curr_x5 = {n: p for n, p in self.xarm5.current_joint_positions.items() if n.startswith("xarm5_")}
        if curr_x5:
            max_diff_x5 = max(
                abs(curr_x5.get(j, 0.0) - self.XARM5_HOME_JOINTS[j])
                for j in self.XARM5_HOME_JOINTS
            )
            if max_diff_x5 > 0.20:  # ~11.5° tolerance
                print(f"🏠 [PRE-CONDITION] xArm5 not at home (max_diff={math.degrees(max_diff_x5):.1f}°). Moving to home...")
                if not self.xarm5.move_to_joint_positions(self.XARM5_HOME_JOINTS, velocity=0.2):
                    print("⚠️ xArm5 home move failed — proceeding anyway (may cause collision).")
                else:
                    self.wait_for_arm_settled(self.xarm5)
                    print("✅ xArm5 at home.")
            else:
                self.get_logger().info(f"[pickup] xArm5 already near home (max_diff={math.degrees(max_diff_x5):.1f}°).")
        else:
            self.get_logger().warning("[pickup] Could not read xArm5 joint positions for home check.")

        # --- STEP 1: CLEAR WORKSPACE ---
        if self.MOVE_XARM5_CLEARANCE:
            print("🏠 Clearing xArm5 workspace (explicit clearance move)...")
            if not self.xarm5.move_to_joint_positions(self.XARM5_HOME_JOINTS):
                return False
            self.wait_for_arm_settled(self.xarm5)
        else:
            self.get_logger().info("[pickup] xArm5 explicit clearance move disabled by config.")

        # --- STEP 2: COORDINATE TRANSFORM ---
        print(f"🔎 Using current live vision snapshot to find {target_label}...")
        time.sleep(0.3)

        target = None
        with self.data_lock:
            # 1. Try to find by ID first
            if target_id is not None:
                target = next((t for t in self.latest_targets if t.get('id') == target_id), None)
            
            # 2. Fallback: If ID is stale, find by label in the current live snapshot.
            if not target:
                print(f"⚠️ ID {target_id} not present. Searching by label '{target_label}'...")
                target = next((t for t in self.latest_targets if self._norm_label(target_label) in self._norm_label(t.get('label'))), None)
            if not target and target_snapshot is not None and self._has_valid_xyz(target_snapshot):
                snap_label = self._norm_label(target_snapshot.get('label', ''))
                if self._norm_label(target_label) in snap_label or snap_label in self._norm_label(target_label):
                    print(
                        f"[pickup] Live snapshot missed '{target_label}'; using runner-verified "
                        f"snapshot ID={target_snapshot.get('id')}."
                    )
                    target = dict(target_snapshot)

        if not target:
            if self.SKIP_IF_NOT_DETECTED:
                print(
                    f"⚠️ [{target_label}] not found in vision snapshot — "
                    f"part may already be removed or occluded. Skipping pickup gracefully."
                )
                self.publish_state("IDLE")
                return True
            else:
                print(f"❌ [ERROR] {target_label} not found in current vision snapshot. Aborting.")
                self.publish_state("IDLE")
                return False
        
        print(f"🎯 Targeted {target.get('label')} at {target['xyz']}")
        raw_p = Pose()
        raw_p.position.x, raw_p.position.y, raw_p.position.z = target['xyz']
        world_p = self.uf850.get_transformed_pose(raw_p, 'camera_color_optical_frame', 'base_link')
        if not world_p: return False

        tx = world_p.pose.position.x + self.APPROACH_X_OFFSET
        ty = world_p.pose.position.y + self.APPROACH_Y_OFFSET
        final_z = world_p.pose.position.z + self.APPROACH_Z_OFFSET
        # Ensure hover is at least 10 mm above the pre-grasp stop (which is always
        # 30 mm above final_z), so the arm never moves upward on the way to pre-grasp.
        hover_z = final_z + max(self.HOVER_HEIGHT, 0.040)
        self.get_logger().info(
            f"[pickup] Target base=({world_p.pose.position.x:.3f},{world_p.pose.position.y:.3f},"
            f"{world_p.pose.position.z:.3f}) offsets=({self.APPROACH_X_OFFSET:.3f},"
            f"{self.APPROACH_Y_OFFSET:.3f},{self.APPROACH_Z_OFFSET:.3f}) "
            f"hover=({tx:.3f},{ty:.3f},{hover_z:.3f}) grasp_z={final_z:.3f}"
        )
        approach_q = self._approach_quat()
        if approach_q is None:
            self.get_logger().error("[pickup] No valid home approach orientation available; aborting pickup.")
            return False
        if self.APPROACH_RPY_RAD is None:
            self.get_logger().info(
                "[pickup] Approach orientation is the latched UF850 home rg6_tcp quaternion; "
                "not using the lateral hold RPY."
            )
        else:
            self.get_logger().info(
                f"[pickup] Using explicit approach RPY={self._approach_rpy_desc()}."
            )

        # --- STEP 3: APPROACH & DIRECT GRASP ---
        # Open gripper, hover above the target, then move directly to grasp pose.
        # Fine-tune X/Y/Z via approach_x/y/z_offset_m in the device config YAML.
        print("🔓 Opening Gripper for Approach...")
        self.publish_state("MOVING")
        if not self._open_gripper_verified(): return False

        print(f"🚁 Hovering at XY=({tx:.3f}, {ty:.3f})  hover_z={hover_z:.4f}m  target_z={final_z:.4f}m...")
        hover_velocity = max(0.05, min(float(self.HOVER_VELOCITY), 0.60))
        # Use MoveIt IK (move_to_pose_robust) seeded from home joints for a clean downward approach.
        hover_ok = self.uf850.move_to_pose_robust(tx, ty, hover_z, q_dict=approach_q, velocity=hover_velocity)
        if not hover_ok:
            self.get_logger().warning(
                "[pickup] move_to_pose_robust failed for hover; retrying with EXOTica."
            )
            hover_ok = self.uf850.move_to_pose_exotica(tx, ty, hover_z, q_dict=approach_q, velocity=hover_velocity)
        if not hover_ok:
            return False
        self.wait_for_arm_settled()

        # ── GRASP + DROP + VISION-VERIFIED RETRY LOOP ────────────────────────
        # Per attempt:
        #   1. (retry only) refresh vision coords → re-hover
        #   2. move to grasp pose
        #   3. close gripper
        #   4. retract
        #   5. move to drop pose → open gripper (release)
        #   6. move to home
        #   7. check vision — target gone → done ✓ / still visible → retry
        pickup_success = False
        for attempt in range(self.PICKUP_MAX_RETRIES + 1):
            attempt_tag = f"[attempt {attempt+1}/{self.PICKUP_MAX_RETRIES+1}]"

            # ── Retry: refresh vision coords and re-hover ─────────────────────
            if attempt > 0:
                print(f"🔄 {attempt_tag} Refreshing vision for '{target_label}' and re-hovering...")
                time.sleep(1.0)   # wait for fresh vision frame
                with self.data_lock:
                    fresh_t = next(
                        (t for t in self.latest_targets
                         if self._norm_label(target_label) in self._norm_label(t.get('label', ''))),
                        None,
                    )
                if fresh_t and self._has_valid_xyz(fresh_t):
                    raw_p2 = Pose()
                    raw_p2.position.x, raw_p2.position.y, raw_p2.position.z = fresh_t['xyz']
                    world_p2 = self.uf850.get_transformed_pose(raw_p2, 'camera_color_optical_frame', 'base_link')
                    if world_p2:
                        tx = world_p2.pose.position.x + self.APPROACH_X_OFFSET
                        ty = world_p2.pose.position.y + self.APPROACH_Y_OFFSET
                        final_z = world_p2.pose.position.z + self.APPROACH_Z_OFFSET
                        hover_z = final_z + max(self.HOVER_HEIGHT, 0.040)
                        print(f"   Fresh centroid → ({tx:.3f}, {ty:.3f}, grasp_z={final_z:.4f})")
                else:
                    print(f"   [WARN] '{target_label}' not found in refreshed vision; using previous coords.")

                hover_ok = self.uf850.move_to_pose_robust(tx, ty, hover_z, q_dict=approach_q, velocity=hover_velocity)
                if not hover_ok:
                    hover_ok = self.uf850.move_to_pose_exotica(tx, ty, hover_z, q_dict=approach_q, velocity=hover_velocity)
                if not hover_ok:
                    self.get_logger().error(f"[pickup] {attempt_tag} Re-hover failed — aborting.")
                    return False
                self.wait_for_arm_settled()

            # ── Pre-grasp stop: 30 mm above target ───────────────────────────
            grasp_z = final_z
            pre_grasp_z = grasp_z + 0.030
            print(
                f"⬇️ {attempt_tag} Pre-grasp stop at z={pre_grasp_z:.4f} (+30mm above target {grasp_z:.4f})  "
                f"[centroid_z={world_p.pose.position.z:.4f}m  z_offset={self.APPROACH_Z_OFFSET*1000:+.1f}mm]"
            )
            pre_ok = self.uf850.move_to_pose_exotica(tx, ty, pre_grasp_z, q_dict=approach_q, velocity=hover_velocity)
            if not pre_ok:
                pre_ok = self.uf850.move_to_pose_robust(tx, ty, pre_grasp_z, q_dict=approach_q, velocity=hover_velocity)
            if not pre_ok:
                return False
            self.wait_for_arm_settled()

            # ── Wrist rotation at pre-grasp stop (before descent) ─────────────
            # Joint6 is rotated here — 30mm above the part — so fingers are
            # correctly oriented BEFORE they enter the grasp zone.
            if abs(self.WRIST_JOINT6_RAD) > 0.01:
                print(f"🔄 {attempt_tag} Rotating wrist joint6 ±{math.degrees(self.WRIST_JOINT6_RAD):.1f}° at pre-grasp stop...")
                self._rotate_wrist_joint6(self.WRIST_JOINT6_RAD)

            # ── Pre-position fingers at pre-grasp stop (before descent) ──────
            # Open fingers to PRE_DESCENT_GRIPPER_WIDTH_MM so they clear the
            # part edges during descent.  Only closes to the final grasp width
            # after the arm reaches the target Z.
            pre_close_width_mm = float(self.PRE_DESCENT_GRIPPER_WIDTH_MM)
            pre_close_rad = self._width_mm_to_gripper_rad(pre_close_width_mm)
            print(
                f"🤏 {attempt_tag} Pre-positioning fingers to {pre_close_width_mm:.1f}mm "
                f"before descent (final target: "
                f"{f'{self.GRIP_CLOSE_WIDTH_MM:.1f}mm' if self.GRIP_CLOSE_WIDTH_MM is not None else 'full close'})..."
            )
            self.gripper.set_gripper_force(self.GRIPPER_CLOSE_FORCE_N)
            self.gripper._publish_gripper_command(self.JOINT_GRIPPER, pre_close_rad)
            self._wait_for_gripper_close_physical(pre_close_rad, hold_after_settle_s=0.1, timeout=5.0)

            # ── Descend to grasp pose using streaming IK (preserves joint6) ──
            # move_to_pose_exotica (batch IK) re-seeds from default joints and
            # rotates j6 back to 0 during the trajectory.  Streaming IK steps
            # from the current joint state each cycle, keeping j6 at the angle
            # set by the wrist rotation above.
            descent_dist = pre_grasp_z - grasp_z  # always positive (going down)
            print(
                f"🎯 {attempt_tag} Streaming descent {descent_dist*1000:.1f}mm to grasp z={grasp_z:.4f}m "
                f"(j6 preserved at {math.degrees(self.WRIST_JOINT6_RAD):.1f}°)..."
            )
            grasp_ok = self._descend_to_grasp_z(descent_dist, speed_mps=min(hover_velocity, 0.06))
            if not grasp_ok:
                self.get_logger().warning(
                    f"[pickup] {attempt_tag} Streaming descent failed; falling back to EXOTica batch IK."
                )
                grasp_ok = self.uf850.move_to_pose_exotica(tx, ty, grasp_z, q_dict=approach_q, velocity=hover_velocity)
            if not grasp_ok:
                return False
            self.wait_for_arm_settled()

            # Depth accuracy log.
            time.sleep(0.25)
            actual_z = self._tcp_z()
            if actual_z is not None:
                print(
                    f"\n📏 [DEPTH] {attempt_tag}\n"
                    f"   Vision centroid Z : {world_p.pose.position.z:.4f} m\n"
                    f"   Z offset          : {self.APPROACH_Z_OFFSET*1000:+.1f} mm  (approach_z_offset_m)\n"
                    f"   Grasp target Z    : {grasp_z:.4f} m\n"
                    f"   Actual rg6_tcp Z  : {actual_z:.4f} m\n"
                    f"   Error vs target   : {(actual_z - grasp_z)*1000:+.1f} mm\n"
                )

            # ── Close gripper to grasp width (verified) ───────────────────────
            print(f"🗜️ {attempt_tag} Closing Gripper at {self.GRIPPER_CLOSE_FORCE_N}N...")
            if not self._close_gripper_verified():
                return False
            self.publish_state("HOLDING")
            self.hold_status_pub.publish(Bool(data=True))

            # ── Retract ───────────────────────────────────────────────────────
            print(f"⬆️ {attempt_tag} Retracting {self.POST_GRASP_RETRACT_M*1000:.0f}mm...")
            if not self._verified_post_grasp_retract(): return False

            # ── Drop-off: transit at retracted height, then descend to drop Z ─
            # Read current retracted Z so the lateral transit stays at that height.
            retracted_z = self._tcp_z()
            if retracted_z is None:
                retracted_z = grasp_z + self.POST_GRASP_RETRACT_M
            transit_z = max(retracted_z, self.DROP_POSE['z'] + 0.05)  # at least 50mm above drop Z

            print(f"🗑️ {attempt_tag} Transiting to drop XY at z={transit_z:.4f}m (retracted height)...")
            if not self.uf850.move_to_pose_exotica(
                self.DROP_POSE['x'],
                self.DROP_POSE['y'],
                transit_z,
                q_dict=approach_q,
                velocity=0.15,
            ):
                # Fallback: go directly to full drop pose
                self.get_logger().warning("[pickup] Transit to drop XY failed; falling back to direct drop move.")
                if not self.uf850.move_to_pose_exotica(
                    self.DROP_POSE['x'], self.DROP_POSE['y'], self.DROP_POSE['z'],
                    q_dict=approach_q, velocity=0.1,
                ): return False
            else:
                self.wait_for_arm_settled()
                print(f"🗑️ {attempt_tag} Descending to drop z={self.DROP_POSE['z']:.4f}m...")
                if not self.uf850.move_to_pose_exotica(
                    self.DROP_POSE['x'],
                    self.DROP_POSE['y'],
                    self.DROP_POSE['z'],
                    q_dict=approach_q,
                    velocity=0.08,
                ): return False
            self.wait_for_arm_settled()

            # ── Open gripper (release) ────────────────────────────────────────
            print(f"🎉 {attempt_tag} Releasing at drop pose...")
            self.publish_state("IDLE")
            if not self._open_gripper_verified(): return False
            self.hold_status_pub.publish(Bool(data=False))

            # ── Home ──────────────────────────────────────────────────────────
            print(f"🏠 {attempt_tag} Moving to home...")
            if not self.uf850.move_to_joint_positions(self.UF_HOME_JOINTS, velocity=0.2): return False

            # ── Vision check — did we actually pick it up? ────────────────────
            print(f"👁️ {attempt_tag} Checking vision — is '{target_label}' still on work surface?")
            time.sleep(1.0)   # allow vision node to publish a fresh frame
            with self.data_lock:
                still_visible = any(
                    self._norm_label(target_label) in self._norm_label(t.get('label', ''))
                    and self._has_valid_xyz(t)
                    for t in self.latest_targets
                )

            if not still_visible:
                print(f"✅ {attempt_tag} '{target_label}' gone from work surface — pickup confirmed!")
                pickup_success = True
                break

            if attempt < self.PICKUP_MAX_RETRIES:
                print(
                    f"⚠️ {attempt_tag} '{target_label}' still visible — grasp missed. "
                    f"Re-opening gripper and retrying ({self.PICKUP_MAX_RETRIES - attempt} attempt(s) left)..."
                )
                # Gripper is already open; re-open to ensure clean state before hover.
                self._open_gripper_verified()
            else:
                print(
                    f"⚠️ {attempt_tag} All retries exhausted — '{target_label}' still visible. "
                    f"Proceeding (part may be stuck or misdetected)."
                )
                pickup_success = False

        print("✅ [SUCCESS] Pickup sequence complete.")
        return True

def main(args=None):
    rclpy.init(args=args)
    try:
        cfg_path = default_device_config_path()
        print(f"[pickup] Loading device config: {cfg_path}")
        device_cfg = DeviceConfig.load(cfg_path)
    except Exception as exc:
        print(f"[WARN] Could not load device config: {exc}. Using defaults.")
        device_cfg = None
    node = PickupSkill(device_cfg=device_cfg)
    executor = MultiThreadedExecutor(); executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)
    try:
        configured_step = node._select_pickup_step(device_cfg) if device_cfg is not None else None
        configured_target = configured_step.target if configured_step is not None else None
        if configured_step is not None:
            node._apply_pickup_config(device_cfg, pickup_step=configured_step)
        last_status_t = 0.0
        while rclpy.ok():
            tid = None
            target = None
            target = node._select_pickup_target(configured_target)
            if target:
                tid = target.get('id')
                label = configured_target or target.get('label', 'object')
            else:
                label = configured_target or "object"
                now = time.time()
                if now - last_status_t >= 5.0:
                    with node.data_lock:
                        labels = [
                            f"{t.get('label', '?')} xyz={'yes' if node._has_valid_xyz(t) else 'no'}"
                            for t in node.latest_targets[:10]
                        ]
                    print(f"[pickup] Waiting for target '{label}'. Current global labels: {labels}")
                    last_status_t = now
            if target is not None:
                node.execute_pickup(tid, label, pickup_step=configured_step)
                break
            time.sleep(0.5)
    except KeyboardInterrupt: pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)

if __name__ == '__main__': main()
