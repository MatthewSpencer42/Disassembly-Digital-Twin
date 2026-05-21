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
        self.HOVER_HEIGHT = 0.05    
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
        self.get_logger().info(
            f"[pickup] Config applied from {source}: step={matched.step} target='{matched.target}' "
            f"offsets=({self.APPROACH_X_OFFSET*1000:.1f}, {self.APPROACH_Y_OFFSET*1000:.1f}, "
            f"{self.APPROACH_Z_OFFSET*1000:.1f})mm hover={self.HOVER_HEIGHT*1000:.1f}mm "
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

        self.wait_for_arm_settled(timeout=5.0)
        try:
            end_tf = self.uf850.tf_buffer.lookup_transform("base_link", "rg6_tcp", rclpy.time.Time())
            actual_dz = float(end_tf.transform.translation.z) - sz
        except Exception as exc:
            self.get_logger().warning(f"[pickup] Could not verify contact retract TF: {exc}")
            return True
        min_expected = max(0.002, min(distance_m * 0.6, distance_m - 0.001))
        self.get_logger().info(
            f"[pickup] Contact retract actual dz={actual_dz*1000:.1f}mm "
            f"target={distance_m*1000:.1f}mm"
        )
        if actual_dz < min_expected:
            self.uf850._hold_current_arm_position()
            self.get_logger().error(
                f"[pickup] Contact retract unsafe/incomplete: actual dz={actual_dz*1000:.1f}mm "
                f"(minimum {min_expected*1000:.1f}mm)."
            )
            return False
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
        return travelled >= min_expected

    def _verified_post_grasp_retract(self):
        distance_m = max(0.03, abs(float(self.POST_GRASP_RETRACT_M)))
        speed_mps = max(0.005, min(float(self.POST_GRASP_RETRACT_SPEED), 0.05))
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
        end_z = self._tcp_z()
        if end_z is None:
            return True
        actual_lift = end_z - sz
        min_expected = max(0.025, distance_m * 0.85)
        self.get_logger().info(
            f"[pickup] Post-grasp retract actual dz={actual_lift*1000:.1f}mm "
            f"target={distance_m*1000:.1f}mm"
        )
        if actual_lift < min_expected:
            self.uf850._hold_current_arm_position()
            self.get_logger().error(
                f"[pickup] Post-grasp retract incomplete: actual dz={actual_lift*1000:.1f}mm "
                f"(minimum {min_expected*1000:.1f}mm)."
            )
            return False
        return True

    def execute_pickup(self, target_id, target_label, interactive=True, pickup_step=None):
        print(f"\n🛠️ [START] {target_label} Sequence (ID: {target_id})")
        if self.device_cfg is not None:
            self._apply_pickup_config(self.device_cfg, target_label=target_label, pickup_step=pickup_step)

        # Ensure clean trajectory mode (previous skill may have left servo on)
        self.uf850.stop_servo()

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
            if not self.uf850.move_to_joint_positions(self.UF_HOME_JOINTS, velocity=0.2): 
                return False
            self.wait_for_arm_settled()

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
                    if not self.uf850.move_to_joint_positions(self.UF_HOME_JOINTS, velocity=0.2):
                        return False
                    self.wait_for_arm_settled()

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
        hover_z = final_z + self.HOVER_HEIGHT
        descent_distance = (
            float(self.DESCENT_DISTANCE_M)
            if self.DESCENT_DISTANCE_M is not None
            else self.HOVER_HEIGHT + 0.02
        )
        self.get_logger().info(
            f"[pickup] Target base=({world_p.pose.position.x:.3f},{world_p.pose.position.y:.3f},"
            f"{world_p.pose.position.z:.3f}) offsets=({self.APPROACH_X_OFFSET:.3f},"
            f"{self.APPROACH_Y_OFFSET:.3f},{self.APPROACH_Z_OFFSET:.3f}) "
            f"hover=({tx:.3f},{ty:.3f},{hover_z:.3f}) descent={descent_distance*1000:.1f}mm"
        )
        approach_q = self._current_tcp_quat()

        # --- STEP 3: APPROACH & CLOSED-LOOP DESCENT ---
        print("🔓 Opening Gripper for Approach...")
        self.publish_state("MOVING")
        if not self._open_gripper_verified(): return False
        
        print(f"🚁 Hovering at {tx:.3f}, {ty:.3f}...")
        hover_velocity = max(0.05, min(float(self.HOVER_VELOCITY), 0.60))
        if not self.uf850.move_to_pose_exotica(tx, ty, hover_z, q_dict=approach_q, velocity=hover_velocity): return False
        self.wait_for_arm_settled()

        print("🗜️ Closing Gripper to 0 radians for search...")
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: 0.0},
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N
        ): return False
        self.wait_for_gripper(0.0)

        print("⬇️ EXOTica stepped tactile descent...")
        descent_start_z = self._tcp_z()
        contact_assumed_from_depth = False
        descent_ok = self.uf850.move_linear_z_with_effort_stop_exotica(
            descent_distance_m=descent_distance,
            step_m=self.DESCENT_STEP_M,
            threshold_nm=self.TORQUE_THRESHOLD,
            joint_index=2,
            rate_hz=self.DESCENT_RATE_HZ,
            command_alpha=0.35,
            max_joint_step_rad=0.015,
            q_dict=approach_q,
            target_x=tx,
            target_y=ty,
        )
        if not descent_ok:
            if self.ACCEPT_MAX_DEPTH_WITHOUT_CONTACT and self._descent_reached_expected_depth(
                descent_start_z,
                descent_distance,
            ):
                self.get_logger().warning(
                    "[pickup] Accepting max-depth descent as contact for pickup; "
                    "continuing with contact retract and grasp."
                )
                contact_assumed_from_depth = True
            else:
                self.get_logger().error("[pickup] Descent failed before reaching usable pickup depth.")
                self.uf850._hold_current_arm_position()
                return False
        if not descent_ok and not self.ACCEPT_MAX_DEPTH_WITHOUT_CONTACT:
            return False
        self.wait_for_arm_settled()

        if contact_assumed_from_depth:
            self.get_logger().warning(
                "[pickup] Skipping contact retract because no torque contact was confirmed; "
                "closing at max-depth pose."
            )
        else:
            print(f"⬆️ Retracting {self.CONTACT_RETRACT_M*1000:.0f}mm after contact...")
            if not self._retract_after_contact(): return False
            self.wait_for_arm_settled()

        # --- STEP 4: GRASP & RETRACT ---
        print(f"🗜️ Final Grasp at {self.GRIPPER_CLOSE_FORCE_N}N...")
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: math.radians(self.CLOSE_DEG)},
            gripper_force_n=self.GRIPPER_CLOSE_FORCE_N
        ): return False
        self.wait_for_gripper(self.CLOSE_DEG)
        self.publish_state("HOLDING")
        self.hold_status_pub.publish(Bool(data=True))

        print(f"⬆️ Final Retract {self.POST_GRASP_RETRACT_M*1000:.0f}mm...")
        if not self._verified_post_grasp_retract(): return False

        # --- STEP 5: DROP-OFF ---
        print("🗑️ Moving to Drop Pose...")
        drop_q = self._current_tcp_quat() or approach_q
        if not self.uf850.move_to_pose_exotica(
            self.DROP_POSE['x'],
            self.DROP_POSE['y'],
            self.DROP_POSE['z'],
            q_dict=drop_q,
            velocity=0.1,
        ): return False
        self.wait_for_arm_settled()

        print("🎉 Finalizing: Release & Home...")
        self.publish_state("IDLE")
        if not self._open_gripper_verified(): return False
        self.hold_status_pub.publish(Bool(data=False))
        
        if not self.uf850.move_to_joint_positions(self.UF_HOME_JOINTS, velocity=0.2): return False
        
        print("✅ [SUCCESS] Sequence Complete.")
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
