#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
import threading, math, time, os
from ament_index_python.packages import get_package_share_directory
from disassembly_skill.device_config import DeviceConfig
from disassembly_skill.motion_backend import MotionBackend

HOLD_STATE_FILE = '/tmp/disassembly_hold_state'

def read_hold_state():
    try:
        with open(HOLD_STATE_FILE, 'r') as f:
            return f.read().strip() == 'true'
    except Exception:
        return False

def write_hold_state(held):
    try:
        with open(HOLD_STATE_FILE, 'w') as f:
            f.write('true' if held else 'false')
    except Exception:
        pass

class FlipDropSkill(Node):
    def __init__(self, device_cfg=None):
        super().__init__('flip_drop_skill_node')
        
        # Hardware Backends
        self.uf850 = MotionBackend(self, "uf850_arm")
        self.gripper = MotionBackend(self, "rg6_gripper")
        
        # State Tracking
        self.is_holding_object = False
        self.hold_event = threading.Event()
        self.hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/object_hold_state/is_held', self.hold_status_callback, self.hold_qos)
        
        # Interfaces
        self.state_update_pub = self.create_publisher(String, '/robot_state/manip_arm/update', 10)
        self.hold_status_pub = self.create_publisher(Bool, '/object_hold_state/is_held', self.hold_qos)
        
        # Configuration
        self.PLANNING_FRAME = "base_link"
        self.ROBOT_EE_LINK = "rg6_tcp"
        self.JOINT_GRIPPER = "rg6_right_drive_joint"
        self.OPEN_DEG, self.CLOSE_DEG = -35.0, 35.0
        self.RETRACT_Z_HEIGHT = 0.15           
        self.INTERMEDIATE_POSE = {'x': 0.929872, 'y': -0.633943, 'z': 1.0977}
        self.TORQUE_THRESHOLD = 3.0            
        self.DESCENT_SPEED = 0.2              # Reduced for safety
        self.RETRACT_VELOCITY = 0.04
        self.RETRACT_STEP_M = 0.025
        self.DROP_TRANSFER_VELOCITY = 0.06
        self.DROP_TRANSFER_STEP_M = 0.050
        self.GRIPPER_CLOSE_FORCE_N = 100.0     # Synced with Flip Skill
        self.GRIPPER_OPEN_FORCE_N = 40.0
        self.ROTATION_DEG = 180.0
        self.WRIST_ROTATION_VELOCITY = 0.15
        self.INTERMEDIATE_ORIENTATION = None
        self.DROP_MIN_REACHABLE_Y = -0.54
        self.DROP_ACCEPT_Y_TOLERANCE = 0.06
        self.DROP_ACCEPT_X_TOLERANCE = 0.05
        self.DROP_ACCEPT_Z_TOLERANCE = 0.08
        self.PICKUP_DROP_Z_OFFSET_M = 0.0

        if device_cfg is not None:
            self._apply_flip_drop_config(device_cfg)

        self.get_logger().info("Flip-Drop Skill: Modernized Production Version.")

    def _apply_flip_drop_config(self, cfg):
        fd_steps = [s for s in cfg.disassembly_sequence if s.action == 'flip_drop']
        if not fd_steps:
            return
        p = fd_steps[0].parameters
        pickup_drop = None
        pickup_steps = [s for s in cfg.disassembly_sequence if s.action == 'pickup']
        if pickup_steps:
            pickup_params = pickup_steps[0].parameters
            if all(k in pickup_params for k in ('drop_x', 'drop_y', 'drop_z')):
                pickup_drop = {
                    'x': pickup_params['drop_x'],
                    'y': pickup_params['drop_y'],
                    'z': pickup_params['drop_z'],
                }
        self.RETRACT_Z_HEIGHT = p.get('retract_height_m', self.RETRACT_Z_HEIGHT)
        self.RETRACT_VELOCITY = p.get('retract_velocity', self.RETRACT_VELOCITY)
        self.RETRACT_STEP_M = p.get('retract_step_m', self.RETRACT_STEP_M)
        self.DROP_TRANSFER_VELOCITY = p.get('drop_transfer_velocity', self.DROP_TRANSFER_VELOCITY)
        self.DROP_TRANSFER_STEP_M = p.get('drop_transfer_step_m', self.DROP_TRANSFER_STEP_M)
        self.GRIPPER_CLOSE_FORCE_N = p.get('gripper_close_force_n', self.GRIPPER_CLOSE_FORCE_N)
        self.DROP_ACCEPT_Y_TOLERANCE = p.get('drop_accept_y_tolerance_m', self.DROP_ACCEPT_Y_TOLERANCE)
        self.DROP_ACCEPT_X_TOLERANCE = p.get('drop_accept_x_tolerance_m', self.DROP_ACCEPT_X_TOLERANCE)
        self.DROP_ACCEPT_Z_TOLERANCE = p.get('drop_accept_z_tolerance_m', self.DROP_ACCEPT_Z_TOLERANCE)
        self.PICKUP_DROP_Z_OFFSET_M = p.get('pickup_drop_z_offset_m', self.PICKUP_DROP_Z_OFFSET_M)
        use_pickup_drop_xyz = bool(p.get('use_pickup_drop_xyz', True))
        if use_pickup_drop_xyz and pickup_drop is not None:
            ix = pickup_drop['x']
            iy = pickup_drop['y']
            iz = pickup_drop['z'] + self.PICKUP_DROP_Z_OFFSET_M
        else:
            ix = p.get('intermediate_x', self.INTERMEDIATE_POSE['x'])
            iy = p.get('intermediate_y', self.INTERMEDIATE_POSE['y'])
            iz = p.get('intermediate_z', self.INTERMEDIATE_POSE['z'])
        self.INTERMEDIATE_POSE = {'x': ix, 'y': iy, 'z': iz}
        self.ROTATION_DEG = p.get('rotation_deg', self.ROTATION_DEG)
        self.WRIST_ROTATION_VELOCITY = p.get('wrist_rotation_velocity', self.WRIST_ROTATION_VELOCITY)
        self.DROP_MIN_REACHABLE_Y = p.get('drop_min_reachable_y', self.DROP_MIN_REACHABLE_Y)
        q_keys = ('intermediate_qx', 'intermediate_qy', 'intermediate_qz', 'intermediate_qw')
        if all(k in p for k in q_keys):
            self.INTERMEDIATE_ORIENTATION = {
                'qx': p['intermediate_qx'],
                'qy': p['intermediate_qy'],
                'qz': p['intermediate_qz'],
                'qw': p['intermediate_qw'],
            }
        self.get_logger().info(
            f"[flip_drop] Drop XYZ=({self.INTERMEDIATE_POSE['x']:.3f}, "
            f"{self.INTERMEDIATE_POSE['y']:.3f}, {self.INTERMEDIATE_POSE['z']:.3f}) "
            f"{'from pickup drop config' if use_pickup_drop_xyz and pickup_drop is not None else 'from flip_drop config'}; "
            "orientation unchanged."
        )

    def publish_state(self, s): self.state_update_pub.publish(String(data=s))
    def publish_hold_status(self, h):
        self.hold_status_pub.publish(Bool(data=h))
        write_hold_state(h)
    def hold_status_callback(self, msg):
        self.is_holding_object = msg.data
        if self.is_holding_object: self.hold_event.set()

    def _current_uf_joint_positions(self):
        return {n: p for n, p in self.uf850.current_joint_positions.items() if n.startswith("uf850_")}

    @staticmethod
    def _wrap_to_pi(angle_rad):
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def _quat_angle_error(q_target, q_current):
        target = (
            float(q_target["qx"]),
            float(q_target["qy"]),
            float(q_target["qz"]),
            float(q_target["qw"]),
        )
        current = (
            float(q_current["qx"]),
            float(q_current["qy"]),
            float(q_current["qz"]),
            float(q_current["qw"]),
        )
        target_norm = math.sqrt(sum(v * v for v in target))
        current_norm = math.sqrt(sum(v * v for v in current))
        if target_norm <= 1e-9 or current_norm <= 1e-9:
            return math.inf
        dot = abs(sum((target[i] / target_norm) * (current[i] / current_norm) for i in range(4)))
        dot = max(-1.0, min(1.0, dot))
        return 2.0 * math.acos(dot)

    def _wrist_flip_target(self, current_j6):
        rotation_rad = math.radians(abs(float(self.ROTATION_DEG)))
        candidates = [
            self._wrap_to_pi(current_j6 + rotation_rad),
            self._wrap_to_pi(current_j6 - rotation_rad),
        ]
        target_j6 = min(candidates, key=lambda v: abs(v))
        applied_delta = self._wrap_to_pi(target_j6 - current_j6)
        print(
            f"🔄 Wrist flip: j6 {math.degrees(current_j6):.1f}° -> "
            f"{math.degrees(target_j6):.1f}° "
            f"(delta {math.degrees(applied_delta):.1f}°, configured {self.ROTATION_DEG:.1f}°)"
        )
        return target_j6

    def _planned_retract_z(self, distance_m):
        self.uf850.stop_servo(timeout_sec=2.0)
        try:
            tf = self.uf850.tf_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time())
            sx, sy, sz = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
            q = {
                'qx': tf.transform.rotation.x,
                'qy': tf.transform.rotation.y,
                'qz': tf.transform.rotation.z,
                'qw': tf.transform.rotation.w,
            }
        except Exception as e:
            self.get_logger().error(f"TF Error before retract: {e}")
            return None

        distance_m = abs(float(distance_m))
        target_z = sz + distance_m
        velocity = max(0.02, min(float(self.RETRACT_VELOCITY), 0.30))
        print(
            f"🚀 Continuous planned Z retract: current {self.ROBOT_EE_LINK}=({sx:.4f},{sy:.4f},{sz:.4f}); "
            f"target Z={target_z:.4f} (+{distance_m*1000:.1f}mm) velocity={velocity:.2f}"
        )
        ok = self.uf850.move_to_pose_exotica(sx, sy, target_z, q, velocity=velocity)
        if not ok:
            print("[RETRACT] EXOTica retract failed; trying Cartesian fallback.")
            ok = self.uf850.move_cartesian_to_pose(sx, sy, target_z, q, velocity=velocity)
        if not ok:
            print("[RETRACT] Planned Z retract failed.")
            return None

        self.wait_for_arm_settled()
        try:
            end_tf = self.uf850.tf_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time())
            ez = end_tf.transform.translation.z
            actual_dz = ez - sz
            print(f"[RETRACT] actual dz={actual_dz*1000:.1f}mm target_dz={distance_m*1000:.1f}mm")
            min_expected_dz = min(distance_m * 0.50, distance_m - 0.010)
            min_expected_dz = max(0.010, min_expected_dz)
            if actual_dz < min_expected_dz:
                self.get_logger().error(
                    f"Unsafe retract result: expected +Z motion but actual dz={actual_dz*1000:.1f}mm "
                    f"(minimum accepted {min_expected_dz*1000:.1f}mm). Aborting flip-drop."
                )
                self.uf850._hold_current_arm_position()
                return None
        except Exception as e:
            self.get_logger().warning(f"Could not read final TF after retract: {e}")
        return sx, sy, sz, q

    def _drop_orientation(self, fallback_q):
        return self.INTERMEDIATE_ORIENTATION if self.INTERMEDIATE_ORIENTATION is not None else fallback_q

    def _current_tcp_pose(self):
        try:
            tf = self.uf850.tf_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time())
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
                {
                    'qx': tf.transform.rotation.x,
                    'qy': tf.transform.rotation.y,
                    'qz': tf.transform.rotation.z,
                    'qw': tf.transform.rotation.w,
                },
            )
        except Exception as e:
            self.get_logger().error(f"TF Error reading current TCP pose: {e}")
            return None

    def _accept_current_drop_pose(self, target_x, target_y, target_z):
        current = self._current_tcp_pose()
        if current is None:
            return False
        cx, cy, cz, _ = current
        x_err = abs(target_x - cx)
        y_err = abs(target_y - cy)
        z_err = abs(target_z - cz)
        y_on_drop_side = cy <= target_y + self.DROP_ACCEPT_Y_TOLERANCE
        acceptable = (
            x_err <= self.DROP_ACCEPT_X_TOLERANCE
            and y_err <= self.DROP_ACCEPT_Y_TOLERANCE
            and z_err <= self.DROP_ACCEPT_Z_TOLERANCE
            and y_on_drop_side
        )
        print(
            f"[DROP] Reachable-pose check: tcp=({cx:.4f},{cy:.4f},{cz:.4f}) "
            f"target=({target_x:.4f},{target_y:.4f},{target_z:.4f}) "
            f"err=({x_err*1000:.1f},{y_err*1000:.1f},{z_err*1000:.1f})mm "
            f"acceptable={acceptable}"
        )
        return acceptable

    def _cartesian_chunked_transfer(self, target_x, target_y, target_z, q_keep):
        """Move to drop zone in small Cartesian chunks to avoid IK branch jumps."""
        current = self._current_tcp_pose()
        if current is None:
            return False
        cx, cy, cz, _ = current

        print(
            f"[DROP] Chunked Cartesian transfer from ({cx:.4f}, {cy:.4f}, {cz:.4f}) "
            f"to ({target_x:.4f}, {target_y:.4f}, {target_z:.4f})"
        )

        # The difficult part is the long Y sweep.  Keep X/Z fixed first, then
        # settle X and Z after reaching the drop-side workspace.
        start_y = cy
        y_delta = target_y - start_y
        step_m = max(0.025, min(float(self.DROP_TRANSFER_STEP_M), 0.080))
        velocity = max(0.03, min(float(self.DROP_TRANSFER_VELOCITY), 0.30))
        n_y_steps = max(3, int(math.ceil(abs(y_delta) / step_m)))
        for step in range(1, n_y_steps + 1):
            yi = start_y + y_delta * (step / n_y_steps)
            print(f"[DROP]   Y step {step}/{n_y_steps}: y={yi:.4f}")
            if not self.uf850.move_cartesian_to_pose(cx, yi, cz, q_keep, velocity=velocity):
                print(f"[DROP]   Y step {step}/{n_y_steps} failed.")
                if self._accept_current_drop_pose(target_x, target_y, target_z):
                    print("[DROP] Current reachable pose is acceptable for flip-drop; not pushing into joint limit.")
                    return True
                return False
            self.wait_for_arm_settled(timeout=1.2)
            current = self._current_tcp_pose()
            if current is None:
                return False
            cx, cy, cz, _ = current

        # Now move to target X at the same safe Z.
        if abs(cx - target_x) > 0.004:
            print(f"[DROP]   X settle: x={target_x:.4f}")
            if not self.uf850.move_cartesian_to_pose(target_x, cy, cz, q_keep, velocity=velocity):
                print("[DROP]   X settle failed.")
                if self._accept_current_drop_pose(target_x, target_y, target_z):
                    print("[DROP] Current reachable pose is acceptable for flip-drop; not pushing into joint limit.")
                    return True
                return False
            self.wait_for_arm_settled(timeout=1.2)
            current = self._current_tcp_pose()
            if current is None:
                return False
            cx, cy, cz, _ = current

        # Finally adjust Z if the configured drop Z is reachable from this branch.
        if abs(cz - target_z) > 0.004:
            print(f"[DROP]   Z settle: z={target_z:.4f}")
            if not self.uf850.move_cartesian_to_pose(target_x, target_y, target_z, q_keep, velocity=min(velocity, 0.05)):
                if self._accept_current_drop_pose(target_x, target_y, target_z):
                    print("[DROP]   Z settle failed; current safe Z is acceptable for wrist drop.")
                    return True
                print("[DROP]   Z settle failed and current pose is outside drop tolerance.")
                return False
            self.wait_for_arm_settled(timeout=1.2)

        return True

    def _servo_track_position(self, target_x, target_y, target_z, timeout_s=16.0):
        """Teleop-style closed-loop TCP position tracking using MoveIt Servo."""
        if not self.uf850.start_servo(timeout_sec=8.0):
            print("[DROP/SERVO] Could not start MoveIt Servo for final approach.")
            return False

        print(
            f"[DROP/SERVO] Tracking TCP to ({target_x:.4f}, {target_y:.4f}, {target_z:.4f}) "
            "with proportional base-frame velocity."
        )
        rate_hz = 25.0
        dt = 1.0 / rate_hz
        gain = 0.85
        max_xy_speed = 0.045
        max_z_speed = 0.025
        pos_tol = 0.008
        start_t = time.time()
        last_log_t = 0.0

        try:
            while rclpy.ok() and (time.time() - start_t) < timeout_s:
                current = self._current_tcp_pose()
                if current is None:
                    return False
                cx, cy, cz, _ = current
                ex = target_x - cx
                ey = target_y - cy
                ez = target_z - cz
                err = math.sqrt(ex * ex + ey * ey + ez * ez)
                if err <= pos_tol:
                    self.uf850._publish_zero_twist()
                    print(
                        f"[DROP/SERVO] Reached target: err={err*1000:.1f}mm "
                        f"tcp=({cx:.4f},{cy:.4f},{cz:.4f})"
                    )
                    return True

                vx_cmd = max(-max_xy_speed, min(max_xy_speed, gain * ex))
                vy_cmd = max(-max_xy_speed, min(max_xy_speed, gain * ey))
                vz_cmd = max(-max_z_speed, min(max_z_speed, gain * ez))

                # UF850 Servo is configured around uf850_link6 while this skill
                # tracks rg6_tcp in base_link.  On hardware, the lateral Y
                # response is inverted relative to the base-frame TCP error.
                vx = vx_cmd
                vy = -vy_cmd
                vz = vz_cmd

                if not self.uf850.publish_servo_velocity(vx, vy, vz):
                    print("[DROP/SERVO] Servo velocity publish failed.")
                    return False

                now = time.time()
                if now - last_log_t > 0.5:
                    print(
                        f"[DROP/SERVO] err=({ex*1000:.1f},{ey*1000:.1f},{ez*1000:.1f})mm "
                        f"cmd=({vx*1000:.1f},{vy*1000:.1f},{vz*1000:.1f})mm/s"
                    )
                    last_log_t = now
                time.sleep(dt)

            self.uf850._publish_zero_twist()
            current = self._current_tcp_pose()
            if current is not None:
                cx, cy, cz, _ = current
                err = math.sqrt((target_x - cx) ** 2 + (target_y - cy) ** 2 + (target_z - cz) ** 2)
                print(f"[DROP/SERVO] Timed out with remaining error {err*1000:.1f}mm.")
            return False
        finally:
            self.uf850._publish_zero_twist()
            self.uf850.stop_servo(timeout_sec=3.0)

    def _exotica_track_position(
        self,
        target_x,
        target_y,
        target_z,
        q_keep,
        timeout_s=14.0,
        require_orientation=False,
    ):
        """Arm-teleop style realtime EXOTica tracking to a TCP pose."""
        self.uf850.stop_servo(timeout_sec=2.0)
        try:
            roll, pitch, yaw = self.uf850._quaternion_to_rpy(
                float(q_keep["qx"]),
                float(q_keep["qy"]),
                float(q_keep["qz"]),
                float(q_keep["qw"]),
            )
        except Exception:
            roll, pitch, yaw = (math.pi, 0.0, 0.0)

        pos_tol = 0.008
        orientation_tol = math.radians(5.0)
        last_log_t = [0.0]

        print(
            f"[DROP/EXO-TRACK] Tracking TCP to ({target_x:.4f}, {target_y:.4f}, {target_z:.4f}) "
            + (
                "with realtime EXOTica full-pose streaming."
                if require_orientation
                else "with realtime EXOTica IK streaming."
            )
        )

        def _target_fn():
            current = self._current_tcp_pose()
            if current is None:
                return None
            cx, cy, cz, cq = current
            ex = target_x - cx
            ey = target_y - cy
            ez = target_z - cz
            err = math.sqrt(ex * ex + ey * ey + ez * ez)
            q_err = self._quat_angle_error(q_keep, cq) if require_orientation else 0.0
            now = time.time()
            if now - last_log_t[0] > 0.5:
                if require_orientation:
                    print(
                        f"[DROP/EXO-TRACK] err=({ex*1000:.1f},{ey*1000:.1f},{ez*1000:.1f})mm "
                        f"q_err={math.degrees(q_err):.1f}°"
                    )
                else:
                    print(f"[DROP/EXO-TRACK] err=({ex*1000:.1f},{ey*1000:.1f},{ez*1000:.1f})mm")
                last_log_t[0] = now
            if err <= pos_tol and (not require_orientation or q_err <= orientation_tol):
                print(
                    f"[DROP/EXO-TRACK] Reached target: err={err*1000:.1f}mm "
                    f"q_err={math.degrees(q_err):.1f}° "
                    f"tcp=({cx:.4f},{cy:.4f},{cz:.4f})"
                )
                return None
            return (target_x, target_y, target_z, roll, pitch, yaw)

        result = self.uf850.move_cartesian_realtime_exotica(
            _target_fn,
            rate_hz=50.0,
            max_step_m=0.004,
            joint_smooth_alpha=0.75,
            timeout_s=timeout_s,
            max_joint_delta_rad=0.18,
        )
        ok = result == "DONE"
        if not ok:
            print(f"[DROP/EXO-TRACK] Failed with result={result}.")
        return ok

    def _move_to_flip_drop_pose(self, q_drop, q_fallback, min_safe_z):
        configured_z = float(self.INTERMEDIATE_POSE['z'])
        target_x = self.INTERMEDIATE_POSE['x']
        configured_y = self.INTERMEDIATE_POSE['y']
        target_y = max(float(configured_y), float(self.DROP_MIN_REACHABLE_Y))
        if target_y != configured_y:
            print(
                f"[DROP] Configured flip-drop Y={configured_y:.4f} is outside the reliable "
                f"held-orientation workspace; using reachable Y={target_y:.4f}."
            )

        current = self._current_tcp_pose()
        if current is None:
            print("[DROP] Cannot read current TCP pose for flip-drop transfer.")
            return False

        _, _, current_z, _ = current
        if current_z < min_safe_z - 0.01:
            print(
                f"[DROP] Current Z={current_z:.4f} is below safe travel Z={min_safe_z:.4f}; "
                "retracting before lateral transfer."
            )
            if not self.uf850.move_to_pose_exotica(
                current[0],
                current[1],
                min_safe_z,
                q_fallback,
                velocity=max(0.02, min(float(self.RETRACT_VELOCITY), 0.05)),
            ):
                return False

        print(
            f"[DROP] Trying continuous low-speed EXOTica transfer to flip-drop zone "
            f"({target_x:.4f}, {target_y:.4f}, {configured_z:.4f}) using held TCP orientation."
        )
        if self.uf850.move_to_pose_exotica(
            target_x,
            target_y,
            configured_z,
            q_fallback,
            velocity=max(0.03, min(float(self.DROP_TRANSFER_VELOCITY), 0.08)),
        ):
            self._accept_current_drop_pose(target_x, target_y, configured_z)
            return True

        print("[DROP] Continuous transfer failed; falling back to chunked Cartesian transfer.")
        if self._cartesian_chunked_transfer(
            target_x,
            target_y,
            configured_z,
            q_fallback,
        ):
            self._accept_current_drop_pose(target_x, target_y, configured_z)
            return True

        if self._accept_current_drop_pose(target_x, target_y, configured_z):
            print("[DROP] Current reachable pose accepted after chunked transfer.")
            return True

        print("[DROP] Chunked transfer failed outside tolerance; skipping Servo near joint limit.")
        return False

    def wait_for_arm_settled(self, timeout=10.0):
        start_t = time.time(); settle_timer = 0.0; last_pos = {}
        NOISE_TOLERANCE = 0.006 
        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = self._current_uf_joint_positions()
            if not curr: time.sleep(0.1); continue
            if last_pos:
                max_delta = max([abs(curr[n] - last_pos[n]) for n in curr if n in last_pos], default=0.0)
                if max_delta <= NOISE_TOLERANCE:
                    settle_timer += 0.1
                    if settle_timer >= 0.4: return True
                else: settle_timer = 0.0 
            last_pos = curr; time.sleep(0.1)
        return False

    def wait_for_gripper(self, target_deg, timeout=8.0):
        target_rad = math.radians(target_deg)
        start_t = time.time(); last_pos = 999.0; stall_timer = 0.0; is_closing = target_deg < 0
        position_tolerance = 0.08
        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = self.gripper.current_joint_positions.get(self.JOINT_GRIPPER, 999)
            if curr == 999: time.sleep(0.1); continue
            if abs(curr - target_rad) < position_tolerance: return True
            if abs(curr - last_pos) < 0.002:
                stall_timer += 0.1
                if stall_timer >= 0.8:
                    if is_closing:
                        self.get_logger().info(f"✅ Grasp confirmed at {curr:.3f} rad.")
                        return True
                    else:
                        self.get_logger().info(
                            f"Gripper open accepted at mechanical limit {curr:.3f} rad "
                            f"(target {target_rad:.3f} rad)."
                        )
                        return True
            else: stall_timer = 0.0
            last_pos = curr; time.sleep(0.1)
        return False

    def _return_to_start_height(self, sx, sy, sz, q_start):
        print(
            f"⬇️ STEP 6: Descending back to pre-retract hold height "
            f"({sx:.4f}, {sy:.4f}, {sz:.4f})..."
        )
        ok = self.uf850.move_to_pose_exotica(
            sx,
            sy,
            sz,
            q_start,
            velocity=max(0.03, min(float(self.DROP_TRANSFER_VELOCITY), 0.06)),
        )
        if not ok:
            print("[RETURN] Planned descent to pre-retract height failed; trying Cartesian fallback.")
            ok = self.uf850.move_cartesian_to_pose(
                sx,
                sy,
                sz,
                q_start,
                velocity=max(0.03, min(float(self.DROP_TRANSFER_VELOCITY), 0.05)),
            )
        if not ok:
            self.uf850._hold_current_arm_position()
            return False

        self.wait_for_arm_settled()
        current = self._current_tcp_pose()
        if current is None:
            return False
        _cx, _cy, cz, _q = current
        z_err = abs(float(cz) - float(sz))
        print(f"[RETURN] final_z={cz:.4f} target_z={sz:.4f} err={z_err*1000:.1f}mm")
        if z_err > 0.015:
            self.get_logger().error(
                f"Flip-drop return descent did not reach start height: error={z_err*1000:.1f}mm"
            )
            return False
        return True

    def execute_flip_drop(self, interactive=False):
        """Standard Flip-Drop sequence with fixed 180-degree logic."""
        print(f"\n🛠️ [START] Flip-Drop Sequential Task")

        try:
            start_tf = self.uf850.tf_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time())
            sx, sy, sz = start_tf.transform.translation.x, start_tf.transform.translation.y, start_tf.transform.translation.z
            q_start = {'qx': start_tf.transform.rotation.x, 'qy': start_tf.transform.rotation.y,
                       'qz': start_tf.transform.rotation.z, 'qw': start_tf.transform.rotation.w}
        except Exception as e:
            self.get_logger().error(f"TF Error: {e}")
            return False

        # 1. RETRACT
        print("🚀 STEP 1: Vertical planned retract...")
        self.publish_state("FLIPPING")
        retract_info = self._planned_retract_z(self.RETRACT_Z_HEIGHT)
        if retract_info is None:
            return False

        q_drop = self._drop_orientation(q_start)

        # 2. TRAVEL
        print(
            f"🚛 STEP 2: Traveling to Flip-Drop pose "
            f"({self.INTERMEDIATE_POSE['x']:.4f}, {self.INTERMEDIATE_POSE['y']:.4f}, "
            f"{self.INTERMEDIATE_POSE['z']:.4f})..."
        )
        if not self._move_to_flip_drop_pose(
            q_drop=q_drop,
            q_fallback=q_start,
            min_safe_z=sz + self.RETRACT_Z_HEIGHT,
        ):
            return False
        self.wait_for_arm_settled()

        # 3. FIXED 180° FLIP
        print("🔄 STEP 3: Executing Single 180° Flip...")
        joints = self._current_uf_joint_positions()
        orig_j6 = joints.get("uf850_joint6", 0.0)
        joints["uf850_joint6"] = self._wrist_flip_target(orig_j6)
        
        if not self.uf850.move_to_joint_positions(joints, velocity=self.WRIST_ROTATION_VELOCITY):
            return False
        self.wait_for_arm_settled()
        
        # 4. FLIP BACK
        print("🔄 STEP 4: Resetting Orientation...")
        joints["uf850_joint6"] = orig_j6
        if not self.uf850.move_to_joint_positions(joints, velocity=self.WRIST_ROTATION_VELOCITY):
            return False
        self.wait_for_arm_settled()

        # 5. RETURN TO SAFE PICK XY HEIGHT
        print("🏠 STEP 5: Returning to Pick XY at safe retract height...")
        if not self.uf850.move_to_pose_exotica(
            sx,
            sy,
            sz + self.RETRACT_Z_HEIGHT,
            q_start,
            velocity=0.14,
        ):
            return False
        self.wait_for_arm_settled()

        # 6. DESCEND BACK TO PRE-RETRACT HOLD HEIGHT
        if not self._return_to_start_height(sx, sy, sz, q_start):
            return False

        print("✅ [SUCCESS] Flip-Drop Complete. Chassis remains held.")
        self.publish_state("HOLDING")
        self.publish_hold_status(True)
        return True

def main(args=None):
    rclpy.init(args=args)
    try:
        cfg_path = os.path.join(
            get_package_share_directory('disassembly_skill'),
            'config', 'device_configs', 'hdd.yaml',
        )
        device_cfg = DeviceConfig.load(cfg_path)
    except Exception as exc:
        print(f"[WARN] Could not load device config: {exc}. Using defaults.")
        device_cfg = None
    node = FlipDropSkill(device_cfg=device_cfg)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Run spin in a separate thread so the main loop can control execution
    thread = threading.Thread(target=executor.spin, daemon=False)
    thread.start()

    # Wait for TF buffer to populate before doing anything
    time.sleep(2.0)

    # --- SINGLE RUN LOGIC ---
    try:
        print("🕒 Waiting for arm to hold object before starting...")
        # Check file-based hold state as fallback (survives process death)
        if not node.is_holding_object and read_hold_state():
            print("📦 Hold state detected from file (previous skill). Proceeding...")
            node.is_holding_object = True
            node.hold_event.set()

        # Wait until the manager or a previous skill sets the HOLD status
        while rclpy.ok():
            if node.is_holding_object:
                print("📦 Object Hold detected. Starting Flip-Drop...")
                if node.execute_flip_drop(interactive=False):
                    print("🏁 Skill finished successfully. Shutting down.")
                else:
                    print("⚠️ Skill exited with errors. Shutting down.")
                break # 🎯 EXIT THE LOOP AFTER ONE RUN
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
