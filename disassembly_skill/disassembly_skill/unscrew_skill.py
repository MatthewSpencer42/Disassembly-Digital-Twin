#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Pose
import json, time, threading, copy, math
from disassembly_skill.motion_backend import MotionBackend

class UnscrewSkill(Node):
    def __init__(self, device_cfg=None):
        super().__init__('unscrew_skill_node')
        
        # --- ⚙️ CONFIGURATION SECTION (CORE PARAMETERS) ---
        # Physical Geometry
        self.CONFIG = {
            "TOOL_LENGTH": 0.240,           # m (Screwdriver length)
            "HOVER_DISTANCE": 0.015,        # m (15mm above screw tip)
            "TRANSIT_LIFT": 0.030,          # m (Safe travel height)
            "REACH_LIMIT": 0.680,           # m (xArm reach radius)
            "MM_PER_PIX": 0.000130,         # m/px (Vision calibration)
            "COARSE_VELOCITY": 0.20,        # trajectory scaling for coarse pose moves
            "COARSE_POSITION_TOLERANCE": 0.010,  # m, abort descent/search if hover TCP is off target
            "COARSE_PLANNER": "moveit",     # MoveIt IK is the verified stable coarse path on xArm5
            "COARSE_ONLY_DEBUG": False,      # temporary: verify all screw hover poses before unscrewing
            "ALIGN_ONLY_DEBUG": True,        # temporary: XY visual servo/search only, no Z descent
            "SCREW_CAMERA_Z_MIN": 0.45,      # m, reject bad depth hits from arm/occluders
            "SCREW_CAMERA_Z_MAX": 0.65,
            "SCREW_WORLD_Z_MIN": 0.90,       # m, HDD screw surface sanity range in base_link
            "SCREW_WORLD_Z_MAX": 1.05,
            "PRE_RETRACT": 0.0,             # disabled: bad xArm mode can turn this into lateral motion
            
            # Descent & Alignment
            "XY_SPEED_ALIGN": 0.004,        # m/s
            "Z_SPEED_DESCENT": 0.004,       # m/s
            "ALIGN_TOLERANCE_PX": 4.0,      # pixels
            "FORCE_THRESHOLD": 5.0,         # N (Contact detection)

            # Spiral Search
            "SPIRAL_TIMEOUT": 15.0,         # s (Vision fallback)
            "SPIRAL_START_DIST_MM": 5.0,   # mm (First side distance)
            "SPIRAL_GAP_MM": 5.0,           # mm (Increase per 2 sides)
            "SPIRAL_SPEED": 0.001,          # m/s
            "SEARCH_MAX_STEP": 0.001,       # m, clamps realtime target motion
            "SEARCH_JOINT_SMOOTH_ALPHA": 0.35,
            "VISUAL_SERVO_GAIN": 0.8,
            "VISUAL_SERVO_MAX_RADIUS": 0.025,  # m, bound search around verified coarse hover
            "ALIGN_MOVE_STEP": 0.0005,      # m, max target-position change per control tick
            "ALIGN_VERIFY_TOLERANCE": 0.015,  # m, abort align/search if TCP does not follow
            "ALIGN_ERROR_FILTER_ALPHA": 0.35,
            "ALIGN_TARGET_FILTER_ALPHA": 0.25,
            "ALIGN_STABLE_CYCLES": 2,
            "ALIGN_ONLY_TIMEOUT": 90.0,
            "ALIGN_FULL_SPEED_ERROR_PX": 150.0,
            "ALIGN_MIN_SPEED": 0.0006,
            "ALIGN_Z_LOCK_GAIN": 1.2,
            "ALIGN_Z_LOCK_SPEED": 0.003,
            "ALIGN_TARGET_WAIT_S": 3.0,
            "ALIGN_TARGET_LOST_GRACE_S": 0.6,
            "ALIGN_RATE_HZ": 15.0,
            "ALIGN_JOINT_ALPHA": 0.35,
            "ALIGN_MAX_JOINT_STEP_RAD": 0.035,
            "ALIGN_MAX_JOINT_VELOCITY_RAD_S": 0.6,
            
            # Extraction
            "EXTRACTION_MAX_TIME": 20.0,    # s
            "EXTRACTION_COMPLIANCE_K": 0.015, # Velocity gain per Newton (increased for responsiveness)
            "EXTRACTION_STABLE_TIME": 3.0,  # s
            "EXTRACTION_Z_SPEED_CAP": 0.03, # m/s max compliant lift speed
            "POST_GRASP_RETRACT": 0.015,     # m (15mm as requested)
            "RETRACT_SPEED": 0.05,          # m/s
            
            # TF Frames
            "WORLD_FRAME": "base_link",
            "XARM_BASE_FRAME": 'xarm5_base_link',
            "CAMERA_FRAME": 'camera_color_optical_frame'
        }
        
        if device_cfg is not None:
            self._apply_unscrew_config(device_cfg)

        # --- Motion Backend ---
        self.moveit_backend = MotionBackend(self, "xarm5_arm_no_slide")
        
        # --- ROS 2 Interfaces ---
        self.vision_sub = self.create_subscription(String, '/vision/agent_state', self.vision_callback, 10)
        self.bin_sub = self.create_subscription(String, '/vision/bin_coordinates', self.bin_callback, 10)
        self.state_pub = self.create_publisher(String, '/robot_state/tool_arm/update', 10)
        self.tool_pub = self.create_publisher(Int8, '/tool_cmd', 10)

        # Thread Safety & State
        self.data_lock = threading.Lock()
        self.latest_targets = []
        self.local_view = {}
        self.cached_bin1_xyz = None  
        
        self.get_logger().info("🚀 Refactored Unscrew Skill Active (Compliance & Safety Updated).")

    def _apply_unscrew_config(self, cfg):
        unscrew_steps = [s for s in cfg.disassembly_sequence if s.action == 'unscrew']
        if not unscrew_steps:
            return
        p = unscrew_steps[0].parameters
        if 'force_threshold_n' in p:
            self.CONFIG['FORCE_THRESHOLD'] = p['force_threshold_n']
        if 'align_tolerance_px' in p:
            self.CONFIG['ALIGN_TOLERANCE_PX'] = p['align_tolerance_px']
        if 'spiral_timeout_s' in p:
            self.CONFIG['SPIRAL_TIMEOUT'] = p['spiral_timeout_s']
        key_map = {
            'hover_distance_m': 'HOVER_DISTANCE',
            'transit_lift_m': 'TRANSIT_LIFT',
            'reach_limit_m': 'REACH_LIMIT',
            'mm_per_px': 'MM_PER_PIX',
            'coarse_velocity': 'COARSE_VELOCITY',
            'coarse_position_tolerance_m': 'COARSE_POSITION_TOLERANCE',
            'coarse_only_debug': 'COARSE_ONLY_DEBUG',
            'align_only_debug': 'ALIGN_ONLY_DEBUG',
            'screw_camera_z_min_m': 'SCREW_CAMERA_Z_MIN',
            'screw_camera_z_max_m': 'SCREW_CAMERA_Z_MAX',
            'screw_world_z_min_m': 'SCREW_WORLD_Z_MIN',
            'screw_world_z_max_m': 'SCREW_WORLD_Z_MAX',
            'pre_retract_m': 'PRE_RETRACT',
            'xy_speed_align_mps': 'XY_SPEED_ALIGN',
            'z_speed_descent_mps': 'Z_SPEED_DESCENT',
            'spiral_start_dist_mm': 'SPIRAL_START_DIST_MM',
            'spiral_gap_mm': 'SPIRAL_GAP_MM',
            'spiral_speed_mps': 'SPIRAL_SPEED',
            'search_max_step_m': 'SEARCH_MAX_STEP',
            'search_joint_smooth_alpha': 'SEARCH_JOINT_SMOOTH_ALPHA',
            'visual_servo_gain': 'VISUAL_SERVO_GAIN',
            'visual_servo_max_radius_m': 'VISUAL_SERVO_MAX_RADIUS',
            'align_move_step_m': 'ALIGN_MOVE_STEP',
            'align_verify_tolerance_m': 'ALIGN_VERIFY_TOLERANCE',
            'align_error_filter_alpha': 'ALIGN_ERROR_FILTER_ALPHA',
            'align_target_filter_alpha': 'ALIGN_TARGET_FILTER_ALPHA',
            'align_stable_cycles': 'ALIGN_STABLE_CYCLES',
            'align_only_timeout_s': 'ALIGN_ONLY_TIMEOUT',
            'align_full_speed_error_px': 'ALIGN_FULL_SPEED_ERROR_PX',
            'align_min_speed_mps': 'ALIGN_MIN_SPEED',
            'align_z_lock_gain': 'ALIGN_Z_LOCK_GAIN',
            'align_z_lock_speed_mps': 'ALIGN_Z_LOCK_SPEED',
            'align_target_wait_s': 'ALIGN_TARGET_WAIT_S',
            'align_target_lost_grace_s': 'ALIGN_TARGET_LOST_GRACE_S',
            'align_rate_hz': 'ALIGN_RATE_HZ',
            'align_joint_alpha': 'ALIGN_JOINT_ALPHA',
            'align_max_joint_step_rad': 'ALIGN_MAX_JOINT_STEP_RAD',
            'align_max_joint_velocity_rad_s': 'ALIGN_MAX_JOINT_VELOCITY_RAD_S',
            'retract_speed_mps': 'RETRACT_SPEED',
        }
        for param_name, config_name in key_map.items():
            if param_name in p:
                self.CONFIG[config_name] = p[param_name]
        if 'coarse_planner' in p:
            self.CONFIG['COARSE_PLANNER'] = str(p['coarse_planner']).lower()
        self.get_logger().info("Unscrew config applied from device config.")

    # =========================================================================
    # CALLBACKS & HELPERS
    # =========================================================================

    def vision_callback(self, msg):
        try:
            raw_data = msg.data.strip().strip("'").strip('"')
            data = json.loads(raw_data)
            with self.data_lock:
                self.latest_targets = [obj for obj in data.get("global_view", {}).get("objects", [])
                                     if "screw" in obj.get("label", "").lower()]
                self.local_view = data
        except Exception: pass

    def bin_callback(self, msg):
        try:
            raw_data = msg.data.strip().strip("'").strip('"')
            data = json.loads(raw_data)
            if "bin_1" in data and "xyz" in data["bin_1"]:
                with self.data_lock:
                    self.cached_bin1_xyz = data["bin_1"]["xyz"]
        except Exception: pass

    def wait_for_arm_settled(self, timeout=20.0):
        """Dynamically monitors joint states to ensures precision moves."""
        start_t = time.time()
        settle_timer = 0.0
        last_positions = {}
        NOISE_TOLERANCE = 0.006 

        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr_positions = self.moveit_backend.current_joint_positions.copy()
            if not curr_positions:
                time.sleep(0.1); continue
                
            if last_positions:
                max_delta = 0.0
                for j_name, j_pos in curr_positions.items():
                    if j_name in last_positions:
                        delta = abs(j_pos - last_positions[j_name])
                        if delta > max_delta: max_delta = delta
                            
                if max_delta <= NOISE_TOLERANCE:
                    settle_timer += 0.1
                    if settle_timer >= 0.4: return True
                else: settle_timer = 0.0 
                    
            last_positions = curr_positions
            time.sleep(0.1)
        return False

    def _screwdriver_tcp_error(self, target_xyz):
        """Return physical TCP error against a commanded base_link target."""
        tx, ty, tz = (float(v) for v in target_xyz)
        try:
            tf_check = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], "screwdriver_tcp", rclpy.time.Time()
            )
        except Exception as e:
            print(f"[DIAG] TF lookup failed: {e}")
            return None

        ax = tf_check.transform.translation.x
        ay = tf_check.transform.translation.y
        az = tf_check.transform.translation.z
        dx = ax - tx
        dy = ay - ty
        dz = az - tz
        pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)
        return pos_err, (dx, dy, dz), (ax, ay, az), (tx, ty, tz)

    def _verify_screwdriver_tcp(self, target_xyz, tolerance_m: float) -> bool:
        """Closed-loop guard: verify the physical TCP TF reached the commanded pose."""
        err = self._screwdriver_tcp_error(target_xyz)
        if err is None:
            return False
        pos_err, (dx, dy, dz), (ax, ay, az), (tx, ty, tz) = err
        print(f"[DIAG] screwdriver_tcp ACTUAL:   ({ax:.4f}, {ay:.4f}, {az:.4f})")
        print(f"[DIAG] screwdriver_tcp TARGET:   ({tx:.4f}, {ty:.4f}, {tz:.4f})")
        print(f"[DIAG] Position error: dx={dx:.4f} dy={dy:.4f} dz={dz:.4f} m | norm={pos_err:.4f} m")
        return pos_err <= float(tolerance_m)

    def _stream_hover_exotica_verified(self, target_xyz, timeout_s: float = 10.0) -> bool:
        """Teleop-style fallback for coarse hover when MoveGroup reports success but TCP does not move."""
        backend = self.moveit_backend
        planner = backend._single_arm_exotica_planner
        if planner is None or not planner.available:
            print("[DIAG] EXOTica streaming recovery unavailable.")
            return False
        if not backend.state_received.wait(timeout=2.0):
            print("[DIAG] EXOTica streaming recovery has no joint states.")
            return False
        if not backend._ensure_trajectory_mode():
            print("[DIAG] EXOTica streaming recovery could not enter trajectory mode.")
            return False

        tx, ty, tz = (float(v) for v in target_xyz)
        try:
            tf0 = backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], "screwdriver_tcp", rclpy.time.Time()
            )
            ref_roll, ref_pitch, ref_yaw = backend._quaternion_to_rpy(
                tf0.transform.rotation.x,
                tf0.transform.rotation.y,
                tf0.transform.rotation.z,
                tf0.transform.rotation.w,
            )
        except Exception:
            ref_roll, ref_pitch, ref_yaw = 0.0, 0.0, 0.0

        seed_joints = {
            name: float(backend.current_joint_positions.get(name, 0.0))
            for name in planner.controlled_joint_names
        }
        rate_hz = max(10.0, float(self.CONFIG["ALIGN_RATE_HZ"]))
        dt = 1.0 / rate_hz
        joint_alpha = max(0.0, min(1.0, float(self.CONFIG["ALIGN_JOINT_ALPHA"])))
        max_joint_step = min(
            float(self.CONFIG["ALIGN_MAX_JOINT_STEP_RAD"]),
            float(self.CONFIG["ALIGN_MAX_JOINT_VELOCITY_RAD_S"]) / rate_hz,
        )
        deadline = time.time() + float(timeout_s)
        last_diag = 0.0
        print("[DIAG] Starting EXOTica streaming recovery for verified coarse hover.")
        recovery_tolerance = min(float(self.CONFIG["COARSE_POSITION_TOLERANCE"]), 0.010)
        stable_cycles = 0

        while rclpy.ok() and time.time() < deadline:
            full_positions = dict(backend.current_joint_positions)
            full_positions.update(seed_joints)
            result = None
            for pose_rpy in ((ref_roll, ref_pitch, ref_yaw), (0.0, 0.0, 0.0)):
                result = planner.solve_pose_goal_joint_positions(
                    full_positions,
                    [tx, ty, tz, pose_rpy[0], pose_rpy[1], pose_rpy[2]],
                    max_retries=3,
                )
                if result is not None:
                    break
            if result is None:
                print(f"[DIAG] EXOTica streaming recovery IK failed: {planner.last_error}")
                return False

            filtered = {}
            for name in planner.controlled_joint_names:
                previous = float(seed_joints.get(name, backend.current_joint_positions.get(name, 0.0)))
                solved = float(result[name])
                target_val = previous + joint_alpha * (solved - previous)
                delta = max(min(target_val - previous, max_joint_step), -max_joint_step)
                filtered[name] = previous + delta

            backend._publish_direct_joint_command(filtered)
            seed_joints = dict(filtered)

            err = self._screwdriver_tcp_error(target_xyz)
            if err is not None:
                pos_err, _, (ax, ay, az), _ = err
                if pos_err <= recovery_tolerance:
                    stable_cycles += 1
                    if stable_cycles >= 3:
                        print(
                            f"[DIAG] EXOTica streaming recovery reached hover: "
                            f"actual=({ax:.4f},{ay:.4f},{az:.4f}) err={pos_err*1000:.1f}mm"
                        )
                        backend._hold_current_arm_position()
                        self.wait_for_arm_settled(timeout=3.0)
                        return self._verify_screwdriver_tcp(target_xyz, recovery_tolerance)
                else:
                    stable_cycles = 0
                now = time.time()
                if now - last_diag > 0.75:
                    print(
                        f"[DIAG] EXOTica streaming recovery TCP err={pos_err*1000:.1f}mm "
                        f"actual=({ax:.3f},{ay:.3f},{az:.3f})"
                    )
                    last_diag = now
            time.sleep(dt)

        backend._hold_current_arm_position()
        print("[DIAG] EXOTica streaming recovery timed out before TCP reached hover.")
        return self._verify_screwdriver_tcp(target_xyz, recovery_tolerance)

    def _nearest_detection_to_crosshair(self, detections, crosshair):
        """Pick the local target nearest the tool-camera crosshair."""
        if not detections:
            return None
        cx, cy = crosshair if len(crosshair) >= 2 else [320, 240]

        def _dist(item):
            center = item.get("center") or item.get("contact_point") or [320, 240]
            return math.hypot(float(center[0]) - float(cx), float(center[1]) - float(cy))

        return min(detections, key=_dist)

    def _approach_verified_hover(self, tx: float, ty: float, hover_z: float) -> bool:
        """Move to screw hover and require physical TCP verification after each attempt."""
        preferred = str(self.CONFIG.get("COARSE_PLANNER", "moveit")).lower()
        # Keep coarse-position debugging deterministic: only use the configured
        # planner, then verify physical TCP. Extra fallbacks hide the real issue.
        fallback_order = ()
        methods = []
        for method in (preferred, *fallback_order):
            if method not in methods:
                methods.append(method)

        hover_target = (tx, ty, hover_z)
        for method in methods:
            print(f"[DIAG] Coarse hover attempt using {method}.")
            if method == "direct":
                approach_ok = self.moveit_backend.move_to_pose_direct_exotica(
                    tx, ty, hover_z, velocity=self.CONFIG["COARSE_VELOCITY"]
                )
            elif method == "exotica":
                approach_ok = self.moveit_backend.move_to_pose_exotica(
                    tx, ty, hover_z, velocity=self.CONFIG["COARSE_VELOCITY"]
                )
            else:
                approach_ok = self.moveit_backend.move_to_pose_robust(
                    tx, ty, hover_z, velocity=self.CONFIG["COARSE_VELOCITY"]
                )

            if not approach_ok:
                print(f"[DIAG] {method} coarse hover command failed.")
                if method == "moveit":
                    print("[DIAG] Trying verified EXOTica streaming recovery after MoveIt coarse command failure.")
                    if self._stream_hover_exotica_verified(hover_target, timeout_s=10.0):
                        print("[DIAG] Coarse hover verified with EXOTica streaming recovery.")
                        return True
                continue

            self.wait_for_arm_settled(timeout=8.0)
            if self._verify_screwdriver_tcp(hover_target, self.CONFIG["COARSE_POSITION_TOLERANCE"]):
                print(f"[DIAG] Coarse hover verified with {method}.")
                return True
            print(f"[DIAG] {method} command completed but TCP verification failed.")
            if method == "moveit":
                print("[DIAG] MoveIt action success disagreed with TF; trying verified EXOTica streaming recovery.")
                if self._stream_hover_exotica_verified(hover_target, timeout_s=10.0):
                    print("[DIAG] Coarse hover verified with EXOTica streaming recovery.")
                    return True

        print(
            f"❌ Coarse xArm TCP exceeds "
            f"{self.CONFIG['COARSE_POSITION_TOLERANCE']*1000:.1f}mm after all planners. "
            "Aborting before descent/search."
        )
        return False

    # =========================================================================
    # 1. ALIGNMENT & DESCENT (EXOTica real-time IK — no servo mode)
    # =========================================================================
    def perform_xy_align_search_only(self):
        """Debug mode: MoveIt Servo XY align/spiral search only, with Z locked."""
        print("\n[ALIGN-ONLY] Starting MoveIt Servo XY align/search with Z locked.")
        self.wait_for_arm_settled(timeout=3.0)

        try:
            start_tf = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], "screwdriver_tcp", rclpy.time.Time()
            )
            locked_z = float(start_tf.transform.translation.z)
        except Exception as exc:
            print(f"[ALIGN-ONLY] Could not read initial screwdriver_tcp: {exc}")
            return False

        if not self.moveit_backend.start_servo(timeout_sec=8.0):
            print("[ALIGN-ONLY] Failed to start MoveIt Servo.")
            return False

        def _stop_servo():
            try:
                self.moveit_backend._publish_zero_twist()
                self.moveit_backend.stop_servo(timeout_sec=3.0)
            except Exception as exc:
                print(f"[ALIGN-ONLY] Servo stop failed: {exc}")

        def _get_target():
            with self.data_lock:
                local = copy.deepcopy(self.local_view)
            local_view = local.get("local_view", {})
            crosshair = local_view.get("crosshair") or [320, 240]
            candidates = local_view.get("screw_heads", [])
            target_class = "screw_head"
            if not candidates:
                candidates = local_view.get("screws", [])
                target_class = "screw"
            if not candidates:
                return None, None, crosshair
            target = self._nearest_detection_to_crosshair(candidates, crosshair)
            return target_class, target, crosshair

        def _get_tcp():
            try:
                tf_now = self.moveit_backend.tf_buffer.lookup_transform(
                    self.CONFIG["WORLD_FRAME"], "screwdriver_tcp", rclpy.time.Time()
                )
                return (
                    float(tf_now.transform.translation.x),
                    float(tf_now.transform.translation.y),
                    float(tf_now.transform.translation.z),
                )
            except Exception:
                return None

        start_t = time.time()
        align_timeout = max(float(self.CONFIG["ALIGN_ONLY_TIMEOUT"]), float(self.CONFIG["SPIRAL_TIMEOUT"]))
        prev_vx = 0.0
        prev_vy = 0.0
        stable_cycles = 0
        no_target_logged = False
        commanded_since_t = None
        commanded_start_xy = None
        stall_recoveries = 0
        spiral_side_idx = 0
        spiral_side_len_mm = float(self.CONFIG["SPIRAL_START_DIST_MM"])
        spiral_stall_count = 0

        while rclpy.ok() and (time.time() - start_t) < align_timeout:
            tcp = _get_tcp()
            if tcp is None:
                print("\n[ALIGN-ONLY] Could not read screwdriver_tcp during align.")
                _stop_servo()
                return False
            ex, ey, ez = tcp
            z_err = abs(ez - locked_z)
            if z_err > float(self.CONFIG["ALIGN_VERIFY_TOLERANCE"]):
                print(f"\n[ALIGN-ONLY] Z drift {z_err*1000:.1f}mm exceeds tolerance.")
                _stop_servo()
                return False

            target_class, target, crosshair = _get_target()
            if target is None:
                if not no_target_logged:
                    print("[ALIGN-ONLY] No local screw target; starting closed-loop square spiral.")
                    no_target_logged = True
                search_speed = max(0.0005, min(float(self.CONFIG["SPIRAL_SPEED"]), 0.003))
                while (
                    rclpy.ok()
                    and (time.time() - start_t) < align_timeout
                ):
                    target_class, target, _ = _get_target()
                    if target is not None:
                        print("[ALIGN-ONLY] Local target regained during spiral.")
                        prev_vx = 0.0
                        prev_vy = 0.0
                        stable_cycles = 0
                        no_target_logged = False
                        break
                    tcp = _get_tcp()
                    if tcp is not None:
                        z_err = abs(tcp[2] - locked_z)
                        if z_err > float(self.CONFIG["ALIGN_VERIFY_TOLERANCE"]):
                            print(f"\n[ALIGN-ONLY] Z drift {z_err*1000:.1f}mm exceeds tolerance.")
                            _stop_servo()
                            return False
                    dist_m = spiral_side_len_mm / 1000.0
                    if spiral_side_idx % 4 == 0:
                        dx, dy, name = -dist_m, 0.0, "-X"
                    elif spiral_side_idx % 4 == 1:
                        dx, dy, name = 0.0, -dist_m, "-Y"
                    elif spiral_side_idx % 4 == 2:
                        dx, dy, name = dist_m, 0.0, "+X"
                    else:
                        dx, dy, name = 0.0, dist_m, "+Y"
                    print(
                        f"  [SPIRAL] side={name} length={spiral_side_len_mm:.1f}mm "
                        f"speed={search_speed*1000:.1f}mm/s"
                    )
                    result = self.moveit_backend.move_servo_xy_closed_loop(
                        dx,
                        dy,
                        speed_mps=search_speed,
                        timeout=max(2.0, dist_m / search_speed + 1.0),
                        stop_check=lambda: _get_target()[1] is not None,
                        z_lock_m=locked_z,
                        z_gain=float(self.CONFIG["ALIGN_Z_LOCK_GAIN"]),
                        z_speed_cap=float(self.CONFIG["ALIGN_Z_LOCK_SPEED"]),
                    )
                    if result == "STOPPED":
                        print("[ALIGN-ONLY] Local target regained during spiral.")
                        prev_vx = 0.0
                        prev_vy = 0.0
                        stable_cycles = 0
                        no_target_logged = False
                        spiral_side_idx = 0
                        spiral_side_len_mm = float(self.CONFIG["SPIRAL_START_DIST_MM"])
                        spiral_stall_count = 0
                        break
                    status = result.get("status") if isinstance(result, dict) else ("DONE" if result else "FAILED")
                    travelled = float(result.get("travelled_m", 0.0)) if isinstance(result, dict) else 0.0
                    target_dist = float(result.get("target_m", dist_m)) if isinstance(result, dict) else dist_m
                    print(
                        f"  [SPIRAL] {status}: travelled={travelled*1000:.1f}/"
                        f"{target_dist*1000:.1f}mm"
                    )
                    if travelled < max(0.0004, target_dist * 0.25):
                        spiral_stall_count += 1
                        print(f"  [SPIRAL] low motion detected ({spiral_stall_count}/3); restarting Servo.")
                        self.moveit_backend.stop_servo(timeout_sec=2.0)
                        time.sleep(0.2)
                        if not self.moveit_backend.start_servo(timeout_sec=5.0):
                            print("[ALIGN-ONLY] Servo restart failed during spiral.")
                            _stop_servo()
                            return False
                        if spiral_stall_count >= 3:
                            print("[ALIGN-ONLY] Spiral stalled repeatedly; aborting search.")
                            _stop_servo()
                            return False
                    else:
                        spiral_stall_count = 0
                    spiral_side_idx += 1
                    if spiral_side_idx % 2 == 0:
                        spiral_side_len_mm += float(self.CONFIG["SPIRAL_GAP_MM"])
                continue

            center = target.get("center", [320, 240])
            err_x = float(crosshair[0] - center[0])
            err_y = float(crosshair[1] - center[1])
            dist_px = math.hypot(err_x, err_y)
            if dist_px <= float(self.CONFIG["ALIGN_TOLERANCE_PX"]):
                stable_cycles += 1
                self.moveit_backend._publish_zero_twist()
                print(
                    f"  [ALIGN/{target_class}] stable "
                    f"{stable_cycles}/{self.CONFIG['ALIGN_STABLE_CYCLES']} "
                    f"ErrX={err_x:.1f}px ErrY={err_y:.1f}px"
                )
                if stable_cycles >= int(self.CONFIG["ALIGN_STABLE_CYCLES"]):
                    print(f"\n[ALIGN-ONLY] Aligned on {target_class}.")
                    _stop_servo()
                    return True
                time.sleep(0.08)
                continue

            stable_cycles = 0
            max_xy = float(self.CONFIG["XY_SPEED_ALIGN"])
            min_xy = min(float(self.CONFIG["ALIGN_MIN_SPEED"]), max_xy)
            full_speed_error = max(float(self.CONFIG["ALIGN_FULL_SPEED_ERROR_PX"]), 1.0)

            def _axis_speed(error_px: float) -> float:
                if abs(error_px) <= float(self.CONFIG["ALIGN_TOLERANCE_PX"]):
                    return 0.0
                ratio = min(abs(error_px) / full_speed_error, 1.0)
                speed = min_xy + (max_xy - min_xy) * ratio
                return math.copysign(speed, error_px)

            if abs(err_y) > float(self.CONFIG["ALIGN_TOLERANCE_PX"]):
                target_vx = -_axis_speed(err_y)
                target_vy = 0.0
                axis = "Yimg->Xbase"
            else:
                target_vx = 0.0
                target_vy = -_axis_speed(err_x)
                axis = "Ximg->Ybase"

            alpha = max(0.05, min(1.0, float(self.CONFIG["ALIGN_ERROR_FILTER_ALPHA"])))
            vx = alpha * target_vx + (1.0 - alpha) * prev_vx
            vy = alpha * target_vy + (1.0 - alpha) * prev_vy
            prev_vx, prev_vy = vx, vy
            vz = max(
                min((locked_z - ez) * float(self.CONFIG["ALIGN_Z_LOCK_GAIN"]), float(self.CONFIG["ALIGN_Z_LOCK_SPEED"])),
                -float(self.CONFIG["ALIGN_Z_LOCK_SPEED"]),
            )
            print(
                f"  [ALIGN/{target_class}/{axis}] ErrX={err_x:5.1f} ErrY={err_y:5.1f} "
                f"cmd=({vx*1000:.1f},{vy*1000:.1f},{vz*1000:.1f})mm/s Zerr={z_err*1000:.1f}mm"
            )
            if commanded_since_t is None or commanded_start_xy is None:
                commanded_since_t = time.time()
                commanded_start_xy = (ex, ey)
            elif time.time() - commanded_since_t > 2.0:
                moved = math.hypot(ex - commanded_start_xy[0], ey - commanded_start_xy[1])
                if moved < 0.0003:
                    print(
                        "\n[ALIGN-ONLY] Servo commands active but screwdriver_tcp moved "
                        f"only {moved*1000:.2f}mm in 2s; restarting Servo and continuing."
                    )
                    stall_recoveries += 1
                    if stall_recoveries > 3:
                        print("[ALIGN-ONLY] Servo stalled repeatedly; aborting alignment.")
                        _stop_servo()
                        return False
                    self.moveit_backend.stop_servo(timeout_sec=2.0)
                    time.sleep(0.2)
                    if not self.moveit_backend.start_servo(timeout_sec=5.0):
                        print("[ALIGN-ONLY] Servo restart failed.")
                        return False
                    prev_vx = 0.0
                    prev_vy = 0.0
                    commanded_since_t = time.time()
                    commanded_start_xy = (ex, ey)
                    continue
                commanded_since_t = time.time()
                commanded_start_xy = (ex, ey)
            self.moveit_backend.jog_cartesian_servo(vx, vy, vz, duration=0.15)
            time.sleep(0.03)

        _stop_servo()
        print("\n[ALIGN-ONLY] Search timed out without alignment.")
        return False

    def perform_staircase_descent(self):
        """EXOTica real-time IK descent — TouchLab/teleoperation style, no servo mode."""
        print("\n[DESCENT] Starting EXOTica real-time IK descent (warm-start + joint smoothing)...")

        # Snapshot FT baseline
        with self.data_lock:
            base_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)

        # Get initial EE orientation to hold constant during descent
        try:
            tf0 = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], "screwdriver_tcp", rclpy.time.Time()
            )
            ref_roll, ref_pitch, ref_yaw = self.moveit_backend._quaternion_to_rpy(
                tf0.transform.rotation.x, tf0.transform.rotation.y,
                tf0.transform.rotation.z, tf0.transform.rotation.w,
            )
        except Exception as exc:
            print(f"[DESCENT] Could not read initial EE orientation: {exc}. Using zero RPY.")
            ref_roll, ref_pitch, ref_yaw = 0.0, 0.0, 0.0

        retry_count = 0
        MAX_RETRIES = 3

        # Spiral state (persists across retries so the arm doesn't re-trace)
        spiral_state = {
            "side_idx": 0,
            "side_len_mm": self.CONFIG["SPIRAL_START_DIST_MM"],
            "side_progress_m": 0.0,
        }
        contact_flag = [False]

        def _get_ee():
            try:
                t = self.moveit_backend.tf_buffer.lookup_transform(
                    self.CONFIG["WORLD_FRAME"], "screwdriver_tcp", rclpy.time.Time()
                )
                return t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
            except Exception:
                return None

        servo_origin = _get_ee()
        if servo_origin is None:
            print("[DESCENT] Could not read screwdriver_tcp before visual servo.")
            return False

        def _clamp_search_xy(x, y):
            max_radius = float(self.CONFIG["VISUAL_SERVO_MAX_RADIUS"])
            ox, oy = servo_origin[0], servo_origin[1]
            dx = float(x) - ox
            dy = float(y) - oy
            radius = math.hypot(dx, dy)
            if radius <= max_radius or radius <= 1e-9:
                return float(x), float(y), radius
            scale = max_radius / radius
            return ox + dx * scale, oy + dy * scale, max_radius

        def target_fn():
            """50 Hz target provider for move_cartesian_realtime_exotica."""
            with self.data_lock:
                local = copy.deepcopy(self.local_view)

            # FT contact check
            diff_fz = abs(local.get('force_torque', {}).get('force', {}).get('z', 0.0) - base_fz)
            if diff_fz > self.CONFIG["FORCE_THRESHOLD"]:
                contact_flag[0] = True
                return None  # stop loop

            ee = _get_ee()
            if ee is None:
                return None
            ex, ey, ez = ee

            # Priority 1: screw_head (accurate centre for bit alignment)
            # Priority 2: screw body (occluded head / head not trained)
            # If neither is visible, keep searching. Holes are not a stop condition
            # here because the requested behavior is screw_head -> screw -> spiral.
            local_view = local.get('local_view', {})
            screw = local_view.get('screw_heads', [])
            using_screw_class = False
            if not screw:
                screw = local_view.get('screws', [])
                using_screw_class = bool(screw)
            crosshair = local_view.get('crosshair') or [320, 240]

            if screw:
                # Visual alignment + Z descent
                target = self._nearest_detection_to_crosshair(screw, crosshair)
                center = target.get('center', [320, 240]) if target else [320, 240]
                err_x = crosshair[0] - center[0]
                err_y = crosshair[1] - center[1]
                dist_px = math.hypot(err_x, err_y)
                MAX_XY = self.CONFIG["XY_SPEED_ALIGN"]
                dt = 1.0 / 50.0
                gain = float(self.CONFIG["VISUAL_SERVO_GAIN"])
                dx = max(min((err_y * self.CONFIG["MM_PER_PIX"]) * -gain * dt, MAX_XY * dt), -MAX_XY * dt)
                dy = max(min((err_x * self.CONFIG["MM_PER_PIX"]) * -gain * dt, MAX_XY * dt), -MAX_XY * dt)
                next_x, next_y, radius = _clamp_search_xy(ex + dx, ey + dy)
                z_step = self.CONFIG["Z_SPEED_DESCENT"] * dt
                if dist_px > 30.0:
                    z_step = 0.0
                elif dist_px > self.CONFIG["ALIGN_TOLERANCE_PX"]:
                    z_step *= self.CONFIG["ALIGN_TOLERANCE_PX"] / dist_px
                cls_tag = "screw" if using_screw_class else "screw_head"
                print(
                    f"  [VIS/{cls_tag}] ErrX={err_x:5.1f} ErrY={err_y:5.1f} "
                    f"Fz={diff_fz:.2f}N R={radius*1000:.1f}mm Vz={z_step*1000:.1f}mm/s",
                    end="\r",
                )
                return (next_x, next_y, ez - z_step, ref_roll, ref_pitch, ref_yaw)
            else:
                # Square spiral search (XY only, no Z)
                ss = spiral_state
                step_m = self.CONFIG["SPIRAL_SPEED"] / 50.0
                side_len_m = ss["side_len_mm"] / 1000.0
                si = ss["side_idx"] % 4
                if si == 0:   dx, dy = -step_m,  0.0
                elif si == 1: dx, dy =  0.0,     -step_m
                elif si == 2: dx, dy =  step_m,   0.0
                else:         dx, dy =  0.0,      step_m
                ss["side_progress_m"] += step_m
                if ss["side_progress_m"] >= side_len_m:
                    ss["side_progress_m"] = 0.0
                    ss["side_idx"] += 1
                    if ss["side_idx"] % 2 == 0:
                        ss["side_len_mm"] += self.CONFIG["SPIRAL_GAP_MM"]
                next_x, next_y, radius = _clamp_search_xy(ex + dx, ey + dy)
                print(f"  [SPIRAL] no screw target | R={radius*1000:.1f}mm", end="\r")
                return (next_x, next_y, ez, ref_roll, ref_pitch, ref_yaw)

        while retry_count <= MAX_RETRIES:
            contact_flag[0] = False
            result = self.moveit_backend.move_cartesian_realtime_exotica(
                target_fn,
                rate_hz=50.0,
                max_step_m=self.CONFIG["SEARCH_MAX_STEP"],
                joint_smooth_alpha=self.CONFIG["SEARCH_JOINT_SMOOTH_ALPHA"],
                timeout_s=self.CONFIG["SPIRAL_TIMEOUT"],
            )

            if contact_flag[0]:
                print(f"\n[CONTACT] FT contact detected.")
                time.sleep(0.3)

                # Fine XY surface alignment
                print("[SURFACE ALIGN] Correcting XY on surface...")
                align_deadline = time.time() + 3.0
                while rclpy.ok() and time.time() < align_deadline:
                    with self.data_lock:
                        al = copy.deepcopy(self.local_view)
                    al_local = al.get('local_view', {})
                    al_screw = al_local.get('screw_heads', [])
                    if not al_screw:
                        al_screw = al_local.get('screws', [])
                    al_cross = al_local.get('crosshair') or [320, 240]
                    if not al_screw:
                        print("[SURFACE ALIGN] No screw visible, skipping.")
                        break
                    al_target = self._nearest_detection_to_crosshair(al_screw, al_cross)
                    al_center = al_target.get('center', [320, 240]) if al_target else [320, 240]
                    al_ex = al_cross[0] - al_center[0]
                    al_ey = al_cross[1] - al_center[1]
                    if math.hypot(al_ex, al_ey) <= self.CONFIG["ALIGN_TOLERANCE_PX"]:
                        print(f"[SURFACE ALIGN] Aligned.")
                        break
                    ee = _get_ee()
                    if ee:
                        dt = 1.0 / 50.0
                        MAX_XY = self.CONFIG["XY_SPEED_ALIGN"]
                        cdx = max(min((al_ey * self.CONFIG["MM_PER_PIX"]) * -2.0 * dt, MAX_XY * dt), -MAX_XY * dt)
                        cdy = max(min((al_ex * self.CONFIG["MM_PER_PIX"]) * -2.0 * dt, MAX_XY * dt), -MAX_XY * dt)
                        ex, ey, ez = ee
                        next_x, next_y, _ = _clamp_search_xy(ex + cdx, ey + cdy)
                        self.moveit_backend.move_cartesian_realtime_exotica(
                            lambda next_x=next_x, next_y=next_y, ez=ez: (next_x, next_y, ez, ref_roll, ref_pitch, ref_yaw),
                            timeout_s=0.12,
                            rate_hz=50.0,
                            max_step_m=self.CONFIG["SEARCH_MAX_STEP"],
                            joint_smooth_alpha=self.CONFIG["SEARCH_JOINT_SMOOTH_ALPHA"],
                        )
                    time.sleep(0.05)
                time.sleep(0.2)

                # Seating check
                print("[SEATING CHECK] Running short unscrew to verify bit engagement...")
                with self.data_lock:
                    pre_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)
                peak_fz_spike = 0.0
                self.tool_pub.publish(Int8(data=-1))
                spin_start = time.time()
                while time.time() - spin_start < 1.0:
                    with self.data_lock:
                        curr_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)
                    peak_fz_spike = max(peak_fz_spike, abs(curr_fz - pre_fz))
                    time.sleep(0.05)
                self.tool_pub.publish(Int8(data=0))
                time.sleep(0.3)
                with self.data_lock:
                    post_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)
                z_spike = max(peak_fz_spike, abs(post_fz - pre_fz))
                print(f"[SEATING CHECK] Peak={peak_fz_spike:.2f}N post={abs(post_fz-pre_fz):.2f}N (threshold 3.0N)")

                if z_spike > 3.0:
                    print("[ALIGNED] Bit seated. Proceeding to extraction.")
                    return True
                else:
                    retry_count += 1
                    if retry_count > MAX_RETRIES:
                        print("[ABORT] Max retries exceeded.")
                        return False
                    print(f"[NOT ALIGNED] Retry {retry_count}/{MAX_RETRIES} — retracting 5mm...")
                    self.moveit_backend.retract_z_exotica(0.005, speed_mps=0.03)
                    with self.data_lock:
                        base_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)
                    spiral_state["side_idx"] = 0
                    spiral_state["side_len_mm"] = self.CONFIG["SPIRAL_START_DIST_MM"]
                    spiral_state["side_progress_m"] = 0.0
                    continue

            elif result == "TIMEOUT":
                print("[TIMEOUT] Descent timed out without contact.")
                return "TIMEOUT"
            else:
                print(f"[DESCENT] Ended unexpectedly: {result}")
                return False

        return False

    # =========================================================================
    # 2. COMPLIANT EXTRACTION
    # =========================================================================
    def perform_compliant_extraction(self):
        """EXOTica real-time compliant extraction — force-proportional Z lift, no servo mode."""
        print("\n[EXTRACTION] Starting compliant EXOTica extraction...")

        with self.data_lock:
            baseline_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)

        self.tool_pub.publish(Int8(data=-1))  # Unscrew
        time.sleep(0.2)
        self.tool_pub.publish(Int8(data=2))   # Grab

        # Get initial EE orientation
        try:
            tf0 = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], "screwdriver_tcp", rclpy.time.Time()
            )
            ref_roll, ref_pitch, ref_yaw = self.moveit_backend._quaternion_to_rpy(
                tf0.transform.rotation.x, tf0.transform.rotation.y,
                tf0.transform.rotation.z, tf0.transform.rotation.w,
            )
        except Exception:
            ref_roll, ref_pitch, ref_yaw = 0.0, 0.0, 0.0

        peak_upward_force = [0.0]
        last_increase_time = [time.time()]
        start_time = time.time()

        def target_fn():
            if time.time() - start_time > self.CONFIG["EXTRACTION_MAX_TIME"]:
                return None
            with self.data_lock:
                current_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)
            upward_force = -(current_fz - baseline_fz)
            if upward_force > peak_upward_force[0] + 1.0:
                peak_upward_force[0] = upward_force
                last_increase_time[0] = time.time()
            if time.time() - last_increase_time[0] >= self.CONFIG["EXTRACTION_STABLE_TIME"]:
                print(f"\n[FREE] Force stabilized at {peak_upward_force[0]:.2f}N.")
                return None
            z_speed = max(0.0, min(
                upward_force * self.CONFIG["EXTRACTION_COMPLIANCE_K"],
                self.CONFIG["EXTRACTION_Z_SPEED_CAP"],
            ))
            z_step = z_speed / 50.0
            print(f"  [EXTRACT] Fz_up={upward_force:.2f}N Vz={z_speed*1000:.1f}mm/s Peak={peak_upward_force[0]:.2f}N", end="\r", flush=True)
            try:
                t = self.moveit_backend.tf_buffer.lookup_transform(
                    self.CONFIG["WORLD_FRAME"], "screwdriver_tcp", rclpy.time.Time()
                )
                return (
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z + z_step,
                    ref_roll, ref_pitch, ref_yaw,
                )
            except Exception:
                return None

        self.moveit_backend.move_cartesian_realtime_exotica(
            target_fn,
            rate_hz=50.0,
            max_step_m=0.002,
            joint_smooth_alpha=0.6,
            timeout_s=self.CONFIG["EXTRACTION_MAX_TIME"] + 2.0,
        )

        self.tool_pub.publish(Int8(data=0))
        time.sleep(0.5)

        print(f"\n[POST-GRASP] Retracting {self.CONFIG['POST_GRASP_RETRACT']*1000:.0f}mm...")
        if not self.moveit_backend.retract_z_exotica(
            self.CONFIG["POST_GRASP_RETRACT"],
            speed_mps=self.CONFIG["RETRACT_SPEED"],
        ):
            print("[WARN] Post-grasp retract reported failure.")
        self.wait_for_arm_settled()
        return True

    # =========================================================================
    # MAIN SEQUENCE
    # =========================================================================
    def execute_unscrew_command(self, target_id, target_label, interactive=True, target_data_override=None):
        print(f"\n🛠️ [START] Unscrew Sequence on ID: {target_id} ({target_label})")
        
        # Verify Bin 1 location
        with self.data_lock: bin1_raw = copy.deepcopy(self.cached_bin1_xyz)
        if not bin1_raw:
            print("❌ Bin 1 location missing. Aborting.")
            return False

        # Find Target Object
        with self.data_lock:
            target_data = copy.deepcopy(target_data_override) if target_data_override else None
            if target_data:
                print(f"[DIAG] Using runner-provided screw snapshot ID={target_data.get('id')} label={target_data.get('label')}")
            if not target_data:
                target_data = next((t for t in self.latest_targets if t.get('id') == target_id), None)
            if not target_data and target_label:
                target_label_l = str(target_label).lower()
                exact_matches = [
                    t for t in self.latest_targets
                    if t.get("label", "").lower() == target_label_l
                ]
                partial_matches = [
                    t for t in self.latest_targets
                    if target_label_l in t.get("label", "").lower()
                ]
                candidates = exact_matches or partial_matches
                target_data = candidates[0] if candidates else None
        if not target_data or 'xyz' not in target_data:
            print(f"❌ Target {target_id} not found in vision.")
            return False
        if target_data.get("id") != target_id:
            print(
                f"[DIAG] Target ID {target_id} shifted; using "
                f"ID={target_data.get('id')} label={target_data.get('label')}"
            )

        # ── Camera-chain sanity check ──────────────────────────────────────────
        try:
            cam_tf = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], self.CONFIG["CAMERA_FRAME"], rclpy.time.Time()
            )
            cx = cam_tf.transform.translation.x
            cy = cam_tf.transform.translation.y
            cz = cam_tf.transform.translation.z
            print(f"[DIAG] {self.CONFIG['CAMERA_FRAME']} origin in base_link: ({cx:.4f}, {cy:.4f}, {cz:.4f})")
            print(f"[DIAG] Expected from calib: (~1.058, ~0.074, ~1.480)")
            q = cam_tf.transform.rotation
            print(f"[DIAG] Camera orientation quat: x={q.x:.3f} y={q.y:.3f} z={q.z:.3f} w={q.w:.3f}")
        except Exception as e:
            print(f"[DIAG] Camera TF lookup FAILED: {e} — check handeye publisher and camera driver TF")

        # Transforms
        raw_pose = Pose()
        raw_pose.position.x, raw_pose.position.y, raw_pose.position.z = target_data['xyz']
        print(f"[DIAG] Vision raw xyz in {self.CONFIG['CAMERA_FRAME']}: "
              f"({raw_pose.position.x:.4f}, {raw_pose.position.y:.4f}, {raw_pose.position.z:.4f})")
        if not (self.CONFIG["SCREW_CAMERA_Z_MIN"] <= raw_pose.position.z <= self.CONFIG["SCREW_CAMERA_Z_MAX"]):
            print(
                f"❌ Rejecting target: camera z={raw_pose.position.z:.4f}m outside "
                f"[{self.CONFIG['SCREW_CAMERA_Z_MIN']:.2f}, {self.CONFIG['SCREW_CAMERA_Z_MAX']:.2f}]m. "
                "This is likely a bad depth sample or occlusion."
            )
            return False
        world_pose = self.moveit_backend.get_transformed_pose(raw_pose, self.CONFIG["CAMERA_FRAME"], self.CONFIG["WORLD_FRAME"])
        base_pose = self.moveit_backend.get_transformed_pose(raw_pose, self.CONFIG["CAMERA_FRAME"], self.CONFIG["XARM_BASE_FRAME"])

        if not world_pose or not base_pose: return False

        tx, ty = world_pose.pose.position.x, world_pose.pose.position.y
        tz_screw = world_pose.pose.position.z
        dist_base = math.hypot(base_pose.pose.position.x, base_pose.pose.position.y)
        hover_z = tz_screw + self.CONFIG["HOVER_DISTANCE"]

        print(f"[DIAG] Screw in base_link: ({tx:.4f}, {ty:.4f}, {tz_screw:.4f})")
        print(f"[DIAG] hover_z = {tz_screw:.4f} + {self.CONFIG['HOVER_DISTANCE']:.3f} = {hover_z:.4f}")
        print(f"[DIAG] xarm5 base dist: {dist_base:.3f}m (limit {self.CONFIG['REACH_LIMIT']:.3f}m)")

        if not (self.CONFIG["SCREW_WORLD_Z_MIN"] <= tz_screw <= self.CONFIG["SCREW_WORLD_Z_MAX"]):
            print(
                f"❌ Rejecting target: base_link screw z={tz_screw:.4f}m outside "
                f"[{self.CONFIG['SCREW_WORLD_Z_MIN']:.2f}, {self.CONFIG['SCREW_WORLD_Z_MAX']:.2f}]m. "
                "This would command the screwdriver to the wrong height."
            )
            return False

        if dist_base > self.CONFIG["REACH_LIMIT"]:
            print(f"❌ Reach {dist_base:.3f}m exceeds limit.")
            return False

        if interactive: input(f"👉 GATE 1: Approach Hover ({hover_z:.3f}m) [ENTER]")

        pre_retract = float(self.CONFIG.get("PRE_RETRACT", 0.0))
        if pre_retract > 0.0:
            # Optional only. For xArm this is disabled in the HDD config because
            # hardware mode faults can turn this initial lift into lateral motion.
            if not self.moveit_backend.retract_z_exotica(pre_retract, speed_mps=self.CONFIG["RETRACT_SPEED"]):
                print("❌ Z-only pre-retract failed. Aborting before coarse approach.")
                return False
            self.wait_for_arm_settled()
        else:
            print("[DIAG] Pre-retract disabled; moving directly to verified screw hover.")

        if not self._approach_verified_hover(tx, ty, hover_z):
            return False

        if bool(self.CONFIG.get("COARSE_ONLY_DEBUG", False)):
            print("[DEBUG] Coarse-only mode active: verified hover reached; skipping descent, visual servo, tool spin, extraction, and bin drop.")
            return True

        if bool(self.CONFIG.get("ALIGN_ONLY_DEBUG", False)):
            print("[DEBUG] Align-only mode active: running XY search/align with Z locked; skipping descent, tool spin, extraction, and bin drop.")
            return self.perform_xy_align_search_only()

        if interactive: input("👉 GATE 2: Start EXOTica Descent [ENTER]")

        # Stage 1: Descent
        staircase_res = self.perform_staircase_descent()
        if staircase_res == "TIMEOUT" or staircase_res is False:
            print("⚠️ Descent failed or timed out. Lifting to safety.")
            self.moveit_backend.retract_z_exotica(
                self.CONFIG["TRANSIT_LIFT"],
                speed_mps=self.CONFIG["RETRACT_SPEED"],
            )
            self._navigate_to_bin(bin1_raw)
            return False

        # Stage 2: Extraction
        if interactive: input("👉 GATE 3: Start Compliant Extraction [ENTER]")
        if self.perform_compliant_extraction():
            print("🎉 Screw Extracted.")
            self._navigate_to_bin(bin1_raw)
            return True

        return False

    def _navigate_to_bin(self, bin1_raw):
        print("🗑️ [DROP-OFF] Navigating to Bin 1...")
        bin_pose = Pose()
        bin_pose.position.x, bin_pose.position.y, bin_pose.position.z = bin1_raw
        world_bin = self.moveit_backend.get_transformed_pose(bin_pose, self.CONFIG["CAMERA_FRAME"], self.CONFIG["WORLD_FRAME"])
        
        if world_bin:
            bx, by = world_bin.pose.position.x, world_bin.pose.position.y
            bz = world_bin.pose.position.z + 0.030
            
            if self.moveit_backend.move_to_pose_exotica(bx, by, bz, velocity=self.CONFIG["COARSE_VELOCITY"]):
                self.wait_for_arm_settled()
                print("⏬ [RELEASE] Dropping screw...")
                self.tool_pub.publish(Int8(data=3)) # Release
                time.sleep(1.0)
                self.tool_pub.publish(Int8(data=0))
                print("♻️ Reset Complete.")

def main(args=None):
    rclpy.init(args=args)
    node = UnscrewSkill()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        time.sleep(2.0)
        while rclpy.ok():
            target_id, target_label = None, None
            with node.data_lock:
                if node.latest_targets:
                    target_id = node.latest_targets[0].get('id')
                    target_label = node.latest_targets[0].get('label', 'screw')
            
            if target_id is not None:
                node.execute_unscrew_command(target_id, target_label, interactive=False)
                with node.data_lock: node.latest_targets = []
            time.sleep(0.5)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
