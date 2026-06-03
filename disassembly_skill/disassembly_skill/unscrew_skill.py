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
            "HOVER_DISTANCE": 0.003,        # m (3mm above screw tip)
            "TRANSIT_LIFT": 0.030,          # m (Safe travel height)
            "REACH_LIMIT": 0.680,           # m (xArm reach radius)
            "REACH_SOFT_MARGIN": 0.120,     # m, screwdriver TCP can extend past nominal flange reach
            "MM_PER_PIX": 0.000130,         # m/px (Vision calibration)
            "COARSE_VELOCITY": 0.20,        # trajectory scaling for coarse pose moves
            "COARSE_POSITION_TOLERANCE": 0.010,  # m, abort descent/search if hover TCP is off target
            "COARSE_PLANNER": "exotica",    # EXOTica IK with slide correction — accurate hover position
            "COARSE_X_OFFSET": 0.0,          # m, base_link-frame coarse target trim for xArm/URDF mismatch
            "COARSE_Y_OFFSET": -0.005,          # m, base_link-frame coarse target trim for xArm/URDF mismatch
            "COARSE_ONLY_DEBUG": False,      # temporary: verify all screw hover poses before unscrewing
            "ALIGN_ONLY_DEBUG": True,        # current debug path: run XY align only, skip descent/extraction
            "DESCENT_ONLY_DEBUG": False,     # validate conical descent, then stop before extraction/bin
            "SCREW_CAMERA_Z_MIN": 0.45,      # m, reject bad depth hits from arm/occluders
            "SCREW_CAMERA_Z_MAX": 0.65,
            "SCREW_WORLD_Z_MIN": 0.90,       # m, HDD screw surface sanity range in base_link
            "SCREW_WORLD_Z_MAX": 1.05,
            "COARSE_HOVER_Z_OVERRIDE": None, # m — if set, overrides depth-derived hover_z for coarse approach
            "PRE_RETRACT": 0.0,             # disabled: bad xArm mode can turn this into lateral motion
            
            # Descent & Alignment
            "XY_SPEED_ALIGN": 0.060,        # m/s, capped lower near final pixel alignment
            "Z_SPEED_DESCENT": 0.020,       # m/s
            "ALIGN_TOLERANCE_PX": 4.0,      # pixels
            "FORCE_THRESHOLD": 5.0,         # N (Contact detection)
            "ENGAGEMENT_DEPTH_MM": 5.0,     # descend below detected screw Z before giving up
            "DESCENT_MAX_DISTANCE": 0.060,  # m, hard travel guard; contact is the real stop

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
            "ALIGN_MIN_SPEED": 0.012,       # m/s, below this xArm may not visibly move
            "ALIGN_Z_LOCK_GAIN": 1.2,
            "ALIGN_Z_LOCK_SPEED": 0.003,
            "ALIGN_TARGET_WAIT_S": 3.0,
            "ALIGN_TARGET_LOST_GRACE_S": 0.6,
            "ENABLE_SPIRAL_SEARCH": True,    # when the local camera sees no screw, search around verified hover
            "ALIGN_RATE_HZ": 15.0,
            "ALIGN_JOINT_ALPHA": 0.35,
            "ALIGN_MAX_JOINT_STEP_RAD": 0.035,
            "ALIGN_MAX_JOINT_VELOCITY_RAD_S": 0.6,
            # Hover alignment only needs to bring the screw into the conical
            # descent capture window; the descent loop keeps correcting XY.
            "PRE_DESCENT_ALIGN_TOLERANCE_PX": 65.0,
            "PRE_DESCENT_ALIGN_TIMEOUT": 45.0,
            "PRE_DESCENT_ALIGN_STABLE_CYCLES": 1,
            "FINAL_ALIGN_TOLERANCE_PX": 3.0,
            "FINAL_ALIGN_TIMEOUT": 20.0,
            "FINAL_ALIGN_MAX_RADIUS": 0.012,
            "FINAL_ALIGN_STABLE_CYCLES": 1,
            "FINAL_ALIGN_XY_SPEED": 0.006,
            "FINAL_ALIGN_MIN_SPEED": 0.002,
            "FINAL_ALIGN_ERROR_ALPHA": 0.15,
            
            # Extraction
            "EXTRACTION_MAX_TIME": 20.0,    # s
            "EXTRACTION_COMPLIANCE_K": 0.015, # Velocity gain per Newton (increased for responsiveness)
            "EXTRACTION_STABLE_TIME": 3.0,  # s
            "EXTRACTION_Z_SPEED_CAP": 0.03, # m/s max compliant lift speed
            "POST_GRASP_RETRACT": 0.015,     # m (15mm as requested)
            "RETRACT_SPEED": 0.06,          # m/s, safety lift after failed unscrew phases
            "POST_UNSCREW_RETRACT_SPEED": 0.06,  # m/s, closed-loop lift after screw grab
            "UNSCREW_PROBE_DURATION": 0.50,  # s, first short engagement test spin
            "UNSCREW_PROBE_FORCE_INCREASE": 0.30,  # N, axial force change required to trust engagement
            "UNSCREW_MIN_SPIN_TIME": 1.0,    # s, minimum continuous unscrew after probe
            "UNSCREW_MAX_SPIN_TIME": 4.0,    # s, hard cap before grabbing/retracting
            "UNSCREW_RELEASE_FORCE_RATIO": 0.50,  # stable drop below peak ratio indicates loosening
            "UNSCREW_RELEASE_DEBOUNCE": 0.15,     # s, how long ΔFz must stay below release_level
            "SCREW_GRIPPER_ENABLED": True,         # send tool cmd=2 (grab) / cmd=3 (release); disable for no-gripper configs
            "UNSCREW_FORCE_IDEAL_DELTA": 0.15,    # N, preferred axial force increase while spinning
            "UNSCREW_FORCE_DEADBAND": 0.04,       # N, no Z relief inside this band
            "UNSCREW_Z_RELIEF_GAIN": 0.0015,      # m/s per N above ideal force while unscrewing
            "UNSCREW_Z_RELIEF_SPEED_CAP": 0.004,  # m/s, max upward relief while unscrewing
            "UNSCREW_Z_RELIEF_MAX": 0.030,        # m, max relief before gripper close
            "TOOL_RELEASE_SETTLE": 0.50,      # s, allow tool gripper to open before unscrew
            "TOOL_GRAB_SETTLE": 0.50,        # s, allow tool gripper to close before retract
            "BIN_RELEASE_HEIGHT": 0.020,     # m, release screw 2 cm above Bin 1 reference
            "BIN_Z_TRIM": -0.050,            # m, source-only trim because bin marker Z is above usable drop height
            "XARM_HOME_JOINTS": {
                "xarm5_joint1": 0.0,
                "xarm5_joint2": 0.0,
                "xarm5_joint3": -math.pi / 2,
                "xarm5_joint4": math.pi / 2,
                "xarm5_joint5": 0.0,
            },
            
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
        self.tool_status_sub = self.create_subscription(String, '/tool_status', self.tool_status_callback, 10)
        self.state_pub = self.create_publisher(String, '/robot_state/tool_arm/update', 10)
        self.tool_pub = self.create_publisher(Int8, '/tool_cmd', 10)

        # Thread Safety & State
        self.data_lock = threading.Lock()
        self.latest_targets = []
        self.local_view = {}
        self._align_detected_hole = False   # set by _get_vision when hole seen, no screw
        self.cached_bin1_xyz = None
        self._bin1_locked = False    # once set, bin1 is frozen for the session
        self.last_contact_fz_diff = 0.0
        self.tool_status = {}

        self.get_logger().info("🚀 Refactored Unscrew Skill Active (Compliance & Safety Updated).")

    # ------------------------------------------------------------------
    # State helper — use this instead of calling state_pub directly.
    # ------------------------------------------------------------------
    def _pub_state(self, state: str):
        """Publish tool-arm state to /robot_state/tool_arm/update."""
        try:
            self.state_pub.publish(String(data=state))
        except Exception:
            pass

    def _apply_unscrew_config(self, cfg, target_label=None):
        unscrew_steps = [s for s in cfg.disassembly_sequence if s.action == 'unscrew']
        if not unscrew_steps:
            return
        step = unscrew_steps[0]
        if target_label:
            for s in unscrew_steps:
                if s.target == target_label:
                    step = s
                    break
        p = step.parameters
        if 'force_threshold_n' in p:
            self.CONFIG['FORCE_THRESHOLD'] = p['force_threshold_n']
        if 'align_tolerance_px' in p:
            self.CONFIG['ALIGN_TOLERANCE_PX'] = p['align_tolerance_px']
        if 'spiral_timeout_s' in p:
            self.CONFIG['SPIRAL_TIMEOUT'] = p['spiral_timeout_s']
        key_map = {
            'transit_lift_m': 'TRANSIT_LIFT',
            'reach_limit_m': 'REACH_LIMIT',
            'reach_soft_margin_m': 'REACH_SOFT_MARGIN',
            'mm_per_px': 'MM_PER_PIX',
            'coarse_position_tolerance_m': 'COARSE_POSITION_TOLERANCE',
            'coarse_only_debug': 'COARSE_ONLY_DEBUG',
            'align_only_debug': 'ALIGN_ONLY_DEBUG',
            'descent_only_debug': 'DESCENT_ONLY_DEBUG',
            'screw_camera_z_min_m': 'SCREW_CAMERA_Z_MIN',
            'screw_camera_z_max_m': 'SCREW_CAMERA_Z_MAX',
            'screw_world_z_min_m': 'SCREW_WORLD_Z_MIN',
            'screw_world_z_max_m': 'SCREW_WORLD_Z_MAX',
            'coarse_hover_z_m': 'COARSE_HOVER_Z_OVERRIDE',
            'coarse_velocity': 'COARSE_VELOCITY',
            'pre_retract_m': 'PRE_RETRACT',
            'xy_speed_align_mps': 'XY_SPEED_ALIGN',
            'z_speed_descent_mps': 'Z_SPEED_DESCENT',
            'descent_max_distance_m': 'DESCENT_MAX_DISTANCE',
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
            'enable_spiral_search': 'ENABLE_SPIRAL_SEARCH',
            'align_rate_hz': 'ALIGN_RATE_HZ',
            'align_joint_alpha': 'ALIGN_JOINT_ALPHA',
            'align_max_joint_step_rad': 'ALIGN_MAX_JOINT_STEP_RAD',
            'align_max_joint_velocity_rad_s': 'ALIGN_MAX_JOINT_VELOCITY_RAD_S',
            'pre_descent_align_tolerance_px': 'PRE_DESCENT_ALIGN_TOLERANCE_PX',
            'pre_descent_align_timeout_s': 'PRE_DESCENT_ALIGN_TIMEOUT',
            'pre_descent_align_stable_cycles': 'PRE_DESCENT_ALIGN_STABLE_CYCLES',
            'final_align_tolerance_px': 'FINAL_ALIGN_TOLERANCE_PX',
            'final_align_timeout_s': 'FINAL_ALIGN_TIMEOUT',
            'final_align_max_radius_m': 'FINAL_ALIGN_MAX_RADIUS',
            'final_align_stable_cycles': 'FINAL_ALIGN_STABLE_CYCLES',
            'final_align_xy_speed_mps': 'FINAL_ALIGN_XY_SPEED',
            'final_align_min_speed_mps': 'FINAL_ALIGN_MIN_SPEED',
            'final_align_error_filter_alpha': 'FINAL_ALIGN_ERROR_ALPHA',
            'skip_final_align_on_contact': 'SKIP_FINAL_ALIGN_ON_CONTACT',
            'engagement_depth_mm': 'ENGAGEMENT_DEPTH_MM',
            'unscrew_probe_duration_s': 'UNSCREW_PROBE_DURATION',
            'unscrew_probe_force_increase_n': 'UNSCREW_PROBE_FORCE_INCREASE',
            'unscrew_min_spin_s': 'UNSCREW_MIN_SPIN_TIME',
            'unscrew_max_spin_s': 'UNSCREW_MAX_SPIN_TIME',
            'unscrew_release_force_ratio': 'UNSCREW_RELEASE_FORCE_RATIO',
            'unscrew_release_debounce_s': 'UNSCREW_RELEASE_DEBOUNCE',
            'use_screw_gripper': 'SCREW_GRIPPER_ENABLED',
            'unscrew_force_ideal_delta_n': 'UNSCREW_FORCE_IDEAL_DELTA',
            'unscrew_force_deadband_n': 'UNSCREW_FORCE_DEADBAND',
            'unscrew_z_relief_gain_mps_per_n': 'UNSCREW_Z_RELIEF_GAIN',
            'unscrew_z_relief_speed_cap_mps': 'UNSCREW_Z_RELIEF_SPEED_CAP',
            'unscrew_z_relief_max_m': 'UNSCREW_Z_RELIEF_MAX',
            'tool_release_settle_s': 'TOOL_RELEASE_SETTLE',
            'tool_grab_settle_s': 'TOOL_GRAB_SETTLE',
        }
        for param_name, config_name in key_map.items():
            if param_name in p:
                self.CONFIG[config_name] = p[param_name]
        if 'coarse_planner' in p:
            self.CONFIG['COARSE_PLANNER'] = str(p['coarse_planner']).lower()
        # mm-unit coarse XY offset fine-tune (stored internally in metres)
        if 'coarse_x_offset_mm' in p:
            self.CONFIG['COARSE_X_OFFSET'] = float(p['coarse_x_offset_mm']) / 1000.0
        if 'coarse_y_offset_mm' in p:
            self.CONFIG['COARSE_Y_OFFSET'] = float(p['coarse_y_offset_mm']) / 1000.0
        self.get_logger().info("Unscrew config applied from device config.")

    # =========================================================================
    # CALLBACKS & HELPERS
    # =========================================================================

    def vision_callback(self, msg):
        try:
            raw_data = msg.data.strip().strip("'").strip('"')
            data = json.loads(raw_data)
            def _has_valid_xyz(obj):
                xyz = obj.get("xyz")
                return isinstance(xyz, (list, tuple)) and len(xyz) >= 3 and all(v is not None for v in xyz[:3])
            with self.data_lock:
                self.latest_targets = [obj for obj in data.get("global_view", {}).get("objects", [])
                                     if "screw" in obj.get("label", "").lower() and _has_valid_xyz(obj)]
                self.local_view = data
        except Exception: pass

    def bin_callback(self, msg):
        try:
            raw_data = msg.data.strip().strip("'").strip('"')
            data = json.loads(raw_data)
            if "bin_1" in data and "xyz" in data["bin_1"]:
                with self.data_lock:
                    if not self._bin1_locked:
                        self.cached_bin1_xyz = data["bin_1"]["xyz"]
                        self._bin1_locked = True
                        self.get_logger().info(
                            f"[BIN] Bin-1 coordinates locked at startup: "
                            f"({self.cached_bin1_xyz[0]:.4f}, "
                            f"{self.cached_bin1_xyz[1]:.4f}, "
                            f"{self.cached_bin1_xyz[2]:.4f})"
                        )
        except Exception: pass

    def tool_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                with self.data_lock:
                    self.tool_status = data
        except Exception:
            pass

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
                    position_tolerance_m=max(0.008, float(self.CONFIG["COARSE_POSITION_TOLERANCE"])),
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
        # The long move to coarse hover must be planned. Teleop-style tracking is
        # only a short local recovery after a planner result fails verification;
        # using it as the primary long move is slow and visibly stepped.
        methods = []
        if preferred in ("exotica", "moveit", "direct", "track"):
            methods.append(preferred)
        else:
            print(f"[DIAG] Unknown COARSE_PLANNER='{preferred}', using exotica.")
            methods.append("exotica")
        for method in ("exotica", "moveit", "track"):
            if method not in methods:
                methods.append(method)

        hover_target = (tx, ty, hover_z)
        for method in methods:
            print(f"[DIAG] Coarse hover attempt using {method}.")
            if method == "track":
                approach_ok = self._stream_hover_exotica_verified(
                    hover_target,
                    timeout_s=max(18.0, float(self.CONFIG["ALIGN_ONLY_TIMEOUT"]) * 0.35),
                )
            elif method == "direct":
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
                if method in ("exotica", "moveit", "direct"):
                    print("[DIAG] Trying short EXOTica tracking recovery after planned coarse command failure.")
                    if self._stream_hover_exotica_verified(hover_target, timeout_s=8.0):
                        print("[DIAG] Coarse hover verified with EXOTica streaming recovery.")
                        return True
                continue

            if method == "track":
                print("[DIAG] Coarse hover verified with EXOTica tracking recovery.")
                return True

            self.wait_for_arm_settled(timeout=8.0)
            if self._verify_screwdriver_tcp(hover_target, self.CONFIG["COARSE_POSITION_TOLERANCE"]):
                print(f"[DIAG] Coarse hover verified with {method}.")
                return True
            print(f"[DIAG] {method} command completed but TCP verification failed.")
            print("[DIAG] Planner result disagreed with TF; trying short EXOTica tracking recovery.")
            if self._stream_hover_exotica_verified(hover_target, timeout_s=8.0):
                print("[DIAG] Coarse hover verified with EXOTica streaming recovery.")
                return True

        print(
            f"❌ Coarse xArm TCP exceeds "
            f"{self.CONFIG['COARSE_POSITION_TOLERANCE']*1000:.1f}mm after all planners. "
            "Aborting before descent/search."
        )
        return False

    # =========================================================================
    # 1. LEGACY ALIGNMENT (EXOTica real-time IK — retained for comparison only)
    # =========================================================================
    def _perform_xy_align_search_only_exotica_legacy(
        self,
        tolerance_px: float = None,
        timeout_s: float = None,
        stable_cycles: int = None,
    ):
        """EXOTica streaming XY alignment with Z locked at hover.

        Replaces the servo-based implementation. MoveIt Servo halts at near-
        singular arm configurations (arm near reach limit) via its hard-stop
        singularity threshold. EXOTica IK streaming solves full IK each cycle
        and is not limited by Jacobian singularities.

        Z is locked at the current hover height. XY correction is proportional
        to pixel error (vx ∝ -err_y, vy ∝ -err_x — both negative, matching original servo).
        When no visual target is found, a square spiral search advances the
        IK target in XY until a screw becomes visible again.
        """
        print("\n[ALIGN] Starting EXOTica XY align/search with Z locked.")
        self.wait_for_arm_settled(timeout=3.0)

        # Ensure servo is stopped so EXOTica direct-joint commands are accepted
        self.moveit_backend.stop_servo(timeout_sec=2.0)

        ee_link = "screwdriver_tcp"
        try:
            tf0 = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
            )
            locked_z = float(tf0.transform.translation.z)
            init_x   = float(tf0.transform.translation.x)
            init_y   = float(tf0.transform.translation.y)
            ref_roll, ref_pitch, ref_yaw = self.moveit_backend._quaternion_to_rpy(
                tf0.transform.rotation.x, tf0.transform.rotation.y,
                tf0.transform.rotation.z, tf0.transform.rotation.w,
            )
        except Exception as exc:
            print(f"[ALIGN] Could not read initial screwdriver_tcp: {exc}")
            return False

        rate_hz         = max(float(self.CONFIG["ALIGN_RATE_HZ"]), 1.0)
        dt              = 1.0 / rate_hz
        max_xy          = float(self.CONFIG["XY_SPEED_ALIGN"])
        min_xy          = min(float(self.CONFIG["ALIGN_MIN_SPEED"]), max_xy)
        full_spd_err    = max(float(self.CONFIG["ALIGN_FULL_SPEED_ERROR_PX"]), 1.0)
        tol_px          = float(tolerance_px if tolerance_px is not None else self.CONFIG["ALIGN_TOLERANCE_PX"])
        alpha_xy        = max(0.05, min(1.0, float(self.CONFIG["ALIGN_ERROR_FILTER_ALPHA"])))
        joint_alpha     = max(0.05, min(1.0, float(self.CONFIG["ALIGN_JOINT_ALPHA"])))
        align_stable    = int(stable_cycles if stable_cycles is not None else self.CONFIG["ALIGN_STABLE_CYCLES"])
        align_timeout   = (
            float(timeout_s)
            if timeout_s is not None
            else max(float(self.CONFIG["ALIGN_ONLY_TIMEOUT"]), float(self.CONFIG["SPIRAL_TIMEOUT"]))
        )
        spiral_speed    = max(0.0005, min(float(self.CONFIG["SPIRAL_SPEED"]), 0.003))
        spiral_gap_mm   = float(self.CONFIG["SPIRAL_GAP_MM"])
        spiral_start_mm = float(self.CONFIG["SPIRAL_START_DIST_MM"])
        spiral_max_r    = float(self.CONFIG["VISUAL_SERVO_MAX_RADIUS"])  # hard bound on search
        verify_tol      = float(self.CONFIG["ALIGN_VERIFY_TOLERANCE"])
        enable_spiral_search = bool(self.CONFIG.get("ENABLE_SPIRAL_SEARCH", False))
        max_joint_step = min(
            max(0.001, float(self.CONFIG.get("ALIGN_MAX_JOINT_STEP_RAD", 0.02))),
            max(0.001, float(self.CONFIG.get("ALIGN_MAX_JOINT_VELOCITY_RAD_S", 0.6)) / rate_hz),
        )
        max_joint_delta = max(0.08, max_joint_step * 4.0)
        LOG_INTERVAL_S  = 0.5

        def _axis_speed(error_px: float) -> float:
            abs_err = abs(error_px)
            if abs_err <= tol_px:
                return 0.0
            ratio = min(abs_err / full_spd_err, 1.0)
            speed = min_xy + (max_xy - min_xy) * ratio
            # Fast when far away, but deliberately cap near center so the
            # crosshair does not overshoot once the detector is within a few
            # dozen pixels.
            if abs_err <= 20.0:
                speed = min(speed, max(min_xy, 0.0120))
            elif abs_err <= 45.0:
                speed = min(speed, max(min_xy, 0.0240))
            elif abs_err <= 80.0:
                speed = min(speed, max(min_xy, 0.0400))
            return math.copysign(speed, error_px)

        def _candidate_center(det):
            center = det.get("center") or det.get("contact_point")
            if center and len(center) >= 2:
                return [float(center[0]), float(center[1])]
            box = det.get("box")
            if box and len(box) >= 4:
                return [0.5 * (float(box[0]) + float(box[2])), 0.5 * (float(box[1]) + float(box[3]))]
            return [320.0, 240.0]

        # Locked pixel anchor: once we latch onto a screw on the first visible
        # frame, subsequent frames pick the candidate nearest that locked pixel
        # rather than nearest the crosshair.  This prevents the servo from
        # jumping to a neighbouring screw that drifts closer to centre while
        # the arm is already tracking the correct one.
        locked_pixel = [None]   # [px, py] once latched; None until first detection

        def _iou(a, b):
            ax1,ay1,ax2,ay2 = float(a[0]),float(a[1]),float(a[2]),float(a[3])
            bx1,by1,bx2,by2 = float(b[0]),float(b[1]),float(b[2]),float(b[3])
            ix1,iy1 = max(ax1,bx1), max(ay1,by1)
            ix2,iy2 = min(ax2,bx2), min(ay2,by2)
            iw,ih = max(0.0,ix2-ix1), max(0.0,iy2-iy1)
            inter = iw*ih
            if inter == 0.0: return 0.0
            union = max(0.0,ax2-ax1)*max(0.0,ay2-ay1) + max(0.0,bx2-bx1)*max(0.0,by2-by1) - inter
            return inter/union if union > 0 else 0.0

        def _hole_overlaps_screw(screw_box, holes, iou_thresh=0.10):
            """True if any hole bbox overlaps the screw bbox by IoU > iou_thresh,
            OR if any hole center falls inside the screw bbox.
            Catches co-located detections even when hole center is slightly outside."""
            for h in (holes or []):
                hb = h.get("box")
                if not hb or len(hb) < 4:
                    continue
                # Center-in-box check
                hcx = (hb[0] + hb[2]) / 2.0
                hcy = (hb[1] + hb[3]) / 2.0
                if screw_box[0] <= hcx <= screw_box[2] and screw_box[1] <= hcy <= screw_box[3]:
                    return True
                # IoU overlap check
                if _iou(screw_box, hb) >= iou_thresh:
                    return True
            return False

        def _get_vision():
            with self.data_lock:
                lv = copy.deepcopy(self.local_view).get("local_view", {})
            crosshair  = lv.get("crosshair") or [320, 240]
            candidates = lv.get("screw_heads", [])
            label      = "screw_head"
            if not candidates:
                holes = lv.get("holes", [])
                raw_screws = lv.get("screws", [])
                # Filter screws that overlap with a stable hole detection
                candidates = [
                    s for s in raw_screws
                    if not _hole_overlaps_screw(s.get("box") or [], holes)
                ]
                suppressed = len(raw_screws) - len(candidates)
                if suppressed:
                    print(f"[ALIGN] Suppressed {suppressed} hole-overlapping screw detection(s).")
                # If holes are visible but screws are gone (stabilizer killed fluctuating
                # detections), signal HOLE so the align loop aborts cleanly instead of
                # spiral-searching around an already-removed position.
                # Exception: if any hole is inside a raw screw's bounding box, the
                # screw may be partially removed or mis-detected — defer to spiral
                # search (10 s cap) instead of aborting immediately.
                if not candidates and holes:
                    hole_inside_screw_box = raw_screws and any(
                        _hole_overlaps_screw(s.get("box") or [], holes)
                        for s in raw_screws
                    )
                    if hole_inside_screw_box:
                        if hole_spiral_deadline[0] == 0.0:
                            hole_spiral_deadline[0] = time.time() + HOLE_SPIRAL_TIMEOUT_S
                            print(
                                f"[ALIGN] Hole inside screw bbox — deferring to spiral search "
                                f"(timeout {HOLE_SPIRAL_TIMEOUT_S:.0f}s)."
                            )
                        return None, None, crosshair, False, lv
                    best_hole = holes[0]
                    best_hole = dict(best_hole)
                    best_hole["center"] = _candidate_center(best_hole)
                    print("[ALIGN] Hole visible, no stable screw — signalling HOLE to abort alignment.")
                    self._align_detected_hole = True
                    return "hole", best_hole, crosshair, False, lv
                label = "screw"
            if candidates:
                # First frame: pick nearest to crosshair (best proxy for the
                # coarse-hover target) and lock that pixel as the anchor.
                # Subsequent frames: pick nearest to the locked pixel so we
                # keep tracking the same physical screw even if another one
                # drifts closer to image centre.
                anchor = locked_pixel[0] if locked_pixel[0] is not None else crosshair
                det = self._nearest_detection_to_crosshair(candidates, anchor)
                if det is not None:
                    det = dict(det)
                    det["center"] = _candidate_center(det)
                    if locked_pixel[0] is None:
                        locked_pixel[0] = list(det["center"])
                        print(
                            f"[ALIGN] Locked onto screw at pixel "
                            f"({locked_pixel[0][0]:.0f}, {locked_pixel[0][1]:.0f}) "
                            f"— will track this detection for the full alignment."
                        )
                    return label, det, crosshair, False, lv
            return None, None, crosshair, False, lv

        # Mutable state shared across closures
        target_xy     = [init_x, init_y]   # running IK target position (XY only)
        stable_cnt    = [0]
        no_target_cnt = [0]                # debounce: frames without a vision target
        aligned_flag  = [False]
        abort_flag    = [False]
        abort_reason  = [""]
        prev_vx_s     = [0.0]
        prev_vy_s     = [0.0]
        axis_state = {
            "image_y_base_x": {"sign": -1.0, "last_abs_err": None, "last_t": 0.0},
            "image_x_base_y": {"sign": -1.0, "last_abs_err": None, "last_t": 0.0},
        }

        # Spiral state (resets when target is found)
        spiral_on          = [False]
        spiral_side_idx    = [0]
        spiral_side_len_mm = [spiral_start_mm]
        spiral_remain_m    = [spiral_start_mm / 1000.0]
        spiral_center      = [init_x, init_y]  # updated to target_xy when spiral starts

        # Hole-overlap spiral: when a hole is detected inside a screw bbox,
        # defer to spiral search rather than aborting; cap at 10 s.
        HOLE_SPIRAL_TIMEOUT_S   = 10.0
        hole_spiral_deadline    = [0.0]   # 0.0 = not yet triggered

        last_log_t = [time.time()]
        start_t    = time.time()

        def stop_fn():
            if not rclpy.ok():
                return True
            if aligned_flag[0]:
                return True
            if abort_flag[0]:
                print(f"\n[ALIGN] {abort_reason[0]}")
                return True
            if time.time() - start_t >= align_timeout:
                print("\n[ALIGN] Timeout reached.")
                return True
            # Hole-overlap spiral timeout
            if hole_spiral_deadline[0] > 0.0 and time.time() >= hole_spiral_deadline[0]:
                print("\n[ALIGN] Hole-overlap spiral search timed out (10s) — aborting.")
                self._align_detected_hole = True
                return True
            # Z drift guard
            try:
                tf = self.moveit_backend.tf_buffer.lookup_transform(
                    self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
                )
                z_err = abs(float(tf.transform.translation.z) - locked_z)
                if z_err > verify_tol:
                    print(f"\n[ALIGN] Z drift {z_err*1000:.1f}mm — aborting.")
                    return True
            except Exception:
                pass
            return False

        def target_fn():
            # Get current EE position
            try:
                tf = self.moveit_backend.tf_buffer.lookup_transform(
                    self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
                )
                ex = float(tf.transform.translation.x)
                ey = float(tf.transform.translation.y)
                ez = float(tf.transform.translation.z)
            except Exception:
                return None

            current_r = math.hypot(ex - init_x, ey - init_y)
            if current_r > spiral_max_r:
                abort_flag[0] = True
                abort_reason[0] = (
                    f"XY radius {current_r*1000:.1f}mm exceeded "
                    f"{spiral_max_r*1000:.1f}mm fine-align limit — aborting."
                )
                return None

            t_class, t_det, crosshair, using_stale_target, local_snapshot = _get_vision()

            # Hole detected and no stable screw — abort alignment cleanly
            if t_class == "hole":
                abort_flag[0] = True
                abort_reason[0] = "HOLE_DETECTED"
                return None

            if t_det is not None:
                # Target visible: visual servo XY correction
                no_target_cnt[0] = 0
                if spiral_on[0]:
                    print("[ALIGN] Target regained — resuming visual servo.")
                    spiral_on[0]          = False
                    spiral_side_idx[0]    = 0
                    spiral_side_len_mm[0] = spiral_start_mm
                    spiral_remain_m[0]    = spiral_start_mm / 1000.0
                    spiral_center[0]      = init_x  # reset so next spiral re-anchors
                    spiral_center[1]      = init_y
                    prev_vx_s[0]          = 0.0
                    prev_vy_s[0]          = 0.0

                center = t_det.get("center", [320, 240])
                err_x  = float(crosshair[0] - center[0])
                err_y  = float(crosshair[1] - center[1])
                dist_px = math.hypot(err_x, err_y)
                stale_suffix = " stale" if using_stale_target else ""

                if max(abs(err_x), abs(err_y)) <= tol_px:
                    stable_cnt[0] += 1
                    for state in axis_state.values():
                        state["last_abs_err"] = None
                    log_str = (
                        f"[ALIGN/{t_class}{stale_suffix}] STABLE {stable_cnt[0]}/{align_stable} "
                        f"ErrX={err_x:.1f} ErrY={err_y:.1f}px"
                    )
                    if stable_cnt[0] >= align_stable:
                        print(f"\n[ALIGN] Aligned on {t_class}.")
                        aligned_flag[0] = True
                        return None  # signal done
                    # Hold current position while counting stable cycles
                    target_xy[0] = ex
                    target_xy[1] = ey
                else:
                    stable_cnt[0] = 0
                    # Local-camera-only coordinate descent:
                    # - image Y error is corrected by base X motion
                    # - image X error is corrected by base Y motion
                    # Only move the dominant pixel axis each cycle. Keep the
                    # calibrated signs fixed; online sign flipping can reverse
                    # a correct approach when detector noise briefly grows.
                    now = time.time()
                    if abs(err_y) >= abs(err_x):
                        axis_key = "image_y_base_x"
                        axis_label = "imgY->baseX"
                        axis_err = err_y
                    else:
                        axis_key = "image_x_base_y"
                        axis_label = "imgX->baseY"
                        axis_err = err_x

                    state = axis_state[axis_key]

                    if axis_key == "image_y_base_x":
                        tvx = float(state["sign"]) * _axis_speed(axis_err)
                        tvy = 0.0
                    else:
                        tvx = 0.0
                        tvy = float(state["sign"]) * _axis_speed(axis_err)

                    vx = alpha_xy * tvx + (1.0 - alpha_xy) * prev_vx_s[0]
                    vy = alpha_xy * tvy + (1.0 - alpha_xy) * prev_vy_s[0]
                    prev_vx_s[0] = vx
                    prev_vy_s[0] = vy
                    state["last_abs_err"] = abs(axis_err)
                    state["last_t"] = now
                    new_x = ex + vx * dt
                    new_y = ey + vy * dt
                    # Bound visual servo travel to spiral_max_r from init so the arm
                    # cannot overshoot the screw region and lose the camera view.
                    servo_r = math.hypot(new_x - init_x, new_y - init_y)
                    if servo_r <= spiral_max_r:
                        target_xy[0] = new_x
                        target_xy[1] = new_y
                    else:
                        abort_flag[0] = True
                        abort_reason[0] = (
                            f"Commanded XY radius {servo_r*1000:.1f}mm would exceed "
                            f"{spiral_max_r*1000:.1f}mm fine-align limit — aborting."
                        )
                        return None
                    log_str = (
                        f"[ALIGN/{t_class}{stale_suffix}/{axis_label}] "
                        f"ErrX={err_x:5.1f} ErrY={err_y:5.1f} dist={dist_px:.0f}px "
                        f"cmd=({vx*1000:.1f},{vy*1000:.1f})mm/s sign={state['sign']:+.0f} "
                        f"r={servo_r*1000:.1f}mm Z={ez*1000:.1f}mm"
                    )
            else:
                # No target: debounce before entering spiral search.
                # Vision runs ~5 Hz; the 15 Hz control loop will see several
                # stale/empty frames between real vision updates.  Hold position
                # for the first 4 consecutive no-target frames (~267 ms) before
                # triggering the spiral so transient detection gaps don't kick
                # the arm away from the screw.
                stable_cnt[0]    = 0
                prev_vx_s[0]     = 0.0
                prev_vy_s[0]     = 0.0
                for state in axis_state.values():
                    state["last_abs_err"] = None
                no_target_cnt[0] += 1

                # If we already locked onto a pixel, tolerate more misses before
                # spiralling — fluctuating detections would otherwise oscillate
                # between servo-mode and spiral-mode and never converge.
                no_target_thresh = 12 if locked_pixel[0] is not None else 5
                if no_target_cnt[0] < no_target_thresh or not enable_spiral_search:
                    # Hold: vision is lagging, wait for the next real frame
                    target_xy[0] = ex
                    target_xy[1] = ey
                    screws_n = len(local_snapshot.get("screws", []) or [])
                    heads_n = len(local_snapshot.get("screw_heads", []) or [])
                    log_str = (
                        f"[ALIGN] No local screw target heads={heads_n} screws={screws_n} "
                        f"miss={no_target_cnt[0]}/{'12' if locked_pixel[0] else '5'} "
                        f"— holding Z={ez*1000:.1f}mm"
                    )
                else:
                    if not spiral_on[0]:
                        print("[ALIGN] No target — starting spiral search.")
                        spiral_on[0]      = True
                        # Anchor the radius bound here, not at init.  Visual servo
                        # may have already moved the arm toward the screw before
                        # losing detection; spiral should search around THAT position.
                        spiral_center[0]  = target_xy[0]
                        spiral_center[1]  = target_xy[1]

                    step = spiral_speed * dt
                    if spiral_remain_m[0] <= 0.0:
                        spiral_side_idx[0] += 1
                        if spiral_side_idx[0] % 2 == 0:
                            spiral_side_len_mm[0] += spiral_gap_mm
                        spiral_remain_m[0] = spiral_side_len_mm[0] / 1000.0

                    idx = spiral_side_idx[0] % 4
                    if   idx == 0: sdx, sdy, sname = -step, 0.0,  "-X"
                    elif idx == 1: sdx, sdy, sname =  0.0, -step, "-Y"
                    elif idx == 2: sdx, sdy, sname =  step, 0.0,  "+X"
                    else:          sdx, sdy, sname =  0.0,  step, "+Y"

                    # Hard radius bound around the spiral center (captured when spiral
                    # first starts), not around init.  Visual servo may have already
                    # moved the arm toward the screw; spiral searches around that spot.
                    new_x = target_xy[0] + sdx
                    new_y = target_xy[1] + sdy
                    dist_from_center = math.hypot(
                        new_x - spiral_center[0], new_y - spiral_center[1]
                    )
                    if dist_from_center <= spiral_max_r:
                        target_xy[0] = new_x
                        target_xy[1] = new_y
                    # else: hold at current target; spiral side counter still advances
                    spiral_remain_m[0] -= step
                    log_str = (
                        f"[ALIGN/spiral] dir={sname} side={spiral_side_len_mm[0]:.1f}mm "
                        f"r={dist_from_center*1000:.1f}mm/max={spiral_max_r*1000:.0f}mm Z={ez*1000:.1f}mm"
                    )

            now = time.time()
            if now - last_log_t[0] >= LOG_INTERVAL_S:
                print(f"  {log_str}")
                last_log_t[0] = now

            return (target_xy[0], target_xy[1], locked_z, ref_roll, ref_pitch, ref_yaw)

        result = self.moveit_backend.move_cartesian_realtime_exotica(
            target_fn,
            stop_fn=stop_fn,
            rate_hz=rate_hz,
            max_step_m=max(max_xy * dt, 0.0005),
            joint_smooth_alpha=joint_alpha,
            max_joint_delta_rad=max_joint_delta,
            max_joint_step_rad=max_joint_step,
            timeout_s=align_timeout + 5.0,
        )

        if result in ("DONE", "STOPPED"):
            if aligned_flag[0]:
                return True
            if abort_flag[0]:
                print(f"[ALIGN] Aborted: {abort_reason[0]}")
                return False
            print("[ALIGN] Stopped without alignment.")
            return False
        print(f"\n[ALIGN] EXOTica returned {result}.")
        return False

    # =========================================================================
    # SAFETY RETRACT
    # =========================================================================
    def _safe_servo_lift_z(self, distance_m: float = None, speed_mps: float = None, timeout_s: float = None) -> bool:
        """Lift screwdriver_tcp vertically using MoveIt Servo only.

        On the current xArm5 Servo setup, negative Z twist increases the
        screwdriver_tcp Z position in base_link. This avoids EXOTica replanning
        during failure recovery, where IK branch changes can add lateral motion.
        """
        ee_link = "screwdriver_tcp"
        distance = abs(float(distance_m if distance_m is not None else self.CONFIG["TRANSIT_LIFT"]))
        speed = abs(float(speed_mps if speed_mps is not None else self.CONFIG["RETRACT_SPEED"]))
        speed = max(min(speed, 0.12), 0.006)
        timeout = float(timeout_s) if timeout_s is not None else max(45.0, (distance / speed) * 8.0)

        try:
            tf0 = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
            )
            start_x = float(tf0.transform.translation.x)
            start_y = float(tf0.transform.translation.y)
            start_z = float(tf0.transform.translation.z)
        except Exception as exc:
            print(f"[SAFETY] Could not read {ee_link} before lift: {exc}")
            return False

        target_z = start_z + distance
        print(
            f"[SAFETY] Servo lifting {distance*1000:.1f}mm from Z={start_z*1000:.1f}mm "
            f"to {target_z*1000:.1f}mm."
        )

        if not self.moveit_backend.start_servo(timeout_sec=8.0):
            print("[SAFETY] Failed to start MoveIt Servo for lift.")
            return False

        rate_hz = max(float(self.CONFIG["ALIGN_RATE_HZ"]), 20.0)
        dt = 1.0 / rate_hz
        deadline = time.time() + timeout
        max_lateral_drift = 0.008
        last_log_t = 0.0

        try:
            while rclpy.ok() and time.time() < deadline:
                try:
                    tf = self.moveit_backend.tf_buffer.lookup_transform(
                        self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
                    )
                    x = float(tf.transform.translation.x)
                    y = float(tf.transform.translation.y)
                    z = float(tf.transform.translation.z)
                except Exception as exc:
                    print(f"\n[SAFETY] Could not read {ee_link} during lift: {exc}")
                    return False

                if z >= target_z - 0.001:
                    self.moveit_backend._publish_zero_twist()
                    print(f"\n[SAFETY] Lift complete at Z={z*1000:.1f}mm.")
                    return True

                drift = math.hypot(x - start_x, y - start_y)
                if drift > max_lateral_drift:
                    self.moveit_backend._publish_zero_twist()
                    print(
                        f"\n[SAFETY] Lift aborted: lateral drift {drift*1000:.1f}mm "
                        f"exceeded {max_lateral_drift*1000:.1f}mm."
                    )
                    return False

                if not self.moveit_backend.publish_servo_velocity(0.0, 0.0, -speed):
                    print("\n[SAFETY] Servo lift command failed.")
                    return False
                now = time.time()
                if now - last_log_t >= 1.0:
                    print(
                        f"  [SAFETY] lifting Z={z*1000:.1f}/{target_z*1000:.1f}mm "
                        f"cmd_z={-speed*1000:.1f}mm/s drift={drift*1000:.1f}mm"
                    )
                    last_log_t = now
                time.sleep(dt)

            self.moveit_backend._publish_zero_twist()
            print("\n[SAFETY] Lift timed out before reaching target Z.")
            return False
        finally:
            try:
                self.moveit_backend._publish_zero_twist()
                self.moveit_backend.stop_servo(timeout_sec=3.0)
            except Exception as exc:
                print(f"[SAFETY] Servo stop after lift failed: {exc}")

    # =========================================================================
    # 1. ALIGNMENT (local-camera MoveIt Servo, Z locked)
    # =========================================================================
    def perform_xy_align_search_only(
        self,
        tolerance_px: float = None,
        timeout_s: float = None,
        max_radius_m: float = None,
        stable_cycles: int = None,
        max_xy_mps: float = None,
        min_xy_mps: float = None,
        error_alpha: float = None,
        screw_head_only: bool = False,
        enable_spiral: bool = None,
    ):
        """Fine XY alignment from the local tool camera only.

        Global vision is only for the coarse hover. During fine alignment,
        missing local tool-camera detections command zero velocity. MoveIt Servo
        is used here because EXOTica pose streaming was observed to drift the
        TCP even when target XY was held constant on the 5-DOF xArm.
        """
        if tolerance_px is None and timeout_s is None and max_radius_m is None and stable_cycles is None:
            # Use the continuous EXOTica implementation for normal unscrew flow.
            # The planned-step fallback is reliable but visibly stop/start; the
            # realtime EXOTica target loop streams small XY updates like the
            # smooth object-hold descent and flip retract paths.
            return self._perform_xy_align_search_only_exotica_legacy()

        print("\n[ALIGN] Starting local-camera MoveIt Servo XY alignment with Z locked.")
        self.wait_for_arm_settled(timeout=3.0)

        ee_link = "screwdriver_tcp"
        try:
            tf0 = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
            )
            locked_z = float(tf0.transform.translation.z)
            init_x = float(tf0.transform.translation.x)
            init_y = float(tf0.transform.translation.y)
        except Exception as exc:
            print(f"[ALIGN] Could not read initial {ee_link}: {exc}")
            return False

        rate_hz = max(float(self.CONFIG["ALIGN_RATE_HZ"]), 1.0)
        dt = 1.0 / rate_hz
        max_xy = float(max_xy_mps if max_xy_mps is not None else self.CONFIG["XY_SPEED_ALIGN"])
        min_xy = min(float(min_xy_mps if min_xy_mps is not None else self.CONFIG["ALIGN_MIN_SPEED"]), max_xy)
        full_spd_err = max(float(self.CONFIG["ALIGN_FULL_SPEED_ERROR_PX"]), 1.0)
        tol_px = float(tolerance_px if tolerance_px is not None else self.CONFIG["ALIGN_TOLERANCE_PX"])
        alpha_xy = max(0.05, min(1.0, float(error_alpha if error_alpha is not None else self.CONFIG["ALIGN_ERROR_FILTER_ALPHA"])))
        align_stable = int(stable_cycles if stable_cycles is not None else self.CONFIG["ALIGN_STABLE_CYCLES"])
        align_timeout = (
            float(timeout_s)
            if timeout_s is not None
            else max(float(self.CONFIG["ALIGN_ONLY_TIMEOUT"]), float(self.CONFIG["SPIRAL_TIMEOUT"]))
        )
        max_radius = float(max_radius_m if max_radius_m is not None else self.CONFIG["VISUAL_SERVO_MAX_RADIUS"])
        verify_tol = float(self.CONFIG["ALIGN_VERIFY_TOLERANCE"])
        z_lock_gain = float(self.CONFIG["ALIGN_Z_LOCK_GAIN"])
        z_lock_speed = float(self.CONFIG["ALIGN_Z_LOCK_SPEED"])
        enable_spiral_search = bool(enable_spiral if enable_spiral is not None else self.CONFIG.get("ENABLE_SPIRAL_SEARCH", True))
        spiral_speed = max(0.0005, min(float(self.CONFIG["SPIRAL_SPEED"]), max_xy))
        spiral_gap_mm = float(self.CONFIG["SPIRAL_GAP_MM"])
        spiral_start_mm = float(self.CONFIG["SPIRAL_START_DIST_MM"])
        log_interval = 0.5

        def _axis_speed(error_px: float) -> float:
            abs_err = abs(error_px)
            if abs_err <= tol_px:
                return 0.0
            ratio = min(abs_err / full_spd_err, 1.0)
            speed = min_xy + (max_xy - min_xy) * ratio
            # Fast when far away, but deliberately cap near center so the
            # crosshair does not overshoot once the detector is within a few
            # dozen pixels.
            if abs_err <= 20.0:
                speed = min(speed, max(min_xy, 0.0120))
            elif abs_err <= 45.0:
                speed = min(speed, max(min_xy, 0.0240))
            elif abs_err <= 80.0:
                speed = min(speed, max(min_xy, 0.0400))
            return math.copysign(speed, error_px)

        def _candidate_center(det):
            center = det.get("center") or det.get("contact_point")
            if center and len(center) >= 2:
                return [float(center[0]), float(center[1])]
            box = det.get("box")
            if box and len(box) >= 4:
                return [0.5 * (float(box[0]) + float(box[2])), 0.5 * (float(box[1]) + float(box[3]))]
            return [320.0, 240.0]

        def _choose_detection(candidates, crosshair, preferred_center=None):
            normalized = []
            for det in candidates or []:
                d = dict(det)
                d["center"] = _candidate_center(d)
                normalized.append(d)
            if not normalized:
                return None
            if preferred_center is not None:
                close = min(
                    normalized,
                    key=lambda d: math.hypot(d["center"][0] - preferred_center[0], d["center"][1] - preferred_center[1]),
                )
                if math.hypot(close["center"][0] - preferred_center[0], close["center"][1] - preferred_center[1]) <= 90.0:
                    return close
            return self._nearest_detection_to_crosshair(normalized, crosshair)

        def _get_vision():
            with self.data_lock:
                lv = copy.deepcopy(self.local_view).get("local_view", {})
            crosshair = lv.get("crosshair") or [320, 240]
            heads = lv.get("screw_heads", []) or []
            screws = lv.get("screws", []) or []
            # Priority rule: if screw_head is visible, always use screw_head.
            # If it is not visible, fall back to screw. Within the chosen class,
            # prefer continuity with the tracked center to avoid hopping between
            # nearby screw-like detections.
            if heads:
                preferred = tracked_center if tracked_label == "screw_head" else None
                det = _choose_detection(heads, crosshair, preferred)
                if det is not None:
                    return "screw_head", det, crosshair, lv
            if screw_head_only:
                return None, None, crosshair, lv
            if screws:
                preferred = tracked_center if tracked_label == "screw" else None
                det = _choose_detection(screws, crosshair, preferred)
                if det is not None:
                    return "screw", det, crosshair, lv
            return None, None, crosshair, lv

        def _get_tcp():
            tf = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
            )
            return (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                float(tf.transform.translation.z),
            )

        servo_active = False

        def _ensure_servo_active():
            nonlocal servo_active
            if servo_active:
                return True
            if not self.moveit_backend.start_servo(timeout_sec=8.0):
                print("[ALIGN] Failed to start MoveIt Servo.")
                return False
            servo_active = True
            return True

        def _stop_servo_if_active():
            nonlocal servo_active
            if not servo_active:
                return
            self.moveit_backend._publish_zero_twist()
            self.moveit_backend.stop_servo(timeout_sec=3.0)
            servo_active = False

        def _planned_spiral_step(dx, dy):
            """Move screwdriver_tcp in a small planned XY step while no local target is visible."""
            _stop_servo_if_active()
            try:
                sx, sy, _ = _get_tcp()
            except Exception as exc:
                print(f"\n[ALIGN] Could not read {ee_link} before planned spiral step: {exc}")
                return False, 0.0

            tx = sx + float(dx)
            ty = sy + float(dy)
            projected_from_start = math.hypot(tx - init_x, ty - init_y)
            projected_from_center = math.hypot(tx - spiral_center_x, ty - spiral_center_y)
            if projected_from_start > max_radius or projected_from_center > max_radius:
                print(
                    f"\n[ALIGN] Spiral radius limit reached: "
                    f"from_start={projected_from_start*1000:.1f}mm "
                    f"from_center={projected_from_center*1000:.1f}mm "
                    f"limit={max_radius*1000:.1f}mm."
                )
                return False, 0.0

            ok = self.moveit_backend.move_to_pose_exotica(
                tx,
                ty,
                locked_z,
                velocity=max(0.08, min(float(self.CONFIG["COARSE_VELOCITY"]), 0.60)),
            )
            if not ok:
                print("\n[ALIGN] Planned spiral step failed.")
                return False, 0.0
            self.wait_for_arm_settled(timeout=3.0)
            try:
                ex2, ey2, _ = _get_tcp()
            except Exception:
                return True, math.hypot(dx, dy)
            actual_step = math.hypot(ex2 - sx, ey2 - sy)
            requested_step = math.hypot(dx, dy)
            if actual_step < min(0.0005, 0.5 * requested_step):
                print(
                    f"\n[ALIGN] Planned spiral step produced no TCP motion "
                    f"(requested={requested_step*1000:.1f}mm actual={actual_step*1000:.1f}mm)."
                )
                return False, actual_step
            return True, actual_step

        # Hardware-validated local camera mapping:
        #   negative base X reduces positive image Y error
        #   positive base Y reduces positive image X error
        prev_vx = 0.0
        prev_vy = 0.0
        stable_cnt = 0
        no_target_cnt = 0
        tracked_center = None
        tracked_label = None
        spiral_on = False
        spiral_side_idx = 0
        spiral_side_len_mm = spiral_start_mm
        spiral_remain_m = spiral_start_mm / 1000.0
        spiral_center_x = init_x
        spiral_center_y = init_y
        last_log_t = 0.0
        start_t = time.time()

        try:
            while rclpy.ok() and time.time() - start_t < align_timeout:
                try:
                    ex, ey, ez = _get_tcp()
                except Exception as exc:
                    print(f"\n[ALIGN] Could not read {ee_link}: {exc}")
                    return False

                radius = math.hypot(ex - init_x, ey - init_y)
                if radius > max_radius:
                    if servo_active:
                        self.moveit_backend._publish_zero_twist()
                    print(
                        f"\n[ALIGN] Aborted: XY radius {radius*1000:.1f}mm exceeded "
                        f"{max_radius*1000:.1f}mm fine-align limit."
                    )
                    return False

                z_err = locked_z - ez
                if abs(z_err) > verify_tol:
                    if servo_active:
                        self.moveit_backend._publish_zero_twist()
                    print(f"\n[ALIGN] Aborted: Z drift {abs(z_err)*1000:.1f}mm exceeds tolerance.")
                    return False

                t_class, t_det, crosshair, local_snapshot = _get_vision()
                if t_det is None:
                    stable_cnt = 0
                    prev_vx = 0.0
                    prev_vy = 0.0
                    no_target_cnt += 1
                    tracked_center = None
                    tracked_label = None
                    z_err = locked_z - ez
                    vz = max(min(-z_err * z_lock_gain, z_lock_speed), -z_lock_speed)
                    screws_n = len(local_snapshot.get("screws", []) or [])
                    heads_n = len(local_snapshot.get("screw_heads", []) or [])

                    if no_target_cnt < 5 or not enable_spiral_search:
                        if servo_active:
                            self.moveit_backend._publish_zero_twist()
                        now = time.time()
                        if now - last_log_t >= log_interval:
                            print(
                                f"  [ALIGN] No live local screw target heads={heads_n} screws={screws_n} "
                                f"miss={no_target_cnt} — holding r={radius*1000:.1f}mm Z={ez*1000:.1f}mm"
                            )
                            last_log_t = now
                        time.sleep(dt)
                        continue

                    if not spiral_on:
                        print("[ALIGN] No target — starting bounded square spiral search.")
                        spiral_on = True
                        spiral_side_idx = 0
                        spiral_side_len_mm = spiral_start_mm
                        spiral_remain_m = spiral_start_mm / 1000.0
                        spiral_center_x = ex
                        spiral_center_y = ey

                    if spiral_remain_m <= 0.0:
                        spiral_side_idx += 1
                        if spiral_side_idx % 2 == 0:
                            spiral_side_len_mm += spiral_gap_mm
                        spiral_remain_m = spiral_side_len_mm / 1000.0

                    idx = spiral_side_idx % 4
                    if idx == 0:
                        vx, vy, direction = -spiral_speed, 0.0, "-X"
                    elif idx == 1:
                        vx, vy, direction = 0.0, -spiral_speed, "-Y"
                    elif idx == 2:
                        vx, vy, direction = spiral_speed, 0.0, "+X"
                    else:
                        vx, vy, direction = 0.0, spiral_speed, "+Y"

                    planned_step = min(spiral_remain_m, max(0.002, spiral_speed * 1.0))
                    step_x = math.copysign(planned_step, vx) if abs(vx) > 0.0 else 0.0
                    step_y = math.copysign(planned_step, vy) if abs(vy) > 0.0 else 0.0
                    ok, actual_step = _planned_spiral_step(step_x, step_y)
                    if not ok:
                        return False

                    spiral_remain_m = max(0.0, spiral_remain_m - max(actual_step, planned_step))
                    now = time.time()
                    if now - last_log_t >= log_interval:
                        try:
                            lx, ly, lz = _get_tcp()
                            radius = math.hypot(lx - init_x, ly - init_y)
                            ez = lz
                        except Exception:
                            pass
                        print(
                            f"  [ALIGN/spiral] No target heads={heads_n} screws={screws_n} "
                            f"miss={no_target_cnt} dir={direction} side={spiral_side_len_mm:.1f}mm "
                            f"step=({step_x*1000:.1f},{step_y*1000:.1f})mm "
                            f"r={radius*1000:.1f}mm Z={ez*1000:.1f}mm"
                        )
                        last_log_t = now
                    continue

                no_target_cnt = 0
                if spiral_on:
                    print("[ALIGN] Target regained — stopping spiral and resuming visual servo.")
                    spiral_on = False
                now = time.time()
                center = t_det.get("center", [320, 240])
                if tracked_center is not None:
                    jump_px = math.hypot(center[0] - tracked_center[0], center[1] - tracked_center[1])
                    # screw fallback detections can jump between nearby screw-like
                    # objects. Reject those jumps instead of chasing a different
                    # target after the screw_head detector drops out.
                    if t_class == "screw_head" and tracked_label != "screw_head":
                        # Upgrade to screw_head whenever it is visible. The head
                        # is the preferred alignment target even if it appears a
                        # few dozen pixels from the fallback screw center.
                        pass
                    elif t_class != tracked_label and jump_px > 45.0:
                        if servo_active:
                            self.moveit_backend._publish_zero_twist()
                        stable_cnt = 0
                        prev_vx = 0.0
                        prev_vy = 0.0
                        if now - last_log_t >= log_interval:
                            print(
                                f"  [ALIGN] Rejecting {t_class} jump {jump_px:.0f}px "
                                f"from tracked {tracked_label}; holding."
                            )
                            last_log_t = now
                        time.sleep(dt)
                        continue
                    alpha_target = 0.35 if t_class == tracked_label else 0.15
                    center = [
                        alpha_target * float(center[0]) + (1.0 - alpha_target) * float(tracked_center[0]),
                        alpha_target * float(center[1]) + (1.0 - alpha_target) * float(tracked_center[1]),
                    ]
                tracked_center = [float(center[0]), float(center[1])]
                tracked_label = t_class
                err_x = float(crosshair[0] - center[0])
                err_y = float(crosshair[1] - center[1])
                dist_px = math.hypot(err_x, err_y)

                if max(abs(err_x), abs(err_y)) <= tol_px:
                    if servo_active:
                        self.moveit_backend._publish_zero_twist()
                    stable_cnt += 1
                    if time.time() - last_log_t >= log_interval:
                        print(
                            f"  [ALIGN/{t_class}] stable {stable_cnt}/{align_stable} "
                            f"ErrX={err_x:.1f}px ErrY={err_y:.1f}px r={radius*1000:.1f}mm"
                        )
                        last_log_t = time.time()
                    if stable_cnt >= align_stable:
                        print(f"\n[ALIGN] Aligned on {t_class}.")
                        return True
                    time.sleep(dt)
                    continue

                stable_cnt = 0
                axis_label = "imgXY->baseXY"

                target_vx = -_axis_speed(err_y)
                target_vy = _axis_speed(err_x)
                combined = math.hypot(target_vx, target_vy)
                if combined > max_xy:
                    scale = max_xy / combined
                    target_vx *= scale
                    target_vy *= scale

                vx = alpha_xy * target_vx + (1.0 - alpha_xy) * prev_vx
                vy = alpha_xy * target_vy + (1.0 - alpha_xy) * prev_vy
                prev_vx, prev_vy = vx, vy
                # Servo Z sign is inverted relative to base_link TCP Z on this xArm:
                # negative command increased TCP Z in hardware logs.
                vz = max(min(-z_err * z_lock_gain, z_lock_speed), -z_lock_speed)

                projected_r = math.hypot((ex + vx * dt) - init_x, (ey + vy * dt) - init_y)
                if projected_r > max_radius:
                    if servo_active:
                        self.moveit_backend._publish_zero_twist()
                    print(
                        f"\n[ALIGN] Aborted: commanded radius {projected_r*1000:.1f}mm would exceed "
                        f"{max_radius*1000:.1f}mm fine-align limit."
                    )
                    return False

                if not _ensure_servo_active():
                    return False
                if not self.moveit_backend.publish_servo_velocity(vx, vy, vz):
                    print("\n[ALIGN] Servo velocity command failed.")
                    return False

                if now - last_log_t >= log_interval:
                    print(
                        f"  [ALIGN/{t_class}/{axis_label}] ErrX={err_x:5.1f} ErrY={err_y:5.1f} "
                        f"dist={dist_px:.0f}px cmd=({vx*1000:.2f},{vy*1000:.2f},{vz*1000:.2f})mm/s "
                        f"r={radius*1000:.1f}mm Z={ez*1000:.1f}mm"
                    )
                    last_log_t = now
                time.sleep(dt)

            print("\n[ALIGN] Timeout reached.")
            return False
        finally:
            try:
                if rclpy.ok():
                    _stop_servo_if_active()
            except Exception as exc:
                print(f"[ALIGN] Servo stop failed: {exc}")

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
    # 2. LEGACY EXOTICA CONICAL DESCENT (retained for comparison only)
    # =========================================================================
    def _perform_exotica_descent_legacy(self, hover_z: float = None, screw_z: float = None) -> bool:
        """EXOTica streaming conical descent: XY visual correction + Z descent in one loop.

        MoveIt Servo cannot produce Z motion at the post-XY-alignment joint
        configuration because the Z column of the Jacobian is near-zero there.
        EXOTica IK streaming is used instead — it solves full IK each cycle and
        is not limited by the Jacobian singularity.

        The ~0.2 rad/step IK solution for Z motion is the legitimate path at
        this configuration (not a wrong branch). joint_smooth_alpha=ALIGN_JOINT_ALPHA
        (0.35) applies only 0.07 rad per cycle, giving smooth gradual descent.

        Z speed is clamped to >= 12 mm/s so each step (0.8 mm at 15 Hz) exceeds
        EXOTica's ~0.3 mm convergence tolerance, ensuring motion every cycle.

        Conical constraint: Z descends only when dist_px <= 30 px.
        Primary stop:   FT contact spike.
        Secondary stop: Z-depth limit (screw surface - engagement_depth).
        Hard stop:      max descent travel without force contact.
        """
        print("\n[DESCENT] Starting EXOTica conical descent (XY visual + Z together)...")

        with self.data_lock:
            base_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)

        force_threshold = float(self.CONFIG["FORCE_THRESHOLD"])
        rate_hz = max(float(self.CONFIG["ALIGN_RATE_HZ"]), 1.0)
        dt = 1.0 / rate_hz
        max_xy = float(self.CONFIG["XY_SPEED_ALIGN"])
        min_xy = min(float(self.CONFIG["ALIGN_MIN_SPEED"]), max_xy)
        full_speed_error = max(float(self.CONFIG["ALIGN_FULL_SPEED_ERROR_PX"]), 1.0)
        tolerance_px = float(self.CONFIG["ALIGN_TOLERANCE_PX"])
        # Minimum 12 mm/s so each step (step = speed * dt) exceeds EXOTica's IK
        # convergence tolerance of ~0.3 mm. Config value of 4 mm/s produces
        # 0.27 mm/step at 15 Hz — below tolerance, causing the IK to report
        # "already at goal" and produce no movement.
        z_speed = max(float(self.CONFIG["Z_SPEED_DESCENT"]), 0.012)
        z_step = z_speed * dt
        max_descent_m = max(float(self.CONFIG.get("DESCENT_MAX_DISTANCE", 0.060)), 0.005)
        min_z_stop = start_z - max_descent_m
        timeout = max(120.0, (max_descent_m / max(z_speed, 0.001)) * 6.0)
        alpha_xy = max(0.05, min(1.0, float(self.CONFIG["ALIGN_ERROR_FILTER_ALPHA"])))
        joint_alpha = max(0.05, min(1.0, float(self.CONFIG["ALIGN_JOINT_ALPHA"])))
        LOG_INTERVAL_S = 0.5

        def _axis_speed(error_px: float) -> float:
            if abs(error_px) <= tolerance_px:
                return 0.0
            ratio = min(abs(error_px) / full_speed_error, 1.0)
            speed = min_xy + (max_xy - min_xy) * ratio
            return math.copysign(speed, error_px)

        # Lock pixel anchor for descent: once latched, track the same screw
        # across frames rather than re-selecting nearest-to-crosshair each time.
        descent_locked_pixel = [None]

        def _get_vision_target():
            with self.data_lock:
                lv = copy.deepcopy(self.local_view).get('local_view', {})
            crosshair = lv.get('crosshair') or [320, 240]
            candidates = lv.get('screw_heads', [])
            label = "screw_head"
            if not candidates:
                candidates = lv.get('screws', [])
                label = "screw"
            if not candidates:
                return None, None, crosshair
            anchor = descent_locked_pixel[0] if descent_locked_pixel[0] is not None else crosshair
            det = self._nearest_detection_to_crosshair(candidates, anchor)
            if det is not None and descent_locked_pixel[0] is None:
                c = det.get("center") or det.get("contact_point") or [320, 240]
                descent_locked_pixel[0] = [float(c[0]), float(c[1])]
            return label, det, crosshair

        # Read initial EE pose for reference orientation (held constant during descent)
        ee_link = "screwdriver_tcp"
        try:
            tf0 = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
            )
            ref_roll, ref_pitch, ref_yaw = self.moveit_backend._quaternion_to_rpy(
                tf0.transform.rotation.x, tf0.transform.rotation.y,
                tf0.transform.rotation.z, tf0.transform.rotation.w,
            )
            start_z = float(tf0.transform.translation.z)
        except Exception as exc:
            print(f"[DESCENT] Cannot read initial screwdriver_tcp: {exc}")
            return False

        if screw_z is not None:
            min_z_stop = screw_z - engagement_m
        elif hover_z is not None:
            min_z_stop = hover_z - float(self.CONFIG["HOVER_DISTANCE"]) - engagement_m
        else:
            min_z_stop = start_z - 0.040

        print(
            f"[DESCENT] Start Z={start_z*1000:.1f}mm  hard-stop={min_z_stop*1000:.1f}mm  "
            f"max descent={max_descent_m*1000:.1f}mm  "
            f"z_step={z_step*1000:.2f}mm/cycle  FT threshold={force_threshold:.1f}N"
        )

        # Shared mutable state for closures
        contact_flag = [False]
        prev_vx_s = [0.0]
        prev_vy_s = [0.0]
        last_log_t = [time.time()]
        start_t = time.time()

        def stop_fn():
            if not rclpy.ok():
                return True
            with self.data_lock:
                fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)
            fz_diff = abs(fz - base_fz)
            if fz_diff > force_threshold:
                contact_flag[0] = True
                print(f"\n[CONTACT] FT spike {fz_diff:.2f}N — contact detected.")
                return True
            return False

        def target_fn():
            # Get current EE pose
            try:
                tf = self.moveit_backend.tf_buffer.lookup_transform(
                    self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
                )
                ex = float(tf.transform.translation.x)
                ey = float(tf.transform.translation.y)
                ez = float(tf.transform.translation.z)
            except Exception:
                return None  # TF failure → stop cleanly

            # Hard travel guard only; force contact is the real success condition.
            if ez <= min_z_stop:
                print(
                    f"\n[DESCENT] Reached hard descent limit at {ez*1000:.1f}mm "
                    "without force contact."
                )
                return None

            # XY visual correction — vx ∝ -err_y, vy ∝ -err_x (both negative)
            t_class, t_det, crosshair = _get_vision_target()
            if t_det is not None:
                center = t_det.get("center", [320, 240])
                err_x = float(crosshair[0] - center[0])
                err_y = float(crosshair[1] - center[1])
                dist_px = math.hypot(err_x, err_y)

                target_vx = -_axis_speed(err_y)
                target_vy = -_axis_speed(err_x)
                combined = math.hypot(target_vx, target_vy)
                if combined > max_xy:
                    scale = max_xy / combined
                    target_vx *= scale
                    target_vy *= scale

                vx = alpha_xy * target_vx + (1.0 - alpha_xy) * prev_vx_s[0]
                vy = alpha_xy * target_vy + (1.0 - alpha_xy) * prev_vy_s[0]
                prev_vx_s[0] = vx
                prev_vy_s[0] = vy

                dx = vx * dt
                dy = vy * dt
                # Conical: descend only when within 30 px of centre
                dz = -z_step if dist_px <= 30.0 else 0.0

                log_str = (
                    f"[DESCENT/{t_class}] ErrX={err_x:5.1f} ErrY={err_y:5.1f} "
                    f"dist={dist_px:.0f}px dz={dz*1000:.2f}mm "
                    f"Z={ez*1000:.1f}mm moved={(start_z-ez)*1000:.1f}mm"
                )
            else:
                dx, dy, dz = 0.0, 0.0, 0.0
                prev_vx_s[0] = 0.0
                prev_vy_s[0] = 0.0
                log_str = (
                    f"[DESCENT/no-target] Z={ez*1000:.1f}mm "
                    f"moved={(start_z-ez)*1000:.1f}mm — holding"
                )

            now = time.time()
            if now - last_log_t[0] >= LOG_INTERVAL_S:
                print(f"  {log_str}")
                last_log_t[0] = now
            else:
                print(f"  {log_str}", end="\r")

            return (ex + dx, ey + dy, ez + dz, ref_roll, ref_pitch, ref_yaw)

        result = self.moveit_backend.move_cartesian_realtime_exotica(
            target_fn,
            stop_fn=stop_fn,
            rate_hz=rate_hz,
            max_step_m=max(z_step * 2.0, 0.003),
            joint_smooth_alpha=joint_alpha,
            timeout_s=timeout + 5.0,
        )

        if result in ("DONE", "STOPPED"):
            if contact_flag[0]:
                return True
            print("[DESCENT] Stopped without contact.")
            return False
        print(f"\n[DESCENT] EXOTica returned {result} — descent failed.")
        return False

    # =========================================================================
    # 2. SERVO CONICAL DESCENT (validated local-camera XY + downward Z)
    # =========================================================================
    def perform_exotica_descent(self, hover_z: float = None, screw_z: float = None) -> bool:
        """Conical descent using MoveIt Servo.

        Keeps the same public method name used by the sequence, but uses the
        hardware-validated Servo signs from fine alignment. Descent is gated:
        Z only moves down once the local target is near the crosshair.
        """
        print("\n[DESCENT] Starting Servo conical descent (local XY + gated Z)...")

        with self.data_lock:
            base_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)

        ee_link = "screwdriver_tcp"
        try:
            tf0 = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
            )
            start_x = float(tf0.transform.translation.x)
            start_y = float(tf0.transform.translation.y)
            start_z = float(tf0.transform.translation.z)
        except Exception as exc:
            print(f"[DESCENT] Cannot read initial {ee_link}: {exc}")
            return False

        max_descent_m = max(float(self.CONFIG.get("DESCENT_MAX_DISTANCE", 0.060)), 0.005)
        min_z_stop = start_z - max_descent_m

        rate_hz = max(float(self.CONFIG["ALIGN_RATE_HZ"]), 1.0)
        dt = 1.0 / rate_hz
        max_xy = float(self.CONFIG["XY_SPEED_ALIGN"])
        min_xy = min(float(self.CONFIG["ALIGN_MIN_SPEED"]), max_xy)
        full_spd_err = max(float(self.CONFIG["ALIGN_FULL_SPEED_ERROR_PX"]), 1.0)
        tol_px = float(self.CONFIG["ALIGN_TOLERANCE_PX"])
        alpha_xy = max(0.05, min(1.0, float(self.CONFIG["ALIGN_ERROR_FILTER_ALPHA"])))
        max_radius = float(self.CONFIG["VISUAL_SERVO_MAX_RADIUS"])
        z_speed = min(max(float(self.CONFIG["Z_SPEED_DESCENT"]), 0.008), 0.050)
        descent_slow_gate_px = max(25.0, tol_px + 12.0)
        descent_xy_max = min(max_xy, 0.010)
        force_threshold = float(self.CONFIG["FORCE_THRESHOLD"])
        log_interval = 0.5

        print(
            f"[DESCENT] Start Z={start_z*1000:.1f}mm  hard-stop={min_z_stop*1000:.1f}mm  "
            f"max descent={max_descent_m*1000:.1f}mm  "
            f"fine={tol_px:.0f}px slow_gate={descent_slow_gate_px:.0f}px  "
            f"z_speed={z_speed*1000:.1f}mm/s"
        )

        def _axis_speed(error_px: float) -> float:
            abs_err = abs(error_px)
            if abs_err <= tol_px:
                return 0.0
            ratio = min(abs_err / full_spd_err, 1.0)
            speed = min_xy + (max_xy - min_xy) * ratio
            if abs_err <= 20.0:
                speed = min(speed, max(min_xy, 0.0120))
            elif abs_err <= 45.0:
                speed = min(speed, max(min_xy, 0.0240))
            elif abs_err <= 80.0:
                speed = min(speed, max(min_xy, 0.0400))
            return math.copysign(speed, error_px)

        def _candidate_center(det):
            center = det.get("center") or det.get("contact_point")
            if center and len(center) >= 2:
                return [float(center[0]), float(center[1])]
            box = det.get("box")
            if box and len(box) >= 4:
                return [0.5 * (float(box[0]) + float(box[2])), 0.5 * (float(box[1]) + float(box[3]))]
            return [320.0, 240.0]

        def _choose_detection(candidates, crosshair, preferred_center=None):
            normalized = []
            for det in candidates or []:
                d = dict(det)
                d["center"] = _candidate_center(d)
                normalized.append(d)
            if not normalized:
                return None
            if preferred_center is not None:
                close = min(
                    normalized,
                    key=lambda d: math.hypot(d["center"][0] - preferred_center[0], d["center"][1] - preferred_center[1]),
                )
                if math.hypot(close["center"][0] - preferred_center[0], close["center"][1] - preferred_center[1]) <= 90.0:
                    return close
            return self._nearest_detection_to_crosshair(normalized, crosshair)

        def _get_vision():
            with self.data_lock:
                lv = copy.deepcopy(self.local_view).get("local_view", {})
            crosshair = lv.get("crosshair") or [320, 240]
            heads = lv.get("screw_heads", []) or []
            screws = lv.get("screws", []) or []
            if heads:
                preferred = tracked_center if tracked_label == "screw_head" else None
                det = _choose_detection(heads, crosshair, preferred)
                if det is not None:
                    return "screw_head", det, crosshair
            if screws:
                preferred = tracked_center if tracked_label == "screw" else None
                det = _choose_detection(screws, crosshair, preferred)
                if det is not None:
                    return "screw", det, crosshair
            return None, None, crosshair

        def _get_tcp():
            tf = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
            )
            return (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                float(tf.transform.translation.z),
            )

        if not self.moveit_backend.start_servo(timeout_sec=8.0):
            print("[DESCENT] Failed to start MoveIt Servo.")
            return False

        prev_vx = 0.0
        prev_vy = 0.0
        tracked_center = None
        tracked_label = None
        contact_flag = False
        last_log_t = 0.0
        start_t = time.time()
        no_target_since = None
        # If the model consistently reports a jump for many frames the camera has
        # likely shifted onto a neighbouring screw.  Reset the tracking anchor so
        # the persistent detection becomes the new reference and descent can resume.
        _JUMP_REJECT_RESET_FRAMES = 20   # ≈ 0.67 s at 30 Hz
        consecutive_jump_rejects = 0
        no_target_grace_s = max(0.4, float(self.CONFIG.get("ALIGN_TARGET_LOST_GRACE_S", 0.6)))
        no_target_timeout_s = max(2.0, float(self.CONFIG.get("SPIRAL_TIMEOUT", 15.0)))
        enable_spiral_search = bool(self.CONFIG.get("ENABLE_SPIRAL_SEARCH", True))
        spiral_cfg_speed = float(self.CONFIG.get("SPIRAL_SPEED", 0.001))
        # During descent the TCP is still high enough to use the same square
        # spiral search shape as hover alignment, but Servo commands below a
        # few mm/s often do not produce reliable TF progress on hardware.
        spiral_speed = min(
            max(spiral_cfg_speed, float(self.CONFIG.get("ALIGN_MIN_SPEED", 0.012))),
            descent_xy_max,
        )
        spiral_gap_m = max(float(self.CONFIG.get("SPIRAL_GAP_MM", 2.0)) / 1000.0, 0.001)
        spiral_side_len_m = max(float(self.CONFIG.get("SPIRAL_START_DIST_MM", 2.0)) / 1000.0, 0.001)
        spiral_side_idx = 0
        spiral_side_progress_m = 0.0
        spiral_started = False

        try:
            while rclpy.ok():
                try:
                    ex, ey, ez = _get_tcp()
                except Exception as exc:
                    print(f"\n[DESCENT] Could not read {ee_link}: {exc}")
                    return False

                with self.data_lock:
                    fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)
                fz_diff = abs(float(fz) - base_fz)
                if fz_diff > force_threshold:
                    contact_flag = True
                    self.last_contact_fz_diff = fz_diff
                    self.moveit_backend._publish_zero_twist()
                    print(f"\n[CONTACT] FT spike {fz_diff:.2f}N — contact detected.")
                    return True

                moved_down = start_z - ez
                if ez <= min_z_stop:
                    self.last_contact_fz_diff = max(self.last_contact_fz_diff, fz_diff)
                    self.moveit_backend._publish_zero_twist()
                    print(
                        f"\n[DESCENT] Reached hard descent limit at {ez*1000:.1f}mm "
                        f"without contact (Fz={fz_diff:.2f}N). Aborting before unscrew."
                    )
                    return False

                radius = math.hypot(ex - start_x, ey - start_y)

                t_class, t_det, crosshair = _get_vision()
                if t_det is None:
                    prev_vx = prev_vy = 0.0
                    if no_target_since is None:
                        no_target_since = time.time()
                        spiral_started = False
                        spiral_side_idx = 0
                        spiral_side_progress_m = 0.0
                        spiral_side_len_m = max(float(self.CONFIG.get("SPIRAL_START_DIST_MM", 2.0)) / 1000.0, 0.001)
                    missing_s = time.time() - no_target_since
                    if missing_s >= no_target_timeout_s:
                        self.moveit_backend._publish_zero_twist()
                        print(
                            f"\n[DESCENT] Local screw target not visible for {missing_s:.1f}s "
                            f"during descent search. Aborting before unscrew."
                        )
                        return False

                    vz = 0.0
                    if enable_spiral_search and missing_s >= no_target_grace_s:
                        if not spiral_started:
                            print("[DESCENT] No local screw target — starting bounded XY spiral at locked Z.")
                            spiral_started = True
                        if spiral_side_progress_m >= spiral_side_len_m:
                            spiral_side_idx += 1
                            spiral_side_progress_m = 0.0
                            if spiral_side_idx % 2 == 0:
                                spiral_side_len_m += spiral_gap_m
                        idx = spiral_side_idx % 4
                        if idx == 0:
                            vx, vy, direction = -spiral_speed, 0.0, "-X"
                        elif idx == 1:
                            vx, vy, direction = 0.0, -spiral_speed, "-Y"
                        elif idx == 2:
                            vx, vy, direction = spiral_speed, 0.0, "+X"
                        else:
                            vx, vy, direction = 0.0, spiral_speed, "+Y"
                        spiral_side_progress_m += spiral_speed * dt
                        projected_radius = math.hypot(
                            (ex - start_x) + vx * dt,
                            (ey - start_y) + vy * dt,
                        )
                        if projected_radius > max_radius:
                            self.moveit_backend._publish_zero_twist()
                            print(
                                f"\n[DESCENT] Spiral search would exceed XY radius "
                                f"{projected_radius*1000:.1f}mm > {max_radius*1000:.1f}mm. "
                                "Aborting before unscrew."
                            )
                            return False
                        log_str = (
                            f"[DESCENT/spiral] no-target={missing_s:.1f}/{no_target_timeout_s:.1f}s "
                            f"dir={direction} side={spiral_side_len_m*1000:.1f}mm "
                            f"r={projected_radius*1000:.1f}/{max_radius*1000:.0f}mm "
                            f"speed={spiral_speed*1000:.1f}mm/s "
                            f"Z={ez*1000:.1f}mm moved={moved_down*1000:.1f}mm Fz={fz_diff:.2f}N"
                        )
                    else:
                        vx = vy = 0.0
                        log_str = (
                            f"[DESCENT/no-target] holding Z={ez*1000:.1f}mm "
                            f"missing={missing_s:.1f}/{no_target_timeout_s:.1f}s "
                            f"moved={moved_down*1000:.1f}mm Fz={fz_diff:.2f}N"
                        )
                else:
                    if no_target_since is not None and spiral_started:
                        print("[DESCENT] Target reacquired — resuming conical descent.")
                    no_target_since = None
                    spiral_started = False
                    spiral_side_idx = 0
                    spiral_side_progress_m = 0.0
                    center = t_det.get("center", [320, 240])
                    now = time.time()
                    if tracked_center is not None:
                        jump_px = math.hypot(center[0] - tracked_center[0], center[1] - tracked_center[1])
                        if t_class == "screw_head" and tracked_label != "screw_head":
                            # Screw head is the preferred target. Allow upgrade
                            # from fallback screw to screw_head instead of
                            # rejecting it as a target jump.
                            pass
                        elif (t_class != tracked_label and jump_px > 35.0) or jump_px > 80.0:
                            consecutive_jump_rejects += 1
                            vx = vy = vz = 0.0
                            prev_vx = prev_vy = 0.0
                            log_str = (
                                f"[DESCENT/reject-{t_class}] jump={jump_px:.0f}px from "
                                f"{tracked_label}; holding Z={ez*1000:.1f}mm "
                                f"moved={moved_down*1000:.1f}mm Fz={fz_diff:.2f}N"
                            )
                            if not self.moveit_backend.publish_servo_velocity(vx, vy, vz):
                                print("\n[DESCENT] Servo hold command failed.")
                                return False
                            # After ~0.67 s of consistent jump-rejections the camera has
                            # likely shifted to an adjacent screw.  Reset the anchor so
                            # the next persistent detection becomes the new reference.
                            if consecutive_jump_rejects >= _JUMP_REJECT_RESET_FRAMES:
                                print(
                                    f"\n[DESCENT] {consecutive_jump_rejects} consecutive "
                                    f"jump-rejects — resetting tracked anchor to accept "
                                    f"current detection."
                                )
                                tracked_center = None
                                tracked_label = None
                                consecutive_jump_rejects = 0
                            if now - last_log_t >= log_interval:
                                print(f"  {log_str}")
                                last_log_t = now
                            time.sleep(dt)
                            continue

                        alpha_target = 0.35 if t_class == tracked_label else 0.15
                        center = [
                            alpha_target * float(center[0]) + (1.0 - alpha_target) * float(tracked_center[0]),
                            alpha_target * float(center[1]) + (1.0 - alpha_target) * float(tracked_center[1]),
                        ]
                    tracked_center = [float(center[0]), float(center[1])]
                    tracked_label = t_class
                    consecutive_jump_rejects = 0   # good detection accepted
                    err_x = float(crosshair[0] - center[0])
                    err_y = float(crosshair[1] - center[1])
                    dist_px = math.hypot(err_x, err_y)

                    if max(abs(err_x), abs(err_y)) <= tol_px:
                        target_vx = 0.0
                        target_vy = 0.0
                    else:
                        target_vx = -_axis_speed(err_y)
                        target_vy = _axis_speed(err_x)
                    combined = math.hypot(target_vx, target_vy)
                    if combined > descent_xy_max:
                        scale = descent_xy_max / combined
                        target_vx *= scale
                        target_vy *= scale
                    vx = alpha_xy * target_vx + (1.0 - alpha_xy) * prev_vx
                    vy = alpha_xy * target_vy + (1.0 - alpha_xy) * prev_vy
                    prev_vx, prev_vy = vx, vy
                    # Positive Servo Z command lowered TCP Z in the validated
                    # alignment logs. Keep descending while XY servo corrects,
                    # but slow down as the crosshair error grows.
                    max_err = max(abs(err_x), abs(err_y))
                    if max_err <= tol_px:
                        z_factor = 1.0
                    elif max_err <= 10.0:
                        z_factor = 0.75
                    elif max_err <= descent_slow_gate_px:
                        z_factor = 0.35
                    else:
                        z_factor = 0.0
                    vz = z_speed * z_factor

                    log_str = (
                        f"[DESCENT/{t_class}] ErrX={err_x:5.1f} ErrY={err_y:5.1f} "
                        f"dist={dist_px:.0f}px cmd=({vx*1000:.1f},{vy*1000:.1f},{vz*1000:.1f})mm/s "
                        f"Z={ez*1000:.1f}mm moved={moved_down*1000:.1f}mm Fz={fz_diff:.2f}N"
                    )

                if not self.moveit_backend.publish_servo_velocity(vx, vy, vz):
                    print("\n[DESCENT] Servo velocity command failed.")
                    return False

                now = time.time()
                if now - last_log_t >= log_interval:
                    print(f"  {log_str}")
                    last_log_t = now
                time.sleep(dt)

            self.moveit_backend._publish_zero_twist()
            print("\n[DESCENT] Stopped before force contact.")
            return contact_flag
        finally:
            try:
                self.moveit_backend._publish_zero_twist()
                self.moveit_backend.stop_servo(timeout_sec=3.0)
            except Exception as exc:
                print(f"[DESCENT] Servo stop failed: {exc}")

    # =========================================================================
    # 3. COMPLIANT EXTRACTION
    # =========================================================================
    def _current_fz(self) -> float:
        with self.data_lock:
            return float(self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0))

    def _sample_fz_average(self, duration_s: float = 0.12, rate_hz: float = 50.0) -> float:
        samples = []
        deadline = time.time() + max(float(duration_s), 0.02)
        dt = 1.0 / max(float(rate_hz), 1.0)
        while rclpy.ok() and time.time() < deadline:
            samples.append(self._current_fz())
            time.sleep(dt)
        return sum(samples) / len(samples) if samples else self._current_fz()

    def _wait_for_tool_controller(self, timeout_s: float = 4.0) -> bool:
        deadline = time.time() + max(float(timeout_s), 0.1)
        last_status = {}
        while rclpy.ok() and time.time() < deadline:
            with self.data_lock:
                last_status = dict(self.tool_status)
            if bool(last_status.get("connected")):
                return True
            if bool(last_status.get("fake")):
                print("[UNSCREW] Tool controller is in fake mode; refusing real screwdriver spin.")
                return False
            time.sleep(0.05)
        subscriber_count = self.tool_pub.get_subscription_count()
        if last_status:
            print(
                f"[UNSCREW] Tool controller is not connected; /tool_cmd subscribers={subscriber_count}, "
                f"status={last_status}. Aborting before screwdriver spin."
            )
        else:
            print(
                f"[UNSCREW] No /tool_status received from tool_commander; "
                f"/tool_cmd subscribers={subscriber_count}. Aborting before screwdriver spin."
            )
        return False

    def _reset_tool_for_unscrew_start(self):
        """Stop screwdriver motor and (if gripper enabled) open screw gripper."""
        gripper = bool(self.CONFIG.get("SCREW_GRIPPER_ENABLED", True))
        print(
            "[TOOL] Startup reset: stopping screwdriver (cmd=0)"
            + (", opening screw gripper (cmd=3)." if gripper else " (gripper disabled).")
        )
        self.tool_pub.publish(Int8(data=0))
        time.sleep(0.10)
        if gripper:
            self.tool_pub.publish(Int8(data=3))
            time.sleep(max(float(self.CONFIG["TOOL_RELEASE_SETTLE"]), 0.1))

    def _run_unscrew_probe(self, baseline_fz: float) -> tuple[bool, float]:
        probe_duration = max(float(self.CONFIG["UNSCREW_PROBE_DURATION"]), 0.05)
        threshold = max(float(self.CONFIG["UNSCREW_PROBE_FORCE_INCREASE"]), 0.01)
        peak_delta = 0.0

        print(
            f"[UNSCREW] Probe: spinning for {probe_duration:.2f}s; "
            f"need ΔFz >= {threshold:.2f}N."
        )
        self.tool_pub.publish(Int8(data=-1))
        deadline = time.time() + probe_duration
        while rclpy.ok() and time.time() < deadline:
            peak_delta = max(peak_delta, abs(self._current_fz() - baseline_fz))
            time.sleep(0.02)
        self.tool_pub.publish(Int8(data=0))
        time.sleep(0.05)

        post_delta = abs(self._current_fz() - baseline_fz)
        peak_delta = max(peak_delta, post_delta)
        engaged = peak_delta >= threshold
        print(
            f"[UNSCREW] Probe ΔFz peak={peak_delta:.2f}N post={post_delta:.2f}N "
            f"-> {'engaged' if engaged else 'not engaged'}."
        )
        return engaged, peak_delta

    def _continue_unscrew_until_released(self, baseline_fz: float, initial_peak_delta: float) -> bool:
        min_time = max(float(self.CONFIG["UNSCREW_MIN_SPIN_TIME"]), 0.1)
        max_time = max(float(self.CONFIG["UNSCREW_MAX_SPIN_TIME"]), min_time)
        threshold = max(float(self.CONFIG["UNSCREW_PROBE_FORCE_INCREASE"]), 0.01)
        release_ratio = max(0.05, min(0.95, float(self.CONFIG["UNSCREW_RELEASE_FORCE_RATIO"])))
        release_debounce = max(0.05, float(self.CONFIG.get("UNSCREW_RELEASE_DEBOUNCE", 0.15)))
        ideal_delta = max(float(self.CONFIG["UNSCREW_FORCE_IDEAL_DELTA"]), threshold)
        deadband = max(float(self.CONFIG["UNSCREW_FORCE_DEADBAND"]), 0.0)
        relief_gain = max(float(self.CONFIG["UNSCREW_Z_RELIEF_GAIN"]), 0.0)
        relief_cap = max(min(float(self.CONFIG["UNSCREW_Z_RELIEF_SPEED_CAP"]), 0.020), 0.001)
        relief_max = max(float(self.CONFIG["UNSCREW_Z_RELIEF_MAX"]), 0.0)
        ee_link = "screwdriver_tcp"

        try:
            tf0 = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
            )
            start_x = float(tf0.transform.translation.x)
            start_y = float(tf0.transform.translation.y)
            start_z = float(tf0.transform.translation.z)
        except Exception as exc:
            print(f"[UNSCREW] Could not read {ee_link} before force-relief unscrew: {exc}")
            return False

        print(
            f"[UNSCREW] Continuing spin: min={min_time:.1f}s max={max_time:.1f}s "
            f"release_ratio={release_ratio:.2f} idealΔFz={ideal_delta:.2f}N "
            f"relief_gain={relief_gain*1000:.1f}mm/s/N relief_cap={relief_cap*1000:.1f}mm/s."
        )

        if not self.moveit_backend.start_servo(timeout_sec=8.0):
            print("[UNSCREW] Failed to start MoveIt Servo for force-relief Z retract.")
            return False

        self.tool_pub.publish(Int8(data=-1))
        start_t = time.time()
        peak_delta = max(float(initial_peak_delta), threshold)
        validated_force = peak_delta >= threshold
        release_since = None
        last_log_t = 0.0
        max_lateral_drift = 0.008
        rate_hz = max(float(self.CONFIG["ALIGN_RATE_HZ"]), 20.0)
        dt = 1.0 / rate_hz
        # Grab the screw mid-spin so the gripper secures it as threads disengage.
        # Send cmd=2 at 50% of min_time (at least 0.5 s into the spin).
        grab_sent = False
        grab_after_s = max(min_time * 0.5, 0.5)
        try:
            while rclpy.ok() and time.time() - start_t < max_time:
                loop_start = time.time()
                elapsed = loop_start - start_t
                delta = abs(self._current_fz() - baseline_fz)
                peak_delta = max(peak_delta, delta)
                if delta >= threshold:
                    validated_force = True

                # Send gripper-grab while motor is still spinning so the screw
                # is captured before it can fall once threads disengage.
                if not grab_sent and elapsed >= grab_after_s:
                    grab_sent = True
                    if bool(self.CONFIG.get("SCREW_GRIPPER_ENABLED", True)):
                        print(f"[UNSCREW] Closing gripper on screw (cmd=2) at t={elapsed:.1f}s.")
                        self.tool_pub.publish(Int8(data=2))

                try:
                    tf = self.moveit_backend.tf_buffer.lookup_transform(
                        self.CONFIG["WORLD_FRAME"], ee_link, rclpy.time.Time()
                    )
                    x = float(tf.transform.translation.x)
                    y = float(tf.transform.translation.y)
                    z = float(tf.transform.translation.z)
                except Exception as exc:
                    print(f"\n[UNSCREW] Could not read {ee_link} during force relief: {exc}")
                    return False

                z_relief = max(0.0, z - start_z)
                force_over = delta - ideal_delta
                if relief_max > 0.0 and force_over > deadband and z_relief < relief_max:
                    vz_lift = min(relief_cap, relief_gain * force_over)
                    if z_relief + vz_lift * dt > relief_max:
                        vz_lift = max((relief_max - z_relief) / dt, 0.0)
                    # Negative Servo Z increases TCP Z on this xArm setup.
                    cmd_z = -vz_lift
                else:
                    cmd_z = 0.0

                if not self.moveit_backend.publish_servo_velocity(0.0, 0.0, cmd_z):
                    print("\n[UNSCREW] Servo force-relief command failed.")
                    return False

                release_level = max(threshold, peak_delta * release_ratio)
                if elapsed >= min_time and validated_force and delta <= release_level:
                    release_since = release_since or time.time()
                    if time.time() - release_since >= release_debounce:
                        print(
                            f"\n[UNSCREW] Force relaxed: ΔFz={delta:.2f}N "
                            f"peak={peak_delta:.2f}N. Screw likely loosened."
                        )
                        return True
                else:
                    release_since = None

                now = time.time()
                if now - last_log_t >= 0.5:
                    print(
                        f"  [UNSCREW] t={elapsed:.1f}s ΔFz={delta:.2f}N "
                        f"peak={peak_delta:.2f}N release<= {release_level:.2f}N "
                        f"z_relief={z_relief*1000:.1f}/{relief_max*1000:.0f}mm "
                        f"cmd_z={cmd_z*1000:.1f}mm/s"
                    )
                    last_log_t = now
                # Sleep only the remaining portion of dt so loop runs at a
                # consistent rate regardless of how long the body took.
                # Irregular input to MoveIt Servo causes uneven Z-relief motion.
                spent = time.time() - loop_start
                if dt - spent > 0.001:
                    time.sleep(dt - spent)

            print(
                f"\n[UNSCREW] Max spin time reached ({max_time:.1f}s) without validated force release. "
                "Treating as failed engagement."
            )
            return False
        finally:
            try:
                self.moveit_backend._publish_zero_twist()
                self.moveit_backend.stop_servo(timeout_sec=3.0)
            except Exception as exc:
                print(f"[UNSCREW] Servo stop after force relief failed: {exc}")
            self.tool_pub.publish(Int8(data=0))
            time.sleep(0.1)

    def perform_compliant_extraction(self) -> tuple:
        """Probe bit engagement, unscrew, close tool gripper, then closed-loop retract.

        Returns (success: bool, no_force_spike: bool).
        no_force_spike is True only when the probe detected zero axial engagement —
        the caller can then retract, realign, and redescend before retrying.
        """
        print("\n[EXTRACTION] Starting force-gated unscrew and closed-loop retract...")
        if not self._wait_for_tool_controller():
            return False, False

        print("[TOOL] Resetting screwdriver and opening screw gripper before unscrew.")
        self.tool_pub.publish(Int8(data=0))
        time.sleep(0.10)
        if bool(self.CONFIG.get("SCREW_GRIPPER_ENABLED", True)):
            self.tool_pub.publish(Int8(data=3))
            time.sleep(max(float(self.CONFIG["TOOL_RELEASE_SETTLE"]), 0.1))

        baseline_fz = self._sample_fz_average(0.15)
        engaged, peak_delta = self._run_unscrew_probe(baseline_fz)
        if not engaged:
            # First probe failed — try a second probe immediately at the same
            # position before triggering the expensive retract/realign/redescend.
            print(
                "[UNSCREW] Probe 1 did not show enough axial force increase "
                f"(ΔFz={peak_delta:.2f}N). Trying probe 2 immediately..."
            )
            time.sleep(0.10)  # brief settle so FT sensor stabilises
            baseline_fz2 = self._sample_fz_average(0.15)
            engaged, peak_delta = self._run_unscrew_probe(baseline_fz2)
            if not engaged:
                print(
                    "[UNSCREW] Probe 2 also did not show enough axial force increase. "
                    "Keeping tool stopped and retrying alignment/descent."
                )
                self.tool_pub.publish(Int8(data=0))
                return False, True  # retryable: no force spike
            print(f"[UNSCREW] Probe 2 engaged (ΔFz={peak_delta:.2f}N) — proceeding with unscrew.")

        if not self._continue_unscrew_until_released(baseline_fz, peak_delta):
            self.tool_pub.publish(Int8(data=0))
            return False, True

        if bool(self.CONFIG.get("SCREW_GRIPPER_ENABLED", True)):
            print("[GRAB] Confirming tool gripper closed on screw (cmd=2).")
            self.tool_pub.publish(Int8(data=2))
            time.sleep(max(float(self.CONFIG["TOOL_GRAB_SETTLE"]), 0.1))

        if not self._post_unscrew_style_retract("[RETRACT] Closed-loop Servo retract with gripper closed"):
            print("[RETRACT] Closed-loop retract failed; not navigating to bin.")
            return False, False

        self.wait_for_arm_settled()
        return True, False

    def _post_unscrew_style_retract(self, label: str = "[RETRACT] Closed-loop Servo retract") -> bool:
        lift_dist = max(float(self.CONFIG["TRANSIT_LIFT"]), float(self.CONFIG["POST_GRASP_RETRACT"]))
        retract_speed = float(self.CONFIG["POST_UNSCREW_RETRACT_SPEED"])
        print(f"{label}: {lift_dist*1000:.0f}mm at {retract_speed*1000:.0f}mm/s.")
        return self._safe_servo_lift_z(lift_dist, speed_mps=retract_speed)

    # =========================================================================
    # MAIN SEQUENCE
    # =========================================================================
    def execute_unscrew_command(self, target_id, target_label, interactive=True, target_data_override=None):
        print(f"\n🛠️ [START] Unscrew Sequence on ID: {target_id} ({target_label})")
        self._align_detected_hole = False   # reset for this run
        self._reset_tool_for_unscrew_start()
        
        # Verify Bin 1 location — only required when the screw gripper is enabled.
        # When disabled the screwdriver leaves the screw in place; no bin needed.
        with self.data_lock: bin1_raw = copy.deepcopy(self.cached_bin1_xyz)
        if not bin1_raw and bool(self.CONFIG.get("SCREW_GRIPPER_ENABLED", True)):
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
        xyz = target_data.get('xyz') if target_data else None
        if not isinstance(xyz, (list, tuple)) or len(xyz) < 3 or any(v is None for v in xyz[:3]):
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
        raw_pose.position.x, raw_pose.position.y, raw_pose.position.z = xyz[:3]
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

        tx_raw, ty_raw = world_pose.pose.position.x, world_pose.pose.position.y
        tx = tx_raw + float(self.CONFIG["COARSE_X_OFFSET"])
        ty = ty_raw + float(self.CONFIG["COARSE_Y_OFFSET"])
        tz_screw = world_pose.pose.position.z
        dist_base = math.hypot(base_pose.pose.position.x, base_pose.pose.position.y)
        _hover_z_override = self.CONFIG.get("COARSE_HOVER_Z_OVERRIDE")
        if _hover_z_override is not None:
            hover_z = float(_hover_z_override)
        else:
            hover_z = tz_screw + self.CONFIG["HOVER_DISTANCE"]

        print(f"[DIAG] Screw in base_link raw: ({tx_raw:.4f}, {ty_raw:.4f}, {tz_screw:.4f})")
        print(
            f"[DIAG] Coarse XY offset: dx={self.CONFIG['COARSE_X_OFFSET']*1000:.1f}mm "
            f"dy={self.CONFIG['COARSE_Y_OFFSET']*1000:.1f}mm -> target=({tx:.4f}, {ty:.4f})"
        )
        if _hover_z_override is not None:
            print(f"[DIAG] hover_z = {hover_z:.4f} (fixed override from coarse_hover_z_m)")
        else:
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
            hard_limit = self.CONFIG["REACH_LIMIT"] + self.CONFIG["REACH_SOFT_MARGIN"]
            if dist_base > hard_limit:
                print(
                    f"❌ Reach {dist_base:.3f}m exceeds hard safety limit "
                    f"{hard_limit:.3f}m."
                )
                return False
            print(
                f"⚠️ Reach {dist_base:.3f}m exceeds nominal arm limit "
                f"{self.CONFIG['REACH_LIMIT']:.3f}m, but screwdriver_tcp can extend beyond the flange. "
                "Continuing with verified teleop-style EXOTica tracking."
            )

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

        self._pub_state("MOVING")
        if not self._approach_verified_hover(tx, ty, hover_z):
            self._pub_state("IDLE")
            return False

        if bool(self.CONFIG.get("COARSE_ONLY_DEBUG", False)):
            print("[DEBUG] Coarse-only mode active: verified hover reached; skipping descent, visual servo, tool spin, extraction, and bin drop.")
            self._pub_state("IDLE")
            return True

        if bool(self.CONFIG.get("ALIGN_ONLY_DEBUG", False)):
            print("[DEBUG] Align-only mode active: running XY search/align with Z locked; skipping descent, tool spin, extraction, and bin drop.")
            self._pub_state("ALIGNING")
            result = self.perform_xy_align_search_only()
            self._pub_state("IDLE")
            return result

        # Phase 1: Servo XY align (Z locked at hover)
        print("\n[PHASE 1] Servo XY alignment at hover...")
        self._pub_state("ALIGNING")
        if not self._perform_xy_align_search_only_exotica_legacy(
            tolerance_px=self.CONFIG["PRE_DESCENT_ALIGN_TOLERANCE_PX"],
            timeout_s=self.CONFIG["PRE_DESCENT_ALIGN_TIMEOUT"],
            stable_cycles=self.CONFIG["PRE_DESCENT_ALIGN_STABLE_CYCLES"],
        ):
            self._post_unscrew_style_retract("[RETRACT] XY-align failure lift")
            self._pub_state("IDLE")
            if self._align_detected_hole:
                print("⚠️ XY alignment aborted — hole detected at target position (screw already removed).")
                self.tool_pub.publish(Int8(data=0))
                self._move_xarm_home()
                return "HOLE"
            print("⚠️ XY alignment failed.")
            return self._finish_failed_run_at_bin(bin1_raw)

        if interactive: input("👉 GATE 2: Start conical descent [ENTER]")

        # Phase 2: Servo conical descent until FT contact or screw depth
        print("\n[PHASE 2] Servo conical descent to contact...")
        self._pub_state("DESCENDING")
        if not self.perform_exotica_descent(hover_z=hover_z, screw_z=tz_screw):
            print("⚠️ Descent failed or timed out. Lifting to safety.")
            self._post_unscrew_style_retract("[RETRACT] Descent failure lift")
            self._pub_state("IDLE")
            return self._finish_failed_run_at_bin(bin1_raw)

        if bool(self.CONFIG.get("DESCENT_ONLY_DEBUG", False)):
            print("[DEBUG] Descent-only validation complete after contact/depth stop.")
            self._pub_state("IDLE")
            return True

        time.sleep(0.3)

        if bool(self.CONFIG.get("SKIP_FINAL_ALIGN_ON_CONTACT", False)):
            print("\n[PHASE 3] Skipping final XY alignment — contact detected, proceeding directly to unscrew.")
        else:
            print("\n[PHASE 3] Final XY alignment at contact...")
            self._pub_state("ALIGNING")
            if not self.perform_xy_align_search_only(
                tolerance_px=self.CONFIG["FINAL_ALIGN_TOLERANCE_PX"],
                timeout_s=self.CONFIG["FINAL_ALIGN_TIMEOUT"],
                max_radius_m=self.CONFIG["FINAL_ALIGN_MAX_RADIUS"],
                stable_cycles=self.CONFIG["FINAL_ALIGN_STABLE_CYCLES"],
                max_xy_mps=self.CONFIG["FINAL_ALIGN_XY_SPEED"],
                min_xy_mps=self.CONFIG["FINAL_ALIGN_MIN_SPEED"],
                error_alpha=self.CONFIG["FINAL_ALIGN_ERROR_ALPHA"],
                screw_head_only=False,
                enable_spiral=False,
            ):
                print("⚠️ Final XY alignment failed.")
                self._post_unscrew_style_retract("[RETRACT] Final-align failure lift")
                self._pub_state("IDLE")
                return self._finish_failed_run_at_bin(bin1_raw)

        if interactive: input("👉 GATE 3: Start compliant extraction [ENTER]")

        # Phase 4: Compliant extraction — up to MAX_PROBE_RETRIES attempts.
        # On failure (no-force-spike or spin timeout), retract EXACTLY 5 mm
        # closed-loop, realign, redescend, and retry.  Only abort early when
        # the closed-loop retract itself fails (physical blockage / servo fault)
        # or when an alignment/descent step fails after retract.
        self._pub_state("UNSCREWING")
        MAX_PROBE_RETRIES = 5
        for probe_attempt in range(MAX_PROBE_RETRIES):
            ok, no_force_spike = self.perform_compliant_extraction()
            if ok:
                print("🎉 Screw Extracted.")
                self._pub_state("DROPPING")
                return self._navigate_to_bin(bin1_raw)

            # no_force_spike=False means the closed-loop retract itself failed
            # (not recoverable by realigning).  Everything else is retryable.
            if not no_force_spike:
                print("⚠️ Retract failed during extraction — aborting retry loop.")
                break

            if probe_attempt >= MAX_PROBE_RETRIES - 1:
                print(f"⚠️ Max extraction retries ({MAX_PROBE_RETRIES}) reached.")
                break

            print(
                f"[RETRY {probe_attempt + 1}/{MAX_PROBE_RETRIES}] Extraction failed. "
                "Retracting exactly 5 mm (closed-loop), realigning, redescending..."
            )
            retract_speed = float(self.CONFIG.get("POST_UNSCREW_RETRACT_SPEED", 0.06))
            if not self._safe_servo_lift_z(0.005, speed_mps=retract_speed):
                print("⚠️ Closed-loop 5 mm retry retract failed. Aborting.")
                break

            # XY align at current (lifted) Z before re-descent.
            if not self.perform_xy_align_search_only(
                tolerance_px=self.CONFIG["FINAL_ALIGN_TOLERANCE_PX"],
                timeout_s=self.CONFIG["FINAL_ALIGN_TIMEOUT"],
                max_radius_m=self.CONFIG["FINAL_ALIGN_MAX_RADIUS"],
                stable_cycles=self.CONFIG["FINAL_ALIGN_STABLE_CYCLES"],
                max_xy_mps=self.CONFIG["FINAL_ALIGN_XY_SPEED"],
                min_xy_mps=self.CONFIG["FINAL_ALIGN_MIN_SPEED"],
                error_alpha=self.CONFIG["FINAL_ALIGN_ERROR_ALPHA"],
                screw_head_only=False,
                enable_spiral=False,
            ):
                print("⚠️ Retry XY alignment failed. Aborting.")
                break

            # Redescend to surface contact.
            if not self.perform_exotica_descent(hover_z=hover_z, screw_z=tz_screw):
                print("⚠️ Retry descent failed. Aborting.")
                break

            # Fine XY alignment on surface before next extraction attempt.
            time.sleep(0.3)
            if not self.perform_xy_align_search_only(
                tolerance_px=self.CONFIG["FINAL_ALIGN_TOLERANCE_PX"],
                timeout_s=self.CONFIG["FINAL_ALIGN_TIMEOUT"],
                max_radius_m=self.CONFIG["FINAL_ALIGN_MAX_RADIUS"],
                stable_cycles=self.CONFIG["FINAL_ALIGN_STABLE_CYCLES"],
                max_xy_mps=self.CONFIG["FINAL_ALIGN_XY_SPEED"],
                min_xy_mps=self.CONFIG["FINAL_ALIGN_MIN_SPEED"],
                error_alpha=self.CONFIG["FINAL_ALIGN_ERROR_ALPHA"],
                screw_head_only=False,
                enable_spiral=False,
            ):
                print("⚠️ Retry post-contact XY alignment failed. Aborting.")
                break

        print("⚠️ Extraction failed. Lifting to safety and stopping this run.")
        self._post_unscrew_style_retract("[RETRACT] Extraction failure lift")
        self._pub_state("IDLE")
        return self._finish_failed_run_at_bin(bin1_raw)

    def _finish_failed_run_at_bin(self, bin1_raw) -> bool:
        if not bool(self.CONFIG.get("SCREW_GRIPPER_ENABLED", True)):
            print("[FAILURE] Screw gripper disabled — skipping bin; homing.")
            self.tool_pub.publish(Int8(data=0))
            self._move_xarm_home()
        else:
            print("[FAILURE] Moving to Bin 1 drop position, then home.")
            self._navigate_to_bin(bin1_raw)
        return False

    def _navigate_to_bin(self, bin1_raw):
        gripper_enabled = bool(self.CONFIG.get("SCREW_GRIPPER_ENABLED", True))

        # ── If screw gripper is disabled there is nothing to drop. ────────────
        # The screwdriver leaves the screw in place; just stop the tool and home.
        if not gripper_enabled:
            print("🏠 [DROP-OFF] Screw gripper disabled — skipping bin; stopping tool and homing.")
            self.tool_pub.publish(Int8(data=0))
            return self._move_xarm_home()

        release_height = float(self.CONFIG["BIN_RELEASE_HEIGHT"])
        z_trim = float(self.CONFIG["BIN_Z_TRIM"])
        print(f"🗑️ [DROP-OFF] Navigating to cached Bin 1, release={release_height*100:.0f}cm trim={z_trim*1000:.0f}mm...")

        bin_pose = Pose()
        bin_pose.position.x, bin_pose.position.y, bin_pose.position.z = bin1_raw
        world_bin = self.moveit_backend.get_transformed_pose(
            bin_pose, self.CONFIG["CAMERA_FRAME"], self.CONFIG["WORLD_FRAME"]
        )
        if not world_bin:
            print("[DROP-OFF] Could not transform cached Bin 1 coordinate.")
            return False

        bx = world_bin.pose.position.x
        by = world_bin.pose.position.y
        raw_bz = world_bin.pose.position.z
        bz = raw_bz + release_height + z_trim
        print(
            f"[DROP-OFF] Bin raw camera xyz=({bin1_raw[0]:.4f},{bin1_raw[1]:.4f},{bin1_raw[2]:.4f}) "
            f"world_z={raw_bz:.4f} + release={release_height:.3f} + trim={z_trim:.3f} -> drop_z={bz:.4f}"
        )

        # ── Stage 1: XY transit at current retracted height ───────────────────
        # Read the TCP Z after the post-unscrew retract so the arm moves
        # horizontally first (safe clearance) before descending into the bin.
        transit_z = bz  # fallback if TF read fails
        try:
            import rclpy.time as rtime
            tf_now = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"],
                self.moveit_backend.default_ik_link,
                rtime.Time(),
            )
            transit_z = float(tf_now.transform.translation.z)
            print(f"[DROP-OFF] Current TCP Z after retract: {transit_z*1000:.1f}mm  → XY transit at this height")
        except Exception as e:
            print(f"[DROP-OFF] TF read failed ({e}); using bin drop_z={transit_z:.4f} as transit height")

        print(f"[DROP-OFF] Stage 1: XY transit to bin XY=({bx:.3f},{by:.3f}) at Z={transit_z:.4f}...")
        ok = self.moveit_backend.move_to_pose_exotica(bx, by, transit_z, velocity=self.CONFIG["COARSE_VELOCITY"])
        if not ok:
            print("[DROP-OFF] Stage 1 (XY transit) failed.")
            return False
        self.wait_for_arm_settled()

        # ── Stage 2: Descend to bin drop height ───────────────────────────────
        print(f"[DROP-OFF] Stage 2: Descending to drop_z={bz:.4f}...")
        ok = self.moveit_backend.move_to_pose_exotica(bx, by, bz, velocity=self.CONFIG["COARSE_VELOCITY"])
        if not ok:
            print("[DROP-OFF] Stage 2 (descent) failed.")
            return False
        self.wait_for_arm_settled()

        # ── Release & home ────────────────────────────────────────────────────
        print("⏬ [RELEASE] Opening tool gripper above Bin 1...")
        self.tool_pub.publish(Int8(data=3))   # Release (gripper_enabled already confirmed above)
        time.sleep(1.0)
        self.tool_pub.publish(Int8(data=0))
        print("♻️ Screw released. Moving xArm5 to home and waiting.")
        return self._move_xarm_home()

    def _move_xarm_home(self) -> bool:
        home_joints = dict(self.CONFIG["XARM_HOME_JOINTS"])
        try:
            self.moveit_backend.stop_servo(timeout_sec=2.0)
        except Exception:
            pass
        print("[HOME] Moving xArm5 to home pose after Bin 1 drop...")
        ok = self.moveit_backend.move_to_joint_positions(home_joints, velocity=self.CONFIG["COARSE_VELOCITY"])
        if ok:
            self.wait_for_arm_settled(timeout=3.0)
            print("[HOME] xArm5 home reached. Waiting; not selecting another screw.")
            self._pub_state("IDLE")
            return True
        print("[HOME] Failed to reach xArm5 home pose.")
        self._pub_state("IDLE")
        return False

def main(args=None):
    import os
    from ament_index_python.packages import get_package_share_directory
    from disassembly_skill.device_config import DeviceConfig
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
    node = UnscrewSkill(device_cfg=device_cfg)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=False)
    spin_thread.start()

    try:
        time.sleep(2.0)
        while rclpy.ok():
            target_id, target_label = None, None
            with node.data_lock:
                if node.latest_targets:
                    target_id = node.latest_targets[0].get('id')
                    target_label = node.latest_targets[0].get('label', 'screw')
            
            if target_id is not None:
                result = node.execute_unscrew_command(target_id, target_label, interactive=False)
                with node.data_lock: node.latest_targets = []
                if not result:
                    print("[DEBUG] Unscrew target failed; stopping CLI run instead of moving to another target.")
                    break
                print("[DEBUG] Unscrew target complete; stopping CLI run instead of moving to another target.")
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)

if __name__ == '__main__': main()
