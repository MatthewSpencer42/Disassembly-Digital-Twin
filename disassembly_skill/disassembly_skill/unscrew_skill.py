#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Pose
import json, time, threading, copy, math
from disassembly_skill.motion_backend import MotionBackend

class UnscrewSkill(Node):
    def __init__(self):
        super().__init__('unscrew_skill_node')
        
        # --- ⚙️ CONFIGURATION SECTION (CORE PARAMETERS) ---
        # Physical Geometry
        self.CONFIG = {
            "TOOL_LENGTH": 0.240,           # m (Screwdriver length)
            "HOVER_DISTANCE": 0.005,        # m (10mm as requested)
            "TRANSIT_LIFT": 0.030,          # m (Safe travel height)
            "REACH_LIMIT": 0.680,           # m (xArm reach radius)
            "MM_PER_PIX": 0.000130,         # m/px (Vision calibration)
            
            # Descent & Alignment
            "XY_SPEED_ALIGN": 0.004,        # m/s, bounded by closed-loop TCP safety limits
            "Z_SPEED_DESCENT": 0.002,       # m/s, slow until bit seating is proven
            "ALIGN_TOLERANCE_PX": 4.0,      # pixels
            "FORCE_THRESHOLD": 5.0,         # N (Contact detection)
            "MAX_SEARCH_RADIUS": 0.012,     # m from hover; hard abort if exceeded
            "MAX_DESCENT_DEPTH": 0.012,     # m below hover; hard abort if exceeded
            "MAX_HOVER_LIFT": 0.006,        # m above hover during search/descent

            # Spiral Search
            "SPIRAL_TIMEOUT": 10.0,         # s (Vision fallback)
            "SPIRAL_START_DIST_MM": 2.0,    # mm (First side distance)
            "SPIRAL_GAP_MM": 2.0,           # mm (Increase per 2 sides)
            "SPIRAL_SPEED": 0.003,          # m/s
            
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

    def _current_tcp_quaternion(self):
        try:
            tf = self.moveit_backend.tf_buffer.lookup_transform(
                self.CONFIG["WORLD_FRAME"],
                "screwdriver_tcp",
                rclpy.time.Time(),
            )
            q = tf.transform.rotation
            return {"qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
        except Exception as exc:
            self.get_logger().warning(f"Could not read current screwdriver_tcp orientation: {exc}")
            return None

    # =========================================================================
    # 1. ALIGNMENT & DESCENT (EXOTica real-time IK — no servo mode)
    # =========================================================================
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

        start_ee = None
        try:
            start_ee = (
                tf0.transform.translation.x,
                tf0.transform.translation.y,
                tf0.transform.translation.z,
            )
        except Exception:
            start_ee = _get_ee()
        if start_ee is None:
            print("[DESCENT] Could not read starting screwdriver_tcp pose; aborting.")
            return False
        start_x, start_y, start_z = start_ee
        max_radius = float(self.CONFIG["MAX_SEARCH_RADIUS"])
        min_z = start_z - float(self.CONFIG["MAX_DESCENT_DEPTH"])
        max_z = start_z + float(self.CONFIG["MAX_HOVER_LIFT"])
        abort_reason = [None]
        print(
            "[DESCENT] Safety envelope: "
            f"XY radius <= {max_radius*1000:.1f}mm, "
            f"Z [{min_z:.4f}, {max_z:.4f}]"
        )

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

        def _bounded_target(nx, ny, nz, context):
            actual = _get_ee()
            if actual is not None:
                ax, ay, az = actual
                actual_radius = math.hypot(ax - start_x, ay - start_y)
                if actual_radius > max_radius + 0.003:
                    abort_reason[0] = (
                        f"{context}: actual TCP XY radius {actual_radius*1000:.1f}mm "
                        f"exceeded {max_radius*1000:.1f}mm limit"
                    )
                    print(f"\n[SAFETY] {abort_reason[0]}")
                    return None
                if az < min_z - 0.002 or az > max_z + 0.004:
                    abort_reason[0] = (
                        f"{context}: actual TCP Z {az:.4f} outside "
                        f"[{min_z:.4f}, {max_z:.4f}]"
                    )
                    print(f"\n[SAFETY] {abort_reason[0]}")
                    return None

            requested_radius = math.hypot(nx - start_x, ny - start_y)
            if requested_radius > max_radius:
                abort_reason[0] = (
                    f"{context}: requested TCP XY radius {requested_radius*1000:.1f}mm "
                    f"exceeded {max_radius*1000:.1f}mm limit"
                )
                print(f"\n[SAFETY] {abort_reason[0]}")
                return None
            if nz < min_z or nz > max_z:
                abort_reason[0] = (
                    f"{context}: requested TCP Z {nz:.4f} outside "
                    f"[{min_z:.4f}, {max_z:.4f}]"
                )
                print(f"\n[SAFETY] {abort_reason[0]}")
                return None
            return (nx, ny, nz, ref_roll, ref_pitch, ref_yaw)

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

            screw = local.get('local_view', {}).get('screw_heads', [])
            crosshair = local.get('local_view', {}).get('crosshair', [320, 240])

            if screw:
                # Visual alignment + Z descent
                err_x = crosshair[0] - screw[0].get('center', [320, 240])[0]
                err_y = crosshair[1] - screw[0].get('center', [320, 240])[1]
                dist_px = math.hypot(err_x, err_y)
                MAX_XY = self.CONFIG["XY_SPEED_ALIGN"]
                dt = 1.0 / 50.0
                dx = max(min((err_y * self.CONFIG["MM_PER_PIX"]) * -2.5 * dt, MAX_XY * dt), -MAX_XY * dt)
                dy = max(min((err_x * self.CONFIG["MM_PER_PIX"]) * -2.5 * dt, MAX_XY * dt), -MAX_XY * dt)
                z_step = self.CONFIG["Z_SPEED_DESCENT"] * dt
                if dist_px > 30.0:
                    z_step = 0.0
                elif dist_px > self.CONFIG["ALIGN_TOLERANCE_PX"]:
                    z_step *= self.CONFIG["ALIGN_TOLERANCE_PX"] / dist_px
                print(f"  [VIS] ErrX={err_x:5.1f} ErrY={err_y:5.1f} Fz={diff_fz:.2f}N Vz={z_step*1000:.1f}mm/s", end="\r")
                return _bounded_target(ex + dx, ey + dy, ez - z_step, "visual descent")
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
                return _bounded_target(ex + dx, ey + dy, ez, "spiral search")

        while retry_count <= MAX_RETRIES:
            contact_flag[0] = False
            result = self.moveit_backend.move_cartesian_realtime_exotica(
                target_fn,
                rate_hz=50.0,
                max_step_m=0.001,
                joint_smooth_alpha=0.35,
                timeout_s=self.CONFIG["SPIRAL_TIMEOUT"],
            )

            if abort_reason[0]:
                print(f"[DESCENT] Aborted by safety envelope: {abort_reason[0]}")
                return False

            if contact_flag[0]:
                print(f"\n[CONTACT] FT contact detected.")
                time.sleep(0.3)

                # Fine XY surface alignment
                print("[SURFACE ALIGN] Correcting XY on surface...")
                align_deadline = time.time() + 3.0
                while rclpy.ok() and time.time() < align_deadline:
                    with self.data_lock:
                        al = copy.deepcopy(self.local_view)
                    al_screw = al.get('local_view', {}).get('screw_heads', [])
                    al_cross = al.get('local_view', {}).get('crosshair', [320, 240])
                    if not al_screw:
                        print("[SURFACE ALIGN] No screw visible, skipping.")
                        break
                    al_ex = al_cross[0] - al_screw[0].get('center', [320, 240])[0]
                    al_ey = al_cross[1] - al_screw[0].get('center', [320, 240])[1]
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
                        target = _bounded_target(ex + cdx, ey + cdy, ez, "surface align")
                        if target is None:
                            return False
                        self.moveit_backend.move_cartesian_realtime_exotica(
                            lambda target=target: target,
                            timeout_s=0.12, rate_hz=50.0,
                            max_step_m=0.001,
                            joint_smooth_alpha=0.35,
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
    def execute_unscrew_command(self, target_id, target_label, interactive=True):
        print(f"\n🛠️ [START] Unscrew Sequence on ID: {target_id} ({target_label})")
        
        # Verify Bin 1 location
        with self.data_lock: bin1_raw = copy.deepcopy(self.cached_bin1_xyz)
        if not bin1_raw:
            print("❌ Bin 1 location missing. Aborting.")
            return False

        # Find Target Object
        with self.data_lock:
            target_data = next((t for t in self.latest_targets if t.get('id') == target_id), None)
        if not target_data or 'xyz' not in target_data:
            print(f"❌ Target {target_id} not found in vision.")
            return False

        # Transforms
        raw_pose = Pose()
        raw_pose.position.x, raw_pose.position.y, raw_pose.position.z = target_data['xyz']
        world_pose = self.moveit_backend.get_transformed_pose(raw_pose, self.CONFIG["CAMERA_FRAME"], self.CONFIG["WORLD_FRAME"])
        base_pose = self.moveit_backend.get_transformed_pose(raw_pose, self.CONFIG["CAMERA_FRAME"], self.CONFIG["XARM_BASE_FRAME"])

        if not world_pose or not base_pose: return False
        
        tx, ty = world_pose.pose.position.x, world_pose.pose.position.y
        screw_z = world_pose.pose.position.z
        dist_base = math.hypot(base_pose.pose.position.x, base_pose.pose.position.y)
        # The commanded link for xarm5 is screwdriver_tcp, which is already the
        # physical tool tip. Do not add TOOL_LENGTH here; that drives the tip
        # ~24 cm above/away from the screw.
        hover_z = screw_z + self.CONFIG["HOVER_DISTANCE"]

        print(f"[DIAG] Vision raw xyz in {self.CONFIG['CAMERA_FRAME']}: "
              f"({raw_pose.position.x:.4f}, {raw_pose.position.y:.4f}, {raw_pose.position.z:.4f})")
        print(f"[DIAG] Screw in {self.CONFIG['WORLD_FRAME']}: ({tx:.4f}, {ty:.4f}, {screw_z:.4f})")
        print(f"[DIAG] screwdriver_tcp hover_z = {screw_z:.4f} + {self.CONFIG['HOVER_DISTANCE']:.4f} = {hover_z:.4f}")
        print(f"[DIAG] xarm5 base dist: {dist_base:.3f}m (limit {self.CONFIG['REACH_LIMIT']:.3f}m)")

        if dist_base > self.CONFIG["REACH_LIMIT"]:
            print(f"❌ Reach {dist_base:.3f}m exceeds limit.")
            return False

        if interactive: input(f"👉 GATE 1: Approach Hover ({hover_z:.3f}m) [ENTER]")
        
        # Do not pre-retract here. In this workspace that caused an unlabelled
        # 30 mm jerk before the actual hover command. The hover target itself is
        # already above the screw by HOVER_DISTANCE.
        print("[DIAG] Pre-retract disabled; moving directly to screw hover.")

        # Robust Hover
        q_current = self._current_tcp_quaternion()
        if q_current:
            print(
                "[DIAG] Holding current screwdriver_tcp orientation: "
                f"({q_current['qx']:.4f}, {q_current['qy']:.4f}, "
                f"{q_current['qz']:.4f}, {q_current['qw']:.4f})"
            )
        if not self.moveit_backend.move_to_pose_exotica(
            tx, ty, hover_z, q_dict=q_current, velocity=0.05
        ):
            print("❌ Approach failed.")
            return False
        self.wait_for_arm_settled()

        if interactive: input("👉 GATE 2: Start EXOTica Descent [ENTER]")

        # Stage 1: Descent
        staircase_res = self.perform_staircase_descent()
        if staircase_res == "TIMEOUT" or staircase_res is False:
            print("⚠️ Descent failed or timed out. Lifting to safety; skipping bin drop because no screw was captured.")
            self.tool_pub.publish(Int8(data=0))
            self.moveit_backend.retract_z_exotica(
                self.CONFIG["TRANSIT_LIFT"], speed_mps=self.CONFIG["RETRACT_SPEED"])
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
            
            if self.moveit_backend.move_to_pose_exotica(bx, by, bz, velocity=0.1):
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
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__': main()
