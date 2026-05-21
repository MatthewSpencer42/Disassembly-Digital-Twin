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

class ObjectFlipSkill(Node):
    def __init__(self, device_cfg=None):
        super().__init__('object_flip_skill_node')
        
        self.uf850 = MotionBackend(self, "uf850_arm")
        self.gripper = MotionBackend(self, "rg6_gripper")
        
        self.state_update_pub = self.create_publisher(String, '/robot_state/manip_arm/update', 10)

        self.PLANNING_FRAME = "base_link"
        self.ROBOT_EE_LINK = "rg6_tcp"
        self.JOINT_GRIPPER = "rg6_right_drive_joint"
        self.OPEN_DEG, self.CLOSE_DEG = -35.0, 35.0
        self.RETRACT_Z_HEIGHT = 0.1           
        self.TORQUE_THRESHOLD = 3.0            
        self.DESCENT_SPEED = 0.2              
        self.DESCENT_STEP_M = 0.0005
        self.DESCENT_RATE_HZ = 50.0
        self.RETRACT_VELOCITY = 0.5
        self.GRIPPER_OPEN_FORCE_N = 40.0      
        self.GRIPPER_CLOSE_FORCE_N = 100.0     
        self.GRIP_WIDTH_MM = 101.6
        self.GRIP_CONTACT_MARGIN_MM = 20.0
        self.FLIP_SEAT_EXTRA_DESCENT_M = 0.0
        self.FLIP_SEAT_ACCEPT_TOLERANCE_M = 0.012
        self.ROTATION_DEG = 180.0
        self.WRIST_ROTATION_VELOCITY = 0.35
        self._last_retract_start_z = None
        
        self.is_holding_object = False
        self.hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hold_status_pub = self.create_publisher(Bool, '/object_hold_state/is_held', self.hold_qos)
        self.create_subscription(Bool, '/object_hold_state/is_held', self.hold_status_callback, self.hold_qos)

        if device_cfg is not None:
            self._apply_flip_config(device_cfg)

        self.get_logger().info("Object Flip Skill: Compliant with Multi-Arm State Manager.")

    def _apply_flip_config(self, cfg):
        flip_steps = [s for s in cfg.disassembly_sequence if s.action == 'flip']
        if not flip_steps:
            return
        p = flip_steps[0].parameters
        self.RETRACT_Z_HEIGHT = p.get('retract_height_m', self.RETRACT_Z_HEIGHT)
        self.GRIPPER_CLOSE_FORCE_N = p.get('gripper_close_force_n', self.GRIPPER_CLOSE_FORCE_N)
        self.TORQUE_THRESHOLD = p.get('torque_threshold_nm', self.TORQUE_THRESHOLD) if hasattr(self, 'TORQUE_THRESHOLD') else 3.0
        self.ROTATION_DEG = p.get('rotation_deg', self.ROTATION_DEG)
        self.WRIST_ROTATION_VELOCITY = p.get('wrist_rotation_velocity', self.WRIST_ROTATION_VELOCITY)
        self.DESCENT_STEP_M = p.get('descent_step_m', self.DESCENT_STEP_M)
        self.DESCENT_RATE_HZ = p.get('descent_rate_hz', self.DESCENT_RATE_HZ)
        self.FLIP_SEAT_EXTRA_DESCENT_M = p.get('seat_extra_descent_m', self.FLIP_SEAT_EXTRA_DESCENT_M)
        self.FLIP_SEAT_ACCEPT_TOLERANCE_M = p.get(
            'seat_accept_tolerance_m',
            self.FLIP_SEAT_ACCEPT_TOLERANCE_M,
        )
        hold_steps = [s for s in cfg.disassembly_sequence if s.action == 'hold']
        if hold_steps:
            hold_p = hold_steps[0].parameters
            self.GRIP_WIDTH_MM = hold_p.get('grip_width_mm', self.GRIP_WIDTH_MM)
            self.GRIP_CONTACT_MARGIN_MM = hold_p.get(
                'grip_contact_margin_mm',
                self.GRIP_CONTACT_MARGIN_MM,
            )
        self.GRIP_WIDTH_MM = p.get('grip_width_mm', self.GRIP_WIDTH_MM)
        self.GRIP_CONTACT_MARGIN_MM = p.get('grip_contact_margin_mm', self.GRIP_CONTACT_MARGIN_MM)

    def publish_state(self, s):
        self.state_update_pub.publish(String(data=s))

    def publish_hold_status(self, h):
        self.hold_status_pub.publish(Bool(data=h))

    def hold_status_callback(self, msg):
        self.is_holding_object = msg.data

    def _current_uf_joint_positions(self):
        return {n: p for n, p in self.uf850.current_joint_positions.items() if n.startswith("uf850_")}

    def _current_tcp_pose(self):
        try:
            tf = self.uf850.tf_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time())
            q = tf.transform.rotation
            return (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                float(tf.transform.translation.z),
                {"qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w},
            )
        except Exception as exc:
            self.get_logger().warning(f"[flip] Unable to read current TCP pose: {exc}")
            return None

    @staticmethod
    def _wrap_to_pi(angle_rad):
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

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
            return False

        distance_m = abs(float(distance_m))
        self._last_retract_start_z = float(sz)
        target_z = sz + distance_m
        velocity = min(float(self.RETRACT_VELOCITY), 0.12)
        print(
            f"🚀 Planned Z retract: current {self.ROBOT_EE_LINK}=({sx:.4f},{sy:.4f},{sz:.4f}); "
            f"target Z={target_z:.4f} (+{distance_m*1000:.1f}mm)"
        )
        ok = self.uf850.move_to_pose_exotica(sx, sy, target_z, q, velocity=velocity)
        if not ok:
            print("[RETRACT] EXOTica retract failed; trying Cartesian fallback.")
            ok = self.uf850.move_cartesian_to_pose(sx, sy, target_z, q, velocity=min(velocity, 0.08))
        if not ok:
            print("[RETRACT] Planned Z retract failed.")
            return False

        self.wait_for_arm_settled()
        try:
            end_tf = self.uf850.tf_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time())
            ez = end_tf.transform.translation.z
            actual_dz = float(ez - sz)
            print(f"[RETRACT] actual dz={actual_dz*1000:.1f}mm target_dz={distance_m*1000:.1f}mm")
            min_expected_dz = max(0.010, min(distance_m * 0.50, distance_m - 0.010))
            if actual_dz < min_expected_dz:
                self.get_logger().error(
                    f"Unsafe flip retract result: expected +Z but actual dz={actual_dz*1000:.1f}mm. Aborting."
                )
                self.uf850._hold_current_arm_position()
                return False
        except Exception as e:
            self.get_logger().warning(f"Could not read final TF after retract: {e}")
        return True

    @staticmethod
    def _gripper_width_mm_to_rad(width_mm):
        rad_open = -0.625
        rad_close = 0.625
        mm_open = 160.0
        mm_close = 0.0
        width = max(mm_close, min(mm_open, float(width_mm)))
        normalized = (width - mm_close) / (mm_open - mm_close)
        return rad_close + normalized * (rad_open - rad_close)

    def _flip_close_width_mm(self):
        return max(0.0, float(self.GRIP_WIDTH_MM) - float(self.GRIP_CONTACT_MARGIN_MM))

    def _flip_close_target_rad(self):
        return self._gripper_width_mm_to_rad(self._flip_close_width_mm())

    def _wait_for_gripper_rad(self, target_rad, is_closing, start_rad=None, timeout=7.0):
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
                        moved = 0.0 if start_rad is None else float(curr) - float(start_rad)
                        if moved >= motion_acceptance_rad:
                            self.get_logger().info(
                                f"Grasp confirmed by closing stall at {curr:.3f} rad (moved {moved:.3f} rad)."
                            )
                            return True
                        self.get_logger().warning(
                            f"Gripper close stalled without meaningful motion: current={curr:.3f} target={target_rad:.3f}"
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

    def _open_gripper_for_release(self):
        target_rad = math.radians(self.OPEN_DEG)
        self.get_logger().info(f"[flip] Opening gripper to {target_rad:.3f}rad")
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: target_rad},
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N,
        ):
            return False
        return self._wait_for_gripper_rad(target_rad, is_closing=False)

    def _close_gripper_for_flip(self):
        target_rad = self._flip_close_target_rad()
        target_width = self._flip_close_width_mm()
        current = self.gripper.current_joint_positions.get(self.JOINT_GRIPPER, None)
        is_closing = True if current is None else target_rad > float(current)
        self.get_logger().info(
            f"[flip] Closing gripper to target={target_rad:.3f}rad "
            f"(target_width={target_width:.1f}mm, configured_width={float(self.GRIP_WIDTH_MM):.1f}mm)"
        )
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: target_rad},
            gripper_force_n=self.GRIPPER_CLOSE_FORCE_N,
        ):
            return False
        return self._wait_for_gripper_rad(
            target_rad,
            is_closing=is_closing,
            start_rad=None if current is None else float(current),
        )

    def _seat_after_flip(self):
        current = self._current_tcp_pose()
        if current is None:
            return False
        _cx, _cy, current_z, _q = current
        reference_z = self._last_retract_start_z
        if reference_z is None:
            reference_z = current_z - abs(float(self.RETRACT_Z_HEIGHT))

        target_z = float(reference_z) + float(self.FLIP_SEAT_EXTRA_DESCENT_M)
        min_tcp_z = getattr(self.uf850, "min_tcp_z", None)
        if min_tcp_z is not None:
            target_z = max(target_z, float(min_tcp_z) + 0.002)
        descent_distance = max(0.0, float(current_z) - target_z)
        if descent_distance <= 0.002:
            self.get_logger().info("[flip] Return descent skipped; TCP already at or below target hold height.")
            return True

        self.get_logger().info(
            f"[flip] Returning flipped HDD to held height: current_z={current_z:.4f}, "
            f"target_z={target_z:.4f}, distance={descent_distance*1000:.1f}mm"
        )
        cx, cy, _cz, qd = current
        if not self.uf850.move_to_pose_exotica(cx, cy, target_z, qd, velocity=0.08):
            self.get_logger().warning("[flip] Planned return descent failed; trying Cartesian fallback.")
            if not self.uf850.move_cartesian_to_pose(cx, cy, target_z, qd, velocity=0.05):
                self.uf850._hold_current_arm_position()
                return False

        self.wait_for_arm_settled()
        final_pose = self._current_tcp_pose()
        if final_pose is None:
            return False
        final_z = final_pose[2]
        z_err = abs(final_z - target_z)
        if z_err <= float(self.FLIP_SEAT_ACCEPT_TOLERANCE_M):
            self.get_logger().info(
                f"[flip] Return descent reached held height: final_z={final_z:.4f}, "
                f"target_z={target_z:.4f}, err={z_err*1000:.1f}mm"
            )
            return True

        self.get_logger().error(
            f"[flip] Return descent failed: final_z={final_z:.4f}, target_z={target_z:.4f}, "
            f"error={(final_z-target_z)*1000:.1f}mm"
        )
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

    def wait_for_gripper(self, target_deg, timeout=7.0):
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

    def execute_flip(self, interactive=True):
        # Ensure clean state (previous skill may have left servo on a different MotionBackend)
        self.uf850.stop_servo()

        # Wait up to 2 seconds for hold message to arrive (ROS sync buffer)
        sw = time.time()
        while not self.is_holding_object and (time.time() - sw) < 2.0:
            time.sleep(0.1)

        if not self.is_holding_object:
            self.get_logger().error("❌ Error: No object held. Ensure 'object_hold_state/is_held' is publishing True.")
            return False

        self.publish_state("FLIPPING")

        # --- STEP 1: PLANNED LIFT ---
        print(f"🚀 STEP 1: Retracting {self.RETRACT_Z_HEIGHT*100:.1f}cm (planned Z trajectory)...")
        if not self._planned_retract_z(self.RETRACT_Z_HEIGHT):
            return False

        # --- STEP 2: CONFIGURED SINGLE WRIST ROTATION ---
        print(f"🔄 STEP 2: Executing Single {self.ROTATION_DEG:.1f}° Flip...")
        joints = self._current_uf_joint_positions()
        current_j6 = joints.get("uf850_joint6", 0.0)
        joints["uf850_joint6"] = self._wrist_flip_target(current_j6)
            
        if not self.uf850.move_to_joint_positions(joints, velocity=self.WRIST_ROTATION_VELOCITY): 
            return False
        self.wait_for_arm_settled()

        # --- STEP 3: RETURN TO HELD HEIGHT ---
        print("⬇️ STEP 3: Returning flipped HDD to held height...")
        if not self._seat_after_flip():
            return False

        # Keep the chassis held after the wrist flip. Releasing/regrasping here
        # can drop the HDD if the surface touches before the gripper is seated.
        self.publish_state("HOLDING")
        self.publish_hold_status(True)
        print("🎉 FLIP COMPLETE — chassis remains held.")
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
    node = ObjectFlipSkill(device_cfg=device_cfg)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=False)
    spin_thread.start()
    # Wait for TF buffer to populate before doing anything
    time.sleep(2.0)
    try:
        if not node.is_holding_object:
            print("❌ No object held. Run object_hold_skill first.")
        else:
            success = node.execute_flip(interactive=False)
            if success:
                print("✅ Flip complete. Broadcasting hold state. Press Ctrl+C to exit.")
                while rclpy.ok():
                    node.publish_hold_status(True)
                    time.sleep(1.0)
            else:
                print("❌ Flip failed.")
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
