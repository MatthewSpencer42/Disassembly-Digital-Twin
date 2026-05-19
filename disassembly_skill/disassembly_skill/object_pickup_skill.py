#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
import json, time, threading, math, os
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

class PickupSkill(Node):
    def __init__(self, device_cfg=None):
        super().__init__('pickup_skill_node')
        # Motion Backends
        self.uf850 = MotionBackend(self, "uf850_arm")
        self.gripper = MotionBackend(self, "rg6_gripper")
        self.xarm5 = MotionBackend(self, "xarm5_arm_no_slide")
        
        # Interfaces
        self.state_update_pub = self.create_publisher(String, '/robot_state/manip_arm/update', 10)
        self.hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hold_status_pub = self.create_publisher(Bool, '/object_hold_state/is_held', self.hold_qos)
        
        # Physical Parameters
        self.UF_TOOL_LENGTH = 0.26
        self.HOVER_HEIGHT = 0.05    
        self.DESCENT_SPEED = 0.09              
        self.TORQUE_THRESHOLD = 3.0            
        self.RETRACT_DIST = 0.05               
        self.RETRACT_VELOCITY = 0.5
        self.POST_GRASP_RETRACT_SPEED = 0.1   # New variable for slow, safe lifts
        self.OPEN_DEG, self.CLOSE_DEG = 33.0, -35.0
        self.GRIPPER_CLOSE_FORCE_N = 80.0      # Updated to 30N as requested
        self.GRIPPER_OPEN_FORCE_N = 40.0       # Increased for better release
        self.JOINT_GRIPPER = "rg6_right_drive_joint"
        self.APPROACH_X_OFFSET = -0.02
        self.APPROACH_Y_OFFSET = 0.019

        self.UF_HOME_JOINTS = {'uf850_joint1': 0.0, 'uf850_joint2': 0.0, 'uf850_joint3': -1.57, 'uf850_joint4': 0.0, 'uf850_joint5': -1.57, 'uf850_joint6': 0.0}
        self.DROP_POSE = {'x': 0.92, 'y': -0.36, 'z': 1.25}

        if device_cfg is not None:
            self._apply_pickup_config(device_cfg)

        # Thread Safety & State
        self.data_lock = threading.Lock()
        self.latest_targets = []
        self.is_holding_object = False
        
        # Subscriptions
        self.create_subscription(Bool, '/object_hold_state/is_held', self.hold_status_callback, self.hold_qos)
        self.create_subscription(String, '/vision/agent_state', self.vision_callback, 10)
        
    def _apply_pickup_config(self, cfg, target_label=None):
        # Find most-specific matching pickup step
        pickup_steps = [s for s in cfg.disassembly_sequence if s.action == 'pickup']
        if not pickup_steps:
            return
        # Try to match by target label, fall back to first pickup step
        matched = next(
            (s for s in pickup_steps if target_label and target_label.lower() in s.target.lower()),
            pickup_steps[0],
        )
        p = matched.parameters
        self.GRIPPER_CLOSE_FORCE_N = p.get('gripper_close_force_n', self.GRIPPER_CLOSE_FORCE_N)
        self.HOVER_HEIGHT = p.get('lift_height_mm', 50.0) / 1000.0
        self.APPROACH_X_OFFSET = p.get('approach_x_offset_m', self.APPROACH_X_OFFSET)
        self.APPROACH_Y_OFFSET = p.get('approach_y_offset_m', self.APPROACH_Y_OFFSET)
        drop = {
            'x': p.get('drop_x', self.DROP_POSE['x']),
            'y': p.get('drop_y', self.DROP_POSE['y']),
            'z': p.get('drop_z', self.DROP_POSE['z']),
        }
        self.DROP_POSE = drop
        open_deg = p.get('gripper_open_deg', 33.0)
        close_deg = p.get('gripper_close_deg', -35.0)
        if close_deg < 0:  # pickup skill convention: negative = close
            self.OPEN_DEG = abs(open_deg)
            self.CLOSE_DEG = -abs(close_deg)

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

    def _select_pickup_target(self):
        pickup_keywords = ("pcb_main", "pcb", "board", "circuit")
        with self.data_lock:
            valid = [t for t in self.latest_targets if self._has_valid_xyz(t)]
        preferred = [t for t in valid if self._label_contains(t, pickup_keywords)]
        return preferred[0] if preferred else None

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
        is_closing = target_deg < 0
        is_opening = target_deg > 0
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

    def execute_pickup(self, target_id, target_label, interactive=True):
        print(f"\n🛠️ [START] {target_label} Sequence (ID: {target_id})")

        # Ensure clean trajectory mode (previous skill may have left servo on)
        self.uf850.stop_servo()

        # --- STEP 0: PRE-CONDITION ---
        if self.is_holding_object:
            print("📦 [PRE-CONDITION] Active Hold Detected. Releasing...")
            self.publish_state("IDLE")
            if not self.gripper.move_to_joint_positions(
                {self.JOINT_GRIPPER: math.radians(self.OPEN_DEG)},
                gripper_force_n=self.GRIPPER_OPEN_FORCE_N
            ): return False
            self.wait_for_gripper(self.OPEN_DEG)
            self.hold_status_pub.publish(Bool(data=False))
            write_hold_state(False)

            print("⬆️ Vertical Retract (30cm)...")
            # Try Cartesian move first (cleanest)
            if not self.uf850.retract_relative_z(0.30, velocity=0.1):
                print("⚠️ Cartesian retract failed (singularity/planning). Falling back to Robust Joint Move...")
                # Fallback: Lift Joint 3 and Joint 2 to guarantee vertical clearance
                safe_joints = self.uf850.current_joint_positions.copy()
                safe_joints['uf850_joint3'] = -1.8 # Lift elbow
                safe_joints['uf850_joint2'] = -0.2 # Lift shoulder slightly
                if not self.uf850.move_to_joint_positions(safe_joints, velocity=0.2):
                    print("❌ [CRITICAL] Both Cartesian and Joint retract failed.")
                    return False
            self.wait_for_arm_settled()

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
                    print("⚠️ [PRE-CONDITION] Arm not at home. Retracting 30cm first...")
                    if not self.uf850.retract_relative_z(0.30, velocity=0.1):
                        print("⚠️ Cartesian retract failed. Falling back to Joint Move...")
                        safe_joints = self.uf850.current_joint_positions.copy()
                        safe_joints['uf850_joint3'] = -1.8
                        safe_joints['uf850_joint2'] = -0.2
                        if not self.uf850.move_to_joint_positions(safe_joints, velocity=0.2):
                            print("❌ [CRITICAL] Both retract methods failed.")
                            return False
                    self.wait_for_arm_settled()

                    print("🏠 Homing UF850...")
                    if not self.uf850.move_to_joint_positions(self.UF_HOME_JOINTS, velocity=0.2):
                        return False
                    self.wait_for_arm_settled()

        # --- STEP 1: CLEAR WORKSPACE ---
        print("🏠 Clearing xArm5 workspace...")
        if not self.xarm5.move_to_joint_positions({'xarm5_joint1': 0.0, 'xarm5_joint2': 0.0, 'xarm5_joint3': -1.57, 'xarm5_joint4': 1.57, 'xarm5_joint5': 0.0}):
            return False
        self.wait_for_arm_settled(self.xarm5)

        # --- STEP 2: COORDINATE TRANSFORM ---
        print(f"🔎 Using current live vision snapshot to find {target_label}...")
        time.sleep(0.3)

        target = None
        with self.data_lock:
            # 1. Try to find by ID first
            target = next((t for t in self.latest_targets if t.get('id') == target_id), None)
            
            # 2. Fallback: If ID is stale, find by label in the current live snapshot.
            if not target:
                print(f"⚠️ ID {target_id} not present. Searching by label '{target_label}'...")
                target = next((t for t in self.latest_targets if target_label.lower() in t.get('label', '').lower()), None)

        if not target: 
            print(f"❌ [ERROR] {target_label} not found in current vision snapshot. Aborting.")
            return False
        
        print(f"🎯 Targeted {target.get('label')} at {target['xyz']}")
        raw_p = Pose()
        raw_p.position.x, raw_p.position.y, raw_p.position.z = target['xyz']
        world_p = self.uf850.get_transformed_pose(raw_p, 'camera_color_optical_frame', 'base_link')
        if not world_p: return False

        tx, ty = world_p.pose.position.x + self.APPROACH_X_OFFSET, world_p.pose.position.y + self.APPROACH_Y_OFFSET
        final_z = world_p.pose.position.z + self.UF_TOOL_LENGTH
        hover_z = final_z + self.HOVER_HEIGHT

        # --- STEP 3: APPROACH & CLOSED-LOOP DESCENT ---
        print("🔓 Opening Gripper for Approach...")
        self.publish_state("MOVING")
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: math.radians(self.OPEN_DEG)},
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N
        ): return False
        
        print(f"🚁 Hovering at {tx:.3f}, {ty:.3f}...")
        if not self.uf850.move_to_pose_exotica(tx, ty, hover_z, velocity=0.1): return False
        self.wait_for_arm_settled()

        print("🗜️ Closing Gripper to 0 radians for search...")
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: 0.0},
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N
        ): return False
        self.wait_for_gripper(0.0)

        print("⬇️ EXOTica stepped tactile descent...")
        if not self.uf850.move_linear_z_with_effort_stop_exotica(
            descent_distance_m=self.HOVER_HEIGHT + 0.02,
            step_m=0.0005,
            threshold_nm=self.TORQUE_THRESHOLD,
            joint_index=2,
        ):
            return False
        self.wait_for_arm_settled()

        print("⬆️ Retracting 10mm after contact...")
        if not self.uf850.retract_servo_z_closed_loop(0.01, speed_mps=self.POST_GRASP_RETRACT_SPEED): return False
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
        write_hold_state(True)

        print("⬆️ Final Retract 30mm...")
        if not self.uf850.retract_servo_z_closed_loop(0.03, speed_mps=self.POST_GRASP_RETRACT_SPEED): return False
        self.wait_for_arm_settled()

        # --- STEP 5: DROP-OFF ---
        print("🗑️ Moving to Drop Pose...")
        if not self.uf850.move_to_pose_exotica(self.DROP_POSE['x'], self.DROP_POSE['y'], self.DROP_POSE['z'], velocity=0.1): return False
        self.wait_for_arm_settled()

        print("🎉 Finalizing: Release & Home...")
        self.publish_state("IDLE")
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: math.radians(self.OPEN_DEG)},
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N
        ): return False
        self.wait_for_gripper(self.OPEN_DEG)
        self.hold_status_pub.publish(Bool(data=False))
        write_hold_state(False)
        
        if not self.uf850.move_to_joint_positions(self.UF_HOME_JOINTS, velocity=0.2): return False
        
        print("✅ [SUCCESS] Sequence Complete.")
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
    node = PickupSkill(device_cfg=device_cfg)
    executor = MultiThreadedExecutor(); executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(1.0)
    # Check file-based hold state as fallback (survives process death)
    if not node.is_holding_object and read_hold_state():
        print("📦 Hold state detected from file (previous skill). Proceeding...")
        node.is_holding_object = True
    try:
        while rclpy.ok():
            tid = None
            with node.data_lock:
                target = node._select_pickup_target()
                if target:
                    tid = target.get('id')
                    label = target.get('label', 'pcb')
                else:
                    label = "pcb"
            if tid and node.execute_pickup(tid, label): break
            time.sleep(0.5)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
