#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
import threading, json, math, time, os
from disassembly_skill.motion_backend import MotionBackend

HOLD_STATE_FILE = '/tmp/disassembly_hold_state'

def write_hold_state(held):
    try:
        with open(HOLD_STATE_FILE, 'w') as f:
            f.write('true' if held else 'false')
    except Exception:
        pass

class ObjectHoldSkill(Node):
    def __init__(self):
        super().__init__('object_hold_skill_node')

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
        self.vision_reset_pub = self.create_publisher(String, '/vision/reset_tracker', 10)

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
        self.APPROACH_VELOCITY = 0.1
        self.TORQUE_THRESHOLD = 3.0
        self.DESCENT_SPEED_MPS = 0.02
        self.DESCENT_STEP_M = 0.0005
        self.DESCENT_DISTANCE_M = 0.06
        self.DESCENT_RATE_HZ = 30.0
        self.RETRACT_VELOCITY = 0.05
        self.POST_GRASP_RETRACT_SPEED = 0.1

        self.get_logger().info("Object Hold Skill: Top-Down Cartesian Tactile Mode Active.")
        self.publish_state("IDLE")

    def _log_pose_diagnostics(self, target_data, world_xyz, hover_xyz, quaternion_dict):
        raw_xyz = target_data.get("xyz", [None, None, None])
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

    def publish_state(self, s):
        self.state_update_pub.publish(String(data=s))

    def publish_hold_status(self, h):
        self.hold_status_pub.publish(Bool(data=h))
        write_hold_state(h)

    def vision_callback(self, msg):
        try:
            raw_data = msg.data.strip().strip("'").strip('"')
            data = json.loads(raw_data)
            with self.data_lock:
                self.latest_targets = data.get("global_view", {}).get("objects", [])
        except Exception as exc:
            self.get_logger().warning(f"Failed to parse /vision/agent_state payload: {exc}")

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
        start_t = time.time()
        last_pos = 999.0
        stall_timer = 0.0
        is_closing = target_deg > 0

        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = self.gripper.current_joint_positions.get(self.JOINT_GRIPPER, 999)
            if curr == 999:
                time.sleep(0.1)
                continue
            if abs(curr - target_rad) < 0.05:
                return True
            if abs(curr - last_pos) < 0.002:
                stall_timer += 0.1
                if stall_timer >= 0.8:
                    if is_closing:
                        self.get_logger().info(f"Grasp confirmed (Force reached) at {curr:.3f} rad.")
                        return True
                    else:
                        self.get_logger().warn(f"Gripper STUCK at {curr:.3f} rad while trying to open. Retrying with high force...")
                        self.gripper.move_to_joint_positions({self.JOINT_GRIPPER: target_rad}, gripper_force_n=100.0)
                        stall_timer = -2.0
            else:
                stall_timer = 0.0
            last_pos = curr
            time.sleep(0.1)
        return False

    def _get_target_by_label(self, label):
        with self.data_lock:
            return next((t for t in self.latest_targets if label.lower() in t.get('label', '').lower()), None)

    def _run_hold_sequence(self, part_id, target_label, interactive):
        target_data = self._get_target_by_id(part_id)

        # Fallback: ID may have shifted after vision reset — match by label
        if not target_data or 'xyz' not in target_data:
            print(f"⚠️ ID {part_id} not found. Searching by label '{target_label}'...")
            target_data = self._get_target_by_label(target_label)

        if not target_data or 'xyz' not in target_data:
            print(f"ABORT: Vision data for '{target_label}' (ID {part_id}) is missing.")
            return False

        # --- Math & Transforms ---
        p = Pose()
        p.position.x, p.position.y, p.position.z = target_data["xyz"]
        p.orientation.w = 1.0

        t_pose = self.uf850.get_transformed_pose(p, self.CAMERA_FRAME, self.PLANNING_FRAME)
        if not t_pose:
            return False

        wx = t_pose.pose.position.x
        wy = t_pose.pose.position.y
        wz = t_pose.pose.position.z

        try:
            current_tf = self.uf850.tf_buffer.lookup_transform(
                self.PLANNING_FRAME,
                self.ROBOT_EE_LINK,
                rclpy.time.Time(),
            )
            qd = {
                'qx': float(current_tf.transform.rotation.x),
                'qy': float(current_tf.transform.rotation.y),
                'qz': float(current_tf.transform.rotation.z),
                'qw': float(current_tf.transform.rotation.w),
            }
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to read current {self.ROBOT_EE_LINK} orientation, using neutral fallback: {exc}"
            )
            qd = {'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}

        hz = wz + self.HOVER_Z_OFFSET
        hover_x = wx
        hover_y = wy

        self._log_pose_diagnostics(
            target_data=target_data,
            world_xyz=(wx, wy, wz),
            hover_xyz=(hover_x, hover_y, hz),
            quaternion_dict=qd,
        )

        # --- STEP 1: DIRECT HOVER ---
        self.publish_state("MOVING")
        if interactive: input(f"STEP 1: Hover sideways over {target_label}? [Enter]")

        print("Ensuring gripper is open...")
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: math.radians(self.OPEN_DEG)},
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N,
        ):
            return False
        if not self.wait_for_gripper(self.OPEN_DEG):
            self.get_logger().warning("Gripper did not confirm open position before hover move.")

        print(f"Moving to hover pose with EXOTica at X: {hover_x:.3f}, Y: {hover_y:.3f}, Z: {hz:.3f}...")
        if not self.uf850.move_to_pose_exotica(hover_x, hover_y, hz, qd, velocity=self.APPROACH_VELOCITY):
            print("[ERROR] EXOTica pose planning failed. Aborting; no MoveIt Cartesian fallback in EXOTica-only mode.")
            return False
        if not self.wait_for_arm_settled():
            print("[ERROR] Arm did not settle after hover move.")
            return False

        print("Starting EXOTica stepped tactile descent with effort spike stop...")
        if not self.uf850.move_linear_z_with_effort_stop_exotica(
            descent_distance_m=self.DESCENT_DISTANCE_M,
            step_m=self.DESCENT_STEP_M,
            threshold_nm=self.TORQUE_THRESHOLD,
            joint_index=4,
        ):
            print("[ERROR] Tactile descent failed before contact.")
            return False
        self.wait_for_arm_settled()

        print("Tactile descent complete. Gripper remains open as requested.")
        self.publish_state("HOLDING")
        return True

    def execute_hold(self, part_id, target_label, interactive=True):
        print(f"\n[START] {target_label} Hold Sequence on ID: {part_id}")
        self.publish_hold_status(False)
        self.publish_state("MOVING")
        success = False
        try:
            success = self._run_hold_sequence(part_id, target_label, interactive)
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
    node = ObjectHoldSkill()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(2.0)
    print("Single-Shot Mode: Waiting for initial vision data on /vision/agent_state...")

    print("Sending reset command to vision tracker...")
    node.vision_reset_pub.publish(String(data='reset'))
    time.sleep(1.0)

    is_held = False
    try:
        target_id = None
        target_label_to_pass = ""

        while rclpy.ok() and target_id is None:
            with node.data_lock:
                chassis_targets = [t for t in node.latest_targets if 'chassis' in t.get('label', '').lower() or 'lid' in t.get('label', '').lower()]
                if chassis_targets:
                    target_id = chassis_targets[0].get('id')
                    target_label_to_pass = chassis_targets[0].get('label', 'chassis')
            if target_id is None:
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
        # Persist final state to file so downstream skills can read it after this node dies
        write_hold_state(is_held)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
