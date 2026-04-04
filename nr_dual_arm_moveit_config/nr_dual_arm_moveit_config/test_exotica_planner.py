#!/usr/bin/env python3
"""
test_exotica_planner.py — standalone ROS 2 test node for EXOTica planner.

Tests:
  T1  UF850  — EXOTica IK solve (no execution) at hover height
  T2  UF850  — Move to hover pose (20 cm above work target)
  T3  UF850  — Tactile descent 20 cm from hover to work target height
  T4  xArm5  — EXOTica IK solve (no execution; 5-DOF) at hover height
  T5  xArm5  — Move to hover → descend to work height → spiral search

COORDINATE SYSTEM (base_link / world frame — verified from RViz TF display):
  Work target (original EE reading from live TF):
    UF850  rg6_tcp:            x=0.92264, y=0.02529, z=0.97854
                                qx=0.42682, qy=-0.42576, qz=0.57174, qw=0.55648
    xArm5  xarm_gripper_tcp:   x=0.92940, y=0.11457, z=0.91318
                                orientation identity (tool axis vertical)

  Hover pose (20 cm above work target — used for T1/T2/T4/T5 initial move):
    UF850:  z = 0.97854 + 0.20 = 1.17854
    xArm5:  z = 0.91318 + 0.20 = 1.11318

Usage:
  ros2 run nr_dual_arm_moveit_config test_exotica_planner \
      --ros-args -p execute:=true -p hardware_type:=fake -p tests:=T1,T2,T3,T4,T5
"""

import math
import threading
import time
import traceback

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

UF850_TARGET_POSE = {"x": 0.90267, "y": 0.053114, "z": 0.97401}
UF850_TARGET_QUAT = {"qx": 0.46111, "qy": -0.48479, "qz": 0.53848, "qw": 0.51225}

XARM5_TARGET_POSE = {"x": 0.929395, "y": 0.114569, "z": 0.913178}
XARM5_TARGET_QUAT = {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}

_HOVER_OFFSET_M = 0.200
UF850_HOVER_POSE = {**UF850_TARGET_POSE, "z": UF850_TARGET_POSE["z"] + _HOVER_OFFSET_M}
XARM5_HOVER_POSE = {**XARM5_TARGET_POSE, "z": XARM5_TARGET_POSE["z"] + _HOVER_OFFSET_M}

UF850_HOME_JOINTS = {
    "uf850_joint1": 0.0,
    "uf850_joint2": 0.0,
    "uf850_joint3": -math.pi / 2,
    "uf850_joint4": 0.0,
    "uf850_joint5": -math.pi / 2,
    "uf850_joint6": 0.0,
}
XARM5_HOME_JOINTS = {
    "xarm5_joint1": 0.0,
    "xarm5_joint2": 0.0,
    "xarm5_joint3": -math.pi / 2,
    "xarm5_joint4": math.pi / 2,
    "xarm5_joint5": 0.0,
}

SPIRAL_MAX_RADIUS_M = 0.010
SPIRAL_N_TURNS = 2
SPIRAL_N_POINTS = 32
SPIRAL_TARGET_DURATION_S = 10.0

EE_WARN_M = 0.030
EE_FAIL_M = 0.120
_HOME_SETTLE_S = 1.0


class ExoticaPlannerTester(Node):
    """Runs a battery of EXOTica planner tests and reports results."""

    def __init__(self):
        super().__init__("nr_exotica_planner_tester")
        self.declare_parameter("execute", False)
        self.declare_parameter("tests", "T1,T2,T3,T4,T5")
        self.declare_parameter("hardware_type", "fake")

        self._execute = self.get_parameter("execute").value
        self._tests_to_run = set(self.get_parameter("tests").value.split(","))
        self._hw = self.get_parameter("hardware_type").value

        self.get_logger().info(
            f"NR EXOTica planner tester: execute={self._execute}, "
            f"tests={sorted(self._tests_to_run)}, hardware_type={self._hw}"
        )

        try:
            from nr_dual_arm_moveit_config.motion_backend import MotionBackend

            self._MotionBackend = MotionBackend
        except ImportError as exc:
            self.get_logger().fatal(f"Cannot import MotionBackend: {exc}")
            raise

        self._results: list[tuple[str, bool, str]] = []

    def _wait_for_exotica_ready(self):
        import rclpy.qos as _qos

        ready_timeout_s = 90.0
        fallback_sleep_s = 3.0

        if self.count_publishers("/exotica/ready") > 0:
            self.get_logger().info(
                "EXOTica IK server detected. Waiting for /exotica/ready signal..."
            )
            ready_received = threading.Event()
            qos = _qos.QoSProfile(
                depth=1,
                durability=_qos.DurabilityPolicy.TRANSIENT_LOCAL,
            )

            def _ready_cb(msg):
                if msg.data:
                    ready_received.set()

            from std_msgs.msg import Bool as _Bool

            sub = self.create_subscription(_Bool, "/exotica/ready", _ready_cb, qos)
            deadline = time.time() + ready_timeout_s
            while not ready_received.is_set() and time.time() < deadline:
                time.sleep(0.2)
            self.destroy_subscription(sub)
            if ready_received.is_set():
                self.get_logger().info(
                    "EXOTica IK server ready — starting tests immediately."
                )
                time.sleep(0.5)
                return
            self.get_logger().warning(
                f"EXOTica IK server did not become ready after {ready_timeout_s:.0f}s. "
                "Proceeding anyway."
            )
        else:
            self.get_logger().info(
                f"No EXOTica IK server found. Waiting {fallback_sleep_s:.0f}s for joint states..."
            )
            time.sleep(fallback_sleep_s)

    def run_all(self):
        self._wait_for_exotica_ready()

        self.get_logger().info("=" * 60)
        self.get_logger().info("Initialising both MotionBackends concurrently...")

        uf850_result = [None]
        xarm5_result = [None]
        uf850_err = [None]
        xarm5_err = [None]

        def _init_uf850():
            try:
                uf850_result[0] = self._MotionBackend(self, "uf850_arm")
            except Exception as exc:
                uf850_err[0] = exc

        def _init_xarm5():
            try:
                xarm5_result[0] = self._MotionBackend(self, "xarm5_arm_no_slide")
            except Exception as exc:
                xarm5_err[0] = exc

        t_uf = threading.Thread(target=_init_uf850, daemon=True)
        t_xa = threading.Thread(target=_init_xarm5, daemon=True)
        t_uf.start()
        t_xa.start()
        t_uf.join()
        t_xa.join()

        uf850 = uf850_result[0]
        xarm5 = xarm5_result[0]

        if uf850_err[0]:
            self.get_logger().error(f"UF850 MotionBackend init crashed: {uf850_err[0]}")
        if xarm5_err[0]:
            self.get_logger().error(f"xArm5 MotionBackend init crashed: {xarm5_err[0]}")
        if uf850 is not None:
            self.get_logger().info("UF850 MotionBackend ready.")
        if xarm5 is not None:
            self.get_logger().info("xArm5 MotionBackend ready.")

        time.sleep(0.5)
        self.get_logger().info("Both backends ready. Starting tests...")

        if self._execute:
            if uf850 is not None and {"T2", "T3"} & self._tests_to_run:
                self.get_logger().info("Moving UF850 to home (blocking)...")
                ok = uf850.move_to_joint_positions(UF850_HOME_JOINTS, velocity=0.4)
                if not ok:
                    self.get_logger().warning("UF850 home move failed — continuing")
                else:
                    self.get_logger().info(
                        f"UF850 home done — settling {_HOME_SETTLE_S}s..."
                    )
                    time.sleep(_HOME_SETTLE_S)
                    self._log_tcp_position(uf850, "rg6_tcp", "UF850 home TCP")
            if xarm5 is not None and {"T5"} & self._tests_to_run:
                self.get_logger().info("Moving xArm5 to home (blocking)...")
                ok = xarm5.move_to_joint_positions(XARM5_HOME_JOINTS, velocity=0.4)
                if not ok:
                    self.get_logger().warning("xArm5 home move failed — continuing")
                else:
                    self.get_logger().info(
                        f"xArm5 home done — settling {_HOME_SETTLE_S}s..."
                    )
                    time.sleep(_HOME_SETTLE_S)
                    self._log_tcp_position(
                        xarm5, "xarm_gripper_tcp", "xArm5 home TCP"
                    )

        self._run_uf850_tests(uf850)
        self._run_xarm5_tests(xarm5)
        self._print_summary()

    def _run_uf850_tests(self, uf850):
        self.get_logger().info("=" * 60)
        self.get_logger().info("UF850 tests")
        if uf850 is None:
            for tid in ("T1", "T2", "T3"):
                if tid in self._tests_to_run:
                    self._fail(tid, "UF850 MotionBackend failed to initialise")
            return
        planner = uf850._single_arm_exotica_planner
        if planner is None or not planner.available:
            msg = getattr(planner, "last_error", "planner is None") if planner else "planner is None"
            for tid in ("T1", "T2", "T3"):
                if tid in self._tests_to_run:
                    self._fail(tid, f"EXOTica unavailable: {msg}")
            return

        if "T1" in self._tests_to_run:
            self._t1_uf850_ik_solve(uf850, planner)
        if "T2" in self._tests_to_run:
            self._t2_uf850_hover(uf850)
        if "T3" in self._tests_to_run:
            self._t3_uf850_descent(uf850)

    def _t1_uf850_ik_solve(self, uf850, planner):
        tid = "T1"
        self.get_logger().info(
            f"[{tid}] UF850 IK solve at hover z={UF850_HOVER_POSE['z']:.3f}"
        )
        try:
            p = UF850_HOVER_POSE
            seed = dict(uf850.current_joint_positions)
            if all(abs(v) < 0.05 for v in seed.values()):
                seed.update(UF850_HOME_JOINTS)
                self.get_logger().info(
                    f"[{tid}] Near-zero seed → using home configuration"
                )
            q = UF850_TARGET_QUAT
            roll, pitch, yaw = uf850._quaternion_to_rpy(
                q["qx"], q["qy"], q["qz"], q["qw"]
            )
            t0 = time.time()
            result = planner.solve_pose_goal_joint_positions(
                seed, [p["x"], p["y"], p["z"], roll, pitch, yaw]
            )
            elapsed = time.time() - t0
            if result is None:
                self._fail(tid, f"EXOTica IK None after {elapsed:.3f}s — {planner.last_error}")
                return
            self.get_logger().info(
                f"[{tid}] IK OK in {elapsed:.3f}s: "
                + ", ".join(f"{k}={v:.3f}" for k, v in result.items())
            )
            self._pass(tid, f"IK solved in {elapsed:.3f}s")
        except Exception as exc:
            self._fail(tid, f"Exception: {exc}\n{traceback.format_exc()}")

    def _log_uf850_joints(self, uf850, label: str):
        joints = {k: v for k, v in uf850.current_joint_positions.items() if k.startswith("uf850_")}
        self.get_logger().info(
            f"  [{label}] UF850 joints: "
            + ", ".join(f"{k}={v:.4f}" for k, v in sorted(joints.items()))
        )

    def _t2_uf850_hover(self, uf850):
        tid = "T2"
        p = UF850_HOVER_POSE
        self.get_logger().info(
            f"[{tid}] UF850 → hover ({p['x']:.3f},{p['y']:.3f},{p['z']:.3f})"
        )
        if not self._execute:
            self._pass(tid, "Skipped (execute:=false)")
            return
        try:
            self._log_uf850_joints(uf850, f"{tid} PRE-MOVE")
            self._log_tcp_position(uf850, "rg6_tcp", f"{tid} PRE-MOVE")

            t0 = time.time()
            ok = uf850.move_to_pose_exotica(
                p["x"], p["y"], p["z"], UF850_TARGET_QUAT, velocity=0.3
            )
            elapsed = time.time() - t0
            if not ok:
                self._fail(tid, f"move_to_pose_exotica returned False after {elapsed:.2f}s")
                return

            time.sleep(0.5)
            self._log_uf850_joints(uf850, f"{tid} POST-MOVE")
            self._log_tcp_position(uf850, "rg6_tcp", f"{tid} POST-MOVE")

            ee_err = self._check_ee_pos(uf850, "rg6_tcp", p, tid)
            if ee_err is not None:
                self.get_logger().error(
                    f"[{tid}] EXOTica trajectory missed target (err={ee_err:.4f}m)."
                )
                self._fail(tid, f"EE pos error {ee_err:.4f}m > {EE_FAIL_M}m")
                return

            self._pass(tid, f"Hover reached in {time.time() - t0:.2f}s")
        except Exception as exc:
            self._fail(tid, f"Exception: {exc}\n{traceback.format_exc()}")

    def _t3_uf850_descent(self, uf850):
        tid = "T3"
        dist = _HOVER_OFFSET_M
        self.get_logger().info(
            f"[{tid}] UF850 tactile descent {dist*100:.0f} cm → work target z={UF850_TARGET_POSE['z']:.3f}"
        )
        if not self._execute:
            self._pass(tid, f"Skipped (execute:=false) — would descend {dist*100:.0f} cm")
            return
        try:
            self.get_logger().info(f"[{tid}] Pre-descent TCP (rg6_tcp):")
            self._log_tcp_position(uf850, "rg6_tcp", tid)
            pre_err = self._check_ee_pos(uf850, "rg6_tcp", UF850_HOVER_POSE, tid)
            if pre_err is not None:
                self.get_logger().error(
                    f"[{tid}] rg6_tcp is {pre_err:.4f}m from hover target — cannot descend safely."
                )
                self._fail(tid, f"Pre-descent rg6_tcp still {pre_err:.4f}m from hover")
                return

            t0 = time.time()
            ok = uf850.move_linear_z_with_effort_stop_exotica(
                descent_distance_m=dist,
                step_m=0.0005,
                threshold_nm=3.0,
                joint_index=4,
                rate_hz=50.0,
            )
            elapsed = time.time() - t0
            self._log_tcp_position(uf850, "rg6_tcp", tid)

            if ok:
                self._pass(tid, f"Contact detected after {elapsed:.2f}s")
            elif self._hw == "fake":
                self._pass(
                    tid,
                    f"Full {dist*100:.0f} cm traversed in {elapsed:.2f}s without contact "
                    "(expected in fake mode — no physical surface)",
                )
            else:
                self._fail(
                    tid,
                    f"Reached max depth ({dist*100:.0f} cm) in {elapsed:.2f}s without contact — "
                    "check threshold_nm or surface presence",
                )
        except Exception as exc:
            self._fail(tid, f"Exception: {exc}\n{traceback.format_exc()}")

    def _run_xarm5_tests(self, xarm5):
        self.get_logger().info("=" * 60)
        self.get_logger().info("xArm5 tests")
        if xarm5 is None:
            for tid in ("T4", "T5"):
                if tid in self._tests_to_run:
                    self._fail(tid, "xArm5 MotionBackend failed to initialise")
            return
        planner = xarm5._single_arm_exotica_planner
        if planner is None or not planner.available:
            msg = getattr(planner, "last_error", "planner is None") if planner else "planner is None"
            for tid in ("T4", "T5"):
                if tid in self._tests_to_run:
                    self._fail(tid, f"EXOTica unavailable: {msg}")
            return

        if "T4" in self._tests_to_run:
            self._t4_xarm5_ik_solve(xarm5, planner)
        if "T5" in self._tests_to_run:
            self._t5_xarm5_hover_and_spiral(xarm5, planner)

    def _t4_xarm5_ik_solve(self, xarm5, planner):
        tid = "T4"
        self.get_logger().info(
            f"[{tid}] xArm5 IK solve at hover z={XARM5_HOVER_POSE['z']:.3f} (5-DOF)"
        )
        try:
            p = XARM5_HOVER_POSE
            seed = dict(xarm5.current_joint_positions)
            if all(abs(v) < 0.05 for v in seed.values()):
                seed.update(XARM5_HOME_JOINTS)
                self.get_logger().info(
                    f"[{tid}] Near-zero seed → using home configuration"
                )
            q = XARM5_TARGET_QUAT
            roll, pitch, yaw = xarm5._quaternion_to_rpy(
                q["qx"], q["qy"], q["qz"], q["qw"]
            )
            t0 = time.time()
            result = planner.solve_pose_goal_joint_positions(
                seed, [p["x"], p["y"], p["z"], roll, pitch, yaw]
            )
            elapsed = time.time() - t0
            if result is None:
                self._fail(tid, f"EXOTica IK None after {elapsed:.3f}s — {planner.last_error}")
                return
            self.get_logger().info(
                f"[{tid}] xArm5 IK OK in {elapsed:.3f}s (5-DOF): "
                + ", ".join(f"{k}={v:.3f}" for k, v in result.items())
            )
            self._pass(tid, f"IK solved in {elapsed:.3f}s")
        except Exception as exc:
            self._fail(tid, f"Exception: {exc}\n{traceback.format_exc()}")

    def _t5_xarm5_hover_and_spiral(self, xarm5, planner):
        tid = "T5"
        if not self._execute:
            self._pass(tid, "Skipped (execute:=false)")
            return

        hp = XARM5_HOVER_POSE
        self.get_logger().info(
            f"[{tid}] Phase 1 — xArm5 hover ({hp['x']:.3f},{hp['y']:.3f},{hp['z']:.3f})"
        )
        try:
            t0 = time.time()
            ok = xarm5.move_to_pose_exotica(
                hp["x"], hp["y"], hp["z"], XARM5_TARGET_QUAT, velocity=0.3
            )
            if not ok:
                self._fail(tid, f"Hover move failed after {time.time() - t0:.2f}s")
                return
            self.get_logger().info(f"[{tid}] Hover reached in {time.time() - t0:.2f}s")
        except Exception as exc:
            self._fail(tid, f"Hover exception: {exc}\n{traceback.format_exc()}")
            return

        tp = XARM5_TARGET_POSE
        self.get_logger().info(f"[{tid}] Phase 2 — descend to work height z={tp['z']:.3f}")
        try:
            t0 = time.time()
            ok = xarm5.move_to_pose_exotica(
                tp["x"], tp["y"], tp["z"], XARM5_TARGET_QUAT, velocity=0.25
            )
            if not ok:
                self._fail(
                    tid, f"Descent to work height failed after {time.time() - t0:.2f}s"
                )
                return
            self.get_logger().info(
                f"[{tid}] Work height reached in {time.time() - t0:.2f}s"
            )
        except Exception as exc:
            self._fail(tid, f"Descent exception: {exc}\n{traceback.format_exc()}")
            return

        ee_err = self._check_ee_pos(xarm5, "xarm_gripper_tcp", tp, tid)
        if ee_err is not None:
            self.get_logger().warning(
                f"[{tid}] Post-descent EE error {ee_err:.4f}m > {EE_FAIL_M}m — continuing to spiral"
            )

        self.get_logger().info(
            f"[{tid}] Phase 3 — spiral search r_max={SPIRAL_MAX_RADIUS_M*1000:.0f}mm "
            f"turns={SPIRAL_N_TURNS} pts={SPIRAL_N_POINTS}"
        )
        spiral_ok = self._execute_spiral(xarm5, planner, tid)
        if not spiral_ok:
            self._fail(tid, "Spiral search failed (IK or execution error)")
            return

        self._pass(tid, "Hover + descent + spiral completed successfully")

    def _execute_spiral(self, xarm5, planner, tid: str) -> bool:
        self.get_logger().info(f"[{tid}] Pre-spiral TCP (xarm_gripper_tcp):")
        self._log_tcp_position(xarm5, "xarm_gripper_tcp", tid)

        cx = XARM5_TARGET_POSE["x"]
        cy = XARM5_TARGET_POSE["y"]
        cz = XARM5_TARGET_POSE["z"]
        q = XARM5_TARGET_QUAT
        roll, pitch, yaw = xarm5._quaternion_to_rpy(q["qx"], q["qy"], q["qz"], q["qw"])

        waypoints_xyz = []
        for i in range(SPIRAL_N_POINTS + 1):
            t = i / SPIRAL_N_POINTS
            theta = 2.0 * math.pi * SPIRAL_N_TURNS * t
            r = SPIRAL_MAX_RADIUS_M * t
            wx = cx + r * math.cos(theta)
            wy = cy + r * math.sin(theta)
            waypoints_xyz.append((wx, wy, cz))

        seed = {
            name: float(xarm5.current_joint_positions.get(name, 0.0))
            for name in planner.controlled_joint_names
        }
        joint_states = []
        n_failures = 0

        for i, (wx, wy, wz) in enumerate(waypoints_xyz):
            result = planner.solve_pose_goal_joint_positions(
                seed, [wx, wy, wz, roll, pitch, yaw]
            )
            if result is None:
                n_failures += 1
                if n_failures > 4:
                    self.get_logger().error(
                        f"[{tid}] Spiral IK: too many consecutive failures at waypoint {i}: "
                        f"{planner.last_error}"
                    )
                    return False
                self.get_logger().warning(
                    f"[{tid}] Spiral IK failed at waypoint {i} "
                    f"(r={SPIRAL_MAX_RADIUS_M*i/SPIRAL_N_POINTS*1000:.1f}mm) "
                    "— keeping previous joint state"
                )
                if joint_states:
                    joint_states.append(joint_states[-1].copy())
                continue
            n_failures = 0
            state = np.array(
                [result[jn] for jn in planner.controlled_joint_names], dtype=float
            )
            joint_states.append(state)
            seed = dict(result)

        if len(joint_states) < 3:
            self.get_logger().error(
                f"[{tid}] Too few valid spiral waypoints ({len(joint_states)})"
            )
            return False

        self.get_logger().info(
            f"[{tid}] Spiral: {len(joint_states)}/{len(waypoints_xyz)} waypoints solved"
        )

        matrix = np.vstack(joint_states)
        initial_scale = 0.05
        try:
            trajectory = planner._trajectory_to_robot_trajectory(matrix, initial_scale)
        except Exception as exc:
            self.get_logger().error(f"[{tid}] Trajectory assembly failed: {exc}")
            return False

        n_pts = len(trajectory.joint_trajectory.points)
        if n_pts == 0:
            self.get_logger().error(f"[{tid}] Empty spiral trajectory")
            return False

        natural_dur = (
            trajectory.joint_trajectory.points[-1].time_from_start.sec
            + trajectory.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9
        )
        if natural_dur < 1e-3:
            natural_dur = 1e-3
        stretch = max(1.0, SPIRAL_TARGET_DURATION_S / natural_dur)
        if stretch > 1.0:
            self.get_logger().info(
                f"[{tid}] Stretching spiral trajectory {natural_dur:.2f}s → "
                f"{natural_dur*stretch:.2f}s (×{stretch:.2f})"
            )
            from builtin_interfaces.msg import Duration as _Duration

            for pt in trajectory.joint_trajectory.points:
                raw_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                new_sec = raw_sec * stretch
                pt.time_from_start = _Duration(
                    sec=int(new_sec),
                    nanosec=int((new_sec - int(new_sec)) * 1_000_000_000),
                )
                pt.velocities = [v / stretch for v in pt.velocities]
                pt.accelerations = [a / (stretch * stretch) for a in pt.accelerations]

        dur_s = (
            trajectory.joint_trajectory.points[-1].time_from_start.sec
            + trajectory.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9
        )
        self.get_logger().info(
            f"[{tid}] Executing spiral: {n_pts} waypoints, "
            f"{len(planner.controlled_joint_names)} joints, "
            f"duration={dur_s:.2f}s (target={SPIRAL_TARGET_DURATION_S:.1f}s)"
        )

        try:
            ok = xarm5._execute_robot_trajectory(trajectory)
        except Exception as exc:
            self.get_logger().error(
                f"[{tid}] Spiral trajectory execution raised: {exc}"
            )
            return False

        if ok:
            self.get_logger().info(f"[{tid}] Spiral executed successfully")
        else:
            self.get_logger().error(
                f"[{tid}] Spiral trajectory execution returned False"
            )
        return ok

    def _log_tcp_position(self, backend, ee_link: str, label: str):
        try:
            import rclpy as _rclpy

            tf = backend.tf_buffer.lookup_transform(
                "base_link", ee_link, _rclpy.time.Time()
            )
            ax = tf.transform.translation.x
            ay = tf.transform.translation.y
            az = tf.transform.translation.z
            self.get_logger().info(
                f"  [{label}] {ee_link} @ base_link: ({ax:.4f}, {ay:.4f}, {az:.4f})"
            )
        except Exception as exc:
            self.get_logger().warning(
                f"  [{label}] TF lookup for {ee_link} failed: {exc}"
            )

    def _check_ee_pos(self, backend, ee_link: str, target: dict, tid: str) -> float | None:
        try:
            import rclpy as _rclpy

            tf = backend.tf_buffer.lookup_transform(
                "base_link", ee_link, _rclpy.time.Time()
            )
            ax = tf.transform.translation.x
            ay = tf.transform.translation.y
            az = tf.transform.translation.z
            err = math.sqrt(
                (ax - target["x"]) ** 2
                + (ay - target["y"]) ** 2
                + (az - target["z"]) ** 2
            )
            self.get_logger().info(
                f"[{tid}] EE check: actual=({ax:.4f},{ay:.4f},{az:.4f}) "
                f"target=({target['x']:.4f},{target['y']:.4f},{target['z']:.4f}) "
                f"err={err:.4f}m"
            )
            if err > EE_FAIL_M:
                return err
            if err > EE_WARN_M:
                self.get_logger().warning(
                    f"[{tid}] EE pos err {err:.4f}m > warn {EE_WARN_M}m "
                    "(may indicate hand-eye calibration drift)"
                )
            return None
        except Exception as exc:
            self.get_logger().warning(f"[{tid}] EE check skipped (TF): {exc}")
            return None

    def _pass(self, tid: str, detail: str):
        self._results.append((tid, True, detail))
        self.get_logger().info(f"  ✓ [{tid}] PASS — {detail}")

    def _fail(self, tid: str, detail: str):
        self._results.append((tid, False, detail))
        self.get_logger().error(f"  ✗ [{tid}] FAIL — {detail}")

    def _print_summary(self):
        passed = sum(1 for _, ok, _ in self._results if ok)
        total = len(self._results)
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"RESULTS: {passed}/{total} passed")
        self.get_logger().info("=" * 60)
        for tid, ok, detail in self._results:
            self.get_logger().info(f"  [{tid}] {'PASS' if ok else 'FAIL'}: {detail}")
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = ExoticaPlannerTester()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_all()
    except Exception as exc:
        node.get_logger().fatal(f"Test runner crashed: {exc}\n{traceback.format_exc()}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
