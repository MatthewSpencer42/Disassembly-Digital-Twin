#!/usr/bin/env python3
"""
exotica_ik_server_node.py — Pre-warms both EXOTica single-arm planners and serves IK requests.

Published topics:
  /exotica/ready  (std_msgs/Bool, TransientLocal)  — True once both planners are initialized.
  /exotica_ik/response  (std_msgs/String)  — JSON IK response.

Subscribed topics:
  /exotica_ik/request  (std_msgs/String)  — JSON IK request.

Request JSON:  {"id": str, "group": str, "current": {name: val}, "pose_rpy": [x,y,z,r,p,y]}
Response JSON: {"id": str, "joints": {name: val}|null, "error": str|null}
"""
from __future__ import annotations

import json
import threading
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


class ExoticaIKServerNode(Node):
    """Pre-warms EXOTica IK planners for both arms and serves IK requests over topics."""

    _GROUPS = ("uf850_arm", "xarm5_arm_no_slide")

    def __init__(self):
        super().__init__("exotica_ik_server")

        self.declare_parameter("hardware_type", "fake")
        self._hardware_type: str = (
            self.get_parameter("hardware_type").get_parameter_value().string_value
        )

        # Planners — keyed by group name, populated in background threads.
        self._planners: dict = {}
        self._planners_lock = threading.Lock()
        # Per-group solve locks: serialise concurrent IK requests for the same
        # arm so _problem / _solver shared state is never touched by two threads.
        self._solve_locks: dict = {}  # group -> threading.Lock()

        # Thread pool: 4 workers lets requests queue and drain quickly while
        # per-group solve locks ensure the planner's shared state is safe.
        self._executor_pool = ThreadPoolExecutor(max_workers=4)

        # /exotica/ready publisher — TransientLocal so late subscribers get the message.
        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_pub = self.create_publisher(Bool, "/exotica/ready", ready_qos)

        # /exotica_ik/response publisher and /exotica_ik/request subscriber.
        self._response_pub = self.create_publisher(String, "/exotica_ik/response", 10)
        self._request_sub = self.create_subscription(
            String, "/exotica_ik/request", self._on_request, 10
        )

        # Kick off planner initialization in the background so the node spins immediately.
        init_thread = threading.Thread(target=self._init_planners, daemon=True)
        init_thread.start()

    # ------------------------------------------------------------------
    # Planner initialization
    # ------------------------------------------------------------------

    def _init_planners(self) -> None:
        """Initialize both EXOTica planners sequentially in a background thread."""
        from dual_arm_moveit_config.exotica_planner import ExoticaSingleArmPosePlanner

        all_ok = True
        for group in self._GROUPS:
            self.get_logger().info(
                f"[EXOTica IK server] Initializing planner for group '{group}' …"
            )
            try:
                planner = ExoticaSingleArmPosePlanner(
                    self, group, hardware_type=self._hardware_type
                )
                with self._planners_lock:
                    self._planners[group] = planner
                    self._solve_locks[group] = threading.Lock()
                if planner.available:
                    self.get_logger().info(
                        f"[EXOTica IK server] Planner for '{group}' ready."
                    )
                else:
                    self.get_logger().error(
                        f"[EXOTica IK server] Planner for '{group}' failed to initialize: "
                        f"{planner.last_error}"
                    )
                    all_ok = False
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"[EXOTica IK server] Exception while initializing '{group}': {exc}"
                )
                all_ok = False

        if all_ok:
            self.get_logger().info(
                "EXOTica IK server ready (both planners initialized)"
            )
        else:
            self.get_logger().warn(
                "[EXOTica IK server] One or more planners failed; server running in degraded mode."
            )

        msg = Bool()
        msg.data = all_ok
        self._ready_pub.publish(msg)

    # ------------------------------------------------------------------
    # Request handler
    # ------------------------------------------------------------------

    def _on_request(self, msg: String) -> None:
        """Receive a JSON IK request and dispatch it to the thread pool."""
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f"[EXOTica IK server] Invalid JSON in request: {exc}"
            )
            return

        request_id = request.get("id", "unknown")
        # Submit work to the pool; do not block the subscription callback.
        self._executor_pool.submit(self._handle_request, request_id, request)

    def _handle_request(self, request_id: str, request: dict) -> None:
        """Resolve an IK request and publish the response. Runs in the thread pool."""
        group = request.get("group", "")
        current = request.get("current", {})
        pose_rpy = request.get("pose_rpy", [])
        position_tolerance_m = request.get("position_tolerance_m")
        max_retries = min(int(request.get("max_retries", 10)), 10)

        joints: dict | None = None
        error: str | None = None

        try:
            with self._planners_lock:
                planner = self._planners.get(group)
                solve_lock = self._solve_locks.get(group)

            if planner is None:
                error = (
                    f"No planner available for group '{group}'. "
                    "Either the group name is wrong or initialization is still in progress."
                )
            elif not planner.available:
                error = (
                    f"Planner for '{group}' is not available: {planner.last_error}"
                )
            else:
                # Hold the per-group lock for the ENTIRE solve so that two
                # concurrent requests for the same arm cannot corrupt the
                # shared _problem / _solver state inside the planner.
                with solve_lock:
                    result = planner.solve_pose_goal_joint_positions(
                        current,
                        pose_rpy,
                        max_retries=max_retries,
                        position_tolerance_m=position_tolerance_m,
                    )
                if result is None:
                    error = planner.last_error or "IK returned no solution."
                else:
                    joints = result

        except Exception as exc:  # noqa: BLE001
            error = f"Unhandled exception during IK solve: {exc}"
            self.get_logger().error(f"[EXOTica IK server] {error}")

        response = {"id": request_id, "joints": joints, "error": error}
        out = String()
        out.data = json.dumps(response)
        self._response_pub.publish(out)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = ExoticaIKServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
