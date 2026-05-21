#!/usr/bin/env python3
import json
import time

from vision_agent.runtime_env import setup_python_env

setup_python_env(__file__)

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from rclpy.qos import qos_profile_sensor_data

from vision_agent.common import (
    DASHBOARD_HEIGHT,
    DEBUG_JPEG_QUALITY,
    DEBUG_PUBLISH_RATE_HZ,
    GLOBAL_COLOR_DISPLAY_TOPIC,
    GLOBAL_RELIABLE_QOS,
    LOCAL_COLOR_TOPIC,
    PERF_LOG_INTERVAL_SEC,
    PROCESSING_RATE_HZ,
    PUBLISH_RAW_DEBUG,
)


class DashboardNode(Node):
    def __init__(self):
        super().__init__("vision_dashboard_node")
        self.get_logger().info("Starting dashboard node")

        self.bridge = CvBridge()
        self.frame_global = None
        self.frame_local = None
        self.global_state = {"objects": [], "bin_locations": {}}
        self.local_state = {"screws": [], "screw_heads": [], "tool_tips": [], "holes": [], "crosshair": []}
        self.assembly_state = {"state": "unknown", "confidence": 0.0}
        self.robot_states = {"tool_arm": "IDLE", "manip_arm": "IDLE"}
        self.latest_zeroed_wrench = None
        self.wrench_offset = None
        self._fps_time = time.time()
        self._fps_count = 0
        self._fps_display = 0.0
        self._global_rx_time = time.time()
        self._global_rx_count = 0
        self._global_rx_fps = 0.0
        self._local_rx_time = time.time()
        self._local_rx_count = 0
        self._local_rx_fps = 0.0
        self._perf_time = time.time()
        self._perf_stats = {"debug_ms": [0.0, 0]}
        self._global_state_time = 0.0
        self._local_state_time = 0.0
        self._assembly_state_time = 0.0
        self._robot_state_time = 0.0
        self._exotica_ready = False
        self._system_health = {}

        self.create_subscription(
            Image,
            GLOBAL_COLOR_DISPLAY_TOPIC,
            self.cb_global_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            LOCAL_COLOR_TOPIC,
            self.cb_local_image,
            10,
        )
        self.create_subscription(String, "/vision/global_state", self.cb_global_state, 10)
        self.create_subscription(String, "/vision/local_state", self.cb_local_state, 10)
        self.create_subscription(String, "/vision/assembly_state", self.cb_assembly_state, 10)
        # Legacy JSON bundle — kept as a fallback (may not be published)
        self.create_subscription(String, "/robot_states", self.cb_robot_states, 10)
        # Per-skill state topics — real-time updates from each skill
        self.create_subscription(String, "/robot_state/manip_arm/update", self.cb_manip_arm_state, 10)
        self.create_subscription(String, "/robot_state/tool_arm/update",  self.cb_tool_arm_state,  10)
        hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/object_hold_state/is_held", self.cb_hold_status, hold_qos)
        ready_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/exotica/ready", self.cb_exotica_ready, ready_qos)
        self.create_subscription(
            WrenchStamped,
            "/robotiq_force_torque_sensor_broadcaster/wrench",
            self.cb_wrench,
            10,
        )

        self.agent_state_pub = self.create_publisher(String, "/vision/agent_state", 10)
        self.debug_pub_raw = self.create_publisher(Image, "/vision/debug_feed", 10)
        self.debug_pub_compressed = self.create_publisher(CompressedImage, "/vision/debug_feed/compressed", 10)
        self.state_timer = self.create_timer(1.0 / PROCESSING_RATE_HZ, self.publish_agent_state)
        self.debug_timer = self.create_timer(1.0 / DEBUG_PUBLISH_RATE_HZ, self.publish_dashboard)
        self.health_timer = self.create_timer(1.0, self.update_system_health)

    def cb_global_image(self, msg):
        try:
            self.frame_global = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._global_rx_count += 1
            now = time.time()
            elapsed = now - self._global_rx_time
            if elapsed >= 1.0:
                self._global_rx_fps = self._global_rx_count / elapsed
                self._global_rx_count = 0
                self._global_rx_time = now
        except Exception:
            pass

    def cb_local_image(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            if img.shape[1] > 640:
                img = cv2.resize(img, (640, 480))
            self.frame_local = img
            self._local_rx_count += 1
            now = time.time()
            elapsed = now - self._local_rx_time
            if elapsed >= 1.0:
                self._local_rx_fps = self._local_rx_count / elapsed
                self._local_rx_count = 0
                self._local_rx_time = now
        except Exception:
            pass

    def cb_global_state(self, msg):
        try:
            self.global_state = json.loads(msg.data)
            self._global_state_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Global state parse error: {e}")

    def cb_local_state(self, msg):
        try:
            self.local_state = json.loads(msg.data)
            self._local_state_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Local state parse error: {e}")

    def cb_assembly_state(self, msg):
        try:
            self.assembly_state = json.loads(msg.data)
            self._assembly_state_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Assembly state parse error: {e}")

    def cb_robot_states(self, msg):
        """Legacy JSON bundle subscriber — overwritten by per-skill callbacks if they publish."""
        try:
            data = json.loads(msg.data)
            # Only apply if per-skill topics haven't already updated these fields
            now = time.time()
            if now - self._robot_state_time > 2.0:
                self.robot_states = data
                self._robot_state_time = now
        except Exception as e:
            self.get_logger().error(f"Robot state parse error: {e}")

    def cb_manip_arm_state(self, msg):
        self.robot_states["manip_arm"] = msg.data
        self._robot_state_time = time.time()

    def cb_tool_arm_state(self, msg):
        self.robot_states["tool_arm"] = msg.data
        self._robot_state_time = time.time()

    def cb_hold_status(self, msg):
        if bool(msg.data):
            # Reflect HOLDING in manip_arm state if it's currently IDLE/MOVING
            if self.robot_states.get("manip_arm", "IDLE") in ("IDLE", "MOVING"):
                self.robot_states["manip_arm"] = "HOLDING"
        else:
            if self.robot_states.get("manip_arm", "IDLE") == "HOLDING":
                self.robot_states["manip_arm"] = "IDLE"
        self._robot_state_time = time.time()

    def cb_exotica_ready(self, msg):
        self._exotica_ready = bool(msg.data)

    def cb_wrench(self, msg):
        if self.wrench_offset is None:
            self.wrench_offset = {
                "fx": msg.wrench.force.x,
                "fy": msg.wrench.force.y,
                "fz": msg.wrench.force.z,
                "tx": msg.wrench.torque.x,
                "ty": msg.wrench.torque.y,
                "tz": msg.wrench.torque.z,
            }
        self.latest_zeroed_wrench = {
            "force": {
                "x": msg.wrench.force.x - self.wrench_offset["fx"],
                "y": msg.wrench.force.y - self.wrench_offset["fy"],
                "z": msg.wrench.force.z - self.wrench_offset["fz"],
            },
            "torque": {
                "x": msg.wrench.torque.x - self.wrench_offset["tx"],
                "y": msg.wrench.torque.y - self.wrench_offset["ty"],
                "z": msg.wrench.torque.z - self.wrench_offset["tz"],
            },
        }

    def publish_agent_state(self):
        packet = {
            "timestamp": self.get_clock().now().nanoseconds,
            "global_view": {"objects": self.global_state.get("objects", [])},
            "local_view": {
                "screws": self.local_state.get("screws", []),
                "screw_heads": self.local_state.get("screw_heads", []),
                "tool_tips": self.local_state.get("tool_tips", []),
                "holes": self.local_state.get("holes", []),
                "crosshair": self.local_state.get("crosshair", []),
            },
            "assembly_state": self.assembly_state,
            "force_torque": self.latest_zeroed_wrench if self.latest_zeroed_wrench else {},
        }
        self.agent_state_pub.publish(String(data=json.dumps(packet)))

    def _record_stage_time(self, elapsed_s):
        stat = self._perf_stats["debug_ms"]
        stat[0] += elapsed_s * 1000.0
        stat[1] += 1

    def _maybe_log_perf(self):
        if PERF_LOG_INTERVAL_SEC <= 0:
            return
        now = time.time()
        if now - self._perf_time < PERF_LOG_INTERVAL_SEC:
            return
        total_ms, count = self._perf_stats["debug_ms"]
        avg_ms = (total_ms / count) if count else 0.0
        self.get_logger().info(
            f"Dashboard perf: dbg={self._fps_display:.1f}fps | global={self._global_rx_fps:.1f}fps | "
            f"local={self._local_rx_fps:.1f}fps | debug={avg_ms:.1f}ms"
        )
        self._perf_stats["debug_ms"] = [0.0, 0]
        self._perf_time = now

    def update_system_health(self):
        now = time.time()
        node_names = {name for name, _namespace in self.get_node_names_and_namespaces()}
        moveit_ready = "move_group" in node_names
        global_state_ok = (now - self._global_state_time) < 5.0
        local_state_ok = (now - self._local_state_time) < 5.0
        global_image_ok = self.frame_global is not None and (now - self._global_rx_time) < 2.5
        local_image_ok = self.frame_local is not None and (now - self._local_rx_time) < 2.5
        self._system_health = {
            "MoveIt": {
                "ok": moveit_ready,
                "detail": "move_group online" if moveit_ready else "waiting for move_group",
            },
            "EXOTica": {
                "ok": self._exotica_ready,
                "detail": "IK server ready" if self._exotica_ready else "waiting for /exotica/ready",
            },
            "Global Vision": {
                "ok": global_state_ok and global_image_ok,
                "detail": "state ok | frames live" if global_state_ok and global_image_ok else "waiting for global stream",
            },
            "Local Vision": {
                "ok": local_state_ok and local_image_ok,
                "detail": "state ok | frames live" if local_state_ok and local_image_ok else "waiting for local stream",
            },
        }

    @staticmethod
    def _status_color(ok):
        return (0, 210, 80) if ok else (0, 165, 255)

    def draw_system_health_panel(self, height):
        width = 300
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        panel[:] = (14, 17, 23)

        def draw_text(text, x, y, size=0.8, color=(245, 245, 245), thickness=2):
            cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness, cv2.LINE_AA)

        all_ok = bool(self._system_health) and all(item["ok"] for item in self._system_health.values())
        header_color = (0, 210, 80) if all_ok else (0, 165, 255)
        cv2.rectangle(panel, (16, 16), (width - 16, 82), (28, 34, 45), -1)
        cv2.rectangle(panel, (16, 16), (width - 16, 82), (58, 68, 84), 1)
        draw_text("STACK", 32, 46, 0.82, (235, 241, 248), 2)
        draw_text("READY" if all_ok else "LOADING", 32, 72, 0.62, header_color, 2)

        rows = [
            ("MoveIt", self._system_health.get("MoveIt", {"ok": False, "detail": "waiting"})),
            ("EXOTica", self._system_health.get("EXOTica", {"ok": False, "detail": "waiting"})),
            ("Global Vision", self._system_health.get("Global Vision", {"ok": False, "detail": "waiting"})),
            ("Local Vision", self._system_health.get("Local Vision", {"ok": False, "detail": "waiting"})),
        ]
        y = 125
        for label, item in rows:
            ok = bool(item["ok"])
            color = self._status_color(ok)
            cv2.rectangle(panel, (16, y - 30), (width - 16, y + 34), (26, 31, 42), -1)
            cv2.circle(panel, (36, y), 8, color, -1)
            draw_text(label, 54, y - 4, 0.58, (245, 248, 252), 2)
            draw_text(str(item["detail"])[:28], 54, y + 22, 0.42, (172, 184, 198), 1)
            y += 76

        y = max(y + 8, height - 118)
        cv2.line(panel, (24, y - 18), (width - 24, y - 18), (58, 68, 84), 1)
        draw_text("RUNTIME", 24, y + 6, 0.56, (215, 224, 232), 2)
        draw_text(f"debug {self._fps_display:.1f} fps", 24, y + 36, 0.46, (180, 192, 205), 1)
        draw_text(f"global objects {len(self.global_state.get('objects', []))}", 24, y + 60, 0.46, (180, 192, 205), 1)
        draw_text(f"local targets {sum(len(self.local_state.get(k, [])) for k in ('screws', 'screw_heads', 'tool_tips', 'holes'))}", 24, y + 84, 0.46, (180, 192, 205), 1)
        return panel

    def draw_wide_dashboard(self, width):
        objects = self.global_state.get("objects", [])
        bin_locations = self.global_state.get("bin_locations", {})
        status = self.assembly_state
        wrench_data = self.latest_zeroed_wrench
        panel = np.zeros((DASHBOARD_HEIGHT, width, 3), dtype=np.uint8)
        panel[:] = (10, 13, 18)

        def draw_text(img, text, x, y, size=0.58, color=(232, 238, 246), thickness=1):
            cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness, cv2.LINE_AA)

        def card(x1, y1, x2, y2, title):
            cv2.rectangle(panel, (x1, y1), (x2, y2), (23, 28, 38), -1)
            cv2.rectangle(panel, (x1, y1), (x2, y2), (54, 64, 78), 1)
            draw_text(panel, title, x1 + 14, y1 + 28, 0.55, (174, 186, 202), 1)

        cv2.rectangle(panel, (0, 0), (width, 46), (18, 23, 32), -1)
        draw_text(panel, "DISASSEMBLY DASHBOARD", 20, 30, 0.72, (245, 248, 252), 2)
        global_fps_color = (0, 255, 0) if self._global_rx_fps >= 15 else (0, 255, 255) if self._global_rx_fps >= 5 else (0, 0, 255)
        local_fps_color = (0, 255, 0) if self._local_rx_fps >= 15 else (0, 255, 255) if self._local_rx_fps >= 5 else (0, 0, 255)
        draw_text(panel, f"GLOBAL {self._global_rx_fps:.1f} FPS", width - 330, 30, 0.55, global_fps_color, 2)
        draw_text(panel, f"LOCAL {self._local_rx_fps:.1f} FPS", width - 160, 30, 0.55, local_fps_color, 2)

        gap = 14
        col_w = max(190, (width - 5 * gap) // 4)
        x1 = gap
        x2 = x1 + col_w
        x3 = x2 + gap
        x4 = x3 + col_w
        x5 = x4 + gap
        x6 = x5 + col_w
        x7 = x6 + gap
        x8 = width - gap

        card(x1, 60, x2, DASHBOARD_HEIGHT - 14, "DETECTED PARTS")
        y = 104
        raw_count = self.global_state.get("raw_detection_count")
        if objects:
            for obj in sorted(objects, key=lambda x: x.get("id", 999))[:6]:
                label = str(obj.get("label", "Unknown"))
                obj_id = obj.get("id", "?")
                xyz = obj.get("xyz")
                display = f"#{obj_id} {label[:14]}"
                display += f" Z:{xyz[2]:.3f}m" if xyz else " No Depth"
                draw_text(panel, display, x1 + 16, y, 0.52, (80, 235, 145), 1)
                y += 24
        else:
            detail = "No model detections" if not raw_count else f"{raw_count} raw, 0 published"
            draw_text(panel, detail, x1 + 16, y, 0.55, (135, 145, 158), 1)

        card(x3, 60, x4, DASHBOARD_HEIGHT - 14, "VISION STATE")
        state = status.get("state", "unknown").upper()
        box_color = (64, 72, 84)
        if state == "UNSCREWED":
            box_color = (0, 200, 0)
        elif state == "SCREWED":
            box_color = (0, 140, 255)
        elif "MISALIGN" in state:
            box_color = (0, 0, 255)
        cv2.rectangle(panel, (x3 + 16, 92), (x4 - 16, 144), box_color, -1)
        draw_text(panel, state[:18], x3 + 28, 126, 0.68, (255, 255, 255), 2)
        draw_text(panel, f"Confidence {status.get('confidence', 0.0):.2f}", x3 + 16, 174, 0.52, (190, 202, 216), 1)
        draw_text(panel, f"Tool arm {self.robot_states.get('tool_arm', 'IDLE')}", x3 + 16, 202, 0.52, (190, 202, 216), 1)
        draw_text(panel, f"Manip arm {self.robot_states.get('manip_arm', 'IDLE')}", x3 + 16, 228, 0.52, (190, 202, 216), 1)

        card(x5, 60, x6, DASHBOARD_HEIGHT - 14, "FORCE/TORQUE")
        if wrench_data:
            fx = wrench_data["force"]["x"]
            fy = wrench_data["force"]["y"]
            fz = wrench_data["force"]["z"]
            tx = wrench_data["torque"]["x"]
            ty = wrench_data["torque"]["y"]
            tz = wrench_data["torque"]["z"]
            readings = [
                f"Fx {fx:>6.2f} N",
                f"Fy {fy:>6.2f} N",
                f"Fz {fz:>6.2f} N",
                f"Tx {tx:>6.3f} Nm",
                f"Ty {ty:>6.3f} Nm",
                f"Tz {tz:>6.3f} Nm",
            ]
            for idx, line in enumerate(readings):
                color = (80, 235, 145) if idx < 3 else (190, 202, 216)
                draw_text(panel, line, x5 + 16, 104 + idx * 22, 0.50, color, 1)
        else:
            draw_text(panel, "Waiting for FT300", x5 + 16, 104, 0.55, (135, 145, 158), 1)

        card(x7, 60, x8, DASHBOARD_HEIGHT - 14, "DROP BIN")
        bin1_xyz = bin_locations.get("bin_1", {}).get("xyz")
        if bin1_xyz:
            draw_text(panel, f"BIN 1 Z {bin1_xyz[2]:.3f} m", x7 + 16, 108, 0.58, (80, 235, 145), 1)
            draw_text(panel, f"X {bin1_xyz[0]:.3f}", x7 + 16, 144, 0.50, (190, 202, 216), 1)
            draw_text(panel, f"Y {bin1_xyz[1]:.3f}", x7 + 16, 170, 0.50, (190, 202, 216), 1)
        else:
            draw_text(panel, "BIN 1 waiting", x7 + 16, 108, 0.55, (135, 145, 158), 1)
        return panel

    @staticmethod
    def _scale_point(point, scale_x, scale_y):
        return [int(round(point[0] * scale_x)), int(round(point[1] * scale_y))]

    @staticmethod
    def _scale_box(box, scale_x, scale_y):
        return [
            int(round(box[0] * scale_x)),
            int(round(box[1] * scale_y)),
            int(round(box[2] * scale_x)),
            int(round(box[3] * scale_y)),
        ]

    def _global_scale_factors(self, frame):
        source_size = self.global_state.get("image_size") or []
        if len(source_size) != 2 or source_size[0] <= 0 or source_size[1] <= 0:
            return 1.0, 1.0
        src_w, src_h = source_size
        dst_h, dst_w = frame.shape[:2]
        return dst_w / float(src_w), dst_h / float(src_h)

    def draw_global_overlay(self, frame):
        vis = frame.copy()
        scale_x, scale_y = self._global_scale_factors(vis)
        for obj in self.global_state.get("objects", []):
            box = obj.get("box")
            if not box:
                continue
            box = self._scale_box(box, scale_x, scale_y)
            cv2.rectangle(vis, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
            text = f"ID:{obj.get('id', '?')} {str(obj.get('label', 'obj'))}"
            xyz = obj.get("xyz")
            if xyz:
                text += f" Z:{xyz[2]:.2f}m"
            cv2.putText(vis, text, (int(box[0]), int(box[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return vis

    def draw_local_overlay(self, frame):
        vis = frame.copy()
        source_size = self.local_state.get("image_size") or []
        if len(source_size) == 2 and source_size[0] > 0 and source_size[1] > 0:
            scale_x = vis.shape[1] / float(source_size[0])
            scale_y = vis.shape[0] / float(source_size[1])
        else:
            scale_x = 1.0
            scale_y = 1.0

        def _item_point(item, box):
            point = item.get("centroid") or item.get("center") or item.get("contact_point")
            if point and len(point) == 2:
                return int(round(float(point[0]) * scale_x)), int(round(float(point[1]) * scale_y))
            return int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)

        def _label_text(item, label, cx, cy):
            conf = item.get("confidence", item.get("conf"))
            if conf is None:
                return f"{label} ({cx},{cy})"
            return f"{label} {float(conf):.2f} ({cx},{cy})"

        def _draw_label(text, x, y, color):
            text_y = max(16, y - 7)
            cv2.putText(vis, text, (x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(vis, text, (x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        for screw in self.local_state.get("screws", []):
            box = screw.get("box")
            if not box:
                continue
            box = self._scale_box(box, scale_x, scale_y)
            cx, cy = _item_point(screw, box)
            cv2.rectangle(vis, (box[0], box[1]), (box[2], box[3]), (0, 200, 255), 2)
            cv2.circle(vis, (cx, cy), 4, (0, 200, 255), -1)
            _draw_label(_label_text(screw, "screw", cx, cy), box[0], box[1], (0, 200, 255))
        for screw in self.local_state.get("screw_heads", []):
            box = screw.get("box")
            if not box:
                continue
            box = self._scale_box(box, scale_x, scale_y)
            cx, cy = _item_point(screw, box)
            cv2.rectangle(vis, (box[0], box[1]), (box[2], box[3]), (255, 255, 0), 2)
            cv2.circle(vis, (cx, cy), 5, (255, 255, 0), -1)
            _draw_label(_label_text(screw, "screw_head", cx, cy), box[0], box[1], (255, 255, 0))
        for tool in self.local_state.get("tool_tips", []):
            box = tool.get("box")
            if not box:
                continue
            box = self._scale_box(box, scale_x, scale_y)
            cx, cy = _item_point(tool, box)
            cv2.rectangle(vis, (box[0], box[1]), (box[2], box[3]), (255, 0, 255), 2)
            cv2.circle(vis, (cx, cy), 5, (255, 0, 255), -1)
            _draw_label(_label_text(tool, "tool_head", cx, cy), box[0], box[1], (255, 0, 255))
        for hole in self.local_state.get("holes", []):
            box = hole.get("box")
            if not box:
                continue
            box = self._scale_box(box, scale_x, scale_y)
            cx, cy = _item_point(hole, box)
            cv2.rectangle(vis, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
            cv2.circle(vis, (cx, cy), 3, (0, 0, 255), -1)
            _draw_label(_label_text(hole, "hole", cx, cy), box[0], box[1], (0, 0, 255))
        crosshair = self.local_state.get("crosshair")
        crosshair_cfg = self.local_state.get("crosshair_config") or {}
        if crosshair and len(crosshair) == 2:
            cross_x = int(round(crosshair[0] * scale_x))
            cross_y = int(round(crosshair[1] * scale_y))
            arm = int(crosshair_cfg.get("arm_px", 20))
            thick = int(crosshair_cfg.get("thickness", 1))
            cv2.line(vis, (cross_x - arm, cross_y), (cross_x + arm, cross_y), (0, 255, 0), thick)
            cv2.line(vis, (cross_x, cross_y - arm), (cross_x, cross_y + arm), (0, 255, 0), thick)
            cv2.circle(vis, (cross_x, cross_y), 1, (0, 0, 255), -1)
        return vis

    def publish_dashboard(self):
        if self.frame_global is None and self.frame_local is None:
            return
        t0 = time.perf_counter()
        vis_global = self.draw_global_overlay(self.frame_global) if self.frame_global is not None else np.zeros((480, 640, 3), dtype=np.uint8)
        vis_local = self.draw_local_overlay(self.frame_local) if self.frame_local is not None else np.zeros((480, 640, 3), dtype=np.uint8)

        def resize_h(img, target_h):
            h, w = img.shape[:2]
            scale = target_h / h
            return cv2.resize(img, (int(w * scale), target_h))

        viz_g = resize_h(vis_global, 360)
        viz_l = resize_h(vis_local, 360)
        top_row = np.hstack((viz_g, viz_l))
        dashboard = self.draw_wide_dashboard(top_row.shape[1])
        final_frame = np.vstack((top_row, dashboard))

        health_panel = self.draw_system_health_panel(final_frame.shape[0])
        final_frame = np.hstack((health_panel, final_frame))

        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.format = "jpeg"
        ok, encoded = cv2.imencode(".jpg", final_frame, [int(cv2.IMWRITE_JPEG_QUALITY), DEBUG_JPEG_QUALITY])
        if ok:
            compressed_msg.data = encoded.tobytes()
            self.debug_pub_compressed.publish(compressed_msg)

        if PUBLISH_RAW_DEBUG:
            raw_msg = self.bridge.cv2_to_imgmsg(final_frame, "bgr8")
            raw_msg.header.stamp = self.get_clock().now().to_msg()
            raw_msg.header.frame_id = "vision_debug"
            self.debug_pub_raw.publish(raw_msg)

        self._record_stage_time(time.perf_counter() - t0)
        self._fps_count += 1
        now = time.time()
        elapsed = now - self._fps_time
        if elapsed >= 1.0:
            self._fps_display = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_time = now
        self._maybe_log_perf()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
