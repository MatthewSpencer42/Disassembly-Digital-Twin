#!/usr/bin/env python3
import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from vision_agent.common import (
    DASHBOARD_HEIGHT,
    DEBUG_JPEG_QUALITY,
    DEBUG_PUBLISH_RATE_HZ,
    LOCAL_CROSSHAIR_OFFSET_X,
    LOCAL_CROSSHAIR_OFFSET_Y,
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
        self.local_state = {"screw_heads": [], "tool_tips": [], "holes": [], "crosshair": []}
        self.assembly_state = {"state": "unknown", "confidence": 0.0}
        self.robot_states = {"tool_arm": "OFFLINE", "manip_arm": "OFFLINE"}
        self.latest_zeroed_wrench = None
        self.wrench_offset = None
        self._fps_time = time.time()
        self._fps_count = 0
        self._fps_display = 0.0
        self._perf_time = time.time()
        self._perf_stats = {"debug_ms": [0.0, 0]}

        self.create_subscription(
            CompressedImage,
            "/camera/camera/color/image_raw/compressed",
            self.cb_global_image,
            10,
        )
        self.create_subscription(
            CompressedImage,
            "/tool_cam/image_raw/compressed",
            self.cb_local_image,
            10,
        )
        self.create_subscription(String, "/vision/global_state", self.cb_global_state, 10)
        self.create_subscription(String, "/vision/local_state", self.cb_local_state, 10)
        self.create_subscription(String, "/vision/assembly_state", self.cb_assembly_state, 10)
        self.create_subscription(String, "/robot_states", self.cb_robot_states, 10)
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

    def cb_global_image(self, msg):
        try:
            self.frame_global = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def cb_local_image(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            if img.shape[1] > 640:
                img = cv2.resize(img, (640, 480))
            self.frame_local = img
        except Exception:
            pass

    def cb_global_state(self, msg):
        try:
            self.global_state = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Global state parse error: {e}")

    def cb_local_state(self, msg):
        try:
            self.local_state = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Local state parse error: {e}")

    def cb_assembly_state(self, msg):
        try:
            self.assembly_state = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Assembly state parse error: {e}")

    def cb_robot_states(self, msg):
        try:
            self.robot_states = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Robot state parse error: {e}")

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
        now = time.time()
        if now - self._perf_time < PERF_LOG_INTERVAL_SEC:
            return
        total_ms, count = self._perf_stats["debug_ms"]
        avg_ms = (total_ms / count) if count else 0.0
        self.get_logger().info(f"Dashboard perf: FPS={self._fps_display:.1f} | debug={avg_ms:.1f}ms")
        self._perf_stats["debug_ms"] = [0.0, 0]
        self._perf_time = now

    def draw_wide_dashboard(self, width):
        objects = self.global_state.get("objects", [])
        bin_locations = self.global_state.get("bin_locations", {})
        status = self.assembly_state
        wrench_data = self.latest_zeroed_wrench
        panel = np.zeros((DASHBOARD_HEIGHT, width, 3), dtype=np.uint8)

        def draw_text(img, text, x, y, size=0.8, color=(255, 255, 255), thickness=2):
            cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)

        cv2.rectangle(panel, (0, 0), (width, 45), (40, 40, 40), -1)
        draw_text(panel, "SYSTEM DASHBOARD", 20, 35, 1.0, (0, 255, 255), 2)
        fps_color = (0, 255, 0) if self._fps_display >= 15 else (0, 255, 255) if self._fps_display >= 5 else (0, 0, 255)
        draw_text(panel, f"FPS: {self._fps_display:.1f}", width - 200, 35, 0.9, fps_color, 2)

        col1_x = 20
        draw_text(panel, "DETECTED PARTS", col1_x, 80, 0.75, (200, 200, 200), 2)
        y = 120
        if objects:
            for obj in sorted(objects, key=lambda x: x.get("id", 999))[:6]:
                label = obj.get("label", "Unknown")
                obj_id = obj.get("id", "?")
                xyz = obj.get("xyz")
                display = f"#{obj_id}: {label[:10]}"
                display += f" Z:{xyz[2]:.3f}m" if xyz else " No Depth"
                draw_text(panel, f"> {display}", col1_x, y, 0.8, (0, 255, 0), 2)
                y += 35
        else:
            draw_text(panel, "No parts detected", col1_x, y, 0.85, (100, 100, 100), 2)

        col2_x = width // 2 - 120
        draw_text(panel, "VISION AI STATE", col2_x, 80, 0.75, (200, 200, 200), 2)
        state = status.get("state", "unknown").upper()
        box_color = (50, 50, 50)
        if state == "UNSCREWED":
            box_color = (0, 200, 0)
        elif state == "SCREWED":
            box_color = (0, 140, 255)
        elif "MISALIGN" in state:
            box_color = (0, 0, 255)
        cv2.rectangle(panel, (col2_x, 100), (col2_x + 230, 160), box_color, -1)
        draw_text(panel, state, col2_x + 12, 140, 0.9, (255, 255, 255), 2)
        draw_text(panel, f"Conf: {status.get('confidence', 0.0):.2f}", col2_x, 200, 0.8, (180, 180, 180), 2)
        draw_text(panel, "ROBOT STATES", col2_x, 250, 0.75, (200, 200, 200), 2)
        draw_text(panel, f"Tool Arm: {self.robot_states.get('tool_arm', 'OFFLINE')}", col2_x, 290, 0.8, (180, 180, 180), 2)
        draw_text(panel, f"Manip Arm: {self.robot_states.get('manip_arm', 'OFFLINE')}", col2_x, 325, 0.8, (180, 180, 180), 2)

        y = 370
        if wrench_data:
            fx = wrench_data["force"]["x"]
            fy = wrench_data["force"]["y"]
            fz = wrench_data["force"]["z"]
            tx = wrench_data["torque"]["x"]
            ty = wrench_data["torque"]["y"]
            tz = wrench_data["torque"]["z"]
            draw_text(panel, "SENSORS (Zeroed)", col2_x, y, 0.75, (200, 200, 200), 2)
            y += 35
            draw_text(panel, f"Force X:  {fx:>7.2f} N", col2_x + 10, y, 0.8, (0, 255, 0), 2)
            draw_text(panel, f"Force Y:  {fy:>7.2f} N", col2_x + 10, y + 30, 0.8, (0, 255, 0), 2)
            draw_text(panel, f"Force Z:  {fz:>7.2f} N", col2_x + 10, y + 60, 0.8, (0, 255, 0), 2)
            draw_text(panel, f"Torque X: {tx:>7.3f} Nm", col2_x + 10, y + 100, 0.8, (180, 180, 180), 2)
            draw_text(panel, f"Torque Y: {ty:>7.3f} Nm", col2_x + 10, y + 130, 0.8, (180, 180, 180), 2)
            draw_text(panel, f"Torque Z: {tz:>7.3f} Nm", col2_x + 10, y + 160, 0.8, (180, 180, 180), 2)

        col3_x = width - 350
        draw_text(panel, "LOCATIONS (Cam Frame)", col3_x, 80, 0.75, (200, 200, 200), 2)
        y = 120
        if "workspace" in bin_locations and bin_locations["workspace"].get("xyz"):
            draw_text(panel, f"WORK: Z:{bin_locations['workspace']['xyz'][2]:.3f}m", col3_x, y, 0.85, (0, 255, 255), 2)
            y += 40
        for i in range(1, 4):
            key = f"bin_{i}"
            xyz = bin_locations.get(key, {}).get("xyz")
            if xyz:
                draw_text(panel, f"BIN {i}: Z:{xyz[2]:.3f}m", col3_x, y, 0.85, (0, 255, 0), 2)
            else:
                draw_text(panel, f"BIN {i}: ...", col3_x, y, 0.85, (0, 255, 255), 2)
            y += 40
        return panel

    def draw_global_overlay(self, frame):
        vis = frame.copy()
        for bin_data in self.global_state.get("bin_locations", {}).values():
            polygon = bin_data.get("polygon")
            if polygon:
                poly = np.array(polygon, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(vis, [poly], True, (0, 255, 255), 2)
        for obj in self.global_state.get("objects", []):
            box = obj.get("box")
            if not box:
                continue
            cv2.rectangle(vis, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
            text = f"ID:{obj.get('id', '?')} {obj.get('label', 'obj')}"
            xyz = obj.get("xyz")
            if xyz:
                text += f" Z:{xyz[2]:.2f}m"
            cv2.putText(vis, text, (int(box[0]), int(box[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return vis

    def draw_local_overlay(self, frame):
        vis = frame.copy()
        for screw in self.local_state.get("screw_heads", []):
            box = screw.get("box")
            if not box:
                continue
            cx = int((box[0] + box[2]) / 2)
            cy = int((box[1] + box[3]) / 2)
            cv2.rectangle(vis, (box[0], box[1]), (box[2], box[3]), (255, 255, 0), 2)
            cv2.circle(vis, (cx, cy), 5, (255, 255, 0), -1)
            cv2.putText(vis, "Screw", (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        for tool in self.local_state.get("tool_tips", []):
            box = tool.get("box")
            if not box:
                continue
            cx = int((box[0] + box[2]) / 2)
            cy = int((box[1] + box[3]) / 2)
            cv2.circle(vis, (cx, cy), 5, (255, 0, 255), -1)
            cv2.putText(vis, "Tool", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        for hole in self.local_state.get("holes", []):
            box = hole.get("box")
            if not box:
                continue
            cx = int((box[0] + box[2]) / 2)
            cy = int((box[1] + box[3]) / 2)
            cv2.rectangle(vis, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
            cv2.circle(vis, (cx, cy), 3, (0, 0, 255), -1)
            cv2.putText(vis, "Hole", (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        crosshair = self.local_state.get("crosshair")
        if crosshair and len(crosshair) == 2:
            cross_x, cross_y = int(crosshair[0]), int(crosshair[1])
        else:
            h_loc, w_loc = vis.shape[:2]
            cross_x = (w_loc // 2) + LOCAL_CROSSHAIR_OFFSET_X
            cross_y = (h_loc // 2) + LOCAL_CROSSHAIR_OFFSET_Y
        cv2.line(vis, (cross_x - 20, cross_y), (cross_x + 20, cross_y), (0, 255, 0), 2)
        cv2.line(vis, (cross_x, cross_y - 20), (cross_x, cross_y + 20), (0, 255, 0), 2)
        cv2.circle(vis, (cross_x, cross_y), 2, (0, 0, 255), -1)
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

        viz_g = resize_h(vis_global, 480)
        viz_l = resize_h(vis_local, 480)
        top_row = np.hstack((viz_g, viz_l))
        dashboard = self.draw_wide_dashboard(top_row.shape[1])
        final_frame = np.vstack((top_row, dashboard))
        cv2.putText(final_frame, "GLOBAL (RGB+D)", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(final_frame, "TOOL CAMERA", (viz_g.shape[1] + 20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
