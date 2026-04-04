#!/usr/bin/env python3
import sys
import os
import json
import math
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from threading import Lock

# Find the repository root (disassembly_pipeline) robustly
def find_repo_root(current_path, target_name="disassembly_pipeline"):
    curr = os.path.abspath(current_path)
    while curr != os.path.dirname(curr):
        if os.path.basename(curr) == target_name:
            return curr
        if os.path.exists(os.path.join(curr, target_name)):
            return os.path.join(curr, target_name)
        curr = os.path.dirname(curr)
    return None


WS_ROOT = find_repo_root(__file__)
if not WS_ROOT or not os.path.exists(os.path.join(WS_ROOT, "vision_training")):
    WS_ROOT = "/home/adip/workspace/dev_ws/src/disassembly_pipeline"

if os.path.exists(WS_ROOT):
    venv_paths = [
        os.path.join(WS_ROOT, "vision_training", ".venv", "lib", "python3.10", "site-packages"),
        os.path.join(WS_ROOT, "vision_training", "train_vision_model", ".venv", "lib", "python3.10", "site-packages"),
    ]
    for path in venv_paths:
        if os.path.exists(path):
            sys.path.insert(0, path)
            break
else:
    sys.path.insert(0, "/home/adip/workspace/dev_ws/src/disassembly_pipeline/vision_training/.venv/lib/python3.10/site-packages")

os.environ["NO_ALBUMENTATIONS_UPDATE"] = "1"
os.environ["TRANSFORMERS_VERBOSITY"] = "error"
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

DASHBOARD_HEIGHT = 550
PROCESSING_RATE_HZ = 30.0
GLOBAL_SCOUT_EVERY_N_FRAMES = 12
LOCAL_INFERENCE_EVERY_N_FRAMES = 1
REFEREE_EVERY_N_FRAMES = 3
DEBUG_PUBLISH_RATE_HZ = 8.0
DEBUG_JPEG_QUALITY = 75
PERF_LOG_INTERVAL_SEC = 5.0
LOCAL_CROSSHAIR_OFFSET_X = -3
LOCAL_CROSSHAIR_OFFSET_Y = -10

PATH_SCOUT = os.path.join(WS_ROOT, "vision_training", "Project 1 (Segmentation)", "rfdetr", "global_model", "checkpoint_best_ema.pt")
PATH_SNIPER = os.path.join(WS_ROOT, "vision_training", "Project 2 (Tool-Screw)", "rfdetr", "local_model", "checkpoint_best_ema.pt")
PATH_REFEREE = os.path.join(WS_ROOT, "vision_training", "Project 3 (Classification)", "yolo11", "state_model", "best.pt")

SHAPE_CONFIG = {
    0: {"TL": (-10, -175), "TR": (-355, -175), "BR": (-380, 15), "BL": (15, 15)},
    1: {"TL": (-115, -15), "TR": (15, -15), "BR": (24, 55), "BL": (-112, 55)},
    2: {"TL": (-14, -15), "TR": (85, -15), "BR": (85, 25), "BL": (-17, 25)},
    3: {"TL": (-17, -12), "TR": (20, -12), "BR": (0, 160), "BL": (-45, 160)},
}

from vision_agent.agents.referee import RefereeAgent
from vision_agent.agents.scout import ScoutAgent
from vision_agent.agents.sniper import SniperAgent


def calculate_orientation_pca(pts):
    if pts is None or len(pts) < 3:
        return 0.0
    rect = cv2.minAreaRect(pts)
    width, height = rect[1]
    if max(width, height) == 0:
        return 0.0
    if min(width, height) / max(width, height) > 0.85:
        return None
    pts_float = pts.reshape(-1, 2).astype(np.float64)
    _, eigenvectors, _ = cv2.PCACompute2(pts_float, mean=None)
    angle_rad = math.atan2(eigenvectors[0, 1], eigenvectors[0, 0])
    return math.degrees(angle_rad)


class AngleStabilizer:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.histories = {}

    def update(self, obj_id, angle_deg):
        if obj_id not in self.histories:
            self.histories[obj_id] = deque(maxlen=self.window_size)
        rad = math.radians(angle_deg)
        self.histories[obj_id].append((math.cos(rad), math.sin(rad)))
        avg_cos = sum(v[0] for v in self.histories[obj_id]) / len(self.histories[obj_id])
        avg_sin = sum(v[1] for v in self.histories[obj_id]) / len(self.histories[obj_id])
        return math.degrees(math.atan2(avg_sin, avg_cos))


class Point3DStabilizer:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.histories = {}

    def update(self, obj_id, xyz):
        if xyz is None:
            return None
        if obj_id not in self.histories:
            self.histories[obj_id] = deque(maxlen=self.window_size)
        self.histories[obj_id].append(xyz)
        avg_x = sum(p[0] for p in self.histories[obj_id]) / len(self.histories[obj_id])
        avg_y = sum(p[1] for p in self.histories[obj_id]) / len(self.histories[obj_id])
        avg_z = sum(p[2] for p in self.histories[obj_id]) / len(self.histories[obj_id])
        return (round(avg_x, 4), round(avg_y, 4), round(avg_z, 4))


class StaticAnchorTracker:
    def __init__(self, tolerance=60, max_disappeared=3000, deadband=5.0, alpha=0.2):
        self.next_object_id = 0
        self.anchors = {}
        self.tolerance = tolerance
        self.max_disappeared = max_disappeared
        self.deadband = deadband
        self.alpha = alpha

    def update(self, rects, labels):
        assigned_ids = {}
        if len(rects) == 0:
            for obj_id in list(self.anchors.keys()):
                self.anchors[obj_id]["disappeared"] += 1
                if self.anchors[obj_id]["disappeared"] > self.max_disappeared:
                    del self.anchors[obj_id]
            return assigned_ids

        used_anchors = set()
        input_centroids = [(int((r[0] + r[2]) / 2.0), int((r[1] + r[3]) / 2.0)) for r in rects]
        for i, (cx, cy) in enumerate(input_centroids):
            label = labels[i]
            best_id = None
            min_dist = self.tolerance
            for obj_id, anchor in self.anchors.items():
                if obj_id in used_anchors or anchor["label"] != label:
                    continue
                dist = math.hypot(cx - anchor["centroid"][0], cy - anchor["centroid"][1])
                if dist < min_dist:
                    min_dist = dist
                    best_id = obj_id
            if best_id is not None:
                used_anchors.add(best_id)
                old_cx, old_cy = self.anchors[best_id]["centroid"]
                if min_dist < self.deadband:
                    final_cx, final_cy = old_cx, old_cy
                else:
                    final_cx = int(self.alpha * cx + (1 - self.alpha) * old_cx)
                    final_cy = int(self.alpha * cy + (1 - self.alpha) * old_cy)
                assigned_ids[best_id] = (final_cx, final_cy)
                self.anchors[best_id]["centroid"] = (final_cx, final_cy)
                self.anchors[best_id]["disappeared"] = 0
            else:
                new_id = self.next_object_id
                self.next_object_id += 1
                self.anchors[new_id] = {"centroid": (cx, cy), "label": label, "disappeared": 0}
                assigned_ids[new_id] = (cx, cy)
                used_anchors.add(new_id)
        for obj_id in list(self.anchors.keys()):
            if obj_id not in used_anchors:
                self.anchors[obj_id]["disappeared"] += 1
                if self.anchors[obj_id]["disappeared"] > self.max_disappeared:
                    del self.anchors[obj_id]
        return assigned_ids


@dataclass
class SharedState:
    lock: Lock = field(default_factory=Lock)
    frame_global: np.ndarray = None
    frame_depth_meters: np.ndarray = None
    frame_local: np.ndarray = None
    intrinsics: dict = None
    robot_states: dict = field(default_factory=lambda: {"tool_arm": "OFFLINE", "manip_arm": "OFFLINE"})
    wrench_offset: dict = None
    latest_zeroed_wrench: dict = None
    global_state: dict = field(default_factory=lambda: {"objects": [], "bin_locations": {}})
    local_state: dict = field(default_factory=lambda: {"screw_heads": [], "tool_tips": [], "holes": [], "crosshair": []})
    assembly_state: dict = field(default_factory=lambda: {"state": "unknown", "confidence": 0.0})


class SourceNode(Node):
    def __init__(self, shared):
        super().__init__("vision_source_node")
        self.shared = shared
        self.bridge = CvBridge()
        self.create_subscription(CompressedImage, "/camera/camera/color/image_raw/compressed", self.cb_global, 10)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.cb_depth, 10)
        self.create_subscription(CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", self.cb_info, 10)
        self.create_subscription(CompressedImage, "/tool_cam/image_raw/compressed", self.cb_local, 10)
        self.create_subscription(WrenchStamped, "/robotiq_force_torque_sensor_broadcaster/wrench", self.cb_wrench, 10)
        self.create_subscription(String, "/robot_states", self.cb_robot_states, 10)

    def cb_global(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            with self.shared.lock:
                self.shared.frame_global = img
        except Exception:
            pass

    def cb_depth(self, msg):
        try:
            raw_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            with self.shared.lock:
                self.shared.frame_depth_meters = raw_depth.astype(np.float32) / 1000.0
        except Exception:
            pass

    def cb_info(self, msg):
        with self.shared.lock:
            if self.shared.intrinsics is None:
                k = msg.k
                self.shared.intrinsics = {"fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5]}

    def cb_local(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            if img.shape[1] > 640:
                img = cv2.resize(img, (640, 480))
            with self.shared.lock:
                self.shared.frame_local = img
        except Exception:
            pass

    def cb_wrench(self, msg):
        with self.shared.lock:
            if self.shared.wrench_offset is None:
                self.shared.wrench_offset = {
                    "fx": msg.wrench.force.x, "fy": msg.wrench.force.y, "fz": msg.wrench.force.z,
                    "tx": msg.wrench.torque.x, "ty": msg.wrench.torque.y, "tz": msg.wrench.torque.z,
                }
            self.shared.latest_zeroed_wrench = {
                "force": {
                    "x": msg.wrench.force.x - self.shared.wrench_offset["fx"],
                    "y": msg.wrench.force.y - self.shared.wrench_offset["fy"],
                    "z": msg.wrench.force.z - self.shared.wrench_offset["fz"],
                },
                "torque": {
                    "x": msg.wrench.torque.x - self.shared.wrench_offset["tx"],
                    "y": msg.wrench.torque.y - self.shared.wrench_offset["ty"],
                    "z": msg.wrench.torque.z - self.shared.wrench_offset["tz"],
                },
            }

    def cb_robot_states(self, msg):
        try:
            with self.shared.lock:
                self.shared.robot_states = json.loads(msg.data)
        except Exception:
            pass


class GlobalNode(Node):
    def __init__(self, shared):
        super().__init__("vision_global_node")
        self.shared = shared
        self.scout = ScoutAgent(PATH_SCOUT)
        self.tracker = StaticAnchorTracker(tolerance=60, max_disappeared=3000)
        self.angle_stabilizer = AngleStabilizer(window_size=15)
        self.xyz_stabilizer = Point3DStabilizer(window_size=15)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.marker_size_mm = 20.0
        self.smoothing_window = 10
        self.buffers = {}
        self.active_polygons = {}
        self.frame_counter = 0
        self.last_scout_results = []
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.pub = self.create_publisher(String, "/vision/global_state", 10)
        self.bin_pub = self.create_publisher(String, "/vision/bin_coordinates", 10)
        self.timer = self.create_timer(1.0 / PROCESSING_RATE_HZ, self.tick)

    def _run(self, frame):
        return self.scout.scan(frame)

    def get_smoothed_values(self, marker_id, raw_scale, raw_cx, raw_cy):
        if marker_id not in self.buffers:
            self.buffers[marker_id] = {"scale": deque(maxlen=self.smoothing_window), "cx": deque(maxlen=self.smoothing_window), "cy": deque(maxlen=self.smoothing_window)}
        b = self.buffers[marker_id]
        b["scale"].append(raw_scale)
        b["cx"].append(raw_cx)
        b["cy"].append(raw_cy)
        return (sum(b["scale"]) / len(b["scale"]), sum(b["cx"]) / len(b["cx"]), sum(b["cy"]) / len(b["cy"]))

    def get_3d_coordinates(self, cx, cy, segments_pts=None):
        with self.shared.lock:
            depth = self.shared.frame_depth_meters
            intrinsics = self.shared.intrinsics
        if depth is None or intrinsics is None:
            return None
        if segments_pts is not None:
            mask = np.zeros(depth.shape, dtype=np.uint8)
            cv2.fillPoly(mask, [segments_pts], 255)
            valid = depth[mask == 255]
            valid = valid[valid > 0.001]
            if len(valid) == 0:
                return None
            depth_val = float(np.median(valid))
        else:
            h, w = depth.shape
            cx = max(0, min(w - 1, cx))
            cy = max(0, min(h - 1, cy))
            depth_val = float(depth[cy, cx])
            if depth_val < 0.001:
                return None
        z = depth_val
        x = (cx - intrinsics["cx"]) * z / intrinsics["fx"]
        y = (cy - intrinsics["cy"]) * z / intrinsics["fy"]
        return (round(x, 4), round(y, 4), round(z, 4))

    def tick(self):
        with self.shared.lock:
            frame = None if self.shared.frame_global is None else self.shared.frame_global.copy()
        if frame is None:
            return
        if self.future and self.future.done():
            try:
                self.last_scout_results = self.future.result()[0]
            except Exception as e:
                self.get_logger().error(f"Scout Error: {e}")
            self.future = None
        if self.frame_counter % GLOBAL_SCOUT_EVERY_N_FRAMES == 0 and self.future is None:
            self.future = self.pool.submit(self._run, frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        bin_locations = {}
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in SHAPE_CONFIG:
                    continue
                perimeter = cv2.arcLength(corners[i][0], True)
                raw_px_per_mm = (perimeter / 4.0) / self.marker_size_mm
                c = corners[i][0].astype(float)
                raw_cx, raw_cy = np.mean(c[:, 0]), np.mean(c[:, 1])
                px_per_mm, cx, cy = self.get_smoothed_values(int(marker_id), raw_px_per_mm, raw_cx, raw_cy)
                poly_pts = []
                for key in ["TL", "TR", "BR", "BL"]:
                    off_x, off_y = SHAPE_CONFIG[int(marker_id)][key]
                    poly_pts.append([int(cx + off_x * px_per_mm), int(cy + off_y * px_per_mm)])
                poly = np.array(poly_pts, np.int32).reshape((-1, 1, 2))
                self.active_polygons[int(marker_id)] = poly
                xyz = self.get_3d_coordinates(int(cx), int(cy), poly)
                key_name = "workspace" if int(marker_id) == 0 else f"bin_{int(marker_id)}"
                bin_locations[key_name] = {"id": int(marker_id), "px": [int(cx), int(cy)], "xyz": xyz, "polygon": poly.reshape(-1, 2).tolist()}

        workspace_poly = self.active_polygons.get(0)
        detections = list(self.last_scout_results)
        detections.sort(key=lambda x: x.get("box", [0])[0])
        rects, labels, valid = [], [], []
        for obj in detections:
            box = obj.get("box") or obj.get("bbox") or obj.get("xyxy")
            if not box:
                continue
            raw_cx = int((box[0] + box[2]) / 2)
            raw_cy = int((box[1] + box[3]) / 2)
            if workspace_poly is not None and cv2.pointPolygonTest(workspace_poly, (raw_cx, raw_cy), False) < 0:
                continue
            valid.append(obj)
            rects.append(box)
            labels.append(obj.get("label"))
        tracked = self.tracker.update(rects, labels)
        objects = []
        for obj in valid:
            box = obj.get("box") or obj.get("bbox") or obj.get("xyxy")
            raw_cx = int((box[0] + box[2]) / 2)
            raw_cy = int((box[1] + box[3]) / 2)
            obj_id, min_dist = -1, 9999.0
            for tracked_id, tracked_center in tracked.items():
                d = math.hypot(raw_cx - tracked_center[0], raw_cy - tracked_center[1])
                if d < min_dist and d < 50:
                    min_dist, obj_id = d, tracked_id
            cx, cy = tracked.get(obj_id, (raw_cx, raw_cy))
            pts = None
            angle = 0.0
            segments = obj.get("segments") or obj.get("mask")
            if segments:
                pts = np.array(segments, np.int32).reshape((-1, 1, 2))
                raw_angle = calculate_orientation_pca(pts)
                if raw_angle is not None:
                    angle = self.angle_stabilizer.update(obj_id, raw_angle)
            xyz = self.get_3d_coordinates(cx, cy, pts)
            if xyz and obj_id != -1:
                xyz = self.xyz_stabilizer.update(obj_id, xyz)
            objects.append({"id": int(obj_id), "label": obj.get("label"), "confidence": obj.get("confidence"), "box": box, "xyz": xyz, "angle": angle})

        packet = {"timestamp": self.get_clock().now().nanoseconds, "objects": objects, "bin_locations": bin_locations}
        with self.shared.lock:
            self.shared.global_state = packet
        self.pub.publish(String(data=json.dumps(packet)))
        if bin_locations:
            self.bin_pub.publish(String(data=json.dumps(bin_locations)))
        self.frame_counter += 1


class LocalNode(Node):
    def __init__(self, shared):
        super().__init__("vision_local_node")
        self.shared = shared
        self.sniper = SniperAgent(PATH_SNIPER)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.last_data = {"screw_heads": [], "tool_tips": [], "holes": [], "crosshair": []}
        self.frame_counter = 0
        self.pub = self.create_publisher(String, "/vision/local_state", 10)
        self.timer = self.create_timer(1.0 / PROCESSING_RATE_HZ, self.tick)

    def tick(self):
        with self.shared.lock:
            frame = None if self.shared.frame_local is None else self.shared.frame_local.copy()
        if frame is None:
            return
        if self.future and self.future.done():
            try:
                self.last_data = self.future.result()
            except Exception as e:
                self.get_logger().error(f"Sniper Error: {e}")
            self.future = None
        if self.frame_counter % LOCAL_INFERENCE_EVERY_N_FRAMES == 0 and self.future is None:
            self.future = self.pool.submit(self.sniper.target, frame)
        h, w = frame.shape[:2]
        packet = dict(self.last_data)
        packet["crosshair"] = [int((w // 2) + LOCAL_CROSSHAIR_OFFSET_X), int((h // 2) + LOCAL_CROSSHAIR_OFFSET_Y)]
        packet["timestamp"] = self.get_clock().now().nanoseconds
        with self.shared.lock:
            self.shared.local_state = packet
        self.pub.publish(String(data=json.dumps(packet)))
        self.frame_counter += 1


class ClassifierNode(Node):
    def __init__(self, shared):
        super().__init__("vision_classifier_node")
        self.shared = shared
        self.referee = RefereeAgent(PATH_REFEREE)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.last_status = {"state": "unknown", "confidence": 0.0}
        self.frame_counter = 0
        self.pub = self.create_publisher(String, "/vision/assembly_state", 10)
        self.timer = self.create_timer(1.0 / PROCESSING_RATE_HZ, self.tick)

    def tick(self):
        with self.shared.lock:
            frame = None if self.shared.frame_local is None else self.shared.frame_local.copy()
        if frame is None:
            return
        if self.future and self.future.done():
            try:
                self.last_status = self.future.result()
            except Exception as e:
                self.get_logger().error(f"Referee Error: {e}")
            self.future = None
        if self.frame_counter % REFEREE_EVERY_N_FRAMES == 0 and self.future is None:
            self.future = self.pool.submit(self.referee.inspect, frame)
        packet = dict(self.last_status)
        packet["timestamp"] = self.get_clock().now().nanoseconds
        with self.shared.lock:
            self.shared.assembly_state = packet
        self.pub.publish(String(data=json.dumps(packet)))
        self.frame_counter += 1


class DashboardNode(Node):
    def __init__(self, shared):
        super().__init__("vision_dashboard_node")
        self.shared = shared
        self.bridge = CvBridge()
        self._fps_time = time.time()
        self._fps_count = 0
        self._fps_display = 0.0
        self._perf_time = time.time()
        self._debug_time_total = 0.0
        self._debug_count = 0
        self.agent_pub = self.create_publisher(String, "/vision/agent_state", 10)
        self.debug_pub = self.create_publisher(CompressedImage, "/vision/debug_feed/compressed", 10)
        self.state_timer = self.create_timer(1.0 / PROCESSING_RATE_HZ, self.publish_agent_state)
        self.debug_timer = self.create_timer(1.0 / DEBUG_PUBLISH_RATE_HZ, self.publish_dashboard)

    def publish_agent_state(self):
        with self.shared.lock:
            packet = {
                "timestamp": self.get_clock().now().nanoseconds,
                "global_view": {"objects": self.shared.global_state.get("objects", [])},
                "local_view": {
                    "screw_heads": self.shared.local_state.get("screw_heads", []),
                    "tool_tips": self.shared.local_state.get("tool_tips", []),
                    "holes": self.shared.local_state.get("holes", []),
                    "crosshair": self.shared.local_state.get("crosshair", []),
                },
                "assembly_state": self.shared.assembly_state,
                "force_torque": self.shared.latest_zeroed_wrench if self.shared.latest_zeroed_wrench else {},
            }
        self.agent_pub.publish(String(data=json.dumps(packet)))

    def draw_wide_dashboard(self, width, objects, bin_locations, status, wrench_data, robot_states):
        panel = np.zeros((DASHBOARD_HEIGHT, width, 3), dtype=np.uint8)
        def draw_text(img, text, x, y, size=0.8, color=(255,255,255), thickness=2):
            cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)
        cv2.rectangle(panel, (0, 0), (width, 45), (40, 40, 40), -1)
        draw_text(panel, "SYSTEM DASHBOARD", 20, 35, 1.0, (0,255,255), 2)
        fps_color = (0,255,0) if self._fps_display >= 15 else (0,255,255) if self._fps_display >= 5 else (0,0,255)
        draw_text(panel, f"FPS: {self._fps_display:.1f}", width - 200, 35, 0.9, fps_color, 2)
        col1_x = 20
        draw_text(panel, "DETECTED PARTS", col1_x, 80, 0.75, (200,200,200), 2)
        y = 120
        for obj in sorted(objects, key=lambda x: x.get("id", 999))[:6]:
            xyz = obj.get("xyz")
            msg = f"#{obj.get('id','?')}: {obj.get('label','Unknown')[:10]}"
            msg += f" Z:{xyz[2]:.3f}m" if xyz else " No Depth"
            draw_text(panel, f"> {msg}", col1_x, y, 0.8, (0,255,0), 2)
            y += 35
        col2_x = width // 2 - 120
        draw_text(panel, "VISION AI STATE", col2_x, 80, 0.75, (200,200,200), 2)
        state = status.get("state", "unknown").upper()
        box_color = (50,50,50)
        if state == "UNSCREWED": box_color = (0,200,0)
        elif state == "SCREWED": box_color = (0,140,255)
        elif "MISALIGN" in state: box_color = (0,0,255)
        cv2.rectangle(panel, (col2_x, 100), (col2_x + 230, 160), box_color, -1)
        draw_text(panel, state, col2_x + 12, 140, 0.9, (255,255,255), 2)
        draw_text(panel, f"Conf: {status.get('confidence',0.0):.2f}", col2_x, 200, 0.8, (180,180,180), 2)
        draw_text(panel, "ROBOT STATES", col2_x, 250, 0.75, (200,200,200), 2)
        draw_text(panel, f"Tool Arm: {robot_states.get('tool_arm','OFFLINE')}", col2_x, 290, 0.8, (180,180,180), 2)
        draw_text(panel, f"Manip Arm: {robot_states.get('manip_arm','OFFLINE')}", col2_x, 325, 0.8, (180,180,180), 2)
        if wrench_data:
            fx, fy, fz = wrench_data["force"]["x"], wrench_data["force"]["y"], wrench_data["force"]["z"]
            tx, ty, tz = wrench_data["torque"]["x"], wrench_data["torque"]["y"], wrench_data["torque"]["z"]
            draw_text(panel, f"Fx {fx:7.2f} Fy {fy:7.2f} Fz {fz:7.2f}", col2_x, 380, 0.75, (180,180,180), 2)
            draw_text(panel, f"Tx {tx:7.3f} Ty {ty:7.3f} Tz {tz:7.3f}", col2_x, 415, 0.75, (180,180,180), 2)
        col3_x = width - 350
        draw_text(panel, "LOCATIONS (Cam Frame)", col3_x, 80, 0.75, (200,200,200), 2)
        y = 120
        for key in ["workspace", "bin_1", "bin_2", "bin_3"]:
            xyz = bin_locations.get(key, {}).get("xyz")
            label = key.upper()
            if xyz:
                draw_text(panel, f"{label}: Z:{xyz[2]:.3f}m", col3_x, y, 0.85, (0,255,0), 2)
            else:
                draw_text(panel, f"{label}: ...", col3_x, y, 0.85, (0,255,255), 2)
            y += 40
        return panel

    def publish_dashboard(self):
        t0 = time.perf_counter()
        with self.shared.lock:
            frame_global = None if self.shared.frame_global is None else self.shared.frame_global.copy()
            frame_local = None if self.shared.frame_local is None else self.shared.frame_local.copy()
            global_state = json.loads(json.dumps(self.shared.global_state))
            local_state = json.loads(json.dumps(self.shared.local_state))
            assembly_state = dict(self.shared.assembly_state)
            wrench = None if self.shared.latest_zeroed_wrench is None else dict(self.shared.latest_zeroed_wrench)
            robot_states = dict(self.shared.robot_states)
        if frame_global is None and frame_local is None:
            return
        vis_global = frame_global if frame_global is not None else np.zeros((480, 640, 3), dtype=np.uint8)
        for bin_data in global_state.get("bin_locations", {}).values():
            poly = bin_data.get("polygon")
            if poly:
                cv2.polylines(vis_global, [np.array(poly, dtype=np.int32).reshape((-1,1,2))], True, (0,255,255), 2)
        for obj in global_state.get("objects", []):
            box = obj.get("box")
            if not box:
                continue
            cv2.rectangle(vis_global, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0,255,0), 2)
            msg = f"ID:{obj.get('id','?')} {obj.get('label','obj')}"
            xyz = obj.get("xyz")
            if xyz:
                msg += f" Z:{xyz[2]:.2f}m"
            cv2.putText(vis_global, msg, (int(box[0]), int(box[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)
        vis_local = frame_local if frame_local is not None else np.zeros((480, 640, 3), dtype=np.uint8)
        for coll, color, label in [("screw_heads",(255,255,0),"Screw"), ("holes",(0,0,255),"Hole"), ("tool_tips",(255,0,255),"Tool")]:
            for item in local_state.get(coll, []):
                box = item.get("box")
                if not box:
                    continue
                cv2.rectangle(vis_local, (box[0], box[1]), (box[2], box[3]), color, 2)
                cx, cy = int((box[0]+box[2])/2), int((box[1]+box[3])/2)
                cv2.circle(vis_local, (cx, cy), 4, color, -1)
                cv2.putText(vis_local, label, (box[0], box[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cross = local_state.get("crosshair", [])
        if len(cross) == 2:
            cx, cy = int(cross[0]), int(cross[1])
            cv2.line(vis_local, (cx-20, cy), (cx+20, cy), (0,255,0), 2)
            cv2.line(vis_local, (cx, cy-20), (cx, cy+20), (0,255,0), 2)
            cv2.circle(vis_local, (cx, cy), 2, (0,0,255), -1)
        def resize_h(img, target_h):
            h, w = img.shape[:2]
            scale = target_h / h
            return cv2.resize(img, (int(w * scale), target_h))
        top_row = np.hstack((resize_h(vis_global, 480), resize_h(vis_local, 480)))
        dashboard = self.draw_wide_dashboard(top_row.shape[1], global_state.get("objects", []), global_state.get("bin_locations", {}), assembly_state, wrench, robot_states)
        final = np.vstack((top_row, dashboard))
        cv2.putText(final, "GLOBAL (RGB+D)", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv2.putText(final, "TOOL CAMERA", (top_row.shape[1]//2 + 20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        ok, encoded = cv2.imencode(".jpg", final, [int(cv2.IMWRITE_JPEG_QUALITY), DEBUG_JPEG_QUALITY])
        if ok:
            msg.data = encoded.tobytes()
            self.debug_pub.publish(msg)
        self._fps_count += 1
        now = time.time()
        elapsed = now - self._fps_time
        if elapsed >= 1.0:
            self._fps_display = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_time = now
        self._debug_time_total += (time.perf_counter() - t0) * 1000.0
        self._debug_count += 1
        if now - self._perf_time >= PERF_LOG_INTERVAL_SEC:
            avg_debug = self._debug_time_total / self._debug_count if self._debug_count else 0.0
            self.get_logger().info(f"Dashboard perf: FPS={self._fps_display:.1f} | debug={avg_debug:.1f}ms")
            self._debug_time_total = 0.0
            self._debug_count = 0
            self._perf_time = now


def main(args=None):
    rclpy.init(args=args)
    shared = SharedState()
    source = SourceNode(shared)
    global_node = GlobalNode(shared)
    local_node = LocalNode(shared)
    classifier_node = ClassifierNode(shared)
    dashboard_node = DashboardNode(shared)
    executor = MultiThreadedExecutor(num_threads=6)
    for node in [source, global_node, local_node, classifier_node, dashboard_node]:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in [source, global_node, local_node, classifier_node, dashboard_node]:
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
