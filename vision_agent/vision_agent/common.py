#!/usr/bin/env python3
import math
import os
import sys
from collections import deque

from vision_agent.runtime_env import setup_python_env

WS_ROOT = setup_python_env(__file__)

import cv2
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

GLOBAL_COLOR_RAW_TOPIC = "/camera/cropped/color/image_raw"
GLOBAL_COLOR_INFERENCE_TOPIC = "/camera/cropped/color/image_inference"
GLOBAL_COLOR_DISPLAY_TOPIC = "/camera/cropped/color/image_display"
GLOBAL_COLOR_TOPIC = GLOBAL_COLOR_INFERENCE_TOPIC
GLOBAL_DEPTH_TOPIC = "/camera/cropped/depth/image_raw"
GLOBAL_CAMERA_INFO_TOPIC = "/camera/cropped/depth/camera_info"
GLOBAL_COLOR_CAMERA_INFO_TOPIC = "/camera/cropped/color/camera_info"
LOCAL_COLOR_TOPIC = "/tool_cam/image_raw/compressed"
GLOBAL_RELIABLE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

DASHBOARD_HEIGHT = 260
PROCESSING_RATE_HZ = 30.0
GLOBAL_SCOUT_EVERY_N_FRAMES = 12
LOCAL_INFERENCE_EVERY_N_FRAMES = 1
REFEREE_EVERY_N_FRAMES = 3
DEBUG_PUBLISH_RATE_HZ = 8.0
DEBUG_JPEG_QUALITY = 75
PERF_LOG_INTERVAL_SEC = 0.0
PUBLISH_RAW_DEBUG = False
# No detection → no crosshair drawn at all.

# Small tool head: detected bounding-box area < LOCAL_TOOL_HEAD_AREA_THRESHOLD_PX2
LOCAL_CROSSHAIR_SMALL_OFFSET_X = -3
LOCAL_CROSSHAIR_SMALL_OFFSET_Y = -7
LOCAL_CROSSHAIR_SMALL_ARM_PX = 20

# Large tool head: detected bounding-box area >= LOCAL_TOOL_HEAD_AREA_THRESHOLD_PX2
LOCAL_CROSSHAIR_LARGE_OFFSET_X = -3
LOCAL_CROSSHAIR_LARGE_OFFSET_Y = 10
LOCAL_CROSSHAIR_LARGE_ARM_PX = 40

# Line thickness in pixels for both crosshair variants.
LOCAL_CROSSHAIR_LINE_THICKNESS = 2

# Pixel² area boundary that separates the two tool head sizes.
LOCAL_TOOL_HEAD_AREA_THRESHOLD_PX2 = 45000

def resolve_checkpoint_path(*parts):
    base_path = os.path.join(WS_ROOT, *parts)
    candidates = (
        base_path,
        os.path.splitext(base_path)[0] + ".pth",
        os.path.splitext(base_path)[0] + ".pt",
    )
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return base_path


PATH_SCOUT = resolve_checkpoint_path(
    "vision_training",
    "Project 1 (Segmentation)",
    "rfdetr",
    "global_model",
    "checkpoint_best_ema.pt",
)
PATH_SNIPER = resolve_checkpoint_path(
    "vision_training",
    "Project 2 (Tool-Screw)",
    "rfdetr",
    "local_model",
    "checkpoint_best_ema.pt",
)
PATH_REFEREE = os.path.join(
    WS_ROOT,
    "vision_training",
    "Project 3 (Classification)",
    "yolo11",
    "state_model",
    "best.pt",
)

SHAPE_CONFIG = {
    0: {"TL": (-10, -175), "TR": (-355, -175), "BR": (-380, 15), "BL": (15, 15)},
    1: {"TL": (-115, -15), "TR": (15, -15), "BR": (24, 55), "BL": (-112, 55)},
    2: {"TL": (-14, -15), "TR": (85, -15), "BR": (85, 25), "BL": (-17, 25)},
    3: {"TL": (-17, -12), "TR": (20, -12), "BR": (0, 160), "BL": (-45, 160)},
}


def calculate_orientation_pca(pts):
    if pts is None or len(pts) < 3:
        return 0.0, (0, 0), (0, 0)
    rect = cv2.minAreaRect(pts)
    width, height = rect[1]
    if max(width, height) == 0:
        return 0.0, (0, 0), (0, 0)
    if min(width, height) / max(width, height) > 0.85:
        return None, None, None

    pts_float = pts.reshape(-1, 2).astype("float64")
    mean, eigenvectors, _ = cv2.PCACompute2(pts_float, mean=None)
    center = (int(mean[0, 0]), int(mean[0, 1]))
    angle_rad = math.atan2(eigenvectors[0, 1], eigenvectors[0, 0])
    angle_deg = math.degrees(angle_rad)
    end_point = (
        int(center[0] + eigenvectors[0, 0] * 50),
        int(center[1] + eigenvectors[0, 1] * 50),
    )
    return float(angle_deg), center, end_point


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

        input_centroids = []
        for start_x, start_y, end_x, end_y in rects:
            input_centroids.append((int((start_x + end_x) / 2.0), int((start_y + end_y) / 2.0)))

        used_anchors = set()
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
