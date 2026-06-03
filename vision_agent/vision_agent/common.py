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
LOCAL_INFERENCE_EVERY_N_FRAMES = 3   # run local YOLO every 3rd frame (~10 Hz at 30 Hz loop)
REFEREE_EVERY_N_FRAMES = 3
DEBUG_PUBLISH_RATE_HZ = 8.0
DEBUG_JPEG_QUALITY = 75
PERF_LOG_INTERVAL_SEC = 0.0
PUBLISH_RAW_DEBUG = False
# No detection → no crosshair drawn at all.

# Small tool head: detected bounding-box area < LOCAL_TOOL_HEAD_AREA_THRESHOLD_PX2
LOCAL_CROSSHAIR_SMALL_OFFSET_X = -3
LOCAL_CROSSHAIR_SMALL_OFFSET_Y = -10
LOCAL_CROSSHAIR_SMALL_ARM_PX = 20

# Large tool head: detected bounding-box area >= LOCAL_TOOL_HEAD_AREA_THRESHOLD_PX2
LOCAL_CROSSHAIR_LARGE_OFFSET_X = -13
LOCAL_CROSSHAIR_LARGE_OFFSET_Y = 154
LOCAL_CROSSHAIR_LARGE_ARM_PX = LOCAL_CROSSHAIR_SMALL_ARM_PX

# Line thickness in pixels for both crosshair variants.
LOCAL_CROSSHAIR_LINE_THICKNESS = 2

# Pixel² area boundary that separates the two tool head sizes.
LOCAL_TOOL_HEAD_AREA_THRESHOLD_PX2 = 45000

# ── Vision backend selector ────────────────────────────────────────────────
# RF-DETR is the default — more reliable detection for all HDD classes.
# Override at launch time with:  VISION_BACKEND=yolo ros2 launch ...
VISION_BACKEND: str = os.getenv("VISION_BACKEND", "rfdetr").lower()


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


# ── RF-DETR model paths (legacy) ──────────────────────────────────────────
PATH_SCOUT_RFDETR = resolve_checkpoint_path(
    "vision_training",
    "Project 1 (Segmentation)",
    "rfdetr",
    "global_model",
    "checkpoint_best_ema.pt",
)
PATH_SNIPER_RFDETR = resolve_checkpoint_path(
    "vision_training",
    "Project 2 (Tool-Screw)",
    "rfdetr",
    "local_model",
    "checkpoint_best_ema.pt",
)

# ── YOLOv11 model paths ───────────────────────────────────────────────────
PATH_SCOUT_YOLO = os.path.join(
    WS_ROOT,
    "vision_training",
    "Project 1 (Segmentation)",
    "rfdetr",
    "global_model",
    "best.pt",
)
PATH_SNIPER_YOLO = os.path.join(
    WS_ROOT,
    "vision_training",
    "Project 2 (Tool-Screw)",
    "rfdetr",
    "local_model",
    "best.pt",
)

# ── Active paths (resolved by backend) ───────────────────────────────────
PATH_SCOUT  = PATH_SCOUT_YOLO  if VISION_BACKEND == "yolo" else PATH_SCOUT_RFDETR
PATH_SNIPER = PATH_SNIPER_YOLO if VISION_BACKEND == "yolo" else PATH_SNIPER_RFDETR

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


class DetectionStabilizer:
    """Temporal persistence filter for local (tool-cam) YOLO detections.

    A detection is only reported if it overlaps (IoU ≥ iou_thresh) with a
    detection in at least ``min_hits`` of the last ``window`` inference results.
    This suppresses single-frame false positives while keeping true detections.

    Parameters
    ----------
    window     : number of past inference frames to consider (default 5)
    min_hits   : how many of those frames must contain a matching detection (default 3)
    iou_thresh : minimum IoU to consider two boxes as the same object (default 0.35)
    """

    _CATEGORIES = ("screws", "screw_heads", "tool_tips", "holes")

    def __init__(self, window: int = 7, min_hits: int = 4, iou_thresh: float = 0.40):
        self.window = window
        self.min_hits = min_hits
        self.iou_thresh = iou_thresh
        self._history: deque = deque(maxlen=window)

    # ------------------------------------------------------------------
    def update(self, new_dets: dict) -> dict:
        """Push ``new_dets`` into history and return the stable subset.

        Works for any dict of ``{category: [det, ...]}``.  The _CATEGORIES
        constant is the default set used by the local sniper; the global scout
        passes its own per-label keys and they are handled identically.
        """
        self._history.append(new_dets)
        # Not enough history yet — pass everything through unchanged.
        if len(self._history) < self.min_hits:
            return new_dets

        # Collect all category keys seen across history + current frame.
        all_cats = set(new_dets.keys())
        for frame in self._history:
            all_cats.update(frame.keys())

        result = {}
        for cat in all_cats:
            stable = []
            for det in new_dets.get(cat, []):
                box = det.get("box")
                if not box:
                    continue
                hits = sum(
                    1
                    for frame in self._history
                    if any(
                        self._iou(box, pd["box"]) >= self.iou_thresh
                        for pd in frame.get(cat, [])
                        if pd.get("box")
                    )
                )
                if hits >= self.min_hits:
                    stable.append(det)
            result[cat] = stable
        return result

    # ------------------------------------------------------------------
    @staticmethod
    def _iou(a, b) -> float:
        ax1, ay1, ax2, ay2 = float(a[0]), float(a[1]), float(a[2]), float(a[3])
        bx1, by1, bx2, by2 = float(b[0]), float(b[1]), float(b[2]), float(b[3])
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        if inter == 0.0:
            return 0.0
        ua = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        ub = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = ua + ub - inter
        return inter / union if union > 0.0 else 0.0
