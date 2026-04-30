#!/usr/bin/env python3
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor

from vision_agent.runtime_env import setup_python_env

setup_python_env(__file__)

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from vision_agent.common import (
    GLOBAL_SCOUT_EVERY_N_FRAMES,
    PATH_SCOUT,
    PERF_LOG_INTERVAL_SEC,
    PROCESSING_RATE_HZ,
    SHAPE_CONFIG,
    GLOBAL_CAMERA_INFO_TOPIC,
    GLOBAL_COLOR_INFERENCE_TOPIC,
    GLOBAL_DEPTH_TOPIC,
    GLOBAL_RELIABLE_QOS,
    AngleStabilizer,
    Point3DStabilizer,
    StaticAnchorTracker,
    calculate_orientation_pca,
)
from vision_agent.agents.scout import ScoutAgent


class GlobalVisionNode(Node):
    def __init__(self):
        super().__init__("vision_global_node")
        self.get_logger().info("Starting global vision node")

        self.scout = ScoutAgent(PATH_SCOUT)
        self.bridge = CvBridge()
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

        self.frame_global = None
        self.frame_depth_meters = None
        self.intrinsics = None
        self.frame_counter = 0
        self.last_scout_results = []
        self.scout_future = None
        self.pool = ThreadPoolExecutor(max_workers=1)

        self._perf_time = time.time()
        self._perf_stats = {"scout_ms": [0.0, 0]}

        self.create_subscription(
            Image,
            GLOBAL_COLOR_INFERENCE_TOPIC,
            self.cb_global,
            GLOBAL_RELIABLE_QOS,
        )
        self.create_subscription(
            Image,
            GLOBAL_DEPTH_TOPIC,
            self.cb_depth,
            GLOBAL_RELIABLE_QOS,
        )
        self.create_subscription(
            CameraInfo,
            GLOBAL_CAMERA_INFO_TOPIC,
            self.cb_info,
            GLOBAL_RELIABLE_QOS,
        )
        self.create_subscription(String, "/vision/reset_tracker", self.cb_reset_request, 10)

        self.state_pub = self.create_publisher(String, "/vision/global_state", 10)
        self.bin_pub = self.create_publisher(String, "/vision/bin_coordinates", 10)
        self.timer = self.create_timer(1.0 / PROCESSING_RATE_HZ, self.processing_loop)

    def cb_reset_request(self, _msg):
        self.get_logger().info("Resetting global tracker state")
        self.tracker.anchors.clear()
        self.tracker.next_object_id = 0
        self.angle_stabilizer.histories.clear()
        self.xyz_stabilizer.histories.clear()
        self.buffers.clear()
        self.active_polygons.clear()

    def cb_global(self, msg):
        try:
            self.frame_global = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def cb_depth(self, msg):
        try:
            raw_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.frame_depth_meters = raw_depth.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f"Depth Error: {e}")

    def cb_info(self, msg):
        if self.intrinsics is None:
            k = msg.k
            self.intrinsics = {"fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5]}
            self.get_logger().info(f"Global intrinsics loaded: fx={k[0]:.1f}, fy={k[4]:.1f}")

    def get_smoothed_values(self, marker_id, raw_scale, raw_cx, raw_cy):
        if marker_id not in self.buffers:
            self.buffers[marker_id] = {
                "scale": [],
                "cx": [],
                "cy": [],
            }
        buf = self.buffers[marker_id]
        for key, value in (("scale", raw_scale), ("cx", raw_cx), ("cy", raw_cy)):
            buf[key].append(value)
            if len(buf[key]) > self.smoothing_window:
                buf[key].pop(0)
        return (
            sum(buf["scale"]) / len(buf["scale"]),
            sum(buf["cx"]) / len(buf["cx"]),
            sum(buf["cy"]) / len(buf["cy"]),
        )

    def get_3d_coordinates(self, cx, cy, segments_pts=None):
        if self.frame_depth_meters is None or self.intrinsics is None:
            return None
        if segments_pts is not None:
            mask = np.zeros(self.frame_depth_meters.shape, dtype=np.uint8)
            cv2.fillPoly(mask, [segments_pts], 255)
            valid_depths = self.frame_depth_meters[mask == 255]
            valid_depths = valid_depths[valid_depths > 0.001]
            if len(valid_depths) == 0:
                return None
            depth_val_m = float(np.median(valid_depths))
        else:
            h, w = self.frame_depth_meters.shape
            cx = max(0, min(w - 1, cx))
            cy = max(0, min(h - 1, cy))
            depth_val_m = float(self.frame_depth_meters[cy, cx])
            if depth_val_m < 0.001:
                return None

        z_m = depth_val_m
        x_m = (cx - self.intrinsics["cx"]) * z_m / self.intrinsics["fx"]
        y_m = (cy - self.intrinsics["cy"]) * z_m / self.intrinsics["fy"]
        return (round(x_m, 4), round(y_m, 4), round(z_m, 4))

    def _record_stage_time(self, key, elapsed_s):
        stat = self._perf_stats[key]
        stat[0] += elapsed_s * 1000.0
        stat[1] += 1

    def _maybe_log_perf(self):
        if PERF_LOG_INTERVAL_SEC <= 0:
            return
        now = time.time()
        if now - self._perf_time < PERF_LOG_INTERVAL_SEC:
            return
        avg_ms = 0.0
        total_ms, count = self._perf_stats["scout_ms"]
        if count:
            avg_ms = total_ms / count
        self.get_logger().info(f"Global perf: scout={avg_ms:.1f}ms")
        self._perf_stats["scout_ms"] = [0.0, 0]
        self._perf_time = now

    def _run_scout(self, frame):
        t0 = time.perf_counter()
        detections, _ = self.scout.scan(frame, draw_debug=False)
        return detections, time.perf_counter() - t0

    def _poll_future(self):
        if self.scout_future and self.scout_future.done():
            try:
                self.last_scout_results, elapsed = self.scout_future.result()
                self._record_stage_time("scout_ms", elapsed)
            except Exception as e:
                self.get_logger().error(f"Scout Error: {e}")
            self.scout_future = None

    def processing_loop(self):
        self._poll_future()
        self._maybe_log_perf()

        if self.frame_global is None:
            return
        if self.intrinsics is None:
            self.get_logger().warn("Waiting for camera intrinsics...", throttle_duration_sec=2.0)
        elif self.frame_depth_meters is None:
            self.get_logger().warn("Waiting for depth image...", throttle_duration_sec=2.0)

        if self.frame_counter % GLOBAL_SCOUT_EVERY_N_FRAMES == 0 and self.scout_future is None:
            self.scout_future = self.pool.submit(self._run_scout, self.frame_global.copy())

        detections = list(self.last_scout_results)
        timestamp = self.get_clock().now().nanoseconds
        objects = []
        bin_locations = {}

        gray = cv2.cvtColor(self.frame_global, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in SHAPE_CONFIG:
                    continue
                perimeter = cv2.arcLength(corners[i][0], True)
                raw_px_per_mm = (perimeter / 4.0) / self.marker_size_mm
                c = corners[i][0].astype(float)
                raw_cx, raw_cy = np.mean(c[:, 0]), np.mean(c[:, 1])
                px_per_mm, cx, cy = self.get_smoothed_values(marker_id, raw_px_per_mm, raw_cx, raw_cy)
                poly_pts = []
                for key in ["TL", "TR", "BR", "BL"]:
                    off_x, off_y = SHAPE_CONFIG[marker_id][key]
                    poly_pts.append([int(cx + (off_x * px_per_mm)), int(cy + (off_y * px_per_mm))])
                poly_arr = np.array(poly_pts, np.int32).reshape((-1, 1, 2))
                self.active_polygons[int(marker_id)] = poly_arr
                xyz_meters = self.get_3d_coordinates(int(cx), int(cy), poly_arr)
                key_name = "workspace" if int(marker_id) == 0 else f"bin_{int(marker_id)}"
                bin_locations[key_name] = {
                    "id": int(marker_id),
                    "px": [int(cx), int(cy)],
                    "xyz": xyz_meters,
                    "polygon": poly_arr.reshape(-1, 2).tolist(),
                }

        workspace_poly = self.active_polygons.get(0)
        detections.sort(key=lambda x: x.get("box", [0])[0])
        valid_objects = []
        rects = []
        labels = []
        for obj in detections:
            box = obj.get("box") or obj.get("bbox") or obj.get("xyxy")
            label = obj.get("label")
            if not box:
                continue
            raw_cx = int((box[0] + box[2]) / 2)
            raw_cy = int((box[1] + box[3]) / 2)
            if workspace_poly is not None and cv2.pointPolygonTest(workspace_poly, (raw_cx, raw_cy), False) < 0:
                continue
            valid_objects.append(obj)
            rects.append(box)
            labels.append(label)

        tracked_objects = self.tracker.update(rects, labels)
        for obj in valid_objects:
            box = obj.get("box") or obj.get("bbox") or obj.get("xyxy")
            raw_cx = int((box[0] + box[2]) / 2)
            raw_cy = int((box[1] + box[3]) / 2)
            obj_id = -1
            min_dist = 9999.0
            for tracked_id, tracked_center in tracked_objects.items():
                dist = math.hypot(raw_cx - tracked_center[0], raw_cy - tracked_center[1])
                if dist < min_dist and dist < 50:
                    min_dist = dist
                    obj_id = tracked_id

            if obj_id != -1 and obj_id in tracked_objects:
                cx, cy = tracked_objects[obj_id]
            else:
                cx, cy = raw_cx, raw_cy

            pts = None
            angle = 0.0
            segments = obj.get("segments") or obj.get("mask")
            if segments:
                pts = np.array(segments, np.int32).reshape((-1, 1, 2))
                raw_angle, _, _ = calculate_orientation_pca(pts)
                if raw_angle is not None:
                    angle = self.angle_stabilizer.update(obj_id, raw_angle)

            xyz_meters = self.get_3d_coordinates(cx, cy, pts)
            if xyz_meters and obj_id != -1:
                xyz_meters = self.xyz_stabilizer.update(obj_id, xyz_meters)

            objects.append(
                {
                    "id": int(obj_id),
                    "label": obj.get("label"),
                    "confidence": obj.get("confidence"),
                    "box": box,
                    "xyz": xyz_meters,
                    "angle": angle,
                }
            )

        packet = {
            "timestamp": timestamp,
            "objects": objects,
            "bin_locations": bin_locations,
            "image_size": [int(self.frame_global.shape[1]), int(self.frame_global.shape[0])],
        }
        self.state_pub.publish(String(data=json.dumps(packet)))
        if bin_locations:
            self.bin_pub.publish(String(data=json.dumps(bin_locations)))

        self.frame_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = GlobalVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pool.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
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
