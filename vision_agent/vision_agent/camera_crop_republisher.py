#!/usr/bin/env python3
"""
CameraCropRepublisher
=====================
Subscribes to Orbbec camera topics, crops to a configurable ROI, and
republishes the cropped streams.

Performance design
------------------
* Color input   : /camera/color/image_raw/compressed  (~2 MB MJPG vs 25 MB raw)
* Depth input   : /camera/depth/image_raw              (16-bit, rate-limited)
* Input QoS     : BEST_EFFORT / KEEP_LAST depth=1  — no back-pressure on camera
* Output QoS    : RELIABLE    / KEEP_LAST depth=1  — compatible with RViz etc.

Concurrency model
-----------------
Two MutuallyExclusiveCallbackGroups (color + depth) run on the MultiThreadedExecutor.
Within each group only ONE callback runs at a time, so there is no
concurrent access to publishers.  Frame dropping is handled naturally by
depth=1 on the subscription: if we are still processing frame N when N+1
arrives, N+1 is kept and N+2 overwrites it — we always grab the freshest
available frame when the callback finishes.

Four color output streams (single decode per frame):
  .../image_raw             : exact cropped raw Image          → geometry / verification
  .../image_inference       : resized cropped raw Image        → vision model inference
  .../image_raw/compressed  : optional full-res crop JPEG      → compatibility only
  .../image_display         : resized raw Image                → RViz / visualization

Depth output    : rate-limited via depth_rate_divisor (default 1 → pass-through;
                  camera SW alignment already limits to ~2 fps)
"""
import copy
import os
import sys
import threading
import time

from vision_agent.runtime_env import setup_python_env

REPO_ROOT = setup_python_env(__file__)

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField


# ── Topic names ───────────────────────────────────────────────────────────────
COLOR_RAW_TOPIC          = "/camera/color/image_raw"
COLOR_COMPRESSED_TOPIC   = "/camera/color/image_raw/compressed"
COLOR_CAMERA_INFO_TOPIC  = "/camera/color/camera_info"
DEPTH_IMAGE_TOPIC        = "/camera/depth/image_raw"
DEPTH_CAMERA_INFO_TOPIC  = "/camera/depth/camera_info"

# Base topic published as full-resolution cropped raw image
CROPPED_COLOR_IMAGE_TOPIC      = "/camera/cropped/color/image_raw"
CROPPED_COLOR_INFERENCE_TOPIC  = "/camera/cropped/color/image_inference"
# Compressed sub-topic → optional compatibility transport
CROPPED_COLOR_COMPRESSED_TOPIC = "/camera/cropped/color/image_raw/compressed"
# Legacy alias kept so existing RViz panels still work
CROPPED_COLOR_DISPLAY_TOPIC    = "/camera/cropped/color/image_display"
CROPPED_COLOR_CAMERA_INFO_TOPIC= "/camera/cropped/color/camera_info"
CROPPED_DEPTH_IMAGE_TOPIC      = "/camera/cropped/depth/image_raw"
CROPPED_DEPTH_CAMERA_INFO_TOPIC    = "/camera/cropped/depth/camera_info"
CROPPED_POINTCLOUD_TOPIC           = "/camera/cropped/depth_registered/points"
CROPPED_COLOR_DISPLAY_COMPRESSED_TOPIC = "/camera/cropped/color/image_display/compressed"

# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_CROP_X      = 1100
DEFAULT_CROP_Y      = 500
DEFAULT_CROP_WIDTH  = 1800
DEFAULT_CROP_HEIGHT = 1100
DEFAULT_CROP_REFERENCE_WIDTH = 0
DEFAULT_CROP_REFERENCE_HEIGHT = 0

PC_SPATIAL_STRIDE  = 4
PC_TEMPORAL_STRIDE = 6

# ── QoS ───────────────────────────────────────────────────────────────────────
_INPUT_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

_DEPTH_INPUT_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

_OUTPUT_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class CameraCropRepublisher(Node):
    def __init__(self):
        super().__init__("camera_crop_republisher")
        self.bridge = CvBridge()

        # Color and depth each get their own exclusive group so they never
        # block each other, but within each group callbacks are serialized.
        self._color_cb_group = MutuallyExclusiveCallbackGroup()
        self._depth_cb_group = MutuallyExclusiveCallbackGroup()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("crop_x",      DEFAULT_CROP_X)
        self.declare_parameter("crop_y",      DEFAULT_CROP_Y)
        self.declare_parameter("crop_width",  DEFAULT_CROP_WIDTH)
        self.declare_parameter("crop_height", DEFAULT_CROP_HEIGHT)
        self.declare_parameter("crop_reference_width", DEFAULT_CROP_REFERENCE_WIDTH)
        self.declare_parameter("crop_reference_height", DEFAULT_CROP_REFERENCE_HEIGHT)
        self.declare_parameter("enable_depth",      True)
        self.declare_parameter("enable_pointcloud", False)
        self.declare_parameter("depth_rate_divisor", 1)
        self.declare_parameter("display_height", 480)
        self.declare_parameter("display_rate_divisor", 2)
        self.declare_parameter("inference_width", 1280)
        self.declare_parameter("inference_height", 0)
        self.declare_parameter("publish_color_raw", True)  # kept for back-compat
        self.declare_parameter("publish_color_compressed", False)
        self.declare_parameter("publish_display_compressed", True)
        self.declare_parameter("preferred_color_input", "auto")
        self.declare_parameter("preferred_color_fallback_sec", 2.5)
        self.declare_parameter("color_input_stale_sec", 1.0)

        enable_depth      = self.get_parameter("enable_depth").value
        enable_pointcloud = self.get_parameter("enable_pointcloud").value
        self._publish_color_raw = bool(self.get_parameter("publish_color_raw").value)
        self._publish_color_compressed = bool(self.get_parameter("publish_color_compressed").value)
        self._publish_display_compressed = bool(self.get_parameter("publish_display_compressed").value)
        self._depth_rate_divisor = max(1, int(self.get_parameter("depth_rate_divisor").value))
        self._display_height     = int(self.get_parameter("display_height").value)
        self._display_rate_divisor = max(1, int(self.get_parameter("display_rate_divisor").value))
        self._inference_width    = max(0, int(self.get_parameter("inference_width").value))
        self._inference_height   = max(0, int(self.get_parameter("inference_height").value))
        # ── State ─────────────────────────────────────────────────────────────
        self._roi_log_cache  = None
        self._depth_frame_count = 0
        self._depth_ready_logged = False
        self._depth_input_logged = False
        self._depth_info_lock = threading.Lock()
        self.latest_cropped_depth_camera_info: CameraInfo | None = None
        self._preferred_color_input = str(self.get_parameter("preferred_color_input").value).lower()
        self._preferred_color_fallback_sec = max(
            0.0, float(self.get_parameter("preferred_color_fallback_sec").value)
        )
        self._color_input_stale_sec = max(
            0.1, float(self.get_parameter("color_input_stale_sec").value)
        )
        self._active_color_input = None
        self._color_frame_counter = 0
        self._color_input_start_time = time.monotonic()
        self._last_color_frame_time = None

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Image, COLOR_RAW_TOPIC,
            self.cb_color_raw, _INPUT_QOS,
            callback_group=self._color_cb_group,
        )
        self.create_subscription(
            CompressedImage, COLOR_COMPRESSED_TOPIC,
            self.cb_color_compressed, _INPUT_QOS,
            callback_group=self._color_cb_group,
        )
        self.create_subscription(
            CameraInfo, COLOR_CAMERA_INFO_TOPIC,
            self.cb_color_camera_info, _INPUT_QOS,
            # Not in the exclusive color group — camera_info processing is fast
            # and thread-safe; letting it run concurrently avoids blocking color decodes.
        )
        if enable_depth:
            self.create_subscription(
                Image, DEPTH_IMAGE_TOPIC,
                self.cb_depth_image, _DEPTH_INPUT_QOS,
                callback_group=self._depth_cb_group,
            )
            self.create_subscription(
                CameraInfo, DEPTH_CAMERA_INFO_TOPIC,
                self.cb_depth_camera_info, _DEPTH_INPUT_QOS,
                callback_group=self._depth_cb_group,
            )

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_color_compressed = self.create_publisher(
            CompressedImage, CROPPED_COLOR_COMPRESSED_TOPIC, _OUTPUT_QOS,
        ) if self._publish_color_compressed else None
        self.pub_color_image = self.create_publisher(
            Image, CROPPED_COLOR_IMAGE_TOPIC, _OUTPUT_QOS,
        ) if self._publish_color_raw else None
        self.pub_color_inference = self.create_publisher(
            Image, CROPPED_COLOR_INFERENCE_TOPIC, _OUTPUT_QOS,
        )
        # Display topics use sensor_data QoS (BEST_EFFORT) so RViz can subscribe without mismatch
        _DISPLAY_QOS = qos_profile_sensor_data
        self.pub_color_display = self.create_publisher(
            Image, CROPPED_COLOR_DISPLAY_TOPIC, _DISPLAY_QOS,
        ) if self._display_height > 0 else None
        self.pub_display_compressed = self.create_publisher(
            CompressedImage, CROPPED_COLOR_DISPLAY_COMPRESSED_TOPIC, _DISPLAY_QOS,
        ) if (self._display_height > 0 and self._publish_display_compressed) else None
        self.pub_color_camera_info = self.create_publisher(
            CameraInfo, CROPPED_COLOR_CAMERA_INFO_TOPIC, _OUTPUT_QOS,
        )
        self.pub_depth_image = self.create_publisher(
            Image, CROPPED_DEPTH_IMAGE_TOPIC, _OUTPUT_QOS,
        ) if enable_depth else None
        self.pub_depth_camera_info = self.create_publisher(
            CameraInfo, CROPPED_DEPTH_CAMERA_INFO_TOPIC, _OUTPUT_QOS,
        ) if enable_depth else None
        self.pub_pointcloud = self.create_publisher(
            PointCloud2, CROPPED_POINTCLOUD_TOPIC, _OUTPUT_QOS,
        ) if (enable_depth and enable_pointcloud) else None

        depth_fps_in = 15
        eff_depth = (
            f"~{depth_fps_in // self._depth_rate_divisor}fps"
            if enable_depth else "OFF"
        )
        disp_str = f"{self._display_height}px" if self._display_height > 0 else "full-res"
        self.get_logger().info(
            f"CameraCropRepublisher ready  "
            f"(color input=auto(raw/compressed) → output=RELIABLE, depth={eff_depth}, "
            f"display={disp_str}, "
            f"pointcloud={'ON' if (enable_depth and enable_pointcloud) else 'OFF'})\n"
            f"  color input preference: {self._preferred_color_input}\n"
            f"  exact crop:            /camera/cropped/color/image_raw\n"
            f"  inference image:       /camera/cropped/color/image_inference\n"
            f"  preview raw:           "
            f"{('/camera/cropped/color/image_display (' + disp_str + ', every ' + str(self._display_rate_divisor) + ' frame(s))') if self.pub_color_display is not None else 'OFF'}\n"
            f"  preview compressed:    "
            f"{('/camera/cropped/color/image_display/compressed (' + disp_str + ')') if self.pub_display_compressed is not None else 'OFF'}\n"
            f"  compressed full crop:  {'ON' if self._publish_color_compressed else 'OFF'}"
        )

    # ── ROI helpers ────────────────────────────────────────────────────────────

    def _get_crop_roi(self, width, height):
        cx = max(0, int(self.get_parameter("crop_x").value))
        cy = max(0, int(self.get_parameter("crop_y").value))
        cw = int(self.get_parameter("crop_width").value)
        ch = int(self.get_parameter("crop_height").value)
        ref_w = max(0, int(self.get_parameter("crop_reference_width").value))
        ref_h = max(0, int(self.get_parameter("crop_reference_height").value))

        if ref_w > 0 and ref_h > 0 and (ref_w != width or ref_h != height):
            scale_x = width / float(ref_w)
            scale_y = height / float(ref_h)
            cx = int(round(cx * scale_x))
            cy = int(round(cy * scale_y))
            if cw > 0:
                cw = int(round(cw * scale_x))
            if ch > 0:
                ch = int(round(ch * scale_y))

        if cx >= width or cy >= height:
            return 0, 0, width, height
        cw = min(cw if cw > 0 else width  - cx, width  - cx)
        ch = min(ch if ch > 0 else height - cy, height - cy)
        return cx, cy, cw, ch

    def _log_roi_once(self, name, w, h, roi):
        key = (name, w, h, roi)
        if self._roi_log_cache == key:
            return
        self._roi_log_cache = key
        cx, cy, cw, ch = roi
        self.get_logger().info(
            f"{name} crop roi  x={cx} y={cy} w={cw} h={ch}  (source {w}×{h})"
        )

    def _crop_camera_info(self, msg, roi):
        cx, cy, cw, ch = roi
        out = copy.deepcopy(msg)
        out.width  = cw
        out.height = ch
        k, p = list(out.k), list(out.p)
        k[2] -= float(cx);  k[5] -= float(cy)
        p[2] -= float(cx);  p[6] -= float(cy)
        out.k = k;  out.p = p
        out.roi.x_offset = cx;  out.roi.y_offset = cy
        out.roi.width = cw;     out.roi.height = ch
        return out

    def _resize_for_inference(self, cropped_bgr):
        if self._inference_width <= 0 and self._inference_height <= 0:
            return cropped_bgr

        h, w = cropped_bgr.shape[:2]
        if self._inference_width > 0 and self._inference_height > 0:
            out_w = self._inference_width
            out_h = self._inference_height
        elif self._inference_width > 0:
            scale = self._inference_width / float(w)
            out_w = self._inference_width
            out_h = max(1, int(round(h * scale)))
        else:
            scale = self._inference_height / float(h)
            out_h = self._inference_height
            out_w = max(1, int(round(w * scale)))

        if out_w == w and out_h == h:
            return cropped_bgr

        return cv2.resize(cropped_bgr, (out_w, out_h), interpolation=cv2.INTER_LINEAR)

    def _resize_for_display(self, cropped_bgr, inference_bgr=None):
        if self._display_height <= 0:
            return None

        h, w = cropped_bgr.shape[:2]
        if h <= self._display_height:
            return cropped_bgr

        disp_h = self._display_height
        disp_w = max(1, round(w * disp_h / h))

        if inference_bgr is not None:
            inf_h, inf_w = inference_bgr.shape[:2]
            if inf_w == disp_w and inf_h == disp_h:
                return inference_bgr
            # Resize from inference image (smaller source = faster than from full crop)
            return cv2.resize(inference_bgr, (disp_w, disp_h), interpolation=cv2.INTER_LINEAR)

        return cv2.resize(cropped_bgr, (disp_w, disp_h), interpolation=cv2.INTER_LINEAR)

    # ── Color ──────────────────────────────────────────────────────────────────

    def _accept_color_source(self, source_name: str) -> bool:
        if self._preferred_color_input in {"raw", "compressed"}:
            if self._active_color_input is not None:
                if self._active_color_input == source_name:
                    return True
                if (
                    source_name == self._preferred_color_input
                    and self._active_color_input != self._preferred_color_input
                ):
                    previous_source = self._active_color_input
                    self._active_color_input = source_name
                    self.get_logger().info(
                        f"Preferred {source_name} color input became available; "
                        f"switching from {previous_source}"
                    )
                    return True
                if (
                    self._last_color_frame_time is not None
                    and (time.monotonic() - self._last_color_frame_time) > self._color_input_stale_sec
                ):
                    previous_source = self._active_color_input
                    self._active_color_input = source_name
                    self.get_logger().warn(
                        f"Active {previous_source} color input went stale; "
                        f"switching to {source_name}"
                    )
                    return True
                return False

            if source_name == self._preferred_color_input:
                self._active_color_input = source_name
                self.get_logger().info(
                    f"Using preferred {source_name} color input for crop republisher"
                )
                return True

            if (time.monotonic() - self._color_input_start_time) < self._preferred_color_fallback_sec:
                return False

            self._active_color_input = source_name
            self.get_logger().warn(
                f"Preferred {self._preferred_color_input} color input unavailable after "
                f"{self._preferred_color_fallback_sec:.1f}s, falling back to {source_name}"
            )
            return True

        if self._active_color_input is None:
            self._active_color_input = source_name
            self.get_logger().info(f"Using {source_name} color input for crop republisher")
            return True

        return self._active_color_input == source_name

    def _publish_cropped_color(self, img_bgr, header):
        self._color_frame_counter += 1
        h, w = img_bgr.shape[:2]
        roi = self._get_crop_roi(w, h)
        self._log_roi_once("color", w, h, roi)
        cx, cy, cw, ch = roi

        cropped_bgr = np.ascontiguousarray(img_bgr[cy:cy + ch, cx:cx + cw])
        self._last_color_frame_time = time.monotonic()

        if self.pub_color_compressed is not None:
            ok, enc = cv2.imencode(".jpg", cropped_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                comp = CompressedImage()
                comp.header = header
                comp.format = "jpeg"
                comp.data = enc.tobytes()
                self.pub_color_compressed.publish(comp)

        if self.pub_color_image is not None:
            raw_msg = self.bridge.cv2_to_imgmsg(cropped_bgr, encoding="bgr8")
            raw_msg.header = header
            self.pub_color_image.publish(raw_msg)

        inference_bgr = self._resize_for_inference(cropped_bgr)
        inference_msg = self.bridge.cv2_to_imgmsg(inference_bgr, encoding="bgr8")
        inference_msg.header = header
        self.pub_color_inference.publish(inference_msg)

        if (
            self._color_frame_counter == 1
            or self._color_frame_counter % self._display_rate_divisor == 0
        ):
            display_bgr = self._resize_for_display(cropped_bgr, inference_bgr=inference_bgr)
            if display_bgr is not None:
                if self.pub_color_display is not None:
                    disp_msg = self.bridge.cv2_to_imgmsg(display_bgr, encoding="bgr8")
                    disp_msg.header = header
                    self.pub_color_display.publish(disp_msg)
                if self.pub_display_compressed is not None:
                    ok, enc = cv2.imencode(".jpg", display_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ok:
                        comp = CompressedImage()
                        comp.header = header
                        comp.format = "jpeg"
                        comp.data = enc.tobytes()
                        self.pub_display_compressed.publish(comp)

    def cb_color_raw(self, msg: Image):
        if not self._accept_color_source("raw"):
            return
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._publish_cropped_color(img_bgr, msg.header)
        except Exception as exc:
            self.get_logger().error(f"Raw color crop error: {exc}")

    def cb_color_compressed(self, msg: CompressedImage):
        if not self._accept_color_source("compressed"):
            return
        try:
            img_bgr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._publish_cropped_color(img_bgr, msg.header)
        except Exception as exc:
            self.get_logger().error(f"Color crop error: {exc}")

    def cb_color_camera_info(self, msg: CameraInfo):
        roi = self._get_crop_roi(msg.width, msg.height)
        self.pub_color_camera_info.publish(self._crop_camera_info(msg, roi))

    # ── Depth ──────────────────────────────────────────────────────────────────

    def cb_depth_image(self, msg: Image):
        self._depth_frame_count += 1
        if self._depth_frame_count % self._depth_rate_divisor != 0:
            return
        try:
            if not self._depth_input_logged:
                self._depth_input_logged = True
                self.get_logger().info(
                    f"Depth input received: {msg.width}x{msg.height} {msg.encoding} step={msg.step}"
                )
            roi = self._get_crop_roi(msg.width, msg.height)
            cx, cy, cw, ch = roi

            desired_encoding = "16UC1" if msg.encoding == "16UC1" else "passthrough"
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding)
            if depth_raw.dtype != np.uint16:
                depth_raw = depth_raw.astype(np.uint16, copy=False)

            cropped = np.ascontiguousarray(depth_raw[cy:cy + ch, cx:cx + cw])

            depth_out = self.bridge.cv2_to_imgmsg(cropped, encoding="16UC1")
            depth_out.header = msg.header
            if self.pub_depth_image is not None:
                self.pub_depth_image.publish(depth_out)
                if not self._depth_ready_logged:
                    self._depth_ready_logged = True
                    self.get_logger().info(
                        f"Depth crop stream connected: {msg.width}x{msg.height} {msg.encoding} -> 16UC1"
                    )

            if self.pub_pointcloud is not None:
                pc_tick = self._depth_frame_count // self._depth_rate_divisor
                if pc_tick % PC_TEMPORAL_STRIDE == 0:
                    with self._depth_info_lock:
                        cam_info = self.latest_cropped_depth_camera_info
                    if cam_info is not None:
                        self.pub_pointcloud.publish(
                            self._build_pointcloud(
                                depth_out.header, cropped,
                                msg.encoding, cam_info,
                                stride=PC_SPATIAL_STRIDE,
                            )
                        )
        except Exception as exc:
            self.get_logger().error(f"Depth crop error: {exc}")

    def cb_depth_camera_info(self, msg: CameraInfo):
        roi = self._get_crop_roi(msg.width, msg.height)
        info = self._crop_camera_info(msg, roi)
        with self._depth_info_lock:
            self.latest_cropped_depth_camera_info = info
        if self.pub_depth_camera_info is not None:
            self.pub_depth_camera_info.publish(info)

    # ── Point cloud ────────────────────────────────────────────────────────────

    def _build_pointcloud(self, header, depth_img, encoding, cam_info, stride=1):
        if encoding == "16UC1":
            depth_m = depth_img[::stride, ::stride].astype(np.float32) / 1000.0
        elif encoding == "32FC1":
            depth_m = depth_img[::stride, ::stride].astype(np.float32)
        else:
            raise ValueError(f"Unsupported depth encoding: {encoding}")

        h, w = depth_m.shape
        fx = float(cam_info.k[0]) / stride
        fy = float(cam_info.k[4]) / stride
        cx = float(cam_info.k[2]) / stride
        cy = float(cam_info.k[5]) / stride

        uu, vv = np.meshgrid(np.arange(w, dtype=np.float32),
                             np.arange(h, dtype=np.float32))
        valid = depth_m > 0.0
        x = np.where(valid, (uu - cx) * depth_m / fx, np.nan).astype(np.float32)
        y = np.where(valid, (vv - cy) * depth_m / fy, np.nan).astype(np.float32)
        z = np.where(valid, depth_m,                  np.nan).astype(np.float32)

        pc = PointCloud2()
        pc.header = header
        pc.height = h;  pc.width = w
        pc.fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        pc.is_bigendian = False
        pc.point_step = 12
        pc.row_step    = w * 12
        pc.data        = np.dstack((x, y, z)).tobytes()
        pc.is_dense    = False
        return pc


def main(args=None):
    rclpy.init(args=args)
    node = CameraCropRepublisher()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
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
