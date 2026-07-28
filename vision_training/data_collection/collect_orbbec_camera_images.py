#!/usr/bin/env python3
import os
import sys
from datetime import datetime


def _sanitize_python_path() -> None:
    blocked_prefixes = [
        os.path.expanduser("~/.local/lib/python3.10/site-packages"),
        os.path.expanduser("~/.local/lib/python3.10/site-packages/rerun_sdk"),
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), ".venv"),
    ]
    keep = []
    for path in sys.path:
        normalized = os.path.abspath(path) if path else path
        if normalized and any(normalized.startswith(prefix) for prefix in blocked_prefixes):
            continue
        keep.append(path)
    sys.path[:] = keep


_sanitize_python_path()

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def find_repo_root(current_path: str, target_name: str = "agentic_disassembly") -> str | None:
    curr = os.path.abspath(current_path)
    while curr != os.path.dirname(curr):
        if os.path.basename(curr) == target_name:
            return curr
        curr = os.path.dirname(curr)
    return None


WS_ROOT = find_repo_root(__file__)
EXPECTED_4K_WIDTH = 3840
EXPECTED_4K_HEIGHT = 2160
DATA_COLLECTION_ROOT = os.path.join(
    WS_ROOT or os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")),
    "vision_training",
    "data_collection",
    "data",
)
SAVE_SUBDIR = "hdd"


class OrbbecImageCollector(Node):
    def __init__(self) -> None:
        super().__init__("orbbec_image_collector")
        self.bridge = CvBridge()

        self.image_topic = "/camera/color/image_raw"
        self.camera_info_topic = "/camera/color/camera_info"

        self.frame = None
        self.frame_shape_logged = False
        self.camera_info_logged = False
        self.is_recording = False
        self.counter = 0

        self.timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_path = os.path.join(
            DATA_COLLECTION_ROOT,
            SAVE_SUBDIR,
            f"session_{self.timestamp_str}",
        )
        self.output_dir = os.path.join(self.session_path, "orbbec_camera")
        os.makedirs(self.output_dir, exist_ok=True)

        self.sub_image = self.create_subscription(Image, self.image_topic, self.cb_image, 10)
        self.sub_camera_info = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.cb_camera_info, 10
        )

        self.display_timer = self.create_timer(1.0 / 15.0, self.display_callback)
        self.record_timer = self.create_timer(0.2, self.record_callback)

        self.get_logger().info("--- Orbbec Color Image Collection System ---")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Camera info topic: {self.camera_info_topic}")
        self.get_logger().info(f"Saving images to: {self.output_dir}")
        self.print_controls()

    def print_controls(self) -> None:
        print("\n" + "=" * 60)
        print("  ORBBEC DATA COLLECTION CONTROLS")
        print(f"  Output directory: {self.output_dir}")
        print("  - [S]: Save snapshot")
        print("  - [R]: Toggle continuous recording (5 Hz)")
        print("  - [Q]: Quit")
        print("=" * 60 + "\n")

    def _log_resolution(self, width: int, height: int, source: str) -> None:
        is_4k = width == EXPECTED_4K_WIDTH and height == EXPECTED_4K_HEIGHT
        status = "4K OK" if is_4k else "NOT 4K"
        self.get_logger().info(f"{source} resolution: {width}x{height} [{status}]")

    def cb_camera_info(self, msg: CameraInfo) -> None:
        if self.camera_info_logged:
            return
        self._log_resolution(msg.width, msg.height, "CameraInfo")
        self.camera_info_logged = True

    def cb_image(self, msg: Image) -> None:
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if not self.frame_shape_logged:
                height, width = self.frame.shape[:2]
                self._log_resolution(width, height, "Image")
                self.frame_shape_logged = True
        except Exception as exc:
            self.get_logger().warn(f"Failed to decode image frame: {exc}")

    def save_frame(self, mode: str = "record") -> bool:
        if self.frame is None:
            if mode == "snapshot":
                self.get_logger().warn("Waiting for image frame...")
            return False

        timestamp = datetime.now().strftime("%H%M%S_%f")
        file_name = f"orbbec_{timestamp}.png"
        file_path = os.path.join(self.output_dir, file_name)

        # Save the original full-resolution frame without any preview resizing.
        cv2.imwrite(file_path, self.frame)
        self.counter += 1
        return True

    def record_callback(self) -> None:
        if self.is_recording:
            self.save_frame(mode="record")

    def display_callback(self) -> None:
        target_h = 720

        if self.frame is not None:
            height, width = self.frame.shape[:2]
            scale = target_h / height
            preview = cv2.resize(self.frame, (int(width * scale), target_h))
        else:
            preview = np.zeros((target_h, 1280, 3), dtype=np.uint8)
            cv2.putText(
                preview,
                "WAITING FOR /camera/color/image_raw ...",
                (60, 360),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )

        info_panel = np.zeros((70, preview.shape[1], 3), dtype=np.uint8)
        color_rec = (0, 0, 255) if self.is_recording else (0, 255, 0)
        mode_text = "● RECORDING" if self.is_recording else "○ IDLE"

        cv2.putText(info_panel, mode_text, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_rec, 2)
        cv2.putText(
            info_panel,
            f"TOTAL IMAGES: {self.counter}",
            (300, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            info_panel,
            "[S] SNAP | [R] REC | [Q] QUIT",
            (max(preview.shape[1] - 430, 20), 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (200, 200, 200),
            2,
        )

        final_frame = np.vstack((preview, info_panel))
        cv2.imshow("Orbbec Color Image Collection", final_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Shutting down...")
            cv2.destroyAllWindows()
            rclpy.shutdown()
            sys.exit(0)
        elif key == ord("s"):
            if self.save_frame(mode="snapshot"):
                self.get_logger().info(f"Snapshot #{self.counter} saved.")
        elif key == ord("r"):
            self.is_recording = not self.is_recording
            self.get_logger().info(f"Recording: {'ON' if self.is_recording else 'OFF'}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrbbecImageCollector()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
