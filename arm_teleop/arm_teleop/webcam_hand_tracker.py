from __future__ import annotations

from collections import deque
import os
import statistics
import tempfile
import time
import tkinter as tk
import warnings
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from PIL import Image, ImageTk
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from arm_teleop.hand_math import cross, matrix_to_quat, normalize


class WebcamHandTracker(Node):
    def __init__(self):
        super().__init__("webcam_hand_tracker")

        self.declare_parameter("camera_index", -1)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("model_complexity", 0)
        self.declare_parameter("show_visualization", True)
        self.declare_parameter("visualization_scale", 1.6)
        self.declare_parameter("mirror_view", True)
        self.declare_parameter("max_num_hands", 2)
        self.declare_parameter("min_detection_confidence", 0.6)
        self.declare_parameter("min_tracking_confidence", 0.5)
        self.declare_parameter("frame_id", "webcam_hand_tracking")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("pinch_on_threshold", 0.055)
        self.declare_parameter("pinch_off_threshold", 0.075)
        self.declare_parameter("pinky_pinch_on_threshold", 0.060)
        self.declare_parameter("pinky_pinch_off_threshold", 0.080)
        self.declare_parameter("fist_on_threshold", 0.085)
        self.declare_parameter("fist_off_threshold", 0.110)
        self.declare_parameter("gripper_aperture_min_ratio", 0.55)
        self.declare_parameter("gripper_aperture_max_ratio", 2.00)
        self.declare_parameter("gripper_aperture_filter_alpha", 0.18)
        self.declare_parameter("gripper_aperture_deadband", 0.02)
        self.declare_parameter("gripper_aperture_max_step", 0.04)
        self.declare_parameter("gripper_aperture_median_window", 7)
        self.declare_parameter("depth_scale_gain", 0.80)
        self.declare_parameter("depth_raw_blend", 0.15)
        self.declare_parameter("enable_uf850", True)
        self.declare_parameter("enable_xarm5", True)
        self.declare_parameter("uf850.hand", "right")
        self.declare_parameter("xarm5.hand", "left")

        self.right_pub = self.create_publisher(PoseStamped, "/teleop_hand_tracking/right/wrist", 10)
        self.left_pub = self.create_publisher(PoseStamped, "/teleop_hand_tracking/left/wrist", 10)
        self.right_pinch_pub = self.create_publisher(Bool, "/teleop_hand_tracking/right/pinch", 10)
        self.left_pinch_pub = self.create_publisher(Bool, "/teleop_hand_tracking/left/pinch", 10)
        self.right_pinky_pinch_pub = self.create_publisher(Bool, "/teleop_hand_tracking/right/pinky_pinch", 10)
        self.left_pinky_pinch_pub = self.create_publisher(Bool, "/teleop_hand_tracking/left/pinky_pinch", 10)
        self.right_fist_pub = self.create_publisher(Bool, "/teleop_hand_tracking/right/fist", 10)
        self.left_fist_pub = self.create_publisher(Bool, "/teleop_hand_tracking/left/fist", 10)
        self.right_gripper_aperture_pub = self.create_publisher(Float32, "/teleop_hand_tracking/right/gripper_aperture", 10)
        self.left_gripper_aperture_pub = self.create_publisher(Float32, "/teleop_hand_tracking/left/gripper_aperture", 10)
        self.debug_pub = self.create_publisher(String, "/teleop_hand_tracking/debug", 10)
        self.create_subscription(Bool, "/teleop_status/right_arm_enabled", self._right_arm_enabled_cb, 10)
        self.create_subscription(Bool, "/teleop_status/left_arm_enabled", self._left_arm_enabled_cb, 10)

        self._frame_id = self.get_parameter("frame_id").value
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        self._show_visualization = bool(self.get_parameter("show_visualization").value)
        self._visualization_scale = max(float(self.get_parameter("visualization_scale").value), 0.5)
        self._mirror_view = bool(self.get_parameter("mirror_view").value)
        self._period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._pinch_on_threshold = float(self.get_parameter("pinch_on_threshold").value)
        self._pinch_off_threshold = float(self.get_parameter("pinch_off_threshold").value)
        self._pinky_pinch_on_threshold = float(self.get_parameter("pinky_pinch_on_threshold").value)
        self._pinky_pinch_off_threshold = float(self.get_parameter("pinky_pinch_off_threshold").value)
        self._fist_on_threshold = float(self.get_parameter("fist_on_threshold").value)
        self._fist_off_threshold = float(self.get_parameter("fist_off_threshold").value)
        self._gripper_aperture_min_ratio = float(self.get_parameter("gripper_aperture_min_ratio").value)
        self._gripper_aperture_max_ratio = max(
            self._gripper_aperture_min_ratio + 1e-3,
            float(self.get_parameter("gripper_aperture_max_ratio").value),
        )
        self._gripper_aperture_alpha = max(
            0.01,
            min(1.0, float(self.get_parameter("gripper_aperture_filter_alpha").value)),
        )
        self._gripper_aperture_deadband = max(0.0, float(self.get_parameter("gripper_aperture_deadband").value))
        self._gripper_aperture_max_step = max(0.001, float(self.get_parameter("gripper_aperture_max_step").value))
        self._gripper_aperture_median_window = max(1, int(self.get_parameter("gripper_aperture_median_window").value))
        self._depth_scale_gain = float(self.get_parameter("depth_scale_gain").value)
        self._depth_raw_blend = max(0.0, min(1.0, float(self.get_parameter("depth_raw_blend").value)))
        self._enable_uf850 = bool(self.get_parameter("enable_uf850").value)
        self._enable_xarm5 = bool(self.get_parameter("enable_xarm5").value)
        self._uf850_hand = self._normalize_hand_name(str(self.get_parameter("uf850.hand").value), "right")
        self._xarm5_hand = self._normalize_hand_name(str(self.get_parameter("xarm5.hand").value), "left")
        if self._enable_uf850 and self._enable_xarm5 and self._uf850_hand == self._xarm5_hand:
            self._xarm5_hand = "left" if self._uf850_hand == "right" else "right"
        self._pinch_state = {"left": False, "right": False}
        self._pinky_pinch_state = {"left": False, "right": False}
        self._fist_state = {"left": False, "right": False}
        self._gripper_aperture_state = {
            "left": {"filtered": 0.0, "initialized": False, "history": deque(maxlen=self._gripper_aperture_median_window)},
            "right": {"filtered": 0.0, "initialized": False, "history": deque(maxlen=self._gripper_aperture_median_window)},
        }
        self._arm_enabled = {"right": False, "left": False}
        self._window_initialized = False
        self._window_name = "Webcam Hand Tracking"
        self._window_size = None
        self._visual_root: tk.Tk | None = None
        self._visual_label: tk.Label | None = None
        self._visual_image = None

        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError("OpenCV is required for webcam teleoperation.") from exc
        self.cv2 = cv2

        # MediaPipe imports matplotlib internally on this platform. Keep it on a
        # headless backend and silence the optional 3D projection warning.
        if "MPLBACKEND" not in os.environ:
            os.environ["MPLBACKEND"] = "Agg"
        if "MPLCONFIGDIR" not in os.environ:
            os.environ["MPLCONFIGDIR"] = os.path.join(tempfile.gettempdir(), "mplconfig")
        warnings.filterwarnings(
            "ignore",
            message="Unable to import Axes3D.*",
            category=UserWarning,
        )
        try:
            import mediapipe as mp
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "MediaPipe is required for webcam hand tracking. Install the Python package 'mediapipe'."
            ) from exc
        self.mp = mp

        requested_camera_index = int(self.get_parameter("camera_index").value)
        self.camera_index, self.cap = self._open_camera(requested_camera_index)
        if self.cap is None:
            available = ", ".join(self._available_video_devices()) or "none"
            raise RuntimeError(
                "Unable to open a usable webcam. "
                f"requested_camera_index={requested_camera_index}, available_devices={available}"
            )

        self.hands = self.mp.solutions.hands.Hands(
            static_image_mode=False,
            model_complexity=int(self.get_parameter("model_complexity").value),
            max_num_hands=int(self.get_parameter("max_num_hands").value),
            min_detection_confidence=float(self.get_parameter("min_detection_confidence").value),
            min_tracking_confidence=float(self.get_parameter("min_tracking_confidence").value),
        )
        self.drawer = self.mp.solutions.drawing_utils
        self.hand_styles = self.mp.solutions.drawing_styles

        self.timer = self.create_timer(self._period, self._tick)
        self.get_logger().info(
            f"Webcam hand tracker started on camera_index={self.camera_index}. "
            "Topics: /teleop_hand_tracking/{left,right}/wrist"
        )

    def _available_video_devices(self) -> list[str]:
        devices = sorted(Path("/dev").glob("video*"), key=lambda path: path.name)
        return [str(path) for path in devices]

    def _candidate_camera_indices(self, requested_index: int) -> list[int]:
        candidates: list[int] = []
        if requested_index >= 0:
            candidates.append(requested_index)
        for device in self._available_video_devices():
            try:
                index = int(Path(device).name.replace("video", ""))
            except ValueError:
                continue
            if index not in candidates:
                candidates.append(index)
        return candidates

    def _try_open_camera(self, camera_index: int):
        cap = self.cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, float(self._camera_width))
        cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, float(self._camera_height))
        ok, _frame = cap.read()
        if not ok:
            cap.release()
            return None
        return cap

    def _open_camera(self, requested_index: int):
        attempted: list[int] = []
        for camera_index in self._candidate_camera_indices(requested_index):
            attempted.append(camera_index)
            cap = self._try_open_camera(camera_index)
            if cap is not None:
                if requested_index >= 0 and camera_index != requested_index:
                    self.get_logger().warning(
                        f"Requested camera_index={requested_index} was unavailable. "
                        f"Falling back to camera_index={camera_index}."
                    )
                elif requested_index < 0:
                    self.get_logger().info(f"Auto-selected camera_index={camera_index}.")
                actual_width = int(cap.get(self.cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(cap.get(self.cv2.CAP_PROP_FRAME_HEIGHT))
                self.get_logger().info(
                    "Using webcam "
                    f"camera_index={camera_index} at resolution {actual_width}x{actual_height} "
                    f"(requested {self._camera_width}x{self._camera_height})."
                )
                return camera_index, cap
        self.get_logger().error(
            f"Failed to open any usable camera. attempted_indices={attempted}, "
            f"available_devices={self._available_video_devices()}"
        )
        return None, None

    def _normalize_hand_name(self, value: str, default: str) -> str:
        hand = value.strip().lower()
        return hand if hand in ("left", "right") else default

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        try:
            if self._show_visualization:
                if self._visual_root is not None:
                    self._visual_root.destroy()
        except Exception:
            pass
        return super().destroy_node()

    def _on_visualization_resize(self, event):
        if event.widget is self._visual_root and event.width > 1 and event.height > 1:
            self._window_size = (int(event.width), int(event.height))

    def _close_visualization_window(self):
        self.get_logger().info("Visualization window requested shutdown.")
        if self._visual_root is not None:
            try:
                self._visual_root.destroy()
            except Exception:
                pass
            self._visual_root = None
            self._visual_label = None
            self._visual_image = None
        time.sleep(0.1)
        if rclpy.ok():
            rclpy.shutdown()

    def _ensure_visualization_window(self, frame):
        if self._window_initialized:
            return
        height, width = frame.shape[0], frame.shape[1]
        window_width = max(int(width * self._visualization_scale), 640)
        window_height = max(int(height * self._visualization_scale), 480)
        self._visual_root = tk.Tk()
        self._visual_root.title(self._window_name)
        self._visual_root.geometry(f"{window_width}x{window_height}")
        self._visual_root.minsize(320, 240)
        self._visual_root.configure(bg="black")
        self._visual_root.protocol("WM_DELETE_WINDOW", self._close_visualization_window)
        self._visual_root.bind("<Configure>", self._on_visualization_resize)
        self._visual_label = tk.Label(self._visual_root, bg="black", bd=0, highlightthickness=0)
        self._visual_label.pack(fill=tk.BOTH, expand=True)
        self._visual_root.update_idletasks()
        self._window_size = (
            max(self._visual_label.winfo_width(), window_width),
            max(self._visual_label.winfo_height(), window_height),
        )
        self._window_initialized = True

    def _show_visualization_frame(self, frame):
        self._ensure_visualization_window(frame)
        if self._visual_root is None or self._visual_label is None:
            return
        current_width = max(self._visual_label.winfo_width(), 1)
        current_height = max(self._visual_label.winfo_height(), 1)
        self._window_size = (current_width, current_height)
        display_frame = self.cv2.resize(
            frame,
            (current_width, current_height),
            interpolation=self.cv2.INTER_LINEAR,
        )
        rgb_frame = self.cv2.cvtColor(display_frame, self.cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb_frame)
        self._visual_image = ImageTk.PhotoImage(image=image)
        self._visual_label.configure(image=self._visual_image)
        try:
            self._visual_root.update_idletasks()
            self._visual_root.update()
        except tk.TclError:
            self._close_visualization_window()

    def _normalized_point(self, landmark) -> list[float]:
        return [
            float(landmark.x) - 0.5,
            0.5 - float(landmark.y),
            -float(landmark.z),
        ]

    def _pose_from_landmarks(self, hand_landmarks) -> tuple[list[float], list[float]] | None:
        wrist = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.WRIST])
        index_mcp = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP])
        middle_mcp = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.MIDDLE_FINGER_MCP])
        pinky_mcp = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.PINKY_MCP])
        raw_palm_center = [
            (wrist[0] + index_mcp[0] + middle_mcp[0] + pinky_mcp[0]) / 4.0,
            (wrist[1] + index_mcp[1] + middle_mcp[1] + pinky_mcp[1]) / 4.0,
            (wrist[2] + index_mcp[2] + middle_mcp[2] + pinky_mcp[2]) / 4.0,
        ]
        palm_width = sum((index_mcp[i] - pinky_mcp[i]) ** 2 for i in range(3)) ** 0.5
        palm_height = sum((wrist[i] - middle_mcp[i]) ** 2 for i in range(3)) ** 0.5
        palm_scale_depth = 0.5 * (palm_width + palm_height) * self._depth_scale_gain
        palm_center = [
            raw_palm_center[0],
            raw_palm_center[1],
            self._depth_raw_blend * raw_palm_center[2] + (1.0 - self._depth_raw_blend) * palm_scale_depth,
        ]

        x_axis = normalize([index_mcp[i] - wrist[i] for i in range(3)])
        side_axis = normalize([pinky_mcp[i] - wrist[i] for i in range(3)])
        if x_axis is None or side_axis is None:
            return None
        z_axis = normalize(cross(x_axis, side_axis))
        if z_axis is None:
            return None
        y_axis = normalize(cross(z_axis, x_axis))
        if y_axis is None:
            return None
        quat = matrix_to_quat(
            [
                [x_axis[0], y_axis[0], z_axis[0]],
                [x_axis[1], y_axis[1], z_axis[1]],
                [x_axis[2], y_axis[2], z_axis[2]],
            ]
        )
        return palm_center, quat

    def _compute_pinch(self, hand_landmarks, label: str) -> tuple[bool, float]:
        thumb_tip = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.THUMB_TIP])
        index_tip = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP])
        distance = sum((thumb_tip[i] - index_tip[i]) ** 2 for i in range(3)) ** 0.5
        previous = self._pinch_state.get(label, False)
        if previous:
            current = distance < self._pinch_off_threshold
        else:
            current = distance < self._pinch_on_threshold
        self._pinch_state[label] = current
        return current, distance

    def _compute_pinky_pinch(self, hand_landmarks, label: str) -> tuple[bool, float]:
        thumb_tip = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.THUMB_TIP])
        pinky_tip = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.PINKY_TIP])
        distance = sum((thumb_tip[i] - pinky_tip[i]) ** 2 for i in range(3)) ** 0.5
        previous = self._pinky_pinch_state.get(label, False)
        if previous:
            current = distance < self._pinky_pinch_off_threshold
        else:
            current = distance < self._pinky_pinch_on_threshold
        self._pinky_pinch_state[label] = current
        return current, distance

    def _compute_fist(self, hand_landmarks, label: str) -> tuple[bool, float]:
        wrist = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.WRIST])
        index_mcp = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP])
        middle_mcp = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.MIDDLE_FINGER_MCP])
        pinky_mcp = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.PINKY_MCP])
        palm_center = [
            (wrist[0] + index_mcp[0] + middle_mcp[0] + pinky_mcp[0]) / 4.0,
            (wrist[1] + index_mcp[1] + middle_mcp[1] + pinky_mcp[1]) / 4.0,
            (wrist[2] + index_mcp[2] + middle_mcp[2] + pinky_mcp[2]) / 4.0,
        ]
        tip_indices = (
            self.mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp.solutions.hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp.solutions.hands.HandLandmark.RING_FINGER_TIP,
            self.mp.solutions.hands.HandLandmark.PINKY_TIP,
        )
        tip_distances = []
        for tip_index in tip_indices:
            tip = self._normalized_point(hand_landmarks.landmark[tip_index])
            tip_distances.append(sum((tip[i] - palm_center[i]) ** 2 for i in range(3)) ** 0.5)
        average_distance = sum(tip_distances) / max(len(tip_distances), 1)
        previous = self._fist_state.get(label, False)
        if previous:
            current = average_distance < self._fist_off_threshold
        else:
            current = average_distance < self._fist_on_threshold
        self._fist_state[label] = current
        return current, average_distance

    def _compute_gripper_aperture(self, hand_landmarks) -> tuple[float, float]:
        thumb_tip = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.THUMB_TIP])
        index_tip = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP])
        thumb_index_distance = sum((thumb_tip[i] - index_tip[i]) ** 2 for i in range(3)) ** 0.5

        index_mcp = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP])
        pinky_mcp = self._normalized_point(hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.PINKY_MCP])
        palm_width = sum((index_mcp[i] - pinky_mcp[i]) ** 2 for i in range(3)) ** 0.5
        if palm_width < 1e-6:
            return 0.0, thumb_index_distance

        aperture_ratio = thumb_index_distance / palm_width
        normalized_aperture = (aperture_ratio - self._gripper_aperture_min_ratio) / (
            self._gripper_aperture_max_ratio - self._gripper_aperture_min_ratio
        )
        normalized_aperture = max(0.0, min(1.0, normalized_aperture))
        return normalized_aperture, aperture_ratio

    def _filter_gripper_aperture(self, hand: str, aperture: float) -> float:
        # Only apply a short median window to reject single-frame outliers.
        # All smoothing, deadband, and hysteresis is handled in the teleop node.
        state = self._gripper_aperture_state[hand]
        raw = max(0.0, min(1.0, float(aperture)))
        state["history"].append(raw)
        return float(statistics.median(state["history"]))

    def _publish_pose(self, publisher, position: list[float], quat: list[float]):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        publisher.publish(msg)

    def _right_arm_enabled_cb(self, msg: Bool):
        self._arm_enabled["right"] = bool(msg.data)

    def _left_arm_enabled_cb(self, msg: Bool):
        self._arm_enabled["left"] = bool(msg.data)

    def _robot_enabled(self, robot_name: str) -> bool:
        if robot_name == "uf850":
            return self._enable_uf850 and self._arm_enabled[self._uf850_hand]
        if robot_name == "xarm5":
            return self._enable_xarm5 and self._arm_enabled[self._xarm5_hand]
        return False

    def _robot_status_text(self, robot_name: str) -> tuple[str, tuple[int, int, int]]:
        if robot_name == "uf850":
            if not self._enable_uf850:
                return "UF850 (NONE): OFF", (0, 140, 255)
            hand_name = self._uf850_hand.upper()
            enabled = self._robot_enabled("uf850")
            return (
                f"UF850 ({hand_name}): {'ENABLED' if enabled else 'DISABLED'}",
                (0, 255, 0) if enabled else (0, 140, 255),
            )
        if robot_name == "xarm5":
            if not self._enable_xarm5:
                return "XARM5 (NONE): OFF", (0, 140, 255)
            hand_name = self._xarm5_hand.upper()
            enabled = self._robot_enabled("xarm5")
            return (
                f"XARM5 ({hand_name}): {'ENABLED' if enabled else 'DISABLED'}",
                (0, 255, 0) if enabled else (0, 140, 255),
            )
        return "UNKNOWN: OFF", (0, 140, 255)

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Failed to read frame from webcam.")
            return

        if self._mirror_view:
            frame = self.cv2.flip(frame, 1)

        rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.hands.process(rgb)
        rgb.flags.writeable = True

        tracked = {}
        debug_lines = []

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                label = handedness.classification[0].label.lower()
                pose = self._pose_from_landmarks(hand_landmarks)
                if pose is None:
                    continue
                pinch, pinch_distance = self._compute_pinch(hand_landmarks, label)
                pinky_pinch, pinky_pinch_distance = self._compute_pinky_pinch(hand_landmarks, label)
                fist, fist_metric = self._compute_fist(hand_landmarks, label)
                gripper_aperture_raw, gripper_aperture_ratio = self._compute_gripper_aperture(hand_landmarks)
                gripper_aperture = self._filter_gripper_aperture(label, gripper_aperture_raw)
                tracked[label] = {
                    "pose": pose,
                    "pinch": pinch,
                    "pinch_distance": pinch_distance,
                    "pinky_pinch": pinky_pinch,
                    "pinky_pinch_distance": pinky_pinch_distance,
                    "fist": fist,
                    "fist_metric": fist_metric,
                    "gripper_aperture": gripper_aperture,
                    "gripper_aperture_ratio": gripper_aperture_ratio,
                }
                wrist, quat = pose
                debug_lines.append(
                    f"{label}: pos=({wrist[0]:+.3f},{wrist[1]:+.3f},{wrist[2]:+.3f}) "
                    f"pinch={'on' if pinch else 'off'} d={pinch_distance:.3f} "
                    f"pinky={'on' if pinky_pinch else 'off'} d={pinky_pinch_distance:.3f} "
                    f"fist={'on' if fist else 'off'} m={fist_metric:.3f} "
                    f"grip={gripper_aperture:.2f} raw={gripper_aperture_raw:.2f} r={gripper_aperture_ratio:.2f} "
                    f"quat=({quat[0]:+.2f},{quat[1]:+.2f},{quat[2]:+.2f},{quat[3]:+.2f})"
                )
                if self._show_visualization:
                    self.drawer.draw_landmarks(
                        frame,
                        hand_landmarks,
                        self.mp.solutions.hands.HAND_CONNECTIONS,
                        self.hand_styles.get_default_hand_landmarks_style(),
                        self.hand_styles.get_default_hand_connections_style(),
                    )
                    thumb_tip_px = hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.THUMB_TIP]
                    index_tip_px = hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP]
                    thumb_point = (
                        int(thumb_tip_px.x * frame.shape[1]),
                        int(thumb_tip_px.y * frame.shape[0]),
                    )
                    index_point = (
                        int(index_tip_px.x * frame.shape[1]),
                        int(index_tip_px.y * frame.shape[0]),
                    )
                    self.cv2.line(frame, thumb_point, index_point, (0, 255, 255), 3)
                    self.cv2.circle(frame, thumb_point, 6, (0, 255, 255), -1)
                    self.cv2.circle(frame, index_point, 6, (0, 255, 255), -1)
                    wrist_px = hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.WRIST]
                    x_px = int(wrist_px.x * frame.shape[1])
                    y_px = int(wrist_px.y * frame.shape[0])

        if "right" in tracked:
            self._publish_pose(self.right_pub, tracked["right"]["pose"][0], tracked["right"]["pose"][1])
        if "left" in tracked:
            self._publish_pose(self.left_pub, tracked["left"]["pose"][0], tracked["left"]["pose"][1])

        right_pinch_msg = Bool()
        right_pinch_msg.data = bool(tracked.get("right", {}).get("pinch", False))
        self.right_pinch_pub.publish(right_pinch_msg)
        left_pinch_msg = Bool()
        left_pinch_msg.data = bool(tracked.get("left", {}).get("pinch", False))
        self.left_pinch_pub.publish(left_pinch_msg)
        right_pinky_msg = Bool()
        right_pinky_msg.data = bool(tracked.get("right", {}).get("pinky_pinch", False))
        self.right_pinky_pinch_pub.publish(right_pinky_msg)
        left_pinky_msg = Bool()
        left_pinky_msg.data = bool(tracked.get("left", {}).get("pinky_pinch", False))
        self.left_pinky_pinch_pub.publish(left_pinky_msg)
        right_fist_msg = Bool()
        right_fist_msg.data = bool(tracked.get("right", {}).get("fist", False))
        self.right_fist_pub.publish(right_fist_msg)
        left_fist_msg = Bool()
        left_fist_msg.data = bool(tracked.get("left", {}).get("fist", False))
        self.left_fist_pub.publish(left_fist_msg)
        right_gripper_aperture_msg = Float32()
        right_gripper_aperture_msg.data = float(tracked.get("right", {}).get("gripper_aperture", 0.0))
        self.right_gripper_aperture_pub.publish(right_gripper_aperture_msg)
        left_gripper_aperture_msg = Float32()
        left_gripper_aperture_msg.data = float(tracked.get("left", {}).get("gripper_aperture", 0.0))
        self.left_gripper_aperture_pub.publish(left_gripper_aperture_msg)

        debug_msg = String()
        debug_msg.data = " | ".join(debug_lines) if debug_lines else "no hands tracked"
        self.debug_pub.publish(debug_msg)

        if self._show_visualization:
            self._show_visualization_frame(frame)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WebcamHandTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
