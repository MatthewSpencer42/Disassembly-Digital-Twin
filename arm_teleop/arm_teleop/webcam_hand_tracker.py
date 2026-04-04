from __future__ import annotations

import os
import tempfile
import time
import warnings
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from arm_teleop.hand_math import cross, matrix_to_quat, normalize


class WebcamHandTracker(Node):
    def __init__(self):
        super().__init__("webcam_hand_tracker")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("show_visualization", True)
        self.declare_parameter("mirror_view", True)
        self.declare_parameter("max_num_hands", 2)
        self.declare_parameter("min_detection_confidence", 0.6)
        self.declare_parameter("min_tracking_confidence", 0.5)
        self.declare_parameter("frame_id", "webcam_hand_tracking")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("pinch_on_threshold", 0.055)
        self.declare_parameter("pinch_off_threshold", 0.075)

        self.right_pub = self.create_publisher(PoseStamped, "/teleop_hand_tracking/right/wrist", 10)
        self.left_pub = self.create_publisher(PoseStamped, "/teleop_hand_tracking/left/wrist", 10)
        self.right_pinch_pub = self.create_publisher(Bool, "/teleop_hand_tracking/right/pinch", 10)
        self.left_pinch_pub = self.create_publisher(Bool, "/teleop_hand_tracking/left/pinch", 10)
        self.debug_pub = self.create_publisher(String, "/teleop_hand_tracking/debug", 10)
        self.create_subscription(Bool, "/teleop_status/right_arm_enabled", self._right_arm_enabled_cb, 10)
        self.create_subscription(Bool, "/teleop_status/left_arm_enabled", self._left_arm_enabled_cb, 10)

        self._frame_id = self.get_parameter("frame_id").value
        self._show_visualization = bool(self.get_parameter("show_visualization").value)
        self._mirror_view = bool(self.get_parameter("mirror_view").value)
        self._period = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._pinch_on_threshold = float(self.get_parameter("pinch_on_threshold").value)
        self._pinch_off_threshold = float(self.get_parameter("pinch_off_threshold").value)
        self._pinch_state = {"left": False, "right": False}
        self._arm_enabled = {"right": False, "left": False}

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
        qt_font_dir = Path(self.cv2.__file__).resolve().parent / "qt" / "fonts"
        if qt_font_dir.is_dir() and "QT_QPA_FONTDIR" not in os.environ:
            os.environ["QT_QPA_FONTDIR"] = str(qt_font_dir)

        try:
            import mediapipe as mp
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "MediaPipe is required for webcam hand tracking. Install the Python package 'mediapipe'."
            ) from exc
        self.mp = mp

        camera_index = int(self.get_parameter("camera_index").value)
        self.cap = self.cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Unable to open webcam index {camera_index}.")

        self.hands = self.mp.solutions.hands.Hands(
            static_image_mode=False,
            model_complexity=1,
            max_num_hands=int(self.get_parameter("max_num_hands").value),
            min_detection_confidence=float(self.get_parameter("min_detection_confidence").value),
            min_tracking_confidence=float(self.get_parameter("min_tracking_confidence").value),
        )
        self.drawer = self.mp.solutions.drawing_utils
        self.hand_styles = self.mp.solutions.drawing_styles

        self.timer = self.create_timer(self._period, self._tick)
        self.get_logger().info("Webcam hand tracker started. Topics: /teleop_hand_tracking/{left,right}/wrist")

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        try:
            if self._show_visualization:
                self.cv2.destroyAllWindows()
        except Exception:
            pass
        return super().destroy_node()

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
        palm_center = [
            (wrist[0] + index_mcp[0] + middle_mcp[0] + pinky_mcp[0]) / 4.0,
            (wrist[1] + index_mcp[1] + middle_mcp[1] + pinky_mcp[1]) / 4.0,
            (wrist[2] + index_mcp[2] + middle_mcp[2] + pinky_mcp[2]) / 4.0,
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
                tracked[label] = {
                    "pose": pose,
                    "pinch": pinch,
                    "pinch_distance": pinch_distance,
                }
                wrist, quat = pose
                debug_lines.append(
                    f"{label}: pos=({wrist[0]:+.3f},{wrist[1]:+.3f},{wrist[2]:+.3f}) "
                    f"pinch={'on' if pinch else 'off'} d={pinch_distance:.3f} "
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
                    wrist_px = hand_landmarks.landmark[self.mp.solutions.hands.HandLandmark.WRIST]
                    x_px = int(wrist_px.x * frame.shape[1])
                    y_px = int(wrist_px.y * frame.shape[0])
                    self.cv2.putText(
                        frame,
                        f"{label.upper()} {'PINCH' if pinch else ''}",
                        (x_px + 10, y_px - 10),
                        self.cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0) if label == "right" else (255, 180, 0),
                        2,
                    )

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

        debug_msg = String()
        debug_msg.data = " | ".join(debug_lines) if debug_lines else "no hands tracked"
        self.debug_pub.publish(debug_msg)

        if self._show_visualization:
            status = f"L:{'yes' if 'left' in tracked else 'no'} R:{'yes' if 'right' in tracked else 'no'}"
            self.cv2.putText(frame, "Webcam Hand Tracking", (20, 30), self.cv2.FONT_HERSHEY_SIMPLEX, 0.9, (40, 220, 40), 2)
            self.cv2.putText(frame, status, (20, 60), self.cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 220, 220), 2)
            self.cv2.putText(
                frame,
                f"UF850: {'ENABLED' if self._arm_enabled['right'] else 'DISABLED'}",
                (20, 90),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if self._arm_enabled["right"] else (0, 140, 255),
                2,
            )
            self.cv2.putText(
                frame,
                f"XARM5: {'ENABLED' if self._arm_enabled['left'] else 'DISABLED'}",
                (20, 120),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if self._arm_enabled["left"] else (0, 140, 255),
                2,
            )
            self.cv2.imshow("Webcam Hand Tracking", frame)
            key = self.cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                self.get_logger().info("Visualization window requested shutdown.")
                time.sleep(0.1)
                if rclpy.ok():
                    rclpy.shutdown()


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
