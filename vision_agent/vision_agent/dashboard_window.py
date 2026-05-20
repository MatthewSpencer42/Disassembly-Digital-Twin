#!/usr/bin/env python3
import json
import os
import sys
import threading
import time
from io import BytesIO
from xml.etree import ElementTree

# OpenCV wheels can point Qt at cv2/qt/plugins, which often breaks PyQt's xcb
# loading on ROS workstations. Force the system Qt plugins before QApplication.
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"
os.environ.pop("QT_PLUGIN_PATH", None)

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from PIL import Image as PilImage
from PyQt5.QtCore import QCoreApplication, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGraphicsDropShadowEffect,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String


PREFERRED_WINDOW_WIDTH = 1816
PREFERRED_WINDOW_HEIGHT = 1000
CONTENT_MIN_HEIGHT = 980
WINDOW_MIN_WIDTH = 900
CAMERA_PANEL_MIN_HEIGHT = 300
STACK_PANEL_HEIGHT = 148
VISION_PANEL_HEIGHT = 126
ROBOT_PANEL_HEIGHT = 116
FORCE_PANEL_HEIGHT = 126
DROP_BIN_PANEL_HEIGHT = 96

DARK_THEME = {
    "root": "#141a22",
    "card": "#1b2430",
    "field": "#202a36",
    "shadow": "#06090d",
    "text": "#f7fbff",
    "muted": "#b5c0cd",
    "accent": "#4fd18b",
    "bad": "#ff5d5d",
    "image_bg": "#080c11",
    "button_bg": "#202b38",
    "button_border": "#303c4c",
}

LIGHT_THEME = {
    "root": "#dfe7f0",
    "card": "#e9f0f7",
    "field": "#f3f7fb",
    "shadow": "#aebccc",
    "text": "#101820",
    "muted": "#4f5d6b",
    "accent": "#0f9f5f",
    "bad": "#d92d20",
    "image_bg": "#d8e1eb",
    "button_bg": "#edf3fa",
    "button_border": "#ffffff",
}


class DashboardDataNode(Node):
    def __init__(self):
        super().__init__("vision_dashboard_window")
        self.lock = threading.Lock()

        self.global_frame = None
        self.local_frame = None
        self.global_state = {"objects": [], "bin_locations": {}}
        self.local_state = {"screws": [], "screw_heads": [], "tool_tips": [], "holes": [], "crosshair": []}
        self.assembly_state = {"state": "unknown", "confidence": 0.0}
        self.robot_states = {"tool_arm": "OFFLINE", "manip_arm": "OFFLINE"}
        self.robot_description = ""
        self.robot_description_last = 0.0
        self.wrench = None
        self.wrench_offset = None
        self.exotica_ready = False

        self.global_last = 0.0
        self.local_last = 0.0
        self.global_state_last = 0.0
        self.local_state_last = 0.0
        self.global_count = 0
        self.local_count = 0
        self.global_fps = 0.0
        self.local_fps = 0.0
        self.fps_time = time.time()

        ready_qos = QoSProfile(depth=1)
        ready_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        description_qos = QoSProfile(depth=1)
        description_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Image, "/camera/cropped/color/image_display", self._global_image_cb, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, "/tool_cam/image_raw/compressed", self._local_image_cb, 10)
        self.create_subscription(String, "/vision/global_state", self._global_state_cb, 10)
        self.create_subscription(String, "/vision/local_state", self._local_state_cb, 10)
        self.create_subscription(String, "/vision/assembly_state", self._assembly_state_cb, 10)
        self.create_subscription(String, "/robot_states", self._robot_states_cb, 10)
        self.create_subscription(String, "/robot_description", self._robot_description_cb, description_qos)
        self.create_subscription(WrenchStamped, "/robotiq_force_torque_sensor_broadcaster/wrench", self._wrench_cb, 10)
        self.create_subscription(Bool, "/exotica/ready", self._exotica_ready_cb, ready_qos)

    def snapshot(self):
        with self.lock:
            return {
                "global_frame": None if self.global_frame is None else self.global_frame.copy(),
                "local_frame": None if self.local_frame is None else self.local_frame.copy(),
                "global_state": dict(self.global_state),
                "local_state": dict(self.local_state),
                "assembly_state": dict(self.assembly_state),
                "robot_states": dict(self.robot_states),
                "robot_description": self.robot_description,
                "robot_description_last": self.robot_description_last,
                "wrench": None if self.wrench is None else dict(self.wrench),
                "exotica_ready": self.exotica_ready,
                "global_last": self.global_last,
                "local_last": self.local_last,
                "global_state_last": self.global_state_last,
                "local_state_last": self.local_state_last,
                "global_fps": self.global_fps,
                "local_fps": self.local_fps,
            }

    def update_fps(self):
        now = time.time()
        if now - self.fps_time < 1.0:
            return
        with self.lock:
            elapsed = max(now - self.fps_time, 1e-6)
            self.global_fps = self.global_count / elapsed
            self.local_fps = self.local_count / elapsed
            self.global_count = 0
            self.local_count = 0
            self.fps_time = now

    def _image_msg_to_bgr(self, msg):
        encoding = msg.encoding.lower()
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            if encoding in ("bgr8", "rgb8"):
                row_stride = int(msg.step)
                row_width = int(msg.width) * 3
                if row_stride < row_width:
                    return None
                frame = raw.reshape((int(msg.height), row_stride))[:, :row_width]
                frame = frame.reshape((int(msg.height), int(msg.width), 3))
                if encoding == "bgr8":
                    frame = frame[:, :, ::-1]
                return np.ascontiguousarray(frame)
            if encoding == "mono8":
                row_stride = int(msg.step)
                if row_stride < int(msg.width):
                    return None
                frame = raw.reshape((int(msg.height), row_stride))[:, : int(msg.width)]
                return np.ascontiguousarray(np.repeat(frame[:, :, None], 3, axis=2))
        except Exception:
            return None
        return None

    def _global_image_cb(self, msg):
        frame = self._image_msg_to_bgr(msg)
        if frame is None:
            return
        with self.lock:
            self.global_frame = frame
            self.global_last = time.time()
            self.global_count += 1

    def _local_image_cb(self, msg):
        try:
            frame = np.array(PilImage.open(BytesIO(bytes(msg.data))).convert("RGB"))
        except Exception:
            return
        with self.lock:
            self.local_frame = np.ascontiguousarray(frame)
            self.local_last = time.time()
            self.local_count += 1

    def _global_state_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.global_state = data
            self.global_state_last = time.time()

    def _local_state_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.local_state = data
            self.local_state_last = time.time()

    def _assembly_state_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.assembly_state = data

    def _robot_states_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.robot_states = data

    def _robot_description_cb(self, msg):
        with self.lock:
            self.robot_description = msg.data
            self.robot_description_last = time.time()

    def _exotica_ready_cb(self, msg):
        with self.lock:
            self.exotica_ready = bool(msg.data)

    def _wrench_cb(self, msg):
        values = (
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        )
        if self.wrench_offset is None:
            self.wrench_offset = values
        ox, oy, oz, otx, oty, otz = self.wrench_offset
        with self.lock:
            self.wrench = {
                "fx": values[0] - ox,
                "fy": values[1] - oy,
                "fz": values[2] - oz,
                "tx": values[3] - otx,
                "ty": values[4] - oty,
                "tz": values[5] - otz,
            }


class Card(QFrame):
    def __init__(self, title, theme):
        super().__init__()
        self.title = QLabel(title)
        self.title.setObjectName("cardTitle")
        self.content = QWidget()
        self.content.setObjectName("cardContent")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 14)
        layout.setSpacing(8)
        layout.addWidget(self.title)
        layout.addWidget(self.content, 1)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.shadow = QGraphicsDropShadowEffect(self)
        self.shadow.setBlurRadius(18)
        self.shadow.setOffset(7, 7)
        self.setGraphicsEffect(self.shadow)
        Card.apply_theme(self, theme)

    def apply_theme(self, theme):
        self.shadow.setColor(QColor(theme["shadow"]))
        self.setStyleSheet(f"""
            QFrame {{
                background: {theme["card"]};
                border: 0px;
                border-radius: 12px;
            }}
            QLabel#cardTitle {{
                color: {theme["text"]};
                background: transparent;
                font-size: 18px;
                font-weight: 700;
            }}
            QWidget#cardContent {{
                background: {theme["field"]};
                border: 0px;
                border-radius: 8px;
            }}
        """)


class VideoCard(Card):
    def __init__(self, title, theme, fill=False):
        super().__init__(title, theme)
        self.fill = fill
        self.image = QLabel("Waiting for image")
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumHeight(260)
        self.image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout = QVBoxLayout(self.content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.image)

    def apply_theme(self, theme):
        super().apply_theme(theme)
        self.image.setStyleSheet(f"background: {theme['image_bg']}; color: {theme['muted']}; border-radius: 6px;")

    def set_frame(self, frame, overlays, source_size, theme):
        self.image.setPixmap(render_frame(frame, self.image.size(), overlays, source_size, theme, fill=self.fill))


class StatusCard(Card):
    def __init__(self, theme):
        super().__init__("Stack Status", theme)
        self.rows = []
        grid = QGridLayout(self.content)
        grid.setContentsMargins(14, 10, 14, 10)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(4)
        for i, name in enumerate(("MoveIt", "EXOTica", "Global Vision", "Local Vision")):
            dot = QLabel()
            dot.setFixedSize(14, 14)
            label = QLabel(name)
            state = QLabel("WAITING")
            grid.addWidget(dot, i, 0, Qt.AlignCenter)
            grid.addWidget(label, i, 1)
            grid.addWidget(state, i, 2)
            grid.setColumnStretch(1, 1)
            self.rows.append((dot, label, state))
        self.apply_theme(theme)

    def apply_theme(self, theme):
        super().apply_theme(theme)
        for dot, label, state in self.rows:
            label.setStyleSheet(f"color: {theme['text']}; font-weight: 700; background: transparent;")
            state.setStyleSheet(f"color: {theme['bad']}; font-weight: 700; background: transparent;")
            dot.setStyleSheet(f"background: {theme['bad']}; border-radius: 7px;")

    def update_items(self, items, theme):
        for (dot, label, state), (name, ok) in zip(self.rows, items):
            color = theme["accent"] if ok else theme["bad"]
            label.setText(name)
            state.setText("ONLINE" if ok else "WAITING")
            state.setStyleSheet(f"color: {color}; font-weight: 700; background: transparent;")
            dot.setStyleSheet(f"background: {color}; border-radius: 7px;")


class TextCard(Card):
    def __init__(self, title, theme, fixed_height=None, scroll=True):
        super().__init__(title, theme)
        if fixed_height:
            self.setFixedHeight(fixed_height)
        self.text = QTextEdit()
        self.text.setReadOnly(True)
        self.text.setFrameShape(QFrame.NoFrame)
        self.text.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded if scroll else Qt.ScrollBarAlwaysOff)
        self.text.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        layout = QVBoxLayout(self.content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.text)
        self.apply_theme(theme)

    def apply_theme(self, theme):
        super().apply_theme(theme)
        self.text.setStyleSheet(f"""
            QTextEdit {{
                background: {theme["field"]};
                color: {theme["text"]};
                border: 0px;
                border-radius: 8px;
                padding: 8px;
                font-size: 15px;
            }}
            QScrollBar:vertical {{
                background: {theme["field"]};
                width: 10px;
                margin: 6px 0 6px 0;
                border-radius: 5px;
            }}
            QScrollBar::handle:vertical {{
                background: {theme["card"]};
                min-height: 30px;
                border-radius: 5px;
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
                height: 0px;
            }}
        """)

    def set_text(self, text):
        if self.text.toPlainText() == text:
            return
        bar = self.text.verticalScrollBar()
        value = bar.value()
        self.text.setPlainText(text)
        bar.setValue(min(value, bar.maximum()))


class DetectedPartsCard(Card):
    def __init__(self, theme):
        super().__init__("Detected Parts", theme)
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["ID", "Label", "Confidence"])
        self.table.verticalHeader().setVisible(False)
        self.table.setShowGrid(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        layout = QVBoxLayout(self.content)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.table)
        self.apply_theme(theme)

    def apply_theme(self, theme):
        super().apply_theme(theme)
        self.table.setStyleSheet(f"""
            QTableWidget {{
                background: {theme["field"]};
                color: {theme["text"]};
                border: 0px;
                border-radius: 8px;
                gridline-color: transparent;
                font-size: 15px;
            }}
            QHeaderView::section {{
                background: {theme["field"]};
                color: {theme["text"]};
                border: 0px;
                font-weight: 700;
                padding: 6px;
            }}
            QScrollBar:vertical {{
                background: {theme["field"]};
                width: 10px;
                margin: 6px 0 6px 0;
                border-radius: 5px;
            }}
            QScrollBar::handle:vertical {{
                background: {theme["card"]};
                min-height: 30px;
                border-radius: 5px;
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
                height: 0px;
            }}
        """)

    def update_rows(self, objects):
        objects = objects or []
        self.table.setRowCount(len(objects))
        for row, obj in enumerate(objects):
            conf = obj.get("confidence")
            values = [
                str(obj.get("id", "?")),
                str(obj.get("label", "part")),
                f"{float(conf):.2f}" if conf is not None else "-",
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                self.table.setItem(row, col, item)


class DashboardWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.theme_name = "dark"
        self.theme = DARK_THEME
        self.setWindowTitle("Disassembly Vision Dashboard")
        self.configure_window_size()

        self.title = QLabel("Disassembly Vision Dashboard")
        self.status = QLabel("Waiting for streams")
        self.theme_button = QPushButton("Light Theme")
        self.theme_button.clicked.connect(self.toggle_theme)

        self.global_video = VideoCard("Global Camera", self.theme, fill=False)
        self.global_video.setMinimumHeight(CAMERA_PANEL_MIN_HEIGHT)
        self.local_video = VideoCard("Tool Camera", self.theme, fill=False)
        self.local_video.setMinimumHeight(CAMERA_PANEL_MIN_HEIGHT)
        self.stack = StatusCard(self.theme)
        self.stack.setFixedHeight(STACK_PANEL_HEIGHT)
        self.detected = DetectedPartsCard(self.theme)
        self.detected.setMinimumHeight(160)
        self.detected.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.vision = TextCard("Vision State", self.theme, fixed_height=VISION_PANEL_HEIGHT, scroll=False)
        self.robot = TextCard("Robot State", self.theme, fixed_height=ROBOT_PANEL_HEIGHT, scroll=False)
        self.force = TextCard("Force/Torque", self.theme, fixed_height=FORCE_PANEL_HEIGHT, scroll=False)
        self.bin = TextCard("Drop Bin", self.theme, fixed_height=DROP_BIN_PANEL_HEIGHT, scroll=False)
        self.cards = [
            self.global_video,
            self.local_video,
            self.stack,
            self.detected,
            self.vision,
            self.robot,
            self.force,
            self.bin,
        ]

        root = QWidget()
        root.setMinimumHeight(CONTENT_MIN_HEIGHT)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(24, 16, 24, 20)
        root_layout.setSpacing(14)

        header = QHBoxLayout()
        header.addWidget(self.title, 1)
        header.addWidget(self.status)
        header.addWidget(self.theme_button)
        root_layout.addLayout(header)

        main = QHBoxLayout()
        main.setSpacing(18)

        camera_column = QVBoxLayout()
        camera_column.setSpacing(14)
        camera_column.addWidget(self.global_video, 1)
        camera_column.addWidget(self.local_video, 1)
        camera_widget = QWidget()
        camera_widget.setLayout(camera_column)
        camera_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        side = QVBoxLayout()
        side.setSpacing(14)
        side.addWidget(self.stack)
        side.addWidget(self.detected, 1)
        side.addWidget(self.vision)
        side.addWidget(self.robot)
        side.addWidget(self.force)
        side.addWidget(self.bin)
        side_widget = QWidget()
        side_widget.setLayout(side)
        side_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        main.addWidget(camera_widget, 1)
        main.addWidget(side_widget, 1)
        root_layout.addLayout(main, 1)

        self.setCentralWidget(root)
        self.apply_theme()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def configure_window_size(self):
        screen = QApplication.primaryScreen()
        if screen is None:
            self.resize(1400, 1000)
            self.setMinimumSize(WINDOW_MIN_WIDTH, CONTENT_MIN_HEIGHT)
            return

        available = screen.availableGeometry()
        preferred_w = min(PREFERRED_WINDOW_WIDTH, int(available.width() * 0.96))
        preferred_h = min(PREFERRED_WINDOW_HEIGHT, int(available.height() * 0.96))
        minimum_w = min(preferred_w, max(WINDOW_MIN_WIDTH, int(available.width() * 0.35)))
        minimum_h = min(preferred_h, max(CONTENT_MIN_HEIGHT, int(available.height() * 0.88)))
        self.resize(max(WINDOW_MIN_WIDTH, preferred_w), max(CONTENT_MIN_HEIGHT, preferred_h))
        self.setMinimumSize(minimum_w, minimum_h)

    def apply_theme(self):
        t = self.theme
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background: {t["root"]};
                color: {t["text"]};
                font-family: DejaVu Sans;
            }}
            QLabel {{
                background: transparent;
            }}
            QPushButton {{
                background: {t["button_bg"]};
                color: {t["text"]};
                border: 1px solid {t["button_border"]};
                border-radius: 0px;
                padding: 10px 18px;
                font-weight: 700;
            }}
            QScrollBar:vertical {{
                background: {t["root"]};
                width: 10px;
                margin: 6px 0 6px 0;
                border-radius: 5px;
            }}
            QScrollBar::handle:vertical {{
                background: {t["card"]};
                min-height: 34px;
                border-radius: 5px;
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
                height: 0px;
            }}
        """)
        self.title.setStyleSheet("font-size: 28px; font-weight: 800;")
        self.status.setStyleSheet(f"color: {t['muted']}; font-size: 15px; font-weight: 700;")
        for card in self.cards:
            card.apply_theme(t)
        self.theme_button.setText("Light Theme" if self.theme_name == "dark" else "Dark Theme")

    def toggle_theme(self):
        self.theme_name = "light" if self.theme_name == "dark" else "dark"
        self.theme = LIGHT_THEME if self.theme_name == "light" else DARK_THEME
        self.apply_theme()

    def stack_items(self, snap, now):
        try:
            node_names = set(self.node.get_node_names())
        except Exception:
            node_names = set()
        try:
            planning_scene_publishers = self.node.count_publishers("/monitored_planning_scene")
        except Exception:
            planning_scene_publishers = 0
        moveit_ok = "move_group" in node_names or planning_scene_publishers > 0
        global_ok = (now - snap["global_state_last"] < 5.0) and (now - snap["global_last"] < 5.0)
        local_ok = (now - snap["local_state_last"] < 5.0) and (now - snap["local_last"] < 5.0)
        return [
            ("MoveIt", moveit_ok),
            ("EXOTica", snap["exotica_ready"]),
            ("Global Vision", global_ok),
            ("Local Vision", local_ok),
        ]

    def tick(self):
        self.node.update_fps()
        snap = self.node.snapshot()
        now = time.time()

        self.status.setText(f"Global {snap['global_fps']:.1f} fps | Local {snap['local_fps']:.1f} fps")
        self.stack.update_items(self.stack_items(snap, now), self.theme)
        self.detected.update_rows(snap["global_state"].get("objects", []))
        self.vision.set_text(
            f"{str(snap['assembly_state'].get('state', 'unknown')).upper()}\n"
            f"Confidence: {float(snap['assembly_state'].get('confidence', 0.0)):.2f}"
        )
        robot = snap["robot_states"]
        self.robot.set_text(
            f"Tool arm:  {robot.get('tool_arm', 'OFFLINE')}\n"
            f"Manip arm: {robot.get('manip_arm', 'OFFLINE')}"
        )
        wrench = snap["wrench"]
        if wrench:
            self.force.set_text(
                f"Fx {wrench['fx']:>7.2f} N   Tx {wrench['tx']:>7.3f} Nm\n"
                f"Fy {wrench['fy']:>7.2f} N   Ty {wrench['ty']:>7.3f} Nm\n"
                f"Fz {wrench['fz']:>7.2f} N   Tz {wrench['tz']:>7.3f} Nm"
            )
        else:
            self.force.set_text("Waiting for FT300 wrench")
        xyz = (snap["global_state"].get("bin_locations", {}).get("bin_1", {}) or {}).get("xyz")
        if xyz:
            self.bin.set_text(f"BIN 1   X {xyz[0]:.3f}m   Y {xyz[1]:.3f}m   Z {xyz[2]:.3f}m")
        else:
            self.bin.set_text("BIN 1 waiting")

        self.global_video.set_frame(
            snap["global_frame"],
            global_overlays(snap["global_state"], self.theme),
            snap["global_state"].get("image_size"),
            self.theme,
        )
        self.local_video.set_frame(
            snap["local_frame"],
            local_overlays(snap["local_state"]),
            snap["local_state"].get("image_size"),
            self.theme,
        )


def global_overlays(state, theme):
    overlays = []
    for obj in state.get("objects", []):
        label = f"{obj.get('id', '?')} {obj.get('label', 'part')}"
        conf = obj.get("confidence")
        if conf is not None:
            label += f" {float(conf):.2f}"
        overlays.append({"box": obj.get("box"), "label": label, "color": theme["accent"]})
    return overlays


def local_overlays(state):
    overlays = []
    for key, label, color in (
        ("screws", "screw", "#ffb020"),
        ("screw_heads", "head", "#35d0ff"),
        ("tool_tips", "tool", "#d95cff"),
        ("holes", "hole", "#ff5d5d"),
    ):
        for item in state.get(key, []):
            box = item.get("box")
            point = item.get("centroid") or item.get("center") or item.get("contact_point")
            if (not point or len(point) != 2) and box and len(box) == 4:
                point = [int((float(box[0]) + float(box[2])) / 2), int((float(box[1]) + float(box[3])) / 2)]
            overlay_label = label
            conf = item.get("confidence", item.get("conf"))
            if conf is not None:
                overlay_label += f" {float(conf):.2f}"
            if point and len(point) == 2:
                overlay_label += f" ({int(point[0])},{int(point[1])})"
            overlays.append({"box": box, "label": overlay_label, "point": point, "color": color})
    crosshair = state.get("crosshair")
    if crosshair and len(crosshair) == 2:
        cfg = state.get("crosshair_config") or {}
        overlays.append({
            "kind": "crosshair",
            "point": crosshair,
            "color": "#48ff70",
            "radius": int(cfg.get("arm_px", 20)),
            "thickness": int(cfg.get("thickness", 1)),
        })
    return overlays


def robot_model_summary(robot_description):
    if not robot_description:
        return "Waiting for /robot_description"
    try:
        root = ElementTree.fromstring(robot_description)
    except Exception:
        return "URDF received\nParse error"

    links = [link.get("name", "") for link in root.findall("link")]
    joints = root.findall("joint")
    child_links = set()
    for joint in joints:
        child = joint.find("child")
        if child is not None and child.get("link"):
            child_links.add(child.get("link"))
    root_links = [name for name in links if name and name not in child_links]
    root_link = root_links[0] if root_links else "unknown"
    return (
        f"{root.get('name', 'robot')}\n"
        f"Links {len(links)}   Joints {len(joints)}\n"
        f"Root {root_link}"
    )


def render_frame(frame, size, overlays, source_size, theme, fill=False):
    width = max(1, size.width())
    height = max(1, size.height())
    pixmap = QPixmap(width, height)
    pixmap.fill(QColor(theme["image_bg"]))
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.Antialiasing, True)

    if frame is None:
        painter.setPen(QColor(theme["muted"]))
        painter.drawText(pixmap.rect(), Qt.AlignCenter, "Waiting for image")
        painter.end()
        return pixmap

    h, w = frame.shape[:2]
    if fill:
        view_w = width
        view_h = height
        vx = 0
        vy = 0
        scale = max(view_w / float(w), view_h / float(h))
    else:
        view_w = width
        view_h = height
        vx = 0
        vy = 0
        scale = min(view_w / float(w), view_h / float(h))

    out_w = max(1, int(w * scale))
    out_h = max(1, int(h * scale))
    rgb = np.ascontiguousarray(frame)
    source_image = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
    image = QPixmap.fromImage(source_image).scaled(out_w, out_h, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
    ox = vx + (view_w - out_w) // 2
    oy = vy + (view_h - out_h) // 2
    painter.drawPixmap(ox, oy, image)

    if source_size and len(source_size) == 2 and source_size[0] > 0 and source_size[1] > 0:
        sx = out_w / float(source_size[0])
        sy = out_h / float(source_size[1])
    else:
        sx = out_w / float(w)
        sy = out_h / float(h)

    for item in overlays or []:
        if item.get("kind") == "crosshair":
            point = item.get("point")
            if not point or len(point) != 2:
                continue
            cx = int(ox + float(point[0]) * sx)
            cy = int(oy + float(point[1]) * sy)
            radius = int(item.get("radius", 20))
            thick = int(item.get("thickness", 1))
            painter.setPen(QPen(QColor("#05070a"), thick + 1))
            painter.drawLine(cx - radius, cy, cx + radius, cy)
            painter.drawLine(cx, cy - radius, cx, cy + radius)
            painter.setPen(QPen(QColor(item.get("color", "#48ff70")), thick))
            painter.drawLine(cx - radius, cy, cx + radius, cy)
            painter.drawLine(cx, cy - radius, cx, cy + radius)
            painter.setBrush(QColor("#ff355d"))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(cx - 2, cy - 2, 4, 4)
            continue

        box = item.get("box")
        if not box:
            continue
        x1, y1, x2, y2 = [int(v) for v in box]
        x1 = int(ox + x1 * sx)
        x2 = int(ox + x2 * sx)
        y1 = int(oy + y1 * sy)
        y2 = int(oy + y2 * sy)
        color = QColor(item.get("color", theme["accent"]))
        painter.setPen(QPen(color, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(x1, y1, x2 - x1, y2 - y1)
        point = item.get("point")
        if point and len(point) == 2:
            px = int(ox + float(point[0]) * sx)
            py = int(oy + float(point[1]) * sy)
            painter.setBrush(color)
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(px - 4, py - 4, 8, 8)
        label = item.get("label")
        if label:
            painter.setPen(color)
            painter.setFont(QFont("DejaVu Sans", 10, QFont.Bold))
            painter.drawText(x1 + 4, max(14, y1 - 6), label)

    painter.end()
    return pixmap


def main(args=None):
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    rclpy.init(args=args)
    node = DashboardDataNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    QCoreApplication.setLibraryPaths(["/usr/lib/x86_64-linux-gnu/qt5/plugins"])
    app = QApplication(sys.argv)
    window = DashboardWindow(node)
    window.show()
    try:
        return app.exec_()
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
