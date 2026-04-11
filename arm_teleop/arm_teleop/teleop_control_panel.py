"""
Teleop Control Panel — Tkinter GUI node for arm teleop control.

Buttons:
  Right Arm        — toggles the right-hand assigned arm teleop enable state
  Left Arm         — toggles the left-hand assigned arm teleop enable state
  Both Arms        — toggles all configured arms together
  Go Home & Reset  — disables teleop, moves arms to home, clears calibration
  Recalibrate Only — clears calibration so next hand detection re-calibrates

Status indicators reflect /teleop_status/{right,left}_arm_enabled topics.
"""
from __future__ import annotations

import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class TeleopControlPanelNode(Node):
    def __init__(self):
        super().__init__("teleop_control_panel")
        self._status: dict[str, bool] = {"right": False, "left": False}
        self._hand_pose_seen: dict[str, float] = {"right": 0.0, "left": 0.0}
        self._hand_pose_timeout_sec = 0.5
        self._exotica_ready = False
        self._exotica_server_seen = False
        self._on_update: callable = None  # set by TeleopControlPanel after construction

        self.create_subscription(
            Bool, "/teleop_status/right_arm_enabled",
            lambda msg: self._status_cb("right", msg), 10,
        )
        self.create_subscription(
            Bool, "/teleop_status/left_arm_enabled",
            lambda msg: self._status_cb("left", msg), 10,
        )
        self.create_subscription(
            PoseStamped, "/teleop_hand_tracking/right/wrist",
            lambda msg: self._hand_pose_cb("right", msg), 10,
        )
        self.create_subscription(
            PoseStamped, "/teleop_hand_tracking/left/wrist",
            lambda msg: self._hand_pose_cb("left", msg), 10,
        )
        ready_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/exotica/ready", self._exotica_ready_cb, ready_qos)
        self.create_timer(0.5, self._poll_exotica_server)
        self.create_timer(0.2, self._refresh_ui_tick)
        self._go_home_cli = self.create_client(Trigger, "/exotica_arm_teleop/go_home")
        self._recalibrate_cli = self.create_client(Trigger, "/exotica_arm_teleop/recalibrate")
        self._right_enable_cli = self.create_client(SetBool, "/exotica_arm_teleop/set_right_arm_enabled")
        self._left_enable_cli = self.create_client(SetBool, "/exotica_arm_teleop/set_left_arm_enabled")
        self._all_enable_cli = self.create_client(SetBool, "/exotica_arm_teleop/set_all_arms_enabled")

    def _status_cb(self, hand: str, msg: Bool):
        self._status[hand] = bool(msg.data)
        if self._on_update is not None:
            self._on_update()

    def _hand_pose_cb(self, hand: str, _msg: PoseStamped):
        self._hand_pose_seen[hand] = self.get_clock().now().nanoseconds / 1e9
        if self._on_update is not None:
            self._on_update()

    def _exotica_ready_cb(self, msg: Bool):
        self._exotica_server_seen = True
        self._exotica_ready = bool(msg.data)
        if self._on_update is not None:
            self._on_update()

    def _poll_exotica_server(self):
        seen = self.count_publishers("/exotica/ready") > 0
        if seen != self._exotica_server_seen:
            self._exotica_server_seen = seen
            if not seen:
                self._exotica_ready = False
            if self._on_update is not None:
                self._on_update()

    def _refresh_ui_tick(self):
        if self._on_update is not None:
            self._on_update()

    def call_go_home(self, on_done=None):
        self._call_trigger(self._go_home_cli, "go_home", on_done)

    def call_recalibrate(self, on_done=None):
        self._call_trigger(self._recalibrate_cli, "recalibrate", on_done)

    def call_set_right_enabled(self, enabled: bool, on_done=None):
        self._call_set_bool(self._right_enable_cli, "set_right_arm_enabled", enabled, on_done)

    def call_set_left_enabled(self, enabled: bool, on_done=None):
        self._call_set_bool(self._left_enable_cli, "set_left_arm_enabled", enabled, on_done)

    def call_set_all_enabled(self, enabled: bool, on_done=None):
        self._call_set_bool(self._all_enable_cli, "set_all_arms_enabled", enabled, on_done)

    def _call_trigger(self, client, name: str, on_done=None):
        if not client.service_is_ready():
            msg = f"/{name} service not available (teleop node not running?)"
            self.get_logger().warning(msg)
            if on_done:
                on_done(False, msg)
            return
        future = client.call_async(Trigger.Request())

        def _cb(f):
            try:
                res = f.result()
                if on_done:
                    on_done(res.success, res.message)
            except Exception as exc:
                if on_done:
                    on_done(False, str(exc))

        future.add_done_callback(_cb)

    def _call_set_bool(self, client, name: str, value: bool, on_done=None):
        if not client.service_is_ready():
            msg = f"/{name} service not available (teleop node not running?)"
            self.get_logger().warning(msg)
            if on_done:
                on_done(False, msg)
            return
        request = SetBool.Request()
        request.data = bool(value)
        future = client.call_async(request)

        def _cb(f):
            try:
                res = f.result()
                if on_done:
                    on_done(res.success, res.message)
            except Exception as exc:
                if on_done:
                    on_done(False, str(exc))

        future.add_done_callback(_cb)


class TeleopControlPanel:
    _BG = "#2b2b2b"
    _FG = "#cccccc"
    _ENABLED_COLOR = "#27ae60"
    _DISABLED_COLOR = "#555555"
    _READY_COLOR = "#1f8b4c"
    _STARTING_COLOR = "#d68910"
    _OFFLINE_COLOR = "#7f8c8d"
    _DETECTED_COLOR = "#1f8b4c"
    _NO_POSE_COLOR = "#555555"
    _BTN_HOME_COLOR = "#c0392b"
    _BTN_RECAL_COLOR = "#2980b9"
    _BTN_ENABLE_COLOR = "#1f8b4c"
    _BTN_DISABLE_COLOR = "#8e44ad"
    _BTN_ALL_COLOR = "#d35400"

    def __init__(self, node: TeleopControlPanelNode):
        self._node = node
        self._root = tk.Tk()
        self._root.title("Teleop Control Panel")
        self._root.resizable(False, False)
        self._root.configure(bg=self._BG)
        self._fixed_size: tuple[int, int] | None = None

        # Wire node status updates → UI refresh (thread-safe via root.after)
        node._on_update = lambda: self._root.after(0, self._refresh_status)

        self._build_ui()
        self._freeze_window_size()
        self._refresh_status()
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        root = self._root
        pad = {"padx": 14, "pady": 6}

        title_font = tkfont.Font(family="Helvetica", size=14, weight="bold")
        tk.Label(
            root, text="Teleop Control Panel",
            font=title_font, bg=self._BG, fg="#ffffff",
        ).pack(pady=(14, 4))

        # ---- Arm status indicators ----
        status_frame = tk.LabelFrame(
            root, text="Arm Status",
            bg=self._BG, fg="#aaaaaa", padx=10, pady=8,
        )
        status_frame.pack(fill="x", **pad)

        self._status_labels: dict[str, tk.Label] = {}
        for hand in ("right", "left"):
            row = tk.Frame(status_frame, bg=self._BG)
            row.pack(fill="x", pady=3)
            tk.Label(
                row, text=f"{hand.capitalize()} arm:",
                width=12, anchor="w", bg=self._BG, fg=self._FG,
            ).pack(side="left")
            lbl = tk.Label(
                row, text="DISABLED", width=10,
                bg=self._DISABLED_COLOR, fg="#ffffff",
                relief="raised", padx=6, pady=3,
            )
            lbl.pack(side="left")
            self._status_labels[hand] = lbl

        hand_frame = tk.LabelFrame(
            root, text="Hand Tracking",
            bg=self._BG, fg="#aaaaaa", padx=10, pady=8,
        )
        hand_frame.pack(fill="x", **pad)

        self._hand_labels: dict[str, tk.Label] = {}
        for hand in ("right", "left"):
            row = tk.Frame(hand_frame, bg=self._BG)
            row.pack(fill="x", pady=3)
            tk.Label(
                row, text=f"{hand.capitalize()} hand:",
                width=12, anchor="w", bg=self._BG, fg=self._FG,
            ).pack(side="left")
            lbl = tk.Label(
                row, text="NO POSE", width=10,
                bg=self._NO_POSE_COLOR, fg="#ffffff",
                relief="raised", padx=6, pady=3,
            )
            lbl.pack(side="left")
            self._hand_labels[hand] = lbl

        exotica_frame = tk.LabelFrame(
            root, text="EXOTica IK Status",
            bg=self._BG, fg="#aaaaaa", padx=10, pady=8,
        )
        exotica_frame.pack(fill="x", **pad)

        self._exotica_labels: dict[str, tk.Label] = {}
        for key, label in (("uf850", "UF850 IK:"), ("xarm5", "xArm5 IK:")):
            row = tk.Frame(exotica_frame, bg=self._BG)
            row.pack(fill="x", pady=3)
            tk.Label(
                row, text=label,
                width=12, anchor="w", bg=self._BG, fg=self._FG,
            ).pack(side="left")
            lbl = tk.Label(
                row, text="OFFLINE", width=10,
                bg=self._OFFLINE_COLOR, fg="#ffffff",
                relief="raised", padx=6, pady=3,
            )
            lbl.pack(side="left")
            self._exotica_labels[key] = lbl

        control_frame = tk.LabelFrame(
            root, text="Arm Controls",
            bg=self._BG, fg="#aaaaaa", padx=10, pady=8,
        )
        control_frame.pack(fill="x", **pad)

        toggle_font = tkfont.Font(family="Helvetica", size=12, weight="bold")
        self._toggle_buttons: dict[str, tk.Button] = {}
        for key, label in (("right", "Right Arm"), ("left", "Left Arm"), ("all", "Both Arms")):
            btn = tk.Button(
                control_frame,
                text=label,
                font=toggle_font,
                fg="#ffffff",
                relief="raised",
                pady=12,
                command=lambda button_key=key: self._on_toggle(button_key),
            )
            btn.pack(fill="x", pady=4)
            self._toggle_buttons[key] = btn

        # ---- Action buttons ----
        btn_frame = tk.Frame(root, bg=self._BG)
        btn_frame.pack(fill="x", **pad)

        home_font = tkfont.Font(family="Helvetica", size=12, weight="bold")
        tk.Button(
            btn_frame,
            text="Go Home & Reset",
            font=home_font,
            bg=self._BTN_HOME_COLOR, fg="#ffffff",
            activebackground="#e74c3c",
            relief="raised", pady=10,
            command=self._on_go_home,
        ).pack(fill="x", pady=(0, 6))

        recal_font = tkfont.Font(family="Helvetica", size=11)
        tk.Button(
            btn_frame,
            text="Recalibrate Only",
            font=recal_font,
            bg=self._BTN_RECAL_COLOR, fg="#ffffff",
            activebackground="#3498db",
            relief="raised", pady=7,
            command=self._on_recalibrate,
        ).pack(fill="x")

        # ---- Status bar ----
        self._status_bar = tk.Label(
            root, text="Ready.",
            anchor="w", bg="#1e1e1e", fg="#888888", padx=8, pady=5,
        )
        self._status_bar.pack(fill="x", side="bottom")

    # ------------------------------------------------------------------
    # UI update helpers
    # ------------------------------------------------------------------

    def _refresh_status(self):
        for hand, lbl in self._status_labels.items():
            if self._node._status.get(hand, False):
                lbl.config(text="ENABLED", bg=self._ENABLED_COLOR)
            else:
                lbl.config(text="DISABLED", bg=self._DISABLED_COLOR)
        now = self._node.get_clock().now().nanoseconds / 1e9
        for hand, lbl in self._hand_labels.items():
            detected = (now - self._node._hand_pose_seen[hand]) <= self._node._hand_pose_timeout_sec
            if detected:
                lbl.config(text="DETECTED", bg=self._DETECTED_COLOR)
            else:
                lbl.config(text="NO POSE", bg=self._NO_POSE_COLOR)
        if self._node._exotica_ready:
            exotica_text = "READY"
            exotica_color = self._READY_COLOR
        elif self._node._exotica_server_seen:
            exotica_text = "STARTING"
            exotica_color = self._STARTING_COLOR
        else:
            exotica_text = "OFFLINE"
            exotica_color = self._OFFLINE_COLOR
        for lbl in self._exotica_labels.values():
            lbl.config(text=exotica_text, bg=exotica_color)
        self._refresh_toggle_buttons()

    def _refresh_toggle_buttons(self):
        right_enabled = self._node._status.get("right", False)
        left_enabled = self._node._status.get("left", False)
        if "right" in self._toggle_buttons:
            self._configure_toggle_button(self._toggle_buttons["right"], "Right Arm", right_enabled)
        if "left" in self._toggle_buttons:
            self._configure_toggle_button(self._toggle_buttons["left"], "Left Arm", left_enabled)
        if "all" in self._toggle_buttons:
            all_enabled = right_enabled and left_enabled
            any_enabled = right_enabled or left_enabled
            if all_enabled:
                self._toggle_buttons["all"].config(
                    text="Disable Both Arms",
                    bg=self._BTN_DISABLE_COLOR,
                    activebackground="#9b59b6",
                )
            elif any_enabled:
                self._toggle_buttons["all"].config(
                    text="Enable Both Arms",
                    bg=self._BTN_ALL_COLOR,
                    activebackground="#e67e22",
                )
            else:
                self._toggle_buttons["all"].config(
                    text="Enable Both Arms",
                    bg=self._BTN_ALL_COLOR,
                    activebackground="#e67e22",
                )

    def _configure_toggle_button(self, button: tk.Button, label: str, enabled: bool):
        if enabled:
            button.config(
                text=f"Disable {label}",
                bg=self._BTN_DISABLE_COLOR,
                activebackground="#9b59b6",
            )
        else:
            button.config(
                text=f"Enable {label}",
                bg=self._BTN_ENABLE_COLOR,
                activebackground="#27ae60",
            )

    def _set_status(self, text: str):
        self._status_bar.config(text=text)

    def _freeze_window_size(self):
        self._root.update_idletasks()
        width = self._root.winfo_reqwidth()
        height = self._root.winfo_reqheight()
        self._fixed_size = (width, height)
        self._root.geometry(f"{width}x{height}")
        self._root.minsize(width, height)
        self._root.maxsize(width, height)

    # ------------------------------------------------------------------
    # Button callbacks
    # ------------------------------------------------------------------

    def _on_go_home(self):
        self._set_status("Sending Go Home command...")
        self._node.call_go_home(
            on_done=lambda ok, msg: self._root.after(
                0, lambda: self._set_status(f"{'OK' if ok else 'FAILED'}: {msg}")
            )
        )

    def _on_recalibrate(self):
        self._set_status("Sending Recalibrate command...")
        self._node.call_recalibrate(
            on_done=lambda ok, msg: self._root.after(
                0, lambda: self._set_status(f"{'OK' if ok else 'FAILED'}: {msg}")
            )
        )

    def _on_toggle(self, which: str):
        if which == "right":
            desired = not self._node._status.get("right", False)
            action = "Enabling" if desired else "Disabling"
            self._set_status(f"{action} right arm...")
            self._node.call_set_right_enabled(
                desired,
                on_done=lambda ok, msg: self._root.after(
                    0, lambda: self._set_status(f"{'OK' if ok else 'FAILED'}: {msg}")
                ),
            )
            return
        if which == "left":
            desired = not self._node._status.get("left", False)
            action = "Enabling" if desired else "Disabling"
            self._set_status(f"{action} left arm...")
            self._node.call_set_left_enabled(
                desired,
                on_done=lambda ok, msg: self._root.after(
                    0, lambda: self._set_status(f"{'OK' if ok else 'FAILED'}: {msg}")
                ),
            )
            return
        desired = not (self._node._status.get("right", False) and self._node._status.get("left", False))
        action = "Enabling" if desired else "Disabling"
        self._set_status(f"{action} both arms...")
        self._node.call_set_all_enabled(
            desired,
            on_done=lambda ok, msg: self._root.after(
                0, lambda: self._set_status(f"{'OK' if ok else 'FAILED'}: {msg}")
            ),
        )

    def _on_close(self):
        self._root.destroy()

    # ------------------------------------------------------------------

    def run(self):
        self._root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopControlPanelNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        panel = TeleopControlPanel(node)
        panel.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
