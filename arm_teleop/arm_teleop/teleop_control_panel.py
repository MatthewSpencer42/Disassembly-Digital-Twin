"""
Teleop Control Panel — Tkinter GUI node for arm teleop control.

Buttons:
  Go Home & Reset  — disables teleop, moves arms to home, clears calibration
  Recalibrate Only — clears calibration so next hand detection re-calibrates

Status indicators reflect /teleop_status/{right,left}_arm_enabled topics.
"""
from __future__ import annotations

import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class TeleopControlPanelNode(Node):
    def __init__(self):
        super().__init__("teleop_control_panel")
        self._status: dict[str, bool] = {"right": False, "left": False}
        self._on_update: callable = None  # set by TeleopControlPanel after construction

        self.create_subscription(
            Bool, "/teleop_status/right_arm_enabled",
            lambda msg: self._status_cb("right", msg), 10,
        )
        self.create_subscription(
            Bool, "/teleop_status/left_arm_enabled",
            lambda msg: self._status_cb("left", msg), 10,
        )
        self._go_home_cli = self.create_client(Trigger, "/exotica_arm_teleop/go_home")
        self._recalibrate_cli = self.create_client(Trigger, "/exotica_arm_teleop/recalibrate")

    def _status_cb(self, hand: str, msg: Bool):
        self._status[hand] = bool(msg.data)
        if self._on_update is not None:
            self._on_update()

    def call_go_home(self, on_done=None):
        self._call_trigger(self._go_home_cli, "go_home", on_done)

    def call_recalibrate(self, on_done=None):
        self._call_trigger(self._recalibrate_cli, "recalibrate", on_done)

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


class TeleopControlPanel:
    _BG = "#2b2b2b"
    _FG = "#cccccc"
    _ENABLED_COLOR = "#27ae60"
    _DISABLED_COLOR = "#555555"
    _BTN_HOME_COLOR = "#c0392b"
    _BTN_RECAL_COLOR = "#2980b9"

    def __init__(self, node: TeleopControlPanelNode):
        self._node = node
        self._root = tk.Tk()
        self._root.title("Teleop Control Panel")
        self._root.resizable(False, False)
        self._root.configure(bg=self._BG)

        # Wire node status updates → UI refresh (thread-safe via root.after)
        node._on_update = lambda: self._root.after(0, self._refresh_status)

        self._build_ui()
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

    def _set_status(self, text: str):
        self._status_bar.config(text=text)

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
