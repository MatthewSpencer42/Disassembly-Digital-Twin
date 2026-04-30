"""Joy Teleop Control Panel — Tkinter GUI for gamepad arm teleoperation.

Shows:
  • Gamepad connection status
  • Active arm indicator (switches with LB button)
  • EXOTica IK server status
  • Per-arm enable / disable buttons
  • Gripper state for the active arm
  • Go Home button

Talks to joy_arm_teleop node via:
  Services  : ~/set_xarm5_enabled, ~/set_uf850_enabled,
              ~/set_all_arms_enabled, ~/go_home
  Topics    : /joy_teleop_status/{xarm5_enabled, uf850_enabled,
              active_arm, controller_connected, gripper_closed,
              exotica_ready}
"""
from __future__ import annotations

import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


_NODE_NS = "/joy_arm_teleop"


class JoyTeleopPanelNode(Node):
    def __init__(self):
        super().__init__("joy_teleop_control_panel")

        self.xarm5_enabled: bool = False
        self.uf850_enabled: bool = False
        self.active_arm: str = "xarm5"
        self.controller_connected: bool = False
        self.gripper_closed: bool = False
        self.exotica_ready: bool = False
        self._on_update: callable | None = None

        sub = lambda topic, msg_type, attr: self.create_subscription(
            msg_type, topic,
            lambda msg, a=attr: self._bool_cb(a, msg),
            10,
        )
        sub("/joy_teleop_status/xarm5_enabled", Bool, "xarm5_enabled")
        sub("/joy_teleop_status/uf850_enabled", Bool, "uf850_enabled")
        sub("/joy_teleop_status/controller_connected", Bool, "controller_connected")
        sub("/joy_teleop_status/gripper_closed", Bool, "gripper_closed")
        sub("/joy_teleop_status/exotica_ready", Bool, "exotica_ready")

        self.create_subscription(
            String, "/joy_teleop_status/active_arm", self._active_arm_cb, 10
        )

        # Also subscribe to /exotica/ready with latched QoS as backup
        ready_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/exotica/ready", self._exotica_latched_cb, ready_qos)

        self.create_timer(0.25, self._refresh_tick)

        self._xarm5_en_cli = self.create_client(SetBool, f"{_NODE_NS}/set_xarm5_enabled")
        self._uf850_en_cli = self.create_client(SetBool, f"{_NODE_NS}/set_uf850_enabled")
        self._all_en_cli = self.create_client(SetBool, f"{_NODE_NS}/set_all_arms_enabled")
        self._home_cli = self.create_client(Trigger, f"{_NODE_NS}/go_home")

    def _bool_cb(self, attr: str, msg: Bool):
        setattr(self, attr, bool(msg.data))
        if self._on_update:
            self._on_update()

    def _active_arm_cb(self, msg: String):
        self.active_arm = msg.data
        if self._on_update:
            self._on_update()

    def _exotica_latched_cb(self, msg: Bool):
        self.exotica_ready = bool(msg.data)
        if self._on_update:
            self._on_update()

    def _refresh_tick(self):
        if self._on_update:
            self._on_update()

    # ── Service calls ──────────────────────────────────────────────────────────

    def call_set_xarm5_enabled(self, enabled: bool, on_done=None):
        self._call_bool(self._xarm5_en_cli, "set_xarm5_enabled", enabled, on_done)

    def call_set_uf850_enabled(self, enabled: bool, on_done=None):
        self._call_bool(self._uf850_en_cli, "set_uf850_enabled", enabled, on_done)

    def call_set_all_enabled(self, enabled: bool, on_done=None):
        self._call_bool(self._all_en_cli, "set_all_arms_enabled", enabled, on_done)

    def call_go_home(self, on_done=None):
        if not self._home_cli.service_is_ready():
            if on_done:
                on_done(False, "go_home service not available")
            return
        future = self._home_cli.call_async(Trigger.Request())

        def _cb(f):
            try:
                res = f.result()
                if on_done:
                    on_done(res.success, res.message)
            except Exception as exc:
                if on_done:
                    on_done(False, str(exc))

        future.add_done_callback(_cb)

    def _call_bool(self, client, name: str, value: bool, on_done=None):
        if not client.service_is_ready():
            msg = f"{name} service not available"
            self.get_logger().warning(msg)
            if on_done:
                on_done(False, msg)
            return
        req = SetBool.Request()
        req.data = bool(value)
        future = client.call_async(req)

        def _cb(f):
            try:
                res = f.result()
                if on_done:
                    on_done(res.success, res.message)
            except Exception as exc:
                if on_done:
                    on_done(False, str(exc))

        future.add_done_callback(_cb)


class JoyTeleopControlPanel:
    # ── Theme ──────────────────────────────────────────────────────────────────
    _BG = "#2b2b2b"
    _FG = "#cccccc"
    _ACTIVE_COLOR = "#1f8b4c"       # active arm highlight
    _INACTIVE_COLOR = "#444444"     # inactive arm
    _ENABLED_COLOR = "#27ae60"      # arm enabled indicator
    _DISABLED_COLOR = "#555555"     # arm disabled indicator
    _CONN_COLOR = "#1f8b4c"         # controller connected
    _DISC_COLOR = "#c0392b"         # controller disconnected
    _READY_COLOR = "#1f8b4c"        # EXOTica ready
    _STARTING_COLOR = "#d68910"     # EXOTica starting
    _OFFLINE_COLOR = "#7f8c8d"      # EXOTica offline
    _BTN_ENABLE = "#1f8b4c"
    _BTN_DISABLE = "#8e44ad"
    _BTN_HOME = "#c0392b"
    _BTN_BOTH = "#d35400"

    def __init__(self, node: JoyTeleopPanelNode):
        self._node = node
        self._root = tk.Tk()
        self._root.title("Joy Teleop Control Panel")
        self._root.resizable(False, False)
        self._root.configure(bg=self._BG)

        node._on_update = lambda: self._root.after(0, self._refresh)

        self._build_ui()
        self._root.update_idletasks()
        w = self._root.winfo_reqwidth()
        h = self._root.winfo_reqheight()
        self._root.geometry(f"{w}x{h}")
        self._root.minsize(w, h)
        self._root.maxsize(w, h)

        self._refresh()
        self._root.protocol("WM_DELETE_WINDOW", self._root.destroy)

    # ── UI construction ────────────────────────────────────────────────────────

    def _build_ui(self):
        root = self._root
        pad = {"padx": 14, "pady": 5}
        bold14 = tkfont.Font(family="Helvetica", size=14, weight="bold")
        bold12 = tkfont.Font(family="Helvetica", size=12, weight="bold")
        norm11 = tkfont.Font(family="Helvetica", size=11)

        tk.Label(
            root, text="Joy Teleop Control Panel",
            font=bold14, bg=self._BG, fg="#ffffff",
        ).pack(pady=(14, 4))

        # ── Connection status ──────────────────────────────────────────────────
        conn_frame = tk.LabelFrame(
            root, text="Connection", bg=self._BG, fg="#aaaaaa", padx=10, pady=8,
        )
        conn_frame.pack(fill="x", **pad)

        self._ctrl_lbl = self._indicator_row(conn_frame, "Gamepad:")
        self._exotica_lbl = self._indicator_row(conn_frame, "EXOTica IK:")

        # ── Active arm indicator ───────────────────────────────────────────────
        arm_frame = tk.LabelFrame(
            root, text="Active Arm  (LB switch, X toggle selected, B toggle both)", bg=self._BG, fg="#aaaaaa",
            padx=10, pady=8,
        )
        arm_frame.pack(fill="x", **pad)

        arm_row = tk.Frame(arm_frame, bg=self._BG)
        arm_row.pack(fill="x")
        self._xarm5_active_lbl = tk.Label(
            arm_row, text="xArm5", width=10,
            font=bold12, bg=self._INACTIVE_COLOR, fg="#ffffff",
            relief="raised", padx=8, pady=5,
        )
        self._xarm5_active_lbl.pack(side="left", padx=(0, 8))

        self._uf850_active_lbl = tk.Label(
            arm_row, text="UF850", width=10,
            font=bold12, bg=self._INACTIVE_COLOR, fg="#ffffff",
            relief="raised", padx=8, pady=5,
        )
        self._uf850_active_lbl.pack(side="left")

        # Gripper state sits beside the active arm labels
        self._gripper_lbl = tk.Label(
            arm_row, text="OPEN", width=8,
            font=norm11, bg="#2c3e50", fg="#ffffff",
            relief="raised", padx=6, pady=5,
        )
        self._gripper_lbl.pack(side="right")
        tk.Label(arm_row, text="Gripper:", bg=self._BG, fg=self._FG).pack(side="right", padx=(0, 4))

        # ── Arm enable status ──────────────────────────────────────────────────
        status_frame = tk.LabelFrame(
            root, text="Arm Status", bg=self._BG, fg="#aaaaaa", padx=10, pady=8,
        )
        status_frame.pack(fill="x", **pad)

        self._xarm5_status_lbl = self._indicator_row(status_frame, "xArm5:")
        self._uf850_status_lbl = self._indicator_row(status_frame, "UF850:")

        # ── Arm controls ───────────────────────────────────────────────────────
        ctrl_frame = tk.LabelFrame(
            root, text="Arm Controls", bg=self._BG, fg="#aaaaaa", padx=10, pady=8,
        )
        ctrl_frame.pack(fill="x", **pad)

        self._xarm5_btn = tk.Button(
            ctrl_frame, font=bold12, fg="#ffffff", relief="raised", pady=10,
            command=self._toggle_xarm5,
        )
        self._xarm5_btn.pack(fill="x", pady=3)

        self._uf850_btn = tk.Button(
            ctrl_frame, font=bold12, fg="#ffffff", relief="raised", pady=10,
            command=self._toggle_uf850,
        )
        self._uf850_btn.pack(fill="x", pady=3)

        self._both_btn = tk.Button(
            ctrl_frame, font=bold12, fg="#ffffff", relief="raised", pady=10,
            command=self._toggle_both,
        )
        self._both_btn.pack(fill="x", pady=3)

        # ── Actions ────────────────────────────────────────────────────────────
        action_frame = tk.Frame(root, bg=self._BG)
        action_frame.pack(fill="x", **pad)

        tk.Button(
            action_frame, text="Go Home & Reset",
            font=bold12, bg=self._BTN_HOME, fg="#ffffff",
            activebackground="#e74c3c", relief="raised", pady=10,
            command=self._go_home,
        ).pack(fill="x")

        # ── Status bar ─────────────────────────────────────────────────────────
        self._status_bar = tk.Label(
            root, text="Ready.", anchor="w",
            bg="#1e1e1e", fg="#888888", padx=8, pady=5,
        )
        self._status_bar.pack(fill="x", side="bottom")

    def _indicator_row(self, parent, label_text: str) -> tk.Label:
        row = tk.Frame(parent, bg=self._BG)
        row.pack(fill="x", pady=3)
        tk.Label(row, text=label_text, width=12, anchor="w",
                 bg=self._BG, fg=self._FG).pack(side="left")
        lbl = tk.Label(row, text="—", width=12,
                       bg=self._OFFLINE_COLOR, fg="#ffffff",
                       relief="raised", padx=6, pady=3)
        lbl.pack(side="left")
        return lbl

    # ── UI refresh ─────────────────────────────────────────────────────────────

    def _refresh(self):
        n = self._node

        # Controller
        if n.controller_connected:
            self._ctrl_lbl.config(text="CONNECTED", bg=self._CONN_COLOR)
        else:
            self._ctrl_lbl.config(text="DISCONNECTED", bg=self._DISC_COLOR)

        # EXOTica
        if n.exotica_ready:
            self._exotica_lbl.config(text="READY", bg=self._READY_COLOR)
        else:
            self._exotica_lbl.config(text="STARTING…", bg=self._STARTING_COLOR)

        # Active arm highlight
        if n.active_arm == "xarm5":
            self._xarm5_active_lbl.config(bg=self._ACTIVE_COLOR)
            self._uf850_active_lbl.config(bg=self._INACTIVE_COLOR)
        else:
            self._xarm5_active_lbl.config(bg=self._INACTIVE_COLOR)
            self._uf850_active_lbl.config(bg=self._ACTIVE_COLOR)

        # Gripper
        if n.gripper_closed:
            self._gripper_lbl.config(text="CLOSED", bg="#8e44ad")
        else:
            self._gripper_lbl.config(text="OPEN", bg="#2c3e50")

        # Arm enable status indicators
        self._xarm5_status_lbl.config(
            text="ENABLED" if n.xarm5_enabled else "DISABLED",
            bg=self._ENABLED_COLOR if n.xarm5_enabled else self._DISABLED_COLOR,
        )
        self._uf850_status_lbl.config(
            text="ENABLED" if n.uf850_enabled else "DISABLED",
            bg=self._ENABLED_COLOR if n.uf850_enabled else self._DISABLED_COLOR,
        )

        # Arm control buttons
        self._xarm5_btn.config(
            text="Disable xArm5" if n.xarm5_enabled else "Enable xArm5",
            bg=self._BTN_DISABLE if n.xarm5_enabled else self._BTN_ENABLE,
        )
        self._uf850_btn.config(
            text="Disable UF850" if n.uf850_enabled else "Enable UF850",
            bg=self._BTN_DISABLE if n.uf850_enabled else self._BTN_ENABLE,
        )
        both_on = n.xarm5_enabled and n.uf850_enabled
        self._both_btn.config(
            text="Disable Both Arms" if both_on else "Enable Both Arms",
            bg=self._BTN_DISABLE if both_on else self._BTN_BOTH,
        )

    def _set_status(self, text: str):
        self._status_bar.config(text=text)

    # ── Button callbacks ───────────────────────────────────────────────────────

    def _toggle_xarm5(self):
        desired = not self._node.xarm5_enabled
        self._set_status(f"{'Enabling' if desired else 'Disabling'} xArm5...")
        self._node.call_set_xarm5_enabled(
            desired,
            on_done=lambda ok, msg: self._root.after(
                0, lambda: self._set_status(f"{'OK' if ok else 'FAILED'}: {msg}")
            ),
        )

    def _toggle_uf850(self):
        desired = not self._node.uf850_enabled
        self._set_status(f"{'Enabling' if desired else 'Disabling'} UF850...")
        self._node.call_set_uf850_enabled(
            desired,
            on_done=lambda ok, msg: self._root.after(
                0, lambda: self._set_status(f"{'OK' if ok else 'FAILED'}: {msg}")
            ),
        )

    def _toggle_both(self):
        desired = not (self._node.xarm5_enabled and self._node.uf850_enabled)
        self._set_status(f"{'Enabling' if desired else 'Disabling'} both arms...")
        self._node.call_set_all_enabled(
            desired,
            on_done=lambda ok, msg: self._root.after(
                0, lambda: self._set_status(f"{'OK' if ok else 'FAILED'}: {msg}")
            ),
        )

    def _go_home(self):
        self._set_status("Sending Go Home command...")
        self._node.call_go_home(
            on_done=lambda ok, msg: self._root.after(
                0, lambda: self._set_status(f"{'OK' if ok else 'FAILED'}: {msg}")
            ),
        )

    def run(self):
        self._root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopPanelNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        panel = JoyTeleopControlPanel(node)
        panel.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
