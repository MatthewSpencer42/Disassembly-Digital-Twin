#!/usr/bin/env python3
"""
config_runner.py — GUI-driven deterministic disassembly executor.

Loads a device config YAML and walks the arms through every step with a
tkinter control panel: step list (colour-coded status), log pane, and
Next / Skip / Stop buttons.  No LLM involved.

Usage
-----
  ros2 run disassembly_skill config_runner \
      --ros-args \
      -p config:=/path/to/hdd.yaml \
      -p interactive:=true \
      -p start_step:=1

Parameters
----------
  config       path to device config YAML  (default: built-in hdd.yaml)
  interactive  pause before each step for GUI confirmation  (default: true)
  start_step   1-based step to begin from — resume partial runs  (default: 1)
"""

from __future__ import annotations

import copy
import json
import math
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import font as tkfont

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, String

from ament_index_python.packages import get_package_share_directory
from disassembly_skill.device_config import DeviceConfig
from disassembly_skill.object_flip_drop_skill import FlipDropSkill
from disassembly_skill.object_flip_skill import ObjectFlipSkill
from disassembly_skill.object_hold_skill import ObjectHoldSkill
from disassembly_skill.object_pickup_skill import PickupSkill
from disassembly_skill.unscrew_skill import UnscrewSkill

DEFAULT_CONFIG = (
    Path(get_package_share_directory("disassembly_skill"))
    / "config" / "device_configs" / "hdd.yaml"
)

# ── status colour palette ─────────────────────────────────────────────────────
_COL = {
    "pending":  {"bg": "#2b2b2b", "fg": "#888888"},
    "skipped":  {"bg": "#2b2b2b", "fg": "#555555"},
    "running":  {"bg": "#1a3a5c", "fg": "#7ecfff"},
    "waiting":  {"bg": "#3a3000", "fg": "#ffd966"},
    "done":     {"bg": "#1a3a1a", "fg": "#66cc66"},
    "failed":   {"bg": "#3a1a1a", "fg": "#ff6666"},
}
_BG  = "#1e1e1e"
_FG  = "#d4d4d4"
_BTN = {"activebackground": "#444", "relief": "flat", "bd": 0, "pady": 8, "padx": 16}

ACTION_ICON = {
    "hold":      "✋",
    "unscrew":   "🔩",
    "pickup":    "📦",
    "flip":      "🔄",
    "flip_drop": "🗑️",
}


# ─────────────────────────────────────────────────────────────────────────────
# GUI
# ─────────────────────────────────────────────────────────────────────────────

class RunnerGUI:
    """tkinter window — must be created and mainloop()'d on the main thread."""

    def __init__(self, cfg: DeviceConfig, on_next, on_skip, on_stop):
        self._on_next = on_next
        self._on_skip = on_skip
        self._on_stop = on_stop
        self._steps = cfg.disassembly_sequence

        self.root = tk.Tk()
        self.root.title(f"Config Runner — {cfg.device.device_model.upper()}")
        self.root.configure(bg=_BG)
        self.root.geometry("900x560")
        self.root.minsize(720, 420)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        mono = tkfont.Font(family="Monospace", size=10)
        bold = tkfont.Font(family="Monospace", size=10, weight="bold")
        sm   = tkfont.Font(family="Monospace", size=9)

        # ── top bar ──────────────────────────────────────────────────────────
        top = tk.Frame(self.root, bg="#111", pady=6)
        top.pack(fill="x")
        tk.Label(
            top,
            text=f"  {cfg.device.device_model.upper()} — {len(self._steps)} steps",
            bg="#111", fg="#aaa", font=bold,
        ).pack(side="left")
        self._status_lbl = tk.Label(
            top, text="Initialising…", bg="#111", fg="#ffd966", font=sm
        )
        self._status_lbl.pack(side="right", padx=10)

        # ── robot-state strip (below top bar) ────────────────────────────────
        state_row = tk.Frame(self.root, bg="#181818", pady=3)
        state_row.pack(fill="x")
        tk.Label(state_row, text="  ARM STATE:", bg="#181818", fg="#555", font=sm).pack(side="left")
        self._arm_state_lbl = tk.Label(
            state_row, text="IDLE", bg="#181818", fg="#888", font=bold, width=10, anchor="w"
        )
        self._arm_state_lbl.pack(side="left", padx=(2, 16))
        tk.Label(state_row, text="GRIP HOLD:", bg="#181818", fg="#555", font=sm).pack(side="left")
        self._held_lbl = tk.Label(
            state_row, text="—", bg="#181818", fg="#888", font=bold, width=8, anchor="w"
        )
        self._held_lbl.pack(side="left", padx=2)

        # ── body ─────────────────────────────────────────────────────────────
        body = tk.Frame(self.root, bg=_BG)
        body.pack(fill="both", expand=True, padx=8, pady=4)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        # left — step list
        left = tk.Frame(body, bg=_BG, width=270)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        left.pack_propagate(False)

        tk.Label(left, text="SEQUENCE", bg=_BG, fg="#666", font=sm).pack(anchor="w", pady=(4, 2))

        self._step_frames: list[tk.Frame] = []
        self._step_labels: list[tk.Label] = []
        for step in self._steps:
            icon = ACTION_ICON.get(step.action.lower(), "·")
            row = tk.Frame(left, bg=_COL["pending"]["bg"], padx=6, pady=4)
            row.pack(fill="x", pady=1)
            lbl = tk.Label(
                row,
                text=f"{icon} {step.step:>2}.  {step.action:<9}  {step.target}",
                bg=_COL["pending"]["bg"],
                fg=_COL["pending"]["fg"],
                font=mono, anchor="w",
            )
            lbl.pack(fill="x")
            self._step_frames.append(row)
            self._step_labels.append(lbl)

        # right — log
        right = tk.Frame(body, bg=_BG)
        right.grid(row=0, column=1, sticky="nsew")

        tk.Label(right, text="LOG", bg=_BG, fg="#666", font=sm).pack(anchor="w", pady=(4, 2))

        log_frame = tk.Frame(right, bg="#111")
        log_frame.pack(fill="both", expand=True)
        self._log = tk.Text(
            log_frame,
            bg="#111", fg="#ccc", font=sm,
            wrap="word", state="disabled",
            bd=0, relief="flat",
            selectbackground="#333",
        )
        sb = tk.Scrollbar(log_frame, command=self._log.yview, bg="#333", troughcolor="#111", bd=0)
        self._log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self._log.pack(fill="both", expand=True)

        # colour tags for log
        self._log.tag_config("ok",   foreground="#66cc66")
        self._log.tag_config("fail", foreground="#ff6666")
        self._log.tag_config("warn", foreground="#ffd966")
        self._log.tag_config("info", foreground="#aaaaaa")
        self._log.tag_config("head", foreground="#7ecfff")

        # ── button bar ────────────────────────────────────────────────────────
        btns = tk.Frame(self.root, bg="#111", pady=8)
        btns.pack(fill="x")

        self._btn_next = tk.Button(
            btns, text="▶  NEXT / CONFIRM",
            bg="#1a3a5c", fg="#7ecfff", font=bold,
            command=self._on_next, **_BTN,
        )
        self._btn_next.pack(side="left", padx=(12, 4))

        self._btn_skip = tk.Button(
            btns, text="⏭  SKIP",
            bg="#2b2b2b", fg="#aaaaaa", font=bold,
            command=self._on_skip, **_BTN,
        )
        self._btn_skip.pack(side="left", padx=4)

        self._btn_stop = tk.Button(
            btns, text="⏹  STOP",
            bg="#3a1a1a", fg="#ff6666", font=bold,
            command=self._on_stop, **_BTN,
        )
        self._btn_stop.pack(side="right", padx=12)

    # ── public API (safe to call from any thread) ─────────────────────────────

    def set_step_status(self, step_idx: int, status: str) -> None:
        """status: pending | running | waiting | done | failed | skipped"""
        def _update():
            col = _COL.get(status, _COL["pending"])
            row = self._step_frames[step_idx]
            lbl = self._step_labels[step_idx]
            row.configure(bg=col["bg"])
            lbl.configure(bg=col["bg"], fg=col["fg"])
        self.root.after(0, _update)

    def set_status_bar(self, msg: str, colour: str = "#ffd966") -> None:
        def _update():
            self._status_lbl.configure(text=msg, fg=colour)
        self.root.after(0, _update)

    def set_robot_state(self, arm_state: str, is_held: bool) -> None:
        """Update the arm-state / grip-hold strip."""
        _ARM_COLOURS = {
            "HOLDING": "#66cc66",
            "MOVING":  "#7ecfff",
            "FLIPPING": "#ffb347",
            "FLIP_DROPPING": "#ffb347",
            "IDLE":    "#888888",
        }
        arm_col = _ARM_COLOURS.get(arm_state.upper(), "#888888")
        held_txt = "✓ HELD" if is_held else "—"
        held_col = "#66cc66" if is_held else "#555555"
        def _update():
            self._arm_state_lbl.configure(text=arm_state.upper(), fg=arm_col)
            self._held_lbl.configure(text=held_txt, fg=held_col)
        self.root.after(0, _update)

    def set_buttons_enabled(self, next_: bool, skip: bool) -> None:
        def _update():
            self._btn_next.configure(state="normal" if next_ else "disabled")
            self._btn_skip.configure(state="normal" if skip  else "disabled")
        self.root.after(0, _update)

    def log(self, msg: str, tag: str = "info") -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        def _update():
            self._log.configure(state="normal")
            self._log.insert("end", f"[{ts}] {msg}\n", tag)
            self._log.see("end")
            self._log.configure(state="disabled")
        self.root.after(0, _update)

    def _on_close(self):
        self._on_stop()
        self.root.destroy()

    def mainloop(self):
        self.root.mainloop()


# ─────────────────────────────────────────────────────────────────────────────
# Runner (executes in a background thread)
# ─────────────────────────────────────────────────────────────────────────────

class ConfigRunner(Node):

    def __init__(self, cfg: DeviceConfig, interactive: bool, start_step: int, gui: RunnerGUI | None):
        super().__init__("config_runner")
        self.cfg = cfg
        self.interactive = interactive
        self.start_step = start_step
        self.gui = gui

        # Gate events for interactive mode
        self._proceed_event = threading.Event()   # Next button
        self._skip_event    = threading.Event()   # Skip button
        self._stop_event    = threading.Event()   # Stop button

        # Subscribe to vision state so we can read screw counts at runtime
        self._vision_lock    = threading.Lock()
        self._latest_vision: dict = {}
        self.create_subscription(String, "/vision/agent_state", self._vision_cb, 10)

        # ── Robot state tracking (updates GUI strip) ──────────────────────────
        self._arm_state = "IDLE"
        self._is_held   = False
        self.create_subscription(String, "/robot_state/manip_arm/update", self._arm_state_cb, 10)
        _hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/object_hold_state/is_held", self._hold_status_cb, _hold_qos)

        # Publishers so the runner can seed IDLE on both arm state topics at launch.
        self._manip_state_pub = self.create_publisher(String, "/robot_state/manip_arm/update", 10)
        self._tool_state_pub  = self.create_publisher(String, "/robot_state/tool_arm/update",  10)

        self.hold_skill      = ObjectHoldSkill(device_cfg=cfg)
        self.unscrew_skill   = UnscrewSkill(device_cfg=cfg)
        self.flip_skill      = ObjectFlipSkill(device_cfg=cfg)
        self.flip_drop_skill = FlipDropSkill(device_cfg=cfg)
        self.pickup_skill    = PickupSkill(device_cfg=cfg)

        # One-shot timer: publish IDLE on both arm-state topics 0.5 s after
        # startup so the dashboard never shows OFFLINE at system launch.
        self._init_state_timer = self.create_timer(0.5, self._publish_initial_idle)

    # ── GUI callbacks (called from main thread via button press) ──────────────

    def gui_next(self):
        self._skip_event.clear()
        self._proceed_event.set()

    def gui_skip(self):
        self._skip_event.set()
        self._proceed_event.set()

    def gui_stop(self):
        self._stop_event.set()
        self._proceed_event.set()  # unblock any wait

    # ── Startup IDLE ──────────────────────────────────────────────────────────

    def _publish_initial_idle(self):
        """Fire once 0.5 s after launch; seeds both arm-state topics so the
        dashboard shows IDLE rather than OFFLINE on a fresh system start."""
        self._manip_state_pub.publish(String(data="IDLE"))
        self._tool_state_pub.publish(String(data="IDLE"))
        self._init_state_timer.cancel()

    # ── Vision ────────────────────────────────────────────────────────────────

    def _vision_cb(self, msg: String):
        try:
            with self._vision_lock:
                self._latest_vision = json.loads(msg.data)
        except Exception:
            pass

    def _arm_state_cb(self, msg: String):
        self._arm_state = msg.data
        if self.gui:
            self.gui.set_robot_state(self._arm_state, self._is_held)

    def _hold_status_cb(self, msg: Bool):
        self._is_held = bool(msg.data)
        if self.gui:
            self.gui.set_robot_state(self._arm_state, self._is_held)

    def _global_screws(self, zone_label: str | None = None) -> list:
        with self._vision_lock:
            objs = self._latest_vision.get("global_view", {}).get("objects", [])
        screws = [o for o in objs if "screw" in o.get("label", "").lower()]
        if not zone_label:
            return screws
        zone_label = zone_label.lower()
        exact = [o for o in screws if o.get("label", "").lower() == zone_label]
        if exact:
            return exact
        return [o for o in screws if zone_label in o.get("label", "").lower()]

    def _all_global_screws(self) -> list:
        with self._vision_lock:
            objs = self._latest_vision.get("global_view", {}).get("objects", [])
        return [o for o in objs if "screw" in o.get("label", "").lower()]

    def _global_objects(self) -> list:
        with self._vision_lock:
            return list(self._latest_vision.get("global_view", {}).get("objects", []) or [])

    def _find_global_label(self, label: str):
        label_norm = str(label or "").strip().lower()
        if not label_norm:
            return None
        for obj in self._global_objects():
            obj_label = str(obj.get("label", "")).strip().lower()
            if obj_label == label_norm or label_norm in obj_label:
                xyz = obj.get("xyz")
                if isinstance(xyz, (list, tuple)) and len(xyz) >= 3 and all(v is not None for v in xyz[:3]):
                    return obj
        return None

    def _wait_for_global_label(self, label: str, timeout_s: float = 3.0):
        deadline = time.time() + max(float(timeout_s), 0.1)
        while time.time() < deadline and not self._stop_event.is_set():
            obj = self._find_global_label(label)
            if obj is not None:
                return obj
            time.sleep(0.2)
        return None

    def _screw_key(self, obj) -> str:
        obj_id = obj.get("id", None)
        if obj_id is not None:
            return f"id:{obj_id}"
        xyz = obj.get("xyz") or []
        if len(xyz) >= 3:
            try:
                return "xyz:" + ",".join(f"{float(v):.3f}" for v in xyz[:3])
            except Exception:
                pass
        return f"anon:{id(obj)}"

    def _collect_unscrew_targets(self, zone_label: str, expected_count: int, timeout_s: float = 5.0) -> list:
        """Collect visible screw detections over several fresh frames.

        Vision can briefly report only part of the screw set. For the runner,
        use the union of visible screw IDs over a short window, preferring the
        requested zone but allowing other visible screw labels when the zone
        detector reports fewer than the config expects.
        """
        deadline = time.time() + max(float(timeout_s), 0.5)
        by_key: dict[str, dict] = {}
        best_zone_count = 0
        while time.time() < deadline and not self._stop_event.is_set():
            zone_screws = self._global_screws(zone_label)
            all_screws = self._all_global_screws()
            visible = list(zone_screws)
            if len(zone_screws) < expected_count:
                seen = {self._screw_key(o) for o in visible}
                visible.extend(o for o in all_screws if self._screw_key(o) not in seen)
            for obj in visible:
                by_key[self._screw_key(obj)] = copy.deepcopy(obj)
            best_zone_count = max(best_zone_count, len(zone_screws))
            if len(by_key) >= expected_count:
                break
            time.sleep(0.25)
        targets = list(by_key.values())
        if targets:
            self._log(
                f"  Vision union: {len(targets)} visible screw candidate(s) "
                f"({best_zone_count} matched zone '{zone_label}')",
                "info",
            )
        return targets

    def _screw_distance_from_xarm(self, obj) -> float:
        xyz = obj.get("xyz")
        if not isinstance(xyz, (list, tuple)) or len(xyz) < 3 or any(v is None for v in xyz[:3]):
            return math.inf
        try:
            raw_pose = Pose()
            raw_pose.position.x = float(xyz[0])
            raw_pose.position.y = float(xyz[1])
            raw_pose.position.z = float(xyz[2])
            base_pose = self.unscrew_skill.moveit_backend.get_transformed_pose(
                raw_pose,
                self.unscrew_skill.CONFIG["CAMERA_FRAME"],
                self.unscrew_skill.CONFIG["XARM_BASE_FRAME"],
            )
            if base_pose is not None:
                return math.hypot(
                    float(base_pose.pose.position.x),
                    float(base_pose.pose.position.y),
                )
        except Exception:
            pass
        return math.inf

    # ── Gate ──────────────────────────────────────────────────────────────────

    def _gate(self, msg: str) -> bool:
        """Returns True=proceed, False=skip, raises SystemExit on Stop."""
        if not self.interactive:
            return True
        self._proceed_event.clear()
        self._skip_event.clear()
        if self.gui:
            self.gui.set_status_bar(msg, "#ffd966")
            self.gui.set_buttons_enabled(next_=True, skip=True)
        self._proceed_event.wait()
        if self.gui:
            self.gui.set_buttons_enabled(next_=False, skip=False)
        if self._stop_event.is_set():
            raise SystemExit("Stopped by user")
        return not self._skip_event.is_set()

    def _log(self, msg: str, tag: str = "info"):
        print(f"[config_runner] {msg}")
        if self.gui:
            self.gui.log(msg, tag)

    # ── Vision cache reset ────────────────────────────────────────────────────

    def _reset_vision_caches(self, wait_s: float = 1.5):
        """Clear stale vision snapshots in all skills, then wait for fresh data."""
        self._log("  [vision] Resetting vision caches for next step…", "info")
        with self._vision_lock:
            self._latest_vision = {}
        for skill in (self.pickup_skill, self.unscrew_skill):
            lock = getattr(skill, "data_lock", None)
            if lock is None:
                continue
            with lock:
                if hasattr(skill, "latest_targets"):
                    skill.latest_targets = []
                if hasattr(skill, "local_view"):
                    skill.local_view = {}
        time.sleep(wait_s)   # let vision node publish a fresh snapshot

    def _sync_hold_state(self, held: bool):
        """Keep all in-process skills consistent with the runner hold state."""
        held = bool(held)
        self._is_held = held
        for skill in (self.hold_skill, self.flip_skill, self.pickup_skill):
            if hasattr(skill, "is_holding_object"):
                skill.is_holding_object = held
        if self.gui:
            self.gui.set_robot_state(self._arm_state, self._is_held)
        if held:
            try:
                self.hold_skill.publish_hold_status(True)
            except Exception:
                pass

    def _held_now(self) -> bool:
        if self._is_held:
            self._sync_hold_state(True)
            return True
        return False

    def _previous_hold_step(self, before_step_number: int):
        holds = [
            s for s in self.cfg.disassembly_sequence
            if s.action.lower() == "hold" and s.step < before_step_number
        ]
        return holds[-1] if holds else None

    def _ensure_hold_for_step(self, step, reason: str) -> bool:
        if self._held_now():
            self._is_held = True
            self._log(f"  [state] Active hold detected — no re-hold needed before {reason}.", "info")
            return True
        hold_step = self._previous_hold_step(step.step)
        if hold_step is None:
            self._log(f"  [state] No active hold and no previous hold step before {reason}.", "warn")
            return True
        self._log(
            f"  [state] No active hold before {reason}; re-running hold step {hold_step.step}.",
            "warn",
        )
        ok = self._run_hold(hold_step)
        if ok:
            self._is_held = True
            self._reset_vision_caches()
        return ok

    # ── Step dispatch ─────────────────────────────────────────────────────────

    def _run_hold(self, step) -> bool:
        if self._held_now() and not bool(step.parameters.get("force_rehold", False)):
            self._is_held = True
            self._log(
                f"  [state] Active hold detected — step {step.step} hold already satisfied.",
                "info",
            )
            return True
        return self.hold_skill.execute_hold(
            part_id=None, target_label=step.target, interactive=False, hold_step=step
        )

    def _run_unscrew(self, step) -> bool:
        """
        Iterate over all visible screws for this zone, nearest to xArm first.
        Holes confirmed by sniper are counted as already-removed and skipped.
        If a screw fails in interactive mode, Next retries that screw and Skip
        advances to the next screw without aborting the whole zone.
        """
        screw_count = step.parameters.get("screw_count", 1)
        zone_name   = step.target
        if not self._ensure_hold_for_step(step, "unscrew"):
            return False

        detected = self._collect_unscrew_targets(zone_name, screw_count)
        detected = sorted(
            detected,
            key=lambda obj: (
                self._screw_distance_from_xarm(obj),
                int(obj.get("id", 999999)) if str(obj.get("id", "")).isdigit() else 999999,
            ),
        )
        self._log(
            f"  Vision: {len(detected)} screw candidate(s) visible  "
            f"(config expects {screw_count} in zone '{zone_name}')",
            "info",
        )
        if len(detected) < screw_count:
            self._log(
                f"  [WARN] Only {len(detected)} detected — "
                f"{screw_count - len(detected)} may be already removed or occluded.",
                "warn",
            )

        targets_to_try = detected
        if targets_to_try:
            order = ", ".join(
                f"ID={obj.get('id')}({self._screw_distance_from_xarm(obj):.3f}m)"
                for obj in targets_to_try
            )
            self._log(f"  Nearest-first screw order: {order}", "info")
        done, holes, failures, skipped = 0, 0, 0, 0

        i = 0
        while i < len(targets_to_try):
            obj = targets_to_try[i]
            if self._stop_event.is_set():
                return False
            obj_id  = obj.get("id")
            obj_lbl = obj.get("label", "screw")

            # ── Per-screw gate: show ID and wait for user confirmation ────────
            self._log(
                f"  Screw {i+1}/{len(targets_to_try)}  ID={obj_id}  label={obj_lbl}",
                "head",
            )
            if self.interactive:
                proceed = self._gate(
                    f"Screw {i+1}/{len(targets_to_try)}: ID={obj_id}  [{obj_lbl}]"
                    "  —  Next to unscrew / Skip to skip / Stop to abort"
                )
                if self._stop_event.is_set():
                    return False
                if not proceed:   # user pressed Skip
                    self._log(f"  Skipped screw ID={obj_id} by user.", "warn")
                    skipped += 1
                    i += 1
                    continue

            result = self.unscrew_skill.execute_unscrew_command(
                target_id=obj_id,
                target_label=obj_lbl,
                interactive=False,
                target_data_override=copy.deepcopy(obj),
            )

            if result == "HOLE":
                self._log(f"  → ID={obj_id}: sniper confirmed hole — skipping.", "warn")
                holes += 1
                i += 1
            elif result:
                if step.parameters.get("align_only_debug", False):
                    self._log(f"  → ID={obj_id}: XY alignment verified.", "ok")
                elif step.parameters.get("coarse_only_debug", False):
                    self._log(f"  → ID={obj_id}: coarse pose verified.", "ok")
                else:
                    self._log(f"  → ID={obj_id}: extracted.", "ok")
                done += 1
                i += 1
            else:
                self._log(f"  → ID={obj_id}: FAILED.", "fail")
                failures += 1
                if not self.interactive:
                    return False
                self._log(
                    "  Press Next to retry this screw, Skip to skip only this screw and continue, Stop to abort.",
                    "warn",
                )
                proceed = self._gate(
                    f"Unscrew ID={obj_id} failed. [Next = retry / Skip = skip this screw / Stop = abort]"
                )
                if self._stop_event.is_set():
                    return False
                if proceed:
                    self._log(f"  Retrying screw ID={obj_id}.", "warn")
                    continue
                skipped += 1
                self._log(f"  Skipping failed screw ID={obj_id}; moving to next screw.", "warn")
                i += 1

        self._log(
            f"  Zone '{zone_name}': {done} "
            f"{'XY alignments verified' if step.parameters.get('align_only_debug', False) else ('coarse poses verified' if step.parameters.get('coarse_only_debug', False) else 'extracted')}, "
            f"{holes} holes skipped, {skipped} user-skipped, {failures} failed attempt(s).",
            "ok" if failures == 0 and skipped == 0 else "warn",
        )
        return True

    def _run_pickup(self, step) -> bool:
        if not self._ensure_hold_for_step(step, "pickup"):
            return False
        self._reset_vision_caches(wait_s=2.0)
        target = self._wait_for_global_label(step.target, timeout_s=3.0)
        if target is None:
            self._log(
                f"  [pickup] Target '{step.target}' is not visible in global vision. "
                "Skipping pickup; part may already be removed or occluded.",
                "warn",
            )
            return True
        self._log(
            f"  [pickup] Target '{step.target}' visible: ID={target.get('id')} "
            f"label={target.get('label')}.",
            "info",
        )
        self.pickup_skill.is_holding_object = self._held_now()
        self.pickup_skill._apply_pickup_config(self.cfg, target_label=step.target, pickup_step=step)
        return self.pickup_skill.execute_pickup(
            target_id=None, target_label=step.target, interactive=False, pickup_step=step
        )

    def _run_flip(self, step) -> bool:
        if not self._ensure_hold_for_step(step, "flip"):
            return False
        self.flip_skill.is_holding_object = self._held_now()
        return self.flip_skill.execute_flip(interactive=False)

    def _run_flip_drop(self, _step) -> bool:
        return self.flip_drop_skill.execute_flip_drop(interactive=False)

    def _dispatch(self, step) -> bool:
        action = step.action.lower()
        dispatch = {
            "hold":      self._run_hold,
            "unscrew":   self._run_unscrew,
            "pickup":    self._run_pickup,
            "flip":      self._run_flip,
            "flip_drop": self._run_flip_drop,
        }
        fn = dispatch.get(action)
        if fn is None:
            self._log(f"Unknown action '{action}' — skipping.", "warn")
            return True
        result = fn(step)
        # After a successful hold the robot pose has changed; always flush stale
        # vision caches so the next step (unscrew / pickup) sees fresh detections.
        if action == "hold" and result:
            self._reset_vision_caches()
        return result

    # ── Main sequence loop ────────────────────────────────────────────────────

    def run(self) -> bool:
        sequence = self.cfg.disassembly_sequence
        total    = len(sequence)

        for step in sequence:
            idx = step.step - 1  # 0-based GUI index

            if step.step < self.start_step:
                self._log(f"[skip] step {step.step}: {step.label}", "info")
                if self.gui:
                    self.gui.set_step_status(idx, "skipped")
                continue

            # ── pre-confirmation gate ──
            icon = ACTION_ICON.get(step.action.lower(), "·")
            self._log(
                f"\n{'='*54}\n"
                f"{icon} STEP {step.step}/{total}  {step.action.upper()}  →  {step.target}\n"
                f"  {step.label}\n{'='*54}",
                "head",
            )
            if self.gui:
                self.gui.set_step_status(idx, "waiting")

            proceed = self._gate(
                f"Step {step.step}/{total}: {icon} {step.action}  →  {step.target}   [Next to run / Skip]"
            )

            if self._stop_event.is_set():
                self._log("Stopped by user.", "fail")
                if self.gui:
                    self.gui.set_step_status(idx, "failed")
                return False

            if not proceed:
                self._log(f"Step {step.step} skipped by user.", "warn")
                if self.gui:
                    self.gui.set_step_status(idx, "skipped")
                continue

            # ── execute ──
            if self.gui:
                self.gui.set_step_status(idx, "running")
                self.gui.set_status_bar(f"Running step {step.step}…", "#7ecfff")

            t0      = time.monotonic()
            success = self._dispatch(step)
            elapsed = time.monotonic() - t0

            if success:
                self._log(f"Step {step.step} done in {elapsed:.1f}s.", "ok")
                if self.gui:
                    self.gui.set_step_status(idx, "done")
                # Brief post-step confirmation so the user can inspect before continuing
                self._gate(f"Step {step.step} complete ({elapsed:.1f}s).  [Next to continue / Skip]")
                if self._stop_event.is_set():
                    return False
            else:
                self._log(f"Step {step.step} FAILED after {elapsed:.1f}s.", "fail")
                if self.gui:
                    self.gui.set_step_status(idx, "failed")
                    self.gui.set_status_bar(f"Step {step.step} FAILED — press Stop or Next to retry", "#ff6666")
                # Let the user decide: Next = retry, Stop = abort
                if self.interactive:
                    self._log("Press Next to retry this step, Stop to abort.", "warn")
                    self._gate(f"Step {step.step} failed.  [Next to retry / Stop to abort]")
                    if self._stop_event.is_set():
                        return False
                    # Re-run the same step once
                    if self.gui:
                        self.gui.set_step_status(idx, "running")
                    t0      = time.monotonic()
                    success = self._dispatch(step)
                    elapsed = time.monotonic() - t0
                    if success:
                        self._log(f"Step {step.step} retry succeeded in {elapsed:.1f}s.", "ok")
                        if self.gui:
                            self.gui.set_step_status(idx, "done")
                    else:
                        self._log(f"Step {step.step} retry also FAILED. Aborting.", "fail")
                        if self.gui:
                            self.gui.set_status_bar("ABORTED — step failed twice", "#ff6666")
                        return False
                else:
                    return False

        self._log("\nALL STEPS COMPLETE — disassembly sequence finished.", "ok")
        if self.gui:
            self.gui.set_status_bar("COMPLETE", "#66cc66")
        return True


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    param_node = rclpy.create_node("config_runner_params")
    param_node.declare_parameter("config",      str(DEFAULT_CONFIG))
    param_node.declare_parameter("interactive", True)
    param_node.declare_parameter("start_step",  1)

    config_path = Path(param_node.get_parameter("config").value)
    interactive = param_node.get_parameter("interactive").value
    start_step  = param_node.get_parameter("start_step").value
    param_node.destroy_node()

    if not config_path.exists():
        print(f"[ERROR] Config file not found: {config_path}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    try:
        cfg = DeviceConfig.load(config_path)
    except ValueError as exc:
        print(f"[ERROR] Invalid config: {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    print(f"Loaded: {cfg.device.device_model} — {len(cfg.disassembly_sequence)} steps")
    for w in cfg.warnings:
        print(f"  [warn] {w}")

    # Button callbacks use a ref-list so they work before runner is created.
    runner_ref: list[ConfigRunner | None] = [None]

    def _next(): runner_ref[0] and runner_ref[0].gui_next()
    def _skip(): runner_ref[0] and runner_ref[0].gui_skip()
    def _stop(): runner_ref[0] and runner_ref[0].gui_stop()

    # ── Build GUI immediately (before any slow node init) ─────────────────────
    gui: RunnerGUI | None = None
    if interactive:
        gui = RunnerGUI(cfg, on_next=_next, on_skip=_skip, on_stop=_stop)
        gui.set_buttons_enabled(next_=False, skip=False)
        gui.set_status_bar("Initialising ROS nodes…", "#aaaaaa")

    result_holder: list[bool] = [False]

    # ── All heavy init + sequence execution in one background thread ──────────
    def _init_and_run():
        # Node init (slow — loads robot model, MoveIt, etc.)
        runner = ConfigRunner(cfg, interactive=interactive, start_step=start_step, gui=gui)
        runner_ref[0] = runner

        executor = MultiThreadedExecutor()
        for node in [runner, runner.hold_skill, runner.unscrew_skill,
                     runner.flip_skill, runner.flip_drop_skill, runner.pickup_skill]:
            executor.add_node(node)

        threading.Thread(target=executor.spin, daemon=True).start()
        time.sleep(1.5)  # let subscriptions warm up

        if gui:
            gui.set_status_bar("Ready — press Next to begin", "#66cc66")
            # Buttons are enabled by the first _gate() call inside runner.run()

        try:
            result_holder[0] = runner.run()
        except SystemExit:
            result_holder[0] = False
        finally:
            executor.shutdown(timeout_sec=2.0)
            rclpy.shutdown()
            if gui:
                gui.root.after(500, gui.root.destroy)

    run_thread = threading.Thread(target=_init_and_run, daemon=True)
    run_thread.start()

    if gui:
        gui.mainloop()
    else:
        run_thread.join()

    sys.exit(0 if result_holder[0] else 1)
