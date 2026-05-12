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
from std_msgs.msg import String

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

        self.hold_skill      = ObjectHoldSkill(device_cfg=cfg)
        self.unscrew_skill   = UnscrewSkill(device_cfg=cfg)
        self.flip_skill      = ObjectFlipSkill(device_cfg=cfg)
        self.flip_drop_skill = FlipDropSkill(device_cfg=cfg)
        self.pickup_skill    = PickupSkill(device_cfg=cfg)

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

    # ── Vision ────────────────────────────────────────────────────────────────

    def _vision_cb(self, msg: String):
        try:
            with self._vision_lock:
                self._latest_vision = json.loads(msg.data)
        except Exception:
            pass

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

    # ── Step dispatch ─────────────────────────────────────────────────────────

    def _run_hold(self, step) -> bool:
        return self.hold_skill.execute_hold(
            part_id=None, target_label=step.target, interactive=False
        )

    def _run_unscrew(self, step) -> bool:
        """
        Iterate over all detected screws in this zone (up to screw_count).
        Holes confirmed by sniper are counted as already-removed and skipped.
        If fewer screws than configured are found, warn and continue.
        """
        screw_count = step.parameters.get("screw_count", 1)
        zone_name   = step.target

        detected = self._global_screws(zone_name)
        self._log(
            f"  Vision: {len(detected)} screw(s) in global view  "
            f"(config expects {screw_count} in zone '{zone_name}')",
            "info",
        )
        if len(detected) < screw_count:
            self._log(
                f"  [WARN] Only {len(detected)} detected — "
                f"{screw_count - len(detected)} may be already removed or occluded.",
                "warn",
            )

        targets_to_try = detected[:screw_count]
        done, holes, failures = 0, 0, 0

        for i, obj in enumerate(targets_to_try):
            if self._stop_event.is_set():
                return False
            obj_id  = obj.get("id")
            obj_lbl = obj.get("label", "screw")
            self._log(f"  Screw {i+1}/{len(targets_to_try)}  ID={obj_id}  label={obj_lbl}", "head")

            result = self.unscrew_skill.execute_unscrew_command(
                target_id=obj_id,
                target_label=obj_lbl,
                interactive=False,
                target_data_override=copy.deepcopy(obj),
            )

            if result == "HOLE":
                self._log(f"  → ID={obj_id}: sniper confirmed hole — skipping.", "warn")
                holes += 1
            elif result:
                if step.parameters.get("align_only_debug", False):
                    self._log(f"  → ID={obj_id}: XY alignment verified.", "ok")
                elif step.parameters.get("coarse_only_debug", False):
                    self._log(f"  → ID={obj_id}: coarse pose verified.", "ok")
                else:
                    self._log(f"  → ID={obj_id}: extracted.", "ok")
                done += 1
            else:
                self._log(f"  → ID={obj_id}: FAILED.", "fail")
                failures += 1
                # Stop the zone on first failure — arm may need repositioning
                return False

        self._log(
            f"  Zone '{zone_name}': {done} "
            f"{'XY alignments verified' if step.parameters.get('align_only_debug', False) else ('coarse poses verified' if step.parameters.get('coarse_only_debug', False) else 'extracted')}, "
            f"{holes} holes skipped, {failures} failed.",
            "ok" if failures == 0 else "warn",
        )
        return True

    def _run_pickup(self, step) -> bool:
        self.pickup_skill._apply_pickup_config(self.cfg, target_label=step.target)
        return self.pickup_skill.execute_pickup(
            target_id=None, target_label=step.target, interactive=False
        )

    def _run_flip(self, _step) -> bool:
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
        return fn(step)

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
