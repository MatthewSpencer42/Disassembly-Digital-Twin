#!/usr/bin/env python3
"""Deterministic device-config runner for the base disassembly skills."""

from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from disassembly_skill.device_config import DeviceConfig, SequenceStep
from disassembly_skill.object_flip_drop_skill import FlipDropSkill
from disassembly_skill.object_flip_skill import ObjectFlipSkill
from disassembly_skill.object_hold_skill import ObjectHoldSkill
from disassembly_skill.object_pickup_skill import PickupSkill
from disassembly_skill.unscrew_skill import UnscrewSkill


DEFAULT_CONFIG = (
    Path(get_package_share_directory("disassembly_skill"))
    / "config"
    / "device_configs"
    / "hdd.yaml"
)


class ConfigRunner(Node):
    def __init__(self, cfg: DeviceConfig, interactive: bool, start_step: int):
        super().__init__("config_runner")
        self.cfg = cfg
        self.interactive = interactive
        self.start_step = start_step
        self._vision_lock = threading.Lock()
        self._latest_vision: dict = {}
        self.create_subscription(String, "/vision/agent_state", self._vision_cb, 10)

        self.hold_skill = ObjectHoldSkill()
        self.unscrew_skill = UnscrewSkill()
        self.flip_skill = ObjectFlipSkill()
        self.flip_drop_skill = FlipDropSkill()
        self.pickup_skill = PickupSkill()

    @property
    def skill_nodes(self) -> list[Node]:
        return [
            self.hold_skill,
            self.unscrew_skill,
            self.flip_skill,
            self.flip_drop_skill,
            self.pickup_skill,
        ]

    def _vision_cb(self, msg: String) -> None:
        try:
            with self._vision_lock:
                self._latest_vision = json.loads(msg.data)
        except Exception:
            pass

    def _global_screws(self, zone_label: str | None = None) -> list[dict]:
        with self._vision_lock:
            objects = self._latest_vision.get("global_view", {}).get("objects", [])
        screws = [obj for obj in objects if "screw" in obj.get("label", "").lower()]
        if not zone_label:
            return screws
        zone_label = zone_label.lower()
        exact = [obj for obj in screws if obj.get("label", "").lower() == zone_label]
        if exact:
            return exact
        return [obj for obj in screws if zone_label in obj.get("label", "").lower()]

    def _confirm(self, prompt: str) -> str:
        if not self.interactive:
            return "run"
        answer = input(f"{prompt}  [Enter=run, s=skip, q=quit] ").strip().lower()
        if answer == "s":
            return "skip"
        if answer == "q":
            return "quit"
        return "run"

    def _log(self, msg: str) -> None:
        print(f"[config_runner] {msg}", flush=True)

    def _run_hold(self, step: SequenceStep) -> bool:
        return bool(self.hold_skill.execute_hold(part_id=None, target_label=step.target, interactive=False))

    def _run_unscrew(self, step: SequenceStep) -> bool:
        screw_count = int(step.parameters.get("screw_count", 1))
        detected = self._global_screws(step.target)
        self._log(
            f"  Vision: {len(detected)} screw(s) in global view "
            f"(config expects {screw_count} in zone '{step.target}')"
        )
        if not detected:
            self._log("  [WARN] No screws detected for this zone.")
            return False

        for index, obj in enumerate(detected[:screw_count], start=1):
            obj_id = obj.get("id")
            obj_label = obj.get("label", "screw")
            self._log(f"  Screw {index}/{min(len(detected), screw_count)}  ID={obj_id}  label={obj_label}")
            if not self.unscrew_skill.execute_unscrew_command(
                target_id=obj_id,
                target_label=obj_label,
                interactive=False,
            ):
                self._log(f"  -> ID={obj_id}: FAILED.")
                return False
            self._log(f"  -> ID={obj_id}: done.")
        return True

    def _run_pickup(self, step: SequenceStep) -> bool:
        return bool(self.pickup_skill.execute_pickup(target_id=None, target_label=step.target, interactive=False))

    def _run_flip(self, _step: SequenceStep) -> bool:
        return bool(self.flip_skill.execute_flip(interactive=False))

    def _run_flip_drop(self, _step: SequenceStep) -> bool:
        return bool(self.flip_drop_skill.execute_flip_drop(interactive=False))

    def _dispatch(self, step: SequenceStep) -> bool:
        dispatch = {
            "hold": self._run_hold,
            "unscrew": self._run_unscrew,
            "pickup": self._run_pickup,
            "flip": self._run_flip,
            "flip_drop": self._run_flip_drop,
        }
        handler = dispatch.get(step.action.lower())
        if handler is None:
            self._log(f"Unknown action '{step.action}', skipping.")
            return True
        return handler(step)

    def run_sequence(self) -> bool:
        sequence = self.cfg.disassembly_sequence
        total = len(sequence)
        for step in sequence:
            if step.step < self.start_step:
                self._log(f"[skip] step {step.step}: {step.label}")
                continue

            self._log("")
            self._log("=" * 54)
            self._log(f"STEP {step.step}/{total}  {step.action.upper()}  ->  {step.target}")
            self._log(f"  {step.label}")
            self._log("=" * 54)

            decision = self._confirm(f"Step {step.step}/{total}: {step.action} -> {step.target}")
            if decision == "quit":
                return False
            if decision == "skip":
                self._log(f"Step {step.step} skipped by user.")
                continue

            start = time.monotonic()
            success = self._dispatch(step)
            elapsed = time.monotonic() - start
            if not success:
                self._log(f"Step {step.step} FAILED after {elapsed:.1f}s.")
                return False
            self._log(f"Step {step.step} done in {elapsed:.1f}s.")

        self._log("ALL STEPS COMPLETE.")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    param_node = rclpy.create_node("config_runner_params")
    param_node.declare_parameter("config", str(DEFAULT_CONFIG))
    param_node.declare_parameter("interactive", True)
    param_node.declare_parameter("start_step", 1)

    config_path = Path(param_node.get_parameter("config").value)
    interactive = bool(param_node.get_parameter("interactive").value)
    start_step = int(param_node.get_parameter("start_step").value)
    param_node.destroy_node()

    if not config_path.exists():
        print(f"[ERROR] Config file not found: {config_path}", file=sys.stderr)
        rclpy.shutdown()
        raise SystemExit(1)

    try:
        cfg = DeviceConfig.load(config_path)
    except ValueError as exc:
        print(f"[ERROR] Invalid config: {exc}", file=sys.stderr)
        rclpy.shutdown()
        raise SystemExit(1)

    print(f"Loaded: {cfg.device.device_model} - {len(cfg.disassembly_sequence)} steps")
    for warning in cfg.warnings:
        print(f"  [warn] {warning}")

    runner = ConfigRunner(cfg, interactive=interactive, start_step=start_step)
    executor = MultiThreadedExecutor()
    executor.add_node(runner)
    for node in runner.skill_nodes:
        executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)
    try:
        ok = runner.run_sequence()
    finally:
        executor.shutdown(timeout_sec=2.0)
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
