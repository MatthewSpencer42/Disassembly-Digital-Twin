#!/usr/bin/env python3

# -----------------------------------------------------------------------------
# Groq Master Agent — Same ReAct flow as master_agent.py but using Groq's
# free-tier LLM API (OpenAI-compatible chat completions endpoint).
# -----------------------------------------------------------------------------
import asyncio
import inspect
import os
import re
import ast
import time
import json
import threading
import multiprocessing as mp
import math
from typing import List, Tuple, Optional, Dict, Any

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from langgraph.graph import StateGraph, END

from disassembly_skill.unscrew_skill import UnscrewSkill
from disassembly_skill.object_hold_skill import ObjectHoldSkill
from disassembly_skill.object_flip_skill import ObjectFlipSkill
from disassembly_skill.object_flip_drop_skill import FlipDropSkill
from disassembly_skill.object_pickup_skill import PickupSkill

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
try:
    from disassembly_skill.device_config import DeviceConfig
    _DEVICE_CONFIG_AVAILABLE = True
except ImportError:
    _DEVICE_CONFIG_AVAILABLE = False

# -----------------------------------------------------------------------------
# Groq uses OpenAI-compatible API — same SDK, different base_url
# -----------------------------------------------------------------------------
from openai import AsyncOpenAI, OpenAIError

DEBUG_FULL_OUTPUT = False
FORBIDDEN_HEADERS = ("Plan:", "Action:", "Observation:", "Final Answer:")
FIRST_LINE_RE = re.compile(r"([^\r\n]*)")
ACTION_RE = re.compile(r"^Action:\s*([\w_]+)\s*\((.*)\)\s*$")


def _read_api_key(filename: str, env_var: str) -> str:
    env_value = os.getenv(env_var)
    if env_value:
        return env_value.strip()
    candidates = [
        Path(get_package_share_directory("disassembly_skill")) / "config" / filename,
        Path(__file__).resolve().parents[1] / "config" / filename,
    ]
    for path in candidates:
        if path.exists():
            return path.read_text().strip()
    raise RuntimeError(
        f"Missing {filename}. Set {env_var} or place it under disassembly_skill/config."
    )

# -----------------------------------------------------------------------------
# LIVE GUI DASHBOARD PROCESS (WITH COLOR MAPPING)
# -----------------------------------------------------------------------------
def run_live_dashboard(q: mp.Queue):
    """Runs a standalone Tkinter window in a separate process with multi-color highlights."""
    try:
        import tkinter as tk
        from tkinter import scrolledtext
    except ImportError:
        print("Tkinter not installed. Run 'sudo apt-get install python3-tk' for the live window.")
        return

    root = tk.Tk()
    root.title("Groq LangGraph Autonomous Brain Monitor")
    root.geometry("850x650")
    root.configure(bg="#1e1e1e")

    title = tk.Label(root, text="Agent Execution State (Groq)", fg="white", bg="#1e1e1e", font=("Arial", 16, "bold"))
    title.pack(pady=10)

    node_frame = tk.Frame(root, bg="#1e1e1e")
    node_frame.pack(pady=10)

    node_colors = {
        "VISION": "#FFD700",
        "THINK": "#9370DB",
        "PLAN": "#1E90FF",
        "ACTION": "#FFA500",
        "ACT": "#32CD32"
    }

    nodes = ["VISION", "THINK", "PLAN", "ACTION", "ACT"]
    labels = {}

    for n in nodes:
        lbl = tk.Label(node_frame, text=n, width=12, height=2, font=("Arial", 12, "bold"),
                       bg="#333333", fg="gray", relief="ridge", borderwidth=2)
        lbl.pack(side=tk.LEFT, padx=5)
        labels[n] = lbl

    log_area = scrolledtext.ScrolledText(root, wrap=tk.WORD, bg="#000000", font=("Consolas", 11))
    log_area.pack(expand=True, fill='both', padx=20, pady=20)

    for n, hex_color in node_colors.items():
        log_area.tag_config(n, foreground=hex_color)

    log_area.insert(tk.END, "Waiting for vision snapshot and mission trigger...\n\n")
    log_area.see(tk.END)

    def update_gui():
        while not q.empty():
            msg = q.get()
            active_node = msg.get("node", "").upper()
            text = msg.get("text", "")

            for n in nodes:
                labels[n].config(bg="#333333", fg="gray")

            if active_node in labels:
                bg_color = node_colors.get(active_node, "#00AA00")
                fg_color = "black" if active_node == "VISION" else "white"
                labels[active_node].config(bg=bg_color, fg=fg_color)

            if text:
                log_tag = active_node if active_node in node_colors else None
                log_area.insert(tk.END, f"[{active_node}] {text}\n\n", log_tag)
                log_area.see(tk.END)

        root.after(100, update_gui)

    root.after(100, update_gui)
    root.mainloop()

# -----------------------------------------------------------------------------
# Tool Class
# -----------------------------------------------------------------------------
class Tool:
    def __init__(self, fn, description: str, args: Optional[Dict[str, str]] = None):
        self.fn = fn
        self.description = description
        self.args = args or {}

# -----------------------------------------------------------------------------
# Terminal color utilities
# -----------------------------------------------------------------------------
class TColor:
    RESET = "\033[0m"
    REASONING   = "\033[38;5;244m"
    PLAN        = "\033[38;5;39m"
    ACTION      = "\033[38;5;214m"
    OBSERVATION = "\033[38;5;82m"
    FINAL       = "\033[38;5;201m"
    ERROR       = "\033[38;5;196m"
    VISION      = "\033[38;5;226m"

def print_stage(text: str):
    if text.startswith("Reasoning:"): color = TColor.REASONING
    elif text.startswith("Plan:"): color = TColor.PLAN
    elif text.startswith("Action:"): color = TColor.ACTION
    elif text.startswith("Observation:"): color = TColor.OBSERVATION
    elif text.startswith("Final Answer:"): color = TColor.FINAL
    else: color = TColor.ERROR
    print(color + text + TColor.RESET, flush=True)

# -----------------------------------------------------------------------------
# Action parser
# -----------------------------------------------------------------------------
def _const_or_name(node: ast.AST) -> Any:
    if isinstance(node, ast.Constant): return node.value
    if isinstance(node, ast.Name): return node.id
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)) and isinstance(node.operand, ast.Constant):
        return -node.operand.value if isinstance(node.op, ast.USub) else +node.operand.value
    raise ValueError("Unsupported argument expression")

def parse_action(response: str) -> Tuple[Optional[str], List[Any], Dict[str, Any]]:
    m = ACTION_RE.match(response.strip())
    if not m: return None, [], {}
    name, args_str = m.group(1), (m.group(2) or "").strip()
    if args_str == "": return name, [], {}

    # Normalize "key: value" syntax to "key=value" for Python ast parsing
    # Handles patterns like: part_id: 0, label: "HDD_Chassis"
    args_str = re.sub(r'(\w+)\s*:\s*', r'\1=', args_str)

    try:
        node = ast.parse(f"f({args_str})", mode="eval")
        call = node.body
        if not isinstance(call, ast.Call): return name, [], {}
        args, kwargs = [], {}
        for a in call.args:
            try: args.append(_const_or_name(a))
            except ValueError: pass
        for kw in call.keywords:
            if kw.arg is None: continue
            try: kwargs[kw.arg] = _const_or_name(kw.value)
            except ValueError: pass
        return name, args, kwargs
    except SyntaxError: return name, [], {}

# -----------------------------------------------------------------------------
# Agent state
# -----------------------------------------------------------------------------
class AgentState(dict):
    input: str
    history: List[str]
    phase: str

# -----------------------------------------------------------------------------
# Groq Master Agent Node
# -----------------------------------------------------------------------------
class GroqMasterAgentNode(Node):
    def __init__(self):
        super().__init__('groq_master_agent')

        # --- Load ALL device configs from the configs directory ---
        self.all_configs: Dict[str, Any] = {}
        self._device_label_sets: Dict[str, set] = {}
        self.device_cfg = None
        self._identified_device_class: Optional[str] = None

        if _DEVICE_CONFIG_AVAILABLE:
            configs_dir = Path(__file__).parent.parent / 'config' / 'device_configs'
            for yaml_path in sorted(configs_dir.glob('*.yaml')):
                try:
                    cfg = DeviceConfig.load(yaml_path)
                    device_id = cfg.device.device_class
                    self.all_configs[device_id] = cfg
                    labels: set = set()
                    for c in cfg.components:
                        labels.add(c.label.lower())
                    for z in cfg.screw_zones:
                        labels.add(z.zone_name.lower())
                    for s in cfg.disassembly_sequence:
                        if s.target:
                            labels.add(s.target.lower())
                    self._device_label_sets[device_id] = labels
                    self.get_logger().info(
                        f"Loaded config: {device_id} / {cfg.device.device_model} "
                        f"({len(cfg.disassembly_sequence)} steps, {len(labels)} labels)"
                    )
                except Exception as exc:
                    self.get_logger().warning(f"Could not load {yaml_path.name}: {exc}")

        if self.all_configs:
            device_blocks = []
            for device_id, cfg in self.all_configs.items():
                ctx = cfg.to_llm_context()
                removable = [c['label'] for c in ctx['components'] if c.get('removable')]
                fixed = [c['label'] for c in ctx['components'] if not c.get('removable')]
                zones_str = ", ".join(
                    f"{z['zone_name']}({z['screw_count']} screws -> {z['parent_component']})"
                    for z in ctx['screw_zones']
                )
                block = (
                    f"  {device_id}:\n"
                    f"    Removable parts: {removable}\n"
                    f"    Fixed/chassis (DO NOT remove): {fixed if fixed else 'none listed'}\n"
                    f"    Screw zones: {zones_str if zones_str else 'none'}"
                )
                device_blocks.append(block)

            self.MISSION_PROMPT = (
                "You are a robotic disassembly agent. Identify and fully disassemble the device visible in the scene.\n\n"
                "KNOWN DEVICES (use visible object labels to identify which device is present):\n"
                + "\n".join(device_blocks) + "\n\n"
                "INSTRUCTIONS:\n"
                "1. Identify the device by matching the detected object labels to the known device label sets above.\n"
                "2. Each device has a REQUIRED SEQUENCE shown in the think/plan prompts — follow it exactly in order.\n"
                "   Do NOT invent your own order. Do NOT skip steps unless a target is confirmed absent from the scene.\n"
                "3. Execute one action at a time and observe the result before planning the next step.\n\n"
                "RULES:\n"
                "A. Always follow the REQUIRED SEQUENCE for the identified device — never jump ahead.\n"
                "B. A step is 'done' only when its Observation in history explicitly confirms success.\n"
                "C. If a target label is still visible after an attempt, retry that same step before moving on.\n"
                "D. CRITICAL: After a successful unscrew, the freed component may shift and disappear from vision. "
                "   ALWAYS attempt the following pickup step regardless — do NOT declare it 'absent' and skip it.\n"
                "E. Output Final Answer only when all steps are confirmed done in the history."
            )
        else:
            self.MISSION_PROMPT = (
                "Your objective is to fully disassemble the assembly in the current scene. "
                "First, analyze the vision data to deduce which object serves as the primary structural base. "
                "PRIORITY RULE: If any screws are visible, unscrew them all before using any other removal tools. "
                "Next, extract all removable sub-components visible on the current side one by one. "
                "After attempting to remove a macro-component, check the next vision frame — "
                "if the part is still present, retry. "
                "Once the current visible side is fully stripped, flip the object to access the opposite side. "
                "Output Final Answer only when all sides of the primary base are completely empty."
            )

        # START GUI PROCESS
        self.gui_queue = mp.Queue()
        self.gui_process = mp.Process(target=run_live_dashboard, args=(self.gui_queue,))
        self.gui_process.daemon = True
        self.gui_process.start()

        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.loop_thread.start()

        self.vision_lock = threading.Lock()
        self.detected_objects = []
        self.has_printed_startup_vision = False
        self.auto_start_triggered = False

        # Spatial Memory
        self.cleared_zones = []
        self.unscrew_call_counts = {}
        self.EXCLUSION_RADIUS = 0.010

        # --- Groq API Setup ---
        GROQ_API_KEY = _read_api_key("groq_api_key.txt", "GROQ_API_KEY")

        self.GROQ_MODEL = "llama-3.3-70b-versatile"
        self.groq_client = AsyncOpenAI(
            api_key=GROQ_API_KEY,
            base_url="https://api.groq.com/openai/v1"
        )

        self.unscrew_skill = UnscrewSkill(device_cfg=None)
        self.hold_skill = ObjectHoldSkill(device_cfg=None)
        self.flip_skill = ObjectFlipSkill(device_cfg=None)
        self.flip_drop_skill = FlipDropSkill(device_cfg=None)
        self.pickup_skill = PickupSkill(device_cfg=None)

        pickup_targets_all: set = set()
        for cfg in self.all_configs.values():
            for s in cfg.disassembly_sequence:
                if s.action == 'pickup':
                    pickup_targets_all.add(s.target)
        pickup_targets_desc = (
            f"components: {sorted(pickup_targets_all)}"
            if pickup_targets_all else "delicate internal components (PCBs, boards)"
        )

        self.tools: Dict[str, Tool] = {
            "hold_object": Tool(
                self.hold_object,
                "Secures the main device chassis to the table using tactile feedback. Must be done before unscrewing or part extraction. Requires the ID and label of the part to be held.",
                args={"part_id": "int: ID of the holding target", "label": "str: label of the holding target"}
            ),
            "unscrew": Tool(
                self.unscrew,
                "Unthreads a screw. Requires the ID and label of the target screw.",
                args={"unscrew_id": "int: ID of the target screw", "unscrew_label": "str: label of the target screw"}
            ),
            "flip_object": Tool(
                self.flip_object,
                "Flips the main chassis over to expose the back side. Use this only when you need to access parts or screws located on the underside of the device."
            ),
            "pickup_object": Tool(
                self.pickup_object,
                "Uses the precision gripper to safely extract delicate internal components. "
                "Use for extracting " + pickup_targets_desc + ". "
                "MANDATORY RULE: Use this tool for extracting individual removable components that have been unscrewed. "
                "For any other component (lids, frames, modules) use 'flip_drop' for removal instead. "
                "CRITICAL WARNING: Using this tool causes the robot to release the main chassis. "
                "You MUST call the 'hold_object' tool immediately after this action succeeds to re-secure the workspace. "
                "Requires the ID and label of the target object.",
                args={"pickup_id": "int: ID of the target object", "pickup_label": "str: label of the target object"}
            ),
            "flip_drop": Tool(
                self.flip_drop,
                "A clearing action that flips the entire chassis upside down to dump out loose unthreaded screws, junk, or detached lids. WARNING: Ensure no delicate parts like PCBs remain inside before using this. Use this as the final clearing step for a side ONLY AFTER precision parts have been picked up."
            ),
        }

        self.app = self._build_graph()
        self.goal_sub = self.create_subscription(String, '/agent_goal', self.goal_callback, 10)
        self.vision_sub = self.create_subscription(String, '/vision/agent_state', self.vision_callback, 10)

        self.get_logger().info(f"Groq Master Agent Ready (model: {self.GROQ_MODEL}). Awaiting Vision.")

    def _run_async_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _declare_parameter_safe(self, name, default):
        try:
            self.declare_parameter(name, default)
        except Exception:
            pass

    # -------------------------------------------------------------------------
    # Tools implementations
    # -------------------------------------------------------------------------
    def _get_obj_snapshot(self, obj_id):
        """Return a frozen copy of the detected object with the given id, or None."""
        with self.vision_lock:
            for obj in self.detected_objects:
                if obj.get('id') == obj_id:
                    return dict(obj)
        return None

    async def hold_object(self, part_id=None, label=None):
        if part_id is None or label is None:
            return "hold_object failed: missing part_id or label"
        if self.device_cfg is not None:
            self.hold_skill._apply_hold_config(self.device_cfg, target_label=label)
        self.hold_skill.execute_hold(part_id=part_id, target_label=label, interactive=False)
        return f"hold_object succeeded — '{label}' (id={part_id}) secured."

    def _find_exact_visible_target(self, label):
        target = str(label or "").strip().lower()
        with self.vision_lock:
            for obj in self.detected_objects:
                if str(obj.get('label', '')).strip().lower() == target:
                    return dict(obj)
        return None

    def _ensure_hold_before_flip(self) -> tuple[bool, str]:
        if self.flip_skill.is_holding_object or self.hold_skill.is_holding_object:
            self.flip_skill.is_holding_object = True
            return True, "already held"
        if self.device_cfg is None:
            return False, "no device config"
        hold_step = next((s for s in self.device_cfg.disassembly_sequence if s.action == 'hold'), None)
        if hold_step is None or not hold_step.target:
            return False, "no hold step configured"
        obj_data = self._find_exact_visible_target(hold_step.target)
        if obj_data is None:
            return False, f"hold target '{hold_step.target}' not visible"
        self.hold_skill._apply_hold_config(self.device_cfg, target_label=hold_step.target, hold_step=hold_step)
        ok = self.hold_skill.execute_hold(
            part_id=obj_data.get('id'),
            target_label=hold_step.target,
            interactive=False,
            hold_step=hold_step,
            target_data_override=obj_data,
        )
        if not ok:
            return False, "hold failed"
        self.flip_skill.is_holding_object = True
        return True, "hold succeeded"

    async def unscrew(self, unscrew_id=None, unscrew_label=None):
        if unscrew_id is None or unscrew_label is None:
            return "unscrew failed: missing unscrew_id or unscrew_label"
        label_key = str(unscrew_label).lower()
        if self.unscrew_call_counts.get(label_key, 0) >= 2:
            return f"unscrew done — '{unscrew_label}' (id={unscrew_id}) marked complete."
        self.unscrew_call_counts[label_key] = self.unscrew_call_counts.get(label_key, 0) + 1
        obj_data = self._get_obj_snapshot(unscrew_id)
        target_xyz = obj_data.get('xyz') if obj_data else None
        if self.device_cfg is not None:
            self.unscrew_skill._apply_unscrew_config(self.device_cfg, target_label=unscrew_label)
        result = self.unscrew_skill.execute_unscrew_command(
            target_id=unscrew_id,
            target_label=unscrew_label,
            interactive=False,
            target_data_override=obj_data,
        )
        if result is True:
            if target_xyz:
                self.cleared_zones.append(target_xyz)
                print(f"[MEMORY] Exclusion zone added at {target_xyz} (r={self.EXCLUSION_RADIUS*1000:.0f}mm)")
            return f"unscrew succeeded — '{unscrew_label}' (id={unscrew_id}) unthreaded."
        if result == "HOLE":
            return (f"unscrew skipped — '{unscrew_label}' (id={unscrew_id}) was already absent "
                    f"(hole detected). Treat this step as done.")
        return (f"unscrew FAILED — could not extract '{unscrew_label}' (id={unscrew_id}). "
                f"The screw is still present. Retry this step.")

    async def flip_object(self):
        if self.device_cfg is not None:
            self.flip_skill._apply_flip_config(self.device_cfg)
        hold_ok, hold_note = self._ensure_hold_before_flip()
        if not hold_ok:
            return f"flip_object FAILED — could not hold device before flip ({hold_note}). Retry hold_object."
        ok = self.flip_skill.execute_flip(interactive=False)
        if not ok:
            return "flip_object FAILED — could not flip device. Retry."
        return "flip_object succeeded — device flipped to expose opposite side."

    async def flip_drop(self):
        ok = self.flip_drop_skill.execute_flip_drop(interactive=False)
        if not ok:
            return "flip_drop FAILED — could not dump parts. Retry."
        return "flip_drop succeeded — loose parts dumped."

    async def pickup_object(self, pickup_id=None, pickup_label=None):
        if pickup_id is None or pickup_label is None:
            return "pickup_object failed: missing pickup_id or pickup_label"
        obj_data = self._get_obj_snapshot(pickup_id)
        if self.device_cfg is not None:
            self.pickup_skill._apply_pickup_config(self.device_cfg, target_label=pickup_label)
        ok = self.pickup_skill.execute_pickup(
            target_id=pickup_id,
            target_label=pickup_label,
            interactive=False,
            target_snapshot=obj_data,
        )
        if not ok:
            return (f"pickup_object FAILED — could not extract '{pickup_label}' (id={pickup_id}). "
                    f"Retry this step.")
        return f"pickup_object succeeded — '{pickup_label}' (id={pickup_id}) extracted."

    # -------------------------------------------------------------------------
    # Device identification
    # -------------------------------------------------------------------------
    def _identify_device(self, detected_lower: set) -> Optional[str]:
        """Return the device_class with the most label overlap against detected scene labels."""
        best, best_score = None, 0
        for device_id, known_labels in self._device_label_sets.items():
            score = len(detected_lower & known_labels)
            if score > best_score:
                best_score, best = score, device_id
        return best if best_score > 0 else None

    # -------------------------------------------------------------------------
    # Vision handling
    # -------------------------------------------------------------------------
    def vision_callback(self, msg):
        try:
            data = json.loads(msg.data.strip("'"))
            raw_objects = data.get("global_view", {}).get("objects", [])

            # Spatial Memory Filtering
            def _is_screw_or_hole(label: str) -> bool:
                lbl = (label or "").lower()
                return "screw" in lbl or "hole" in lbl

            temp_list = []
            for o in raw_objects:
                obj_id = o.get("id")
                obj_label = o.get("label")
                obj_xyz = o.get("xyz")

                if not obj_xyz or len(obj_xyz) < 3:
                    temp_list.append({"id": obj_id, "label": obj_label, "xyz": obj_xyz})
                    continue

                if not _is_screw_or_hole(obj_label):
                    temp_list.append({"id": obj_id, "label": obj_label, "xyz": obj_xyz})
                    continue

                is_cleared = False
                for cleared_xyz in self.cleared_zones:
                    dist = math.hypot(
                        obj_xyz[0] - cleared_xyz[0],
                        obj_xyz[1] - cleared_xyz[1],
                        obj_xyz[2] - cleared_xyz[2]
                    )
                    if dist < self.EXCLUSION_RADIUS:
                        is_cleared = True
                        break

                if not is_cleared:
                    temp_list.append({"id": obj_id, "label": obj_label, "xyz": obj_xyz})

            with self.vision_lock:
                self.detected_objects = temp_list

                # Identify device from visible labels (done once on first detection)
                if self._identified_device_class is None and self.detected_objects:
                    detected_lower = {o.get('label', '').lower() for o in self.detected_objects}
                    identified = self._identify_device(detected_lower)
                    if identified:
                        self._identified_device_class = identified
                        self.device_cfg = self.all_configs[identified]
                        self.get_logger().info(f"Device identified from vision: {identified}")

                if not self.has_printed_startup_vision and len(self.detected_objects) > 0:
                    vis_text = f"Startup Snapshot ({len(self.detected_objects)} valid objects detected)."
                    print(f"\n{TColor.VISION}Vision: {vis_text}{TColor.RESET}")

                    self.gui_queue.put({"node": "VISION", "text": vis_text})
                    self.has_printed_startup_vision = True

                if not self.auto_start_triggered and len(self.detected_objects) > 0:
                    self.auto_start_triggered = True
                    seq_ctx = self._build_sequence_context()
                    mission = (
                        self.MISSION_PROMPT + "\n\n" + seq_ctx
                        if seq_ctx else self.MISSION_PROMPT
                    )
                    initial_state = {"input": mission, "history": [], "phase": "plan"}
                    asyncio.run_coroutine_threadsafe(self.run_agent(initial_state), self.loop)
        except Exception as exc:
            self.get_logger().warning(f"Vision callback failed: {exc}")

    # -------------------------------------------------------------------------
    # Prompt builders
    # -------------------------------------------------------------------------
    _ACTION_TO_TOOL = {
        'hold':      'hold_object',
        'unscrew':   'unscrew',
        'pickup':    'pickup_object',
        'flip':      'flip_object',
        'flip_drop': 'flip_drop',
    }

    def _build_sequence_context(self) -> str:
        """Return the device's ordered disassembly sequence for injection into every prompt."""
        if self.device_cfg is None:
            return ""
        lines = [
            f"REQUIRED SEQUENCE for {self._identified_device_class} "
            f"({self.device_cfg.device.device_model}) — follow these steps strictly in order:",
        ]
        for i, step in enumerate(self.device_cfg.disassembly_sequence, 1):
            tool = self._ACTION_TO_TOOL.get(step.action, step.action)
            target = step.target or ""
            target_str = f"  target_label=\"{target}\"" if target else ""
            lines.append(f"  Step {i}: {tool}{target_str}  [{step.label}]")
        lines.append(
            "Rules: "
            "(a) Execute the next incomplete step. "
            "(b) Skip a step ONLY if its target label is absent AND the step has NOT yet been attempted. "
            "    Do NOT skip a step simply because its target is absent AFTER the preceding step just succeeded — "
            "    the object may have shifted when its screw was removed. Always attempt pickup after a successful unscrew. "
            "(c) Never jump ahead — complete each step before the next. "
            "(d) A step is DONE only if its Observation in the history confirms success. "
            "    Do NOT re-apply the 'absent = skip' logic to steps that are already confirmed done."
        )
        return "\n".join(lines)

    def format_tool_list(self) -> str:
        lines = []
        for name, tool in self.tools.items():
            if tool.args:
                args_str = ", ".join(f"{arg}: {desc}" for arg, desc in tool.args.items())
                lines.append(f"- {name}({args_str}): {tool.description}")
            else:
                lines.append(f"- {name}(): {tool.description}")
        return "\n".join(lines)

    def build_think_prompt(self, user_input: str, history: List[str]) -> str:
        tool_list = self.format_tool_list()
        with self.vision_lock:
            vision_lines = [
                f"  id={o['id']} label={o['label']} xyz={o.get('xyz', 'unknown')}"
                for o in self.detected_objects
            ]
        vision_str = "\n".join(vision_lines) if vision_lines else "  (no objects detected)"
        device_str = (
            f"Identified device: {self._identified_device_class} ({self.device_cfg.device.device_model})"
            if self.device_cfg else "Device: not yet identified — infer from visible labels."
        )
        seq_ctx = self._build_sequence_context()
        guide_parts = [
            "You are the reasoning stage of a ReAct robotic disassembly agent.",
            "Output MUST start with: Reasoning:",
            "Do NOT output Plan:, Action:, Observation:, or Final Answer: here.",
            "Identify the next INCOMPLETE step from the sequence below and state what must be done.",
            "",
            device_str,
        ]
        if seq_ctx:
            guide_parts.append(seq_ctx)
        guide_parts += [
            "",
            f"Current scene ({len(self.detected_objects)} objects):",
            vision_str,
        ]
        guide = "\n".join(guide_parts)
        return "\n".join([guide, "", "Tools available:", tool_list, "", f"Mission: {user_input}", *history, ""])

    def build_plan_prompt(self, user_input: str, history: List[str]) -> str:
        tool_list = self.format_tool_list()
        seq_ctx = self._build_sequence_context()
        guide_parts = [
            "You are a ReAct agent controlling a robot arm.",
            "Output exactly ONE line.",
            "It MUST start with: Plan:  (or you may output Final Answer: if the task is complete).",
            "",
            "Rules:",
            "- Output exactly ONE line only.",
            "- Do NOT output Action:, Observation:, or Reasoning: here.",
            "- Plan the next step from the REQUIRED SEQUENCE that has not yet been completed.",
            "- Never skip ahead in the sequence.",
            "",
        ]
        if seq_ctx:
            guide_parts.append(seq_ctx)
            guide_parts.append("")
        guide = "\n".join(guide_parts)
        return "\n".join([guide, "Tools available:", tool_list, "", f"User: {user_input}", *history, ""])

    def build_action_prompt(self, user_input: str, history: List[str]) -> str:
        tool_list = self.format_tool_list()
        seq_ctx = self._build_sequence_context()
        guide_parts = [
            "You are a ReAct agent controlling a robot arm.",
            "Output exactly ONE line.",
            "It MUST start with: Action:",
            "",
            "Rules:",
            "- Output exactly ONE line only.",
            "- Do NOT output Plan:, Observation:, Reasoning:, or Final Answer: here.",
            "- Use ONLY the tool names/signatures exactly as listed.",
            "- Do not invent extra keyword arguments.",
            "- Format: Action: tool_name(arg_name=value, ...)",
            "- Example: Action: unscrew(unscrew_id=1, unscrew_label=\"pcb_screw\")",
            "- Use the target_label exactly as shown in the REQUIRED SEQUENCE.",
            "",
        ]
        if seq_ctx:
            guide_parts.append(seq_ctx)
            guide_parts.append("")
        guide = "\n".join(guide_parts)
        return "\n".join([guide, "Tools available:", tool_list, "", f"User: {user_input}", *history, ""])

    # -------------------------------------------------------------------------
    # Groq LLM calls (chat completions API)
    # -------------------------------------------------------------------------
    async def call_llm_first_line(self, prompt: str, max_tokens: int = 200, timeout_s: int = 60) -> str:
        try:
            resp = await asyncio.wait_for(
                self.groq_client.chat.completions.create(
                    model=self.GROQ_MODEL,
                    messages=[{"role": "user", "content": prompt}],
                    max_tokens=max_tokens,
                    temperature=0.1,
                ), timeout=timeout_s)
            text = resp.choices[0].message.content or ""
            text = text.strip()
            if not text: return ""
            m = FIRST_LINE_RE.match(text)
            return m.group(1).strip() if m else text.splitlines()[0].strip()
        except (asyncio.TimeoutError, OpenAIError) as e:
            return f"Plan: (API error: {type(e).__name__}) proceed cautiously."

    async def call_llm_full(self, prompt: str, max_tokens: int = 800, timeout_s: int = 60) -> str:
        try:
            resp = await asyncio.wait_for(
                self.groq_client.chat.completions.create(
                    model=self.GROQ_MODEL,
                    messages=[{"role": "user", "content": prompt}],
                    max_tokens=max_tokens,
                    temperature=0.1,
                ), timeout=timeout_s)
            text = resp.choices[0].message.content or ""
            return text.strip()
        except (asyncio.TimeoutError, OpenAIError) as e:
            return f"Reasoning: (API error: {type(e).__name__})"

    # -------------------------------------------------------------------------
    # Reasoning sanitizer
    # -------------------------------------------------------------------------
    def sanitize_reasoning(self, txt: str) -> str:
        txt = (txt or "").strip()
        if not txt.startswith("Reasoning:"): txt = "Reasoning:\n" + txt
        lines = txt.splitlines()
        out = [lines[0]]
        for line in lines[1:]:
            if any(line.strip().startswith(h) for h in FORBIDDEN_HEADERS): continue
            out.append(line)
        return "\n".join(out).strip()

    # -------------------------------------------------------------------------
    # LangGraph nodes
    # -------------------------------------------------------------------------
    async def think(self, state: AgentState) -> AgentState:
        txt = await self.call_llm_full(self.build_think_prompt(state["input"], state["history"]), max_tokens=800)
        txt = self.sanitize_reasoning(txt)
        state["history"].append(txt)
        print_stage(txt)
        return state

    async def plan_node(self, state: AgentState) -> AgentState:
        line = await self.call_llm_first_line(self.build_plan_prompt(state["input"], state["history"]), max_tokens=120)
        if line.startswith("Final Answer:"):
            state["history"].append(line); print_stage(line); return state
        if not line.startswith("Plan:"): line = "Plan: Prepare next simple step"
        state["history"].append(line); print_stage(line); state["phase"] = "action"
        return state

    async def action_node(self, state: AgentState) -> AgentState:
        if state["history"] and state["history"][-1].startswith("Final Answer:"): return state
        line = await self.call_llm_first_line(self.build_action_prompt(state["input"], state["history"]), max_tokens=120)
        if not line.startswith("Action:"):
            state["history"].append("Observation: No valid action produced. Re-evaluating.")
            print_stage("Observation: No valid action produced. Re-evaluating.")
            return state
        # Validate tool name before accepting
        name, _, _ = parse_action(line)
        if name and name not in self.tools:
            state["history"].append(f"Observation: Unknown tool '{name}'. Available: {', '.join(self.tools.keys())}")
            print_stage(f"Observation: Unknown tool '{name}'. Use one of: {', '.join(self.tools.keys())}")
            return state
        state["history"].append(line); print_stage(line)
        return state

    async def act(self, state: AgentState) -> AgentState:
        if not state["history"]: return state
        last = state["history"][-1]
        if last.startswith("Final Answer:"): return state
        if last.startswith("Action:"):
            name, args, kwargs = parse_action(last)
            tool = self.tools.get(name or "")
            if not tool: obs = f"Observation: Unknown tool '{name}'."
            else:
                try:
                    if inspect.iscoroutinefunction(tool.fn): result = await tool.fn(*args, **kwargs)
                    else: result = tool.fn(*args, **kwargs)
                    obs = f"Observation: {result}"
                except Exception as e: obs = f"Observation: Tool errored: {e}"
            state["history"].append(obs); print_stage(obs)
            self.gui_queue.put({"node": "ACT", "text": obs})
        return state

    # -------------------------------------------------------------------------
    # Runner & Graph Compile
    # -------------------------------------------------------------------------
    def _build_graph(self):
        graph = StateGraph(AgentState)
        graph.add_node("think", self.think)
        graph.add_node("plan", self.plan_node)
        graph.add_node("action", self.action_node)
        graph.add_node("act", self.act)
        graph.set_entry_point("think")
        graph.add_edge("think", "plan")
        graph.add_edge("plan", "action")
        graph.add_edge("action", "act")
        def _branch(s: AgentState) -> str:
            return "end" if any(line.startswith("Final Answer:") for line in s["history"]) else "continue"
        graph.add_conditional_edges("act", _branch, {"continue": "think", "end": END})

        app = graph.compile()
        try:
            with open("/tmp/disassembly_groq_master_agent_architecture.md", "w") as f:
                f.write("```mermaid\n")
                f.write(app.get_graph().draw_mermaid())
                f.write("\n```")
        except Exception as exc:
            self.get_logger().warning(f"Could not write agent architecture graph: {exc}")
        return app

    async def run_agent(self, state: AgentState):
        print("PROMPT:", state["input"], flush=True)
        self.gui_queue.put({"node": "VISION", "text": f"Mission Started: {state['input']}"})

        async for event in self.app.astream(state, config={"recursion_limit": 400}):
            current_node = list(event.keys())[0]
            latest_history = state["history"][-1] if state["history"] else ""
            self.gui_queue.put({"node": current_node, "text": latest_history})

        return state["history"]

    def goal_callback(self, msg):
        initial_state = {"input": msg.data, "history": [], "phase": "plan"}
        asyncio.run_coroutine_threadsafe(self.run_agent(initial_state), self.loop)

# -----------------------------------------------------------------------------
# Script entrypoint
# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = GroqMasterAgentNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)
    executor.add_node(node.unscrew_skill)
    executor.add_node(node.hold_skill)
    executor.add_node(node.flip_skill)
    executor.add_node(node.flip_drop_skill)
    executor.add_node(node.pickup_skill)

    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
