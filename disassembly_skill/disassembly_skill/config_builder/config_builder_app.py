"""Device Config Builder — dark card UI matching device_config_builder.jsx design."""

from __future__ import annotations

import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any, Dict, List, Optional

from disassembly_skill.config_builder.models.config_model import (
    DeviceConfigDocument,
    default_step_params,
)
from disassembly_skill.config_builder.utils.constants import (
    APPROACH_AXES,
    BIT_TYPES,
    COMPONENT_TYPES,
    FIXTURING_TYPES,
    GRASP_TYPES,
    MATERIAL_TYPES,
    RG6_MAX_FORCE_N,
    RG6_MAX_GRIP_MM,
    SCREW_TYPES,
    STRATEGY_TYPES,
    TOOL_CHANGE_METHODS,
)
from disassembly_skill.config_builder.utils.validation import validate_config
from disassembly_skill.config_builder.utils.yaml_export import (
    load_project,
    load_yaml,
    save_project,
    save_yaml,
)

# Default config directory (resolved relative to this file)
_CONFIG_DIR = (
    Path(__file__).resolve().parents[4] / "config" / "device_configs"
)

# ── Colour palette ────────────────────────────────────────────────────────────
_BG = "#0a0f1a"
_SURFACE = "#111827"
_SURFACE2 = "#1a2235"
_BORDER = "#1e2a3a"
_ACCENT = "#3b82f6"
_TEXT = "#e2e8f0"
_TEXT_DIM = "#64748b"
_WHITE = "#ffffff"
_DANGER = "#ef4444"
_SUCCESS = "#22c55e"
_WARN = "#f59e0b"
_INFO_ICON_FG = "#3b82f6"

ACTION_STYLE: Dict[str, Dict[str, str]] = {
    "hold":        {"bg": "#1e3a5f", "fg": "#60a5fa"},
    "unscrew":     {"bg": "#3d2800", "fg": "#fbbf24"},
    "pickup":      {"bg": "#0a2d14", "fg": "#4ade80"},
    "flip":        {"bg": "#2d1040", "fg": "#c084fc"},
    "flip_drop":   {"bg": "#2d1029", "fg": "#f472b6"},
    "tool_change": {"bg": "#0a2d2d", "fg": "#2dd4bf"},
}
ACTION_ORDER = ("hold", "unscrew", "pickup", "flip", "flip_drop", "tool_change")

_MONO    = ("Courier", 10)
_MONO_SM = ("Courier", 9)
_MONO_LG = ("Courier", 11, "bold")
_LBL     = ("Courier", 9, "bold")


# ── Tooltip texts ─────────────────────────────────────────────────────────────

_TT_FIXTURING = {
    "gripper_only":    "Device held only by the robot gripper. No external fixture.",
    "passive_jig":     "Device rests in a passive jig/tray for stability.",
    "vice":            "Device clamped in a bench vice.",
    "soft_jaw_vice":   "Vice with soft jaws to protect delicate surfaces.",
    "tray":            "Device placed in a tray, gravity-stabilised.",
    "custom_fixture":  "Custom-built fixture specific to this device.",
    "none":            "No fixturing. Device placed on a flat surface.",
}

_TT_MATERIAL = {
    "aluminium": "Aluminium or alloy housing.",
    "plastic":   "Plastic housing.",
    "steel":     "Steel housing.",
    "mixed":     "Multiple materials (e.g. aluminium lid, plastic base).",
    "unknown":   "Material not yet determined.",
}

_TT_STRATEGY = {
    "lateral_clamp":  "Gripper approaches from the side and clamps both faces along the width axis.",
    "top_down_clamp": "Gripper descends from above and presses down on the device surface.",
    "edge_clamp":     "Gripper grips a specific edge or lip of the device.",
    "fixture_press":  "Gripper positions over a fixture but does not fully close (device already constrained).",
}

# Per-parameter tooltip text
_TT: Dict[str, str] = {
    # Hold
    "strategy":              "How the gripper contacts the device. Choose based on accessible surfaces.",
    "grip_width_mm":         "Distance between gripper fingers when closed (mm). RG6 max = 160 mm.",
    "approach_axis":         "Direction the gripper descends toward the device. +z = top-down.",
    "gripper_open_deg":      "Gripper jaw angle before approach (°). Positive = open.",
    "gripper_close_deg":     "Gripper jaw angle when holding (°). Negative = closed on RG6 convention.",
    "gripper_close_force_n": "Clamping / holding force in Newtons. RG6 maximum = 120 N.",
    "torque_threshold_nm":   "Joint torque threshold to detect contact or slippage (Nm).",
    "descent_speed_mps":     "Speed of descent toward the device surface (m/s).",
    "tilt_deg":              "Wrist tilt angle for better surface contact (°).",
    "hover_x_offset_m":      "X offset from vision centroid to final hover position (m).",
    "hover_y_offset_m":      "Y offset from vision centroid to final hover position (m).",
    # Unscrew
    "screw_type":            "Driver bit profile. Must match the actual screw head (torx, phillips, hex…).",
    "screw_count":           "Number of screws in this zone. The skill iterates over each one.",
    "engagement_depth_mm":   "How far the bit descends into the screw head before rotating (mm).",
    "initial_torque_nm":     "Starting torque for bit engagement before full rotation (Nm).",
    "max_torque_nm":         "Maximum allowed torque before the skill backs off (Nm).",
    "rotation_speed_rpm":    "Screwdriver rotation speed (RPM). Lower = better for fragile heads.",
    "force_threshold_n":     "Axial force below which the screw is considered fully released (N).",
    "align_tolerance_px":    "Pixel tolerance for screw hole centring via vision (px).",
    "spiral_timeout_s":      "Timeout for spiral search if the screw hole is not found (s).",
    # Pickup
    "grasp_type":            "Grip geometry. parallel=flat surfaces, precision=small/fragile, pinch=thin edges, wide=large span.",
    "lift_height_mm":        "Height to lift the component above its resting position (mm).",
    "lift_speed_mps":        "Upward lift speed (m/s). Slower for fragile or cable-connected parts.",
    "approach_x_offset_m":   "X offset from component centroid for the grasp approach (m).",
    "approach_y_offset_m":   "Y offset from component centroid for the grasp approach (m).",
    "drop_x":                "Drop zone X position in base_link frame (m).",
    "drop_y":                "Drop zone Y position in base_link frame (m).",
    "drop_z":                "Drop zone Z position in base_link frame (m).",
    # Flip
    "retract_height_m":      "Height to retract before rotating / moving (m). Must clear surrounding structure.",
    "rotation_deg":          "Rotation angle in degrees. Typically 180 to invert the device.",
    # Flip-drop
    "intermediate_x":        "Waypoint X over the bin before inversion (m, base_link frame).",
    "intermediate_y":        "Waypoint Y over the bin before inversion (m, base_link frame).",
    "intermediate_z":        "Waypoint Z height over the bin (m, base_link frame).",
    "purpose":               "Human-readable label for why this flip-drop is performed.",
    # Tool change
    "required_bit":          "Screwdriver bit that must be installed before the next unscrew step.",
    "method":                "manual = operator swaps bit. automatic_future = reserved for ATC hardware.",
}

_TT_FIELD = {
    "fixturing": (
        "How the device is constrained during disassembly.\n"
        "gripper_only — held only by the robot.\n"
        "passive_jig — rests in a static holder.\n"
        "vice / soft_jaw_vice — clamped externally.\n"
        "tray — gravity-stabilised in a tray."
    ),
    "material": (
        "Dominant housing material.\n"
        "Affects grip force limits and surface fragility."
    ),
    "approach_axis": (
        "Cartesian axis along which the robot approaches the target.\n"
        "+z = top-down,  +y = from the front,  +x = from the left."
    ),
    "strategy": (
        "Gripper contact strategy for the HOLD action:\n"
        "lateral_clamp — grips both side faces.\n"
        "top_down_clamp — presses down from above.\n"
        "edge_clamp — grips a specific edge.\n"
        "fixture_press — holds position without closing fully."
    ),
    "grasp_type": (
        "Gripper geometry for the PICKUP action:\n"
        "parallel — flat opposing faces.\n"
        "precision — small / fragile part.\n"
        "pinch — thin edge.\n"
        "wide — large span component."
    ),
    "required_bit": (
        "Screwdriver bit code to install before the next unscrew step.\n"
        "T8 = Torx 8,  PH1 = Phillips 1,  H2.5 = Hex 2.5 mm, etc.\n"
        "Tool change is currently manual — operator performs the swap."
    ),
}


# ── Tooltip widget ────────────────────────────────────────────────────────────

class _Tooltip:
    """Lightweight hover tooltip for any Tkinter widget."""

    def __init__(self, widget: tk.Widget, text: str) -> None:
        self._widget = widget
        self._text = text
        self._win: Optional[tk.Toplevel] = None
        widget.bind("<Enter>", self._show, add="+")
        widget.bind("<Leave>", self._hide, add="+")
        widget.bind("<ButtonPress>", self._hide, add="+")

    def _show(self, event: tk.Event) -> None:
        if self._win or not self._text:
            return
        x = self._widget.winfo_rootx() + 20
        y = self._widget.winfo_rooty() + self._widget.winfo_height() + 4
        self._win = tw = tk.Toplevel(self._widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        tw.wm_attributes("-topmost", True)
        tk.Label(
            tw, text=self._text, justify="left",
            bg=_SURFACE2, fg=_TEXT,
            font=("Courier", 9),
            relief="flat", padx=10, pady=8,
            wraplength=320,
            highlightbackground=_ACCENT,
            highlightthickness=1,
        ).pack()

    def _hide(self, event: tk.Event = None) -> None:
        if self._win:
            self._win.destroy()
            self._win = None


# ── Widget helpers ────────────────────────────────────────────────────────────

def _frame(parent: tk.Widget, bg: str = _SURFACE, **kw) -> tk.Frame:
    return tk.Frame(parent, bg=bg, **kw)


def _label(parent: tk.Widget, text: str, fg: str = _TEXT_DIM,
           bg: str = _SURFACE, font=_LBL, **kw) -> tk.Label:
    return tk.Label(parent, text=text, fg=fg, bg=bg, font=font, **kw)


def _entry(parent: tk.Widget, var: tk.Variable, width: int = 20,
           bg: str = _BG, fg: str = _TEXT) -> tk.Entry:
    return tk.Entry(
        parent, textvariable=var, width=width,
        bg=bg, fg=fg, insertbackground=fg,
        relief="flat", font=_MONO,
        highlightbackground=_BORDER,
        highlightcolor=_ACCENT,
        highlightthickness=1,
    )


def _combo(parent: tk.Widget, var: tk.StringVar, values: tuple,
           width: int = 14) -> ttk.Combobox:
    return ttk.Combobox(
        parent, textvariable=var, values=list(values),
        width=width, font=_MONO, state="readonly",
    )


def _btn(parent: tk.Widget, text: str, cmd, bg: str = _ACCENT,
         fg: str = _WHITE, font=_LBL, **kw) -> tk.Button:
    return tk.Button(
        parent, text=text, command=cmd,
        bg=bg, fg=fg, relief="flat",
        activebackground=_SURFACE2, activeforeground=fg,
        font=font, padx=8, pady=4, cursor="hand2", **kw,
    )


def _info(parent: tk.Widget, tooltip: str, bg: str = _SURFACE) -> tk.Label:
    """Small ⓘ label that shows a tooltip on hover."""
    lbl = tk.Label(
        parent, text=" ⓘ", fg=_INFO_ICON_FG, bg=bg,
        font=("Courier", 9), cursor="question_arrow",
    )
    _Tooltip(lbl, tooltip)
    return lbl


def _section_hdr(parent: tk.Widget, text: str) -> None:
    tk.Label(
        parent, text=text, fg=_TEXT_DIM, bg=_SURFACE,
        font=("Courier", 9, "bold"), anchor="w",
    ).pack(fill="x", pady=(0, 8))


def _divider(parent: tk.Widget) -> None:
    tk.Frame(parent, bg=_BORDER, height=1).pack(fill="x", pady=10)


def _apply_ttk_style() -> None:
    s = ttk.Style()
    s.theme_use("default")
    for name in ("TCombobox",):
        s.configure(name,
                    fieldbackground=_BG, background=_BG,
                    foreground=_TEXT, selectbackground=_SURFACE2,
                    bordercolor=_BORDER, arrowcolor=_TEXT_DIM, font=_MONO)
        s.map(name,
              fieldbackground=[("readonly", _BG)],
              background=[("readonly", _BG)],
              foreground=[("readonly", _TEXT)])
    s.configure("Vertical.TScrollbar",
                background=_SURFACE, troughcolor=_BG,
                arrowcolor=_TEXT_DIM, bordercolor=_BORDER)
    s.configure("Horizontal.TScrollbar",
                background=_SURFACE, troughcolor=_BG,
                arrowcolor=_TEXT_DIM, bordercolor=_BORDER)


# ══════════════════════════════════════════════════════════════════════════════
class ConfigBuilderApp:
    """Three-panel config builder: sidebar | step cards | YAML preview."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.doc = DeviceConfigDocument()
        self._expanded: Dict[int, bool] = {}
        self._yaml_after_id: Optional[str] = None

        root.title("Device Config Builder")
        root.configure(bg=_BG)
        root.geometry("1560x960")
        root.minsize(1100, 640)

        _apply_ttk_style()
        self._build_ui()
        self._rebuild_all()

    # ── UI skeleton ────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self._build_header(self.root)

        body = _frame(self.root, bg=_BG)
        body.pack(fill="both", expand=True)
        body.grid_columnconfigure(1, weight=1)
        body.grid_rowconfigure(0, weight=1)

        # Sidebar
        sb_outer = tk.Frame(body, bg=_SURFACE,
                             highlightbackground=_BORDER, highlightthickness=1)
        sb_outer.grid(row=0, column=0, sticky="nsew")
        sb_c = tk.Canvas(sb_outer, bg=_SURFACE, highlightthickness=0, width=272)
        sb_s = ttk.Scrollbar(sb_outer, orient="vertical", command=sb_c.yview)
        self._sb_inner = _frame(sb_c, bg=_SURFACE)
        sb_c.create_window((0, 0), window=self._sb_inner, anchor="nw", tags="sbw")
        sb_c.configure(yscrollcommand=sb_s.set)
        self._sb_inner.bind(
            "<Configure>",
            lambda e: sb_c.configure(scrollregion=sb_c.bbox("all")))
        sb_c.bind("<Configure>",
                  lambda e: sb_c.itemconfig("sbw", width=e.width))
        sb_s.pack(side="right", fill="y")
        sb_c.pack(side="left", fill="both", expand=True)
        self._build_sidebar(self._sb_inner)

        # Center
        center = _frame(body, bg=_BG)
        center.grid(row=0, column=1, sticky="nsew")
        self._build_center(center)

        # YAML preview
        pv_outer = tk.Frame(body, bg=_SURFACE,
                             highlightbackground=_BORDER, highlightthickness=1)
        pv_outer.grid(row=0, column=2, sticky="nsew")
        self._build_yaml_preview(pv_outer)

    def _build_header(self, parent: tk.Widget) -> None:
        hdr = tk.Frame(parent, bg=_SURFACE,
                        highlightbackground=_BORDER, highlightthickness=1)
        hdr.pack(fill="x")
        left = _frame(hdr, bg=_SURFACE)
        left.pack(side="left", padx=20, pady=14)
        tk.Label(left, text="Device Config Builder",
                 fg=_WHITE, bg=_SURFACE,
                 font=("Courier", 14, "bold")).pack(anchor="w")
        tk.Label(left, text="DISASSEMBLY SKILL  /  CONFIG EDITOR",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_MONO_SM).pack(anchor="w")

        right = _frame(hdr, bg=_SURFACE)
        right.pack(side="right", padx=20, pady=14)
        for txt, cmd, bg, fg in [
            ("New",            self._new_config,   _SURFACE2, _TEXT_DIM),
            ("Open",           self._open_project,  _SURFACE2, _TEXT_DIM),
            ("Save  Ctrl+S",   self._save_project,  _SURFACE2, _ACCENT),
            ("Export YAML  Ctrl+E", self._export_yaml, _ACCENT, _WHITE),
        ]:
            _btn(right, txt, cmd, bg=bg, fg=fg).pack(side="left", padx=4)

        self.root.bind("<Control-s>", lambda e: self._save_project())
        self.root.bind("<Control-e>", lambda e: self._export_yaml())

    # ── Sidebar ────────────────────────────────────────────────────────────

    def _build_sidebar(self, parent: tk.Widget) -> None:
        P = dict(padx=16, pady=10)

        # ── DEVICE ──────────────────────────────────────────────────────────
        sec = _frame(parent, bg=_SURFACE)
        sec.pack(fill="x", **P)
        _section_hdr(sec, "DEVICE")

        self._sv = {}   # StringVars keyed by field name
        for key, lbl in [("class", "Device Class"),
                          ("model", "Model / Label")]:
            self._sv[key] = tk.StringVar()
            row = _frame(sec, bg=_SURFACE)
            row.pack(fill="x", pady=(0, 8))
            tk.Label(row, text=lbl, fg=_TEXT_DIM, bg=_SURFACE,
                     font=_LBL).pack(side="left", anchor="w")
            _entry(sec, self._sv[key]).pack(fill="x", pady=(0, 4))
            self._sv[key].trace_add("write", lambda *_: self._schedule_yaml())

        # Material + Fixturing (with info icons)
        row2 = _frame(sec, bg=_SURFACE)
        row2.pack(fill="x", pady=(0, 8))
        for key, lbl, vals, tt_map in [
            ("material",  "Material",  MATERIAL_TYPES,  _TT_MATERIAL),
            ("fixturing", "Fixturing", FIXTURING_TYPES, _TT_FIXTURING),
        ]:
            self._sv[key] = tk.StringVar()
            col = _frame(row2, bg=_SURFACE)
            col.pack(side="left", expand=True, fill="x", padx=(0, 8))
            hrow = _frame(col, bg=_SURFACE)
            hrow.pack(fill="x")
            tk.Label(hrow, text=lbl, fg=_TEXT_DIM, bg=_SURFACE,
                     font=_LBL).pack(side="left")
            _info(hrow,
                  _TT_FIELD.get(key, "") + "\n\n" +
                  "\n".join(f"  {k}: {v}" for k, v in tt_map.items()),
                  bg=_SURFACE).pack(side="left")
            cb = _combo(col, self._sv[key], vals, width=13)
            cb.pack(fill="x", pady=(2, 0))
            self._sv[key].trace_add("write", lambda *_: self._schedule_yaml())

        # Dimensions
        self._sv["dim_l"] = tk.StringVar()
        self._sv["dim_w"] = tk.StringVar()
        self._sv["dim_h"] = tk.StringVar()
        dim_row = _frame(sec, bg=_SURFACE)
        dim_row.pack(fill="x", pady=(0, 8))
        for key, lbl in [("dim_l", "L mm"), ("dim_w", "W mm"), ("dim_h", "H mm")]:
            col = _frame(dim_row, bg=_SURFACE)
            col.pack(side="left", expand=True, fill="x", padx=(0, 6))
            tk.Label(col, text=lbl, fg=_TEXT_DIM, bg=_SURFACE,
                     font=_LBL, anchor="w").pack(fill="x")
            _entry(col, self._sv[key], width=7).pack(fill="x")
            self._sv[key].trace_add("write", lambda *_: self._schedule_yaml())

        # Notes
        self._sv["device_notes"] = tk.StringVar()
        tk.Label(sec, text="Notes", fg=_TEXT_DIM, bg=_SURFACE,
                 font=_LBL, anchor="w").pack(fill="x")
        _entry(sec, self._sv["device_notes"]).pack(fill="x")
        self._sv["device_notes"].trace_add("write",
                                            lambda *_: self._schedule_yaml())

        _divider(parent)

        # ── COMPONENTS ──────────────────────────────────────────────────────
        cs = _frame(parent, bg=_SURFACE)
        cs.pack(fill="x", **P)
        ch = _frame(cs, bg=_SURFACE)
        ch.pack(fill="x")
        _section_hdr(ch, "COMPONENTS")
        _btn(ch, "+ Add", self._add_component,
             bg=_SURFACE2, fg=_ACCENT,
             font=_LBL).pack(side="right", anchor="ne")
        self._comp_frame = _frame(cs, bg=_SURFACE)
        self._comp_frame.pack(fill="x")

        _divider(parent)

        # ── SCREW ZONES ─────────────────────────────────────────────────────
        zs = _frame(parent, bg=_SURFACE)
        zs.pack(fill="x", **P)
        zh = _frame(zs, bg=_SURFACE)
        zh.pack(fill="x")
        _section_hdr(zh, "SCREW ZONES")
        _btn(zh, "+ Add", self._add_zone,
             bg=_SURFACE2, fg=_ACCENT,
             font=_LBL).pack(side="right", anchor="ne")
        self._zone_frame = _frame(zs, bg=_SURFACE)
        self._zone_frame.pack(fill="x")

        _divider(parent)

        # ── ADD STEP ────────────────────────────────────────────────────────
        as_ = _frame(parent, bg=_SURFACE)
        as_.pack(fill="x", **P)
        _section_hdr(as_, "ADD STEP")
        for action in ACTION_ORDER:
            st = ACTION_STYLE[action]
            _btn(as_,
                 f"+ {action.upper().replace('_', ' ')}",
                 lambda a=action: self._add_step(a),
                 bg=st["bg"], fg=st["fg"],
                 font=("Courier", 9, "bold")).pack(fill="x", pady=3)

        _divider(parent)

        # ── SEQUENCE LIST ────────────────────────────────────────────────────
        sl = _frame(parent, bg=_SURFACE)
        sl.pack(fill="x", **P)
        _section_hdr(sl, "SEQUENCE")
        self._seq_list_frame = _frame(sl, bg=_SURFACE)
        self._seq_list_frame.pack(fill="x")

        _divider(parent)

        # ── WARNINGS ────────────────────────────────────────────────────────
        ws = _frame(parent, bg=_SURFACE)
        ws.pack(fill="x", padx=16, pady=(0, 16))
        _section_hdr(ws, "WARNINGS")
        self._warn_frame = _frame(ws, bg=_SURFACE)
        self._warn_frame.pack(fill="x")

    # ── Center (scrollable cards) ──────────────────────────────────────────

    def _build_center(self, parent: tk.Widget) -> None:
        top = _frame(parent, bg=_BG)
        top.pack(fill="x", padx=20, pady=(14, 4))
        self._step_count_lbl = tk.Label(top, text="0 steps",
                                         fg=_TEXT_DIM, bg=_BG, font=_MONO_SM)
        self._step_count_lbl.pack(side="left")

        cf = _frame(parent, bg=_BG)
        cf.pack(fill="both", expand=True)

        self._cc = tk.Canvas(cf, bg=_BG, highlightthickness=0)
        sc = ttk.Scrollbar(cf, orient="vertical", command=self._cc.yview)
        self.cards_frame = _frame(self._cc, bg=_BG)
        self._cc.configure(yscrollcommand=sc.set)
        self._cc.create_window((0, 0), window=self.cards_frame,
                                anchor="nw", tags="cw")
        self.cards_frame.bind(
            "<Configure>",
            lambda e: self._cc.configure(scrollregion=self._cc.bbox("all")))
        self._cc.bind(
            "<Configure>",
            lambda e: self._cc.itemconfig("cw", width=e.width))
        for w in (self._cc, self.cards_frame):
            w.bind("<MouseWheel>",
                   lambda e: self._cc.yview_scroll(
                       -1 * (e.delta // 120), "units"))
        sc.pack(side="right", fill="y")
        self._cc.pack(side="left", fill="both", expand=True,
                       padx=(20, 0), pady=(0, 16))

    def _build_yaml_preview(self, parent: tk.Widget) -> None:
        hdr = _frame(parent, bg=_SURFACE)
        hdr.pack(fill="x")
        tk.Label(hdr, text="YAML PREVIEW",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_LBL).pack(
            side="left", padx=14, pady=10)
        _btn(hdr, "Copy", self._copy_yaml,
             bg=_SURFACE2, fg=_ACCENT,
             font=_LBL).pack(side="right", padx=10, pady=8)

        self._yaml_text = tk.Text(
            parent, bg=_BG, fg="#93c5fd",
            insertbackground=_TEXT, font=("Courier", 10),
            relief="flat", state="disabled",
            wrap="none", width=48,
            highlightthickness=0, padx=14, pady=12,
        )
        ys = ttk.Scrollbar(parent, orient="vertical",
                            command=self._yaml_text.yview)
        xs = ttk.Scrollbar(parent, orient="horizontal",
                            command=self._yaml_text.xview)
        self._yaml_text.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        xs.pack(side="bottom", fill="x")
        ys.pack(side="right", fill="y")
        self._yaml_text.pack(fill="both", expand=True)

    # ── Rebuild helpers ────────────────────────────────────────────────────

    def _rebuild_all(self) -> None:
        self._load_device_vars()
        self._rebuild_comp_list()
        self._rebuild_zone_list()
        self._rebuild_cards()
        self._rebuild_seq_list()
        self._refresh_yaml()
        self._refresh_warnings()

    def _load_device_vars(self) -> None:
        dev = self.doc.data.get("device", {})
        dims = dev.get("dimensions_mm", {})
        self._sv["class"].set(dev.get("class", ""))
        self._sv["model"].set(dev.get("model", ""))
        self._sv["material"].set(dev.get("material", "mixed"))
        self._sv["fixturing"].set(dev.get("fixturing", "gripper_only"))
        self._sv["dim_l"].set(str(dims.get("length", "")))
        self._sv["dim_w"].set(str(dims.get("width", "")))
        self._sv["dim_h"].set(str(dims.get("height", "")))
        self._sv["device_notes"].set(dev.get("notes", ""))

    def _sync_device(self) -> None:
        dev = self.doc.data.setdefault("device", {})
        dev["class"]    = self._sv["class"].get().strip()
        dev["model"]    = self._sv["model"].get().strip()
        dev["material"] = self._sv["material"].get()
        dev["fixturing"] = self._sv["fixturing"].get()
        dev["notes"]    = self._sv["device_notes"].get().strip()
        try:
            dev.setdefault("dimensions_mm", {})
            dev["dimensions_mm"]["length"] = float(self._sv["dim_l"].get())
            dev["dimensions_mm"]["width"]  = float(self._sv["dim_w"].get())
            dev["dimensions_mm"]["height"] = float(self._sv["dim_h"].get())
        except ValueError:
            pass

    # ── Component list ─────────────────────────────────────────────────────

    def _rebuild_comp_list(self) -> None:
        for w in self._comp_frame.winfo_children():
            w.destroy()
        comps = self.doc.data.get("components", {})
        for lbl in list(comps.keys()):
            comp = comps[lbl]
            dot_col = ("#60a5fa" if comp.get("type") == "chassis"
                       else "#4ade80" if comp.get("removable")
                       else _TEXT_DIM)
            row = _frame(self._comp_frame, bg=_SURFACE)
            row.pack(fill="x", pady=3)
            tk.Label(row, text="●", fg=dot_col, bg=_SURFACE,
                     font=("Courier", 8)).pack(side="left", padx=(0, 4))
            # truncated name with fixed expand
            name_lbl = tk.Label(row, text=lbl, fg=_TEXT, bg=_SURFACE,
                                 font=_MONO_SM, anchor="w")
            name_lbl.pack(side="left", fill="x", expand=True)
            # fixed-width delete button so it never gets squished
            del_btn = tk.Button(
                row, text="×", command=lambda l=lbl: self._remove_component(l),
                bg="#3b1111", fg=_DANGER, relief="flat",
                font=("Courier", 10, "bold"),
                cursor="hand2", width=2, padx=4, pady=1,
            )
            del_btn.pack(side="right", padx=(4, 0))
        if not comps:
            tk.Label(self._comp_frame, text="No components.",
                     fg=_TEXT_DIM, bg=_SURFACE, font=_MONO_SM).pack(anchor="w")

    def _add_component(self) -> None:
        from tkinter.simpledialog import askstring
        lbl = askstring("Add Component", "Component label:", parent=self.root)
        if lbl:
            self.doc.add_component(lbl.strip())
            self._rebuild_comp_list()
            self._schedule_yaml()

    def _remove_component(self, lbl: str) -> None:
        self.doc.remove_component(lbl)
        self._rebuild_comp_list()
        self._schedule_yaml()

    # ── Zone list ──────────────────────────────────────────────────────────

    def _rebuild_zone_list(self) -> None:
        for w in self._zone_frame.winfo_children():
            w.destroy()
        zones = self.doc.data.get("screw_zones", {})
        for name in list(zones.keys()):
            z = zones[name]
            row = _frame(self._zone_frame, bg=_SURFACE)
            row.pack(fill="x", pady=3)
            tk.Label(row, text="⚙", fg="#fbbf24", bg=_SURFACE,
                     font=("Courier", 8)).pack(side="left", padx=(0, 4))
            info_txt = f"({z.get('screw_count', 0)}× {z.get('screw_type', '')})"
            name_lbl = tk.Label(row,
                                 text=f"{name}  {info_txt}",
                                 fg=_TEXT, bg=_SURFACE,
                                 font=_MONO_SM, anchor="w")
            name_lbl.pack(side="left", fill="x", expand=True)
            del_btn = tk.Button(
                row, text="×",
                command=lambda n=name: self._remove_zone(n),
                bg="#3b1111", fg=_DANGER, relief="flat",
                font=("Courier", 10, "bold"),
                cursor="hand2", width=2, padx=4, pady=1,
            )
            del_btn.pack(side="right", padx=(4, 0))
        if not zones:
            tk.Label(self._zone_frame, text="No screw zones.",
                     fg=_TEXT_DIM, bg=_SURFACE, font=_MONO_SM).pack(anchor="w")

    def _add_zone(self) -> None:
        from tkinter.simpledialog import askstring
        name = askstring("Add Screw Zone", "Zone name:", parent=self.root)
        if name:
            self.doc.add_zone(name.strip())
            self._rebuild_zone_list()
            self._schedule_yaml()

    def _remove_zone(self, name: str) -> None:
        self.doc.remove_zone(name)
        self._rebuild_zone_list()
        self._schedule_yaml()

    # ── Sequence sidebar list ──────────────────────────────────────────────

    def _rebuild_seq_list(self) -> None:
        for w in self._seq_list_frame.winfo_children():
            w.destroy()
        seq = self.doc.data.get("disassembly_sequence", [])
        self._step_count_lbl.config(
            text=f"{len(seq)} step{'s' if len(seq) != 1 else ''}")
        for i, step in enumerate(seq):
            action = step.get("action", "hold")
            st = ACTION_STYLE.get(action, {"bg": _SURFACE2, "fg": _TEXT_DIM})
            row = _frame(self._seq_list_frame, bg=_SURFACE)
            row.pack(fill="x", pady=2)
            row.bind("<Button-1>", lambda e, idx=i: self._scroll_to_card(idx))
            badge = tk.Label(
                row, text=str(step.get("step", i + 1)),
                fg=st["fg"], bg=st["bg"],
                font=("Courier", 8, "bold"),
                width=3, padx=4, pady=2,
            )
            badge.pack(side="left")
            badge.bind("<Button-1>", lambda e, idx=i: self._scroll_to_card(idx))
            tk.Label(row, text=step.get("label", action)[:26],
                     fg=_TEXT, bg=_SURFACE,
                     font=_MONO_SM, anchor="w").pack(side="left", padx=6)

    # ── Step cards ─────────────────────────────────────────────────────────

    def _rebuild_cards(self) -> None:
        for w in self.cards_frame.winfo_children():
            w.destroy()
        seq = self.doc.data.get("disassembly_sequence", [])
        if not seq:
            tk.Label(self.cards_frame,
                     text="No steps yet.\nUse sidebar buttons to add a step.",
                     fg=_TEXT_DIM, bg=_BG, font=_MONO,
                     justify="center", pady=60).pack()
            return
        for i, step in enumerate(seq):
            self._build_step_card(self.cards_frame, step, i,
                                   self._expanded.get(i, True))

    def _build_step_card(self, parent: tk.Widget, step: Dict[str, Any],
                          idx: int, expanded: bool) -> None:
        action = step.get("action", "hold")
        st = ACTION_STYLE.get(action, {"bg": _SURFACE2, "fg": _TEXT_DIM})

        # Card with coloured left accent bar
        outer = tk.Frame(parent, bg=st["fg"], padx=2)
        outer.pack(fill="x", pady=5, padx=18)

        inner = _frame(outer, bg=_SURFACE)
        inner.pack(fill="both", expand=True)

        # ── header ──────────────────────────────────────────────────────────
        hdr = _frame(inner, bg=_SURFACE2, cursor="hand2")
        hdr.pack(fill="x")

        left = _frame(hdr, bg=_SURFACE2)
        left.pack(side="left", fill="both", expand=True, padx=12, pady=10)
        tk.Label(left, text=action.upper().replace("_", " "),
                 fg=st["fg"], bg=st["bg"],
                 font=("Courier", 8, "bold"),
                 padx=6, pady=2).pack(side="left")
        tk.Label(left, text=f"#{step.get('step', idx + 1)}",
                 fg=_TEXT_DIM, bg=_SURFACE2,
                 font=_MONO_SM).pack(side="left", padx=(8, 0))

        lv = tk.StringVar(value=step.get("label", ""))
        lv.trace_add("write",
                     lambda *_, s=step, v=lv: (
                         s.__setitem__("label", v.get()),
                         self._schedule_yaml(),
                         self.root.after(400, self._rebuild_seq_list),
                     ))
        tk.Entry(left, textvariable=lv, bg=_SURFACE2, fg=_WHITE,
                 insertbackground=_WHITE,
                 font=("Courier", 11, "bold"),
                 relief="flat", bd=0, width=32).pack(side="left", padx=10)

        # right controls
        right = _frame(hdr, bg=_SURFACE2)
        right.pack(side="right", padx=10, pady=6)

        body = _frame(inner, bg=_SURFACE)
        exp_state = [expanded]

        def _toggle(b=body, es=exp_state, i=idx) -> None:
            es[0] = not es[0]
            self._expanded[i] = es[0]
            if es[0]:
                b.pack(fill="x", padx=16, pady=(8, 16))
            else:
                b.pack_forget()
            tgl.config(text="▾" if es[0] else "▸")

        tgl = tk.Label(right, text="▾" if expanded else "▸",
                       fg=_TEXT_DIM, bg=_SURFACE2,
                       font=("Courier", 11), cursor="hand2")
        tgl.pack(side="left", padx=3)
        tgl.bind("<Button-1>", lambda e: _toggle())
        hdr.bind("<Button-1>", lambda e: _toggle())

        for txt, cmd_fn, col in [
            ("↑", lambda e, i=idx: self._move_step(i, -1), _TEXT_DIM),
            ("↓", lambda e, i=idx: self._move_step(i,  1), _TEXT_DIM),
            ("×", lambda e, i=idx: self._delete_step(i),   _DANGER),
        ]:
            lbl = tk.Label(right, text=txt, fg=col, bg=_SURFACE2,
                           font=("Courier", 11, "bold"), cursor="hand2",
                           padx=4)
            lbl.pack(side="left")
            lbl.bind("<Button-1>", cmd_fn)

        if expanded:
            body.pack(fill="x", padx=16, pady=(8, 16))

        # ── body ────────────────────────────────────────────────────────────
        self._build_common_fields(body, step)
        tk.Frame(body, bg=_BORDER, height=1).pack(fill="x", pady=10)

        params = step.setdefault("parameters", {})
        builders = {
            "hold":        self._build_hold_editor,
            "unscrew":     self._build_unscrew_editor,
            "pickup":      self._build_pickup_editor,
            "flip":        self._build_flip_editor,
            "flip_drop":   self._build_flip_drop_editor,
            "tool_change": self._build_tool_change_editor,
        }
        builders.get(action, lambda p, q: None)(body, params)

    # ── Common fields ──────────────────────────────────────────────────────

    def _build_common_fields(self, parent: tk.Widget,
                              step: Dict[str, Any]) -> None:
        comp_labels = self.doc.component_labels()
        zone_names  = self.doc.zone_names()

        row1 = _frame(parent, bg=_SURFACE)
        row1.pack(fill="x", pady=(10, 6))

        # Target
        tc = _frame(row1, bg=_SURFACE)
        tc.pack(side="left", fill="x", expand=True, padx=(0, 12))
        hr = _frame(tc, bg=_SURFACE)
        hr.pack(fill="x")
        tk.Label(hr, text="Target", fg=_TEXT_DIM, bg=_SURFACE,
                 font=_LBL).pack(side="left")
        _info(hr,
              "Component or screw zone that this step acts upon.\n"
              "Choose from the components / zones defined in the sidebar.",
              bg=_SURFACE).pack(side="left")
        tv = tk.StringVar(value=step.get("target", ""))
        tv.trace_add("write",
                     lambda *_, s=step, v=tv: (
                         s.__setitem__("target", v.get()),
                         self._schedule_yaml()))
        ttk.Combobox(tc, textvariable=tv,
                     values=comp_labels + zone_names,
                     width=24, font=_MONO).pack(fill="x", pady=(4, 0))

        # Notes
        nc = _frame(row1, bg=_SURFACE)
        nc.pack(side="left", fill="x", expand=True)
        tk.Label(nc, text="Notes", fg=_TEXT_DIM, bg=_SURFACE,
                 font=_LBL, anchor="w").pack(fill="x")
        nv = tk.StringVar(value=step.get("notes", ""))
        nv.trace_add("write",
                     lambda *_, s=step, v=nv: (
                         s.__setitem__("notes", v.get()),
                         self._schedule_yaml()))
        _entry(nc, nv, width=28).pack(fill="x", pady=(4, 0))

        # Depends on steps
        row2 = _frame(parent, bg=_SURFACE)
        row2.pack(fill="x", pady=6)
        hr2 = _frame(row2, bg=_SURFACE)
        hr2.pack(side="left")
        tk.Label(hr2, text="Depends on steps",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_LBL).pack(side="left")
        _info(hr2,
              "Step numbers that must complete before this step starts.\n"
              "Enter as comma-separated integers, e.g. 1, 3",
              bg=_SURFACE).pack(side="left")
        dv = tk.StringVar(
            value=", ".join(str(x) for x in step.get("depends_on_steps", [])))

        def _on_deps(*_, s=step, v=dv):
            try:
                s["depends_on_steps"] = [
                    int(x.strip()) for x in v.get().split(",") if x.strip()]
            except ValueError:
                pass
            self._schedule_yaml()
        dv.trace_add("write", _on_deps)
        _entry(row2, dv, width=22).pack(side="left", padx=8)

        # Reveals
        row3 = _frame(parent, bg=_SURFACE)
        row3.pack(fill="x", pady=6)
        hr3 = _frame(row3, bg=_SURFACE)
        hr3.pack(side="left")
        tk.Label(hr3, text="Reveals",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_LBL).pack(side="left")
        _info(hr3,
              "Component labels that become accessible / visible once this step completes.\n"
              "Enter as comma-separated names, e.g. PCB_Main, Internal_Screw_Zone",
              bg=_SURFACE).pack(side="left")
        rv = tk.StringVar(value=", ".join(step.get("reveals", [])))

        def _on_rev(*_, s=step, v=rv):
            s["reveals"] = [x.strip() for x in v.get().split(",") if x.strip()]
            self._schedule_yaml()
        rv.trace_add("write", _on_rev)
        _entry(row3, rv, width=32).pack(side="left", padx=8)

    # ── Action editors ─────────────────────────────────────────────────────

    def _prow(self, parent: tk.Widget) -> tk.Frame:
        row = _frame(parent, bg=_SURFACE)
        row.pack(fill="x", pady=6)
        return row

    def _pe(self, parent: tk.Widget, label: str, params: Dict[str, Any],
            key: str, cast=float, width: int = 10) -> None:
        """Param entry column with label + info icon."""
        col = _frame(parent, bg=_SURFACE)
        col.pack(side="left", padx=(0, 14))
        hrow = _frame(col, bg=_SURFACE)
        hrow.pack(fill="x")
        tk.Label(hrow, text=label, fg=_TEXT_DIM, bg=_SURFACE,
                 font=_LBL, anchor="w").pack(side="left")
        if key in _TT:
            _info(hrow, _TT[key], bg=_SURFACE).pack(side="left")
        v = tk.StringVar(value=str(params.get(key, "")))
        v.trace_add("write",
                    lambda *_, p=params, k=key, vv=v, c=cast:
                    self._on_param(p, k, vv, c))
        _entry(col, v, width=width).pack(fill="x", pady=(4, 0))

    def _pc(self, parent: tk.Widget, label: str, params: Dict[str, Any],
            key: str, values: tuple, width: int = 13) -> None:
        """Param combo column with label + info icon."""
        col = _frame(parent, bg=_SURFACE)
        col.pack(side="left", padx=(0, 14))
        hrow = _frame(col, bg=_SURFACE)
        hrow.pack(fill="x")
        tk.Label(hrow, text=label, fg=_TEXT_DIM, bg=_SURFACE,
                 font=_LBL, anchor="w").pack(side="left")
        tt = _TT_FIELD.get(key, _TT.get(key, ""))
        if tt:
            _info(hrow, tt, bg=_SURFACE).pack(side="left")
        v = tk.StringVar(value=params.get(key, values[0]))
        v.trace_add("write",
                    lambda *_, p=params, k=key, vv=v:
                    self._on_param(p, k, vv, str))
        _combo(col, v, values, width=width).pack(fill="x", pady=(4, 0))

    # hold
    def _build_hold_editor(self, parent: tk.Widget,
                            params: Dict[str, Any]) -> None:
        tk.Label(parent, text="HOLD PARAMETERS",
                 fg="#60a5fa", bg=_SURFACE, font=_LBL).pack(
            anchor="w", pady=(4, 10))
        tk.Label(parent, text="✋  UF850 + RG6 gripper — clamps device for stability",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_MONO_SM,
                 anchor="w").pack(anchor="w", pady=(0, 8))

        force_warn = tk.Label(parent, text="", fg=_WARN, bg=_SURFACE,
                               font=_MONO_SM, anchor="w")
        force_warn.pack(anchor="w")

        r1 = self._prow(parent)
        self._pc(r1, "Strategy",     params, "strategy",     STRATEGY_TYPES)
        self._pc(r1, "Approach Axis", params, "approach_axis", APPROACH_AXES)
        self._pe(r1, "Grip Width mm", params, "grip_width_mm", float, 9)

        r2 = self._prow(parent)
        self._pe(r2, "Close Force N",  params, "gripper_close_force_n", float, 9)
        self._pe(r2, "Open Deg",       params, "gripper_open_deg",      float, 8)
        self._pe(r2, "Close Deg",      params, "gripper_close_deg",     float, 8)

        r3 = self._prow(parent)
        self._pe(r3, "Descent m/s",      params, "descent_speed_mps",   float, 9)
        self._pe(r3, "Tilt Deg",         params, "tilt_deg",             float, 8)
        self._pe(r3, "Torque Thresh Nm", params, "torque_threshold_nm", float, 12)

        r4 = self._prow(parent)
        self._pe(r4, "Hover X Offset m", params, "hover_x_offset_m", float, 11)
        self._pe(r4, "Hover Y Offset m", params, "hover_y_offset_m", float, 11)

        def _check(*_):
            try:
                f = float(params.get("gripper_close_force_n", 0))
                force_warn.config(
                    text=f"⚠  Force {f} N exceeds RG6 max {RG6_MAX_FORCE_N} N"
                    if f > RG6_MAX_FORCE_N else "")
            except (ValueError, TypeError):
                pass
        _check()

    # unscrew
    def _build_unscrew_editor(self, parent: tk.Widget,
                               params: Dict[str, Any]) -> None:
        tk.Label(parent, text="UNSCREW PARAMETERS",
                 fg="#fbbf24", bg=_SURFACE, font=_LBL).pack(
            anchor="w", pady=(4, 10))
        tk.Label(parent, text="⚙  xArm5 + FT300 force sensor + screwdriver",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_MONO_SM,
                 anchor="w").pack(anchor="w", pady=(0, 8))

        r1 = self._prow(parent)
        self._pc(r1, "Screw Type", params, "screw_type", SCREW_TYPES)
        self._pe(r1, "Count",      params, "screw_count",        int,   6)
        self._pe(r1, "Engage Depth mm", params, "engagement_depth_mm", float, 10)

        r2 = self._prow(parent)
        self._pe(r2, "Init Torque Nm", params, "initial_torque_nm", float, 10)
        self._pe(r2, "Max Torque Nm",  params, "max_torque_nm",     float, 10)
        self._pe(r2, "Speed RPM",      params, "rotation_speed_rpm", int,   8)

        r3 = self._prow(parent)
        self._pe(r3, "Force Thresh N",    params, "force_threshold_n",  float, 10)
        self._pe(r3, "Align Tolerance px", params, "align_tolerance_px", float, 14)
        self._pe(r3, "Spiral Timeout s",   params, "spiral_timeout_s",  float, 12)

    # pickup
    def _build_pickup_editor(self, parent: tk.Widget,
                              params: Dict[str, Any]) -> None:
        tk.Label(parent, text="PICKUP PARAMETERS",
                 fg="#4ade80", bg=_SURFACE, font=_LBL).pack(
            anchor="w", pady=(4, 10))
        tk.Label(parent, text="✋  UF850 + RG6 gripper",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_MONO_SM,
                 anchor="w").pack(anchor="w", pady=(0, 8))

        r1 = self._prow(parent)
        self._pc(r1, "Grasp Type",    params, "grasp_type",    GRASP_TYPES)
        self._pc(r1, "Approach Axis", params, "approach_axis", APPROACH_AXES)
        self._pe(r1, "Grip Width mm", params, "grip_width_mm", float, 9)

        r2 = self._prow(parent)
        self._pe(r2, "Close Force N",  params, "gripper_close_force_n", float, 9)
        self._pe(r2, "Lift Height mm", params, "lift_height_mm",        float, 10)
        self._pe(r2, "Lift Speed m/s", params, "lift_speed_mps",        float, 10)

        r3 = self._prow(parent)
        self._pe(r3, "Approach X Offset m", params, "approach_x_offset_m", float, 14)
        self._pe(r3, "Approach Y Offset m", params, "approach_y_offset_m", float, 14)

        tk.Label(parent, text="Drop Zone  (base_link frame)",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_LBL,
                 anchor="w").pack(anchor="w", pady=(8, 2))
        r4 = self._prow(parent)
        self._pe(r4, "X", params, "drop_x", float, 9)
        self._pe(r4, "Y", params, "drop_y", float, 9)
        self._pe(r4, "Z", params, "drop_z", float, 9)

    # flip
    def _build_flip_editor(self, parent: tk.Widget,
                            params: Dict[str, Any]) -> None:
        tk.Label(parent, text="FLIP PARAMETERS",
                 fg="#c084fc", bg=_SURFACE, font=_LBL).pack(
            anchor="w", pady=(4, 10))
        tk.Label(parent, text="↺  UF850 — rotates chassis 180°",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_MONO_SM,
                 anchor="w").pack(anchor="w", pady=(0, 8))
        r1 = self._prow(parent)
        self._pe(r1, "Retract Height m", params, "retract_height_m",     float, 12)
        self._pe(r1, "Close Force N",    params, "gripper_close_force_n", float, 9)
        self._pe(r1, "Rotation Deg",     params, "rotation_deg",          float, 9)

    # flip_drop
    def _build_flip_drop_editor(self, parent: tk.Widget,
                                 params: Dict[str, Any]) -> None:
        tk.Label(parent, text="FLIP-DROP PARAMETERS",
                 fg="#f472b6", bg=_SURFACE, font=_LBL).pack(
            anchor="w", pady=(4, 10))
        tk.Label(parent, text="⇣  UF850 — inverts chassis to dump loose parts",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_MONO_SM,
                 anchor="w").pack(anchor="w", pady=(0, 8))
        r1 = self._prow(parent)
        self._pe(r1, "Retract Height m", params, "retract_height_m",     float, 12)
        self._pe(r1, "Close Force N",    params, "gripper_close_force_n", float, 9)

        tk.Label(parent, text="Intermediate pose  (base_link frame)",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_LBL,
                 anchor="w").pack(anchor="w", pady=(8, 2))
        r2 = self._prow(parent)
        self._pe(r2, "X", params, "intermediate_x", float, 10)
        self._pe(r2, "Y", params, "intermediate_y", float, 10)
        self._pe(r2, "Z", params, "intermediate_z", float, 10)

    # tool_change
    def _build_tool_change_editor(self, parent: tk.Widget,
                                   params: Dict[str, Any]) -> None:
        tk.Label(parent, text="TOOL CHANGE",
                 fg="#2dd4bf", bg=_SURFACE, font=_LBL).pack(
            anchor="w", pady=(4, 10))

        info_box = tk.Frame(parent,
                             bg="#0a2d2d",
                             highlightbackground="#2dd4bf",
                             highlightthickness=1)
        info_box.pack(fill="x", pady=(0, 10))
        tk.Label(info_box,
                 text="🔧  Manual bit swap required before the next unscrew step.\n"
                      "     The robot will pause and wait for operator confirmation.",
                 fg="#2dd4bf", bg="#0a2d2d",
                 font=_MONO_SM, justify="left",
                 padx=10, pady=8).pack(anchor="w")

        r1 = self._prow(parent)
        self._pc(r1, "Required Bit", params, "required_bit",
                 BIT_TYPES, width=8)
        self._pc(r1, "Method", params, "method",
                 TOOL_CHANGE_METHODS, width=14)

        # Notes row
        r2 = self._prow(parent)
        col = _frame(r2, bg=_SURFACE)
        col.pack(side="left", fill="x", expand=True)
        hrow = _frame(col, bg=_SURFACE)
        hrow.pack(fill="x")
        tk.Label(hrow, text="Operator Notes",
                 fg=_TEXT_DIM, bg=_SURFACE, font=_LBL).pack(side="left")
        _info(hrow,
              "Instruction shown to operator when the robot pauses for tool change.",
              bg=_SURFACE).pack(side="left")
        nv = tk.StringVar(value=params.get("notes", ""))
        nv.trace_add("write",
                     lambda *_, p=params, v=nv:
                     self._on_param(p, "notes", v, str))
        _entry(col, nv, width=42).pack(fill="x", pady=(4, 0))

    # ── Data callbacks ─────────────────────────────────────────────────────

    def _on_param(self, container: Dict[str, Any], key: str,
                  var: tk.Variable, cast) -> None:
        try:
            container[key] = cast(var.get())
        except (ValueError, TypeError):
            pass
        self._schedule_yaml()

    def _schedule_yaml(self) -> None:
        if self._yaml_after_id:
            self.root.after_cancel(self._yaml_after_id)
        self._yaml_after_id = self.root.after(250, self._refresh_yaml)

    def _refresh_yaml(self) -> None:
        self._sync_device()
        try:
            import yaml as _yaml
            txt = _yaml.safe_dump(
                self.doc.data, sort_keys=False,
                allow_unicode=False, default_flow_style=False)
        except Exception as exc:
            txt = f"# YAML error: {exc}"
        self._yaml_text.configure(state="normal")
        self._yaml_text.delete("1.0", "end")
        self._yaml_text.insert("1.0", txt)
        self._yaml_text.configure(state="disabled")
        self._yaml_after_id = None

    def _refresh_warnings(self) -> None:
        for w in self._warn_frame.winfo_children():
            w.destroy()
        errors, warnings = validate_config(self.doc.data)
        items = [(e, _DANGER) for e in errors] + [(w, _WARN) for w in warnings]
        if not items:
            tk.Label(self._warn_frame, text="✓  No issues",
                     fg=_SUCCESS, bg=_SURFACE, font=_MONO_SM).pack(anchor="w")
        else:
            for msg, col in items:
                tk.Label(self._warn_frame,
                         text=f"• {msg}",
                         fg=col, bg=_SURFACE,
                         font=_MONO_SM, wraplength=230,
                         justify="left").pack(anchor="w", pady=2)

    def _copy_yaml(self) -> None:
        self.root.clipboard_clear()
        self.root.clipboard_append(self._yaml_text.get("1.0", "end"))
        messagebox.showinfo("Copied", "YAML copied to clipboard.",
                             parent=self.root)

    # ── Sequence ops ───────────────────────────────────────────────────────

    def _add_step(self, action: str) -> None:
        self.doc.add_sequence_step(action)
        new_idx = len(self.doc.data["disassembly_sequence"]) - 1
        self._expanded[new_idx] = True
        self._rebuild_cards()
        self._rebuild_seq_list()
        self._schedule_yaml()
        self.root.after(100, lambda: self._cc.yview_moveto(1.0))

    def _delete_step(self, idx: int) -> None:
        self.doc.remove_sequence_step(idx)
        self._expanded.pop(idx, None)
        new_exp = {}
        for k, v in self._expanded.items():
            if k < idx:
                new_exp[k] = v
            elif k > idx:
                new_exp[k - 1] = v
        self._expanded = new_exp
        self._rebuild_cards()
        self._rebuild_seq_list()
        self._schedule_yaml()

    def _move_step(self, idx: int, direction: int) -> None:
        new_idx = self.doc.move_sequence_step(idx, direction)
        self._expanded[new_idx], self._expanded[idx] = (
            self._expanded.get(idx, True),
            self._expanded.get(new_idx, True),
        )
        self._rebuild_cards()
        self._rebuild_seq_list()
        self._schedule_yaml()

    def _scroll_to_card(self, idx: int) -> None:
        seq = self.doc.data.get("disassembly_sequence", [])
        if seq:
            self._cc.yview_moveto(idx / max(len(seq), 1))

    # ── File operations ────────────────────────────────────────────────────

    def _config_dir(self) -> Optional[str]:
        if _CONFIG_DIR.exists():
            return str(_CONFIG_DIR)
        return None

    def _new_config(self) -> None:
        if messagebox.askyesno("New Config",
                                "Discard current config and start fresh?",
                                parent=self.root):
            self.doc = DeviceConfigDocument()
            self._expanded.clear()
            self._rebuild_all()

    def _open_project(self) -> None:
        path = filedialog.askopenfilename(
            title="Open Config",
            initialdir=self._config_dir(),
            filetypes=[
                ("YAML config",   "*.yaml *.yml"),
                ("JSON project",  "*.json"),
                ("All files",     "*.*"),
            ],
            parent=self.root,
        )
        if not path:
            return
        try:
            if path.endswith(".json"):
                data = load_project(path)
            else:
                data = load_yaml(path)
            self.doc = DeviceConfigDocument(data)
            self._expanded.clear()
            self._rebuild_all()
        except Exception as exc:
            messagebox.showerror("Open Error", str(exc), parent=self.root)

    def _save_project(self) -> None:
        self._sync_device()
        path = filedialog.asksaveasfilename(
            title="Save Project",
            initialdir=self._config_dir(),
            defaultextension=".json",
            filetypes=[("JSON project", "*.json")],
            parent=self.root,
        )
        if path:
            try:
                save_project(path, self.doc.data)
            except Exception as exc:
                messagebox.showerror("Save Error", str(exc), parent=self.root)

    def _export_yaml(self) -> None:
        self._sync_device()
        errors, _ = validate_config(self.doc.data)
        if errors:
            if not messagebox.askyesno(
                    "Validation Errors",
                    f"{len(errors)} error(s) found.\nExport anyway?",
                    parent=self.root):
                return
        path = filedialog.asksaveasfilename(
            title="Export YAML",
            initialdir=self._config_dir(),
            defaultextension=".yaml",
            filetypes=[
                ("YAML config", "*.yaml *.yml"),
                ("All files",   "*.*"),
            ],
            parent=self.root,
        )
        if path:
            try:
                save_yaml(path, self.doc.data)
                messagebox.showinfo("Exported",
                                     f"Config exported to:\n{path}",
                                     parent=self.root)
            except Exception as exc:
                messagebox.showerror("Export Error", str(exc), parent=self.root)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    root = tk.Tk()
    ConfigBuilderApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
