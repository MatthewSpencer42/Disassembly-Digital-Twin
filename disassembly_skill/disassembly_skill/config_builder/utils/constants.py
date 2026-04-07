"""Shared constants for device configuration tooling."""

ACTION_TYPES = ("hold", "unscrew", "pickup", "flip", "flip_drop", "tool_change")

ACTION_COLOURS = {
    "hold": "#1E90FF",
    "unscrew": "#FFA500",
    "pickup": "#32CD32",
    "flip": "#9370DB",
    "flip_drop": "#FF69B4",
    "tool_change": "#20B2AA",
}

ARM_NAMES = ("uf850", "xarm5")

COMPONENT_TYPES = (
    "chassis",
    "lid",
    "board",
    "storage",
    "memory",
    "screw_zone",
    "fastener",
    "drive",
    "battery",
    "connector",
    "shield",
    "bracket",
    "other",
)

FIXTURING_TYPES = ("gripper_only", "passive_jig", "vice", "soft_jaw_vice", "tray", "custom_fixture", "none")

MATERIAL_TYPES = ("aluminium", "plastic", "steel", "mixed", "unknown")

SCREW_TYPES = ("phillips", "torx", "hex", "flat", "tri_wing", "pentalobe")

APPROACH_AXES = ("+x", "-x", "+y", "-y", "+z", "-z")

GRASP_TYPES = ("parallel", "pinch", "wide", "precision")

STRATEGY_TYPES = ("lateral_clamp", "top_down_clamp", "edge_clamp", "fixture_press")

# Screwdriver bits available on the xArm5 tool changer
BIT_TYPES = (
    # Torx
    "T4", "T5", "T6", "T8", "T10", "T15", "T20", "T25",
    # Phillips
    "PH0", "PH1", "PH2", "PH3",
    # Hex / Allen
    "H1.5", "H2", "H2.5", "H3", "H4", "H5",
    # Speciality
    "TW1",       # Tri-wing
    "PL1", "PL2",  # Pentalobe
    "SL3", "SL4",  # Flat / slotted
)

# Tool-change method (currently all manual)
TOOL_CHANGE_METHODS = ("manual", "automatic_future")

# Hardware limits
RG6_MAX_GRIP_MM = 160.0
RG6_MAX_FORCE_N = 120.0

# Legacy alias so existing validation code still works
STEP_TYPES = ACTION_TYPES
