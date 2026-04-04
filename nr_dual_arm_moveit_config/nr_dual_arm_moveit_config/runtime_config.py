#!/usr/bin/env python3

from __future__ import annotations


def normalize_xacro_hardware_type(hardware_type: str) -> str:
    """Map launch-level hardware modes onto the xacro/plugin-level modes."""
    if hardware_type == "twin":
        return "real"
    return hardware_type


def joint_topics_for_hardware(hardware_type: str) -> tuple[str, str]:
    normalized = normalize_xacro_hardware_type(hardware_type)
    if normalized == "real":
        return "/robot_joint_commands", "/robot_joint_states"
    return "/isaac_joint_commands", "/isaac_joint_states"


def use_filtered_joint_states(hardware_type: str) -> bool:
    return hardware_type == "isaac"

