"""Config builder data model — spec-aligned schema."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, List, Optional


def default_step_params(action: str) -> Dict[str, Any]:
    """Return default parameters for a given action type."""
    if action == "hold":
        return {
            "strategy": "lateral_clamp",
            "grip_width_mm": 85.0,
            "approach_axis": "+z",
            "gripper_open_deg": 35.0,
            "gripper_close_deg": -35.0,
            "gripper_close_force_n": 100.0,
            "torque_threshold_nm": 3.0,
            "descent_speed_mps": 0.015,
            "descent_step_m": 0.00035,
            "descent_rate_hz": 50.0,
            "tilt_deg": 7.0,
            "hover_x_offset_m": -0.01,
            "hover_y_offset_m": -0.005,
            "approach_velocity": 0.08,
            "hover_velocity": 0.08,
        }
    if action == "unscrew":
        return {
            "screw_type": "phillips",
            "screw_count": 1,
            "engagement_depth_mm": 3.0,
            "initial_torque_nm": 0.5,
            "max_torque_nm": 1.5,
            "rotation_speed_rpm": 60,
            "force_threshold_n": 5.0,
            "align_tolerance_px": 4.0,
            "spiral_timeout_s": 15.0,
        }
    if action == "pickup":
        return {
            "grasp_type": "parallel",
            "grip_width_mm": 50.0,
            "gripper_close_force_n": 80.0,
            "approach_axis": "+z",
            "lift_height_mm": 50.0,
            "lift_speed_mps": 0.02,
            "approach_x_offset_m": -0.02,
            "approach_y_offset_m": 0.019,
            "drop_x": 0.92,
            "drop_y": -0.36,
            "drop_z": 0.99,
        }
    if action == "flip":
        return {
            "retract_height_m": 0.10,
            "gripper_close_force_n": 100.0,
            "rotation_deg": 180,
        }
    if action == "flip_drop":
        return {
            "retract_height_m": 0.05,
            "retract_velocity": 0.04,
            "retract_step_m": 0.025,
            "gripper_close_force_n": 100.0,
            "use_pickup_drop_xyz": True,
            "pickup_drop_z_offset_m": 0.04,
            "intermediate_x": 0.92,
            "intermediate_y": -0.36,
            "intermediate_z": 0.99,
            "drop_transfer_velocity": 0.06,
            "drop_transfer_step_m": 0.050,
            "wrist_rotation_velocity": 0.15,
            "purpose": "dump_loose_parts",
        }
    if action == "tool_change":
        return {
            "required_bit": "T8",
            "method": "manual",
            "notes": "Operator must swap bit before next step.",
        }
    return {}


def default_config_dict() -> Dict[str, Any]:
    """Return a default device configuration document (spec schema)."""
    return {
        "device": {
            "class": "hdd",
            "model": "new_device",
            "dimensions_mm": {"length": 147.0, "width": 101.6, "height": 26.1},
            "material": "mixed",
            "fixturing": "gripper_only",
            "notes": "",
        },
        "components": {
            "device_chassis": {
                "label": "device_chassis",
                "type": "chassis",
                "removable": False,
                "notes": "Primary structural base.",
            }
        },
        "screw_zones": {},
        "disassembly_sequence": [
            {
                "step": 1,
                "action": "hold",
                "label": "Secure chassis",
                "target": "device_chassis",
                "depends_on_steps": [],
                "reveals": [],
                "parameters": default_step_params("hold"),
                "notes": "",
            }
        ],
        "metadata": {
            "created": "",
            "created_by": "device_config_builder",
            "hardware": {
                "manipulation_arm": "uf850_rg6",
                "tooling_arm": "xarm5_ft300_screwdriver_micro_camera",
                "global_camera": "intel_realsense_d455",
            },
            "framework_version": "1.0",
            "total_steps": 1,
        },
    }


class DeviceConfigDocument:
    """Mutable document wrapper used by the builder UI."""

    def __init__(self, data: Optional[Dict[str, Any]] = None) -> None:
        self.data = deepcopy(data) if data else default_config_dict()
        self._migrate_legacy_schema()

    def _migrate_legacy_schema(self) -> None:
        """Silently upgrade old-format configs to current spec schema."""
        device = self.data.get("device", {})
        # Old key → new key migration
        if "device_class" in device and "class" not in device:
            device["class"] = device.pop("device_class")
        if "device_model" in device and "model" not in device:
            device["model"] = device.pop("device_model")

        # Components: list → dict
        components = self.data.get("components", [])
        if isinstance(components, list):
            new_components: Dict[str, Any] = {}
            for item in components:
                label = item.get("label", "unknown")
                new_components[label] = item
            self.data["components"] = new_components

        # Screw zones: list → dict
        zones = self.data.get("screw_zones", [])
        if isinstance(zones, list):
            new_zones: Dict[str, Any] = {}
            for item in zones:
                name = item.get("zone_name", "unknown")
                entry = {k: v for k, v in item.items() if k != "zone_name"}
                new_zones[name] = entry
            self.data["screw_zones"] = new_zones

        # Sequence steps: old keys → new keys
        for i, step in enumerate(self.data.get("disassembly_sequence", []), start=1):
            if "type" in step and "action" not in step:
                step["action"] = step.pop("type")
            if "name" in step and "label" not in step:
                step["label"] = step.pop("name")
            if "component" in step and "target" not in step:
                step["target"] = step.pop("component")
            if "zone_name" in step and "target" not in step:
                step["target"] = step.pop("zone_name")
            if "params" in step and "parameters" not in step:
                step["parameters"] = step.pop("params")
            step.setdefault("step", i)
            step.setdefault("depends_on_steps", [])
            step.setdefault("reveals", [])
            step.setdefault("parameters", default_step_params(step.get("action", "hold")))
            step.setdefault("notes", "")

    def reset(self) -> None:
        self.data = default_config_dict()

    def _renumber_sequence(self) -> None:
        for i, step in enumerate(self.data.get("disassembly_sequence", []), start=1):
            step["step"] = i
        self.data.setdefault("metadata", {})["total_steps"] = len(
            self.data.get("disassembly_sequence", [])
        )

    # --- Components ---

    def add_component(self, label: str = "new_component") -> str:
        components = self.data.setdefault("components", {})
        base, counter = label, 1
        while label in components:
            label = f"{base}_{counter}"
            counter += 1
        components[label] = {"label": label, "type": "other", "removable": True, "notes": ""}
        return label

    def rename_component(self, old_label: str, new_label: str) -> None:
        components = self.data.get("components", {})
        if old_label not in components or old_label == new_label:
            return
        components[new_label] = components.pop(old_label)
        components[new_label]["label"] = new_label

    def remove_component(self, label: str) -> None:
        self.data.get("components", {}).pop(label, None)

    def component_labels(self) -> List[str]:
        return list(self.data.get("components", {}).keys())

    # --- Zones ---

    def add_zone(self, zone_name: str = "new_zone") -> str:
        zones = self.data.setdefault("screw_zones", {})
        base, counter = zone_name, 1
        while zone_name in zones:
            zone_name = f"{base}_{counter}"
            counter += 1
        zones[zone_name] = {
            "parent_component": "",
            "screw_count": 1,
            "screw_type": "phillips",
            "depends_on_removal_of": [],
        }
        return zone_name

    def rename_zone(self, old_name: str, new_name: str) -> None:
        zones = self.data.get("screw_zones", {})
        if old_name not in zones or old_name == new_name:
            return
        zones[new_name] = zones.pop(old_name)

    def remove_zone(self, zone_name: str) -> None:
        self.data.get("screw_zones", {}).pop(zone_name, None)

    def zone_names(self) -> List[str]:
        return list(self.data.get("screw_zones", {}).keys())

    # --- Sequence ---

    def add_sequence_step(self, action: str = "hold") -> None:
        sequence = self.data.setdefault("disassembly_sequence", [])
        sequence.append(
            {
                "step": len(sequence) + 1,
                "action": action,
                "label": f"New {action} step",
                "target": "",
                "depends_on_steps": [],
                "reveals": [],
                "parameters": default_step_params(action),
                "notes": "",
            }
        )
        self._renumber_sequence()

    def remove_sequence_step(self, index: int) -> None:
        sequence = self.data.get("disassembly_sequence", [])
        if 0 <= index < len(sequence):
            sequence.pop(index)
            self._renumber_sequence()

    def move_sequence_step(self, index: int, direction: int) -> int:
        sequence = self.data.get("disassembly_sequence", [])
        target = index + direction
        if 0 <= index < len(sequence) and 0 <= target < len(sequence):
            sequence[index], sequence[target] = sequence[target], sequence[index]
            self._renumber_sequence()
            return target
        return index
