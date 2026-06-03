"""Validation logic for device configurations (spec-aligned schema)."""

from __future__ import annotations

from collections import defaultdict
from typing import Any, Dict, List, Set, Tuple

from .constants import ACTION_TYPES, RG6_MAX_FORCE_N, RG6_MAX_GRIP_MM


def _as_list(value: Any) -> List[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return [value]


def _detect_cycle(graph: Dict[str, List[str]]) -> bool:
    visiting: Set[str] = set()
    visited: Set[str] = set()

    def dfs(node: str) -> bool:
        if node in visited:
            return False
        if node in visiting:
            return True
        visiting.add(node)
        for neighbor in graph.get(node, []):
            if dfs(neighbor):
                return True
        visiting.remove(node)
        visited.add(node)
        return False

    return any(dfs(node) for node in graph)


def validate_config(data: Dict[str, Any]) -> Tuple[List[str], List[str]]:
    errors: List[str] = []
    warnings: List[str] = []

    # --- Components ---
    components_raw = data.get("components", {})
    # Support both dict (spec) and list (legacy) formats
    if isinstance(components_raw, dict):
        component_labels = set(components_raw.keys())
    else:
        component_labels = {item.get("label", "") for item in components_raw if item.get("label")}

    # --- Zones ---
    zones_raw = data.get("screw_zones", {})
    if isinstance(zones_raw, dict):
        zone_items = [(name, entry) for name, entry in zones_raw.items()]
    else:
        zone_items = [(item.get("zone_name", ""), item) for item in zones_raw]

    dependency_graph: Dict[str, List[str]] = defaultdict(list)
    for zone_name, zone in zone_items:
        if not zone_name:
            errors.append("Every screw zone must have a name.")
        parent = zone.get("parent_component", "")
        if parent and parent not in component_labels:
            errors.append(f"Screw zone '{zone_name}' references unknown parent component '{parent}'.")
        for dep in _as_list(zone.get("depends_on_removal_of")):
            if dep not in component_labels:
                errors.append(f"Screw zone '{zone_name}' depends on unknown component '{dep}'.")
            dependency_graph[zone_name].append(dep)

    # --- Sequence ---
    sequence = data.get("disassembly_sequence", [])

    if not sequence:
        errors.append("Disassembly sequence must contain at least one step.")
        return errors, warnings

    for step_index, step in enumerate(sequence, start=1):
        # Support both spec keys and legacy keys
        step_label = step.get("label") or step.get("name") or f"step_{step_index}"
        action = step.get("action") or step.get("type")
        params = step.get("parameters") or step.get("params") or {}
        target = step.get("target") or step.get("component") or ""

        if action not in ACTION_TYPES:
            errors.append(f"Step '{step_label}' has unsupported action '{action}'.")
            continue

        # Hardware limit checks
        grip_mm = params.get("grip_width_mm")
        if grip_mm is not None and float(grip_mm) > RG6_MAX_GRIP_MM:
            errors.append(
                f"Step '{step_label}' grip_width_mm {grip_mm} exceeds RG6 max {RG6_MAX_GRIP_MM} mm."
            )

        force_n = params.get("gripper_close_force_n") or params.get("grip_force_n")
        if force_n is not None and float(force_n) > RG6_MAX_FORCE_N:
            errors.append(
                f"Step '{step_label}' grip force {force_n} N exceeds RG6 max {RG6_MAX_FORCE_N} N."
            )

        # FT300 is xarm5 only — unscrew always uses xarm5 (hardware fact)
        if action not in ("unscrew",) and params.get("ft_sensor_required"):
            errors.append(f"Step '{step_label}' requests FT sensor but action '{action}' does not use xArm5.")

        # Component reference check
        if target and target not in component_labels and target not in dict(zone_items):
            warnings.append(f"Step '{step_label}' target '{target}' is not defined in components or zones.")

    if _detect_cycle(dependency_graph):
        errors.append("Circular dependency detected in screw-zone depends_on_removal_of relationships.")

    if not any((s.get("action") or s.get("type")) == "unscrew" for s in sequence):
        warnings.append("Sequence contains no unscrew steps.")

    if not any((s.get("action") or s.get("type")) == "pickup" for s in sequence):
        warnings.append("Sequence contains no pickup step; held parts may never be removed.")

    return errors, warnings
