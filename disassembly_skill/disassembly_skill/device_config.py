"""Typed device-config middleware for disassembly skills.

Supports both the legacy schema (device_class/device_model, list components/zones,
name/type/component/params) and the current spec schema (class/model, dict
components/zones, action/label/target/parameters).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

from disassembly_skill.config_builder.utils.validation import validate_config
from disassembly_skill.config_builder.utils.yaml_export import load_yaml


@dataclass
class DeviceInfo:
    device_class: str
    device_model: str
    dimensions_mm: Dict[str, float]
    material: str = "unknown"
    fixturing: str = "unknown"
    notes: str = ""


@dataclass
class Component:
    label: str
    type: str
    removable: bool
    notes: str = ""


@dataclass
class ScrewZone:
    zone_name: str
    parent_component: str
    screw_count: int
    screw_type: str
    depends_on_removal_of: List[str] = field(default_factory=list)
    notes: str = ""


@dataclass
class SequenceStep:
    # Spec-schema fields
    step: int = 0
    action: str = ""
    label: str = ""
    target: str = ""
    depends_on_steps: List[int] = field(default_factory=list)
    reveals: List[str] = field(default_factory=list)
    parameters: Dict[str, Any] = field(default_factory=dict)
    notes: str = ""

    # Legacy aliases (read-only properties for backward compat)
    @property
    def name(self) -> str:
        return self.label

    @property
    def type(self) -> str:
        return self.action

    @property
    def component(self) -> str:
        return self.target

    @property
    def zone_name(self) -> str:
        return self.target

    @property
    def params(self) -> Dict[str, Any]:
        return self.parameters


@dataclass
class DeviceConfig:
    device: DeviceInfo
    components: List[Component]
    screw_zones: List[ScrewZone]
    disassembly_sequence: List[SequenceStep]
    metadata: Dict[str, Any] = field(default_factory=dict)
    source_path: Optional[Path] = None
    warnings: List[str] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any], *, source_path: Optional[str | Path] = None) -> "DeviceConfig":
        errors, warnings = validate_config(data)
        if errors:
            raise ValueError("Invalid device config:\n- " + "\n- ".join(errors))

        device_raw = data.get("device", {})
        # Support both spec key (class) and legacy key (device_class)
        device_class = device_raw.get("class") or device_raw.get("device_class", "")
        device_model = device_raw.get("model") or device_raw.get("device_model", "")

        # Components: support dict (spec) or list (legacy)
        components_raw = data.get("components", {})
        if isinstance(components_raw, dict):
            components_list = list(components_raw.values())
        else:
            components_list = components_raw

        # Zones: support dict (spec) or list (legacy)
        zones_raw = data.get("screw_zones", {})
        if isinstance(zones_raw, dict):
            zones_list = [{"zone_name": k, **v} for k, v in zones_raw.items()]
        else:
            zones_list = zones_raw

        # Sequence: support spec keys (action/label/target/parameters) or
        # legacy keys (type/name/component/params)
        def _parse_step(item: Dict[str, Any], index: int) -> SequenceStep:
            action = item.get("action") or item.get("type", "")
            label = item.get("label") or item.get("name", "")
            target = item.get("target") or item.get("component") or item.get("zone_name", "")
            params = item.get("parameters") or item.get("params") or {}
            return SequenceStep(
                step=item.get("step", index),
                action=action,
                label=label,
                target=target,
                depends_on_steps=list(item.get("depends_on_steps", [])),
                reveals=list(item.get("reveals", [])),
                parameters=dict(params),
                notes=item.get("notes", ""),
            )

        return cls(
            device=DeviceInfo(
                device_class=device_class,
                device_model=device_model,
                dimensions_mm=device_raw.get("dimensions_mm", {}),
                material=device_raw.get("material", "unknown"),
                fixturing=device_raw.get("fixturing", "unknown"),
                notes=device_raw.get("notes", ""),
            ),
            components=[
                Component(
                    label=item.get("label", ""),
                    type=item.get("type", "other"),
                    removable=bool(item.get("removable", True)),
                    notes=item.get("notes", ""),
                )
                for item in components_list
            ],
            screw_zones=[
                ScrewZone(
                    zone_name=item.get("zone_name", ""),
                    parent_component=item.get("parent_component", ""),
                    screw_count=int(item.get("screw_count", 0)),
                    screw_type=item.get("screw_type", ""),
                    depends_on_removal_of=list(item.get("depends_on_removal_of", []) or []),
                    notes=item.get("notes", ""),
                )
                for item in zones_list
            ],
            disassembly_sequence=[
                _parse_step(item, i + 1)
                for i, item in enumerate(data.get("disassembly_sequence", []))
            ],
            metadata=dict(data.get("metadata", {}) or {}),
            source_path=Path(source_path) if source_path else None,
            warnings=warnings,
        )

    @classmethod
    def load(cls, path: str | Path) -> "DeviceConfig":
        config_path = Path(path)
        return cls.from_dict(load_yaml(config_path), source_path=config_path)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "device": {
                "class": self.device.device_class,
                "model": self.device.device_model,
                "dimensions_mm": self.device.dimensions_mm,
                "material": self.device.material,
                "fixturing": self.device.fixturing,
                "notes": self.device.notes,
            },
            "components": {
                component.label: {
                    "label": component.label,
                    "type": component.type,
                    "removable": component.removable,
                    "notes": component.notes,
                }
                for component in self.components
            },
            "screw_zones": {
                zone.zone_name: {
                    "parent_component": zone.parent_component,
                    "screw_count": zone.screw_count,
                    "screw_type": zone.screw_type,
                    "depends_on_removal_of": zone.depends_on_removal_of,
                    "notes": zone.notes,
                }
                for zone in self.screw_zones
            },
            "disassembly_sequence": [
                {
                    "step": step.step,
                    "action": step.action,
                    "label": step.label,
                    "target": step.target,
                    "depends_on_steps": step.depends_on_steps,
                    "reveals": step.reveals,
                    "parameters": step.parameters,
                    "notes": step.notes,
                }
                for step in self.disassembly_sequence
            ],
            "metadata": self.metadata,
        }

    def component(self, label: str) -> Optional[Component]:
        return next((c for c in self.components if c.label == label), None)

    def screw_zone(self, zone_name: str) -> Optional[ScrewZone]:
        return next((z for z in self.screw_zones if z.zone_name == zone_name), None)

    def step(self, label: str) -> Optional[SequenceStep]:
        return next((s for s in self.disassembly_sequence if s.label == label), None)

    def to_llm_context(self) -> Dict[str, Any]:
        return {
            "device_model": self.device.device_model,
            "device_class": self.device.device_class,
            "fixturing": self.device.fixturing,
            "materials": self.device.material,
            "components": [
                {
                    "label": c.label,
                    "type": c.type,
                    "removable": c.removable,
                }
                for c in self.components
            ],
            "screw_zones": [
                {
                    "zone_name": z.zone_name,
                    "parent_component": z.parent_component,
                    "screw_count": z.screw_count,
                }
                for z in self.screw_zones
            ],
            "sequence_overview": [
                {
                    "step": s.step,
                    "action": s.action,
                    "type": s.action,
                    "label": s.label,
                    "target": s.target,
                    "component": s.target,
                    "reveals": s.reveals,
                }
                for s in self.disassembly_sequence
            ],
            "warnings": list(self.warnings),
        }
