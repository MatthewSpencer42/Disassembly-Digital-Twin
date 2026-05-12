"""Typed device configuration loader for deterministic disassembly runs."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class DeviceInfo:
    device_class: str
    device_model: str
    dimensions_mm: dict[str, float]
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
    depends_on_removal_of: list[str] = field(default_factory=list)
    notes: str = ""


@dataclass
class SequenceStep:
    step: int
    action: str
    label: str
    target: str
    depends_on_steps: list[int] = field(default_factory=list)
    reveals: list[str] = field(default_factory=list)
    parameters: dict[str, Any] = field(default_factory=dict)
    notes: str = ""

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
    def params(self) -> dict[str, Any]:
        return self.parameters


@dataclass
class DeviceConfig:
    device: DeviceInfo
    components: list[Component]
    screw_zones: list[ScrewZone]
    disassembly_sequence: list[SequenceStep]
    metadata: dict[str, Any] = field(default_factory=dict)
    source_path: Path | None = None
    warnings: list[str] = field(default_factory=list)

    @classmethod
    def load(cls, path: str | Path) -> "DeviceConfig":
        config_path = Path(path)
        with config_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        return cls.from_dict(data, source_path=config_path)

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, source_path: str | Path | None = None) -> "DeviceConfig":
        errors, warnings = _validate_config(data)
        if errors:
            raise ValueError("Invalid device config:\n- " + "\n- ".join(errors))

        device_raw = data.get("device", {}) or {}
        components_raw = data.get("components", {}) or {}
        zones_raw = data.get("screw_zones", {}) or {}
        sequence_raw = data.get("disassembly_sequence", []) or []

        components_iter = components_raw.values() if isinstance(components_raw, dict) else components_raw
        zones_iter = (
            [{"zone_name": name, **value} for name, value in zones_raw.items()]
            if isinstance(zones_raw, dict)
            else zones_raw
        )

        return cls(
            device=DeviceInfo(
                device_class=device_raw.get("class") or device_raw.get("device_class", ""),
                device_model=device_raw.get("model") or device_raw.get("device_model", ""),
                dimensions_mm=dict(device_raw.get("dimensions_mm", {}) or {}),
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
                for item in components_iter
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
                for item in zones_iter
            ],
            disassembly_sequence=[
                _parse_step(item, index + 1)
                for index, item in enumerate(sequence_raw)
            ],
            metadata=dict(data.get("metadata", {}) or {}),
            source_path=Path(source_path) if source_path else None,
            warnings=warnings,
        )

    def screw_zone(self, zone_name: str) -> ScrewZone | None:
        return next((zone for zone in self.screw_zones if zone.zone_name == zone_name), None)

    def step(self, label: str) -> SequenceStep | None:
        return next((step for step in self.disassembly_sequence if step.label == label), None)

    def to_llm_context(self) -> dict[str, Any]:
        return {
            "device_model": self.device.device_model,
            "device_class": self.device.device_class,
            "components": [component.__dict__ for component in self.components],
            "screw_zones": [zone.__dict__ for zone in self.screw_zones],
            "sequence_overview": [
                {
                    "step": step.step,
                    "action": step.action,
                    "type": step.action,
                    "label": step.label,
                    "target": step.target,
                    "component": step.target,
                    "reveals": step.reveals,
                    "parameters": step.parameters,
                }
                for step in self.disassembly_sequence
            ],
        }


def _parse_step(item: dict[str, Any], index: int) -> SequenceStep:
    return SequenceStep(
        step=int(item.get("step", index)),
        action=item.get("action") or item.get("type", ""),
        label=item.get("label") or item.get("name", ""),
        target=item.get("target") or item.get("component") or item.get("zone_name", ""),
        depends_on_steps=list(item.get("depends_on_steps", []) or []),
        reveals=list(item.get("reveals", []) or []),
        parameters=dict(item.get("parameters") or item.get("params") or {}),
        notes=item.get("notes", ""),
    )


def _validate_config(data: dict[str, Any]) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []

    if not data.get("device"):
        errors.append("Missing device section")
    sequence = data.get("disassembly_sequence", []) or []
    if not sequence:
        errors.append("Missing disassembly_sequence")
        return errors, warnings

    first_action = (sequence[0].get("action") or sequence[0].get("type") or "").lower()
    if first_action != "hold":
        errors.append("First disassembly step must be a hold")

    has_hold = False
    has_pickup = False
    for item in sequence:
        action = (item.get("action") or item.get("type") or "").lower()
        has_hold = has_hold or action == "hold"
        has_pickup = has_pickup or action == "pickup"
        if action == "unscrew" and not has_hold:
            errors.append("Unscrew step requires an active hold before it")

    if not has_pickup:
        warnings.append("No pickup step configured")
    if (sequence[-1].get("action") or sequence[-1].get("type") or "").lower() == "hold":
        warnings.append("Sequence ends with an active hold; ensure release behaviour is intentional.")

    return errors, warnings
