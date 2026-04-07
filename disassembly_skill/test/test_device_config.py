from pathlib import Path

import pytest

from disassembly_skill.config_builder.utils.validation import validate_config
from disassembly_skill.device_config import DeviceConfig


FIXTURE = (
    Path(__file__).resolve().parents[1] / "config" / "device_configs" / "device_config_example_hdd.yaml"
)


def test_device_config_loads_example():
    config = DeviceConfig.load(FIXTURE)

    assert config.device.device_model == "hdd_generic_3_5"
    assert len(config.components) == 3
    assert len(config.screw_zones) == 1
    assert config.step("remove_top_cover_screws").type == "unscrew"
    assert any("active hold" in item.lower() for item in config.warnings)


def test_to_llm_context_exposes_sequence_summary():
    config = DeviceConfig.load(FIXTURE)

    llm_context = config.to_llm_context()

    assert llm_context["device_model"] == "hdd_generic_3_5"
    first_step = llm_context["sequence_overview"][0]
    assert first_step.get("type", first_step["action"]) == "hold"
    assert any(item["zone_name"] == "top_cover_screws" for item in llm_context["screw_zones"])


def test_validation_rejects_unscrew_without_hold():
    bad = {
        "device": {"device_class": "storage_device", "device_model": "bad", "dimensions_mm": {}},
        "components": [{"label": "chassis", "type": "chassis", "removable": False}],
        "screw_zones": [],
        "disassembly_sequence": [
            {"name": "remove", "type": "unscrew", "component": "chassis", "zone_name": "zone_1", "params": {"arm": "xarm5"}}
        ],
        "metadata": {},
    }

    errors, warnings = validate_config(bad)

    assert any("first disassembly step must be a hold" in item.lower() for item in errors)
    assert any("requires an active hold" in item.lower() for item in errors)
    assert any("no pickup step" in item.lower() for item in warnings)


def test_validation_rejects_invalid_ft300_arm():
    bad = {
        "device": {"device_class": "storage_device", "device_model": "bad", "dimensions_mm": {}},
        "components": [{"label": "chassis", "type": "chassis", "removable": False}],
        "screw_zones": [],
        "disassembly_sequence": [
            {"name": "hold", "type": "hold", "component": "chassis", "params": {"arm": "uf850", "ft_sensor_required": True}}
        ],
        "metadata": {},
    }

    with pytest.raises(ValueError):
        DeviceConfig.from_dict(bad)
