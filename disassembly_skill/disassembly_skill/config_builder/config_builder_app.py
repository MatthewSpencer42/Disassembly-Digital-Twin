#!/usr/bin/env python3
"""Minimal config-builder entry point for the base branch."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def main() -> None:
    default_config = (
        Path(get_package_share_directory("disassembly_skill"))
        / "config"
        / "device_configs"
        / "hdd.yaml"
    )
    print("Device config builder GUI is not included on this base branch.")
    print(f"Default editable config: {default_config}")
