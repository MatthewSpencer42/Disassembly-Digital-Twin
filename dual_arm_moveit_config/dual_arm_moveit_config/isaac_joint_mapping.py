"""Joint mapping between the legacy Isaac USD and the active robot model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


SLIDER_ACTIVE_ZERO_M = 0.054

RG6_ACTIVE_OPEN_RAD = -0.625
RG6_ACTIVE_CLOSE_RAD = 0.625
RG6_ISAAC_OPEN_RAD = 0.610865
RG6_ISAAC_CLOSE_RAD = -0.75


@dataclass(frozen=True)
class JointTransform:
    source_name: str
    target_name: str
    position_scale: float = 1.0
    position_offset: float = 0.0

    def position(self, value: float) -> float:
        return self.position_scale * float(value) + self.position_offset

    def velocity(self, value: float) -> float:
        return self.position_scale * float(value)

    def effort(self, value: float) -> float:
        return float(value) / self.position_scale

    def inverse(self) -> "JointTransform":
        return JointTransform(
            source_name=self.target_name,
            target_name=self.source_name,
            position_scale=1.0 / self.position_scale,
            position_offset=-self.position_offset / self.position_scale,
        )


@dataclass(frozen=True)
class TranslatedJointSample:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]


def _rg6_isaac_to_active_transform() -> JointTransform:
    scale = (
        (RG6_ACTIVE_CLOSE_RAD - RG6_ACTIVE_OPEN_RAD)
        / (RG6_ISAAC_CLOSE_RAD - RG6_ISAAC_OPEN_RAD)
    )
    offset = RG6_ACTIVE_OPEN_RAD - scale * RG6_ISAAC_OPEN_RAD
    return JointTransform(
        source_name="rg6_l_out",
        target_name="rg6_right_drive_joint",
        position_scale=scale,
        position_offset=offset,
    )


ISAAC_TO_ACTIVE_TRANSFORMS = (
    JointTransform(
        "slider_slider_joint",
        "uf_slide_joint",
        position_scale=-1.0,
        position_offset=SLIDER_ACTIVE_ZERO_M,
    ),
    *(JointTransform(f"u1_joint{i}", f"uf850_joint{i}") for i in range(1, 7)),
    *(JointTransform(f"xarm5_joint{i}", f"xarm5_joint{i}") for i in range(1, 6)),
    _rg6_isaac_to_active_transform(),
)

ACTIVE_TO_ISAAC_TRANSFORMS = tuple(
    transform.inverse() for transform in ISAAC_TO_ACTIVE_TRANSFORMS
)


def translate_joint_sample(
    names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float],
    efforts: Sequence[float],
    transforms: Sequence[JointTransform],
) -> TranslatedJointSample:
    """Translate joint arrays by name, preserving complete optional fields."""
    source_index = {name: index for index, name in enumerate(names)}
    selected = [
        (transform, source_index[transform.source_name])
        for transform in transforms
        if transform.source_name in source_index
    ]

    have_positions = len(positions) >= len(names)
    have_velocities = len(velocities) >= len(names)
    have_efforts = len(efforts) >= len(names)

    return TranslatedJointSample(
        names=tuple(transform.target_name for transform, _ in selected),
        positions=(
            tuple(transform.position(positions[index]) for transform, index in selected)
            if have_positions
            else ()
        ),
        velocities=(
            tuple(transform.velocity(velocities[index]) for transform, index in selected)
            if have_velocities
            else ()
        ),
        efforts=(
            tuple(transform.effort(efforts[index]) for transform, index in selected)
            if have_efforts
            else ()
        ),
    )
