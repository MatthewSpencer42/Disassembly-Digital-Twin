import math

from dual_arm_moveit_config.isaac_joint_mapping import (
    ACTIVE_TO_ISAAC_TRANSFORMS,
    ISAAC_TO_ACTIVE_TRANSFORMS,
    RG6_ACTIVE_CLOSE_RAD,
    RG6_ACTIVE_OPEN_RAD,
    RG6_ISAAC_CLOSE_RAD,
    RG6_ISAAC_OPEN_RAD,
    translate_joint_sample,
)


def _translate_position(name, value, transforms):
    result = translate_joint_sample(
        [name],
        [value],
        [],
        [],
        transforms,
    )
    assert len(result.names) == 1
    assert len(result.positions) == 1
    return result.names[0], result.positions[0]


def test_isaac_state_joint_names_and_slider_coordinate():
    result = translate_joint_sample(
        ["slider_slider_joint", "u1_joint3", "xarm5_joint2"],
        [-0.2, -1.25, 0.4],
        [0.1, 0.2, 0.3],
        [],
        ISAAC_TO_ACTIVE_TRANSFORMS,
    )

    assert result.names == (
        "uf_slide_joint",
        "uf850_joint3",
        "xarm5_joint2",
    )
    assert math.isclose(result.positions[0], 0.254)
    assert math.isclose(result.positions[1], -1.25)
    assert math.isclose(result.positions[2], 0.4)
    assert math.isclose(result.velocities[0], -0.1)


def test_rg6_endpoints_map_in_both_directions():
    _, active_open = _translate_position(
        "rg6_l_out",
        RG6_ISAAC_OPEN_RAD,
        ISAAC_TO_ACTIVE_TRANSFORMS,
    )
    _, active_close = _translate_position(
        "rg6_l_out",
        RG6_ISAAC_CLOSE_RAD,
        ISAAC_TO_ACTIVE_TRANSFORMS,
    )
    _, isaac_open = _translate_position(
        "rg6_right_drive_joint",
        RG6_ACTIVE_OPEN_RAD,
        ACTIVE_TO_ISAAC_TRANSFORMS,
    )
    _, isaac_close = _translate_position(
        "rg6_right_drive_joint",
        RG6_ACTIVE_CLOSE_RAD,
        ACTIVE_TO_ISAAC_TRANSFORMS,
    )

    assert math.isclose(active_open, RG6_ACTIVE_OPEN_RAD)
    assert math.isclose(active_close, RG6_ACTIVE_CLOSE_RAD)
    assert math.isclose(isaac_open, RG6_ISAAC_OPEN_RAD)
    assert math.isclose(isaac_close, RG6_ISAAC_CLOSE_RAD)


def test_round_trip_positions():
    active_names = [
        "uf_slide_joint",
        "uf850_joint1",
        "xarm5_joint4",
        "rg6_right_drive_joint",
    ]
    active_positions = [0.31, 0.2, -0.4, 0.15]

    isaac = translate_joint_sample(
        active_names,
        active_positions,
        [],
        [],
        ACTIVE_TO_ISAAC_TRANSFORMS,
    )
    restored = translate_joint_sample(
        isaac.names,
        isaac.positions,
        [],
        [],
        ISAAC_TO_ACTIVE_TRANSFORMS,
    )

    assert restored.names == tuple(active_names)
    for actual, expected in zip(restored.positions, active_positions):
        assert math.isclose(actual, expected)
