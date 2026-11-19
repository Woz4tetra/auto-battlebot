"""Sprites stand a robot up with the pitch and ground roll the scene pipeline uses."""

import math
import random
from types import SimpleNamespace
from typing import Any
from unittest.mock import call

import pytest

GROUND_ROLL_UPRIGHT_DEG = 7.5
GROUND_ROLL_INVERTED_DEG = -4.0


def rotation_calls(blender_stubs: Any, inverted: bool) -> list[Any]:
    import render_sprites

    config = SimpleNamespace(
        ground_roll_upright=GROUND_ROLL_UPRIGHT_DEG, ground_roll_inverted=GROUND_ROLL_INVERTED_DEG
    )
    blender_stubs.mathutils.Matrix.Rotation.reset_mock()
    render_sprites.upright_matrix(config, inverted=inverted)
    return blender_stubs.mathutils.Matrix.Rotation.call_args_list


@pytest.mark.parametrize(
    ("inverted", "pitch_deg", "roll_deg"),
    [(False, -90.0, GROUND_ROLL_UPRIGHT_DEG), (True, 90.0, GROUND_ROLL_INVERTED_DEG)],
)
def test_upright_matrix_matches_pose_single_robot(
    blender_stubs: Any,
    monkeypatch: pytest.MonkeyPatch,
    inverted: bool,
    pitch_deg: float,
    roll_deg: float,
) -> None:
    from synthgen import robots

    assert rotation_calls(blender_stubs, inverted) == [
        call(pytest.approx(math.radians(pitch_deg)), 4, "X"),
        call(pytest.approx(math.radians(roll_deg)), 4, "Y"),
    ]
    robot = SimpleNamespace(
        config=SimpleNamespace(
            flat_ground=False,
            ground_roll_upright=GROUND_ROLL_UPRIGHT_DEG,
            ground_roll_inverted=GROUND_ROLL_INVERTED_DEG,
        ),
        meshes=[],
        parent=SimpleNamespace(),
    )
    monkeypatch.setattr(robots, "compute_ground_z", lambda *_: 0.0)
    monkeypatch.setattr(random, "random", lambda: 0.9 if inverted else 0.0)
    posed: list[tuple[float, ...]] = []
    blender_stubs.mathutils.Euler.side_effect = lambda rotation: posed.append(tuple(rotation))
    rand_cfg = SimpleNamespace(air_probability=0.0)
    robots.pose_single_robot(robot, rand_cfg, arena_radius=1.0)
    assert posed[0][0] == pytest.approx(math.radians(pitch_deg))
    assert posed[0][1] == pytest.approx(math.radians(roll_deg))
