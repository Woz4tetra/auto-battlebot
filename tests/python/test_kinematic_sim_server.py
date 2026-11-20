"""kinematic_sim_server constants that mirror the C++ side."""

from __future__ import annotations

import math
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]

from auto_battlebot.kinematic_sim import kinematic_sim_server


def test_default_opponent_radius() -> None:
    source = (REPO_ROOT / "src" / "robot_filter" / "ground_truth_robot_filter.cpp").read_text()
    length, width = (float(v) for v in re.search(r"Size\{([\d.]+), ([\d.]+),", source).groups())
    assert kinematic_sim_server.DEFAULT_OPPONENT_RADIUS_M == pytest.approx(
        0.5 * math.hypot(length, width), abs=5e-4
    )


def _server():
    from auto_battlebot.kinematic_sim.config.kinematic import KinematicSimConfig, OpponentConfig

    cfg = KinematicSimConfig()
    cfg.opponents = [OpponentConfig(behavior="static", start_pos=[0.5, 0.5])]
    server = kinematic_sim_server.KinematicServer(cfg)
    server._reset()
    return server


def test_opponent_spawned_in_hazard_is_pushed_out() -> None:
    import numpy as np

    from auto_battlebot.kinematic_sim.config.kinematic import ObstacleConfig, OpponentConfig

    hole = ObstacleConfig(kind="hole", center=[0.5, 0.5], radius=0.25)
    opponent = kinematic_sim_server.Opponent(
        OpponentConfig(behavior="static", start_pos=[0.5, 0.5]),
        2.0,
        2.0,
        np.random.default_rng(0),
        obstacles=[hole],
    )
    assert math.hypot(opponent.x - 0.5, opponent.y - 0.5) >= 0.25 + 0.11 - 1e-6


def test_respawn_keeps_time_and_opponents() -> None:
    server = _server()
    server._sim_time = 12.5
    server._tick = 250
    server._opponents[0].x = -0.3
    server._respawn()
    assert server._sim_time == 12.5
    assert server._tick == 250
    assert server._opponents[0].x == -0.3


def test_render_camera_keeps_buffer_identity() -> None:
    server = _server()
    buffer = server._rgb
    server._render_camera()
    assert server._rgb is buffer
    assert server._rgb.flags["C_CONTIGUOUS"]


def test_plant_moving_blocks_follow_runtime_opponent_order() -> None:
    from auto_battlebot.kinematic_sim.config.kinematic import KinematicSimConfig, OpponentConfig

    cfg = KinematicSimConfig()
    cfg.opponents = [
        OpponentConfig(behavior="static", start_pos=[0.5, 0.5], hazard_radius=0.0),
        OpponentConfig(behavior="static", start_pos=[-0.5, 0.5], hazard_radius=0.2),
        OpponentConfig(behavior="static", start_pos=[0.5, -0.5], hazard_radius=0.0),
        OpponentConfig(behavior="static", start_pos=[-0.5, -0.5], hazard_radius=0.3),
    ]
    server = kinematic_sim_server.KinematicServer(cfg)
    captured = []
    real_make_plant = kinematic_sim_server.make_plant

    def spy(*args):
        captured.append(args[-1])
        return real_make_plant(*args)

    kinematic_sim_server.make_plant = spy
    try:
        server._reset()
    finally:
        kinematic_sim_server.make_plant = real_make_plant
    runtime = [(o.x, o.y, o.hazard_radius) for o in server._opponents if o.hazard_radius > 0.0]
    built = [(o.start_pos[0], o.start_pos[1], o.hazard_radius) for o in captured[-1]]
    assert built == runtime


def test_moving_block_poses_match_the_blocks_the_plant_was_built_with() -> None:
    from auto_battlebot.kinematic_sim.config.kinematic import KinematicSimConfig, OpponentConfig

    cfg = KinematicSimConfig()
    cfg.opponents = [
        OpponentConfig(behavior="static", start_pos=[0.5, 0.5], hazard_radius=0.0),
        OpponentConfig(behavior="static", start_pos=[-0.5, 0.5], hazard_radius=0.2),
        OpponentConfig(behavior="static", start_pos=[-0.5, -0.5], hazard_radius=0.3),
    ]
    server = kinematic_sim_server.KinematicServer(cfg)
    server._reset()
    poses = server._moving_block_poses()
    assert [(x, y, r) for x, y, _, r in poses] == [(-0.5, 0.5, 0.2), (-0.5, -0.5, 0.3)]


def test_importing_the_plants_package_does_not_import_mujoco() -> None:
    code = "import sys, auto_battlebot.kinematic_sim.plants; sys.exit(int('mujoco' in sys.modules))"
    result = subprocess.run([sys.executable, "-c", code], cwd=REPO_ROOT, check=False)
    assert result.returncode == 0


def test_rl_eval_server_skips_the_camera_compositor() -> None:
    from unittest import mock

    from auto_battlebot.kinematic_sim import rl_eval_server

    server = object.__new__(rl_eval_server.RlEvalServer)
    with mock.patch.object(server, "_world", create=True) as world:
        server._render_camera()
    world.compose_world.assert_not_called()
