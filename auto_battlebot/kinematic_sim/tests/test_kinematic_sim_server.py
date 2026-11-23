"""kinematic_sim_server constants that mirror the C++ side."""

from __future__ import annotations

import math
import re
import subprocess
import sys

import pytest

from auto_battlebot.kinematic_sim import kinematic_sim_server
from auto_battlebot.paths import REPO_ROOT


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


def recv_exact(sock: object, size: int) -> bytes:
    import socket

    assert isinstance(sock, socket.socket)
    data = b""
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        assert chunk, "server closed the connection"
        data += chunk
    return data


def test_wrapper_script_serves_a_frame_to_the_cpp_wire_format(tmp_path) -> None:
    import socket
    import struct
    import time

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    config = (REPO_ROOT / "simulation" / "sim_mrs_buff_mk3.toml").read_text()
    config = re.sub(r"(?m)^port = \d+$", f"port = {port}", config, count=1)
    (tmp_path / "sim.toml").write_text(config)
    server = subprocess.Popen(
        [sys.executable, "simulation/kinematic_sim_server.py", str(tmp_path / "sim.toml")],
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    try:
        deadline = time.monotonic() + 60.0
        while True:
            try:
                sock = socket.create_connection(("127.0.0.1", port), timeout=30.0)
                break
            except OSError:
                assert server.poll() is None, server.stderr.read() if server.stderr else ""
                assert time.monotonic() < deadline, "server never listened"
                time.sleep(0.2)
        with sock:
            sock.sendall(struct.pack("<3d", 0.5, 0.0, 0.0))
            header = recv_exact(sock, 4 + 4 + 16 * 8 + 5 * 8)
            width, height = struct.unpack_from("<II", header)
            fx, _fy, _cx, _cy, sim_time = struct.unpack_from("<5d", header, 8 + 16 * 8)
            assert (width, height) == (640, 360)
            assert fx > 0.0 and sim_time >= 0.0
            recv_exact(sock, width * height * 3 + width * height * 4)
            (robots,) = struct.unpack("<I", recv_exact(sock, 4))
            assert robots >= 2
            poses = struct.unpack(f"<{3 * robots}d", recv_exact(sock, 24 * robots))
            assert all(math.isfinite(v) for v in poses)
    finally:
        server.kill()
        server.wait(timeout=10)
