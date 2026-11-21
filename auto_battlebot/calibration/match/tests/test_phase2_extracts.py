from __future__ import annotations

import math
from typing import Any

import numpy as np

from auto_battlebot.calibration.jig.drive_protocol import TrainerLink
from auto_battlebot.calibration.match.match_windows import (
    CommandLog,
    central_difference_velocity,
    gate_acceleration_spikes,
    hold_commands_on_grid,
)
from auto_battlebot.calibration.sysid.coverage import straight_run_room_m
from auto_battlebot.control.firmware_mixer import (
    LoopState,
    YawConfig,
    _capturing_mask,
    _update_capture,
)
from auto_battlebot.mujoco_sim.firmware import FirmwareMixer, LegacyHoldState, YawLoopState
from auto_battlebot.perception.camera_geometry import FrameGeometry, _pixels_to_field_rays
from auto_battlebot.perception.trt_yolo import TrtYoloModel


class _FakeSerial:
    def __init__(self) -> None:
        self.written: list[bytes] = []

    def write(self, data: bytes) -> None:
        self.written.append(data)


def test_prime_radio_turns_on_telemetry_and_channels() -> None:
    link: Any = object.__new__(TrainerLink)
    link._serial = _FakeSerial()
    link._prime_radio()
    assert link._serial.written == [b"telemetry on\r\n", b"channels on\r\n"]


def test_hold_commands_on_grid_blanks_outside_the_log_span() -> None:
    commands = CommandLog(
        t_ns=np.array([1_000, 2_000], dtype=np.int64),
        lin=np.array([0.5, -0.5]),
        ang=np.array([0.1, 0.2]),
        auto_switch_latched=np.array([True, True]),
    )
    stamps = np.array([500, 1_000, 1_500, 2_000, 2_000 + 10**9, 2_001 + 10**9], dtype=np.int64)
    lin, ang, auto = hold_commands_on_grid(commands, stamps)
    assert auto.tolist() == [False, True, True, True, True, False]
    assert lin[1:5].tolist() == [0.5, 0.5, -0.5, -0.5]
    assert ang[1:3].tolist() == [0.1, 0.1]


def test_central_difference_velocity_uses_one_sided_edges_and_skips_isolated() -> None:
    dt = 0.1
    x = np.arange(8) * 0.2
    y = np.zeros(8)
    theta = np.zeros(8)
    usable = np.array([True, True, True, True, False, True, False, False])
    v, w, both = central_difference_velocity(x, y, theta, usable, dt)
    assert both.tolist() == [False, True, True, False, False, False, False, False]
    assert np.allclose(v[:4], 2.0)
    assert np.allclose(w[:4], 0.0)
    assert np.isnan(v[4:]).all()


def test_gate_acceleration_spikes_gates_the_neighbors_too() -> None:
    n = 9
    v = np.zeros(n)
    w = np.zeros(n)
    v[5:] = 10.0
    both = np.ones(n, dtype=bool)
    spike = gate_acceleration_spikes(v, w, both, 0.01)
    assert spike.tolist() == [False, False, False, True, True, True, False, False, False]
    assert not gate_acceleration_spikes(np.zeros(n), w, both, 0.01).any()


def test_straight_run_room_subtracts_a_robot_length_and_stops() -> None:
    assert math.isclose(straight_run_room_m(3.0), 2.4)
    assert straight_run_room_m(0.3) == 0.0


def test_capturing_mask_needs_a_released_zero_rate_turn_with_heading_offset() -> None:
    active = np.array([True, True, True, True, False])
    turning = np.array([False, True, False, False, False])
    idle = np.array([False, False, True, False, False])
    rate_cmd = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    offset = np.array([2.0, 2.0, 2.0, 2.0, 2.0])
    mask = _capturing_mask(active, turning, idle, rate_cmd, offset)
    assert mask.tolist() == [True, False, False, False, False]
    assert not _capturing_mask(active, turning, idle, rate_cmd, np.zeros(5)).any()


def test_update_capture_holds_where_the_released_turn_stopped() -> None:
    c = YawConfig()
    s = LoopState.idle(2)
    s.capturing[:] = True
    s.setpoint[:] = 10.0
    turning = np.array([False, False])
    idle = np.array([False, False])
    rate_dps = np.array([c.capture_rate * 0.5, c.capture_rate * 10.0])
    heading = np.array([90.0, 90.0])
    _update_capture(s, c, turning, idle, rate_dps, heading, 0.01)
    assert s.capturing.tolist() == [False, True]
    assert s.setpoint.tolist() == [90.0, 10.0]
    assert s.capture_time.tolist() == [0.01, 0.01]
    _update_capture(s, c, np.array([True, True]), idle, rate_dps, heading, 0.01)
    assert s.capturing.tolist() == [True, True]
    assert s.capture_time.tolist() == [0.0, 0.0]


def test_firmware_mixer_state_groups_start_cleared_and_arm_on_step() -> None:
    mixer = FirmwareMixer()
    assert mixer.hold == LegacyHoldState()
    assert mixer.loop == YawLoopState()
    mixer.step(a_percent=0.0, b_percent=50.0, heading_deg=0.0, dt=0.01)
    assert mixer.loop.active


def test_pixels_to_field_rays_rotates_the_pinhole_ray_into_field_frame() -> None:
    tf = np.eye(4)
    tf[:3, :3] = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    geometry = FrameGeometry(0, 100.0, 100.0, 50.0, 50.0, tf, "t", (1.0, 1.0))
    rays = _pixels_to_field_rays(np.array([[50.0, 50.0], [150.0, 50.0]]), geometry)
    assert np.allclose(rays[0], [0.0, -1.0, 0.0])
    assert np.allclose(rays[1], [1.0, -1.0, 0.0])


class _FakeEngine:
    def __init__(self, names: list[str]) -> None:
        self.names = names
        self.num_io_tensors = len(names)

    def get_tensor_name(self, i: int) -> str:
        return self.names[i]


class _FakeContext:
    def __init__(self) -> None:
        self.addresses: dict[str, int] = {}

    def get_tensor_shape(self, name: str) -> tuple[int, ...]:
        return (1, 3, 4, 4) if name == "images" else (1, 8, 2)

    def set_tensor_address(self, name: str, address: int) -> None:
        self.addresses[name] = address


class _FakeCuda:
    def __init__(self) -> None:
        self.sizes: list[int] = []

    def mem_alloc(self, nbytes: int) -> int:
        self.sizes.append(nbytes)
        return 1000 + len(self.sizes)


def test_bind_all_io_tensors_gives_every_tensor_an_address() -> None:
    detector: Any = object.__new__(TrtYoloModel)
    detector.engine = _FakeEngine(["images", "output0", "protos"])
    detector.context = _FakeContext()
    detector._cuda = _FakeCuda()
    buffers, d_input, d_output = detector._bind_all_io_tensors("images", "output0")
    assert buffers == [1001, 1002, 1003]
    assert (d_input, d_output) == (1001, 1002)
    assert detector.context.addresses == {"images": 1001, "output0": 1002, "protos": 1003}
    assert detector._cuda.sizes == [3 * 4 * 4 * 4, 8 * 2 * 4, 8 * 2 * 4]
