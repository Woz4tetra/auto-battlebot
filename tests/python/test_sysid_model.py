"""Sysid model layout, shape checks, and synthetic-session and telemetry contracts."""

from __future__ import annotations

import re
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from auto_battlebot.calibration.sysid import model, synthetic
from auto_battlebot.calibration.sysid.pose_stream import TagPose, state_at
from auto_battlebot.calibration.sysid.profile import load_profile
from auto_battlebot.calibration.sysid.telemetry import FLIP_SWITCH_DOWN, LiveClock
from auto_battlebot.control.plant import PARAM_BOUNDS
from auto_battlebot.recording.esp32_clock import fit_robot_clock
from auto_battlebot.recording.sysid_io import parse_robot_tags

REPO = Path(__file__).resolve().parents[2]
TAG_SIGMA = (0.006, 0.002, 0.01)


def _truth_subs() -> tuple[np.ndarray, list[model.SubWindow]]:
    truth = synthetic.TRUTH.to_vector(model.NAMES)
    _travel_subs = model.expected_subwindows(truth, model.CATALOG[0], False, TAG_SIGMA)
    return truth, _travel_subs[0]


def test_hist_covers_delay_bound() -> None:
    assert model.HIST_S >= PARAM_BOUNDS["delay_s"][1]


def test_subwindow_rejects_wrong_state_shape() -> None:
    _, subs = _truth_subs()
    with pytest.raises(ValueError, match="state0"):
        replace(subs[0], state0=np.zeros(4))


def test_subwindow_rejects_wrong_command_length() -> None:
    _, subs = _truth_subs()
    with pytest.raises(ValueError, match="cmd_lin"):
        replace(subs[0], cmd_lin=subs[0].cmd_lin[:-1])


def test_shift_commands_fractional_delay() -> None:
    cmd = np.zeros((1, model.HIST_N + model.SUB_N))
    cmd[0, model.HIST_N :] = 1.0
    out = model._delayed(cmd, 0.035)
    assert out.shape == (1, model.SUB_N)
    assert np.allclose(out[0, :3], 0.0)
    assert out[0, 3] == pytest.approx(0.5)
    assert np.allclose(out[0, 4:], 1.0)


def test_residual_rows_tag_block_first() -> None:
    truth, subs = _truth_subs()
    batch = model.stack(subs, list(range(len(subs))), 0.3)
    base = model.residuals(truth, batch)
    assert base.shape[1] == model.TAG_ROWS + model.GYRO_N + model.NUISANCE
    batch.gyro[:] += 1.0
    gyro_moved = np.abs(model.residuals(truth, batch) - base).max(axis=0) > 1e-9
    assert not gyro_moved[: model.TAG_ROWS].any()
    assert gyro_moved[model.TAG_ROWS : model.TAG_ROWS + model.GYRO_N].all()


def test_tag_tau_residual_zero_off_grid() -> None:
    truth, subs = _truth_subs()
    for sub in subs:
        assert np.allclose(sub.tag_tau_s / model.DT_S, np.round(sub.tag_tau_s / model.DT_S))
    batch = model.stack(subs, list(range(len(subs))), 0.3)
    tag_residuals = model.residuals(truth, batch)[:, : model.TAG_ROWS]
    assert np.abs(tag_residuals).max() < 0.5


def test_true_delay_on_grid() -> None:
    steps = synthetic.TRUTH.delay_s / synthetic.DT
    assert steps == pytest.approx(round(steps))


def test_synthetic_tag_pose_matches_stamp() -> None:
    sim = synthetic.simulate_session(load_profile(), seconds=20.0, seed=6)
    truth = {round(r.t_s * 1e9): r for r in sim.truth}
    camera = synthetic.camera_pose()
    offsets = []
    for frame in sim.frames:
        parsed = parse_robot_tags(frame.payload, frame.log_ns)
        row = truth[parsed.image_stamp_ns - synthetic.T0_NS]
        for det in parsed.detections:
            centre = camera @ np.append(det.solutions[0].tvec_camera_m, 1.0)
            dx, dy = centre[0] - row.x_m, centre[1] - row.y_m
            c, s = np.cos(row.yaw_rad), np.sin(row.yaw_rad)
            offsets.append((c * dx + s * dy, -s * dx + c * dy))
    assert len(offsets) > 500
    assert np.asarray(offsets).std(axis=0).max() < 0.004


def test_flip_switch_down_matches_firmware() -> None:
    header = (REPO / "firmware/mr_stabs_mk2/include/crsf_bridge.h").read_text()
    match = re.search(r"typedef enum three_state_switch\s*\{\s*DOWN\s*=\s*(\d+)", header)
    assert match is not None
    assert FLIP_SWITCH_DOWN == int(match.group(1))


def test_live_clock_matches_fit_robot_clock() -> None:
    rng = np.random.default_rng(0)
    robot = np.arange(0, 20_000, 20, dtype=np.int64)
    host = 5_000_000_000 + (robot * 1_000_000 * (1.0 + 30e-6)).astype(np.int64)
    host = host + rng.integers(0, 4_000_000, robot.shape)
    clock = LiveClock(window_s=600.0, refit_s=0.001)
    for r, h in zip(robot, host):
        clock.push(int(r), int(h))
    offline = fit_robot_clock(robot, host).segments[-1]
    probe = np.array([5_000, 15_000])
    assert np.allclose(clock.to_host_ns(probe), offline.to_host_ns(probe), atol=1_000)


def _pose(stamp_ns: int, x: float) -> TagPose:
    return TagPose(
        stamp_ns=stamp_ns,
        x=x,
        y=0.0,
        yaw=0.0,
        pitch_from_rest_rad=0.0,
        upside_down=False,
        sigma_along=0.01,
        sigma_across=0.01,
        sigma_yaw=0.01,
    )


def test_state_at_requires_both_sides() -> None:
    stamps = [-60, -40, -20, 0, 20]
    poses = [_pose(ms * 1_000_000, 0.5 * ms / 1000.0) for ms in stamps]
    assert state_at(poses, 20_000_000) is None
    balanced = poses + [_pose(40_000_000, 0.02)]
    est = state_at(balanced, 0)
    assert est is not None
    assert est.v == pytest.approx(0.5, abs=0.05)
