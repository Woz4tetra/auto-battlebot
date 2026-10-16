"""The batched sensing model: sampler statistics, the delivery queue, and file validation."""

from __future__ import annotations

import copy
import math
from typing import Any

import numpy as np
import pytest
import torch

from auto_battlebot.perception.sensing_model import (
    Context,
    DurationTable,
    MeasurementModel,
    OurEkfErrorModel,
    SensingModelFile,
    TruePoses,
    _Durations,
)

QUANTILES = [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99, 1.0]
MEASURED = [0.03, 0.05, 0.1, 0.3, 0.8, 2.0, 6.0, 12.0]
DROPPED = [0.03, 0.06, 0.08, 0.15, 0.3, 0.7, 3.0, 6.0]


def _robot(**overrides: Any) -> dict[str, Any]:
    robot: dict[str, Any] = {
        "speed_bins_mps": [0.0, 0.5],
        "position": {
            "sigma_along_m": [0.004, 0.008],
            "sigma_cross_m": [0.006, 0.010],
            "bias_along_m": [0.0, 0.0],
            "bias_cross_m": [0.0, 0.0],
        },
        "heading": {
            "sigma_rad": [0.1, 0.15],
            "flip_model": "independent",
            "flip_rate": [0.0, 0.01],
        },
        "dropout": {
            "robot_distance_bins_m": [0.0],
            "wall_distance_bins_m": [0.0],
            "quantiles": QUANTILES,
            "measured_s": [MEASURED],
            "dropped_s": [DROPPED],
        },
        "ekf_error": {
            "sigma": {
                "along": [0.01, 0.02],
                "cross": [0.01, 0.02],
                "theta": [0.02, 0.04],
                "v": [0.05, 0.1],
                "omega": [0.2, 0.4],
            },
            "tau_s": {"along": 0.1, "cross": 0.1, "theta": 0.05, "v": 0.08, "omega": 0.03},
        },
    }
    robot.update(overrides)
    return robot


def _spec_dict() -> dict[str, Any]:
    return {
        "spec_version": 1,
        "venue": "test",
        "frame_rate_hz": 30.0,
        "latency_s": 0.063,
        "frame_drop_prob": 0.0,
        "robots": {"ours": _robot(), "opponent": _robot()},
        "randomization": {
            "frame_rate_hz": [30.0, 60.0],
            "latency_s": [0.04, 0.08],
            "frame_drop_scale": [0.5, 2.0],
            "error_scale": [0.7, 1.5],
            "offset_sigma_m": 0.0,
            "flip_scale": [0.5, 5.0],
            "dropout_stretch": [0.7, 2.0],
            "ekf_sigma_scale": [0.5, 1.5],
            "ekf_tau_scale": [0.5, 2.0],
        },
    }


def _spec(d: dict[str, Any] | None = None) -> SensingModelFile:
    return SensingModelFile.from_dict(d if d is not None else _spec_dict())


def _truth(worlds: int, robots: int, speed: float = 0.0, yaw: float = 0.0) -> TruePoses:
    xy = torch.zeros(worlds, robots, 2, dtype=torch.float64)
    xy[:, :, 0] = torch.arange(robots, dtype=torch.float64)
    velocity = torch.zeros(worlds, robots, 2, dtype=torch.float64)
    velocity[..., 0] = speed * math.cos(yaw)
    velocity[..., 1] = speed * math.sin(yaw)
    return TruePoses(
        xy=xy,
        yaw=torch.full((worlds, robots), yaw, dtype=torch.float64),
        velocity=velocity,
        yaw_rate=torch.zeros(worlds, robots, dtype=torch.float64),
    )


def _far(worlds: int, robots: int) -> Context:
    far = torch.full((worlds, robots), 10.0, dtype=torch.float64)
    return Context(robot_distance_m=far, wall_distance_m=far.clone())


def test_inverse_cdf_reproduces_quantiles() -> None:
    table = DurationTable.parse(QUANTILES, [MEASURED], 1, "test")
    sampler = _Durations(table, torch.device("cpu"), torch.float64)
    g = torch.Generator().manual_seed(1)
    u = torch.rand(100_000, generator=g, dtype=torch.float64)
    draws = sampler.sample(torch.zeros(100_000, dtype=torch.long), u).numpy()
    for q, v in zip(QUANTILES[1:-1], MEASURED[1:-1]):
        assert np.quantile(draws, q) == pytest.approx(v, rel=0.02)
    assert draws.min() >= MEASURED[0] and draws.max() <= MEASURED[-1]


def test_disabled_returns_truth() -> None:
    model = MeasurementModel(_spec(), ["ours", "opponent"], worlds=3, enabled=False)
    truth = _truth(3, 2, speed=1.0, yaw=0.3)
    out = model.step(0.5, truth, _far(3, 2))
    assert bool(out.delivered.all()) and bool(out.measured.all())
    torch.testing.assert_close(out.xy, truth.xy)
    torch.testing.assert_close(out.yaw, truth.yaw)


def test_delivery_delay_and_order() -> None:
    model = MeasurementModel(_spec(), ["opponent"], worlds=4, seed=3)
    model.reset(torch.ones(4, dtype=torch.bool), 0.0)
    dt = 0.001
    stamps: list[list[float]] = [[] for _ in range(4)]
    truth = _truth(4, 1)
    for k in range(2000):
        now = k * dt
        out = model.step(now, truth, _far(4, 1))
        for w in torch.nonzero(out.delivered).flatten().tolist():
            delay = now - float(out.stamp[w])
            latency = float(model.params.latency_s[w])
            assert latency - 1e-9 <= delay < latency + dt + 1e-9
            stamps[w].append(float(out.stamp[w]))
    for w in range(4):
        assert stamps[w] == sorted(stamps[w])
        spacing = np.diff(stamps[w])
        period = float(model.params.frame_period_s[w])
        # Captures land on the first 1 ms step at or after each frame time, so the spacing
        # jitters by up to one step while the clock itself keeps the frame period.
        assert np.abs(spacing - period).max() < dt + 1e-9
        assert spacing.mean() == pytest.approx(period, abs=dt / len(spacing) + 1e-9)


def test_measured_fraction_matches_tables() -> None:
    model = MeasurementModel(_spec(), ["opponent"], worlds=256, seed=4, randomize=False)
    model.reset(torch.ones(256, dtype=torch.bool), 0.0)
    truth = _truth(256, 1)
    measured = delivered = 0
    for k in range(int(60.0 / 0.004)):
        out = model.step(k * 0.004, truth, _far(256, 1))
        delivered += int(out.delivered.sum())
        measured += int(out.measured.sum())
    table = DurationTable.parse(QUANTILES, [MEASURED], 1, "m")
    mean_m = _table_mean(table)
    mean_d = _table_mean(DurationTable.parse(QUANTILES, [DROPPED], 1, "d"))
    assert measured / delivered == pytest.approx(mean_m / (mean_m + mean_d), abs=0.03)


def _table_mean(table: DurationTable) -> float:
    sampler = _Durations(table, torch.device("cpu"), torch.float64)
    u = torch.linspace(0.0, 1.0, 200_001, dtype=torch.float64)
    return float(sampler.sample(torch.zeros_like(u, dtype=torch.long), u).mean())


def test_context_picks_cell() -> None:
    d = _spec_dict()
    drop = d["robots"]["opponent"]["dropout"]
    drop["robot_distance_bins_m"] = [0.0, 0.1]
    # Near another robot: measured runs are one frame and gaps are long.
    drop["measured_s"] = [[0.03] * 7 + [0.031], MEASURED]
    drop["dropped_s"] = [[1.0] * 7 + [1.01], DROPPED]
    model = MeasurementModel(_spec(d), ["opponent"], worlds=64, seed=5, randomize=False)
    model.reset(torch.ones(64, dtype=torch.bool), 0.0)
    truth = _truth(64, 1)
    near = Context(
        robot_distance_m=torch.zeros(64, 1, dtype=torch.float64),
        wall_distance_m=torch.full((64, 1), 10.0, dtype=torch.float64),
    )
    fractions = []
    for context in (near, _far(64, 1)):
        measured = delivered = 0
        for k in range(int(20.0 / 0.004)):
            out = model.step(k * 0.004 + len(fractions) * 100.0, truth, context)
            delivered += int(out.delivered.sum())
            measured += int(out.measured.sum())
        fractions.append(measured / delivered)
    assert fractions[0] < 0.1 < 0.5 < fractions[1]


def test_position_noise_is_body_frame() -> None:
    d = _spec_dict()
    pos = d["robots"]["opponent"]["position"]
    pos["sigma_along_m"] = [0.05, 0.05]
    pos["sigma_cross_m"] = [0.0, 0.0]
    pos["bias_cross_m"] = [0.01, 0.01]
    d["robots"]["opponent"]["heading"]["flip_rate"] = [0.0, 0.0]
    model = MeasurementModel(_spec(d), ["opponent"], worlds=2000, seed=6, randomize=False)
    truth = _truth(2000, 1, yaw=math.pi / 2)
    err = model._noisy_pose(model._samplers[0], truth, 0, torch.zeros(2000, dtype=torch.bool))[0]
    err = err - truth.xy[:, 0]
    # Heading +y: along-track noise lands on y, the cross bias on -x.
    assert float(err[:, 1].std()) == pytest.approx(0.05, rel=0.05)
    torch.testing.assert_close(err[:, 0], torch.full((2000,), -0.01, dtype=torch.float64))


def test_ekf_error_is_ar1() -> None:
    worlds = 512
    model = OurEkfErrorModel(_spec(), worlds=worlds, seed=7, randomize=False)
    speed = torch.zeros(worlds, dtype=torch.float64)
    model.reset(torch.ones(worlds, dtype=torch.bool), 0.0, speed)
    zeros2 = torch.zeros(worlds, 2, dtype=torch.float64)
    zeros = torch.zeros(worlds, dtype=torch.float64)
    period = torch.full((worlds,), 1.0 / 30.0, dtype=torch.float64)
    dt = 0.02
    errors = []
    for k in range(1, 400):
        out = model.step(
            k * dt, zeros2, zeros, zeros2, zeros, torch.full_like(zeros, k * dt), period
        )
        errors.append(out.xy[:, 0].clone())
    e = torch.stack(errors[50:]).numpy()
    assert e.std() == pytest.approx(0.01, rel=0.05)
    lag1 = np.corrcoef(e[1:].ravel(), e[:-1].ravel())[0, 1]
    assert lag1 == pytest.approx(math.exp(-dt / 0.1), abs=0.02)


def test_ekf_coasts_then_holds_stale() -> None:
    d = _spec_dict()
    for c in ("along", "cross", "theta", "v", "omega"):
        d["robots"]["ours"]["ekf_error"]["sigma"][c] = [0.0, 0.0]
    model = OurEkfErrorModel(_spec(d), worlds=1, randomize=False)
    model.reset(torch.ones(1, dtype=torch.bool), 0.0, torch.zeros(1, dtype=torch.float64))
    period = torch.full((1,), 1.0 / 30.0, dtype=torch.float64)
    yaw = torch.zeros(1, dtype=torch.float64)
    vel = torch.tensor([[1.0, 0.0]], dtype=torch.float64)
    w = torch.zeros(1, dtype=torch.float64)
    last = torch.zeros(1, dtype=torch.float64)
    xs = {}
    for k in range(0, 41):
        t = k * 0.01
        true_xy = torch.tensor([[t * 0.5, 0.0]], dtype=torch.float64)  # truth slows to 0.5 m/s
        out = model.step(t, true_xy, yaw, vel, w, last, period)
        xs[k] = (float(out.xy[0, 0]), bool(out.stale[0]))
    # The gap starts past 2.5 frames (~0.083 s); from there the pose coasts at 1 m/s until 0.2 s.
    origin = xs[9][0]
    assert xs[15][0] == pytest.approx(origin + 0.06, abs=1e-9)
    assert xs[20][0] == pytest.approx(origin + 0.11, abs=1e-9)
    assert xs[30][0] == xs[20][0] and xs[30][1] and not xs[19][1]


def test_seed_reproduces_stream() -> None:
    def run() -> torch.Tensor:
        model = MeasurementModel(_spec(), ["ours", "opponent"], worlds=8, seed=11)
        model.reset(torch.ones(8, dtype=torch.bool), 0.0)
        truth = _truth(8, 2, speed=0.7)
        out = [model.step(k * 0.004, truth, _far(8, 2)).xy for k in range(300)]
        return torch.stack(out)

    torch.testing.assert_close(run(), run(), rtol=0, atol=0)


def test_context_from_rect_arena() -> None:
    xy = torch.tensor([[[0.0, 0.0], [0.5, 0.0], [0.9, 0.9]]], dtype=torch.float64)
    ctx = Context.from_rect_arena(xy, 0.1, torch.tensor([1.0, 1.0], dtype=torch.float64))
    torch.testing.assert_close(
        ctx.robot_distance_m,
        torch.tensor([[0.3, 0.3, 0.7849]], dtype=torch.float64),
        atol=1e-4,
        rtol=0,
    )
    torch.testing.assert_close(
        ctx.wall_distance_m, torch.tensor([[1.0, 0.5, 0.1]], dtype=torch.float64)
    )


def test_bad_flip_model_lists_valid_values() -> None:
    d = copy.deepcopy(_spec_dict())
    d["robots"]["opponent"]["heading"]["flip_model"] = "burst"
    with pytest.raises(ValueError, match="independent, bursty"):
        _spec(d)


def test_spec_version_mismatch_fails() -> None:
    d = _spec_dict()
    d["spec_version"] = 2
    with pytest.raises(ValueError, match="spec_version"):
        _spec(d)


def test_bursty_flips_follow_run_tables() -> None:
    d = _spec_dict()
    d["robots"]["opponent"]["heading"] = {
        "sigma_rad": [0.0, 0.0],
        "flip_model": "bursty",
        "flip_runs": {
            "quantiles": [0.0, 1.0],
            "normal_s": [0.9, 1.1],
            "flipped_s": [0.09, 0.11],
        },
    }
    model = MeasurementModel(_spec(d), ["opponent"], worlds=64, seed=12, randomize=False)
    model.reset(torch.ones(64, dtype=torch.bool), 0.0)
    truth = _truth(64, 1)
    flipped = captured = 0
    for k in range(int(30.0 / 0.004)):
        capture = model.next_capture <= k * 0.004
        model.step(k * 0.004, truth, _far(64, 1))
        captured += int(capture.sum())
        flipped += int((capture & model.is_flipped[:, 0]).sum())
    assert flipped / captured == pytest.approx(0.1 / 1.1, abs=0.02)
