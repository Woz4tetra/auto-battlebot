"""fit_plant_calib rise time constant and actuation lag estimators."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from playground.calibration import fit_plant_calib as fit

DT = 0.01


def test_rise_tau_ignores_late_noise_dip_into_band() -> None:
    t = np.arange(0, 3.0, DT)
    vss = 1.0
    tau = 0.055
    v = vss * (1.0 - np.exp(-t / tau))
    v[250] = 0.5
    gap = np.zeros(len(t), dtype=bool)
    assert fit._rise_tau(v, vss, DT, 0.01, gap) == pytest.approx(tau, rel=0.25)


def test_actuation_lag_is_not_earlier_than_the_true_delay() -> None:
    delay = 8
    n = 6000
    cmd = np.zeros(n)
    for k in range(10):
        start = 200 + k * 500
        cmd[start : start + 250] = 0.6 if k % 2 == 0 else -0.6
    vel = np.zeros(n)
    for k in range(delay + 1, n):
        vel[k] = cmd[k - delay]
    session = SimpleNamespace(
        dt_s=DT,
        cmd_lin_norm=cmd,
        v_raw_mps=vel,
        cmd_ang=np.zeros(n),
        w_raw_radps=np.zeros(n),
        in_truth_gap=np.zeros(n, dtype=bool),
    )
    lag_ms, count, _ = fit.fit_actuation_lag(session)
    assert count >= 6
    assert lag_ms == pytest.approx(delay * DT * 1000.0, abs=0.3 * DT * 1000.0)


def _step_session(polarity: float, lag_samples: int = 5) -> SimpleNamespace:
    n = 1200
    cmd = np.zeros(n)
    for start in range(40, n - 80, 80):
        cmd[start : start + 40] = 0.8 if (start // 80) % 2 == 0 else -0.8
    delayed = np.concatenate([np.zeros(lag_samples), cmd[:-lag_samples]])
    vel = np.zeros(n)
    for i in range(1, n):
        vel[i] = vel[i - 1] + (delayed[i] * 2.0 - vel[i - 1]) * 0.2
    return SimpleNamespace(
        dt_s=DT,
        cmd_lin_norm=cmd,
        v_raw_mps=polarity * vel,
        cmd_ang=cmd.copy(),
        w_raw_radps=polarity * vel,
        in_truth_gap=np.zeros(n, dtype=bool),
    )


def test_mirrored_measured_velocity_gives_the_same_actuation_lag() -> None:
    upright = fit.fit_actuation_lag(_step_session(1.0))
    mirrored = fit.fit_actuation_lag(_step_session(-1.0))
    assert upright[1] > 6
    assert mirrored[1] == upright[1]
    assert mirrored[0] == pytest.approx(upright[0])
    assert mirrored[2] == pytest.approx(upright[2])
