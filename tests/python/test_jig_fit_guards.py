"""Guards in jig_fit: routing, rise fit, mirror pairing, delay estimate, parameter output."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from auto_battlebot.calibration.jig.jig_fit import (
    Loaded,
    _mirror_pairs,
    _rise_tau,
    fit_delay,
    window_delay,
    write_params,
)
from auto_battlebot.calibration.jig.velocity_jig import WaveformSpec
from auto_battlebot.compat import tomllib
from auto_battlebot.control.plant import MODEL_LADDER, PlantParams


def _run(name: str, kind: str, channel: str) -> Any:
    spec = WaveformSpec(name=name, kind=kind, channel=channel)
    return SimpleNamespace(spec=spec, name=name)


def test_a_combined_grid_run_is_not_selected_for_a_linear_fit() -> None:
    loaded = Loaded(runs=[_run("a", "grid", "combined"), _run("b", "step", "linear")])
    picked = loaded.select(kinds=("step", "grid"), channels=("linear",))
    assert [r.name for r in picked] == ["b"]


def test_sessions_that_reuse_a_log_name_stay_separate() -> None:
    loaded = Loaded(
        runs=[_run("LOG-0", "step", "linear"), _run("LOG-0", "step", "linear")],
        session_ids=["day1", "day2"],
    )
    train, test = loaded.split_session("day1")
    assert test == [loaded.runs[0]]
    assert train == [loaded.runs[1]]


def test_rise_tau_ignores_a_late_sample_that_dips_back_into_the_band() -> None:
    dt, tau, v_ss = 0.005, 0.055, 1.0
    t = np.arange(120) * dt
    mv = v_ss * (1.0 - np.exp(-t / tau))
    mv[100] = 0.5
    assert _rise_tau(mv, v_ss, dt, 0.01) == pytest.approx(tau, rel=0.1)


def test_mirror_pairs_match_on_the_command_issued_not_the_effective_one() -> None:
    cells = [
        (0.5, 0.0650, 1.0, 2.0, 0.08),
        (0.5, -0.0573, 1.0, -1.7, -0.08),
    ]
    pairs = _mirror_pairs(cells)
    assert len(pairs) == 1
    lin, mag, w_pos, w_neg = pairs[0]
    assert lin == pytest.approx(0.5)
    assert mag == pytest.approx(0.5 * (0.0650 + 0.0573))
    assert (w_pos, w_neg) == (2.0, -1.7)


def test_a_cell_without_a_mirror_is_dropped() -> None:
    assert _mirror_pairs([(0.5, 0.065, 1.0, 2.0, 0.08)]) == []


DT = 0.005
TRUE_DELAY_S = 0.0376
TAU_S = 0.02
EXPECTED_LAG_MS = 44.3


def _delay_run() -> Any:
    n = 4000
    t = np.arange(n) * DT
    cmd = np.zeros(n)
    for k, e in enumerate(range(200, n - 300, 400)):
        cmd[e : e + 200] = 0.6 if k % 2 == 0 else -0.6
    v = np.zeros(n)
    for e in np.flatnonzero(np.diff(cmd) != 0.0) + 1:
        td = e * DT + TRUE_DELAY_S
        step = cmd[e] - cmd[e - 1]
        v += np.where(t > td, step * (1.0 - np.exp(-(t - td) / TAU_S)), 0.0)
    window = 4
    smooth = np.convolve(
        np.pad(v, (window // 2, window - 1 - window // 2), mode="edge"),
        np.ones(window) / window,
        "valid",
    )
    return SimpleNamespace(
        dt=DT,
        cmd_lin=cmd,
        cmd_ang=np.zeros(n),
        v=smooth,
        v_onset=v,
        w=np.zeros(n),
        truth=SimpleNamespace(clock=SimpleNamespace(measured=True)),
    )


def test_delay_estimate_stays_on_the_light_smoothing_and_backward_difference() -> None:
    stack = fit_delay([_delay_run()])
    assert stack.lag_ms == pytest.approx(EXPECTED_LAG_MS, abs=1.5)
    assert stack.lag_ms > TRUE_DELAY_S * 1000.0


def test_window_delay_is_zero_for_the_rung_without_a_delay_term() -> None:
    params = PlantParams(delay_s=0.05)
    assert window_delay(params, MODEL_LADDER[0]) == 0.0
    assert window_delay(params, MODEL_LADDER[1]) == pytest.approx(0.05)


def test_written_params_zero_the_terms_the_structure_disables(tmp_path: Path) -> None:
    params = PlantParams(c_drift=-0.854, c_drift_bias=-0.333)
    stage = SimpleNamespace(estimates={})
    out = tmp_path / "plant.toml"
    write_params(out, params, stage, "note", MODEL_LADDER[3])  # type: ignore[arg-type]
    with open(out, "rb") as handle:
        plant = tomllib.load(handle)["plant"]
    assert plant["c_drift"] == 0.0
    assert plant["c_drift_bias"] == 0.0
