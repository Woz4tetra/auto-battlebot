from __future__ import annotations

import numpy as np
import pytest

from auto_battlebot.control.plant import (
    FULL_MODEL,
    Plant,
    PlantParams,
    WindowSet,
    concat_windows,
    delayed_commands,
    make_windows,
    predict_windows,
    simulate,
    toml_float,
)

DT = 0.01
PARAMS = PlantParams(delay_s=0.0)


def _run(n: int = 400) -> dict[str, np.ndarray]:
    t = np.arange(n) * DT
    cmd_lin = 0.6 * np.sign(np.sin(t * 2.0))
    cmd_ang = 0.4 * np.cos(t * 1.3)
    final, traj = simulate(cmd_lin, cmd_ang, DT, PARAMS)
    del final
    out = {k: traj[k] for k in ("x", "y", "theta", "v", "w")}
    return {"t": t, "cmd_lin": cmd_lin, "cmd_ang": cmd_ang, **out}


def _windows(run: dict[str, np.ndarray], **kwargs: object) -> WindowSet:
    windows = make_windows(
        run["t"],
        run["v"],
        run["w"],
        run["theta"],
        run["x"],
        run["y"],
        run["cmd_lin"],
        run["cmd_ang"],
        dt=DT,
        delay_s=0.0,
        **kwargs,  # type: ignore[arg-type]
    )
    assert windows is not None
    return windows


@pytest.mark.parametrize("value", [0.0, 5.0, -3.0, 1e6, 0.25])
def test_toml_float_always_has_decimal_point(value: float) -> None:
    text = toml_float(value)
    assert "." in text or "e" in text
    assert isinstance(float(text), float)


def test_stepper_matches_simulate_at_integer_delay() -> None:
    dt = 2.0**-7
    delay_steps = 3
    params = PlantParams(delay_s=delay_steps * dt)
    rng = np.random.default_rng(1)
    n = 80
    cmd_lin = np.repeat(rng.uniform(-1, 1, n // 8), 8)
    cmd_ang = np.repeat(rng.uniform(-1, 1, n // 8), 8)
    t = np.arange(n) * dt
    shifted_lin, shifted_ang = delayed_commands(t, cmd_lin, cmd_ang, params.delay_s, dt)
    _, traj = simulate(shifted_lin, shifted_ang, dt, params)
    stepper = Plant(params)
    for k in range(n):
        stepper.push_command(k * dt, cmd_lin[k], cmd_ang[k])
        state = stepper.step(dt)
        assert float(state.x) == pytest.approx(traj["x"][k], abs=1e-9)
        assert float(state.theta) == pytest.approx(traj["theta"][k], abs=1e-9)


def test_delayed_commands_zero_before_delay() -> None:
    n = 20
    t = np.arange(n) * DT
    ones = np.ones(n)
    lin, ang = delayed_commands(t, ones, -ones, 5 * DT, DT)
    np.testing.assert_array_equal(lin[:5], 0.0)
    np.testing.assert_array_equal(ang[:5], 0.0)
    np.testing.assert_allclose(lin[5:], 1.0)


def test_cut_windows_drops_window_with_any_invalid_sample() -> None:
    run = _run()
    every = _windows(run, horizons=(0.1,), stride_s=0.05)
    valid = np.ones(len(run["t"]), dtype=bool)
    bad = int(every.start_indices[10]) + 3
    valid[bad] = False
    kept = _windows(run, horizons=(0.1,), stride_s=0.05, valid=valid)
    span = int(every.horizon_steps.max())
    for start in kept.start_indices:
        assert valid[start : start + span + 1].all()
    assert len(kept.start_indices) < len(every.start_indices)
    assert every.start_indices[10] not in kept.start_indices


def test_cut_windows_reports_snapped_horizons() -> None:
    windows = _windows(_run(), horizons=(0.033,))
    assert windows.horizon_steps.tolist() == [3]
    assert windows.horizons_s[0] == pytest.approx(0.03)


def test_predict_windows_horizon_index() -> None:
    errors = predict_windows(_windows(_run(), horizons=(0.03, 0.1, 0.2)), PARAMS)
    np.testing.assert_allclose(errors.along_m, 0.0, atol=1e-9)
    np.testing.assert_allclose(errors.cross_m, 0.0, atol=1e-9)
    np.testing.assert_allclose(errors.heading_rad, 0.0, atol=1e-9)


def test_concat_windows_drops_partial_origin() -> None:
    run = _run()
    attributed = _windows(run, horizons=(0.1,), origin=0)
    anonymous = _windows(run, horizons=(0.1,))
    both = concat_windows([attributed, attributed])
    assert both.origin_run_index is not None
    assert len(both.origin_run_index) == 2 * len(attributed.start_indices)
    assert concat_windows([attributed, anonymous]).origin_run_index is None


def test_window_set_rejects_misshapen_command_array() -> None:
    windows = _windows(_run(), horizons=(0.1,))
    with pytest.raises(ValueError, match="u_lin"):
        WindowSet(
            start_indices=windows.start_indices,
            horizons_s=windows.horizons_s,
            horizon_steps=windows.horizon_steps,
            dt=windows.dt,
            u_lin=windows.u_lin.T,
            u_ang=windows.u_ang,
            truth=windows.truth,
            state0=windows.state0,
        )


def test_subsample_keeps_shapes() -> None:
    windows = _windows(_run(), horizons=(0.1,))
    thin = windows.subsample(5)
    assert thin.count() == 5
    assert thin.u_lin.shape == (5, int(windows.horizon_steps.max()))


def test_gradient_free_names_excludes_delay() -> None:
    assert "delay_s" not in FULL_MODEL.gradient_free_names()
