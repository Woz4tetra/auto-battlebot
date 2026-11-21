from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from auto_battlebot.calibration.match import fit_match_plant


def _match_windows(starts: list[int], runs: list[int]) -> SimpleNamespace:
    n = len(starts)
    steps = 6
    state0 = SimpleNamespace(
        v=np.zeros(n), w=np.zeros(n), x=np.zeros(n), y=np.zeros(n), theta=np.zeros(n)
    )
    truth = {
        "x": np.arange(n)[:, None] * np.ones((n, steps)),
        "y": np.zeros((n, steps)),
        "theta": np.zeros((n, steps)),
    }
    windows = SimpleNamespace(
        count=lambda: n,
        u_lin=np.full((n, 3), 0.5),
        u_ang=np.zeros((n, 3)),
        state0=state0,
        start_indices=np.array(starts),
        truth=truth,
        horizon_steps=np.array([2, 4, 6]),
        horizons_s=np.array([0.1, 0.2, 0.3]),
    )
    return SimpleNamespace(windows=windows, run_index=np.array(runs))


def test_noise_floor_pairs_windows_whose_starts_are_further_apart_than_the_horizon_span() -> None:
    floor = fit_match_plant.noise_floor(_match_windows([0, 10], [0, 0]))

    assert floor is not None
    assert floor["pairs"][0] == 1
    assert np.all(floor["pos_rmse_mm"] > 0.0)


def test_noise_floor_skips_overlapping_windows_from_the_same_run() -> None:
    assert fit_match_plant.noise_floor(_match_windows([0, 3], [0, 0])) is None


def test_noise_floor_pairs_overlapping_windows_from_different_runs() -> None:
    floor = fit_match_plant.noise_floor(_match_windows([0, 3], [0, 1]))

    assert floor is not None
    assert floor["pairs"][0] == 1
