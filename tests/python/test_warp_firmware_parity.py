"""The Warp firmware kernel tracks the scalar FirmwareMixer to under half a percent RMS."""

from __future__ import annotations

import numpy as np

from playground.rl import check_warp_firmware as check


def _synthetic_span(seed: int, samples: int) -> tuple[np.ndarray, ...]:
    rng = np.random.default_rng(seed)
    t = np.arange(samples) * check.DT
    forward = 60.0 * np.sin(0.8 * t) + rng.normal(0.0, 2.0, samples)
    turn = 40.0 * np.sin(1.7 * t + seed) + rng.normal(0.0, 2.0, samples)
    heading = 30.0 * np.sin(0.5 * t)
    yaw_rate = rng.normal(0.0, 20.0, samples)
    return forward - turn, forward + turn, heading, yaw_rate


def test_warp_kernel_rms_under_half_percent_of_scalar_mixer() -> None:
    check.wp.init()
    rows = [_synthetic_span(seed, samples) for seed, samples in ((0, 600), (1, 900))]
    diff = check.kernel_vs_scalar_difference(rows)
    rms = np.sqrt((diff**2).mean(axis=0))
    assert diff.shape == (1500, 2)
    assert rms.max() < check.GATE_RMS_PERCENT
