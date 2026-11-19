"""fit_process_noise growth law stays non-negative."""

from __future__ import annotations

import numpy as np

from playground.calibration import fit_process_noise as noise


def test_shrinking_variance_never_predicts_negative_variance() -> None:
    horizons = np.array([0.05, 0.1, 0.2, 0.4, 0.8])
    rng = np.random.default_rng(0)
    scale = np.array([0.3, 0.25, 0.1, 0.05, 0.3])
    residuals = rng.normal(size=(500, len(horizons))) * scale
    fit = noise.fit_growth("x", horizons, residuals)
    coeffs = [fit.const_var, fit.a_per_s, fit.b_per_s2, fit.c_per_s3]
    assert min(coeffs) >= 0.0
    assert np.all(fit.variance(np.linspace(0.0, 2.0, 50)) >= 0.0)
