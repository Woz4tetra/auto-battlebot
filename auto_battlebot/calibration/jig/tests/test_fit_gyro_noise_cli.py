"""The fit_gyro_noise CLI recovers the bias of a synthetic stationary jig log."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import numpy as np

from auto_battlebot.paths import REPO_ROOT

SAMPLES = 120_000
GYRO_DPS_PER_LSB = 0.070
BIAS_LSB = (0, -4, 10)


def test_cli_prints_the_injected_bias_per_axis(tmp_path: Path) -> None:
    rng = np.random.default_rng(3)
    gyro = rng.normal(0.0, 3.0, size=(SAMPLES, 3)).round().astype(int) + np.array(BIAS_LSB)
    t_us = np.arange(SAMPLES) * 1000
    rows = [
        f"{t},0,{g[0]},{g[1]},{g[2]},0,0,4096"
        for t, g in zip(t_us.tolist(), gyro.tolist(), strict=True)
    ]
    log = tmp_path / "still.csv"
    log.write_text("\n".join(rows) + "\n")
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "auto_battlebot.calibration.jig.fit_gyro_noise",
            str(log),
            "--min-minutes",
            "1",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    assert "120,000 samples, 2.0 min" in result.stdout
    match = re.search(r"bias_dps = \[([^\]]+)\]", result.stdout)
    assert match, result.stdout
    bias = [float(v) for v in match.group(1).split(",")]
    expected = [b * GYRO_DPS_PER_LSB for b in BIAS_LSB]
    assert np.allclose(bias, expected, atol=0.002)
    assert "arw_deg_sqrt_hr = " in result.stdout
