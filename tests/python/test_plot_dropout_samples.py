"""The summary plot bins samples by the same phase periodic_gaps drops measurements at."""

from __future__ import annotations

import os
import subprocess
import sys

import numpy as np

from playground.opponent_prediction import evaluate_opponent_prediction as ev
from playground.opponent_prediction import plot_dropout_samples as plots


def test_gap_phase_matches_periodic_gaps() -> None:
    stamps_ns = np.int64(5_000_000_000) + np.arange(0, 40_000_000_000, 33_333_333, dtype=np.int64)
    gap_s = 1.0
    keep = ev.periodic_gaps(stamps_ns, gap_s, plots.SUMMARY_PERIOD_S)
    rel = plots.seconds_after_gap_phase(stamps_ns, int(stamps_ns[0]))
    into = plots.seconds_into_gap_period(rel)
    in_gap = (into >= 0.0) & (into < gap_s)
    after_phase = rel > 0.0
    assert not keep[in_gap & after_phase].any()
    assert keep[~in_gap | ~after_phase].all()


def test_importing_the_plot_module_selects_the_agg_backend() -> None:
    code = (
        "import matplotlib.pyplot as plt\n"
        "from playground.opponent_prediction import plot_dropout_samples\n"
        "print(plt.get_backend().lower())\n"
    )
    out = subprocess.run(
        [sys.executable, "-c", code],
        env={**os.environ, "MPLBACKEND": "svg"},
        capture_output=True,
        text=True,
        check=True,
    )
    assert out.stdout.strip() == "agg"
