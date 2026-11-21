"""The per-stage boxplot lists the slowest stage at the top."""

from __future__ import annotations

import importlib.util
import sys

import matplotlib
import matplotlib.pyplot as plt

from auto_battlebot.paths import REPO_ROOT

matplotlib.use("Agg")

SCRIPT = REPO_ROOT / "scripts" / "mcap_latency_report.py"
SPEC = importlib.util.spec_from_file_location("mcap_latency_report", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
report = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = report
SPEC.loader.exec_module(report)


def test_slowest_stage_is_drawn_at_the_top():
    stage_ms = {"slow": [10.0, 11.0], "mid": [5.0, 6.0], "fast": [1.0, 2.0]}
    samples = report.LatencySamples(stage_t_s={}, stage_ms=stage_ms, rate_hz=[], duration_s=1.0)
    stats = report.aggregate(samples, None)
    assert [s.stage for s in stats] == ["slow", "mid", "fast"]
    fig, ax = plt.subplots()
    report.plot_stage_distributions(ax, samples, stats)
    fig.canvas.draw()
    labels = {t.get_position()[1]: t.get_text() for t in ax.get_yticklabels()}
    assert labels[max(labels)] == "slow"
    assert labels[min(labels)] == "fast"
    plt.close(fig)
