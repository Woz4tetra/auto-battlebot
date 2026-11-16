"""jig_report spread scaling and the runs-needed extrapolation."""

from __future__ import annotations

import math

from playground.calibration import jig_report as report


def _status(value: float, spread: float, runs_now: int = 2) -> report.ParamStatus:
    return report.ParamStatus(
        name="coupling",
        value=value,
        spread=spread,
        n=2,
        note="",
        sources=(("step", "linear"),),
        runs_now=runs_now,
        commanded_s_now=10.0,
    )


def test_near_zero_value_has_finite_relative_spread() -> None:
    assert math.isfinite(_status(0.02, 0.04).rel)
    assert math.isfinite(_status(0.0, 0.04).rel)


def test_flat_slope_falls_back_to_root_n() -> None:
    section = report.section_next([_status(1.0, 0.5)], {"coupling": -1e-9})
    html = section.render()
    assert 'class="chip">root-N estimate' in html


def test_steep_slope_uses_the_measurement() -> None:
    section = report.section_next([_status(1.0, 0.5)], {"coupling": -0.5})
    assert 'class="chip">measured slope' in section.render()
