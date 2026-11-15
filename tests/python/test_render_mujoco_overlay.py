"""render_mujoco_overlay draws measured and MuJoCo paths in the fit plots' colours."""

from __future__ import annotations

from auto_battlebot.mujoco_sim import plots
from playground.calibration import render_mujoco_overlay


def _bgr(hex_colour: str) -> tuple[int, int, int]:
    r, g, b = (int(hex_colour[i : i + 2], 16) for i in (1, 3, 5))
    return b, g, r


def test_colors_match_plots() -> None:
    assert render_mujoco_overlay.MEAS == _bgr(plots.MEAS)
    assert render_mujoco_overlay.MUJOCO == _bgr(plots.MUJOCO)
