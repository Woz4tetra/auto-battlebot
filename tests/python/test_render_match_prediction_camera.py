"""Camera ticks map to the match grid slot they were rounded into when the grid was built."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from auto_battlebot.calibration.match.match_windows import ReplayTrack, _fill_grid
from playground.calibration.render_match_prediction_camera import grid_slots


def test_slots_follow_cumulative_rounding() -> None:
    dt_ns = 16_666_667
    rng = np.random.default_rng(0)
    indices = np.array([0, 1, 2, 4, 5, 6, 8, 9])
    jitter = rng.integers(-3_000_000, 3_000_000, len(indices))
    stamps = 1_000_000_000 + indices * dt_ns + jitter
    zeros = np.zeros(len(stamps))
    track = ReplayTrack(
        replay_path=Path("replay.mcap"),
        svo_index=indices,
        stamp_ns=stamps,
        x=zeros,
        y=zeros,
        theta=zeros,
        live=np.ones(len(stamps), bool),
        opp_x=zeros,
        opp_y=zeros,
        field_size=None,
        rebase_offset_std_ns=0.0,
    )
    _t, _grid, dt, t0_ns = _fill_grid(track)
    slots = grid_slots(stamps, t0_ns, dt, int(indices[-1]) + 1)
    np.testing.assert_array_equal(slots, indices)
