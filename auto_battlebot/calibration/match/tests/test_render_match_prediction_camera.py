"""Camera ticks map to the match grid slot they were rounded into when the grid was built."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from auto_battlebot.calibration.match import (
    render_match_prediction_camera as _render_match_prediction_camera,
)
from auto_battlebot.calibration.match.match_windows import ReplayTrack, _fill_grid

grid_slots = _render_match_prediction_camera.grid_slots


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
        our_live_fresh=np.ones(len(stamps), bool),
        opp_x=zeros,
        opp_y=zeros,
        field_size=None,
        rebase_offset_std_ns=0.0,
    )
    _t, _grid, dt, t0_ns = _fill_grid(track)
    slots = grid_slots(stamps, t0_ns, dt, int(indices[-1]) + 1)
    np.testing.assert_array_equal(slots, indices)


def _jpeg(width: int, height: int) -> bytes:
    import cv2

    ok, encoded = cv2.imencode(".jpg", np.full((height, width, 3), 90, np.uint8))
    assert ok
    return encoded.tobytes()


def test_truncated_jpeg_is_skipped() -> None:
    decode_full_frame = _render_match_prediction_camera.decode_full_frame

    whole = _jpeg(64, 48)
    assert decode_full_frame(whole, 64, 48) is not None
    assert decode_full_frame(whole[: len(whole) // 3], 64, 48) is None
    assert decode_full_frame(b"", 64, 48) is None
    assert decode_full_frame(whole, 32, 48) is None


def test_frame_meta_entry_carries_the_separate_svo_path() -> None:
    frame_meta_entry = _render_match_prediction_camera.frame_meta_entry

    legacy = {"image_stamp_ns": 5, "svo_frame_index": 7, "svo_path": "data/svo/a.svo2"}
    assert frame_meta_entry(1, legacy) == (1, 5, 7, "data/svo/a.svo2")
    inline = {"image_stamp_ns": 5, "video_frame_index": 9}
    assert frame_meta_entry(1, inline) == (1, 5, 9, "")
