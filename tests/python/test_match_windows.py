from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from auto_battlebot.calibration.jig.drive_protocol import MixConfig
from auto_battlebot.calibration.match import match_windows
from auto_battlebot.calibration.match.match_windows import (
    ReplayTrack,
    load_commands,
    load_replay_track,
)

READBACK_SCALE = 1024.0


def _status(ch0: float, ch1: float) -> dict:
    return {
        "hardware_id": "opentx_transmitter",
        "name": "channels",
        "values": {"values/0": str(ch0), "values/1": str(ch1), "values/15": "1024"},
    }


def test_load_commands_matches_mixconfig(monkeypatch) -> None:
    channels = [(300.0, -200.0), (-1024.0, 512.0), (800.0, 800.0)]
    monkeypatch.setattr(
        match_windows, "iter_messages", lambda *a, **k: [("t", i, b"") for i in range(3)]
    )
    statuses = iter(channels)
    monkeypatch.setattr(
        match_windows, "decode_diagnostic_array", lambda data: [_status(*next(statuses))]
    )
    mix = MixConfig(mode="tank", reverse_angular=True, read_invert_a=False, read_invert_b=True)
    commands = load_commands("x.mcap")
    expected = [mix.from_channels(a / READBACK_SCALE, b / READBACK_SCALE) for a, b in channels]
    np.testing.assert_allclose(commands.lin, [lin for lin, _ in expected])
    np.testing.assert_allclose(commands.ang, [ang for _, ang in expected])
    assert commands.auto_switch_latched.all()


def _track(svo_index: list[int]) -> ReplayTrack:
    zeros = np.zeros(len(svo_index))
    return ReplayTrack(
        replay_path=Path("r.mcap"),
        svo_index=np.array(svo_index),
        stamp_ns=np.arange(len(svo_index), dtype=np.int64),
        x=zeros,
        y=zeros,
        theta=zeros,
        our_live_fresh=np.ones(len(svo_index), dtype=bool),
        opp_x=zeros,
        opp_y=zeros,
        field_size=None,
        rebase_offset_std_ns=0.0,
    )


@pytest.mark.parametrize("svo_index", [[0, 1, 1, 2], [3, 2, 1]])
def test_replay_track_requires_strictly_increasing_svo_index(svo_index: list[int]) -> None:
    with pytest.raises(ValueError, match="strictly increasing"):
        _track(svo_index)


def test_replay_track_accepts_gaps() -> None:
    assert len(_track([0, 1, 4, 9]).svo_index) == 4


def _patch_replay(monkeypatch, rebase_offsets_ns: list[int]) -> None:
    metas = [(i * 1000, 10_000 + i, i) for i in range(len(rebase_offsets_ns))]
    ours = {i: (0.0, 0.0, 0.0, 10_000 + i + off) for i, off in enumerate(rebase_offsets_ns)}
    monkeypatch.setattr(match_windows, "_load_frame_meta", lambda path: metas)
    monkeypatch.setattr(match_windows, "_load_live_by_tick", lambda path, log: {})
    monkeypatch.setattr(match_windows, "_load_marker_poses", lambda path, log: (ours, {}, None))


def test_load_replay_track_rejects_drifting_offset(monkeypatch) -> None:
    _patch_replay(monkeypatch, [i * 5_000_000 for i in range(10)])
    with pytest.raises(ValueError, match="drifted"):
        load_replay_track("r.mcap")


def test_load_replay_track_accepts_constant_offset(monkeypatch) -> None:
    _patch_replay(monkeypatch, [7_000_000] * 10)
    assert load_replay_track("r.mcap").rebase_offset_std_ns == 0.0
