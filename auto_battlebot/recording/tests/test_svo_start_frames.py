from pathlib import Path

from auto_battlebot.recording.svo_start_frames import (
    DEFAULT_START_FRAMES,
    load_start_frames,
    start_frame_for,
)


def test_checked_in_table_loads() -> None:
    table = load_start_frames()
    assert DEFAULT_START_FRAMES.exists()
    assert table["2026-05-02T14-12-27"] == 3000


def test_unlisted_recording_starts_at_zero(tmp_path: Path) -> None:
    table_path = tmp_path / "frames.toml"
    table_path.write_text('[start_frames]\n"a" = 12\n')
    table = load_start_frames(table_path)
    assert start_frame_for("data/svo/x/a.svo2", table) == 12
    assert start_frame_for("data/svo/x/b.svo2", table) == 0
