from pathlib import Path

import pytest

from auto_battlebot.recording.recording_catalog import (
    DEFAULT_CATALOG,
    load_catalog,
    load_start_frames,
    start_frame_for,
)


def test_checked_in_catalog_loads() -> None:
    assert DEFAULT_CATALOG.exists()
    catalog = load_catalog()
    assert catalog["2026-05-02T14-12-27"].start_frame == 3000
    assert catalog["2026-05-01T11-22-34"].note == "field finding"
    assert load_start_frames()["2026-05-02T11-45-08"] == 13000


def test_checked_in_keys_match_path_stems() -> None:
    for name, rec in load_catalog().items():
        assert rec.path.stem == name
        assert not rec.path.is_absolute()
        assert rec.path.suffix in {".svo2", ".mcap"}


def test_missing_start_frame_defaults_to_zero(tmp_path: Path) -> None:
    catalog_path = tmp_path / "recordings.toml"
    catalog_path.write_text(
        '[recordings.a]\npath = "data/x/a.svo2"\nstart_frame = 12\n\n'
        '[recordings.b]\npath = "data/x/b.svo2"\n'
    )
    catalog = load_catalog(catalog_path)
    assert catalog["b"].start_frame == 0
    assert catalog["b"].note == ""
    table = load_start_frames(catalog_path)
    assert start_frame_for("data/svo/x/a.svo2", table) == 12
    assert start_frame_for("data/svo/x/b.svo2", table) == 0
    assert start_frame_for("data/svo/x/c.svo2", table) == 0


def test_non_string_path_is_rejected(tmp_path: Path) -> None:
    catalog_path = tmp_path / "recordings.toml"
    catalog_path.write_text("[recordings.a]\npath = 3\n")
    with pytest.raises(ValueError, match="string `path`"):
        load_catalog(catalog_path)
