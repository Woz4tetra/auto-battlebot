"""auto_battlebot.perception.sysid_sessions: truth comes only from the tag-only bundle."""

from __future__ import annotations

from pathlib import Path

import pandas as pd
import pytest

from auto_battlebot.perception import sysid_sessions


def _write_bundle(root: Path, sources: list[str]) -> None:
    bundle = root / "session_tag"
    bundle.mkdir()
    stamps = [i * 5_000_000 for i in range(len(sources))]
    pd.DataFrame({"stamp_ns": stamps, "upside_down": [0] * len(stamps)}).to_csv(
        bundle / "smoothed.csv", index=False
    )
    pd.DataFrame({"stamp_ns": stamps, "source": sources, "rejected": [0] * len(stamps)}).to_csv(
        bundle / "measurements.csv", index=False
    )


def test_truth_bundle_rejects_yolo_filled(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sysid_sessions, "BUNDLES", tmp_path)
    _write_bundle(tmp_path, ["tag", "tag", "yolo"])
    with pytest.raises(ValueError, match="YOLO"):
        sysid_sessions._truth("session")


def test_truth_bundle_accepts_tag_only(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sysid_sessions, "BUNDLES", tmp_path)
    _write_bundle(tmp_path, ["tag", "tag", "tag"])
    truth, valid = sysid_sessions._truth("session")
    assert len(truth) == 3
    assert valid.all()
