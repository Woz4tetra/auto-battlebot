"""download_cage_video: one fight schema and compression-agnostic corpus listing."""

from __future__ import annotations

import tarfile
from pathlib import Path

import pytest

from playground.bgsub_cage import download_cage_video as dl


def _tar(path: Path, mode: str, members: list[str]) -> Path:
    source = path.parent / "payload.txt"
    source.write_text("x")
    with tarfile.open(path, mode) as archive:
        for member in members:
            archive.add(source, arcname=member)
    return path


@pytest.mark.parametrize(("mode", "name"), [("w", "plain.tar"), ("w:gz", "zipped.tar.gz")])
def test_corpus_tournaments_reads_plain_and_gzipped_tars(
    tmp_path: Path, mode: str, name: str
) -> None:
    members = ["tournamentID-abc_1/a.png", "tournamentID-abc_1/b.png", "tournamentID-z9/c.png"]
    tarball = _tar(tmp_path / name, mode, members)
    assert dl.corpus_tournaments(tarball) == ["abc_1", "z9"]


def test_corpus_tournaments_missing_tarball_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        dl.corpus_tournaments(tmp_path / "none.tar")


def test_bot_fights_keyed_by_gameid_reach_build_clip_as_id(monkeypatch) -> None:
    payload = {"fights": [{"gameID": "g7", "tournamentID": "t1", "matchLength": 180}]}
    monkeypatch.setattr(dl, "api_get", lambda path, key: payload)
    (fight,) = dl.bot_fights("bot", None, None, 60)
    assert fight["id"] == "g7"

    requested: list[str] = []

    def fake_api_get(path: str, key):
        requested.append(path)
        return {"fight": {"recordings": []}}

    monkeypatch.setattr(dl, "api_get", fake_api_get)
    assert dl.build_clip(fight, None) is None
    assert requested == ["/tournaments/t1/fights/g7"]
