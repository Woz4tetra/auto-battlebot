"""print_mass.slice_grams keeps the PrusaSlicer flatpak files under $HOME."""

import subprocess
from pathlib import Path

import pytest

CAGE_PACK = Path(__file__).resolve().parents[2] / "hardware" / "cage_pack"


def test_slice_grams_writes_gcode_under_home(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.syspath_prepend(str(CAGE_PACK))
    import print_mass

    seen: list[list[str]] = []

    def fake_run(cmd: list[str], **_: object) -> None:
        seen.append(cmd)
        Path(cmd[cmd.index("-o") + 1]).write_text("; filament used [g] = 5.48\n")

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert print_mass.slice_grams(Path("part.stl")) == 5.48
    datadir = Path(seen[0][seen[0].index("--datadir") + 1])
    assert Path.home() in datadir.parents
