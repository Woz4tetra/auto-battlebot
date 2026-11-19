"""Cable routing guards in hardware/cage_pack/cage_pack_assembly.py."""

import importlib
from pathlib import Path
from types import ModuleType

import pytest

CAGE_PACK = Path(__file__).resolve().parents[2] / "hardware" / "cage_pack"


@pytest.fixture()
def assembly(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    pytest.importorskip("build123d")
    pytest.importorskip("recipe")
    monkeypatch.syspath_prepend(str(CAGE_PACK))
    return importlib.import_module("cage_pack_assembly")


def test_run_rejects_corner_below_bend_radius(assembly: ModuleType) -> None:
    min_bend_r = assembly.CABLES["usb"].min_bend_r
    short_leg = min_bend_r / 4
    points = [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (100.0, short_leg, 0.0), (100.0, 100.0, 0.0)]
    with pytest.raises(ValueError, match="corners need"):
        assembly._run("usb", points)


def test_run_builds_a_corner_wider_than_the_bend_radius(assembly: ModuleType) -> None:
    points = [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (100.0, 100.0, 0.0)]
    _, length = assembly._run("usb", points)
    assert length < 200.0
