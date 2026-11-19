"""Layout clearances in hardware/cage_pack against the numbers the design chose them for."""

import importlib
from pathlib import Path
from types import ModuleType

import pytest

CAGE_PACK = Path(__file__).resolve().parents[2] / "hardware" / "cage_pack"

PCBA_ZED_MIN_GAP_MM = 2.0
M25_SCREW_LEN_MM = 6.0
M25_DIAMETER_MM = 2.5
BRASS_ENGAGEMENT_DIAMETERS = 1.25
RJ45_JACK_HALF_WIDTH_MM = 19.0
UPS_BOSS_MIN_CLEARANCE_MM = 3.0


@pytest.fixture()
def cl(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    monkeypatch.syspath_prepend(str(CAGE_PACK))
    return importlib.import_module("cage_layout")


def test_zed_bosses_clear_tripod_screw_head_and_washer(cl: ModuleType) -> None:
    head_h, washer_t = cl.CUP_HEAD[1], cl.CUP_WASHER[1]
    assert cl.BOSS_H > head_h + washer_t


def test_pcba_clears_zed_body(cl: ModuleType) -> None:
    overlaps_x = cl.PCBA_X0 < cl.ZED_BODY_X1 and cl.PCBA_X1 > cl.ZED_BODY_X0
    assert overlaps_x
    assert cl.ZED_Y0 - cl.PCBA_Y1 >= PCBA_ZED_MIN_GAP_MM


def test_display_screw_engages_brass_standoff_thread(cl: ModuleType) -> None:
    engagement = M25_SCREW_LEN_MM - cl.DISP_BRACKET_T
    assert engagement >= BRASS_ENGAGEMENT_DIAMETERS * M25_DIAMETER_MM


def test_lid_bosses_clear_rj45_jack_and_openups_holes(cl: ModuleType) -> None:
    pytest.importorskip("build123d")
    pytest.importorskip("recipe")
    enclosure = importlib.import_module("enclosure")
    half_boss = cl.LID_BOSS_SIZE / 2
    ups_ys = [y for _, y in cl.UPS_HOLES]
    ups_lo, ups_hi = min(ups_ys), max(ups_ys)
    clearance = UPS_BOSS_MIN_CLEARANCE_MM + half_boss
    bosses = enclosure.lid_bosses()
    for wall, along in bosses:
        if wall == "bottom":
            assert abs(along - cl.RJ45_AT[0]) >= RJ45_JACK_HALF_WIDTH_MM + half_boss, along
        if wall == "-x":
            assert along + clearance <= ups_lo or along - clearance >= ups_hi, along


def test_shield_hood_above_camera_sweep(cl: ModuleType) -> None:
    pytest.importorskip("build123d")
    pytest.importorskip("recipe")
    camera_range = importlib.import_module("camera_range")
    envelope_top = float(camera_range.sweep("aim")["envelope"][1][2])
    assert cl.SWEEP_Z_MAX >= envelope_top
    assert cl.SHIELD_HOOD_Z0 > envelope_top
