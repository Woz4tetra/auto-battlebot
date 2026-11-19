"""Key frames of logo/gen_splash_animation.py land on the events they are named for."""

import importlib.util
from pathlib import Path
from types import ModuleType

import pytest

LOGO_SCRIPT = Path(__file__).resolve().parents[2] / "logo" / "gen_splash_animation.py"


@pytest.fixture(scope="module")
def splash() -> ModuleType:
    spec = importlib.util.spec_from_file_location("gen_splash_animation", LOGO_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_halves_meet_at_centre_before_spin(splash: ModuleType) -> None:
    kf = splash.KEYFRAMES
    meet = kf[splash.HALVES_MEET_FRAME]
    assert (meet.left, meet.right, meet.angle) == (0.0, 0.0, 0.0)
    assert kf[splash.HALVES_MEET_FRAME - 1].left < 0.0


def test_spin_completes_one_full_turn_with_halves_still_closed(splash: ModuleType) -> None:
    done = splash.KEYFRAMES[splash.SPIN_COMPLETE_FRAME]
    assert (done.left, done.right, done.angle, done.pill) == (0.0, 0.0, 360.0, 0.0)


def test_split_and_pill_wipe_start_together(splash: ModuleType) -> None:
    first = splash.KEYFRAMES[splash.SPLIT_START_FRAME]
    assert first.left < 0.0 < first.right
    assert 0.0 < first.pill < 1.0


def test_pill_fully_revealed_at_its_named_frame(splash: ModuleType) -> None:
    kf = splash.KEYFRAMES
    assert kf[splash.PILL_REVEALED_FRAME].pill == 1.0
    assert kf[splash.PILL_REVEALED_FRAME - 1].pill < 1.0


def test_overshoot_peak_is_the_widest_split(splash: ModuleType) -> None:
    split = splash.KEYFRAMES[splash.SPLIT_START_FRAME :]
    widths = [k.right - k.left for k in split]
    assert widths.index(max(widths)) == splash.OVERSHOOT_PEAK_FRAME - splash.SPLIT_START_FRAME


def test_final_frames_hold_the_settled_layout(splash: ModuleType) -> None:
    kf = splash.KEYFRAMES
    assert kf[splash.SETTLED_FRAME] == kf[-1]
