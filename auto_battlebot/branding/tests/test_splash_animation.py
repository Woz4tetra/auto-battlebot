"""Key frames of splash_animation land on the events they are named for."""

import shutil
from pathlib import Path
from types import ModuleType

import pytest

from auto_battlebot.branding import splash_animation


@pytest.fixture(scope="module")
def splash() -> ModuleType:
    return splash_animation


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


@pytest.mark.skipif(shutil.which("inkscape") is None, reason="inkscape not installed")
def test_pill_layer_survives_dangling_clip_path(splash: ModuleType) -> None:
    layer = splash.render_pill_layer(splash.ASSETS_DIR / "bwbots_logo.svg", 100.0)
    assert layer.width > 50 and layer.height > 10


def test_wrapper_cli_writes_a_gif_of_the_requested_size(tmp_path: Path) -> None:
    import subprocess
    import sys

    from PIL import Image

    from auto_battlebot.paths import REPO_ROOT

    out = tmp_path / "splash.gif"
    subprocess.run(
        [
            sys.executable,
            "logo/gen_splash_animation.py",
            "--size",
            "160x128",
            "--no-verify",
            "--out",
            str(out),
        ],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
    )
    with Image.open(out) as gif:
        assert gif.size == (160, 128)
        assert gif.n_frames > 10
