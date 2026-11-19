"""training/yolo/make_domain_mix_arms.py: nested arms, balanced prefixes, cage-high arm naming."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def arms(load_training):
    return load_training("training/yolo/make_domain_mix_arms.py")


def _domain(arms, per_group: int):
    frames = []
    for venue in (arms.NHRL, arms.MASSD):
        for view in ("pinhole", "rectified", "distorted"):
            for index in range(per_group):
                image = Path(f"/d/{venue}_{view}_{index}.png")
                frames.append(arms.DomainFrame(image, venue, view, index % 2 == 0))
    return frames


def test_domain_order_is_balanced_at_every_prefix(arms) -> None:
    domain = _domain(arms, 20)
    order = arms.domain_order(domain, 0, (arms.NHRL, arms.MASSD))
    by_image = {frame.image: frame for frame in domain}
    for prefix in (6, 12, 30, 60):
        picked = [by_image[image] for image in order[:prefix]]
        venues = [f.venue for f in picked]
        assert abs(venues.count(arms.NHRL) - venues.count(arms.MASSD)) <= 1
        counts = [[f.view for f in picked].count(v) for v in ("pinhole", "rectified", "distorted")]
        assert max(counts) - min(counts) <= 2


def test_larger_arm_contains_smaller(arms) -> None:
    domain = _domain(arms, 50)
    real = [Path(f"/r/{i}.png") for i in range(5)]
    randomized = [Path(f"/s/{i}.png") for i in range(40)]
    template = next(a for a in arms.ARMS if a.name == "d2500")
    small = dataclasses.replace(template, domain=40)
    large = dataclasses.replace(template, domain=200)
    for seed in range(5):
        small_frames, _ = arms.build_arm(small, real, randomized, domain, seed, [])
        large_frames, _ = arms.build_arm(large, real, randomized, domain, seed, [])
        assert set(small_frames) <= set(large_frames)


def test_cagehigh_arms_flagged_for_eval_leak(arms) -> None:
    flagged = {a.name for a in arms.ARMS if a.extra_real}
    assert flagged
    assert all(name.endswith("_cagehigh") for name in flagged)
    assert {a.name for a in arms.ARMS if a.name.endswith("_cagehigh")} == flagged


def test_arms_default_to_the_cage_venues_and_all_venue_arms_say_so(arms) -> None:
    by_name = {arm.name: arm for arm in arms.ARMS}
    assert by_name["d2500"].venues == arms.CAGE_VENUES
    assert by_name["nhrl_only"].venues == (arms.NHRL,)
    assert by_name["d50000"].venues == arms.ALL_VENUES


def test_real_repeat_multiplies_real_frames_but_extra_real_is_written_once(arms) -> None:
    domain = _domain(arms, 20)
    real = [Path(f"/r/{i}.png") for i in range(5)]
    extra = [Path(f"/e/{i}.png") for i in range(3)]
    randomized = [Path(f"/s/{i}.png") for i in range(40)]
    template = next(a for a in arms.ARMS if a.name == "d2500")
    arm = dataclasses.replace(template, domain=10, real_repeat=3, extra_real=True)

    frames, row = arms.build_arm(arm, real, randomized, domain, 0, extra)

    assert [frames.count(path) for path in real] == [3] * 5
    assert [frames.count(path) for path in extra] == [1] * 3
    assert row["real"] == 5 * 3 + 3
    assert row["extra_real"] == 3
    assert all(frames.count(path) == 1 for path in randomized if path in frames)
