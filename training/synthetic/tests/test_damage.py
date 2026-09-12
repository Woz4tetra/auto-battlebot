"""Damage draws: what is protected, what gets removed, and how big a cutter is."""

import math
import random

import numpy as np
import pytest
from synthgen.damage import (
    CUTTER_CUBE,
    CUTTER_ICOSPHERE,
    MECHANISM_NONE,
    MECHANISM_PARTS,
    DamageBudget,
    InstanceDamage,
    bbox_contains,
    bbox_corner_volume,
    cutter_radius_m,
    draw_chunk_damage,
    draw_part_damage,
    protected_part_indices,
    sample_surface_points,
)


def box(centre, half):
    """The 8 corners of an axis-aligned box, the way Blender's bound_box comes out."""
    cx, cy, cz = centre
    h = half
    return np.array(
        [
            [cx + sx * h, cy + sy * h, cz + sz * h]
            for sx in (-1, 1)
            for sy in (-1, 1)
            for sz in (-1, 1)
        ],
        dtype=np.float64,
    )


class TestBoxHelpers:
    def test_volume_is_the_spanned_box(self):
        assert bbox_corner_volume(box((0, 0, 0), 0.5)) == pytest.approx(1.0)

    def test_empty_corners_have_no_volume(self):
        assert bbox_corner_volume(np.zeros((0, 3))) == 0.0

    def test_contains_respects_the_margin(self):
        corners = box((0, 0, 0), 0.1)
        assert bbox_contains(corners, np.array([0.0, 0.0, 0.0]), 0.0)
        assert not bbox_contains(corners, np.array([0.15, 0.0, 0.0]), 0.0)
        assert bbox_contains(corners, np.array([0.15, 0.0, 0.0]), 0.06)


class TestProtection:
    def test_keypoint_anchor_is_protected(self):
        parts = [box((0.0, 0, 0), 0.05), box((0.5, 0, 0), 0.05)]
        keypoints = [np.array([0.0, 0.0, 0.0])]
        assert protected_part_indices(parts, ["a", "b"], keypoints, (), 1.0, 0.0) == {0}

    def test_oversized_part_is_protected(self):
        # One part holds 8^3 = 512 times the volume of the other, so it is nearly all of it.
        parts = [box((0, 0, 0), 0.4), box((1.0, 0, 0), 0.05)]
        assert protected_part_indices(parts, ["big", "small"], [], (), 0.45, 0.0) == {0}

    def test_name_pattern_is_protected(self):
        parts = [box((0, 0, 0), 0.05), box((1.0, 0, 0), 0.05)]
        got = protected_part_indices(parts, ["Chassis.001", "wheel"], [], ("chassis",), 1.0, 0.0)
        assert got == {0}

    def test_nothing_protected_when_no_rule_bites(self):
        parts = [box((0, 0, 0), 0.05), box((1.0, 0, 0), 0.05)]
        assert protected_part_indices(parts, ["a", "b"], [], (), 1.0, 0.0) == set()


class TestPartDraw:
    def test_single_mesh_cannot_lose_parts(self):
        parts = [box((0, 0, 0), 0.1)]
        assert draw_part_damage(parts, ["only"], [], (0.1, 0.3), (), 1.0, 0.0) is None

    def test_all_protected_yields_nothing(self):
        parts = [box((0, 0, 0), 0.05), box((0.2, 0, 0), 0.05)]
        keypoints = [np.array([0.0, 0.0, 0.0]), np.array([0.2, 0.0, 0.0])]
        assert draw_part_damage(parts, ["a", "b"], keypoints, (0.1, 0.3), (), 1.0, 0.0) is None

    def test_never_removes_a_protected_part(self):
        random.seed(0)
        parts = [box((i * 0.2, 0, 0), 0.05) for i in range(6)]
        names = [f"part{i}" for i in range(6)]
        keypoints = [np.array([0.0, 0.0, 0.0])]
        for _ in range(50):
            draw = draw_part_damage(parts, names, keypoints, (0.05, 1.0), (), 1.0, 0.0)
            assert draw is not None
            assert 0 not in draw.part_indices

    def test_removes_at_least_one_part_even_at_low_severity(self):
        random.seed(1)
        parts = [box((i * 0.2, 0, 0), 0.05) for i in range(3)]
        draw = draw_part_damage(parts, ["a", "b", "c"], [], (0.0, 0.0), (), 1.0, 0.0)
        assert draw is not None and len(draw.part_indices) == 1

    def test_damage_is_the_fraction_of_all_parts(self):
        random.seed(2)
        parts = [box((i * 0.2, 0, 0), 0.05) for i in range(4)]
        draw = draw_part_damage(parts, list("abcd"), [], (1.0, 1.0), (), 1.0, 0.0)
        assert draw is not None
        assert draw.damage == pytest.approx(len(draw.part_indices) / 4)


class TestCutterSize:
    def test_cube_half_extent_removes_the_requested_volume(self):
        volume, fraction = 0.008, 0.125
        half = cutter_radius_m(volume, fraction, CUTTER_CUBE)
        assert (2 * half) ** 3 == pytest.approx(volume * fraction)

    def test_sphere_radius_removes_the_requested_volume(self):
        volume, fraction = 0.008, 0.125
        radius = cutter_radius_m(volume, fraction, CUTTER_ICOSPHERE)
        assert (4 / 3) * math.pi * radius**3 == pytest.approx(volume * fraction)

    def test_unknown_shape_is_an_error(self):
        with pytest.raises(ValueError, match="cutter shape"):
            cutter_radius_m(1.0, 0.1, "dodecahedron")


class TestChunkDraw:
    def test_no_surface_points_yields_nothing(self):
        bounds = box((0, 0, 0), 0.1)
        assert draw_chunk_damage([], bounds, [], (0.1, 0.2), (CUTTER_CUBE,), 0.0) is None

    def test_seeds_away_from_keypoints(self):
        random.seed(3)
        surface = [np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])]
        keypoints = [np.array([0.0, 0.0, 0.0])]
        for _ in range(25):
            draw = draw_chunk_damage(
                surface, box((0, 0, 0), 0.5), keypoints, (0.01, 0.02), (CUTTER_CUBE,), 0.05
            )
            assert draw is not None
            assert draw.centre == (1.0, 0.0, 0.0)

    def test_gives_up_when_every_seed_sits_on_a_keypoint(self):
        random.seed(4)
        surface = [np.array([0.0, 0.0, 0.0])]
        keypoints = [np.array([0.0, 0.0, 0.0])]
        got = draw_chunk_damage(
            surface, box((0, 0, 0), 0.5), keypoints, (0.1, 0.2), (CUTTER_CUBE,), 0.01
        )
        assert got is None

    def test_zero_volume_mesh_yields_nothing(self):
        surface = [np.array([0.0, 0.0, 0.0])]
        flat = np.zeros((8, 3))
        assert draw_chunk_damage(surface, flat, [], (0.1, 0.2), (CUTTER_CUBE,), 0.0) is None


class TestInstanceDamage:
    def test_undamaged_reads_as_undamaged(self):
        assert not InstanceDamage("mrs_buff_mk3", 0.0, MECHANISM_NONE).is_damaged

    def test_manifest_row_rounds_and_names_the_mechanism(self):
        row = InstanceDamage("mrs_buff_mk3", 0.183333, MECHANISM_PARTS).as_row()
        assert row == {"class": "mrs_buff_mk3", "damage": 0.1833, "mechanism": "parts"}


class TestSurfaceSampling:
    def test_short_lists_pass_through(self):
        points = [np.zeros(3), np.ones(3)]
        assert len(sample_surface_points(points, limit=8)) == 2

    def test_long_lists_are_capped(self):
        random.seed(5)
        points = [np.array([float(i), 0.0, 0.0]) for i in range(500)]
        assert len(sample_surface_points(points, limit=16)) == 16


class TestDamageBudget:
    """The damaged/clean scene split has to hold over a handful of scenes, not thousands."""

    def test_half_alternates_exactly(self):
        budget = DamageBudget()
        taken = [budget.take(0.5) for _ in range(10)]
        assert sum(taken) == 5
        assert budget.scenes == 10 and budget.damaged == 5

    @pytest.mark.parametrize("probability", [0.0, 0.2, 0.35, 0.5, 0.65, 1.0])
    def test_lands_on_the_ratio(self, probability):
        budget = DamageBudget()
        n = 200
        taken = sum(budget.take(probability) for _ in range(n))
        # A tracked split is never more than one scene off its share.
        assert abs(taken - probability * n) <= 1

    def test_never_damages_at_zero(self):
        budget = DamageBudget()
        assert not any(budget.take(0.0) for _ in range(20))

    def test_always_damages_at_one(self):
        budget = DamageBudget()
        assert all(budget.take(1.0) for _ in range(20))

    def test_short_runs_are_not_a_lottery(self):
        # Ten scenes is what a 100-frame probe renders; a coin flip put one in ten on the
        # damaged side, which is what this replaces.
        budget = DamageBudget()
        assert sum(budget.take(0.5) for _ in range(10)) == 5
