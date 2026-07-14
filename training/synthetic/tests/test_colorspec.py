"""Tests for color parsing and color -> material matching."""

import pytest
from synthgen.colorspec import (
    ColorMappingEntry,
    color_distance,
    color_from_material_name,
    match_material_type,
)
from synthgen.constants import COLOR_MATCH_FALLBACK_MAX_DIST

MAPPING = (
    ColorMappingEntry(color=(128, 128, 128), tolerance=1, material="aluminum"),
    ColorMappingEntry(color=(0, 0, 0), tolerance=1, material="steel"),
    ColorMappingEntry(color=(255, 32, 65), tolerance=1, material="red_tpu"),
)


class TestColorFromMaterialName:
    def test_parses_float_triplet_name(self) -> None:
        assert color_from_material_name("0.501961_0.501961_0.501961_0.0") == (128, 128, 128)

    def test_rejects_out_of_range(self) -> None:
        assert color_from_material_name("2.0_0.5_0.5_x") is None

    def test_rejects_non_numeric(self) -> None:
        assert color_from_material_name("Metal_Shiny_01") is None


class TestColorDistance:
    def test_zero_for_identical(self) -> None:
        assert color_distance((10, 20, 30), (10, 20, 30)) == 0.0

    def test_euclidean(self) -> None:
        assert color_distance((0, 0, 0), (3, 4, 0)) == pytest.approx(5.0)


class TestMatchMaterialType:
    def test_exact_match(self) -> None:
        material, dist, fallback = match_material_type((128, 128, 128), MAPPING)
        assert material == "aluminum"
        assert dist == 0.0
        assert fallback is False

    def test_within_tolerance(self) -> None:
        material, _, fallback = match_material_type((128, 128, 129), MAPPING)
        assert material == "aluminum"
        assert fallback is False

    def test_nearest_fallback_within_ceiling(self) -> None:
        # 20 units off gray: outside tolerance 1, inside the 45.0 fallback.
        material, dist, fallback = match_material_type((148, 128, 128), MAPPING)
        assert material == "aluminum"
        assert dist == pytest.approx(20.0)
        assert fallback is True

    def test_no_match_beyond_fallback_ceiling(self) -> None:
        far_color = (128 + int(COLOR_MATCH_FALLBACK_MAX_DIST) + 60, 255, 255)
        material, _, fallback = match_material_type(far_color, MAPPING)
        assert material is None
        assert fallback is False

    def test_empty_mapping(self) -> None:
        material, dist, fallback = match_material_type((1, 2, 3), ())
        assert material is None
        assert dist == float("inf")
        assert fallback is False
