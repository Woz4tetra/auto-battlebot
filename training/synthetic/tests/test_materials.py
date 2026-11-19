"""Material roughness jitter stays inside its bound and the valid roughness range."""

import random
from types import SimpleNamespace
from typing import Any

import pytest

JITTER = 0.1


class FakeMaterial:
    def __init__(self, roughness: float) -> None:
        self.roughness = roughness

    def get_principled_shader_value(self, name: str) -> float:
        assert name == "Roughness"
        return self.roughness

    def set_principled_shader_value(self, name: str, value: float) -> None:
        self.roughness = value

    def get_name(self) -> str:
        return "fake"


def test_roughness_jitter_stays_within_bound_and_unit_range(blender_stubs: Any) -> None:
    from synthgen.materials import jitter_materials

    random.seed(3)
    starts = [0.0, 0.05, 0.5, 0.95, 1.0]
    materials = [FakeMaterial(start) for start in starts]
    mesh = SimpleNamespace(get_materials=lambda: materials)
    jitter_materials([mesh], JITTER)
    for start, material in zip(starts, materials):
        assert abs(material.roughness - start) <= JITTER + 1e-12
        assert 0.0 <= material.roughness <= 1.0
    assert any(m.roughness != s for m, s in zip(materials, starts))


def test_jitter_materials_takes_no_hue_argument(blender_stubs: Any) -> None:
    from synthgen.materials import jitter_materials

    with pytest.raises(TypeError):
        jitter_materials([], JITTER, 5.0)  # type: ignore[call-arg]
