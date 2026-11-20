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


class FakeMaterialStore:
    def __init__(self, names: list[str]) -> None:
        self.names = names

    def get(self, name: str) -> object | None:
        return f"bpy:{name}" if name in self.names else None


def test_load_cc_materials_asks_blenderproc_only_for_names_not_in_the_blend_file(
    blender_stubs: Any, tmp_path: Any
) -> None:
    from synthgen.materials import load_cc_materials

    blender_stubs.bpy.data.materials = FakeMaterialStore(["Wood01"])
    loader = blender_stubs.blenderproc.loader.load_ccmaterials
    loader.return_value = [SimpleNamespace(get_name=lambda: "Tile02")]
    config = {
        "a": SimpleNamespace(cc_texture="Wood01"),
        "b": SimpleNamespace(cc_texture="Tile02"),
        "c": SimpleNamespace(cc_texture=None),
    }

    result = load_cc_materials(config, tmp_path, lambda path: path)

    assert loader.call_args.kwargs["used_assets"] == ["Tile02"]
    assert set(result) == {"Wood01", "Tile02"}


def test_load_cc_materials_skips_blenderproc_when_every_name_is_in_the_blend_file(
    blender_stubs: Any, tmp_path: Any
) -> None:
    from synthgen.materials import load_cc_materials

    blender_stubs.bpy.data.materials = FakeMaterialStore(["Wood01"])
    config = {"a": SimpleNamespace(cc_texture="Wood01")}

    result = load_cc_materials(config, tmp_path, lambda path: path)

    blender_stubs.blenderproc.loader.load_ccmaterials.assert_not_called()
    assert set(result) == {"Wood01"}
