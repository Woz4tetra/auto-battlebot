"""set_denoiser clears BlenderProc's compositor denoiser and enables Cycles' own."""

from types import SimpleNamespace
from typing import Any

import pytest


def cycles_scene(blender_stubs: Any, compute_device_type: str) -> SimpleNamespace:
    scene = SimpleNamespace(cycles=SimpleNamespace())
    blender_stubs.bpy.context.scene = scene
    blender_stubs.bpy.context.view_layer = SimpleNamespace(cycles=SimpleNamespace())
    preferences = SimpleNamespace(compute_device_type=compute_device_type)
    blender_stubs.bpy.context.preferences.addons = {
        "cycles": SimpleNamespace(preferences=preferences)
    }
    return scene


@pytest.mark.parametrize(
    ("name", "compute_device_type", "expected"),
    [
        ("OPTIX", "OPTIX", "OPTIX"),
        ("OPTIX", "CUDA", "OPENIMAGEDENOISE"),
        ("INTEL", "OPTIX", "OPENIMAGEDENOISE"),
    ],
)
def test_set_denoiser_clears_the_blenderproc_denoiser_then_picks_the_cycles_one(
    blender_stubs: Any, name: str, compute_device_type: str, expected: str
) -> None:
    from synthgen.render_settings import set_denoiser

    scene = cycles_scene(blender_stubs, compute_device_type)

    set_denoiser(name)

    blender_stubs.blenderproc.renderer.set_denoiser.assert_called_once_with(None)
    assert scene.cycles.denoiser == expected
    assert scene.cycles.use_denoising is True


def test_set_denoiser_none_leaves_cycles_denoising_untouched(blender_stubs: Any) -> None:
    from synthgen.render_settings import set_denoiser

    scene = cycles_scene(blender_stubs, "OPTIX")

    set_denoiser("none")

    blender_stubs.blenderproc.renderer.set_denoiser.assert_called_once_with(None)
    assert not hasattr(scene.cycles, "denoiser")


def test_set_denoiser_rejects_unknown_names(blender_stubs: Any) -> None:
    from synthgen.render_settings import set_denoiser

    cycles_scene(blender_stubs, "OPTIX")

    with pytest.raises(ValueError, match="unknown denoiser"):
        set_denoiser("nlmeans")
