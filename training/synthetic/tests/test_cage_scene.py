"""Cage stages share one Blender file: each one touches only the objects its own build made."""

from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

LIT_ENERGY_W = 800.0


def make_light(name: str, energy: float = LIT_ENERGY_W) -> SimpleNamespace:
    return SimpleNamespace(name=name, type="LIGHT", data=SimpleNamespace(energy=energy))


def make_collection(*objects: SimpleNamespace) -> SimpleNamespace:
    return SimpleNamespace(objects=list(objects), hide_render=True, hide_viewport=True)


def make_stage(cage_scene: Any, collection: SimpleNamespace) -> Any:
    return cage_scene.CageStage(
        SimpleNamespace(render_samples=None),
        MagicMock(),
        SimpleNamespace(render_k=None, render_size=(640, 480)),
        640,
        480,
        collection,
        [],
        {},
        None,
        SimpleNamespace(),
    )


def test_stage_lights_scoped_to_own_cage(
    blender_stubs: SimpleNamespace, monkeypatch: pytest.MonkeyPatch
) -> None:
    from synthgen import cage_scene

    own = make_light("wash_0")
    other_cage = make_light("wash_1")
    stage = make_stage(cage_scene, make_collection(own))
    make_stage(cage_scene, make_collection(other_cage))
    other_cage.data.energy = 0.0
    own.data.energy = 0.0
    for name in ("apply_exposure", "set_world", "set_denoiser"):
        monkeypatch.setattr(cage_scene, name, lambda *_a, **_k: None)
    stage.activate()
    assert own.data.energy == LIT_ENERGY_W
    assert other_cage.data.energy == 0.0


def test_house_bot_lookup_scoped_to_stage(
    blender_stubs: SimpleNamespace, monkeypatch: pytest.MonkeyPatch
) -> None:
    from synthgen import cage_scene

    own_bot = SimpleNamespace(name="house_bot")
    other_cages_bot = SimpleNamespace(name="house_bot")
    blender_stubs.bpy.data.objects = [other_cages_bot, own_bot]
    monkeypatch.setattr(cage_scene, "house_bot_keypoints", lambda parent: (np.zeros(3), np.ones(3)))
    house_bot = cage_scene._house_bot(False, [own_bot])
    assert house_bot is not None
    assert house_bot.parent is own_bot
    assert cage_scene._house_bot(False, [SimpleNamespace(name="mat")]) is None


def test_render_state_round_trips_fov_through_lens_and_sensor_only(
    blender_stubs: SimpleNamespace, monkeypatch: pytest.MonkeyPatch
) -> None:
    from synthgen import cage_scene

    cam = SimpleNamespace(
        lens=35.0,
        sensor_width=36.0,
        sensor_height=24.0,
        sensor_fit="HORIZONTAL",
        shift_x=0.1,
        shift_y=-0.2,
        clip_start=0.1,
        clip_end=100.0,
    )
    scene = MagicMock()
    scene.camera.data = cam
    blender_stubs.bpy.context.scene = scene
    state = cage_scene.snapshot_render_state()
    cam.lens = 12.0
    cam.sensor_width = 5.0
    cam.sensor_height = 3.0
    cam.sensor_fit = "VERTICAL"
    monkeypatch.setattr(cage_scene, "write_denoiser_without_availability_check", lambda *_a: None)
    cage_scene.restore_render_state(state)
    assert (cam.lens, cam.sensor_width, cam.sensor_height, cam.sensor_fit) == (
        35.0,
        36.0,
        24.0,
        "HORIZONTAL",
    )
    assert not hasattr(cam, "lens_unit")
    assert not hasattr(cam, "angle")
