"""Preview render settings: every mode transition writes its own state whole."""

from types import SimpleNamespace

import numpy as np
import pytest


def make_renderer(preview_render, setups: dict):
    renderer = object.__new__(preview_render.PreviewRenderer)
    renderer._cage_state = object()
    renderer._setups = setups
    renderer._applied = None
    renderer._spec = SimpleNamespace(
        render=SimpleNamespace(
            denoiser="none", samples=64, transmission_bounces=4, glossy_bounces=2
        )
    )
    return renderer


def make_setup(preview_render, k_value: float, size: tuple[int, int]):
    return preview_render.ViewSetup(
        lens=SimpleNamespace(warps=False),
        size=size,
        intrinsics=np.eye(3) * k_value,
        intrinsics_size=size,
        resolution_percentage=100,
    )


def test_intrinsics_are_rewritten_on_every_transition_even_for_a_cached_view(
    blender_stubs, monkeypatch: pytest.MonkeyPatch
) -> None:
    from synthgen import preview_render

    base = make_setup(preview_render, 1.0, (640, 360))
    other = make_setup(preview_render, 2.0, (320, 180))
    setups = {("pinhole", True, 1.0): base, ("pinhole", False, 1.0): other}
    renderer = make_renderer(preview_render, setups)
    monkeypatch.setattr(preview_render, "restore_render_state", lambda _state: None)
    monkeypatch.setattr(renderer, "_write_mode_settings", lambda *_: None)
    written = blender_stubs.blenderproc.camera.set_intrinsics_from_K_matrix

    renderer._apply("pinhole", preview=True, alpha=1.0)
    renderer._apply("pinhole", preview=False, alpha=1.0)
    renderer._apply("pinhole", preview=True, alpha=1.0)
    renderer._apply("pinhole", preview=True, alpha=1.0)

    assert [call.args[0][0, 0] for call in written.call_args_list] == [1.0, 2.0, 1.0]


@pytest.mark.parametrize("preview", [True, False])
def test_full_mode_disables_persistent_data_and_preview_enables_it(
    blender_stubs, preview: bool
) -> None:
    from synthgen import preview_render

    renderer = make_renderer(preview_render, {})
    scene = SimpleNamespace(
        render=SimpleNamespace(image_settings=SimpleNamespace()),
        cycles=SimpleNamespace(use_persistent_data=not preview),
    )

    renderer._write_mode_settings(scene, preview)

    assert scene.cycles.use_persistent_data is preview, (
        "a stale Cycles cache must never reach a saved full-quality artifact"
    )


def test_capture_scene_state_sets_the_cage_intrinsics_before_the_snapshot(
    blender_stubs, monkeypatch: pytest.MonkeyPatch
) -> None:
    from synthgen import preview_render

    renderer = make_renderer(preview_render, {})
    renderer._k_rect = {1.0: np.eye(3)}
    renderer._render_size = (1920, 1080)
    order: list[str] = []
    blender_stubs.blenderproc.camera.set_intrinsics_from_K_matrix.side_effect = lambda *_: (
        order.append("intrinsics")
    )
    monkeypatch.setattr(
        preview_render, "snapshot_render_state", lambda: order.append("snapshot") or object()
    )

    renderer.capture_scene_state()

    assert order == ["intrinsics", "snapshot"], (
        "a snapshot before the intrinsics would restore BlenderProc's default camera"
    )
    blender_stubs.blenderproc.camera.set_intrinsics_from_K_matrix.assert_called_once()
    assert blender_stubs.blenderproc.camera.set_intrinsics_from_K_matrix.call_args.args[1:] == (
        1920,
        1080,
    )


def mode_scene() -> SimpleNamespace:
    return SimpleNamespace(
        render=SimpleNamespace(image_settings=SimpleNamespace(), use_compositing=None),
        cycles=SimpleNamespace(),
    )


@pytest.mark.parametrize(
    ("denoiser", "preview", "expected_name", "compositing"),
    [
        ("none", True, "OPTIX", False),
        ("none", False, None, False),
        ("OPTIX", False, "OPTIX", False),
        ("INTEL", False, "INTEL", True),
        ("INTEL", True, "INTEL", True),
    ],
)
def test_denoiser_goes_through_blenderproc_and_compositing_follows_intel(
    blender_stubs,
    denoiser: str,
    preview: bool,
    expected_name: str | None,
    compositing: bool,
) -> None:
    from synthgen import preview_render

    renderer = make_renderer(preview_render, {})
    renderer._spec.render.denoiser = denoiser
    scene = mode_scene()

    renderer._write_mode_settings(scene, preview)

    blender_stubs.blenderproc.renderer.set_denoiser.assert_called_once_with(expected_name)
    assert scene.render.use_compositing is compositing


def test_one_way_glass_rewrites_visibility_only_when_the_hidden_set_changes(
    blender_stubs, monkeypatch: pytest.MonkeyPatch
) -> None:
    from synthgen import preview_render

    writes: list[bool] = []

    class Pane:
        def animation_data_clear(self) -> None:
            pass

        @property
        def hide_render(self) -> bool:
            return False

        @hide_render.setter
        def hide_render(self, value: bool) -> None:
            writes.append(value)

    panes = {"north": SimpleNamespace(blender_obj=Pane())}
    monkeypatch.setattr(preview_render, "panels_by_wall", lambda _objects: panes)
    hidden_sets = iter([("north",), ("north",), ()])
    monkeypatch.setattr(preview_render, "panels_outside_camera", lambda *_: next(hidden_sets))
    glass = preview_render.OneWayGlass(SimpleNamespace(), {})

    camera = np.zeros(3)
    glass.update(camera)
    glass.update(camera)
    glass.update(camera)

    assert writes == [True, False]
