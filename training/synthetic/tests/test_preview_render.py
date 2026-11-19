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
