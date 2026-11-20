"""Cage builders that depend on Blender state: depsgraph timing, world nodes, denoiser restore."""

from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest


def test_mat_uvs_come_from_the_box_not_from_matrix_world(blender_stubs: Any) -> None:
    from synthgen import cage
    from synthgen.cage_spec import Box, mat_image_uv

    class BlenderObject:
        def __init__(self, data: Any) -> None:
            self.data = data

        @property
        def matrix_world(self) -> Any:
            raise AssertionError("matrix_world is identity until the depsgraph updates")

    verts = [SimpleNamespace(co=SimpleNamespace(x=x, y=y)) for x, y in ((-1.0, -1.0), (1.0, 1.0))]
    uvs = [SimpleNamespace(uv=None) for _ in verts]
    mesh = SimpleNamespace(
        uv_layers=MagicMock(active=SimpleNamespace(data=uvs)),
        polygons=[SimpleNamespace(loop_indices=[0, 1])],
        vertices=verts,
        loops=[SimpleNamespace(vertex_index=i) for i in range(2)],
    )
    box = Box(name="mat", center=(1.0, 2.0, 0.0), size=(4.0, 6.0, 0.1), material="mat")
    cage._set_mat_uvs(SimpleNamespace(blender_obj=BlenderObject(mesh)), box, 5.0)
    assert uvs[0].uv == pytest.approx(mat_image_uv(-1.0, -1.0, 5.0))
    assert uvs[1].uv == pytest.approx(mat_image_uv(3.0, 5.0, 5.0))


def test_drop_nodes_keeps_only_background_and_output(blender_stubs: Any) -> None:
    from synthgen import cage

    nodes = [
        SimpleNamespace(type=t)
        for t in ("BACKGROUND", "OUTPUT_WORLD", "TEX_ENVIRONMENT", "MAPPING", "TEX_COORD")
    ]
    tree = SimpleNamespace(nodes=MagicMock())
    tree.nodes.__iter__.side_effect = lambda: iter(list(nodes))
    cage.drop_nodes_added_by_previous_hdri_loads(tree)
    removed = [c.args[0].type for c in tree.nodes.remove.call_args_list]
    assert removed == ["TEX_ENVIRONMENT", "MAPPING", "TEX_COORD"]


def test_set_world_drops_previous_hdri_nodes_before_loading_the_next(
    blender_stubs: Any, monkeypatch: pytest.MonkeyPatch, tmp_path: Any
) -> None:
    from synthgen import cage

    (tmp_path / "sky.exr").touch()
    order: list[str] = []
    monkeypatch.setattr(
        cage, "drop_nodes_added_by_previous_hdri_loads", lambda _t: order.append("drop")
    )
    blender_stubs.blenderproc.world.set_world_background_hdr_img.side_effect = lambda *_a, **_k: (
        order.append("load")
    )
    spec = SimpleNamespace(
        world=SimpleNamespace(
            hdri="sky.exr", ambient_strength=1.0, hdri_rotation_deg=0.0, background_color=(0, 0, 0)
        )
    )
    cage.set_world(spec, repo_root=tmp_path)
    assert order == ["drop", "load"]


def test_restore_render_state_writes_denoiser_fields_without_calling_set_denoiser(
    blender_stubs: Any,
) -> None:
    from synthgen import cage_scene

    scene = MagicMock()
    blender_stubs.bpy.context.scene = scene
    state = cage_scene.snapshot_render_state()
    scene.cycles.denoiser = "unset"
    blender_stubs.blenderproc.renderer.set_denoiser.reset_mock()
    cage_scene.restore_render_state(state)
    blender_stubs.blenderproc.renderer.set_denoiser.assert_not_called()
    assert scene.cycles.denoiser == state.denoiser
