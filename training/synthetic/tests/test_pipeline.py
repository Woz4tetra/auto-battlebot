"""Pipeline invariants that hold without a Blender install: manifests, mounts, warps, cages."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from synthgen.lens import build_lens_view

from auto_battlebot.perception.camera_calibration import load_camera_calibration

ECAM25_PATH = (
    Path(__file__).resolve().parents[3] / "config" / "cameras" / "ecam25_h01r1_estimated.toml"
)
SIZE = (320, 180)


def make_scene_state(pipeline, cam_count: int, mounts: tuple):
    return pipeline.SceneState(
        scene_idx=4,
        arena_radius=2.0,
        scene_robots=[],
        robot_positions=[],
        active_distractors=[],
        cam_count=cam_count,
        mounts=mounts,
    )


def test_scene_state_rejects_mounts_that_do_not_match_the_camera_count(blender_stubs) -> None:
    from synthgen import pipeline

    with pytest.raises(ValueError, match="scene 4 has 2 mounts for 3 cameras"):
        make_scene_state(pipeline, cam_count=3, mounts=(object(), object()))


def test_scene_state_accepts_no_mounts_and_one_mount_per_camera(blender_stubs) -> None:
    from synthgen import pipeline

    assert make_scene_state(pipeline, cam_count=3, mounts=()).mounts == ()
    mounts = (object(), object(), object())
    assert make_scene_state(pipeline, cam_count=3, mounts=mounts).mounts == mounts


def test_warp_render_data_blends_colour_but_keeps_ids_and_depth_unblended(blender_stubs) -> None:
    from synthgen import pipeline

    lens = build_lens_view(load_camera_calibration(ECAM25_PATH), SIZE, "rectified", alpha=1.0)
    render_w, render_h = lens.render_size
    ids = np.zeros((render_h, render_w), dtype=np.int32)
    ids[: render_h // 2] = 7
    ids[render_h // 2 :] = 300_000
    depth = np.zeros((render_h, render_w), dtype=np.float32)
    depth[:, : render_w // 2] = 1.0
    depth[:, render_w // 2 :] = 4.0
    gradient = np.tile(np.linspace(0, 255, render_w, dtype=np.float32), (render_h, 1))
    colour = np.stack([gradient] * 3, axis=-1).astype(np.uint8)
    data = {
        "colors": [colour],
        "category_id_segmaps": [ids],
        "instance_segmaps": [ids],
        "depth": [depth],
    }

    warped = pipeline._warp_render_data(lens, data)

    assert set(np.unique(warped["category_id_segmaps"][0]).tolist()) <= {0, 7, 300_000}
    assert set(np.unique(warped["instance_segmaps"][0]).tolist()) <= {0, 7, 300_000}
    assert set(np.unique(warped["depth"][0]).tolist()) <= {0.0, 1.0, 4.0}
    assert len(np.unique(warped["colors"][0])) > 10


def test_warp_render_data_passes_an_unwarped_view_through_untouched(blender_stubs) -> None:
    from synthgen import pipeline

    data = {"colors": [np.zeros((4, 4, 3), dtype=np.uint8)]}
    assert pipeline._warp_render_data(None, data) is data
    lens = build_lens_view(load_camera_calibration(ECAM25_PATH), SIZE, "pinhole")
    assert pipeline._warp_render_data(lens, data) is data


def test_arrange_environment_deactivates_every_cage_it_is_not_using(
    blender_stubs, monkeypatch: pytest.MonkeyPatch
) -> None:
    from synthgen import pipeline

    monkeypatch.setattr(pipeline, "set_ground_visible", lambda *_: None)
    used = SimpleNamespace(
        arena_radius=2.5, activate=lambda _lights: None, deactivate=lambda: pytest.fail("used")
    )
    deactivated: list[str] = []
    idle = [
        SimpleNamespace(name=name, deactivate=lambda name=name: deactivated.append(name))
        for name in ("cage_a", "cage_b")
    ]
    assets = SimpleNamespace(cages=[idle[0], used, idle[1]], lights=[], ground=object())

    radius = pipeline._arrange_environment(SimpleNamespace(), assets, used)

    assert radius == 2.5
    assert deactivated == ["cage_a", "cage_b"], (
        "cages share the world origin, so a cage left showing blends its floor and lights in"
    )


def test_clearing_blockers_keeps_poses_and_mounts_aligned(
    blender_stubs, monkeypatch: pytest.MonkeyPatch
) -> None:
    from synthgen import pipeline

    poses = [np.eye(4) * (i + 1) for i in range(3)]
    original = [pose.copy() for pose in poses]
    mounts = ("wall_a", "wall_b", "wall_c")
    cleared: list[list[np.ndarray]] = []
    monkeypatch.setattr(pipeline, "robot_centroid", lambda _positions: (0.0, 0.0))
    monkeypatch.setattr(
        pipeline, "clear_poses_blocking_keypoints", lambda poses, *_: cleared.append(poses)
    )
    cage = SimpleNamespace(sample_camera_poses=lambda count, _centroid: (poses, mounts, 1))
    cfg = SimpleNamespace(output=SimpleNamespace(images_per_scene=3, ignore_obstructions=False))

    result = pipeline._scene_camera_poses(cfg, cage, [], [], [], 10, 0)

    assert cleared == [poses]
    assert len(result.poses) == len(result.mounts) == result.cam_count == 3
    assert all(np.array_equal(a, b) for a, b in zip(result.poses, original))
    assert result.mounts == mounts
    assert result.fallback_count == 1


def test_hdri_scene_has_no_mounts(blender_stubs, monkeypatch: pytest.MonkeyPatch) -> None:
    from synthgen import pipeline

    monkeypatch.setattr(pipeline, "setup_scene_cameras", lambda *_: ([np.eye(4), np.eye(4)], 2, 0))
    cfg = SimpleNamespace(
        camera=None, output=SimpleNamespace(images_per_scene=2, ignore_obstructions=True)
    )

    result = pipeline._scene_camera_poses(cfg, None, [], [], [], 10, 0)

    assert result.mounts == ()
    assert result.cam_count == 2


def test_fresh_run_replaces_the_manifest_and_a_resume_keeps_it(blender_stubs, tmp_path) -> None:
    from synthgen import pipeline

    manifest = tmp_path / "manifest.jsonl"
    layout = SimpleNamespace(manifest_path=manifest)
    manifest.write_text('{"image": "a.jpg"}\n')

    pipeline._reset_manifest_for_fresh_run(layout, start_index=12)
    assert manifest.read_text() == '{"image": "a.jpg"}\n'

    pipeline._reset_manifest_for_fresh_run(layout, start_index=0)
    assert not manifest.exists()
    pipeline._reset_manifest_for_fresh_run(layout, start_index=0)


def test_absent_instance_pass_is_none_whether_none_or_empty(blender_stubs) -> None:
    from synthgen import pipeline

    assert pipeline._map_or_none_when_pass_absent(None, 0) is None
    assert pipeline._map_or_none_when_pass_absent([], 0) is None
    assert pipeline._map_or_none_when_pass_absent(["a", "b"], 1) == "b"


def test_buffer_cleanup_frees_render_images_and_leaves_cached_textures(blender_stubs) -> None:
    from synthgen import pipeline

    def make_image(name: str, users: int) -> SimpleNamespace:
        return SimpleNamespace(
            name=name, users=users, buffers_free=lambda name=name: freed.append(name)
        )

    freed: list[str] = []
    removed: list[str] = []
    images = [
        make_image("Render Result", 0),
        make_image("Viewer Node", 2),
        make_image("cc_texture_Rock_Color", 0),
    ]
    blender_stubs.bpy.data.images.__iter__.side_effect = lambda: iter(images)
    blender_stubs.bpy.data.images.remove.side_effect = lambda img: removed.append(img.name)

    pipeline._free_render_buffers_keeping_cached_textures()

    assert freed == ["Render Result", "Viewer Node"]
    assert removed == ["Render Result"]
    blender_stubs.bpy.ops.outliner.orphans_purge.assert_not_called()
