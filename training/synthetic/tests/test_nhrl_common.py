"""nhrl_common.py: pure-numpy helpers behind the NHRL distractor keypoints and review state."""

import download_nhrl_bots
import generate_nhrl_meshes
import nhrl_common
import numpy as np
import pytest


def _bar(length: float = 4.0, width: float = 1.0, height: float = 0.5) -> np.ndarray:
    xs = np.linspace(-length / 2, length / 2, 9)
    zs = np.linspace(-width / 2, width / 2, 3)
    ys = np.linspace(0.0, height, 5)
    return np.array([[x, y, z] for x in xs for y in ys for z in zs])


def _yawed(vertices: np.ndarray, degrees: float) -> np.ndarray:
    angle = np.radians(degrees)
    rotation = np.array(
        [[np.cos(angle), 0.0, np.sin(angle)], [0.0, 1.0, 0.0], [-np.sin(angle), 0.0, np.cos(angle)]]
    )
    return vertices @ rotation.T


def test_footprint_axes_deterministic_sign() -> None:
    for degrees in range(0, 360, 15):
        vertices = _yawed(_bar(), degrees)
        axes = nhrl_common.footprint_axes(vertices, 0.2)
        assert axes is not None
        u1 = axes[1]
        assert u1[0] > -1e-9
        assert u1[0] > 1e-9 or u1[1] >= 0.0
        mirrored = nhrl_common.footprint_axes(vertices[::-1] * np.array([-1.0, 1.0, -1.0]), 0.2)
        assert mirrored is not None
        assert np.allclose(mirrored[1], u1, atol=1e-6)


def test_keypoints_reach_silhouette_ends() -> None:
    vertices = _bar()
    front, back = nhrl_common.compute_bottom_slice_keypoints(vertices, 0.2)
    assert front[0] == pytest.approx(2.0)
    assert back[0] == pytest.approx(-2.0)
    assert front[2] == pytest.approx(back[2], abs=1e-9)


def test_flipping_the_axis_swaps_front_and_back() -> None:
    vertices = _bar()
    base = np.array([0.0, 0.0])
    front, back = nhrl_common.keypoints_along_axis(base, np.array([1.0, 0.0]), vertices, 0.0)
    flipped_front, flipped_back = nhrl_common.keypoints_along_axis(
        base, np.array([-1.0, 0.0]), vertices, 0.0
    )
    assert np.allclose(front, flipped_back)
    assert np.allclose(back, flipped_front)


def test_convex_hull_is_counter_clockwise_and_drops_interior() -> None:
    points = np.array([[0, 0], [2, 0], [2, 2], [0, 2], [1, 1], [1, 0.5]], dtype=float)
    hull = nhrl_common.convex_hull_2d(points)
    assert len(hull) == 4
    area = 0.5 * np.sum(hull[:, 0] * np.roll(hull[:, 1], -1) - np.roll(hull[:, 0], -1) * hull[:, 1])
    assert area > 0.0


def test_imports_with_numpy_only() -> None:
    import ast
    from pathlib import Path

    tree = ast.parse(Path(nhrl_common.__file__).read_text())
    imported = {
        (node.module or "").split(".")[0] if isinstance(node, ast.ImportFrom) else alias.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in (getattr(node, "names", None) or [None])
        if node is not None
    }
    heavy = {"scipy", "cv2", "bpy", "blenderproc", "torch", "PIL", "yaml"}
    assert not imported & heavy


def _gltf_import_rotation(model: np.ndarray) -> np.ndarray:
    return np.column_stack([model[:, 0], -model[:, 2], model[:, 1]])


def test_world_to_model_inverts_gltf_import() -> None:
    model = np.array([[1.0, 2.0, 3.0], [-4.0, 0.5, 6.0], [0.0, 0.0, 1.0]])
    world = _gltf_import_rotation(model)
    assert np.allclose(nhrl_common.world_to_model(world), model)
    assert np.allclose(nhrl_common.world_to_model(np.array([[0.0, 0.0, 1.0]])), [[0.0, 1.0, 0.0]])


def test_overwrite_keeps_reviewed_sidecar() -> None:
    assert nhrl_common.sidecar_action({"reviewed": True}, overwrite=True) == "keep_reviewed"
    assert nhrl_common.sidecar_action({"reviewed": True}, overwrite=False) == "keep_reviewed"


def test_sidecar_action_rewrites_unreviewed_only_with_overwrite() -> None:
    assert nhrl_common.sidecar_action(None, overwrite=False) == "write"
    assert nhrl_common.sidecar_action({"reviewed": False}, overwrite=False) == "skip"
    assert nhrl_common.sidecar_action({"reviewed": False}, overwrite=True) == "write"


def test_rejected_bot_skipped_by_download_and_meshing() -> None:
    rejected = {
        nhrl_common.STATE_REJECTED_KEY: True,
        "status": nhrl_common.STATUS_DONE,
        "thumbnail_file": "bot.png",
    }
    accepted = {"status": nhrl_common.STATUS_DONE, "thumbnail_file": "bot.png"}
    assert nhrl_common.is_rejected(rejected)
    assert not nhrl_common.is_rejected(accepted)
    assert not generate_nhrl_meshes._usable(rejected)
    assert generate_nhrl_meshes._usable(accepted)


def test_rejected_thumbnail_is_sticky(tmp_path, monkeypatch) -> None:
    state_dir = tmp_path / "bots"
    state_dir.mkdir()
    bot = {"cleanName": "clyde", "name": "Clyde", "totalFights": 9, "weightClasses": [12]}
    entry = {nhrl_common.STATE_REJECTED_KEY: True, "status": "selected"}
    nhrl_common.save_state(state_dir / nhrl_common.STATE_FILENAME, {"clyde": entry})
    (state_dir / nhrl_common.BOT_CACHE_FILENAME).write_text("[]")
    monkeypatch.setattr(download_nhrl_bots, "load_or_fetch_bots", lambda *a: [bot])
    monkeypatch.setattr(
        download_nhrl_bots,
        "download_thumbnail",
        lambda *a: pytest.fail("rejected thumbnail was downloaded"),
    )
    monkeypatch.setattr("sys.argv", ["download_nhrl_bots.py", str(state_dir)])
    download_nhrl_bots.main()
    saved = nhrl_common.load_state(state_dir / nhrl_common.STATE_FILENAME)
    assert saved["clyde"][nhrl_common.STATE_REJECTED_KEY] is True
