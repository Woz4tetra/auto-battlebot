"""nhrl_common.py: pure-numpy helpers behind the NHRL distractor keypoints and review state."""

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
