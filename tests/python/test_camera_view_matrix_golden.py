"""camera_view_matrix matches the golden file the C++ sim keypoint test checks against."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
GOLDEN = REPO_ROOT / "tests" / "keypoint_model" / "camera_view_matrix_golden.json"


def _camera_view_matrix():
    spec = importlib.util.spec_from_file_location(
        "camera_utils", REPO_ROOT / "simulation" / "camera_utils.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.camera_view_matrix


def test_camera_view_matrix_matches_golden() -> None:
    camera_view_matrix = _camera_view_matrix()
    cases = json.loads(GOLDEN.read_text())
    assert cases
    for case in cases:
        matrix = camera_view_matrix(case["position"], case["lookat"])
        np.testing.assert_allclose(matrix, np.array(case["camera_from_world"]), atol=1e-10)
