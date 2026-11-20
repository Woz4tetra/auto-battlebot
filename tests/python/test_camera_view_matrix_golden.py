"""camera_view_matrix matches the golden file the C++ sim keypoint test checks against."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from auto_battlebot.kinematic_sim.camera_utils import camera_view_matrix

REPO_ROOT = Path(__file__).resolve().parents[2]
GOLDEN = REPO_ROOT / "tests" / "keypoint_model" / "camera_view_matrix_golden.json"


def test_camera_view_matrix_matches_golden() -> None:
    cases = json.loads(GOLDEN.read_text())
    assert cases
    for case in cases:
        matrix = camera_view_matrix(case["position"], case["lookat"])
        np.testing.assert_allclose(matrix, np.array(case["camera_from_world"]), atol=1e-10)
