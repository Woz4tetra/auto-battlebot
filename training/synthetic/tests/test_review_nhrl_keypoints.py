"""review_nhrl_keypoints sidecar writes: saving a review and rejecting a model."""

import json
from pathlib import Path

import cv2
import numpy as np
import review_nhrl_keypoints as review


def write_rejected_model(tmp_path: Path) -> Path:
    glb = tmp_path / "yeschef.glb"
    cv2.imwrite(str(tmp_path / "yeschef_top.png"), np.zeros((20, 20, 3), dtype=np.uint8))
    sidecar = {
        "name": "yeschef",
        "rejected": True,
        "reviewed": False,
        "topdown": {
            "image": "yeschef_top.png",
            "hull": [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]],
            "affine": [[10.0, 0.0, 10.0], [0.0, 10.0, 10.0]],
            "y_ground": 0.0,
        },
    }
    glb.with_suffix(".json").write_text(json.dumps(sidecar))
    return glb


def test_save_clears_rejected(tmp_path: Path) -> None:
    glb = write_rejected_model(tmp_path)
    model = review.Model(glb)
    assert model.rejected
    model.save(reviewed=True)
    stored = json.loads(glb.with_suffix(".json").read_text())
    assert stored["rejected"] is False
    assert stored["reviewed"] is True
    assert not model.rejected
    assert "topdown" in stored
