"""YOLO keypoint fixes: projection, calibration against the tag, board gate, dropout-only use."""

from __future__ import annotations

import math

import numpy as np
import pandas as pd
import pytest

from auto_battlebot.perception import yolo_field_fixes as yff

K = np.array([[665.0, 0.0, 960.0], [0.0, 665.0, 600.0], [0.0, 0.0, 1.0]])
PLANE_H = -0.023
AXLE = -0.046
HALF_SEP = 0.065
OFFSET_NS = -25_000_000
FPS = 60.0


def _camera() -> np.ndarray:
    """Camera 1.04 m up, looking down and 23 degrees off vertical toward +y (up frame)."""
    tilt = math.radians(23.0)
    # OpenCV camera axes in the up frame: x right, y down the image, z forward.
    z = np.array([0.0, math.sin(tilt), -math.cos(tilt)])
    x = np.array([1.0, 0.0, 0.0])
    y = np.cross(z, x)
    pose = np.eye(4)
    pose[:3, :3] = np.stack([x, y, z], axis=1)
    pose[:3, 3] = [0.0, -0.45, 1.04]
    return pose


CAM = _camera()


def up_from_camera_at(stamps_ns: np.ndarray) -> np.ndarray:
    return np.broadcast_to(CAM, (len(stamps_ns), 4, 4)).copy()


def _pixel(p_up: np.ndarray) -> np.ndarray:
    cam_from_up = np.linalg.inv(CAM)
    p = cam_from_up[:3, :3] @ p_up.T + cam_from_up[:3, 3:4]
    uv = K @ p
    return (uv[:2] / uv[2]).T


def _truth(t: np.ndarray) -> dict[str, np.ndarray]:
    return {
        "x": 0.3 * np.sin(0.7 * t),
        "y": 0.25 * np.sin(0.45 * t + 0.3),
        "yaw": 0.9 * t,
    }


def _detections(stamps_ns: np.ndarray, rng: np.random.Generator) -> pd.DataFrame:
    """YOLO rows for video stamps; the frame was captured at stamp + OFFSET_NS."""
    tr = _truth((stamps_ns + OFFSET_NS) / 1e9)
    c, s = np.cos(tr["yaw"]), np.sin(tr["yaw"])
    mid_x, mid_y = tr["x"] - AXLE * c, tr["y"] - AXLE * s
    front = np.stack([mid_x + HALF_SEP * c, mid_y + HALF_SEP * s, np.full_like(c, PLANE_H)], 1)
    back = np.stack([mid_x - HALF_SEP * c, mid_y - HALF_SEP * s, np.full_like(c, PLANE_H)], 1)
    fu = _pixel(front) + rng.normal(0, 0.6, (len(c), 2))
    bu = _pixel(back) + rng.normal(0, 0.6, (len(c), 2))
    lo, hi = np.minimum(fu, bu) - 30, np.maximum(fu, bu) + 30
    return pd.DataFrame(
        {
            "stamp_ns": stamps_ns,
            "conf": 0.8,
            "x1": lo[:, 0],
            "y1": lo[:, 1],
            "x2": hi[:, 0],
            "y2": hi[:, 1],
            "front_u": fu[:, 0],
            "front_v": fu[:, 1],
            "front_conf": 0.9,
            "back_u": bu[:, 0],
            "back_v": bu[:, 1],
            "back_conf": 0.9,
        }
    )


@pytest.fixture(scope="module")
def scene() -> dict:
    rng = np.random.default_rng(3)
    video = (np.arange(0, 40.0, 1.0 / FPS) * 1e9).astype(np.int64) + 7_000_000
    det = _detections(video, rng)
    # Tag frames share the camera frames; a 1.5 s dropout at 20 s.
    tag_t = video + OFFSET_NS
    tag_t = tag_t[(tag_t < 20e9) | (tag_t > 21.5e9)]
    tr = _truth(tag_t / 1e9)
    return {"det": det, "tag_t": tag_t, "truth": tr, "video": video}


def test_projection_round_trips() -> None:
    stamps = np.array([1_000_000_000], dtype=np.int64)
    det = _detections(stamps, np.random.default_rng(0))
    out = yff.project_detections(det, K, up_from_camera_at(stamps), PLANE_H, AXLE)
    tr = _truth((stamps + OFFSET_NS) / 1e9)
    assert out["x"][0] == pytest.approx(tr["x"][0], abs=0.003)
    assert out["y"][0] == pytest.approx(tr["y"][0], abs=0.003)
    assert out["sep"][0] == pytest.approx(2 * HALF_SEP, abs=0.005)


def test_calibration_recovers_the_projection(scene: dict) -> None:
    tr = scene["truth"]
    cal = yff.calibrate(
        scene["det"], scene["tag_t"], tr["x"], tr["y"], tr["yaw"], K, up_from_camera_at, 0.0
    )
    assert cal.offset_ns == pytest.approx(OFFSET_NS, abs=2_000_000)
    assert cal.plane_height_m == pytest.approx(PLANE_H, abs=0.004)
    assert cal.axle_offset_m == pytest.approx(AXLE, abs=0.004)
    assert cal.position_error_mm["p50"] < 3.0
    assert cal.keypoint_sep_m[0] < 2 * HALF_SEP < cal.keypoint_sep_m[1]


def test_fixes_fill_only_the_dropout_and_skip_the_board(scene: dict) -> None:
    tr = scene["truth"]
    cal = yff.calibrate(
        scene["det"], scene["tag_t"], tr["x"], tr["y"], tr["yaw"], K, up_from_camera_at, 0.0
    )
    det = scene["det"].copy()
    # A board-sized detection, more confident than the robot, in every dropout frame.
    gap = (det["stamp_ns"] + OFFSET_NS > 20e9) & (det["stamp_ns"] + OFFSET_NS < 21.5e9)
    board = det[gap].assign(x1=700.0, y1=400.0, x2=1000.0, y2=640.0, conf=0.95)
    det = pd.concat([det, board], ignore_index=True)
    fixes, report = yff.select_fixes(det, cal, K, up_from_camera_at, scene["tag_t"], None)
    assert report["dropped_by_size_gate"] == len(board)
    t = fixes["stamp_ns"].to_numpy() / 1e9
    assert t.min() > 20.0 and t.max() < 21.5
    assert len(fixes) == pytest.approx(1.5 * FPS, abs=3)
    truth = _truth(t)
    err = np.hypot(fixes["x"] - truth["x"], fixes["y"] - truth["y"])
    assert float(err.max()) < 0.01


def test_flat_mask_drops_inverted_and_tilted() -> None:
    esp32 = pd.DataFrame(
        {
            "stamp_ns": np.arange(6) * 100_000_000,
            "is_upside_down": [0, 0, 1, 0, 0, 0],
            "orientation_y": [-3.0, -3.5, -3.0, 30.0, -3.2, -3.1],
            "orientation_z": [0.0, 0.5, 0.0, 0.0, 25.0, 0.2],
        }
    )
    stamps = np.arange(6) * 100_000_000 + 10_000_000
    assert yff.flat_mask(esp32, stamps).tolist() == [True, True, False, False, False, True]
