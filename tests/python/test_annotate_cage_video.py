"""annotate_cage_video keypoint order and fighter class selection."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from auto_battlebot.eval import metrics
from playground.bgsub_cage import annotate_cage_video as annotate


class _Tensor:
    def __init__(self, array: np.ndarray) -> None:
        self._array = array

    def cpu(self) -> _Tensor:
        return self

    def numpy(self) -> np.ndarray:
        return self._array

    def __len__(self) -> int:
        return len(self._array)


def _result(front_xy: tuple[float, float], back_xy: tuple[float, float]) -> SimpleNamespace:
    keypoints = np.zeros((1, 2, 3))
    keypoints[0, annotate.FRONT_IDX] = (*front_xy, 0.9)
    keypoints[0, annotate.BACK_IDX] = (*back_xy, 0.9)
    boxes = _Tensor(np.array([[0.0, 0.0, 10.0, 10.0]]))
    boxes.xyxy = boxes  # type: ignore[attr-defined]
    boxes.conf = _Tensor(np.array([0.9]))  # type: ignore[attr-defined]
    boxes.cls = _Tensor(np.array([0.0]))  # type: ignore[attr-defined]
    return SimpleNamespace(
        names={0: "robot"},
        boxes=boxes,
        keypoints=SimpleNamespace(data=_Tensor(keypoints)),
    )


def test_keypoint_order_matches_scoring() -> None:
    assert annotate.FRONT_IDX == metrics.FRONT_IDX
    assert annotate.BACK_IDX == metrics.BACK_IDX


def test_heading_points_from_back_to_front() -> None:
    pose = annotate.parse_poses(_result(front_xy=(8.0, 5.0), back_xy=(2.0, 5.0)), None)[0]
    assert pose["heading_deg"] == 0.0
    pose = annotate.parse_poses(_result(front_xy=(5.0, 2.0), back_xy=(5.0, 8.0)), None)[0]
    assert pose["heading_deg"] == 90.0


def test_fighter_class_index_2class() -> None:
    assert annotate.fighter_class_index({0: "robot", 1: "other"}) == 0


def test_fighter_class_index_4class() -> None:
    names = {0: "mr_stabs", 1: "mrs_buff", 2: "nhrl_robot", 3: "house_bot"}
    assert annotate.fighter_class_index(names) == 2
