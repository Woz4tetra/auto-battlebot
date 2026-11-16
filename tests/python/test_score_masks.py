"""training/deeplab/score_masks.py: boundary frame-edge handling and field filter precedence."""

from __future__ import annotations

import numpy as np
import pytest


@pytest.fixture(scope="module")
def scorer(load_training):
    return load_training("training/deeplab/score_masks.py")


def test_boundary_ignores_frame_edge(scorer) -> None:
    mask = np.zeros((20, 20), dtype=np.uint8)
    mask[10:, :] = 1
    ring = scorer.boundary(mask)
    assert not ring[-1].any()
    assert not ring[11:, 0].any()
    assert ring[10].all()


def test_field_filter_overrides_exclude_field(scorer) -> None:
    fields = ["cage3", "unclassified", "cage4"]
    indices, reason = scorer.select_frames(fields, "unclassified", "unclassified")
    assert indices == [1]
    assert "--field-filter" in reason


def test_exclude_field_applies_without_filter(scorer) -> None:
    fields = ["cage3", "unclassified", "cage4"]
    indices, _ = scorer.select_frames(fields, None, "unclassified")
    assert indices == [0, 2]


def test_verdict_inverts_for_lower_is_better(scorer) -> None:
    assert scorer.verdict_for(0.1, 0.3, True) == "better"
    assert scorer.verdict_for(0.1, 0.3, False) == "worse"
    assert scorer.verdict_for(-0.3, -0.1, False) == "better"
    assert scorer.verdict_for(-0.1, 0.1, True) == "ns"


def test_unknown_metric_has_no_direction(scorer) -> None:
    assert scorer.higher_is_better("iou")
    assert scorer.higher_is_better("bf1@3")
    with pytest.raises(ValueError):
        scorer.higher_is_better("hausdorff")
