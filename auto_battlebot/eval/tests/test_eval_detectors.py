from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from auto_battlebot.eval import FrameKey
from auto_battlebot.eval.detectors import PrecomputedDetector


def _replay(tmp_path: Path) -> PrecomputedDetector:
    path = tmp_path / "detections.json"
    path.write_text(
        json.dumps(
            {
                "labels": ["a"],
                "frames": {"5": [{"xyxy": [0, 0, 1, 1], "score": 0.9, "class_id": 0}]},
            }
        )
    )
    return PrecomputedDetector(path, conf_threshold=0.5)


def test_precomputed_replays_single_recording(tmp_path: Path) -> None:
    detections = _replay(tmp_path).detect(np.zeros((2, 2, 3)), FrameKey(".", 5))
    assert len(detections) == 1


def test_precomputed_rejects_multi_recording_root(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="multi-recording"):
        _replay(tmp_path).detect(np.zeros((2, 2, 3)), FrameKey("rec_a", 5))


def test_build_detector_takes_keywords_and_defaults_preprocessing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from auto_battlebot.eval import detectors
    from auto_battlebot.perception.trt_yolo import PreprocessMode

    seen: list[dict] = []

    class FakeModel:
        def __init__(self, path: str, **kwargs) -> None:
            seen.append(kwargs)

    monkeypatch.setattr(detectors, "TrtYoloModel", FakeModel)
    engine = tmp_path / "m.engine"
    engine.write_bytes(b"")
    built = detectors.build_detector("m", engine, ["a", "b"], {}, conf=0.3, nms_iou=0.4)
    assert isinstance(built, detectors.EngineDetector)
    assert seen[-1]["preprocess"] is PreprocessMode.LETTERBOX
    assert seen[-1]["conf_threshold"] == 0.3
    assert seen[-1]["nms_iou_threshold"] == 0.4
    detectors.build_detector("m", engine, ["a"], {}, conf=0.3, nms_iou=0.4, stretch=["m"])
    assert seen[-1]["preprocess"] is PreprocessMode.STRETCH
