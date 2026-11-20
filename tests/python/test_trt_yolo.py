from __future__ import annotations

import numpy as np
import pytest

from auto_battlebot.perception import trt_yolo
from auto_battlebot.perception.trt_yolo import (
    CPP_LETTERBOX_PADDING,
    PreprocessMode,
    letterbox,
    non_max_suppression,
    parse_end2end_detections,
    preprocess_frame,
    resolve_model_layout,
    scale_detections_to_frame,
)

PAD_VALUE = 114
FRAME_H = 100
FRAME_W = 200
ODD_PAD_TARGET_H = 123


def test_letterbox_padding_split_matches_cpp() -> None:
    image = np.full((FRAME_H, FRAME_W, 3), 255, dtype=np.uint8)
    out, scale, pad_left, pad_top = letterbox(
        image, ODD_PAD_TARGET_H, FRAME_W, padding=CPP_LETTERBOX_PADDING
    )
    assert scale == 1.0
    assert (pad_left, pad_top) == (0.0, 11.0)
    assert out[10, 0, 0] == PAD_VALUE
    assert out[11, 0, 0] == 255
    assert out[110, 0, 0] == 255
    assert out[111, 0, 0] == PAD_VALUE


def test_preprocess_modes_differ_in_scale_axes() -> None:
    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    boxed = preprocess_frame(frame, 160, 160, CPP_LETTERBOX_PADDING, PreprocessMode.LETTERBOX)
    stretched = preprocess_frame(frame, 160, 160, CPP_LETTERBOX_PADDING, PreprocessMode.STRETCH)
    assert boxed.scale_x == boxed.scale_y == 0.8
    assert (stretched.scale_x, stretched.scale_y) == (0.8, 1.6)
    assert (stretched.pad_left, stretched.pad_top) == (0.0, 0.0)
    assert boxed.blob.shape == stretched.blob.shape == (1, 3, 160, 160)


def test_unknown_preprocess_mode_is_refused() -> None:
    with pytest.raises(ValueError, match="not a valid PreprocessMode"):
        PreprocessMode("squash")


def _end2end_rows(num_keypoints: int, max_det: int) -> tuple[np.ndarray, int]:
    num_features = trt_yolo.E2E_FIXED_COLUMNS + 3 * num_keypoints
    rows = np.zeros((max_det, num_features), dtype=np.float32)
    rows[0, :6] = [10, 20, 110, 120, 0.9, 2.0]
    rows[0, 6:] = [30, 40, 0.8, 50, 60, 0.7]
    rows[1, :6] = [1, 2, 3, 4, 0.1, 1.0]
    return rows, num_features


def test_parse_end2end_row_layout() -> None:
    rows, num_features = _end2end_rows(num_keypoints=2, max_det=5)
    for prediction in (rows, rows.T):
        found = parse_end2end_detections(prediction, 5, num_features, conf_thres=0.5)
        assert len(found) == 1
        xyxy, conf, cls_id, kps = found[0]
        assert list(xyxy) == [10, 20, 110, 120]
        assert conf == pytest.approx(0.9)
        assert cls_id == 2
        assert kps.shape == (2, 3)
        assert list(kps[1]) == pytest.approx([50, 60, 0.7])


def test_end2end_coordinates_are_input_pixels_mapped_back_through_pad_and_scale() -> None:
    rows, num_features = _end2end_rows(num_keypoints=2, max_det=5)
    found = parse_end2end_detections(rows, 5, num_features, conf_thres=0.5)
    mapped = scale_detections_to_frame(found, 200, 400, 0.5, 10.0, 20.0)
    xyxy, _, _, kps = mapped[0]
    assert list(xyxy) == [0.0, 0.0, 200.0, 200.0]
    assert list(kps[0, :2]) == [40.0, 40.0]


def test_output_layout_detects_end2end_vs_raw() -> None:
    end2end = resolve_model_layout(300, 12)
    assert end2end.is_end2end
    assert (end2end.num_predictions, end2end.num_features) == (300, 12)
    assert end2end.num_classes is None
    assert end2end.num_keypoints == 2
    raw = resolve_model_layout(12, 8400)
    assert not raw.is_end2end
    assert (raw.num_predictions, raw.num_features) == (8400, 12)
    assert raw.num_classes == 1


def test_infer_layout_seg_columns_not_keypoints() -> None:
    mask_coefficients = 32
    num_features = 4 + 1 + mask_coefficients
    seg = resolve_model_layout(num_features, 8400, num_classes=1)
    assert (seg.num_classes, seg.num_keypoints) == (1, 0)
    pose = resolve_model_layout(4 + 1 + 3 * 4, 8400)
    assert (pose.num_classes, pose.num_keypoints) == (1, 4)
    forced = resolve_model_layout(num_features, 8400, num_classes=1, num_keypoints=2)
    assert forced.num_keypoints == 2


def _raw_predictions(extra_columns: int) -> np.ndarray:
    num_classes, num_keypoints = 2, 1
    num_features = 4 + num_classes + 3 * num_keypoints + extra_columns
    pred = np.zeros((3, num_features), dtype=np.float32)
    pred[0, :4] = [50, 50, 20, 20]
    pred[0, 4:6] = [0.9, 0.1]
    pred[0, 6:9] = [55, 56, 0.8]
    pred[1, :4] = [150, 150, 20, 20]
    pred[1, 4:6] = [0.2, 0.7]
    pred[1, 6:9] = [155, 156, 0.6]
    pred[2, :4] = [250, 250, 20, 20]
    pred[2, 4:6] = [0.1, 0.1]
    pred[2, 9:] = 7.0
    return pred


def _run_nms(prediction: np.ndarray, num_features: int) -> list[trt_yolo.DetectionTuple]:
    return non_max_suppression(
        prediction,
        num_features=num_features,
        num_predictions=3,
        num_classes=2,
        num_keypoints=1,
        conf_thres=0.5,
        iou_thres=0.5,
    )


def test_nms_accepts_either_orientation_and_ignores_extra_columns() -> None:
    plain = _raw_predictions(extra_columns=0)
    with_masks = _raw_predictions(extra_columns=4)
    baseline = _run_nms(plain, 9)
    assert [d[2] for d in baseline] == [0, 1]
    for prediction, features in ((plain.T, 9), (with_masks, 13), (with_masks.T, 13)):
        found = _run_nms(prediction, features)
        assert [d[2] for d in found] == [0, 1]
        for a, b in zip(found, baseline):
            np.testing.assert_allclose(a[0], b[0])
            np.testing.assert_allclose(a[3], b[3])
    assert list(baseline[0][0]) == [40, 40, 60, 60]


def _logit(p: np.ndarray) -> np.ndarray:
    return np.log(p + 1e-6) - np.log(1 - p + 1e-6)


def test_nms_applies_sigmoid_to_raw_logits() -> None:
    pred = _raw_predictions(extra_columns=0)
    pred[:, 4:6] = _logit(pred[:, 4:6])
    pred[0, 8] = 5.0
    found = _run_nms(pred, 9)
    assert [d[2] for d in found] == [0, 1]
    assert found[0][1] == pytest.approx(0.9, abs=1e-3)
    assert found[0][3][0, 2] == pytest.approx(1 / (1 + np.exp(-5.0)), abs=1e-4)


def test_engine_host_code_allowed_on_property_and_setter_runtimes() -> None:
    from auto_battlebot.perception import trt_yolo

    class PropertyRuntime:
        engine_host_code_allowed = False

    class SetterRuntime:
        def __init__(self) -> None:
            self.calls: list[bool] = []

        def set_engine_host_code_allowed(self, value: bool) -> None:
            self.calls.append(value)

    prop = PropertyRuntime()
    trt_yolo._allow_engine_host_code(prop)
    assert prop.engine_host_code_allowed is True
    setter = SetterRuntime()
    trt_yolo._allow_engine_host_code(setter)
    assert setter.calls == [True]
