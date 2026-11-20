"""Detector variants and the inference loop over GT images."""

from __future__ import annotations

import json
from collections.abc import Sequence
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

from auto_battlebot.eval.dataset import Frame, FrameKey, GtFrame, Taxonomy
from auto_battlebot.perception.trt_yolo import PreprocessMode, TrtYoloModel


class EngineDetector:
    """A TensorRT engine, run per image."""

    def __init__(self, model: TrtYoloModel) -> None:
        self._model = model

    def describe(self) -> str:
        return str(self._model.describe())

    def detect(self, image: np.ndarray, _key: FrameKey) -> list:
        return self._model.infer(image)


class FieldCropDetector:
    """Crops each frame to its field box before inference, then maps detections back."""

    def __init__(
        self,
        inner: EngineDetector,
        boxes_path: Path,
        images: dict[FrameKey, Path],
        margin: float,
    ) -> None:
        self._inner = inner
        self._margin = margin
        payload = json.loads(boxes_path.read_text())
        self._by_frame: dict[FrameKey, tuple[float, float, float, float]] = {}
        for key, path in images.items():
            box = payload.get(path.stem)
            if box:
                self._by_frame[key] = tuple(box)
        missing = len(images) - len(self._by_frame)
        if missing:
            print(f"  {missing} of {len(images)} frames have no field box; passed through whole")

    def describe(self) -> str:
        return f"{self._inner.describe()}, field crop (margin {self._margin:.2f})"

    def _crop(self, image: np.ndarray, key: FrameKey) -> tuple[int, int, int, int] | None:
        box = self._by_frame.get(key)
        if box is None:
            return None
        height, width = image.shape[:2]
        x0, y0, x1, y1 = box
        mx, my = (x1 - x0) * self._margin, (y1 - y0) * self._margin
        px0 = int(round(max(x0 - mx, 0.0) * width))
        py0 = int(round(max(y0 - my, 0.0) * height))
        px1 = max(int(round(min(x1 + mx, 1.0) * width)), px0 + 1)
        py1 = max(int(round(min(y1 + my, 1.0) * height)), py0 + 1)
        return px0, py0, px1, py1

    def detect(self, image: np.ndarray, key: FrameKey) -> list:
        crop = self._crop(image, key)
        if crop is None:
            return self._inner.detect(image, key)
        px0, py0, px1, py1 = crop
        detections = self._inner.detect(image[py0:py1, px0:px1], key)
        shifted = []
        for xyxy, conf, cls_id, kps in detections:
            xyxy = np.asarray(xyxy, dtype=np.float64).copy()
            kps = np.asarray(kps, dtype=np.float64).copy()
            xyxy[0] += px0
            xyxy[2] += px0
            xyxy[1] += py0
            xyxy[3] += py0
            if kps.size:
                kps[:, 0] += px0
                kps[:, 1] += py0
            shifted.append((xyxy, conf, cls_id, kps))
        return shifted


class PrecomputedDetector:
    """Detections read from a JSON file instead of produced by an engine."""

    def __init__(self, path: Path, conf_threshold: float) -> None:
        payload = json.loads(path.read_text())
        self.labels: list[str] = list(payload.get("labels", []))
        self._conf = conf_threshold
        self._by_stamp: dict[int, list] = {}
        for stamp, rows in payload.get("frames", {}).items():
            self._by_stamp[int(stamp)] = [
                (
                    np.asarray(row["xyxy"], dtype=np.float64),
                    float(row.get("score", 1.0)),
                    int(row.get("class_id", 0)),
                    row.get("kps", []),
                )
                for row in rows
                if float(row.get("score", 1.0)) >= conf_threshold
            ]
        self._path = path

    def describe(self) -> str:
        total = sum(len(rows) for rows in self._by_stamp.values())
        return (
            f"precomputed {self._path.name}: {len(self._by_stamp)} frames, "
            f"{total} detections at conf >= {self._conf}"
        )

    def detect(self, _image: np.ndarray, key: FrameKey) -> list:
        if key.dataset != ".":
            raise ValueError(
                f"precomputed detections are keyed by stamp alone, so {key.dataset} cannot be "
                "replayed against a multi-recording root"
            )
        return self._by_stamp.get(key.stamp_ns, [])


class Detector(Protocol):
    def describe(self) -> str: ...

    def detect(self, image: np.ndarray, key: FrameKey, /) -> list: ...


def infer_frames(
    gt_frames: dict[FrameKey, GtFrame],
    images: dict[FrameKey, Path],
    detector: Detector,
    class_labels: list[str],
    taxonomy: Taxonomy,
) -> list[Frame]:
    """Run the candidate on every GT frame's image and pair the results."""
    frames = []
    for key, (gt_boxes, gt_labels, gt_keypoints) in gt_frames.items():
        image = cv2.imread(str(images[key]))
        if image is None:
            raise SystemExit(f"Failed to read image {images[key]}")
        detections = detector.detect(image, key)
        labeled = [
            (xyxy, conf, class_labels[cls_id], kps)
            for xyxy, conf, cls_id, kps in detections
            if cls_id < len(class_labels)
        ]
        keep = [d for d in labeled if d[2] not in taxonomy.exclude]
        gt_keep = [i for i, lbl in enumerate(gt_labels) if lbl not in taxonomy.exclude]
        frames.append(
            Frame(
                gt_boxes=gt_boxes[gt_keep],
                gt_labels=[gt_labels[i] for i in gt_keep],
                gt_keypoints=[gt_keypoints[i] for i in gt_keep],
                pred_boxes=np.asarray([d[0] for d in keep], dtype=np.float64).reshape(-1, 4),
                pred_labels=[d[2] for d in keep],
                pred_scores=np.asarray([d[1] for d in keep], dtype=np.float64),
                pred_keypoints=[np.asarray(d[3], dtype=np.float64).reshape(-1, 3) for d in keep],
            )
        )
    return frames


def parse_candidates(entries: list[str]) -> dict[str, Path]:
    candidates = {}
    for entry in entries:
        if "=" in entry:
            name, _, path = entry.partition("=")
        else:
            name, path = Path(entry).stem, entry
        candidates[name] = Path(path)
    return candidates


def build_detector(
    name: str,
    engine_path: Path,
    class_labels: list[str],
    images: dict[FrameKey, Path],
    *,
    conf: float,
    nms_iou: float,
    stretch: Sequence[str] = (),
    field_boxes: Sequence[str] = (),
    crop_margin: float = 0.0,
) -> Detector:
    """The detector for one candidate, with any preprocessing that candidate was trained on."""
    if not engine_path.exists():
        raise SystemExit(f"Candidate not found: {engine_path}")
    if engine_path.suffix == ".json":
        return PrecomputedDetector(engine_path, conf)

    inner = EngineDetector(
        TrtYoloModel(
            str(engine_path),
            conf_threshold=conf,
            nms_iou_threshold=nms_iou,
            num_classes=len(class_labels),
            preprocess=(
                PreprocessMode.STRETCH if name in set(stretch) else PreprocessMode.LETTERBOX
            ),
        )
    )
    boxes = parse_candidates(list(field_boxes)).get(name)
    if boxes is None:
        return inner
    return FieldCropDetector(inner, boxes, images, crop_margin)
