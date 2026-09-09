"""Detector variants and the inference loop that turns GT images into scored frames.

All three detectors are duck-typed on detect() and describe(): a TensorRT engine, the
same engine run over a field crop, and a replay of predictions computed earlier.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.eval.dataset import Frame, GtFrame, Taxonomy
from auto_battlebot.perception.trt_yolo import TrtYoloModel


class EngineDetector:
    """A TensorRT engine, run per image."""

    def __init__(self, model: TrtYoloModel) -> None:
        self._model = model

    def describe(self) -> str:
        return str(self._model.describe())

    def detect(self, image: np.ndarray, _stamp_ns: int) -> list:
        return self._model.infer(image)


class FieldCropDetector:
    """Crops each frame to its field box before inference, then maps detections back.

    Arm E of `input_geometry_2026-09-05` trains on field-cropped images, so it has to be
    evaluated on field-cropped frames. Cropping here rather than pre-cropping the eval set
    leaves the GT in full-frame coordinates, so every candidate is scored against the same
    boxes and the paired bootstrap stays paired.

    Boxes come from a JSON keyed by frame stem, as written by
    `training/deeplab/build_field_crop_dataset.py masks --recursive`. A frame with no field
    detected passes through uncropped, matching what the dataset builder does.
    """

    def __init__(
        self,
        inner: EngineDetector,
        boxes_path: Path,
        images: dict[int, Path],
        margin: float,
    ) -> None:
        self._inner = inner
        self._margin = margin
        payload = json.loads(boxes_path.read_text())
        self._by_stamp: dict[int, tuple[float, float, float, float]] = {}
        for stamp, path in images.items():
            box = payload.get(path.stem)
            if box:
                self._by_stamp[stamp] = tuple(box)
        missing = len(images) - len(self._by_stamp)
        if missing:
            print(f"  {missing} of {len(images)} frames have no field box; passed through whole")

    def describe(self) -> str:
        return f"{self._inner.describe()}, field crop (margin {self._margin:.2f})"

    def _crop(self, image: np.ndarray, stamp_ns: int) -> tuple[int, int, int, int] | None:
        box = self._by_stamp.get(stamp_ns)
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

    def detect(self, image: np.ndarray, stamp_ns: int) -> list:
        crop = self._crop(image, stamp_ns)
        if crop is None:
            return self._inner.detect(image, stamp_ns)
        px0, py0, px1, py1 = crop
        detections = self._inner.detect(image[py0:py1, px0:px1], stamp_ns)
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
    """Detections read from a JSON file instead of produced by an engine.

    Some detectors cannot be handed one image at a time. Background subtraction needs the
    whole recording to build its background, so it is run separately by
    `background_subtraction_predict.py` and its output is replayed here, which keeps every
    metric, threshold and plot identical to an engine's.

    Format: {"labels": [...], "frames": {"<stamp_ns>": [{"xyxy": [x1, y1, x2, y2],
    "score": float, "class_id": int, "kps": [[x, y, v], ...]}]}}.
    """

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

    def detect(self, _image: np.ndarray, stamp_ns: int) -> list:
        return self._by_stamp.get(stamp_ns, [])


# Every detector is duck-typed on detect() and describe(); this keeps the two signatures
# that pass one around from drifting apart as variants are added.
Detector = EngineDetector | PrecomputedDetector | FieldCropDetector


def infer_frames(
    gt_frames: dict[int, GtFrame],
    images: dict[int, Path],
    detector: Detector,
    class_labels: list[str],
    taxonomy: Taxonomy,
) -> list[Frame]:
    """Run the candidate on every GT frame's image and pair the results."""
    frames = []
    for gt_stamp, (gt_boxes, gt_labels, gt_keypoints) in gt_frames.items():
        image = cv2.imread(str(images[gt_stamp]))
        if image is None:
            raise SystemExit(f"Failed to read image {images[gt_stamp]}")
        detections = detector.detect(image, gt_stamp)
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
    images: dict[int, Path],
    args: argparse.Namespace,
) -> Detector:
    """The detector for one candidate, with any preprocessing that candidate was trained on.

    Geometry arms differ in how the frame reaches the tensor, and an engine fed the wrong
    preprocessing fails quietly rather than loudly: a stretch-trained engine handed a
    letterboxed frame simply sees the wrong aspect ratio. Both modes are opt-in per
    candidate so a mixed run scores each arm the way it was trained.
    """
    if not engine_path.exists():
        raise SystemExit(f"Candidate not found: {engine_path}")
    if engine_path.suffix == ".json":
        return PrecomputedDetector(engine_path, args.conf)

    inner = EngineDetector(
        TrtYoloModel(
            str(engine_path),
            conf_threshold=args.conf,
            nms_iou_threshold=args.nms_iou,
            num_classes=len(class_labels),
            preprocess="stretch" if name in set(args.stretch or []) else "letterbox",
        )
    )
    boxes = parse_candidates(args.field_boxes or []).get(name)
    if boxes is None:
        return inner
    return FieldCropDetector(inner, boxes, images, args.crop_margin)
