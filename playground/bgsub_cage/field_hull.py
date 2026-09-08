"""Cage floor region for a fixed camera: DeepLab floor mask plus a convex hull.

Lifted out of ``training/model_eval/rembg_field.py``, which holds the same two
pieces but no longer imports: it depends on ``auto_battlebot.background_subtraction``
and ``auto_battlebot.floor_background``, both deleted in c4fed999.

The hull matters because a robot standing on the floor punches a hole in the raw
mask exactly where the robot is, and masking detections by that mask would delete
the things being counted. The true field is a square seen in perspective, so it is
convex, and the hull can only ever recover floor the mask lost.

One hull per clip, not per frame. The camera does not move, so the mask is built
from a per-pixel median over sampled frames: robots are absent from any given pixel
in most samples, so the median has no robots on it and the hull has no holes to
repair.

The DeepLab loader lives in ``auto_battlebot.segmentation``, so this imports normally.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

DEEPLAB_CHECKPOINT = Path("data/models/field_deeplabv3p_r50_2026-07-29.pth")
FLOOR_CLASS = 1
MEDIAN_SAMPLES = 60


class FieldSegmenter:
    """The trained DeepLab floor model at its own input size."""

    def __init__(self, checkpoint: Path = DEEPLAB_CHECKPOINT) -> None:
        try:
            import torch

            from auto_battlebot.segmentation.load_deeplabv3 import common_transforms, load_model
        except ImportError as error:  # pragma: no cover - environment problem, not logic
            raise ImportError(
                "DeepLab helpers not importable. Install the project with "
                f"`pip install -e .` so auto_battlebot resolves. Original error: {error}"
            ) from error

        self._torch = torch
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._model, self.config = load_model(checkpoint, self._device)
        self._transform = common_transforms(pad_size=self.config.pad_size)

    def raw_mask(self, bgr: np.ndarray) -> np.ndarray:
        """uint8 0/255 floor mask at the frame's own size."""
        size = self.config.image_size
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (size, size), interpolation=cv2.INTER_LINEAR)
        tensor = self._transform(resized).unsqueeze(0).to(self._device)

        with self._torch.no_grad():
            output = self._model(tensor)
        prediction = output.argmax(dim=1).squeeze(0).cpu().numpy()

        pad = self.config.pad_size
        if pad > 0:
            prediction = prediction[pad:-pad, pad:-pad]

        mask = (prediction == FLOOR_CLASS).astype(np.uint8) * 255
        height, width = bgr.shape[:2]
        return cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)


def largest_component_hull(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray | None]:
    """Largest blob in a 0/255 mask, replaced by its convex hull.

    Returns the filled hull mask and its polygon, or (zeros, None) when the mask is
    empty. The polygon is what gets cached and drawn; the mask is what gates boxes.
    """
    binary = (mask > 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 1:
        return np.zeros_like(mask), None

    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    points = cv2.findNonZero((labels == largest).astype(np.uint8))
    if points is None:
        return np.zeros_like(mask), None

    polygon = cv2.convexHull(points)
    hull = np.zeros_like(mask)
    cv2.fillConvexPoly(hull, polygon, 255)
    return hull, polygon


def median_frame(video_path: Path, samples: int = MEDIAN_SAMPLES) -> np.ndarray:
    """Per-pixel median over frames spread evenly across the clip.

    Robots move, so any given pixel shows bare floor in most samples and the median
    is the empty cage. This is also the background model the report's first candidate
    method is built on.
    """
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {video_path}")

    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if total <= 0:
        raise RuntimeError(f"{video_path} reports no frames")

    indices = np.linspace(0, total - 1, min(samples, total), dtype=int)
    frames: list[np.ndarray] = []
    wanted = set(int(index) for index in indices)
    position = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if position in wanted:
            frames.append(frame)
        position += 1
    capture.release()

    if not frames:
        raise RuntimeError(f"No frames read from {video_path}")
    return np.median(np.stack(frames), axis=0).astype(np.uint8)


def compute_hull(
    video_path: Path, segmenter: FieldSegmenter, samples: int = MEDIAN_SAMPLES
) -> dict[str, Any]:
    """Median frame, DeepLab floor mask, convex hull. One call per clip."""
    background = median_frame(video_path, samples)
    mask = segmenter.raw_mask(background)
    hull, polygon = largest_component_hull(mask)

    height, width = background.shape[:2]
    frame_area = float(height * width)
    return {
        "video": video_path.name,
        "width": width,
        "height": height,
        "samples": samples,
        "polygon": [] if polygon is None else polygon.reshape(-1, 2).tolist(),
        "raw_mask_fraction": float((mask > 0).sum()) / frame_area,
        "hull_fraction": float((hull > 0).sum()) / frame_area,
        "_hull": hull,
        "_background": background,
    }


def hull_mask_from_polygon(polygon: list[list[int]], shape: tuple[int, int]) -> np.ndarray:
    mask = np.zeros(shape, dtype=np.uint8)
    if polygon:
        cv2.fillConvexPoly(mask, np.array(polygon, dtype=np.int32), 255)
    return mask


def save_hull(record: dict[str, Any], destination: Path) -> None:
    payload = {key: value for key, value in record.items() if not key.startswith("_")}
    destination.write_text(json.dumps(payload, indent=2) + "\n")


def load_hull(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())
