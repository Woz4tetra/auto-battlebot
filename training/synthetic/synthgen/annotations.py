"""Annotation extraction math and YOLO label file writers (no Blender imports)."""

from enum import IntEnum
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np

from synthgen.constants import (
    BACKGROUND_CATEGORY_ID,
    KEYPOINT_DEPTH_TOLERANCE_M,
    AnnotationMode,
)


class KeypointVisibility(IntEnum):
    OUT_OF_FRAME = 0
    OCCLUDED = 1
    VISIBLE = 2


class YoloBox(NamedTuple):
    cx: float
    cy: float
    w: float
    h: float


YoloAnnotation = tuple[
    int,
    YoloBox,
    list[tuple[float, float, int]],
]
"""(class_id, (cx, cy, w, h), [(kp_x, kp_y, vis), ...])"""

YoloSegAnnotation = tuple[int, list[tuple[float, float]]]
"""(class_id, [(x1, y1), (x2, y2), ...])"""


def normalize_annotation_mode(mode: str) -> AnnotationMode:
    """Parse the ``output.annotation_mode`` config value, ignoring case and padding."""
    try:
        return AnnotationMode(mode.strip().lower())
    except ValueError:
        raise ValueError(
            f"Unsupported output.annotation_mode='{mode}'. "
            f"Expected one of: {sorted(m.value for m in AnnotationMode)}"
        ) from None


def check_keypoint_visibility(
    x_norm: float,
    y_norm: float,
    kp_depth: float,
    depth_map: np.ndarray,
    img_w: int,
    img_h: int,
    tolerance: float = KEYPOINT_DEPTH_TOLERANCE_M,
    ignore_occlusion: bool = False,
) -> KeypointVisibility:
    """Visibility flag of a projected keypoint; *ignore_occlusion* skips the depth test."""
    if not (0 <= x_norm <= 1 and 0 <= y_norm <= 1):
        return KeypointVisibility.OUT_OF_FRAME

    if ignore_occlusion:
        return KeypointVisibility.VISIBLE

    px = min(int(x_norm * img_w), img_w - 1)
    py = min(int(y_norm * img_h), img_h - 1)
    rendered_depth = depth_map[py, px]

    if abs(rendered_depth - kp_depth) < tolerance:
        return KeypointVisibility.VISIBLE
    return KeypointVisibility.OCCLUDED


def bbox_from_category_segmap(
    seg_map: np.ndarray, category_id: int, img_w: int, img_h: int
) -> YoloBox | None:
    """Normalized box around *category_id*, or None when it is not in the map."""
    mask = seg_map.squeeze() == category_id
    if not np.any(mask):
        return None

    ys, xs = np.where(mask)
    x_min, x_max = int(xs.min()), int(xs.max())
    y_min, y_max = int(ys.min()), int(ys.max())

    cx = (x_min + x_max) / 2.0 / img_w
    cy = (y_min + y_max) / 2.0 / img_h
    w = (x_max - x_min) / img_w
    h = (y_max - y_min) / img_h
    return YoloBox(cx, cy, w, h)


def segmentation_annotations_from_segmap(
    seg_map: np.ndarray,
    img_w: int,
    img_h: int,
    min_bbox_dim: int = 1,
) -> list[YoloSegAnnotation]:
    """Convert a category-id segmentation map into YOLO-seg annotations."""
    seg_2d = seg_map.squeeze()
    annotations: list[YoloSegAnnotation] = []
    for class_id in sorted(int(v) for v in np.unique(seg_2d).tolist()):
        if class_id <= BACKGROUND_CATEGORY_ID:
            continue

        mask = (seg_2d == class_id).astype(np.uint8)
        if not np.any(mask):
            continue

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            xs = contour[:, 0, 0]
            ys = contour[:, 0, 1]
            bbox_w = int(xs.max() - xs.min() + 1)
            bbox_h = int(ys.max() - ys.min() + 1)
            if bbox_w < min_bbox_dim or bbox_h < min_bbox_dim:
                continue

            if contour.shape[0] >= 3:
                polygon = [(float(x) / img_w, float(y) / img_h) for x, y in contour[:, 0, :]]
            else:
                x_min = int(xs.min())
                x_max = int(xs.max())
                y_min = int(ys.min())
                y_max = int(ys.max())
                polygon = [
                    (float(x_min) / img_w, float(y_min) / img_h),
                    (float(x_max) / img_w, float(y_min) / img_h),
                    (float(x_max) / img_w, float(y_max) / img_h),
                    (float(x_min) / img_w, float(y_max) / img_h),
                ]
            annotations.append((class_id, polygon))
    return annotations


def write_yolo_labels(
    filepath: Path,
    annotations: list[YoloAnnotation],
) -> None:
    """Write one YOLO keypoint annotation line per detected object."""
    with open(filepath, "w") as f:
        for class_id, bbox, keypoints in annotations:
            cx, cy, w, h = bbox
            parts = [f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"]
            for kp_x, kp_y, vis in keypoints:
                if vis == KeypointVisibility.OUT_OF_FRAME:
                    parts.append("0.000000 0.000000 0")
                else:
                    parts.append(f"{kp_x:.6f} {kp_y:.6f} {vis}")
            f.write(" ".join(parts) + "\n")


def write_yolo_seg_labels(filepath: Path, annotations: list[YoloSegAnnotation]) -> None:
    """Write YOLO segmentation labels (class + polygon points)."""
    with open(filepath, "w") as f:
        for class_id, polygon in annotations:
            parts = [str(class_id)]
            for x, y in polygon:
                parts.append(f"{x:.6f}")
                parts.append(f"{y:.6f}")
            f.write(" ".join(parts) + "\n")


def write_label_index(filepath: Path, labels: dict[int, str]) -> None:
    """Write a human-readable class-id map for segmentation mode."""
    with open(filepath, "w") as f:
        for class_id, name in sorted(labels.items()):
            f.write(f"{class_id}:{name}\n")


def build_names_list(labels: dict[int, str]) -> list[str]:
    """Build a dense names list indexed by class id."""
    if not labels:
        return []
    max_id = max(int(class_id) for class_id in labels)
    names: list[str] = []
    for class_id in range(max_id + 1):
        names.append(labels.get(class_id, f"unknown_{class_id}"))
    return names


def write_data_yml(
    filepath: Path, dataset_root: Path, names: list[str], annotation_mode: str
) -> None:
    """Write YOLO dataset metadata as data.yml."""
    lines = [
        f"path: {dataset_root.resolve()}",
        "train: images",
        "val: images",
        f"nc: {len(names)}",
        f"names: {names}",
    ]
    if annotation_mode == AnnotationMode.KEYPOINTS_BBOX:
        lines.extend(
            [
                "kpt_shape: [2, 3]",
                "flip_idx: [0, 1]",
            ]
        )
    filepath.write_text("\n".join(lines) + "\n", encoding="utf-8")
