"""Annotation extraction math and YOLO label file writers (no Blender imports).

The byte format of every file written here (label lines, ``data.yml``,
``label_index.txt``) is a frozen contract with the downstream dataset tooling in
``training/yolo/`` and ``training/deeplab/``.
"""

from pathlib import Path

import cv2
import numpy as np

from synthgen.constants import (
    ANNOTATION_MODE_KEYPOINTS_BBOX,
    ANNOTATION_MODE_SEGMENTATION_BBOX,
    BACKGROUND_CATEGORY_ID,
    KEYPOINT_DEPTH_TOLERANCE_M,
)

YoloAnnotation = tuple[
    int,
    tuple[float, float, float, float],
    list[tuple[float, float, int]],
]
"""(class_id, (cx, cy, w, h), [(kp_x, kp_y, vis), ...])"""

YoloSegAnnotation = tuple[int, list[tuple[float, float]]]
"""(class_id, [(x1, y1), (x2, y2), ...])"""


def normalize_annotation_mode(mode: str) -> str:
    """Validate and canonicalize the ``output.annotation_mode`` config value.

    Args:
        mode: Raw config string.

    Returns:
        The normalized mode.

    Raises:
        ValueError: If the mode is not a supported annotation mode.
    """
    mode_norm = mode.strip().lower()
    valid_modes = {
        ANNOTATION_MODE_KEYPOINTS_BBOX,
        ANNOTATION_MODE_SEGMENTATION_BBOX,
    }
    if mode_norm not in valid_modes:
        raise ValueError(
            f"Unsupported output.annotation_mode='{mode}'. Expected one of: {sorted(valid_modes)}"
        )
    return mode_norm


def check_keypoint_visibility(
    x_norm: float,
    y_norm: float,
    kp_depth: float,
    depth_map: np.ndarray,
    img_w: int,
    img_h: int,
    tolerance: float = KEYPOINT_DEPTH_TOLERANCE_M,
    ignore_occlusion: bool = False,
) -> int:
    """Determine keypoint visibility: 0=out-of-frame, 1=occluded, 2=visible.

    When *ignore_occlusion* is set, any in-frame keypoint is reported as visible
    (flag 2) without the depth test, so keypoints are annotated straight through
    occluding objects.

    Args:
        x_norm: Normalized image x of the projected keypoint.
        y_norm: Normalized image y (top-left origin).
        kp_depth: Camera-space depth of the keypoint.
        depth_map: Rendered depth map for the frame.
        img_w: Image width in pixels.
        img_h: Image height in pixels.
        tolerance: Depth agreement (meters) for the keypoint to be visible.
        ignore_occlusion: Skip the depth test entirely.

    Returns:
        YOLO visibility flag (0, 1, or 2).
    """
    if not (0 <= x_norm <= 1 and 0 <= y_norm <= 1):
        return 0

    if ignore_occlusion:
        return 2

    px = min(int(x_norm * img_w), img_w - 1)
    py = min(int(y_norm * img_h), img_h - 1)
    rendered_depth = depth_map[py, px]

    if abs(rendered_depth - kp_depth) < tolerance:
        return 2
    return 1


def bbox_from_category_segmap(
    seg_map: np.ndarray, category_id: int, img_w: int, img_h: int
) -> tuple[float, float, float, float] | None:
    """Compute a YOLO-format bounding box from the segmentation map.

    Args:
        seg_map: Category- or instance-id segmentation map.
        category_id: The id to bound.
        img_w: Image width in pixels.
        img_h: Image height in pixels.

    Returns:
        ``(cx, cy, w, h)`` normalized to [0, 1], or None if not visible.
    """
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
    return (cx, cy, w, h)


def segmentation_annotations_from_segmap(
    seg_map: np.ndarray,
    img_w: int,
    img_h: int,
    min_bbox_dim: int = 1,
) -> list[YoloSegAnnotation]:
    """Convert a category-id segmentation map into YOLO-seg annotations.

    Args:
        seg_map: Category-id segmentation map.
        img_w: Image width in pixels.
        img_h: Image height in pixels.
        min_bbox_dim: Contours whose bbox is smaller than this (either axis,
            pixels) are dropped.

    Returns:
        One ``(class_id, polygon)`` entry per kept contour.
    """
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
                # Preserve tiny visible objects (e.g. heavily occluded or edge-clipped)
                # by emitting a minimal rectangle polygon.
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
                if vis == 0:
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
    if annotation_mode == ANNOTATION_MODE_KEYPOINTS_BBOX:
        lines.extend(
            [
                "kpt_shape: [2, 3]",
                "flip_idx: [0, 1]",
            ]
        )
    filepath.write_text("\n".join(lines) + "\n", encoding="utf-8")
