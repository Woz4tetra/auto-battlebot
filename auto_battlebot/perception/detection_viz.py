"""Annotate frames with TrtYoloModel detections, shared by the image and video CLIs."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml

from auto_battlebot.perception.trt_yolo import DetectionTuple

BOX_COLORS = [
    (0, 0, 255),
    (0, 255, 0),
    (255, 0, 0),
    (255, 255, 0),
    (255, 0, 255),
    (0, 255, 255),
]


def load_class_names(spec: str) -> list[str] | None:
    """Resolve class names from a data.yaml/yml path or a comma-separated list.

    Used to label boxes with real class names (e.g. per-robot names) instead of class_N.
    """
    if not spec:
        return None
    path = Path(spec)
    if path.exists():
        meta = yaml.safe_load(path.read_text())
        raw = meta.get("names", []) if isinstance(meta, dict) else []
        if isinstance(raw, dict):
            raw = [raw[k] for k in sorted(raw, key=int)]
        return [str(v) for v in raw]
    return [s.strip() for s in spec.split(",")]


def draw_detections(
    frame: np.ndarray,
    detections: list[DetectionTuple],
    class_names: list[str] | None,
    kp_conf_threshold: float = 0.5,
) -> np.ndarray:
    """Draw boxes, labels, and keypoints on frame."""
    out: np.ndarray = frame.copy()
    for xyxy, conf, cls_id, kps in detections:
        x1, y1, x2, y2 = map(int, xyxy)
        color = BOX_COLORS[cls_id % len(BOX_COLORS)]
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        conf_display = min(1.0, max(0.0, conf))
        label = (
            class_names[cls_id] if class_names and cls_id < len(class_names) else f"class_{cls_id}"
        ) + f" {conf_display:.2f}"
        cv2.putText(out, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        for j in range(kps.shape[0]):
            x, y, kp_conf = kps[j]
            if kp_conf >= kp_conf_threshold:
                cx, cy = int(round(x)), int(round(y))
                cv2.circle(out, (cx, cy), 4, (255, 255, 255), -1)
                cv2.circle(out, (cx, cy), 3, color, -1)
    return out
