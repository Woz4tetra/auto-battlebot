#!/usr/bin/env python3
"""Robot-lighting frames for the MassD arena, from hand labels instead of a pose model.

`playground/cage_scene/pick_robot_frames.py` runs a keypoint model over NHRL clips to find
frames where MRS BUFF is clean. This clip already has hand-drawn ground truth in
`training/data/nhrl_cage_high_eval/<clip>/`, so the frames come straight from there: pick the
labelled frames where MRS BUFF has both keypoints and no other robot overlaps its box, crop
the banners off the same way the target was cropped, and project the keypoints through the
fitted camera onto the floor plane. Position is the midpoint, heading is back to front.

The output is the layout `render_cage_view.py --robot-frames` and
`grade_render.py --robot-frames` read:

    <out>/robot_frames/<clip>_f<frame>/frame.png     banner-cropped real frame
    <out>/robot_frames/<clip>_f<frame>/pose.json     x_m, y_m, yaw_deg, box, keypoints
    <out>/robot_frames/<clip>_f<frame>/overlay.png   box and keypoints drawn on

Usage:
    venv/bin/python playground/massd_scene/robot_frames_from_labels.py runs/massd_scene \\
        training/data/nhrl_cage_high_eval/r1_beeroll_vs_mrsbuff
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from auto_battlebot.perception.cage_calibration import WORLD_FROM_HFIELD, load_cage_calibration

TARGET_CLASS = "mrs_buff_mk3"


def load_boxes(label_path: Path, width: int, height: int) -> list[dict[str, Any]]:
    """Every labelled robot in one frame, in pixels, with keypoints when both are marked."""
    out = []
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        class_id = int(parts[0])
        cx, cy, bw, bh = (float(v) for v in parts[1:5])
        box = [
            (cx - bw / 2) * width,
            (cy - bh / 2) * height,
            (cx + bw / 2) * width,
            (cy + bh / 2) * height,
        ]
        keypoints = None
        if len(parts) >= 11:
            values = [float(v) for v in parts[5:11]]
            if values[2] >= 2 and values[5] >= 2:
                keypoints = np.array(
                    [
                        [values[0] * width, values[1] * height],
                        [values[3] * width, values[4] * height],
                    ]
                )
        out.append({"class_id": class_id, "box": box, "keypoints": keypoints})
    return out


def boxes_overlap(a: list[float], b: list[float]) -> bool:
    return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])


def pixels_to_floor(
    points: np.ndarray, tf_camera_from_hfield: np.ndarray, k: np.ndarray
) -> np.ndarray:
    """Where image points land on the z = 0 plane of the hfield frame."""
    rays = (np.linalg.inv(k) @ np.c_[points, np.ones(len(points))].T).T
    tf_hfield_from_camera = np.linalg.inv(tf_camera_from_hfield)
    rotation, translation = tf_hfield_from_camera[:3, :3], tf_hfield_from_camera[:3, 3]
    directions = (rotation @ rays.T).T
    scale = -translation[2] / directions[:, 2]
    return translation[None, :] + directions * scale[:, None]


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument("eval_dir", type=Path, help="a nhrl_cage_high_eval dataset for this clip")
    parser.add_argument("--class-id", type=int, default=1, help="MRS BUFF's class id in the labels")
    args = parser.parse_args()

    target_dirs = sorted((args.scene_dir / "targets").iterdir())
    meta = json.loads((target_dirs[0] / "meta.json").read_text())
    clip = meta["clip"]
    top, bottom = meta["banner_crop"]
    full_width, full_height = meta["full_size"]
    pose_path = args.scene_dir / "poses" / f"{clip}.toml"
    calibration = load_cage_calibration(pose_path)
    k_data = json.loads((args.scene_dir / "camera_rect.json").read_text())
    k = np.array(
        [[k_data["fx"], 0.0, k_data["cx"]], [0.0, k_data["fy"], k_data["cy"]], [0.0, 0.0, 1.0]]
    )

    out_root = args.scene_dir / "robot_frames"
    out_root.mkdir(parents=True, exist_ok=True)
    kept = 0
    for label_path in sorted((args.eval_dir / "labels").iterdir()):
        image_path = args.eval_dir / "images" / f"{label_path.stem}.png"
        if not image_path.exists():
            continue
        detections = load_boxes(label_path, full_width, full_height)
        targets = [
            d for d in detections if d["class_id"] == args.class_id and d["keypoints"] is not None
        ]
        if len(targets) != 1:
            continue
        target = targets[0]
        others = [d for d in detections if d is not target]
        if any(boxes_overlap(target["box"], d["box"]) for d in others):
            continue
        # The banner crop moves every pixel up; keep anything the crop did not eat.
        box = [target["box"][0], target["box"][1] - top, target["box"][2], target["box"][3] - top]
        points = target["keypoints"] - np.array([0.0, top])
        if box[1] < 0 or box[3] > bottom - top or points[:, 1].min() < 0:
            continue

        floor_h = pixels_to_floor(points, calibration.tf_camera_from_fieldcenter, k)
        floor_w = (WORLD_FROM_HFIELD[:3, :3] @ floor_h.T).T
        front_w, back_w = floor_w[0], floor_w[1]
        centre = (front_w + back_w) / 2
        heading = front_w - back_w

        frame_index = int(label_path.stem)
        name = f"{clip}_f{frame_index:019d}"
        frame_dir = out_root / name
        frame_dir.mkdir(exist_ok=True)
        cv2.imwrite(str(frame_dir / "frame.png"), cv2.imread(str(image_path))[top:bottom])
        pose = {
            "clip": clip,
            "frame_index": frame_index,
            "pose_toml": str(pose_path),
            "robot": TARGET_CLASS,
            "x_m": float(centre[0]),
            "y_m": float(centre[1]),
            "yaw_deg": math.degrees(math.atan2(heading[1], heading[0])),
            "front_w": front_w.tolist(),
            "back_w": back_w.tolist(),
            "box_rect": [float(v) for v in box],
            "keypoints_rect": points.tolist(),
            "source": "hand labels",
        }
        (frame_dir / "pose.json").write_text(json.dumps(pose, indent=2) + "\n")
        overlay = cv2.imread(str(frame_dir / "frame.png"))
        cv2.rectangle(
            overlay, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 255), 2
        )
        for point, color in zip(points, [(0, 255, 0), (255, 0, 0)]):
            cv2.circle(overlay, (int(point[0]), int(point[1])), 8, color, -1)
        cv2.imwrite(str(frame_dir / "overlay.png"), overlay)
        kept += 1
        print(f"{name}: ({centre[0]:+.2f}, {centre[1]:+.2f}) m, yaw {pose['yaw_deg']:+.0f} deg")
    print(f"{kept} frames -> {out_root}")


if __name__ == "__main__":
    main()
