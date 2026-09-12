#!/usr/bin/env python3
"""Real frames with MRS BUFF MK3 on the mat, with its pose recovered for re-rendering.

Walks each Cage 2 clip that has a fitted camera pose, runs the yolo26x pose model on every
--stride-th frame, and keeps frames where exactly one `mrs_buff_mk3` is detected with both
keypoints confident, clear of every other robot, and inside the mat hull. Each kept frame is
rectified like the targets and its keypoints are undistorted and projected through the clip's
camera onto the mat plane, which gives the robot's W-frame position and heading.

    <out>/robot_frames/<clip>_f<frame>/frame.png      rectified real frame
    <out>/robot_frames/<clip>_f<frame>/pose.json      x_m, y_m, yaw_deg, box, keypoints, pose toml
    <out>/robot_frames/<clip>_f<frame>/overlay.png    box, keypoints and projected heading

Usage:
    venv/bin/python playground/cage_scene/pick_robot_frames.py runs/cage_scene \\
        data/downloads/mrsbuff_may26 --per-clip 3 --stride 90
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from auto_battlebot.perception.cage_calibration import WORLD_FROM_HFIELD, load_cage_calibration
from auto_battlebot.perception.camera_calibration import (
    load_camera_calibration,
    rectify_maps,
    undistort_points,
)
from auto_battlebot.perception.field_pose import pixels_to_field_plane

DEFAULT_MODEL = Path("data/eval_models/yolo26x-pose_all_robot_keypoints_2026-09-05.pt")
DEFAULT_CALIBRATION = Path("config/cameras/brettzone_cage_high.toml")
TARGET_CLASS = "mrs_buff_mk3"
MIN_KEYPOINT_CONF = 0.5
MIN_BOX_CONF = 0.4
MIN_SEPARATION_PX = 120.0
HULL_MARGIN_PX = 40


def load_statuses(scene_dir: Path) -> dict[str, str]:
    summary = scene_dir / "poses" / "summary.csv"
    with summary.open() as handle:
        return {row["clip"]: row["status"] for row in csv.DictReader(handle)}


def box_gap(a: np.ndarray, b: np.ndarray) -> float:
    dx = max(0.0, max(a[0], b[0]) - min(a[2], b[2]))
    dy = max(0.0, max(a[1], b[1]) - min(a[3], b[3]))
    return math.hypot(dx, dy)


def pick_detection(result: Any, names: dict[int, str]) -> dict[str, Any] | None:
    """The one confident MRS BUFF box with both keypoints, clear of every other box."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0 or result.keypoints is None:
        return None
    xyxy = boxes.xyxy.cpu().numpy()
    conf = boxes.conf.cpu().numpy()
    cls = boxes.cls.cpu().numpy().astype(int)
    kpts = result.keypoints.xy.cpu().numpy()
    kconf = result.keypoints.conf.cpu().numpy() if result.keypoints.conf is not None else None
    ours = [i for i in range(len(cls)) if names[cls[i]] == TARGET_CLASS and conf[i] >= MIN_BOX_CONF]
    if len(ours) != 1:
        return None
    i = ours[0]
    if kconf is None or kpts.shape[1] < 2 or kconf[i][:2].min() < MIN_KEYPOINT_CONF:
        return None
    for j in range(len(cls)):
        if j != i and conf[j] >= 0.25 and box_gap(xyxy[i], xyxy[j]) < MIN_SEPARATION_PX:
            return None
    return {
        "box": xyxy[i].tolist(),
        "conf": float(conf[i]),
        "front": kpts[i][0].tolist(),
        "back": kpts[i][1].tolist(),
        "keypoint_conf": kconf[i][:2].tolist(),
        "others": [xyxy[j].tolist() for j in range(len(cls)) if j != i and conf[j] >= 0.25],
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument("clip_dir", type=Path)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--per-clip", type=int, default=3)
    parser.add_argument("--stride", type=int, default=90, help="frames between inference passes")
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument(
        "--max-frames", type=int, default=0, help="stop decoding after this many frames"
    )
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(str(args.model))
    names = {int(k): str(v) for k, v in model.names.items()}
    calibration = load_camera_calibration(args.calibration)
    map_x, map_y, k_rect = rectify_maps(calibration, (calibration.width, calibration.height))
    statuses = load_statuses(args.scene_dir)
    out_root = args.scene_dir / "robot_frames"
    out_root.mkdir(parents=True, exist_ok=True)

    kept_total = 0
    for video in sorted(args.clip_dir.glob("*.mp4")):
        pose_path = args.scene_dir / "poses" / f"{video.stem}.toml"
        if not pose_path.exists() or statuses.get(video.stem) != "ok":
            continue
        hull = cv2.imread(
            str(args.scene_dir / "targets" / video.stem / "hull_mask.png"), cv2.IMREAD_GRAYSCALE
        )
        hull_inner = cv2.erode(hull, np.ones((2 * HULL_MARGIN_PX + 1,) * 2, np.uint8))
        cage = load_cage_calibration(pose_path)
        tf = cage.tf_camera_from_fieldcenter
        capture = cv2.VideoCapture(str(video))
        total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        if args.max_frames:
            total = min(total, args.max_frames)
        candidates: list[tuple[int, dict[str, Any], np.ndarray]] = []
        index = 0
        while index < total:
            ok, frame = capture.read()
            if not ok:
                break
            if index % args.stride == 0:
                result = model.predict(frame, imgsz=args.imgsz, conf=0.25, verbose=False)[0]
                det = pick_detection(result, names)
                if det is not None:
                    centre = undistort_points(
                        np.array([[(det["box"][0] + det["box"][2]) / 2, det["box"][3]]]),
                        calibration,
                        k_rect,
                    )[0]
                    u, v = int(round(centre[0])), int(round(centre[1]))
                    if 0 <= v < hull.shape[0] and 0 <= u < hull.shape[1] and hull_inner[v, u] > 0:
                        candidates.append((index, det, frame.copy()))
            index += 1
        capture.release()
        if not candidates:
            print(f"{video.stem}: no clean MRS BUFF frames")
            continue
        # Spread the picks over the clip: first, last, and evenly between.
        picks = [
            candidates[int(round(i))]
            for i in np.linspace(0, len(candidates) - 1, min(args.per_clip, len(candidates)))
        ]
        for frame_index, det, frame in picks:
            rectified = cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)
            pts = undistort_points(np.array([det["front"], det["back"]]), calibration, k_rect)
            box = undistort_points(
                np.array([[det["box"][0], det["box"][1]], [det["box"][2], det["box"][3]]]),
                calibration,
                k_rect,
            )
            floor_h = pixels_to_field_plane(pts, tf, k_rect)
            if np.isnan(floor_h).any():
                continue
            floor_w = (WORLD_FROM_HFIELD[:3, :3] @ floor_h.T).T
            front_w, back_w = floor_w[0], floor_w[1]
            centre_w = (front_w + back_w) / 2
            heading = front_w - back_w
            yaw = math.degrees(math.atan2(heading[1], heading[0]))
            name = f"{video.stem}_f{frame_index:05d}"
            frame_dir = out_root / name
            frame_dir.mkdir(exist_ok=True)
            cv2.imwrite(str(frame_dir / "frame.png"), rectified)
            pose = {
                "clip": video.stem,
                "frame_index": frame_index,
                "pose_toml": str(pose_path),
                "robot": TARGET_CLASS,
                "x_m": float(centre_w[0]),
                "y_m": float(centre_w[1]),
                "yaw_deg": yaw,
                "front_w": front_w.tolist(),
                "back_w": back_w.tolist(),
                "box_rect": [
                    float(box[0][0]),
                    float(box[0][1]),
                    float(box[1][0]),
                    float(box[1][1]),
                ],
                "keypoints_rect": pts.tolist(),
                "detection": det,
            }
            (frame_dir / "pose.json").write_text(json.dumps(pose, indent=2) + "\n")
            overlay = rectified.copy()
            cv2.rectangle(
                overlay,
                (int(box[0][0]), int(box[0][1])),
                (int(box[1][0]), int(box[1][1])),
                (0, 255, 255),
                2,
            )
            cv2.circle(overlay, (int(pts[0][0]), int(pts[0][1])), 8, (0, 255, 0), -1)
            cv2.circle(overlay, (int(pts[1][0]), int(pts[1][1])), 8, (0, 0, 255), -1)
            text = f"{name}  x {centre_w[0]:+.2f} y {centre_w[1]:+.2f} yaw {yaw:.0f}"
            cv2.putText(overlay, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(overlay, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imwrite(str(frame_dir / "overlay.png"), overlay)
            kept_total += 1
            print(
                f"{name}: x {centre_w[0]:+.2f} y {centre_w[1]:+.2f} m, yaw {yaw:.0f} deg, "
                f"{len(det['others'])} other boxes"
            )
    print(f"{kept_total} robot frames under {out_root}")


if __name__ == "__main__":
    main()
