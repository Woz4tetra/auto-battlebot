"""Annotate fixed-camera cage clips with YOLO detections, gated by the cage hull.

Runs one or more ultralytics .pt models over each clip and writes an annotated mp4
per model. Detections whose box centre falls inside the DeepLab convex hull draw
solid; detections outside draw dim and dashed. Keeping both on screen is the point:
without the dropped boxes you cannot tell "the hull deleted it" from "the network
never found it".

Also writes a compact per-clip stats JSON, which is where the write-up's numbers
come from. Per-frame kept counts are what reliability is measured on: a 1v1 fight
has two robots, so a frame with fewer than two kept `robot` boxes is a dropout.

Requires PYTHONPATH=training/deeplab for the sibling DeepLab import in field_hull.

Usage:
    PYTHONPATH=training/deeplab python annotate_cage_video.py CLIP_OR_DIR \
        --models a.pt b.pt -o OUT_DIR
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any, NamedTuple

import cv2
import numpy as np
from tqdm import tqdm
from ultralytics import YOLO

from playground.bgsub_cage.field_hull import (
    FieldSegmenter,
    compute_hull,
    hull_mask_from_polygon,
    load_hull,
    save_hull,
)

HULL_COLOR = (255, 220, 60)  # BGR cyan-ish
KEPT_COLORS = [(90, 230, 90), (60, 200, 255)]  # robot green, house_bot amber
DROPPED_COLOR = (120, 120, 120)  # off-field: no overlap with the hull at all
WALL_COLOR = (80, 140, 235)  # dropped but overlapping the hull, so pinned at the wall

# A dropped box with at least this much of its area inside the hull is a robot against
# the cage wall, not something outside the cage.
WALL_OVERLAP_MIN = 0.10

# Keypoint order is [front, back], matching auto_battlebot/eval/scoring.py. The heading
# vector is drawn back -> front so the arrow points where the robot faces.
FRONT_IDX = 0
BACK_IDX = 1
POSE_COLOR = (235, 100, 235)  # magenta, any posed robot
TARGET_COLOR = (80, 255, 255)  # yellow, the class named by --pose-target
KEYPOINT_MIN_CONF = 0.25
# Arrow drawn past the front keypoint so a short front-back span is still readable.
HEADING_EXTEND = 1.6
TEXT_COLOR = (245, 245, 245)
FONT = cv2.FONT_HERSHEY_SIMPLEX

# Full box dumps every Nth frame. The per-frame count arrays cover every frame; this
# keeps a readable sample of actual geometry without a 20 MB JSON per clip.
BOX_DUMP_STRIDE = 10


def dashed_rectangle(
    canvas: np.ndarray, box: tuple[int, int, int, int], color: tuple[int, int, int], dash: int = 10
) -> None:
    x1, y1, x2, y2 = box
    for x in range(x1, x2, dash * 2):
        cv2.line(canvas, (x, y1), (min(x + dash, x2), y1), color, 1)
        cv2.line(canvas, (x, y2), (min(x + dash, x2), y2), color, 1)
    for y in range(y1, y2, dash * 2):
        cv2.line(canvas, (x1, y), (x1, min(y + dash, y2)), color, 1)
        cv2.line(canvas, (x2, y), (x2, min(y + dash, y2)), color, 1)


def parse_poses(result: Any, target_name: str | None) -> list[dict[str, Any]]:
    """Ultralytics pose output to boxes plus a back->front heading vector.

    Keypoints come as (n, 2, 3): front then back, each x, y, conf. The heading is only
    reported when both keypoints clear KEYPOINT_MIN_CONF, because a vector built from a
    guessed keypoint points somewhere arbitrary and would be worse than drawing nothing.
    """
    poses: list[dict[str, Any]] = []
    boxes = result.boxes
    if boxes is None or not len(boxes):
        return poses

    names = result.names
    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    classes = boxes.cls.cpu().numpy().astype(int)
    kpts = result.keypoints
    data = kpts.data.cpu().numpy() if kpts is not None else None

    for index, ((x1, y1, x2, y2), conf, cls) in enumerate(zip(xyxy, confs, classes)):
        name = names.get(int(cls), str(cls))
        entry: dict[str, Any] = {
            "box": (int(x1), int(y1), int(x2), int(y2)),
            "conf": float(conf),
            "cls": int(cls),
            "name": name,
            "is_target": target_name is not None and name == target_name,
            "front": None,
            "back": None,
            "heading_deg": None,
        }
        if data is not None and index < len(data) and data[index].shape[0] >= 2:
            front = data[index][FRONT_IDX]
            back = data[index][BACK_IDX]
            if front[2] >= KEYPOINT_MIN_CONF and back[2] >= KEYPOINT_MIN_CONF:
                entry["front"] = (float(front[0]), float(front[1]))
                entry["back"] = (float(back[0]), float(back[1]))
                dx = front[0] - back[0]
                dy = front[1] - back[1]
                if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                    # Screen y grows downward, so negate to report a normal CCW angle.
                    entry["heading_deg"] = float(np.degrees(np.arctan2(-dy, dx)))
        poses.append(entry)
    return poses


def draw_pose(canvas: np.ndarray, pose: dict[str, Any]) -> None:
    """Box, keypoints and heading arrow for one posed robot."""
    color = TARGET_COLOR if pose["is_target"] else POSE_COLOR
    x1, y1, x2, y2 = pose["box"]
    thickness = 3 if pose["is_target"] else 2
    cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)

    label = f"{pose['name']} {pose['conf']:.2f}"
    if pose["heading_deg"] is not None:
        label += f"  {pose['heading_deg']:+.0f}deg"
    cv2.putText(
        canvas, label, (x1, min(canvas.shape[0] - 6, y2 + 20)), FONT, 0.55, color, 2, cv2.LINE_AA
    )

    front, back = pose["front"], pose["back"]
    if front is None or back is None:
        return
    fx, fy = front
    bx, by = back
    tip = (int(bx + (fx - bx) * HEADING_EXTEND), int(by + (fy - by) * HEADING_EXTEND))
    cv2.arrowedLine(
        canvas, (int(bx), int(by)), tip, color, thickness + 1, cv2.LINE_AA, tipLength=0.3
    )
    cv2.circle(canvas, (int(bx), int(by)), 5, color, -1, cv2.LINE_AA)
    cv2.circle(canvas, (int(fx), int(fy)), 5, (255, 255, 255), -1, cv2.LINE_AA)


def draw_frame(
    frame: np.ndarray,
    detections: list[dict[str, Any]],
    polygon: np.ndarray | None,
    names: dict[int, str],
    header: str,
    poses: list[dict[str, Any]] | None = None,
) -> np.ndarray:
    canvas = frame.copy()
    if polygon is not None and len(polygon):
        cv2.polylines(canvas, [polygon], isClosed=True, color=HULL_COLOR, thickness=2)

    for detection in detections:
        x1, y1, x2, y2 = detection["box"]
        label = f"{names.get(detection['cls'], detection['cls'])} {detection['conf']:.2f}"
        if detection["kept"]:
            color = KEPT_COLORS[detection["cls"] % len(KEPT_COLORS)]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            cv2.putText(canvas, label, (x1, max(14, y1 - 6)), FONT, 0.5, color, 2, cv2.LINE_AA)
        else:
            at_wall = detection["overlap"] >= WALL_OVERLAP_MIN
            color = WALL_COLOR if at_wall else DROPPED_COLOR
            dashed_rectangle(canvas, (x1, y1, x2, y2), color)
            suffix = " wall" if at_wall else " off-field"
            cv2.putText(
                canvas, label + suffix, (x1, max(14, y1 - 6)), FONT, 0.45, color, 1, cv2.LINE_AA
            )

    for pose in poses or []:
        draw_pose(canvas, pose)

    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(canvas, header, (10, 23), FONT, 0.6, TEXT_COLOR, 1, cv2.LINE_AA)
    return canvas


def open_encoder(destination: Path, width: int, height: int, fps: float, encoder: list[str]):
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-nostdin",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-s",
        f"{width}x{height}",
        "-r",
        f"{fps:.6f}",
        "-i",
        "-",
        *encoder,
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        "-y",
        str(destination),
    ]
    return subprocess.Popen(command, stdin=subprocess.PIPE)


def pick_encoder() -> list[str]:
    probe = subprocess.run(
        ["ffmpeg", "-hide_banner", "-encoders"], capture_output=True, text=True, check=False
    )
    if "h264_nvenc" in probe.stdout:
        return ["-c:v", "h264_nvenc", "-preset", "p5", "-cq", "23"]
    return ["-c:v", "libx264", "-crf", "23", "-preset", "veryfast"]


def box_overlap(integral: np.ndarray, box: tuple[int, int, int, int]) -> float:
    """Fraction of a box's area that lies inside the hull, via a summed-area table."""
    x1, y1, x2, y2 = box
    area = (x2 - x1) * (y2 - y1)
    if area <= 0:
        return 0.0
    inside = integral[y2, x2] - integral[y1, x2] - integral[y2, x1] + integral[y1, x1]
    return float(inside) / float(area)


def parse_detections(
    result: Any, hull_mask: np.ndarray, integral: np.ndarray, has_hull: bool
) -> list[dict[str, Any]]:
    """Ultralytics boxes to plain dicts, each tagged against the cage hull.

    `kept` is the gate: the box centre is inside the hull. Centre rather than full
    containment, because a robot pinned against the cage wall has a box that spills
    onto the barrier while the robot itself is still on the floor.

    `overlap` records how much of the box is inside anyway, which separates the two
    reasons a box gets dropped. A robot at the wall lands just outside the centre
    test but still overlaps the hull; a person in the crowd overlaps nothing. Without
    that split, a wall pin and a spectator look like the same failure.
    """
    detections: list[dict[str, Any]] = []
    boxes = result.boxes
    if boxes is None or not len(boxes):
        return detections

    height, width = hull_mask.shape[:2]
    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    classes = boxes.cls.cpu().numpy().astype(int)
    for (x1, y1, x2, y2), conf, cls in zip(xyxy, confs, classes):
        box = (
            int(np.clip(x1, 0, width - 1)),
            int(np.clip(y1, 0, height - 1)),
            int(np.clip(x2, 1, width)),
            int(np.clip(y2, 1, height)),
        )
        cx = int(np.clip((x1 + x2) / 2, 0, width - 1))
        cy = int(np.clip((y1 + y2) / 2, 0, height - 1))
        detections.append(
            {
                "box": box,
                "conf": float(conf),
                "cls": int(cls),
                "kept": (not has_hull) or bool(hull_mask[cy, cx]),
                "overlap": 1.0 if not has_hull else box_overlap(integral, box),
            }
        )
    return detections


class FrameSummary(NamedTuple):
    """One frame's counts, split so the two drop reasons stay apart."""

    per_class: list[int]
    dropped_at_wall: int
    dropped_off_field: int
    confs: list[float]
    areas: list[float]


def summarize_frame(detections: list[dict[str, Any]], class_count: int) -> FrameSummary:
    """Kept count per class, both drop reasons, and the kept boxes' confidences and areas."""
    per_class = [0] * class_count
    at_wall = 0
    off_field = 0
    confs: list[float] = []
    areas: list[float] = []
    for detection in detections:
        if not detection["kept"]:
            if detection["overlap"] >= WALL_OVERLAP_MIN:
                at_wall += 1
            else:
                off_field += 1
            continue
        per_class[detection["cls"]] += 1
        confs.append(detection["conf"])
        x1, y1, x2, y2 = detection["box"]
        areas.append(float((x2 - x1) * (y2 - y1)))
    return FrameSummary(per_class, at_wall, off_field, confs, areas)


def maybe_dump(
    box_dump: list[dict[str, Any]], frame_index: int, detections: list[dict[str, Any]]
) -> None:
    """Record full box geometry every BOX_DUMP_STRIDE frames.

    Per-frame counts cover every frame; this keeps a readable sample of actual boxes
    without a 20 MB JSON per clip.
    """
    if frame_index % BOX_DUMP_STRIDE:
        return
    box_dump.append(
        {
            "frame": frame_index,
            "boxes": [
                {
                    "cls": d["cls"],
                    "conf": round(d["conf"], 3),
                    "box": list(d["box"]),
                    "kept": d["kept"],
                    "overlap": round(d["overlap"], 3),
                }
                for d in detections
            ],
        }
    )


def zero_runs(counts: list[int], expected: int) -> list[list[int]]:
    """Contiguous frame ranges where fewer than `expected` robots were kept."""
    runs: list[list[int]] = []
    start: int | None = None
    for index, value in enumerate(counts):
        if value < expected and start is None:
            start = index
        elif value >= expected and start is not None:
            runs.append([start, index - 1])
            start = None
    if start is not None:
        runs.append([start, len(counts) - 1])
    return runs


def annotate(
    video_path: Path,
    model: YOLO,
    model_tag: str,
    hull_record: dict[str, Any],
    output_dir: Path,
    args: argparse.Namespace,
    encoder: list[str],
    pose_model: YOLO | None = None,
) -> dict[str, Any]:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {video_path}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

    polygon_list = hull_record.get("polygon") or []
    polygon = np.array(polygon_list, dtype=np.int32) if polygon_list else None
    hull_mask = hull_mask_from_polygon(polygon_list, (height, width))
    integral = cv2.integral((hull_mask > 0).astype(np.uint8))
    has_hull = bool(polygon_list)

    names: dict[int, str] = model.names
    stem = video_path.stem
    destination = output_dir / f"{stem}_{model_tag}.mp4"
    writer = open_encoder(destination, width, height, fps, encoder)

    kept_per_class: list[list[int]] = []
    wall_counts: list[int] = []
    off_field_counts: list[int] = []
    target_per_frame: list[int] = []
    heading_per_frame: list[int] = []
    heading_values: list[float] = []
    box_dump: list[dict[str, Any]] = []
    conf_values: list[float] = []
    area_values: list[float] = []
    class_totals: Counter[int] = Counter()

    batch: list[np.ndarray] = []
    batch_indices: list[int] = []
    frame_index = 0

    def flush() -> None:
        if not batch:
            return
        results = model.predict(
            batch, conf=args.conf, imgsz=args.imgsz, device=args.device, verbose=False
        )
        pose_results = (
            pose_model.predict(
                batch,
                conf=args.pose_conf,
                imgsz=args.pose_imgsz,
                device=args.device,
                verbose=False,
            )
            if pose_model is not None
            else [None] * len(batch)
        )
        for local, (result, source_index) in enumerate(zip(results, batch_indices)):
            detections = parse_detections(result, hull_mask, integral, has_hull)
            summary = summarize_frame(detections, len(names))
            kept_per_class.append(summary.per_class)
            wall_counts.append(summary.dropped_at_wall)
            off_field_counts.append(summary.dropped_off_field)
            conf_values.extend(summary.confs)
            area_values.extend(summary.areas)
            class_totals.update(dict(enumerate(summary.per_class)))
            maybe_dump(box_dump, source_index, detections)

            poses = (
                parse_poses(pose_results[local], args.pose_target) if pose_model is not None else []
            )
            targets = [p for p in poses if p["is_target"]]
            headed = [p for p in targets if p["heading_deg"] is not None]
            target_per_frame.append(len(targets))
            heading_per_frame.append(len(headed))
            heading_values.extend(p["heading_deg"] for p in headed)

            header = (
                f"{model_tag}  |  {stem[:52]}  |  frame {source_index}  |  "
                f"kept {sum(summary.per_class)}  at-wall {summary.dropped_at_wall}  "
                f"off-field {summary.dropped_off_field}"
            )
            if pose_model is not None:
                header += f"  |  {args.pose_target}: {len(targets)}  heading: {len(headed)}"
            writer.stdin.write(
                draw_frame(batch[local], detections, polygon, names, header, poses).tobytes()
            )
        batch.clear()
        batch_indices.clear()

    with tqdm(total=total, desc=f"{model_tag} {stem[:38]}", unit="f", leave=False) as progress:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            batch.append(frame)
            batch_indices.append(frame_index)
            frame_index += 1
            progress.update(1)
            if len(batch) == args.batch:
                flush()
        flush()

    capture.release()
    writer.stdin.close()
    writer.wait()

    return build_stats(
        video_path=video_path,
        model_tag=model_tag,
        destination=destination,
        names=names,
        hull_record=hull_record,
        args=args,
        fps=fps,
        width=width,
        height=height,
        kept_per_class=kept_per_class,
        wall_counts=wall_counts,
        off_field_counts=off_field_counts,
        target_per_frame=target_per_frame,
        heading_per_frame=heading_per_frame,
        heading_values=heading_values,
        class_totals=class_totals,
        conf_values=conf_values,
        area_values=area_values,
        box_dump=box_dump,
    )


def percentiles(values: list[float], as_int: bool = False) -> dict[str, float | int]:
    if not values:
        return {}
    marks = (5, 25, 50, 75, 95)
    if as_int:
        return {str(p): int(np.percentile(values, p)) for p in marks}
    return {str(p): round(float(np.percentile(values, p)), 3) for p in marks}


def pose_stats(
    target_per_frame: list[int],
    heading_per_frame: list[int],
    heading_values: list[float],
) -> dict[str, Any]:
    """Pose coverage for the named target class. Empty when no pose model ran."""
    if not target_per_frame:
        return {}
    frames = len(target_per_frame)
    found = sum(1 for value in target_per_frame if value)
    headed = sum(1 for value in heading_per_frame if value)
    return {
        "pose_frames_with_target": found,
        "pose_target_rate": round(found / frames, 4),
        "pose_frames_with_heading": headed,
        "pose_heading_rate": round(headed / frames, 4),
        "pose_heading_given_target": round(headed / found, 4) if found else 0.0,
        "pose_target_per_frame": target_per_frame,
        "pose_heading_deg_sample": [round(v, 1) for v in heading_values[::30]],
    }


def build_stats(
    *,
    video_path: Path,
    model_tag: str,
    destination: Path,
    names: dict[int, str],
    hull_record: dict[str, Any],
    args: argparse.Namespace,
    fps: float,
    width: int,
    height: int,
    kept_per_class: list[list[int]],
    wall_counts: list[int],
    off_field_counts: list[int],
    target_per_frame: list[int],
    heading_per_frame: list[int],
    heading_values: list[float],
    class_totals: Counter,
    conf_values: list[float],
    area_values: list[float],
    box_dump: list[dict[str, Any]],
) -> dict[str, Any]:
    """Per-clip record. `robot` is class 0, and a 1v1 fight always has two of them.

    That makes `both_robots_rate` a reliability measure with no hand labelling: any
    frame holding fewer than two kept `robot` boxes is a miss.
    """
    robot_counts = [counts[0] if counts else 0 for counts in kept_per_class]
    frames = len(robot_counts)
    both = sum(1 for value in robot_counts if value >= 2)
    at_least_one = sum(1 for value in robot_counts if value >= 1)
    runs = zero_runs(robot_counts, 2)
    empty_runs = zero_runs(robot_counts, 1)

    return {
        "clip": video_path.name,
        "model": model_tag,
        "output": destination.name,
        "frames": frames,
        "fps": round(fps, 3),
        "width": width,
        "height": height,
        "conf": args.conf,
        "imgsz": args.imgsz,
        "class_names": {str(k): v for k, v in names.items()},
        "hull_fraction": hull_record.get("hull_fraction"),
        "raw_mask_fraction": hull_record.get("raw_mask_fraction"),
        "kept_total_by_class": {str(k): v for k, v in sorted(class_totals.items())},
        "dropped_at_wall_total": int(sum(wall_counts)),
        "dropped_off_field_total": int(sum(off_field_counts)),
        "frames_with_two_robots": both,
        "frames_with_one_robot": at_least_one,
        "both_robots_rate": round(both / frames, 4) if frames else 0.0,
        "any_robot_rate": round(at_least_one / frames, 4) if frames else 0.0,
        "dropout_runs_under_two": len(runs),
        "longest_dropout_frames_under_two": max((b - a + 1 for a, b in runs), default=0),
        "blind_runs_zero_robots": len(empty_runs),
        "longest_blind_frames": max((b - a + 1 for a, b in empty_runs), default=0),
        "conf_percentiles": percentiles(conf_values),
        "box_area_px_percentiles": percentiles(area_values, as_int=True),
        "robot_count_per_frame": robot_counts,
        "dropped_at_wall_per_frame": wall_counts,
        "dropped_off_field_per_frame": off_field_counts,
        "box_dump_stride": BOX_DUMP_STRIDE,
        "box_dump": box_dump,
        **pose_stats(target_per_frame, heading_per_frame, heading_values),
    }


def hull_preview(record: dict[str, Any], destination: Path) -> None:
    """Median frame with the hull outline drawn on it, for eyeballing before a full run."""
    preview = record["_background"].copy()
    hull = record["_hull"]
    tint = preview.copy()
    tint[hull > 0] = (0.65 * tint[hull > 0] + 0.35 * np.array([60, 220, 60])).astype(np.uint8)
    preview = tint
    polygon = record.get("polygon") or []
    if polygon:
        cv2.polylines(
            preview, [np.array(polygon, dtype=np.int32)], True, HULL_COLOR, 3, cv2.LINE_AA
        )
    caption = (
        f"{record['video']}  hull {record['hull_fraction'] * 100:.1f}% of frame  "
        f"(raw mask {record['raw_mask_fraction'] * 100:.1f}%)"
    )
    cv2.rectangle(preview, (0, 0), (preview.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(preview, caption, (10, 23), FONT, 0.6, TEXT_COLOR, 1, cv2.LINE_AA)
    cv2.imwrite(str(destination), preview)


def collect_videos(target: Path) -> list[Path]:
    if target.is_file():
        return [target]
    return sorted(p for p in target.glob("*.mp4") if "_yolo" not in p.stem)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("input", type=Path, help="Clip or directory of clips")
    parser.add_argument("--models", type=Path, nargs="+", required=True, help="ultralytics .pt")
    parser.add_argument("-o", "--output", type=Path, required=True, help="Output directory")
    parser.add_argument("-c", "--conf", type=float, default=0.25, help="Confidence (default 0.25)")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference size (training was 640)")
    parser.add_argument("--device", default="0", help="Torch device (default 0)")
    parser.add_argument("--batch", type=int, default=16, help="Frames per predict call")
    parser.add_argument("--hull-samples", type=int, default=60, help="Frames in the median")
    parser.add_argument("--rebuild-hulls", action="store_true", help="Ignore cached hull JSON")
    parser.add_argument(
        "--hull-only", action="store_true", help="Build hulls and previews, no YOLO"
    )
    parser.add_argument(
        "--pose-model",
        type=Path,
        default=None,
        help="Optional ultralytics pose .pt drawn on top: box, keypoints and a "
        "back-to-front heading arrow",
    )
    parser.add_argument(
        "--pose-conf", type=float, default=0.25, help="Pose confidence (default 0.25)"
    )
    parser.add_argument(
        "--pose-imgsz", type=int, default=640, help="Pose inference size (default 640)"
    )
    parser.add_argument(
        "--pose-target",
        default="mrs_buff_mk3",
        help="Pose class highlighted and counted (default mrs_buff_mk3)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    videos = collect_videos(args.input)
    if not videos:
        print(f"No clips found in {args.input}")
        return 1

    args.output.mkdir(parents=True, exist_ok=True)
    preview_dir = args.output / "hull_previews"
    preview_dir.mkdir(exist_ok=True)

    print(f"{len(videos)} clips")
    segmenter = FieldSegmenter()
    print(
        f"DeepLab: {segmenter.config.backbone}/{segmenter.config.decoder} "
        f"at {segmenter.config.image_size}+{segmenter.config.pad_size}"
    )

    hulls: dict[str, dict[str, Any]] = {}
    for video in videos:
        cache = video.with_suffix(".hull.json")
        preview = preview_dir / f"{video.stem}_hull.jpg"
        # A cached hull still needs its preview drawn if the preview is missing, so a
        # rerun into a fresh output directory does not silently skip the eyeball check.
        if cache.exists() and preview.exists() and not args.rebuild_hulls:
            hulls[video.name] = load_hull(cache)
            continue
        record = compute_hull(video, segmenter, args.hull_samples)
        save_hull(record, cache)
        hull_preview(record, preview)
        hulls[video.name] = {k: v for k, v in record.items() if not k.startswith("_")}
        print(
            f"  {video.name}: hull {record['hull_fraction'] * 100:.1f}% "
            f"(raw {record['raw_mask_fraction'] * 100:.1f}%)"
        )

    if args.hull_only:
        print(f"Hull previews in {preview_dir}")
        return 0

    encoder = pick_encoder()
    pose_model = None
    if args.pose_model:
        pose_model = YOLO(str(args.pose_model))
        print(
            f"Pose: {args.pose_model.name}  classes={pose_model.names}  target={args.pose_target}"
        )
        if args.pose_target not in pose_model.names.values():
            print(
                f"  WARNING: '{args.pose_target}' is not a class of this model; "
                f"no box will ever be marked as the target"
            )
    stats: list[dict[str, Any]] = []
    for model_path in args.models:
        tag = model_path.stem.split("_")[0]
        print(f"\n=== {tag} ({model_path.name}) ===")
        model = YOLO(str(model_path))
        for video in videos:
            record = annotate(
                video, model, tag, hulls[video.name], args.output, args, encoder, pose_model
            )
            stats.append(record)
            (args.output / f"{video.stem}_{tag}.dets.json").write_text(
                json.dumps(record, separators=(",", ":")) + "\n"
            )
            print(
                f"  {video.name}: both-robot {record['both_robots_rate'] * 100:.1f}%  "
                f"any {record['any_robot_rate'] * 100:.1f}%  "
                f"wall-drops {record['dropped_at_wall_total']}  "
                f"off-field {record['dropped_off_field_total']}  "
                f"longest blind {record['longest_blind_frames']}f"
                + (
                    f"  |  {args.pose_target} {record['pose_target_rate'] * 100:.1f}%"
                    f"  heading {record['pose_heading_rate'] * 100:.1f}%"
                    if "pose_target_rate" in record
                    else ""
                )
            )

    summary = [
        {
            key: value
            for key, value in record.items()
            if key
            not in (
                "robot_count_per_frame",
                "dropped_at_wall_per_frame",
                "dropped_off_field_per_frame",
                "box_dump",
                "pose_target_per_frame",
            )
        }
        for record in stats
    ]
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"\nWrote {len(stats)} annotated clips to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
