#!/usr/bin/env python3
"""Build a hand-labeling eval dataset from fixed-camera fight video, with model pre-labels.

Like make_eval_dataset.py, but the source is an mp4 rather than an MCAP, so there is no
/camera/image stream and no SVO frame index. One subdataset per video:

    <output-dir>/<video stem>/images/<stamp_ns>.png
    <output-dir>/<video stem>/labels/<stamp_ns>.txt   (model pre-labels, ready to correct)
    <output-dir>/<video stem>/data.yaml               (schema + source video + frame indices)

Frames are named by a nanosecond stamp derived from the frame index and the video's frame
rate, so the stem stays an integer (score.py keys frames on `int(path.stem)`) and sorts in
capture order. `frame_indices` in data.yaml maps back to the source video.

Frames are decoded sequentially rather than seeked. Seeking h264 lands on non-keyframes and
smears the decode, which is the same failure the SVO reader hits.

Pre-labels come from two engines, merged the way the deployed C++ merges them: keypoint
detections win, and a blob detection close to one is dropped so the same robot is not
labeled twice. See `merge_detections` for how faithful that port is.

Only classes that can carry keypoints reach the label file (`KEEP_DATASET_CLASSES`), so the
blob engine shapes the output by suppression alone. A box without its keypoints is not a
usable annotation in a kpt_shape dataset.

Usage:
    python training/model_eval/make_video_eval_dataset.py \
        data/downloads/mrsbuff_may26/BZ-...-Cage-2-Overhead-High.mp4=50 \
        data/downloads/massd_resurgence6_mrsbuff/r1_beeroll_vs_mrsbuff.mp4=25
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml
from tqdm import tqdm

from auto_battlebot.perception.trt_yolo import TrtYoloModel

_HERE = Path(__file__).parent
_REPO = _HERE.parent.parent
DEFAULT_OUTPUT_DIR = _HERE.parent / "data" / "nhrl_cage_high_eval"

DEFAULT_BBOX_ENGINE = (
    _REPO / "data/eval_models/yolo26x_nhrl_robots_bbox_2class_2026-09-04_x86_64_sm89.engine"
)
DEFAULT_POSE_ENGINE = (
    _REPO / "data/models/yolo26x-pose_our_robot_keypoints_rect384x640_2026-09-07_x86_64_sm89.engine"
)

# Deployed thresholds. Blob is set in config/_desktop.toml; the keypoint model has no config
# override anywhere, so 0.50 is the struct default in include/keypoint_model/config.hpp.
DEFAULT_BBOX_CONF = 0.6
DEFAULT_POSE_CONF = 0.5
NMS_IOU = 0.45
# Matches CPP_LETTERBOX_PADDING and the C++ letterbox_padding default of 0.1f.
LETTERBOX_PADDING = 0.1

# Class schema, matching nhrl_keypoints_eval_test so the existing taxonomies apply unchanged.
SCHEMA: dict = {
    "names": ["mr_stabs_mk2", "mrs_buff_mk3", "opponent", "house_bot", "object"],
    "colors": ["#ea2e2e", "#0ea5e9", "#22c55e", "#f59e0b", "#a855f7"],
    "nc": 5,
    "kpt_shape": [2, 3],
    "flip_idx": [0, 1],
}

# Engine class index -> dataset class index.
# The pose engine is trained on our_robot_keypoints: [mr_stabs_mk2, mrs_buff_mk3].
POSE_CLASS_TO_DATASET = {0: 0, 1: 1}
# The bbox engine is trained on nhrl_robots_bbox_2class: [robot, house_bot]. The C++ reads the
# same two slots as label_indices = ["OPPONENT", "HOUSE_BOT"], so "robot" is the opponent class;
# our own robot is identified by the pose model, never by this one.
BBOX_CLASS_TO_DATASET = {0: 2, 1: 3}

# Only classes that can carry keypoints are written. This is a kpt_shape [2, 3] dataset, and the
# blob engine has num_keypoints=0, so an opponent or house_bot row is a box with its keypoints
# missing rather than a usable annotation. The blob pass still runs, because its detections are
# what suppress duplicate boxes on our own robot; they just do not reach the label file.
KEEP_DATASET_CLASSES = {0, 1}

# Suppression radius floor in pixels, guarding against a degenerate zero-size box. The C++ floor
# is 0.20 m, which for two 3 lb robots sits below the size-scaled term and so rarely binds.
MIN_SUPPRESS_RADIUS_PX = 16.0
BLOB_OVERWRITE_SIZE_SCALE = 0.5


@dataclass
class Row:
    """One YOLO label row: a box, and keypoints when the detection carries them."""

    class_id: int
    cx: float
    cy: float
    w: float
    h: float
    keypoints: list[tuple[float, float, int]]

    def format(self) -> str:
        parts = [str(self.class_id), f"{self.cx:.6f}", f"{self.cy:.6f}", f"{self.w:.6f}"]
        parts.append(f"{self.h:.6f}")
        for kx, ky, vis in self.keypoints:
            parts += [f"{kx:.6f}", f"{ky:.6f}", str(vis)]
        return " ".join(parts)


def parse_job(token: str) -> tuple[Path, int]:
    """`path=count` into an existing video and a positive frame count."""
    path_text, _, count_text = token.rpartition("=")
    if not path_text or not count_text.isdigit() or int(count_text) <= 0:
        raise argparse.ArgumentTypeError(
            f"expected PATH=COUNT with a positive count, got {token!r}"
        )
    path = Path(path_text)
    if not path.is_file():
        raise argparse.ArgumentTypeError(f"no such video: {path}")
    return path, int(count_text)


def selected_indices(count: int, n_pick: int) -> list[int]:
    """`n_pick` evenly spaced frame indices across the whole clip, both ends inclusive."""
    if n_pick <= 0 or count <= 0:
        return []
    if n_pick >= count:
        return list(range(count))
    return sorted({int(round(i)) for i in np.linspace(0, count - 1, n_pick)})


def long_side(xyxy: np.ndarray) -> float:
    """Longer box edge in pixels, standing in for the robot's extent."""
    return float(max(xyxy[2] - xyxy[0], xyxy[3] - xyxy[1]))


def center(xyxy: np.ndarray) -> tuple[float, float]:
    return (float(xyxy[0] + xyxy[2]) / 2.0, float(xyxy[1] + xyxy[3]) / 2.0)


def merge_detections(bbox_dets: list, pose_dets: list) -> list[Row]:
    """Keypoint detections win; blobs near one are dropped. Image-space port of the C++ rule.

    `RobotFrontBackFilter::is_blob_suppressed_by_keypoint` drops a blob whose field-frame
    position lies within `max(0.20, 0.5 * (blob.size.x + keypoint.size.x))` metres of any
    keypoint measurement, class-blind. That runs after both models' output has been projected
    onto the field plane, so reproducing it exactly needs a field transform. These videos have
    no field fit, so the same rule is applied to box centres in pixels with the box's longer
    edge standing in for `size.x`. The size-scaled term carries the behaviour; the 0.20 m floor
    is replaced by a small pixel guard because it rarely binds between two 3 lb robots.

    Not ported: `suppress_blobs_near_our_anchor`, which needs the tracker's held pose across
    frames and has no meaning on independently sampled frames.
    """
    rows: list[Row] = []
    pose_shapes: list[tuple[tuple[float, float], float]] = []

    for xyxy, _conf, class_id, keypoints in pose_dets:
        dataset_class = POSE_CLASS_TO_DATASET.get(int(class_id))
        if dataset_class is None:
            continue
        pose_shapes.append((center(xyxy), long_side(xyxy)))
        rows.append(_row(xyxy, dataset_class, keypoints))

    for xyxy, _conf, class_id, _keypoints in bbox_dets:
        dataset_class = BBOX_CLASS_TO_DATASET.get(int(class_id))
        if dataset_class is None:
            continue
        blob_center, blob_size = center(xyxy), long_side(xyxy)
        suppressed = any(
            float(np.hypot(blob_center[0] - pc[0], blob_center[1] - pc[1]))
            <= max(MIN_SUPPRESS_RADIUS_PX, BLOB_OVERWRITE_SIZE_SCALE * (blob_size + pose_size))
            for pc, pose_size in pose_shapes
        )
        if not suppressed:
            rows.append(_row(xyxy, dataset_class, None))

    return [row for row in rows if row.class_id in KEEP_DATASET_CLASSES]


def _row(xyxy: np.ndarray, dataset_class: int, keypoints: np.ndarray | None) -> Row:
    """A detection in pixels into a normalized YOLO row. Normalization happens later."""
    points: list[tuple[float, float, int]] = []
    if keypoints is not None:
        for kx, ky, visibility in keypoints:
            # The C++ applies the detection threshold again per keypoint, dropping the ones
            # below it. Keeping them as "labeled but not visible" is more useful to correct.
            points.append(
                (float(kx), float(ky), 2 if float(visibility) >= DEFAULT_POSE_CONF else 1)
            )
    return Row(
        dataset_class,
        *center(xyxy),
        float(xyxy[2] - xyxy[0]),
        float(xyxy[3] - xyxy[1]),
        points,
    )


def normalize(row: Row, width: int, height: int) -> Row:
    """Pixel row into a normalized row, clamped to the frame."""

    def clamp(value: float) -> float:
        return min(max(value, 0.0), 1.0)

    return Row(
        row.class_id,
        clamp(row.cx / width),
        clamp(row.cy / height),
        clamp(row.w / width),
        clamp(row.h / height),
        [(clamp(kx / width), clamp(ky / height), vis) for kx, ky, vis in row.keypoints],
    )


def write_video(
    path: Path,
    n_pick: int,
    output_dir: Path,
    bbox_model: TrtYoloModel | None,
    pose_model: TrtYoloModel | None,
) -> tuple[int, int]:
    """Sample n_pick frames and write an edit_labels-ready subdataset. Returns (frames, boxes)."""
    capture = cv2.VideoCapture(str(path))
    count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = capture.get(cv2.CAP_PROP_FPS) or 60.0
    keep = set(selected_indices(count, n_pick))

    dataset_dir = output_dir / path.stem
    images_dir, labels_dir = dataset_dir / "images", dataset_dir / "labels"
    # Clear any prior extraction so re-runs don't mix old and new samples.
    for stale in (images_dir, labels_dir):
        if stale.is_dir():
            for old in stale.iterdir():
                old.unlink()
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)

    frame_indices: list[int] = []
    index = written = boxes = 0
    with tqdm(total=len(keep), desc=f"  {path.stem[:48]}", unit="frame", leave=False) as bar:
        while written < len(keep):
            ok, frame = capture.read()
            if not ok:
                break
            if index in keep:
                stamp_ns = int(round(index / fps * 1e9))
                stem = f"{stamp_ns:019d}"
                cv2.imwrite(str(images_dir / f"{stem}.png"), frame)

                rows: list[Row] = []
                if bbox_model is not None and pose_model is not None:
                    height, width = frame.shape[:2]
                    rows = [
                        normalize(row, width, height)
                        for row in merge_detections(
                            bbox_model.infer(frame), pose_model.infer(frame)
                        )
                    ]
                text = "\n".join(row.format() for row in rows)
                (labels_dir / f"{stem}.txt").write_text(text + "\n" if text else "")

                boxes += len(rows)
                frame_indices.append(index)
                written += 1
                bar.update(1)
            index += 1
    capture.release()

    (dataset_dir / "data.yaml").write_text(
        yaml.safe_dump(
            {
                **SCHEMA,
                "source_video": str(path),
                "fps": round(fps, 6),
                "video_frame_count": count,
                "frame_indices": frame_indices,
            },
            sort_keys=False,
        )
    )
    print(f"  {path.stem[:58]:58s} {written:4d} frames, {boxes:4d} pre-labeled boxes")
    return written, boxes


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("jobs", nargs="+", type=parse_job, metavar="PATH=COUNT")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--bbox-engine", type=Path, default=DEFAULT_BBOX_ENGINE)
    parser.add_argument("--pose-engine", type=Path, default=DEFAULT_POSE_ENGINE)
    parser.add_argument("--bbox-conf", type=float, default=DEFAULT_BBOX_CONF)
    parser.add_argument("--pose-conf", type=float, default=DEFAULT_POSE_CONF)
    parser.add_argument(
        "--no-prelabel", action="store_true", help="write empty labels instead of running models"
    )
    args = parser.parse_args()

    bbox_model = pose_model = None
    if not args.no_prelabel:
        # num_classes is passed explicitly: the raw head is split using it, and a wrong count
        # parses to num_keypoints=0 and near-zero recall rather than failing.
        bbox_model = TrtYoloModel(
            str(args.bbox_engine),
            conf_threshold=args.bbox_conf,
            nms_iou_threshold=NMS_IOU,
            num_classes=2,
            letterbox_padding=LETTERBOX_PADDING,
        )
        pose_model = TrtYoloModel(
            str(args.pose_engine),
            conf_threshold=args.pose_conf,
            nms_iou_threshold=NMS_IOU,
            num_classes=2,
            letterbox_padding=LETTERBOX_PADDING,
        )
        print(f"bbox: {bbox_model.describe()}")
        print(f"pose: {pose_model.describe()}")

    print(f"Classes: {SCHEMA['names']}")
    total_frames = total_boxes = 0
    for path, n_pick in args.jobs:
        frames, boxes = write_video(path, n_pick, args.output_dir, bbox_model, pose_model)
        total_frames += frames
        total_boxes += boxes

    print(f"\nWrote {total_frames} frames, {total_boxes} boxes, under {args.output_dir}")
    print(
        f"Next: python training/model_eval/edit_labels.py {args.output_dir / args.jobs[0][0].stem}"
    )


if __name__ == "__main__":
    main()
