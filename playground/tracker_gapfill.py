#!/usr/bin/env python3
"""Measure what a detect-every-N-frames architecture costs, by running a tracker in the gaps.

The proposal this grades: run the heavy detector every few seconds and a cheap tracker in
between. This does not ask "is the tracker good". It asks how much accuracy the cadence costs,
as a function of N, which is the number that decides whether the two-tier design is worth
building.

ByteTrack is deliberately not one of the arms. It associates per-frame detections through a
Kalman filter, so with detections every N frames it degenerates into constant-velocity coasting.
That is what `coast_every_30` measures, and it is the control the tracker has to beat. A method
that only matches coasting is not paying for its latency.

LiteTrack is what the plan named. Its weights are not published for direct download, so the arms
run OpenCV's ViT tracker instead: the same class of lightweight transformer template tracker,
runnable today, and the relative shape of the cadence curve is what transfers.

Frames come from the recording's `/camera/image`, in order, because a tracker needs consecutive
video. The eval dataset's frames are sparse samples from those recordings, so predictions are
recorded only when the stream reaches a labelled frame.

Usage:
    python playground/tracker_gapfill.py <gt> \
        --engine data/models/<detector>.engine \
        --tracker data/eval_models/object_tracking_vittrack_2023sep.onnx -o <preds dir>
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from auto_battlebot.floor_background import find_recording
from auto_battlebot.mcap_io import decode_compressed_image, iter_messages
from auto_battlebot.trt_yolo import TrtYoloModel

REPO = Path(__file__).resolve().parents[1]

CAMERA_IMAGE_TOPIC = "/camera/image"

# Recordings run at 30 fps, so these are 0 ms, 167 ms, 500 ms, 1.0 s and 3.0 s of coasting.
# 90 is the cadence the original proposal suggested.
CADENCES = (1, 5, 15, 30, 90)
COAST_CADENCE = 30

# A track whose tracker has lost confidence is dropped rather than held. Emitting a drifted box
# would flatter recall at the cost of precision and hide the failure this is meant to measure.
MIN_TRACK_SCORE = 0.20

# Two robots in frame, plus slack. Seeding more tracks than the arena can hold turns background
# false positives into permanent phantom tracks.
MAX_TRACKS = 3

# A re-detection is treated as the same object as an existing track above this IoU.
REASSOCIATE_IOU = 0.3


def iou(a: np.ndarray, b: np.ndarray) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    if x2 <= x1 or y2 <= y1:
        return 0.0
    inter = (x2 - x1) * (y2 - y1)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return float(inter / (area_a + area_b - inter + 1e-9))


def to_xywh(box: np.ndarray, size: tuple[int, int]) -> tuple[int, int, int, int]:
    width, height = size
    x1 = max(0, min(int(box[0]), width - 2))
    y1 = max(0, min(int(box[1]), height - 2))
    x2 = max(x1 + 2, min(int(box[2]), width - 1))
    y2 = max(y1 + 2, min(int(box[3]), height - 1))
    return x1, y1, x2 - x1, y2 - y1


class Track:
    """One tracked object: a template tracker plus the box it last reported."""

    def __init__(self, tracker_path: str, frame: np.ndarray, box: np.ndarray, score: float):
        params = cv2.TrackerVit_Params()  # type: ignore[attr-defined]
        params.net = tracker_path
        self.tracker = cv2.TrackerVit_create(params)  # type: ignore[attr-defined]
        self.tracker.init(frame, to_xywh(box, (frame.shape[1], frame.shape[0])))
        self.box = box.astype(np.float64).copy()
        self.score = score
        self.alive = True

    def update(self, frame: np.ndarray) -> None:
        ok, rect = self.tracker.update(frame)
        confidence = float(self.tracker.getTrackingScore())
        if not ok or confidence < MIN_TRACK_SCORE:
            self.alive = False
            return
        x, y, w, h = rect
        self.box = np.array([x, y, x + w, y + h], dtype=np.float64)
        self.score = confidence


class CoastTrack:
    """Constant-velocity hold, the control arm. No pixels, just the last two detections."""

    def __init__(self, box: np.ndarray, score: float):
        self.box = box.astype(np.float64).copy()
        self.velocity = np.zeros(4)
        self.score = score
        self.alive = True

    def reseed(self, box: np.ndarray, score: float) -> None:
        self.velocity = box.astype(np.float64) - self.box
        self.box = box.astype(np.float64).copy()
        self.score = score

    def update(self, _frame: np.ndarray) -> None:
        self.box = self.box + self.velocity


class Arm:
    """One cadence setting, holding its own tracks and its own re-detect schedule."""

    def __init__(self, name: str, cadence: int, tracker_path: str | None):
        self.name = name
        self.cadence = cadence
        self.tracker_path = tracker_path
        self.tracks: list = []
        self.predictions: dict[str, list[dict]] = {}
        self.tracker_seconds = 0.0
        self.tracker_steps = 0

    def redetect(self, frame: np.ndarray, detections: list[tuple[np.ndarray, float]]) -> None:
        """Replace the track set from a fresh detection.

        Existing tracks that a detection still agrees with are rebuilt on that detection rather
        than kept, so a track never drifts across a re-detect boundary.
        """
        kept = []
        for box, score in detections[:MAX_TRACKS]:
            if self.tracker_path is None:
                previous = next(
                    (t for t in self.tracks if iou(t.box, box) >= REASSOCIATE_IOU), None
                )
                track = previous if previous is not None else CoastTrack(box, score)
                track.reseed(box, score)
                track.alive = True
            else:
                track = Track(self.tracker_path, frame, box, score)
            kept.append(track)
        self.tracks = kept

    def step(self, frame: np.ndarray) -> None:
        start = time.perf_counter()
        for track in self.tracks:
            if track.alive:
                track.update(frame)
        self.tracker_seconds += time.perf_counter() - start
        self.tracker_steps += max(1, len(self.tracks))
        self.tracks = [t for t in self.tracks if t.alive]

    def record(self, stem: str) -> None:
        self.predictions[stem] = [
            {
                "xyxy": [float(v) for v in track.box],
                "score": round(float(min(1.0, max(0.0, track.score))), 4),
                "class_id": 0,
            }
            for track in self.tracks
            if track.alive
        ]


def run_recording(
    subdataset: Path,
    recordings: list[Path],
    model: TrtYoloModel,
    tracker_path: str,
    arms: list[Arm],
) -> dict:
    """Step the recording once, driving every arm from the same decoded frames."""
    data = yaml.safe_load((subdataset / "data.yaml").read_text())
    mcap = find_recording(str(data["source_mcap"]), recordings)

    gt_stems = {
        path.stem
        for path in (subdataset / "images").glob("*.png")
        if (subdataset / "labels" / f"{path.stem}.txt").exists()
    }
    if not gt_stems:
        return {"error": "no labelled frames"}

    gt_stamps = {int(stem) for stem in gt_stems}
    last_stamp = max(gt_stamps)

    for arm in arms:
        arm.tracks = []

    detect_seconds = 0.0
    detect_calls = 0
    index = 0
    matched = 0

    for _topic, _log_time, payload in iter_messages(mcap, [CAMERA_IMAGE_TOPIC]):
        message = decode_compressed_image(payload)
        frame = message.image
        stamp = message.stamp_ns

        # One detection per frame, shared by every arm that re-detects here. detect_every_1
        # needs it on every frame anyway, so nothing is wasted.
        start = time.perf_counter()
        raw = model.infer(frame)
        detect_seconds += time.perf_counter() - start
        detect_calls += 1
        detections = sorted(
            ((np.asarray(box, dtype=np.float64), float(score)) for box, score, _cls, _kp in raw),
            key=lambda item: -item[1],
        )

        for arm in arms:
            if index % arm.cadence == 0:
                arm.redetect(frame, detections)
            else:
                arm.step(frame)

        if stamp in gt_stamps:
            matched += 1
            for arm in arms:
                arm.record(str(stamp))

        index += 1
        if stamp >= last_stamp:
            break

    return {
        "frames_stepped": index,
        "gt_frames_matched": matched,
        "gt_frames_expected": len(gt_stems),
        "detect_ms_mean": round(1000 * detect_seconds / max(1, detect_calls), 2),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="eval dataset root")
    parser.add_argument("--engine", required=True, help="detector TensorRT engine")
    parser.add_argument("--tracker", required=True, help="ViT tracker ONNX")
    parser.add_argument(
        "--recordings", type=Path, nargs="+", default=[Path("data/saved_recordings")]
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--conf", type=float, default=0.6, help="detector threshold")
    parser.add_argument("--nms-iou", type=float, default=0.45)
    parser.add_argument(
        "--only", nargs="*", default=None, help="sub dataset name substrings to limit the run to"
    )
    args = parser.parse_args()

    model = TrtYoloModel(
        args.engine, conf_threshold=args.conf, nms_iou_threshold=args.nms_iou, num_classes=2
    )

    arms = [Arm(f"detect_every_{n}", n, args.tracker) for n in CADENCES]
    arms.append(Arm(f"coast_every_{COAST_CADENCE}", COAST_CADENCE, None))

    subdatasets = sorted(d for d in args.gt.iterdir() if (d / "data.yaml").exists())
    if args.only:
        subdatasets = [d for d in subdatasets if any(s in d.name for s in args.only)]

    report: dict[str, dict] = {}
    for subdataset in subdatasets:
        print(f"{subdataset.name} ...", flush=True)
        report[subdataset.name] = run_recording(
            subdataset, list(args.recordings), model, args.tracker, arms
        )
        print(f"  {report[subdataset.name]}")

    args.output.mkdir(parents=True, exist_ok=True)
    for arm in arms:
        (args.output / f"{arm.name}.json").write_text(
            json.dumps({"labels": ["opponent"], "frames": arm.predictions})
        )
        report.setdefault("_timing", {})[arm.name] = {
            "tracker_ms_per_object_step": round(
                1000 * arm.tracker_seconds / max(1, arm.tracker_steps), 3
            ),
            "frames_predicted": len(arm.predictions),
        }
    (args.output / "run_report.json").write_text(json.dumps(report, indent=2))
    print(f"Wrote {len(arms)} arms to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
