#!/usr/bin/env python3
"""Render side-by-side clips showing what each detection-alternatives experiment actually does.

The numbers in `docs/experiments/perception_performance/detection_alternatives_2026-09-02.md` say
which arm wins. These clips say why, on consecutive frames rather than the sparse scored ones.

    --experiment 1   RGB with the deployed detector's boxes coloured by whether the depth
                     prominence gate would keep them, beside the prominence map itself. The
                     failure to look for: boxes on arena furniture are bright (prominent) and
                     survive, boxes on the flat robots are dim and get dropped.
    --experiment 2   The deployed detector beside YOLOE prompted with exemplars from other
                     recordings. Out of the cage the deployed side should show extra boxes on
                     background that the prompted side does not.
    --experiment 3   Re-detect every 30 frames with a ViT tracker filling the gaps, beside the
                     same cadence with constant-velocity coasting. Frames where a detection
                     actually ran are marked.

Ground truth is drawn in white where a labelled frame happens to fall inside the window.

Frame index is the position in the recording's `/camera/image` stream, which is also the SVO
frame index (verified against `image_stream_index` on both venues). `svo_start_frame` in
`data.yaml` is not an offset into these MCAPs.

Usage:
    python playground/make_experiment_clips.py --experiment 1 --subdataset massd \
        --engine data/models/<detector>.engine -o clips/
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from auto_battlebot.floor_background import find_recording
from auto_battlebot.mcap_io import decode_compressed_image, iter_messages
from auto_battlebot.trt_yolo import TrtYoloModel
from playground.cache_gt_depth import resolve_svo
from playground.depth_gated_subtraction import box_prominent_fraction, depth_prominence

REPO = Path(__file__).resolve().parents[1]

CAMERA_IMAGE_TOPIC = "/camera/image"
FONT = cv2.FONT_HERSHEY_SIMPLEX

GT_BGR = (255, 255, 255)
KEEP_BGR = (80, 220, 80)
DROP_BGR = (60, 60, 235)
PLAIN_BGR = (0, 200, 255)
ALT_BGR = (235, 170, 60)

PANEL_W, PANEL_H = 960, 540
HEADER_H = 54
FPS = 15


def gt_boxes(subdataset: Path, stamp: int, size: tuple[int, int]) -> list[list[float]]:
    path = subdataset / "labels" / f"{stamp}.txt"
    if not path.exists():
        return []
    width, height = size
    rows = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cx, cy, box_w, box_h = (float(v) for v in parts[1:5])
        rows.append(
            [
                (cx - box_w / 2) * width,
                (cy - box_h / 2) * height,
                (cx + box_w / 2) * width,
                (cy + box_h / 2) * height,
            ]
        )
    return rows


def draw_box(canvas: np.ndarray, box, color, label: str = "", thickness: int = 2) -> None:
    x1, y1, x2, y2 = (int(v) for v in box)
    cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)
    if label:
        cv2.putText(canvas, label, (x1, max(14, y1 - 5)), FONT, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, label, (x1, max(14, y1 - 5)), FONT, 0.5, color, 1, cv2.LINE_AA)


def panel(image: np.ndarray, title: str, subtitle: str = "") -> np.ndarray:
    """Scale a frame to the panel size and put a titled header above it."""
    body = cv2.resize(image, (PANEL_W, PANEL_H))
    header = np.zeros((HEADER_H, PANEL_W, 3), np.uint8)
    header[:] = (28, 28, 28)
    cv2.putText(header, title, (12, 24), FONT, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
    if subtitle:
        cv2.putText(header, subtitle, (12, 44), FONT, 0.46, (170, 170, 170), 1, cv2.LINE_AA)
    return np.vstack([header, body])


def prominence_heatmap(prominence: np.ndarray, threshold: float) -> np.ndarray:
    """Prominence as colour, with unmeasured pixels left black and the gate contour drawn."""
    finite = np.isfinite(prominence)
    scaled = np.zeros(prominence.shape, np.uint8)
    scaled[finite] = np.clip(prominence[finite] / 0.9 * 255.0, 0, 255).astype(np.uint8)
    coloured = cv2.applyColorMap(scaled, cv2.COLORMAP_INFERNO)
    coloured[~finite] = (0, 0, 0)
    fires = (finite & (prominence > threshold)).astype(np.uint8) * 255
    contours, _ = cv2.findContours(fires, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(coloured, contours, -1, (255, 255, 255), 1)
    return coloured


def iter_window(mcap: Path, start: int, count: int):
    """Decoded frames [start, start+count) of the image stream, with their index and stamp."""
    for index, (_topic, _log, payload) in enumerate(iter_messages(mcap, [CAMERA_IMAGE_TOPIC])):
        if index < start:
            continue
        if index >= start + count:
            return
        message = decode_compressed_image(payload)
        yield index, message.stamp_ns, message.image


def open_svo(svo_path: Path, start_index: int):
    import pyzed.sl as sl  # type: ignore[import-untyped]

    init = sl.InitParameters()
    init.set_from_svo_file(str(svo_path))
    init.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
    init.coordinate_units = sl.UNIT.METER
    init.depth_minimum_distance = 0.3
    init.sdk_verbose = 0
    camera = sl.Camera()
    if camera.open(init) != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"Could not open {svo_path}")
    camera.set_svo_position(start_index)
    return camera, sl


def clip_experiment_1(args, subdataset: Path, mcap: Path, writer_for) -> None:
    """Detector boxes coloured by the prominence gate, beside the prominence map."""
    svo = resolve_svo(
        str(yaml.safe_load((subdataset / "data.yaml").read_text())["source_mcap"]),
        [Path("data/svo")],
    )
    if svo is None:
        raise SystemExit(f"No SVO for {subdataset.name}")
    camera, sl = open_svo(svo, args.start)
    runtime = sl.RuntimeParameters()
    depth_mat = sl.Mat()
    model = TrtYoloModel(args.engine, conf_threshold=args.conf, num_classes=2)

    writer = None
    for index, stamp, frame in iter_window(mcap, args.start, args.frames):
        camera.set_svo_position(index)
        if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue
        camera.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        size = (frame.shape[1], frame.shape[0])
        prominence, _ = depth_prominence(depth_mat.get_data(), size, args.window_px, 6.0)

        left = frame.copy()
        heat = prominence_heatmap(prominence, args.prominence)
        for box in gt_boxes(subdataset, stamp, size):
            draw_box(left, box, GT_BGR, "GT", 2)
            draw_box(heat, box, GT_BGR, "", 2)

        kept = dropped = 0
        for box, score, _cls, _kp in model.infer(frame):
            box = np.asarray(box, dtype=np.float64)
            fraction = box_prominent_fraction(prominence, box, args.prominence)
            keep = fraction >= args.box_fraction
            kept += keep
            dropped += not keep
            colour = KEEP_BGR if keep else DROP_BGR
            draw_box(left, box, colour, f"{score:.2f} prom {fraction:.2f}")
            draw_box(heat, box, colour, "")

        canvas = np.hstack(
            [
                panel(
                    left,
                    "Deployed detector, gated by depth prominence",
                    f"green kept ({kept})   red dropped ({dropped})   white GT",
                ),
                panel(
                    heat,
                    f"Depth prominence (NEURAL_PLUS), window {args.window_px}px",
                    f"bright = stands proud of surroundings   contour = > {args.prominence:.2f} m",
                ),
            ]
        )
        writer = writer or writer_for(canvas)
        writer.write(canvas)
    camera.close()
    if writer:
        writer.release()


def clip_experiment_2(args, subdataset: Path, mcap: Path, writer_for) -> None:
    """Deployed detector beside YOLOE prompted with exemplars from the other recordings."""
    from exemplar_detect import exemplar_embedding, pick_exemplars, prompted_model
    from ultralytics import YOLOE

    root = subdataset.parent
    subdatasets = sorted(d for d in root.iterdir() if (d / "data.yaml").exists())
    exemplars = pick_exemplars(subdatasets, subdataset, args.exemplars)
    owner = {p: o for o in subdatasets for p in exemplars if p.parent.parent == o}
    embedding = exemplar_embedding(
        YOLOE(args.weights, task="segment"), args.weights, exemplars, owner
    )
    yoloe = prompted_model(args.weights, embedding)
    sources = sorted({p.parent.parent.name[:22] for p in exemplars})
    model = TrtYoloModel(args.engine, conf_threshold=args.conf, num_classes=2)

    writer = None
    for _index, stamp, frame in iter_window(mcap, args.start, args.frames):
        size = (frame.shape[1], frame.shape[0])
        left, right = frame.copy(), frame.copy()
        for box in gt_boxes(subdataset, stamp, size):
            draw_box(left, box, GT_BGR, "GT", 2)
            draw_box(right, box, GT_BGR, "GT", 2)

        deployed = list(model.infer(frame))
        for box, score, _cls, _kp in deployed:
            draw_box(left, np.asarray(box, dtype=np.float64), PLAIN_BGR, f"{score:.2f}")

        subtitle_2 = f"{{}} boxes   prompted from {len(exemplars)} crops: {', '.join(sources[:3])}"
        result = yoloe.predict(frame, conf=args.yoloe_conf, verbose=False)[0]
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        for box, score in zip(boxes, scores):
            draw_box(right, box, ALT_BGR, f"{score:.2f}")

        canvas = np.hstack(
            [
                panel(left, f"Deployed YOLO26 (conf {args.conf})", f"{len(deployed)} boxes"),
                panel(
                    right,
                    f"YOLOE exemplar prompt (conf {args.yoloe_conf})",
                    subtitle_2.format(len(boxes)),
                ),
            ]
        )
        writer = writer or writer_for(canvas)
        writer.write(canvas)
    if writer:
        writer.release()


def draw_arm(frame, arm, boxes, colour, redetect: bool):
    """One arm's live tracks over the frame, with a green border on re-detect frames."""
    canvas = frame.copy()
    for box in boxes:
        draw_box(canvas, box, GT_BGR, "GT", 2)
    for track in arm.tracks:
        if track.alive:
            draw_box(canvas, track.box, colour, f"{track.score:.2f}")
    if redetect:
        cv2.rectangle(canvas, (2, 2), (canvas.shape[1] - 3, canvas.shape[0] - 3), KEEP_BGR, 6)
    return canvas


def clip_experiment_3(args, subdataset: Path, mcap: Path, writer_for) -> None:
    """Tracker gap-fill beside constant-velocity coasting, at the same re-detect cadence."""
    from tracker_gapfill import Arm

    model = TrtYoloModel(args.engine, conf_threshold=args.conf, num_classes=2)
    tracked = Arm(f"detect_every_{args.cadence}", args.cadence, args.tracker)
    coasted = Arm(f"coast_every_{args.cadence}", args.cadence, None)

    writer = None
    for index, stamp, frame in iter_window(mcap, args.start, args.frames):
        detections = sorted(
            ((np.asarray(b, dtype=np.float64), float(s)) for b, s, _c, _k in model.infer(frame)),
            key=lambda item: -item[1],
        )
        redetect = (index - args.start) % args.cadence == 0
        for arm in (tracked, coasted):
            if redetect:
                arm.redetect(frame, detections)
            else:
                arm.step(frame)

        since = (index - args.start) % args.cadence
        boxes = gt_boxes(subdataset, stamp, (frame.shape[1], frame.shape[0]))
        left = draw_arm(frame, tracked, boxes, PLAIN_BGR, redetect)
        right = draw_arm(frame, coasted, boxes, ALT_BGR, redetect)

        state = "DETECTION" if redetect else f"tracking, {since} frames since detect"
        canvas = np.hstack(
            [
                panel(left, f"ViT tracker, re-detect every {args.cadence} frames", state),
                panel(right, f"Constant-velocity coast, re-detect every {args.cadence}", state),
            ]
        )
        writer = writer or writer_for(canvas)
        writer.write(canvas)
    if writer:
        writer.release()


def densest_gt_window(subdataset: Path, frames: int) -> int:
    """Start index of the window holding the most labelled frames.

    GT frames were sampled where robots are actually visible, so the busiest stretch of ground
    truth is also the most informative stretch of fight to watch.
    """
    import json

    indices = []
    for path in sorted((subdataset / "camera_transforms").glob("*.json")):
        record = json.loads(path.read_text())
        if record.get("method") not in ("exact", "interpolated"):
            continue
        if (subdataset / "labels" / f"{record['stamp_ns']}.txt").exists():
            indices.append(int(record["image_stream_index"]))
    if not indices:
        raise SystemExit("No labelled frames with poses; pass --start explicitly")
    indices.sort()
    best_start, best_count = indices[0], 0
    for start in indices:
        count = sum(1 for i in indices if start <= i < start + frames)
        if count > best_count:
            best_start, best_count = start, count
    print(f"  densest window: {best_count} labelled frames from index {best_start}")
    return best_start


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--experiment", type=int, choices=(1, 2, 3), required=True)
    parser.add_argument("--gt", type=Path, default=Path("training/data/nhrl_keypoints_eval_test"))
    parser.add_argument("--subdataset", required=True, help="substring of the sub dataset name")
    parser.add_argument("--start", type=int, default=None, help="image index (default: first GT)")
    parser.add_argument("--frames", type=int, default=450)
    parser.add_argument("-o", "--output", type=Path, required=True, help="output mp4")
    parser.add_argument(
        "--engine",
        default="data/models/yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31_x86_64_sm89.engine",
    )
    parser.add_argument("--conf", type=float, default=0.6)
    parser.add_argument("--weights", default="data/eval_models/yoloe-26l-seg.pt")
    parser.add_argument("--yoloe-conf", type=float, default=0.15)
    parser.add_argument("--exemplars", type=int, default=8)
    parser.add_argument(
        "--tracker", default="data/eval_models/object_tracking_vittrack_2023sep.onnx"
    )
    parser.add_argument("--cadence", type=int, default=30)
    parser.add_argument("--prominence", type=float, default=0.20)
    parser.add_argument("--window-px", type=int, default=75)
    parser.add_argument("--box-fraction", type=float, default=0.20)
    parser.add_argument("--fps", type=int, default=FPS)
    args = parser.parse_args()

    matches = [
        d
        for d in sorted(args.gt.iterdir())
        if (d / "data.yaml").exists() and args.subdataset in d.name
    ]
    if len(matches) != 1:
        raise SystemExit(f"--subdataset matched {len(matches)} sub datasets, need exactly 1")
    subdataset = matches[0]
    data = yaml.safe_load((subdataset / "data.yaml").read_text())
    mcap = find_recording(str(data["source_mcap"]), [Path("data/saved_recordings")])

    if args.start is None:
        args.start = densest_gt_window(subdataset, args.frames)
    print(f"{subdataset.name}: frames {args.start}..{args.start + args.frames}")

    args.output.parent.mkdir(parents=True, exist_ok=True)

    def writer_for(canvas: np.ndarray):
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        return cv2.VideoWriter(
            str(args.output), fourcc, args.fps, (canvas.shape[1], canvas.shape[0])
        )

    runner = {1: clip_experiment_1, 2: clip_experiment_2, 3: clip_experiment_3}[args.experiment]
    runner(args, subdataset, mcap, writer_for)
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
