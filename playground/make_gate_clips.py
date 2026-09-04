#!/usr/bin/env python3
"""Render one clip per recording showing all three false-positive gates side by side.

`logo_false_positive_2026-09-03.md` scores the gates on sparse ground-truth frames. The scores say
which gate wins; they do not show what any of them is looking at, or why one of them rates a flat
painted banner as maximally prominent. These clips run consecutive frames so both are visible.

Four panels per frame, on the same detector boxes:

    top left      the frame, ground truth in white, every detector box labelled with all three
                  fractions so a box can be traced across the other panels
    top right     background-subtraction foreground, boxes coloured by the RGB gate
    bottom left   height above the fitted field plane, boxes coloured by the height gate
    bottom right  depth prominence, boxes coloured by the prominence gate

Green is kept, red is dropped, and the header of each gate panel carries its running counts.

What to look for. On MassD the floor banner is a persistent box on the arena floor: the RGB panel
should show nothing there (it has been in the background since the field init) while the prominence
panel lights it up bright, because the banner lies against the arena wall and the max filter reaches
over the wall to the room beyond. The height panel should leave it dark, since it sits on the plane
rather than above it. On the May recordings the failure runs the other way: the robots are dark on a
dark floor, so the RGB panel is faint over real robots and the gate drops boxes it should keep.

The SVO is decoded strictly sequentially. Seeking lands on non-keyframes and smears both the image
and the depth, and every frame in the window is wanted anyway.

Usage:
    python playground/make_gate_clips.py --engine data/models/<detector>.engine -o clips/
    python playground/make_gate_clips.py --subdataset massd --frames 600 -o clips/
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from auto_battlebot.background_subtraction import SubtractionParams, subtract, warp_forward
from auto_battlebot.camera_geometry import NOMINAL_FIELD_SIZE_M, load_camera_info
from auto_battlebot.floor_background import (
    FLOOR_MARGIN_M,
    RASTER_PX_PER_M,
    FloorRaster,
    PoseGeometry,
    build_floor_background,
    find_recording,
    read_recording,
)
from auto_battlebot.mcap_io import decode_compressed_image, iter_messages
from auto_battlebot.trt_yolo import TrtYoloModel
from playground.cache_gt_depth import resolve_svo
from playground.depth_gated_subtraction import (
    DEFAULT_BOX_FOREGROUND_FRACTION,
    DEFAULT_BOX_HEIGHT_FRACTION,
    DEFAULT_BOX_PROMINENT_FRACTION,
    DEFAULT_HEIGHT_BAND_M,
    DEFAULT_PROMINENCE_M,
    DEFAULT_WINDOW_PX,
    MAX_RANGE_M,
    box_band_fraction,
    box_foreground_fraction,
    box_prominent_fraction,
    depth_prominence,
    field_height,
)

CAMERA_IMAGE_TOPIC = "/camera/image"
FONT = cv2.FONT_HERSHEY_SIMPLEX
GT_BGR = (255, 255, 255)
KEEP_BGR = (80, 220, 80)
DROP_BGR = (60, 60, 235)
PANEL_W, PANEL_H = 640, 360
HEADER_H = 48
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
        cv2.putText(canvas, label, (x1, max(14, y1 - 5)), FONT, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, label, (x1, max(14, y1 - 5)), FONT, 0.45, color, 1, cv2.LINE_AA)


def panel(image: np.ndarray, title: str, subtitle: str = "") -> np.ndarray:
    body = cv2.resize(image, (PANEL_W, PANEL_H))
    header = np.zeros((HEADER_H, PANEL_W, 3), np.uint8)
    header[:] = (28, 28, 28)
    cv2.putText(header, title, (10, 20), FONT, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
    if subtitle:
        cv2.putText(header, subtitle, (10, 38), FONT, 0.42, (170, 170, 170), 1, cv2.LINE_AA)
    return np.vstack([header, body])


def heatmap(values: np.ndarray, span: float, fires: np.ndarray) -> np.ndarray:
    """A float map as colour, unmeasured left black, with the gate's own region outlined."""
    finite = np.isfinite(values)
    scaled = np.zeros(values.shape, np.uint8)
    scaled[finite] = np.clip(values[finite] / span * 255.0, 0, 255).astype(np.uint8)
    coloured = cv2.applyColorMap(scaled, cv2.COLORMAP_INFERNO)
    coloured[~finite] = (0, 0, 0)
    contours, _ = cv2.findContours(
        fires.astype(np.uint8) * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    cv2.drawContours(coloured, contours, -1, (255, 255, 255), 1)
    return coloured


def pose_by_frame(recording) -> dict[int, np.ndarray]:
    """{image frame index: field_from_camera}, for the ticks the pipeline actually processed."""
    out: dict[int, np.ndarray] = {}
    for tick, (stamp, _matrix) in enumerate(recording.dynamic):
        index = recording.frame_index(stamp)
        if index >= 0:
            out[index] = recording.field_from_camera(tick)
    return out


def busiest_gt_frame(subdataset: Path, predictions: Path | None) -> int | None:
    """The SVO index of the ground-truth frame carrying the most false positives.

    That frame is where a gate has the most to prove, so it is the most informative place to
    start a clip. Falls back to the first labelled frame when no predictions are supplied.
    """
    transforms = sorted((subdataset / "camera_transforms").glob("*.json"))
    if not transforms:
        return None
    index_of = {}
    for path in transforms:
        record = json.loads(path.read_text())
        if record.get("method") in ("exact", "interpolated"):
            index_of[str(record["stamp_ns"])] = int(record["image_stream_index"])
    if not index_of:
        return None

    if predictions is not None and predictions.exists():
        payload = json.loads(predictions.read_text())
        best_stamp, best_count = None, -1
        for stamp, rows in payload.get("frames", {}).items():
            if stamp not in index_of:
                continue
            truth = gt_boxes(subdataset, int(stamp), (1, 1))
            # Cheap stand-in for a match: more detections than labels means false positives.
            excess = len(rows) - len(truth)
            if excess > best_count:
                best_stamp, best_count = stamp, excess
        if best_stamp is not None and best_count > 0:
            return index_of[best_stamp]
    return min(index_of.values())


def render(subdataset: Path, args: argparse.Namespace, model: TrtYoloModel) -> Path | None:
    data = yaml.safe_load((subdataset / "data.yaml").read_text())
    mcap = find_recording(str(data["source_mcap"]), list(args.recordings))
    svo = resolve_svo(str(data["source_mcap"]), list(args.svo_roots))
    if svo is None:
        print(f"  {subdataset.name}: no SVO, skipped")
        return None

    recording = read_recording(mcap)
    poses = pose_by_frame(recording)
    if not poses:
        print(f"  {subdataset.name}: no pose chain, skipped")
        return None

    predictions = args.predictions / "yolo_only.json" if args.predictions else None
    start = args.start if args.start is not None else busiest_gt_frame(subdataset, predictions)
    if start is None:
        print(f"  {subdataset.name}: no usable start frame, skipped")
        return None
    start = max(0, start - args.lead)

    intrinsics = load_camera_info(subdataset)
    if intrinsics is None:
        print(f"  {subdataset.name}: no camera_info.json, skipped")
        return None

    raster = FloorRaster(NOMINAL_FIELD_SIZE_M, RASTER_PX_PER_M, FLOOR_MARGIN_M)
    background, coverage = build_floor_background(
        recording, raster, args.background_samples, intrinsics
    )
    compare_mask = cv2.bitwise_and(raster.floor_mask, coverage)
    params = SubtractionParams(
        threshold=args.threshold,
        illumination=args.illumination,
        edge_tolerance=args.edge_tolerance,
        min_area=400,
    )

    import pyzed.sl as sl  # type: ignore[import-untyped]

    init = sl.InitParameters()
    init.set_from_svo_file(str(svo))
    init.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
    init.coordinate_units = sl.UNIT.METER
    init.depth_minimum_distance = 0.3
    init.sdk_verbose = 0
    zed = sl.Camera()
    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        print(f"  {subdataset.name}: could not open {svo.name}, skipped")
        return None
    # One seek to the window, then strictly sequential grabs.
    zed.set_svo_position(start)
    runtime = sl.RuntimeParameters()
    depth_mat = sl.Mat()

    out_path = args.output / f"{subdataset.name}.mp4"
    args.output.mkdir(parents=True, exist_ok=True)
    writer = None
    held = None
    totals = {"rgb": [0, 0], "height": [0, 0], "prom": [0, 0]}

    for index, stamp, frame in iter_window(mcap, start, args.frames):
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            break
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        depth = depth_mat.get_data()
        size = (frame.shape[1], frame.shape[0])

        pose = poses.get(index, held)
        if pose is None:
            continue
        stale = index not in poses
        held = pose
        geometry = PoseGeometry(pose, intrinsics)

        homography = raster.image_from_raster(geometry)
        warped = warp_forward(background, homography, size)
        valid = warp_forward(compare_mask, homography, size, nearest=True)
        valid = cv2.bitwise_and(valid, raster.in_front_mask(homography, size))
        valid = cv2.erode(valid, np.ones((11, 11), np.uint8))
        _, foreground = subtract(frame, warped, valid, params)

        prominence, _ = depth_prominence(depth, size, args.window_px, MAX_RANGE_M)
        height, _ = field_height(depth, size, geometry)
        band = tuple(args.height_band)

        rgb_panel = cv2.cvtColor(foreground, cv2.COLOR_GRAY2BGR)
        height_panel = heatmap(
            height, band[1] * 2.0, np.isfinite(height) & (height >= band[0]) & (height <= band[1])
        )
        prom_panel = heatmap(
            prominence, 0.9, np.isfinite(prominence) & (prominence > args.prominence)
        )
        left = frame.copy()

        truth = gt_boxes(subdataset, stamp, size)
        for box in truth:
            for canvas in (left, rgb_panel, height_panel, prom_panel):
                draw_box(canvas, box, GT_BGR, "", 2)

        counts = {"rgb": [0, 0], "height": [0, 0], "prom": [0, 0]}
        for box, score, _cls, _kp in model.infer(frame):
            box = np.asarray(box, dtype=np.float64)
            fractions = {
                "rgb": box_foreground_fraction(foreground, box),
                "height": box_band_fraction(height, box, band),
                "prom": box_prominent_fraction(prominence, box, args.prominence),
            }
            passes = {
                "rgb": fractions["rgb"] >= args.box_rgb_fraction,
                "height": fractions["height"] >= args.box_height_fraction,
                "prom": fractions["prom"] >= args.box_fraction,
            }
            for key, ok in passes.items():
                counts[key][0 if ok else 1] += 1
                totals[key][0 if ok else 1] += 1
            draw_box(
                left,
                box,
                (200, 200, 60),
                f"{score:.2f} fg{fractions['rgb']:.2f} h{fractions['height']:.2f} "
                f"p{fractions['prom']:.2f}",
            )
            for key, canvas in (("rgb", rgb_panel), ("height", height_panel), ("prom", prom_panel)):
                draw_box(canvas, box, KEEP_BGR if passes[key] else DROP_BGR, "")

        tag = "  POSE HELD" if stale else ""
        canvas = np.vstack(
            [
                np.hstack(
                    [
                        panel(
                            left,
                            f"{subdataset.name[:44]}  frame {index}{tag}",
                            f"white GT ({len(truth)})   yellow: every detector box, "
                            f"fg / height / prominence",
                        ),
                        panel(
                            rgb_panel,
                            f"Background subtraction  >= {args.box_rgb_fraction:.2f} of box",
                            f"kept {counts['rgb'][0]}  dropped {counts['rgb'][1]}   "
                            f"white = differs from the floor median",
                        ),
                    ]
                ),
                np.hstack(
                    [
                        panel(
                            height_panel,
                            f"Height above field plane  band {band[0]:.2f}-{band[1]:.2f} m",
                            f"kept {counts['height'][0]}  dropped {counts['height'][1]}   "
                            f"contour = inside the band",
                        ),
                        panel(
                            prom_panel,
                            f"Depth prominence  > {args.prominence:.2f} m",
                            f"kept {counts['prom'][0]}  dropped {counts['prom'][1]}   "
                            f"bright = stands proud of surroundings",
                        ),
                    ]
                ),
            ]
        )
        if writer is None:
            writer = cv2.VideoWriter(
                str(out_path),
                cv2.VideoWriter_fourcc(*"mp4v"),
                args.fps,
                (canvas.shape[1], canvas.shape[0]),
            )
        writer.write(canvas)

    zed.close()
    if writer is None:
        print(f"  {subdataset.name}: no frames rendered")
        return None
    writer.release()
    summary = "  ".join(
        f"{key} kept {v[0]}/{v[0] + v[1]}" for key, v in totals.items() if v[0] + v[1]
    )
    print(f"  {subdataset.name}: from frame {start}  {summary}")
    return out_path


def iter_window(mcap: Path, start: int, count: int):
    """Decoded frames [start, start+count) of the image stream, with their index and stamp."""
    for index, (_topic, _log, payload) in enumerate(iter_messages(mcap, [CAMERA_IMAGE_TOPIC])):
        if index < start:
            continue
        if index >= start + count:
            return
        message = decode_compressed_image(payload)
        yield index, message.stamp_ns, message.image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--gt", type=Path, default=Path("training/data/nhrl_keypoints_eval_test"))
    parser.add_argument(
        "--subdataset", default=None, help="substring filter; default renders every recording"
    )
    parser.add_argument(
        "--engine",
        default="data/models/yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31_x86_64_sm89.engine",
    )
    parser.add_argument("--conf", type=float, default=0.6)
    parser.add_argument("-o", "--output", type=Path, required=True, help="dir for the clips")
    parser.add_argument(
        "--predictions",
        type=Path,
        default=None,
        help="arm dir whose yolo_only.json picks the busiest ground-truth frame to start on",
    )
    parser.add_argument("--start", type=int, default=None, help="SVO index; overrides the pick")
    parser.add_argument("--lead", type=int, default=60, help="frames to start before that frame")
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument(
        "--skip-existing", action="store_true", help="leave clips already in the output dir"
    )
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument(
        "--recordings",
        type=Path,
        nargs="+",
        default=[Path("data/recordings"), Path("data/saved_recordings")],
    )
    parser.add_argument("--svo-roots", type=Path, nargs="+", default=[Path("data/svo")])
    parser.add_argument("--background-samples", type=int, default=160)
    parser.add_argument("--threshold", type=int, default=35)
    parser.add_argument("--edge-tolerance", type=float, default=0.5)
    parser.add_argument("--illumination", choices=("local", "global", "none"), default="local")
    parser.add_argument("--prominence", type=float, default=DEFAULT_PROMINENCE_M)
    parser.add_argument("--window-px", type=int, default=DEFAULT_WINDOW_PX)
    parser.add_argument("--height-band", type=float, nargs=2, default=list(DEFAULT_HEIGHT_BAND_M))
    parser.add_argument("--box-fraction", type=float, default=DEFAULT_BOX_PROMINENT_FRACTION)
    parser.add_argument("--box-rgb-fraction", type=float, default=DEFAULT_BOX_FOREGROUND_FRACTION)
    parser.add_argument("--box-height-fraction", type=float, default=DEFAULT_BOX_HEIGHT_FRACTION)
    args = parser.parse_args()

    subdatasets = sorted(d for d in args.gt.iterdir() if (d / "data.yaml").exists())
    if args.subdataset:
        subdatasets = [d for d in subdatasets if args.subdataset in d.name]
    if not subdatasets:
        raise SystemExit(f"No sub datasets matched under {args.gt}")

    model = TrtYoloModel(args.engine, conf_threshold=args.conf, num_classes=2)
    written = []
    for subdataset in subdatasets:
        out_path = args.output / f"{subdataset.name}.mp4"
        if args.skip_existing and out_path.exists():
            print(f"  {subdataset.name}: already rendered, skipped")
            written.append(out_path)
            continue
        # One recording that cannot be read must not take the rest of the batch down with it.
        try:
            path = render(subdataset, args, model)
        except BaseException as error:  # noqa: BLE001
            print(f"  {subdataset.name}: FAILED ({type(error).__name__}: {error})", flush=True)
            continue
        if path is not None:
            written.append(path)

    print(f"\nWrote {len(written)} clips to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
