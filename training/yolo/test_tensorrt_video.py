"""Run YOLO pose inference on a video using a TensorRT engine file."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm

from auto_battlebot.perception.detection_viz import draw_detections, load_class_names
from auto_battlebot.perception.trt_yolo import TrtYoloModel, resolve_num_classes


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(
        description="Run YOLO pose inference on video using a TensorRT engine"
    )
    parser.add_argument("engine", type=str, help="Path to TensorRT engine file")
    parser.add_argument("video", type=str, help="Path to input video")
    parser.add_argument(
        "-o",
        "--output",
        default="",
        type=str,
        help="Output video path (default: <video_stem>_trt_annotated.mp4)",
    )
    parser.add_argument(
        "-c",
        "--conf",
        default=0.5,
        type=float,
        help="Confidence threshold (default: 0.5)",
    )
    parser.add_argument(
        "--iou",
        default=0.45,
        type=float,
        help="NMS IoU threshold (default: 0.45)",
    )
    parser.add_argument(
        "--imgsz",
        default=0,
        type=int,
        help="Input size H=W (default: from engine)",
    )
    parser.add_argument(
        "--num-classes",
        default=0,
        type=int,
        help="Number of classes (default: inferred from engine output)",
    )
    parser.add_argument(
        "--num-keypoints",
        default=None,
        type=int,
        help="Number of keypoints per detection (default: inferred)",
    )
    parser.add_argument(
        "--bbox-half-wh",
        action="store_true",
        help="Treat bbox 3rd/4th as half-width/half-height (x2=cx+w not cx+w/2)",
    )
    parser.add_argument(
        "--swap-wh",
        action="store_true",
        help="Swap bbox 3rd/4th (use as height, width instead of width, height)",
    )
    parser.add_argument(
        "--bbox-xyxy",
        action="store_true",
        help=(
            "Treat raw bbox output as already-decoded xyxy (x1, y1, x2, y2) and skip "
            "conversion.  By default the script converts from Ultralytics-standard "
            "cx, cy, w, h (center format) to xyxy.  Only pass this flag if your model "
            "bakes the decoding step into its output."
        ),
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show video while processing",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Do not save output video",
    )
    parser.add_argument(
        "--names",
        default="",
        type=str,
        help="Label boxes with class names: path to a data.yaml/yml or a comma-separated list",
    )
    parser.add_argument(
        "--max-frames",
        default=0,
        type=int,
        help="Stop after this many frames (default: 0 = whole video)",
    )
    return parser


def open_video(video_path: Path) -> tuple[cv2.VideoCapture, int, int, int, int]:
    """Open the video and return (cap, width, height, fps, total_frames)."""
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
    return cap, width, height, fps, total_frames


def _write_and_show(
    annotated: np.ndarray, writer: cv2.VideoWriter | None, args: argparse.Namespace
) -> bool:
    """Write the annotated frame and optionally display it. Returns True to stop processing."""
    if writer is not None:
        writer.write(annotated)
    if args.show:
        cv2.imshow("TensorRT YOLO", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("Interrupted by user")
            return True
    return False


def process_video(
    cap: cv2.VideoCapture,
    writer: cv2.VideoWriter | None,
    model: TrtYoloModel,
    total_frames: int,
    args: argparse.Namespace,
    class_names: list[str] | None,
) -> tuple[int, float]:
    """Run inference over every frame, draw, and write/show. Returns (frame_count, elapsed)."""
    start_time = time.time()
    frame_count = 0
    limit = args.max_frames if args.max_frames > 0 else None
    if limit is not None and (total_frames <= 0 or limit < total_frames):
        total_frames = limit
    try:
        pbar = tqdm(
            total=total_frames if total_frames > 0 else None,
            desc="Processing",
            unit="frame",
        )
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            detections = model.infer(frame)
            annotated = draw_detections(frame, detections, class_names=class_names)
            if _write_and_show(annotated, writer, args):
                break
            frame_count += 1
            elapsed = time.time() - start_time
            pbar.set_postfix({"FPS": f"{frame_count / elapsed:.1f}" if elapsed > 0 else "0"})
            pbar.update(1)
            if limit is not None and frame_count >= limit:
                break
        pbar.close()
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()

    elapsed = time.time() - start_time
    return frame_count, elapsed


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    video_path = Path(args.video)
    engine_path = Path(args.engine)
    if not video_path.exists():
        raise FileNotFoundError(f"Video not found: {video_path}")
    if not engine_path.exists():
        raise FileNotFoundError(f"Engine not found: {engine_path}")

    class_names = load_class_names(args.names)
    if class_names is not None:
        print(f"Labeling boxes with {len(class_names)} class names from --names")

    resolved_num_classes = resolve_num_classes(args.num_classes, class_names)

    print("Loading engine...")
    model = TrtYoloModel(
        str(engine_path),
        conf_threshold=args.conf,
        nms_iou_threshold=args.iou,
        num_classes=resolved_num_classes,
        num_keypoints=args.num_keypoints if (args.num_keypoints or 0) > 0 else -1,
        letterbox_padding=0.0,
        bbox_half_wh=args.bbox_half_wh,
        swap_wh=args.swap_wh,
        bbox_xyxy=args.bbox_xyxy,
    )
    if args.imgsz > 0 and (args.imgsz != model.input_h or args.imgsz != model.input_w):
        raise RuntimeError(
            f"Engine has fixed input shape {model.input_h}x{model.input_w}; "
            "--imgsz must match or be 0"
        )
    print(f"Model: {model.describe()}")

    cap, width, height, fps, total_frames = open_video(video_path)
    print(f"Video: {width}x{height} @ {fps} FPS, {total_frames} frames")

    if args.output:
        output_path = Path(args.output)
    else:
        output_path = video_path.parent / f"{video_path.stem}_trt_annotated.mp4"
    writer = None
    if not args.no_save:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
        print(f"Saving to {output_path}")

    frame_count, elapsed = process_video(cap, writer, model, total_frames, args, class_names)

    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    print(f"Processed {frame_count} frames in {elapsed:.2f}s, avg FPS: {avg_fps:.2f}")
    if not args.no_save:
        print(f"Output: {output_path}")


if __name__ == "__main__":
    main()
