"""Run YOLO pose inference on a video using a TensorRT engine file.

Thin CLI around auto_battlebot.trt_yolo (shared with training/model_eval/score.py).
Matches the C++ YoloKeypointModel pipeline: same preprocessing (letterbox, BGR->RGB,
normalize), same output layout [1, num_features, num_predictions], and same NMS/postprocess.

Expects engines built from ONNX models exported with end2end disabled (the default
in convert_to_onnx.py). The model output has sigmoid already applied to class scores
and keypoint visibility, and bboxes are in xyxy format. If raw logits are detected
(values outside [0,1]), sigmoid is applied as a fallback.

Usage:
  python training/yolo/test_tensorrt_video.py path/to/model.engine path/to/video.mp4 -o out.mp4
  python training/yolo/test_tensorrt_video.py model.engine video.mp4 --show --conf 0.5

Requires: tensorrt, pycuda, opencv-python, numpy, tqdm
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from tqdm import tqdm

from auto_battlebot.trt_yolo import DetectionTuple, TrtYoloModel


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
    colors = [
        (0, 0, 255),
        (0, 255, 0),
        (255, 0, 0),
        (255, 255, 0),
        (255, 0, 255),
        (0, 255, 255),
    ]
    for xyxy, conf, cls_id, kps in detections:
        x1, y1, x2, y2 = map(int, xyxy)
        color = colors[cls_id % len(colors)]
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
        default=0,
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

    print("Loading engine...")
    model = TrtYoloModel(
        str(engine_path),
        conf_threshold=args.conf,
        nms_iou_threshold=args.iou,
        num_classes=args.num_classes,
        # CLI 0 means "infer from the engine layout"; the library sentinel for that is -1
        # (0 there means "no keypoints", used for seg engines).
        num_keypoints=args.num_keypoints if args.num_keypoints > 0 else -1,
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

    class_names = load_class_names(args.names)
    if class_names is not None:
        print(f"Labeling boxes with {len(class_names)} class names from --names")

    frame_count, elapsed = process_video(cap, writer, model, total_frames, args, class_names)

    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    print(f"Processed {frame_count} frames in {elapsed:.2f}s, avg FPS: {avg_fps:.2f}")
    if not args.no_save:
        print(f"Output: {output_path}")


if __name__ == "__main__":
    main()
