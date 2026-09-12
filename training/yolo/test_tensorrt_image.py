"""Run YOLO inference on still images using a TensorRT engine file.

Thin CLI around auto_battlebot.perception.trt_yolo, the same library the video CLI and
training/model_eval/score.py use, so detections match the C++ YoloKeypointModel path:
same letterbox preprocessing, same output decode, same per-class NMS.

Inputs accept files, directories, and shell-style globs. Quote a glob to let the script
expand it instead of the shell (needed for recursive '**' patterns).

Usage:
  python training/yolo/test_tensorrt_image.py model.engine frame.png -o out/
  python training/yolo/test_tensorrt_image.py model.engine 'renders/**/*.png' --num-classes 2
  python training/yolo/test_tensorrt_image.py model.engine images_dir/ --names data.yaml --show

Requires: tensorrt, pycuda, opencv-python, numpy
"""

from __future__ import annotations

import argparse
import glob
import time
from pathlib import Path

import cv2

from auto_battlebot.perception.detection_viz import draw_detections, load_class_names
from auto_battlebot.perception.trt_yolo import DetectionTuple, TrtYoloModel

IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(
        description="Run YOLO inference on still images using a TensorRT engine"
    )
    parser.add_argument("engine", type=str, help="Path to TensorRT engine file")
    parser.add_argument(
        "images",
        nargs="+",
        type=str,
        help="Image files, directories, or globs (quote globs to expand them here)",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="",
        type=str,
        help="Output directory for annotated images (default: do not save)",
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
        help="Number of classes (default: inferred from engine output, or from --names)",
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
        help="Display each annotated image; any key advances, q quits",
    )
    parser.add_argument(
        "--names",
        default="",
        type=str,
        help="Label boxes with class names: path to a data.yaml/yml or a comma-separated list",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Descend into subdirectories when an input is a directory",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Print one line per image instead of one line per detection",
    )
    return parser


def resolve_image_paths(specs: list[str], recursive: bool) -> list[Path]:
    """Expand files, directories, and globs into a sorted, deduplicated image list."""
    paths: list[Path] = []
    for spec in specs:
        path = Path(spec)
        if path.is_dir():
            pattern = "**/*" if recursive else "*"
            paths.extend(p for p in path.glob(pattern) if p.suffix.lower() in IMAGE_SUFFIXES)
        elif path.exists():
            paths.append(path)
        else:
            # Unexpanded glob (quoted, or no shell match). recursive=True costs nothing
            # when the pattern has no '**'.
            matches = [Path(m) for m in glob.glob(spec, recursive=True)]
            if not matches:
                raise FileNotFoundError(f"No images match: {spec}")
            paths.extend(m for m in matches if m.is_file() and m.suffix.lower() in IMAGE_SUFFIXES)
    return sorted(set(paths))


def report(path: Path, detections: list[DetectionTuple], quiet: bool) -> None:
    """Print the detections for one image."""
    print(f"{path}: {len(detections)} detections")
    if quiet:
        return
    for xyxy, conf, cls_id, kps in detections:
        x1, y1, x2, y2 = (int(v) for v in xyxy)
        visible = int((kps[:, 2] >= 0.5).sum()) if kps.size else 0
        print(
            f"  class {cls_id} conf {conf:.3f} box ({x1}, {y1}) ({x2}, {y2}) "
            f"keypoints {visible}/{kps.shape[0]}"
        )


def output_names(image_paths: list[Path]) -> dict[Path, str]:
    """Map each input to a unique annotated filename.

    A recursive glob often pulls the same stem from several directories (train/val
    splits, per-scene folders). Bare stems would overwrite each other, so a colliding
    name takes its parent directory as a prefix, then a counter if that still collides.
    """
    names: dict[Path, str] = {}
    used: set[str] = set()
    for path in image_paths:
        candidates = [path.stem, f"{path.parent.name}_{path.stem}"]
        name = next((c for c in candidates if c not in used), "")
        if not name:
            index = 2
            while f"{path.parent.name}_{path.stem}_{index}" in used:
                index += 1
            name = f"{path.parent.name}_{path.stem}_{index}"
        used.add(name)
        names[path] = f"{name}_det.png"
    return names


def process_images(
    image_paths: list[Path],
    model: TrtYoloModel,
    out_dir: Path | None,
    class_names: list[str] | None,
    args: argparse.Namespace,
) -> tuple[int, int, float]:
    """Infer on each image, then report, save, and show it.

    Returns (images processed, detections found, seconds spent in infer).
    """
    out_names = output_names(image_paths) if out_dir is not None else {}
    processed = 0
    total_detections = 0
    inference_s = 0.0
    for path in image_paths:
        frame = cv2.imread(str(path))
        if frame is None:
            print(f"{path}: unreadable, skipping")
            continue
        start = time.perf_counter()
        detections = model.infer(frame)
        inference_s += time.perf_counter() - start
        processed += 1
        total_detections += len(detections)
        report(path, detections, args.quiet)

        if out_dir is None and not args.show:
            continue
        annotated = draw_detections(frame, detections, class_names=class_names)
        if out_dir is not None:
            cv2.imwrite(str(out_dir / out_names[path]), annotated)
        if args.show:
            cv2.imshow("TensorRT YOLO", annotated)
            if cv2.waitKey(0) & 0xFF == ord("q"):
                print("Interrupted by user")
                break
    return processed, total_detections, inference_s


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    engine_path = Path(args.engine)
    if not engine_path.exists():
        raise FileNotFoundError(f"Engine not found: {engine_path}")
    image_paths = resolve_image_paths(args.images, args.recursive)
    print(f"Found {len(image_paths)} images")

    class_names = load_class_names(args.names)
    if class_names is not None:
        print(f"Labeling boxes with {len(class_names)} class names from --names")

    # Determine the class count before building the model. An explicit --num-classes wins;
    # otherwise fall back to the number of names from --names. A raw multi-class pose head
    # ([1, 4 + nc + 3*nk, anchors]) cannot be split correctly without it: the layout
    # inference would guess a single class, misread the extra class scores as keypoint
    # coordinates, and drop nearly every detection.
    resolved_num_classes = (
        args.num_classes if args.num_classes > 0 else (len(class_names) if class_names else 0)
    )

    print("Loading engine...")
    model = TrtYoloModel(
        str(engine_path),
        conf_threshold=args.conf,
        nms_iou_threshold=args.iou,
        num_classes=resolved_num_classes,
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

    out_dir = Path(args.output) if args.output else None
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)

    try:
        processed, total_detections, inference_s = process_images(
            image_paths, model, out_dir, class_names, args
        )
    finally:
        if args.show:
            cv2.destroyAllWindows()

    if processed:
        mean_ms = 1000.0 * inference_s / processed
        print(
            f"Done: {processed} images, {total_detections} detections, "
            f"{mean_ms:.1f} ms mean inference"
        )
    if out_dir is not None:
        print(f"Annotated images written to {out_dir}")


if __name__ == "__main__":
    main()
