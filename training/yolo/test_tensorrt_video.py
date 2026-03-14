"""Run YOLO pose inference on a video using a TensorRT engine file.

Matches the C++ YoloKeypointModel pipeline: same preprocessing (letterbox, BGR->RGB,
normalize), same output layout [1, num_features, num_predictions], and same NMS/postprocess.

Expects engines built from ONNX models exported with end2end disabled (the default
in convert_to_onnx.py). The model output has sigmoid already applied to class scores
and keypoint visibility, and bboxes are in xyxy format. If raw logits are detected
(values outside [0,1]), sigmoid is applied as a fallback.

Usage:
  python training/yolo/test_tensorrt_video.py path/to/video.mp4 path/to/model.engine -o out.mp4
  python training/yolo/test_tensorrt_video.py video.mp4 model.engine --show --conf 0.5

Requires: tensorrt, pycuda, opencv-python, numpy, tqdm
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm

import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # noqa: F401


def letterbox(
    image: np.ndarray,
    target_h: int,
    target_w: int,
    pad_val: float = 114.0,
    padding: float = 0.0,
) -> tuple[np.ndarray, float, float, float]:
    """Resize with aspect ratio and pad to target size. Returns (padded image, scale, pad_left, pad_top)."""
    h, w = image.shape[:2]
    scale = min(target_h / h, target_w / w)
    new_w = int(round(w * scale))
    new_h = int(round(h * scale))
    if new_w == target_w and new_h == target_h:
        if image.shape[0] == target_h and image.shape[1] == target_w:
            return image, scale, 0.0, 0.0
        resized = cv2.resize(
            image, (target_w, target_h), interpolation=cv2.INTER_LINEAR
        )
        return resized, scale, 0.0, 0.0
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    left = int(round((target_w - new_w) / 2.0 - padding))
    top = int(round((target_h - new_h) / 2.0 - padding))
    left = max(0, left)
    top = max(0, top)
    right = target_w - new_w - left
    bottom = target_h - new_h - top
    right = max(0, right)
    bottom = max(0, bottom)
    out = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(pad_val,) * 3
    )
    return out, scale, float(left), float(top)


def preprocess_frame(
    frame: np.ndarray,
    input_h: int,
    input_w: int,
    letterbox_padding: float = 0.0,
) -> tuple[np.ndarray, float, float, float]:
    """BGR frame -> NCHW float32 [0,1] RGB, plus scale and pad for inverse transform."""
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    padded, scale, pad_left, pad_top = letterbox(
        rgb, input_h, input_w, padding=letterbox_padding
    )
    blob = padded.astype(np.float32) / 255.0
    blob = np.transpose(blob, (2, 0, 1))
    blob = np.expand_dims(blob, axis=0)
    return blob, scale, pad_left, pad_top


def sigmoid(x: np.ndarray) -> np.ndarray:
    """Numerically stable sigmoid. Fallback for models that output raw logits."""
    return np.where(x >= 0, 1.0 / (1.0 + np.exp(-x)), np.exp(x) / (1.0 + np.exp(x)))


def xywh2xyxy(boxes: np.ndarray, half_wh: bool = False) -> None:
    """In-place: (cx, cy, w, h) -> (x1, y1, x2, y2). If half_wh, 3rd/4th are half-width/half-height (no /2)."""
    cx, cy = boxes[:, 0].copy(), boxes[:, 1].copy()
    w, h = boxes[:, 2].copy(), boxes[:, 3].copy()
    if half_wh:
        boxes[:, 0] = cx - w
        boxes[:, 1] = cy - h
        boxes[:, 2] = cx + w
        boxes[:, 3] = cy + h
    else:
        boxes[:, 0] = cx - w / 2.0
        boxes[:, 1] = cy - h / 2.0
        boxes[:, 2] = cx + w / 2.0
        boxes[:, 3] = cy + h / 2.0


def nms(
    bboxes: np.ndarray,
    scores: np.ndarray,
    iou_threshold: float,
) -> np.ndarray:
    """Return indices to keep. bboxes: (N, 4) x1,y1,x2,y2; scores: (N,)."""
    if bboxes.size == 0:
        return np.array([], dtype=np.int64)
    order = np.argsort(-scores)
    keep: list[int] = []
    suppressed = np.zeros(bboxes.shape[0], dtype=bool)
    areas = (bboxes[:, 2] - bboxes[:, 0]) * (bboxes[:, 3] - bboxes[:, 1])
    for i in order:
        if suppressed[i]:
            continue
        keep.append(int(i))
        ix1 = np.maximum(bboxes[i, 0], bboxes[:, 0])
        iy1 = np.maximum(bboxes[i, 1], bboxes[:, 1])
        ix2 = np.minimum(bboxes[i, 2], bboxes[:, 2])
        iy2 = np.minimum(bboxes[i, 3], bboxes[:, 3])
        inter = np.maximum(0, ix2 - ix1) * np.maximum(0, iy2 - iy1)
        iou = inter / (areas[i] + areas - inter)
        suppressed[iou > iou_threshold] = True
    return np.array(keep, dtype=np.int64)


def non_max_suppression(
    prediction: np.ndarray,
    num_features: int,
    num_predictions: int,
    num_keypoints: int,
    conf_thres: float,
    iou_thres: float,
    max_det: int = 300,
    bbox_half_wh: bool = False,
    swap_wh: bool = False,
    bbox_xyxy: bool = False,
) -> list[tuple[np.ndarray, float, int, np.ndarray]]:
    """prediction: (num_features, num_predictions) or (num_predictions, num_features). Returns list of (xyxy, conf, class_id, keypoints (N,3)). Per-class NMS. Class scores and keypoint visibility are expected to be sigmoid-activated (values in [0,1]); sigmoid is applied as fallback if raw logits are detected."""
    num_keypoint_vals = num_keypoints * 3
    num_classes = num_features - 4 - num_keypoint_vals
    if num_classes <= 0 or num_predictions <= 0:
        return []
    if prediction.shape[0] == num_predictions and prediction.shape[1] == num_features:
        transposed = prediction
    else:
        transposed = prediction.T
    raw_class = transposed[:, 4 : 4 + num_classes].astype(np.float32)
    raw_min, raw_max = float(np.min(raw_class)), float(np.max(raw_class))
    if raw_min >= 0.0 and raw_max <= 1.0:
        class_scores = raw_class
    else:
        class_scores = sigmoid(raw_class.astype(np.float64)).astype(np.float32)
    max_scores = np.max(class_scores, axis=1)
    best_cls = np.argmax(class_scores, axis=1)
    mask = max_scores >= conf_thres
    if not np.any(mask):
        return []
    boxes_xywh = transposed[mask, :4].copy()
    if swap_wh:
        boxes_xywh[:, 2], boxes_xywh[:, 3] = (
            boxes_xywh[:, 3].copy(),
            boxes_xywh[:, 2].copy(),
        )
    scores = max_scores[mask]
    cls_ids = best_cls[mask]
    kp_data = transposed[
        mask, 4 + num_classes : 4 + num_classes + num_keypoint_vals
    ].copy()
    kp_vis = kp_data[:, 2::3]
    kp_vis_min, kp_vis_max = np.min(kp_vis), np.max(kp_vis)
    if not (kp_vis_min >= 0.0 and kp_vis_max <= 1.0):
        kp_data[:, 2::3] = sigmoid(kp_vis.astype(np.float64)).astype(np.float32)
    if not bbox_xyxy:
        xywh2xyxy(boxes_xywh, half_wh=bbox_half_wh)
    # Per-class NMS (match C++ behavior)
    all_keep: list[int] = []
    for c in range(num_classes):
        c_mask = cls_ids == c
        if not np.any(c_mask):
            continue
        c_indices = np.where(c_mask)[0]
        c_boxes = boxes_xywh[c_indices]
        c_scores = scores[c_indices]
        c_keep = nms(c_boxes, c_scores, iou_thres)
        all_keep.extend(c_indices[c_keep].tolist())
    all_keep = np.array(all_keep)
    all_keep = all_keep[np.argsort(-scores[all_keep])[:max_det]]
    out = []
    for i in all_keep:
        x1, y1, x2, y2 = boxes_xywh[i]
        out.append(
            (
                np.array([x1, y1, x2, y2]),
                float(scores[i]),
                int(cls_ids[i]),
                kp_data[i].reshape(num_keypoints, 3),
            )
        )
    return out


def parse_end2end_detections(
    prediction: np.ndarray,
    num_predictions: int,
    num_features: int,
    conf_thres: float,
) -> list[tuple[np.ndarray, float, int, np.ndarray]]:
    """Parse post-NMS end-to-end output from YOLO26 Pose26 / v10Detect heads.

    Per-row layout: [x1, y1, x2, y2, conf, class_id, kp0_x, kp0_y, kp0_vis, ...]
    Box coords and keypoint xy are in letterbox input pixel space (not normalised).
    conf is the winner-class probability; class_id is the integer class index stored
    as a float.  Rows beyond the actual detection count are zero-padded.
    """
    if prediction.shape == (num_features, num_predictions):
        rows = prediction.T
    else:
        rows = prediction  # already (num_predictions, num_features)

    num_kpt_vals = max(0, num_features - 6)  # 4 bbox + 1 conf + 1 class_id
    num_keypoints = num_kpt_vals // 3

    out: list[tuple[np.ndarray, float, int, np.ndarray]] = []
    for i in range(num_predictions):
        conf = float(rows[i, 4])
        if conf < conf_thres:
            continue
        xyxy = rows[i, :4].copy()
        cls_id = int(round(float(rows[i, 5])))
        if num_keypoints > 0:
            kps = rows[i, 6 : 6 + num_kpt_vals].reshape(num_keypoints, 3).copy()
        else:
            kps = np.empty((0, 3), dtype=np.float32)
        out.append((xyxy, conf, cls_id, kps))
    return out


def scale_detections_to_frame(
    detections: list[tuple[np.ndarray, float, int, np.ndarray]],
    orig_h: int,
    orig_w: int,
    scale: float,
    pad_left: float,
    pad_top: float,
    input_w: int = 0,
    input_h: int = 0,
) -> list[tuple[np.ndarray, float, int, np.ndarray]]:
    """Map box and keypoint coords from letterbox input space back to original frame.
    If input_w/input_h are set and box coords are in [0,1], scale from normalized to pixel first."""
    if not detections:
        return []
    result = []
    xyxy0 = detections[0][0]
    need_scale = (
        input_w > 0 and input_h > 0 and np.max(xyxy0) <= 1.0 and np.min(xyxy0) >= 0.0
    )
    for xyxy, conf, cls_id, kps in detections:
        xyxy = xyxy.copy()
        kps = kps.copy()
        if need_scale:
            xyxy[0] *= input_w
            xyxy[2] *= input_w
            xyxy[1] *= input_h
            xyxy[3] *= input_h
            kps[:, 0] *= input_w
            kps[:, 1] *= input_h
        xyxy[0] = (xyxy[0] - pad_left) / scale
        xyxy[2] = (xyxy[2] - pad_left) / scale
        xyxy[1] = (xyxy[1] - pad_top) / scale
        xyxy[3] = (xyxy[3] - pad_top) / scale
        kps[:, 0] = (kps[:, 0] - pad_left) / scale
        kps[:, 1] = (kps[:, 1] - pad_top) / scale
        result.append((xyxy, conf, cls_id, kps))
    return result


def draw_detections(
    frame: np.ndarray,
    detections: list[tuple[np.ndarray, float, int, np.ndarray]],
    class_names: list[str] | None,
    kp_conf_threshold: float = 0.5,
) -> np.ndarray:
    """Draw boxes, labels, and keypoints on frame."""
    out = frame.copy()
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
            class_names[cls_id]
            if class_names and cls_id < len(class_names)
            else f"class_{cls_id}"
        ) + f" {conf_display:.2f}"
        cv2.putText(out, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        for j in range(kps.shape[0]):
            x, y, kp_conf = kps[j]
            if kp_conf >= kp_conf_threshold:
                cx, cy = int(round(x)), int(round(y))
                cv2.circle(out, (cx, cy), 4, (255, 255, 255), -1)
                cv2.circle(out, (cx, cy), 3, color, -1)
    return out


def load_engine(engine_path: str):
    """Load TensorRT engine and create execution context."""
    logger = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f:
        engine_data = f.read()
    runtime = trt.Runtime(logger)
    # Allow engine host code (required for engines built with lean / host code, match C++ TrtEngine::load)
    if hasattr(runtime, "engine_host_code_allowed"):
        runtime.engine_host_code_allowed = True
    elif hasattr(runtime, "set_engine_host_code_allowed"):
        runtime.set_engine_host_code_allowed(True)
    engine = runtime.deserialize_cuda_engine(engine_data)
    if engine is None:
        raise RuntimeError("Failed to deserialize engine")
    context = engine.create_execution_context()
    return engine, context


def main() -> None:
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
    args = parser.parse_args()

    video_path = Path(args.video)
    engine_path = Path(args.engine)
    if not video_path.exists():
        raise FileNotFoundError(f"Video not found: {video_path}")
    if not engine_path.exists():
        raise FileNotFoundError(f"Engine not found: {engine_path}")

    print("Loading engine...")
    engine, context = load_engine(str(engine_path))

    input_name = None
    output_name = None
    for i in range(engine.num_io_tensors):
        name = engine.get_tensor_name(i)
        if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
            input_name = name
        else:
            output_name = name
    if input_name is None or output_name is None:
        raise RuntimeError("Engine must have exactly one input and one output")

    input_shape = context.get_tensor_shape(input_name)
    output_shape = context.get_tensor_shape(output_name)
    if len(input_shape) != 4 or input_shape[0] != 1 or input_shape[1] != 3:
        raise RuntimeError(
            f"Expected input shape [1, 3, H, W], got {list(input_shape)}"
        )
    input_h, input_w = int(input_shape[2]), int(input_shape[3])
    if args.imgsz > 0 and (args.imgsz != input_h or args.imgsz != input_w):
        raise RuntimeError(
            f"Engine has fixed input shape {input_h}x{input_w}; --imgsz must match or be 0"
        )

    dim1, dim2 = int(output_shape[1]), int(output_shape[2])
    # End-to-end (post-NMS) models output [1, max_det, features] where max_det > features
    # but max_det is small (e.g. 300).  Raw pre-NMS models output [1, features, anchors]
    # where anchors >> features (e.g. 8400 vs 12).  The reliable distinguisher is that
    # the larger dimension for pre-NMS is in the thousands, while for post-NMS it is not.
    is_end2end = dim1 > dim2
    if is_end2end:
        num_predictions, num_features = dim1, dim2
    else:
        num_features, num_predictions = dim1, dim2
    num_classes = args.num_classes
    num_keypoints = args.num_keypoints
    if is_end2end:
        # Post-NMS layout: [x1,y1,x2,y2, conf, class_id, kp0_x, kp0_y, kp0_vis, ...]
        num_keypoints = max(0, (num_features - 6)) // 3
        num_classes = -1  # not used for end2end path
    elif num_classes <= 0 or num_keypoints <= 0:
        num_classes = 1 if args.num_classes <= 0 else args.num_classes
        remainder = num_features - 4 - num_classes
        if remainder > 0 and remainder % 3 == 0:
            num_keypoints = remainder // 3
        else:
            num_keypoints = max(0, (num_features - 4 - 1) // 3)
        if args.num_keypoints > 0:
            num_keypoints = args.num_keypoints
        if args.num_classes > 0:
            num_classes = args.num_classes
    fmt = "end2end post-NMS" if is_end2end else "raw pre-NMS"
    print(
        f"Input shape: [1, 3, {input_h}, {input_w}], output: [1, {dim1}, {dim2}] ({fmt}), "
        f"num_keypoints={num_keypoints}"
        + (f", num_classes={num_classes}" if not is_end2end else "")
    )

    input_nbytes = 1 * 3 * input_h * input_w * 4
    output_nbytes = int(np.prod([1, output_shape[1], output_shape[2]])) * 4
    d_input = cuda.mem_alloc(input_nbytes)
    d_output = cuda.mem_alloc(output_nbytes)
    context.set_tensor_address(input_name, int(d_input))
    context.set_tensor_address(output_name, int(d_output))
    stream = cuda.Stream()

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS)) or 30
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
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

    start_time = time.time()
    frame_count = 0
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
            orig_h, orig_w = frame.shape[0], frame.shape[1]
            blob, scale, pad_left, pad_top = preprocess_frame(frame, input_h, input_w)
            blob = np.ascontiguousarray(blob.astype(np.float32))
            cuda.memcpy_htod_async(d_input, blob, stream)
            context.execute_async_v3(stream_handle=stream.handle)
            out_host = np.empty(
                (1, int(output_shape[1]), int(output_shape[2])), dtype=np.float32
            )
            cuda.memcpy_dtoh_async(out_host, d_output, stream)
            stream.synchronize()
            prediction = out_host[0]
            if is_end2end:
                detections = parse_end2end_detections(
                    prediction,
                    num_predictions,
                    num_features,
                    args.conf,
                )
            else:
                detections = non_max_suppression(
                    prediction,
                    num_features,
                    num_predictions,
                    num_keypoints,
                    args.conf,
                    args.iou,
                    bbox_half_wh=args.bbox_half_wh,
                    swap_wh=args.swap_wh,
                    bbox_xyxy=args.bbox_xyxy,
                )
            detections = scale_detections_to_frame(
                detections,
                orig_h,
                orig_w,
                scale,
                pad_left,
                pad_top,
                input_w=input_w,
                input_h=input_h,
            )
            annotated = draw_detections(frame, detections, class_names=None)
            if writer is not None:
                writer.write(annotated)
            if args.show:
                cv2.imshow("TensorRT YOLO", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("Interrupted by user")
                    break
            frame_count += 1
            elapsed = time.time() - start_time
            pbar.set_postfix(
                {"FPS": f"{frame_count / elapsed:.1f}" if elapsed > 0 else "0"}
            )
            pbar.update(1)
        pbar.close()
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()

    elapsed = time.time() - start_time
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    print(f"Processed {frame_count} frames in {elapsed:.2f}s, avg FPS: {avg_fps:.2f}")
    if not args.no_save:
        print(f"Output: {output_path}")


if __name__ == "__main__":
    main()
