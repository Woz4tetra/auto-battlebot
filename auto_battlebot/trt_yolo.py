"""TensorRT YOLO inference matching the C++ perception pipeline.

Shared by training/yolo/test_tensorrt_video.py (video visualization) and
training/model_eval/score.py (detector evaluation). Mirrors the C++ YoloKeypointModel /
YoloSegRobotBlobModel pipeline: same letterbox preprocessing (114 padding, configurable
sub-pixel padding nudge), same output decode, and same per-class NMS.

Supports both head types:

- raw pre-NMS: output [1, features, anchors] with features = 4 bbox + num_classes scores
  + 3 * num_keypoints keypoint values, optionally followed by extra columns that are
  ignored (YOLO-seg mask coefficients)
- end2end post-NMS: output [1, max_det, features] with per-row
  [x1, y1, x2, y2, conf, class_id, kp...]

Detections are (xyxy, confidence, class_id, keypoints (K, 3)) tuples in original frame
pixel coordinates.

Requires: tensorrt, pycuda, opencv-python, numpy.
"""

from __future__ import annotations

from typing import Any, cast

import cv2
import numpy as np

DetectionTuple = tuple[np.ndarray, float, int, np.ndarray]

# C++ models nudge the letterbox split by 0.1 px (top gets round(pad - 0.1), bottom
# round(pad + 0.1)), which biases odd padding to the bottom/right side.
CPP_LETTERBOX_PADDING = 0.1


def letterbox(
    image: np.ndarray,
    target_h: int,
    target_w: int,
    pad_val: float = 114.0,
    padding: float = 0.0,
) -> tuple[np.ndarray, float, float, float]:
    """Resize with aspect ratio and pad to target size.

    Returns (padded image, scale, pad_left, pad_top).
    """
    h, w = image.shape[:2]
    scale = min(target_h / h, target_w / w)
    new_w = int(round(w * scale))
    new_h = int(round(h * scale))
    if new_w == target_w and new_h == target_h:
        if image.shape[0] == target_h and image.shape[1] == target_w:
            return image, scale, 0.0, 0.0
        resized = cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
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
    padded, scale, pad_left, pad_top = letterbox(rgb, input_h, input_w, padding=letterbox_padding)
    blob = padded.astype(np.float32) / 255.0
    blob = np.transpose(blob, (2, 0, 1))
    blob = np.expand_dims(blob, axis=0)
    return blob, scale, pad_left, pad_top


def sigmoid(x: np.ndarray) -> np.ndarray:
    """Numerically stable sigmoid. Fallback for models that output raw logits."""
    result = np.where(x >= 0, 1.0 / (1.0 + np.exp(-x)), np.exp(x) / (1.0 + np.exp(x)))
    return cast(np.ndarray, result)


def xywh2xyxy(boxes: np.ndarray, half_wh: bool = False) -> None:
    """In-place: (cx, cy, w, h) -> (x1, y1, x2, y2).

    If half_wh, 3rd/4th are half-width/half-height (no /2).
    """
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


def _orient_predictions(
    prediction: np.ndarray, num_predictions: int, num_features: int
) -> np.ndarray:
    """Return predictions as (num_predictions, num_features), transposing if needed."""
    if prediction.shape[0] == num_predictions and prediction.shape[1] == num_features:
        return prediction
    return cast(np.ndarray, prediction.T)


def _activate_class_scores(raw_class: np.ndarray) -> np.ndarray:
    """Sigmoid-activate class scores only if raw logits are detected (values outside [0,1])."""
    raw_min, raw_max = float(np.min(raw_class)), float(np.max(raw_class))
    if raw_min >= 0.0 and raw_max <= 1.0:
        return raw_class
    return sigmoid(raw_class.astype(np.float64)).astype(np.float32)


def _activate_keypoint_visibility(kp_data: np.ndarray) -> None:
    """In-place sigmoid of the keypoint visibility column if raw logits are detected."""
    if kp_data.size == 0:
        return
    kp_vis = kp_data[:, 2::3]
    kp_vis_min, kp_vis_max = np.min(kp_vis), np.max(kp_vis)
    if not (kp_vis_min >= 0.0 and kp_vis_max <= 1.0):
        kp_data[:, 2::3] = sigmoid(kp_vis.astype(np.float64)).astype(np.float32)


def _per_class_nms(
    boxes_xywh: np.ndarray,
    scores: np.ndarray,
    cls_ids: np.ndarray,
    num_classes: int,
    iou_thres: float,
) -> np.ndarray:
    """Per-class NMS (match C++ behavior). Returns kept indices into the filtered arrays."""
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
    return np.array(all_keep)


def _build_detections(
    keep_indices: np.ndarray,
    boxes_xywh: np.ndarray,
    scores: np.ndarray,
    cls_ids: np.ndarray,
    kp_data: np.ndarray,
    num_keypoints: int,
) -> list[DetectionTuple]:
    """Assemble the (xyxy, conf, class_id, keypoints) tuples for the kept indices."""
    out = []
    for i in keep_indices:
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


def non_max_suppression(
    prediction: np.ndarray,
    num_features: int,
    num_predictions: int,
    num_classes: int,
    num_keypoints: int,
    conf_thres: float,
    iou_thres: float,
    max_det: int = 300,
    bbox_half_wh: bool = False,
    swap_wh: bool = False,
    bbox_xyxy: bool = False,
) -> list[DetectionTuple]:
    """Apply per-class NMS to raw predictions.

    prediction: (num_features, num_predictions) or (num_predictions, num_features).
    Feature columns beyond 4 + num_classes + 3 * num_keypoints (e.g. YOLO-seg mask
    coefficients) are ignored. Returns list of (xyxy, conf, class_id, keypoints (N, 3)).
    Class scores and keypoint visibility are expected to be sigmoid-activated (values in
    [0,1]); sigmoid is applied as fallback if raw logits are detected.
    """
    num_keypoint_vals = num_keypoints * 3
    if num_classes <= 0 or num_predictions <= 0:
        return []
    transposed = _orient_predictions(prediction, num_predictions, num_features)
    raw_class = transposed[:, 4 : 4 + num_classes].astype(np.float32)
    class_scores = _activate_class_scores(raw_class)
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
    kp_data = transposed[mask, 4 + num_classes : 4 + num_classes + num_keypoint_vals].copy()
    _activate_keypoint_visibility(kp_data)
    if not bbox_xyxy:
        xywh2xyxy(boxes_xywh, half_wh=bbox_half_wh)
    all_keep = _per_class_nms(boxes_xywh, scores, cls_ids, num_classes, iou_thres)
    all_keep = all_keep[np.argsort(-scores[all_keep])[:max_det]]
    return _build_detections(all_keep, boxes_xywh, scores, cls_ids, kp_data, num_keypoints)


def parse_end2end_detections(
    prediction: np.ndarray,
    num_predictions: int,
    num_features: int,
    conf_thres: float,
) -> list[DetectionTuple]:
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

    out: list[DetectionTuple] = []
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
    detections: list[DetectionTuple],
    orig_h: int,
    orig_w: int,
    scale: float,
    pad_left: float,
    pad_top: float,
    input_w: int = 0,
    input_h: int = 0,
) -> list[DetectionTuple]:
    """Map box and keypoint coords from letterbox input space back to original frame.
    If input_w/input_h are set and box coords are in [0,1], scale from normalized to pixel first."""
    if not detections:
        return []
    result = []
    xyxy0 = detections[0][0]
    need_scale = input_w > 0 and input_h > 0 and np.max(xyxy0) <= 1.0 and np.min(xyxy0) >= 0.0
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


def load_engine(engine_path: str) -> tuple[Any, Any]:
    """Load TensorRT engine and create execution context."""
    import tensorrt as trt

    logger = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f:
        engine_data = f.read()
    runtime = trt.Runtime(logger)
    # Allow engine host code (required for engines built with lean / host code,
    # match C++ TrtEngine::load)
    if hasattr(runtime, "engine_host_code_allowed"):
        runtime.engine_host_code_allowed = True
    elif hasattr(runtime, "set_engine_host_code_allowed"):
        runtime.set_engine_host_code_allowed(True)
    engine = runtime.deserialize_cuda_engine(engine_data)
    if engine is None:
        raise RuntimeError("Failed to deserialize engine")
    context = engine.create_execution_context()
    return engine, context


def _infer_raw_head_dims(
    num_features: int, num_classes: int, num_keypoints: int
) -> tuple[int, int]:
    """Infer (num_classes, num_keypoints) for a raw pre-NMS head, honoring overrides.

    num_classes > 0 is trusted; a remainder that is not a multiple of 3 is then extra
    columns to ignore (YOLO-seg mask coefficients), not keypoints. Without an explicit
    class count, fall back to the single-class pose guess. num_keypoints >= 0 overrides.
    """
    if num_classes > 0:
        inferred_classes = num_classes
        remainder = num_features - 4 - inferred_classes
        inferred_keypoints = remainder // 3 if remainder > 0 and remainder % 3 == 0 else 0
    else:
        inferred_classes = 1
        remainder = num_features - 4 - inferred_classes
        if remainder > 0 and remainder % 3 == 0:
            inferred_keypoints = remainder // 3
        else:
            inferred_keypoints = max(0, (num_features - 4 - 1) // 3)
    if num_keypoints >= 0:
        inferred_keypoints = num_keypoints
    return inferred_classes, inferred_keypoints


def resolve_model_layout(
    dim1: int, dim2: int, num_classes: int = 0, num_keypoints: int = -1
) -> tuple[bool, int, int, int, int]:
    """Determine the output layout from the engine output dims.

    num_classes / num_keypoints override inference when set (> 0 / >= 0).
    Returns (is_end2end, num_predictions, num_features, num_classes, num_keypoints).
    """
    # End-to-end (post-NMS) models output [1, max_det, features] where max_det > features
    # but max_det is small (e.g. 300).  Raw pre-NMS models output [1, features, anchors]
    # where anchors >> features (e.g. 8400 vs 12).  The reliable distinguisher is that
    # the larger dimension for pre-NMS is in the thousands, while for post-NMS it is not.
    is_end2end = dim1 > dim2
    if is_end2end:
        num_predictions, num_features = dim1, dim2
    else:
        num_features, num_predictions = dim1, dim2
    if is_end2end:
        # Post-NMS layout: [x1,y1,x2,y2, conf, class_id, kp0_x, kp0_y, kp0_vis, ...]
        num_keypoints = max(0, (num_features - 6)) // 3
        num_classes = -1  # not used for end2end path
    elif num_classes <= 0 or num_keypoints < 0:
        num_classes, num_keypoints = _infer_raw_head_dims(num_features, num_classes, num_keypoints)
    return is_end2end, num_predictions, num_features, num_classes, num_keypoints


class TrtYoloModel:
    """A loaded TensorRT YOLO engine with preprocessing, inference, and decode.

    Handles single-output pose/detect engines and multi-output seg engines (the 3-dim
    detection head is used; mask prototype outputs are ignored). `infer` returns
    detections in original frame pixel coordinates.
    """

    def __init__(
        self,
        engine_path: str,
        conf_threshold: float,
        nms_iou_threshold: float = 0.45,
        num_classes: int = 0,
        num_keypoints: int = -1,
        letterbox_padding: float = CPP_LETTERBOX_PADDING,
        bbox_half_wh: bool = False,
        swap_wh: bool = False,
        bbox_xyxy: bool = False,
    ) -> None:
        import pycuda.autoinit  # noqa: F401
        import pycuda.driver as cuda
        import tensorrt as trt

        self._cuda = cuda
        self.conf_threshold = conf_threshold
        self.nms_iou_threshold = nms_iou_threshold
        self.letterbox_padding = letterbox_padding
        self.bbox_half_wh = bbox_half_wh
        self.swap_wh = swap_wh
        self.bbox_xyxy = bbox_xyxy

        self.engine, self.context = load_engine(engine_path)
        input_name = None
        output_name = None
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                if input_name is not None:
                    raise RuntimeError("Engine must have exactly one input")
                input_name = name
            elif output_name is None and len(self.context.get_tensor_shape(name)) == 3:
                # Detection head. Seg engines also expose 4-dim mask prototypes; skip them.
                output_name = name
        if input_name is None or output_name is None:
            raise RuntimeError("Engine must have one input and one 3-dim detection output")
        self.input_name = input_name
        self.output_name = output_name

        input_shape = self.context.get_tensor_shape(input_name)
        if len(input_shape) != 4 or input_shape[0] != 1 or input_shape[1] != 3:
            raise RuntimeError(f"Expected input shape [1, 3, H, W], got {list(input_shape)}")
        self.input_h, self.input_w = int(input_shape[2]), int(input_shape[3])
        self.output_shape = self.context.get_tensor_shape(output_name)

        dim1, dim2 = int(self.output_shape[1]), int(self.output_shape[2])
        (
            self.is_end2end,
            self.num_predictions,
            self.num_features,
            self.num_classes,
            self.num_keypoints,
        ) = resolve_model_layout(dim1, dim2, num_classes, num_keypoints)

        # Bind all IO tensors; unused outputs (mask prototypes) still need an address.
        self._buffers = []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = self.context.get_tensor_shape(name)
            nbytes = int(np.prod([int(d) for d in shape])) * 4
            buf = cuda.mem_alloc(nbytes)
            self._buffers.append(buf)
            self.context.set_tensor_address(name, int(buf))
            if name == input_name:
                self._d_input = buf
            elif name == output_name:
                self._d_output = buf
        self.stream = cuda.Stream()

    def describe(self) -> str:
        """One-line human-readable layout summary."""
        fmt = "end2end post-NMS" if self.is_end2end else "raw pre-NMS"
        detail = f"num_keypoints={self.num_keypoints}"
        if not self.is_end2end:
            detail += f", num_classes={self.num_classes}"
        return (
            f"input [1, 3, {self.input_h}, {self.input_w}], "
            f"output [1, {int(self.output_shape[1])}, {int(self.output_shape[2])}] ({fmt}), "
            f"{detail}"
        )

    def _run(self, blob: np.ndarray) -> np.ndarray:
        """One forward pass; returns the raw prediction (output dims 1 and 2)."""
        cuda = self._cuda
        blob = np.ascontiguousarray(blob.astype(np.float32))
        cuda.memcpy_htod_async(self._d_input, blob, self.stream)
        self.context.execute_async_v3(stream_handle=self.stream.handle)
        out_host = np.empty(
            (1, int(self.output_shape[1]), int(self.output_shape[2])), dtype=np.float32
        )
        cuda.memcpy_dtoh_async(out_host, self._d_output, self.stream)
        self.stream.synchronize()
        return cast(np.ndarray, out_host[0])

    def infer(self, frame_bgr: np.ndarray) -> list[DetectionTuple]:
        """Detect on one BGR frame; returns detections in frame pixel coordinates."""
        orig_h, orig_w = frame_bgr.shape[:2]
        blob, scale, pad_left, pad_top = preprocess_frame(
            frame_bgr, self.input_h, self.input_w, self.letterbox_padding
        )
        prediction = self._run(blob)
        if self.is_end2end:
            detections = parse_end2end_detections(
                prediction, self.num_predictions, self.num_features, self.conf_threshold
            )
        else:
            detections = non_max_suppression(
                prediction,
                self.num_features,
                self.num_predictions,
                self.num_classes,
                self.num_keypoints,
                self.conf_threshold,
                self.nms_iou_threshold,
                bbox_half_wh=self.bbox_half_wh,
                swap_wh=self.swap_wh,
                bbox_xyxy=self.bbox_xyxy,
            )
        return scale_detections_to_frame(
            detections,
            orig_h,
            orig_w,
            scale,
            pad_left,
            pad_top,
            input_w=self.input_w,
            input_h=self.input_h,
        )
