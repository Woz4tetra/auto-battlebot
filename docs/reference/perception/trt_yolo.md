# Python TensorRT YOLO (`trt_yolo`)

`auto_battlebot/perception/trt_yolo.py` mirrors the C++ `YoloKeypointModel` and `YoloBboxRobotBlobModel` path so eval scores the pipeline the robot runs. `training/yolo/test_tensorrt_video.py` (video visualization) and `auto_battlebot/eval` (detector scoring) share it. It needs tensorrt, pycuda, opencv-python, and numpy.

## Head types

- Raw pre-NMS: output `[1, features, anchors]`, with features = 4 bbox + `num_classes` scores + 3 * `num_keypoints` keypoint values. Extra trailing columns (YOLO-seg mask coefficients) are ignored. Class scores and keypoint visibility are expected in [0, 1]; if a value falls outside that range the decoder applies a sigmoid.
- End2end post-NMS: output `[1, max_det, features]`, one row per detection: `[x1, y1, x2, y2, conf, class_id, kp0_x, kp0_y, kp0_vis, ...]`. Box and keypoint xy are in letterbox input pixels, not normalised. `conf` is the winner-class probability and `class_id` is an integer stored as a float. Rows past the detection count are zero-padded. Produced by YOLO26 Pose26 and v10Detect heads.

Detections are `(xyxy, confidence, class_id, keypoints (K, 3))` tuples in original frame pixel coordinates.

## Layout detection

`resolve_model_layout` calls the head end2end when output dim 1 is larger than dim 2, and raw otherwise. A raw head whose feature count exceeds its anchor count (a very small input) would be misread as end2end. Typical shapes: `[1, 300, 12]` end2end, `[1, 12, 8400]` raw.

For a raw head, a positive `num_classes` is trusted. A remainder after `4 + num_classes` that is not a multiple of 3 is extra columns, not keypoints. Without a class count the head is guessed as single-class pose. `num_keypoints >= 0` overrides the inferred count.

Auto-inferring `num_classes` (0) only works for single-class engines. A 4-class 2-keypoint pose head has the same 14 feature columns as a 1-class 3-keypoint one, and the guess picks the latter, so pass `num_classes` for multi-class pose engines and check the printed layout line. The same holds in `training/model_eval/score.py`: `--candidate-labels` sets the label count per candidate, and the label count sets `num_classes`, which splits the raw tensor into scores and keypoints. A 2-class engine read with three labels parses to `num_keypoints=0`, so a grid mixing 2-class and 3-class engines needs it.

## Preprocessing

- `PreprocessMode.LETTERBOX` pads with value 114. The C++ models nudge the pad split by 0.1 px: top gets `round(pad - 0.1)` and bottom gets `round(pad + 0.1)`, which biases odd padding to the bottom and right. `CPP_LETTERBOX_PADDING` holds the 0.1.
- `PreprocessMode.STRETCH` resizes ignoring aspect ratio, so its x and y scales differ.
- The mode must match how the engine's weights were trained. A stretch-trained engine fed a letterboxed frame sees the wrong aspect ratio.

## Rectangular input size

A 16:9 source letterboxed into a square 640x640 tensor wastes 44% of it on grey padding. `training/yolo/convert_to_onnx.py --imgsz 384 640` cuts that to 6% at the same detection scale. Both sides must be multiples of 32. The C++ model reads the shape from the engine, so a rectangular engine is a drop-in swap.
