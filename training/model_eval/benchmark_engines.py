"""Per-engine inference latency, reported the same way on the dev box and the Jetson.

`trtexec` is not available everywhere -- on megamind TensorRT comes from the pip wheel,
which ships no binary -- so this reuses `auto_battlebot.trt_yolo.TrtYoloModel`, the same
class `score.py` scores with. That also makes the numbers directly comparable to the
accuracy run instead of measuring a differently-configured engine.

Two levels are reported, because they answer different questions:

  gpu    `_run` only: H2D copy, `execute_async_v3`, D2H copy, stream sync. Comparable to
         `trtexec` with data transfers.
  total  `infer`: letterbox preprocess + `_run` + NMS/decode. This is what the C++
         `update()` call actually costs, and it is the one the frame-period decision
         rests on -- `docs/experiments/parallel_yolo_batch/shared_preprocess_2026-07-24.md`
         found GPU inference is 1-3 ms while a full per-model update is ~12 ms on the
         Jetson, so quoting GPU time alone understates a bigger model by ~4x.

Feed it a real eval frame (the default) rather than noise: NMS cost scales with the
number of surviving boxes, and a noise image decodes to a degenerate detection count.
"""

import argparse
import statistics
import time
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.trt_yolo import TrtYoloModel


def _percentile(values: list[float], q: float) -> float:
    """Nearest-rank percentile; avoids a numpy import just for this."""
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round(q * (len(ordered) - 1)))))
    return ordered[idx]


def _time_loop(
    fn: Callable[[np.ndarray], object], frame: np.ndarray, iterations: int, warmup: int
) -> list[float]:
    for _ in range(warmup):
        fn(frame)
    samples = []
    for _ in range(iterations):
        start = time.perf_counter()
        fn(frame)
        samples.append((time.perf_counter() - start) * 1000.0)
    return samples


def _load_frame(path: str | None) -> np.ndarray:
    if path is None:
        # 640x640 mid-grey: valid input, but detection counts are meaningless. Only used
        # when no eval image is available; the printed banner says so.
        return np.full((640, 640, 3), 128, dtype=np.uint8)
    frame = cv2.imread(path)
    if frame is None:
        raise SystemExit(f"could not read frame: {path}")
    return frame


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--candidate",
        action="append",
        default=[],
        metavar="NAME=ENGINE",
        help="Engine to time, repeatable. Same NAME=path form as score.py --candidate.",
    )
    parser.add_argument("--frame", default=None, help="Image to run on (default: grey fill)")
    parser.add_argument("--iterations", type=int, default=300, help="Timed iterations")
    parser.add_argument("--warmup", type=int, default=50, help="Untimed warmup iterations")
    parser.add_argument("--conf", type=float, default=0.5, help="Confidence threshold")
    parser.add_argument("--nms-iou", type=float, default=0.45, help="NMS IoU threshold")
    args = parser.parse_args()

    if not args.candidate:
        raise SystemExit("pass at least one --candidate NAME=ENGINE")

    frame = _load_frame(args.frame)
    source = args.frame if args.frame else "grey fill (NMS cost NOT representative)"
    print(f"frame: {source}  {frame.shape[1]}x{frame.shape[0]}")
    print(f"iterations: {args.iterations} (warmup {args.warmup})\n")

    header = (
        f"{'candidate':<12} {'level':<6} {'median ms':>10} "
        f"{'mean ms':>10} {'p90 ms':>10} {'dets':>6}"
    )
    print(header)
    print("-" * len(header))

    for spec in args.candidate:
        if "=" not in spec:
            raise SystemExit(f"--candidate must be NAME=ENGINE, got {spec!r}")
        name, engine_path = spec.split("=", 1)
        if not Path(engine_path).exists():
            raise SystemExit(f"engine not found: {engine_path}")

        model = TrtYoloModel(engine_path, conf_threshold=args.conf, nms_iou_threshold=args.nms_iou)
        blob_shape = (1, 3, model.input_h, model.input_w)
        blob = np.ascontiguousarray(np.zeros(blob_shape, dtype=np.float32))
        detections = model.infer(frame)

        for level, fn, arg in (("gpu", model._run, blob), ("total", model.infer, frame)):
            samples = _time_loop(fn, arg, args.iterations, args.warmup)
            print(
                f"{name:<12} {level:<6} "
                f"{statistics.median(samples):>10.3f} {statistics.fmean(samples):>10.3f} "
                f"{_percentile(samples, 0.90):>10.3f} "
                f"{len(detections) if level == 'total' else '':>6}"
            )
        del model


if __name__ == "__main__":
    main()
