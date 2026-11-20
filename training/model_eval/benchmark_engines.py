"""Per-engine inference latency, reported the same way on the dev box and the Jetson."""

import argparse
import csv
import statistics
import time
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.perception.trt_yolo import TrtYoloModel, preprocess_frame


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
    parser.add_argument("--csv", default=None, help="Also write the table to this CSV path")
    parser.add_argument(
        "--num-classes",
        type=int,
        default=0,
        help="Class count of the head. 0 auto-infers, which only works for single-class "
        "engines; check the printed layout line",
    )
    parser.add_argument(
        "--num-keypoints",
        type=int,
        default=-1,
        help="Keypoint count of the head. -1 infers it from --num-classes.",
    )
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

    rows: list[dict[str, object]] = []
    layouts: list[str] = []

    for spec in args.candidate:
        if "=" not in spec:
            raise SystemExit(f"--candidate must be NAME=ENGINE, got {spec!r}")
        name, engine_path = spec.split("=", 1)
        if not Path(engine_path).exists():
            raise SystemExit(f"engine not found: {engine_path}")

        model = TrtYoloModel(
            engine_path,
            conf_threshold=args.conf,
            nms_iou_threshold=args.nms_iou,
            num_classes=args.num_classes,
            num_keypoints=args.num_keypoints,
        )
        layouts.append(f"{name:<12} {model.describe()}")
        blob_shape = (1, 3, model.input_h, model.input_w)
        blob = np.ascontiguousarray(np.zeros(blob_shape, dtype=np.float32))
        detections = model.infer(frame)

        def _pre(f: np.ndarray, m: TrtYoloModel = model) -> object:
            return preprocess_frame(f, m.input_h, m.input_w, m.letterbox_padding, m.preprocess)

        means: dict[str, float] = {}
        for level, fn, arg in (
            ("pre", _pre, frame),
            ("gpu", model._run, blob),
            ("total", model.infer, frame),
        ):
            samples = _time_loop(fn, arg, args.iterations, args.warmup)
            means[level] = statistics.fmean(samples)
            row = {
                "candidate": name,
                "level": level,
                "median_ms": round(statistics.median(samples), 4),
                "mean_ms": round(means[level], 4),
                "p90_ms": round(_percentile(samples, 0.90), 4),
                "dets": len(detections) if level == "total" else "",
            }
            rows.append(row)
            print(
                f"{name:<12} {level:<6} "
                f"{row['median_ms']:>10.3f} {row['mean_ms']:>10.3f} "
                f"{row['p90_ms']:>10.3f} {row['dets']!s:>6}"
            )

        post_mean = means["total"] - means["pre"] - means["gpu"]
        rows.append(
            {
                "candidate": name,
                "level": "post",
                "median_ms": "",
                "mean_ms": round(post_mean, 4),
                "p90_ms": "",
                "dets": "",
            }
        )
        print(f"{name:<12} {'post':<6} {'':>10} {post_mean:>10.3f} {'':>10} {'':>6}")
        del model

    print("\nlayouts:")
    for line in layouts:
        print(f"  {line}")

    if args.csv:
        out = Path(args.csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
