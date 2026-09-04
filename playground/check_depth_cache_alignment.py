#!/usr/bin/env python3
"""Prove the cached depth belongs to the frame the labels describe.

`cache_gt_depth.py` reaches each wanted frame with `set_svo_position`. Seeking an SVO can smear
the decode, and a smeared seek is silent: the depth still arrives, still has the right shape, and
is simply the wrong frame. Every conclusion drawn from a depth gate would then be measuring
noise against labels that describe a different moment.

The check re-seeks the same indices and pulls the LEFT image instead of depth, then compares it to
the ground-truth PNG that carries the same stamp.

The comparison is relative, not absolute. The pipeline's PNGs are rectified and colour-processed
differently from a raw SVO retrieve, and desktop playback frames are affine-warped a few percent
against live Jetson frames, so even a perfectly matched pair has a mean absolute difference in the
teens. An absolute threshold cannot tell that apart from a wrong frame.

What can: sweep the neighbouring indices too. Whatever constant penalty rectification adds applies
to all of them equally, so if the seek lands where it claims, the claimed index is the argmin and
its neighbours are worse. A smeared or off-by-N seek puts the argmin somewhere else. The reported
number is the fraction of sampled frames whose best offset is 0, plus the margin between the
claimed index and its best neighbour.

The comparison is restricted to the ground-truth boxes, dilated. Over a whole frame the arena,
walls and crowd are identical from one index to the next and swamp the only thing that moved; the
robot is a few percent of the pixels. Inside its own box it is most of them, which is what makes a
one-frame offset visible at all.

Usage:
    python playground/check_depth_cache_alignment.py training/data/nhrl_keypoints_eval_test \
        --svo-roots data/svo -o training/data/eval_results/depth_cache_alignment.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from playground.cache_gt_depth import gt_frame_indices, resolve_svo

# Fraction of sampled frames whose best-matching SVO index must be the claimed one.
DEFAULT_ARGMIN_PASS = 0.8

# Indices either side of the claimed one to sweep.
DEFAULT_SWEEP = 3

# Pixels of slack around each GT box, so a robot that moved slightly still falls inside the window
# being compared rather than half outside it.
BOX_PAD_PX = 20


def grayscale(mat, shape: tuple[int, int]) -> np.ndarray:
    gray = cv2.cvtColor(mat.get_data()[:, :, :3], cv2.COLOR_BGR2GRAY)
    return gray if gray.shape == shape else cv2.resize(gray, (shape[1], shape[0]))


def gt_region_mask(subdataset: Path, stem: str, shape: tuple[int, int]) -> np.ndarray | None:
    """Padded union of a frame's GT boxes, or None when the frame has no labels."""
    path = subdataset / "labels" / f"{stem}.txt"
    if not path.exists():
        return None
    height, width = shape
    mask = np.zeros(shape, bool)
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cx, cy, box_w, box_h = (float(v) for v in parts[1:5])
        x1 = max(0, int((cx - box_w / 2) * width) - BOX_PAD_PX)
        y1 = max(0, int((cy - box_h / 2) * height) - BOX_PAD_PX)
        x2 = min(width, int((cx + box_w / 2) * width) + BOX_PAD_PX)
        y2 = min(height, int((cy + box_h / 2) * height) + BOX_PAD_PX)
        mask[y1:y2, x1:x2] = True
    return mask if mask.any() else None


def compare(
    svo_path: Path, wanted: dict[int, int], subdataset: Path, limit: int, sweep: int
) -> dict:
    images = subdataset / "images"
    import pyzed.sl as sl  # type: ignore[import-untyped]

    init = sl.InitParameters()
    init.set_from_svo_file(str(svo_path))
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_units = sl.UNIT.METER
    init.sdk_verbose = 0

    camera = sl.Camera()
    if camera.open(init) != sl.ERROR_CODE.SUCCESS:
        return {"error": "open failed"}

    total = camera.get_svo_number_of_frames()
    runtime = sl.RuntimeParameters()
    left = sl.Mat()

    ordered = sorted(wanted.items(), key=lambda kv: kv[1])
    step = max(1, len(ordered) // limit)
    sampled = ordered[::step][:limit]

    at_claimed, best_offsets, margins = [], [], []
    by_offset: dict[int, list[float]] = {o: [] for o in range(-sweep, sweep + 1)}
    for stamp, index in sampled:
        reference = images / f"{stamp}.png"
        if not reference.exists() or index < 0 or index >= total:
            continue
        gt_gray = cv2.imread(str(reference), cv2.IMREAD_GRAYSCALE)
        if gt_gray is None:
            continue
        region = gt_region_mask(subdataset, str(stamp), gt_gray.shape)
        if region is None:
            continue

        scores: dict[int, float] = {}
        for offset in range(-sweep, sweep + 1):
            probe = index + offset
            if probe < 0 or probe >= total:
                continue
            camera.set_svo_position(probe)
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue
            camera.retrieve_image(left, sl.VIEW.LEFT)
            svo_gray = grayscale(left, gt_gray.shape)
            delta = np.abs(gt_gray.astype(np.int16) - svo_gray.astype(np.int16))
            scores[offset] = float(delta[region].mean())
        if 0 not in scores or len(scores) < 2:
            continue

        for offset, value in scores.items():
            by_offset[offset].append(value)
        best = min(scores, key=lambda k: scores[k])
        others = [v for k, v in scores.items() if k != 0]
        best_offsets.append(best)
        at_claimed.append(scores[0])
        margins.append(min(others) - scores[0])

    camera.close()
    if not at_claimed:
        return {"error": "no comparable frames"}

    # Per-frame argmin is a weak matcher: when a robot barely moves between adjacent indices, the
    # winner among neighbours is decided by noise. Averaging the curve over every sampled frame is
    # the statistic that actually separates a good seek from a bad one. A correct seek gives a
    # V centred on 0; a broken one gives a flat curve, because then no index is special.
    curve = {
        offset: round(float(np.mean(values)), 2)
        for offset, values in sorted(by_offset.items())
        if values
    }
    far = [v for o, v in curve.items() if abs(o) >= max(2, sweep // 2)]
    hit_rate = float(np.mean([o == 0 for o in best_offsets]))
    return {
        "compared": len(at_claimed),
        "argmin_is_claimed": round(hit_rate, 3),
        "offset_histogram": {str(o): best_offsets.count(o) for o in sorted(set(best_offsets))},
        "mad_curve": {str(k): v for k, v in curve.items()},
        "curve_argmin": min(curve, key=lambda k: curve[k]),
        # How much better the claimed index is than the distant ones. A flat curve means the seek
        # carries no frame-specific information and the cache cannot be trusted.
        "curve_depth": round(float(np.mean(far)) - curve[0], 2) if far else None,
        "mad_at_claimed_mean": round(float(np.mean(at_claimed)), 2),
        "margin_over_best_neighbour_mean": round(float(np.mean(margins)), 2),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="eval dataset root")
    parser.add_argument("--svo-roots", type=Path, nargs="+", default=[Path("data/svo")])
    parser.add_argument("--limit", type=int, default=20, help="frames sampled per sub dataset")
    parser.add_argument("--argmin-pass", type=float, default=DEFAULT_ARGMIN_PASS)
    parser.add_argument("--sweep", type=int, default=DEFAULT_SWEEP, help="indices swept each side")
    parser.add_argument("-o", "--output", type=Path, required=True)
    args = parser.parse_args()

    report: dict[str, dict] = {}
    for subdataset in sorted(d for d in args.gt.iterdir() if (d / "data.yaml").exists()):
        wanted = gt_frame_indices(subdataset)
        data = yaml.safe_load((subdataset / "data.yaml").read_text())
        svo = resolve_svo(str(data.get("source_mcap", "")), list(args.svo_roots))
        if not wanted or svo is None:
            report[subdataset.name] = {"error": "no poses or no SVO"}
            continue
        result = compare(svo, wanted, subdataset, args.limit, args.sweep)
        if "curve_argmin" in result:
            # +-1 counts as aligned: SVO image stamps sit about half a frame before the pipeline
            # stamps that name the GT images, so the true minimum lies between two indices and
            # which side wins is arbitrary. A cache off by more than that is not usable.
            aligned = abs(result["curve_argmin"]) <= 1 and (result["curve_depth"] or 0) > 0
            result["verdict"] = "aligned" if aligned else "SUSPECT"
        report[subdataset.name] = result
        print(f"{subdataset.name}: {result}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    suspect = [k for k, v in report.items() if v.get("verdict") == "SUSPECT"]
    print(f"\n{len(suspect)} suspect: {suspect}" if suspect else "\nAll sub datasets aligned.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
