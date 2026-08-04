"""Measure how far a YOLO-seg mask centroid sits from its own bounding-box center.

The segmentation head emits a mask and a box for the same detection, but the pipeline
only consumes a position. This script asks whether the mask centroid carries position
information the box center does not:

1. Per detection, the offset between the mask centroid and that detection's box center,
   in pixels and as a fraction of the box's longer side (the same normalization
   score.py uses for keypoint PCK).
2. Over IoU-matched detections, how far each position estimate lands from the ground-truth
   box center. A detect-only model can be passed as a third estimator so the mask
   centroid, the seg box center, and a bbox-only model are graded on one scale.

Inference runs through ultralytics on the .pt weights, not the TensorRT engine, because
TrtYoloModel discards the mask coefficients. Detection metrics from this path differ
slightly from score.py's engine numbers; the offsets measured here are self-consistent
because centroid and box come from the same forward pass.

Example:

    python training/model_eval/mask_centroid_vs_box.py training/data/nhrl_keypoints_eval_test \
        --seg-weights data/models_v2/yolo26n-seg_nhrl_robots_2026-04-27.pt \
        --seg-labels opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3 \
        --bbox-weights data/models/yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31.pt \
        --bbox-labels opponent,house_bot \
        --taxonomy training/model_eval/taxonomy_merged.yaml --conf 0.5
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import cv2
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from score import BACK_IDX, FRONT_IDX, Frame, Taxonomy, load_gt, match_indices

# Categorical slots 1-3 of the validated default palette, in fixed order.
SERIES_COLORS = ("#2a78d6", "#eb6834", "#1baf7a")
GRID_COLOR = "#c9c8c2"
TEXT_SECONDARY = "#52514e"

# Offset thresholds reported as "fraction of detections within", as a share of the box's
# longer side. 0.1 matches score.py's PCK_FRACTION.
NEAR_THRESHOLDS = (0.02, 0.05, 0.1)

MASK_BINARY_THRESHOLD = 0.5

# The two points an estimate can be graded against. The GT box center is what the labels
# encode; the keypoint midpoint is what the aim controller is steering at.
REFERENCES = {
    "gtbox": "the GT box center",
    "kpmid": "the keypoint front/back midpoint",
}

# Display names, in the order they appear in tables and the plot.
ESTIMATORS = {
    "centroid": "mask centroid",
    "seg_boxcenter": "seg box center",
    "bbox_model": "bbox-only model",
    "gt_boxcenter": "GT box center",
}

COMPARISONS = (
    ("centroid", "seg_boxcenter"),
    ("centroid", "bbox_model"),
    ("seg_boxcenter", "bbox_model"),
    ("centroid", "gt_boxcenter"),
    ("seg_boxcenter", "gt_boxcenter"),
)

# `gt_boxcenter` is the GT box center, so grading it against the `gtbox` reference is
# identically zero. Report it only against the keypoint midpoint.
DEGENERATE = {("gtbox", "gt_boxcenter")}


def box_centers(boxes: np.ndarray) -> np.ndarray:
    """(N, 2) center of each (N, 4) xyxy box."""
    if len(boxes) == 0:
        return np.zeros((0, 2))
    return np.stack([(boxes[:, 0] + boxes[:, 2]) / 2, (boxes[:, 1] + boxes[:, 3]) / 2], axis=1)


def longer_sides(boxes: np.ndarray) -> np.ndarray:
    """(N,) longer side of each box, the scale offsets are normalized against."""
    if len(boxes) == 0:
        return np.zeros(0)
    return np.maximum(boxes[:, 2] - boxes[:, 0], boxes[:, 3] - boxes[:, 1])


def mask_centroid(mask: np.ndarray) -> tuple[float, float, int]:
    """Pixel-mass centroid (x, y) of a full-resolution binary mask, plus its pixel count."""
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return float("nan"), float("nan"), 0
    return float(xs.mean()), float(ys.mean()), int(len(xs))


def predict(model, image: np.ndarray, args: argparse.Namespace, want_masks: bool):
    """Boxes, scores, class ids and (optionally) full-resolution masks for one image."""
    result = model.predict(
        image,
        conf=args.conf,
        iou=args.nms_iou,
        imgsz=args.imgsz,
        retina_masks=want_masks,
        verbose=False,
    )[0]
    boxes = result.boxes.xyxy.cpu().numpy().astype(np.float64).reshape(-1, 4)
    scores = result.boxes.conf.cpu().numpy().astype(np.float64)
    class_ids = result.boxes.cls.cpu().numpy().astype(int)
    masks = None
    if want_masks and result.masks is not None:
        masks = result.masks.data.cpu().numpy() > MASK_BINARY_THRESHOLD
        if masks.shape[1:] != image.shape[:2]:
            raise SystemExit(
                f"Mask shape {masks.shape[1:]} does not match image {image.shape[:2]}; "
                "retina_masks did not return full-resolution masks"
            )
    return boxes, scores, class_ids, masks


def keep_mask(class_ids: np.ndarray, class_labels: list[str], taxonomy: Taxonomy) -> np.ndarray:
    """Boolean selector dropping unknown class ids and taxonomy-excluded labels."""
    return np.asarray(
        [
            cid < len(class_labels) and class_labels[cid] not in taxonomy.exclude
            for cid in class_ids
        ],
        dtype=bool,
    ).reshape(-1)


def gt_keep_indices(gt_labels: list[str], taxonomy: Taxonomy) -> list[int]:
    return [i for i, label in enumerate(gt_labels) if label not in taxonomy.exclude]


def detect(
    model,
    image: np.ndarray,
    class_labels: list[str],
    taxonomy: Taxonomy,
    args: argparse.Namespace,
    want_masks: bool,
):
    """Predict on one image and drop unknown and taxonomy-excluded classes."""
    boxes, scores, class_ids, masks = predict(model, image, args, want_masks)
    keep = keep_mask(class_ids, class_labels, taxonomy)
    labels = [class_labels[cid] for cid in class_ids[keep]]
    return boxes[keep], scores[keep], labels, (masks[keep] if masks is not None else None)


def gt_matches(
    gt_boxes: np.ndarray,
    gt_labels: list[str],
    boxes: np.ndarray,
    labels: list[str],
    scores: np.ndarray,
    iou: float,
) -> dict[int, int]:
    """{gt index: prediction index} for class-blind IoU matches at `iou`."""
    frame = Frame(
        gt_boxes=gt_boxes,
        gt_labels=gt_labels,
        gt_keypoints=[],
        pred_boxes=boxes,
        pred_labels=labels,
        pred_scores=scores,
        pred_keypoints=[],
    )
    return {g: p for g, p in match_indices(frame, iou) if g is not None and p is not None}


def centroid_rows(
    stamp: int,
    boxes: np.ndarray,
    scores: np.ndarray,
    labels: list[str],
    masks: np.ndarray | None,
) -> tuple[list[dict], np.ndarray]:
    """Per-detection offset rows, plus the (N, 2) centroids (NaN where no mask)."""
    centers = box_centers(boxes)
    sides = longer_sides(boxes)
    centroids = np.full_like(centers, np.nan)
    rows = []
    for i in range(len(boxes)):
        if masks is None or i >= len(masks):
            continue
        cx, cy, area = mask_centroid(masks[i])
        centroids[i] = (cx, cy)
        box_area = max((boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1]), 1e-9)
        offset = centroids[i] - centers[i]
        distance = float(np.hypot(*offset))
        rows.append(
            {
                "stamp": stamp,
                "label": labels[i],
                "conf": scores[i],
                "box_w": boxes[i, 2] - boxes[i, 0],
                "box_h": boxes[i, 3] - boxes[i, 1],
                "box_longer_side_px": sides[i],
                "mask_fill_frac": area / box_area,
                "dx_px": offset[0],
                "dy_px": offset[1],
                "offset_px": distance,
                "offset_norm": distance / max(sides[i], 1e-9),
            }
        )
    return rows, centroids


def keypoint_midpoints(gt_keypoints: list[np.ndarray]) -> np.ndarray:
    """(N, 2) front/back keypoint midpoint per GT box, NaN where either end is unlabeled.

    This is the point the aim controller actually wants: the middle of the robot chassis
    along its heading axis, which is not the same as the center of its bounding box."""
    mids = np.full((len(gt_keypoints), 2), np.nan)
    for i, kps in enumerate(gt_keypoints):
        if len(kps) <= max(FRONT_IDX, BACK_IDX):
            continue
        if kps[FRONT_IDX, 2] <= 0 or kps[BACK_IDX, 2] <= 0:
            continue
        mids[i] = (kps[FRONT_IDX, :2] + kps[BACK_IDX, :2]) / 2
    return mids


def error_rows(
    stamp: int,
    gt_labels: list[str],
    gt_sides: np.ndarray,
    references: dict[str, np.ndarray],
    estimates: dict[str, np.ndarray],
    pairs: dict[str, dict[int, int]],
) -> list[dict]:
    """One row per GT box: distance from each estimator to each reference point.

    `references` holds an (N, 2) target per GT box (the GT box center, the keypoint
    midpoint), `estimates` an (N, 2) position array per estimator, and `pairs` the
    matching {gt index: prediction index}. Emits `err_<reference>_<estimator>_px`, left
    empty when the estimator did not match that GT box or either point is NaN."""
    rows = []
    for g in range(len(gt_labels)):
        row: dict = {
            "stamp": stamp,
            "gt_label": gt_labels[g],
            "gt_longer_side_px": gt_sides[g],
            "seg_matched": g in pairs["centroid"],
            "bbox_matched": g in pairs["bbox_model"],
            "has_keypoints": bool(np.isfinite(references["kpmid"][g, 0])),
        }
        for ref_name, targets in references.items():
            if np.isnan(targets[g, 0]):
                continue
            for name, positions in estimates.items():
                p = pairs[name].get(g)
                if p is None or np.isnan(positions[p, 0]):
                    continue
                distance = float(np.hypot(*(positions[p] - targets[g])))
                row[f"err_{ref_name}_{name}_px"] = distance
        rows.append(row)
    return rows


def collect(args: argparse.Namespace) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Run both models over the GT frames.

    Returns (per-detection offsets for the seg model, per-matched-GT-box position errors)."""
    from ultralytics import YOLO

    gt_frames, names, images = load_gt(args.gt)
    taxonomy = Taxonomy(args.taxonomy)
    print(f"GT: {len(gt_frames)} frames, classes: {names}")

    seg_labels = [label.strip() for label in args.seg_labels.split(",")]
    seg_model = YOLO(str(args.seg_weights))
    if seg_model.task != "segment":
        raise SystemExit(f"{args.seg_weights} is a '{seg_model.task}' model, expected 'segment'")

    bbox_model = None
    bbox_labels: list[str] = []
    if args.bbox_weights:
        bbox_labels = [label.strip() for label in args.bbox_labels.split(",")]
        bbox_model = YOLO(str(args.bbox_weights))

    offset_rows = []
    matched_rows = []
    for stamp, (gt_boxes_all, gt_labels_all, gt_kps_all) in gt_frames.items():
        image = cv2.imread(str(images[stamp]))
        if image is None:
            raise SystemExit(f"Failed to read image {images[stamp]}")
        gt_idx = gt_keep_indices(gt_labels_all, taxonomy)
        gt_boxes = gt_boxes_all[gt_idx]
        gt_labels = [gt_labels_all[i] for i in gt_idx]
        gt_centers = box_centers(gt_boxes)
        gt_sides = longer_sides(gt_boxes)
        gt_mids = keypoint_midpoints([gt_kps_all[i] for i in gt_idx])

        boxes, scores, labels, masks = detect(
            seg_model, image, seg_labels, taxonomy, args, want_masks=True
        )
        centers = box_centers(boxes)
        rows, centroids = centroid_rows(stamp, boxes, scores, labels, masks)
        offset_rows.extend(rows)

        # Position error against the GT box center, over class-blind IoU matches.
        seg_pairs = gt_matches(gt_boxes, gt_labels, boxes, labels, scores, args.iou)

        bbox_pairs: dict[int, int] = {}
        bbox_centers = np.zeros((0, 2))
        if bbox_model is not None:
            b_boxes, b_scores, b_labels, _ = detect(
                bbox_model, image, bbox_labels, taxonomy, args, want_masks=False
            )
            bbox_centers = box_centers(b_boxes)
            bbox_pairs = gt_matches(gt_boxes, gt_labels, b_boxes, b_labels, b_scores, args.iou)

        # The GT box center is itself an estimator of the aim point, and the best any
        # box-based method can do: it is the label, with no detection error in it.
        identity = dict(enumerate(range(len(gt_boxes))))
        matched_rows.extend(
            error_rows(
                stamp,
                gt_labels,
                gt_sides,
                {"gtbox": gt_centers, "kpmid": gt_mids},
                {
                    "centroid": centroids,
                    "seg_boxcenter": centers,
                    "bbox_model": bbox_centers,
                    "gt_boxcenter": gt_centers,
                },
                {
                    "centroid": seg_pairs,
                    "seg_boxcenter": seg_pairs,
                    "bbox_model": bbox_pairs,
                    "gt_boxcenter": identity,
                },
            )
        )

    offsets = pd.DataFrame(offset_rows)
    matched = pd.DataFrame(matched_rows)
    for column in [c for c in matched.columns if c.endswith("_px") and c.startswith("err_")]:
        matched[column.replace("_px", "_norm")] = matched[column] / matched["gt_longer_side_px"]
    return offsets, matched


def describe(values: np.ndarray) -> dict[str, float]:
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return {}
    return {
        "n": len(values),
        "mean": float(values.mean()),
        "median": float(np.median(values)),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
        "max": float(values.max()),
    }


def bootstrap_paired_delta(
    a: np.ndarray, b: np.ndarray, resamples: int, seed: int, alpha: float
) -> tuple[float, float, float]:
    """Mean of (a - b) with a percentile CI, resampling the pairs."""
    finite = np.isfinite(a) & np.isfinite(b)
    diff = a[finite] - b[finite]
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(diff), size=(resamples, len(diff)))
    samples = diff[idx].mean(axis=1)
    return (
        float(diff.mean()),
        float(np.percentile(samples, 100 * alpha / 2)),
        float(np.percentile(samples, 100 * (1 - alpha / 2))),
    )


def summarize(
    offsets: pd.DataFrame, matched: pd.DataFrame, args: argparse.Namespace
) -> pd.DataFrame:
    rows = []
    for name, values in (
        ("centroid_to_boxcenter_px", offsets["offset_px"].to_numpy()),
        ("centroid_to_boxcenter_norm", offsets["offset_norm"].to_numpy()),
        ("mask_fill_frac", offsets["mask_fill_frac"].to_numpy()),
    ):
        rows.append({"quantity": name, **describe(values)})
    for suffix in ("_norm", "_px"):
        for reference in REFERENCES:
            for estimator in ESTIMATORS:
                column = f"err_{reference}_{estimator}{suffix}"
                if column in matched and (reference, estimator) not in DEGENERATE:
                    rows.append({"quantity": column, **describe(matched[column].to_numpy())})
    summary = pd.DataFrame(rows)

    print("\nMask centroid vs its own box center, per seg detection:")
    for threshold in NEAR_THRESHOLDS:
        share = float((offsets["offset_norm"] <= threshold).mean())
        print(f"  within {threshold:.2f} of the box's longer side: {share:.3f}")
    print(summary.to_string(index=False, float_format=lambda v: f"{v:.4f}"))

    conf_pct = round(100 * (1 - args.alpha))
    for reference, reference_label in REFERENCES.items():
        print(f"\nError vs {reference_label}, paired over GT boxes both matched ({conf_pct}% CI):")
        print_comparisons(matched, reference, args)
    return summary


def print_comparisons(matched: pd.DataFrame, reference: str, args: argparse.Namespace) -> None:
    """Paired-bootstrap delta between every estimator pair, under one reference point."""
    for a, b in COMPARISONS:
        if (reference, a) in DEGENERATE or (reference, b) in DEGENERATE:
            continue
        a_col = f"err_{reference}_{a}_px"
        b_col = f"err_{reference}_{b}_px"
        if a_col not in matched or b_col not in matched:
            continue
        pairs = matched[[a_col, b_col]].dropna()
        if pairs.empty:
            continue
        delta, low, high = bootstrap_paired_delta(
            pairs[a_col].to_numpy(),
            pairs[b_col].to_numpy(),
            args.bootstrap,
            args.seed,
            args.alpha,
        )
        better = ESTIMATORS[a] if delta < 0 else ESTIMATORS[b]
        verdict = "ns" if low <= 0 <= high else f"{better} better"
        print(
            f"  {ESTIMATORS[a]} - {ESTIMATORS[b]} (n={len(pairs)}): {delta:+.3f} px "
            f"[{low:+.3f}, {high:+.3f}] -> {verdict}"
        )


def _ecdf(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    values = np.sort(values[np.isfinite(values)])
    return values, np.arange(1, len(values) + 1) / len(values)


def _style(ax: plt.Axes) -> None:
    ax.grid(axis="both", color=GRID_COLOR, alpha=0.5, linewidth=0.8)
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(GRID_COLOR)
    ax.tick_params(colors=TEXT_SECONDARY)
    ax.yaxis.label.set_color(TEXT_SECONDARY)
    ax.xaxis.label.set_color(TEXT_SECONDARY)


def plot(offsets: pd.DataFrame, matched: pd.DataFrame, output: Path) -> None:
    fig, (left, right) = plt.subplots(1, 2, figsize=(12, 4.8))

    x, y = _ecdf(offsets["offset_norm"].to_numpy())
    left.plot(x, y, color=SERIES_COLORS[0], linewidth=2)
    median = float(np.median(x))
    left.axvline(median, color=TEXT_SECONDARY, linewidth=1, linestyle="--")
    left.annotate(
        f"median {median:.3f}",
        xy=(median, 0.5),
        xytext=(8, -14),
        textcoords="offset points",
        color=TEXT_SECONDARY,
        fontsize=9,
    )
    left.set_xlim(0, min(0.3, float(np.percentile(x, 99.5))))
    left.set_ylim(0, 1.02)
    left.set_xlabel("centroid offset from box center (fraction of box's longer side)")
    left.set_ylabel("fraction of detections")
    left.set_title("Mask centroid sits on the box center", loc="left")
    _style(left)

    series = [
        ("centroid", SERIES_COLORS[0]),
        ("seg_boxcenter", SERIES_COLORS[1]),
        ("bbox_model", SERIES_COLORS[2]),
    ]
    for estimator, color in series:
        column = f"err_gtbox_{estimator}_norm"
        if column not in matched:
            continue
        x, y = _ecdf(matched[column].to_numpy())
        if len(x) == 0:
            continue
        right.plot(x, y, color=color, linewidth=2, label=ESTIMATORS[estimator])
    right.set_xlim(0, 0.25)
    right.set_ylim(0, 1.02)
    right.set_xlabel("distance to ground-truth box center (fraction of GT box's longer side)")
    right.set_ylabel("fraction of matched robots")
    right.set_title("All three position estimates land together", loc="left")
    right.legend(loc="lower right", frameon=False)
    _style(right)

    fig.tight_layout()
    fig.savefig(output / "mask_centroid_vs_box.png", dpi=150)
    plt.close(fig)


def plot_keypoint_reference(matched: pd.DataFrame, output: Path) -> None:
    """Distance from each position estimate to the keypoint midpoint, the true aim point.

    A dot plot rather than overlaid ECDFs: four categories against one measure reads
    better as rows, and it keeps the series count inside the validated palette's
    all-pairs cap."""
    estimators = [e for e in ESTIMATORS if f"err_kpmid_{e}_norm" in matched]
    if not estimators:
        return
    fig, ax = plt.subplots(figsize=(9, 0.85 * len(estimators) + 2.2))
    positions = np.arange(len(estimators))[::-1]
    for pos, estimator in zip(positions, estimators):
        values = matched[f"err_kpmid_{estimator}_norm"].dropna().to_numpy()
        median = float(np.median(values))
        p10, p90 = np.percentile(values, [10, 90])
        ax.plot([p10, p90], [pos, pos], color=SERIES_COLORS[0], linewidth=2, alpha=0.35)
        ax.plot([median], [pos], "o", color=SERIES_COLORS[0], markersize=10)
        ax.annotate(
            f"{median:.3f}  (n={len(values)})",
            xy=(median, pos),
            xytext=(0, 14),
            textcoords="offset points",
            ha="center",
            color=TEXT_SECONDARY,
            fontsize=9,
        )
    ax.set_yticks(positions, [ESTIMATORS[e] for e in estimators])
    ax.set_ylim(-0.6, len(estimators) - 0.4)
    ax.set_xlim(left=0)
    ax.set_xlabel(
        "distance to keypoint front/back midpoint (fraction of GT box's longer side)\n"
        "dot = median, bar = p10-p90"
    )
    ax.set_title("Every box-based estimate misses the aim point by the same margin", loc="left")
    _style(ax)
    ax.grid(axis="y", visible=False)
    fig.tight_layout()
    fig.savefig(output / "keypoint_midpoint_error.png", dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("gt", type=Path, help="GT dataset dir, or a root of subdatasets")
    parser.add_argument("--seg-weights", type=Path, required=True, help="YOLO-seg .pt weights")
    parser.add_argument(
        "--seg-labels", required=True, help="comma-separated GT label per seg class index"
    )
    parser.add_argument("--bbox-weights", type=Path, help="optional detect-only .pt weights")
    parser.add_argument("--bbox-labels", help="comma-separated GT label per bbox class index")
    parser.add_argument("--taxonomy", type=Path, help="label -> archetype mapping yaml")
    parser.add_argument("--iou", type=float, default=0.5, help="IoU match threshold")
    parser.add_argument("--conf", type=float, default=0.5, help="inference confidence threshold")
    parser.add_argument("--nms-iou", type=float, default=0.45, help="inference NMS IoU threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="inference image size")
    parser.add_argument("--bootstrap", type=int, default=1000, help="paired-bootstrap resamples")
    parser.add_argument("--seed", type=int, default=0, help="bootstrap RNG seed")
    parser.add_argument("--alpha", type=float, default=0.05, help="significance level")
    parser.add_argument(
        "--output", type=Path, help="output dir (default: <gt>/scores_mask_centroid)"
    )
    args = parser.parse_args()
    if args.bbox_weights and not args.bbox_labels:
        raise SystemExit("--bbox-weights requires --bbox-labels")
    args.output = args.output or (args.gt / "scores_mask_centroid")
    args.output.mkdir(parents=True, exist_ok=True)

    offsets, matched = collect(args)
    summary = summarize(offsets, matched, args)
    offsets.to_csv(args.output / "centroid_offsets.csv", index=False)
    matched.to_csv(args.output / "position_errors.csv", index=False)
    summary.to_csv(args.output / "centroid_summary.csv", index=False)
    plot(offsets, matched, args.output)
    plot_keypoint_reference(matched, args.output)
    print(
        f"\nWrote {args.output}/{{centroid_offsets.csv, position_errors.csv, "
        "centroid_summary.csv, mask_centroid_vs_box.png, keypoint_midpoint_error.png}"
    )


if __name__ == "__main__":
    main()
