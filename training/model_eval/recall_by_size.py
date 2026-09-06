#!/usr/bin/env python3
"""Agnostic recall split by ground-truth object size, for two or more engines.

`score.py` reports one recall over the whole eval set, which answers whether an arm is
better but not where it differs. Quantization is expected to fail unevenly: a distant robot
produces low-amplitude activations, and if a tensor's INT8 scale is set wide those
activations round to zero before they reach the head, while a close robot survives. That is
a claim about the size distribution of the misses, and a single recall number cannot
confirm or refute it.

Boxes are binned by sqrt(area) in source pixels, the same measure COCO's small/medium/large
split uses (32 px and 96 px), so a bin edge means the same thing here as it does there.
Matching is `score.py`'s own class-blind greedy IoU matcher at the same threshold, so the
totals in the last row reproduce that report's agnostic recall.

Usage:
    venv/bin/python training/model_eval/recall_by_size.py training/data/nhrl_keypoints_eval_test \
        --candidate B16=data/models/<fp16>.engine \
        --candidate B8=data/models/<int8>.engine \
        --labels "opponent,house_bot" --taxonomy training/model_eval/taxonomy_merged.yaml \
        --conf 0.5 --baseline B16 --output <dir>
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from score import Taxonomy, build_detector, infer_frames, load_gt, match_indices, parse_candidates

DEFAULT_EDGES = "16,24,32,48,64"


def bin_labels(edges: list[float]) -> list[str]:
    """Human-readable name per bin, in the order `np.digitize` produces."""
    names = [f"<{edges[0]:g}"]
    names += [f"{lo:g}-{hi:g}" for lo, hi in zip(edges, edges[1:])]
    names.append(f">{edges[-1]:g}")
    return names


def per_frame_counts(
    frames: list, edges: list[float], iou_threshold: float
) -> tuple[np.ndarray, np.ndarray]:
    """(hits, totals) arrays of shape (num_frames, num_bins), GT boxes binned by sqrt-area.

    Kept per frame rather than summed so the bootstrap can resample frames and re-sum, which
    is what makes the resampling paired across candidates.
    """
    num_bins = len(edges) + 1
    hits = np.zeros((len(frames), num_bins))
    totals = np.zeros((len(frames), num_bins))
    for index, frame in enumerate(frames):
        if len(frame.gt_boxes) == 0:
            continue
        widths = frame.gt_boxes[:, 2] - frame.gt_boxes[:, 0]
        heights = frame.gt_boxes[:, 3] - frame.gt_boxes[:, 1]
        which = np.digitize(np.sqrt(np.clip(widths * heights, 0, None)), edges)
        # match_indices reports an unmatched GT box as (g, None), so both halves must be
        # present for it to count as found.
        matched = {
            g for g, p in match_indices(frame, iou_threshold) if g is not None and p is not None
        }
        for gt_index, bin_index in enumerate(which):
            totals[index, bin_index] += 1
            if gt_index in matched:
                hits[index, bin_index] += 1
    return hits, totals


def bootstrap_delta(
    base: tuple[np.ndarray, np.ndarray],
    other: tuple[np.ndarray, np.ndarray],
    bin_index: int,
    draws: int,
    seed: int,
    alpha: float,
) -> tuple[float, float]:
    """Percentile CI on the recall difference in one bin, resampling frames paired."""
    rng = np.random.default_rng(seed)
    num_frames = base[0].shape[0]
    samples = np.empty(draws)
    for draw in range(draws):
        idx = rng.integers(0, num_frames, num_frames)
        base_total = base[1][idx, bin_index].sum()
        other_total = other[1][idx, bin_index].sum()
        if base_total == 0 or other_total == 0:
            samples[draw] = np.nan
            continue
        samples[draw] = (
            other[0][idx, bin_index].sum() / other_total
            - base[0][idx, bin_index].sum() / base_total
        )
    finite = samples[np.isfinite(samples)]
    if finite.size == 0:
        return float("nan"), float("nan")
    return (
        float(np.percentile(finite, 100 * alpha / 2)),
        float(np.percentile(finite, 100 * (1 - alpha / 2))),
    )


def plot_recall_by_size(table: pd.DataFrame, names: list[str], output: Path) -> None:
    """Grouped bars, one group per size bin, annotated with the GT count in that bin."""
    bins = list(table["bin"])
    positions = np.arange(len(bins))
    width = 0.8 / max(1, len(names))
    _, axis = plt.subplots(figsize=(1.6 * len(bins) + 3, 4.2))
    for offset, name in enumerate(names):
        axis.bar(positions + offset * width, table[f"{name}_recall"], width, label=name)
    axis.set_xticks(positions + width * (len(names) - 1) / 2)
    axis.set_xticklabels([f"{b}\nn={int(n)}" for b, n in zip(bins, table["gt_boxes"])])
    axis.set_xlabel("GT sqrt(area), source pixels")
    axis.set_ylabel("agnostic recall")
    axis.set_ylim(0, 1)
    axis.grid(axis="y", alpha=0.3)
    axis.legend()
    plt.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output, dpi=140)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gt", type=Path, help="eval dataset root")
    parser.add_argument("--candidate", action="append", default=[], metavar="NAME=ENGINE")
    parser.add_argument("--labels", required=True, help="GT label per engine class, in order")
    parser.add_argument("--taxonomy", type=Path, help="label -> archetype mapping yaml")
    parser.add_argument("--conf", type=float, default=0.5, help="confidence threshold")
    parser.add_argument("--nms-iou", type=float, default=0.45, help="NMS IoU threshold")
    parser.add_argument("--iou", type=float, default=0.5, help="match IoU threshold")
    parser.add_argument(
        "--bins", default=DEFAULT_EDGES, help=f"bin edges (default: {DEFAULT_EDGES})"
    )
    parser.add_argument("--baseline", help="candidate the deltas are measured against")
    parser.add_argument("--bootstrap", type=int, default=1000, help="paired bootstrap draws")
    parser.add_argument("--seed", type=int, default=0, help="bootstrap RNG seed")
    parser.add_argument("--alpha", type=float, default=0.05, help="1 - CI coverage")
    parser.add_argument("--output", type=Path, required=True, help="csv/plot output dir")
    # build_detector reads these; every arm here is letterbox, but the attributes must exist.
    parser.set_defaults(stretch=None, field_boxes=None, crop_margin=0.0)
    args = parser.parse_args()

    candidates = parse_candidates(args.candidate)
    if len(candidates) < 2:
        raise SystemExit("pass at least two --candidate NAME=ENGINE")
    baseline = args.baseline or next(iter(candidates))
    if baseline not in candidates:
        raise SystemExit(f"--baseline {baseline} is not one of {list(candidates)}")
    edges = [float(edge) for edge in args.bins.split(",")]
    class_labels = [label.strip() for label in args.labels.split(",")]

    gt_frames, names, images = load_gt(args.gt)
    print(f"GT: {len(gt_frames)} frames, classes: {names}")
    taxonomy = Taxonomy(args.taxonomy)

    counts: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for name, engine_path in candidates.items():
        print(f"{name}: {engine_path}")
        detector = build_detector(name, engine_path, class_labels, images, args)
        frames = infer_frames(gt_frames, images, detector, class_labels, taxonomy)
        counts[name] = per_frame_counts(frames, edges, args.iou)
        del detector

    rows = []
    for bin_index, bin_name in enumerate(bin_labels(edges)):
        total = counts[baseline][1][:, bin_index].sum()
        row: dict[str, object] = {"bin": bin_name, "gt_boxes": int(total)}
        for name, (hits, totals) in counts.items():
            row[f"{name}_recall"] = hits[:, bin_index].sum() / max(1.0, totals[:, bin_index].sum())
        for name in counts:
            if name == baseline:
                continue
            delta = float(row[f"{name}_recall"]) - float(row[f"{baseline}_recall"])  # type: ignore[arg-type]
            low, high = bootstrap_delta(
                counts[baseline], counts[name], bin_index, args.bootstrap, args.seed, args.alpha
            )
            row[f"{name}_delta"] = delta
            row[f"{name}_ci"] = f"[{low:+.3f}, {high:+.3f}]"
            row[f"{name}_verdict"] = (
                "ns" if low <= 0 <= high else ("better" if delta > 0 else "worse")
            )
        rows.append(row)

    table = pd.DataFrame(rows)
    args.output.mkdir(parents=True, exist_ok=True)
    table.to_csv(args.output / "recall_by_size.csv", index=False)
    plot_recall_by_size(table, list(counts), args.output / "recall_by_size.png")
    print()
    print(table.to_string(index=False))
    print(f"\nwrote {args.output}/recall_by_size.csv and recall_by_size.png")


if __name__ == "__main__":
    main()
