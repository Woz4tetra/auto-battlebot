#!/usr/bin/env python3
"""Score candidate TensorRT detector engines against a YOLO ground-truth dataset.

Each candidate engine runs inference directly on the GT images (no playback, no stamp
alignment), matching the C++ pipeline's preprocessing and NMS. Predictions are compared
against the labels at three label levels:

    agnostic   every label collapses to "robot": pure localization (did it find the robot)
    archetype  labels map through taxonomy.yaml: NHRL archetype naming
    instance   labels as-is: per-robot-instance naming

The gap between agnostic recall and class-aware mAP is the cost of splitting the
OPPONENT category. Outputs a summary table (stdout + summary.csv) and plots.

The GT argument is either a single dataset dir (data.yaml + images/ + labels/) or a root
containing such subdatasets (e.g. training/data/nhrl_keypoints_eval_test). If a
validation_state.json is present, only frames it marks `pass` are scored; failing that, an
edit_labels.py .edit_state.json is used the same way.

--labels maps engine class indices to GT label names, in class order (mirrors the C++
label_indices config). When the engine head carries keypoints (YOLO-pose), keypoint
metrics are added over IoU-matched boxes: mean pixel error, PCK@0.1 of the GT box's
longer side, and heading error (angle of the front->back keypoint vector, mean degrees
and accuracy within 10 degrees).

With two or more candidates, a paired bootstrap reports whether each candidate differs
significantly from the baseline (the first candidate, or --baseline NAME). Because every
candidate is scored on the same GT frames, resampling frames paired cancels the
"this frame is just hard" variance, so a few hundred frames can resolve real differences.
mAP is dataset-level and left as a point estimate; the detection questions are covered by
the recall / precision / localization tests, which are frame-decomposable.

Usage:
    python training/model_eval/score.py training/data/nhrl_keypoints_eval_test \
        --candidate deployed=data/models/<model>_x86_64_sm89.engine \
        --labels opponent,opponent,house_bot,mr_stabs_mk2,mrs_buff_mk3 \
        [--taxonomy training/model_eval/taxonomy.yaml] [--conf 0.5] [--nms-iou 0.45] \
        [--iou 0.5] [--baseline NAME] [--bootstrap 1000] [--seed 0] [--alpha 0.05] \
        [--output <dir>]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

from auto_battlebot.eval import (
    Taxonomy,
    load_gt,
    parse_candidates,
    plot_headline,
    run_significance,
    score_candidate,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument(
        "gt", type=Path, help="GT dataset dir, or a root of subdatasets (from edit_labels.py)"
    )
    parser.add_argument(
        "--candidate",
        action="append",
        required=True,
        metavar="NAME=ENGINE",
        help="candidate TensorRT engine, repeatable (bare path uses the file stem as name)",
    )
    parser.add_argument(
        "--labels",
        required=True,
        help="comma-separated GT label per engine class index (the C++ label_indices map)",
    )
    parser.add_argument(
        "--stretch",
        action="append",
        default=[],
        metavar="NAME",
        help="candidate trained on anisotropically-resized images; fit frames to the tensor "
        "by stretching instead of letterboxing. Repeatable.",
    )
    parser.add_argument(
        "--candidate-labels",
        action="append",
        default=[],
        metavar="NAME=LABELS",
        help="comma-separated GT label per engine class index for one candidate, overriding "
        "--labels. A grid that mixes 2-class and 3-class engines needs this: the label count "
        "sets num_classes, which is what splits the raw tensor into scores and keypoints, so "
        "a 2-class engine read with three labels parses to num_keypoints=0. Repeatable.",
    )
    parser.add_argument(
        "--field-boxes",
        action="append",
        default=[],
        metavar="NAME=JSON",
        help="candidate trained on field-cropped images; crop each frame to the box for its "
        "stem in JSON before inference. Written by build_field_crop_dataset.py masks "
        "--recursive. Repeatable.",
    )
    parser.add_argument(
        "--crop-margin",
        type=float,
        default=0.20,
        help="margin around the field box for --field-boxes candidates, as a fraction of the "
        "box per side. Must match the margin the arm's training corpus was built with.",
    )
    parser.add_argument("--taxonomy", type=Path, help="label -> archetype mapping yaml")
    parser.add_argument("--iou", type=float, default=0.5, help="IoU match threshold")
    parser.add_argument(
        "--conf", type=float, default=0.5, help="inference confidence threshold (default: 0.5)"
    )
    parser.add_argument(
        "--nms-iou", type=float, default=0.45, help="inference NMS IoU threshold (default: 0.45)"
    )
    parser.add_argument(
        "--baseline",
        help="candidate name to compare the others against (default: first --candidate)",
    )
    parser.add_argument(
        "--bootstrap",
        type=int,
        default=1000,
        help="paired-bootstrap resamples for significance (0 disables)",
    )
    parser.add_argument("--seed", type=int, default=0, help="bootstrap RNG seed")
    parser.add_argument(
        "--alpha", type=float, default=0.05, help="significance level (0.05 = 95%% CI)"
    )
    parser.add_argument("--output", type=Path, help="plot/csv output dir (default: <gt>/scores)")
    args = parser.parse_args()
    args.output = args.output or (args.gt / "scores")
    args.output.mkdir(parents=True, exist_ok=True)

    gt_frames, names, images = load_gt(args.gt)
    taxonomy = Taxonomy(args.taxonomy)
    print(f"GT: {len(gt_frames)} frames, classes: {names}")

    class_labels = [label.strip() for label in args.labels.split(",")]
    label_overrides = {}
    for entry in args.candidate_labels:
        name, sep, labels = entry.partition("=")
        if not sep or not labels.strip():
            raise SystemExit(f"--candidate-labels wants NAME=LABELS, got {entry!r}")
        label_overrides[name] = [label.strip() for label in labels.split(",")]
    candidates = parse_candidates(args.candidate)
    stray = sorted(set(label_overrides) - set(candidates))
    if stray:
        raise SystemExit(f"--candidate-labels {stray} are not candidates: {list(candidates)}")
    for labels in [class_labels, *label_overrides.values()]:
        unknown = sorted(set(labels) - set(names) - set(taxonomy.exclude))
        if unknown:
            print(f"Warning: labels {unknown} not in GT classes; they can only score as FP")

    rows = []
    stats_by_candidate: dict[str, dict] = {}
    for name, engine_path in candidates.items():
        print(f"Scoring {name}: {engine_path}")
        candidate_rows, stats = score_candidate(
            name,
            engine_path,
            gt_frames,
            images,
            label_overrides.get(name, class_labels),
            taxonomy,
            args,
        )
        rows.extend(candidate_rows)
        stats_by_candidate[name] = stats

    summary = pd.DataFrame(rows)
    core_cols = [
        "candidate",
        "level",
        "map50",
        "map50_95",
        "precision",
        "recall",
        "f1",
        "localization_recall",
        "wrong_class_rate",
    ]
    core_cols += [c for c in summary.columns if c.startswith("kp_")]
    print(summary[core_cols].to_string(index=False, float_format=lambda v: f"{v:.3f}"))
    summary.to_csv(args.output / "summary.csv", index=False)
    plot_headline(summary, args.output)

    outputs = "summary.csv, headline.png, confusion_*.png"
    baseline = args.baseline or next(iter(stats_by_candidate))
    if args.baseline and args.baseline not in stats_by_candidate:
        raise SystemExit(f"--baseline {args.baseline!r} is not one of {list(stats_by_candidate)}")
    if len(stats_by_candidate) >= 2 and args.bootstrap > 0:
        significance = run_significance(stats_by_candidate, baseline, args)
        conf_pct = round(100 * (1 - args.alpha))
        print(
            f"\nPaired bootstrap vs baseline '{baseline}' "
            f"({args.bootstrap} resamples, {conf_pct}% CI on delta = candidate - baseline):"
        )
        print(
            significance.to_string(
                index=False,
                float_format=lambda v: f"{v:.3f}",
                columns=[
                    "candidate",
                    "level",
                    "metric",
                    "baseline_val",
                    "candidate_val",
                    "delta",
                    "ci_low",
                    "ci_high",
                    "verdict",
                ],
            )
        )
        significance.to_csv(args.output / "significance.csv", index=False)
        outputs += ", significance.csv"
    elif len(stats_by_candidate) < 2:
        print("\n(Only one candidate; add another --candidate for a significance comparison.)")

    print(f"Wrote {args.output}/{{{outputs}}}")


if __name__ == "__main__":
    main()
