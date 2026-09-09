"""Detector scoring: run candidate engines on a labeled dataset and compare them.

dataset       label taxonomy, frame records, and loading GT from disk
detectors     detector variants and the inference loop over GT images
metrics       detection and keypoint metrics over matched frames
significance  paired bootstrap over frame-decomposable metrics
plots         summary figures
scoring       score_candidate, the per-candidate orchestrator

The names below are the public surface; the command line over them is
`training/model_eval/score.py`.
"""

from auto_battlebot.eval.dataset import (
    AGNOSTIC_LABEL,
    LEVELS,
    Frame,
    GtFrame,
    Taxonomy,
    load_gt,
    reviewed_stems,
)
from auto_battlebot.eval.detectors import (
    Detector,
    EngineDetector,
    FieldCropDetector,
    PrecomputedDetector,
    build_detector,
    infer_frames,
    parse_candidates,
)
from auto_battlebot.eval.metrics import (
    BACK_IDX,
    FRONT_IDX,
    HEADING_ACC_KEY,
    PCK_KEY,
    compute_map,
    iou_matrix,
    keypoint_metrics_from_stats,
    keypoint_per_frame,
    match_frame,
    match_indices,
    pr_from_counts,
    pr_per_frame,
)
from auto_battlebot.eval.plots import plot_confusion, plot_headline
from auto_battlebot.eval.scoring import score_candidate
from auto_battlebot.eval.significance import run_significance

__all__ = [
    "AGNOSTIC_LABEL",
    "BACK_IDX",
    "FRONT_IDX",
    "HEADING_ACC_KEY",
    "LEVELS",
    "PCK_KEY",
    "Detector",
    "EngineDetector",
    "FieldCropDetector",
    "Frame",
    "GtFrame",
    "PrecomputedDetector",
    "Taxonomy",
    "build_detector",
    "compute_map",
    "infer_frames",
    "iou_matrix",
    "keypoint_metrics_from_stats",
    "keypoint_per_frame",
    "load_gt",
    "match_frame",
    "match_indices",
    "parse_candidates",
    "plot_confusion",
    "plot_headline",
    "pr_from_counts",
    "pr_per_frame",
    "reviewed_stems",
    "run_significance",
    "score_candidate",
]
