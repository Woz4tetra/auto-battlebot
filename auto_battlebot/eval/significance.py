"""Paired bootstrap: does a candidate actually differ from the baseline?

Every candidate is scored on the same GT frames, so resampling frames paired cancels the
"this frame is just hard" variance and a few hundred frames can resolve real differences.
"""

from __future__ import annotations

import argparse

import numpy as np
import pandas as pd

from auto_battlebot.eval.dataset import LEVELS
from auto_battlebot.eval.metrics import (
    HEADING_ACC_KEY,
    PCK_KEY,
    keypoint_metrics_from_stats,
)

HIGHER_IS_BETTER = {
    "recall",
    "precision",
    "f1",
    "localization_recall",
    PCK_KEY,
    HEADING_ACC_KEY,
}

# Detection metrics tested per taxonomy level; keypoint metrics are level-independent.
DETECTION_METRICS = ("recall", "precision", "f1", "localization_recall")
KEYPOINT_METRICS = ("kp_err_px", PCK_KEY, "kp_heading_err_deg", HEADING_ACC_KEY)


def _resample_ratio(num: np.ndarray, den: np.ndarray, idx: np.ndarray) -> np.ndarray:
    """Ratio sum(num)/sum(den) over each resampled frame set; nan where den == 0.

    idx is (B, n_frames); num/den are per-frame arrays. Fully vectorized."""
    n = num[idx].sum(axis=1)
    d = den[idx].sum(axis=1)
    return np.where(d > 0, n / np.where(d > 0, d, 1.0), np.nan)


def _metric_samples(stats: dict, metric: str, level: str, idx: np.ndarray) -> np.ndarray:
    """Bootstrap distribution (length B) of `metric`, recomputed on each resample in idx.

    Pass a (1, n_frames) arange for the full-sample point estimate."""
    if metric in DETECTION_METRICS:
        c = stats["pr"][level]
        tp, fp, fn, wr = c["tp"], c["fp"], c["fn"], c["wrong"]
        if metric == "recall":
            return _resample_ratio(tp, tp + wr + fn, idx)
        if metric == "precision":
            return _resample_ratio(tp, tp + wr + fp, idx)
        if metric == "localization_recall":
            return _resample_ratio(tp + wr, tp + wr + fn, idx)
        # f1 from precision and recall on the same resample
        p = _resample_ratio(tp, tp + wr + fp, idx)
        r = _resample_ratio(tp, tp + wr + fn, idx)
        s = p + r
        return np.where(s > 0, 2 * p * r / np.where(s > 0, s, 1.0), 0.0)
    k = stats["kp"]
    if metric == "kp_err_px":
        return _resample_ratio(k["err_sum"], k["err_cnt"], idx)
    if metric == PCK_KEY:
        return _resample_ratio(k["pck_cnt"], k["err_cnt"], idx)
    if metric == "kp_heading_err_deg":
        return _resample_ratio(k["head_err_sum"], k["head_cnt"], idx)
    if metric == HEADING_ACC_KEY:
        return _resample_ratio(k["head_correct"], k["head_cnt"], idx)
    raise KeyError(metric)


def run_significance(
    stats_by_candidate: dict[str, dict], baseline: str, args: argparse.Namespace
) -> pd.DataFrame:
    """Paired-bootstrap each candidate against the baseline on the same resampled frames.

    Reports delta (candidate - baseline) with an (1 - alpha) CI and a verdict: "better"/
    "worse" when the CI excludes 0 (direction from metric polarity), "ns" otherwise."""
    n = stats_by_candidate[baseline]["n_frames"]
    rng = np.random.default_rng(args.seed)
    idx = rng.integers(0, n, size=(args.bootstrap, n))
    full = np.arange(n)[None, :]
    lo_pct, hi_pct = 100 * args.alpha / 2, 100 * (1 - args.alpha / 2)

    plan = [(m, level) for level in LEVELS for m in DETECTION_METRICS]
    # Keypoint metrics exist when the baseline engine carries keypoints and boxes matched.
    available = keypoint_metrics_from_stats(stats_by_candidate[baseline]["kp"])
    plan += [(m, "-") for m in KEYPOINT_METRICS if m in available]

    rows = []
    base = stats_by_candidate[baseline]
    for name, cand in stats_by_candidate.items():
        if name == baseline:
            continue
        for metric, level in plan:
            base_pt = float(_metric_samples(base, metric, level, full)[0])
            cand_pt = float(_metric_samples(cand, metric, level, full)[0])
            diff = _metric_samples(cand, metric, level, idx) - _metric_samples(
                base, metric, level, idx
            )
            if np.all(np.isnan(diff)):
                lo = hi = float("nan")
                verdict = "n/a"
            else:
                lo, hi = (float(v) for v in np.nanpercentile(diff, [lo_pct, hi_pct]))
                if lo > 0 or hi < 0:
                    improved = (cand_pt - base_pt > 0) == (metric in HIGHER_IS_BETTER)
                    verdict = "better" if improved else "worse"
                else:
                    verdict = "ns"
            rows.append(
                {
                    "candidate": name,
                    "baseline": baseline,
                    "level": level,
                    "metric": metric,
                    "baseline_val": base_pt,
                    "candidate_val": cand_pt,
                    "delta": cand_pt - base_pt,
                    "ci_low": lo,
                    "ci_high": hi,
                    "verdict": verdict,
                }
            )
    return pd.DataFrame(rows)
