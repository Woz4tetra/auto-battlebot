#!/usr/bin/env python3
"""Stage 3 of the embedding prototype probe: Evals A, B, and D on GT frames.

Per (embedder, padding, recording), seeds a gallery from the seed-frame opponent crops
and scores every eval-frame candidate by max cosine similarity to the gallery. Reports:

- Eval A: ROC AUC of GT opponent boxes vs all negatives (GT negatives + hard-negative
  proposals with IoU < 0.3 to every GT box), top-1 identity rate per frame, and the
  similarity-vs-match-time trend.
- Eval B: of GT opponents the engine misses at the deployed threshold (0.6), how many
  have a low-conf proposal (IoU >= 0.5) whose similarity clears the operating point.
  The operating point is the pooled negative 98th percentile per (embedder, padding):
  2 false accepts per 100 negative candidates, matching the plan's budget.
- Eval D (hard-negative half): AUC of opponent vs the hard-negative proposal pool alone
  and the false-accept rate at the operating point. The static-FP half needs the
  full-playback pass (stage 4).

Sensitivity variants at 10% padding: full gallery vs single mean prototype vs seeding
from only the first seed frame.

Writes training/data/embedding_probe/metrics.csv and figures/, prints the pad010
gallery-variant table.

Usage:
    cd training/model_eval && python embedding_probe_score.py
"""

from __future__ import annotations

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import embedding_probe_common as common
import matplotlib.pyplot as plt

EMBEDDER_NAMES = ("dinov2", "clip", "resnet50")
PAD_KEYS = {"pad000": 0.0, "pad010": 0.10, "pad025": 0.25}
DEFAULT_PAD_KEY = "pad010"
FALSE_ACCEPT_BUDGET = 0.02  # operating point: fraction of pooled negatives accepted


def roc_auc(pos: np.ndarray, neg: np.ndarray) -> float:
    """Mann-Whitney AUC with tie handling; nan when either side is empty."""
    if len(pos) == 0 or len(neg) == 0:
        return float("nan")
    scores = np.concatenate([pos, neg])
    order = scores.argsort(kind="mergesort")
    ranks = np.empty(len(scores))
    ranks[order] = np.arange(1, len(scores) + 1)
    # average ranks for ties
    sorted_scores = scores[order]
    start = 0
    for end in range(1, len(scores) + 1):
        if end == len(scores) or sorted_scores[end] != sorted_scores[start]:
            ranks[order[start:end]] = 0.5 * (start + 1 + end)
            start = end
    rank_sum = ranks[: len(pos)].sum()
    u_stat = rank_sum - len(pos) * (len(pos) + 1) / 2
    return float(u_stat / (len(pos) * len(neg)))


def aligned_features(recording: str, embedder: str, pad_key: str, n_rows: int) -> np.ndarray:
    npz = np.load(common.recording_dir(recording) / common.EMBEDDINGS_NPZ.format(embedder=embedder))
    features = npz[pad_key]
    order = npz["row_index"]
    aligned = np.zeros((n_rows, features.shape[1]), dtype=np.float32)
    aligned[order] = features
    return aligned


def gallery_vectors(candidates: pd.DataFrame, features: np.ndarray, variant: str) -> np.ndarray:
    seed_mask = (
        (candidates.role == "seed")
        & (candidates.source == "gt")
        & (candidates.label == common.OPPONENT)
    )
    seed_rows = candidates.index[seed_mask].to_numpy()
    if variant == "seed1":
        first_stamp = candidates.loc[seed_rows, "stamp_ns"].min()
        seed_rows = candidates.index[seed_mask & (candidates.stamp_ns == first_stamp)].to_numpy()
    vectors = features[seed_rows]
    if variant == "mean":
        mean = vectors.mean(axis=0, keepdims=True)
        vectors = mean / np.clip(np.linalg.norm(mean, axis=1, keepdims=True), 1e-12, None)
    return vectors


def score_recording(candidates: pd.DataFrame, features: np.ndarray, variant: str) -> dict:
    gallery = gallery_vectors(candidates, features, variant)
    sims = (features @ gallery.T).max(axis=1)
    evals = candidates[candidates.role == "eval"].copy()
    evals["sim"] = sims[evals.index.to_numpy()]

    gt = evals[evals.source == "gt"]
    proposals = evals[evals.source == "proposal"]
    pos = gt[gt.label == common.OPPONENT]
    gt_neg = gt[gt.label != common.OPPONENT]
    hard_neg = proposals[proposals.best_iou < common.HARD_NEGATIVE_IOU]

    pos_sims = pos.sim.to_numpy()
    neg_sims = np.concatenate([gt_neg.sim.to_numpy(), hard_neg.sim.to_numpy()])

    # Top-1: per eval frame with an opponent GT box, is the most similar Eval-A
    # candidate an opponent GT box?
    eval_a = pd.concat([gt, hard_neg])
    top1_hits, top1_total = 0, 0
    for _stamp, group in eval_a.groupby("stamp_ns"):
        frame_gt = group[group.source == "gt"]
        if common.OPPONENT not in frame_gt.label.values:
            continue
        top1_total += 1
        best = group.loc[group.sim.idxmax()]
        if best.source == "gt" and best.label == common.OPPONENT:
            top1_hits += 1

    # Similarity vs match time for GT opponents (seconds from first eval stamp).
    t0 = evals.stamp_ns.min()
    times = (pos.stamp_ns.to_numpy() - t0) / 1e9
    slope = float(np.polyfit(times, pos_sims, 1)[0]) if len(pos) >= 2 else float("nan")

    # Eval B bookkeeping: per GT opponent box, deployed hit / rescuable / best rescue sim.
    misses = []
    for stamp, group in evals.groupby("stamp_ns"):
        frame_gt = group[(group.source == "gt") & (group.label == common.OPPONENT)]
        frame_props = group[group.source == "proposal"]
        for gt_index, gt_row in frame_gt.iterrows():
            matched = frame_props[
                (frame_props.best_gt_index == gt_row.best_gt_index)
                & (frame_props.best_iou >= common.PROPOSAL_MATCH_IOU)
            ]
            deployed_hit = bool(
                ((matched.conf >= common.DEPLOYED_CONF) & (matched.label == common.OPPONENT)).any()
            )
            if deployed_hit:
                continue
            best_sim = float(matched.sim.max()) if len(matched) else float("nan")
            misses.append({"stamp_ns": stamp, "rescuable": len(matched) > 0, "best_sim": best_sim})
    miss_frame = pd.DataFrame(misses)

    return {
        "auc": roc_auc(pos_sims, neg_sims),
        "auc_vs_hard_neg": roc_auc(pos_sims, hard_neg.sim.to_numpy()),
        "top1": top1_hits / top1_total if top1_total else float("nan"),
        "top1_frames": top1_total,
        "sim_slope_per_s": slope,
        "n_pos": len(pos),
        "n_neg": len(neg_sims),
        "n_hard_neg": len(hard_neg),
        "pos_sims": pos_sims,
        "neg_sims": neg_sims,
        "hard_neg_sims": hard_neg.sim.to_numpy(),
        "pos_times": times,
        "misses": miss_frame,
        "n_gt_opponents": len(pos),
    }


def compute_all(recordings: list[str]) -> tuple[pd.DataFrame, dict[tuple, dict]]:
    """Score every (embedder, pad, variant, recording) combination."""
    rows = []
    detail: dict[tuple, dict] = {}
    for embedder in EMBEDDER_NAMES:
        for pad_key in PAD_KEYS:
            variants = ("gallery", "mean", "seed1") if pad_key == DEFAULT_PAD_KEY else ("gallery",)
            for variant in variants:
                per_recording = {}
                for recording in recordings:
                    candidates = common.load_candidates(recording)
                    features = aligned_features(recording, embedder, pad_key, len(candidates))
                    result = score_recording(candidates, features, variant)
                    per_recording[recording] = result
                    detail[(embedder, pad_key, variant, common.short_name(recording))] = result
                # Operating point: pooled negatives across recordings at the budget.
                pooled_neg = np.concatenate(
                    [result["neg_sims"] for result in per_recording.values()]
                )
                threshold = float(np.quantile(pooled_neg, 1 - FALSE_ACCEPT_BUDGET))
                for recording, result in per_recording.items():
                    rows.append(
                        metric_row(embedder, pad_key, variant, recording, result, threshold)
                    )
    return pd.DataFrame(rows), detail


def metric_row(
    embedder: str, pad_key: str, variant: str, recording: str, result: dict, threshold: float
) -> dict:
    misses = result["misses"]
    rescuable = misses[misses.rescuable] if len(misses) else misses
    rescued = int((rescuable.best_sim >= threshold).sum()) if len(rescuable) else 0
    hard_sims = result["hard_neg_sims"]
    neg_sims = result["neg_sims"]
    return {
        "embedder": embedder,
        "pad": pad_key,
        "variant": variant,
        "recording": common.short_name(recording),
        "auc": result["auc"],
        "auc_vs_hard_neg": result["auc_vs_hard_neg"],
        "top1": result["top1"],
        "top1_frames": result["top1_frames"],
        "sim_slope_per_s": result["sim_slope_per_s"],
        "n_gt_opponents": result["n_gt_opponents"],
        "n_neg": result["n_neg"],
        "threshold": threshold,
        "deployed_misses": len(misses),
        "rescuable_misses": int(misses.rescuable.sum()) if len(misses) else 0,
        "blind_misses": int((~misses.rescuable).sum()) if len(misses) else 0,
        "rescued": rescued,
        "false_accept_rate_neg": float((neg_sims >= threshold).mean())
        if len(neg_sims)
        else float("nan"),
        "false_accept_rate_hard_neg": float((hard_sims >= threshold).mean())
        if len(hard_sims)
        else float("nan"),
    }


def roc_figures(detail: dict[tuple, dict], shorts: list[str], figures_dir) -> None:
    for embedder in EMBEDDER_NAMES:
        fig, axes = plt.subplots(2, 4, figsize=(16, 8))
        for axis, short in zip(axes.flat, shorts):
            result = detail[(embedder, DEFAULT_PAD_KEY, "gallery", short)]
            pos, neg = result["pos_sims"], result["neg_sims"]
            thresholds = np.unique(np.concatenate([pos, neg]))[::-1]
            axis.plot(
                [(neg >= t).mean() for t in thresholds], [(pos >= t).mean() for t in thresholds]
            )
            axis.plot([0, 1], [0, 1], "k:", linewidth=0.5)
            axis.set_title(f"{short} AUC={result['auc']:.3f}")
            axis.set_xlabel("FPR")
            axis.set_ylabel("TPR")
        fig.suptitle(f"Eval A ROC, {embedder}, pad010, gallery seeds")
        fig.tight_layout()
        fig.savefig(figures_dir / f"roc_{embedder}.png", dpi=110)
        plt.close(fig)


def time_figure(
    detail: dict[tuple, dict], table: pd.DataFrame, best: str, shorts: list[str], figures_dir
) -> None:
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    for axis, short in zip(axes.flat, shorts):
        result = detail[(best, DEFAULT_PAD_KEY, "gallery", short)]
        axis.scatter(result["pos_times"], result["pos_sims"], s=8, label="opponent GT")
        axis.axhline(table[table.recording == short].threshold.iloc[0], color="r", linewidth=0.8)
        axis.set_title(f"{short} slope={result['sim_slope_per_s']:+.4f}/s")
        axis.set_xlabel("match time (s)")
        axis.set_ylabel("similarity")
        axis.set_ylim(0, 1)
    fig.suptitle(f"Opponent similarity vs match time, {best} (red: operating point)")
    fig.tight_layout()
    fig.savefig(figures_dir / "sim_vs_time.png", dpi=110)
    plt.close(fig)


def print_table(table: pd.DataFrame, best: str, shorts: list[str]) -> None:
    print(f"\nEval A/B/D summary (pad010, gallery seeds); best core embedder: {best}")
    print(
        f"{'embedder':9s} {'rec':6s} {'AUC':>6s} {'AUChn':>6s} {'top1':>6s} "
        f"{'slope/s':>8s} {'miss':>4s} {'resc':>4s} {'blind':>5s} {'rescued':>7s} {'FA%':>5s}"
    )
    for embedder in EMBEDDER_NAMES:
        for short in shorts:
            row = table[(table.embedder == embedder) & (table.recording == short)].iloc[0]
            marker = "*" if short in common.CORE_RECORDINGS else " "
            print(
                f"{embedder:9s} {short:5s}{marker} {row.auc:6.3f} {row.auc_vs_hard_neg:6.3f} "
                f"{row.top1:6.1%} {row.sim_slope_per_s:+8.4f} {row.deployed_misses:4d} "
                f"{row.rescuable_misses:4d} {row.blind_misses:5d} {row.rescued:7d} "
                f"{row.false_accept_rate_neg:5.1%}"
            )
    print("\n* core recording. metrics.csv has all pad/variant rows.")


def main() -> None:
    recordings = sorted(common.SHORT_NAMES)
    shorts = [common.short_name(recording) for recording in recordings]
    metrics, detail = compute_all(recordings)
    common.OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    metrics.to_csv(common.OUTPUT_ROOT / "metrics.csv", index=False)

    figures_dir = common.OUTPUT_ROOT / "figures"
    figures_dir.mkdir(exist_ok=True)

    table = metrics[(metrics.pad == DEFAULT_PAD_KEY) & (metrics.variant == "gallery")]
    core = table[table.recording.isin(common.CORE_RECORDINGS)]
    best = core.groupby("embedder").auc.mean().idxmax()

    roc_figures(detail, shorts, figures_dir)
    time_figure(detail, table, best, shorts, figures_dir)
    print_table(table, best, shorts)


if __name__ == "__main__":
    main()
