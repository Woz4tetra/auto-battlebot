#!/usr/bin/env python3
"""Score DeepLab field-mask checkpoints against held-out ground truth, per field type.

The global mean IoU this project has been reading is roughly a Cage-6 score -- one field
is a third of the corpus -- so a checkpoint that maximises it can be regressing the small
cages the whole way. Everything here is therefore reported **macro-averaged across field
types** and as **worst field**, with the global mean kept only for continuity.

Metrics, per field and macro-averaged:

    field IoU       the primary number.
    boundary F1     at each --boundary-tol, in pixels at the model's native resolution.
                    The mask feeds PointCloudFieldFilter, which fits the field plane and
                    hands wall bounds to navigation: 2% wrong in the interior costs
                    nothing, 2% wrong at the wall line moves the boundary. IoU cannot
                    tell those apart.
    worst-field IoU the metric that actually gates deployment.

With two or more candidates a paired bootstrap reports whether each differs from the
baseline. Resampling is stratified by field and shared across candidates, so every field
stays represented in every resample (a macro-average over a resample that dropped a small
field is not the same estimator) and per-frame difficulty cancels.

Checkpoints are graded as `.pth` directly -- engine conversion is only needed for the
deployment candidate, and a per-arm conversion would add ~20 runs of overhead to measure
a quantisation effect this experiment is not asking about.

Usage:
    python3 training/deeplab/score_masks.py \
        training/data/deeplab_field_2026-07-28/eval \
        --candidate deployed=data/models/field_deeplabv3p_r50_2026-04-29.pth \
        --by-field --boundary-tol 2,5,10 --bootstrap 1000 \
        --output training/data/deeplab_field_2026-07-28/scores/anchor
"""

from __future__ import annotations

import argparse
import json
import re
from glob import glob
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import cv2
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
from field_labels import UNCLASSIFIED
from load_deeplabv3 import common_transforms, load_model
from torch.utils.data import DataLoader, Dataset
from tqdm import tqdm

FIELD_CLASS = 1  # channel 1 of the model head is the field; channel 0 is background
UNKNOWN_FIELD = "unknown"
EPOCH_RE = re.compile(r"_ep(\d+)$")


class EvalFrames(Dataset):
    """Eval images and their ground-truth masks, preprocessed exactly as in training."""

    def __init__(self, images: list[Path], masks: list[Path], image_size: int, pad_size: int):
        self.images = images
        self.masks = masks
        self.image_size = image_size
        self.transforms = common_transforms(pad_size=pad_size)

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        size = (self.image_size, self.image_size)
        image = cv2.imread(str(self.images[index]))
        if image is None:
            raise SystemExit(f"Failed to read image {self.images[index]}")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, size, interpolation=cv2.INTER_NEAREST)

        mask = cv2.imread(str(self.masks[index]), cv2.IMREAD_UNCHANGED)
        if mask is None:
            raise SystemExit(f"Failed to read mask {self.masks[index]}")
        mask = mask[:, :, 0] if mask.ndim == 3 else mask
        mask = cv2.resize(mask, size, interpolation=cv2.INTER_NEAREST)

        return self.transforms(image), torch.from_numpy((mask > 0).astype(np.uint8))


def resolve_eval_set(target: Path) -> tuple[list[Path], list[Path]]:
    """Image/mask pairs from an eval directory or a newline-delimited image list."""
    if target.is_dir():
        images = sorted(target.glob("*.jpg"))
    elif target.suffix == ".txt":
        root = target.parent
        images = [(root / line).resolve() for line in target.read_text().split()]
    else:
        raise SystemExit(f"Eval target must be a directory or a .txt image list: {target}")

    pairs = [(img, img.with_name(f"{img.stem}_mask.png")) for img in images]
    missing = [img for img, mask in pairs if not mask.exists()]
    if missing:
        raise SystemExit(f"{len(missing)} image(s) have no mask, e.g. {missing[0]}")
    if not pairs:
        raise SystemExit(f"No image/mask pairs found at {target}")
    return [p[0] for p in pairs], [p[1] for p in pairs]


def load_field_index(eval_target: Path, override: Path | None) -> dict[str, str]:
    """Map image name -> field. Written by make_field_splits.py at the corpus root."""
    if override is not None:
        candidates = [override]
    else:
        base = eval_target if eval_target.is_dir() else eval_target.parent
        candidates = [base / "field_index.json", base.parent / "field_index.json"]
    for path in candidates:
        if path.exists():
            index = json.loads(path.read_text())
            print(f"Field index: {path} ({len(index):,} frames)")
            return {name: entry["field"] for name, entry in index.items()}
    print(
        "Warning: no field_index.json found; every frame scores as one field, so the "
        "macro-average and worst-field numbers collapse to the global mean."
    )
    return {}


def boundary(mask: np.ndarray) -> np.ndarray:
    """Inner 1px contour of a binary mask.

    BORDER_REPLICATE keeps the image frame from counting as a boundary: the floor runs
    off the edge of nearly every frame, and a spurious contour there would swamp the
    wall line, which is the edge that actually matters downstream.
    """
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(mask, kernel, borderType=cv2.BORDER_REPLICATE)
    return mask ^ eroded


def frame_stats(
    pred: np.ndarray, truth: np.ndarray, tolerances: list[int]
) -> tuple[float, float, dict[int, tuple[float, float, float, float]]]:
    """Per-frame sufficient statistics: (intersection, union, {tol: boundary counts}).

    Kept as counts rather than ratios so the bootstrap can resample frames and re-sum
    instead of re-running inference on every resample.
    """
    intersection = float(np.count_nonzero(pred & truth))
    union = float(np.count_nonzero(pred | truth))

    pred_edge = boundary(pred)
    truth_edge = boundary(truth)
    pred_total = float(np.count_nonzero(pred_edge))
    truth_total = float(np.count_nonzero(truth_edge))

    counts = {}
    for tol in tolerances:
        size = 2 * tol + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        near_truth = cv2.dilate(truth_edge, kernel, borderType=cv2.BORDER_CONSTANT, borderValue=0)
        near_pred = cv2.dilate(pred_edge, kernel, borderType=cv2.BORDER_CONSTANT, borderValue=0)
        counts[tol] = (
            float(np.count_nonzero(pred_edge & near_truth)),  # precision numerator
            pred_total,
            float(np.count_nonzero(truth_edge & near_pred)),  # recall numerator
            truth_total,
        )
    return intersection, union, counts


def score_candidate(
    name: str,
    checkpoint: Path,
    images: list[Path],
    masks: list[Path],
    args: argparse.Namespace,
    device: torch.device,
) -> dict[str, np.ndarray]:
    """Run one checkpoint over the eval set, returning per-frame stat arrays."""
    if not checkpoint.exists():
        raise SystemExit(f"Checkpoint not found: {checkpoint}")
    model, config = load_model(checkpoint, device)
    print(
        f"  {config.backbone}/{config.decoder} {config.image_size}+2x{config.pad_size} "
        f"-> {config.input_size}px"
    )

    dataset = EvalFrames(images, masks, config.image_size, config.pad_size)
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
        shuffle=False,
        pin_memory=True,
    )

    n = len(dataset)
    stats: dict[str, np.ndarray] = {
        "inter": np.zeros(n),
        "union": np.zeros(n),
    }
    for tol in args.boundary_tol:
        for key in ("p_hit", "p_tot", "r_hit", "r_tot"):
            stats[f"bf{tol}_{key}"] = np.zeros(n)

    pad = config.pad_size
    position = 0
    with torch.no_grad():
        for batch_images, batch_masks in tqdm(
            loader, desc=f"  {name}", dynamic_ncols=True, total=len(loader)
        ):
            logits = model(batch_images.to(device, non_blocking=True))
            predicted = torch.argmax(logits, dim=1)
            if pad:
                predicted = predicted[:, pad:-pad, pad:-pad]
            predicted = (predicted == FIELD_CLASS).to(torch.uint8).cpu().numpy()
            truth = batch_masks.numpy().astype(np.uint8)

            for i in range(len(predicted)):
                intersection, union, counts = frame_stats(predicted[i], truth[i], args.boundary_tol)
                stats["inter"][position] = intersection
                stats["union"][position] = union
                for tol, (p_hit, p_tot, r_hit, r_tot) in counts.items():
                    stats[f"bf{tol}_p_hit"][position] = p_hit
                    stats[f"bf{tol}_p_tot"][position] = p_tot
                    stats[f"bf{tol}_r_hit"][position] = r_hit
                    stats[f"bf{tol}_r_tot"][position] = r_tot
                position += 1

    del model
    torch.cuda.empty_cache()
    return stats


def _ratio(num: np.ndarray, den: np.ndarray, idx: np.ndarray) -> np.ndarray:
    """sum(num)/sum(den) over each resampled frame set; nan where the denominator is 0."""
    numerator = num[idx].sum(axis=1)
    denominator = den[idx].sum(axis=1)
    return np.where(
        denominator > 0, numerator / np.where(denominator > 0, denominator, 1.0), np.nan
    )


def _f1(precision: np.ndarray, recall: np.ndarray) -> np.ndarray:
    total = precision + recall
    return np.where(total > 0, 2 * precision * recall / np.where(total > 0, total, 1.0), 0.0)


def field_metric(stats: dict[str, np.ndarray], metric: str, idx: np.ndarray) -> np.ndarray:
    """One metric over a resample index set. `idx` is (B, n) of frame positions."""
    if metric == "iou":
        return _ratio(stats["inter"], stats["union"], idx)
    tol = int(metric.split("@")[1])
    precision = _ratio(stats[f"bf{tol}_p_hit"], stats[f"bf{tol}_p_tot"], idx)
    recall = _ratio(stats[f"bf{tol}_r_hit"], stats[f"bf{tol}_r_tot"], idx)
    return _f1(precision, recall)


def aggregate(
    stats: dict[str, np.ndarray],
    metric: str,
    field_positions: dict[str, np.ndarray],
    resamples: dict[str, np.ndarray],
) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray, np.ndarray]:
    """Per-field values plus macro / worst / global, each of length B."""
    per_field = {
        field: field_metric(stats, metric, resamples[field]) for field in sorted(field_positions)
    }
    stacked = np.vstack([per_field[f] for f in sorted(per_field)])
    macro = np.nanmean(stacked, axis=0)
    worst = np.nanmin(stacked, axis=0)
    combined = np.concatenate([resamples[f] for f in sorted(field_positions)], axis=1)
    return per_field, macro, worst, field_metric(stats, metric, combined)


def build_resamples(
    field_positions: dict[str, np.ndarray], count: int, rng: np.random.Generator
) -> dict[str, np.ndarray]:
    """Stratified frame resamples, one index block per field, shared across candidates."""
    resamples = {}
    for field, positions in field_positions.items():
        draws = rng.integers(0, len(positions), size=(count, len(positions)))
        resamples[field] = positions[draws]
    return resamples


def plot_by_field(summary: pd.DataFrame, output: Path) -> None:
    """Per-field IoU per candidate, with the macro-average drawn as a line."""
    fields = sorted(summary["field"].unique())
    candidates = list(dict.fromkeys(summary["candidate"]))
    width = 0.8 / max(len(candidates), 1)
    fig, ax = plt.subplots(figsize=(2 + 1.1 * len(fields), 5))
    x = np.arange(len(fields))
    for i, candidate in enumerate(candidates):
        rows = summary[summary["candidate"] == candidate].set_index("field")
        values = [rows["iou"].get(f, np.nan) for f in fields]
        ax.bar(x + (i - (len(candidates) - 1) / 2) * width, values, width=width, label=candidate)
    ax.set_xticks(x)
    ax.set_xticklabels(fields, rotation=30, ha="right")
    ax.set_ylabel("field IoU")
    ax.set_ylim(0, 1.02)
    ax.set_title("Field IoU by field type (the global mean is ~a Cage-6 score)")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output / "by_field_iou.png", dpi=120)
    plt.close(fig)


def plot_epoch_ladder(aggregates: pd.DataFrame, output: Path) -> bool:
    """Macro IoU vs epoch, one line per run, when the candidates are a checkpoint ladder.

    With two seed arms the spread between the lines is the noise floor delta that every
    Phase-B gate is read against, so it is shaded rather than left to the eye.
    """
    parsed = []
    for _, row in aggregates.iterrows():
        match = EPOCH_RE.search(str(row["candidate"]))
        if match:
            parsed.append(
                {
                    "run": EPOCH_RE.sub("", str(row["candidate"])),
                    "epoch": int(match.group(1)),
                    "macro_iou": row["macro_iou"],
                    "worst_field_iou": row["worst_field_iou"],
                }
            )
    if len({p["epoch"] for p in parsed}) < 2:
        return False

    ladder = pd.DataFrame(parsed).sort_values("epoch")
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharex=True)
    for metric, ax in zip(("macro_iou", "worst_field_iou"), axes):
        pivot = ladder.pivot_table(index="epoch", columns="run", values=metric)
        for run in pivot.columns:
            ax.plot(pivot.index, pivot[run], marker="o", markersize=3, label=run)
        if len(pivot.columns) >= 2:
            ax.fill_between(
                pivot.index,
                pivot.min(axis=1),
                pivot.max(axis=1),
                alpha=0.15,
                color="grey",
                label="seed spread (noise floor)",
            )
        ax.set_xlabel("epoch")
        ax.set_ylabel(metric)
        ax.set_title(metric)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("Where does training flatten? (noise band = spread between seeds)")
    fig.tight_layout()
    fig.savefig(output / "epoch_ladder.png", dpi=120)
    plt.close(fig)
    return True


def select_frames(
    fields: list[str], field_filter: str | None, exclude_field: str
) -> tuple[list[int], str]:
    """Frame indices to score, and a one-line reason for the console.

    --field-filter wins over --exclude-field: naming a field explicitly overrides the
    default exclusion, so `--field-filter unclassified` still works.
    """
    if field_filter:
        wanted = set(field_filter.split(","))
        return (
            [i for i, field in enumerate(fields) if field in wanted],
            f"--field-filter {field_filter}",
        )
    excluded = {f for f in exclude_field.split(",") if f}
    dropped = sorted(excluded & set(fields))
    return (
        [i for i, field in enumerate(fields) if field not in excluded],
        f"--exclude-field {','.join(dropped)}" if dropped else "",
    )


def summarize(
    stats_by_candidate: dict[str, dict[str, np.ndarray]],
    field_positions: dict[str, np.ndarray],
    metrics: list[str],
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Point estimates: one row per candidate/field, one row per candidate."""
    point = {field: positions[None, :] for field, positions in field_positions.items()}
    field_rows = []
    aggregate_rows = []
    for name, stats in stats_by_candidate.items():
        aggregate_row: dict[str, float | str] = {"candidate": name}
        per_field_values: dict[str, dict[str, float]] = {}
        for metric in metrics:
            per_field, macro, worst, overall = aggregate(stats, metric, field_positions, point)
            suffix = "iou" if metric == "iou" else metric
            for field, value in per_field.items():
                per_field_values.setdefault(field, {})[suffix] = float(value[0])
            aggregate_row[f"macro_{suffix}"] = float(macro[0])
            aggregate_row[f"worst_field_{suffix}"] = float(worst[0])
            aggregate_row[f"global_{suffix}"] = float(overall[0])
        aggregate_rows.append(aggregate_row)
        for field, values in per_field_values.items():
            field_rows.append(
                {"candidate": name, "field": field, "frames": len(field_positions[field]), **values}
            )
    return pd.DataFrame(field_rows), pd.DataFrame(aggregate_rows)


def parse_candidates(entries: list[str] | None, patterns: list[str] | None) -> dict[str, Path]:
    """`NAME=path` entries first, then globbed checkpoints named `<run>_<stem>`."""
    candidates: dict[str, Path] = {}
    for entry in entries or []:
        name, _, path = entry.partition("=") if "=" in entry else (Path(entry).stem, "", entry)
        candidates[name] = Path(path)
    for pattern in patterns or []:
        for match in sorted(glob(pattern)):
            path = Path(match)
            name = f"{path.parent.name}_{path.stem}"
            candidates.setdefault(name, path)
    if not candidates:
        raise SystemExit("No candidates. Pass --candidate and/or --candidate-glob.")
    return candidates


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("eval", type=Path, help="eval directory, or a .txt image list")
    parser.add_argument(
        "--candidate", action="append", metavar="NAME=CHECKPOINT", help="repeatable .pth candidate"
    )
    parser.add_argument(
        "--candidate-glob", action="append", help="glob of .pth checkpoints, repeatable"
    )
    parser.add_argument("--baseline", help="candidate to compare the others against")
    parser.add_argument("--by-field", action="store_true", help="report and test each field")
    parser.add_argument(
        "--field-filter", help="score only this field (comma-separated for several)"
    )
    parser.add_argument("--field-index", type=Path, help="override field_index.json location")
    parser.add_argument(
        "--exclude-field",
        default=UNCLASSIFIED,
        help="fields to drop entirely, comma-separated. 'unclassified' is a grab-bag of "
        "scenes whose arena could not be read off the filename, so by default it does not "
        "get a vote in a macro-average over field types (pass '' to keep it)",
    )
    parser.add_argument("--boundary-tol", default="2,5,10", help="boundary F1 tolerances in pixels")
    parser.add_argument("--bootstrap", type=int, default=1000, help="resamples (0 disables)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--alpha", type=float, default=0.05, help="0.05 = 95%% CI")
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--num-workers", type=int, default=8)
    parser.add_argument("--output", type=Path, help="csv/plot output dir")
    args = parser.parse_args()

    args.boundary_tol = [int(t) for t in args.boundary_tol.split(",") if t]
    args.output = args.output or (args.eval.parent / "scores")
    args.output.mkdir(parents=True, exist_ok=True)

    images, masks = resolve_eval_set(args.eval)
    field_of = load_field_index(args.eval, args.field_index)
    fields = [field_of.get(img.name, UNKNOWN_FIELD) for img in images]

    keep, reason = select_frames(fields, args.field_filter, args.exclude_field)
    if not keep:
        raise SystemExit(f"{reason} left no eval frames")
    if len(keep) != len(images):
        images = [images[i] for i in keep]
        masks = [masks[i] for i in keep]
        fields = [fields[i] for i in keep]
        print(f"{reason}: {len(images):,} frames")

    field_positions = {
        field: np.flatnonzero(np.asarray(fields) == field) for field in sorted(set(fields))
    }
    print(f"Eval: {len(images):,} frames over {len(field_positions)} field(s)")
    for field, positions in field_positions.items():
        print(f"  {field:<16}{len(positions):>7,}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    candidates = parse_candidates(args.candidate, args.candidate_glob)

    stats_by_candidate: dict[str, dict[str, np.ndarray]] = {}
    for name, checkpoint in candidates.items():
        print(f"Scoring {name}: {checkpoint}")
        stats_by_candidate[name] = score_candidate(name, checkpoint, images, masks, args, device)

    metrics = ["iou"] + [f"bf1@{tol}" for tol in args.boundary_tol]
    summary, aggregates = summarize(stats_by_candidate, field_positions, metrics)

    headline = ["candidate", "macro_iou", "worst_field_iou", "global_iou"]
    headline += [f"macro_bf1@{tol}" for tol in args.boundary_tol]
    print("\n" + aggregates[headline].to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    if args.by_field:
        print("\nPer field:")
        print(summary.to_string(index=False, float_format=lambda v: f"{v:.4f}"))

    summary.to_csv(args.output / "by_field.csv", index=False)
    aggregates.to_csv(args.output / "aggregate.csv", index=False)
    plot_by_field(summary, args.output)
    outputs = "by_field.csv, aggregate.csv, by_field_iou.png"
    if plot_epoch_ladder(aggregates, args.output):
        outputs += ", epoch_ladder.png"
    if len(stats_by_candidate) >= 2 and args.bootstrap > 0:
        significance = run_significance(stats_by_candidate, field_positions, metrics, args)
        conf = round(100 * (1 - args.alpha))
        print(
            f"\nPaired bootstrap vs '{significance.attrs['baseline']}' "
            f"({args.bootstrap} stratified resamples, {conf}% CI on candidate - baseline):"
        )
        print(significance.to_string(index=False, float_format=lambda v: f"{v:.4f}"))
        significance.to_csv(args.output / "significance.csv", index=False)
        outputs += ", significance.csv"
    elif len(stats_by_candidate) < 2:
        print("\n(Only one candidate; add another for a significance comparison.)")

    print(f"\nWrote {args.output}/{{{outputs}}}")


def run_significance(
    stats_by_candidate: dict[str, dict[str, np.ndarray]],
    field_positions: dict[str, np.ndarray],
    metrics: list[str],
    args: argparse.Namespace,
) -> pd.DataFrame:
    """Paired stratified bootstrap of every candidate against the baseline."""
    baseline = args.baseline or next(iter(stats_by_candidate))
    if baseline not in stats_by_candidate:
        raise SystemExit(f"--baseline {baseline!r} is not one of {list(stats_by_candidate)}")

    rng = np.random.default_rng(args.seed)
    resamples = build_resamples(field_positions, args.bootstrap, rng)
    point = {field: positions[None, :] for field, positions in field_positions.items()}
    lo_pct, hi_pct = 100 * args.alpha / 2, 100 * (1 - args.alpha / 2)

    def series(stats: dict, metric: str, idx: dict) -> dict[str, np.ndarray]:
        per_field, macro, worst, overall = aggregate(stats, metric, field_positions, idx)
        out = {"macro": macro, "worst_field": worst, "global": overall}
        if args.by_field:
            out.update({f"field:{name}": value for name, value in per_field.items()})
        return out

    rows = []
    base_stats = stats_by_candidate[baseline]
    for name, stats in stats_by_candidate.items():
        if name == baseline:
            continue
        for metric in metrics:
            base_point = series(base_stats, metric, point)
            cand_point = series(stats, metric, point)
            base_boot = series(base_stats, metric, resamples)
            cand_boot = series(stats, metric, resamples)
            for scope in base_point:
                delta = cand_boot[scope] - base_boot[scope]
                base_value = float(base_point[scope][0])
                cand_value = float(cand_point[scope][0])
                if np.all(np.isnan(delta)):
                    low = high = float("nan")
                    verdict = "n/a"
                else:
                    low, high = (float(v) for v in np.nanpercentile(delta, [lo_pct, hi_pct]))
                    # Every metric here is higher-is-better.
                    verdict = "better" if low > 0 else "worse" if high < 0 else "ns"
                rows.append(
                    {
                        "candidate": name,
                        "baseline": baseline,
                        "metric": metric,
                        "scope": scope,
                        "baseline_val": base_value,
                        "candidate_val": cand_value,
                        "delta": cand_value - base_value,
                        "ci_low": low,
                        "ci_high": high,
                        "verdict": verdict,
                    }
                )
    frame = pd.DataFrame(rows)
    frame.attrs["baseline"] = baseline
    return frame


if __name__ == "__main__":
    main()
