#!/usr/bin/env python3
"""Render the Meshy fidelity figure: NHRL thumbnail vs Meshy render vs real fight robot.

One row per opponent, three columns. The point of the figure is that the Meshy model
faithfully reproduces the thumbnail it was built from, while the real robot in the fight
has been rebuilt since that thumbnail was taken. That only reads if all three tiles show
the robot large and from a comparable angle, so both image columns are picked, not
hand-chosen:

- Meshy render: from the training set, ranked by box area, sharpness, and HSV-histogram
  similarity to the masked thumbnail, penalized for open-sky background. The histogram
  includes value bins so a belly-up render does not win over a top-side one.
- Real fight: from that opponent's eval recording, ranked by GT box area and sharpness,
  so the robot fills the tile instead of sitting in it as a blob. Picks are kept
  MIN_SEPARATION_NS apart so a variant is not five frames of one moment.

Every cell of the published figure is pinned in LOCKED, so a plain run reproduces it. To
reopen a cell, drop its LOCKED entry and pass `--variants N`: that writes N mosaics with
the k-th ranked pick in the reopened cells, to compare side by side and choose from.

Usage:
    python training/model_eval/make_meshy_fidelity_mosaic.py
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
SYNTH_DIR = REPO / "training/data/meshy_grade"
EVAL_DIR = REPO / "training/data/nhrl_keypoints_eval_test"
THUMB_DIR = REPO / "training/data/distractor_models/robots/thumbnails"

# Row order and the caption under each opponent name, from meshy_grade_2026-07-16.md.
ROWS = [
    ("sphinx", 0.209, "ceiling (unchanged)"),
    ("ironwarrior", 0.113, "above floor"),
    ("wreckcreation", 0.083, "at floor"),
    ("clyde", 0.030, "failed"),
]

# Class ids in training/data/meshy_grade/data.yml.
SYNTH_CLASS = {"clyde": 2, "sphinx": 3, "wreckcreation": 4, "ironwarrior": 5}

# Recording -> opponent (operator-provided; the eval GT only labels a generic opponent).
RECORDING = {
    "clyde": "main_2026-05-02_10-06-02_repaired__2026-05-02T10-06-06",
    "sphinx": "main_2026-05-02_11-45-05_repaired__2026-05-02T11-45-08",
    "wreckcreation": "main_2026-05-02_14-12-25_repaired__2026-05-02T14-12-27",
    "ironwarrior": "main_2026-05-02_15-35-00_repaired__2026-05-02T15-35-04",
}
REAL_OPPONENT_CLASS = 2  # class id of `opponent` in the eval data.yaml

SYNTH_MARGIN = 1.55  # crop side as a multiple of the box's long edge
REAL_MARGIN = 1.85
MIN_SEPARATION_NS = 3_000_000_000  # keep real picks >=3 s apart
SYNTH_POOL = 400  # largest-box candidates scored per opponent
REAL_POOL = 60

# The frames the operator chose, reviewed over three rounds of variants. Every cell is
# settled, so the figure is now fixed; the ranking below only decides what a variant run
# offers if a cell is taken out of this table.
#
# ironwarrior needed hand-picking in both columns. The scoring rewards a large sharp box,
# which kept surfacing the render mid-tumble rather than grounded, and on the real side
# the top of the ranking included between-round frames with a hand in the arena.
LOCKED = {
    ("sphinx", "synth"): "010114.jpg",
    ("sphinx", "real"): "1777737183231579000.png",
    ("ironwarrior", "synth"): "005870.jpg",
    ("ironwarrior", "real"): "1777750593937328000.png",
    ("wreckcreation", "synth"): "005811.jpg",
    ("wreckcreation", "real"): "1777745762217608000.png",
    ("clyde", "synth"): "002724.jpg",
    ("clyde", "real"): "1777731077306718000.png",
}


@dataclass
class Candidate:
    image: Path
    box: tuple[float, float, float, float]  # cx, cy, w, h, normalized
    margin: float
    score: float


def read_boxes(path: Path) -> list[tuple[int, float, float, float, float]]:
    rows = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        rows.append(
            (int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))
        )
    return rows


def imread(path: Path) -> np.ndarray:
    image = cv2.imread(str(path))
    if image is None:
        raise FileNotFoundError(path)
    return image


def crop_box(
    image: np.ndarray, box: tuple[float, float, float, float], margin: float
) -> np.ndarray:
    """Square crop centered on the box, shifted to stay inside the frame."""
    height, width = image.shape[:2]
    cx, cy = box[0] * width, box[1] * height
    side = max(box[2] * width, box[3] * height) * margin
    side = min(side, float(min(width, height)))
    half = side / 2.0
    cx = float(np.clip(cx, half, width - half))
    cy = float(np.clip(cy, half, height - half))
    x0, y0 = int(round(cx - half)), int(round(cy - half))
    x1, y1 = int(round(cx + half)), int(round(cy + half))
    return image[y0:y1, x0:x1]


def sharpness(crop: np.ndarray) -> float:
    gray = cv2.cvtColor(cv2.resize(crop, (128, 128)), cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def sky_fraction(image: np.ndarray, box: tuple[float, float, float, float]) -> float:
    """Fraction of the frame outside the robot box that reads as open sky.

    Airborne renders sit against a bright or blue HDRI horizon. Grounded ones show
    ground texture, which is closer to how the robot is seen in the arena.
    """
    small = cv2.cvtColor(cv2.resize(image, (160, 90)), cv2.COLOR_BGR2HSV)
    hue, sat, val = (small[..., i].astype(int) for i in range(3))
    sky = ((hue > 92) & (hue < 135) & (sat > 50) & (val > 110)) | ((sat < 40) & (val > 205))
    outside = np.ones(sky.shape, dtype=bool)
    cx, cy, bw, bh = box
    x0, x1 = int(max(0, (cx - bw / 2) * 160)), int(min(160, (cx + bw / 2) * 160))
    y0, y1 = int(max(0, (cy - bh / 2) * 90)), int(min(90, (cy + bh / 2) * 90))
    outside[y0:y1, x0:x1] = False
    return float(sky[outside].mean()) if outside.any() else 0.0


def hsv_histogram(image: np.ndarray, mask: np.ndarray | None) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1, 2], mask, [12, 8, 8], [0, 180, 0, 256, 0, 256])
    normalized: np.ndarray = cv2.normalize(hist, hist).flatten()
    return normalized


def thumbnail_histogram(name: str) -> np.ndarray:
    image = imread(THUMB_DIR / f"{name}.png")
    value = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[..., 2]
    foreground = (value > 40).astype(np.uint8)  # drop the black cutout background
    return hsv_histogram(image, foreground)


def crop_histogram(crop: np.ndarray, inner: float = 0.62) -> np.ndarray:
    height, width = crop.shape[:2]
    y0, y1 = int(height * (1 - inner) / 2), int(height * (1 + inner) / 2)
    x0, x1 = int(width * (1 - inner) / 2), int(width * (1 + inner) / 2)
    return hsv_histogram(crop[y0:y1, x0:x1], None)


def rescale(values: list[float]) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        return array
    lo, hi = np.percentile(array, 5), np.percentile(array, 95)
    if hi - lo < 1e-9:
        return np.zeros_like(array)
    scaled: np.ndarray = np.clip((array - lo) / (hi - lo), 0.0, 1.0)
    return scaled


def unclipped_box(
    label_file: Path, want: int, inset: float
) -> tuple[float, float, float, float] | None:
    """First box of class `want` that clears the frame edge by `inset`, else None."""
    for cid, cx, cy, bw, bh in read_boxes(label_file):
        if cid != want:
            continue
        if cx - bw / 2 <= inset or cx + bw / 2 >= 1.0 - inset:
            return None
        if cy - bh / 2 <= inset or cy + bh / 2 >= 1.0 - inset:
            return None
        return (cx, cy, bw, bh)
    return None


def synthetic_pool(want: int) -> list[tuple[float, Path, tuple[float, float, float, float]]]:
    """Largest in-frame boxes of class `want` across the synthetic splits, one per frame."""
    pool: list[tuple[float, Path, tuple[float, float, float, float]]] = []
    seen: set[str] = set()
    for split in ("train", "val"):
        labels, images = SYNTH_DIR / split / "labels", SYNTH_DIR / split / "images"
        if not labels.is_dir():
            continue
        for label_file in sorted(labels.glob("*.txt")):
            if label_file.stem in seen:
                continue
            box = unclipped_box(label_file, want, 0.02)
            if box is None:
                continue
            image_path = images / f"{label_file.stem}.jpg"
            if image_path.exists():
                pool.append((box[2] * box[3], image_path, box))
                seen.add(label_file.stem)
    pool.sort(key=lambda row: -row[0])
    return pool[:SYNTH_POOL]


def synthetic_candidates(name: str) -> list[Candidate]:
    pool = synthetic_pool(SYNTH_CLASS[name])
    reference = thumbnail_histogram(name)
    areas, skies, sharps, similarities = [], [], [], []
    for area, image_path, box in pool:
        image = imread(image_path)
        crop = crop_box(image, box, SYNTH_MARGIN)
        areas.append(area)
        skies.append(sky_fraction(image, box))
        sharps.append(sharpness(crop))
        similarities.append(cv2.compareHist(reference, crop_histogram(crop), cv2.HISTCMP_CORREL))

    area_s, sharp_s, sim_s = rescale(areas), rescale(sharps), rescale(similarities)
    sky_s = np.asarray(skies)
    candidates = [
        Candidate(
            image_path,
            box,
            SYNTH_MARGIN,
            0.85 * area_s[i] + 1.30 * sim_s[i] + 0.35 * sharp_s[i] - 1.10 * sky_s[i],
        )
        for i, (_area, image_path, box) in enumerate(pool)
    ]
    candidates.sort(key=lambda c: -c.score)
    return candidates


def real_candidates(name: str) -> list[Candidate]:
    recording = EVAL_DIR / RECORDING[name]
    labels, images = recording / "labels", recording / "images"
    pool: list[tuple[float, Path, tuple[float, float, float, float]]] = []
    for label_file in sorted(labels.glob("*.txt")):
        box = unclipped_box(label_file, REAL_OPPONENT_CLASS, 0.01)
        if box is None:
            continue
        image_path = images / f"{label_file.stem}.png"
        if image_path.exists():
            pool.append((box[2] * box[3], image_path, box))
    pool.sort(key=lambda row: -row[0])
    pool = pool[:REAL_POOL]

    areas, sharps = [], []
    for area, image_path, box in pool:
        crop = crop_box(imread(image_path), box, REAL_MARGIN)
        areas.append(area)
        sharps.append(sharpness(crop))
    area_s, sharp_s = rescale(areas), rescale(sharps)
    candidates = [
        Candidate(image_path, box, REAL_MARGIN, 1.00 * area_s[i] + 0.80 * sharp_s[i])
        for i, (_area, image_path, box) in enumerate(pool)
    ]
    candidates.sort(key=lambda c: -c.score)

    # Drop near-duplicates so variants show different moments of the fight.
    kept: list[Candidate] = []
    for candidate in candidates:
        stamp = int(candidate.image.stem)
        if all(abs(stamp - int(k.image.stem)) >= MIN_SEPARATION_NS for k in kept):
            kept.append(candidate)
    return kept


def order_cell(name: str, column: str, candidates: list[Candidate]) -> list[Candidate]:
    """Pin one cell of the grid to its LOCKED frame, else leave it in ranked order."""
    locked = LOCKED.get((name, column))
    if locked is None:
        return candidates
    by_file = {c.image.name: c for c in candidates}
    if locked not in by_file:
        raise KeyError(f"locked {name}/{column} image {locked} is not in the candidate pool")
    return [by_file[locked]]


def load_thumbnail(name: str) -> np.ndarray:
    image = imread(THUMB_DIR / f"{name}.png")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ys, xs = np.nonzero(gray > 25)
    if ys.size:  # tighten onto the cutout so the robot fills the tile
        pad = 8
        y0, y1 = max(0, ys.min() - pad), min(image.shape[0], ys.max() + pad)
        x0, x1 = max(0, xs.min() - pad), min(image.shape[1], xs.max() + pad)
        image = image[y0:y1, x0:x1]
    return image


def to_rgb(image: np.ndarray, tile: int = 420) -> np.ndarray:
    return cv2.cvtColor(cv2.resize(image, (tile, tile)), cv2.COLOR_BGR2RGB)


def square(image: np.ndarray) -> np.ndarray:
    """Pad to square on the short axis so resizing does not stretch the robot."""
    height, width = image.shape[:2]
    side = max(height, width)
    canvas = np.zeros((side, side, 3), np.uint8)
    y0, x0 = (side - height) // 2, (side - width) // 2
    canvas[y0 : y0 + height, x0 : x0 + width] = image
    return canvas


def render(variant: int, picks: dict[str, tuple[Candidate, Candidate]], out_path: Path) -> None:
    fig, axes = plt.subplots(len(ROWS), 3, figsize=(11.6, 3.55 * len(ROWS)))
    fig.suptitle(
        "Meshy fidelity: thumbnail vs render vs real fight",
        fontsize=17,
        fontweight="bold",
        x=0.02,
        ha="left",
        y=0.985,
    )
    headers = ["Thumbnail (Meshy input)", "Meshy render (training)", "Real robot (fight)"]

    for row, (name, ap, verdict) in enumerate(ROWS):
        synth, real = picks[name]
        tiles = [
            square(load_thumbnail(name)),
            square(crop_box(imread(synth.image), synth.box, synth.margin)),
            square(crop_box(imread(real.image), real.box, real.margin)),
        ]
        for col in range(3):
            ax = axes[row, col]
            ax.imshow(to_rgb(tiles[col]))
            ax.set_xticks([])
            ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_visible(False)
            if row == 0:
                ax.set_title(headers[col], fontsize=12, fontweight="bold", pad=10)
        axes[row, 0].set_ylabel(
            f"{name}\nAP {ap:.3f}\n{verdict}",
            fontsize=12,
            fontweight="bold",
            rotation=0,
            ha="right",
            va="center",
            labelpad=18,
        )

    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.965))
    fig.savefig(out_path, dpi=110, facecolor="white")
    plt.close(fig)
    print(f"wrote {out_path}")
    for name, (synth, real) in picks.items():
        print(f"   v{variant:02d} {name:14s} synth={synth.image.name} real={real.image.name}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=REPO
        / "docs/experiments/perception_performance/assets/meshy_fidelity_comparison.png",
        help="output path; variants get a _vNN suffix",
    )
    parser.add_argument(
        "--variants",
        type=int,
        default=1,
        help="render N alternates instead of the locked figure (see LOCKED)",
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    ranked = {}
    for name, _ap, _verdict in ROWS:
        synth = order_cell(name, "synth", synthetic_candidates(name))
        real = order_cell(name, "real", real_candidates(name))
        print(f"{name}: {len(synth)} synthetic candidates, {len(real)} real candidates")
        ranked[name] = (synth, real)

    for variant in range(1, args.variants + 1):
        picks = {}
        for name, (synth, real) in ranked.items():
            picks[name] = (synth[(variant - 1) % len(synth)], real[(variant - 1) % len(real)])
        if args.variants == 1:
            out_path = args.output
        else:
            out_path = args.output.with_name(
                f"{args.output.stem}_v{variant:02d}{args.output.suffix}"
            )
        render(variant, picks, out_path)


if __name__ == "__main__":
    main()
