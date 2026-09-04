"""Shared pieces for the embedding prototype probe stages.

See docs/experiments/opponent_embedding/embedding_prototype_probe_plan.md. Stage scripts:
embedding_probe_extract.py (candidate boxes), embedding_probe_embed.py (cached
embeddings), embedding_probe_score.py (Evals A/B/D), embedding_probe_playback.py (Eval C),
embedding_probe_render.py (overlay videos).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

_HERE = Path(__file__).parent
_REPO = _HERE.parent.parent

DEFAULT_EVAL_ROOT = _REPO / "training" / "data" / "nhrl_keypoints_eval_test"
OUTPUT_ROOT = _REPO / "training" / "data" / "embedding_probe"
DEFAULT_ENGINE = (
    _REPO
    / "data"
    / "models"
    / "yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31_x86_64_sm89.engine"
)
# Engine class order mirrors config label_indices (config/_desktop.toml), lowercased.
ENGINE_LABELS = ("opponent", "house_bot")
LOW_CONF = 0.05
DEPLOYED_CONF = 0.6
NMS_IOU = 0.45
OPPONENT = "opponent"
# Proposals below this IoU against every GT box are hard negatives; at or above
# PROPOSAL_MATCH_IOU against a GT opponent they are FN-rescue candidates.
HARD_NEGATIVE_IOU = 0.3
PROPOSAL_MATCH_IOU = 0.5
SEED_FRAMES = 2
PAD_FRACTIONS = (0.0, 0.10, 0.25)
DEFAULT_PAD = 0.10

CANDIDATES_CSV = "candidates.csv"
MANIFEST_JSON = "manifest.json"
EMBEDDINGS_NPZ = "embeddings_{embedder}.npz"

# Recording directory name -> short name used in tables and figures.
SHORT_NAMES = {
    "main_2026-05-01_17-42-20__2026-05-01T17-42-24": "17-42",
    "main_2026-05-02_10-06-02_repaired__2026-05-02T10-06-06": "10-06",
    "main_2026-05-02_11-45-05_repaired__2026-05-02T11-45-08": "11-45",
    "main_2026-05-02_14-12-25_repaired__2026-05-02T14-12-27": "14-12",
    "main_2026-05-02_15-35-00_repaired__2026-05-02T15-35-04": "15-35",
    "main_2026-05-02_16-18-05_repaired__2026-05-02T16-18-07": "16-18",
    "main_2026-05-02_17-26-12_repaired__2026-05-02T17-26-14": "17-26",
    "mrs_buff_mk3_massd_ns_jetson_2026-08-29_13-08-16__2026-08-29T13-20-08": "massd",
}
CORE_RECORDINGS = ("10-06", "11-45", "14-12", "17-42", "massd")
STRESS_RECORDINGS = ("15-35", "16-18", "17-26")


def short_name(recording: str) -> str:
    return SHORT_NAMES.get(recording, recording)


def iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Pairwise IoU between (N, 4) and (M, 4) xyxy boxes."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    union = area_a[:, None] + area_b[None, :] - inter
    return np.where(union > 0, inter / union, 0.0)


def crop_box(image: np.ndarray, box: np.ndarray, pad_fraction: float) -> np.ndarray:
    """Crop an xyxy box with symmetric context padding, clamped to the image."""
    height, width = image.shape[:2]
    x1, y1, x2, y2 = box
    pad_x = (x2 - x1) * pad_fraction
    pad_y = (y2 - y1) * pad_fraction
    xa = int(np.floor(max(0.0, x1 - pad_x)))
    ya = int(np.floor(max(0.0, y1 - pad_y)))
    xb = int(np.ceil(min(float(width), x2 + pad_x)))
    yb = int(np.ceil(min(float(height), y2 + pad_y)))
    xb = max(xb, xa + 1)
    yb = max(yb, ya + 1)
    return image[ya:yb, xa:xb]


def recording_dir(recording: str) -> Path:
    return OUTPUT_ROOT / recording


def load_candidates(recording: str) -> pd.DataFrame:
    path = recording_dir(recording) / CANDIDATES_CSV
    if not path.exists():
        raise SystemExit(f"No candidates for {recording}; run embedding_probe_extract.py first")
    return pd.read_csv(path)


def load_recordings(eval_root: Path) -> dict[str, dict]:
    """Per-recording validated GT frames via score.load_gt.

    Returns {recording: {"stamps": sorted stamps, "frames": {stamp: (boxes, labels)},
    "images": {stamp: path}}}. Keypoints are dropped; this probe is box-only.
    """
    from score import load_gt

    gt_frames, _names, images = load_gt(eval_root)
    recordings: dict[str, dict] = {}
    for stamp, (boxes, labels, _kps) in gt_frames.items():
        recording = images[stamp].parent.parent.name
        entry = recordings.setdefault(recording, {"frames": {}, "images": {}})
        entry["frames"][stamp] = (boxes, labels)
        entry["images"][stamp] = images[stamp]
    for entry in recordings.values():
        entry["stamps"] = sorted(entry["frames"])
    return recordings


def seed_stamps(entry: dict, n_frames: int = SEED_FRAMES) -> list[int]:
    """Earliest validated stamps that contain at least one opponent box."""
    chosen = []
    for stamp in entry["stamps"]:
        _boxes, labels = entry["frames"][stamp]
        if OPPONENT in labels:
            chosen.append(stamp)
        if len(chosen) == n_frames:
            break
    return chosen
