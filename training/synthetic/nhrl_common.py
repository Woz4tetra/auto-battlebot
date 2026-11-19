"""Shared helpers for the NHRL robot distractor pipeline."""

from __future__ import annotations

import json
import os
import re
import tempfile
from pathlib import Path
from typing import Any, Literal, TypedDict

import numpy as np

BRETTZONE_API_BASE = "https://brettzone.nhrl.io/brettZone/api.php"
BRETTZONE_THUMB_BASE = "https://brettzone.nhrl.io/robots-thumb"
MESHY_API_BASE = "https://api.meshy.ai/openapi/v1"

SIDECAR_SCHEMA_VERSION = 3
KEYPOINT_FRAME = "gltf_model"
KEYPOINT_METHOD = "pca_centerline_v1"

STATUS_SKIPPED_NO_THUMBNAIL = "skipped_no_thumbnail"
STATUS_SUBMITTED = "submitted"
STATUS_SUCCEEDED = "succeeded"
STATUS_DOWNLOADED = "downloaded"
STATUS_DONE = "done"
STATUS_FAILED = "failed"

STATE_REJECTED_KEY = "rejected"

STATE_FILENAME = ".nhrl_generation_state.json"
BOT_CACHE_FILENAME = ".brettzone_bots_cache.json"

WEIGHT_CLASS_LABELS = {3: "3lb", 12: "12lb", 30: "30lb"}

_PLACEHOLDER_NAME_RE = re.compile(r"^\s*(bracket seed \d+|\(bye\))\s*$", re.IGNORECASE)
_SANITIZE_RE = re.compile(r"[^A-Za-z0-9._-]+")


_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = Path(__file__).resolve().parents[2]


def resolve_cli_path(path: Path) -> Path:
    """Resolve a CLI path independent of the caller's CWD."""
    if path.is_absolute():
        return path.resolve()
    for candidate in (_PROJECT_ROOT / path, Path.cwd() / path, _SCRIPT_DIR / path):
        resolved = candidate.resolve()
        if resolved.exists():
            return resolved
    return (_PROJECT_ROOT / path).resolve()


def bot_thumbnail_url(clean_name: str) -> str:
    """Public BrettZone thumbnail URL for a bot's clean name."""
    return f"{BRETTZONE_THUMB_BASE}/{clean_name}.png"


def sanitize_name(clean_name: str) -> str:
    """Turn a bot clean name into a filesystem-safe token."""
    token = _SANITIZE_RE.sub("_", clean_name).strip("_")
    return token or "bot"


def model_stem(clean_name: str) -> str:
    """Filename stem (no extension) for a generated NHRL robot model."""
    return f"nhrl_{sanitize_name(clean_name)}"


def weight_class_labels(codes: list[int]) -> list[str]:
    """Map integer NHRL weight-class codes to human-readable labels."""
    return [WEIGHT_CLASS_LABELS.get(int(c), f"{int(c)}lb") for c in codes]


def is_placeholder_bot(name: str) -> bool:
    """True for tournament artifacts (``Bracket Seed N``, ``(Bye)``)."""
    return bool(_PLACEHOLDER_NAME_RE.match(name or ""))


def select_bots(
    bots: list[dict[str, Any]],
    limit: int,
    min_fights: int,
    oversample: float = 0.2,
) -> list[dict[str, Any]]:
    """Filter, rank, and truncate the BrettZone bot list."""
    keep: list[dict[str, Any]] = []
    for bot in bots:
        clean_name = str(bot.get("cleanName", "")).strip()
        if not clean_name:
            continue
        if is_placeholder_bot(str(bot.get("name", ""))):
            continue
        if int(bot.get("totalFights", 0)) < min_fights:
            continue
        keep.append(bot)

    keep.sort(key=lambda b: (-int(b.get("totalFights", 0)), str(b.get("cleanName", ""))))
    take = max(0, int(np.ceil(limit * (1.0 + oversample))))
    return keep[:take]


def atomic_write_json(path: Path, obj: Any) -> None:
    """Write JSON to *path* atomically (temp file + os.replace)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(obj, f, indent=2)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def load_json(path: Path) -> dict[str, Any] | None:
    """Load a JSON object from *path*, or ``None`` if missing/unreadable."""
    if not path.exists():
        return None
    try:
        with path.open() as f:
            data = json.load(f)
        return data if isinstance(data, dict) else None
    except (json.JSONDecodeError, OSError):
        return None


def load_state(state_path: Path) -> dict[str, Any]:
    """Load the generation state map (empty dict if absent)."""
    return load_json(state_path) or {}


def save_state(state_path: Path, state: dict[str, Any]) -> None:
    """Persist the generation state map atomically."""
    atomic_write_json(state_path, state)


def is_rejected(entry: dict[str, Any]) -> bool:
    """True if a state entry was human-rejected (skip download + meshing)."""
    return bool(entry.get(STATE_REJECTED_KEY))


class TopdownSidecar(TypedDict):
    image: str
    image_w: int
    image_h: int
    affine: list[list[float]]
    hull: list[list[float]]
    y_ground: float
    default_axis: list[float]


def build_sidecar(
    *,
    name: str,
    clean_name: str,
    weight_classes: list[str],
    total_fights: int,
    thumbnail_url: str,
    meshy_params: dict[str, Any] | None,
    keypoints: tuple[np.ndarray, np.ndarray] | None,
    generated_at: str,
    source: str,
    reviewed: bool = False,
    topdown: TopdownSidecar | None = None,
) -> dict[str, Any]:
    """Assemble the sidecar JSON payload."""
    kp_obj: dict[str, Any] | None
    if keypoints is None:
        kp_obj = None
    else:
        front, back = keypoints
        kp_obj = {
            "frame": KEYPOINT_FRAME,
            "method": KEYPOINT_METHOD,
            "front": [round(float(v), 6) for v in front],
            "back": [round(float(v), 6) for v in back],
        }
    return {
        "schema_version": SIDECAR_SCHEMA_VERSION,
        "name": name,
        "clean_name": clean_name,
        "weight_classes": weight_classes,
        "total_fights": total_fights,
        "thumbnail_url": thumbnail_url,
        "source": source,
        "meshy": meshy_params,
        "generated_at": generated_at,
        "reviewed": bool(reviewed),
        "topdown": topdown,
        "keypoints": kp_obj,
    }


def sidecar_path_for(model_path: Path) -> Path:
    """Sidecar JSON path for a model file (``foo.glb`` -> ``foo.json``)."""
    return model_path.with_suffix(".json")


def footprint_axes(
    vertices: np.ndarray, slice_fraction: float = 0.05
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float] | None:
    """Derive the horizontal principal axes of a mesh's ground footprint."""
    if vertices.ndim != 2 or vertices.shape[0] < 3 or vertices.shape[1] != 3:
        return None

    y = vertices[:, 1]
    y_min = float(y.min())
    height = float(y.max()) - y_min
    if height <= 1e-6:
        return None

    x_ext_full = float(vertices[:, 0].max() - vertices[:, 0].min())
    z_ext_full = float(vertices[:, 2].max() - vertices[:, 2].min())
    max_horiz_full = max(x_ext_full, z_ext_full)
    if max_horiz_full < 1e-5:
        return None

    slice_pts = vertices
    for frac in (slice_fraction, 0.10, 0.25, 0.5, 1.0):
        pts = vertices[y <= y_min + frac * height]
        if len(pts) < 8:
            continue
        xe = float(pts[:, 0].max() - pts[:, 0].min())
        ze = float(pts[:, 2].max() - pts[:, 2].min())
        if max(xe, ze) >= 0.5 * max_horiz_full:
            slice_pts = pts
            break

    xz = slice_pts[:, [0, 2]].astype(float)
    centroid_xz = xz.mean(axis=0)
    centered = xz - centroid_xz
    cov = centered.T @ centered / max(len(centered) - 1, 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    u1 = np.asarray(eigvecs[:, np.argmax(eigvals)], dtype=float)
    norm = float(np.linalg.norm(u1))
    if norm < 1e-12:
        return None
    u1 = u1 / norm

    if u1[0] < -1e-12 or (abs(u1[0]) <= 1e-12 and u1[1] < 0.0):
        u1 = -u1
    u2 = np.array([-u1[1], u1[0]], dtype=float)

    return centroid_xz, u1, u2, slice_pts, y_min


def keypoints_along_axis(
    base_xz: np.ndarray, axis_u: np.ndarray, points: np.ndarray, y_ground: float
) -> tuple[np.ndarray, np.ndarray]:
    """Project *points* onto *axis_u* and return centered front/back extremes."""
    xz = points[:, [0, 2]].astype(float)
    t = (xz - base_xz) @ axis_u
    t_max = float(t.max())
    t_min = float(t.min())

    def _point_at(t_val: float) -> np.ndarray:
        x = float(base_xz[0] + t_val * axis_u[0])
        z = float(base_xz[1] + t_val * axis_u[1])
        return np.array([x, y_ground, z], dtype=float)

    return _point_at(t_max), _point_at(t_min)


def compute_bottom_slice_keypoints(
    vertices: np.ndarray, slice_fraction: float = 0.05
) -> tuple[np.ndarray, np.ndarray] | None:
    """Derive centered front/back keypoints for a Y-up mesh."""
    axes = footprint_axes(vertices, slice_fraction)
    if axes is None:
        return None
    _centroid_xz, u1, _u2, _slice_pts, y_ground = axes
    xz = vertices[:, [0, 2]]
    base_xz = 0.5 * (xz.min(axis=0) + xz.max(axis=0))
    return keypoints_along_axis(base_xz, u1, vertices, y_ground)


def convex_hull_2d(points_xz: np.ndarray) -> np.ndarray:
    """Convex hull of 2D ``(N, 2)`` points (Andrew's monotone chain, numpy-only)."""
    pts = np.unique(np.asarray(points_xz, dtype=float), axis=0)
    if len(pts) <= 2:
        return pts
    order = np.lexsort((pts[:, 1], pts[:, 0]))
    pts = pts[order]

    def _cross(o: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
        return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))

    def _half(sorted_pts: np.ndarray) -> list[np.ndarray]:
        chain: list[np.ndarray] = []
        for p in sorted_pts:
            while len(chain) >= 2 and _cross(chain[-2], chain[-1], p) <= 0:
                chain.pop()
            chain.append(p)
        return chain[:-1]

    lower = _half(pts)
    upper = _half(pts[::-1])
    return np.array(lower + upper, dtype=float)


SidecarAction = Literal["write", "skip", "keep_reviewed"]


def sidecar_action(existing: dict[str, Any] | None, overwrite: bool) -> SidecarAction:
    if existing is None:
        return "write"
    if existing.get("reviewed"):
        return "keep_reviewed"
    return "write" if overwrite else "skip"


def world_to_model(world: np.ndarray) -> np.ndarray:
    """Blender world (Z-up) to glTF model frame (Y-up), the inverse of the importer's [x, -z, y]."""
    return np.column_stack([world[:, 0], world[:, 2], -world[:, 1]])
