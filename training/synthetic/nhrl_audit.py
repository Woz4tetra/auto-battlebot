"""VRAM audit CSV helpers for distractor models (trimesh-based).

Mirrors the ``distractor_gpu_audit.csv`` schema written by
``download_objaverse.py`` so ``render_scenes.py``'s pre-import VRAM budget
check works for generated NHRL robots.  Kept separate from
``download_objaverse.py`` so the NHRL pipeline runs in the root venv without an
``objaverse`` dependency.
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import trimesh

AUDIT_COLUMNS = [
    "source",
    "name",
    "file",
    "disk_mb",
    "mesh_count",
    "verts",
    "faces",
    "textures",
    "max_tex_dim",
    "geom_mb_est",
    "tex_mb_est",
    "total_gpu_mb_est",
]


def _image_key(img: Any) -> tuple[str, Any]:
    fn = getattr(img, "filename", None)
    return ("fn", fn) if fn else ("id", id(img))


def estimate_model_gpu_row(model_path: Path, source: str) -> dict[str, Any]:
    """Estimate a model's GPU memory footprint and return one audit CSV row."""
    loaded = trimesh.load(str(model_path), force="scene")
    if not isinstance(loaded, trimesh.Scene):
        raise RuntimeError(f"Failed to load model as scene: {model_path}")

    geom_bytes = 0
    tex_bytes = 0
    tex_count = 0
    max_tex_dim = 0
    seen_images: set[tuple[str, Any]] = set()
    mesh_count = 0
    vert_total = 0
    face_total = 0

    for geom in loaded.geometry.values():
        mesh_count += 1
        v = len(geom.vertices) if getattr(geom, "vertices", None) is not None else 0
        fcount = len(geom.faces) if getattr(geom, "faces", None) is not None else 0
        vert_total += v
        face_total += fcount
        # Approximate per-vertex attrs + index buffer.
        geom_bytes += v * (12 + 12 + 8 + 4) + fcount * 12

        mat = getattr(getattr(geom, "visual", None), "material", None)
        if mat is None:
            continue
        for attr in (
            "image",
            "baseColorTexture",
            "metallicRoughnessTexture",
            "normalTexture",
            "emissiveTexture",
            "occlusionTexture",
            "roughnessTexture",
            "metallicTexture",
        ):
            img = getattr(mat, attr, None)
            if img is None:
                continue
            key = _image_key(img)
            if key in seen_images:
                continue
            try:
                w, h = img.size
            except (AttributeError, TypeError):
                continue
            seen_images.add(key)
            max_tex_dim = max(max_tex_dim, w, h)
            tex_count += 1
            # RGBA8 with full mip chain approximation.
            tex_bytes += int(w * h * 4 * (4.0 / 3.0))

    total_bytes = geom_bytes + tex_bytes
    return {
        "source": source,
        "name": model_path.name,
        "file": str(model_path.resolve()),
        "disk_mb": round(model_path.stat().st_size / (1024**2), 3),
        "mesh_count": mesh_count,
        "verts": vert_total,
        "faces": face_total,
        "textures": tex_count,
        "max_tex_dim": max_tex_dim,
        "geom_mb_est": round(geom_bytes / (1024**2), 3),
        "tex_mb_est": round(tex_bytes / (1024**2), 3),
        "total_gpu_mb_est": round(total_bytes / (1024**2), 3),
    }


def load_audit_table(audit_csv: Path) -> dict[str, dict[str, Any]]:
    """Load existing audit rows keyed by absolute file path."""
    rows_by_file: dict[str, dict[str, Any]] = {}
    if not audit_csv.exists():
        return rows_by_file
    with audit_csv.open(newline="") as f:
        for row in csv.DictReader(f):
            file_value = row.get("file")
            if file_value:
                rows_by_file[str(Path(file_value).resolve())] = row
    return rows_by_file


def write_audit_table(audit_csv: Path, rows_by_file: dict[str, dict[str, Any]]) -> None:
    """Write audit rows sorted by descending total estimated GPU MB."""
    rows = list(rows_by_file.values())
    rows.sort(key=lambda r: float(r.get("total_gpu_mb_est", 0.0)), reverse=True)
    audit_csv.parent.mkdir(parents=True, exist_ok=True)
    with audit_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=AUDIT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
