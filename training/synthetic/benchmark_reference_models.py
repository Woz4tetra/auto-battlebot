"""Benchmark generated distractor model quality against a baseline directory.

This script compares two distractor-model directories (candidate vs baseline)
using quick mesh-level metrics from exported assets and optional manifest data.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import numpy as np

MODEL_EXTENSIONS = (".glb", ".gltf", ".obj", ".ply")


def progress_iter(iterable, *, total: int | None = None, desc: str = ""):
    try:
        from tqdm import tqdm  # type: ignore

        yield from tqdm(iterable, total=total, desc=desc, unit="item")
        return
    except ModuleNotFoundError:
        pass

    if total is not None and total > 0:
        print(f"{desc}: 0/{total}")
    for idx, item in enumerate(iterable, start=1):
        if total is not None and total > 0:
            step = max(1, total // 10)
            if idx == total or idx % step == 0:
                print(f"{desc}: {idx}/{total}")
        yield item


@dataclass
class DirStats:
    model_count: int
    loadable_count: int
    watertight_count: int
    median_faces: float
    median_vertices: float
    median_aspect_ratio: float
    mean_manifest_quality: float | None


def discover_model_files(root: Path) -> list[Path]:
    if not root.exists():
        return []
    files: list[Path] = []
    for ext in MODEL_EXTENSIONS:
        files.extend(root.glob(f"*{ext}"))
    return sorted(files)


def load_manifest_quality(root: Path) -> list[float]:
    manifest_path = root / "manifest.json"
    if not manifest_path.exists():
        return []
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception:
        return []
    if not isinstance(payload, dict):
        return []
    qualities: list[float] = []
    for entry in payload.values():
        if not isinstance(entry, dict):
            continue
        score = entry.get("quality_score")
        if isinstance(score, (int, float)):
            qualities.append(float(score))
    return qualities


def _aspect_ratio(bounds: np.ndarray) -> float:
    extents = bounds[1] - bounds[0]
    longest = float(np.max(extents))
    shortest = float(max(np.min(extents), 1e-6))
    return longest / shortest


def collect_stats(root: Path) -> DirStats:
    files = discover_model_files(root)
    if not files:
        return DirStats(
            model_count=0,
            loadable_count=0,
            watertight_count=0,
            median_faces=0.0,
            median_vertices=0.0,
            median_aspect_ratio=0.0,
            mean_manifest_quality=None,
        )

    faces: list[int] = []
    vertices: list[int] = []
    aspects: list[float] = []
    loadable = 0
    watertight = 0

    try:
        import trimesh
    except ModuleNotFoundError:
        trimesh = None

    for path in progress_iter(files, total=len(files), desc=f"Scanning {root.name}"):
        if trimesh is None:
            break
        try:
            mesh = trimesh.load(str(path), force="mesh")
        except Exception:
            continue
        if not hasattr(mesh, "faces") or not hasattr(mesh, "vertices"):
            continue
        loadable += 1
        faces.append(int(len(mesh.faces)))
        vertices.append(int(len(mesh.vertices)))
        aspects.append(_aspect_ratio(np.asarray(mesh.bounds)))
        if bool(getattr(mesh, "is_watertight", False)):
            watertight += 1

    qualities = load_manifest_quality(root)
    mean_quality = float(np.mean(qualities)) if qualities else None

    return DirStats(
        model_count=len(files),
        loadable_count=loadable,
        watertight_count=watertight,
        median_faces=float(np.median(faces)) if faces else 0.0,
        median_vertices=float(np.median(vertices)) if vertices else 0.0,
        median_aspect_ratio=float(np.median(aspects)) if aspects else 0.0,
        mean_manifest_quality=mean_quality,
    )


def compare(candidate: DirStats, baseline: DirStats | None) -> dict:
    payload = {
        "candidate": candidate.__dict__,
        "baseline": baseline.__dict__ if baseline is not None else None,
    }
    if baseline is None:
        payload["delta"] = None
        return payload
    payload["delta"] = {
        "model_count": candidate.model_count - baseline.model_count,
        "loadable_count": candidate.loadable_count - baseline.loadable_count,
        "watertight_count": candidate.watertight_count - baseline.watertight_count,
        "median_faces": candidate.median_faces - baseline.median_faces,
        "median_vertices": candidate.median_vertices - baseline.median_vertices,
        "median_aspect_ratio": candidate.median_aspect_ratio - baseline.median_aspect_ratio,
        "mean_manifest_quality": (
            None
            if candidate.mean_manifest_quality is None or baseline.mean_manifest_quality is None
            else candidate.mean_manifest_quality - baseline.mean_manifest_quality
        ),
    }
    return payload


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--candidate-dir",
        type=Path,
        required=True,
        help="Directory for new reference3d outputs",
    )
    p.add_argument(
        "--baseline-dir",
        type=Path,
        default=None,
        help="Optional baseline directory (for example sam3d outputs)",
    )
    p.add_argument(
        "--output-json",
        type=Path,
        default=None,
        help="Optional path to write JSON benchmark report",
    )
    return p


def main() -> None:
    args = build_parser().parse_args()

    candidate = collect_stats(args.candidate_dir.resolve())
    baseline = collect_stats(args.baseline_dir.resolve()) if args.baseline_dir else None
    report = compare(candidate, baseline)

    print("Benchmark report:")
    print(json.dumps(report, indent=2))

    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"Wrote report to {args.output_json}")


if __name__ == "__main__":
    main()
