#!/usr/bin/env python3
"""Render and grade a list of spec variants, then print one comparison table.

Each variant is a name and a list of `section.key=value` overrides for
`training/synthetic/render_cage_view.py`. Every variant renders the event poses through the
synthetic Docker image into `renders/<prefix>_<name>/`, is graded with grade_render.py, and
the render-vs-target means land side by side so a sweep reads as one table.

Variants come from a JSON file: `{"name": ["frame.height_above_mat=0.03", ...], ...}`, or
from --grid KEY V1 V2 ..., which sweeps one key.

Usage:
    venv/bin/python playground/cage_scene/sweep_spec.py runs/cage_scene --prefix h \\
        --grid frame.height_above_mat 0.03 0.05 0.08 --samples 32
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
IMAGE = "auto-battlebot-synthetic"
KEY_METRICS = [
    ("inside", "l1_gray"),
    ("inside", "ssim_gray"),
    ("inside", "edge_chamfer_px"),
    ("inside", "lab_dmean_L"),
    ("outside", "l1_gray"),
    ("outside", "ssim_gray"),
    ("outside", "edge_chamfer_px"),
    ("outside", "lab_dmean_L"),
    ("full", "l1_gray"),
    ("full", "ssim_gray"),
    ("full", "mat_iou"),
]
ROBOT_METRICS = [
    ("robot", "l1_gray"),
    ("robot", "ssim_gray"),
    ("robot", "lab_dmean_L"),
    ("robot", "edge_chamfer_px"),
    ("robot", "robot_box_iou"),
    ("shadow_ring", "l1_gray"),
    ("shadow_ring", "ssim_gray"),
    ("shadow_ring", "edge_chamfer_px"),
    ("shadow_ring", "shadow_drop_gray"),
    ("shadow_ring", "shadow_ratio"),
    ("mat_rest", "l1_gray"),
]


def container_path(path: Path) -> str:
    return "/workspace/" + str(path.resolve().relative_to(REPO_ROOT))


def render(
    scene_dir: Path,
    spec: Path,
    out: Path,
    overrides: list[str],
    samples: int,
    pose_glob: str,
    auto_exposure: bool,
    robot_frames: Path | None = None,
) -> None:
    command = [
        str(REPO_ROOT / "training/synthetic/docker/run_synthetic.sh"),
        "--gpu",
        IMAGE,
        "blenderproc",
        "run",
        container_path(REPO_ROOT / "training/synthetic/render_cage_view.py"),
        "--",
        "--spec",
        container_path(spec),
        "--pose",
        container_path(scene_dir / "poses"),
        "--pose-glob",
        pose_glob,
        "--camera-rect",
        container_path(scene_dir / "camera_rect.json"),
        "--out",
        container_path(out),
        "--samples",
        str(samples),
    ]
    if auto_exposure:
        command += ["--auto-exposure", container_path(scene_dir / "targets")]
    if robot_frames is not None:
        command += ["--robot-frames", container_path(robot_frames)]
    for item in overrides:
        command += ["--set", item]
    out.mkdir(parents=True, exist_ok=True)
    with (out.parent / f"{out.name}.log").open("w") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False)
    if result.returncode != 0 or not any(out.glob("*.png")):
        raise SystemExit(f"render failed for {out.name}; see {out.parent / f'{out.name}.log'}")


def grade(
    scene_dir: Path, renders: Path, grades: Path, robot_frames: Path | None = None
) -> dict[tuple[str, str], float]:
    command = [
        sys.executable,
        str(REPO_ROOT / "playground/cage_scene/grade_render.py"),
        "--renders",
        str(renders),
        "--targets",
        str(scene_dir / "targets"),
        "--out",
        str(grades),
    ]
    if robot_frames is not None:
        command += ["--robot-frames", str(robot_frames)]
    subprocess.run(command, check=True, capture_output=True)
    agg: dict[tuple[str, str], list[float]] = defaultdict(list)
    with (grades / "metrics.csv").open() as handle:
        for row in csv.DictReader(handle):
            if row["source"] == "render" and row["value"] not in ("nan", "inf"):
                agg[(row["region"], row["metric"])].append(float(row["value"]))
    return {k: float(np.mean(v)) for k, v in agg.items()}


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument(
        "--spec", type=Path, default=REPO_ROOT / "training/synthetic/cage/cage2_overhead_high.toml"
    )
    parser.add_argument("--variants", type=Path, help="JSON {name: [overrides]}")
    parser.add_argument(
        "--grid", nargs="+", metavar="KEY_OR_VALUE", help="KEY V1 V2 ... sweeps one key"
    )
    parser.add_argument(
        "--base", action="append", default=[], help="override applied to every variant"
    )
    parser.add_argument("--prefix", default="sweep")
    parser.add_argument("--samples", type=int, default=32)
    parser.add_argument("--pose-glob", default="cage2_*.toml")
    parser.add_argument("--no-auto-exposure", action="store_true")
    parser.add_argument("--reuse", action="store_true", help="skip variants whose renders exist")
    parser.add_argument(
        "--robot-frames",
        type=Path,
        default=None,
        help="render and grade MRS BUFF at the recovered poses in this directory instead",
    )
    args = parser.parse_args()

    variants: dict[str, list[str]] = {}
    if args.variants:
        variants.update(json.loads(args.variants.read_text()))
    if args.grid:
        key, values = args.grid[0], args.grid[1:]
        short = key.split(".")[-1]
        for value in values:
            label = value.replace("[", "").replace("]", "").replace(", ", "_").replace(",", "_")
            variants[f"{short}_{label}"] = [f"{key}={value}"]
    if not variants:
        variants["base"] = []

    table: dict[str, dict[tuple[str, str], float]] = {}
    for name, overrides in variants.items():
        tag = f"{args.prefix}_{name}"
        renders = args.scene_dir / "renders" / tag
        if not (args.reuse and any(renders.glob("*.png"))):
            print(f"render {tag}: {args.base + overrides}", flush=True)
            render(
                args.scene_dir,
                args.spec,
                renders,
                args.base + overrides,
                args.samples,
                args.pose_glob,
                not args.no_auto_exposure,
                args.robot_frames,
            )
        table[tag] = grade(
            args.scene_dir, renders, args.scene_dir / "grades" / tag, args.robot_frames
        )
        (args.scene_dir / "grades" / tag / "overrides.json").write_text(
            json.dumps(args.base + overrides) + "\n"
        )

    metrics = ROBOT_METRICS if args.robot_frames is not None else KEY_METRICS
    header = "| variant | " + " | ".join(f"{r}.{m}" for r, m in metrics) + " |"
    print(header)
    print("|---|" + "---|" * len(metrics))
    for tag, values in table.items():
        cells = [f"{values.get(key, float('nan')):.3f}" for key in metrics]
        print(f"| {tag} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
