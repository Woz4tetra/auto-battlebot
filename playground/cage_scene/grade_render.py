#!/usr/bin/env python3
"""Grade cage renders against the robot-free target frames, per region.

Each render `<stem>.png` under --renders is paired with the target frames it stands for: the
clip of the same name, or every clip of the event an event pose (`cage2_<event>`) was fitted
from. Metrics are computed on the full frame, inside the mat hull, and outside it, and are
written in long form to `metrics.csv` with a `source` column so baselines sit beside the render:

    render            the render under test
    baseline_mean     the pixel mean of all targets, scored against each target (the
                      clip-to-clip floor from exposure and mat wear)
    baseline_flat     the target's own mat-mean colour painted everywhere
    baseline_previous --baseline-render, another render directory, if given

Metrics: l1_gray, l1_rgb (0..255, at --scale), ssim_gray (11x11 Gaussian, sigma 1.5),
edge_chamfer_px (symmetric Canny edge distance, full resolution), lab_dmean_L/a/b,
lab_dstd_L/a/b, mat_iou (overlap of the render's mat mask with the target hull, full-frame only).
`report.md` holds the per-region means and a per-clip table, and `<stem>_sheet.png` shows
target | render | abs diff | edge overlay.

Usage:
    venv/bin/python playground/cage_scene/grade_render.py \\
        --renders runs/cage_scene/renders/r001 --targets runs/cage_scene/targets \\
        --out runs/cage_scene/grades/r001
    venv/bin/python playground/cage_scene/grade_render.py --compare runs/cage_scene/grades/r001 \\
        runs/cage_scene/grades/r002
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np

REGIONS = ("full", "inside", "outside")
BOUNDARY_ERODE_PX = 3
SSIM_C1 = (0.01 * 255) ** 2
SSIM_C2 = (0.03 * 255) ** 2


def ssim_map(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = a.astype(np.float64)
    b = b.astype(np.float64)
    blur = lambda x: cv2.GaussianBlur(x, (11, 11), 1.5)  # noqa: E731
    mu_a, mu_b = blur(a), blur(b)
    var_a = blur(a * a) - mu_a**2
    var_b = blur(b * b) - mu_b**2
    cov = blur(a * b) - mu_a * mu_b
    num = (2 * mu_a * mu_b + SSIM_C1) * (2 * cov + SSIM_C2)
    den = (mu_a**2 + mu_b**2 + SSIM_C1) * (var_a + var_b + SSIM_C2)
    return np.asarray(num / den)


def canny_auto(gray: np.ndarray) -> np.ndarray:
    grad = cv2.magnitude(cv2.Sobel(gray, cv2.CV_32F, 1, 0), cv2.Sobel(gray, cv2.CV_32F, 0, 1))
    median = float(np.median(grad[grad > 0])) if (grad > 0).any() else 10.0
    return cv2.Canny(gray, int(0.66 * median), int(1.33 * median))


def chamfer_px(edges_a: np.ndarray, edges_b: np.ndarray, region: np.ndarray) -> float:
    """Symmetric mean distance between two edge maps, over edge pixels inside `region`."""
    out = []
    for src, dst in ((edges_a, edges_b), (edges_b, edges_a)):
        if not (dst > 0).any():
            return float("nan")
        dist = cv2.distanceTransform((dst == 0).astype(np.uint8), cv2.DIST_L2, 5)
        pick = (src > 0) & region
        if pick.any():
            out.append(float(dist[pick].mean()))
    return float(np.mean(out)) if out else float("nan")


def region_masks(hull_mask: np.ndarray) -> dict[str, np.ndarray]:
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * BOUNDARY_ERODE_PX + 1,) * 2)
    inside = cv2.erode(hull_mask, kernel) > 0
    outside = cv2.erode(255 - hull_mask, kernel) > 0
    return {"full": np.ones_like(inside, dtype=bool), "inside": inside, "outside": outside}


def image_metrics(
    target_bgr: np.ndarray, image_bgr: np.ndarray, regions: dict[str, np.ndarray], scale: float
) -> dict[tuple[str, str], float]:
    """All per-region metrics for one image against one target."""
    small_size = (int(target_bgr.shape[1] * scale), int(target_bgr.shape[0] * scale))
    t_small = cv2.resize(target_bgr, small_size, interpolation=cv2.INTER_AREA)
    i_small = cv2.resize(image_bgr, small_size, interpolation=cv2.INTER_AREA)
    t_gray = cv2.cvtColor(target_bgr, cv2.COLOR_BGR2GRAY)
    i_gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    t_gray_small = cv2.cvtColor(t_small, cv2.COLOR_BGR2GRAY)
    i_gray_small = cv2.cvtColor(i_small, cv2.COLOR_BGR2GRAY)
    ssim = ssim_map(t_gray_small, i_gray_small)
    t_lab = cv2.cvtColor(target_bgr, cv2.COLOR_BGR2Lab).astype(np.float64)
    i_lab = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2Lab).astype(np.float64)
    t_edges = canny_auto(t_gray)
    i_edges = canny_auto(i_gray)

    out: dict[tuple[str, str], float] = {}
    for name, region in regions.items():
        region_small = (
            cv2.resize(region.astype(np.uint8), small_size, interpolation=cv2.INTER_NEAREST) > 0
        )
        if region_small.sum() == 0 or region.sum() == 0:
            continue
        out[(name, "l1_gray")] = float(
            np.abs(t_gray_small.astype(float) - i_gray_small)[region_small].mean()
        )
        out[(name, "l1_rgb")] = float(np.abs(t_small.astype(float) - i_small)[region_small].mean())
        out[(name, "ssim_gray")] = float(ssim[region_small].mean())
        out[(name, "edge_chamfer_px")] = chamfer_px(t_edges, i_edges, region)
        for channel, label in enumerate("Lab"):
            t_c, i_c = t_lab[:, :, channel][region], i_lab[:, :, channel][region]
            out[(name, f"lab_dmean_{label}")] = float(i_c.mean() - t_c.mean())
            out[(name, f"lab_dstd_{label}")] = float(i_c.std() - t_c.std())
    return out


def mat_mask_iou(target_hull: np.ndarray, render_mat_mask: np.ndarray | None) -> float:
    """Overlap of the rendered mat with the target's mat hull; 1.0 is a perfect outline match."""
    if render_mat_mask is None:
        return float("nan")
    a, b = target_hull > 0, render_mat_mask > 0
    union = (a | b).sum()
    return float((a & b).sum() / union) if union else float("nan")


def contact_sheet(
    target: np.ndarray, render: np.ndarray, hull: np.ndarray, title: str
) -> np.ndarray:
    diff = cv2.applyColorMap(
        np.clip(np.abs(target.astype(int) - render.astype(int)).mean(axis=2) * 3, 0, 255).astype(
            np.uint8
        ),
        cv2.COLORMAP_INFERNO,
    )
    overlay = cv2.cvtColor(cv2.cvtColor(target, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR) // 2
    overlay[canny_auto(cv2.cvtColor(target, cv2.COLOR_BGR2GRAY)) > 0] = (0, 0, 255)
    overlay[canny_auto(cv2.cvtColor(render, cv2.COLOR_BGR2GRAY)) > 0] = (255, 255, 0)
    contours, _ = cv2.findContours(hull, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 255, 0), 2)
    panels = [target, render, diff, overlay]
    half = [
        cv2.resize(p, (p.shape[1] // 2, p.shape[0] // 2), interpolation=cv2.INTER_AREA)
        for p in panels
    ]
    top = np.hstack(half[:2])
    bottom = np.hstack(half[2:])
    sheet = np.vstack([top, bottom])
    cv2.putText(sheet, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
    cv2.putText(sheet, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return sheet


def targets_for_render(stem: str, targets_dir: Path) -> list[Path]:
    direct = targets_dir / stem
    if (direct / "target.png").exists():
        return [direct]
    matches = []
    for meta_path in sorted(targets_dir.glob("*/meta.json")):
        meta = json.loads(meta_path.read_text())
        group = str(meta.get("event_group") or meta.get("event", ""))
        if group and stem.endswith(group):
            matches.append(meta_path.parent)
    return matches


def load_statuses(targets_dir: Path) -> dict[str, str]:
    summary = targets_dir.parent / "poses" / "summary.csv"
    if not summary.exists():
        return {}
    with summary.open() as handle:
        return {row["clip"]: row["status"] for row in csv.DictReader(handle)}


def write_report(rows: list[dict[str, Any]], out: Path) -> None:
    by_key: dict[tuple[str, str, str], list[float]] = defaultdict(list)
    for row in rows:
        if row["value"] == row["value"]:
            by_key[(row["region"], row["metric"], row["source"])].append(float(row["value"]))
    sources = sorted({row["source"] for row in rows}, key=lambda s: (s != "render", s))
    metrics = sorted({row["metric"] for row in rows})
    lines = [
        "# Cage render grade",
        "",
        f"{len({r['clip'] for r in rows})} target clips. "
        "Mean over clips; lower is better except ssim_gray.",
        "",
    ]
    for region in REGIONS:
        lines += [
            f"## {region}",
            "",
            "| metric | " + " | ".join(sources) + " |",
            "|---|" + "---|" * len(sources),
        ]
        for metric in metrics:
            cells = []
            for source in sources:
                values = by_key.get((region, metric, source))
                cells.append(f"{np.mean(values):.3f}" if values else "")
            if any(cells):
                lines.append(f"| {metric} | " + " | ".join(cells) + " |")
        lines.append("")
    lines += [
        "## Per clip, render, inside hull",
        "",
        "| clip | l1_gray | ssim_gray | edge_chamfer_px | mat_iou |",
        "|---|---|---|---|---|",
    ]
    per_clip: dict[str, dict[str, float]] = defaultdict(dict)
    nan = float("nan")
    for row in rows:
        if row["source"] == "render" and row["region"] in ("inside", "full"):
            if row["metric"] == "mat_iou" or row["region"] == "inside":
                per_clip[row["clip"]][row["metric"]] = float(row["value"])
    for clip, values in sorted(per_clip.items()):
        lines.append(
            f"| {clip} | {values.get('l1_gray', nan):.2f} | {values.get('ssim_gray', nan):.3f} | "
            f"{values.get('edge_chamfer_px', nan):.2f} | {values.get('mat_iou', nan):.3f} |"
        )
    (out / "report.md").write_text("\n".join(lines) + "\n")


def compare(dirs: list[Path]) -> None:
    tables = []
    for directory in dirs:
        with (directory / "metrics.csv").open() as handle:
            rows = [r for r in csv.DictReader(handle) if r["source"] == "render"]
        agg: dict[tuple[str, str], list[float]] = defaultdict(list)
        for row in rows:
            if row["value"] == row["value"] and row["value"] != "nan":
                agg[(row["region"], row["metric"])].append(float(row["value"]))
        tables.append({k: float(np.mean(v)) for k, v in agg.items()})
    keys = sorted(set().union(*tables))
    print("| region | metric | " + " | ".join(d.name for d in dirs) + " | delta (last - first) |")
    print("|---|---|" + "---|" * (len(dirs) + 1))
    for region, metric in keys:
        values = [t.get((region, metric), float("nan")) for t in tables]
        print(
            f"| {region} | {metric} | "
            + " | ".join(f"{v:.3f}" for v in values)
            + f" | {values[-1] - values[0]:+.3f} |"
        )


def robot_regions(
    pose: dict[str, Any], shape: tuple[int, int], hull: np.ndarray
) -> dict[str, np.ndarray]:
    """Robot box (dilated a little), a shadow ring around it on the mat, and the rest of the mat."""
    x0, y0, x1, y1 = [int(round(v)) for v in pose["box_rect"]]
    height, width = shape
    box = np.zeros(shape, bool)
    box[max(0, y0 - 8) : min(height, y1 + 8), max(0, x0 - 8) : min(width, x1 + 8)] = True
    ring = np.zeros(shape, bool)
    pad = int(0.6 * max(x1 - x0, y1 - y0)) + 20
    ring[max(0, y0 - pad) : min(height, y1 + pad), max(0, x0 - pad) : min(width, x1 + pad)] = True
    ring &= ~box
    ring &= hull > 0
    mat_rest = (hull > 0) & ~ring & ~box
    return {"robot": box, "shadow_ring": ring, "mat_rest": mat_rest}


def grade_robot_frames(args: argparse.Namespace) -> None:
    """Score renders of MRS BUFF at recovered poses against the real frames they mimic."""
    args.out.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    for pose_json in sorted(args.robot_frames.glob("*/pose.json")):
        name = pose_json.parent.name
        render_path = args.renders / f"{name}.png"
        if not render_path.exists():
            continue
        pose = json.loads(pose_json.read_text())
        target = cv2.imread(str(pose_json.parent / "frame.png"))
        render = cv2.imread(str(render_path))
        hull = cv2.imread(str(args.targets / pose["clip"] / "hull_mask.png"), cv2.IMREAD_GRAYSCALE)
        regions = robot_regions(pose, target.shape[:2], hull)
        robot_mask_path = render_path.with_name(render_path.stem + "_robotmask.png")
        robot_mask = (
            cv2.imread(str(robot_mask_path), cv2.IMREAD_GRAYSCALE)
            if robot_mask_path.exists()
            else None
        )
        flat = np.empty_like(target)
        flat[:] = target[hull > 0].reshape(-1, 3).mean(axis=0).astype(np.uint8)
        for source, image in {"render": render, "baseline_flat": flat}.items():
            for (region, metric), value in image_metrics(
                target, image, regions, args.scale
            ).items():
                rows.append(
                    {
                        "clip": name,
                        "render": name,
                        "region": region,
                        "metric": metric,
                        "source": source,
                        "value": value,
                    }
                )
        # Shadow contrast: how much darker the ring is than the rest of the mat, target vs render.
        t_gray = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY).astype(float)
        r_gray = cv2.cvtColor(render, cv2.COLOR_BGR2GRAY).astype(float)
        for source, gray in (("render", r_gray), ("target", t_gray)):
            rest, ring = gray[regions["mat_rest"]].mean(), gray[regions["shadow_ring"]].mean()
            # Drop in grey levels, and the exposure-free ratio of ring to the rest of the mat.
            for metric, value in (
                ("shadow_drop_gray", rest - ring),
                ("shadow_ratio", ring / max(rest, 1.0)),
            ):
                rows.append(
                    {
                        "clip": name,
                        "render": name,
                        "region": "shadow_ring",
                        "metric": metric,
                        "source": source,
                        "value": float(value),
                    }
                )
        if robot_mask is not None:
            # Robot footprint overlap: is the rendered robot where the detector saw the real one?
            box = regions["robot"]
            inter = ((robot_mask > 0) & box).sum()
            union = ((robot_mask > 0) | box).sum()
            rows.append(
                {
                    "clip": name,
                    "render": name,
                    "region": "robot",
                    "metric": "robot_box_iou",
                    "source": "render",
                    "value": float(inter / union) if union else float("nan"),
                }
            )
            rows.append(
                {
                    "clip": name,
                    "render": name,
                    "region": "robot",
                    "metric": "robot_gray_render",
                    "source": "render",
                    "value": float(r_gray[robot_mask > 0].mean())
                    if (robot_mask > 0).any()
                    else float("nan"),
                }
            )
            rows.append(
                {
                    "clip": name,
                    "render": name,
                    "region": "robot",
                    "metric": "robot_gray_target",
                    "source": "target",
                    "value": float(t_gray[box].mean()),
                }
            )
        x0, y0, x1, y1 = [int(round(v)) for v in pose["box_rect"]]
        pad = int(0.6 * max(x1 - x0, y1 - y0)) + 40
        crop = (
            slice(max(0, y0 - pad), min(target.shape[0], y1 + pad)),
            slice(max(0, x0 - pad), min(target.shape[1], x1 + pad)),
        )
        t_crop, r_crop = target[crop], render[crop]
        scale = 320 / max(1, t_crop.shape[0])
        size = (int(t_crop.shape[1] * scale), 320)
        sheet = np.hstack([cv2.resize(t_crop, size), cv2.resize(r_crop, size)])
        cv2.imwrite(str(args.out / f"{name}_robot_sheet.png"), sheet)
    if not rows:
        raise SystemExit("no robot frame renders matched")
    with (args.out / "metrics.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=["clip", "render", "region", "metric", "source", "value"]
        )
        writer.writeheader()
        writer.writerows(rows)
    agg: dict[tuple[str, str, str], list[float]] = defaultdict(list)
    for row in rows:
        if row["value"] == row["value"]:
            agg[(row["region"], row["metric"], row["source"])].append(float(row["value"]))
    lines = [
        "# Robot frame grade",
        "",
        f"{len({r['clip'] for r in rows})} frames",
        "",
        "| region | metric | render | flat | target |",
        "|---|---|---|---|---|",
    ]
    for region, metric in sorted({(k[0], k[1]) for k in agg}):
        cells = [
            f"{np.mean(agg[(region, metric, s)]):.3f}" if agg.get((region, metric, s)) else ""
            for s in ("render", "baseline_flat", "target")
        ]
        lines.append(f"| {region} | {metric} | " + " | ".join(cells) + " |")
    (args.out / "report.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--renders", type=Path)
    parser.add_argument("--targets", type=Path, default=Path("runs/cage_scene/targets"))
    parser.add_argument("--out", type=Path)
    parser.add_argument("--baseline-render", type=Path, default=None)
    parser.add_argument("--scale", type=float, default=0.25)
    parser.add_argument("--include-failed-poses", action="store_true")
    parser.add_argument("--compare", type=Path, nargs="+", default=None, metavar="GRADE_DIR")
    parser.add_argument(
        "--robot-frames", type=Path, default=None, help="grade robot-frame renders instead"
    )
    args = parser.parse_args()
    if args.compare:
        compare(args.compare)
        return
    if args.robot_frames is not None:
        if args.renders is None or args.out is None:
            parser.error("--renders and --out are required with --robot-frames")
        grade_robot_frames(args)
        return
    if args.renders is None or args.out is None:
        parser.error("--renders and --out are required unless --compare is given")

    statuses = load_statuses(args.targets)
    args.out.mkdir(parents=True, exist_ok=True)
    target_dirs = sorted(p for p in args.targets.iterdir() if (p / "target.png").exists())
    all_targets = {p.name: cv2.imread(str(p / "target.png")) for p in target_dirs}
    mean_target = np.mean(np.stack(list(all_targets.values())).astype(np.float64), axis=0).astype(
        np.uint8
    )

    rows: list[dict[str, Any]] = []
    for render_path in sorted(args.renders.glob("*.png")):
        if render_path.stem.endswith("_matmask"):
            continue
        render = cv2.imread(str(render_path))
        mask_path = render_path.with_name(render_path.stem + "_matmask.png")
        render_mat = (
            cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE) if mask_path.exists() else None
        )
        for target_dir in targets_for_render(render_path.stem, args.targets):
            if not args.include_failed_poses and statuses.get(target_dir.name, "ok") != "ok":
                continue
            target = all_targets[target_dir.name]
            hull = cv2.imread(str(target_dir / "hull_mask.png"), cv2.IMREAD_GRAYSCALE)
            regions = region_masks(hull)
            flat = np.empty_like(target)
            flat[:] = target[hull > 0].reshape(-1, 3).mean(axis=0).astype(np.uint8)
            candidates = {"render": render, "baseline_mean": mean_target, "baseline_flat": flat}
            if (
                args.baseline_render is not None
                and (args.baseline_render / render_path.name).exists()
            ):
                candidates["baseline_previous"] = cv2.imread(
                    str(args.baseline_render / render_path.name)
                )
            for source, image in candidates.items():
                for (region, metric), value in image_metrics(
                    target, image, regions, args.scale
                ).items():
                    rows.append(
                        {
                            "clip": target_dir.name,
                            "render": render_path.stem,
                            "region": region,
                            "metric": metric,
                            "source": source,
                            "value": value,
                        }
                    )
            rows.append(
                {
                    "clip": target_dir.name,
                    "render": render_path.stem,
                    "region": "full",
                    "metric": "mat_iou",
                    "source": "render",
                    "value": mat_mask_iou(hull, render_mat),
                }
            )
            sheet = contact_sheet(
                target, render, hull, f"{render_path.stem} vs {target_dir.name}"[:110]
            )
            cv2.imwrite(str(args.out / f"{target_dir.name}_sheet.png"), sheet)

    if not rows:
        raise SystemExit(f"no render/target pairs between {args.renders} and {args.targets}")
    with (args.out / "metrics.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=["clip", "render", "region", "metric", "source", "value"]
        )
        writer.writeheader()
        writer.writerows(rows)
    write_report(rows, args.out)
    print((args.out / "report.md").read_text())


if __name__ == "__main__":
    main()
