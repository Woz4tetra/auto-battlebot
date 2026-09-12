#!/usr/bin/env python3
"""Orthographic mat albedo from the rectified cage-high targets.

Each target is warped onto the mat plane through its fitted pose (the inverse of the
homography K [r1 r2 t]) into a square texture over the mat, then the clips of one event are
combined by a per-pixel median after normalising each clip to its own mat-region mean, so
exposure drift between fights does not vote. The texture is in the hfield frame: pixel x
runs along hfield +x, pixel y along hfield +y, origin at (-half, -half).

Outputs `<out>/cage2_<event>_albedo.png` with a `.json` sidecar (px per metre, extent,
clips used) and a downscaled `_preview.png`. The default `--out` is the cage environment
directory the Blender spec reads the albedo from.

Usage:
    venv/bin/python training/synthetic/build_cage_floor_texture.py runs/cage_scene --px-per-m 1024
    venv/bin/python training/synthetic/build_cage_floor_texture.py runs/cage_scene \
        --source target_4k.png
"""

from __future__ import annotations

import argparse
import csv
import json
import warnings
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.perception.cage_calibration import load_cage_calibration

DEFAULT_OUT = Path(__file__).resolve().parents[1] / "data/environments/nhrl_3lb_cage/mat_albedo"


def texture_from_hfield(px_per_m: float, half_xy: tuple[float, float]) -> np.ndarray:
    """3x3 map from hfield (x, y) metres on the mat plane to texture pixels."""
    return np.array(
        [
            [px_per_m, 0.0, half_xy[0] * px_per_m],
            [0.0, px_per_m, half_xy[1] * px_per_m],
            [0.0, 0.0, 1.0],
        ]
    )


def image_from_hfield_plane(tf_cam_from_hfield: np.ndarray, k: np.ndarray) -> np.ndarray:
    """3x3 map from hfield (x, y, 1) on the z = 0 plane to image pixels."""
    rt = np.column_stack(
        [tf_cam_from_hfield[:3, 0], tf_cam_from_hfield[:3, 1], tf_cam_from_hfield[:3, 3]]
    )
    return np.asarray(k @ rt)


def warp_to_texture(
    image: np.ndarray,
    tf_cam_from_hfield: np.ndarray,
    k: np.ndarray,
    px_per_m: float,
    half_xy: tuple[float, float],
) -> np.ndarray:
    size = (int(round(2 * half_xy[0] * px_per_m)), int(round(2 * half_xy[1] * px_per_m)))
    h_img_from_tex = image_from_hfield_plane(tf_cam_from_hfield, k) @ np.linalg.inv(
        texture_from_hfield(px_per_m, half_xy)
    )
    return cv2.warpPerspective(
        image,
        h_img_from_tex,
        size,
        flags=cv2.INTER_LANCZOS4 | cv2.WARP_INVERSE_MAP,
        borderMode=cv2.BORDER_CONSTANT,
    )


def flatten_shading(texture: np.ndarray, sigma_px: float) -> np.ndarray:
    """Divide out low-frequency luminance so the light gradient is not baked into the albedo."""
    lab = cv2.cvtColor(texture, cv2.COLOR_BGR2Lab).astype(np.float32)
    lum = lab[:, :, 0]
    low = cv2.GaussianBlur(lum, (0, 0), sigma_px)
    lab[:, :, 0] = np.clip(lum / np.maximum(low, 1.0) * float(lum.mean()), 0, 255)
    return cv2.cvtColor(lab.astype(np.uint8), cv2.COLOR_Lab2BGR)


@dataclass
class WarpedClip:
    """One clip's target frame warped onto the mat plane, with the hull it actually saw."""

    name: str
    texture: np.ndarray
    mask: np.ndarray


def pose_statuses(scene_dir: Path) -> dict[str, str]:
    """Clip -> pose-fit status from the fit summary, so failed fits stay out of the median."""
    summary = scene_dir / "poses" / "summary.csv"
    if not summary.exists():
        return {}
    with summary.open() as handle:
        return {row["clip"]: row["status"] for row in csv.DictReader(handle)}


def warp_targets(
    scene_dir: Path, source: str, px_per_m: float, k_1080: np.ndarray
) -> tuple[dict[str, list[WarpedClip]], tuple[float, float] | None, str | None]:
    """Warp every target with a good pose onto the mat, grouped by event.

    Returns:
        ``(clips per event, mat half extent, cage name)``; the latter two come from the
        calibrations and metadata and are None when nothing warped.
    """
    per_event: dict[str, list[WarpedClip]] = {}
    half_xy: tuple[float, float] | None = None
    cage_name: str | None = None
    statuses = pose_statuses(scene_dir)
    for target_dir in sorted((scene_dir / "targets").iterdir()):
        target = target_dir / source
        pose_path = scene_dir / "poses" / f"{target_dir.name}.toml"
        if not target.exists() or not pose_path.exists():
            continue
        if statuses.get(target_dir.name, "ok") != "ok":
            print(f"skip {target_dir.name}: pose fit {statuses[target_dir.name]}")
            continue
        meta = json.loads((target_dir / "meta.json").read_text())
        if source != "target.png" and "k_rect_4k" not in meta:
            print(f"skip {target_dir.name}: no 4K K in meta.json")
            continue
        k = k_1080 if source == "target.png" else np.array(meta["k_rect_4k"])
        calibration = load_cage_calibration(pose_path)
        tf = calibration.tf_camera_from_fieldcenter
        half_xy = (calibration.field_size_x / 2, calibration.field_size_y / 2)
        cage_name = cage_name or f"cage{meta.get('cage')}"
        image = cv2.imread(str(target))
        hull = (
            cv2.imread(str(target_dir / "hull_mask.png"), cv2.IMREAD_GRAYSCALE)
            if source == "target.png"
            else np.full(image.shape[:2], 255, np.uint8)
        )
        texture = warp_to_texture(image, tf, k, px_per_m, half_xy)
        mask = warp_to_texture(hull, tf, k, px_per_m, half_xy)
        event = str(meta.get("event_group") or meta.get("event", "unknown"))
        per_event.setdefault(event, []).append(WarpedClip(target_dir.name, texture, mask))
        print(f"warped {target_dir.name} -> {texture.shape[1]}x{texture.shape[0]}")
    return per_event, half_xy, cage_name


def combine_clips(clips: list[WarpedClip]) -> tuple[np.ndarray, float]:
    """Median the clips of one event into an albedo; returns it and the mat fraction seen."""
    stack = []
    for clip in clips:
        valid = clip.mask > 127
        if not valid.any():
            continue
        mean = clip.texture[valid].reshape(-1, 3).mean(axis=0)
        gain = 128.0 / np.maximum(mean, 1.0)
        scaled = np.clip(clip.texture.astype(np.float32) * gain, 0, 255)
        scaled[~valid] = np.nan  # the frame did not see this part of the mat
        stack.append(scaled)
    with np.errstate(all="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN columns are the unseen strip
        median = np.nanmedian(np.stack(stack), axis=0)
    unseen = np.isnan(median[:, :, 0])
    # Restore the event's mean colour so the albedo carries the real mat tint, and fill what no
    # clip saw (the near strip the camera cuts off) with that mean.
    target_mean = np.mean(
        [c.texture[c.mask > 127].reshape(-1, 3).mean(axis=0) for c in clips], axis=0
    )
    median[unseen] = 128.0
    albedo = np.clip(median * (target_mean / 128.0), 0, 255).astype(np.uint8)
    return albedo, 1.0 - float(unseen.mean())


def inpaint_hfield_box(
    albedo: np.ndarray, box: list[float], px_per_m: float, half_xy: tuple[float, float]
) -> np.ndarray:
    """Inpaint a rectangle given in hfield metres (a parked robot, a cable)."""
    x, y, w, h = box
    tex = texture_from_hfield(px_per_m, half_xy)
    p0 = tex @ np.array([x, y, 1.0])
    p1 = tex @ np.array([x + w, y + h, 1.0])
    hole = np.zeros(albedo.shape[:2], np.uint8)
    cv2.rectangle(hole, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), 255, -1)
    return np.asarray(cv2.inpaint(albedo, hole, 5, cv2.INPAINT_TELEA))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument("--px-per-m", type=float, default=1024.0)
    parser.add_argument("--source", default="target.png", help="target.png or target_4k.png")
    parser.add_argument(
        "--flatten-sigma", type=float, default=0.0, help="px; 0 disables shading removal"
    )
    parser.add_argument(
        "--inpaint-box",
        type=float,
        nargs=4,
        metavar=("X", "Y", "W", "H"),
        help="hfield metres to inpaint",
    )
    parser.add_argument(
        "--name", default=None, help="texture name prefix; default cage<n> from meta"
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT,
        help="where the albedo lands; the cage spec reads it from here",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    k_data = json.loads((args.scene_dir / "camera_rect.json").read_text())
    k_1080 = np.array(
        [[k_data["fx"], 0.0, k_data["cx"]], [0.0, k_data["fy"], k_data["cy"]], [0.0, 0.0, 1.0]]
    )
    textures_dir = args.out
    textures_dir.mkdir(parents=True, exist_ok=True)

    per_event, half_xy, fitted_name = warp_targets(
        args.scene_dir, args.source, args.px_per_m, k_1080
    )
    cage_name = args.name or fitted_name

    if not per_event or half_xy is None:
        raise SystemExit(
            "no targets with poses; run extract_targets.py and fit_cage_camera.py first"
        )

    for event, clips in per_event.items():
        albedo, seen_fraction = combine_clips(clips)
        if args.flatten_sigma > 0:
            albedo = flatten_shading(albedo, args.flatten_sigma)
        if args.inpaint_box is not None:
            albedo = inpaint_hfield_box(albedo, args.inpaint_box, args.px_per_m, half_xy)
        stem = f"{cage_name}_{event}_albedo"
        cv2.imwrite(str(textures_dir / f"{stem}.png"), albedo)
        preview = cv2.resize(albedo, (512, 512), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(textures_dir / f"{cage_name}_{event}_preview.png"), preview)
        (textures_dir / f"{stem}.json").write_text(
            json.dumps(
                {
                    "event": event,
                    "clips": [clip.name for clip in clips],
                    "source": args.source,
                    "px_per_m": args.px_per_m,
                    "half_extent_m": list(half_xy),
                    "frame": "hfield: pixel x along +x, pixel y along +y, origin at (-half, -half)",
                    "flatten_sigma_px": args.flatten_sigma,
                    "seen_fraction": seen_fraction,
                    "width": int(albedo.shape[1]),
                    "height": int(albedo.shape[0]),
                },
                indent=2,
            )
            + "\n"
        )
        print(f"{stem}.png from {len(clips)} clips, {seen_fraction:.1%} of the mat observed")


if __name__ == "__main__":
    main()
