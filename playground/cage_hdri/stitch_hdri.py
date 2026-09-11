"""Stitch posed walk-around frames into an equirectangular HDRI.

Stage 2 of the cage HDRI pipeline. Reads the frames and poses written by
``extract_svo_walk.py`` and writes:

    cage_walk.exr           linear RGB float32 equirectangular, the deliverable
    cage_walk_preview.png   tone-mapped preview at the same size
    coverage.png            winning priority (grey) with clipped winners in red
    gains.csv               solved per-frame exposure gains, one row per frame
    stitch_report.md        counts, coverage, the highlight boost

How it composes a moving camera into one viewpoint:

- Rotation only. Each frame is mapped onto the sphere by its IMU-fused orientation
  and nothing else. Within a frame that mapping is exact, so every region of the
  output is as sharp as its source. Reprojecting through per-pixel depth was tried
  first and scatters once the camera is metres from the centre.
- One winner per pixel. Frames disagree by their parallax, and averaging them blurs
  everything. Each output pixel takes the single highest-priority observation, and
  the disagreement becomes seams instead of blur. Priority is a hat over the source
  image times exp(-distance / --center-sigma) from the virtual centre, so frames shot
  near the centre define the geometry and far frames only fill what those missed.
- Solved exposure. The ZED auto-exposes and the SVO records no exposure value, so
  the relative gain of every frame is solved per channel from its overlap with
  neighbours in the sequence. That also absorbs white-balance drift.
- Highlights. An unclipped observation outranks a clipped one, so a darker exposure
  of an LED tube wins when there is one. Pixels whose winner is still clipped are
  multiplied by --clipped-boost, a prior rather than a measurement.

Pick the centre with --center-frame: the camera position at that SVO frame becomes
the viewpoint. The default is the walk centroid, which the camera never visited.

Coordinate frame is the one stage 1 wrote: x right, y down, z forward, gravity along
+y. The equirect centre column looks along +z.

Usage:
    python stitch_hdri.py runs/cage_hdri/walk runs/cage_hdri/hdri --center-frame 3600
    python stitch_hdri.py WALK OUT --width 4096 --every 2 --frames 0 90
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import deque
from pathlib import Path

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")

import cv2  # noqa: E402
import numpy as np  # noqa: E402

CLIP_LEVEL = 0.98
DARK_LEVEL = 0.02
GAIN_RES = (512, 256)
GAIN_WINDOW = 8
GAIN_MIN_OVERLAP_PX = 400
# Middle grey target for the median covered luminance, so Blender strength 1.0 is
# a sensible exposure.
MIDDLE_GREY = 0.18
LUMA = np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
PREVIEW_EXPOSURE = 1.0
DEFAULT_CLIPPED_BOOST = 10.0
DEFAULT_CENTER_SIGMA_M = 0.75
# Priority share kept by a clipped observation, so it still fills a pixel nothing
# else saw but loses to any unclipped one.
CLIPPED_PRIORITY = 0.3
# Observation priority above which push-pull trusts the pixel outright.
FILL_TRUST_WEIGHT = 1e-3
REMAP_COLS = 16384


def srgb_to_linear(bgr_u8: np.ndarray) -> np.ndarray:
    """8-bit BGR to linear RGB float32 in [0, 1]."""
    rgb = bgr_u8[:, :, ::-1].astype(np.float32) / 255.0
    return np.where(rgb <= 0.04045, rgb / 12.92, ((rgb + 0.055) / 1.055) ** 2.4)


def linear_to_srgb(rgb: np.ndarray) -> np.ndarray:
    rgb = np.clip(rgb, 0.0, 1.0)
    return np.where(rgb <= 0.0031308, rgb * 12.92, 1.055 * rgb ** (1 / 2.4) - 0.055)


def load_walk(walk_dir: Path) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    data = np.load(walk_dir / "poses.npz")
    poses = {key: data[key] for key in ("frame_index", "world_T_cam", "confidence")}
    intrinsics = {
        key.removeprefix("intrinsics_"): float(data[key])
        for key in data.files
        if key.startswith("intrinsics_")
    }
    return poses, intrinsics


def pano_directions(width: int, height: int) -> np.ndarray:
    """Unit world direction of every equirect pixel, (H*W, 3). Up is -y, centre column +z."""
    lon = (np.arange(width, dtype=np.float32) + 0.5) / width * (2.0 * np.pi) - np.pi
    lat = np.pi / 2.0 - (np.arange(height, dtype=np.float32) + 0.5) / height * np.pi
    cos_lat = np.cos(lat)[:, None]
    dx = np.sin(lon)[None, :] * cos_lat
    dz = np.cos(lon)[None, :] * cos_lat
    dy = np.broadcast_to(-np.sin(lat)[:, None], dx.shape)
    return np.stack([dx, dy, dz], axis=-1).reshape(-1, 3).astype(np.float32)


def project(
    directions: np.ndarray, world_t_cam: np.ndarray, intrinsics: dict[str, float]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Pixel coordinates of every pano direction that lands inside the frame.

    Returns (pano_index, u, v). Rotation only: the frame is treated as seen from the
    virtual centre, which is exact for the frame's own content.
    """
    rotation = world_t_cam[:3, :3].astype(np.float32)
    cam = directions @ rotation  # rotation.T applied to each row
    in_front = cam[:, 2] > 1e-3
    index = np.flatnonzero(in_front)
    cam = cam[index]
    u = intrinsics["fx"] * cam[:, 0] / cam[:, 2] + intrinsics["cx"] - 0.5
    v = intrinsics["fy"] * cam[:, 1] / cam[:, 2] + intrinsics["cy"] - 0.5
    inside = (u >= 0) & (u <= intrinsics["width"] - 1) & (v >= 0) & (v <= intrinsics["height"] - 1)
    return index[inside], u[inside].astype(np.float32), v[inside].astype(np.float32)


def sample(image: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Bilinear samples of a (H, W, C) float image at continuous pixel coordinates."""
    # remap caps every map dimension below 32767, so the sample list is folded into rows.
    count = len(u)
    rows = -(-count // REMAP_COLS)
    padded = rows * REMAP_COLS
    map_u = np.zeros(padded, dtype=np.float32)
    map_v = np.zeros(padded, dtype=np.float32)
    map_u[:count] = u
    map_v[:count] = v
    samples = cv2.remap(
        image, map_u.reshape(rows, REMAP_COLS), map_v.reshape(rows, REMAP_COLS), cv2.INTER_LINEAR
    )
    return samples.reshape(padded, -1)[:count]


def hat(u: np.ndarray, v: np.ndarray, intrinsics: dict[str, float]) -> np.ndarray:
    """Separable hat over the source image so seams fall near frame edges."""
    wx = 1.0 - np.abs(2.0 * u / (intrinsics["width"] - 1) - 1.0)
    wy = 1.0 - np.abs(2.0 * v / (intrinsics["height"] - 1) - 1.0)
    return np.clip(wx * wy, 0.02, 1.0).astype(np.float32)


def solve_gains(frames: list[tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    """Least-squares log gains from pairwise overlap ratios within a sliding window.

    ``frames`` holds (low-res linear RGB (N, 3), usable mask (N,)) per frame. Returns
    gains (F, 3) normalised so the mean log gain is zero per channel.
    """
    count = len(frames)
    rows: list[tuple[int, int, np.ndarray, float]] = []
    window: deque[tuple[int, np.ndarray, np.ndarray]] = deque(maxlen=GAIN_WINDOW)
    for position, (rgb, usable) in enumerate(frames):
        for other_position, other_rgb, other_usable in window:
            overlap = usable & other_usable
            overlap_count = int(overlap.sum())
            if overlap_count < GAIN_MIN_OVERLAP_PX:
                continue
            ratio = np.median(np.log(rgb[overlap]) - np.log(other_rgb[overlap]), axis=0)
            rows.append((position, other_position, ratio, float(np.sqrt(overlap_count))))
        window.append((position, rgb, usable))

    gains = np.ones((count, 3), dtype=np.float64)
    if not rows:
        return gains
    system = np.zeros((len(rows) + 1, count), dtype=np.float64)
    targets = np.zeros((len(rows) + 1, 3), dtype=np.float64)
    for row, (i, j, ratio, weight) in enumerate(rows):
        system[row, i] = weight
        system[row, j] = -weight
        targets[row] = weight * ratio
    # Anchor: mean log gain is zero, so the solve fixes only relative exposure.
    system[-1, :] = 1.0
    log_gains, *_ = np.linalg.lstsq(system, targets, rcond=None)
    return np.exp(log_gains - log_gains.mean(axis=0, keepdims=True))


def push_pull_fill(value_sum: np.ndarray, weight_sum: np.ndarray) -> np.ndarray:
    """Fill unobserved pixels from progressively coarser averages of observed ones."""
    height, width = weight_sum.shape
    levels_value = [value_sum]
    levels_weight = [weight_sum]
    while min(levels_weight[-1].shape) > 4:
        levels_value.append(
            cv2.resize(levels_value[-1], None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        )
        levels_weight.append(
            cv2.resize(levels_weight[-1], None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        )

    coarse_value = levels_value[-1]
    coarse_weight = levels_weight[-1]
    filled = coarse_value / np.maximum(coarse_weight, 1e-12)[:, :, None]
    global_mean = (coarse_value.sum(axis=(0, 1)) / max(float(coarse_weight.sum()), 1e-12)).astype(
        np.float32
    )
    filled = np.where(coarse_weight[:, :, None] > 1e-9, filled, global_mean)
    for value, weight in zip(levels_value[-2::-1], levels_weight[-2::-1]):
        upsampled = cv2.resize(
            filled, (weight.shape[1], weight.shape[0]), interpolation=cv2.INTER_LINEAR
        )
        observed = value / np.maximum(weight, 1e-12)[:, :, None]
        # Only pixels nobody observed lean on the coarser level.
        alpha = np.clip(weight / (weight + FILL_TRUST_WEIGHT), 0.0, 1.0)[:, :, None]
        filled = alpha * observed + (1.0 - alpha) * upsampled
    return filled.reshape(height, width, 3).astype(np.float32)


def tone_map(rgb: np.ndarray) -> np.ndarray:
    exposed = rgb * PREVIEW_EXPOSURE
    reinhard = exposed / (1.0 + exposed)
    return (linear_to_srgb(reinhard)[:, :, ::-1] * 255.0).astype(np.uint8)


def stitch(args: argparse.Namespace) -> int:
    walk_dir: Path = args.walk_dir
    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    width = args.width
    height = width // 2

    poses, intrinsics = load_walk(walk_dir)
    keep = poses["confidence"] >= args.min_confidence
    if args.frames is not None:
        first, last = args.frames
        keep &= (poses["frame_index"] >= first) & (poses["frame_index"] <= last)
    selected = np.flatnonzero(keep)[:: args.every]
    if len(selected) == 0:
        print("no frames pass the confidence gate")
        return 1
    positions = poses["world_T_cam"][:, :3, 3]
    if args.center_frame is None:
        center = positions[selected].mean(axis=0)
        center_source = "walk centroid"
    else:
        nearest = int(np.argmin(np.abs(poses["frame_index"] - args.center_frame)))
        center = positions[nearest]
        center_source = f"camera at frame {int(poses['frame_index'][nearest])}"
    center_text = f"({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})"
    span_text = f"{np.ptp(positions[selected], axis=0).round(2).tolist()} m"
    distances = np.linalg.norm(positions - center, axis=1)
    print(f"{len(selected)} of {len(keep)} frames, centre {center_text} ({center_source})")

    def load_frame(frame_position: int) -> np.ndarray | None:
        frame_index = int(poses["frame_index"][frame_position])
        bgr = cv2.imread(str(walk_dir / "frames" / f"{frame_index:06d}.png"), cv2.IMREAD_COLOR)
        if bgr is None:
            print(f"  missing frame {frame_index}, skipping")
            return None
        return srgb_to_linear(bgr)

    # Pass 1: low-res samples for the gain solve.
    low_directions = pano_directions(*GAIN_RES)
    low_frames: list[tuple[np.ndarray, np.ndarray]] = []
    frame_positions: list[int] = []
    for count, frame_position in enumerate(selected):
        linear = load_frame(int(frame_position))
        if linear is None:
            continue
        index, u, v = project(low_directions, poses["world_T_cam"][frame_position], intrinsics)
        rgb = np.zeros((low_directions.shape[0], 3), dtype=np.float32)
        usable = np.zeros(low_directions.shape[0], dtype=bool)
        samples = sample(linear, u, v)
        rgb[index] = samples
        usable[index] = (samples.max(axis=1) < CLIP_LEVEL) & (samples.min(axis=1) > DARK_LEVEL)
        low_frames.append((rgb, usable))
        frame_positions.append(int(frame_position))
        if count % 200 == 0:
            print(f"  gain pass {count}/{len(selected)}")
    gains = solve_gains(low_frames)
    del low_frames
    gain_low = gains.min(axis=0).round(3).tolist()
    gain_high = gains.max(axis=0).round(3).tolist()
    print(f"gains: min {gain_low} max {gain_high}")

    # Pass 2: full-res winner-take-all composite in radiance units.
    directions = pano_directions(width, height)
    best_priority = np.zeros(height * width, dtype=np.float32)
    best_value = np.zeros((height * width, 3), dtype=np.float32)
    best_clipped = np.zeros(height * width, dtype=bool)
    for count, (frame_position, gain) in enumerate(zip(frame_positions, gains)):
        linear = load_frame(frame_position)
        if linear is None:
            continue
        index, u, v = project(directions, poses["world_T_cam"][frame_position], intrinsics)
        samples = sample(linear, u, v)
        clipped = samples.max(axis=1) >= CLIP_LEVEL
        confidence = float(poses["confidence"][frame_position]) / 100.0
        proximity = float(np.exp(-distances[frame_position] / args.center_sigma))
        priority = hat(u, v, intrinsics) * (confidence * proximity)
        priority = np.where(clipped, priority * CLIPPED_PRIORITY, priority)
        wins = priority > best_priority[index]
        target = index[wins]
        best_priority[target] = priority[wins]
        best_value[target] = samples[wins] / gain.astype(np.float32)
        best_clipped[target] = clipped[wins]
        if count % 200 == 0:
            print(f"  composite {count}/{len(frame_positions)}")
    del directions

    observed = best_priority > 0
    clipped_winner = observed & best_clipped
    best_value[clipped_winner] *= float(args.clipped_boost)
    weight = best_priority.astype(np.float64)
    filled = push_pull_fill(
        (best_value.astype(np.float64) * weight[:, None]).reshape(height, width, 3),
        weight.reshape(height, width),
    )

    # Normalise so the median covered luminance is middle grey.
    luminance = filled.reshape(-1, 3) @ LUMA
    median = float(np.median(luminance[observed]))
    scale = MIDDLE_GREY / max(median, 1e-9)
    filled *= scale

    cv2.imwrite(str(output_dir / "cage_walk.exr"), filled[:, :, ::-1].astype(np.float32))
    cv2.imwrite(str(output_dir / "cage_walk_preview.png"), tone_map(filled))

    coverage = np.clip(best_priority / np.percentile(best_priority[observed], 95), 0.0, 1.0)
    coverage_bgr = cv2.cvtColor(
        (coverage.reshape(height, width) * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR
    )
    coverage_bgr[clipped_winner.reshape(height, width)] = (0, 0, 255)
    cv2.imwrite(str(output_dir / "coverage.png"), coverage_bgr)

    with (output_dir / "gains.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "distance_m", "gain_r", "gain_g", "gain_b"])
        for frame_position, gain in zip(frame_positions, gains):
            writer.writerow(
                [
                    int(poses["frame_index"][frame_position]),
                    f"{distances[frame_position]:.3f}",
                    *(f"{g:.4f}" for g in gain),
                ]
            )

    near = int((distances[frame_positions] < args.center_sigma).sum())
    report = "\n".join(
        [
            "# Cage walk HDRI stitch",
            "",
            f"- source: `{walk_dir}`, {len(frame_positions)} frames used of {len(keep)} "
            "kept by stage 1",
            f"- gate: confidence >= {args.min_confidence}, every {args.every}, "
            f"frames {args.frames or 'all'}",
            f"- output: {width}x{height} equirect, centre {center_text} m ({center_source}), "
            f"{near} frames within {args.center_sigma} m of it",
            f"- walk span: {span_text}",
            f"- coverage: {100.0 * observed.mean():.1f}% of the sphere observed, the rest "
            "push-pull filled",
            f"- clipped winners: {100.0 * clipped_winner.mean():.2f}% of the sphere, "
            f"boosted x{args.clipped_boost:g} (--clipped-boost, a prior)",
            f"- gains: {gains.min():.3f} to {gains.max():.3f} (mean log gain 0)",
            f"- normalisation: median covered luminance {median:.4f} scaled x{scale:.2f} "
            f"to {MIDDLE_GREY}",
            f"- peak radiance after boost and scale: {float(filled.max()):.2f}",
            "",
            "Inspect `coverage.png` before trusting a region: grey is the winning priority,",
            "red is a clipped winner, black was never seen and is filled.",
            "",
        ]
    )
    (output_dir / "stitch_report.md").write_text(report)
    print(report)
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("walk_dir", type=Path, help="Output of extract_svo_walk.py")
    parser.add_argument("output_dir", type=Path, help="Where to write the HDRI and reports")
    parser.add_argument("--width", type=int, default=8192, help="Equirect width (default 8192)")
    parser.add_argument(
        "--min-confidence",
        type=float,
        default=50.0,
        help="Drop frames below this tracking confidence (default 50)",
    )
    parser.add_argument("--every", type=int, default=1, help="Use every Nth kept frame")
    parser.add_argument(
        "--frames",
        type=int,
        nargs=2,
        default=None,
        metavar=("FIRST", "LAST"),
        help="Only use SVO frame indices in this inclusive range (diagnostics)",
    )
    parser.add_argument(
        "--center-frame",
        type=int,
        default=None,
        help="SVO frame whose camera position becomes the viewpoint (default: walk centroid)",
    )
    parser.add_argument(
        "--center-sigma",
        type=float,
        default=DEFAULT_CENTER_SIGMA_M,
        help=f"Metres over which frame priority decays with distance from the centre "
        f"(default {DEFAULT_CENTER_SIGMA_M})",
    )
    parser.add_argument(
        "--clipped-boost",
        type=float,
        default=DEFAULT_CLIPPED_BOOST,
        help=f"Radiance multiplier for pixels whose winner was clipped "
        f"(default {DEFAULT_CLIPPED_BOOST:g})",
    )
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(stitch(parse_args()))
