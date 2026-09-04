#!/usr/bin/env python3
"""Background subtraction proof of concept over a recorded MCAP.

The perception stack publishes the field segmentation once per field init, on /field_mask,
stamped in the CAMERA_WORLD frame (the camera pose held at that init). This script rebuilds a
background image in that frame, warps it into every later frame using the recorded camera
motion, subtracts it, and writes a video of the result.

Two ways to line the background up with a moving camera:

  --warp plane   (default) Plane-induced homography from the recorded pose. The field is a
                 plane, so H = K (R + t n^T / d) K^-1 maps CAMERA_WORLD pixels to the current
                 frame exactly, using /tf and /camera/camera_info. No image matching.
  --warp affine  ORB features plus a RANSAC affine fit against the reference frame, applied
                 with cv2.warpAffine. Needs no pose, but an affine cannot express the
                 perspective change, so it leaves more residual along the field seams.

Two background models:

  --background median  (default) Per-pixel median of frames warped back into CAMERA_WORLD.
                       Robots move, so the median is an empty field.
  --background ref     The single frame captured at field init. Whatever sat on the field at
                       that moment leaves a ghost blob in the difference.

Usage:
    python playground/field_background_subtraction.py RECORDING.mcap -o subtracted.mp4
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from auto_battlebot.background_subtraction import (
    Blob,
    SubtractionParams,
    build_median_background,
    find_blobs,
    subtract,
    warp_forward,
)
from auto_battlebot.mcap_io import (
    decode_camera_info,
    decode_compressed_image,
    decode_image_stamp_ns,
    decode_tf_message,
    iter_messages,
)

CAMERA_IMAGE_TOPIC = "/camera/image"
CAMERA_INFO_TOPIC = "/camera/camera_info"
FIELD_MASK_TOPIC = "/field_mask"
TF_TOPIC = "/tf"

# publish_field_mask blends the RGB frame 50/50 with colorize_labels, whose label 1 is pure
# green (0, 255, 0) in BGR. Field pixels therefore land at 0.5 * G + 127.5 and everything else
# at 0.5 * G, so a threshold at 127 splits them cleanly.
MASK_GREEN_THRESHOLD = 127

# Warping leaves a hard edge where the background ran out, and the arena wall never aligns
# because it is not on the field plane. Both get trimmed out of the compared region.
VALID_ERODE_PX = 5


# ---------------------------------------------------------------------------
# Recording access
# ---------------------------------------------------------------------------


def read_frame_stamps(path: Path) -> list[int]:
    """Header stamps of every /camera/image, in log order, without decoding the JPEGs."""
    return [
        decode_image_stamp_ns(data)
        for _topic, _ts, data in iter_messages(path, [CAMERA_IMAGE_TOPIC])
    ]


def read_intrinsics(path: Path) -> np.ndarray:
    for _topic, _ts, data in iter_messages(path, [CAMERA_INFO_TOPIC]):
        return decode_camera_info(data).intrinsics
    raise RuntimeError(f"{path} has no {CAMERA_INFO_TOPIC} messages")


def read_transforms(path: Path) -> dict[tuple[str, str], dict[int, np.ndarray]]:
    """Group /tf into {(parent, child): {stamp_ns: 4x4 parent-from-child}}."""
    transforms: dict[tuple[str, str], dict[int, np.ndarray]] = {}
    for _topic, _ts, data in iter_messages(path, [TF_TOPIC]):
        for transform in decode_tf_message(data):
            transforms.setdefault(transform.key, {})[transform.stamp_ns] = transform.matrix
    return transforms


def decode_frames(path: Path, indices: set[int]) -> dict[int, np.ndarray]:
    """Decode only the listed /camera/image frames, in one pass."""
    frames: dict[int, np.ndarray] = {}
    for index, (_topic, _ts, data) in enumerate(iter_messages(path, [CAMERA_IMAGE_TOPIC])):
        if index in indices:
            frames[index] = decode_compressed_image(data).image
    return frames


class TransformLookup:
    """Nearest-stamp lookup over one parent/child pair of the recorded TF stream."""

    def __init__(self, by_stamp: dict[int, np.ndarray]) -> None:
        self._stamps = np.array(sorted(by_stamp), dtype=np.int64)
        self._by_stamp = by_stamp

    def at(self, stamp_ns: int) -> np.ndarray:
        index = int(np.argmin(np.abs(self._stamps - stamp_ns)))
        return self._by_stamp[int(self._stamps[index])]


def recover_field_mask(overlay: np.ndarray, erode_px: int) -> np.ndarray:
    """Pull the binary field mask back out of the 50/50 green overlay on /field_mask."""
    mask = np.where(overlay[:, :, 1] > MASK_GREEN_THRESHOLD, 255, 0).astype(np.uint8)

    # The field filter runs on the largest contour, so match it: drop stray specks and fill the
    # holes punched by the hazard squares.
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        mask = np.zeros_like(mask)
        cv2.drawContours(mask, [largest], -1, 255, cv2.FILLED)

    if erode_px > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erode_px * 2 + 1, erode_px * 2 + 1))
        mask = cv2.erode(mask, kernel)
    return mask


# ---------------------------------------------------------------------------
# Aligning the background with a moving camera
# ---------------------------------------------------------------------------


class PlaneWarp:
    """Plane-induced homography from the recorded camera pose.

    The field is a plane n . X = d in CAMERA_WORLD, so for a camera at
    camera_from_camera_world = (R, t) every field point maps through
    H = K (R + t n^T / d) K^-1. Exact for the field, wrong everywhere else, which is fine
    because only the field is compared.
    """

    def __init__(
        self,
        intrinsics: np.ndarray,
        poses: TransformLookup,
        normal: np.ndarray,
        distance: float,
    ) -> None:
        self._intrinsics = intrinsics
        self._inverse_intrinsics = np.linalg.inv(intrinsics)
        self._poses = poses
        self._normal = normal
        self._distance = distance

    def estimate(self, _frame_gray: np.ndarray, stamp_ns: int) -> np.ndarray | None:
        camera_from_camera_world = np.linalg.inv(self._poses.at(stamp_ns))
        rotation = camera_from_camera_world[:3, :3]
        translation = camera_from_camera_world[:3, 3]
        homography = (
            self._intrinsics
            @ (rotation + np.outer(translation, self._normal) / self._distance)
            @ self._inverse_intrinsics
        )
        return homography / homography[2, 2]


class AffineWarp:
    """ORB plus RANSAC affine from a fixed reference image to an arbitrary frame."""

    def __init__(
        self, reference_gray: np.ndarray, reference_roi: np.ndarray, features: int
    ) -> None:
        self._orb = cv2.ORB_create(features)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self._keypoints, self._descriptors = self._orb.detectAndCompute(
            reference_gray, reference_roi
        )

    def estimate(self, frame_gray: np.ndarray, _stamp_ns: int) -> np.ndarray | None:
        """Return a 2x3 reference-to-frame affine, or None when the match is too weak."""
        if self._descriptors is None or len(self._keypoints) < 4:
            return None
        keypoints, descriptors = self._orb.detectAndCompute(frame_gray, None)
        if descriptors is None or len(keypoints) < 4:
            return None

        pairs = self._matcher.knnMatch(self._descriptors, descriptors, k=2)
        good = [a for a, b in pairs if a.distance < 0.75 * b.distance]
        if len(good) < 12:
            return None

        source = np.float32([self._keypoints[m.queryIdx].pt for m in good])
        target = np.float32([keypoints[m.trainIdx].pt for m in good])
        affine, inliers = cv2.estimateAffine2D(
            source, target, method=cv2.RANSAC, ransacReprojThreshold=3.0
        )
        if affine is None or inliers is None or int(inliers.sum()) < 12:
            return None
        return affine


# ---------------------------------------------------------------------------
# Background model
# ---------------------------------------------------------------------------


@dataclass
class Segment:
    """One field init: a background in CAMERA_WORLD plus the frames it applies to."""

    index: int
    start_frame: int
    end_frame: int  # exclusive
    mask: np.ndarray  # uint8 0/255, field region in CAMERA_WORLD
    background: np.ndarray  # BGR background in CAMERA_WORLD
    background_valid: np.ndarray  # uint8 0/255, pixels the background actually observed
    warp: PlaneWarp | AffineWarp

    @property
    def compare_mask(self) -> np.ndarray:
        return cv2.bitwise_and(self.mask, self.background_valid)


def segment_bounds(stamps: list[int], mask_stamps: list[int]) -> list[tuple[int, int]]:
    """Frame ranges each field init owns: [its own frame, the next init's frame)."""
    stamp_array = np.array(stamps, dtype=np.int64)
    starts = [int(np.argmin(np.abs(stamp_array - stamp))) for stamp in mask_stamps]
    return list(zip(starts, starts[1:] + [len(stamps)]))


def sample_indices(start: int, end: int, count: int) -> list[int]:
    """Evenly spaced frames from a segment, always including its first frame."""
    stride = max(1, (end - start) // max(1, count))
    return sorted({start, *range(start, end, stride)})


def build_segment(
    args: argparse.Namespace,
    index: int,
    bounds: tuple[int, int],
    mask_overlay: np.ndarray,
    plane: tuple[np.ndarray, float],
    poses: TransformLookup,
    intrinsics: np.ndarray,
    stamps: list[int],
    frames: dict[int, np.ndarray],
) -> Segment:
    start, end = bounds
    mask = recover_field_mask(mask_overlay, args.mask_erode)
    reference = frames[start]
    height, width = reference.shape[:2]
    size = (width, height)

    if args.warp == "plane":
        warp: PlaneWarp | AffineWarp = PlaneWarp(intrinsics, poses, plane[0], plane[1])
    else:
        roi = cv2.dilate(mask, np.ones((41, 41), np.uint8))
        warp = AffineWarp(cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY), roi, args.orb_features)

    if args.background == "ref":
        background = reference
        background_valid = np.full((height, width), 255, np.uint8)
    else:
        samples = []
        transforms = []
        for frame_index in sample_indices(start, end, args.background_samples):
            image = frames[frame_index]
            estimate = warp.estimate(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), stamps[frame_index])
            if estimate is None:
                continue
            samples.append(image)
            transforms.append(estimate)
        print(
            f"segment {index}: frames {start}-{end - 1}, median over {len(samples)} samples",
            flush=True,
        )
        background, background_valid = build_median_background(samples, transforms, size)

    return Segment(
        index=index,
        start_frame=start,
        end_frame=end,
        mask=mask,
        background=background,
        background_valid=background_valid,
        warp=warp,
    )


def build_segments(
    args: argparse.Namespace,
    stamps: list[int],
    intrinsics: np.ndarray,
    transforms: dict[tuple[str, str], dict[int, np.ndarray]],
) -> list[Segment]:
    """One segment per field init, each with its own background and field mask."""
    masks = [
        decode_compressed_image(data)
        for _topic, _ts, data in iter_messages(args.mcap, [FIELD_MASK_TOPIC])
    ]
    if not masks:
        raise RuntimeError(
            f"{args.mcap} has no {FIELD_MASK_TOPIC} messages, so there is no field to subtract"
        )

    field_from_camera_world = TransformLookup(transforms[("field", "camera_world")])
    poses = TransformLookup(transforms[("camera_world", "camera")])
    bounds = segment_bounds(stamps, [mask.stamp_ns for mask in masks])

    wanted = {
        frame
        for index, (start, end) in enumerate(bounds)
        for frame in (
            [start]
            if args.background == "ref"
            else sample_indices(start, end, args.background_samples)
        )
    }
    frames = decode_frames(args.mcap, wanted)

    segments = []
    for index, (mask, segment_range) in enumerate(zip(masks, bounds)):
        # The field plane is z = 0 in the field frame; express it in CAMERA_WORLD.
        camera_world_from_field = np.linalg.inv(field_from_camera_world.at(mask.stamp_ns))
        normal = camera_world_from_field[:3, :3] @ np.array([0.0, 0.0, 1.0])
        distance = float(normal @ camera_world_from_field[:3, 3])
        segments.append(
            build_segment(
                args,
                index,
                segment_range,
                mask.image,
                (normal, distance),
                poses,
                intrinsics,
                stamps,
                frames,
            )
        )
    return segments


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------


def subtraction_params(args: argparse.Namespace) -> SubtractionParams:
    return SubtractionParams(
        threshold=args.threshold,
        illumination=args.illumination,
        illumination_sigma=args.illumination_sigma,
        edge_tolerance=args.edge_tolerance,
        min_area=args.min_area,
    )


def draw_caption(image: np.ndarray, caption: str) -> None:
    for color, thickness in (((0, 0, 0), 3), ((255, 255, 255), 1)):
        cv2.putText(
            image, caption, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness, cv2.LINE_AA
        )


def render(
    frame: np.ndarray,
    difference: np.ndarray,
    foreground: np.ndarray,
    blobs: list[Blob],
    alpha: float,
    caption: str,
) -> np.ndarray:
    """Black-and-white difference laid over the RGB frame, biggest blobs boxed."""
    # Additive, so a zero difference leaves the RGB untouched and only real change lightens up.
    overlay = cv2.addWeighted(frame, 1.0, cv2.cvtColor(difference, cv2.COLOR_GRAY2BGR), alpha, 0.0)

    contours, _ = cv2.findContours(foreground, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (255, 255, 255), 1)

    for rank, (x, y, width, height, area) in enumerate(blobs):
        color = (0, 0, 255) if rank == 0 else (0, 190, 255)
        cv2.rectangle(overlay, (x, y), (x + width, y + height), color, 2)
        cv2.putText(
            overlay,
            f"#{rank + 1} {area}px",
            (x, max(14, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
            cv2.LINE_AA,
        )

    draw_caption(overlay, caption)
    return overlay


def process_frame(
    frame: np.ndarray, index: int, segment: Segment, stamp_ns: int, args: argparse.Namespace
) -> np.ndarray:
    """One frame of the pipeline: align, subtract, find blobs, draw."""
    height, width = frame.shape[:2]
    size = (width, height)
    warp = segment.warp.estimate(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), stamp_ns)

    if warp is None:
        # No usable alignment; show the frame untouched rather than a bogus difference.
        difference = np.zeros((height, width), np.uint8)
        foreground = difference
        blobs: list[Blob] = []
    else:
        background = warp_forward(segment.background, warp, size)
        valid = warp_forward(segment.compare_mask, warp, size, nearest=True)
        valid = cv2.erode(valid, np.ones((VALID_ERODE_PX * 2 + 1,) * 2, np.uint8))
        difference, foreground = subtract(frame, background, valid, subtraction_params(args))
        blobs = find_blobs(foreground, args.min_area, args.max_blobs)

    caption = (
        f"frame {index}  segment {segment.index}  warp {args.warp}  bg {args.background}  "
        f"blobs {len(blobs)}{'' if warp is not None else '  UNALIGNED'}"
    )
    image = render(frame, difference, foreground, blobs, args.overlay_alpha, caption)
    if args.panel:
        image = np.hstack([image, cv2.cvtColor(difference, cv2.COLOR_GRAY2BGR)])
    return image


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("mcap", type=Path, help="recording to read")
    parser.add_argument("-o", "--output", type=Path, default=Path("background_subtraction.mp4"))
    parser.add_argument("--warp", choices=("plane", "affine"), default="plane")
    parser.add_argument("--background", choices=("median", "ref"), default="median")
    parser.add_argument(
        "--background-samples", type=int, default=48, help="frames per median background"
    )
    parser.add_argument("--threshold", type=int, default=35, help="difference cutoff, 0-255")
    parser.add_argument(
        "--illumination",
        choices=("local", "global", "none"),
        default="local",
        help="how to match background brightness to the frame before differencing",
    )
    parser.add_argument(
        "--illumination-sigma", type=float, default=40.0, help="local gain-field width, pixels"
    )
    parser.add_argument(
        "--edge-tolerance",
        type=float,
        default=0.5,
        help="extra threshold per unit of background gradient; 0 disables",
    )
    parser.add_argument("--min-area", type=int, default=400, help="smallest blob to box, pixels")
    parser.add_argument("--max-blobs", type=int, default=3, help="blobs to box per frame")
    parser.add_argument("--mask-erode", type=int, default=12, help="field mask erosion, pixels")
    parser.add_argument(
        "--overlay-alpha", type=float, default=0.9, help="difference overlay strength"
    )
    parser.add_argument(
        "--orb-features", type=int, default=4000, help="--warp affine feature budget"
    )
    parser.add_argument("--panel", action="store_true", help="append the raw difference as a panel")
    parser.add_argument(
        "--fps", type=float, default=0.0, help="output rate; 0 takes it from stamps"
    )

    args = parser.parse_args()
    if not args.mcap.is_file():
        parser.error(f"no such recording: {args.mcap}")
    return args


def output_fps(stamps: list[int], requested: float) -> float:
    if requested > 0.0:
        return requested
    intervals = np.diff(np.array(stamps, dtype=np.int64))
    return 1e9 / float(np.median(intervals)) if intervals.size else 30.0


def main() -> int:
    args = parse_args()

    stamps = read_frame_stamps(args.mcap)
    if not stamps:
        print(f"{args.mcap} has no {CAMERA_IMAGE_TOPIC} messages", file=sys.stderr)
        return 1

    intrinsics = read_intrinsics(args.mcap)
    segments = build_segments(args, stamps, intrinsics, read_transforms(args.mcap))
    fps = output_fps(stamps, args.fps)

    writer: cv2.VideoWriter | None = None
    written = 0
    for index, (_topic, _ts, data) in enumerate(iter_messages(args.mcap, [CAMERA_IMAGE_TOPIC])):
        if index < segments[0].start_frame:
            continue  # Before the first field init there is nothing to subtract.
        segment = next(s for s in segments if s.start_frame <= index < s.end_frame)
        image = process_frame(
            decode_compressed_image(data).image, index, segment, stamps[index], args
        )

        if writer is None:
            writer = cv2.VideoWriter(
                str(args.output),
                cv2.VideoWriter_fourcc(*"mp4v"),
                fps,
                (image.shape[1], image.shape[0]),
            )
            if not writer.isOpened():
                print(f"Failed to open {args.output} for writing", file=sys.stderr)
                return 1
        writer.write(image)
        written += 1
        if written % 200 == 0:
            print(f"  {written} frames written", flush=True)

    if writer is not None:
        writer.release()
    print(f"Wrote {written} frames to {args.output} at {fps:.2f} fps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
