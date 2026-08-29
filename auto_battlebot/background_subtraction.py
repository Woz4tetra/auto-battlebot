"""Field background subtraction: align a background to a moving camera, then difference it.

The pieces here are the parts that do not care where the frames came from. Callers supply
the alignment (a 2x3 affine or a 3x3 homography) and a mask saying which pixels are worth
comparing; these functions build the background, difference against it, and turn the result
into blobs.

Used by `playground/field_background_subtraction.py` (renders a video from an MCAP) and
`training/model_eval/background_subtraction_predict.py` (emits detections for score.py).
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import cv2
import numpy as np

# The local illumination gain field is smooth, so it is fit at this reduction and scaled up.
ILLUMINATION_DOWNSCALE = 8

Blob = tuple[int, int, int, int, int]  # x, y, width, height, area


@dataclass
class SubtractionParams:
    """Knobs for one difference. Defaults are what the NHRL footage was tuned to."""

    threshold: int = 35
    illumination: str = "local"  # local | global | none
    illumination_sigma: float = 40.0
    edge_tolerance: float = 0.5
    min_area: int = 400
    open_px: int = 5
    close_px: int = 15


def warp_forward(
    image: np.ndarray, warp: np.ndarray, size: tuple[int, int], nearest: bool = False
) -> np.ndarray:
    """Apply a 2x3 affine with warpAffine or a 3x3 homography with warpPerspective."""
    flags = cv2.INTER_NEAREST if nearest else cv2.INTER_LINEAR
    if warp.shape == (2, 3):
        return cv2.warpAffine(image, warp, size, flags=flags)
    return cv2.warpPerspective(image, warp, size, flags=flags)


def warp_inverse(image: np.ndarray, warp: np.ndarray, size: tuple[int, int]) -> np.ndarray:
    """Warp an image back through `warp`, into the frame the warp maps out of."""
    if warp.shape == (2, 3):
        return cv2.warpAffine(image, cv2.invertAffineTransform(warp), size)
    return cv2.warpPerspective(image, np.linalg.inv(warp), size)


def build_median_background(
    samples: list[np.ndarray], warps: list[np.ndarray], size: tuple[int, int]
) -> tuple[np.ndarray, np.ndarray]:
    """Per-pixel median of frames warped into a common frame, plus a coverage mask.

    Robots move between samples, so the median of enough of them is an empty field. Returns
    (background BGR, uint8 0/255 mask of pixels at least one sample actually saw).
    """
    width, height = size
    ones = np.full((height, width), 255, np.uint8)

    stack = np.empty((len(samples), height, width, 3), np.uint8)
    coverage = np.empty((len(samples), height, width), bool)
    for index, (image, warp) in enumerate(zip(samples, warps)):
        stack[index] = warp_inverse(image, warp, size)
        coverage[index] = warp_inverse(ones, warp, size) > 0

    background = np.zeros((height, width, 3), np.uint8)
    for channel in range(3):
        # Pixels a sample never saw would drag the median toward black, so drop them per pixel.
        values = stack[:, :, :, channel].astype(np.float32)
        values[~coverage] = np.nan
        with warnings.catch_warnings():
            # A pixel no sample saw is an all-NaN slice; nan_to_num below handles it.
            warnings.simplefilter("ignore", RuntimeWarning)
            median = np.nanmedian(values, axis=0)
        background[:, :, channel] = np.nan_to_num(median, nan=0.0).astype(np.uint8)

    valid = np.where(coverage.any(axis=0), 255, 0).astype(np.uint8)
    return background, valid


def match_illumination(
    frame_gray: np.ndarray,
    background_gray: np.ndarray,
    valid_mask: np.ndarray,
    mode: str,
    sigma: float,
) -> np.ndarray:
    """Rescale the background to the frame's brightness before differencing.

    Two things shift brightness between the background and the current frame. The ZED runs
    auto-exposure, so panning toward the arena lights lifts the whole field by tens of gray
    levels. Glare off the floor and the polycarbonate moves with the camera, so it lifts one
    corner and not the rest. Left alone either one swamps the threshold and the whole field
    reads as foreground.

    "global" fits one median gain, which fixes exposure only. "local" fits a smooth gain field
    over `sigma` pixels, which also absorbs glare: a gradient that wide is illumination, while
    a robot is far smaller and survives. Both use only pixels inside the valid mask, so the
    unfilled area outside the warped background does not pull the fit.
    """
    if mode == "none":
        return background_gray

    valid = valid_mask > 0
    if not valid.any():
        return background_gray

    if mode == "global":
        background_level = float(np.median(background_gray[valid]))
        if background_level < 1.0:
            return background_gray
        gain: np.ndarray | float = float(np.median(frame_gray[valid])) / background_level
    else:
        # The gain field is smooth by construction, so fit it on a small image and scale the
        # result back up. A sigma-40 blur at full resolution needs a 300-pixel kernel and costs
        # more than the rest of the frame put together.
        height, width = frame_gray.shape
        small = (max(16, width // ILLUMINATION_DOWNSCALE), max(9, height // ILLUMINATION_DOWNSCALE))
        small_sigma = max(1.0, sigma / ILLUMINATION_DOWNSCALE)

        # Normalized blur: weight by the valid mask so the edge of the background does not
        # bleed zeros into the gain field.
        weight = cv2.resize(valid.astype(np.float32), small, interpolation=cv2.INTER_AREA)
        frame_small = cv2.resize(frame_gray.astype(np.float32), small, interpolation=cv2.INTER_AREA)
        background_small = cv2.resize(
            background_gray.astype(np.float32), small, interpolation=cv2.INTER_AREA
        )

        weight_blur = cv2.GaussianBlur(weight, (0, 0), small_sigma)
        frame_blur = cv2.GaussianBlur(frame_small * weight, (0, 0), small_sigma)
        background_blur = cv2.GaussianBlur(background_small * weight, (0, 0), small_sigma)
        # Outside the valid mask every blurred term goes to zero; floor the denominator so
        # the gain there is a harmless 1 rather than a NaN that poisons the cast to uint8.
        gain_small = frame_blur / np.maximum(np.maximum(background_blur, weight_blur), 1e-3)
        gain = cv2.resize(gain_small, (width, height), interpolation=cv2.INTER_LINEAR)

    scaled = np.clip(background_gray.astype(np.float32) * np.clip(gain, 0.5, 2.0), 0, 255)
    return np.asarray(scaled, dtype=np.uint8)


def subtract(
    frame: np.ndarray,
    background: np.ndarray,
    valid_mask: np.ndarray,
    params: SubtractionParams,
) -> tuple[np.ndarray, np.ndarray]:
    """Masked absolute difference and its cleaned-up binary foreground."""
    frame_gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    background_gray = cv2.GaussianBlur(cv2.cvtColor(background, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    background_gray = match_illumination(
        frame_gray, background_gray, valid_mask, params.illumination, params.illumination_sigma
    )

    difference: np.ndarray = cv2.absdiff(frame_gray, background_gray)
    difference[valid_mask == 0] = 0

    # A pixel of residual misalignment costs nothing on bare floor and a lot on the edge of a
    # painted marking, so raise the bar in proportion to how steep the background is there.
    if params.edge_tolerance > 0.0:
        gradient = cv2.magnitude(
            cv2.Sobel(background_gray, cv2.CV_32F, 1, 0, ksize=3),
            cv2.Sobel(background_gray, cv2.CV_32F, 0, 1, ksize=3),
        )
        cutoff = np.minimum(params.threshold + params.edge_tolerance * gradient, 255.0)
        foreground: np.ndarray = np.where(difference > cutoff, 255, 0).astype(np.uint8)
    else:
        _, foreground = cv2.threshold(difference, params.threshold, 255, cv2.THRESH_BINARY)

    if params.open_px > 0:
        foreground = cv2.morphologyEx(
            foreground, cv2.MORPH_OPEN, np.ones((params.open_px,) * 2, np.uint8)
        )
    if params.close_px > 0:
        foreground = cv2.morphologyEx(
            foreground, cv2.MORPH_CLOSE, np.ones((params.close_px,) * 2, np.uint8)
        )
    return difference, foreground


def find_blobs(foreground: np.ndarray, min_area: int, max_blobs: int | None = None) -> list[Blob]:
    """(x, y, w, h, area) for connected components above `min_area`, largest first."""
    count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(foreground, connectivity=8)
    blobs: list[Blob] = [
        (
            int(stats[index, cv2.CC_STAT_LEFT]),
            int(stats[index, cv2.CC_STAT_TOP]),
            int(stats[index, cv2.CC_STAT_WIDTH]),
            int(stats[index, cv2.CC_STAT_HEIGHT]),
            int(stats[index, cv2.CC_STAT_AREA]),
        )
        for index in range(1, count)
        if int(stats[index, cv2.CC_STAT_AREA]) >= min_area
    ]
    blobs.sort(key=lambda blob: -blob[4])
    return blobs if max_blobs is None else blobs[:max_blobs]
