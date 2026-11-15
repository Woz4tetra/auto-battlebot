"""The camera views a render is written in, shared by the batch pipeline and the pose server."""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from auto_battlebot.perception.camera_calibration import CameraCalibration, rectify_maps
from synthgen.constants import VIEW_DISTORTED, VIEW_PINHOLE, VIEW_RECTIFIED, VIEWS

EXTENT_SAMPLES = 65
RENDER_MARGIN_PX = 2
_UNDISTORT_CRITERIA = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-10)
_REMAP_DTYPES = (np.uint8, np.uint16, np.int16, np.float32, np.float64)


@dataclass(frozen=True, eq=False)
class LensView:
    """One view at one output size: the camera Blender renders with, and how to warp its frames."""

    view: str
    alpha: float
    size: tuple[int, int]
    render_k: np.ndarray
    render_size: tuple[int, int]
    sensor_k: np.ndarray
    distortion: np.ndarray
    rect_k: np.ndarray
    max_radius: float = math.inf
    distortion_map: tuple[np.ndarray, np.ndarray] | None = None
    rectify_map: tuple[np.ndarray, np.ndarray] | None = None
    valid: np.ndarray | None = None

    @property
    def warps(self) -> bool:
        """Whether rendered frames need warping into this view at all."""
        return self.distortion_map is not None


def build_lens_view(
    calibration: CameraCalibration, size: tuple[int, int], view: str, alpha: float = 1.0
) -> LensView:
    """The render camera and warp maps for view at output size."""
    if view not in VIEWS:
        raise ValueError(f"unknown view {view!r}; valid views are {list(VIEWS)}")
    width, height = size
    cal = calibration.scaled(width, height)
    rect_map_x, rect_map_y, rect_k = rectify_maps(calibration, size, alpha)
    if view == VIEW_PINHOLE:
        return LensView(view, alpha, size, rect_k, size, cal.K, cal.D, rect_k)

    sensor_norm = _undistort_pixels(cal, _pixel_grid(width, height))
    render_k, render_size = wide_render_camera(cal, size)
    render_uv = sensor_norm * (render_k[0, 0], render_k[1, 1]) + (render_k[0, 2], render_k[1, 2])
    distortion_map = (
        np.ascontiguousarray(render_uv[:, 0].reshape(height, width), dtype=np.float32),
        np.ascontiguousarray(render_uv[:, 1].reshape(height, width), dtype=np.float32),
    )
    rectify_map = None
    valid = None
    if view == VIEW_RECTIFIED:
        rectify_map = (rect_map_x, rect_map_y)
        valid = cv2.remap(
            np.ones((height, width), dtype=np.uint8),
            rect_map_x,
            rect_map_y,
            cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        ).astype(bool)
    return LensView(
        view=view,
        alpha=alpha,
        size=size,
        render_k=render_k,
        render_size=render_size,
        sensor_k=cal.K,
        distortion=cal.D,
        rect_k=rect_k,
        max_radius=float(np.max(np.linalg.norm(sensor_norm, axis=1))),
        distortion_map=distortion_map,
        rectify_map=rectify_map,
        valid=valid,
    )


def wide_render_camera(
    calibration: CameraCalibration, size: tuple[int, int]
) -> tuple[np.ndarray, tuple[int, int]]:
    """A pinhole camera that sees every ray the sensor does, at the sensor's own focal length.

    *calibration* must already be scaled to *size*. The render keeps the sensor's focal length so
    the image centre, where distortion is weakest, renders at sensor resolution; the edges, which a
    wide lens compresses, come out oversampled rather than starved.
    """
    width, height = size
    xs = np.linspace(-0.5, width - 0.5, EXTENT_SAMPLES)
    ys = np.linspace(-0.5, height - 0.5, EXTENT_SAMPLES)
    samples = np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2)
    norm = _undistort_pixels(calibration, samples)
    low, high = norm.min(axis=0), norm.max(axis=0)
    fx, fy = calibration.fx, calibration.fy
    render_width = math.ceil((high[0] - low[0]) * fx) + 2 * RENDER_MARGIN_PX
    render_height = math.ceil((high[1] - low[1]) * fy) + 2 * RENDER_MARGIN_PX
    render_k = np.array(
        [
            [fx, 0.0, RENDER_MARGIN_PX - low[0] * fx],
            [0.0, fy, RENDER_MARGIN_PX - low[1] * fy],
            [0.0, 0.0, 1.0],
        ]
    )
    return render_k, (render_width, render_height)


def warp_image(lens: LensView, image: np.ndarray) -> np.ndarray:
    """A rendered colour frame in *lens*'s view, sampled bilinear."""
    return _warp(lens, image, cv2.INTER_LINEAR)


def warp_label(lens: LensView, label: np.ndarray) -> np.ndarray:
    """A rendered id or depth map in *lens*'s view, sampled nearest so values are never blended."""
    return _warp(lens, label, cv2.INTER_NEAREST)


def render_points_to_output(lens: LensView, render_uv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Render-frame pixels (n, 2) to output-frame pixels, and which of them the view sees.

    Returns:
        ``(output_uv, in_view)``: OpenCV pixel coordinates in the output frame, and a mask that is
        False for points outside the frame, beyond the sensor, or in the rectified black border.
    """
    uv = np.asarray(render_uv, dtype=np.float64).reshape(-1, 2)
    width, height = lens.size
    if lens.view == VIEW_PINHOLE:
        out = uv.copy()
        in_view = np.ones(len(uv), dtype=bool)
    else:
        k = lens.render_k
        norm = np.column_stack(((uv[:, 0] - k[0, 2]) / k[0, 0], (uv[:, 1] - k[1, 2]) / k[1, 1]))
        in_view = np.linalg.norm(norm, axis=1) <= lens.max_radius + 1e-9
        if lens.view == VIEW_DISTORTED:
            rays = np.column_stack((norm, np.ones(len(norm))))
            projected, _ = cv2.projectPoints(
                rays, np.zeros(3), np.zeros(3), lens.sensor_k, lens.distortion
            )
            out = projected.reshape(-1, 2)
        else:
            r = lens.rect_k
            out = norm * (r[0, 0], r[1, 1]) + (r[0, 2], r[1, 2])
    in_view &= (
        (out[:, 0] >= -0.5)
        & (out[:, 0] < width - 0.5)
        & (out[:, 1] >= -0.5)
        & (out[:, 1] < height - 0.5)
    )
    if lens.valid is not None and np.any(in_view):
        px = np.clip(np.rint(out[:, 0]).astype(int), 0, width - 1)
        py = np.clip(np.rint(out[:, 1]).astype(int), 0, height - 1)
        in_view &= lens.valid[py, px]
    return out, in_view


def render_normalized_to_output(
    lens: LensView, x_norm: float, y_norm: float
) -> tuple[float, float] | None:
    """A Blender-normalized render point (top-left origin) in the output frame, or None."""
    render_w, render_h = lens.render_size
    uv, in_view = render_points_to_output(
        lens, np.array([[x_norm * render_w - 0.5, y_norm * render_h - 0.5]])
    )
    if not in_view[0]:
        return None
    width, height = lens.size
    return (float(uv[0, 0] + 0.5) / width, float(uv[0, 1] + 0.5) / height)


def _pixel_grid(width: int, height: int) -> np.ndarray:
    xs, ys = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    return np.column_stack((xs.ravel(), ys.ravel()))


def _undistort_pixels(calibration: CameraCalibration, pixels: np.ndarray) -> np.ndarray:
    """Distorted pixels (n, 2) to undistorted normalized image coordinates (n, 2)."""
    source = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    out = cv2.undistortPointsIter(
        source, calibration.K, calibration.D, None, None, _UNDISTORT_CRITERIA
    )
    return np.asarray(out, dtype=np.float64).reshape(-1, 2)


def _warp(lens: LensView, array: np.ndarray, interpolation: int) -> np.ndarray:
    if lens.distortion_map is None:
        return array
    source = np.asarray(array)
    dtype = source.dtype
    work = source if dtype in _REMAP_DTYPES else source.astype(np.float32)
    squeeze = work.ndim == 3 and work.shape[2] == 1
    if squeeze:
        work = work[:, :, 0]
    map_x, map_y = lens.distortion_map
    out = cv2.remap(work, map_x, map_y, interpolation, borderMode=cv2.BORDER_REPLICATE)
    if lens.rectify_map is not None:
        rect_x, rect_y = lens.rectify_map
        out = cv2.remap(
            out, rect_x, rect_y, interpolation, borderMode=cv2.BORDER_CONSTANT, borderValue=0
        )
    if squeeze:
        out = out[:, :, np.newaxis]
    return out if out.dtype == dtype else out.astype(dtype)
