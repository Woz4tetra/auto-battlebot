"""Camera intrinsics from `config/cameras/<id>.toml`, and the rectification the C++ side does.

Mirrors `include/rgbd_camera/camera_calibration.hpp`: the same TOML keys, and `rectify_maps`
reproduces `Rectifier::build` (`getOptimalNewCameraMatrix` with alpha 1.0, then
`initUndistortRectifyMap`), so a frame rectified here matches what playback hands the
perception stack. Anything that wants to compare against those frames, a fitted camera pose
or a rendered image, must use the rectified matrix this returns, not the calibrated one.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

if sys.version_info >= (3, 11):
    import tomllib
else:  # pragma: no cover - Python 3.10 fallback
    import tomli as tomllib


@dataclass(frozen=True)
class CameraCalibration:
    calibration_id: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, float, float, float, float]  # k1 k2 p1 p2 k3

    @property
    def K(self) -> np.ndarray:  # noqa: N802 - conventional name for the camera matrix
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    @property
    def D(self) -> np.ndarray:  # noqa: N802
        return np.array(self.distortion, dtype=np.float64)

    def scaled(self, width: int, height: int) -> CameraCalibration:
        """The same lens at another resolution: intrinsics scale, distortion does not."""
        sx, sy = width / self.width, height / self.height
        return CameraCalibration(
            self.calibration_id,
            width,
            height,
            self.fx * sx,
            self.fy * sy,
            self.cx * sx,
            self.cy * sy,
            self.distortion,
        )


def load_camera_calibration(path: Path) -> CameraCalibration:
    with path.open("rb") as handle:
        data = tomllib.load(handle)
    keys = ("k1", "k2", "p1", "p2", "k3")
    return CameraCalibration(
        calibration_id=str(data["calibration_id"]),
        width=int(data["width"]),
        height=int(data["height"]),
        fx=float(data["fx"]),
        fy=float(data["fy"]),
        cx=float(data["cx"]),
        cy=float(data["cy"]),
        distortion=tuple(float(data.get(key, 0.0)) for key in keys),  # type: ignore[arg-type]
    )


def rectify_maps(
    calibration: CameraCalibration, size: tuple[int, int], alpha: float = 1.0
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(map_x, map_y, K_rect) for `cv2.remap`, exactly as `Rectifier::build` computes them.

    `size` is (width, height). Alpha 1.0 keeps every source pixel, which is what the C++
    side chooses so the mat edges are never cropped away.
    """
    width, height = size
    cal = calibration.scaled(width, height)
    k_rect, _ = cv2.getOptimalNewCameraMatrix(cal.K, cal.D, size, alpha, size)
    map_x, map_y = cv2.initUndistortRectifyMap(cal.K, cal.D, None, k_rect, size, cv2.CV_16SC2)
    return map_x, map_y, np.asarray(k_rect, dtype=np.float64)


def undistort_points(
    points_xy: np.ndarray, calibration: CameraCalibration, k_rect: np.ndarray
) -> np.ndarray:
    """Distorted pixel coordinates (n, 2) to their positions in the rectified frame."""
    cal = calibration
    source = np.asarray(points_xy, dtype=np.float64).reshape(-1, 1, 2)
    out = cv2.undistortPoints(source, cal.K, cal.D, P=k_rect)
    return np.asarray(out).reshape(-1, 2)
