"""YOLO robot detections as field-frame position fixes for the sysid tag smoother.

The overhead tag drops out while the robot sits flat and in full view: the camera looks down at
an angle, so with the robot heading away from it the 11 degree nose-down tag foreshortens to about
24 px and stops decoding. The deployed YOLO pose model still sees the robot in those frames. Its
front and back keypoints, projected onto a horizontal plane, give the axle centre to about 8 mm,
three times worse than the tag but far better than extrapolating through a second-long gap. Across
40 withheld 1 s tag spans the smoothed position error fell from 95 mm to 8 mm (median). The
keypoint heading (about 5 degrees) is worse than what the IMU yaw rate carries through a gap, so
the smoother takes position only.

Everything the projection needs is fitted per session against the tag, on frames where both see
the robot:

- ``offset_ns``: the video stamp runs ahead of the tag's image stamp by a frame and a half. The
  robot's motion picks the frame; the stamps of frames both streams carry then pin it to the
  stamp-level offset, so a video frame lands within about a millisecond of its camera frame.
- ``plane_height_m``: the height of the plane the keypoints are projected onto. It absorbs where
  the labelled keypoints sit on the chassis, so it need not be a physical height.
- ``axle_offset_m``: from the keypoint midpoint to the axle centre, along the heading.
- the noise, along and across the camera's view, and of the heading.
- size gates. The model sometimes labels the floor marker board as the robot (a 380 px box
  against the robot's 110 to 200 px, keypoints 0.24 to 0.30 m apart against 0.11 to 0.19 m), so a
  detection outside the range the co-visible inliers span, widened by ``GATE_MARGIN``, is dropped.

Fixes fill tag dropouts only: video frames between two tag detections more than
``DROPOUT_MIN_NS`` apart. Between closer tag fixes YOLO adds little and is three times noisier, and
a fit horizon would score against it instead of a tag a few milliseconds away. They are also used
only while the robot is upright and flat by its own IMU: a wheelie or a flip moves the keypoints
off the plane they are projected onto.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

GATE_MARGIN = 1.3
MIN_CONF = 0.5
INLIER_M = 0.03  # a co-visible detection within this of the tag defines the size gates
OFFSETS_MS = tuple(range(-60, 61, 4))
PAIR_MAX_NS = 20_000_000  # a detection pairs with tags no further than this from its stamp
HALF_FRAME_NS = 8_400_000
DROPOUT_MIN_NS = 100_000_000  # tag detections further apart than this bound a dropout
FLAT_LIMIT_DEG = 15.0  # IMU pitch and roll off their upright medians


@dataclass
class YoloCalibration:
    offset_ns: int
    plane_height_m: float
    axle_offset_m: float
    sigma_along_m: float
    sigma_across_m: float
    sigma_yaw_rad: float
    box_diag_px: tuple[float, float]
    keypoint_sep_m: tuple[float, float]
    pairs: int
    position_error_mm: dict[str, float]
    yaw_error_deg: dict[str, float]
    # For plots, not session.json: every confident detection's box diagonal and keypoint
    # separation, and each co-visible pair's position error.
    samples: dict[str, np.ndarray] = field(default_factory=dict, repr=False)

    def as_dict(self) -> dict[str, Any]:
        return {
            "offset_ms": self.offset_ns / 1e6,
            "plane_height_m": self.plane_height_m,
            "axle_offset_m": self.axle_offset_m,
            "sigma_along_mm": 1e3 * self.sigma_along_m,
            "sigma_across_mm": 1e3 * self.sigma_across_m,
            "sigma_yaw_deg": float(np.degrees(self.sigma_yaw_rad)),
            "box_diag_px": list(self.box_diag_px),
            "keypoint_sep_m": list(self.keypoint_sep_m),
            "pairs": self.pairs,
            "position_error_mm": self.position_error_mm,
            "yaw_error_deg": self.yaw_error_deg,
        }


def _wrap(a: np.ndarray) -> np.ndarray:
    return np.asarray((a + np.pi) % (2.0 * np.pi) - np.pi)


def _project(
    u: np.ndarray, v: np.ndarray, k_inv: np.ndarray, up_from_camera: np.ndarray, height: float
) -> tuple[np.ndarray, np.ndarray]:
    """Pixels (N,) onto the plane z = ``height`` of the up frame.

    ``up_from_camera`` is (N, 4, 4).
    """
    rays_cam = k_inv @ np.vstack([u, v, np.ones_like(u)])  # (3, N)
    rays = np.einsum("nij,jn->ni", up_from_camera[:, :3, :3], rays_cam)
    origin = up_from_camera[:, :3, 3]
    scale = (height - origin[:, 2]) / rays[:, 2]
    return origin[:, 0] + scale * rays[:, 0], origin[:, 1] + scale * rays[:, 1]


def project_detections(
    det: pd.DataFrame,
    camera_matrix: np.ndarray,
    up_from_camera: np.ndarray,
    plane_height_m: float,
    axle_offset_m: float,
) -> dict[str, np.ndarray]:
    """Axle-centre x, y, heading and keypoint separation for each detection row."""
    k_inv = np.linalg.inv(camera_matrix)
    fx, fy = _project(
        det["front_u"].to_numpy(float),
        det["front_v"].to_numpy(float),
        k_inv,
        up_from_camera,
        plane_height_m,
    )
    bx, by = _project(
        det["back_u"].to_numpy(float),
        det["back_v"].to_numpy(float),
        k_inv,
        up_from_camera,
        plane_height_m,
    )
    yaw = np.arctan2(fy - by, fx - bx)
    return {
        "x": 0.5 * (fx + bx) + axle_offset_m * np.cos(yaw),
        "y": 0.5 * (fy + by) + axle_offset_m * np.sin(yaw),
        "yaw": yaw,
        "sep": np.hypot(fx - bx, fy - by),
    }


def _box_diag(det: pd.DataFrame) -> np.ndarray:
    return np.asarray(np.hypot(det["x2"] - det["x1"], det["y2"] - det["y1"]), dtype=np.float64)


def _best_per_frame(det: pd.DataFrame) -> pd.DataFrame:
    order = det.sort_values(["stamp_ns", "conf"], ascending=[True, False])
    return order.drop_duplicates("stamp_ns", keep="first").reset_index(drop=True)


def _robust_sigma(x: np.ndarray) -> float:
    return float(1.4826 * np.median(np.abs(x - np.median(x))))


def calibrate(
    det: pd.DataFrame,
    tag_stamp_ns: np.ndarray,
    tag_x: np.ndarray,
    tag_y: np.ndarray,
    tag_yaw: np.ndarray,
    camera_matrix: np.ndarray,
    up_from_camera_at: Any,
    azimuth: float,
) -> YoloCalibration:
    """Fit the projection against tag poses (sorted by stamp, yaw unwrapped) on co-visible frames.

    ``up_from_camera_at(stamps_ns)`` returns the (N, 4, 4) camera pose in the up frame.
    """
    best_per_frame = _best_per_frame(det[det["conf"] >= MIN_CONF])
    stamps = best_per_frame["stamp_ns"].to_numpy(np.int64)
    pose = up_from_camera_at(stamps)

    def nearest_tag(shifted: np.ndarray) -> np.ndarray:
        """Signed stamp of the nearest tag frame minus ``shifted``."""
        j = np.clip(np.searchsorted(tag_stamp_ns, shifted), 1, len(tag_stamp_ns) - 1)
        after, before = tag_stamp_ns[j] - shifted, tag_stamp_ns[j - 1] - shifted
        return np.asarray(np.where(np.abs(after) < np.abs(before), after, before))

    def fit_at(offset_ns: int) -> tuple[float, np.ndarray, np.ndarray] | None:
        shifted = stamps + offset_ns
        pair = np.abs(nearest_tag(shifted)) < PAIR_MAX_NS
        if pair.sum() < 50:
            return None
        sub = best_per_frame[pair]
        tx = np.interp(shifted[pair], tag_stamp_ns, tag_x)
        ty = np.interp(shifted[pair], tag_stamp_ns, tag_y)

        def resid(p: np.ndarray) -> np.ndarray:
            proj = project_detections(sub, camera_matrix, pose[pair], p[0], p[1])
            return np.asarray(np.r_[proj["x"] - tx, proj["y"] - ty])

        fit = least_squares(resid, [0.0, 0.0], loss="soft_l1", f_scale=0.01)
        return float(np.median(np.hypot(*fit.fun.reshape(2, -1)))), fit.x, pair

    coarse: tuple[float, int] | None = None
    for off_ms in OFFSETS_MS:
        result = fit_at(off_ms * 1_000_000)
        if result is not None and (coarse is None or result[0] < coarse[0]):
            coarse = (result[0], off_ms * 1_000_000)
    if coarse is None:
        raise ValueError("too few frames where both the tag and YOLO see the robot")
    # Snap to the stamp offset of frames both streams carry.
    d = nearest_tag(stamps + coarse[1])
    offset_ns = coarse[1] + int(np.median(d[np.abs(d) < HALF_FRAME_NS]))
    final = fit_at(offset_ns)
    if final is None:
        raise ValueError("too few frames where both the tag and YOLO see the robot")
    _, (height, axle), pair = final
    sub = best_per_frame[pair]
    shifted = stamps[pair] + offset_ns
    proj = project_detections(sub, camera_matrix, pose[pair], height, axle)
    dx = proj["x"] - np.interp(shifted, tag_stamp_ns, tag_x)
    dy = proj["y"] - np.interp(shifted, tag_stamp_ns, tag_y)
    dyaw = _wrap(proj["yaw"] - np.interp(shifted, tag_stamp_ns, tag_yaw))
    err = np.hypot(dx, dy)
    c, s = np.cos(azimuth), np.sin(azimuth)
    inl = err < INLIER_M
    diag = _box_diag(sub)[inl]
    sep = proj["sep"][inl]
    confident = det[det["conf"] >= MIN_CONF]
    every = project_detections(
        confident,
        camera_matrix,
        up_from_camera_at(confident["stamp_ns"].to_numpy(np.int64)),
        height,
        axle,
    )
    return YoloCalibration(
        offset_ns=int(offset_ns),
        plane_height_m=float(height),
        axle_offset_m=float(axle),
        sigma_along_m=_robust_sigma(c * dx + s * dy),
        sigma_across_m=_robust_sigma(-s * dx + c * dy),
        sigma_yaw_rad=_robust_sigma(dyaw),
        box_diag_px=(
            float(np.percentile(diag, 0.5) / GATE_MARGIN),
            float(np.percentile(diag, 99.5) * GATE_MARGIN),
        ),
        keypoint_sep_m=(
            float(np.percentile(sep, 0.5) / GATE_MARGIN),
            float(np.percentile(sep, 99.5) * GATE_MARGIN),
        ),
        pairs=int(pair.sum()),
        position_error_mm={f"p{q}": float(1e3 * np.percentile(err, q)) for q in (50, 90, 99)},
        yaw_error_deg={
            f"p{q}": float(np.degrees(np.percentile(np.abs(dyaw), q))) for q in (50, 90, 99)
        },
        samples={
            "box_diag_px": _box_diag(confident),
            "keypoint_sep_m": every["sep"],
            "pair_error_m": err,
        },
    )


def flat_mask(esp32: pd.DataFrame, stamp_ns: np.ndarray) -> np.ndarray:
    """True where the robot's own IMU says it is upright and within ``FLAT_LIMIT_DEG`` of level."""
    e = esp32.sort_values("stamp_ns")
    t = e["stamp_ns"].to_numpy(np.int64)
    upright = e["is_upside_down"].to_numpy() == 0
    pitch = e["orientation_y"].to_numpy(float)
    roll = e["orientation_z"].to_numpy(float)
    ok = upright & (np.abs(pitch - np.median(pitch[upright])) < FLAT_LIMIT_DEG)
    ok &= np.abs(roll - np.median(roll[upright])) < FLAT_LIMIT_DEG
    at = np.clip(np.searchsorted(t, stamp_ns, side="right") - 1, 0, len(t) - 1)
    return np.asarray(ok[at])


def select_fixes(
    det: pd.DataFrame,
    cal: YoloCalibration,
    camera_matrix: np.ndarray,
    up_from_camera_at: Any,
    tag_stamps_ns: np.ndarray,
    esp32: pd.DataFrame | None,
) -> tuple[pd.DataFrame, dict[str, int]]:
    """One fix per video frame inside a tag dropout (see ``DROPOUT_MIN_NS``).

    Returns rows of ``stamp_ns`` (the video stamp moved onto the tag's image clock), ``x``, ``y``,
    ``yaw``, ``conf`` and the counts behind them.
    """
    report: dict[str, int] = {"detections": len(det)}
    cand = det[det["conf"] >= MIN_CONF].reset_index(drop=True)
    report["confident"] = len(cand)
    stamps = cand["stamp_ns"].to_numpy(np.int64)
    proj = project_detections(
        cand, camera_matrix, up_from_camera_at(stamps), cal.plane_height_m, cal.axle_offset_m
    )
    diag = _box_diag(cand)
    sized = (diag >= cal.box_diag_px[0]) & (diag <= cal.box_diag_px[1])
    sized &= (proj["sep"] >= cal.keypoint_sep_m[0]) & (proj["sep"] <= cal.keypoint_sep_m[1])
    report["dropped_by_size_gate"] = int((~sized).sum())
    cand = cand[sized].assign(x=proj["x"][sized], y=proj["y"][sized], yaw=proj["yaw"][sized])
    cand = _best_per_frame(cand)
    shifted = cand["stamp_ns"].to_numpy(np.int64) + cal.offset_ns
    report["video_frames"] = len(shifted)

    # The app drops to about 12 fps once the tag is lost (the full-frame search is slow), so most
    # video frames in a dropout were never processed at all; every one of them gets a fix.
    tags = np.unique(np.asarray(tag_stamps_ns, dtype=np.int64))
    if len(shifted) == 0 or tags.size < 2:
        report["fixes"] = 0
        return pd.DataFrame(columns=["stamp_ns", "x", "y", "yaw", "conf"]), report
    j = np.searchsorted(tags, shifted)
    inside = (j > 0) & (j < tags.size)
    j = np.clip(j, 1, tags.size - 1)
    keep = inside & (tags[j] - tags[j - 1] > DROPOUT_MIN_NS)
    keep &= (tags[j] - shifted > HALF_FRAME_NS) & (shifted - tags[j - 1] > HALF_FRAME_NS)
    report["video_frames_in_dropouts"] = int(keep.sum())
    if esp32 is not None and len(esp32):
        flat = flat_mask(esp32, shifted)
        report["dropped_not_flat"] = int((keep & ~flat).sum())
        keep &= flat
    rows = cand[keep]
    fixes = pd.DataFrame(
        {
            "stamp_ns": shifted[keep],
            "x": rows["x"].to_numpy(),
            "y": rows["y"].to_numpy(),
            "yaw": rows["yaw"].to_numpy(),
            "conf": rows["conf"].to_numpy(),
        }
    )
    report["fixes"] = len(fixes)
    return fixes, report
