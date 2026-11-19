"""AprilTag detection and field-plane pose geometry, shared by capture and analysis."""

from __future__ import annotations

import math
from typing import Iterable, NamedTuple

import cv2
import numpy as np


def make_detector() -> cv2.aruco.ArucoDetector:
    """ArucoDetector tuned for small/distant 36h11 tags (markers ~20-40 px from a 1-1.5 m mount)."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    p = aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin = 5
    p.adaptiveThreshWinSizeMax = 21
    p.adaptiveThreshWinSizeStep = 8
    p.minMarkerPerimeterRate = 0.02
    p.polygonalApproxAccuracyRate = 0.05
    p.errorCorrectionRate = 1.0
    p.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    p.cornerRefinementMinAccuracy = 0.05
    return aruco.ArucoDetector(dictionary, p)


MIN_GAMMA_POWER = 0.35


def auto_gamma(
    frame: np.ndarray, target_mean: float = 120.0, min_mean: float = 110.0
) -> np.ndarray:
    """Brighten an underexposed frame via a gamma curve that lifts its mean toward target_mean."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    mean = float(gray.mean())
    if mean >= min_mean or mean < 1.0:
        return frame
    power = math.log(target_mean / 255.0) / math.log(mean / 255.0)
    power = max(MIN_GAMMA_POWER, min(1.0, power))
    lut = np.clip(255.0 * (np.arange(256) / 255.0) ** power, 0, 255).astype(np.uint8)
    return cv2.LUT(frame, lut)


def detect_markers(
    detector: cv2.aruco.ArucoDetector, frame: np.ndarray, scale: float = 1.0
) -> tuple[np.ndarray, tuple[np.ndarray, ...], np.ndarray | None]:
    """Auto-brighten then detect."""
    disp = auto_gamma(frame)
    img = (
        disp
        if scale == 1.0
        else cv2.resize(disp, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    )
    raw_corners, ids, _ = detector.detectMarkers(img)
    corners = tuple(np.asarray(c, dtype=np.float32) for c in raw_corners)
    if scale != 1.0 and corners:
        corners = tuple(c / scale for c in corners)
    return disp, corners, None if ids is None else np.asarray(ids)


def make_floor_board(
    cols: int, rows: int, marker_size: float, marker_sep: float, first_id: int
) -> cv2.aruco.GridBoard:
    """AprilTag GridBoard for the floor reference, ids numbered right-to-left per row."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    ids = np.arange(first_id, first_id + cols * rows).reshape(rows, cols)[:, ::-1].reshape(-1)
    return cv2.aruco.GridBoard((cols, rows), marker_size, marker_sep, dictionary, ids)


class FieldToCamera(NamedTuple):
    r_fc: np.ndarray
    t_fc: np.ndarray


def solve_floor_extrinsic(
    frames: Iterable[np.ndarray],
    detector: cv2.aruco.ArucoDetector,
    board: cv2.aruco.GridBoard,
    k: np.ndarray,
    d: np.ndarray,
    min_markers: int = 4,
) -> FieldToCamera:
    """Solve the camera pose against the floor grid from every frame that sees the board."""
    board_ids = set(np.asarray(board.getIds()).flatten().tolist())
    obj_all: list[np.ndarray] = []
    img_all: list[np.ndarray] = []
    used = 0
    for frame in frames:
        _, corners, ids = detect_markers(detector, frame)
        if ids is None:
            continue
        keep = [i for i, mid in enumerate(ids.flatten()) if int(mid) in board_ids]
        if len(keep) < min_markers:
            continue
        obj, img = board.matchImagePoints([corners[i] for i in keep], ids[keep].reshape(-1, 1))
        if obj is None or len(obj) < min_markers:
            continue
        obj_all.append(obj.reshape(-1, 3))
        img_all.append(img.reshape(-1, 2))
        used += 1
    if used == 0:
        raise SystemExit(
            "Floor grid not detected in the floor frames. Make sure the grid board was flat and "
            "fully in "
            "view at lock time, and that --floor-cols/--floor-rows/--floor-first-id match it "
            "(default 3x5, "
            "ids 160..174)."
        )
    ok, rvec, tvec = cv2.solvePnP(np.vstack(obj_all), np.vstack(img_all), k, d)
    if not ok:
        raise SystemExit(
            "Floor board PnP failed; check intrinsics and the --floor-marker-size (m)."
        )
    r, _ = cv2.Rodrigues(rvec)
    print(f"floor frame locked from {used} frames, {len(np.vstack(obj_all))} marker corners")
    return FieldToCamera(r, tvec.reshape(3))


def tag_object_points(size: float) -> np.ndarray:
    """Tag corners in the tag frame (x right, y up, z out), order TL, TR, BR, BL, to match aruco."""
    h = size / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def tag_pose_field(
    corners: np.ndarray,
    obj: np.ndarray,
    k: np.ndarray,
    d: np.ndarray,
    r_fc: np.ndarray,
    t_fc: np.ndarray,
    yaw_offset: float,
) -> tuple[float, float, float, np.ndarray, np.ndarray] | None:
    """Solve the tag pose and express its (x, y, yaw) in the floor frame."""
    corners = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, corners, k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    r_ct, _ = cv2.Rodrigues(rvec)
    pos_cam = tvec.reshape(3)
    pos_field = r_fc.T @ (pos_cam - t_fc)
    xaxis_field = r_fc.T @ (r_ct @ np.array([1.0, 0.0, 0.0]))
    yaw = math.atan2(xaxis_field[1], xaxis_field[0]) + yaw_offset
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return float(pos_field[0]), float(pos_field[1]), yaw, rvec, tvec
