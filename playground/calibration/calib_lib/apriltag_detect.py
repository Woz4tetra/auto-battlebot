"""AprilTag detection and field-plane pose geometry, shared by capture and analysis.

apriltag_track.py uses this live (to guide the operator in the preview); analyze_apriltag_mcap.py uses it
to re-detect the recorded camera images after the fact. Keeping detection in one module means the offline
poses are solved with the exact detector, gamma correction, and PnP the operator saw on screen.

Two PnP solves, same as the original single-script flow:
  1. solve_floor_extrinsic locks the camera pose relative to a flat AprilTag GridBoard, defining the field
     frame (z = 0 on the floor). Solved once from several frames in which the board is visible.
  2. tag_pose_field solves the robot tag's 3D pose each frame and expresses its (x, y, yaw) in that frame.
     Because the pose is solved in 3D, the tag's height above the floor does not bias (x, y) the way a
     floor homography would.
"""

from __future__ import annotations

import math
from typing import Iterable

import cv2
import numpy as np


def make_detector() -> cv2.aruco.ArucoDetector:
    """ArucoDetector tuned for small/distant 36h11 tags (markers ~20-40 px from a 1-1.5 m mount).

    The hard limit is pixels-on-target: face-on, 36h11 decodes reliably only above ~18 px edge, and no
    parameter recovers a marker below ~15 px (that is what the 1080p default buys). Within that budget these
    settings squeeze out the margin and sharpen pose:
      - a tight adaptive-threshold window range (5..21, step 8 -> 3 passes). The big windows (>21) were the
        cost driver at 1080p (33 ms -> ~11 ms per frame, the difference between ~30 and ~60 fps) and added
        no detections once auto_gamma normalizes brightness, so they are dropped. Verified 15/15 on both a
        face-on render and an oblique 1080p frame.
      - lower perimeter gate so small markers are not pre-rejected; looser polygon approx tolerates the
        rougher quads low-res markers produce.
      - max error correction: 36h11's Hamming distance (11) leaves plenty of headroom, so accepting more
        corrected bits recovers borderline reads without inviting false positives.
      - subpixel corner refinement: tighter corners -> lower (x, y, yaw) noise, which the plant fit needs.
        It is cheap here (~0-4 ms) so it stays.
    minMarkerDistanceRate is left at its default: lowering it interacts with subpixel refinement to drop a
    valid marker (it merged duplicate candidates), verified on the board photo (15/15 only at the default).
    """
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


def auto_gamma(
    frame: np.ndarray, target_mean: float = 120.0, min_mean: float = 110.0
) -> np.ndarray:
    """Brighten an underexposed frame via a gamma curve that lifts its mean toward target_mean.

    Arena lighting is often dim and the camera underexposes: a raw frame at mean ~52/255 detects 0 tags
    because the black/white marker contrast is crushed into the shadows. A power-law stretch recovers it
    (0 -> ~10 tags on a real dim frame). Well-lit frames (mean >= min_mean) are returned untouched, so this
    never hurts a good exposure. The transform is monotonic and applied per channel, so marker corner
    geometry (hence the recovered pose) is unchanged. Applied at detection time, not when recording, so the
    MCAP keeps the raw frames and the gamma can be retuned offline.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    mean = float(gray.mean())
    if mean >= min_mean or mean < 1.0:
        return frame
    power = math.log(target_mean / 255.0) / math.log(mean / 255.0)
    power = max(0.35, min(1.0, power))  # clamp so very dark frames are not blown into noise
    lut = np.clip(255.0 * (np.arange(256) / 255.0) ** power, 0, 255).astype(np.uint8)
    return cv2.LUT(frame, lut)


def detect_markers(detector: cv2.aruco.ArucoDetector, frame: np.ndarray, scale: float = 1.0):
    """Auto-brighten then detect. Returns (display_frame, corners, ids) where display_frame is the
    original-resolution gamma frame, so a preview draws markers on the same pixels it detected.

    `scale` > 1 upsamples the (gamma-corrected) frame before detection, which recovers small markers
    sitting near the pixels-on-target limit: the robot tag is only ~40 px at 1080p and its detection
    flickers on/off during fast motion. Upsampling 2x lifts the borderline reads over the line
    (measured ~+6 pts on the moving frames of a real run). The offline analyzer uses scale=2 for the
    robot tag; the live preview leaves it at 1 for speed. Corners are divided back by `scale` before
    returning, so PnP solves in the true original-image pixel frame regardless of scale, and the
    subpixel refinement done at the higher resolution actually sharpens the corners.
    """
    disp = auto_gamma(frame)
    img = (
        disp
        if scale == 1.0
        else cv2.resize(disp, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    )
    corners, ids, _ = detector.detectMarkers(img)
    if scale != 1.0 and corners:
        corners = tuple(c / scale for c in corners)
    return disp, corners, ids


def make_floor_board(
    cols: int, rows: int, marker_size: float, marker_sep: float, first_id: int
) -> cv2.aruco.GridBoard:
    """AprilTag GridBoard for the floor reference. Its frame is the field frame (z = 0 on the floor).

    The manufactured board numbers its markers right-to-left within each row (verified by detecting the
    board photo: with first_id=160 the top-left tag is 162, top-right is 160). cv2's GridBoard fills ids
    left-to-right, so each row is reversed to match the physical layout. Without this, matchImagePoints
    pairs each id with a horizontally-mirrored object point and solvePnP returns a garbage extrinsic
    (a reflection is not a rotation: ~440 px reprojection error vs ~16 px when correct).
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    ids = np.arange(first_id, first_id + cols * rows).reshape(rows, cols)[:, ::-1].reshape(-1)
    return cv2.aruco.GridBoard((cols, rows), marker_size, marker_sep, dictionary, ids)


def solve_floor_extrinsic(
    frames: Iterable[np.ndarray],
    detector: cv2.aruco.ArucoDetector,
    board: cv2.aruco.GridBoard,
    k: np.ndarray,
    d: np.ndarray,
    min_markers: int = 4,
) -> tuple[np.ndarray, np.ndarray]:
    """Lock the camera pose relative to the floor grid by PnP over the frames in which the board is visible.

    Returns (R_fc, t_fc) mapping a floor point X_f to camera coords: X_c = R_fc @ X_f + t_fc. The board is
    static, so correspondences from several frames are stacked into one solve to average detection noise.
    Frames are raw camera images (gamma is applied internally), so this works identically on the live lock
    burst and on the /floor/image frames replayed from a recording.
    """
    board_ids = set(board.getIds().flatten().tolist())
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
            "Floor grid not detected in the floor frames. Make sure the grid board was flat and fully in "
            "view at lock time, and that --floor-cols/--floor-rows/--floor-first-id match it (default 3x5, "
            "ids 160..174)."
        )
    ok, rvec, tvec = cv2.solvePnP(np.vstack(obj_all), np.vstack(img_all), k, d)
    if not ok:
        raise SystemExit(
            "Floor board PnP failed; check intrinsics and the --floor-marker-size (m)."
        )
    r, _ = cv2.Rodrigues(rvec)
    print(f"floor frame locked from {used} frames, {len(np.vstack(obj_all))} marker corners")
    return r, tvec.reshape(3)


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
    """Solve the tag pose and express its (x, y, yaw) in the floor frame. None if PnP fails.

    Also returns the raw camera-frame (rvec, tvec) so callers can draw the tag axes in a preview.
    """
    corners = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, corners, k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    r_ct, _ = cv2.Rodrigues(rvec)
    pos_cam = tvec.reshape(3)
    # Floor coords: X_f = R_fc^T (X_c - t_fc).
    pos_field = r_fc.T @ (pos_cam - t_fc)
    xaxis_field = r_fc.T @ (r_ct @ np.array([1.0, 0.0, 0.0]))
    yaw = math.atan2(xaxis_field[1], xaxis_field[0]) + yaw_offset
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return float(pos_field[0]), float(pos_field[1]), yaw, rvec, tvec
