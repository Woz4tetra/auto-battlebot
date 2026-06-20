"""Overhead AprilTag ground-truth capture for drivetrain characterization.

Tracks a fiducial mounted on top of Mrs Buff and writes a (t, x, y, yaw) CSV in the field plane
(metres / radians). This is the clean pose source the plant fit needs: it sidesteps the ZED flat-plane
projection bias and the yaw keypoint flips that corrupt fit_plant.py on perception poses.

Pose is solved with the camera intrinsics (cv2.solvePnP), not a 2D homography. Two PnP solves:
  1. A printed AprilTag GridBoard lying flat on the floor defines the world (field) frame. Its markers are
     detected and the board pose is solved once at startup, fixing the metric driving plane and a 2D frame
     on it. No clicking pixels by hand; correct even for a tilted mount.
  2. Each frame, the robot's AprilTag (known physical size) + intrinsics give the tag's true 3D pose in the
     camera frame, which is then expressed in the floor frame.

Because the tag pose is solved in 3D, the tag's height above the floor (it sits on top of the robot) does
not bias the (x, y) the way a floor homography would: solvePnP recovers the tag's real position and we read
off its floor-plane coordinates directly.

The floor grid and the robot tag share one dictionary (DICT_APRILTAG_36h11) but must use disjoint ids: the
grid uses ids [--floor-first-id, --floor-first-id + cols*rows), the robot tag uses --tag-id outside that.

Intrinsics come from the ZED SDK automatically when --source zed (or an .svo path), or from --intrinsics
(the ones you already have). For a plain camera/video --intrinsics is required. The ZED's rectified LEFT
image has ~zero distortion, so dist_coeffs default to zero.

Timestamps use CLOCK_MONOTONIC (time.monotonic), system-wide on Linux, so when this runs on the same host
as calibrate_drive.py the two CSVs share a clock and need no alignment.

Usage:
    source scripts/activate_python.sh
    # 1. Generate and print the floor grid + robot tag PDFs (see make_print_tags.py), tape them down.
    python playground/calibration/make_print_tags.py --out-dir playground/calibration/print
    # 2. ZED live, intrinsics + fastest fps from the SDK. The defaults here match make_print_tags.py, so
    #    only --tag-size (the robot tag edge length) is needed:
    python playground/calibration/apriltag_track.py \
        --source zed --tag-size 0.13 --out playground/calibration/out/truth_log.csv
    # Any camera/video with your own intrinsics instead of the ZED:
    python playground/calibration/apriltag_track.py --source 0 \
        --intrinsics playground/calibration/zed_intrinsics.json --tag-size 0.13

zed_intrinsics.json (either form is accepted):
    { "camera_matrix": [[fx,0,cx],[0,fy,cy],[0,0,1]], "dist_coeffs": [k1,k2,p1,p2,k3] }
    { "fx": 1050.0, "fy": 1050.0, "cx": 960.0, "cy": 540.0 }
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read camera matrix K (3x3) and distortion D from JSON. Accepts a full matrix or fx/fy/cx/cy."""
    data = json.loads(path.read_text())
    if "camera_matrix" in data:
        k = np.asarray(data["camera_matrix"], dtype=np.float64).reshape(3, 3)
    else:
        k = np.array(
            [[data["fx"], 0, data["cx"]], [0, data["fy"], data["cy"]], [0, 0, 1]], dtype=np.float64
        )
    d = np.asarray(data.get("dist_coeffs", [0, 0, 0, 0, 0]), dtype=np.float64).reshape(-1, 1)
    return k, d


def make_detector() -> cv2.aruco.ArucoDetector:
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    return aruco.ArucoDetector(dictionary, aruco.DetectorParameters())


# ---------------------------------------------------------------------------
# Frame sources
# ---------------------------------------------------------------------------


class CvSource:
    """Generic camera index or video file via OpenCV. Intrinsics must be supplied separately."""

    def __init__(self, source: str) -> None:
        self._cap = cv2.VideoCapture(int(source)) if source.isdigit() else cv2.VideoCapture(source)
        if not self._cap.isOpened():
            raise SystemExit(f"Could not open video source: {source}")

    def read(self) -> tuple[bool, np.ndarray]:
        return self._cap.read()

    def intrinsics(self) -> tuple[np.ndarray, np.ndarray] | None:
        return None

    def release(self) -> None:
        self._cap.release()


class ZedSource:
    """ZED SDK source (live camera or .svo file). Provides the rectified LEFT image and its intrinsics."""

    def __init__(self, source: str) -> None:
        try:
            import pyzed.sl as sl
        except ModuleNotFoundError as e:
            raise SystemExit(
                "pyzed not found. Use --source zed only where the ZED SDK is installed, or pass a camera "
                "index/video with --intrinsics."
            ) from e

        self._sl = sl
        self._zed = sl.Camera()
        init = sl.InitParameters()
        init.coordinate_units = sl.UNIT.METER
        init.depth_mode = sl.DEPTH_MODE.NONE  # we only need the rectified image
        # Fastest mode on the ZED 2i: VGA (672x376) at 100 FPS. The high frame rate sharpens the
        # actuation-lag estimate (fit_plant_calib needs >=60 fps to resolve it). Intrinsics are pulled from
        # the SDK below, so they match this resolution. If the tag is too small to detect from a high mount,
        # step down to RESOLUTION.HD720 @ 60.
        if source == "zed":
            init.camera_resolution = sl.RESOLUTION.VGA
            init.camera_fps = 100
        else:
            init.set_from_svo_file(source)
        err = self._zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            raise SystemExit(f"ZED open failed: {err}")
        if source == "zed":
            info = self._zed.get_camera_information()
            cfg = getattr(info, "camera_configuration", info)
            fps = getattr(cfg, "fps", 0.0) or getattr(cfg, "camera_fps", 0.0)
            print(f"ZED opened at {fps:.0f} fps (VGA, fastest mode)")
        self._runtime = sl.RuntimeParameters()
        self._mat = sl.Mat()

    def read(self) -> tuple[bool, np.ndarray]:
        if self._zed.grab(self._runtime) != self._sl.ERROR_CODE.SUCCESS:
            return False, np.empty(0)
        self._zed.retrieve_image(self._mat, self._sl.VIEW.LEFT)
        return True, self._mat.get_data()[:, :, :3].copy()  # BGRA -> BGR

    def intrinsics(self) -> tuple[np.ndarray, np.ndarray] | None:
        info = self._zed.get_camera_information()
        # SDK 4.x nests calibration under camera_configuration; older builds expose it directly.
        calib = getattr(getattr(info, "camera_configuration", info), "calibration_parameters", None)
        if calib is None:
            return None
        cam = calib.left_cam
        k = np.array([[cam.fx, 0, cam.cx], [0, cam.fy, cam.cy], [0, 0, 1]], dtype=np.float64)
        return k, np.zeros((5, 1))  # rectified LEFT image is undistorted

    def release(self) -> None:
        self._zed.close()


def open_source(source: str) -> CvSource | ZedSource:
    if source == "zed" or source.endswith((".svo", ".svo2")):
        return ZedSource(source)
    return CvSource(source)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def make_floor_board(
    cols: int, rows: int, marker_size: float, marker_sep: float, first_id: int
) -> cv2.aruco.GridBoard:
    """AprilTag GridBoard for the floor reference. Its frame is the field frame (z = 0 on the floor)."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    ids = np.arange(first_id, first_id + cols * rows)
    return cv2.aruco.GridBoard((cols, rows), marker_size, marker_sep, dictionary, ids)


def camera_extrinsic_from_board(
    source: "CvSource | ZedSource",
    detector: cv2.aruco.ArucoDetector,
    board: cv2.aruco.GridBoard,
    k: np.ndarray,
    d: np.ndarray,
    n_frames: int = 20,
    min_markers: int = 4,
) -> tuple[np.ndarray, np.ndarray]:
    """Lock the camera pose relative to the floor grid, once, by PnP over several startup frames.

    Returns (R_fc, t_fc) mapping a floor point X_f to camera coords: X_c = R_fc @ X_f + t_fc. The board is
    static, so correspondences from several frames are stacked into one solve to average detection noise.
    Locking once means the robot may later occlude the grid without affecting the extrinsic.
    """
    board_ids = set(board.getIds().flatten().tolist())
    obj_all: list[np.ndarray] = []
    img_all: list[np.ndarray] = []
    used = 0
    while used < n_frames:
        ok, frame = source.read()
        if not ok:
            break
        corners, ids, _ = detector.detectMarkers(frame)
        if ids is None:
            continue
        keep = [i for i, mid in enumerate(ids.flatten()) if int(mid) in board_ids]
        if len(keep) < min_markers:
            continue
        sel_corners = [corners[i] for i in keep]
        sel_ids = ids[keep].reshape(-1, 1)
        obj, img = board.matchImagePoints(sel_corners, sel_ids)
        if obj is None or len(obj) < min_markers:
            continue
        obj_all.append(obj.reshape(-1, 3))
        img_all.append(img.reshape(-1, 2))
        used += 1
    if used == 0:
        raise SystemExit(
            "Floor grid not detected. Make sure the printed GridBoard is flat, fully in view, and that "
            "--floor-cols/--floor-rows/--floor-first-id match the printed board."
        )
    ok, rvec, tvec = cv2.solvePnP(np.vstack(obj_all), np.vstack(img_all), k, d)
    if not ok:
        raise SystemExit("Floor board PnP failed; check intrinsics and the printed marker size.")
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
) -> tuple[float, float, float] | None:
    """Solve the tag pose and express its (x, y, yaw) in the floor frame. None if PnP fails."""
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
    return float(pos_field[0]), float(pos_field[1]), yaw


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default="zed", help="'zed', an .svo path, a camera index, or a video")
    parser.add_argument("--tag-size", type=float, help="robot AprilTag edge length in metres (required to track)")
    parser.add_argument(
        "--intrinsics", type=Path, default=None, help="K/D JSON; required unless --source provides them"
    )
    parser.add_argument("--out", type=Path, default=Path("playground/calibration/out/truth_log.csv"))
    parser.add_argument("--tag-id", type=int, default=0, help="AprilTag id mounted on the robot")
    parser.add_argument(
        "--yaw-offset-deg", type=float, default=0.0,
        help="tag +x (TL->TR edge) direction relative to robot forward",
    )
    # Floor reference grid (an AprilTag GridBoard taped flat on the floor). Defaults match
    # make_print_tags.py, which generates the printable PDF; keep the two in sync.
    parser.add_argument("--floor-cols", type=int, default=4, help="grid markers in x")
    parser.add_argument("--floor-rows", type=int, default=3, help="grid markers in y")
    parser.add_argument("--floor-marker-size", type=float, default=0.16, help="grid marker edge (m)")
    parser.add_argument("--floor-marker-sep", type=float, default=0.05, help="gap between grid markers (m)")
    parser.add_argument("--floor-first-id", type=int, default=10, help="first grid marker id")
    parser.add_argument("--show", action="store_true", help="display detections for setup/aiming")
    args = parser.parse_args()

    board = make_floor_board(
        args.floor_cols, args.floor_rows, args.floor_marker_size, args.floor_marker_sep, args.floor_first_id
    )

    if args.tag_size is None:
        raise SystemExit("--tag-size is required to track (the robot AprilTag edge length in metres).")
    board_id_range = range(args.floor_first_id, args.floor_first_id + args.floor_cols * args.floor_rows)
    if args.tag_id in board_id_range:
        raise SystemExit(
            f"--tag-id {args.tag_id} collides with the floor grid ids "
            f"[{board_id_range.start}, {board_id_range.stop}). Pick a robot tag id outside that range."
        )

    detector = make_detector()
    source = open_source(args.source)

    # Intrinsics: explicit file wins; otherwise ask the source (the ZED SDK).
    if args.intrinsics is not None:
        k, d = load_intrinsics(args.intrinsics)
    else:
        from_source = source.intrinsics()
        if from_source is None:
            source.release()
            raise SystemExit("No intrinsics: pass --intrinsics (only --source zed/.svo provides them).")
        k, d = from_source
        print(f"intrinsics from source: fx={k[0, 0]:.1f} fy={k[1, 1]:.1f} cx={k[0, 2]:.1f} cy={k[1, 2]:.1f}")

    r_fc, t_fc = camera_extrinsic_from_board(source, detector, board, k, d)
    obj = tag_object_points(args.tag_size)
    yaw_offset = math.radians(args.yaw_offset_deg)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    n_frames = n_seen = 0
    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "yaw", "visible"])
        while True:
            ok, frame = source.read()
            if not ok:
                break
            t = time.monotonic()
            n_frames += 1

            corners, ids, _ = detector.detectMarkers(frame)
            row: list = [f"{t:.4f}", "", "", "", 0]
            if ids is not None and args.tag_id in ids.flatten():
                idx = int(np.where(ids.flatten() == args.tag_id)[0][0])
                quad = corners[idx].reshape(4, 2)
                pose = tag_pose_field(quad, obj, k, d, r_fc, t_fc, yaw_offset)
                if pose is not None:
                    x, y, yaw = pose
                    row = [f"{t:.4f}", f"{x:.5f}", f"{y:.5f}", f"{yaw:.5f}", 1]
                    n_seen += 1
                if args.show:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            writer.writerow(row)

            if args.show:
                cv2.imshow("apriltag_track", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    source.release()
    if args.show:
        cv2.destroyAllWindows()
    seen_pct = 100.0 * n_seen / n_frames if n_frames else 0.0
    print(f"{n_frames} frames, tag seen in {n_seen} ({seen_pct:.1f}%). Truth log: {args.out}")


if __name__ == "__main__":
    main()
