"""Overhead AprilTag ground-truth capture for drivetrain characterization.

Tracks a fiducial mounted on top of Mrs Buff and writes a (t, x, y, yaw) CSV in the field plane
(metres / radians). This is the clean pose source the plant fit needs: it sidesteps the perception
flat-plane projection bias and the yaw keypoint flips that corrupt fit_plant.py on perception poses.

Pose is solved with the camera intrinsics (cv2.solvePnP), not a 2D homography. Two PnP solves:
  1. An AprilTag GridBoard placed flat on the floor defines the world (field) frame. The board pose is
     solved ONCE, interactively, at startup: you place the board in view, the floor frame is locked, and
     then you REMOVE the board so it never obstructs the robot. No clicking pixels by hand; correct even
     for a tilted mount. Because the lock is one-time, the board's absence during driving is fine.
  2. Each frame, the robot's AprilTag (known physical size) + intrinsics give the tag's true 3D pose in the
     camera frame, which is then expressed in the floor frame.

Because the tag pose is solved in 3D, the tag's height above the floor (it sits on top of the robot) does
not bias the (x, y) the way a floor homography would: solvePnP recovers the tag's real position and we read
off its floor-plane coordinates directly.

The floor grid and the robot tag share one dictionary (DICT_APRILTAG_36h11) but must use disjoint ids: the
grid uses ids [--floor-first-id, --floor-first-id + cols*rows), the robot tag uses --tag-id outside that.
Defaults match the manufactured floor board: a 3x5 grid of 65 mm markers with 15 mm gaps, ids 160..174
numbered right-to-left per row, so the robot tag must avoid that range (default --tag-id 20).

Intrinsics come from the OAK on-device calibration automatically when --source oak, or from --intrinsics.
For a plain camera/video --intrinsics is required. The OAK-1 W's wide lens IS distorted, so the real
distortion coefficients are used and solvePnP undistorts the marker corners.

Timestamps use CLOCK_MONOTONIC (time.monotonic), system-wide on Linux, so when this runs on the same host
as calibrate_drive.py the two CSVs share a clock and need no alignment.

Install deps first: pip install -r playground/calibration/requirements.txt

Usage:
    source scripts/activate_python.sh
    # 1. Print the robot tag PDF (see make_print_tags.py) and tape it flat on the robot top. The floor
    #    grid is the manufactured 3x5 board, so it does not need printing.
    python playground/calibration/make_print_tags.py --out-dir playground/calibration/print
    # 2. OAK-1 W live (1080p @ 60 fps), intrinsics from the device; defaults match the manufactured board.
    #    A live preview window opens: aim the floor board and press [L] to lock the frame, remove the board
    #    and press [S] to start tracking (shows the robot's live x/y/yaw), press [Q] to stop. --no-preview
    #    runs headless.
    python playground/calibration/apriltag_track.py \
        --source oak --out playground/calibration/out/truth_log.csv
    # Any other camera/video with your own intrinsics instead of the OAK:
    python playground/calibration/apriltag_track.py --source 0 \
        --intrinsics playground/calibration/cam_intrinsics.json --tag-size 0.13

cam_intrinsics.json (either form is accepted):
    { "camera_matrix": [[fx,0,cx],[0,fy,cy],[0,0,1]], "dist_coeffs": [k1,k2,p1,p2,k3] }
    { "fx": 1050.0, "fy": 1050.0, "cx": 960.0, "cy": 540.0 }
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from contextlib import contextmanager
from pathlib import Path

import cv2
import numpy as np


def wait_for_enter(message: str) -> None:
    """Print an instruction and block until the operator presses Enter.

    On a non-interactive stdin (piped, or an automated video replay) there is no operator to prompt, so we
    print the message and continue without waiting.
    """
    print(f"\n{message}")
    if sys.stdin.isatty():
        input(">>> press Enter to continue... ")
    else:
        print("    (non-interactive stdin, continuing automatically)")


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


def auto_gamma(frame: np.ndarray, target_mean: float = 120.0, min_mean: float = 110.0) -> np.ndarray:
    """Brighten an underexposed frame via a gamma curve that lifts its mean toward target_mean.

    Arena lighting is often dim and the camera underexposes: a raw frame at mean ~52/255 detects 0 tags
    because the black/white marker contrast is crushed into the shadows. A power-law stretch recovers it
    (0 -> ~10 tags on a real dim frame). Well-lit frames (mean >= min_mean) are returned untouched, so this never
    hurts a good exposure. The transform is monotonic and applied per channel, so marker corner geometry
    (hence the recovered pose) is unchanged.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    mean = float(gray.mean())
    if mean >= min_mean or mean < 1.0:
        return frame
    power = math.log(target_mean / 255.0) / math.log(mean / 255.0)
    power = max(0.35, min(1.0, power))  # clamp so very dark frames are not blown into noise
    lut = np.clip(255.0 * (np.arange(256) / 255.0) ** power, 0, 255).astype(np.uint8)
    return cv2.LUT(frame, lut)


def read_frame(source: "CvSource | OakSource") -> tuple[bool, np.ndarray]:
    """Read a frame and auto-brighten it for detection (and for the preview, so dim scenes are visible)."""
    ok, frame = source.read()
    if not ok:
        return False, frame
    return True, auto_gamma(frame)


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


class OakSource:
    """Luxonis OAK camera (OAK-1 W) via DepthAI. Provides the color image and its factory intrinsics.

    Resolution, not detector tuning, is what makes the tags detect at a 1-1.5 m mount: face-on, 36h11
    needs ~18 px edge to decode. 1080p (1920x1080) gives the 65 mm floor markers ~36 px and the 130 mm
    robot tag ~70 px at ~1 m on the wide lens, clearing the cliff, while still running 60 fps for the
    actuation-lag estimate. Bump RESOLUTION to 4K if the mount is higher than ~1.5 m.

    Unlike a pre-rectified image, the OAK-1 W's wide lens is distorted, so intrinsics() returns the real
    distortion coefficients from the on-device calibration and solvePnP undistorts the marker corners.
    """

    RESOLUTION = "1080"  # one of: "1080" (1920x1080), "4k" (3840x2160), "12mp"
    FPS = 60.0

    def __init__(self, source: str = "oak") -> None:
        try:
            import depthai as dai
        except ModuleNotFoundError as e:
            raise SystemExit(
                "depthai not found. Install the calibration deps:\n"
                "  pip install -r playground/calibration/requirements.txt"
            ) from e

        self._dai = dai
        res_map = {
            "1080": dai.ColorCameraProperties.SensorResolution.THE_1080_P,
            "4k": dai.ColorCameraProperties.SensorResolution.THE_4_K,
            "12mp": dai.ColorCameraProperties.SensorResolution.THE_12_MP,
        }

        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # the single OAK-1 W sensor
        cam.setResolution(res_map[self.RESOLUTION])
        cam.setFps(self.FPS)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setInterleaved(False)
        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("video")
        cam.video.link(xout.input)

        self._device = dai.Device(pipeline)
        self._queue = self._device.getOutputQueue("video", maxSize=4, blocking=False)
        # Grab one frame up front to learn the true output size (needed to scale the intrinsics); serve it
        # back on the first read() so it is not wasted.
        self._pending = self._queue.get()
        f0 = self._pending.getCvFrame()
        self._h, self._w = f0.shape[:2]
        speed = str(self._device.getUsbSpeed())
        print(f"OAK opened: {self._w}x{self._h} @ {self.FPS:.0f} fps (CAM_A), USB link: {speed}")
        if "SUPER" not in speed.upper():
            print(f"  WARNING: USB link is {speed} (USB 2.0). 1080p cannot stream at {self.FPS:.0f} fps over "
                  "USB 2.0 and will be throttled to ~25-30 fps. Plug the OAK into a USB 3 port (blue USB-A "
                  "or USB-C) for full frame rate.")

    def read(self) -> tuple[bool, np.ndarray]:
        if self._pending is not None:
            frame = self._pending.getCvFrame()
            self._pending = None
            return True, frame
        frame = self._queue.get()
        return True, frame.getCvFrame()

    def intrinsics(self) -> tuple[np.ndarray, np.ndarray] | None:
        dai = self._dai
        calib = self._device.readCalibration()
        k = np.array(
            calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, self._w, self._h), dtype=np.float64
        )
        d = np.array(
            calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A), dtype=np.float64
        ).reshape(-1, 1)
        model = calib.getDistortionModel(dai.CameraBoardSocket.CAM_A)
        if "FISHEYE" in str(model).upper():
            print(f"WARNING: OAK reports a {model} distortion model; solvePnP assumes Brown-Conrady. "
                  "Pose near the frame edges may be off on this ultra-wide lens.")
        return k, d

    def release(self) -> None:
        self._device.close()


def open_source(source: str) -> CvSource | OakSource:
    if source == "oak":
        return OakSource(source)
    return CvSource(source)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


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


def camera_extrinsic_from_board(
    source: "CvSource | OakSource",
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
    Locking once means the board can then be removed from the arena entirely without affecting the
    extrinsic, so it never obstructs the robot during the driving run.
    """
    board_ids = set(board.getIds().flatten().tolist())
    obj_all: list[np.ndarray] = []
    img_all: list[np.ndarray] = []
    used = 0
    while used < n_frames:
        ok, frame = read_frame(source)
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
            "Floor grid not detected. Make sure the grid board is flat, fully in view, and that "
            "--floor-cols/--floor-rows/--floor-first-id match the board (default 3x5, ids 160..174)."
        )
    ok, rvec, tvec = cv2.solvePnP(np.vstack(obj_all), np.vstack(img_all), k, d)
    if not ok:
        raise SystemExit("Floor board PnP failed; check intrinsics and the --floor-marker-size (m).")
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


WINDOW = "apriltag_track"
PREVIEW_MAX_FPS = 20.0       # cap the preview redraw so imshow/waitKey does not gate the capture loop
PREVIEW_DISPLAY_WIDTH = 1280  # downscale before imshow (the window upscales) to cut render cost


def show_scaled(frame: np.ndarray) -> int:
    """imshow a downscaled copy (the full-screen window upscales it) and return the waitKey code.

    The render/convert cost of imshow scales with the source image size, so handing it a 1280-wide copy
    instead of the full 1920 roughly halves it; waitKey is unavoidable but small.
    """
    h, w = frame.shape[:2]
    if w > PREVIEW_DISPLAY_WIDTH:
        frame = cv2.resize(frame, (PREVIEW_DISPLAY_WIDTH, round(h * PREVIEW_DISPLAY_WIDTH / w)),
                           interpolation=cv2.INTER_NEAREST)
    cv2.imshow(WINDOW, frame)
    return cv2.waitKey(1) & 0xFF


def screen_workarea() -> tuple[int, int, int, int] | None:
    """Usable screen rect (x, y, w, h), excluding panels/taskbars, from X11. None if it can't be read.

    _NET_WORKAREA is the work area the window manager reserves for normal windows (full screen minus docks
    and panels). Falls back to the current xrandr resolution, then gives up (caller leaves the window
    default-sized).
    """
    import subprocess

    try:
        out = subprocess.run(
            ["xprop", "-root", "-notype", "_NET_WORKAREA"], capture_output=True, text=True, timeout=2
        ).stdout
        nums = [int(n) for n in out.split("=", 1)[1].replace(",", " ").split()]
        if len(nums) >= 4:
            return nums[0], nums[1], nums[2], nums[3]  # first desktop's x, y, w, h
    except Exception:
        pass
    try:
        out = subprocess.run(["xrandr", "--current"], capture_output=True, text=True, timeout=2).stdout
        for line in out.splitlines():
            if "*" in line:  # the active mode, e.g. "   1920x1080     60.00*+"
                w, h = line.split()[0].split("x")
                return 0, 0, int(w), int(h)
    except Exception:
        pass
    return None


def open_preview_window() -> None:
    """Create the preview window sized to the usable screen, keeping the image aspect ratio."""
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    area = screen_workarea()
    if area is not None:
        x, y, w, h = area
        cv2.resizeWindow(WINDOW, w, h)
        cv2.moveWindow(WINDOW, x, y)


def draw_hud(frame: np.ndarray, lines: list[str], ok: bool = True) -> None:
    """Overlay a few lines of status text (with a dark outline so it reads on any background)."""
    color = (80, 255, 80) if ok else (60, 170, 255)  # BGR: green when ready, amber when not
    y = 26
    for line in lines:
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)
        y += 26


class FpsMeter:
    """Smoothed end-to-end loop rate (read + detect + draw): the real throughput shown in the preview."""

    def __init__(self, alpha: float = 0.1) -> None:
        self._alpha = alpha
        self._last: float | None = None
        self.fps = 0.0
        self.lo = 0.0  # slowest (min) fps seen, i.e. the worst hitch

    def tick(self) -> float:
        t = time.monotonic()
        if self._last is not None:
            dt = t - self._last
            if dt > 0:
                inst = 1.0 / dt
                self.fps = inst if self.fps == 0.0 else (1 - self._alpha) * self.fps + self._alpha * inst
                self.lo = inst if self.lo == 0.0 else min(self.lo, inst)
        self._last = t
        return self.fps


def draw_fps(frame: np.ndarray, meter: "FpsMeter") -> None:
    """Draw the FPS stat in the top-right corner so it does not collide with the left-aligned HUD."""
    text = f"{meter.fps:4.1f} fps (min {meter.lo:4.1f})"
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    x = max(12, frame.shape[1] - tw - 12)
    cv2.putText(frame, text, (x, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text, (x, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1, cv2.LINE_AA)


class LoopProfiler:
    """Per-stage wall-clock timing for a preview loop, printed to the console every `interval` seconds.

    Reveals what the loop is actually waiting on instead of guessing:
      - a big `read` means the loop is camera-bound (blocking on the next frame, i.e. USB/sensor throughput);
      - a big `detect`/`pose` means it is CPU-bound, and watching those vs the printed tag count shows how
        the cost scales with the number of tags in view.
    Each line: average ms per stage over the interval, the achieved fps, and the current tag count.
    """

    def __init__(self, label: str, interval: float = 1.0) -> None:
        self.label = label
        self.interval = interval
        self._acc: dict[str, float] = {}
        self._n = 0
        self._tags = 0
        self._t0 = time.perf_counter()

    @contextmanager
    def section(self, name: str):
        self._acc.setdefault(name, 0.0)
        t = time.perf_counter()
        try:
            yield
        finally:
            self._acc[name] += time.perf_counter() - t

    def frame(self, tags: int = 0) -> None:
        """Call once per loop iteration. Prints a breakdown and resets when the interval elapses."""
        self._n += 1
        self._tags = tags
        now = time.perf_counter()
        if now - self._t0 < self.interval or self._n == 0:
            return
        fps = self._n / (now - self._t0)
        per = " | ".join(f"{name} {sec / self._n * 1000:5.1f}" for name, sec in self._acc.items())
        total = sum(self._acc.values()) / self._n * 1000
        print(f"[prof:{self.label}] {fps:5.1f} fps  ({total:5.1f} ms/frame, tags={self._tags})  {per}")
        self._acc = {k: 0.0 for k in self._acc}
        self._n = 0
        self._t0 = now


def lock_floor_with_preview(
    source: "CvSource | OakSource",
    detector: cv2.aruco.ArucoDetector,
    board: cv2.aruco.GridBoard,
    k: np.ndarray,
    d: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Show a live preview while aiming the floor board; lock the floor frame on the operator's keypress.

    Lets the operator see exactly what the camera sees and how many board markers are detected before
    committing the one-time floor lock. Press L (or SPACE) to lock once >=4 markers are seen, Q to quit.
    """
    board_ids = set(board.getIds().flatten().tolist())
    fps = FpsMeter()
    prof = LoopProfiler("lock")
    last_show = 0.0
    while True:
        with prof.section("read"):
            ok, frame = read_frame(source)
        if not ok:
            raise SystemExit("camera read failed during floor lock")
        fps.tick()
        with prof.section("detect"):
            corners, ids, _ = detector.detectMarkers(frame)
        n_all = 0 if ids is None else len(ids)
        keep = [i for i, m in enumerate(ids.flatten()) if int(m) in board_ids] if ids is not None else []
        n = len(keep)
        ready = n >= 4
        key = 255
        if time.monotonic() - last_show >= 1.0 / PREVIEW_MAX_FPS:
            last_show = time.monotonic()
            with prof.section("show"):
                if keep:
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i] for i in keep], ids[keep])
                draw_hud(frame, [
                    "FLOOR LOCK (one time)",
                    f"board markers seen: {n}  (need >= 4)",
                    "aim so the grid is flat and fully in view",
                    "[L] lock floor frame    [Q] quit",
                ], ok=ready)
                draw_fps(frame, fps)
                key = show_scaled(frame)
        prof.frame(tags=n_all)
        if key == ord("q"):
            raise SystemExit("quit before floor lock")
        if key in (ord("l"), ord(" ")) and ready:
            return camera_extrinsic_from_board(source, detector, board, k, d)


def confirm_robot_with_preview(
    source: "CvSource | OakSource",
    detector: cv2.aruco.ArucoDetector,
    tag_id: int,
    obj: np.ndarray,
    k: np.ndarray,
    d: np.ndarray,
    r_fc: np.ndarray,
    t_fc: np.ndarray,
    yaw_offset: float,
) -> None:
    """After the lock, preview the robot tag's live field pose so the board can be removed and the robot
    placed in view before recording starts. Press S to start tracking, Q to quit."""
    fps = FpsMeter()
    prof = LoopProfiler("confirm")
    last_show = 0.0
    while True:
        with prof.section("read"):
            ok, frame = read_frame(source)
        if not ok:
            raise SystemExit("camera read failed before tracking")
        fps.tick()
        pose = None
        idx = -1
        with prof.section("detect"):
            corners, ids, _ = detector.detectMarkers(frame)
        n_all = 0 if ids is None else len(ids)
        with prof.section("pose"):
            if ids is not None and tag_id in ids.flatten():
                idx = int(np.where(ids.flatten() == tag_id)[0][0])
                pose = tag_pose_field(corners[idx], obj, k, d, r_fc, t_fc, yaw_offset)
        key = 255
        if time.monotonic() - last_show >= 1.0 / PREVIEW_MAX_FPS:
            last_show = time.monotonic()
            with prof.section("show"):
                if idx >= 0:
                    cv2.aruco.drawDetectedMarkers(frame, [corners[idx]], np.array([[tag_id]]))
                    if pose is not None:
                        cv2.drawFrameAxes(frame, k, d, pose[3], pose[4], obj[1, 0] - obj[0, 0])
                status = (f"robot x={pose[0]:+.3f} y={pose[1]:+.3f} yaw={math.degrees(pose[2]):+6.1f}deg"
                          if pose is not None else f"robot tag (id {tag_id}) not visible")
                draw_hud(frame, [
                    "FLOOR LOCKED.  Remove the grid board.",
                    f"place robot (tag id {tag_id}) in view",
                    status,
                    "[S] start tracking    [Q] quit",
                ], ok=pose is not None)
                draw_fps(frame, fps)
                key = show_scaled(frame)
        prof.frame(tags=n_all)
        if key == ord("q"):
            raise SystemExit("quit before tracking")
        if key == ord("s"):
            return


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default="oak",
                        help="'oak' (OAK-1 W via DepthAI), a camera index, or a video file")
    parser.add_argument("--tag-size", type=float, default=0.13,
                        help="robot AprilTag edge length in metres (default 0.13 = make_print_tags robot tag; "
                             "must match the tag you actually mounted, it sets the metric scale)")
    parser.add_argument(
        "--intrinsics", type=Path, default=None, help="K/D JSON; required unless --source provides them"
    )
    parser.add_argument("--out", type=Path, default=Path("playground/calibration/out/truth_log.csv"))
    parser.add_argument("--tag-id", type=int, default=20, help="AprilTag id mounted on the robot")
    parser.add_argument(
        "--yaw-offset-deg", type=float, default=0.0,
        help="tag +x (TL->TR edge) direction relative to robot forward",
    )
    # Floor reference grid (an AprilTag GridBoard placed flat on the floor for the one-time frame lock).
    # Defaults match the manufactured board: 3x5 markers, 65 mm edge, 15 mm gaps, ids 160..174.
    parser.add_argument("--floor-cols", type=int, default=3, help="grid markers in x")
    parser.add_argument("--floor-rows", type=int, default=5, help="grid markers in y")
    parser.add_argument("--floor-marker-size", type=float, default=0.065, help="grid marker edge (m)")
    parser.add_argument("--floor-marker-sep", type=float, default=0.015, help="gap between grid markers (m)")
    parser.add_argument("--floor-first-id", type=int, default=160, help="lowest grid marker id (board uses 160..174)")
    parser.add_argument("--no-preview", action="store_true",
                        help="disable the live preview window (use for headless / automated video replay)")
    args = parser.parse_args()
    preview = not args.no_preview

    board = make_floor_board(
        args.floor_cols, args.floor_rows, args.floor_marker_size, args.floor_marker_sep, args.floor_first_id
    )

    board_id_range = range(args.floor_first_id, args.floor_first_id + args.floor_cols * args.floor_rows)
    if args.tag_id in board_id_range:
        raise SystemExit(
            f"--tag-id {args.tag_id} collides with the floor grid ids "
            f"[{board_id_range.start}, {board_id_range.stop}). Pick a robot tag id outside that range."
        )

    detector = make_detector()
    source = open_source(args.source)

    # Intrinsics: explicit file wins; otherwise ask the source (the OAK on-device calibration).
    if args.intrinsics is not None:
        k, d = load_intrinsics(args.intrinsics)
    else:
        from_source = source.intrinsics()
        if from_source is None:
            source.release()
            raise SystemExit("No intrinsics: pass --intrinsics (only --source oak provides them).")
        k, d = from_source
        print(f"intrinsics from source: fx={k[0, 0]:.1f} fy={k[1, 1]:.1f} cx={k[0, 2]:.1f} cy={k[1, 2]:.1f}")

    obj = tag_object_points(args.tag_size)
    yaw_offset = math.radians(args.yaw_offset_deg)

    if preview:
        open_preview_window()
        # Phase 1: live-aim the floor board, lock on keypress. Phase 2: confirm the robot tag's live pose
        # with the board removed, start tracking on keypress.
        r_fc, t_fc = lock_floor_with_preview(source, detector, board, k, d)
        confirm_robot_with_preview(source, detector, args.tag_id, obj, k, d, r_fc, t_fc, yaw_offset)
    else:
        wait_for_enter(
            "FLOOR LOCK (one time): place the AprilTag grid board flat in the arena, fully visible to the "
            f"camera ({args.floor_cols}x{args.floor_rows} markers, ids "
            f"{args.floor_first_id}..{args.floor_first_id + args.floor_cols * args.floor_rows - 1})."
        )
        r_fc, t_fc = camera_extrinsic_from_board(source, detector, board, k, d)
        wait_for_enter(
            "Floor frame locked. REMOVE the grid board from the arena so it does not obstruct the robot, "
            "then place the robot (tag up) in view. Tracking starts after you continue."
        )

    if preview:
        print("Recording. Press Q in the preview window to stop.")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    n_frames = n_seen = 0
    fps = FpsMeter()
    prof = LoopProfiler("track")
    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "yaw", "visible"])
        last_show = 0.0
        while True:
            with prof.section("read"):
                ok, frame = read_frame(source)
            if not ok:
                break
            t = time.monotonic()
            fps.tick()
            n_frames += 1

            with prof.section("detect"):
                corners, ids, _ = detector.detectMarkers(frame)
            n_all = 0 if ids is None else len(ids)
            row: list = [f"{t:.4f}", "", "", "", 0]
            pose = None
            idx = -1
            with prof.section("pose"):
                if ids is not None and args.tag_id in ids.flatten():
                    idx = int(np.where(ids.flatten() == args.tag_id)[0][0])
                    quad = corners[idx].reshape(4, 2)
                    pose = tag_pose_field(quad, obj, k, d, r_fc, t_fc, yaw_offset)
                    if pose is not None:
                        row = [f"{t:.4f}", f"{pose[0]:.5f}", f"{pose[1]:.5f}", f"{pose[2]:.5f}", 1]
                        n_seen += 1
            writer.writerow(row)

            # Display is throttled to PREVIEW_MAX_FPS and downscaled so it never gates the record rate.
            stop = False
            if preview and t - last_show >= 1.0 / PREVIEW_MAX_FPS:
                last_show = t
                with prof.section("show"):
                    if pose is not None:
                        cv2.aruco.drawDetectedMarkers(frame, [corners[idx]], np.array([[args.tag_id]]))
                        cv2.drawFrameAxes(frame, k, d, pose[3], pose[4], args.tag_size / 2.0)
                    seen_pct = 100.0 * n_seen / n_frames
                    status = (f"x={pose[0]:+.3f} y={pose[1]:+.3f} yaw={math.degrees(pose[2]):+6.1f}deg"
                              if pose is not None else "tag not visible")
                    draw_hud(frame, [
                        "TRACKING (recording)",
                        status,
                        f"frame {n_frames}   seen {n_seen} ({seen_pct:.0f}%)",
                        "[Q] stop",
                    ], ok=pose is not None)
                    draw_fps(frame, fps)
                    stop = show_scaled(frame) == ord("q")
            prof.frame(tags=n_all)
            if stop:
                break

    source.release()
    if preview:
        cv2.destroyAllWindows()
    seen_pct = 100.0 * n_seen / n_frames if n_frames else 0.0
    print(f"{n_frames} frames, tag seen in {n_seen} ({seen_pct:.1f}%), "
          f"~{fps.fps:.0f} fps (min {fps.lo:.0f}). Truth log: {args.out}")


if __name__ == "__main__":
    main()
