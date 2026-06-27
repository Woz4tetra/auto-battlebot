"""Overhead AprilTag ground-truth capture for drivetrain characterization.

Tracks a fiducial mounted on top of Mrs Buff. This script records the raw camera images to an MCAP file;
analyze_apriltag_mcap.py replays them, re-runs the AprilTag detection, and solves the field-plane
(t, x, y, yaw) pose the plant fit needs. That clean pose source sidesteps the perception flat-plane
projection bias and the yaw keypoint flips that corrupt fit_plant.py on perception poses.

Capture records images, not poses or detections: the floor-board lock burst on /floor/image, the driving
frames on /camera/image, and one metadata message (intrinsics, image size, tag + floor board params); see
apriltag_mcap.py for the layout. Recording images means the floor lock, the detector tuning, the
intrinsics, and the yaw offset can all be corrected and re-run offline without re-driving the robot. The
detection here (in apriltag_detect, shared with analysis) only drives the live preview: the operator aims
the floor board and confirms the robot tag is visible before the drive.

The pose itself is solved (in analysis) with the camera intrinsics (cv2.solvePnP), not a 2D homography.
Two PnP solves:
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
as calibrate_drive.py the recording and the cmd log share a clock and need no alignment.

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
        --source oak --out playground/calibration/out/apriltag_track.mcap
    # Any other camera/video with your own intrinsics instead of the OAK:
    python playground/calibration/apriltag_track.py --source 0 \
        --intrinsics playground/calibration/cam_intrinsics.json --tag-size 0.13
    # Store lossless raw frames instead of JPEG (larger files, no compression-induced corner shift):
    python playground/calibration/apriltag_track.py --source oak --image-format raw
    # 3. Solve the field-plane poses offline and write the (t, x, y, yaw, visible) truth CSV:
    python playground/calibration/analyze_apriltag_mcap.py \
        playground/calibration/out/apriltag_track.mcap --out playground/calibration/out/truth_log.csv

cam_intrinsics.json (either form is accepted):
    { "camera_matrix": [[fx,0,cx],[0,fy,cy],[0,0,1]], "dist_coeffs": [k1,k2,p1,p2,k3] }
    { "fx": 1050.0, "fy": 1050.0, "cx": 960.0, "cy": 540.0 }
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from calib_lib import apriltag_detect as ad
from calib_lib import apriltag_mcap as amcap

FLOOR_LOCK_FRAMES = 20  # frames captured (and recorded) at the lock keypress to solve the floor extrinsic


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


# Detection and pose geometry (make_detector, auto_gamma, make_floor_board, tag_object_points,
# tag_pose_field, solve_floor_extrinsic) live in apriltag_detect so the offline analysis re-detects the
# recorded frames with the exact same code path this preview uses.


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


def record_floor_burst(
    source: "CvSource | OakSource", writer: amcap.CaptureWriter, n_frames: int = FLOOR_LOCK_FRAMES
) -> list[np.ndarray]:
    """Grab and record a short burst of raw frames to /floor/image, returning them for the extrinsic solve.

    The board is fully aimed at this point, so consecutive frames carry it; solve_floor_extrinsic filters
    out any that fall short. Recording the frames (rather than just the solved extrinsic) is what lets the
    floor lock be re-solved offline with corrected intrinsics or detector tuning.
    """
    frames: list[np.ndarray] = []
    while len(frames) < n_frames:
        ok, frame = source.read()
        if not ok:
            break
        writer.write_floor_image(time.monotonic(), frame)
        frames.append(frame)
    if not frames:
        raise SystemExit("camera read failed during floor lock")
    return frames


def lock_floor_with_preview(
    source: "CvSource | OakSource",
    detector: cv2.aruco.ArucoDetector,
    board: cv2.aruco.GridBoard,
    k: np.ndarray,
    d: np.ndarray,
    writer: amcap.CaptureWriter,
) -> tuple[np.ndarray, np.ndarray]:
    """Show a live preview while aiming the floor board; lock the floor frame on the operator's keypress.

    Lets the operator see exactly what the camera sees and how many board markers are detected before
    committing the one-time floor lock. Press L (or SPACE) to lock once >=4 markers are seen, Q to quit.
    On lock, the frame burst is recorded to /floor/image and the extrinsic solved (here for the live
    tracking readout; analysis re-solves it from those same recorded frames).
    """
    board_ids = set(board.getIds().flatten().tolist())
    fps = FpsMeter()
    prof = LoopProfiler("lock")
    last_show = 0.0
    while True:
        with prof.section("read"):
            ok, frame = source.read()
        if not ok:
            raise SystemExit("camera read failed during floor lock")
        fps.tick()
        with prof.section("detect"):
            disp, corners, ids = ad.detect_markers(detector, frame)
        n_all = 0 if ids is None else len(ids)
        keep = [i for i, m in enumerate(ids.flatten()) if int(m) in board_ids] if ids is not None else []
        n = len(keep)
        ready = n >= 4
        key = 255
        if time.monotonic() - last_show >= 1.0 / PREVIEW_MAX_FPS:
            last_show = time.monotonic()
            with prof.section("show"):
                if keep:
                    cv2.aruco.drawDetectedMarkers(disp, [corners[i] for i in keep], ids[keep])
                draw_hud(disp, [
                    "FLOOR LOCK (one time)",
                    f"board markers seen: {n}  (need >= 4)",
                    "aim so the grid is flat and fully in view",
                    "[L] lock floor frame    [Q] quit",
                ], ok=ready)
                draw_fps(disp, fps)
                key = show_scaled(disp)
        prof.frame(tags=n_all)
        if key == ord("q"):
            raise SystemExit("quit before floor lock")
        if key in (ord("l"), ord(" ")) and ready:
            frames = record_floor_burst(source, writer)
            return ad.solve_floor_extrinsic(frames, detector, board, k, d)


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
            ok, frame = source.read()
        if not ok:
            raise SystemExit("camera read failed before tracking")
        fps.tick()
        pose = None
        idx = -1
        with prof.section("detect"):
            disp, corners, ids = ad.detect_markers(detector, frame)
        n_all = 0 if ids is None else len(ids)
        with prof.section("pose"):
            if ids is not None and tag_id in ids.flatten():
                idx = int(np.where(ids.flatten() == tag_id)[0][0])
                pose = ad.tag_pose_field(corners[idx], obj, k, d, r_fc, t_fc, yaw_offset)
        key = 255
        if time.monotonic() - last_show >= 1.0 / PREVIEW_MAX_FPS:
            last_show = time.monotonic()
            with prof.section("show"):
                if idx >= 0:
                    cv2.aruco.drawDetectedMarkers(disp, [corners[idx]], np.array([[tag_id]]))
                    if pose is not None:
                        cv2.drawFrameAxes(disp, k, d, pose[3], pose[4], obj[1, 0] - obj[0, 0])
                status = (f"robot x={pose[0]:+.3f} y={pose[1]:+.3f} yaw={math.degrees(pose[2]):+6.1f}deg"
                          if pose is not None else f"robot tag (id {tag_id}) not visible")
                draw_hud(disp, [
                    "FLOOR LOCKED.  Remove the grid board.",
                    f"place robot (tag id {tag_id}) in view",
                    status,
                    "[S] start tracking    [Q] quit",
                ], ok=pose is not None)
                draw_fps(disp, fps)
                key = show_scaled(disp)
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
    default_out = (Path(__file__).resolve().parent / "out" /
                   f"apriltag_track_{datetime.now():%Y%m%d_%H%M%S}.mcap")
    parser.add_argument("--out", type=Path, default=default_out,
                        help="MCAP recording of raw camera images; solve poses with analyze_apriltag_mcap.py")
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
    parser.add_argument("--show-other-tags", action="store_true",
                        help="in the tracking preview, also draw the floor-frame pose of every non-robot "
                             "tag (floor-marker size for grid ids, robot tag size otherwise)")
    parser.add_argument("--image-format", choices=amcap.IMAGE_FORMATS, default="jpeg",
                        help="jpeg: smaller, lossy (default); raw: lossless bgr8, larger, exact corners")
    args = parser.parse_args()
    preview = not args.no_preview

    board = ad.make_floor_board(
        args.floor_cols, args.floor_rows, args.floor_marker_size, args.floor_marker_sep, args.floor_first_id
    )

    board_id_range = range(args.floor_first_id, args.floor_first_id + args.floor_cols * args.floor_rows)
    if args.tag_id in board_id_range:
        raise SystemExit(
            f"--tag-id {args.tag_id} collides with the floor grid ids "
            f"[{board_id_range.start}, {board_id_range.stop}). Pick a robot tag id outside that range."
        )

    detector = ad.make_detector()
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

    obj = ad.tag_object_points(args.tag_size)
    yaw_offset = math.radians(args.yaw_offset_deg)
    # Non-robot tags solved with --show-other-tags: grid markers use the floor-marker size, every
    # other id falls back to the robot tag size (axes still draw, but the x/y is approximate).
    floor_obj = ad.tag_object_points(args.floor_marker_size)
    n_floor = args.floor_cols * args.floor_rows
    floor_ids = frozenset(range(args.floor_first_id, args.floor_first_id + n_floor))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    writer = amcap.CaptureWriter(args.out, image_format=args.image_format)
    print(f"recording {args.image_format} images to {args.out}")
    # Metadata (filled with the image size from the first recorded frame) carries everything analysis needs
    # except the floor extrinsic, which it re-solves from the recorded /floor/image frames.
    writer.set_metadata({
        "camera_matrix": k.reshape(-1).tolist(),
        "dist_coeffs": d.reshape(-1).tolist(),
        "tag_size": args.tag_size,
        "tag_id": args.tag_id,
        "yaw_offset_deg": args.yaw_offset_deg,
        "floor": {
            "cols": args.floor_cols, "rows": args.floor_rows,
            "marker_size": args.floor_marker_size, "marker_sep": args.floor_marker_sep,
            "first_id": args.floor_first_id,
        },
        "clock": "monotonic",
    })

    n_frames = 0
    fps = FpsMeter()
    prof = LoopProfiler("track")
    last_tags = 0
    last_show = 0.0
    try:
        if preview:
            open_preview_window()
            # Phase 1: live-aim the floor board and lock on keypress (records the floor burst). Phase 2:
            # confirm the robot tag is visible with the board removed, start recording on keypress.
            r_fc, t_fc = lock_floor_with_preview(source, detector, board, k, d, writer)
            confirm_robot_with_preview(source, detector, args.tag_id, obj, k, d, r_fc, t_fc, yaw_offset)
            print("Recording. Press Q in the preview window to stop.")
        else:
            wait_for_enter(
                "FLOOR LOCK (one time): place the AprilTag grid board flat in the arena, fully visible to "
                f"the camera ({args.floor_cols}x{args.floor_rows} markers, ids "
                f"{args.floor_first_id}..{args.floor_first_id + args.floor_cols * args.floor_rows - 1})."
            )
            r_fc, t_fc = ad.solve_floor_extrinsic(
                record_floor_burst(source, writer), detector, board, k, d
            )
            wait_for_enter(
                "Floor frame locked. REMOVE the grid board from the arena so it does not obstruct the "
                "robot, then place the robot (tag up) in view. Recording starts after you continue."
            )

        # Record the driving frames raw to /camera/image. Detection runs only on the throttled preview
        # frames, purely to tell the operator whether the robot tag is in view; analysis re-detects offline.
        while True:
            with prof.section("read"):
                ok, frame = source.read()
            if not ok:
                break
            t = time.monotonic()
            fps.tick()
            n_frames += 1
            with prof.section("record"):
                writer.write_image(t, frame)

            stop = False
            if preview and t - last_show >= 1.0 / PREVIEW_MAX_FPS:
                last_show = t
                with prof.section("show"):
                    disp, corners, ids = ad.detect_markers(detector, frame)
                    last_tags = 0 if ids is None else len(ids)
                    pose = None
                    idx = -1
                    if ids is not None and args.tag_id in ids.flatten():
                        idx = int(np.where(ids.flatten() == args.tag_id)[0][0])
                        pose = ad.tag_pose_field(corners[idx], obj, k, d, r_fc, t_fc, yaw_offset)
                    if idx >= 0:
                        cv2.aruco.drawDetectedMarkers(disp, [corners[idx]], np.array([[args.tag_id]]))
                        if pose is not None:
                            cv2.drawFrameAxes(disp, k, d, pose[3], pose[4], args.tag_size / 2.0)
                    if args.show_other_tags and ids is not None:
                        for j, tid in enumerate(int(t) for t in ids.flatten()):
                            if tid == args.tag_id:
                                continue
                            is_floor = tid in floor_ids
                            tag_obj = floor_obj if is_floor else obj
                            tag_size = args.floor_marker_size if is_floor else args.tag_size
                            tpose = ad.tag_pose_field(
                                corners[j], tag_obj, k, d, r_fc, t_fc, yaw_offset,
                            )
                            if tpose is None:
                                continue
                            cv2.aruco.drawDetectedMarkers(disp, [corners[j]], np.array([[tid]]))
                            cv2.drawFrameAxes(disp, k, d, tpose[3], tpose[4], tag_size / 2.0)
                            c = corners[j].reshape(-1, 2).mean(axis=0)
                            cv2.putText(
                                disp,
                                f"id{tid} x={tpose[0]:+.2f} y={tpose[1]:+.2f} "
                                f"yaw={math.degrees(tpose[2]):+.0f}",
                                (int(c[0]) - 40, int(c[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                (0, 255, 255), 1, cv2.LINE_AA,
                            )
                    status = (f"x={pose[0]:+.3f} y={pose[1]:+.3f} yaw={math.degrees(pose[2]):+6.1f}deg"
                              if pose is not None else "robot tag NOT visible")
                    draw_hud(disp, [
                        "TRACKING (recording images)",
                        status,
                        f"frame {n_frames}",
                        "[Q] stop",
                    ], ok=pose is not None)
                    draw_fps(disp, fps)
                    stop = show_scaled(disp) == ord("q")
            prof.frame(tags=last_tags)
            if stop:
                break
    finally:
        writer.close()
        source.release()
    if preview:
        cv2.destroyAllWindows()
    print(f"{n_frames} frames recorded ~{fps.fps:.0f} fps (min {fps.lo:.0f}). Recording: {args.out}")
    print(f"Solve poses with: python playground/calibration/analyze_apriltag_mcap.py {args.out}")


if __name__ == "__main__":
    main()
