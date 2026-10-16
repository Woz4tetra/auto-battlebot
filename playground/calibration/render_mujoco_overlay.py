"""Draw the fitted MuJoCo robot over a held-out sysid recording, in the camera's own view.

Every held-out window is rolled out the way ``fit_mujoco_plant.py validate`` scores it: the model
starts from the window's measured state and gets the logged motor commands for one second. Each
video frame inside a window then shows two robot silhouettes, both made from the collision hulls
the simulation uses and projected through the recording's camera intrinsics and its
field -> camera transform:

- blue: the measured robot (the smoothed track, with its pitch and roll)
- green: the MuJoCo robot at the same instant, with the full 6-DoF pose the simulator produced

Trails show where each has been since the window started. A red line joins the two axle centres.
The sim resets to the measured pose at every window start, so the green ghost only ever has one
second to drift.

Writes ``overlay.mp4`` (every window in order, at ``--speed`` of real time) and
``contact_sheet.png`` (best, median and worst windows by MuJoCo error, flat and nose-lift, each
cropped around the robot with ghosts every 0.25 s).

``--reset-on-stop`` drops the one-second windows and runs the model open loop through the whole
session instead, snapping it to the measured pose only once the measured robot has sat still for
``--stop-s`` (0.25 s). Each driving bout is then one rollout, as long as the driver kept moving.
That mode writes ``overlay_reset_on_stop.mp4`` and ``error_reset_on_stop.png`` (position error
against time, resets marked). Frames deep inside a stop are skipped.

``--closed-loop`` replays the logged sticks through the firmware mixer with the yaw loop fed the
sim's own heading and gyro (``validate.closed_loop_replay``, check 7), instead of the logged
per-motor commands. Outputs get a ``_closed_loop`` suffix. Use it for reverse driving, where the
logged heading-hold differential spins the open-loop sim.

    source scripts/activate_python.sh
    python playground/calibration/render_mujoco_overlay.py \\
        playground/calibration/out/mujoco_sysid/fit_2026-10-15_yolo_brake \\
        playground/calibration/out/mujoco_sysid/2026-10-15_00-20-58_yolo_box
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pandas as pd

from auto_battlebot.mujoco_sim import mass_properties, validate
from auto_battlebot.mujoco_sim.actuator import PlantParams
from auto_battlebot.mujoco_sim.fit import warp_simulator, window_errors
from auto_battlebot.mujoco_sim.mjcf import CollisionSet
from auto_battlebot.mujoco_sim.plots import _pick_windows
from auto_battlebot.mujoco_sim.rollout import WarpRollout, rest_height
from auto_battlebot.mujoco_sim.session import WindowBatch, build_batch, load_session
from auto_battlebot.recording.mcap_io import iter_video_frames
from auto_battlebot.recording.sysid_io import load_field_from_camera, load_robot_tags

# BGR, matching auto_battlebot/mujoco_sim/plots.py: MEAS #2a78d6, MUJOCO #1baf7a.
MEAS = (214, 120, 42)
MUJOCO = (122, 175, 27)
ERR = (52, 52, 220)
WHITE = (245, 245, 245)
INK = (12, 12, 12)
NOSE_M = np.array([0.11, 0.0, 0.0])  # body-frame nose for the heading tick
LEAD_S = 0.25  # measured-only frames shown before each window
FIX_NEAR_S = 0.05  # the measured pose counts as observed with a fix this close
TRAIL_S = 3.0  # trail length behind both robots
SKIP_STILL_S = 0.5  # reset-on-stop video: frames this far into a stop are left out


@dataclass
class StopRule:
    """The measured robot counts as stopped below both rates; a stop this long resets the sim."""

    hold_s: float = 0.25
    speed_mps: float = 0.05
    yaw_rate: float = 0.5


@dataclass
class Hulls:
    """Collision hull vertices in the body frame (FLU, origin at the axle centre)."""

    pieces: list[np.ndarray]

    @classmethod
    def load(cls, collision: CollisionSet) -> Hulls:
        pieces = []
        for path in collision.pieces:
            verts = [
                [float(t) for t in line.split()[1:4]]
                for line in path.read_text().splitlines()
                if line.startswith("v ")
            ]
            pieces.append(np.asarray(verts))
        return cls(pieces)


def rotation(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Body (FLU) to field, Z-Y-X; positive pitch is nose down, as in pose_from_qpos."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def quat_rotation(wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in wxyz)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


class Camera:
    def __init__(self, k: np.ndarray, cam_from_field: np.ndarray) -> None:
        self.k = k
        self.r = cam_from_field[:3, :3]
        self.t = cam_from_field[:3, 3]

    def project(self, pts: np.ndarray) -> np.ndarray:
        """Field points (N, 3) to pixels (N, 2); NaN behind the camera."""
        cam = np.asarray(pts, dtype=float) @ self.r.T + self.t
        out = np.full((len(cam), 2), np.nan)
        ok = cam[:, 2] > 0.05
        out[ok] = (cam[ok, :2] / cam[ok, 2:3]) @ self.k[:2, :2].T + self.k[:2, 2]
        return out


def draw_robot(
    img: np.ndarray,
    cam: Camera,
    hulls: Hulls,
    pos: np.ndarray,
    rot: np.ndarray,
    color: tuple[int, int, int],
    alpha: float = 0.35,
    outline: int = 2,
) -> None:
    mask = np.zeros(img.shape[:2], np.uint8)
    for piece in hulls.pieces:
        uv = cam.project(piece @ rot.T + pos)
        uv = uv[np.isfinite(uv).all(axis=1)]
        if len(uv) >= 3:
            cv2.fillConvexPoly(mask, cv2.convexHull(uv.astype(np.int32)), 255)
    if alpha > 0:
        tint = np.zeros_like(img)
        tint[:] = color
        sel = mask > 0
        img[sel] = (img[sel] * (1 - alpha) + tint[sel] * alpha).astype(np.uint8)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, color, outline, cv2.LINE_AA)
    nose = cam.project(np.stack([pos, NOSE_M @ rot.T + pos]))
    if np.isfinite(nose).all():
        p0, p1 = (tuple(int(v) for v in p) for p in nose)
        cv2.line(img, p0, p1, color, outline + 1, cv2.LINE_AA)


def draw_path(img: np.ndarray, cam: Camera, pts: np.ndarray, color, thickness: int = 2) -> None:
    uv = cam.project(pts)
    uv = uv[np.isfinite(uv).all(axis=1)]
    if len(uv) >= 2:
        cv2.polylines(img, [uv.astype(np.int32)], False, color, thickness, cv2.LINE_AA)


def text(img: np.ndarray, line: str, org: tuple[int, int], scale: float) -> None:
    cv2.putText(img, line, org, cv2.FONT_HERSHEY_SIMPLEX, scale, INK, 4, cv2.LINE_AA)
    cv2.putText(img, line, org, cv2.FONT_HERSHEY_SIMPLEX, scale, WHITE, 1, cv2.LINE_AA)


@dataclass
class Track:
    """The smoothed (measured) pose, interpolated to any tag-clock stamp."""

    stamp_ns: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    pitch: np.ndarray
    roll: np.ndarray
    fix_ns: np.ndarray  # every accepted tag or YOLO fix

    @classmethod
    def load(cls, bundle: Path) -> Track:
        df = pd.read_csv(bundle / "smoothed.csv")
        meas = pd.read_csv(bundle / "measurements.csv")
        if "rejected" in meas:
            meas = meas[~meas["rejected"].astype(bool)]
        return cls(
            df["stamp_ns"].to_numpy(np.int64),
            df["x"].to_numpy(),
            df["y"].to_numpy(),
            np.unwrap(df["yaw"].to_numpy()),
            df["pitch"].to_numpy(),
            df["roll"].to_numpy(),
            np.sort(meas["stamp_ns"].to_numpy(np.int64)),
        )

    def has_fix(self, t_ns: np.ndarray | int, within_s: float = FIX_NEAR_S) -> np.ndarray:
        """True where a real fix lies within ``within_s``; elsewhere the track is extrapolated."""
        t = np.atleast_1d(np.asarray(t_ns, dtype=np.int64))
        j = np.clip(np.searchsorted(self.fix_ns, t), 1, len(self.fix_ns) - 1)
        gap = np.minimum(np.abs(self.fix_ns[j] - t), np.abs(t - self.fix_ns[j - 1]))
        return gap <= within_s * 1e9

    def at(self, t_ns: np.ndarray | int) -> np.ndarray:
        """(N, 5) x, y, yaw, pitch, roll."""
        t = np.atleast_1d(np.asarray(t_ns, dtype=np.int64)).astype(float)
        s = self.stamp_ns.astype(float)
        return np.stack(
            [np.interp(t, s, v) for v in (self.x, self.y, self.yaw, self.pitch, self.roll)], 1
        )


def best_params(fit_dir: Path) -> tuple[PlantParams, dict[str, Any]]:
    doc = json.loads((fit_dir / "fit.json").read_text())
    run = min(doc["runs"], key=lambda r: r["loss"] if math.isfinite(r["loss"]) else math.inf)
    return PlantParams(**run["best"]), run


class Scene:
    """Everything a frame needs: windows, rollouts, the measured track and the camera."""

    def __init__(
        self,
        fit_dir: Path,
        bundle: Path,
        dt: float,
        horizon: float,
        stop: StopRule | None = None,
        closed_loop: bool = False,
    ) -> None:
        mp = mass_properties.load()
        collision = CollisionSet.load()
        self.hulls = Hulls.load(collision)
        session = json.loads((bundle / "session.json").read_text())
        self.mcap = Path(session["mcap"])
        cal = (session.get("yolo") or {}).get("calibration") or {}
        # Video stamps sit this far off the tag image clock (the YOLO calibration measures it).
        self.video_offset_ns = int(round(float(cal.get("offset_ms", 0.0)) * 1e6))
        t_up_field = np.asarray(session["output_frame"]["T_up_field"], dtype=float)

        self.params, self.run = best_params(fit_dir)
        session_data = load_session(bundle)
        self.still_since_ns: np.ndarray | None = None
        self.lead_ns = int(LEAD_S * 1e9)
        if stop is not None:
            session_data.windows, horizon, self.still_since_ns = stop_segments(
                session_data.smoothed, stop
            )
            self.lead_ns = 0
        self.batch: WindowBatch = build_batch(
            [session_data],
            horizon_s=horizon,
            dt=dt,
            track_half_width=mp.track_half_width,
            wheel_radius=mp.wheel_radius,
        )
        self.rest_z = rest_height(mp, collision, dt)
        self.closed_loop = closed_loop
        if closed_loop:
            self.qpos, self.record_dt = validate.closed_loop_replay(
                mp, collision, self.params, self.batch, self.rest_z
            )
        else:
            simulate = warp_simulator(
                lambda n, steps: WarpRollout(mp, collision, n, steps, timestep=dt)
            )
            self.qpos, self.record_dt = simulate(self.batch, [self.params] * len(self.batch))
        self.errors = window_errors(self.batch, self.qpos, self.record_dt)
        self.horizon_ns = int(round(horizon * 1e9))
        order = np.argsort(self.batch.start_ns)
        self.order = order
        self.starts = self.batch.start_ns[order]
        if stop is not None:
            # Each segment ends where the next reset starts.
            ends = np.r_[self.starts[1:], self.starts[-1] + self.horizon_ns]
            self.ends = np.empty_like(ends)
            self.ends[order] = ends
        else:
            self.ends = self.batch.start_ns + self.horizon_ns
        self.track = Track.load(bundle)

        tags = load_robot_tags(self.mcap)
        self.k = tags[0].camera.matrix
        self.size = (tags[0].camera.width, tags[0].camera.height)
        f2c = load_field_from_camera(self.mcap)
        self._f2c = f2c
        self._up_from_field = t_up_field

    def camera(self, t_ns: int) -> Camera:
        up_from_camera = self._up_from_field @ self._f2c.at(np.array([t_ns]))[0]
        return Camera(self.k, np.linalg.inv(up_from_camera))

    def window_at(self, t_ns: int) -> tuple[int, int] | None:
        """(window index, ns since its start) for the window covering t, lead-in included."""
        j = int(np.searchsorted(self.starts, t_ns + self.lead_ns, side="right")) - 1
        if j < 0:
            return None
        i = int(self.order[j])
        since = t_ns - int(self.batch.start_ns[i])
        if t_ns > self.ends[i]:
            return None
        return i, since

    def still_for_s(self, t_ns: int) -> float:
        """How long the measured robot has been stopped at t (0 when moving)."""
        if self.still_since_ns is None:
            return 0.0
        s = self.track.stamp_ns
        k = int(np.clip(np.searchsorted(s, t_ns, side="right") - 1, 0, len(s) - 1))
        since = self.still_since_ns[k]
        return 0.0 if since < 0 else (t_ns - since) / 1e9

    def sim_pose(self, i: int, since_ns: int) -> tuple[np.ndarray, np.ndarray]:
        k = min(int(round(since_ns / 1e9 / self.record_dt)), self.qpos.shape[1] - 1)
        q = self.qpos[i, max(k, 0)]
        return q[:3].copy(), quat_rotation(q[3:7])

    def sim_path(self, i: int, since_ns: int, from_ns: int = 0) -> np.ndarray:
        k = min(int(round(since_ns / 1e9 / self.record_dt)), self.qpos.shape[1] - 1)
        k0 = min(int(round(from_ns / 1e9 / self.record_dt)), max(k, 0))
        return self.qpos[i, k0 : max(k, 0) + 1, :3]

    def meas_pose(self, t_ns: int) -> tuple[np.ndarray, np.ndarray]:
        x, y, yaw, pitch, roll = self.track.at(t_ns)[0]
        return np.array([x, y, self.rest_z]), rotation(yaw, pitch, roll)

    def meas_path(self, t0_ns: int, t1_ns: int) -> np.ndarray:
        if t1_ns <= t0_ns:
            return np.zeros((0, 3))
        p = self.track.at(np.linspace(t0_ns, t1_ns, 60).astype(np.int64))
        return np.column_stack([p[:, 0], p[:, 1], np.full(len(p), self.rest_z)])


def stop_segments(smoothed: pd.DataFrame, stop: StopRule) -> tuple[pd.DataFrame, float, np.ndarray]:
    """Rollout segments for the reset-on-stop render, as a windows.csv-shaped table.

    A segment starts at the session start and wherever a stop reaches ``stop.hold_s``; the
    initial state is the smoothed one at that instant. Returns the table, the longest segment in
    seconds (every rollout runs that long), and per smoothed sample the stamp its current stop
    began at (-1 while moving).
    """
    t = smoothed["stamp_ns"].to_numpy(np.int64)
    speed = np.hypot(smoothed["vx"].to_numpy(), smoothed["vy"].to_numpy())
    still = (speed < stop.speed_mps) & (np.abs(smoothed["yaw_rate"].to_numpy()) < stop.yaw_rate)
    since = np.full(len(t), -1, dtype=np.int64)
    resets = [0]
    run_start = -1
    for k in range(len(t)):
        if not still[k]:
            run_start = -1
            continue
        if run_start < 0:
            run_start = k
        since[k] = t[run_start]
        if t[k] - t[run_start] >= stop.hold_s * 1e9 and (
            k == 0 or t[k - 1] - t[run_start] < stop.hold_s * 1e9
        ):
            resets.append(k)
    resets = sorted(set(resets))
    starts = t[resets]
    longest = float(np.max(np.diff(np.r_[starts, t[-1]]))) / 1e9
    rows = smoothed.iloc[resets]
    windows = pd.DataFrame(
        {
            "window_id": np.arange(len(resets)),
            "start_ns": starts,
            "kind": "flat",
            "x0": rows["x"].to_numpy(),
            "y0": rows["y"].to_numpy(),
            "yaw0": rows["yaw"].to_numpy(),
            "vx0": rows["vx"].to_numpy(),
            "vy0": rows["vy"].to_numpy(),
            "yaw_rate0": rows["yaw_rate"].to_numpy(),
            "pitch0": rows["pitch"].to_numpy(),
        }
    )
    return windows, longest, since


def render_frame(scene: Scene, img: np.ndarray, t_ns: int, i: int, since_ns: int) -> None:
    cam = scene.camera(t_ns)
    start = int(scene.batch.start_ns[i])
    trail_from = max(start - scene.lead_ns, t_ns - int(TRAIL_S * 1e9))
    m_pos, m_rot = scene.meas_pose(t_ns)
    draw_path(img, cam, scene.meas_path(trail_from, t_ns), MEAS, 2)
    draw_robot(img, cam, scene.hulls, m_pos, m_rot, MEAS)
    err_mm = float("nan")
    if since_ns >= 0:
        s_pos, s_rot = scene.sim_pose(i, since_ns)
        draw_path(img, cam, scene.sim_path(i, since_ns, max(trail_from - start, 0)), MUJOCO, 2)
        draw_robot(img, cam, scene.hulls, s_pos, s_rot, MUJOCO)
        link = cam.project(np.stack([m_pos, s_pos]))
        if np.isfinite(link).all():
            p0, p1 = (tuple(int(v) for v in p) for p in link)
            cv2.line(img, p0, p1, ERR, 1, cv2.LINE_AA)
        err_mm = float(np.hypot(*(s_pos[:2] - m_pos[:2]))) * 1e3

    scale = img.shape[1] / 1920.0
    b = scene.batch
    u = b.u_raw[i, b.pad + max(0, min(int(since_ns / 1e9 / b.dt), b.steps - 1))]
    status = (
        f"+{since_ns / 1e9:5.2f} s  pos err {err_mm:4.0f} mm"
        if since_ns >= 0
        else f"lead-in {since_ns / 1e9:+4.2f} s (measured only)"
    )
    loop = "closed loop (sticks -> firmware)" if scene.closed_loop else "open loop"
    if scene.still_since_ns is None:
        head = f"held-out window {b.window_id[i]} ({b.kind[i]})   {status}"
        legend = f"green = MuJoCo fit, {loop} from window start"
    else:
        head = f"segment {b.window_id[i]} of {len(b)}, since last reset {status}"
        legend = f"green = MuJoCo fit, {loop}, reset after a 0.25 s stop"
    fix = "" if scene.track.has_fix(t_ns)[0] else "   NO FIX: blue is extrapolated, error unscored"
    lines = [
        head + fix,
        f"motor cmd L {u[0]:+.2f}  R {u[1]:+.2f}",
        f"blue = measured (tag + YOLO, smoothed)   {legend}",
    ]
    for n, line in enumerate(lines):
        text(img, line, (int(20 * scale), int((40 + 34 * n) * scale)), 0.85 * scale)


def contact_sheet(scene: Scene, frames: dict[int, tuple[int, np.ndarray]], out: Path) -> None:
    """One panel per picked window: its last frame, ghosts every 0.25 s, cropped to the robot."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    picks = [i for i in _pick_windows(scene.batch, scene.errors, 6) if i in frames]
    if not picks:
        return
    cols = 3
    rows = int(math.ceil(len(picks) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5.2 * cols, 5.0 * rows), squeeze=False)
    for ax, i in zip(axes.flat, picks):
        t_ns, base = frames[i]
        img = base.copy()
        start = int(scene.batch.start_ns[i])
        cam = scene.camera(t_ns)
        ghosts = [0.0, 0.25, 0.5, 0.75, 1.0]
        corners = []
        for g in ghosts:
            since = int(g * 1e9)
            last = g == ghosts[-1]
            alpha, width = (0.35, 2) if last else (0.0, 1)
            m_pos, m_rot = scene.meas_pose(start + since)
            s_pos, s_rot = scene.sim_pose(i, since)
            draw_robot(img, cam, scene.hulls, m_pos, m_rot, MEAS, alpha, width)
            draw_robot(img, cam, scene.hulls, s_pos, s_rot, MUJOCO, alpha, width)
            corners += [m_pos, s_pos]
        draw_path(img, cam, scene.meas_path(start, start + scene.horizon_ns), MEAS, 2)
        draw_path(img, cam, scene.sim_path(i, scene.horizon_ns), MUJOCO, 2)
        uv = cam.project(np.asarray(corners))
        uv = uv[np.isfinite(uv).all(axis=1)]
        cx, cy = uv.mean(axis=0)
        half = max(float(np.ptp(uv[:, 0])), float(np.ptp(uv[:, 1])), 60.0) / 2 + 70
        x0, x1 = int(max(cx - half, 0)), int(min(cx + half, img.shape[1]))
        y0, y1 = int(max(cy - half, 0)), int(min(cy + half, img.shape[0]))
        ax.imshow(cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2RGB))
        ax.set_xticks([])
        ax.set_yticks([])
        e = scene.errors.position[i]
        errs = ", ".join(
            f"{h:.1f} s {1e3 * v:.0f} mm" for h, v in zip(scene.batch.horizons, e) if np.isfinite(v)
        )
        ax.set_title(
            f"{scene.batch.kind[i]} window {scene.batch.window_id[i]}\nMuJoCo error {errs}",
            loc="left",
            fontsize=9,
        )
    for ax in list(axes.flat)[len(picks) :]:
        ax.axis("off")
    fig.suptitle(
        "Held-out windows in the camera view: best, median and worst by MuJoCo error.\n"
        "Blue measured, green MuJoCo; outlines every 0.25 s, filled at 1.0 s",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def error_plot(scene: Scene, out: Path) -> None:
    """Reset-on-stop: MuJoCo position error against session time, and against time since reset."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t0 = int(scene.track.stamp_ns[0])
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7.5), gridspec_kw={"height_ratios": [3, 2]}, constrained_layout=True
    )
    since_all, err_all = [], []
    for i in range(len(scene.batch)):
        start = int(scene.batch.start_ns[i])
        n = int((scene.ends[i] - start) / 1e9 / scene.record_dt) + 1
        n = min(n, scene.qpos.shape[1])
        since = np.arange(n) * scene.record_dt
        stamps = start + (since * 1e9).astype(np.int64)
        meas = scene.track.at(stamps)
        err = np.hypot(scene.qpos[i, :n, 0] - meas[:, 0], scene.qpos[i, :n, 1] - meas[:, 1])
        err = np.where(scene.track.has_fix(stamps), err, np.nan)
        ax.plot((stamps - t0) / 1e9, err * 1e3, color="#1baf7a", lw=1)
        ax.axvline((start - t0) / 1e9, color="#c9c8c4", lw=0.6, zorder=0)
        since_all.append(since)
        err_all.append(err)
    ax.set_yscale("log")
    ax.set_ylim(1, None)
    ax.set_xlabel("session time, s")
    ax.set_ylabel("MuJoCo position error, mm")
    ax.set_title(
        f"{'Closed' if scene.closed_loop else 'Open'} loop between stops: {len(scene.batch)} "
        "resets (grey lines), each after the measured "
        "robot sat still for 0.25 s. Gaps: no tag or YOLO fix within 50 ms, so no error to score",
        loc="left",
        fontsize=10,
    )
    since_cat = np.concatenate(since_all)
    err_cat = np.concatenate(err_all) * 1e3
    keep = np.isfinite(err_cat)
    since_cat, err_cat = since_cat[keep], err_cat[keep]
    edges = np.arange(0, min(since_cat.max(), 10.0) + 0.25, 0.25)
    mid, p50, p90 = [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = (since_cat >= lo) & (since_cat < hi)
        if sel.sum() < 20:
            continue
        mid.append((lo + hi) / 2)
        p50.append(np.median(err_cat[sel]))
        p90.append(np.percentile(err_cat[sel], 90))
    ax2.plot(mid, p50, color="#1baf7a", lw=2, label="median")
    ax2.plot(mid, p90, color="#1baf7a", lw=1.2, ls="--", label="90th percentile")
    ax2.set_yscale("log")
    ax2.set_xlabel("time since last reset, s")
    ax2.set_ylabel("position error, mm")
    ax2.legend(frameon=False)
    ax2.grid(alpha=0.3)
    ax2.set_title("Error growth after a reset, all segments pooled", loc="left", fontsize=10)
    fig.savefig(out, dpi=130)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("fit_dir", type=Path, help="fit_mujoco_plant.py output (holds fit.json)")
    parser.add_argument("bundle", type=Path, help="held-out session bundle")
    parser.add_argument(
        "--out", type=Path, default=None, help="default: <fit_dir>/overlay_<bundle>"
    )
    parser.add_argument("--speed", type=float, default=0.5, help="playback speed of the video")
    parser.add_argument("--width", type=int, default=1280, help="output width, px")
    parser.add_argument("--dt", type=float, default=1e-3, help="MuJoCo timestep")
    parser.add_argument("--horizon", type=float, default=1.0)
    parser.add_argument("--crf", type=int, default=22)
    parser.add_argument(
        "--reset-on-stop", action="store_true", help="one rollout per driving bout, see above"
    )
    parser.add_argument("--stop-s", type=float, default=StopRule.hold_s)
    parser.add_argument("--stop-speed", type=float, default=StopRule.speed_mps, help="m/s")
    parser.add_argument("--stop-yaw-rate", type=float, default=StopRule.yaw_rate, help="rad/s")
    parser.add_argument(
        "--closed-loop", action="store_true", help="drive the sim from the sticks, see above"
    )
    args = parser.parse_args()

    out_dir = args.out or args.fit_dir / f"overlay_{args.bundle.name}"
    out_dir.mkdir(parents=True, exist_ok=True)
    stop = (
        StopRule(args.stop_s, args.stop_speed, args.stop_yaw_rate) if args.reset_on_stop else None
    )
    suffix = ("_reset_on_stop" if stop else "") + ("_closed_loop" if args.closed_loop else "")
    video_name = f"overlay{suffix}.mp4"
    scene = Scene(args.fit_dir, args.bundle, args.dt, args.horizon, stop, args.closed_loop)
    if stop:
        lengths = (scene.ends - scene.batch.start_ns) / 1e9
        print(
            f"{len(lengths)} segments between stops; length median {np.median(lengths):.1f} s, "
            f"p90 {np.percentile(lengths, 90):.1f} s, max {lengths.max():.1f} s",
            flush=True,
        )
    print(
        f"{len(scene.batch)} rollouts, {scene.run['zero_mode']} run, loss {scene.run['loss']:.2f}; "
        f"camera {scene.size[0]}x{scene.size[1]}, video offset "
        f"{scene.video_offset_ns / 1e6:+.1f} ms",
        flush=True,
    )
    picks = set() if stop else set(_pick_windows(scene.batch, scene.errors, 6))
    sheet_frames: dict[int, tuple[int, np.ndarray]] = {}

    encoder = None
    fps = 0.0
    written = 0
    last_stamp = None
    for stamp_ns, image in iter_video_frames(scene.mcap):
        if stamp_ns == 0:
            continue
        if last_stamp is not None and fps == 0.0 and stamp_ns > last_stamp:
            fps = 1e9 / (stamp_ns - last_stamp)
        last_stamp = stamp_ns
        t_ns = int(stamp_ns) + scene.video_offset_ns
        hit = scene.window_at(t_ns)
        if hit is None:
            continue
        i, since = hit
        if stop and scene.still_for_s(t_ns) > SKIP_STILL_S:
            continue
        if (image.shape[1], image.shape[0]) != scene.size:
            raise SystemExit(f"video is {image.shape[1]}x{image.shape[0]}, camera {scene.size}")
        if i in picks and since <= scene.horizon_ns:
            sheet_frames[i] = (t_ns, image.copy())
        render_frame(scene, image, t_ns, i, since)
        out_h = int(round(image.shape[0] * args.width / image.shape[1] / 2)) * 2
        frame = cv2.resize(image, (args.width, out_h), interpolation=cv2.INTER_AREA)
        if encoder is None:
            rate = max(fps, 1.0) * args.speed
            encoder = subprocess.Popen(
                [
                    "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                    "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{args.width}x{out_h}",
                    "-r", f"{rate:.4f}", "-i", "pipe:0",
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", str(args.crf),
                    "-pix_fmt", "yuv420p", str(out_dir / video_name),
                ],
                stdin=subprocess.PIPE,
            )  # fmt: skip
        assert encoder.stdin is not None
        encoder.stdin.write(frame.tobytes())
        written += 1
    if encoder is None or encoder.stdin is None:
        raise SystemExit("no video frame fell inside a window")
    encoder.stdin.close()
    if encoder.wait() != 0:
        raise SystemExit("ffmpeg failed")
    print(f"wrote {out_dir / video_name}: {written} frames at {args.speed}x", flush=True)
    if stop:
        error_plot(scene, out_dir / f"error{suffix}.png")
        print(f"wrote {out_dir / f'error{suffix}.png'}", flush=True)
    else:
        contact_sheet(scene, sheet_frames, out_dir / f"contact_sheet{suffix}.png")
        print(f"wrote {out_dir / f'contact_sheet{suffix}.png'}", flush=True)


if __name__ == "__main__":
    main()
