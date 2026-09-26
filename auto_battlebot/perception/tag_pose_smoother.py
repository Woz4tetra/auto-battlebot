"""Robot pose from the overhead AprilTag recordings: IPPE choice, body pose, RTS smoothing.

Implements "Robot pose recording" in ``docs/plans/mujoco_warp_mr_stabs_plan.md``. The input is
the ``/apriltag/robot_tags`` stream (every IPPE solution per detection) and the camera pose in the
field frame; the output is the axle-center pose on a uniform grid plus one row per detection.

Frames:

- **Recorded field frame.** Whatever ``/tf`` carries. ``FiducialFieldFilter`` puts field z into
  the floor, the homography fit puts it toward the camera.
- **Output frame ("field up").** The recorded field frame when its z points up. When it points
  into the floor the output is that frame turned 180 degrees about field x (y and z negated), so
  it is always right-handed with z up and a counter-clockwise turn seen from above is a positive
  yaw rate, the convention MuJoCo and the FLU body frame share. The camera's height sign decides.
- **Body frame.** FLU at the axle midpoint. Yaw is the heading of body x in the output frame.
  Pitch and roll are ZYX Euler angles, so a nose lift is a negative pitch. On an upside-down
  robot they are taken relative to the upside-down rest pose (body turned 180 degrees about its
  x axis), so both read near zero when it drives inverted.

Dependencies: numpy, pandas, scipy
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import pandas as pd
from scipy.stats import chi2

from auto_battlebot.compat import tomllib
from auto_battlebot.recording.esp32_clock import cross_correlation_delay, resample_linear
from auto_battlebot.recording.sysid_io import FieldFromCamera, RobotTagsFrame, rotation_angle

_NS = 1_000_000_000
_FLIP_X = np.diag([1.0, -1.0, -1.0, 1.0])  # 180 degrees about x: rest pose of an inverted body

KIND_NONE = 0
KIND_POSITION = 1
KIND_RATE = 2

OUTCOME_NONE = 0
OUTCOME_ACCEPTED = 1
OUTCOME_REJECTED = 2
OUTCOME_RESET = 3  # gated, but the filter had lost track, so accepted with inflated covariance


# ---------------------------------------------------------------------------
# Robot description
# ---------------------------------------------------------------------------


@dataclass
class TagMount:
    """Where one tag sits on the body. ``t_body_tag`` maps marker-frame points into the body."""

    tag_id: int
    t_body_tag: np.ndarray
    upside_down: bool  # True for the tag that faces the floor when the robot is upright

    size_m: float | None = None

    @property
    def t_tag_body(self) -> np.ndarray:
        return invert_pose(self.t_body_tag)


@dataclass
class RobotGeometry:
    track_half_width_m: float = 0.06526
    wheel_radius_m: float = 0.025


def invert_pose(t: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    r = t[:3, :3]
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ t[:3, 3]
    return out


def tag_mount_from_table(tag_id: int, table: dict[str, Any]) -> TagMount:
    """One ``[tags.N]`` table: ``translation_m`` and ``rotation`` (R_body_tag), or a 4x4
    ``t_body_tag``; ``upside_down`` defaults to the sign of the tag normal's body z."""
    if "t_body_tag" in table:
        t = np.asarray(table["t_body_tag"], dtype=np.float64).reshape(4, 4)
    else:
        t = np.eye(4)
        t[:3, :3] = np.asarray(table["rotation"], dtype=np.float64).reshape(3, 3)
        t[:3, 3] = np.asarray(table["translation_m"], dtype=np.float64).reshape(3)
    upside_down = bool(table.get("upside_down", t[2, 2] < 0.0))
    size = table.get("size_m")
    return TagMount(
        tag_id=tag_id,
        t_body_tag=t,
        upside_down=upside_down,
        size_m=None if size is None else float(size),
    )


def load_mass_properties(path: Path | str) -> tuple[dict[int, TagMount], RobotGeometry]:
    """Tag mounts and drivetrain geometry from ``mass_properties.toml``."""
    with open(path, "rb") as handle:
        data = tomllib.load(handle)
    mounts = {
        int(key): tag_mount_from_table(int(key), table)
        for key, table in data.get("tags", {}).items()
    }
    geometry_table = data.get("geometry", {})
    geometry = RobotGeometry(
        track_half_width_m=float(
            geometry_table.get("track_half_width_m", RobotGeometry.track_half_width_m)
        ),
        wheel_radius_m=float(geometry_table.get("wheel_radius_m", RobotGeometry.wheel_radius_m)),
    )
    return mounts, geometry


# ---------------------------------------------------------------------------
# Frames and angles
# ---------------------------------------------------------------------------


def up_from_field(field_from_camera: FieldFromCamera) -> tuple[np.ndarray, bool]:
    """(t_up_field, flipped). Flipped when the camera sits at negative field z, i.e. field z
    points into the floor."""
    if len(field_from_camera) == 0:
        raise ValueError("no camera -> field transform in the recording (run field init)")
    cam_z = float(np.median(field_from_camera.matrices[:, 2, 3]))
    if cam_z < 0.0:
        return _FLIP_X.copy(), True
    return np.eye(4), False


def rest_rotation(r_up_body: np.ndarray, upside_down: bool) -> np.ndarray:
    """Body rotation relative to its rest pose: the identity for an upright robot sitting flat."""
    return r_up_body @ _FLIP_X[:3, :3] if upside_down else r_up_body


def euler_zyx(r: np.ndarray) -> tuple[float, float, float]:
    """(yaw, pitch, roll) of a 3x3 rotation, ZYX order."""
    yaw = math.atan2(r[1, 0], r[0, 0])
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    roll = math.atan2(r[2, 1], r[2, 2])
    return yaw, pitch, roll


def attitude_error(r_up_body: np.ndarray, upside_down: bool, rest_pitch_rad: float) -> float:
    """How far a body attitude sits from resting flat on the floor, in radians.

    ``hypot(pitch - rest_pitch, roll)`` with pitch and roll taken relative to the rest pose
    (``rest_rotation``). The robot rests nose-down on its wedge tip, so the rest pitch is not
    zero (0.198 rad per CAD, ``collision.toml``).
    """
    _yaw, pitch, roll = euler_zyx(rest_rotation(r_up_body, upside_down))
    return math.hypot(pitch - rest_pitch_rad, roll)


def wrap_angle(a: np.ndarray | float) -> Any:
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# Detections -> candidate body poses -> IPPE choice
# ---------------------------------------------------------------------------


@dataclass
class Candidates:
    """Every robot tag detection with its (up to two) IPPE solutions as body poses."""

    stamp_ns: np.ndarray  # (N,) image stamps
    frame_index: np.ndarray  # (N,) index into the frame list
    tag_id: np.ndarray  # (N,)
    upside_down: np.ndarray  # (N,) bool
    n_solutions: np.ndarray  # (N,) 1 or 2
    t_up_body: np.ndarray  # (N, 2, 4, 4); slot 1 repeats slot 0 when there is one solution
    tilt: np.ndarray  # (N, 2) attitude error from rest, radians
    reprojection_error_px: np.ndarray  # (N, 2)
    size_px: np.ndarray  # (N,)

    def __len__(self) -> int:
        return int(self.stamp_ns.shape[0])


def build_candidates(
    frames: Sequence[RobotTagsFrame],
    field_from_camera: FieldFromCamera,
    mounts: dict[int, TagMount],
    t_up_field: np.ndarray,
    rest_pitch_rad: float = 0.0,
    rest_pitch_upside_down_rad: float = 0.0,
) -> Candidates:
    """Map every IPPE solution of every robot tag detection into a body pose in the up frame.

    Detections of ids not in ``mounts`` are skipped. When a frame carries more than one robot tag
    (never expected: the two sit on opposite faces) each becomes its own measurement.
    """
    stamps: list[int] = []
    frame_idx: list[int] = []
    ids: list[int] = []
    inverted: list[bool] = []
    n_sol: list[int] = []
    poses: list[np.ndarray] = []
    tilts: list[list[float]] = []
    errors: list[list[float]] = []
    sizes: list[float] = []
    frame_stamps = np.asarray([f.image_stamp_ns for f in frames], dtype=np.int64)
    field_t = field_from_camera.at(frame_stamps) if len(frames) else np.zeros((0, 4, 4))
    for fi, frame in enumerate(frames):
        up_from_camera = t_up_field @ field_t[fi]
        for det in frame.detections:
            mount = mounts.get(det.tag_id)
            if mount is None or not det.solutions:
                continue
            sols = det.solutions[:2]
            pair = []
            pair_tilt = []
            pair_err = []
            for sol in sols:
                t_up_body = up_from_camera @ sol.transform @ mount.t_tag_body
                pair.append(t_up_body)
                pair_tilt.append(
                    attitude_error(
                        t_up_body[:3, :3],
                        mount.upside_down,
                        rest_pitch_upside_down_rad if mount.upside_down else rest_pitch_rad,
                    )
                )
                pair_err.append(sol.reprojection_error_px)
            if len(sols) == 1:
                pair.append(pair[0])
                pair_tilt.append(pair_tilt[0])
                pair_err.append(pair_err[0])
            stamps.append(frame.image_stamp_ns)
            frame_idx.append(fi)
            ids.append(det.tag_id)
            inverted.append(mount.upside_down)
            n_sol.append(len(sols))
            poses.append(np.stack(pair))
            tilts.append(pair_tilt)
            errors.append(pair_err)
            sizes.append(det.size_px)
    return Candidates(
        stamp_ns=np.asarray(stamps, dtype=np.int64),
        frame_index=np.asarray(frame_idx, dtype=np.int64),
        tag_id=np.asarray(ids, dtype=np.int64),
        upside_down=np.asarray(inverted, dtype=bool),
        n_solutions=np.asarray(n_sol, dtype=np.int64),
        t_up_body=np.stack(poses) if poses else np.zeros((0, 2, 4, 4)),
        tilt=np.asarray(tilts, dtype=np.float64).reshape(-1, 2),
        reprojection_error_px=np.asarray(errors, dtype=np.float64).reshape(-1, 2),
        size_px=np.asarray(sizes, dtype=np.float64),
    )


@dataclass
class IppeChoice:
    choice: np.ndarray  # (N,) 0 or 1
    close_call: np.ndarray  # (N,) bool: tilts within the margin, decided by neighbours
    close_calls: int
    unresolved_close_calls: int  # close calls with no neighbour in reach, left to the tilt test


def select_ippe(
    cands: Candidates, close_margin_deg: float = 4.0, neighbour_window_s: float = 0.25
) -> IppeChoice:
    """Pick one IPPE solution per detection.

    The right solution leaves the body close to its rest attitude (resting nose-down on the
    wedge, upright or inverted), since the tag's own ~9.6 degree tilt comes out when its mount is
    removed; ``Candidates.tilt`` holds that attitude error. The solution with the smaller error
    wins. When the two are within ``close_margin_deg`` (tag facing the camera nearly head-on,
    or the body pitched away from rest while driving) the solution
    whose body rotation sits closest to the neighbouring choices wins instead: the nearest
    earlier decision (clear or already resolved) and the nearest later clear one, each within
    ``neighbour_window_s``.
    """
    n = len(cands)
    choice = np.argmin(cands.tilt, axis=1).astype(np.int64) if n else np.zeros(0, np.int64)
    margin = math.radians(close_margin_deg)
    close = (cands.n_solutions == 2) & (np.abs(cands.tilt[:, 0] - cands.tilt[:, 1]) < margin)
    window_ns = int(neighbour_window_s * _NS)
    clear_idx = np.flatnonzero(~close)
    unresolved = 0
    last_decided = -1
    for i in range(n):
        if not close[i]:
            last_decided = i
            continue
        refs: list[np.ndarray] = []
        if last_decided >= 0 and cands.stamp_ns[i] - cands.stamp_ns[last_decided] <= window_ns:
            refs.append(cands.t_up_body[last_decided, choice[last_decided], :3, :3])
        k = int(np.searchsorted(clear_idx, i, side="right"))
        if k < clear_idx.size and cands.stamp_ns[clear_idx[k]] - cands.stamp_ns[i] <= window_ns:
            j = int(clear_idx[k])
            refs.append(cands.t_up_body[j, choice[j], :3, :3])
        if not refs:
            unresolved += 1
            last_decided = i
            continue
        cost = [
            sum(rotation_angle(ref.T @ cands.t_up_body[i, s, :3, :3]) for ref in refs)
            for s in (0, 1)
        ]
        choice[i] = int(np.argmin(cost))
        last_decided = i
    return IppeChoice(
        choice=choice,
        close_call=close,
        close_calls=int(close.sum()),
        unresolved_close_calls=unresolved,
    )


@dataclass
class RawPoses:
    """The chosen body pose per detection, in the up frame."""

    stamp_ns: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    yaw: np.ndarray  # wrapped to [-pi, pi)
    pitch: np.ndarray
    roll: np.ndarray


def chosen_poses(cands: Candidates, ippe: IppeChoice) -> RawPoses:
    n = len(cands)
    out = np.zeros((n, 6))
    for i in range(n):
        t = cands.t_up_body[i, ippe.choice[i]]
        yaw, pitch, roll = euler_zyx(rest_rotation(t[:3, :3], bool(cands.upside_down[i])))
        out[i] = (t[0, 3], t[1, 3], t[2, 3], yaw, pitch, roll)
    return RawPoses(cands.stamp_ns.copy(), *[out[:, k].copy() for k in range(6)])


# ---------------------------------------------------------------------------
# Measurement noise
# ---------------------------------------------------------------------------


@dataclass
class NoiseModel:
    """Per-detection measurement noise: sigma = k * g, g = hypot(reprojection error, e0) / size.

    Position error from PnP grows with corner noise times distance over focal length, and the
    tag's pixel size is focal length times tag size over distance, so noise over size is the
    per-detection factor for the across-view position and the in-plane angle. Depth error grows
    with distance squared, so the along-view position gets a second ``size_ref_px / size``
    factor. The reprojection error stands in for corner noise and ``e0_px`` keeps a sharp
    detection from claiming zero noise.

    "Along" and "across" are the floor directions along and across the camera's horizontal view
    direction (``view_azimuth_rad`` in the output frame). A tilted camera measures depth several
    times worse than lateral position, so a single isotropic sigma would either drown the good
    axis or trust the bad one. The view direction to the robot changes across the box; using the
    optical axis for all of it is an approximation.
    """

    k_along: float = 0.4
    k_across: float = 0.07
    k_yaw: float = 0.5
    k_tilt: float = 0.8
    e0_px: float = 0.5
    size_ref_px: float = 50.0
    view_azimuth_rad: float = 0.0
    calibrated: bool = False
    still_segments: int = 0
    still_detections: int = 0

    def factor(self, reprojection_error_px: np.ndarray, size_px: np.ndarray) -> np.ndarray:
        err = np.hypot(np.asarray(reprojection_error_px, dtype=np.float64), self.e0_px)
        return np.asarray(err / np.maximum(np.asarray(size_px, dtype=np.float64), 1.0))

    def along_factor(self, reprojection_error_px: np.ndarray, size_px: np.ndarray) -> np.ndarray:
        size = np.maximum(np.asarray(size_px, dtype=np.float64), 1.0)
        return self.factor(reprojection_error_px, size_px) * (self.size_ref_px / size)

    def sigmas(
        self, reprojection_error_px: np.ndarray, size_px: np.ndarray
    ) -> dict[str, np.ndarray]:
        g = self.factor(reprojection_error_px, size_px)
        return {
            "along": self.k_along * self.along_factor(reprojection_error_px, size_px),
            "across": self.k_across * g,
            "yaw": self.k_yaw * g,
            "tilt": self.k_tilt * g,
        }

    def as_dict(self) -> dict[str, float | int | bool]:
        return {
            "k_along": self.k_along,
            "k_across": self.k_across,
            "k_yaw": self.k_yaw,
            "k_tilt": self.k_tilt,
            "e0_px": self.e0_px,
            "size_ref_px": self.size_ref_px,
            "view_azimuth_rad": self.view_azimuth_rad,
            "calibrated": self.calibrated,
            "still_segments": self.still_segments,
            "still_detections": self.still_detections,
        }


def view_azimuth(field_from_camera: FieldFromCamera, t_up_field: np.ndarray) -> float:
    """Heading of the camera's optical axis projected on the floor, in the output frame."""
    t_up_camera = t_up_field @ field_from_camera.matrices[len(field_from_camera) // 2]
    axis = t_up_camera[:3, 2]
    return math.atan2(float(axis[1]), float(axis[0]))


def to_view(x: np.ndarray, y: np.ndarray, azimuth: float) -> tuple[np.ndarray, np.ndarray]:
    """(along, across) components of field-frame (x, y)."""
    c, s = math.cos(azimuth), math.sin(azimuth)
    return c * x + s * y, -s * x + c * y


def from_view(
    along: np.ndarray, across: np.ndarray, azimuth: float
) -> tuple[np.ndarray, np.ndarray]:
    c, s = math.cos(azimuth), math.sin(azimuth)
    return c * along - s * across, s * along + c * across


def idle_mask(
    det_stamp_ns: np.ndarray,
    esp32_stamp_ns: np.ndarray,
    left_cmd: np.ndarray,
    right_cmd: np.ndarray,
    threshold_percent: float = 1.0,
    settle_s: float = 0.3,
) -> np.ndarray:
    """Per detection: inside the ESP32 stream, and both motor commands have been under
    ``threshold_percent`` for at least ``settle_s`` (so the robot has coasted to a stop)."""
    t = np.asarray(esp32_stamp_ns, dtype=np.int64)
    order = np.argsort(t, kind="stable")
    t = t[order]
    busy = (np.abs(np.asarray(left_cmd)[order]) > threshold_percent) | (
        np.abs(np.asarray(right_cmd)[order]) > threshold_percent
    )
    det = np.asarray(det_stamp_ns, dtype=np.int64)
    if t.size == 0:
        return np.zeros(det.shape, dtype=bool)
    busy_t = t[busy]
    idx = np.searchsorted(busy_t, det, side="right") - 1
    last_busy = np.where(idx >= 0, busy_t[np.clip(idx, 0, None)], t[0] - int(settle_s * _NS))
    inside = (det >= t[0]) & (det <= t[-1])
    return np.asarray(inside & ((det - last_busy) >= int(settle_s * _NS)))


def find_still_segments(
    stamp_ns: np.ndarray,
    along: np.ndarray,
    across: np.ndarray,
    yaw: np.ndarray,
    *,
    min_duration_s: float = 0.5,
    min_count: int = 10,
    pos_span_m: float = 0.03,
    yaw_span_rad: float = math.radians(5.0),
    idle: np.ndarray | None = None,
) -> list[tuple[int, int]]:
    """(first, last) detection index ranges where the raw pose stays inside a small box.

    Greedy: grow a run while the peak-to-peak spread of both position components and yaw stays
    under the limits, keep it if it lasts ``min_duration_s`` with ``min_count`` detections.
    ``idle`` (per detection, optional) additionally requires settled zero motor commands; with
    it the spread limits only catch the robot being pushed.
    """
    n = stamp_ns.shape[0]
    segments: list[tuple[int, int]] = []
    yaw_u = np.unwrap(yaw) if n else yaw
    ok = np.ones(n, dtype=bool) if idle is None else np.asarray(idle, dtype=bool)
    limits = np.array([pos_span_m, pos_span_m, yaw_span_rad])
    values = np.stack([along, across, yaw_u], axis=1) if n else np.zeros((0, 3))
    i = 0
    while i < n:
        if not ok[i]:
            i += 1
            continue
        lo = values[i].copy()
        hi = lo.copy()
        j = i
        while j + 1 < n and ok[j + 1]:
            nlo = np.minimum(lo, values[j + 1])
            nhi = np.maximum(hi, values[j + 1])
            if np.any(nhi - nlo > limits):
                break
            lo, hi = nlo, nhi
            j += 1
        if (stamp_ns[j] - stamp_ns[i]) / _NS >= min_duration_s and j - i + 1 >= min_count:
            segments.append((i, j))
            i = j + 1
        else:
            i += 1
    return segments


def calibrate_noise(
    raw: RawPoses,
    reprojection_error_px: np.ndarray,
    size_px: np.ndarray,
    segments: list[tuple[int, int]],
    azimuth: float,
) -> NoiseModel:
    """Fit the four gains to the scatter about each still segment's median.

    Without still segments the defaults stand (``calibrated`` false), which were measured on a
    synthetic 0.3 px corner-noise camera and are only a starting point.
    """
    model = NoiseModel(
        size_ref_px=float(np.median(size_px)) if size_px.size else 50.0,
        view_azimuth_rad=azimuth,
    )
    if not segments:
        return model
    along, across = to_view(raw.x, raw.y, azimuth)
    g = model.factor(reprojection_error_px, size_px)
    g_along = model.along_factor(reprojection_error_px, size_px)
    sums = dict.fromkeys(("along", "across", "yaw", "tilt"), 0.0)
    dof = dict.fromkeys(("along", "across", "yaw", "tilt"), 0)
    detections = 0
    for i0, i1 in segments:
        sl = slice(i0, i1 + 1)
        m = i1 - i0 + 1
        detections += m
        for name, values, gain, angular in (
            ("along", along[sl], g_along[sl], False),
            ("across", across[sl], g[sl], False),
            ("yaw", raw.yaw[sl], g[sl], True),
            ("tilt", raw.pitch[sl], g[sl], True),
            ("tilt", raw.roll[sl], g[sl], True),
        ):
            v = np.unwrap(values) if angular else values
            r = v - np.median(v)
            sums[name] += float(np.sum((r / gain) ** 2))
            dof[name] += m - 1
    k = {name: math.sqrt(sums[name] / max(dof[name], 1)) for name in sums}
    model.k_along = k["along"]
    model.k_across = k["across"]
    model.k_yaw = k["yaw"]
    model.k_tilt = k["tilt"]
    model.calibrated = True
    model.still_segments = len(segments)
    model.still_detections = detections
    return model


# ---------------------------------------------------------------------------
# Batched Kalman filter + RTS smoother, constant-acceleration model per axis
# ---------------------------------------------------------------------------


@dataclass
class Knots:
    """Measurement instants shared by every axis of one smoother.

    ``kind[k, a]`` says what axis ``a`` measures at knot ``k`` (nothing, its position, its rate);
    ``r`` is the measurement variance.
    """

    t_s: np.ndarray  # (K,) seconds from t0_ns, non-decreasing
    t0_ns: int
    kind: np.ndarray  # (K, A) int8
    z: np.ndarray  # (K, A)
    r: np.ndarray  # (K, A)

    @property
    def n_axes(self) -> int:
        return int(self.kind.shape[1])


@dataclass
class SmootherRun:
    """Forward filter and RTS pass over a batch of settings. Leading axis is the batch."""

    m_pred: np.ndarray  # (B, K, A, 3)
    p_pred: np.ndarray  # (B, K, A, 3, 3)
    m_filt: np.ndarray
    p_filt: np.ndarray
    m_smooth: np.ndarray
    p_smooth: np.ndarray
    outcome: np.ndarray  # (B, K) OUTCOME_*
    nis: np.ndarray  # (B, K) joint normalised innovation squared, NaN without measurement
    innovation: np.ndarray  # (B, K, A) normalised innovation per axis, NaN if unused


def transition(dt: float) -> np.ndarray:
    return np.array([[1.0, dt, 0.5 * dt * dt], [0.0, 1.0, dt], [0.0, 0.0, 1.0]])


def jerk_noise(dt: float) -> np.ndarray:
    """Discrete process noise of a white-jerk model per unit spectral density."""
    d2, d3 = dt * dt, dt * dt * dt
    return np.array(
        [
            [d3 * d2 / 20.0, d2 * d2 / 8.0, d3 / 6.0],
            [d2 * d2 / 8.0, d3 / 3.0, d2 / 2.0],
            [d3 / 6.0, d2 / 2.0, dt],
        ]
    )


@dataclass
class FilterOptions:
    angular: Sequence[bool]  # per axis: wrap the innovation to [-pi, pi)
    prior_std: Sequence[Sequence[float]]  # per axis: (position, rate, acceleration) std
    gate_probability: float = 0.9999
    gate: bool = True
    reacquire_s: float = 0.3  # accept a gated position fix after this long without one
    max_consecutive_rejects: int = 5
    branch_gap_s: float = 0.1  # see _ForwardState._innovation


def run_smoother(
    knots: Knots,
    q: np.ndarray,
    options: FilterOptions,
    active: np.ndarray | None = None,
    gate_exempt: np.ndarray | None = None,
) -> SmootherRun:
    """Forward Kalman filter with chi-square gating, then the RTS backward pass.

    ``q`` is (B, A): white-jerk spectral density per batch entry and axis. ``active`` (B, K, A)
    masks measurements out per batch entry (cross-validation hold-outs). Axes never interact
    except through the joint gate: a knot's measurements pass or fail together, against the
    chi-square quantile for as many degrees of freedom as axes measured.

    A gated position fix is still taken, with the prediction covariance inflated to the prior,
    once ``reacquire_s`` has passed without an accepted fix or after ``max_consecutive_rejects``
    rejections in a row. That covers the filter losing the robot across a long dropout, where
    every later fix would otherwise fail the gate. The inflation enters ``p_pred``, so the RTS
    pass sees it as extra process noise on that step and stays consistent.

    ``gate_exempt`` (K,) lets knots through the gate unconditionally; ``smooth_with_recheck``
    uses it for fixes the smoothed track vouches for.
    """
    q = np.atleast_2d(np.asarray(q, dtype=np.float64))
    b_count, a_count = q.shape
    k_count = knots.t_s.shape[0]
    if a_count != knots.n_axes:
        raise ValueError("q has a different axis count than the knots")
    has = knots.kind != KIND_NONE  # (K, A) any measurement
    if active is None:
        use = np.broadcast_to(has, (b_count, k_count, a_count)).copy()
    else:
        use = np.asarray(active, dtype=bool) & has[None]
    state = _ForwardState(knots, options, b_count)
    if gate_exempt is not None:
        state.exempt = np.asarray(gate_exempt, dtype=bool)
    for k in range(k_count):
        state.predict(k, q)
        state.store_prediction(k)
        if use[:, k, :].any():
            state.update(k, use[:, k, :])
        state.store_filtered(k)
    m_s, p_s = _rts_backward(knots, state.m_pred, state.p_pred, state.m_filt, state.p_filt)
    return SmootherRun(
        state.m_pred,
        state.p_pred,
        state.m_filt,
        state.p_filt,
        m_s,
        p_s,
        state.outcome,
        state.nis_out,
        state.innov_out,
    )


class _ForwardState:
    """The forward filter's running mean and covariance for every batch entry, plus its logs."""

    def __init__(self, knots: Knots, options: FilterOptions, b_count: int) -> None:
        a_count = knots.n_axes
        k_count = knots.t_s.shape[0]
        self.knots = knots
        self.options = options
        self.angular = np.asarray(options.angular, dtype=bool)
        prior = np.asarray(options.prior_std, dtype=np.float64).reshape(a_count, 3)
        self.prior_cov = np.stack([np.diag(prior[a] ** 2) for a in range(a_count)])
        self.thresholds = np.array(
            [np.inf] + [float(chi2.ppf(options.gate_probability, d)) for d in range(1, a_count + 1)]
        )
        self.h_rows = np.zeros((3, 3))
        self.h_rows[KIND_POSITION, 0] = 1.0
        self.h_rows[KIND_RATE, 1] = 1.0

        self.m_pred = np.zeros((b_count, k_count, a_count, 3))
        self.p_pred = np.zeros((b_count, k_count, a_count, 3, 3))
        self.m_filt = np.zeros_like(self.m_pred)
        self.p_filt = np.zeros_like(self.p_pred)
        self.outcome = np.zeros((b_count, k_count), dtype=np.int8)
        self.nis_out = np.full((b_count, k_count), np.nan)
        self.innov_out = np.full((b_count, k_count, a_count), np.nan)

        # Start from the first position measurement of each axis.
        self.m = np.zeros((b_count, a_count, 3))
        for a in range(a_count):
            first = np.flatnonzero(knots.kind[:, a] == KIND_POSITION)
            if first.size:
                self.m[:, a, 0] = knots.z[first[0], a]
        self.cov = np.broadcast_to(self.prior_cov, (b_count, a_count, 3, 3)).copy()
        self.last_fix = np.full(b_count, -np.inf)
        self.rejects_in_row = np.zeros(b_count, dtype=np.int64)
        self.prev_t = float(knots.t_s[0]) if k_count else 0.0
        self.exempt = np.zeros(k_count, dtype=bool)
        # Angle and rate at each axis's last accepted position fix, for the branch reference.
        self.fix_state = self.m[:, :, :2].copy()
        self.fix_t = np.full((b_count, a_count), -np.inf)

    def predict(self, k: int, q: np.ndarray) -> None:
        t = float(self.knots.t_s[k])
        dt = t - self.prev_t
        self.prev_t = t
        if dt > 0.0:
            f = transition(dt)
            self.m = np.einsum("ij,baj->bai", f, self.m)
            self.cov = f @ self.cov @ f.T + q[:, :, None, None] * jerk_noise(dt)

    def store_prediction(self, k: int) -> None:
        self.m_pred[:, k] = self.m
        self.p_pred[:, k] = self.cov

    def store_filtered(self, k: int) -> None:
        self.m_filt[:, k] = self.m
        self.p_filt[:, k] = self.cov

    def _innovation(self, k: int, h: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Residual, H P and innovation variance.

        An angle measurement is unwrapped onto the branch nearest a reference: the prediction
        normally, but after a gap of more than ``branch_gap_s`` since the axis's last fix, the
        last fixed angle carried forward at its fixed rate. Across a dropout mid-spin the
        acceleration state is too noisy to pick the branch; at 8 rad/s a 0.3 s gap already
        leaves a whole turn to chance.
        """
        t = float(self.knots.t_s[k])
        hx = np.einsum("aj,baj->ba", h, self.m)
        z = np.broadcast_to(self.knots.z[k][None, :], hx.shape)
        seen = np.isfinite(self.fix_t)
        gap = np.where(seen, t - np.where(seen, self.fix_t, 0.0), 0.0)
        carried = self.fix_state[:, :, 0] + self.fix_state[:, :, 1] * gap
        ref = np.where(gap > self.options.branch_gap_s, carried, hx)
        angle = self.angular[None, :] & (self.knots.kind[k] == KIND_POSITION)[None, :]
        resid = np.where(angle, ref + wrap_angle(z - ref) - hx, z - hx)
        hp = np.einsum("aj,bajk->bak", h, self.cov)  # (B, A, 3)
        s = np.einsum("bak,ak->ba", hp, h) + self.knots.r[k][None, :]
        return resid, hp, s

    def update(self, k: int, u: np.ndarray) -> None:
        """Gate and apply knot ``k``'s measurements; ``u`` (B, A) says which axes are in use."""
        kind_k = self.knots.kind[k]
        h = self.h_rows[kind_k]  # (A, 3)
        t = float(self.knots.t_s[k])
        resid, hp, s = self._innovation(k, h)
        nis = np.where(u, resid * resid / s, 0.0).sum(axis=1)
        dof = u.sum(axis=1)
        measured = dof > 0
        passed = (nis <= self.thresholds[dof]) if self.options.gate else measured.copy()
        if self.exempt[k]:
            passed = measured.copy()
        is_position = bool(np.any(kind_k == KIND_POSITION))
        reset = np.zeros_like(measured)
        if is_position:
            lost = (t - self.last_fix > self.options.reacquire_s) | (
                self.rejects_in_row >= self.options.max_consecutive_rejects
            )
            reset = measured & ~passed & lost
        if reset.any():
            inflate = np.where(u[reset][:, :, None, None], self.prior_cov[None], 0.0)
            self.cov[reset] = self.cov[reset] + inflate
            self.p_pred[:, k] = self.cov
            resid, hp, s = self._innovation(k, h)
        take = measured & (passed | reset)
        ut = u & take[:, None]
        gain = hp / s[:, :, None]  # (B, A, 3)
        self.m = self.m + np.where(ut[:, :, None], gain * resid[:, :, None], 0.0)
        self.cov = self.cov - np.where(
            ut[:, :, None, None], gain[:, :, :, None] * hp[:, :, None, :], 0.0
        )
        self.nis_out[measured, k] = nis[measured]
        self.innov_out[:, k] = np.where(u, resid / np.sqrt(s), np.nan)
        self.outcome[measured & passed, k] = OUTCOME_ACCEPTED
        self.outcome[measured & ~passed & ~reset, k] = OUTCOME_REJECTED
        self.outcome[reset, k] = OUTCOME_RESET
        if is_position:
            self.last_fix[take] = t
            self.rejects_in_row[take] = 0
            self.rejects_in_row[measured & ~take] += 1
            fixed = ut & (kind_k == KIND_POSITION)[None, :]
            self.fix_state[fixed] = self.m[fixed][:, :2]
            self.fix_t[fixed] = t


def _rts_backward(
    knots: Knots,
    m_pred: np.ndarray,
    p_pred: np.ndarray,
    m_filt: np.ndarray,
    p_filt: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    m_s = m_filt.copy()
    p_s = p_filt.copy()
    for k in range(knots.t_s.shape[0] - 2, -1, -1):
        f = transition(float(knots.t_s[k + 1] - knots.t_s[k]))
        gain = p_filt[:, k] @ f.T @ np.linalg.inv(p_pred[:, k + 1])
        m_s[:, k] = m_filt[:, k] + np.einsum(
            "baij,baj->bai", gain, m_s[:, k + 1] - m_pred[:, k + 1]
        )
        p_s[:, k] = p_filt[:, k] + gain @ (p_s[:, k + 1] - p_pred[:, k + 1]) @ np.swapaxes(
            gain, -1, -2
        )
    return m_s, p_s


def smooth_with_recheck(
    knots: Knots, q: np.ndarray, options: FilterOptions, max_passes: int = 3
) -> tuple[SmootherRun, np.ndarray]:
    """``run_smoother``, then give gated position fixes a second look against the smoothed track.

    The forward gate sees only the past, so a hard reversal or punch that the process noise
    under-predicts fails it although the fix is good. The smoothed state at a rejected knot
    already excludes that fix and knows what came next, so a fix within the same chi-square gate
    of it (smoothed covariance plus measurement noise) is let through on the next pass. An
    outlier stays far from the smoothed track and stays rejected. Returns the final run and the
    exempted knots.
    """
    q = np.atleast_2d(np.asarray(q, dtype=np.float64))
    exempt = np.zeros(knots.t_s.shape[0], dtype=bool)
    angular = np.asarray(options.angular, dtype=bool)
    run = run_smoother(knots, q, options, gate_exempt=exempt)
    for _ in range(max_passes - 1):
        rejected = run.outcome[0] == OUTCOME_REJECTED
        rejected &= np.any(knots.kind == KIND_POSITION, axis=1)
        if not rejected.any():
            break
        idx = np.flatnonzero(rejected)
        pos = knots.kind[idx] == KIND_POSITION  # (R, A)
        resid = knots.z[idx] - run.m_smooth[0, idx, :, 0]
        resid = np.where(angular[None, :], wrap_angle(resid), resid)
        var = run.p_smooth[0, idx, :, 0, 0] + knots.r[idx]
        nis = np.where(pos, resid * resid / var, 0.0).sum(axis=1)
        dof = pos.sum(axis=1)
        limit = np.array([float(chi2.ppf(options.gate_probability, d)) for d in dof])
        vouched = idx[nis <= limit]
        if vouched.size == 0:
            break
        exempt[vouched] = True
        run = run_smoother(knots, q, options, gate_exempt=exempt)
    return run, exempt


def interpolate_smoothed(
    run: SmootherRun, knots: Knots, q: np.ndarray, grid_ns: np.ndarray, batch: int = 0
) -> tuple[np.ndarray, np.ndarray]:
    """Smoothed mean (G, A, 3) and covariance (G, A, 3, 3) at arbitrary times.

    Exact for the model: predict from the filtered state at the knot before, then apply the
    RTS gain against the smoothed and predicted states at the knot after. Times outside the
    knot span clamp to the end knots.
    """
    t = (np.asarray(grid_ns, dtype=np.int64) - knots.t0_ns).astype(np.float64) / _NS
    kt = knots.t_s
    k_count = kt.shape[0]
    qa = np.asarray(q, dtype=np.float64).reshape(-1)
    idx = np.clip(np.searchsorted(kt, t, side="right") - 1, 0, k_count - 1)
    tau = np.clip(t - kt[idx], 0.0, None)
    nxt = np.minimum(idx + 1, k_count - 1)
    last = idx == k_count - 1
    tau = np.where(last, 0.0, tau)

    mf = run.m_filt[batch][idx]  # (G, A, 3)
    pf = run.p_filt[batch][idx]
    f_tau = np.zeros((t.shape[0], 3, 3))
    q_tau = np.zeros((t.shape[0], 3, 3))
    for g, tt in enumerate(tau):
        f_tau[g] = transition(float(tt))
        q_tau[g] = jerk_noise(float(tt))
    m_t = np.einsum("gij,gaj->gai", f_tau, mf)
    p_t = f_tau[:, None] @ pf @ np.swapaxes(f_tau, -1, -2)[:, None] + (
        qa[None, :, None, None] * q_tau[:, None]
    )
    dt_rest = np.where(last, 0.0, kt[nxt] - kt[idx] - tau)
    f_rest = np.zeros_like(f_tau)
    for g, tt in enumerate(dt_rest):
        f_rest[g] = transition(float(tt))
    gain = p_t @ np.swapaxes(f_rest, -1, -2)[:, None] @ np.linalg.inv(run.p_pred[batch][nxt])
    diff_m = run.m_smooth[batch][nxt] - run.m_pred[batch][nxt]
    diff_p = run.p_smooth[batch][nxt] - run.p_pred[batch][nxt]
    m_out = m_t + np.einsum("gaij,gaj->gai", gain, diff_m)
    p_out = p_t + gain @ diff_p @ np.swapaxes(gain, -1, -2)
    # At the last knot the smoothed state is the filtered one; take it directly.
    m_out[last] = run.m_smooth[batch][idx[last]]
    p_out[last] = run.p_smooth[batch][idx[last]]
    return m_out, p_out


# ---------------------------------------------------------------------------
# Cross-validated process noise
# ---------------------------------------------------------------------------


@dataclass
class CrossValidation:
    groups: dict[str, list[int]]  # group name -> axes sharing one q
    candidates: dict[str, np.ndarray]  # group -> q values tried
    score: dict[str, np.ndarray]  # group -> mean clipped squared normalised held-out error
    chosen: dict[str, float]
    held_out: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "held_out_detections": self.held_out,
            "chosen_q": self.chosen,
            "curve": {
                g: {"q": self.candidates[g].tolist(), "score": self.score[g].tolist()}
                for g in self.groups
            },
        }


def cross_validate_q(
    knots: Knots,
    groups: dict[str, list[int]],
    candidates: dict[str, np.ndarray],
    options: FilterOptions,
    hold_every: int = 5,
    clip: float = 25.0,
) -> CrossValidation:
    """Pick each group's process noise by hold-out: drop every ``hold_every``-th position fix,
    smooth the rest once per candidate, score the smoothed state against the dropped fixes.

    All groups step through their candidate lists together in one batch (entry b uses the b-th
    candidate of every group), and each group is scored on its own axes. The groups are
    independent in the model, so this equals a separate sweep per group except for the joint
    gate. The score is the mean squared held-out residual over its variance, clipped at
    ``clip`` so an outlier fix cannot choose q.
    """
    n_cand = {len(v) for v in candidates.values()}
    if len(n_cand) != 1:
        raise ValueError("every group needs the same number of candidates")
    b_count = n_cand.pop()
    q = np.zeros((b_count, knots.n_axes))
    for g, axes in groups.items():
        for a in axes:
            q[:, a] = candidates[g]
    position_knots = np.flatnonzero(np.any(knots.kind == KIND_POSITION, axis=1))
    held = position_knots[::hold_every]
    active = np.ones((b_count, knots.t_s.shape[0], knots.n_axes), dtype=bool)
    active[:, held, :] = False
    run = run_smoother(knots, q, options, active=active)
    score: dict[str, np.ndarray] = {}
    chosen: dict[str, float] = {}
    for g, axes in groups.items():
        per = np.zeros(b_count)
        for b in range(b_count):
            vals = []
            for a in axes:
                ok = knots.kind[held, a] == KIND_POSITION
                kk = held[ok]
                resid = knots.z[kk, a] - run.m_smooth[b, kk, a, 0]
                if options.angular[a]:
                    resid = wrap_angle(resid)
                vals.append(np.minimum(resid * resid / knots.r[kk, a], clip))
            per[b] = float(np.mean(np.concatenate(vals))) if vals else math.nan
        score[g] = per
        chosen[g] = float(candidates[g][int(np.nanargmin(per))])
    return CrossValidation(groups, candidates, score, chosen, int(held.size))


# ---------------------------------------------------------------------------
# BNO055 yaw rate
# ---------------------------------------------------------------------------


@dataclass
class ImuYawRate:
    stamp_ns: np.ndarray  # uniform grid on the app clock
    rate: np.ndarray  # rad/s, counter-clockwise positive under the assumed heading sign
    valid: np.ndarray


def bno055_yaw_rate(
    stamp_ns: np.ndarray,
    heading_deg: np.ndarray,
    upside_down: np.ndarray,
    grid_dt_s: float = 0.01,
    heading_sign: float = -1.0,
) -> ImuYawRate:
    """Yaw rate from the BNO055 Euler heading (``orientation_x``).

    Assumption, not yet checked on the robot: the heading is compass-style, in degrees on
    [0, 360) and increasing clockwise seen from above with the sensor upright, so the
    counter-clockwise yaw rate is ``-d(heading)/dt`` (``heading_sign = -1``). The session checks
    measure the sign against tag yaw rate and report it. Samples logged while the firmware says
    the robot is upside down are marked invalid, since the sensor's sense of rotation flips.

    The heading updates at 100 Hz with 1/16 degree steps, so it is resampled onto a 10 ms grid
    and differentiated over five samples; a per-loop difference would be mostly quantisation.
    """
    t = np.asarray(stamp_ns, dtype=np.int64)
    if t.size < 5:
        return ImuYawRate(np.zeros(0, np.int64), np.zeros(0), np.zeros(0, bool))
    order = np.argsort(t, kind="stable")
    t = t[order]
    heading = np.unwrap(np.radians(np.asarray(heading_deg, dtype=np.float64)[order]))
    inverted = np.asarray(upside_down, dtype=bool)[order]
    step = int(round(grid_dt_s * _NS))
    grid = np.arange(int(t[0]), int(t[-1]), step, dtype=np.int64)
    h = resample_linear(t, heading, grid, max_gap_s=0.1)
    span = 2
    rate = np.full(grid.shape, np.nan)
    if grid.size > 2 * span:
        rate[span:-span] = (h[2 * span :] - h[: -2 * span]) / (2 * span * grid_dt_s)
    rate *= heading_sign
    inv = np.searchsorted(t, grid, side="right") - 1
    bad = inverted[np.clip(inv, 0, t.size - 1)]
    valid = np.isfinite(rate) & ~bad
    return ImuYawRate(grid, rate, valid)


# ---------------------------------------------------------------------------
# Session pipeline
# ---------------------------------------------------------------------------


@dataclass
class SmootherOptions:
    grid_dt_s: float = 0.005
    # Resting pitch, nose-down on the wedge tip (CAD, collision.toml rest_pitch_rad). The
    # inverted rest pitch is unmeasured; 0 until a session shows otherwise.
    rest_pitch_rad: float = 0.198
    rest_pitch_upside_down_rad: float = 0.0
    close_margin_deg: float = 4.0
    neighbour_window_s: float = 0.25
    gate_probability: float = 0.9999
    reacquire_s: float = 0.3
    hold_every: int = 5
    q_xy: np.ndarray = field(default_factory=lambda: np.logspace(0.0, 6.0, 13))
    q_yaw: np.ndarray = field(default_factory=lambda: np.logspace(1.0, 7.0, 13))
    q_tilt: np.ndarray = field(default_factory=lambda: np.logspace(0.0, 6.0, 13))
    fuse_imu: bool = False
    imu_min_correlation: float = 0.7
    still_yaw_rate: float = 0.15  # rad/s: "not turning" for the IMU rate noise
    spin_rate: float = 3.0  # rad/s: "a spin" for the IMU agreement check


@dataclass
class SessionSmoothing:
    measurements: pd.DataFrame  # one row per detection (measurements.csv plus extras)
    grid: pd.DataFrame  # smoothed.csv columns plus pitch/roll rates
    frame_stamps_ns: np.ndarray
    t_up_field: np.ndarray
    field_flipped: bool
    noise: NoiseModel
    cv_planar: CrossValidation
    cv_tilt: CrossValidation
    ippe: IppeChoice
    rejections: pd.DataFrame
    checks: dict[str, Any]
    imu: ImuYawRate | None
    imu_lag: dict[str, Any] | None
    residual_acf: np.ndarray
    innovation_acf: np.ndarray
    still_segments: list[tuple[int, int]]  # detection index ranges the noise model came from


@dataclass
class _AxisFit:
    """One smoother's knots, run and chosen process noise."""

    knots: Knots
    src: np.ndarray  # per knot: detection index, -1 for an IMU knot
    run: SmootherRun
    q: np.ndarray  # (1, A)
    exempt: np.ndarray  # knots let through the gate by the smoothed re-check

    def det_knot(self, n_detections: int) -> np.ndarray:
        out = np.full(n_detections, -1, dtype=np.int64)
        out[self.src[self.src >= 0]] = np.flatnonzero(self.src >= 0)
        return out


_PLANAR_OPTIONS = {
    "angular": (False, False, True),
    "prior_std": ((1.0, 5.0, 50.0), (1.0, 5.0, 50.0), (math.pi, 40.0, 1000.0)),
}
_TILT_OPTIONS = {"angular": (True, True), "prior_std": ((0.5, 10.0, 300.0), (0.5, 10.0, 300.0))}


def _filter_options(kind: dict[str, Any], opts: SmootherOptions) -> FilterOptions:
    return FilterOptions(
        angular=kind["angular"],
        prior_std=kind["prior_std"],
        gate_probability=opts.gate_probability,
        reacquire_s=opts.reacquire_s,
    )


def _planar_knots(
    stamp_ns: np.ndarray,
    z: np.ndarray,
    sigma: np.ndarray,
    t0_ns: int,
    imu: ImuYawRate | None = None,
    imu_sigma: float = 0.1,
) -> tuple[Knots, np.ndarray]:
    """Knots for (along, across, yaw), detections plus optional IMU yaw-rate knots.

    ``z`` and ``sigma`` are (N, 3). Returns the knots with, per knot, the detection index (-1
    for IMU).
    """
    n = stamp_ns.shape[0]
    stamps = stamp_ns
    src = np.arange(n)
    kind = np.full((n, 3), KIND_POSITION, dtype=np.int8)
    r = sigma**2
    if imu is not None and np.any(imu.valid):
        it = imu.stamp_ns[imu.valid]
        ik = np.zeros((it.size, 3), dtype=np.int8)
        ik[:, 2] = KIND_RATE
        iz = np.zeros((it.size, 3))
        iz[:, 2] = imu.rate[imu.valid]
        ir = np.ones((it.size, 3))
        ir[:, 2] = imu_sigma**2
        stamps = np.concatenate([stamps, it])
        src = np.concatenate([src, np.full(it.size, -1)])
        kind = np.concatenate([kind, ik])
        z = np.concatenate([z, iz])
        r = np.concatenate([r, ir])
    order = np.argsort(stamps, kind="stable")
    knots = Knots(
        t_s=(stamps[order] - t0_ns).astype(np.float64) / _NS,
        t0_ns=t0_ns,
        kind=kind[order],
        z=z[order],
        r=r[order],
    )
    return knots, src[order]


def _fit_axes(
    knots: Knots,
    src: np.ndarray,
    groups: dict[str, list[int]],
    candidates: dict[str, np.ndarray],
    fopts: FilterOptions,
    hold_every: int,
) -> tuple[_AxisFit, CrossValidation]:
    cv = cross_validate_q(knots, groups, candidates, fopts, hold_every=hold_every)
    q = np.zeros((1, knots.n_axes))
    for g, axes in groups.items():
        q[0, axes] = cv.chosen[g]
    run, exempt = smooth_with_recheck(knots, q, fopts)
    return _AxisFit(knots, src, run, q, exempt), cv


def autocorrelation(x: np.ndarray, max_lag: int) -> np.ndarray:
    """Normalised autocorrelation at lags 0..max_lag, ignoring NaN."""
    v = np.asarray(x, dtype=np.float64)
    v = v[np.isfinite(v)]
    out = np.full(max_lag + 1, np.nan)
    if v.size < max_lag + 2:
        return out
    v = v - v.mean()
    denom = float(v @ v)
    if denom <= 0.0:
        return out
    for lag in range(max_lag + 1):
        out[lag] = float(v[: v.size - lag] @ v[lag:]) / denom
    return out


def _whiteness(acf: np.ndarray, n: int) -> dict[str, Any]:
    if n == 0 or not np.isfinite(acf).any():
        return {"samples": n}
    bound = 1.96 / math.sqrt(max(n, 1))
    lags = acf[1:]
    return {
        "samples": n,
        "lag1": float(acf[1]),
        "lag2": float(acf[2]),
        "lag5": float(acf[5]) if acf.size > 5 else math.nan,
        "bound_95": bound,
        "fraction_outside_bound": float(np.mean(np.abs(lags) > bound)),
    }


def smooth_session(
    frames: Sequence[RobotTagsFrame],
    field_from_camera: FieldFromCamera,
    mounts: dict[int, TagMount],
    options: SmootherOptions | None = None,
    esp32: pd.DataFrame | None = None,
) -> SessionSmoothing:
    """The whole "Robot pose recording" pass for one session.

    ``esp32`` (optional) is the ESP32 event table with ``stamp_ns`` already on the app clock
    (``esp32_clock.fit_robot_clock``). It picks still stretches (settled zero commands) for the
    noise calibration, feeds the IMU agreement check and, with ``options.fuse_imu``, the
    yaw-rate fusion.
    """
    opts = options or SmootherOptions()
    t_up_field, flipped = up_from_field(field_from_camera)
    cands = build_candidates(
        frames,
        field_from_camera,
        mounts,
        t_up_field,
        opts.rest_pitch_rad,
        opts.rest_pitch_upside_down_rad,
    )
    if len(cands) < 10:
        raise ValueError(f"only {len(cands)} robot tag detections; nothing to smooth")
    ippe = select_ippe(cands, opts.close_margin_deg, opts.neighbour_window_s)
    raw = chosen_poses(cands, ippe)
    reproj = cands.reprojection_error_px[np.arange(len(cands)), ippe.choice]
    azimuth = view_azimuth(field_from_camera, t_up_field)
    along, across = to_view(raw.x, raw.y, azimuth)

    idle = None
    if esp32 is not None and len(esp32):
        idle = idle_mask(
            raw.stamp_ns,
            esp32["stamp_ns"].to_numpy(),
            esp32["left_cmd"].to_numpy(),
            esp32["right_cmd"].to_numpy(),
        )
    segments = find_still_segments(raw.stamp_ns, along, across, raw.yaw, idle=idle)
    noise = calibrate_noise(raw, reproj, cands.size_px, segments, azimuth)
    sig = noise.sigmas(reproj, cands.size_px)

    t0_ns = int(raw.stamp_ns[0])
    planar_z = np.stack([along, across, np.unwrap(raw.yaw)], axis=1)
    planar_sigma = np.stack([sig["along"], sig["across"], sig["yaw"]], axis=1)
    knots, src = _planar_knots(raw.stamp_ns, planar_z, planar_sigma, t0_ns)
    planar_opts = _filter_options(_PLANAR_OPTIONS, opts)
    planar, cv_planar = _fit_axes(
        knots,
        src,
        {"xy": [0, 1], "yaw": [2]},
        {"xy": np.asarray(opts.q_xy), "yaw": np.asarray(opts.q_yaw)},
        planar_opts,
        opts.hold_every,
    )
    n = raw.stamp_ns.shape[0]
    tilt_knots = Knots(
        t_s=(raw.stamp_ns - t0_ns).astype(np.float64) / _NS,
        t0_ns=t0_ns,
        kind=np.full((n, 2), KIND_POSITION, dtype=np.int8),
        z=np.stack([raw.pitch, raw.roll], axis=1),
        r=np.stack([sig["tilt"] ** 2, sig["tilt"] ** 2], axis=1),
    )
    tilt, cv_tilt = _fit_axes(
        tilt_knots,
        np.arange(n),
        {"tilt": [0, 1]},
        {"tilt": np.asarray(opts.q_tilt)},
        _filter_options(_TILT_OPTIONS, opts),
        opts.hold_every,
    )

    step = int(round(opts.grid_dt_s * _NS))
    grid_ns = np.arange(int(raw.stamp_ns[0]), int(raw.stamp_ns[-1]) + 1, step, dtype=np.int64)
    grid = _grid_frame(planar, tilt, grid_ns, azimuth)

    checks: dict[str, Any] = {}
    imu: ImuYawRate | None = None
    imu_lag: dict[str, Any] | None = None
    if esp32 is not None and len(esp32) >= 5:
        imu = bno055_yaw_rate(
            esp32["stamp_ns"].to_numpy(),
            esp32["orientation_x"].to_numpy(),
            esp32["is_upside_down"].to_numpy(),
        )
        imu_lag, checks["imu_yaw_rate"] = _imu_agreement(grid, imu, opts)
        fused = _fuse_imu(
            raw, planar_z, planar_sigma, t0_ns, planar, planar_opts, imu, checks, opts
        )
        if fused is not None:
            planar = fused
            grid = _grid_frame(planar, tilt, grid_ns, azimuth)

    meas = _measurement_table(raw, cands, ippe, reproj, sig, planar, tilt, azimuth)
    rejected = meas["rejected"].to_numpy()
    accepted_stamps = raw.stamp_ns[~rejected]
    half = step // 2
    pos = np.searchsorted(accepted_stamps, grid_ns - half, side="left")
    grid["has_measurement"] = (pos < accepted_stamps.size) & (
        accepted_stamps[np.minimum(pos, max(accepted_stamps.size - 1, 0))] <= grid_ns + half
    )
    nearest = np.clip(np.searchsorted(raw.stamp_ns, grid_ns, side="right") - 1, 0, None)
    grid["upside_down"] = cands.upside_down[nearest]

    residual_acf, innovation_acf = _session_checks(meas, segments, ippe, checks)
    return SessionSmoothing(
        measurements=meas,
        grid=grid,
        frame_stamps_ns=np.asarray([f.image_stamp_ns for f in frames], dtype=np.int64),
        t_up_field=t_up_field,
        field_flipped=flipped,
        noise=noise,
        cv_planar=cv_planar,
        cv_tilt=cv_tilt,
        ippe=ippe,
        rejections=meas.loc[meas["rejected"], ["stamp_ns", "tag_id", "nis", "x", "y", "yaw"]],
        checks=checks,
        imu=imu,
        imu_lag=imu_lag,
        residual_acf=residual_acf,
        innovation_acf=innovation_acf,
        still_segments=segments,
    )


def _fuse_imu(
    raw: RawPoses,
    planar_z: np.ndarray,
    planar_sigma: np.ndarray,
    t0_ns: int,
    planar: _AxisFit,
    planar_opts: FilterOptions,
    imu: ImuYawRate,
    checks: dict[str, Any],
    opts: SmootherOptions,
) -> _AxisFit | None:
    """Rerun the planar smoother with BNO055 yaw rate as a measurement of the yaw-rate state.

    The IMU is shifted by its measured lag and multiplied by its measured sign first, and only
    used when it correlates with tag yaw rate at ``imu_min_correlation`` or better.
    """
    agreement = checks.get("imu_yaw_rate", {})
    corr = agreement.get("correlation")
    if not opts.fuse_imu or corr is None or abs(float(corr)) < opts.imu_min_correlation:
        checks["imu_fused"] = False
        return None
    sign = float(agreement["measured_sign"])
    lag_s = float(agreement["lag_s"])
    shifted = ImuYawRate(imu.stamp_ns - int(round(lag_s * _NS)), imu.rate * sign, imu.valid)
    still_std = agreement.get("still_rate_std")
    imu_sigma = max(float(still_std) if still_std is not None else 0.05, 0.05)
    knots, src = _planar_knots(raw.stamp_ns, planar_z, planar_sigma, t0_ns, shifted, imu_sigma)
    run, exempt = smooth_with_recheck(knots, planar.q, planar_opts)
    checks["imu_fused"] = {"sign": sign, "lag_s": lag_s, "sigma": imu_sigma}
    return _AxisFit(knots, src, run, planar.q, exempt)


def _measurement_table(
    raw: RawPoses,
    cands: Candidates,
    ippe: IppeChoice,
    reproj: np.ndarray,
    sig: dict[str, np.ndarray],
    planar: _AxisFit,
    tilt: _AxisFit,
    azimuth: float,
) -> pd.DataFrame:
    n = len(cands)
    det_knot = planar.det_knot(n)
    outcome = planar.run.outcome[0, det_knot]
    m_s = planar.run.m_smooth[0, det_knot]
    smoothed_x, smoothed_y = from_view(m_s[:, 0, 0], m_s[:, 1, 0], azimuth)
    smoothed_yaw = m_s[:, 2, 0]
    view_along, view_across = to_view(raw.x, raw.y, azimuth)
    return pd.DataFrame(
        {
            "stamp_ns": raw.stamp_ns,
            "tag_id": cands.tag_id,
            "x": raw.x,
            "y": raw.y,
            "z": raw.z,
            "yaw": smoothed_yaw + wrap_angle(raw.yaw - smoothed_yaw),
            "pitch": raw.pitch,
            "roll": raw.roll,
            "reprojection_error_px": reproj,
            "tag_size_px": cands.size_px,
            "sigma_xy": np.sqrt(0.5 * (sig["along"] ** 2 + sig["across"] ** 2)),
            "sigma_yaw": sig["yaw"],
            "ippe_close_call": ippe.close_call,
            "rejected": outcome == OUTCOME_REJECTED,
            "sigma_along": sig["along"],
            "sigma_across": sig["across"],
            "sigma_tilt": sig["tilt"],
            "reset": outcome == OUTCOME_RESET,
            "recheck_accepted": planar.exempt[det_knot],
            "tilt_rejected": tilt.run.outcome[0] == OUTCOME_REJECTED,
            "upside_down": cands.upside_down,
            "nis": planar.run.nis[0, det_knot],
            "innovation_along": planar.run.innovation[0, det_knot, 0],
            "innovation_across": planar.run.innovation[0, det_knot, 1],
            "view_along": view_along,
            "view_across": view_across,
            "smoothed_x": smoothed_x,
            "smoothed_y": smoothed_y,
            "smoothed_along": m_s[:, 0, 0],
            "smoothed_across": m_s[:, 1, 0],
            "smoothed_yaw": smoothed_yaw,
            "smoothed_pitch": tilt.run.m_smooth[0, :, 0, 0],
            "smoothed_roll": tilt.run.m_smooth[0, :, 1, 0],
        }
    )


def _session_checks(
    meas: pd.DataFrame,
    segments: list[tuple[int, int]],
    ippe: IppeChoice,
    checks: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray]:
    """Noise floor, residual whiteness, IPPE and gating counts. Returns the across-view residual
    and innovation autocorrelations for plotting."""
    use = ~meas["rejected"].to_numpy()
    still = np.zeros(len(meas), dtype=bool)
    for i0, i1 in segments:
        still[i0 : i1 + 1] = True
    checks["noise_floor"] = _noise_floor(meas, still)
    along_res = _view_residual(meas, "along")[use]
    across_res = _view_residual(meas, "across")[use]
    residual_acf = autocorrelation(across_res, 20)
    innovation_acf = autocorrelation(meas["innovation_across"].to_numpy()[use], 20)
    norm = np.concatenate([along_res, across_res])
    checks["residual_whiteness"] = {
        "along_residual": _whiteness(autocorrelation(along_res, 20), int(use.sum())),
        "across_residual": _whiteness(residual_acf, int(use.sum())),
        "along_innovation": _whiteness(
            autocorrelation(meas["innovation_along"].to_numpy()[use], 20), int(use.sum())
        ),
        "across_innovation": _whiteness(innovation_acf, int(use.sum())),
        "normalised_residual_rms": float(np.sqrt(np.mean(norm**2))) if use.any() else None,
    }
    checks["ippe"] = {
        "close_calls": ippe.close_calls,
        "unresolved_close_calls": ippe.unresolved_close_calls,
        "detections": int(len(meas)),
    }
    checks["gating"] = {
        "rejected": int(meas["rejected"].sum()),
        "resets": int(meas["reset"].sum()),
        "recheck_accepted": int(meas["recheck_accepted"].sum()),
        "tilt_rejected": int(meas["tilt_rejected"].sum()),
    }
    return residual_acf, innovation_acf


def _view_residual(meas: pd.DataFrame, axis: str) -> np.ndarray:
    """Raw minus smoothed along one view axis, over that detection's sigma."""
    return np.asarray(
        (meas[f"view_{axis}"] - meas[f"smoothed_{axis}"]) / meas[f"sigma_{axis}"], dtype=np.float64
    )


def _grid_frame(
    planar: _AxisFit, tilt: _AxisFit, grid_ns: np.ndarray, azimuth: float
) -> pd.DataFrame:
    m, cov = interpolate_smoothed(planar.run, planar.knots, planar.q, grid_ns)
    tm, _ = interpolate_smoothed(tilt.run, tilt.knots, tilt.q, grid_ns)
    x, y = from_view(m[:, 0, 0], m[:, 1, 0], azimuth)
    vx, vy = from_view(m[:, 0, 1], m[:, 1, 1], azimuth)
    ax, ay = from_view(m[:, 0, 2], m[:, 1, 2], azimuth)
    c2, s2 = math.cos(azimuth) ** 2, math.sin(azimuth) ** 2
    var_along = np.maximum(cov[:, 0, 0, 0], 0.0)
    var_across = np.maximum(cov[:, 1, 0, 0], 0.0)
    return pd.DataFrame(
        {
            "stamp_ns": grid_ns,
            "x": x,
            "y": y,
            "yaw": m[:, 2, 0],
            "vx": vx,
            "vy": vy,
            "yaw_rate": m[:, 2, 1],
            "ax": ax,
            "ay": ay,
            "pitch": tm[:, 0, 0],
            "roll": tm[:, 1, 0],
            "sigma_x": np.sqrt(c2 * var_along + s2 * var_across),
            "sigma_y": np.sqrt(s2 * var_along + c2 * var_across),
            "sigma_yaw": np.sqrt(np.maximum(cov[:, 2, 0, 0], 0.0)),
            "pitch_rate": tm[:, 0, 1],
            "roll_rate": tm[:, 1, 1],
        }
    )


def _noise_floor(meas: pd.DataFrame, still: np.ndarray) -> dict[str, Any]:
    """Scatter of raw poses about the smoothed pose on the still segments (per detection mask).

    The fit reports its errors as multiples of these numbers.
    """
    use = still & ~meas["rejected"].to_numpy()
    if not use.any():
        return {"detections": 0}
    m = meas[use]
    out: dict[str, Any] = {"detections": int(use.sum())}
    for col, smooth_col, angular in (
        ("x", "smoothed_x", False),
        ("y", "smoothed_y", False),
        ("view_along", "smoothed_along", False),
        ("view_across", "smoothed_across", False),
        ("yaw", "smoothed_yaw", True),
        ("pitch", "smoothed_pitch", True),
        ("roll", "smoothed_roll", True),
    ):
        r = m[col].to_numpy() - m[smooth_col].to_numpy()
        if angular:
            r = wrap_angle(r)
        out[f"sigma_{col.removeprefix('view_')}"] = float(np.std(r))
    out["sigma_xy"] = math.sqrt(0.5 * (out["sigma_x"] ** 2 + out["sigma_y"] ** 2))
    return out


def _imu_agreement(
    grid: pd.DataFrame, imu: ImuYawRate, opts: SmootherOptions
) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    """Lag, sign and fit of BNO055 yaw rate against smoothed tag yaw rate."""
    grid_ns = grid["stamp_ns"].to_numpy()
    imu_rate = np.where(imu.valid, imu.rate, np.nan)
    imu_on_grid = resample_linear(imu.stamp_ns, imu_rate, grid_ns, max_gap_s=0.05)
    tag_rate = grid["yaw_rate"].to_numpy()
    if np.isfinite(imu_on_grid).sum() < 50:
        return None, {"samples": int(np.isfinite(imu_on_grid).sum())}
    lag = cross_correlation_delay(opts.grid_dt_s, tag_rate, imu_on_grid, max_lag_s=0.2)
    lag_dict: dict[str, Any] = lag.as_dict()
    shift = int(round(lag.lag_s / opts.grid_dt_s)) if math.isfinite(lag.lag_s) else 0
    aligned = np.full_like(imu_on_grid, np.nan)
    if shift >= 0:
        aligned[: aligned.size - shift] = imu_on_grid[shift:]
    else:
        aligned[-shift:] = imu_on_grid[: aligned.size + shift]
    aligned = aligned * lag.sign
    spin = np.abs(tag_rate) > opts.spin_rate
    ok = spin & np.isfinite(aligned)
    # Rate noise wherever the tag track says the robot is not turning (it may still drive).
    still = (np.abs(tag_rate) < opts.still_yaw_rate) & np.isfinite(imu_on_grid)
    agreement: dict[str, Any] = {
        "heading_sign_assumed": -1,
        "measured_sign": lag.sign,
        "lag_s": lag.lag_s,
        "correlation": lag.correlation,
        "spin_samples": int(ok.sum()),
        "still_rate_std": float(np.std(imu_on_grid[still])) if still.any() else None,
    }
    if ok.sum() >= 10:
        diff = aligned[ok] - tag_rate[ok]
        agreement["spin_rms_rad_s"] = float(np.sqrt(np.mean(diff**2)))
        agreement["spin_gain"] = float(
            np.dot(aligned[ok], tag_rate[ok]) / max(np.dot(tag_rate[ok], tag_rate[ok]), 1e-9)
        )
    return lag_dict, agreement
