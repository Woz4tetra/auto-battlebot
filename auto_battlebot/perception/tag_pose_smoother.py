"""Robot pose from the overhead AprilTag recordings: IPPE choice, body pose, RTS smoothing."""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import IntEnum
from pathlib import Path
from typing import Any, NamedTuple, Sequence

import numpy as np
import pandas as pd
from scipy.stats import chi2

from auto_battlebot.compat import tomllib
from auto_battlebot.perception import yolo_field_fixes
from auto_battlebot.recording.esp32_clock import cross_correlation_delay, resample_linear
from auto_battlebot.recording.sysid_io import FieldFromCamera, RobotTagsFrame, rotation_angle

_NS = 1_000_000_000
_ROT_180_ABOUT_X = np.diag([1.0, -1.0, -1.0, 1.0])


class KnotKind(IntEnum):
    NONE = 0
    POSITION = 1
    RATE = 2


class Outcome(IntEnum):
    NONE = 0
    ACCEPTED = 1
    REJECTED = 2
    ACCEPTED_AFTER_LOST_TRACK = 3


@dataclass
class TagMount:
    """Where one tag sits on the body. ``t_body_tag`` maps marker-frame points into the body."""

    tag_id: int
    t_body_tag: np.ndarray
    tag_faces_floor_when_upright: bool

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
    """One ``[tags.N]`` table as a TagMount; ``upside_down`` defaults from the normal's body z."""
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
        tag_faces_floor_when_upright=upside_down,
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


class UpFrame(NamedTuple):
    t_up_field: np.ndarray
    flipped: bool


def up_from_field(field_from_camera: FieldFromCamera) -> UpFrame:
    """The up frame; ``flipped`` when the camera sits at negative field z."""
    if len(field_from_camera) == 0:
        raise ValueError("no camera -> field transform in the recording (run field init)")
    cam_z = float(np.median(field_from_camera.matrices[:, 2, 3]))
    if cam_z < 0.0:
        return UpFrame(_ROT_180_ABOUT_X.copy(), True)
    return UpFrame(np.eye(4), False)


def rest_rotation(r_up_body: np.ndarray, upside_down: bool) -> np.ndarray:
    """Body rotation relative to its rest pose: the identity for an upright robot sitting flat."""
    return r_up_body @ _ROT_180_ABOUT_X[:3, :3] if upside_down else r_up_body


def euler_zyx(r: np.ndarray) -> tuple[float, float, float]:
    """(yaw, pitch, roll) of a 3x3 rotation, ZYX order."""
    yaw = math.atan2(r[1, 0], r[0, 0])
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    roll = math.atan2(r[2, 1], r[2, 2])
    return yaw, pitch, roll


def attitude_error(r_up_body: np.ndarray, upside_down: bool, rest_pitch_rad: float) -> float:
    """How far a body attitude sits from resting flat on the floor, in radians."""
    _yaw, pitch, roll = euler_zyx(rest_rotation(r_up_body, upside_down))
    return math.hypot(pitch - rest_pitch_rad, roll)


def wrap_angle(a: np.ndarray | float) -> Any:
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def unwrap_onto(angle: np.ndarray | float, reference: np.ndarray | float) -> Any:
    return reference + wrap_angle(angle - reference)


def _require_shape(name: str, array: np.ndarray, shape: tuple[int, ...]) -> None:
    if array.shape != shape:
        raise ValueError(f"{name} has shape {array.shape}, expected {shape}")


@dataclass
class Candidates:
    """Every robot tag detection with its (up to two) IPPE solutions as body poses."""

    stamp_ns: np.ndarray
    frame_index: np.ndarray
    tag_id: np.ndarray
    upside_down: np.ndarray
    n_solutions: np.ndarray
    t_up_body: np.ndarray
    tilt_from_rest_rad: np.ndarray
    reprojection_error_px: np.ndarray
    size_px: np.ndarray

    def __post_init__(self) -> None:
        n = self.stamp_ns.shape[0]
        per_detection = (
            "stamp_ns",
            "frame_index",
            "tag_id",
            "upside_down",
            "n_solutions",
            "size_px",
        )
        for name in per_detection:
            _require_shape(name, getattr(self, name), (n,))
        _require_shape("t_up_body", self.t_up_body, (n, 2, 4, 4))
        _require_shape("tilt_from_rest_rad", self.tilt_from_rest_rad, (n, 2))
        _require_shape("reprojection_error_px", self.reprojection_error_px, (n, 2))
        if not np.all((self.n_solutions == 1) | (self.n_solutions == 2)):
            raise ValueError("n_solutions must be 1 or 2")
        single = self.n_solutions == 1
        if not np.array_equal(self.t_up_body[single, 0], self.t_up_body[single, 1]):
            raise ValueError("t_up_body slot 1 must repeat slot 0 where there is one solution")

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
    """Map every IPPE solution of every robot tag detection into a body pose."""
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
                        mount.tag_faces_floor_when_upright,
                        rest_pitch_upside_down_rad
                        if mount.tag_faces_floor_when_upright
                        else rest_pitch_rad,
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
            inverted.append(mount.tag_faces_floor_when_upright)
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
        tilt_from_rest_rad=np.asarray(tilts, dtype=np.float64).reshape(-1, 2),
        reprojection_error_px=np.asarray(errors, dtype=np.float64).reshape(-1, 2),
        size_px=np.asarray(sizes, dtype=np.float64),
    )


@dataclass
class IppeChoice:
    solution_index: np.ndarray
    close_call_decided_by_neighbours: np.ndarray
    close_calls: int
    close_calls_left_to_tilt: int

    def __post_init__(self) -> None:
        n = self.solution_index.shape[0]
        _require_shape(
            "close_call_decided_by_neighbours", self.close_call_decided_by_neighbours, (n,)
        )
        if not np.all((self.solution_index == 0) | (self.solution_index == 1)):
            raise ValueError("solution_index must be 0 or 1")


def select_ippe(
    cands: Candidates, close_margin_deg: float = 4.0, neighbour_window_s: float = 0.25
) -> IppeChoice:
    """Pick one IPPE solution per detection."""
    n = len(cands)
    choice = (
        np.argmin(cands.tilt_from_rest_rad, axis=1).astype(np.int64) if n else np.zeros(0, np.int64)
    )
    margin = math.radians(close_margin_deg)
    close = (cands.n_solutions == 2) & (
        np.abs(cands.tilt_from_rest_rad[:, 0] - cands.tilt_from_rest_rad[:, 1]) < margin
    )
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
        solution_index=choice,
        close_call_decided_by_neighbours=close,
        close_calls=int(close.sum()),
        close_calls_left_to_tilt=unresolved,
    )


@dataclass
class RawPoses:
    """The chosen body pose per detection, in the up frame."""

    stamp_ns: np.ndarray
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    yaw_wrapped: np.ndarray
    pitch: np.ndarray
    roll: np.ndarray


def chosen_poses(cands: Candidates, ippe: IppeChoice) -> RawPoses:
    n = len(cands)
    out = np.zeros((n, 6))
    for i in range(n):
        t = cands.t_up_body[i, ippe.solution_index[i]]
        yaw, pitch, roll = euler_zyx(rest_rotation(t[:3, :3], bool(cands.upside_down[i])))
        out[i] = (t[0, 3], t[1, 3], t[2, 3], yaw, pitch, roll)
    return RawPoses(cands.stamp_ns.copy(), *[out[:, k].copy() for k in range(6)])


@dataclass
class NoiseModel:
    """Per-detection measurement noise: sigma = k * g, g = hypot(reprojection error, e0) / size."""

    k_along: float = 0.4
    k_across: float = 0.07
    k_yaw: float = 0.5
    k_tilt: float = 0.8
    e0_px: float = 0.5
    size_ref_px: float = 50.0
    view_azimuth_rad: float = 0.0
    calibrated: bool = False
    noise_calibration_segments: int = 0
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
            "still_segments": self.noise_calibration_segments,
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
    """Per detection: inside the ESP32 stream with both motor commands settled."""
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
    """Detection index ranges where the raw pose stays inside a small box."""
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
    """Fit the four gains to the scatter about each still segment's median."""
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
            ("yaw", raw.yaw_wrapped[sl], g[sl], True),
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
    model.noise_calibration_segments = len(segments)
    model.still_detections = detections
    return model


@dataclass
class Knots:
    """Measurement instants shared by every axis of one smoother."""

    t_since_t0_s: np.ndarray
    t0_ns: int
    kind: np.ndarray
    z: np.ndarray
    variance: np.ndarray

    def __post_init__(self) -> None:
        k = self.t_since_t0_s.shape[0]
        _require_shape("kind", self.kind, (k, self.kind.shape[-1]))
        if self.kind.dtype != np.int8:
            raise ValueError(f"kind must be int8, got {self.kind.dtype}")
        _require_shape("z", self.z, self.kind.shape)
        _require_shape("variance", self.variance, self.kind.shape)
        if np.any(np.diff(self.t_since_t0_s) < 0.0):
            raise ValueError("t_since_t0_s must be non-decreasing")

    @property
    def n_axes(self) -> int:
        return int(self.kind.shape[1])


@dataclass
class SmootherRun:
    """Forward filter and RTS pass over a batch of settings. Leading axis is the batch."""

    m_pred: np.ndarray
    p_pred: np.ndarray
    m_filt: np.ndarray
    p_filt: np.ndarray
    m_smooth: np.ndarray
    p_smooth: np.ndarray
    outcome: np.ndarray
    nis: np.ndarray
    innovation: np.ndarray

    def __post_init__(self) -> None:
        b, k, a = self.m_pred.shape[:3]
        for name in ("m_pred", "m_filt", "m_smooth"):
            _require_shape(name, getattr(self, name), (b, k, a, 3))
        for name in ("p_pred", "p_filt", "p_smooth"):
            _require_shape(name, getattr(self, name), (b, k, a, 3, 3))
        _require_shape("outcome", self.outcome, (b, k))
        _require_shape("nis", self.nis, (b, k))
        _require_shape("innovation", self.innovation, (b, k, a))


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
    angular: Sequence[bool]
    prior_std: Sequence[Sequence[float]]
    gate_probability: float = 0.9999
    gate: bool = True
    reacquire_s: float = 0.3
    max_consecutive_rejects: int = 5
    branch_gap_s: float = 0.1


def run_smoother(
    knots: Knots,
    q: np.ndarray,
    options: FilterOptions,
    active: np.ndarray | None = None,
    gate_exempt: np.ndarray | None = None,
) -> SmootherRun:
    """Forward Kalman filter with chi-square gating, then the RTS backward pass."""
    q = np.atleast_2d(np.asarray(q, dtype=np.float64))
    b_count, a_count = q.shape
    k_count = knots.t_since_t0_s.shape[0]
    if a_count != knots.n_axes:
        raise ValueError("q has a different axis count than the knots")
    has = knots.kind != KnotKind.NONE
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
        k_count = knots.t_since_t0_s.shape[0]
        self.knots = knots
        self.options = options
        self.angular = np.asarray(options.angular, dtype=bool)
        prior = np.asarray(options.prior_std, dtype=np.float64).reshape(a_count, 3)
        self.prior_cov = np.stack([np.diag(prior[a] ** 2) for a in range(a_count)])
        self.thresholds = np.array(
            [np.inf] + [float(chi2.ppf(options.gate_probability, d)) for d in range(1, a_count + 1)]
        )
        self.h_rows = np.zeros((3, 3))
        self.h_rows[KnotKind.POSITION, 0] = 1.0
        self.h_rows[KnotKind.RATE, 1] = 1.0

        self.m_pred = np.zeros((b_count, k_count, a_count, 3))
        self.p_pred = np.zeros((b_count, k_count, a_count, 3, 3))
        self.m_filt = np.zeros_like(self.m_pred)
        self.p_filt = np.zeros_like(self.p_pred)
        self.outcome = np.zeros((b_count, k_count), dtype=np.int8)
        self.nis_out = np.full((b_count, k_count), np.nan)
        self.innov_out = np.full((b_count, k_count, a_count), np.nan)

        self.m = np.zeros((b_count, a_count, 3))
        for a in range(a_count):
            first = np.flatnonzero(knots.kind[:, a] == KnotKind.POSITION)
            if first.size:
                self.m[:, a, 0] = knots.z[first[0], a]
        self.cov = np.broadcast_to(self.prior_cov, (b_count, a_count, 3, 3)).copy()
        self.last_fix = np.full(b_count, -np.inf)
        self.rejects_in_row = np.zeros(b_count, dtype=np.int64)
        self.prev_t = float(knots.t_since_t0_s[0]) if k_count else 0.0
        self.exempt = np.zeros(k_count, dtype=bool)
        self.fix_state = self.m[:, :, :2].copy()
        self.fix_t = np.full((b_count, a_count), -np.inf)

    def predict(self, k: int, q: np.ndarray) -> None:
        t = float(self.knots.t_since_t0_s[k])
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
        """Residual, H P and innovation variance, angles unwrapped onto the branch reference."""
        t = float(self.knots.t_since_t0_s[k])
        hx = np.einsum("aj,baj->ba", h, self.m)
        z = np.broadcast_to(self.knots.z[k][None, :], hx.shape)
        seen = np.isfinite(self.fix_t)
        gap = np.where(seen, t - np.where(seen, self.fix_t, 0.0), 0.0)
        carried = self.fix_state[:, :, 0] + self.fix_state[:, :, 1] * gap
        ref = np.where(gap > self.options.branch_gap_s, carried, hx)
        angle = self.angular[None, :] & (self.knots.kind[k] == KnotKind.POSITION)[None, :]
        resid = np.where(angle, ref + wrap_angle(z - ref) - hx, z - hx)
        hp = np.einsum("aj,bajk->bak", h, self.cov)
        s = np.einsum("bak,ak->ba", hp, h) + self.knots.variance[k][None, :]
        return resid, hp, s

    def update(self, k: int, u: np.ndarray) -> None:
        """Gate and apply knot ``k``'s measurements; ``u`` (B, A) says which axes are in use."""
        kind_k = self.knots.kind[k]
        h = self.h_rows[kind_k]
        t = float(self.knots.t_since_t0_s[k])
        resid, hp, s = self._innovation(k, h)
        nis = np.where(u, resid * resid / s, 0.0).sum(axis=1)
        dof = u.sum(axis=1)
        measured = dof > 0
        passed = (nis <= self.thresholds[dof]) if self.options.gate else measured.copy()
        is_position = bool(np.any(kind_k == KnotKind.POSITION))
        if self.exempt[k] or not is_position:
            passed = measured.copy()
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
        gain = hp / s[:, :, None]
        self.m = self.m + np.where(ut[:, :, None], gain * resid[:, :, None], 0.0)
        self.cov = self.cov - np.where(
            ut[:, :, None, None], gain[:, :, :, None] * hp[:, :, None, :], 0.0
        )
        self.nis_out[measured, k] = nis[measured]
        self.innov_out[:, k] = np.where(u, resid / np.sqrt(s), np.nan)
        self.outcome[measured & passed, k] = Outcome.ACCEPTED
        self.outcome[measured & ~passed & ~reset, k] = Outcome.REJECTED
        self.outcome[reset, k] = Outcome.ACCEPTED_AFTER_LOST_TRACK
        if is_position:
            self.last_fix[take] = t
            self.rejects_in_row[take] = 0
            self.rejects_in_row[measured & ~take] += 1
            fixed = ut & (kind_k == KnotKind.POSITION)[None, :]
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
    for k in range(knots.t_since_t0_s.shape[0] - 2, -1, -1):
        f = transition(float(knots.t_since_t0_s[k + 1] - knots.t_since_t0_s[k]))
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
    """``run_smoother``, then readmit gated fixes the smoothed track vouches for."""
    q = np.atleast_2d(np.asarray(q, dtype=np.float64))
    exempt = np.zeros(knots.t_since_t0_s.shape[0], dtype=bool)
    angular = np.asarray(options.angular, dtype=bool)
    run = run_smoother(knots, q, options, gate_exempt=exempt)
    for _ in range(max_passes - 1):
        rejected = run.outcome[0] == Outcome.REJECTED
        rejected &= np.any(knots.kind == KnotKind.POSITION, axis=1)
        if not rejected.any():
            break
        idx = np.flatnonzero(rejected)
        pos = knots.kind[idx] == KnotKind.POSITION
        resid = knots.z[idx] - run.m_smooth[0, idx, :, 0]
        resid = np.where(angular[None, :], wrap_angle(resid), resid)
        var = run.p_smooth[0, idx, :, 0, 0] + knots.variance[idx]
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
    """Smoothed mean (G, A, 3) and covariance (G, A, 3, 3) at arbitrary times."""
    t = (np.asarray(grid_ns, dtype=np.int64) - knots.t0_ns).astype(np.float64) / _NS
    kt = knots.t_since_t0_s
    k_count = kt.shape[0]
    qa = np.asarray(q, dtype=np.float64).reshape(-1)
    idx = np.clip(np.searchsorted(kt, t, side="right") - 1, 0, k_count - 1)
    tau = np.clip(t - kt[idx], 0.0, None)
    nxt = np.minimum(idx + 1, k_count - 1)
    last = idx == k_count - 1
    tau = np.where(last, 0.0, tau)

    mf = run.m_filt[batch][idx]
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
    m_out[last] = run.m_smooth[batch][idx[last]]
    p_out[last] = run.p_smooth[batch][idx[last]]
    return m_out, p_out


@dataclass
class CrossValidation:
    axes_by_group: dict[str, list[int]]
    q_candidates_by_group: dict[str, np.ndarray]
    heldout_score_by_group: dict[str, np.ndarray]
    chosen: dict[str, float]
    held_out: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "held_out_detections": self.held_out,
            "chosen_q": self.chosen,
            "curve": {
                g: {
                    "q": self.q_candidates_by_group[g].tolist(),
                    "score": self.heldout_score_by_group[g].tolist(),
                }
                for g in self.axes_by_group
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
    """Pick each group's process noise by hold-out on dropped position fixes."""
    n_cand = {len(v) for v in candidates.values()}
    if len(n_cand) != 1:
        raise ValueError("every group needs the same number of candidates")
    b_count = n_cand.pop()
    q = np.zeros((b_count, knots.n_axes))
    for g, axes in groups.items():
        for a in axes:
            q[:, a] = candidates[g]
    position_knots = np.flatnonzero(np.any(knots.kind == KnotKind.POSITION, axis=1))
    held = position_knots[::hold_every]
    active = np.ones((b_count, knots.t_since_t0_s.shape[0], knots.n_axes), dtype=bool)
    active[:, held, :] = False
    run = run_smoother(knots, q, options, active=active)
    score: dict[str, np.ndarray] = {}
    chosen: dict[str, float] = {}
    for g, axes in groups.items():
        per = np.zeros(b_count)
        for b in range(b_count):
            vals = []
            for a in axes:
                ok = knots.kind[held, a] == KnotKind.POSITION
                kk = held[ok]
                resid = knots.z[kk, a] - run.m_smooth[b, kk, a, 0]
                if options.angular[a]:
                    resid = wrap_angle(resid)
                vals.append(np.minimum(resid * resid / knots.variance[kk, a], clip))
            per[b] = float(np.mean(np.concatenate(vals))) if vals else math.nan
        score[g] = per
        chosen[g] = float(candidates[g][int(np.nanargmin(per))])
    return CrossValidation(groups, candidates, score, chosen, int(held.size))


@dataclass
class ImuYawRate:
    app_clock_stamp_ns: np.ndarray
    ccw_rate_rad_s: np.ndarray
    valid: np.ndarray


def bno055_yaw_rate(
    stamp_ns: np.ndarray,
    heading_deg: np.ndarray,
    upside_down: np.ndarray,
    grid_dt_s: float = 0.01,
    heading_sign: float = -1.0,
) -> ImuYawRate:
    """Yaw rate from the BNO055 Euler heading (``orientation_x``)."""
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


def gyro_yaw_rate(
    stamp_ns: np.ndarray, yaw_rate_deg_s: np.ndarray, heading: ImuYawRate
) -> ImuYawRate:
    """Yaw rate from the firmware's gyro ``yaw_rate``, on ``heading``'s grid."""
    if heading.app_clock_stamp_ns.size == 0:
        return heading
    t = np.asarray(stamp_ns, dtype=np.int64)
    order = np.argsort(t, kind="stable")
    rate_ccw = -np.radians(np.asarray(yaw_rate_deg_s, dtype=np.float64)[order])
    rate = resample_linear(t[order], rate_ccw, heading.app_clock_stamp_ns, max_gap_s=0.1)
    wrapped = heading.valid & (np.abs(rate - heading.ccw_rate_rad_s) > GYRO_WRAP_TOLERANCE)
    return ImuYawRate(heading.app_clock_stamp_ns, rate, np.isfinite(rate) & ~wrapped)


@dataclass
class SmootherOptions:
    grid_dt_s: float = 0.005
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
    fuse_imu: bool = True
    imu_min_correlation: float = 0.7
    still_yaw_rate: float = 0.15
    spin_rate: float = 3.0


@dataclass
class SessionSmoothing:
    detections_df: pd.DataFrame
    smoothed_grid_df: pd.DataFrame
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
    noise_calibration_segments: list[tuple[int, int]]
    yolo_fixes: pd.DataFrame | None = None
    yolo_calibration: yolo_field_fixes.YoloCalibration | None = None


@dataclass
class _AxisFit:
    """One smoother's knots, run and chosen process noise."""

    knots: Knots
    knot_source: np.ndarray
    run: SmootherRun
    q: np.ndarray
    gate_exempt_after_recheck: np.ndarray

    def det_knot(self, n_detections: int) -> np.ndarray:
        out = np.full(n_detections, -1, dtype=np.int64)
        out[self.knot_source[self.knot_source >= 0]] = np.flatnonzero(self.knot_source >= 0)
        return out


class KnotSource(IntEnum):
    IMU = -1
    EXTRA = -2


class KnotBuild(NamedTuple):
    knots: Knots
    source_index: np.ndarray


IMU_GAIN_MAX_RATE = 25.0
IMU_MAX_RATE = 30.0
GYRO_WRAP_TOLERANCE = 10.0
GYRO_PITCH_SIGMA = 0.1
GYRO_PITCH_MIN_CORRELATION = 0.3

_PLANAR_OPTIONS = {
    "angular": (False, False, True),
    "prior_std": ((1.0, 5.0, 50.0), (1.0, 5.0, 50.0), (math.pi, 40.0, 1000.0)),
}
_TILT_OPTIONS = {
    "angular": (False, False),
    "prior_std": ((0.5, 10.0, 300.0), (0.5, 10.0, 300.0)),
}


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
    extra: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None,
) -> KnotBuild:
    """Knots for (along, across, yaw) with optional IMU rate knots and extra position fixes."""
    n = stamp_ns.shape[0]
    stamps = stamp_ns
    knot_source = np.arange(n)
    kind = np.full((n, 3), KnotKind.POSITION, dtype=np.int8)
    r = sigma**2
    if imu is not None and np.any(imu.valid):
        it = imu.app_clock_stamp_ns[imu.valid]
        ik = np.zeros((it.size, 3), dtype=np.int8)
        ik[:, 2] = KnotKind.RATE
        iz = np.zeros((it.size, 3))
        iz[:, 2] = imu.ccw_rate_rad_s[imu.valid]
        ir = np.ones((it.size, 3))
        ir[:, 2] = imu_sigma**2
        stamps = np.concatenate([stamps, it])
        knot_source = np.concatenate([knot_source, np.full(it.size, KnotSource.IMU)])
        kind = np.concatenate([kind, ik])
        z = np.concatenate([z, iz])
        r = np.concatenate([r, ir])
    if extra is not None and len(extra[0]):
        es, ez, esig = extra
        measured = np.isfinite(ez)
        stamps = np.concatenate([stamps, np.asarray(es, dtype=np.int64)])
        knot_source = np.concatenate([knot_source, np.full(len(es), KnotSource.EXTRA)])
        kind = np.concatenate(
            [kind, np.where(measured, KnotKind.POSITION, KnotKind.NONE).astype(np.int8)]
        )
        z = np.concatenate([z, np.where(measured, ez, 0.0)])
        r = np.concatenate([r, np.where(measured, np.asarray(esig) ** 2, 1.0)])
    order = np.argsort(stamps, kind="stable")
    knots = Knots(
        t_since_t0_s=(stamps[order] - t0_ns).astype(np.float64) / _NS,
        t0_ns=t0_ns,
        kind=kind[order],
        z=z[order],
        variance=r[order],
    )
    return KnotBuild(knots, knot_source[order])


def _fit_axes(
    knots: Knots,
    knot_source: np.ndarray,
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
    return _AxisFit(knots, knot_source, run, q, exempt), cv


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
    yolo: pd.DataFrame | None = None,
) -> SessionSmoothing:
    """The whole "Robot pose recording" pass for one session."""
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
    reproj = cands.reprojection_error_px[np.arange(len(cands)), ippe.solution_index]
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
    segments = find_still_segments(raw.stamp_ns, along, across, raw.yaw_wrapped, idle=idle)
    noise = calibrate_noise(raw, reproj, cands.size_px, segments, azimuth)
    sig = noise.sigmas(reproj, cands.size_px)

    t0_ns = int(raw.stamp_ns[0])
    planar_z = np.stack([along, across, np.unwrap(raw.yaw_wrapped)], axis=1)
    planar_sigma = np.stack([sig["along"], sig["across"], sig["yaw"]], axis=1)
    knots, knot_source = _planar_knots(raw.stamp_ns, planar_z, planar_sigma, t0_ns)
    planar_opts = _filter_options(_PLANAR_OPTIONS, opts)
    planar, cv_planar = _fit_axes(
        knots,
        knot_source,
        {"xy": [0, 1], "yaw": [2]},
        {"xy": np.asarray(opts.q_xy), "yaw": np.asarray(opts.q_yaw)},
        planar_opts,
        opts.hold_every,
    )
    n = raw.stamp_ns.shape[0]
    tilt_knots = Knots(
        t_since_t0_s=(raw.stamp_ns - t0_ns).astype(np.float64) / _NS,
        t0_ns=t0_ns,
        kind=np.full((n, 2), KnotKind.POSITION, dtype=np.int8),
        z=np.stack([np.unwrap(raw.pitch), np.unwrap(raw.roll)], axis=1),
        variance=np.stack([sig["tilt"] ** 2, sig["tilt"] ** 2], axis=1),
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
        imu, source = _telemetry_yaw_rate(esp32)
        tag_rate = _raw_tag_yaw_rate(raw.stamp_ns, raw.yaw_wrapped, grid_ns)
        imu_lag, checks["imu_yaw_rate"] = _imu_agreement(grid_ns, tag_rate, imu, opts)
        checks["imu_yaw_rate"]["source"] = source
        fused = _fuse_imu(
            raw, planar_z, planar_sigma, t0_ns, planar, planar_opts, imu, checks, opts
        )
        if fused is not None:
            planar = fused
            grid = _grid_frame(planar, tilt, grid_ns, azimuth)

    yolo_fixes: pd.DataFrame | None = None
    yolo_cal: yolo_field_fixes.YoloCalibration | None = None
    if yolo is not None and len(yolo):
        with_yolo = _fuse_yolo(
            yolo,
            frames,
            field_from_camera,
            t_up_field,
            azimuth,
            raw,
            planar_z,
            planar_sigma,
            t0_ns,
            planar,
            planar_opts,
            imu,
            esp32,
            checks,
            opts,
        )
        if with_yolo is not None:
            planar, yolo_fixes, yolo_cal = with_yolo
            grid = _grid_frame(planar, tilt, grid_ns, azimuth)
    if esp32 is not None and len(esp32):
        yolo_stamps = (
            yolo_fixes["stamp_ns"].to_numpy(np.int64)
            if yolo_fixes is not None
            else np.zeros(0, np.int64)
        )
        fused_tilt = _fuse_imu_tilt(
            esp32,
            yolo_stamps,
            raw,
            cands.upside_down,
            tilt,
            tilt_knots,
            _filter_options(_TILT_OPTIONS, opts),
            checks,
            opts.fuse_imu,
        )
        if fused_tilt is not None:
            tilt = fused_tilt
            grid = _grid_frame(planar, tilt, grid_ns, azimuth)

    meas = _measurement_table(raw, cands, ippe, reproj, sig, planar, tilt, azimuth)
    rejected = meas["rejected"].to_numpy()
    accepted_stamps = raw.stamp_ns[~rejected]
    if yolo_fixes is not None:
        yolo_ok = yolo_fixes["stamp_ns"].to_numpy(np.int64)[~yolo_fixes["rejected"].to_numpy()]
        accepted_stamps = np.sort(np.concatenate([accepted_stamps, yolo_ok]))
    half = step // 2
    pos = np.searchsorted(accepted_stamps, grid_ns - half, side="left")
    grid["has_measurement"] = (pos < accepted_stamps.size) & (
        accepted_stamps[np.minimum(pos, max(accepted_stamps.size - 1, 0))] <= grid_ns + half
    )
    nearest = np.clip(np.searchsorted(raw.stamp_ns, grid_ns, side="right") - 1, 0, None)
    grid["upside_down"] = cands.upside_down[nearest]

    residual_acf, innovation_acf = _session_checks(meas, segments, ippe, checks)
    return SessionSmoothing(
        detections_df=meas,
        smoothed_grid_df=grid,
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
        noise_calibration_segments=segments,
        yolo_fixes=yolo_fixes,
        yolo_calibration=yolo_cal,
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
    """Rerun the planar smoother with BNO055 yaw rate measuring the yaw-rate state."""
    fused = _imu_knot_input(imu, checks, opts)
    if fused is None:
        checks["imu_fused"] = False
        return None
    shifted, imu_sigma = fused
    knots, knot_source = _planar_knots(
        raw.stamp_ns, planar_z, planar_sigma, t0_ns, shifted, imu_sigma
    )
    run, exempt = smooth_with_recheck(knots, planar.q, _with_rate(planar_opts))
    agreement = checks["imu_yaw_rate"]
    checks["imu_fused"] = {
        "sign": float(agreement["measured_sign"]),
        "lag_s": float(agreement["lag_s"]),
        "sigma": imu_sigma,
    }
    return _AxisFit(knots, knot_source, run, planar.q, exempt)


def _telemetry_yaw_rate(esp32: pd.DataFrame) -> tuple[ImuYawRate, str]:
    """The gyro yaw rate where the telemetry carries it, else the differentiated heading."""
    heading = bno055_yaw_rate(
        esp32["stamp_ns"].to_numpy(),
        esp32["orientation_x"].to_numpy(),
        esp32["is_upside_down"].to_numpy(),
    )
    gyro = esp32["yaw_rate"].to_numpy(float) if "yaw_rate" in esp32 else np.array([np.nan])
    if np.isfinite(gyro).mean() > 0.9:
        return gyro_yaw_rate(esp32["stamp_ns"].to_numpy(), gyro, heading), "gyro"
    return heading, "heading"


def _gyro_pitch_agreement(
    raw: RawPoses,
    et: np.ndarray,
    rate: np.ndarray,
    upright: np.ndarray,
    interval_s: float = 0.5,
    step_s: float = 0.1,
    min_change_deg: float = 3.0,
) -> dict[str, Any]:
    """Sign and fit of the integrated pitch gyro against tag pitch changes over ``interval_s``."""
    integral = np.concatenate([[0.0], np.cumsum(np.nan_to_num(rate[1:]) * np.diff(et) / _NS)])
    pitch = np.unwrap(raw.pitch)

    def tag_at(stamps: np.ndarray) -> np.ndarray:
        lo = np.searchsorted(raw.stamp_ns, stamps - 30_000_000)
        hi = np.searchsorted(raw.stamp_ns, stamps + 30_000_000)
        return np.array([np.median(pitch[a:b]) if b - a >= 2 else np.nan for a, b in zip(lo, hi)])

    span = int(interval_s * _NS)
    starts = np.arange(int(raw.stamp_ns[0]), int(raw.stamp_ns[-1]) - span, int(step_s * _NS))
    ends = starts + span
    d_tag = tag_at(ends) - tag_at(starts)
    d_gyro = np.interp(ends, et, integral) - np.interp(starts, et, integral)
    flips = np.concatenate([[0], np.cumsum(~upright)])
    clean = flips[np.searchsorted(et, ends)] == flips[np.searchsorted(et, starts)]
    big = np.isfinite(d_tag) & clean & (np.abs(d_tag) > math.radians(min_change_deg))
    if big.sum() < 20:
        return {"intervals": int(big.sum())}
    corr = float(np.corrcoef(d_gyro[big], d_tag[big])[0, 1])
    return {
        "intervals": int(big.sum()),
        "correlation": corr,
        "slope": float(np.polyfit(d_gyro[big], d_tag[big], 1)[0]),
        "sign": 1.0 if corr >= 0 else -1.0,
    }


def _fuse_imu_tilt(
    esp32: pd.DataFrame,
    yolo_stamps_ns: np.ndarray,
    raw: RawPoses,
    upside_down: np.ndarray,
    tilt: _AxisFit,
    tilt_knots: Knots,
    tilt_opts: FilterOptions,
    checks: dict[str, Any],
    fuse_gyro: bool,
) -> _AxisFit | None:
    """Rerun the tilt smoother with the robot IMU's tilt; None when there is nothing to add."""
    lag_ns = int(round(float((checks.get("imu_yaw_rate") or {}).get("lag_s") or 0.0) * _NS))
    e = esp32.sort_values("stamp_ns")
    et = e["stamp_ns"].to_numpy(np.int64)
    upright = ~e["is_upside_down"].to_numpy(bool)
    report: dict[str, Any] = {"lag_s": lag_ns / _NS, "yolo_fixes": int(len(yolo_stamps_ns))}

    gyro_stamps = np.zeros(0, np.int64)
    gyro_rate = np.zeros(0)
    gyro_y = e["gyro_y"].to_numpy(float) if "gyro_y" in e else np.array([np.nan])
    if fuse_gyro and np.isfinite(gyro_y).mean() > 0.9:
        rate = np.radians(gyro_y)
        agreement = _gyro_pitch_agreement(raw, et - lag_ns, rate, upright)
        fused = abs(agreement.get("correlation", 0.0)) >= GYRO_PITCH_MIN_CORRELATION
        if fused:
            keep = upright & np.isfinite(rate)
            stamps = et[keep] - lag_ns
            inside = (stamps >= raw.stamp_ns[0]) & (stamps <= raw.stamp_ns[-1])
            gyro_stamps = stamps[inside]
            gyro_rate = agreement["sign"] * rate[keep][inside]
        report["gyro_pitch"] = {**agreement, "fused": fused, "knots": int(len(gyro_stamps))}
    if len(gyro_stamps) == 0 and len(yolo_stamps_ns) == 0:
        checks["imu_tilt"] = report
        return None

    def imu_at(stamps: np.ndarray, col: str) -> np.ndarray:
        at = np.clip(np.searchsorted(et, stamps + lag_ns, side="right") - 1, 0, len(et) - 1)
        return np.asarray(e[col].to_numpy(float)[at])

    up = ~np.asarray(upside_down, dtype=bool)
    m = len(yolo_stamps_ns)
    z = np.zeros((m, 2))
    sigma = np.zeros(2)
    for a, (col, tag) in enumerate((("orientation_y", raw.pitch), ("orientation_z", raw.roll))):
        x = imu_at(raw.stamp_ns[up], col)
        coef = np.polyfit(x, tag[up], 1)
        resid = tag[up] - np.polyval(coef, x)
        sigma[a] = 1.4826 * float(np.median(np.abs(resid - np.median(resid))))
        z[:, a] = np.polyval(coef, imu_at(yolo_stamps_ns, col))
        before = np.clip(np.searchsorted(raw.stamp_ns, yolo_stamps_ns, side="right") - 1, 0, None)
        branch = tilt_knots.z[before, a]
        z[:, a] = unwrap_onto(z[:, a], branch)
        report[col] = {"gain": float(coef[0]), "offset": float(coef[1]), "sigma": sigma[a]}
    yolo_kind = np.full((m, 2), KnotKind.POSITION, dtype=np.int8)
    if len(gyro_stamps):
        yolo_kind[:, 0] = KnotKind.NONE
    g = len(gyro_stamps)
    gyro_kind = np.tile(np.array([KnotKind.RATE, KnotKind.NONE], dtype=np.int8), (g, 1))
    gyro_z = np.stack([gyro_rate, np.zeros(g)], axis=1)
    gyro_r = np.tile([GYRO_PITCH_SIGMA**2, 1.0], (g, 1))

    n = tilt_knots.t_since_t0_s.shape[0]
    stamps = np.concatenate([raw.stamp_ns, yolo_stamps_ns, gyro_stamps])
    order = np.argsort(stamps, kind="stable")
    knots = Knots(
        t_since_t0_s=(stamps[order] - tilt_knots.t0_ns).astype(np.float64) / _NS,
        t0_ns=tilt_knots.t0_ns,
        kind=np.concatenate([tilt_knots.kind, yolo_kind, gyro_kind])[order],
        z=np.concatenate([tilt_knots.z, z, gyro_z])[order],
        variance=np.concatenate([tilt_knots.variance, np.tile(sigma**2, (m, 1)), gyro_r])[order],
    )
    knot_source = np.concatenate(
        [np.arange(n), np.full(m, KnotSource.EXTRA), np.full(g, KnotSource.IMU)]
    )[order]
    run, exempt = smooth_with_recheck(knots, tilt.q, tilt_opts)
    checks["imu_tilt"] = report
    return _AxisFit(knots, knot_source, run, tilt.q, exempt)


def _with_rate(options: FilterOptions) -> FilterOptions:
    """Options for a run with IMU rate knots: unwrap yaw onto the prediction's branch always."""
    return replace(options, branch_gap_s=math.inf)


def _imu_knot_input(
    imu: ImuYawRate, checks: dict[str, Any], opts: SmootherOptions
) -> tuple[ImuYawRate, float] | None:
    """The IMU shifted by its lag and sign, and its sigma; None when it is not to be fused."""
    agreement = checks.get("imu_yaw_rate", {})
    corr = agreement.get("correlation")
    if not opts.fuse_imu or corr is None or abs(float(corr)) < opts.imu_min_correlation:
        return None
    sign = float(agreement["measured_sign"])
    lag_s = float(agreement["lag_s"])
    gain = float(agreement.get("linear_gain") or 1.0)
    if not 0.5 < gain < 1.5:
        gain = 1.0
    rate = imu.ccw_rate_rad_s * sign / gain
    valid = imu.valid & (np.abs(rate) < IMU_MAX_RATE)
    shifted = ImuYawRate(imu.app_clock_stamp_ns - int(round(lag_s * _NS)), rate, valid)
    spin = agreement.get("linear_rms_rad_s", agreement.get("spin_rms_rad_s"))
    sigmas = [agreement.get("still_rate_std"), spin, 0.05]
    return shifted, max(float(v) for v in sigmas if v is not None)


def _fuse_yolo(
    yolo: pd.DataFrame,
    frames: Sequence[RobotTagsFrame],
    field_from_camera: FieldFromCamera,
    t_up_field: np.ndarray,
    azimuth: float,
    raw: RawPoses,
    planar_z: np.ndarray,
    planar_sigma: np.ndarray,
    t0_ns: int,
    planar: _AxisFit,
    planar_opts: FilterOptions,
    imu: ImuYawRate | None,
    esp32: pd.DataFrame | None,
    checks: dict[str, Any],
    opts: SmootherOptions,
) -> tuple[_AxisFit, pd.DataFrame, yolo_field_fixes.YoloCalibration] | None:
    """Rerun the planar smoother with YOLO position fixes on frames the tag missed."""

    def up_from_camera_at(stamps_ns: np.ndarray) -> np.ndarray:
        return np.asarray(t_up_field @ field_from_camera.at(np.asarray(stamps_ns, np.int64)))

    camera_matrix = frames[0].camera.matrix
    n = len(raw.stamp_ns)
    good = planar.run.outcome[0, planar.det_knot(n)] != Outcome.REJECTED
    tag_t, first = np.unique(raw.stamp_ns[good], return_index=True)
    try:
        cal = yolo_field_fixes.calibrate(
            yolo,
            tag_t,
            raw.x[good][first],
            raw.y[good][first],
            np.unwrap(raw.yaw_wrapped[good][first]),
            camera_matrix,
            up_from_camera_at,
            azimuth,
        )
    except ValueError as exc:
        checks["yolo"] = {"fused": False, "reason": str(exc)}
        return None
    fixes, report = yolo_field_fixes.select_fixes(
        yolo,
        cal,
        camera_matrix,
        up_from_camera_at,
        raw.stamp_ns,
        esp32,
    )
    checks["yolo"] = {"fused": False, "calibration": cal.as_dict(), **report}
    if fixes.empty:
        return None
    fixes = fixes.sort_values("stamp_ns", kind="stable").reset_index(drop=True)
    along, across = to_view(fixes["x"].to_numpy(), fixes["y"].to_numpy(), azimuth)
    ez = np.stack([along, across, np.full(len(fixes), np.nan)], axis=1)
    esig = np.tile([cal.sigma_along_m, cal.sigma_across_m, 1.0], (len(fixes), 1))
    fused = _imu_knot_input(imu, checks, opts) if imu is not None else None
    shifted, imu_sigma = fused if fused is not None else (None, 0.1)
    knots, knot_source = _planar_knots(
        raw.stamp_ns,
        planar_z,
        planar_sigma,
        t0_ns,
        shifted,
        imu_sigma,
        extra=(fixes["stamp_ns"].to_numpy(np.int64), ez, esig),
    )
    run, exempt = smooth_with_recheck(
        knots, planar.q, _with_rate(planar_opts) if shifted is not None else planar_opts
    )
    at = np.flatnonzero(knot_source == KnotSource.EXTRA)
    outcome = run.outcome[0, at]
    table = fixes.assign(
        yaw=np.nan,
        sigma_xy=math.sqrt(0.5 * (cal.sigma_along_m**2 + cal.sigma_across_m**2)),
        sigma_yaw=np.nan,
        rejected=outcome == Outcome.REJECTED,
        nis=run.nis[0, at],
    )
    checks["yolo"].update(
        fused=True, accepted=int((~table["rejected"]).sum()), rejected=int(table["rejected"].sum())
    )
    return _AxisFit(knots, knot_source, run, planar.q, exempt), table, cal


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
    tilt_knot = tilt.det_knot(n)
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
            "yaw": smoothed_yaw + wrap_angle(raw.yaw_wrapped - smoothed_yaw),
            "pitch": raw.pitch,
            "roll": raw.roll,
            "reprojection_error_px": reproj,
            "tag_size_px": cands.size_px,
            "sigma_xy": np.sqrt(0.5 * (sig["along"] ** 2 + sig["across"] ** 2)),
            "sigma_yaw": sig["yaw"],
            "ippe_close_call": ippe.close_call_decided_by_neighbours,
            "rejected": outcome == Outcome.REJECTED,
            "sigma_along": sig["along"],
            "sigma_across": sig["across"],
            "sigma_tilt": sig["tilt"],
            "reset": outcome == Outcome.ACCEPTED_AFTER_LOST_TRACK,
            "recheck_accepted": planar.gate_exempt_after_recheck[det_knot],
            "tilt_rejected": tilt.run.outcome[0, tilt_knot] == Outcome.REJECTED,
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
            "smoothed_pitch": tilt.run.m_smooth[0, tilt_knot, 0, 0],
            "smoothed_roll": tilt.run.m_smooth[0, tilt_knot, 1, 0],
        }
    )


def _session_checks(
    meas: pd.DataFrame,
    segments: list[tuple[int, int]],
    ippe: IppeChoice,
    checks: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray]:
    """Noise floor, residual whiteness, IPPE and gating counts."""
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
        "unresolved_close_calls": ippe.close_calls_left_to_tilt,
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
    pitch_unwrapped = tm[:, 0, 0]
    roll_unwrapped = tm[:, 1, 0]
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
            "pitch": wrap_angle(pitch_unwrapped),
            "roll": wrap_angle(roll_unwrapped),
            "sigma_x": np.sqrt(c2 * var_along + s2 * var_across),
            "sigma_y": np.sqrt(s2 * var_along + c2 * var_across),
            "sigma_yaw": np.sqrt(np.maximum(cov[:, 2, 0, 0], 0.0)),
            "pitch_rate": tm[:, 0, 1],
            "roll_rate": tm[:, 1, 1],
        }
    )


def _noise_floor(meas: pd.DataFrame, still: np.ndarray) -> dict[str, Any]:
    """Scatter of raw poses about the smoothed pose on the still segments."""
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


def _raw_tag_yaw_rate(
    stamp_ns: np.ndarray, yaw: np.ndarray, grid_ns: np.ndarray, span_s: float = 0.1
) -> np.ndarray:
    """Yaw rate from the raw tag detections on ``grid_ns``, NaN across detection gaps."""
    order = np.argsort(stamp_ns, kind="stable")
    yaw_on_grid = resample_linear(
        stamp_ns[order], np.unwrap(np.asarray(yaw)[order]), grid_ns, max_gap_s=0.05
    )
    rate = np.full(grid_ns.shape, np.nan)
    if grid_ns.size < 3:
        return rate
    dt = (grid_ns[1] - grid_ns[0]) / _NS
    half = max(1, int(round(span_s / (2.0 * dt))))
    if grid_ns.size > 2 * half:
        rate[half:-half] = (yaw_on_grid[2 * half :] - yaw_on_grid[: -2 * half]) / (2 * half * dt)
    return rate


def _imu_agreement(
    grid_ns: np.ndarray, tag_rate: np.ndarray, imu: ImuYawRate, opts: SmootherOptions
) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    """Lag, sign and fit of BNO055 yaw rate against the raw tag yaw rate on ``grid_ns``."""
    imu_rate = np.where(imu.valid, imu.ccw_rate_rad_s, np.nan)
    imu_on_grid = resample_linear(imu.app_clock_stamp_ns, imu_rate, grid_ns, max_gap_s=0.05)
    if np.isfinite(imu_on_grid).sum() < 50:
        return None, {"samples": int(np.isfinite(imu_on_grid).sum())}
    lag = cross_correlation_delay(opts.grid_dt_s, tag_rate, imu_on_grid, max_lag_s=0.2)
    lag_dict: dict[str, Any] = lag.as_dict()
    shift = (
        int(round(lag.b_lag_behind_a_s / opts.grid_dt_s))
        if math.isfinite(lag.b_lag_behind_a_s)
        else 0
    )
    aligned = np.full_like(imu_on_grid, np.nan)
    if shift >= 0:
        aligned[: aligned.size - shift] = imu_on_grid[shift:]
    else:
        aligned[-shift:] = imu_on_grid[: aligned.size + shift]
    aligned = aligned * lag.sign
    spin = np.abs(tag_rate) > opts.spin_rate
    ok = spin & np.isfinite(aligned)
    still = (np.abs(tag_rate) < opts.still_yaw_rate) & np.isfinite(imu_on_grid)
    agreement: dict[str, Any] = {
        "heading_sign_assumed": -1,
        "measured_sign": lag.sign,
        "lag_s": lag.b_lag_behind_a_s,
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
        lin = ok & (np.abs(tag_rate) < IMU_GAIN_MAX_RATE)
        if lin.sum() >= 10:
            gain = float(
                np.dot(aligned[lin], tag_rate[lin])
                / max(np.dot(tag_rate[lin], tag_rate[lin]), 1e-9)
            )
            agreement["linear_gain"] = gain
            corrected = aligned[lin] / gain - tag_rate[lin]
            agreement["linear_rms_rad_s"] = float(np.sqrt(np.mean(corrected**2)))
    return lag_dict, agreement
