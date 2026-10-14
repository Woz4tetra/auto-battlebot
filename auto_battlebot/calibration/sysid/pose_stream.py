"""Causal robot pose from the AprilTag stream, for the live sysid monitor.

The offline path (``tag_pose_smoother.smooth_session``) runs a two-pass RTS smoother over the
whole session. The live monitor only needs two things from the tags, and neither needs the
future past a short lag:

- one body pose per detection, for the fit residual and the gates. Each frame goes through the
  offline ``build_candidates`` and ``select_ippe``; ``select_ippe`` looks ahead up to its
  ``neighbour_window_s`` on close calls, so a detection is final once it is that much older
  than the newest frame.
- the state each fit window starts from. ``state_at`` fits a weighted quadratic in time to the
  poses within ``STATE_HALF_SPAN_S`` either side of the start, which is a fixed-lag smoother of
  that one instant. Velocity is its slope projected on the heading; yaw rate comes from the
  gyro, which the caller passes in.

Frames follow ``tag_pose_smoother``: the output frame is the recorded field frame turned so z
points up, the body frame is FLU at the axle midpoint, and yaw is counter-clockwise.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from auto_battlebot.perception.tag_pose_smoother import (
    Candidates,
    NoiseModel,
    TagMount,
    build_candidates,
    chosen_poses,
    select_ippe,
    to_view,
    up_from_field,
    wrap_angle,
)
from auto_battlebot.recording.sysid_io import FieldFromCamera, RobotTagsFrame

_NS = 1_000_000_000
STATE_HALF_SPAN_S = 0.15
MIN_STATE_POSES = 5
NEIGHBOUR_WINDOW_S = 0.25
BUFFER_S = 1.5


@dataclass
class TagPose:
    stamp_ns: int
    x: float
    y: float
    yaw: float
    pitch: float  # relative to the rest pose: zero sitting flat
    upside_down: bool
    sigma_along: float
    sigma_across: float
    sigma_yaw: float


@dataclass
class StateEstimate:
    x: float
    y: float
    theta: float
    v: float
    poses: int


def _concat(a: Candidates, b: Candidates) -> Candidates:
    return Candidates(
        stamp_ns=np.concatenate([a.stamp_ns, b.stamp_ns]),
        frame_index=np.concatenate([a.frame_index, b.frame_index]),
        tag_id=np.concatenate([a.tag_id, b.tag_id]),
        upside_down=np.concatenate([a.upside_down, b.upside_down]),
        n_solutions=np.concatenate([a.n_solutions, b.n_solutions]),
        t_up_body=np.concatenate([a.t_up_body, b.t_up_body]),
        tilt=np.concatenate([a.tilt, b.tilt]),
        reprojection_error_px=np.concatenate([a.reprojection_error_px, b.reprojection_error_px]),
        size_px=np.concatenate([a.size_px, b.size_px]),
    )


def _slice(c: Candidates, keep: np.ndarray) -> Candidates:
    return Candidates(
        stamp_ns=c.stamp_ns[keep],
        frame_index=c.frame_index[keep],
        tag_id=c.tag_id[keep],
        upside_down=c.upside_down[keep],
        n_solutions=c.n_solutions[keep],
        t_up_body=c.t_up_body[keep],
        tilt=c.tilt[keep],
        reprojection_error_px=c.reprojection_error_px[keep],
        size_px=c.size_px[keep],
    )


def _empty_candidates() -> Candidates:
    return Candidates(
        stamp_ns=np.zeros(0, np.int64),
        frame_index=np.zeros(0, np.int64),
        tag_id=np.zeros(0, np.int64),
        upside_down=np.zeros(0, bool),
        n_solutions=np.zeros(0, np.int64),
        t_up_body=np.zeros((0, 2, 4, 4)),
        tilt=np.zeros((0, 2)),
        reprojection_error_px=np.zeros((0, 2)),
        size_px=np.zeros(0),
    )


class PoseStream:
    """Tag frames in, final body poses out, in stamp order."""

    def __init__(self, mounts: dict[int, TagMount], rest_pitch_rad: float) -> None:
        self.mounts = mounts
        self.rest_pitch_rad = rest_pitch_rad
        self.noise = NoiseModel()
        self._field: np.ndarray | None = None
        self._t_up_field: np.ndarray | None = None
        self._buf = _empty_candidates()
        self._emitted_ns = -1
        self.frame_stamps: deque[int] = deque()
        self.poses: deque[TagPose] = deque()
        self.newest_frame_ns = 0

    @property
    def has_field(self) -> bool:
        return self._field is not None

    def set_field(self, t_field_camera: np.ndarray) -> None:
        """The camera pose in the field frame. The first one fixes the output frame."""
        self._field = np.asarray(t_field_camera, dtype=float)
        if self._t_up_field is None:
            series = FieldFromCamera(np.zeros(1, np.int64), self._field[None])
            self._t_up_field, _flipped = up_from_field(series)
            axis = (self._t_up_field @ self._field)[:3, 2]
            self.noise.view_azimuth_rad = math.atan2(float(axis[1]), float(axis[0]))

    def push_frame(self, frame: RobotTagsFrame) -> None:
        self.frame_stamps.append(frame.image_stamp_ns)
        self.newest_frame_ns = max(self.newest_frame_ns, frame.image_stamp_ns)
        if self._field is None or self._t_up_field is None:
            return
        series = FieldFromCamera(np.array([frame.image_stamp_ns], np.int64), self._field[None])
        cands = build_candidates(
            [frame],
            series,
            self.mounts,
            self._t_up_field,
            rest_pitch_rad=self.rest_pitch_rad,
        )
        if len(cands):
            self._buf = _concat(self._buf, cands)

    def finalize(self) -> list[TagPose]:
        """Emit every detection older than the IPPE look-ahead, and trim the buffers."""
        if len(self._buf) == 0:
            return []
        ready_ns = self.newest_frame_ns - int(NEIGHBOUR_WINDOW_S * _NS)
        choice = select_ippe(self._buf, neighbour_window_s=NEIGHBOUR_WINDOW_S)
        raw = chosen_poses(self._buf, choice)
        sig = self.noise.sigmas(
            self._buf.reprojection_error_px[np.arange(len(self._buf)), choice.choice],
            self._buf.size_px,
        )
        out: list[TagPose] = []
        for i in range(len(self._buf)):
            s = int(raw.stamp_ns[i])
            if s <= self._emitted_ns or s > ready_ns:
                continue
            out.append(
                TagPose(
                    stamp_ns=s,
                    x=float(raw.x[i]),
                    y=float(raw.y[i]),
                    yaw=float(raw.yaw[i]),
                    pitch=float(raw.pitch[i])
                    - (0.0 if self._buf.upside_down[i] else self.rest_pitch_rad),
                    upside_down=bool(self._buf.upside_down[i]),
                    sigma_along=float(sig["along"][i]),
                    sigma_across=float(sig["across"][i]),
                    sigma_yaw=float(sig["yaw"][i]),
                )
            )
        if out:
            self._emitted_ns = max(p.stamp_ns for p in out)
        self.poses.extend(out)
        keep = self._buf.stamp_ns >= self.newest_frame_ns - int(BUFFER_S * _NS)
        self._buf = _slice(self._buf, keep)
        return out

    def trim(self, before_ns: int) -> None:
        while self.poses and self.poses[0].stamp_ns < before_ns:
            self.poses.popleft()
        while self.frame_stamps and self.frame_stamps[0] < before_ns:
            self.frame_stamps.popleft()

    def view(self, dx: np.ndarray, dy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(along, across) components of a field-frame offset."""
        return to_view(dx, dy, self.noise.view_azimuth_rad)


def state_at(poses: list[TagPose], t_ns: int) -> StateEstimate | None:
    """Position, heading and forward speed at ``t_ns`` from the poses around it.

    Needs ``MIN_STATE_POSES`` poses within ``STATE_HALF_SPAN_S`` and at least two on each side,
    so the slope is an interpolation and not an extrapolation.
    """
    half = int(STATE_HALF_SPAN_S * _NS)
    near = [p for p in poses if abs(p.stamp_ns - t_ns) <= half]
    if len(near) < MIN_STATE_POSES:
        return None
    tau = np.array([(p.stamp_ns - t_ns) / _NS for p in near])
    if np.sum(tau < 0) < 2 or np.sum(tau > 0) < 2:
        return None
    x = np.array([p.x for p in near])
    y = np.array([p.y for p in near])
    yaw = np.unwrap(np.array([p.yaw for p in near]))
    w_pos = 1.0 / np.array([max(p.sigma_across, 1e-3) for p in near]) ** 2
    w_yaw = 1.0 / np.array([max(p.sigma_yaw, 1e-3) for p in near]) ** 2
    a = np.stack([np.ones_like(tau), tau, tau**2], axis=1)
    cx = np.linalg.lstsq(a * np.sqrt(w_pos)[:, None], x * np.sqrt(w_pos), rcond=None)[0]
    cy = np.linalg.lstsq(a * np.sqrt(w_pos)[:, None], y * np.sqrt(w_pos), rcond=None)[0]
    cyaw = np.linalg.lstsq(a * np.sqrt(w_yaw)[:, None], yaw * np.sqrt(w_yaw), rcond=None)[0]
    theta = float(wrap_angle(cyaw[0]))
    v = float(cx[1] * math.cos(theta) + cy[1] * math.sin(theta))
    return StateEstimate(x=float(cx[0]), y=float(cy[0]), theta=theta, v=v, poses=len(near))
