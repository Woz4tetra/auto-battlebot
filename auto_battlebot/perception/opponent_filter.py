"""Batched mirror of the KalmanMotionEstimator opponent arm.

One filter per batch row, vectorized in torch, so the same code serves the grid search over logged
measurements, the parity test against the C++ golden fixture, and the RL training environment
(one filter per simulated world). It mirrors `src/robot_filter/kalman_motion_estimator.cpp` for a
single non-OURS track: the 6-state model `[px, py, vx, vy, theta, omega]`, the snapshot ring and
rewind, the gated position update, the ungated heading row with its flip test, reinitialization,
and the render (wall stop, clip, sigmas).

Any change to the C++ opponent arm lands here in the same commit;
`tests/test_kalman_motion_estimator.cpp` checks the C++ side and
`tests/python/test_opponent_filter.py` checks this side against the same fixture.

Config fields accept a float or a per-row tensor, so a grid search can run every candidate as its
own row of one batch.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch

SNAPSHOT_CAPACITY = 64
MAX_INTEGRATION_STEP_S = 1.0
COAST_STALE_AGE_S = 0.1
MAX_HEADING_VARIANCE = math.pi * math.pi
FLIP_TEST_MAX_VARIANCE = (math.pi / 4.0) ** 2
PX, PY, VX, VY, THETA, OMEGA = range(6)
N = 6


@dataclass
class OpponentFilterConfig:
    """The KalmanMotionEstimatorConfiguration fields the opponent arm reads."""

    opponent_accel_psd: float | torch.Tensor = 30.0
    opponent_yaw_accel_psd: float | torch.Tensor = 100.0
    opponent_heading_sigma_rad: float | torch.Tensor = 0.15
    initial_velocity_sigma: float | torch.Tensor = 3.0
    initial_yaw_rate_sigma: float | torch.Tensor = 20.0
    gate_nis: float | torch.Tensor = 11.83
    reinit_after_rejects: int = 5
    heading_flip_reinit_count: int = 3
    covariance_floor: float = 1e-6
    opponent_max_coast_s: float | torch.Tensor = 0.1
    render_lead_s: float = 0.0


@dataclass
class RenderedEstimate:
    """What render_opponent writes into RobotDescription, one row per track."""

    x: torch.Tensor
    y: torch.Tensor
    yaw: torch.Tensor
    vx: torch.Tensor
    vy: torch.Tensor
    omega: torch.Tensor
    position_sigma_m: torch.Tensor
    heading_sigma_rad: torch.Tensor
    yaw_rate_sigma_rad_s: torch.Tensor
    is_stale: torch.Tensor
    active: torch.Tensor


@dataclass
class UpdateStats:
    """Per-row outcome of one update(), for the diagnostics the C++ counts."""

    accepted: torch.Tensor
    position_nis: torch.Tensor
    heading_nis: torch.Tensor
    heading_updated: torch.Tensor
    heading_flip: torch.Tensor
    heading_reinit: torch.Tensor
    reinit: torch.Tensor
    rewind_missed: torch.Tensor


def wrap_angle(angle: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(angle), torch.cos(angle))


def _symmetrize(p: torch.Tensor) -> torch.Tensor:
    return (p + p.transpose(-1, -2)) * 0.5


class OpponentFilter:
    """`batch` independent opponent tracks.

    `field_half_extent` is the field's half size `(B, 2)` or `None` for no field (the C++ skips the
    clip and the wall stop when the field size is not positive). `margin` is
    `field_bounds_margin_meters`. `heading_blind` marks rows whose label is in
    `heading_blind_labels`.
    """

    def __init__(
        self,
        batch: int,
        config: OpponentFilterConfig,
        field_half_extent: torch.Tensor | None = None,
        margin: float = 0.0,
        heading_blind: torch.Tensor | None = None,
        device: torch.device | str = "cpu",
        dtype: torch.dtype = torch.float64,
    ) -> None:
        self.batch = batch
        self.device = torch.device(device)
        self.dtype = dtype
        self.config = config

        def per_row(value: float | torch.Tensor) -> torch.Tensor:
            return torch.as_tensor(value, device=self.device, dtype=dtype).expand(batch).clone()

        self.opponent_accel_psd = per_row(config.opponent_accel_psd)
        self.opponent_yaw_accel_psd = per_row(config.opponent_yaw_accel_psd)
        self.opponent_heading_sigma_rad = per_row(config.opponent_heading_sigma_rad)
        self.initial_velocity_sigma = per_row(config.initial_velocity_sigma)
        self.initial_yaw_rate_sigma = per_row(config.initial_yaw_rate_sigma)
        self.gate_nis = per_row(config.gate_nis)
        self.opponent_max_coast_s = per_row(config.opponent_max_coast_s)
        self.reinit_after_rejects = int(config.reinit_after_rejects)
        self.heading_flip_reinit_count = int(config.heading_flip_reinit_count)
        self.covariance_floor = float(config.covariance_floor)
        self.render_lead_s = float(config.render_lead_s)

        self.bounds = (
            None
            if field_half_extent is None
            else field_half_extent.to(device=self.device, dtype=dtype) + margin
        )
        self.heading_blind = (
            torch.zeros(batch, dtype=torch.bool, device=self.device)
            if heading_blind is None
            else heading_blind.to(self.device)
        )

        b = batch
        self.active = torch.zeros(b, dtype=torch.bool, device=self.device)
        self.x = torch.zeros(b, N, device=self.device, dtype=dtype)
        self.p = torch.zeros(b, N, N, device=self.device, dtype=dtype)
        self.stamp = torch.zeros(b, device=self.device, dtype=dtype)
        self.last_measured = torch.zeros(b, device=self.device, dtype=dtype)
        self.heading_initialized = torch.zeros(b, dtype=torch.bool, device=self.device)
        self.consecutive_flips = torch.zeros(b, dtype=torch.long, device=self.device)
        self.consecutive_rejects = torch.zeros(b, dtype=torch.long, device=self.device)
        self.output_stale = torch.zeros(b, dtype=torch.bool, device=self.device)
        # Yaw of the latest accepted measurement's rotation: what the C++ keeps in
        # track.description and renders when the heading is unknown.
        self.measured_yaw = torch.zeros(b, device=self.device, dtype=dtype)

        c = SNAPSHOT_CAPACITY
        self.snap_stamp = torch.zeros(b, c, device=self.device, dtype=dtype)
        self.snap_x = torch.zeros(b, c, N, device=self.device, dtype=dtype)
        self.snap_p = torch.zeros(b, c, N, N, device=self.device, dtype=dtype)
        self.snap_heading = torch.zeros(b, c, dtype=torch.bool, device=self.device)
        self.snap_start = torch.zeros(b, dtype=torch.long, device=self.device)
        self.snap_count = torch.zeros(b, dtype=torch.long, device=self.device)

    # ---------------------------------------------------------------- helpers

    def _rows(self, value: float | torch.Tensor) -> torch.Tensor:
        return torch.as_tensor(value, device=self.device, dtype=self.dtype).expand(self.batch)

    def _push_snapshot(self, mask: torch.Tensor) -> None:
        full = self.snap_count == SNAPSHOT_CAPACITY
        index = torch.where(
            full, self.snap_start, (self.snap_start + self.snap_count) % SNAPSHOT_CAPACITY
        )
        rows = mask.nonzero(as_tuple=True)[0]
        if rows.numel() == 0:
            return
        cols = index[rows]
        self.snap_stamp[rows, cols] = self.stamp[rows]
        self.snap_x[rows, cols] = self.x[rows]
        self.snap_p[rows, cols] = self.p[rows]
        self.snap_heading[rows, cols] = self.heading_initialized[rows]
        self.snap_start = torch.where(
            mask & full, (self.snap_start + 1) % SNAPSHOT_CAPACITY, self.snap_start
        )
        self.snap_count = torch.where(mask & ~full, self.snap_count + 1, self.snap_count)

    def _rewind(self, target: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
        """Restores the newest snapshot at or before `target`. Returns the rows that rewound."""
        offsets = torch.arange(SNAPSHOT_CAPACITY, device=self.device)
        order = (self.snap_start[:, None] + offsets[None, :]) % SNAPSHOT_CAPACITY
        stamps = torch.gather(self.snap_stamp, 1, order)
        valid = offsets[None, :] < self.snap_count[:, None]
        ok = valid & (stamps <= target[:, None])
        # Snapshots are time ordered, and the C++ scan stops at the first one past the target.
        leading = torch.cumprod(ok.long(), dim=1).sum(dim=1)
        rewound = mask & (leading > 0)
        rows = rewound.nonzero(as_tuple=True)[0]
        if rows.numel() == 0:
            return rewound
        found = leading[rows] - 1
        cols = order[rows, found]
        self.x[rows] = self.snap_x[rows, cols]
        self.p[rows] = self.snap_p[rows, cols]
        self.heading_initialized[rows] = self.snap_heading[rows, cols]
        self.stamp[rows] = self.snap_stamp[rows, cols]
        self.snap_count[rows] = found + 1
        return rewound

    def _advance(self, target: torch.Tensor, mask: torch.Tensor) -> None:
        dt_total = target - self.stamp
        move = mask & (dt_total > 0.0)
        if not bool(move.any()):
            return
        dt = torch.clamp(dt_total, max=MAX_INTEGRATION_STEP_S)
        age = self.stamp - self.last_measured
        # Only the part of the step before the coast horizon moves the state.
        move_dt = torch.where(
            dt_total > MAX_INTEGRATION_STEP_S,
            torch.zeros_like(dt),
            torch.minimum(dt, torch.clamp(self.opponent_max_coast_s - age, min=0.0)),
        )

        # The state holds past the horizon; the covariance always propagates through the full
        # transition, so the position sigma keeps growing with the velocity uncertainty.
        f = torch.eye(N, device=self.device, dtype=self.dtype).repeat(self.batch, 1, 1)
        f[:, PX, VX] = dt
        f[:, PY, VY] = dt
        f[:, THETA, OMEGA] = dt

        dt2 = dt * dt
        dt3 = dt2 * dt
        q = torch.zeros(self.batch, N, N, device=self.device, dtype=self.dtype)
        qa = self.opponent_accel_psd
        q[:, PX, PX] = q[:, PY, PY] = qa * dt3 / 3.0
        q[:, PX, VX] = q[:, VX, PX] = qa * dt2 / 2.0
        q[:, PY, VY] = q[:, VY, PY] = qa * dt2 / 2.0
        q[:, VX, VX] = q[:, VY, VY] = qa * dt
        qy = self.opponent_yaw_accel_psd
        q[:, THETA, THETA] = qy * dt3 / 3.0
        q[:, THETA, OMEGA] = q[:, OMEGA, THETA] = qy * dt2 / 2.0
        q[:, OMEGA, OMEGA] = qy * dt

        x = self.x.clone()
        x[:, PX] = self.x[:, PX] + move_dt * self.x[:, VX]
        x[:, PY] = self.x[:, PY] + move_dt * self.x[:, VY]
        x[:, THETA] = self.x[:, THETA] + move_dt * self.x[:, OMEGA]
        x[:, THETA] = torch.where(move_dt > 0.0, wrap_angle(x[:, THETA]), self.x[:, THETA])
        p = _symmetrize(f @ self.p @ f.transpose(1, 2) + q)
        # Caps, in the C++ order: velocity, heading, yaw rate. A row and its column scale together.
        velocity_cap = self.initial_velocity_sigma**2
        yaw_rate_cap = self.initial_yaw_rate_sigma**2
        heading_cap = torch.full_like(velocity_cap, MAX_HEADING_VARIANCE)
        for index, cap in (
            (VX, velocity_cap),
            (VY, velocity_cap),
            (THETA, heading_cap),
            (OMEGA, yaw_rate_cap),
        ):
            v = p[:, index, index]
            scale = torch.where(
                v > cap, torch.sqrt(cap / torch.clamp(v, min=1e-300)), torch.ones_like(v)
            )
            p[:, index, :] *= scale[:, None]
            p[:, :, index] *= scale[:, None]

        self.x = torch.where(move[:, None], x, self.x)
        self.p = torch.where(move[:, None, None], p, self.p)
        self.stamp = torch.where(move, target, self.stamp)
        self._push_snapshot(move)

    def _ekf_update(
        self,
        mask: torch.Tensor,
        measured: torch.Tensor,
        h: torch.Tensor,
        r: torch.Tensor,
        gate: torch.Tensor,
        angle_rows: list[bool],
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """ekf::ekf_update over masked rows. `h` (M, N), `r` (B, M, M), `measured` (B, M)."""
        predicted = torch.einsum("mn,bn->bm", h, self.x)
        innovation = measured - predicted
        for i, is_angle in enumerate(angle_rows):
            if is_angle:
                innovation[:, i] = wrap_angle(innovation[:, i])
        s = h @ self.p @ h.T + r
        chol, info = torch.linalg.cholesky_ex(s)
        ok = mask & (info == 0)
        eye = torch.eye(h.shape[0], device=self.device, dtype=self.dtype).expand_as(s)
        s_inv = torch.cholesky_solve(eye, chol)
        nis = torch.einsum("bm,bmk,bk->b", innovation, s_inv, innovation)
        accepted = ok & ((gate <= 0.0) | (nis <= gate))

        gain = self.p @ h.T @ s_inv
        x = self.x + torch.einsum("bnm,bm->bn", gain, innovation)
        x[:, THETA] = wrap_angle(x[:, THETA])
        i_kh = torch.eye(N, device=self.device, dtype=self.dtype) - gain @ h
        p = i_kh @ self.p @ i_kh.transpose(1, 2) + gain @ r @ gain.transpose(1, 2)
        p = _symmetrize(p)
        diag = torch.diagonal(p, dim1=1, dim2=2)
        floored = torch.clamp(diag, min=self.covariance_floor)
        p = p - torch.diag_embed(diag) + torch.diag_embed(floored)

        self.x = torch.where(accepted[:, None], x, self.x)
        self.p = torch.where(accepted[:, None, None], p, self.p)
        return accepted, nis

    def _init_heading(self, mask: torch.Tensor, yaw: torch.Tensor) -> None:
        rows = mask.nonzero(as_tuple=True)[0]
        if rows.numel() == 0:
            return
        self.x[rows, THETA] = wrap_angle(yaw[rows])
        self.x[rows, OMEGA] = 0.0
        for i in (THETA, OMEGA):
            self.p[rows, i, :] = 0.0
            self.p[rows, :, i] = 0.0
        self.p[rows, THETA, THETA] = self.opponent_heading_sigma_rad[rows] ** 2
        self.p[rows, OMEGA, OMEGA] = self.initial_yaw_rate_sigma[rows] ** 2
        self.heading_initialized[rows] = True
        self.consecutive_flips[rows] = 0

    def _init_track(
        self,
        mask: torch.Tensor,
        xy: torch.Tensor,
        yaw: torch.Tensor,
        measures_heading: torch.Tensor,
        position_variance: torch.Tensor,
        stamp: torch.Tensor,
    ) -> None:
        rows = mask.nonzero(as_tuple=True)[0]
        if rows.numel() == 0:
            return
        self.x[rows] = 0.0
        self.x[rows, PX] = xy[rows, 0]
        self.x[rows, PY] = xy[rows, 1]
        self.p[rows] = 0.0
        self.p[rows, PX, PX] = position_variance[rows]
        self.p[rows, PY, PY] = position_variance[rows]
        vel_var = self.initial_velocity_sigma[rows] ** 2
        self.p[rows, VX, VX] = vel_var
        self.p[rows, VY, VY] = vel_var
        self.p[rows, THETA, THETA] = MAX_HEADING_VARIANCE
        self.p[rows, OMEGA, OMEGA] = self.initial_yaw_rate_sigma[rows] ** 2
        self.heading_initialized[rows] = False
        self.consecutive_flips[rows] = 0
        self._init_heading(mask & measures_heading, yaw)
        self.stamp[rows] = stamp[rows]
        self.last_measured[rows] = stamp[rows]
        self.consecutive_rejects[rows] = 0
        self.output_stale[rows] = False
        self.measured_yaw[rows] = yaw[rows]
        self.active[rows] = True
        self.snap_start[rows] = 0
        self.snap_count[rows] = 0
        self._push_snapshot(mask)

    # ---------------------------------------------------------------- public API

    def update(
        self,
        stamp: float | torch.Tensor,
        measured: torch.Tensor,
        xy: torch.Tensor,
        yaw: torch.Tensor,
        is_keypoint: torch.Tensor,
        position_variance: torch.Tensor,
    ) -> tuple[RenderedEstimate, UpdateStats]:
        """KalmanMotionEstimator::update for the opponent rows.

        `measured` marks rows with a measurement this frame. `yaw` is the measurement's rotation
        (0 for a blob, whose rotation is identity), `is_keypoint` marks keypoint poses, and
        `position_variance` is measurement_noise_for's diagonal.
        """
        t = self._rows(stamp)
        xy = xy.to(device=self.device, dtype=self.dtype)
        yaw = yaw.to(device=self.device, dtype=self.dtype)
        position_variance = self._rows(position_variance)
        measured = measured.to(self.device)
        measures_heading = is_keypoint.to(self.device) & ~self.heading_blind
        false = torch.zeros(self.batch, dtype=torch.bool, device=self.device)

        new = measured & ~self.active
        self._init_track(new, xy, yaw, measures_heading, position_variance, t)
        existing = measured & ~new

        late = existing & (t < self.stamp)
        rewound = self._rewind(t, late)
        rewind_missed = late & ~rewound
        self._advance(t, existing)

        r = torch.zeros(self.batch, 2, 2, device=self.device, dtype=self.dtype)
        r[:, 0, 0] = position_variance
        r[:, 1, 1] = position_variance
        h = torch.zeros(2, N, device=self.device, dtype=self.dtype)
        h[0, PX] = 1.0
        h[1, PY] = 1.0
        accepted, position_nis = self._ekf_update(existing, xy, h, r, self.gate_nis, [False, False])

        heading_rows = accepted & measures_heading
        first_heading = heading_rows & ~self.heading_initialized
        tracked = heading_rows & self.heading_initialized
        within = (wrap_angle(yaw - self.x[:, THETA]).abs() <= math.pi / 2.0) | (
            self.p[:, THETA, THETA] >= FLIP_TEST_MAX_VARIANCE
        )
        do_update = tracked & within
        flip = tracked & ~within
        self._init_heading(first_heading, yaw)

        hh = torch.zeros(1, N, device=self.device, dtype=self.dtype)
        hh[0, THETA] = 1.0
        rh = (self.opponent_heading_sigma_rad**2)[:, None, None]
        _, heading_nis = self._ekf_update(
            do_update, yaw[:, None], hh, rh, torch.zeros_like(t), [True]
        )
        self.consecutive_flips = torch.where(
            do_update, torch.zeros_like(self.consecutive_flips), self.consecutive_flips
        )
        self.consecutive_flips = torch.where(
            flip, self.consecutive_flips + 1, self.consecutive_flips
        )
        heading_reinit = flip & (self.consecutive_flips >= self.heading_flip_reinit_count)
        self._init_heading(heading_reinit, yaw)

        self.last_measured = torch.where(accepted, t, self.last_measured)
        self.consecutive_rejects = torch.where(
            accepted, torch.zeros_like(self.consecutive_rejects), self.consecutive_rejects
        )
        self.output_stale = torch.where(accepted, false, self.output_stale)
        self.measured_yaw = torch.where(accepted, yaw, self.measured_yaw)
        self._push_snapshot(accepted)

        rejected = existing & ~accepted
        self.output_stale = self.output_stale | rejected
        self.consecutive_rejects = torch.where(
            rejected, self.consecutive_rejects + 1, self.consecutive_rejects
        )
        reinit = rejected & (self.consecutive_rejects >= self.reinit_after_rejects)
        self._init_track(reinit, xy, yaw, measures_heading, position_variance, t)

        self.output_stale = self.output_stale | (self.active & ~measured)
        self._advance(t, self.active & self.output_stale)

        stats = UpdateStats(
            accepted=accepted,
            position_nis=position_nis,
            heading_nis=heading_nis,
            heading_updated=do_update,
            heading_flip=flip,
            heading_reinit=heading_reinit,
            reinit=reinit,
            rewind_missed=rewind_missed,
        )
        return self.render(self.output_stale), stats

    def coast(self, now: float | torch.Tensor) -> RenderedEstimate:
        """KalmanMotionEstimator::coast for the opponent rows: advance to now + lead, render."""
        t = self._rows(now)
        self._advance(t + self.render_lead_s, self.active)
        stale = self.output_stale | ((t - self.last_measured) > COAST_STALE_AGE_S)
        return self.render(stale)

    def render(self, stale: torch.Tensor) -> RenderedEstimate:
        x = self.x[:, PX].clone()
        y = self.x[:, PY].clone()
        vx = self.x[:, VX].clone()
        vy = self.x[:, VY].clone()
        if self.bounds is not None:
            hx = self.bounds[:, 0]
            hy = self.bounds[:, 1]
            vx = torch.where(((x >= hx) & (vx > 0.0)) | ((x <= -hx) & (vx < 0.0)), 0.0, vx)
            vy = torch.where(((y >= hy) & (vy > 0.0)) | ((y <= -hy) & (vy < 0.0)), 0.0, vy)
            x = torch.minimum(torch.maximum(x, -hx), hx)
            y = torch.minimum(torch.maximum(y, -hy), hy)

        a = self.p[:, PX, PX]
        c = self.p[:, PY, PY]
        b = self.p[:, PX, PY]
        largest = 0.5 * (a + c) + torch.sqrt(0.25 * (a - c) ** 2 + b * b)
        position_sigma = torch.sqrt(torch.clamp(largest, min=0.0))

        known = self.heading_initialized & ~self.heading_blind
        inf = torch.full_like(x, math.inf)
        return RenderedEstimate(
            x=x,
            y=y,
            yaw=torch.where(known, self.x[:, THETA], self.measured_yaw),
            vx=vx,
            vy=vy,
            omega=torch.where(known, self.x[:, OMEGA], torch.zeros_like(x)),
            position_sigma_m=position_sigma,
            heading_sigma_rad=torch.where(
                known, torch.sqrt(torch.clamp(self.p[:, THETA, THETA], min=0.0)), inf
            ),
            yaw_rate_sigma_rad_s=torch.where(
                known, torch.sqrt(torch.clamp(self.p[:, OMEGA, OMEGA], min=0.0)), inf
            ),
            is_stale=stale.clone(),
            active=self.active.clone(),
        )
