"""The fit behind the sysid monitor: window residuals, Fisher information, refits, suggestions.

Each 1.5 s fit window is cut into three 0.5 s sub-windows. A sub-window starts from the tag
state estimate at its start (``pose_stream.state_at`` plus the gyro) and runs the plant open loop
on the recorded motor commands for 0.5 s, the horizon the Kalman plan scores. Its residuals are:

- tag position, along and across the camera view, at each tag fix, over the per-fix sigma
  floored at ``POS_FLOOR_M`` for model error,
- tag heading at each fix, likewise floored at ``YAW_FLOOR_RAD``,
- gyro yaw rate every 20 ms over ``SIGMA_GYRO``.

Heading has to be in there: with velocity and yaw rate alone the two drift terms never separated
in the prototype.

The fit input is the motor command (``firmware_mixer.motor_to_plant`` of ``left_cmd``/
``right_cmd``), so the same 16 drivetrain parameters come out with the yaw loop on or off and
the loop's feedback stays out of the Fisher matrix. Delay is one of the 16: commands are shifted
by a fractional, linearly interpolated delay, which keeps the residual smooth in it.

Predictions for the page (holdout, ghosts, suggestions) instead run the sticks through
``firmware_mixer.closed_loop_rollout``, so they score the plant the Jetson would command.

Uncertainty is the Cramer-Rao bound ``s^2 (J^T J + prior)^-1`` with ``s^2`` the fit's residual
variance per row when that exceeds 1, so a model that cannot reach the tag noise reports wider
bars instead of tighter ones.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np
from scipy.optimize import least_squares

from auto_battlebot.control.firmware_mixer import (
    LoopState,
    closed_loop_rollout,
    motor_to_plant,
)
from auto_battlebot.control.plant import (
    FULL_MODEL,
    MODEL_LADDER,
    PARAM_BOUNDS,
    ModelStructure,
    PlantParams,
    PlantState,
    simulate,
    wrap_angle,
)

DT = 0.01  # s, the grid everything is resampled onto
WINDOW_S = 1.5
SUB_S = 0.5
SUBS = int(round(WINDOW_S / SUB_S))
SUB_N = int(round(SUB_S / DT))
HIST_S = 0.25  # command history before each sub-window; covers the 0.2 s delay bound
HIST_N = int(round(HIST_S / DT))
MAX_TAGS = 16  # tag fixes kept per sub-window, evenly thinned
GYRO_EVERY = 2  # gyro residual every 20 ms
# Gyro samples per sub-window, at GYRO_EVERY, 2 * GYRO_EVERY, ... samples after the start. The
# sample at the start itself seeds the yaw rate instead.
GYRO_N = (SUB_N - 1) // GYRO_EVERY
FIT_SUBSTEP_S = 0.005

POS_FLOOR_M = 0.01
YAW_FLOOR_RAD = 0.03
SIGMA_GYRO = 0.3  # rad/s

# Fit parameters, with their priors (``PlantParams`` defaults, the stage 2 values), prior
# sigmas, ring targets and finite-difference steps.
#   targets: 4 ms delay, 6 ms each lag, 3% of the prior for each gain, 0.008 each deadzone,
#   0.03 each coupling term, 0.12 rad/s each drift term.


@dataclass(frozen=True)
class ParamSpec:
    key: str
    short: str
    label: str
    group: str
    unit: str
    dp: int
    prior_sd: float
    target_sd: float
    step: float


PARAMS: tuple[ParamSpec, ...] = (
    ParamSpec("dz_lin_fwd", "dz fwd", "Deadzone, forward", "Input map", "", 3, 0.05, 0.008, 1e-3),
    ParamSpec("dz_lin_rev", "dz rev", "Deadzone, reverse", "Input map", "", 3, 0.05, 0.008, 1e-3),
    ParamSpec("dz_ang_l", "dz L", "Deadzone, spin left", "Input map", "", 3, 0.05, 0.008, 1e-3),
    ParamSpec("dz_ang_r", "dz R", "Deadzone, spin right", "Input map", "", 3, 0.05, 0.008, 1e-3),
    ParamSpec("k_fwd", "k fwd", "Gain, forward", "Input map", "m/s", 2, 1.5, 0.168, 1e-2),
    ParamSpec("k_rev", "k rev", "Gain, reverse", "Input map", "m/s", 2, 1.5, 0.145, 1e-2),
    ParamSpec("k_ang", "k ω", "Gain, yaw", "Input map", "rad/s", 1, 15.0, 1.85, 0.1),
    ParamSpec(
        "tau_lin_a", "τ lin a", "Lag, linear accel", "Actuator lag", "s", 3, 0.04, 0.006, 1e-3
    ),
    ParamSpec(
        "tau_lin_d", "τ lin d", "Lag, linear decel", "Actuator lag", "s", 3, 0.04, 0.006, 1e-3
    ),
    ParamSpec("tau_ang_a", "τ ang a", "Lag, yaw accel", "Actuator lag", "s", 3, 0.04, 0.006, 1e-3),
    ParamSpec("tau_ang_d", "τ ang d", "Lag, yaw decel", "Actuator lag", "s", 3, 0.04, 0.006, 1e-3),
    ParamSpec("delay_s", "delay", "Transport delay", "Delay", "s", 3, 0.03, 0.004, 1e-3),
    ParamSpec("c_sb", "c sb", "Steer-brake", "Coupling", "", 2, 0.3, 0.03, 1e-2),
    ParamSpec("c_ad", "c ad", "Angular droop", "Coupling", "", 2, 0.3, 0.03, 1e-2),
    ParamSpec("c_drift", "c drift", "Drift, gain mismatch", "Drift", "rad/s", 2, 1.0, 0.12, 1e-2),
    ParamSpec("c_drift_bias", "c bias", "Drift, drag bias", "Drift", "rad/s", 2, 1.0, 0.12, 1e-2),
)
NAMES = tuple(p.key for p in PARAMS)
NP = len(PARAMS)
PRIOR = PlantParams()
PRIOR_VECTOR = PRIOR.to_vector(NAMES)
PRIOR_SD = np.array([p.prior_sd for p in PARAMS])
TARGET_SD = np.array([p.target_sd for p in PARAMS])
STEPS = np.array([p.step for p in PARAMS])
LO = np.array([PARAM_BOUNDS[n][0] for n in NAMES])
HI = np.array([PARAM_BOUNDS[n][1] for n in NAMES])

# The free parameters of each ladder rung, delay included when the rung uses it.
RUNG_FREE: dict[str, list[str]] = {
    s.name: s.free_names() + (["delay_s"] if s.use_delay else []) for s in MODEL_LADDER
}


def params_of(theta: np.ndarray) -> PlantParams:
    return PRIOR.with_vector(NAMES, theta)


def fills_of(std: np.ndarray) -> np.ndarray:
    out: np.ndarray = np.minimum(1.0, TARGET_SD / np.maximum(std, 1e-12))
    return out


# ---------------------------------------------------------------------------
# Sub-windows
# ---------------------------------------------------------------------------


@dataclass
class SubWindow:
    """One 0.5 s open-loop stretch: its start state, inputs and measurements."""

    state0: np.ndarray  # (5,) x, y, theta, v, w
    cmd_lin: np.ndarray  # (HIST_N + SUB_N,) motor command, history first
    cmd_ang: np.ndarray
    tag_tau: np.ndarray  # (M,) seconds after the start, in (0, SUB_S]
    tag_x: np.ndarray
    tag_y: np.ndarray
    tag_yaw: np.ndarray
    tag_sa: np.ndarray  # sigmas, along and across the view, metres
    tag_sc: np.ndarray
    tag_sy: np.ndarray  # radians
    gyro: np.ndarray  # (GYRO_N,) rad/s CCW, GYRO_EVERY, 2 * GYRO_EVERY, ... samples in
    gyro_ok: np.ndarray  # bool
    # Closed-loop prediction inputs: sticks and the yaw loop seed.
    a_percent: np.ndarray  # (SUB_N,)
    b_percent: np.ndarray
    auto_steer: np.ndarray  # bool (SUB_N,)
    loop_seed: dict[str, float] = field(default_factory=dict)
    end_xy: np.ndarray | None = None  # measured position at the end, for the holdout score


@dataclass
class Batch:
    """Sub-windows stacked for one batched simulation. S rows."""

    state0: np.ndarray  # (S, 5)
    cmd_lin: np.ndarray  # (S, HIST_N + SUB_N)
    cmd_ang: np.ndarray
    tag_pos: np.ndarray  # (S, M) fractional grid index of each fix, 0 = start
    tag_x: np.ndarray
    tag_y: np.ndarray
    tag_yaw: np.ndarray
    tag_wa: np.ndarray  # inverse sigmas, 0 for padding
    tag_wc: np.ndarray
    tag_wy: np.ndarray
    gyro: np.ndarray  # (S, G)
    gyro_w: np.ndarray  # (S, G) inverse sigma, 0 where missing
    owner: np.ndarray  # (S,) index of the window each row came from
    view_c: float
    view_s: float

    @property
    def rows(self) -> int:
        return int(self.state0.shape[0])


def stack(subs: Sequence[SubWindow], owners: Sequence[int], view_azimuth: float) -> Batch:
    s = len(subs)
    g = GYRO_N
    out = Batch(
        state0=np.zeros((s, 5)),
        cmd_lin=np.zeros((s, HIST_N + SUB_N)),
        cmd_ang=np.zeros((s, HIST_N + SUB_N)),
        tag_pos=np.zeros((s, MAX_TAGS)),
        tag_x=np.zeros((s, MAX_TAGS)),
        tag_y=np.zeros((s, MAX_TAGS)),
        tag_yaw=np.zeros((s, MAX_TAGS)),
        tag_wa=np.zeros((s, MAX_TAGS)),
        tag_wc=np.zeros((s, MAX_TAGS)),
        tag_wy=np.zeros((s, MAX_TAGS)),
        gyro=np.zeros((s, g)),
        gyro_w=np.zeros((s, g)),
        owner=np.asarray(owners, dtype=np.int64),
        view_c=float(np.cos(view_azimuth)),
        view_s=float(np.sin(view_azimuth)),
    )
    for i, sub in enumerate(subs):
        out.state0[i] = sub.state0
        out.cmd_lin[i] = sub.cmd_lin
        out.cmd_ang[i] = sub.cmd_ang
        m = min(len(sub.tag_tau), MAX_TAGS)
        out.tag_pos[i, :m] = sub.tag_tau[:m] / DT
        out.tag_x[i, :m] = sub.tag_x[:m]
        out.tag_y[i, :m] = sub.tag_y[:m]
        out.tag_yaw[i, :m] = sub.tag_yaw[:m]
        out.tag_wa[i, :m] = 1.0 / np.hypot(sub.tag_sa[:m], POS_FLOOR_M)
        out.tag_wc[i, :m] = 1.0 / np.hypot(sub.tag_sc[:m], POS_FLOOR_M)
        out.tag_wy[i, :m] = 1.0 / np.hypot(sub.tag_sy[:m], YAW_FLOOR_RAD)
        out.gyro[i] = np.nan_to_num(sub.gyro)
        out.gyro_w[i] = np.where(sub.gyro_ok, 1.0 / SIGMA_GYRO, 0.0)
    return out


def _delayed(cmd: np.ndarray, delay_s: float) -> np.ndarray:
    """Shift each row back by a fractional delay and drop the history: (S, HIST_N + SUB_N) to
    (S, SUB_N). Linear between grid samples, zero before the history starts."""
    shift = delay_s / DT
    idx = np.arange(HIST_N, HIST_N + SUB_N) - shift
    idx = np.clip(idx, 0.0, HIST_N + SUB_N - 1.0)
    lo = np.floor(idx).astype(int)
    hi = np.minimum(lo + 1, HIST_N + SUB_N - 1)
    frac = idx - lo
    out: np.ndarray = cmd[:, lo] * (1.0 - frac) + cmd[:, hi] * frac
    return out


def _interp(track: np.ndarray, pos: np.ndarray) -> np.ndarray:
    """Per-row linear interpolation of a (S, N+1) track at fractional positions (S, M)."""
    pos = np.clip(pos, 0.0, track.shape[1] - 1.0)
    lo = np.floor(pos).astype(int)
    hi = np.minimum(lo + 1, track.shape[1] - 1)
    frac = pos - lo
    rows = np.arange(track.shape[0])[:, None]
    out: np.ndarray = track[rows, lo] * (1.0 - frac) + track[rows, hi] * frac
    return out


# Prior sigmas on each sub-window's start state from the local tag estimate: x, y, theta, v.
# The start state is a nuisance: each sub-window refines it, and the information it could fake
# is projected out of the Jacobian.
NUISANCE_SD = np.array([0.005, 0.005, 0.02, 0.1])
NUISANCE_STEP = np.array([1e-3, 1e-3, 1e-3, 1e-2])
NUISANCE = len(NUISANCE_SD)


def _raw_residuals(
    theta: np.ndarray, batch: Batch, s0: np.ndarray, structure: ModelStructure
) -> np.ndarray:
    p = structure.apply(params_of(theta))
    u_lin = _delayed(batch.cmd_lin, p.delay_s)
    u_ang = _delayed(batch.cmd_ang, p.delay_s)
    state = PlantState(s0[:, 0], s0[:, 1], s0[:, 2], s0[:, 3], s0[:, 4])
    _final, traj = simulate(u_lin, u_ang, DT, p, state, structure, substep_s=FIT_SUBSTEP_S)
    x = np.concatenate([s0[:, :1], traj["x"]], axis=1)
    y = np.concatenate([s0[:, 1:2], traj["y"]], axis=1)
    th = np.concatenate([s0[:, 2:3], traj["theta"]], axis=1)
    dx = _interp(x, batch.tag_pos) - batch.tag_x
    dy = _interp(y, batch.tag_pos) - batch.tag_y
    along = batch.view_c * dx + batch.view_s * dy
    across = -batch.view_s * dx + batch.view_c * dy
    dyaw = wrap_angle(_interp(th, batch.tag_pos) - batch.tag_yaw)
    # traj[k] is the state one step after sample k, so gyro sample j * GYRO_EVERY lines up
    # with traj[j * GYRO_EVERY - 1].
    w = traj["w"][:, GYRO_EVERY - 1 : GYRO_EVERY * GYRO_N : GYRO_EVERY]
    return np.concatenate(
        [
            along * batch.tag_wa,
            across * batch.tag_wc,
            dyaw * batch.tag_wy,
            (w - batch.gyro) * batch.gyro_w,
        ],
        axis=1,
    )


@dataclass
class _Profiled:
    r: np.ndarray  # (S, R + NUISANCE): data rows, then the start-state prior rows
    s0: np.ndarray  # refined start states
    jn: np.ndarray  # (S, R, NUISANCE) data Jacobian in the start state
    a_inv: np.ndarray  # (S, NUISANCE, NUISANCE) inverse of jn^T jn + prior


def _profile(theta: np.ndarray, batch: Batch, structure: ModelStructure) -> _Profiled:
    """One Gauss-Newton step on every sub-window's start state, with the tag prior."""
    s0 = batch.state0
    r0 = _raw_residuals(theta, batch, s0, structure)
    jn = np.zeros(r0.shape + (NUISANCE,))
    for k in range(NUISANCE):
        s1 = s0.copy()
        s1[:, k] += NUISANCE_STEP[k]
        jn[..., k] = (_raw_residuals(theta, batch, s1, structure) - r0) / NUISANCE_STEP[k]
    a = np.einsum("srk,srl->skl", jn, jn) + np.diag(1.0 / NUISANCE_SD**2)
    a_inv = np.linalg.inv(a)
    delta = -np.einsum("skl,sl->sk", a_inv, np.einsum("srk,sr->sk", jn, r0))
    s1 = s0.copy()
    s1[:, :NUISANCE] += delta
    r1 = _raw_residuals(theta, batch, s1, structure)
    r = np.concatenate([r1, delta / NUISANCE_SD], axis=1)
    return _Profiled(r=r, s0=s1, jn=jn, a_inv=a_inv)


def residuals(
    theta: np.ndarray, batch: Batch, structure: ModelStructure = FULL_MODEL
) -> np.ndarray:
    """Normalized residuals with the start states profiled out.

    (S, R + NUISANCE): ``3 * MAX_TAGS`` tag rows (along, across, heading), ``GYRO_N`` gyro rows,
    then the start-state prior rows.
    """
    return _profile(theta, batch, structure).r


def jacobian(
    theta: np.ndarray,
    batch: Batch,
    free: Sequence[int] | None = None,
    structure: ModelStructure = FULL_MODEL,
) -> tuple[np.ndarray, np.ndarray]:
    """(r, J) with J shaped (S, R + NUISANCE, NP); columns outside ``free`` are zero.

    Forward differences at the refined start states, stepped away from a bound, then projected
    so the start states absorb what they can: data rows become ``J - jn A^-1 jn^T J`` and the
    prior rows ``-A^-1 jn^T J / sd``. ``J^T J`` of the result is the Schur complement, the
    information left once the start states are marginalized.
    """
    prof = _profile(theta, batch, structure)
    base = _raw_residuals(theta, batch, prof.s0, structure)
    jac = np.zeros(base.shape + (NP,))
    for i in range(NP) if free is None else free:
        h = STEPS[i] if theta[i] + STEPS[i] <= HI[i] else -STEPS[i]
        t2 = theta.copy()
        t2[i] += h
        jac[..., i] = (_raw_residuals(t2, batch, prof.s0, structure) - base) / h
    m = np.einsum("skl,srl,srp->skp", prof.a_inv, prof.jn, jac)
    data = jac - np.einsum("srk,skp->srp", prof.jn, m)
    prior = -m / NUISANCE_SD[None, :, None]
    return prof.r, np.concatenate([data, prior], axis=1)


def data_mask(batch: Batch) -> np.ndarray:
    """Which residual rows carry a measurement, (S, R): padding and the prior rows do not."""
    rows = np.concatenate([batch.tag_wa, batch.tag_wc, batch.tag_wy, batch.gyro_w], axis=1)
    out: np.ndarray = np.concatenate([rows > 0, np.zeros((rows.shape[0], NUISANCE), bool)], axis=1)
    return out


def information(jac: np.ndarray) -> np.ndarray:
    flat = jac.reshape(-1, NP)
    out: np.ndarray = flat.T @ flat
    return out


def covariance(fisher: np.ndarray, scale: float = 1.0) -> np.ndarray:
    a = fisher + np.diag(1.0 / PRIOR_SD**2)
    out: np.ndarray = max(scale, 1.0) * np.linalg.inv(a)
    return out


def correlation(cov: np.ndarray) -> np.ndarray:
    d = np.sqrt(np.diag(cov))
    out: np.ndarray = cov / np.outer(d, d)
    return out


# ---------------------------------------------------------------------------
# Fit
# ---------------------------------------------------------------------------


@dataclass
class FitResult:
    theta: np.ndarray
    fisher: np.ndarray  # J^T J of the data rows at theta
    scale: float  # residual variance per data row, the covariance inflation
    window_max: dict[int, float]  # owner -> largest |tag residual| in that window
    rows: int


def fit(
    theta0: np.ndarray,
    batch: Batch,
    free_names: Sequence[str] = NAMES,
    structure: ModelStructure = FULL_MODEL,
    max_nfev: int = 6,
) -> FitResult:
    """Warm-started soft-L1 least squares with the prior as extra rows."""
    free = [NAMES.index(n) for n in free_names]
    x0 = np.clip(theta0[free], LO[free] + 1e-9, HI[free] - 1e-9)

    def full(x: np.ndarray) -> np.ndarray:
        t: np.ndarray = theta0.copy()
        t[free] = x
        return t

    def fun(x: np.ndarray) -> np.ndarray:
        t = full(x)
        r = residuals(t, batch, structure).ravel()
        prior = (t[free] - PRIOR_VECTOR[free]) / PRIOR_SD[free]
        return np.concatenate([r, prior])

    def jac(x: np.ndarray) -> np.ndarray:
        t = full(x)
        _r, j = jacobian(t, batch, free, structure)
        rows = j.reshape(-1, NP)[:, free]
        return np.vstack([rows, np.diag(1.0 / PRIOR_SD[free])])

    if batch.rows:
        result = least_squares(
            fun,
            x0,
            jac=jac,
            bounds=(LO[free], HI[free]),
            loss="soft_l1",
            f_scale=3.0,
            x_scale="jac",
            max_nfev=max_nfev,
        )
        theta = full(result.x)
    else:
        theta = theta0.copy()
    r, j = jacobian(theta, batch, structure=structure) if batch.rows else (None, None)
    if r is None or j is None:
        return FitResult(theta, np.zeros((NP, NP)), 1.0, {}, 0)
    active = data_mask(batch)
    n = int(active.sum())
    scale = float(np.sum(r[active] ** 2) / max(n, 1))
    tag_cols = 3 * MAX_TAGS
    worst: dict[int, float] = {}
    for row, owner in enumerate(batch.owner):
        m = float(np.max(np.abs(r[row, :tag_cols]))) if tag_cols else 0.0
        worst[int(owner)] = max(worst.get(int(owner), 0.0), m)
    return FitResult(theta, information(j), scale, worst, n)


# ---------------------------------------------------------------------------
# Closed-loop prediction: holdout and ghosts
# ---------------------------------------------------------------------------


def _loop_seed(subs: Sequence[SubWindow]) -> LoopState:
    def col(key: str, default: float = 0.0) -> np.ndarray:
        return np.array([float(s.loop_seed.get(key, default)) for s in subs])

    heading0 = np.mod(-np.degrees([s.state0[2] for s in subs]), 360.0)
    return LoopState.from_telemetry(
        active=np.array([bool(s.auto_steer[0]) for s in subs]),
        heading0_deg=heading0,
        recorded_heading_deg=col("orientation_x"),
        pid_setpoint=col("pid_setpoint"),
        pid_output=col("pid_output"),
        yaw_rate_cmd=col("yaw_rate_cmd"),
        yaw_rate=col("yaw_rate"),
        b_percent=np.array([s.b_percent[0] for s in subs]),
        a_percent=np.array([s.a_percent[0] for s in subs]),
    )


def predict_closed_loop(
    theta: np.ndarray, subs: Sequence[SubWindow], structure: ModelStructure = FULL_MODEL
) -> dict[str, np.ndarray]:
    """Each sub-window's 0.5 s from its sticks, through the firmware mixer."""
    s0 = np.stack([s.state0 for s in subs])
    hist = (
        np.stack([s.cmd_lin[:HIST_N] for s in subs]),
        np.stack([s.cmd_ang[:HIST_N] for s in subs]),
    )
    return closed_loop_rollout(
        np.stack([s.a_percent for s in subs]),
        np.stack([s.b_percent for s in subs]),
        np.stack([s.auto_steer for s in subs]),
        params_of(theta),
        PlantState(s0[:, 0], s0[:, 1], s0[:, 2], s0[:, 3], s0[:, 4]),
        loop0=_loop_seed(subs),
        history=hist,
        structure=structure,
        substep_s=FIT_SUBSTEP_S,
    )


def holdout_error(
    theta: np.ndarray, subs: Sequence[SubWindow], structure: ModelStructure = FULL_MODEL
) -> float:
    """RMS closed-loop position error at the 0.5 s horizon, metres."""
    scored = [s for s in subs if s.end_xy is not None]
    if not scored:
        return float("nan")
    out = predict_closed_loop(theta, scored, structure)
    end = np.stack([s.end_xy for s in scored if s.end_xy is not None])
    err = np.hypot(out["x"][:, -1] - end[:, 0], out["y"][:, -1] - end[:, 1])
    return float(np.sqrt(np.mean(err**2)))


# ---------------------------------------------------------------------------
# Suggestions
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Motion:
    id: str
    name: str
    how: str
    throttle: np.ndarray  # stick fractions on the DT grid, forward positive
    steer: np.ndarray  # left positive


def _build(segments: Sequence[tuple[Any, ...]]) -> tuple[np.ndarray, np.ndarray]:
    thr: list[float] = []
    st: list[float] = []
    for seg in segments:
        if seg[0] == "ramp":
            _tag, dur, l0, a0, l1, a1 = seg
            n = int(round(dur / DT))
            f = np.linspace(0.0, 1.0, n)
            thr += list(l0 + (l1 - l0) * f)
            st += list(a0 + (a1 - a0) * f)
        else:
            dur, lin, ang = seg
            n = int(round(dur / DT))
            thr += [lin] * n
            st += [ang] * n
    return np.array(thr), np.array(st)


def _pct(a: float) -> str:
    return f"{round(abs(a) * 100)}%"


def catalog() -> list[Motion]:
    out: list[Motion] = []

    def add(mid: str, name: str, how: str, segs: Sequence[tuple[Any, ...]]) -> None:
        thr, st = _build(segs)
        out.append(Motion(mid, name, how, thr, st))

    for amp in (0.25, 0.5, 0.75, 1.0):
        for hold in (0.5, 1.0):
            add(
                f"fwd{amp}-{hold}",
                f"Forward step {_pct(amp)}, {hold:g} s",
                f"Snap throttle to {_pct(amp)}, hold {hold:g} s, release and let it coast.",
                [(0.3, 0, 0), (hold, amp, 0), (1.2, 0, 0)],
            )
            add(
                f"rev{amp}-{hold}",
                f"Reverse step {_pct(amp)}, {hold:g} s",
                f"Snap throttle back to {_pct(amp)}, hold {hold:g} s, release and let it coast.",
                [(0.3, 0, 0), (hold, -amp, 0), (1.2, 0, 0)],
            )
    for amp in (0.25, 0.5, 1.0):
        add(
            f"spl{amp}",
            f"Spin left {_pct(amp)}",
            f"Steer left {_pct(amp)} with no throttle, hold 0.8 s, release.",
            [(0.3, 0, 0), (0.8, 0, amp), (1.2, 0, 0)],
        )
        add(
            f"spr{amp}",
            f"Spin right {_pct(amp)}",
            f"Steer right {_pct(amp)} with no throttle, hold 0.8 s, release.",
            [(0.3, 0, 0), (0.8, 0, -amp), (1.2, 0, 0)],
        )
    add(
        "creepf",
        "Throttle creep, forward",
        "Ease throttle from 0 to 20% over 3 s, until it starts rolling.",
        [(0.3, 0, 0), ("ramp", 3.0, 0, 0, 0.2, 0), (1.0, 0, 0)],
    )
    add(
        "creepr",
        "Throttle creep, reverse",
        "Ease throttle from 0 to 20% reverse over 3 s.",
        [(0.3, 0, 0), ("ramp", 3.0, 0, 0, -0.2, 0), (1.0, 0, 0)],
    )
    add(
        "creepl",
        "Steer creep, left",
        "Ease steering from 0 to 20% left over 3 s, until it starts turning.",
        [(0.3, 0, 0), ("ramp", 3.0, 0, 0, 0, 0.2), (1.0, 0, 0)],
    )
    add(
        "creeprr",
        "Steer creep, right",
        "Ease steering from 0 to 20% right over 3 s.",
        [(0.3, 0, 0), ("ramp", 3.0, 0, 0, 0, -0.2), (1.0, 0, 0)],
    )
    for name, lin, ang in (
        ("forward-left", 0.4, 0.5),
        ("forward-right", 0.4, -0.5),
        ("reverse-left", -0.4, 0.5),
        ("reverse-right", -0.4, -0.5),
    ):
        add(
            f"arc-{name}",
            f"Arc {name}",
            f"Hold {_pct(lin)} {'throttle' if lin > 0 else 'reverse'} with 50% "
            f"{'left' if ang > 0 else 'right'} steer for 1 s.",
            [(0.3, 0, 0), (1.0, lin, ang), (1.2, 0, 0)],
        )
    add(
        "taps",
        "Throttle taps",
        "Four quick 60% taps, 0.15 s on and 0.6 s off.",
        [(0.3, 0, 0)] + [(0.15, 0.6, 0), (0.6, 0, 0)] * 4 + [(0.6, 0, 0)],
    )
    add(
        "rtaps",
        "Reverse taps",
        "Four quick 60% reverse taps, 0.15 s on and 0.6 s off.",
        [(0.3, 0, 0)] + [(0.15, -0.6, 0), (0.6, 0, 0)] * 4 + [(0.6, 0, 0)],
    )
    return out


CATALOG = catalog()


@dataclass
class Candidate:
    motion: Motion
    gains: np.ndarray  # (NP,) ring fill each parameter would gain
    score: float  # summed gain
    per_sec: float
    travel: float  # metres from the start at the farthest point
    fits: bool


def expected_subwindows(
    theta: np.ndarray, motion: Motion, loop_on: bool, tag_sigma: tuple[float, float, float]
) -> tuple[list[SubWindow], float]:
    """Simulate a motion from rest and cut it into the sub-windows a session would record.

    Returns the sub-windows (measurements equal to the prediction, so their residual is zero
    and only the Jacobian matters) and how far the robot travels.
    """
    n = len(motion.throttle)
    a = -motion.throttle * 100.0
    b = -motion.steer * 100.0
    auto = np.full(n, loop_on)
    zero = PlantState(np.zeros(1), np.zeros(1), np.zeros(1), np.zeros(1), np.zeros(1))
    out = closed_loop_rollout(
        a[None], b[None], auto[None], params_of(theta), zero, substep_s=FIT_SUBSTEP_S
    )
    lin, ang = motor_to_plant(out["left"][0], out["right"][0])
    x, y, th, v, w = (out[k][0] for k in ("x", "y", "theta", "v", "w"))
    travel = float(np.max(np.hypot(x, y)))
    subs: list[SubWindow] = []
    pad_lin = np.concatenate([np.zeros(HIST_N), lin])
    pad_ang = np.concatenate([np.zeros(HIST_N), ang])
    # Fixes land on grid samples, so each one's time is exactly the sample it copies.
    tag_tau = np.round(np.arange(1, MAX_TAGS + 1) * (SUB_S / MAX_TAGS) / DT) * DT
    for start in range(0, n - SUB_N + 1, SUB_N):
        k = start - 1
        state0 = np.zeros(5) if k < 0 else np.array([x[k], y[k], th[k], v[k], w[k]])
        idx = np.clip(start + np.round(tag_tau / DT).astype(int) - 1, 0, n - 1)
        gidx = start + GYRO_EVERY - 1 + GYRO_EVERY * np.arange(GYRO_N)
        subs.append(
            SubWindow(
                state0=state0,
                cmd_lin=pad_lin[start : start + HIST_N + SUB_N],
                cmd_ang=pad_ang[start : start + HIST_N + SUB_N],
                tag_tau=tag_tau,
                tag_x=x[idx],
                tag_y=y[idx],
                tag_yaw=th[idx],
                tag_sa=np.full(MAX_TAGS, tag_sigma[0]),
                tag_sc=np.full(MAX_TAGS, tag_sigma[1]),
                tag_sy=np.full(MAX_TAGS, tag_sigma[2]),
                gyro=w[gidx],
                gyro_ok=np.ones(len(gidx), dtype=bool),
                a_percent=a[start : start + SUB_N],
                b_percent=b[start : start + SUB_N],
                auto_steer=auto[start : start + SUB_N],
            )
        )
    return subs, travel


def rank_candidates(
    theta: np.ndarray,
    fisher: np.ndarray,
    scale: float,
    loop_on: bool,
    floor_run_m: float,
    view_azimuth: float,
    tag_sigma: tuple[float, float, float] = (0.006, 0.002, 0.01),
) -> list[Candidate]:
    """Each catalog motion, scored by the ring fill it would add per second of driving."""
    base_fills = fills_of(np.sqrt(np.diag(covariance(fisher, scale))))
    all_subs: list[SubWindow] = []
    owners: list[int] = []
    travels: list[float] = []
    for i, motion in enumerate(CATALOG):
        subs, travel = expected_subwindows(theta, motion, loop_on, tag_sigma)
        all_subs += subs
        owners += [i] * len(subs)
        travels.append(travel)
    batch = stack(all_subs, owners, view_azimuth)
    _r, jac = jacobian(theta, batch)
    out: list[Candidate] = []
    for i, motion in enumerate(CATALOG):
        rows = batch.owner == i
        f = information(jac[rows])
        fills = fills_of(np.sqrt(np.diag(covariance(fisher + f, scale))))
        gains = np.maximum(fills - base_fills, 0.0)
        score = float(np.sum(gains))
        out.append(
            Candidate(
                motion=motion,
                gains=gains,
                score=score,
                per_sec=score / (len(motion.throttle) * DT),
                travel=travels[i],
                fits=travels[i] <= floor_run_m,
            )
        )
    out.sort(key=lambda c: c.per_sec, reverse=True)
    return out


def floor_limited(fills: np.ndarray, candidates: Sequence[Candidate]) -> np.ndarray:
    """A parameter is floor-limited when only motions that do not fit would still fill it."""
    inside = np.zeros(NP)
    outside = np.zeros(NP)
    for c in candidates:
        if c.fits:
            inside = np.maximum(inside, c.gains)
        else:
            outside = np.maximum(outside, c.gains)
    out: np.ndarray = (fills < 0.999) & (inside < 0.01) & (outside > 0.03)
    return out
