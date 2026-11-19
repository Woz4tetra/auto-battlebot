"""The fit behind the sysid monitor: window residuals, Fisher information, refits, suggestions."""

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

DT_S = 0.01
WINDOW_S = 1.5
SUB_S = 0.5
SUBS = int(round(WINDOW_S / SUB_S))
SUB_N = int(round(SUB_S / DT_S))
HIST_S = 0.25
HIST_N = int(round(HIST_S / DT_S))
MAX_TAG_FIXES_PER_SUB = 16
GYRO_STRIDE_SAMPLES = 2
GYRO_N = (SUB_N - 1) // GYRO_STRIDE_SAMPLES
FIT_SUBSTEP_S = 0.005
STATE_DIM = 5
TAG_ROWS = 3 * MAX_TAG_FIXES_PER_SUB

POS_FLOOR_M = 0.01
YAW_FLOOR_RAD = 0.03
SIGMA_GYRO_RADPS = 0.3


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

RUNG_FREE: dict[str, list[str]] = {
    s.name: s.free_names() + (["delay_s"] if s.use_delay else []) for s in MODEL_LADDER
}


def params_of(theta: np.ndarray) -> PlantParams:
    return PRIOR.with_vector(NAMES, theta)


def fills_of(std: np.ndarray) -> np.ndarray:
    out: np.ndarray = np.minimum(1.0, TARGET_SD / np.maximum(std, 1e-12))
    return out


def _require_shape(owner: str, name: str, array: np.ndarray, shape: tuple[int, ...]) -> None:
    if array.shape != shape:
        raise ValueError(f"{owner}.{name} has shape {array.shape}, expected {shape}")


@dataclass
class SubWindow:
    """One 0.5 s open-loop stretch: its start state, inputs and measurements."""

    state0: np.ndarray
    cmd_lin: np.ndarray
    cmd_ang: np.ndarray
    tag_tau_s: np.ndarray
    tag_x: np.ndarray
    tag_y: np.ndarray
    tag_yaw: np.ndarray
    tag_sigma_along_m: np.ndarray
    tag_sigma_across_m: np.ndarray
    tag_sigma_yaw_rad: np.ndarray
    gyro_ccw_radps: np.ndarray
    gyro_ok: np.ndarray
    a_percent: np.ndarray
    b_percent: np.ndarray
    auto_steer: np.ndarray
    loop_seed: dict[str, float] = field(default_factory=dict)
    end_xy: np.ndarray | None = None

    def __post_init__(self) -> None:
        _require_shape("SubWindow", "state0", self.state0, (STATE_DIM,))
        for name in ("cmd_lin", "cmd_ang"):
            _require_shape("SubWindow", name, getattr(self, name), (HIST_N + SUB_N,))
        for name in ("a_percent", "b_percent", "auto_steer"):
            _require_shape("SubWindow", name, getattr(self, name), (SUB_N,))
        for name in ("gyro_ccw_radps", "gyro_ok"):
            _require_shape("SubWindow", name, getattr(self, name), (GYRO_N,))
        tag_shape = self.tag_tau_s.shape
        for name in (
            "tag_x",
            "tag_y",
            "tag_yaw",
            "tag_sigma_along_m",
            "tag_sigma_across_m",
            "tag_sigma_yaw_rad",
        ):
            _require_shape("SubWindow", name, getattr(self, name), tag_shape)
        if self.gyro_ok.dtype != np.bool_ or self.auto_steer.dtype != np.bool_:
            raise ValueError("SubWindow.gyro_ok and auto_steer must be bool arrays")


@dataclass
class Batch:
    """Sub-windows stacked for one batched simulation. S rows."""

    state0: np.ndarray
    cmd_lin: np.ndarray
    cmd_ang: np.ndarray
    tag_grid_index: np.ndarray
    tag_x: np.ndarray
    tag_y: np.ndarray
    tag_yaw: np.ndarray
    tag_inv_sigma_along: np.ndarray
    tag_inv_sigma_across: np.ndarray
    tag_inv_sigma_yaw: np.ndarray
    gyro: np.ndarray
    gyro_inv_sigma: np.ndarray
    owner_window_index: np.ndarray
    view_c: float
    view_s: float

    def __post_init__(self) -> None:
        s = self.state0.shape[0]
        _require_shape("Batch", "state0", self.state0, (s, STATE_DIM))
        for name in ("cmd_lin", "cmd_ang"):
            _require_shape("Batch", name, getattr(self, name), (s, HIST_N + SUB_N))

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
        tag_grid_index=np.zeros((s, MAX_TAG_FIXES_PER_SUB)),
        tag_x=np.zeros((s, MAX_TAG_FIXES_PER_SUB)),
        tag_y=np.zeros((s, MAX_TAG_FIXES_PER_SUB)),
        tag_yaw=np.zeros((s, MAX_TAG_FIXES_PER_SUB)),
        tag_inv_sigma_along=np.zeros((s, MAX_TAG_FIXES_PER_SUB)),
        tag_inv_sigma_across=np.zeros((s, MAX_TAG_FIXES_PER_SUB)),
        tag_inv_sigma_yaw=np.zeros((s, MAX_TAG_FIXES_PER_SUB)),
        gyro=np.zeros((s, g)),
        gyro_inv_sigma=np.zeros((s, g)),
        owner_window_index=np.asarray(owners, dtype=np.int64),
        view_c=float(np.cos(view_azimuth)),
        view_s=float(np.sin(view_azimuth)),
    )
    for i, sub in enumerate(subs):
        out.state0[i] = sub.state0
        out.cmd_lin[i] = sub.cmd_lin
        out.cmd_ang[i] = sub.cmd_ang
        m = min(len(sub.tag_tau_s), MAX_TAG_FIXES_PER_SUB)
        out.tag_grid_index[i, :m] = sub.tag_tau_s[:m] / DT_S
        out.tag_x[i, :m] = sub.tag_x[:m]
        out.tag_y[i, :m] = sub.tag_y[:m]
        out.tag_yaw[i, :m] = sub.tag_yaw[:m]
        out.tag_inv_sigma_along[i, :m] = 1.0 / np.hypot(sub.tag_sigma_along_m[:m], POS_FLOOR_M)
        out.tag_inv_sigma_across[i, :m] = 1.0 / np.hypot(sub.tag_sigma_across_m[:m], POS_FLOOR_M)
        out.tag_inv_sigma_yaw[i, :m] = 1.0 / np.hypot(sub.tag_sigma_yaw_rad[:m], YAW_FLOOR_RAD)
        out.gyro[i] = np.nan_to_num(sub.gyro_ccw_radps)
        out.gyro_inv_sigma[i] = np.where(sub.gyro_ok, 1.0 / SIGMA_GYRO_RADPS, 0.0)
    return out


def _delayed(cmd: np.ndarray, delay_s: float) -> np.ndarray:
    """Delay each (S, HIST_N + SUB_N) command row by ``delay_s`` into (S, SUB_N)."""
    shift = delay_s / DT_S
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


NUISANCE_SD = np.array([0.005, 0.005, 0.02, 0.1])
NUISANCE_STEP = np.array([1e-3, 1e-3, 1e-3, 1e-2])
NUISANCE = len(NUISANCE_SD)


def _yaw_rate_at_gyro_samples(w_traj: np.ndarray) -> np.ndarray:
    first = GYRO_STRIDE_SAMPLES - 1
    out: np.ndarray = w_traj[:, first : GYRO_STRIDE_SAMPLES * GYRO_N : GYRO_STRIDE_SAMPLES]
    return out


def _raw_residuals(
    theta: np.ndarray, batch: Batch, s0: np.ndarray, structure: ModelStructure
) -> np.ndarray:
    p = structure.apply(params_of(theta))
    u_lin = _delayed(batch.cmd_lin, p.delay_s)
    u_ang = _delayed(batch.cmd_ang, p.delay_s)
    state = PlantState(s0[:, 0], s0[:, 1], s0[:, 2], s0[:, 3], s0[:, 4])
    _final, traj = simulate(u_lin, u_ang, DT_S, p, state, structure, substep_s=FIT_SUBSTEP_S)
    x = np.concatenate([s0[:, :1], traj["x"]], axis=1)
    y = np.concatenate([s0[:, 1:2], traj["y"]], axis=1)
    th = np.concatenate([s0[:, 2:3], traj["theta"]], axis=1)
    dx = _interp(x, batch.tag_grid_index) - batch.tag_x
    dy = _interp(y, batch.tag_grid_index) - batch.tag_y
    along = batch.view_c * dx + batch.view_s * dy
    across = -batch.view_s * dx + batch.view_c * dy
    dyaw = wrap_angle(_interp(th, batch.tag_grid_index) - batch.tag_yaw)
    w = _yaw_rate_at_gyro_samples(traj["w"])
    return np.concatenate(
        [
            along * batch.tag_inv_sigma_along,
            across * batch.tag_inv_sigma_across,
            dyaw * batch.tag_inv_sigma_yaw,
            (w - batch.gyro) * batch.gyro_inv_sigma,
        ],
        axis=1,
    )


@dataclass
class _Profiled:
    r: np.ndarray
    s0: np.ndarray
    jn: np.ndarray
    a_inv: np.ndarray

    def __post_init__(self) -> None:
        s, cols = self.r.shape
        data_rows = cols - NUISANCE
        _require_shape("_Profiled", "s0", self.s0, (s, STATE_DIM))
        _require_shape("_Profiled", "jn", self.jn, (s, data_rows, NUISANCE))
        _require_shape("_Profiled", "a_inv", self.a_inv, (s, NUISANCE, NUISANCE))


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
    """Normalized residuals with the start states profiled out."""
    return _profile(theta, batch, structure).r


def jacobian(
    theta: np.ndarray,
    batch: Batch,
    free: Sequence[int] | None = None,
    structure: ModelStructure = FULL_MODEL,
) -> tuple[np.ndarray, np.ndarray]:
    """(r, J) with J shaped (S, R + NUISANCE, NP); columns outside ``free`` are zero."""
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
    rows = np.concatenate(
        [
            batch.tag_inv_sigma_along,
            batch.tag_inv_sigma_across,
            batch.tag_inv_sigma_yaw,
            batch.gyro_inv_sigma,
        ],
        axis=1,
    )
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


@dataclass
class FitResult:
    theta: np.ndarray
    fisher: np.ndarray
    residual_variance: float
    window_max_tag_residual: dict[int, float]
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
    tag_cols = TAG_ROWS
    worst: dict[int, float] = {}
    for row, owner_window_index in enumerate(batch.owner_window_index):
        m = float(np.max(np.abs(r[row, :tag_cols]))) if tag_cols else 0.0
        worst[int(owner_window_index)] = max(worst.get(int(owner_window_index), 0.0), m)
    return FitResult(theta, information(j), scale, worst, n)


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


@dataclass(frozen=True)
class Motion:
    id: str
    name: str
    how: str
    throttle_fwd_pos: np.ndarray
    steer_left_pos: np.ndarray


def _build(segments: Sequence[tuple[Any, ...]]) -> tuple[np.ndarray, np.ndarray]:
    thr: list[float] = []
    st: list[float] = []
    for seg in segments:
        if seg[0] == "ramp":
            _tag, dur, l0, a0, l1, a1 = seg
            n = int(round(dur / DT_S))
            f = np.linspace(0.0, 1.0, n)
            thr += list(l0 + (l1 - l0) * f)
            st += list(a0 + (a1 - a0) * f)
        else:
            dur, lin, ang = seg
            n = int(round(dur / DT_S))
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
    ring_fill_gain: np.ndarray
    ring_fill_total: float
    per_sec: float
    travel_m: float
    fits: bool


def _grid_aligned_fix_times_s() -> np.ndarray:
    spacing_s = SUB_S / MAX_TAG_FIXES_PER_SUB
    out: np.ndarray = np.round(np.arange(1, MAX_TAG_FIXES_PER_SUB + 1) * spacing_s / DT_S) * DT_S
    return out


def expected_subwindows(
    theta: np.ndarray, motion: Motion, loop_on: bool, tag_sigma: tuple[float, float, float]
) -> tuple[list[SubWindow], float]:
    """Simulate a motion from rest and cut it into the sub-windows a session would record."""
    n = len(motion.throttle_fwd_pos)
    a = -motion.throttle_fwd_pos * 100.0
    b = -motion.steer_left_pos * 100.0
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
    tag_tau_s = _grid_aligned_fix_times_s()
    for start in range(0, n - SUB_N + 1, SUB_N):
        k = start - 1
        state0 = np.zeros(5) if k < 0 else np.array([x[k], y[k], th[k], v[k], w[k]])
        idx = np.clip(start + np.round(tag_tau_s / DT_S).astype(int) - 1, 0, n - 1)
        gidx = start + GYRO_STRIDE_SAMPLES - 1 + GYRO_STRIDE_SAMPLES * np.arange(GYRO_N)
        subs.append(
            SubWindow(
                state0=state0,
                cmd_lin=pad_lin[start : start + HIST_N + SUB_N],
                cmd_ang=pad_ang[start : start + HIST_N + SUB_N],
                tag_tau_s=tag_tau_s,
                tag_x=x[idx],
                tag_y=y[idx],
                tag_yaw=th[idx],
                tag_sigma_along_m=np.full(MAX_TAG_FIXES_PER_SUB, tag_sigma[0]),
                tag_sigma_across_m=np.full(MAX_TAG_FIXES_PER_SUB, tag_sigma[1]),
                tag_sigma_yaw_rad=np.full(MAX_TAG_FIXES_PER_SUB, tag_sigma[2]),
                gyro_ccw_radps=w[gidx],
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
        rows = batch.owner_window_index == i
        f = information(jac[rows])
        fills = fills_of(np.sqrt(np.diag(covariance(fisher + f, scale))))
        gains = np.maximum(fills - base_fills, 0.0)
        score = float(np.sum(gains))
        out.append(
            Candidate(
                motion=motion,
                ring_fill_gain=gains,
                ring_fill_total=score,
                per_sec=score / (len(motion.throttle_fwd_pos) * DT_S),
                travel_m=travels[i],
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
            inside = np.maximum(inside, c.ring_fill_gain)
        else:
            outside = np.maximum(outside, c.ring_fill_gain)
    out: np.ndarray = (fills < 0.999) & (inside < 0.01) & (outside > 0.03)
    return out
