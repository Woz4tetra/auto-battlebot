"""Grey-box drivetrain plant model: command in, pose out."""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from auto_battlebot.compat import tomllib
from auto_battlebot.config_profile import expand_section_files

SUBSTEP_S = 0.002

STRAIGHT_W = 1e-6


def toml_float(value: float) -> str:
    """Format a float so TOML reads it back as a float."""
    text = f"{value:.6g}"
    if "." not in text and "e" not in text and "n" not in text:
        text += ".0"
    return text


@dataclass(frozen=True)
class PlantParams:
    """The parameter vector, with stage 2 values as priors."""

    dz_lin_fwd: float = 0.04
    dz_lin_rev: float = 0.04
    dz_ang_l: float = 0.04
    dz_ang_r: float = 0.04
    k_fwd: float = 5.60
    k_rev: float = 4.84
    k_ang: float = 61.5
    tau_lin_a: float = 0.058
    tau_lin_d: float = 0.078
    tau_ang_a: float = 0.066
    tau_ang_d: float = 0.078
    delay_s: float = 0.059
    c_sb: float = 0.0
    c_ad: float = 0.0
    c_drift: float = 0.0
    c_drift_bias: float = 0.0

    def replace(self, **kwargs: float) -> PlantParams:
        return replace(self, **kwargs)

    def to_dict(self) -> dict[str, float]:
        return {f.name: float(getattr(self, f.name)) for f in fields(self)}

    def to_vector(self, names: Sequence[str]) -> np.ndarray:
        return np.array([getattr(self, n) for n in names], dtype=float)

    def with_vector(self, names: Sequence[str], values: Iterable[float]) -> PlantParams:
        return replace(self, **{n: float(v) for n, v in zip(names, values)})

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> PlantParams:
        known = {f.name for f in fields(cls)}
        return cls(**{k: float(v) for k, v in data.items() if k in known})

    @classmethod
    def from_toml(cls, path: Path | str, table: str = "plant") -> PlantParams:
        with open(path, "rb") as handle:
            data = tomllib.load(handle)
        expand_section_files(data, Path(path))
        return cls.from_dict(data.get(table, data))

    def to_toml(self, table: str = "plant") -> str:
        lines = [f"[{table}]"]
        for name, value in self.to_dict().items():
            lines.append(f"{name} = {toml_float(value)}")
        return "\n".join(lines) + "\n"


PARAM_BOUNDS: dict[str, tuple[float, float]] = {
    "dz_lin_fwd": (0.0, 0.25),
    "dz_lin_rev": (0.0, 0.25),
    "dz_ang_l": (0.0, 0.25),
    "dz_ang_r": (0.0, 0.25),
    "k_fwd": (0.5, 12.0),
    "k_rev": (0.5, 12.0),
    "k_ang": (1.0, 150.0),
    "tau_lin_a": (0.005, 0.6),
    "tau_lin_d": (0.005, 0.6),
    "tau_ang_a": (0.005, 0.6),
    "tau_ang_d": (0.005, 0.6),
    "delay_s": (0.0, 0.2),
    "c_sb": (-0.5, 1.5),
    "c_ad": (-0.5, 1.5),
    "c_drift": (-8.0, 8.0),
    "c_drift_bias": (-8.0, 8.0),
}


@dataclass(frozen=True)
class ModelStructure:
    """Which terms are in the model. One rung of the ladder each."""

    name: str = "M4"
    use_delay: bool = True
    use_lag: bool = True
    asymmetric_tau: bool = True
    asymmetric_gain: bool = True
    coupling: bool = True
    drift_prop: bool = False
    drift_offset: bool = False
    pinned: tuple[str, ...] = ()

    def apply(self, p: PlantParams) -> PlantParams:
        """Collapse a full parameter set onto this structure, so disabled terms cannot leak in."""
        out = p
        if not self.use_delay:
            out = out.replace(delay_s=0.0)
        if not self.use_lag:
            out = out.replace(tau_lin_a=0.0, tau_lin_d=0.0, tau_ang_a=0.0, tau_ang_d=0.0)
        elif not self.asymmetric_tau:
            out = out.replace(tau_lin_d=out.tau_lin_a, tau_ang_d=out.tau_ang_a)
        if not self.asymmetric_gain:
            out = out.replace(k_rev=out.k_fwd, dz_lin_rev=out.dz_lin_fwd, dz_ang_r=out.dz_ang_l)
        if not self.coupling:
            out = out.replace(c_sb=0.0, c_ad=0.0)
        if not self.drift_prop:
            out = out.replace(c_drift=0.0)
        if not self.drift_offset:
            out = out.replace(c_drift_bias=0.0)
        return out

    def gradient_free_names(self) -> list[str]:
        """Parameters this structure lets the fit move, excluding delay."""
        names = ["dz_lin_fwd", "dz_ang_l", "k_fwd", "k_ang"]
        if self.asymmetric_gain:
            names += ["dz_lin_rev", "dz_ang_r", "k_rev"]
        if self.use_lag:
            names += ["tau_lin_a", "tau_ang_a"]
            if self.asymmetric_tau:
                names += ["tau_lin_d", "tau_ang_d"]
        if self.coupling:
            names += ["c_sb", "c_ad"]
        if self.drift_prop:
            names += ["c_drift"]
        if self.drift_offset:
            names += ["c_drift_bias"]
        return [n for n in names if n not in self.pinned]


MODEL_LADDER: tuple[ModelStructure, ...] = (
    ModelStructure("M0", False, False, False, False, False),
    ModelStructure("M1", True, False, False, False, False),
    ModelStructure("M2", True, True, False, False, False),
    ModelStructure("M3", True, True, True, True, False),
    ModelStructure("M4", True, True, True, True, True),
    ModelStructure("M5", True, True, True, True, True, True, False),
    ModelStructure("M6", True, True, True, True, True, True, True),
)

FULL_MODEL = MODEL_LADDER[-1]


def effective_command(u: np.ndarray, dz_pos: float, dz_neg: float) -> np.ndarray:
    """Deadzone removal and rescale, per sign."""
    u = np.asarray(u, dtype=float)
    dz = np.where(u >= 0.0, dz_pos, dz_neg)
    dz = np.clip(dz, 0.0, 0.95)
    mag = np.maximum(np.abs(u) - dz, 0.0) / (1.0 - dz)
    out: np.ndarray = np.sign(u) * mag
    return out


def steady_state(u_lin: np.ndarray, u_ang: np.ndarray, p: PlantParams) -> tuple[Any, Any]:
    """Target body speed and yaw rate for a held command, coupling included."""
    lin_eff = effective_command(u_lin, p.dz_lin_fwd, p.dz_lin_rev)
    ang_eff = effective_command(u_ang, p.dz_ang_l, p.dz_ang_r)
    v_target = np.where(lin_eff >= 0.0, p.k_fwd, p.k_rev) * lin_eff
    w_target = p.k_ang * ang_eff
    if p.c_sb:
        v_target = v_target * np.clip(1.0 - p.c_sb * np.abs(ang_eff), 0.0, None)
    if p.c_ad:
        w_target = w_target * np.clip(1.0 - p.c_ad * np.abs(lin_eff), 0.0, None)
    if p.c_drift:
        w_target = w_target + p.c_drift * lin_eff
    if p.c_drift_bias:
        w_target = w_target + p.c_drift_bias * np.sign(lin_eff)
    return v_target, w_target


def _lag_alpha(dt: float, tau: Any) -> Any:
    """Exact first-order discretization over dt. tau = 0 means no lag (the M0/M1 rungs)."""
    tau = np.asarray(tau, dtype=float)
    return np.where(tau > 1e-9, np.exp(-dt / np.maximum(tau, 1e-9)), 0.0)


@dataclass
class PlantState:
    """Model state. Same five numbers as the EKF state vector, in the same order."""

    x: np.ndarray | float = 0.0
    y: np.ndarray | float = 0.0
    theta: np.ndarray | float = 0.0
    v: np.ndarray | float = 0.0
    w: np.ndarray | float = 0.0

    def as_array(self) -> np.ndarray:
        return np.stack(
            [
                np.atleast_1d(np.asarray(self.x, dtype=float)),
                np.atleast_1d(np.asarray(self.y, dtype=float)),
                np.atleast_1d(np.asarray(self.theta, dtype=float)),
                np.atleast_1d(np.asarray(self.v, dtype=float)),
                np.atleast_1d(np.asarray(self.w, dtype=float)),
            ]
        )

    def copy(self) -> PlantState:
        return PlantState(
            np.copy(self.x), np.copy(self.y), np.copy(self.theta), np.copy(self.v), np.copy(self.w)
        )


def integrate_step(
    state: PlantState,
    v_target: Any,
    w_target: Any,
    dt: float,
    p: PlantParams,
) -> PlantState:
    """Advance one substep: pose along the arc, then the first-order velocity update."""
    v = np.asarray(state.v, dtype=float)
    w = np.asarray(state.w, dtype=float)
    theta = np.asarray(state.theta, dtype=float)

    theta_next = theta + w * dt
    turning = np.abs(w) > STRAIGHT_W
    w_safe = np.where(turning, w, 1.0)
    radius = v / w_safe
    dx = np.where(turning, radius * (np.sin(theta_next) - np.sin(theta)), v * dt * np.cos(theta))
    dy = np.where(turning, -radius * (np.cos(theta_next) - np.cos(theta)), v * dt * np.sin(theta))

    tau_lin = np.where(np.abs(v_target) > np.abs(v), p.tau_lin_a, p.tau_lin_d)
    tau_ang = np.where(np.abs(w_target) > np.abs(w), p.tau_ang_a, p.tau_ang_d)
    a_lin = _lag_alpha(dt, tau_lin)
    a_ang = _lag_alpha(dt, tau_ang)

    return PlantState(
        x=np.asarray(state.x, dtype=float) + dx,
        y=np.asarray(state.y, dtype=float) + dy,
        theta=theta_next,
        v=a_lin * v + (1.0 - a_lin) * v_target,
        w=a_ang * w + (1.0 - a_ang) * w_target,
    )


def simulate(
    u_lin: np.ndarray,
    u_ang: np.ndarray,
    dt: float,
    p: PlantParams,
    state: PlantState | None = None,
    structure: ModelStructure = FULL_MODEL,
    substep_s: float = SUBSTEP_S,
) -> tuple[PlantState, dict[str, np.ndarray]]:
    """Run the plant over a command timeline."""
    p = structure.apply(p)
    u_lin = np.asarray(u_lin, dtype=float)
    u_ang = np.asarray(u_ang, dtype=float)
    n = u_lin.shape[-1]
    batch = u_lin.shape[:-1]

    if state is None:
        zeros = np.zeros(batch)
        state = PlantState(zeros.copy(), zeros.copy(), zeros.copy(), zeros.copy(), zeros.copy())
    else:
        state = state.copy()

    if not structure.use_lag:
        substeps = 1
        sub_dt = dt
    else:
        substeps = max(1, int(round(dt / substep_s)))
        sub_dt = dt / substeps

    traj = {
        key: np.zeros(batch + (n,)) for key in ("x", "y", "theta", "v", "w", "v_target", "w_target")
    }

    for k in range(n):
        v_target, w_target = steady_state(u_lin[..., k], u_ang[..., k], p)
        if not structure.use_lag:
            state = PlantState(state.x, state.y, state.theta, v_target, w_target)
        for _ in range(substeps):
            state = integrate_step(state, v_target, w_target, sub_dt, p)
        traj["x"][..., k] = state.x
        traj["y"][..., k] = state.y
        traj["theta"][..., k] = state.theta
        traj["v"][..., k] = state.v
        traj["w"][..., k] = state.w
        traj["v_target"][..., k] = v_target
        traj["w_target"][..., k] = w_target

    return state, traj


class Plant:
    """Stateful wrapper with a zero-order-hold command ring buffer, mirroring the C++ shape."""

    def __init__(
        self,
        params: PlantParams,
        structure: ModelStructure = FULL_MODEL,
        buffer_len: int = 256,
        substep_s: float = SUBSTEP_S,
    ) -> None:
        self.params = structure.apply(params)
        self.structure = structure
        self.substep_s = substep_s
        self._cmd_t = np.zeros(buffer_len)
        self._cmd_lin = np.zeros(buffer_len)
        self._cmd_ang = np.zeros(buffer_len)
        self._n = 0
        self.state = PlantState(0.0, 0.0, 0.0, 0.0, 0.0)
        self.time = 0.0

    def reset(self, state: PlantState | None = None, time: float = 0.0) -> None:
        self.state = state.copy() if state is not None else PlantState(0.0, 0.0, 0.0, 0.0, 0.0)
        self.time = time
        self._n = 0

    def push_command(self, t: float, u_lin: float, u_ang: float) -> None:
        """Record a command at its issue time. Commands are held until the next one arrives."""
        i = self._n % len(self._cmd_t)
        self._cmd_t[i] = t
        self._cmd_lin[i] = u_lin
        self._cmd_ang[i] = u_ang
        self._n += 1

    def command_at(self, t: float) -> tuple[float, float]:
        """The command in effect at time `t`, zero-order held. Zero before the first command."""
        if self._n == 0:
            return 0.0, 0.0
        count = min(self._n, len(self._cmd_t))
        start = self._n - count
        idx = [(start + k) % len(self._cmd_t) for k in range(count)]
        times = self._cmd_t[idx]
        j = int(np.searchsorted(times, t, side="right")) - 1
        if j < 0:
            return 0.0, 0.0
        return float(self._cmd_lin[idx[j]]), float(self._cmd_ang[idx[j]])

    def step(self, dt: float) -> PlantState:
        """Advance dt, reading the command from `now - delay_s`."""
        u_lin, u_ang = self.command_at(self.time - self.params.delay_s)
        v_target, w_target = steady_state(np.array(u_lin), np.array(u_ang), self.params)
        if not self.structure.use_lag:
            self.state = PlantState(
                self.state.x, self.state.y, self.state.theta, v_target, w_target
            )
            substeps, sub_dt = 1, dt
        else:
            substeps = max(1, int(round(dt / self.substep_s)))
            sub_dt = dt / substeps
        for _ in range(substeps):
            self.state = integrate_step(self.state, v_target, w_target, sub_dt, self.params)
        self.time += dt
        return self.state


def delayed_commands(
    t: np.ndarray,
    cmd_lin: np.ndarray,
    cmd_ang: np.ndarray,
    delay_s: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Shift a uniformly sampled command timeline back by `delay_s`."""
    if delay_s <= 0.0:
        return cmd_lin, cmd_ang
    shift = delay_s / dt
    idx = np.arange(len(t)) - shift
    idx = np.clip(idx, 0.0, len(t) - 1.0)
    lo = np.floor(idx).astype(int)
    hi = np.minimum(lo + 1, len(t) - 1)
    frac = idx - lo
    out_lin = cmd_lin[lo] * (1.0 - frac) + cmd_lin[hi] * frac
    out_ang = cmd_ang[lo] * (1.0 - frac) + cmd_ang[hi] * frac
    pre = np.arange(len(t)) < shift
    out_lin = np.where(pre, 0.0, out_lin)
    out_ang = np.where(pre, 0.0, out_ang)
    return out_lin, out_ang


@dataclass
class WindowSet:
    """Open-loop windows cut out of one run."""

    start_indices: np.ndarray
    horizons_s: np.ndarray
    horizon_steps: np.ndarray
    dt: float
    u_lin: np.ndarray
    u_ang: np.ndarray
    truth: dict[str, np.ndarray]
    state0: PlantState
    origin_run_index: np.ndarray | None = None

    def __post_init__(self) -> None:
        windows = len(self.start_indices)
        span = int(np.max(self.horizon_steps))
        if self.horizons_s.shape != self.horizon_steps.shape:
            raise ValueError("horizons_s and horizon_steps must have the same shape")
        for name, array in (("u_lin", self.u_lin), ("u_ang", self.u_ang), *self.truth.items()):
            if array.shape != (windows, span):
                raise ValueError(f"{name} has shape {array.shape}, expected {(windows, span)}")
        if self.origin_run_index is not None and self.origin_run_index.shape != (windows,):
            raise ValueError("origin_run_index must have one entry per window")

    def count(self) -> int:
        return len(self.start_indices)

    def subsample(self, limit: int) -> WindowSet:
        """Thin the window set to at most `limit` windows, evenly across the run."""
        if limit <= 0 or self.count() <= limit:
            return self
        pick = np.linspace(0, self.count() - 1, limit).astype(int)
        return WindowSet(
            start_indices=self.start_indices[pick],
            horizons_s=self.horizons_s,
            horizon_steps=self.horizon_steps,
            dt=self.dt,
            u_lin=self.u_lin[pick],
            u_ang=self.u_ang[pick],
            truth={k: v[pick] for k, v in self.truth.items()},
            state0=PlantState(
                x=np.asarray(self.state0.x)[pick],
                y=np.asarray(self.state0.y)[pick],
                theta=np.asarray(self.state0.theta)[pick],
                v=np.asarray(self.state0.v)[pick],
                w=np.asarray(self.state0.w)[pick],
            ),
            origin_run_index=None if self.origin_run_index is None else self.origin_run_index[pick],
        )


def concat_windows(sets: Sequence[WindowSet]) -> WindowSet:
    """Stack window sets from several runs into one batch. All must share dt and horizons."""
    if not sets:
        raise ValueError("no window sets to concatenate")
    first = sets[0]
    for other in sets[1:]:
        if other.dt != first.dt or not np.array_equal(other.horizon_steps, first.horizon_steps):
            raise ValueError("window sets must share dt and horizons")
    return WindowSet(
        start_indices=np.concatenate([s.start_indices for s in sets]),
        horizons_s=first.horizons_s,
        horizon_steps=first.horizon_steps,
        dt=first.dt,
        u_lin=np.concatenate([s.u_lin for s in sets]),
        u_ang=np.concatenate([s.u_ang for s in sets]),
        truth={k: np.concatenate([s.truth[k] for s in sets]) for k in first.truth},
        state0=PlantState(
            x=np.concatenate([np.asarray(s.state0.x) for s in sets]),
            y=np.concatenate([np.asarray(s.state0.y) for s in sets]),
            theta=np.concatenate([np.asarray(s.state0.theta) for s in sets]),
            v=np.concatenate([np.asarray(s.state0.v) for s in sets]),
            w=np.concatenate([np.asarray(s.state0.w) for s in sets]),
        ),
        origin_run_index=(
            None
            if any(s.origin_run_index is None for s in sets)
            else np.concatenate([s.origin_run_index for s in sets])
        ),
    )


DEFAULT_HORIZONS = (0.033, 0.066, 0.100, 0.200, 0.300, 0.500)


def make_windows(
    t: np.ndarray,
    v: np.ndarray,
    w: np.ndarray,
    theta: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    cmd_lin: np.ndarray,
    cmd_ang: np.ndarray,
    *,
    dt: float,
    delay_s: float,
    horizons: Sequence[float] = DEFAULT_HORIZONS,
    stride_s: float = 0.05,
    valid: np.ndarray | None = None,
    origin: int | None = None,
) -> WindowSet | None:
    """Cut strided windows out of a run, dropping any that overlap invalid samples."""
    horizons_arr = np.asarray(horizons, dtype=float)
    steps = np.maximum(1, np.round(horizons_arr / dt).astype(int))
    span = int(steps.max())
    stride = max(1, int(round(stride_s / dt)))
    starts = np.arange(0, len(t) - span - 1, stride)
    if len(starts) == 0:
        return None

    if valid is not None:
        keep = np.array([bool(valid[s : s + span + 1].all()) for s in starts])
        starts = starts[keep]
        if len(starts) == 0:
            return None

    u_lin_d, u_ang_d = delayed_commands(t, cmd_lin, cmd_ang, delay_s, dt)
    offsets = np.arange(1, span + 1)
    idx = starts[:, None] + offsets[None, :]

    return WindowSet(
        start_indices=starts,
        horizons_s=steps * dt,
        horizon_steps=steps,
        dt=dt,
        u_lin=u_lin_d[idx],
        u_ang=u_ang_d[idx],
        truth={"x": x[idx], "y": y[idx], "theta": theta[idx], "v": v[idx], "w": w[idx]},
        state0=PlantState(x=x[starts], y=y[starts], theta=theta[starts], v=v[starts], w=w[starts]),
        origin_run_index=None if origin is None else np.full(len(starts), origin, dtype=int),
    )


@dataclass
class WindowErrors:
    """Open-loop error at each horizon, decomposed in the body frame at the window start."""

    along_m: np.ndarray
    cross_m: np.ndarray
    heading_rad: np.ndarray
    speed_at_start: np.ndarray
    yaw_rate_at_start: np.ndarray
    horizons_s: np.ndarray
    start_indices: np.ndarray
    u_lin0: np.ndarray | None = None
    u_ang0: np.ndarray | None = None
    origin_run_index: np.ndarray | None = None

    def __post_init__(self) -> None:
        windows = len(self.start_indices)
        horizons = len(self.horizons_s)
        for name, array in (
            ("along_m", self.along_m),
            ("cross_m", self.cross_m),
            ("heading_rad", self.heading_rad),
        ):
            if array.shape != (windows, horizons):
                raise ValueError(f"{name} has shape {array.shape}, expected {(windows, horizons)}")
        for name, array in (
            ("speed_at_start", self.speed_at_start),
            ("yaw_rate_at_start", self.yaw_rate_at_start),
        ):
            if array.shape != (windows,):
                raise ValueError(f"{name} has shape {array.shape}, expected {(windows,)}")

    @property
    def position(self) -> np.ndarray:
        out: np.ndarray = np.hypot(self.along_m, self.cross_m)
        return out

    def concat(self, other: WindowErrors) -> WindowErrors:
        return WindowErrors(
            along_m=np.concatenate([self.along_m, other.along_m]),
            cross_m=np.concatenate([self.cross_m, other.cross_m]),
            heading_rad=np.concatenate([self.heading_rad, other.heading_rad]),
            speed_at_start=np.concatenate([self.speed_at_start, other.speed_at_start]),
            yaw_rate_at_start=np.concatenate([self.yaw_rate_at_start, other.yaw_rate_at_start]),
            horizons_s=self.horizons_s,
            start_indices=np.concatenate([self.start_indices, other.start_indices]),
        )


def wrap_angle(a: np.ndarray) -> np.ndarray:
    """Wrap to (-pi, pi]. Every heading residual goes through this."""
    return (np.asarray(a, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


def predict_windows(
    windows: WindowSet,
    p: PlantParams,
    structure: ModelStructure = FULL_MODEL,
    substep_s: float = SUBSTEP_S,
) -> WindowErrors:
    """Run every window open loop and return body-frame error at each horizon."""
    final, traj = simulate(
        windows.u_lin,
        windows.u_ang,
        windows.dt,
        p,
        state=windows.state0,
        structure=structure,
        substep_s=substep_s,
    )
    del final

    horizon_columns = windows.horizon_steps - 1
    dx = traj["x"][:, horizon_columns] - windows.truth["x"][:, horizon_columns]
    dy = traj["y"][:, horizon_columns] - windows.truth["y"][:, horizon_columns]
    dtheta = wrap_angle(
        traj["theta"][:, horizon_columns] - windows.truth["theta"][:, horizon_columns]
    )

    theta0 = np.asarray(windows.state0.theta, dtype=float)[:, None]
    along = dx * np.cos(theta0) + dy * np.sin(theta0)
    cross = -dx * np.sin(theta0) + dy * np.cos(theta0)

    return WindowErrors(
        along_m=along,
        cross_m=cross,
        heading_rad=dtheta,
        speed_at_start=np.abs(np.asarray(windows.state0.v, dtype=float)),
        yaw_rate_at_start=np.abs(np.asarray(windows.state0.w, dtype=float)),
        horizons_s=windows.horizons_s,
        start_indices=windows.start_indices,
        u_lin0=windows.u_lin[:, 0],
        u_ang0=windows.u_ang[:, 0],
        origin_run_index=windows.origin_run_index,
    )
