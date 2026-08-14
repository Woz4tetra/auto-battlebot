"""Grey-box drivetrain plant model: command in, pose out.

One implementation, used by the plant fit, by the process-noise fit, and (mirrored exactly) by
the C++ filter and the kinematic sim. Every term is a physical effect with a parameter that can
be defended, and each is separately excitable by one phase of the jig drive protocol:

    1. Static input map   deadzone and per-sign gain           (E7, E10 deadzone; E8, E11 gain)
    2. Transport delay    command issue to wheel response      (every command edge)
    3. Actuator lag       first-order, accel and decel apart   (E8, E9, E11)
    4. Cross-coupling     steer-brake and angular droop        (E13 grid)
    5. Kinematics         exact arc integration, 2 ms substeps

Structure is switchable through `ModelStructure` so the model ladder in the plan (M0 static map,
M1 plus delay, M2 plus symmetric lag, M3 plus asymmetry, M4 plus coupling) is the same code path
with terms turned off, not five separate models that could disagree for uninteresting reasons.

The prediction the Kalman filter actually depends on is multi-step, not one-step, so this module
also owns the windowed open-loop prediction used to score it (`predict_windows`). One-step error
can be faked by any model with a delay term; 500 ms of open-loop coast cannot.

See `docs/experiments/kalman_filter/kalman_filter_plan.md` part 1.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

# Substep for the internal integration. At the calibrated 61.5 rad/s top yaw rate this is
# 0.12 rad of rotation per substep, where holding v and w constant costs well under a millimeter.
SUBSTEP_S = 0.002

# Below this yaw rate the arc integration divides by ~0, so it falls back to the straight-line
# form. 1e-6 rad/s over a 500 ms horizon is 5e-7 rad of heading, far below anything measurable.
STRAIGHT_W = 1e-6


def toml_float(value: float) -> str:
    """Format a float so TOML reads it back as a float.

    `0` and `5` are integers in TOML, and a reader that asks for a double gets a type error on a
    parameter that happened to land on a round number. Every emitted value keeps a decimal point.
    """
    text = f"{value:.6g}"
    if "." not in text and "e" not in text and "n" not in text:
        text += ".0"
    return text


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PlantParams:
    """The parameter vector, with stage 2 values as priors.

    Priors come from the 2026-07-03 AprilTag calibration (run `175805`,
    `docs/experiments/control_improvement/stage2_mr_stabs_mk2_calibration.md`). Only the
    actuation lag is trusted; the rest are starting points for the jig fit. Deadzones are floored
    at 0.04 there because that was the smallest command tested, not because motion started there.
    """

    dz_lin_fwd: float = 0.04
    dz_lin_rev: float = 0.04
    dz_ang_l: float = 0.04
    dz_ang_r: float = 0.04
    k_fwd: float = 5.60  # m/s at full command
    k_rev: float = 4.84
    k_ang: float = 61.5  # rad/s, camera-derived and possibly inflated by yaw keypoint flips
    tau_lin_a: float = 0.058
    tau_lin_d: float = 0.078
    tau_ang_a: float = 0.066
    tau_ang_d: float = 0.078  # unmeasured in stage 2; seeded from the linear decel constant
    delay_s: float = 0.059
    c_sb: float = 0.0  # steer-brake: fraction of forward speed lost per unit |angular command|
    c_ad: float = 0.0  # angular droop: fraction of yaw rate lost per unit |linear command|

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
        return cls.from_dict(data.get(table, data))

    def to_toml(self, table: str = "plant") -> str:
        lines = [f"[{table}]"]
        for name, value in self.to_dict().items():
            lines.append(f"{name} = {toml_float(value)}")
        return "\n".join(lines) + "\n"


# Sensible bounds for the joint fit. They are wide enough not to shape the answer and tight
# enough that a diverging fit fails visibly instead of wandering into nonsense.
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
}


@dataclass(frozen=True)
class ModelStructure:
    """Which terms are in the model. One rung of the ladder each."""

    name: str = "M4"
    use_delay: bool = True
    use_lag: bool = True
    asymmetric_tau: bool = True  # accel and decel constants differ
    asymmetric_gain: bool = True  # forward and reverse max speeds differ
    coupling: bool = True  # steer-brake and angular droop

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
        return out

    def free_names(self) -> list[str]:
        """Parameters this structure lets the fit move. Delay is handled by profile search, so it
        is not in this list even when the structure uses it."""
        names = ["dz_lin_fwd", "dz_ang_l", "k_fwd", "k_ang"]
        if self.asymmetric_gain:
            names += ["dz_lin_rev", "dz_ang_r", "k_rev"]
        if self.use_lag:
            names += ["tau_lin_a", "tau_ang_a"]
            if self.asymmetric_tau:
                names += ["tau_lin_d", "tau_ang_d"]
        if self.coupling:
            names += ["c_sb", "c_ad"]
        return names


MODEL_LADDER: tuple[ModelStructure, ...] = (
    ModelStructure("M0", False, False, False, False, False),
    ModelStructure("M1", True, False, False, False, False),
    ModelStructure("M2", True, True, False, False, False),
    ModelStructure("M3", True, True, True, True, False),
    ModelStructure("M4", True, True, True, True, True),
)

FULL_MODEL = MODEL_LADDER[-1]


# ---------------------------------------------------------------------------
# Static input map and steady state
# ---------------------------------------------------------------------------


def effective_command(u: np.ndarray, dz_pos: float, dz_neg: float) -> np.ndarray:
    """Deadzone removal and rescale, per sign.

        u_eff = 0                                  if |u| <= dz
        u_eff = sign(u) * (|u| - dz) / (1 - dz)    otherwise

    The rescale keeps full command mapping to full effect, so the gain stays the max speed rather
    than becoming max speed divided by (1 - dz).
    """
    u = np.asarray(u, dtype=float)
    dz = np.where(u >= 0.0, dz_pos, dz_neg)
    dz = np.clip(dz, 0.0, 0.95)
    mag = np.maximum(np.abs(u) - dz, 0.0) / (1.0 - dz)
    out: np.ndarray = np.sign(u) * mag
    return out


def steady_state(u_lin: np.ndarray, u_ang: np.ndarray, p: PlantParams) -> tuple[Any, Any]:
    """Target body speed and yaw rate for a held command, coupling included.

    Coupling is multiplicative on the target, not on the achieved speed: turning costs a fraction
    of the forward command's authority, it does not brake a robot that is already coasting.
    """
    lin_eff = effective_command(u_lin, p.dz_lin_fwd, p.dz_lin_rev)
    ang_eff = effective_command(u_ang, p.dz_ang_l, p.dz_ang_r)
    v_target = np.where(lin_eff >= 0.0, p.k_fwd, p.k_rev) * lin_eff
    w_target = p.k_ang * ang_eff
    if p.c_sb:
        v_target = v_target * np.clip(1.0 - p.c_sb * np.abs(ang_eff), 0.0, None)
    if p.c_ad:
        w_target = w_target * np.clip(1.0 - p.c_ad * np.abs(lin_eff), 0.0, None)
    return v_target, w_target


def _lag_alpha(dt: float, tau: Any) -> Any:
    """Exact first-order discretization over dt. tau = 0 means no lag (the M0/M1 rungs)."""
    tau = np.asarray(tau, dtype=float)
    return np.where(tau > 1e-9, np.exp(-dt / np.maximum(tau, 1e-9)), 0.0)


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------


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
    """Advance one substep: pose along the arc, then the first-order velocity update.

    Pose uses the velocity at the start of the substep and the velocity update follows it, which
    is the ordering the C++ mirror must match. At a 2 ms substep the difference against using the
    end-of-substep velocity is dt times the acceleration, under a millimeter at any speed this
    drivetrain reaches.

        theta' = theta + w*dt
        x'     = x + (v/w) * (sin(theta') - sin(theta))
        y'     = y - (v/w) * (cos(theta') - cos(theta))
    """
    v = np.asarray(state.v, dtype=float)
    w = np.asarray(state.w, dtype=float)
    theta = np.asarray(state.theta, dtype=float)

    theta_next = theta + w * dt
    turning = np.abs(w) > STRAIGHT_W
    w_safe = np.where(turning, w, 1.0)
    radius = v / w_safe
    dx = np.where(turning, radius * (np.sin(theta_next) - np.sin(theta)), v * dt * np.cos(theta))
    dy = np.where(turning, -radius * (np.cos(theta_next) - np.cos(theta)), v * dt * np.sin(theta))

    # Accel or decel is decided per channel by whether the target is further from zero than the
    # current speed. Braking into a reversal uses the decel constant until the sign flips, which
    # is what the drivetrain does: friction first, then torque.
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
    """Run the plant over a command timeline. Vectorized over leading dimensions.

    `u_lin` and `u_ang` are shaped (..., N) and already delayed (see `delayed_commands`), so this
    function has no notion of transport delay. Keeping the delay in the command sampling rather
    than in a ring buffer is what makes a fractional delay exact and the batch case cheap: a pure
    delay is a shift of the command timeline and nothing more.

    Returns the final state and the per-sample trajectory, so the same call serves both the
    windowed error (final state only) and plotting (trajectory).
    """
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
        # No actuator dynamics: velocity equals its target the instant the delayed command
        # arrives. Substepping is then pointless, so it is skipped.
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
    """Stateful wrapper with a command ring buffer, mirroring the runtime C++ shape.

    The fit uses `simulate`, which shifts the command timeline instead. This class exists so the
    sim and any Python-side runtime experiment step the same model the filter will, and so a
    golden-vector test can compare both against the C++ implementation.

    One deliberate difference from `simulate`: the buffer holds each command until the next one
    arrives, while `delayed_commands` interpolates between command samples. The two agree exactly
    when the delay is a whole number of steps and differ by less than one step's worth of command
    otherwise. Runtime is the zero-order-hold case, because that is what the transmitter does.
    """

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


# ---------------------------------------------------------------------------
# Command sampling and delay
# ---------------------------------------------------------------------------


def delayed_commands(
    t: np.ndarray,
    cmd_lin: np.ndarray,
    cmd_ang: np.ndarray,
    delay_s: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Shift a uniformly sampled command timeline back by `delay_s`.

    The shift is fractional and interpolated linearly between samples. At the fit grid rate the
    command is already a staircase sampled finely, so interpolating between two samples of it is
    a sub-grid-step approximation to the true edge time, which is what makes the delay profile
    smooth enough to search on a 2 ms grid.
    """
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
    # Before the delay has elapsed there was no command, not the first command held early.
    pre = np.arange(len(t)) < shift
    out_lin = np.where(pre, 0.0, out_lin)
    out_ang = np.where(pre, 0.0, out_ang)
    return out_lin, out_ang


# ---------------------------------------------------------------------------
# Windowed open-loop prediction
# ---------------------------------------------------------------------------


@dataclass
class WindowSet:
    """Open-loop windows cut out of one run.

    Each window reinitializes the model from ground truth at its start and runs open loop to each
    horizon. That is exactly what the filter does between camera corrections, so it is what the
    fit optimizes and what the noise model characterizes.
    """

    starts: np.ndarray  # sample index of each window start
    horizons: np.ndarray  # seconds, snapped to the grid (steps * dt), not as requested
    steps: np.ndarray  # horizon in grid samples
    dt: float
    u_lin: np.ndarray  # (W, H) delayed command over the longest horizon
    u_ang: np.ndarray
    truth: dict[str, np.ndarray]  # (W, H) ground truth per sample
    state0: PlantState

    def count(self) -> int:
        return len(self.starts)

    def subsample(self, limit: int) -> WindowSet:
        """Thin the window set to at most `limit` windows, evenly across the run.

        Windows overlap heavily at a 50 ms stride, so thinning costs far less information than
        the count suggests, and it is the knob that keeps a delay profile search over 60 grid
        points to minutes instead of an hour.
        """
        if limit <= 0 or self.count() <= limit:
            return self
        pick = np.linspace(0, self.count() - 1, limit).astype(int)
        return WindowSet(
            starts=self.starts[pick],
            horizons=self.horizons,
            steps=self.steps,
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
        )


def concat_windows(sets: Sequence[WindowSet]) -> WindowSet:
    """Stack window sets from several runs into one batch. All must share dt and horizons."""
    if not sets:
        raise ValueError("no window sets to concatenate")
    first = sets[0]
    for other in sets[1:]:
        if other.dt != first.dt or not np.array_equal(other.steps, first.steps):
            raise ValueError("window sets must share dt and horizons")
    return WindowSet(
        starts=np.concatenate([s.starts for s in sets]),
        horizons=first.horizons,
        steps=first.steps,
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
) -> WindowSet | None:
    """Cut strided windows out of a run, dropping any that overlap invalid samples.

    `valid` is the per-sample mask of usable ground truth (encoder slip excluded, for instance).
    A window is kept only if every sample it spans is valid, so one slipped wheel cannot steer
    the fit through a window that mostly looks fine.
    """
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
        starts=starts,
        # Report the horizons actually evaluated. A requested 33 ms on a 5 ms grid is 35 ms, and
        # a table that says 33 while measuring 35 is a table that lies about its own axis.
        horizons=steps * dt,
        steps=steps,
        dt=dt,
        u_lin=u_lin_d[idx],
        u_ang=u_ang_d[idx],
        truth={"x": x[idx], "y": y[idx], "theta": theta[idx], "v": v[idx], "w": w[idx]},
        state0=PlantState(x=x[starts], y=y[starts], theta=theta[starts], v=v[starts], w=w[starts]),
    )


@dataclass
class WindowErrors:
    """Open-loop error at each horizon, decomposed in the body frame at the window start.

    The decomposition matters because the error sources differ: along-track error comes from
    speed and delay errors, cross-track error comes mostly from heading error and grows faster.
    A Euclidean position error hides which of the two is broken.
    """

    along: np.ndarray  # (W, H) meters
    cross: np.ndarray  # (W, H) meters
    heading: np.ndarray  # (W, H) radians, wrapped
    speed: np.ndarray  # (W,) truth speed at window start, a conditioning variable
    yaw_rate: np.ndarray  # (W,) truth yaw rate at window start
    horizons: np.ndarray  # (H,)
    starts: np.ndarray  # (W,)

    @property
    def position(self) -> np.ndarray:
        out: np.ndarray = np.hypot(self.along, self.cross)
        return out

    def concat(self, other: WindowErrors) -> WindowErrors:
        return WindowErrors(
            along=np.concatenate([self.along, other.along]),
            cross=np.concatenate([self.cross, other.cross]),
            heading=np.concatenate([self.heading, other.heading]),
            speed=np.concatenate([self.speed, other.speed]),
            yaw_rate=np.concatenate([self.yaw_rate, other.yaw_rate]),
            horizons=self.horizons,
            starts=np.concatenate([self.starts, other.starts]),
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
    """Run every window open loop and return the body-frame error at each horizon.

    All windows step together as one batched simulation, so cost scales with the longest horizon
    rather than with the number of windows times their length.
    """
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

    cols = windows.steps - 1  # step h lands at index h-1 in the trajectory
    dx = traj["x"][:, cols] - windows.truth["x"][:, cols]
    dy = traj["y"][:, cols] - windows.truth["y"][:, cols]
    dtheta = wrap_angle(traj["theta"][:, cols] - windows.truth["theta"][:, cols])

    theta0 = np.asarray(windows.state0.theta, dtype=float)[:, None]
    along = dx * np.cos(theta0) + dy * np.sin(theta0)
    cross = -dx * np.sin(theta0) + dy * np.cos(theta0)

    return WindowErrors(
        along=along,
        cross=cross,
        heading=dtheta,
        speed=np.abs(np.asarray(windows.state0.v, dtype=float)),
        yaw_rate=np.abs(np.asarray(windows.state0.w, dtype=float)),
        horizons=windows.horizons,
        starts=windows.starts,
    )
