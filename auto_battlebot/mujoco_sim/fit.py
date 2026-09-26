"""Staged CMA-ES fit of the Mr Stabs Mk2 MuJoCo plant against hand-driven windows.

Each generation scores every candidate on every window at once: one world per (candidate,
window) pair. CMA-ES is gradient-free, so the delay (a tape shift) and the chaotic contact
don't need special handling. Parameters live on a unit cube (log scale where the range spans
decades) and are released in stages so each group is fit on the data that identifies it:

1. flat windows: delay, deadzones, throttle curve, R, efficiency, gearbox friction
2. flat windows: skid and torsional friction, left/right ratio, wheel sliding friction
3. nose-lift windows: armature scale and wheel traction, everything else frozen
4. all windows: every continuous parameter, starting from the staged result

The zero-command mode (brake or coast) is discrete; `fit` runs once per mode asked for.

Loss: at each horizon, the raw measurement nearest to it (within 20 ms) against the prediction
at that measurement's time. Position and heading errors are divided by the session noise floor
and Huber-weighted; nose-lift windows add a pitch term over every measurement in the window.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field, fields

import numpy as np

from auto_battlebot.mujoco_sim.actuator import PlantParams
from auto_battlebot.mujoco_sim.rollout import pose_from_qpos
from auto_battlebot.mujoco_sim.session import HORIZON_TOLERANCE_S, WindowBatch

HUBER_DELTA = 3.0  # noise-floor multiples


@dataclass(frozen=True)
class ParamSpec:
    name: str
    lo: float
    hi: float
    log: bool = False

    def to_unit(self, value: float) -> float:
        if self.log:
            return (math.log(value) - math.log(self.lo)) / (math.log(self.hi) - math.log(self.lo))
        return (value - self.lo) / (self.hi - self.lo)

    def from_unit(self, u: float) -> float:
        u = min(max(u, 0.0), 1.0)
        if self.log:
            return math.exp(math.log(self.lo) + u * (math.log(self.hi) - math.log(self.lo)))
        return self.lo + u * (self.hi - self.lo)


# Bounds from the plan's parameter table; "wide" ones span what the robot could physically be.
PARAM_SPECS: tuple[ParamSpec, ...] = (
    ParamSpec("delay_s", 0.0, 0.06),
    ParamSpec("deadzone_left", 0.0, 0.06),
    ParamSpec("deadzone_right", 0.0, 0.06),
    ParamSpec("curve", -0.9, 2.0),
    ParamSpec("resistance_ohm", 0.05, 1.0, log=True),
    ParamSpec("efficiency", 0.6, 0.95),
    ParamSpec("lr_gain_ratio", 0.8, 1.2),
    ParamSpec("joint_frictionloss", 1e-4, 5e-2, log=True),
    ParamSpec("joint_damping", 1e-6, 1e-3, log=True),
    ParamSpec("wheel_mu_slide", 0.3, 3.0, log=True),
    ParamSpec("wheel_mu_torsion", 1e-4, 5e-2, log=True),
    ParamSpec("wheel_mu_roll", 1e-6, 1e-2, log=True),
    ParamSpec("skid_mu", 0.05, 1.5, log=True),
    ParamSpec("solref_timeconst", 0.004, 0.05, log=True),
    ParamSpec("solimp_dmax", 0.9, 0.9999),
    ParamSpec("armature_scale", 0.5, 1.5),
)
SPEC_BY_NAME = {s.name: s for s in PARAM_SPECS}


@dataclass(frozen=True)
class Stage:
    name: str
    free: tuple[str, ...]
    kinds: tuple[str, ...]


STAGES: tuple[Stage, ...] = (
    Stage(
        "straight",
        (
            "delay_s",
            "deadzone_left",
            "deadzone_right",
            "curve",
            "resistance_ohm",
            "efficiency",
            "joint_frictionloss",
            "joint_damping",
        ),
        ("flat",),
    ),
    Stage(
        "turning",
        ("skid_mu", "wheel_mu_torsion", "lr_gain_ratio", "wheel_mu_slide"),
        ("flat",),
    ),
    Stage("nose_lift", ("armature_scale", "wheel_mu_slide"), ("nose_lift",)),
    Stage("joint", tuple(s.name for s in PARAM_SPECS), ("flat", "nose_lift")),
)


def with_values(base: PlantParams, names: Sequence[str], unit: np.ndarray) -> PlantParams:
    return base.replace(**{n: SPEC_BY_NAME[n].from_unit(float(u)) for n, u in zip(names, unit)})


def unit_of(params: PlantParams, names: Sequence[str]) -> np.ndarray:
    return np.array([SPEC_BY_NAME[n].to_unit(getattr(params, n)) for n in names])


def _huber(r: np.ndarray, delta: float = HUBER_DELTA) -> np.ndarray:
    a = np.abs(r)
    return np.asarray(np.where(a <= delta, 0.5 * a**2, delta * (a - 0.5 * delta)))


def _wrap(a: np.ndarray) -> np.ndarray:
    return np.asarray((a + np.pi) % (2 * np.pi) - np.pi)


@dataclass
class WindowErrors:
    """Errors at each horizon for each window (NaN where no measurement landed near it)."""

    position: np.ndarray  # (N, H) meters
    heading: np.ndarray  # (N, H) radians
    pitch_rms: np.ndarray  # (N,) radians over the whole window, NaN for flat windows
    loss: np.ndarray  # (N,)


def window_errors(batch: WindowBatch, qpos: np.ndarray, record_dt: float) -> WindowErrors:
    """qpos: (N, S, 7) recorded chassis pose, sample k at time k * record_dt from the start."""
    pose = pose_from_qpos(qpos)
    n, samples = qpos.shape[:2]
    sample_t = np.arange(samples) * record_dt
    h = len(batch.horizons)
    pos_err = np.full((n, h), np.nan)
    yaw_err = np.full((n, h), np.nan)
    pitch_rms = np.full(n, np.nan)
    loss = np.zeros(n)
    for i in range(n):
        mt = batch.meas_t[i]
        if len(mt) == 0:
            continue
        px = np.interp(mt, sample_t, pose["x"][i])
        py = np.interp(mt, sample_t, pose["y"][i])
        pyaw = np.interp(mt, sample_t, np.unwrap(pose["yaw"][i]))
        terms: list[float] = []
        for j, horizon in enumerate(batch.horizons):
            k = int(np.argmin(np.abs(mt - horizon)))
            if abs(mt[k] - horizon) > HORIZON_TOLERANCE_S:
                continue
            pos_err[i, j] = float(np.hypot(px[k] - batch.meas_x[i][k], py[k] - batch.meas_y[i][k]))
            yaw_err[i, j] = float(_wrap(np.asarray(pyaw[k] - batch.meas_yaw[i][k])))
            terms.append(float(_huber(np.asarray(pos_err[i, j] / batch.noise_xy[i]))))
            terms.append(float(_huber(np.asarray(yaw_err[i, j] / batch.noise_yaw[i]))))
        if batch.kind[i] == "nose_lift":
            pp = np.interp(mt, sample_t, pose["pitch"][i])
            # Both are the rotation about body +y (FLU), so nose-down is positive in each.
            diff = pp - batch.meas_pitch[i]
            pitch_rms[i] = float(np.sqrt(np.mean(diff**2)))
            resid = diff / batch.noise_pitch[i]
            terms.append(float(np.mean(_huber(resid))))
        loss[i] = float(np.mean(terms)) if terms else 0.0
    return WindowErrors(position=pos_err, heading=yaw_err, pitch_rms=pitch_rms, loss=loss)


# A simulator takes a window batch and one parameter set per world (len == len(batch)) and
# returns the recorded chassis qpos (N, S, 7) plus the recording interval.
Simulator = Callable[[WindowBatch, list[PlantParams]], tuple[np.ndarray, float]]


def warp_simulator(rollout_factory: Callable[[int, int], object]) -> Simulator:
    """Adapt WarpRollout to the Simulator shape, rebuilding the batch only when N changes."""
    cache: dict[tuple[int, int], object] = {}

    def simulate(batch: WindowBatch, params: list[PlantParams]) -> tuple[np.ndarray, float]:
        key = (len(batch), batch.steps)
        if key not in cache:
            cache.clear()
            cache[key] = rollout_factory(len(batch), batch.steps)
        rollout = cache[key]
        volts, zero = batch.tapes(params)
        rollout.set_params(params)  # type: ignore[attr-defined]
        rollout.set_tapes(volts, zero)  # type: ignore[attr-defined]
        qpos = rollout.run(batch.state)  # type: ignore[attr-defined]
        return qpos, batch.dt * rollout.record_every  # type: ignore[attr-defined]

    return simulate


def score_candidates(
    simulate: Simulator, batch: WindowBatch, candidates: list[PlantParams]
) -> tuple[np.ndarray, list[WindowErrors]]:
    """Mean window loss per candidate. Worlds are laid out candidate-major."""
    n = len(batch)
    tiled = batch.take(np.tile(np.arange(n), len(candidates)))
    per_world = [c for c in candidates for _ in range(n)]
    qpos, record_dt = simulate(tiled, per_world)
    errors = []
    losses = np.empty(len(candidates))
    for c in range(len(candidates)):
        sl = slice(c * n, (c + 1) * n)
        err = window_errors(batch, qpos[sl], record_dt)
        errors.append(err)
        losses[c] = float(np.mean(err.loss)) if n else math.inf
        if not np.isfinite(qpos[sl]).all():
            losses[c] = math.inf
    return losses, errors


@dataclass
class StageResult:
    stage: str
    best: PlantParams
    loss: float
    history: list[float] = field(default_factory=list)


@dataclass
class FitResult:
    best: PlantParams
    loss: float
    stages: list[StageResult]


def run_stage(
    simulate: Simulator,
    batch: WindowBatch,
    start: PlantParams,
    stage: Stage,
    popsize: int,
    generations: int,
    sigma0: float = 0.2,
    seed: int = 0,
    log: Callable[[str], None] = print,
) -> StageResult:
    import cma

    sub = batch.take(np.nonzero(np.isin(batch.kind, stage.kinds))[0])
    if len(sub) == 0:
        log(f"[{stage.name}] no {stage.kinds} windows; stage skipped")
        return StageResult(stage.name, start, math.nan)
    x0 = unit_of(start, stage.free)
    es = cma.CMAEvolutionStrategy(
        x0,
        sigma0,
        {"bounds": [0.0, 1.0], "popsize": popsize, "seed": seed + 1, "verbose": -9},
    )
    # Score the starting point first: a stage must never hand back something worse than it got.
    start_loss, _ = score_candidates(simulate, sub, [start])
    best = start
    best_loss = float(start_loss[0]) if np.isfinite(start_loss[0]) else math.inf
    history = []
    for gen in range(generations):
        xs = es.ask()
        cands = [with_values(start, stage.free, np.asarray(x)) for x in xs]
        losses, _ = score_candidates(simulate, sub, cands)
        es.tell(xs, [float(v) if np.isfinite(v) else 1e12 for v in losses])
        k = int(np.argmin(losses))
        if losses[k] < best_loss:
            best, best_loss = cands[k], float(losses[k])
        history.append(best_loss)
        log(
            f"[{stage.name}] gen {gen + 1}/{generations} "
            f"best {best_loss:.4f} median {np.median(losses):.4f}"
        )
        if es.stop():
            break
    return StageResult(stage.name, best, best_loss, history)


def fit(
    simulate: Simulator,
    batch: WindowBatch,
    start: PlantParams | None = None,
    stages: Sequence[Stage] = STAGES,
    popsize: int = 32,
    generations: int = 40,
    seed: int = 0,
    log: Callable[[str], None] = print,
) -> FitResult:
    current = start or PlantParams()
    results = []
    for i, stage in enumerate(stages):
        res = run_stage(
            simulate, batch, current, stage, popsize, generations, seed=seed + 101 * i, log=log
        )
        results.append(res)
        if math.isfinite(res.loss):
            current = res.best
    finite = [r for r in results if math.isfinite(r.loss)]
    return FitResult(current, finite[-1].loss if finite else math.nan, results)


def params_dict(params: PlantParams) -> dict[str, float | bool]:
    return {f.name: getattr(params, f.name) for f in fields(params)}


def restart_spread(results: list[FitResult]) -> dict[str, dict[str, float]]:
    """Per parameter, spread of the restarts' best values in unit-cube terms, to judge
    whether the fit identifies it (plan validation check 6)."""
    out = {}
    for spec in PARAM_SPECS:
        units = np.array([spec.to_unit(getattr(r.best, spec.name)) for r in results])
        values = np.array([getattr(r.best, spec.name) for r in results])
        out[spec.name] = {
            "mean": float(values.mean()),
            "min": float(values.min()),
            "max": float(values.max()),
            "unit_range": float(units.max() - units.min()),
        }
    return out


def tradeoff_pairs(
    results: list[FitResult], threshold: float = 0.8
) -> list[tuple[str, str, float]]:
    """Parameter pairs whose restart values correlate past `threshold`: candidates to pin one
    of from an outside measurement."""
    if len(results) < 3:
        return []
    units = np.array([[s.to_unit(getattr(r.best, s.name)) for s in PARAM_SPECS] for r in results])
    names = [s.name for s in PARAM_SPECS]
    out = []
    with np.errstate(invalid="ignore", divide="ignore"):
        corr = np.corrcoef(units.T)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            if np.isfinite(corr[i, j]) and abs(corr[i, j]) >= threshold:
                out.append((names[i], names[j], float(corr[i, j])))
    return out
