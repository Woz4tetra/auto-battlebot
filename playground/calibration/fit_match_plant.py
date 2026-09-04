"""Fit the drivetrain plant from NHRL match driving instead of jig protocols.

Implements docs/experiments/control_improvement/match_plant_fit_plan.md: join the
original transmitted commands to regenerated replay poses, cut open-loop windows,
climb the model ladder from the jig stage A seed, and score three plants (jig
seed, match fit, current sim values) on identical windows.

The loader lives in `auto_battlebot/calibration/match_windows.py` next to the jig loader; the fit
machinery is shared with the jig fit (`plant.predict_windows`, `jig_fit.joint_fit`
with match bounds, `jig_fit.score`).

Usage:
    source scripts/activate_python.sh
    python playground/calibration/fit_match_plant.py \\
        --fit data/recordings/<replay>.mcap:data/saved_recordings/.../<original>.mcap \\
        --validate <replay>.mcap:<original>.mcap \\
        --out playground/calibration/out/plant_match.toml \\
        --report-dir playground/calibration/out/match_fit
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import re
import sys
from pathlib import Path

import numpy as np

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from auto_battlebot.calibration.jig_fit import (
    FitWeights,
    HorizonReport,
    joint_fit,
    print_horizon_table,
    residual_vector,
    score,
)
from auto_battlebot.calibration.match_windows import (
    MatchRun,
    MatchWindows,
    build_match_run,
    build_match_windows,
    load_commands,
    load_replay_track,
    split_run,
)
from auto_battlebot.plant import (
    MODEL_LADDER,
    PARAM_BOUNDS,
    ModelStructure,
    PlantParams,
    WindowErrors,
    predict_windows,
    simulate,
    toml_float,
    wrap_angle,
)

# Deadzones stay at the jig bracket midpoints: fights rarely dwell near the
# motion threshold, so match data cannot resolve them and the jig brackets stay
# the best available priors (plan step 4).
PINNED_DEADZONES = ("dz_lin_fwd", "dz_lin_rev", "dz_ang_l", "dz_ang_r")

# The plan's objective horizons plus 400 ms, where the EKF acceptance criteria
# C2/C3 are stated. Same set as the jig CLI.
HORIZONS = (0.033, 0.066, 0.100, 0.200, 0.300, 0.400, 0.500)

# Radio-dropout gate: commanded hard, moved a fraction of the prediction.
NONRESP_MIN_CMD = 0.30
NONRESP_RATIO = 0.25
NONRESP_MIN_PRED_M = 0.08
NONRESP_MIN_PRED_RAD = 0.5


# ---------------------------------------------------------------------------
# Seed and bounds from the jig stage A output
# ---------------------------------------------------------------------------


def load_seed(path: Path, widen: float) -> tuple[PlantParams, dict[str, tuple[float, float]]]:
    """Stage A parameters plus per-parameter fit bounds.

    Gains and time constants get the jig error bars widened `widen`x (different
    floor, different battery). Coupling and drift terms keep the wide global
    bounds: the jig grid produced zero passing coupling runs, so match data owns
    them. Everything is clipped into PARAM_BOUNDS so a huge jig spread cannot
    unlock nonsense.
    """
    with open(path, "rb") as handle:
        data = tomllib.load(handle)
    params = PlantParams.from_dict(data["plant"])
    provenance = data.get("plant", {}).get("provenance", {})

    bounds: dict[str, tuple[float, float]] = {}
    spread_re = re.compile(r"([-+0-9.eE]+)\s*\+/-\s*([-+0-9.eEna]+)")
    for name in (
        "k_fwd",
        "k_rev",
        "k_ang",
        "tau_lin_a",
        "tau_lin_d",
        "tau_ang_a",
        "tau_ang_d",
    ):
        text = str(provenance.get(name, ""))
        match = spread_re.search(text)
        value = float(getattr(params, name))
        spread = float("nan")
        if match:
            try:
                spread = float(match.group(2))
            except ValueError:
                spread = float("nan")
        if not np.isfinite(spread) or spread <= 0:
            # No usable error bar: fall back to half the value, still clipped.
            spread = 0.5 * abs(value) / widen
        lo, hi = PARAM_BOUNDS[name]
        bounds[name] = (
            max(lo, value - widen * spread),
            min(hi, value + widen * spread),
        )
    # The global c_sb cap of 1.5 predates the jig grid, which measured
    # 2.70 +/- 0.106. The plan gives coupling terms wide bounds, so the cap
    # must sit above what the jig saw, not below it.
    bounds["c_sb"] = (-0.5, 4.0)
    return params, bounds


def load_sim_plant(path: Path) -> PlantParams:
    """Current sim values (plant C), mapped back to the fit's parameter names."""
    with open(path, "rb") as handle:
        data = tomllib.load(handle)
    robot = data["our_robot"]
    return PlantParams(
        dz_lin_fwd=float(robot["deadzone_linear"]),
        dz_lin_rev=float(robot["deadzone_linear_rev"]),
        dz_ang_l=float(robot["deadzone_angular"]),
        dz_ang_r=float(robot["deadzone_angular_right"]),
        k_fwd=float(robot["max_linear_speed_fwd"]),
        k_rev=float(robot["max_linear_speed_rev"]),
        k_ang=float(robot["max_angular_speed"]),
        tau_lin_a=float(robot["tau_linear_accel"]),
        tau_lin_d=float(robot["tau_linear_decel"]),
        tau_ang_a=float(robot["tau_angular_accel"]),
        tau_ang_d=float(robot["tau_angular_decel"]),
        delay_s=float(data["latency"]["command_ms"]) / 1000.0,
        c_sb=float(robot["steer_brake_coeff"]),
        c_ad=float(robot["angular_droop_coeff"]),
        c_drift=0.0,
        c_drift_bias=0.0,
    )


# ---------------------------------------------------------------------------
# Fit
# ---------------------------------------------------------------------------


def pinned_structure(structure: ModelStructure) -> ModelStructure:
    return dataclasses.replace(structure, pinned=PINNED_DEADZONES)


def holdout_metric(err: WindowErrors, weights: FitWeights) -> float:
    """One scalar for rung selection: the fit objective, evaluated on holdout."""
    res = residual_vector(err, weights)
    return float(np.sqrt(np.mean(res**2)))


def profile_delay_match(
    runs: list[MatchRun],
    start: PlantParams,
    structure: ModelStructure,
    weights: FitWeights,
    delay_grid: np.ndarray,
    *,
    stride_s: float,
    max_windows: int,
    max_nfev: int,
    bounds: dict[str, tuple[float, float]],
    drop_spans: list[tuple[int, int]],
) -> tuple[float, np.ndarray]:
    """Grid-profile the transport delay for one rung. Returns (best_delay, costs).

    Resolution is bounded by the ~33 ms frame grid, so the profile answers
    "which third of a frame" at best; the curve going in the report matters more
    than the minimum (a flat profile means match excitation cannot see it).
    """
    costs = []
    for delay in delay_grid:
        mw = build_match_windows(runs, float(delay), HORIZONS, stride_s, max_windows, drop_spans)
        if mw is None:
            costs.append(float("inf"))
            continue
        _params, cost = joint_fit(
            mw.windows,
            start.replace(delay_s=float(delay)),
            structure,
            weights,
            max_nfev=max_nfev,
            bounds=bounds,
        )
        costs.append(cost)
        print(f"    delay {delay * 1000:6.1f} ms -> cost {cost:.4f}")
    best = int(np.argmin(costs))
    return float(delay_grid[best]), np.asarray(costs)


def subset_errors(err: WindowErrors, mask: np.ndarray) -> WindowErrors:
    return WindowErrors(
        along=err.along[mask],
        cross=err.cross[mask],
        heading=err.heading[mask],
        speed=err.speed[mask],
        yaw_rate=err.yaw_rate[mask],
        horizons=err.horizons,
        starts=err.starts[mask],
        u_lin0=None if err.u_lin0 is None else err.u_lin0[mask],
        u_ang0=None if err.u_ang0 is None else err.u_ang0[mask],
        origin=None if err.origin is None else err.origin[mask],
    )


# ---------------------------------------------------------------------------
# Diagnostics on windows
# ---------------------------------------------------------------------------


def observed_and_predicted(
    mw: MatchWindows, params: PlantParams, structure: ModelStructure
) -> dict[str, np.ndarray]:
    """Per-window observed and predicted body-frame motion at each horizon."""
    ws = mw.windows
    _final, traj = simulate(ws.u_lin, ws.u_ang, ws.dt, params, state=ws.state0, structure=structure)
    cols = ws.steps - 1
    th0 = np.asarray(ws.state0.theta, dtype=float)[:, None]
    x0 = np.asarray(ws.state0.x, dtype=float)[:, None]
    y0 = np.asarray(ws.state0.y, dtype=float)[:, None]

    def body(xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        dx = xs - x0
        dy = ys - y0
        return (
            dx * np.cos(th0) + dy * np.sin(th0),
            -dx * np.sin(th0) + dy * np.cos(th0),
        )

    obs_along, obs_cross = body(ws.truth["x"][:, cols], ws.truth["y"][:, cols])
    pred_along, pred_cross = body(traj["x"][:, cols], traj["y"][:, cols])
    return {
        "obs_along": obs_along,
        "obs_cross": obs_cross,
        "pred_along": pred_along,
        "pred_cross": pred_cross,
        "obs_dtheta": wrap_angle(ws.truth["theta"][:, cols] - th0),
        "pred_dtheta": wrap_angle(traj["theta"][:, cols] - th0),
    }


def flag_nonresponsive(
    mw: MatchWindows, params: PlantParams, structure: ModelStructure
) -> np.ndarray:
    """Windows where the command was large but the robot barely answered.

    The driver reported stretches where the robot did not respond to commands.
    Those windows would teach the fit that commands do nothing, so they are
    flagged, listed as a deliverable, and excluded from the final fit.
    """
    motion = observed_and_predicted(mw, params, structure)
    ws = mw.windows
    h = int(np.argmin(np.abs(ws.horizons - 0.400)))
    obs_disp = np.hypot(motion["obs_along"][:, h], motion["obs_cross"][:, h])
    pred_disp = np.hypot(motion["pred_along"][:, h], motion["pred_cross"][:, h])
    obs_rot = np.abs(motion["obs_dtheta"][:, h])
    pred_rot = np.abs(motion["pred_dtheta"][:, h])
    mean_lin = np.mean(np.abs(ws.u_lin), axis=1)
    mean_ang = np.mean(np.abs(ws.u_ang), axis=1)
    lin_dead = (
        (mean_lin > NONRESP_MIN_CMD)
        & (pred_disp > NONRESP_MIN_PRED_M)
        & (obs_disp < NONRESP_RATIO * pred_disp)
    )
    ang_dead = (
        (mean_ang > NONRESP_MIN_CMD)
        & (pred_rot > NONRESP_MIN_PRED_RAD)
        & (obs_rot < NONRESP_RATIO * pred_rot)
    )
    return lin_dead | ang_dead


def merge_spans(stamps_ns: np.ndarray, span_ns: int) -> list[tuple[int, int]]:
    """Union of [stamp, stamp + span] intervals, merged when they touch."""
    if len(stamps_ns) == 0:
        return []
    out: list[tuple[int, int]] = []
    for start in np.sort(stamps_ns):
        end = int(start) + span_ns
        if out and int(start) <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], end))
        else:
            out.append((int(start), end))
    return out


def noise_floor(
    mw: MatchWindows,
    *,
    cmd_tol: float = 0.06,
    v_tol: float = 0.30,
    w_tol: float = 2.0,
    max_windows: int = 1500,
) -> dict[str, np.ndarray] | None:
    """Repeatability floor from matched-command window pairs.

    The driver repeats spins, charges, and reversals constantly, so pairs of
    non-overlapping windows with near-identical command tapes and initial
    velocity exist in bulk. How differently the robot answers the same command
    is the floor no model can beat; every gate is read relative to it.
    """
    ws = mw.windows
    n = ws.count()
    pick = np.arange(n)
    if n > max_windows:
        pick = np.linspace(0, n - 1, max_windows).astype(int)
    u = np.concatenate([ws.u_lin[pick], ws.u_ang[pick]], axis=1)
    v0 = np.asarray(ws.state0.v, dtype=float)[pick]
    w0 = np.asarray(ws.state0.w, dtype=float)[pick]
    starts = ws.starts[pick]
    runs = mw.run_index[pick]

    cols = ws.steps - 1
    th0 = np.asarray(ws.state0.theta, dtype=float)[pick, None]
    dx = ws.truth["x"][pick][:, cols] - np.asarray(ws.state0.x, dtype=float)[pick, None]
    dy = ws.truth["y"][pick][:, cols] - np.asarray(ws.state0.y, dtype=float)[pick, None]
    along = dx * np.cos(th0) + dy * np.sin(th0)
    cross = -dx * np.sin(th0) + dy * np.cos(th0)
    dth = wrap_angle(ws.truth["theta"][pick][:, cols] - th0)

    span = int(ws.steps.max())
    pos_sq: list[np.ndarray] = []
    head_sq: list[np.ndarray] = []
    count = 0
    block = 256
    m = len(pick)
    for i0 in range(0, m, block):
        i1 = min(i0 + block, m)
        du = np.sqrt(np.mean((u[i0:i1, None, :] - u[None, :, :]) ** 2, axis=2))  # (b, m)
        okay = (
            (du < cmd_tol)
            & (np.abs(v0[i0:i1, None] - v0[None, :]) < v_tol)
            & (np.abs(w0[i0:i1, None] - w0[None, :]) < w_tol)
        )
        # Non-overlapping only: overlapping windows share truth samples and
        # would report a floor of zero.
        sep = (np.abs(starts[i0:i1, None] - starts[None, :]) > span) | (
            runs[i0:i1, None] != runs[None, :]
        )
        okay &= sep
        # Upper triangle in global index so each pair counts once.
        okay &= (np.arange(i0, i1)[:, None]) < np.arange(m)[None, :]
        ii, jj = np.nonzero(okay)
        if len(ii) == 0:
            continue
        ii = ii + i0
        pos_sq.append((along[ii] - along[jj]) ** 2 + (cross[ii] - cross[jj]) ** 2)
        head_sq.append(wrap_angle(dth[ii] - dth[jj]) ** 2)
        count += len(ii)
    if count == 0:
        return None
    pos = np.concatenate(pos_sq, axis=0)
    head = np.concatenate(head_sq, axis=0)
    # A pair difference has twice the single-window variance.
    return {
        "pairs": np.array([count]),
        "pos_rmse_mm": np.sqrt(np.mean(pos, axis=0) / 2.0) * 1e3,
        "head_rmse_deg": np.degrees(np.sqrt(np.mean(head, axis=0) / 2.0)),
        "horizons": ws.horizons,
    }


# ---------------------------------------------------------------------------
# Reporting helpers
# ---------------------------------------------------------------------------


def report_row(report: HorizonReport, name: str, horizon: float) -> float:
    idx = int(np.argmin(np.abs(report.horizons - horizon)))
    return float(report.rows[name][idx])


def class_table(
    err: WindowErrors, classes: np.ndarray, horizon: float
) -> dict[str, tuple[int, float, float]]:
    """Per-maneuver-class (count, pos RMSE mm, heading RMSE deg) at one horizon."""
    h = int(np.argmin(np.abs(err.horizons - horizon)))
    out: dict[str, tuple[int, float, float]] = {}
    for label in sorted(set(classes.tolist())):
        mask = classes == label
        if not mask.any():
            continue
        pos = err.position[mask, h] * 1e3
        head = np.degrees(err.heading[mask, h])
        out[label] = (
            int(mask.sum()),
            float(np.sqrt(np.mean(pos**2))),
            float(np.sqrt(np.mean(head**2))),
        )
    return out


def gain_drift(mw: MatchWindows, params: PlantParams, structure: ModelStructure) -> dict:
    """Observed-over-predicted displacement ratio against time into the recording.

    Battery state is not in the recordings, so voltage sag is absorbed into the
    fitted gains; a negative slope here is the evidence base for promoting
    battery telemetry to the diagnostics stream.
    """
    motion = observed_and_predicted(mw, params, structure)
    ws = mw.windows
    h = int(np.argmin(np.abs(ws.horizons - 0.400)))
    pred = np.hypot(motion["pred_along"][:, h], motion["pred_cross"][:, h])
    obs = np.hypot(motion["obs_along"][:, h], motion["obs_cross"][:, h])
    keep = pred > 0.05
    if keep.sum() < 20:
        return {}
    t = np.array([mw.runs[r].t[int(s)] for r, s in zip(mw.run_index[keep], ws.starts[keep])])
    ratio = obs[keep] / pred[keep]
    slope = float(np.polyfit(t, ratio, 1)[0])
    order = np.argsort(t)
    third = max(1, len(order) // 3)
    return {
        "slope_per_min": slope * 60.0,
        "first_third_median": float(np.median(ratio[order[:third]])),
        "last_third_median": float(np.median(ratio[order[-third:]])),
        "n": int(keep.sum()),
    }


def write_match_params(
    path: Path,
    params: PlantParams,
    structure: ModelStructure,
    bounds: dict[str, tuple[float, float]],
    delay_note: str,
    note: str,
) -> None:
    params = structure.apply(params)
    from auto_battlebot.plant import FULL_MODEL

    disabled = sorted(set(FULL_MODEL.free_names()) - set(structure.free_names()))
    lines = [
        "# Drivetrain plant parameters fit from NHRL match driving.",
        f"# {note}",
        "# Consumed by the C++ filter and simulation/kinematic_sim_server.py. One source of",
        "# truth for the plant numbers: edit the fit, not this file.",
        f"# Model {structure.name}. Terms it disables are written as zero"
        + (f": {', '.join(disabled)}." if disabled else "; there are none."),
        "",
        params.to_toml("plant").rstrip(),
        "",
        "[plant.provenance]",
    ]
    for name, value in params.to_dict().items():
        if name in PINNED_DEADZONES:
            lines.append(f'{name} = "held at the jig bracket midpoint (not fit by match data)"')
        elif name == "delay_s":
            lines.append(f'{name} = "{delay_note}"')
        elif name in bounds:
            lo, hi = bounds[name]
            at_edge = ""
            if abs(value - lo) < 1e-6 or abs(value - hi) < 1e-6:
                at_edge = "; AT BOUND, match data wanted to move further"
            kind = "wide bounds" if name.startswith("c_") else "jig 3-sigma bounds"
            lines.append(
                f'{name} = "{toml_float(value)} inside {kind} [{lo:.4g}, {hi:.4g}]{at_edge}"'
            )
        else:
            lines.append(f'{name} = "{toml_float(value)} fit by match windows (wide bounds)"')
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nwrote {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def parse_pair(text: str) -> tuple[Path, Path]:
    replay, _, original = text.partition(":")
    if not original:
        raise argparse.ArgumentTypeError("expected <replay.mcap>:<original.mcap>")
    return Path(replay), Path(original)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit", type=parse_pair, required=True, metavar="REPLAY:ORIGINAL")
    parser.add_argument(
        "--validate", type=parse_pair, action="append", default=[], metavar="REPLAY:ORIGINAL"
    )
    parser.add_argument(
        "--seed",
        type=Path,
        default=Path("playground/calibration/out/plant_stageA.toml"),
    )
    parser.add_argument("--sim-plant", type=Path, default=Path("simulation/sim_mrs_buff_mk3.toml"))
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--report-dir", type=Path, default=None)
    parser.add_argument("--widen", type=float, default=3.0, help="jig error bar multiplier")
    parser.add_argument("--stride-ms", type=float, default=50.0)
    parser.add_argument("--max-windows", type=int, default=4000)
    parser.add_argument("--delay-windows", type=int, default=1200)
    parser.add_argument("--holdout-fraction", type=float, default=0.30)
    parser.add_argument("--max-nfev", type=int, default=100)
    parser.add_argument(
        "--cmd-lead-ms",
        type=float,
        default=0.0,
        help="shift command timestamps this much earlier before fitting. The channel readback"
        " reaches the log tens of ms after the RF frame carrying the same values left the"
        " radio, so the effective transport delay on the logged timeline can be negative;"
        " a lead moves the delay profile minimum back inside the searchable range."
        " Effective delay on the logged timeline = fitted delay minus this lead.",
    )
    args = parser.parse_args()

    stride_s = args.stride_ms / 1000.0
    weights = FitWeights()

    seed, bounds = load_seed(args.seed, args.widen)
    print("seed (jig stage A) and match-fit bounds:")
    for name, (lo, hi) in bounds.items():
        print(f"  {name:10} {getattr(seed, name):8.4g}  bounds [{lo:.4g}, {hi:.4g}]")
    sim_params = load_sim_plant(args.sim_plant)

    # --- load and join ---
    def load_run(replay: Path, original: Path, role: str) -> MatchRun:
        name = original.stem.replace("auto_battlebot_main_", "").replace("_repaired", "")
        print(f"\nloading {name} ({role})")
        track = load_replay_track(replay)
        commands = load_commands(original)
        if args.cmd_lead_ms:
            commands.t_ns = commands.t_ns - int(args.cmd_lead_ms * 1e6)
        run = build_match_run(track, commands, name=name, role=role)
        n = len(run.t)
        print(
            f"  {n} grid slots at dt {run.dt * 1000:.2f} ms, "
            f"live {np.count_nonzero(run.valid | run.contact)} "
            f"auto {np.count_nonzero(run.auto)} contact {np.count_nonzero(run.contact)} "
            f"valid {np.count_nonzero(run.valid)} "
            f"(rebase offset std {track.rebase_offset_std_ns:.0f} ns)"
        )
        return run

    fit_run = load_run(*args.fit, role="fit")
    train_run, holdout_run = split_run(fit_run, 1.0 - args.holdout_fraction)
    validation_runs = [
        load_run(replay, original, role="validation") for replay, original in args.validate
    ]

    # --- ladder ---
    print("\n=== Model ladder (match windows, stage A seed) ===")
    delay_grid = np.arange(0.010, 0.1101, 0.008)
    results = []
    for rung in MODEL_LADDER:
        structure = pinned_structure(rung)
        start = structure.apply(seed)
        if structure.use_delay:
            print(f"  {structure.name}: profiling delay")
            delay, _costs = profile_delay_match(
                [train_run],
                start,
                structure,
                weights,
                delay_grid,
                stride_s=stride_s,
                max_windows=args.delay_windows,
                max_nfev=25,
                bounds=bounds,
                drop_spans=[],
            )
        else:
            delay = 0.0
        start = start.replace(delay_s=delay)
        train_w = build_match_windows([train_run], delay, HORIZONS, stride_s, args.max_windows)
        holdout_w = build_match_windows([holdout_run], delay, HORIZONS, stride_s, args.max_windows)
        if train_w is None or holdout_w is None:
            print(f"  {structure.name}: no usable windows")
            continue
        fitted, cost = joint_fit(
            train_w.windows, start, structure, weights, max_nfev=args.max_nfev, bounds=bounds
        )
        fitted = fitted.replace(delay_s=delay)
        holdout_err = predict_windows(holdout_w.windows, fitted, structure)
        metric = holdout_metric(holdout_err, weights)
        results.append((structure, fitted, delay, metric))
        print(
            f"  {structure.name}: delay {delay * 1000:5.1f} ms  train cost {cost:8.2f}  "
            f"holdout metric {metric:.4f}  "
            f"holdout pos@400 {report_row(score(holdout_err), 'pos_rmse_mm', 0.4):6.1f} mm  "
            f"({train_w.windows.count()} train / {holdout_w.windows.count()} holdout windows)"
        )

    if not results:
        raise SystemExit("no rung produced windows; nothing to fit")
    metrics = [m for _s, _p, _d, m in results]
    best_i = int(np.argmin(metrics))
    # Prefer the lowest rung within 1% of the best: extra terms must earn their place.
    for i in range(best_i):
        if metrics[i] <= metrics[best_i] * 1.01:
            best_i = i
            break
    structure, fitted, delay, _ = results[best_i]
    print(f"\nselected rung {structure.name} (holdout metric {metrics[best_i]:.4f})")

    # --- radio-dropout gate ---
    # Each recording is flagged on its own window set: dt comes from the
    # recording's frame cadence, so window sets from different recordings
    # cannot be concatenated.
    print("\n=== Radio-dropout gate ===")
    spans: list[tuple[int, int]] = []
    csv_rows: list[list[str | int]] = []
    total_flagged = 0
    gate_groups: list[list[MatchRun]] = [[train_run, holdout_run]] + [
        [run] for run in validation_runs
    ]
    for group in gate_groups:
        group_w = build_match_windows(group, delay, HORIZONS, stride_s, 0)
        if group_w is None:
            continue
        flagged = flag_nonresponsive(group_w, fitted, structure)
        total_flagged += int(flagged.sum())
        stamps = group_w.start_stamps_ns()
        span_ns = int(float(group_w.windows.horizons.max()) * 1e9)
        spans += merge_spans(stamps[flagged], span_ns)
        classes_all = group_w.maneuver_classes()
        for i in np.flatnonzero(flagged):
            run = group_w.runs[group_w.run_index[i]]
            start = int(group_w.windows.starts[i])
            csv_rows.append(
                [
                    run.name,
                    int(stamps[i]),
                    int(stamps[i]) + span_ns,
                    f"{run.t[start]:.2f}",
                    f"{span_ns / 1e9:.2f}",
                    f"{np.mean(np.abs(group_w.windows.u_lin[i])):.3f}",
                    f"{np.mean(np.abs(group_w.windows.u_ang[i])):.3f}",
                    classes_all[i],
                ]
            )
    print(f"  flagged {total_flagged} windows -> {len(spans)} merged spans")

    report_dir = args.report_dir
    if report_dir is not None:
        report_dir.mkdir(parents=True, exist_ok=True)
        with open(report_dir / "nonresponsive_windows.csv", "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "recording",
                    "start_stamp_ns",
                    "end_stamp_ns",
                    "t_into_recording_s",
                    "duration_s",
                    "mean_abs_u_lin",
                    "mean_abs_u_ang",
                    "maneuver",
                ]
            )
            writer.writerows(csv_rows)
        print(f"  wrote {report_dir / 'nonresponsive_windows.csv'}")

    # --- final fit without flagged spans ---
    print("\n=== Final fit (flagged spans excluded) ===")
    train_w = build_match_windows([train_run], delay, HORIZONS, stride_s, args.max_windows, spans)
    assert train_w is not None
    final_params, cost = joint_fit(
        train_w.windows,
        fitted,
        structure,
        weights,
        max_nfev=args.max_nfev,
        bounds=bounds,
    )
    final_params = final_params.replace(delay_s=delay)
    print(f"  {train_w.windows.count()} windows, final cost {cost:.2f}")
    for name, value in final_params.to_dict().items():
        print(f"  {name:12} {value:.6g}")

    # --- three-plant scoring on identical windows ---
    # Windows are identical in starts and truth; each plant's command tape is
    # shifted by that plant's own delay, because the delay lives in the window
    # command matrices, not in the simulator. The jig delay is measured on the
    # send timeline while these commands sit on the readback-log timeline
    # (which lags the RF frame by tens of ms), so the seed is also scored with
    # the match-profiled delay to separate the timeline artifact from the
    # physical gains and time constants.
    plants = {
        "A_jig_seed": (seed, pinned_structure(MODEL_LADDER[4])),  # M4, the jig model
        "A_seed_at_fit_delay": (seed.replace(delay_s=delay), pinned_structure(MODEL_LADDER[4])),
        "B_match_fit": (final_params, structure),
        "C_sim_values": (sim_params, pinned_structure(MODEL_LADDER[4])),
    }
    summary: dict = {"rung": structure.name, "delay_ms": delay * 1000, "gates": {}}
    score_runs: dict[str, list[MatchRun]] = {"holdout(17-42-20)": [holdout_run]}
    for run in validation_runs:
        score_runs[run.name] = [run]
    score_sets: dict[str, MatchWindows] = {}
    for set_name, runs_for_set in list(score_runs.items()):
        scored = build_match_windows(runs_for_set, delay, HORIZONS, stride_s, 0, spans)
        if scored is not None:
            score_sets[set_name] = scored
        else:
            del score_runs[set_name]

    all_reports: dict[str, dict[str, HorizonReport]] = {}
    all_classes: dict[str, dict[str, dict]] = {}
    for set_name, mw in score_sets.items():
        print(f"\n=== Scoring on {set_name}: {mw.windows.count()} windows ===")
        classes = mw.maneuver_classes()
        all_reports[set_name] = {}
        all_classes[set_name] = {}
        per_delay_cache: dict[float, MatchWindows] = {delay: mw}
        for plant_name, (params, plant_structure) in plants.items():
            plant_delay = plant_structure.apply(params).delay_s
            if plant_delay not in per_delay_cache:
                built = build_match_windows(
                    score_runs[set_name], plant_delay, HORIZONS, stride_s, 0, spans
                )
                assert built is not None  # same valid mask as mw, so same starts
                per_delay_cache[plant_delay] = built
            err = predict_windows(per_delay_cache[plant_delay].windows, params, plant_structure)
            rep = score(err)
            all_reports[set_name][plant_name] = rep
            all_classes[set_name][plant_name] = class_table(err, classes, 0.400)
            print_horizon_table(f"{plant_name} on {set_name}", rep)
        floor = noise_floor(mw)
        if floor is not None:
            print(
                f"\n  noise floor ({int(floor['pairs'][0])} matched pairs): pos mm "
                + " ".join(f"{v:7.1f}" for v in floor["pos_rmse_mm"])
                + " | head deg "
                + " ".join(f"{v:5.2f}" for v in floor["head_rmse_deg"])
            )
            summary.setdefault("noise_floor", {})[set_name] = {
                "pairs": int(floor["pairs"][0]),
                "pos_rmse_mm": floor["pos_rmse_mm"].tolist(),
                "head_rmse_deg": floor["head_rmse_deg"].tolist(),
            }

    # --- gates ---
    print("\n=== Gates ===")
    for set_name, reports in all_reports.items():
        a = report_row(reports["A_jig_seed"], "pos_rmse_mm", 0.4)
        b = report_row(reports["B_match_fit"], "pos_rmse_mm", 0.4)
        ah = report_row(reports["A_jig_seed"], "head_rmse_deg", 0.4)
        bh = report_row(reports["B_match_fit"], "head_rmse_deg", 0.4)
        verdict = "PASS" if (b < a and bh < ah * 1.2) or (bh < ah and b < a * 1.2) else "CHECK"
        print(
            f"  {set_name}: A pos@400 {a:.1f} mm head {ah:.2f} deg | "
            f"B pos@400 {b:.1f} mm head {bh:.2f} deg -> {verdict}"
        )
        summary["gates"][set_name] = {
            "A_pos400_mm": a,
            "B_pos400_mm": b,
            "A_head400_deg": ah,
            "B_head400_deg": bh,
        }
        # Per-class regression check (transfer gate: no class regresses > 20%).
        for label, (n, b_pos, _bh) in all_classes[set_name]["B_match_fit"].items():
            a_entry = all_classes[set_name]["A_jig_seed"].get(label)
            if a_entry is None or n < 10:
                continue
            _an, a_pos, _ah2 = a_entry
            if b_pos > 1.2 * a_pos:
                print(f"    class {label} regressed: B {b_pos:.1f} mm vs A {a_pos:.1f} mm (n={n})")

    # --- gain drift over each recording ---
    print("\n=== Gain drift over recording (obs/pred displacement at 400 ms) ===")
    for set_name, mw in score_sets.items():
        drift = gain_drift(mw, final_params, structure)
        if drift:
            print(
                f"  {set_name}: slope {drift['slope_per_min']:+.3f}/min, "
                f"first third {drift['first_third_median']:.3f} -> "
                f"last third {drift['last_third_median']:.3f} (n={drift['n']})"
            )
            summary.setdefault("gain_drift", {})[set_name] = drift

    if report_dir is not None:
        summary["params"] = final_params.to_dict()
        summary["bounds"] = {k: list(v) for k, v in bounds.items()}
        summary["class_tables"] = {
            s: {p: {c: list(v) for c, v in t.items()} for p, t in per_plant.items()}
            for s, per_plant in all_classes.items()
        }
        summary["horizon_tables"] = {
            s: {
                p: {"horizons": r.horizons.tolist(), **{k: v.tolist() for k, v in r.rows.items()}}
                for p, r in per_plant.items()
            }
            for s, per_plant in all_reports.items()
        }
        with open(report_dir / "match_fit_summary.json", "w") as handle:
            json.dump(summary, handle, indent=1)
        print(f"wrote {report_dir / 'match_fit_summary.json'}")

    if args.out is not None:
        delay_note = (
            f"{final_params.delay_s * 1000:.1f} ms from the match delay profile; includes the "
            "constant command-readback-to-camera-stamp offset, so it is comparable to the jig "
            "value only through the same readback path"
        )
        note = "fit from NHRL 2026-05 match driving; see match_plant_fit_report.md"
        write_match_params(args.out, final_params, structure, bounds, delay_note, note)


if __name__ == "__main__":
    main()
