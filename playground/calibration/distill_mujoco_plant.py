"""Distill a fitted MuJoCo plant into the app's grey-box [plant] table for Mr Stabs Mk2.

The app's EKF and MotionProfileNavigation both run JigPlantModel: first-order lag, deadzones,
delay, and steer-brake coupling, with stick commands in. The MuJoCo fit (fit_mujoco_plant.py) has
motor, contact, and inertia terms the app cannot run. This drives the fitted MuJoCo model through
ClosedLoopSim with stick tapes shaped like the velocity jig protocol (deadzone staircases, steps
with coasts, reversals, a steer/brake grid), fits the M4 grey-box model to the result the way
fit_jig_plant.py fits jig data, and scores it on random stick tapes the fit never saw.

Stick tapes go into ClosedLoopSim, which runs the firmware mixer and auto-steer, the drivetrain
delay, and command shaping. That matches what the app's plant is fed: CommandFeedback is the
transmitter's stick readback, after its slew limit and deadzone. The radio link delay is not in
ClosedLoopSim, so it comes from the sessions' clock alignment and is added to the fitted delay
afterwards. A pure shift adds exactly.

Samples after the chassis pitches more than --pitch-gate-deg off its rest pitch are dropped, and
stay dropped for the rest of that tape. The grey-box model is flat and cannot represent a wheelie
or a flip, so the table only covers the command range the robot stays down in. The report lists
which tapes lost samples.

Every restart within --loss-ratio of the best MuJoCo loss is distilled too. The spread across those
tables is the parameter spread the plan hands to the learned sim, and shows which grey-box terms
the MuJoCo data pins.

    python playground/calibration/distill_mujoco_plant.py \\
        playground/calibration/out/mujoco_sysid/fit_<ts> \\
        --holdout playground/calibration/out/mujoco_sysid/s04

Writes <fit_dir>/distilled/plant.toml (the best run's [plant] table, radio delay included) and
distill.json (every run's table, sim and session scores, spread, masked tapes).
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from auto_battlebot.calibration.jig.jig_fit import FitWeights, joint_fit, score
from auto_battlebot.control import plant
from auto_battlebot.control.plant import (
    MODEL_LADDER,
    PARAM_BOUNDS,
    WindowSet,
    concat_windows,
    make_windows,
    predict_windows,
)
from auto_battlebot.mujoco_sim import greybox, mass_properties
from auto_battlebot.mujoco_sim.actuator import NOMINAL_PACK_V, MotorConstants, shape_command
from auto_battlebot.mujoco_sim.actuator import PlantParams as MujocoParams
from auto_battlebot.mujoco_sim.closed_loop import ClosedLoopSim
from auto_battlebot.mujoco_sim.fit import window_errors
from auto_battlebot.mujoco_sim.mjcf import CollisionSet
from auto_battlebot.mujoco_sim.session import build_batch, load_session

REPO_ROOT = Path(__file__).resolve().parents[2]
# The table the app runs today, scored alongside so the report shows what the swap buys.
CURRENT_TABLE = REPO_ROOT / "config" / "mr_stabs_mk2_zed_box.toml"

RECORD_DT = 0.005  # stick and state grid, the jig fit's 200 Hz
REST_S = 0.3
# The jig fit's horizons: C1 is stated at 100 ms, C2 and C3 at 400 ms.
HORIZONS = (0.033, 0.066, 0.100, 0.200, 0.300, 0.400, 0.500)
STRIDE_S = 0.05
STRUCTURE = next(m for m in MODEL_LADDER if m.name == "M4")
# Mrs Buff's jig fit landed c_sb at 2.70, past PARAM_BOUNDS' 1.5, so the bound is widened rather
# than left to decide the answer.
BOUNDS = {"c_sb": (-0.5, 4.0), "c_ad": (-0.5, 2.0)}
COARSE_DELAYS = np.arange(0.0, 0.1001, 0.01)
ACCEPT = {"C1_pos_100ms_mm": 15.0, "C2_pos_400ms_mm": 80.0, "C3_head_400ms_deg": 8.0}


# ---------------------------------------------------------------------------
# Stick tapes
# ---------------------------------------------------------------------------


@dataclass
class Tape:
    name: str
    kind: str  # staircase | step | reversal | grid | random
    lin: np.ndarray  # stick held over [k dt, (k + 1) dt)
    ang: np.ndarray


def _tape(name: str, kind: str, segments: list[tuple[float, float, float]]) -> Tape:
    """segments: (linear, angular, seconds), with a rest before and after."""
    segments = [(0.0, 0.0, REST_S), *segments, (0.0, 0.0, 0.6)]
    lin = np.concatenate([np.full(round(s / RECORD_DT), a) for a, _, s in segments])
    ang = np.concatenate([np.full(round(s / RECORD_DT), b) for _, b, s in segments])
    return Tape(name, kind, lin, ang)


def train_tapes() -> list[Tape]:
    tapes: list[Tape] = []
    levels = np.round(np.arange(0.01, 0.1201, 0.01), 3)
    for sign, tag in ((1.0, "pos"), (-1.0, "neg")):
        tapes.append(_tape(f"stair_lin_{tag}", "staircase", [(sign * u, 0.0, 0.4) for u in levels]))
        tapes.append(_tape(f"stair_ang_{tag}", "staircase", [(0.0, sign * u, 0.4) for u in levels]))
        for u in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.85, 1.0):
            tapes.append(_tape(f"step_lin_{sign * u:+.2f}", "step", [(sign * u, 0.0, 0.8)]))
        for u in (0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0):
            tapes.append(_tape(f"step_ang_{sign * u:+.2f}", "step", [(0.0, sign * u, 0.6)]))
    for u in (0.3, 0.5):
        tapes.append(_tape(f"reversal_{u:.1f}", "reversal", [(u, 0.0, 0.6), (-u, 0.0, 0.6)]))
    for lin in (0.2, 0.35, 0.5):
        for ang in (0.1, 0.25, 0.5, -0.1, -0.25, -0.5):
            tapes.append(_tape(f"grid_{lin:.2f}_{ang:+.2f}", "grid", [(lin, ang, 0.8)]))
    return tapes


def random_tapes(count: int, seed: int, seconds: float = 6.0) -> list[Tape]:
    """Piecewise-constant sticks, holds 0.1 to 0.6 s. The holdout: no tape shape the fit saw."""
    rng = np.random.default_rng(seed)
    tapes = []
    for i in range(count):
        segments: list[tuple[float, float, float]] = []
        total = 0.0
        while total < seconds:
            hold = float(rng.uniform(0.1, 0.6))
            lin = 0.0 if rng.random() < 0.3 else float(rng.uniform(-0.6, 0.6))
            ang = 0.0 if rng.random() < 0.4 else float(rng.uniform(-0.6, 0.6))
            segments.append((lin, ang, hold))
            total += hold
        tapes.append(_tape(f"random_{i:02d}", "random", segments))
    return tapes


# ---------------------------------------------------------------------------
# MuJoCo rollouts
# ---------------------------------------------------------------------------


@dataclass
class Trace:
    name: str
    kind: str
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray  # unwrapped
    v: np.ndarray
    w: np.ndarray
    cmd_lin: np.ndarray  # cmd[k] is the stick held over the step that ends at t[k]
    cmd_ang: np.ndarray
    valid: np.ndarray

    @property
    def masked(self) -> float:
        return float(1.0 - self.valid.mean())


_MODEL: tuple[mass_properties.MassProperties, CollisionSet] | None = None


def _model() -> tuple[mass_properties.MassProperties, CollisionSet]:
    global _MODEL
    if _MODEL is None:
        _MODEL = (mass_properties.load(), CollisionSet.load())
    return _MODEL


def simulate_tape(
    params: MujocoParams, tape: Tape, pack_voltage: float, auto_steer: bool, pitch_gate: float
) -> Trace:
    mp, collision = _model()
    sim = ClosedLoopSim(
        mp, collision, params, start=(0.0, 0.0, 0.0), pack_voltage=pack_voltage,
        auto_steer=auto_steer,
    )  # fmt: skip
    n = len(tape.lin)
    state = np.empty((n + 1, 6))

    def sample(k: int) -> None:
        x, y, yaw = sim.pose()
        state[k] = (x, y, yaw, sim.forward_speed, sim.yaw_rate, sim.pitch)

    sample(0)
    for k in range(n):
        sim.step(float(tape.lin[k]), float(tape.ang[k]), RECORD_DT)
        sample(k + 1)
    tilted = np.abs(state[:, 5] - collision.rest_pitch_rad) > pitch_gate
    # Latched: once the robot has pitched up it lands somewhere the flat model never predicted.
    valid = np.cumsum(tilted) == 0
    return Trace(
        tape.name,
        tape.kind,
        np.arange(n + 1) * RECORD_DT,
        state[:, 0],
        state[:, 1],
        np.unwrap(state[:, 2]),
        state[:, 3],
        state[:, 4],
        np.concatenate([[0.0], tape.lin]),
        np.concatenate([[0.0], tape.ang]),
        valid,
    )


# ---------------------------------------------------------------------------
# Grey-box fit
# ---------------------------------------------------------------------------


def build_windows(traces: list[Trace], delay_s: float, limit: int) -> WindowSet | None:
    sets = []
    for i, tr in enumerate(traces):
        ws = make_windows(
            tr.t, tr.v, tr.w, tr.yaw, tr.x, tr.y, tr.cmd_lin, tr.cmd_ang,
            dt=RECORD_DT, delay_s=delay_s, horizons=HORIZONS, stride_s=STRIDE_S,
            valid=tr.valid, origin=i,
        )  # fmt: skip
        if ws is not None:
            sets.append(ws)
    return concat_windows(sets).subsample(limit) if sets else None


def seed_table(params: MujocoParams, pack_voltage: float) -> plant.PlantParams:
    """Analytic starting point: free speed for the gains, deadzones straight across."""
    mp, _ = _model()
    full = float(shape_command(np.array([1.0]), 0.0, params.curve)[0])
    k_fwd = pack_voltage * full * mp.wheel_radius / (mp.gear_ratio * mp.motor_kt)
    dz = 0.5 * (params.deadzone_left + params.deadzone_right)
    return plant.PlantParams(
        dz_lin_fwd=dz, dz_lin_rev=dz, dz_ang_l=dz, dz_ang_r=dz,
        k_fwd=k_fwd, k_rev=k_fwd, k_ang=min(k_fwd / mp.track_half_width, PARAM_BOUNDS["k_ang"][1]),
        tau_lin_a=0.08, tau_lin_d=0.08, tau_ang_a=0.08, tau_ang_d=0.08,
        delay_s=params.delay_s, c_sb=0.0, c_ad=0.0, c_drift=0.0, c_drift_bias=0.0,
    )  # fmt: skip


def fit_table(
    traces: list[Trace], start: plant.PlantParams, delay_windows: int, max_windows: int
) -> tuple[plant.PlantParams, dict[str, list[float]]]:
    """Profile the delay (coarse 10 ms, then 2 ms around the minimum), then a full joint fit."""
    weights = FitWeights()
    profile: dict[str, list[float]] = {"delay_s": [], "cost": []}
    best: tuple[float, float, plant.PlantParams] = (math.inf, 0.0, start)

    def probe(delay: float) -> None:
        nonlocal best
        ws = build_windows(traces, delay, delay_windows)
        if ws is None:
            return
        params, cost = joint_fit(
            ws, best[2].replace(delay_s=delay), STRUCTURE, weights, max_nfev=30, bounds=BOUNDS
        )
        profile["delay_s"].append(float(delay))
        profile["cost"].append(float(cost))
        if cost < best[0]:
            best = (cost, delay, params.replace(delay_s=delay))

    for d in COARSE_DELAYS:
        probe(float(d))
    centre = best[1]
    for d in np.arange(max(0.0, centre - 0.008), centre + 0.0081, 0.002):
        if not np.any(np.isclose(profile["delay_s"], d)):
            probe(float(d))
    ws = build_windows(traces, best[1], max_windows)
    if ws is None:
        raise RuntimeError("no usable windows: every tape was masked")
    params, _ = joint_fit(ws, best[2], STRUCTURE, weights, bounds=BOUNDS)
    order = np.argsort(profile["delay_s"])
    profile = {k: [v[i] for i in order] for k, v in profile.items()}
    return STRUCTURE.apply(params.replace(delay_s=best[1])), profile


def score_sim(traces: list[Trace], table: plant.PlantParams) -> dict[str, Any]:
    ws = build_windows(traces, table.delay_s, 100_000)
    if ws is None:
        return {"windows": 0}
    rows = score(predict_windows(ws, table, STRUCTURE)).rows
    h = np.asarray(HORIZONS)
    i100, i400 = int(np.argmin(np.abs(h - 0.1))), int(np.argmin(np.abs(h - 0.4)))
    return {
        "windows": ws.count(),
        "horizons_s": list(HORIZONS),
        "pos_rmse_mm": rows["pos_rmse_mm"].tolist(),
        "pos_p95_mm": rows["pos_p95_mm"].tolist(),
        "head_rmse_deg": rows["head_rmse_deg"].tolist(),
        "C1_pos_100ms_mm": float(rows["pos_rmse_mm"][i100]),
        "C2_pos_400ms_mm": float(rows["pos_rmse_mm"][i400]),
        "C3_head_400ms_deg": float(rows["head_rmse_deg"][i400]),
    }


def analytic(params: MujocoParams, pack_voltage: float) -> dict[str, float]:
    """Closed-form straight-line numbers, no friction: the sanity check on the fitted table."""
    mp, _ = _model()
    motor = MotorConstants(mp.gear_ratio, mp.motor_kt)
    full = float(shape_command(np.array([1.0]), 0.0, params.curve)[0])
    b = -motor.bias(params)  # wheel N m per rad/s
    inertia = (
        mp.total_mass * mp.wheel_radius**2 + 2.0 * params.armature_scale * mp.reflected_inertia
    )
    stall = motor.force_limit(params)  # wheel N m at the current limit
    return {
        "free_speed_m_s": motor.free_speed(pack_voltage * full) * mp.wheel_radius,
        "brake_tau_s": inertia / (2.0 * b),
        "current_limited_accel_m_s2": 2.0 * stall * mp.wheel_radius / inertia,
    }


def distill_run(job: dict[str, Any]) -> dict[str, Any]:
    """One MuJoCo parameter set in, one grey-box table and its scores out. Runs in a worker."""
    params = MujocoParams(**job["params"])
    sim_args = (params, job["pack_voltage"], job["auto_steer"], job["pitch_gate"])
    train = [simulate_tape(params, t, *sim_args[1:]) for t in train_tapes()]
    holdout = [simulate_tape(params, t, *sim_args[1:]) for t in random_tapes(job["holdout"], 1)]
    start = seed_table(params, job["pack_voltage"])
    table, profile = fit_table(train, start, job["delay_windows"], job["max_windows"])
    current = plant.PlantParams.from_toml(CURRENT_TABLE)
    return {
        "label": job["label"],
        "mujoco_loss": job["loss"],
        "table": table.to_dict(),
        "delay_profile": profile,
        "analytic": analytic(params, job["pack_voltage"]),
        "masked": {tr.name: tr.masked for tr in train + holdout if tr.masked > 0.0},
        "sim_train": score_sim(train, table),
        "sim_holdout": score_sim(holdout, table),
        "sim_holdout_current": score_sim(holdout, current),
    }


# ---------------------------------------------------------------------------
# Real sessions
# ---------------------------------------------------------------------------


def score_sessions(
    paths: list[Path], doc: dict[str, Any], tables: dict[str, plant.PlantParams]
) -> dict[str, Any]:
    """Every table on the held-out flat windows, through the same per-motor tape and inverse
    mixer the direct grey-box fit uses. Errors in multiples of each session's noise floor."""
    mp, _ = _model()
    batch = build_batch(
        [load_session(p) for p in paths],
        horizon_s=doc["horizon_s"],
        dt=doc["dt"],
        track_half_width=mp.track_half_width,
        wheel_radius=mp.wheel_radius,
    )
    flat = batch.take(np.nonzero(batch.kind == "flat")[0])
    out: dict[str, Any] = {"windows": len(flat), "horizons_s": list(flat.horizons)}
    for name, table in tables.items():
        qpos, record_dt = greybox.predict(flat, table)
        err = window_errors(flat, qpos, record_dt)
        pos = err.position / flat.noise_xy[:, None]
        yaw = err.heading / flat.noise_yaw[:, None]
        out[name] = {
            "pos_rms_x_noise": np.sqrt(np.nanmean(pos**2, axis=0)).tolist(),
            "head_rms_x_noise": np.sqrt(np.nanmean(yaw**2, axis=0)).tolist(),
            "pos_rmse_mm": (1e3 * np.sqrt(np.nanmean(err.position**2, axis=0))).tolist(),
        }
    return out


def session_stat(paths: list[Path], what: str) -> float | None:
    values = []
    for p in paths:
        s = load_session(p)
        if what == "radio":
            if s.radio_delay_s is not None:
                values.append(float(s.radio_delay_s))
        elif "vbat" in s.commands:
            v = s.commands["vbat"].to_numpy(float)
            v = v[np.isfinite(v) & (v > 5.0)]
            if len(v):
                values.append(float(np.median(v)))
    return float(np.median(values)) if values else None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _print_table(results: list[dict[str, Any]]) -> None:
    names = list(results[0]["table"])
    print(f"\n{'':14}" + "".join(f"{r['label']:>11}" for r in results))
    for n in names:
        print(f"  {n:12}" + "".join(f"{r['table'][n]:11.4g}" for r in results))


def _print_scores(title: str, rows: dict[str, dict[str, Any]]) -> None:
    print(f"\n{title}")
    print(f"  {'':22}{'C1 pos@100':>11}{'C2 pos@400':>11}{'C3 head@400':>12}")
    print(f"  {'target':22}{'<15 mm':>11}{'<80 mm':>11}{'<8 deg':>12}")
    for name, s in rows.items():
        if not s.get("windows"):
            print(f"  {name:22} no windows")
            continue
        print(
            f"  {name:22}{s['C1_pos_100ms_mm']:9.1f}mm{s['C2_pos_400ms_mm']:9.1f}mm"
            f"{s['C3_head_400ms_deg']:9.2f}deg"
        )


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("fit_dir", type=Path, help="fit_mujoco_plant.py output dir (holds fit.json)")
    ap.add_argument("--holdout", type=Path, nargs="*", default=[], help="held-out session bundles")
    ap.add_argument("--radio-delay-ms", type=float, default=None, help="default: from sessions")
    ap.add_argument("--pack-voltage", type=float, default=None, help="default: session vbat")
    ap.add_argument("--loss-ratio", type=float, default=1.25, help="restarts to distill")
    ap.add_argument("--max-runs", type=int, default=6)
    ap.add_argument("--pitch-gate-deg", type=float, default=5.0)
    ap.add_argument("--no-auto-steer", action="store_true", help="flip switch UP on the robot")
    ap.add_argument("--holdout-tapes", type=int, default=12)
    ap.add_argument("--delay-windows", type=int, default=1500)
    ap.add_argument("--max-windows", type=int, default=4000)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--out", type=Path, default=None, help="default: <fit_dir>/distilled")
    args = ap.parse_args()

    doc = json.loads((args.fit_dir / "fit.json").read_text())
    sessions = [Path(p) for p in doc["sessions"]] + list(args.holdout)

    radio = args.radio_delay_ms / 1000.0 if args.radio_delay_ms is not None else None
    if radio is None:
        radio = session_stat(sessions, "radio")
        if radio is None:
            raise SystemExit("no session has a radio delay; pass --radio-delay-ms")
    pack = args.pack_voltage or session_stat(sessions, "vbat") or NOMINAL_PACK_V
    print(f"radio delay {radio * 1e3:.1f} ms, pack {pack:.2f} V")

    runs = sorted(
        (r for r in doc["runs"] if math.isfinite(r["loss"])), key=lambda r: float(r["loss"])
    )
    if not runs:
        raise SystemExit(f"{args.fit_dir}: no MuJoCo run with a finite loss")
    keep = [r for r in runs if r["loss"] <= args.loss_ratio * runs[0]["loss"]][: args.max_runs]
    print(f"distilling {len(keep)} of {len(runs)} restarts (loss within {args.loss_ratio}x)")
    jobs = [
        {
            "label": "best" if i == 0 else f"{r['zero_mode']}_s{r['seed']}",
            "loss": float(r["loss"]),
            "params": r["best"],
            "pack_voltage": pack,
            "auto_steer": not args.no_auto_steer,
            "pitch_gate": math.radians(args.pitch_gate_deg),
            "holdout": args.holdout_tapes,
            "delay_windows": args.delay_windows,
            "max_windows": args.max_windows,
        }
        for i, r in enumerate(keep)
    ]
    # One BLAS thread per worker. Spawned rather than forked, so the workers load BLAS after
    # this is set; least_squares otherwise runs every worker's SVD on all cores at once.
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[var] = "1"
    with ProcessPoolExecutor(
        max_workers=max(1, min(args.jobs, len(jobs))),
        mp_context=multiprocessing.get_context("spawn"),
    ) as pool:
        results = list(pool.map(distill_run, jobs))

    best = results[0]
    table = plant.PlantParams.from_dict(best["table"])
    # The app's delay runs from stick readback to motion: radio link plus drivetrain.
    final = table.replace(delay_s=table.delay_s + radio)

    _print_table(results)
    a = best["analytic"]
    print(
        f"\nanalytic (best run, no friction): free speed {a['free_speed_m_s']:.2f} m/s,"
        f" brake tau {a['brake_tau_s'] * 1e3:.0f} ms,"
        f" current-limited accel {a['current_limited_accel_m_s2']:.1f} m/s^2"
    )
    if best["masked"]:
        print("\npitch-gated tapes (fraction of samples dropped), best run:")
        for name, frac in best["masked"].items():
            print(f"  {name:22} {frac:.0%}")
    _print_scores(
        "sim holdout (random stick tapes through MuJoCo), best run:",
        {
            "distilled": best["sim_holdout"],
            "current config": best["sim_holdout_current"],
            "distilled (train)": best["sim_train"],
        },
    )

    spread = {
        n: {
            "min": float(min(r["table"][n] for r in results)),
            "max": float(max(r["table"][n] for r in results)),
        }
        for n in best["table"]
    }
    report: dict[str, Any] = {
        "fit_dir": str(args.fit_dir),
        "radio_delay_s": radio,
        "pack_voltage": pack,
        "auto_steer": not args.no_auto_steer,
        "pitch_gate_deg": args.pitch_gate_deg,
        "plant": final.to_dict(),
        "runs": results,
        "spread": spread,
    }

    if args.holdout:
        tables = {"distilled": table, "current_config": plant.PlantParams.from_toml(CURRENT_TABLE)}
        if "greybox" in doc:
            tables["direct_greybox"] = plant.PlantParams.from_dict(doc["greybox"]["params"])
        real = score_sessions(list(args.holdout), doc, tables)
        report["sessions"] = real
        print(f"\nheld-out sessions, {real['windows']} flat windows, position RMS (mm) at", end="")
        print("".join(f" {h:.1f}s" for h in real["horizons_s"]))
        for name in tables:
            row = real[name]
            mm = " ".join(f"{v:6.1f}" for v in row["pos_rmse_mm"])
            nx = " ".join(f"{v:5.1f}x" for v in row["pos_rms_x_noise"])
            print(f"  {name:16} {mm}   ({nx} noise)")
        validation = args.fit_dir / "validation.json"
        if validation.exists():
            report["mujoco_validation"] = json.loads(validation.read_text()).get("check1_2_errors")

    out = args.out or args.fit_dir / "distilled"
    out.mkdir(parents=True, exist_ok=True)
    header = (
        f"# Distilled from {args.fit_dir} by distill_mujoco_plant.py.\n"
        f"# delay_s = {table.delay_s * 1e3:.1f} ms drivetrain (fit to MuJoCo)"
        f" + {radio * 1e3:.1f} ms radio (session clock alignment).\n"
        f"# Pack {pack:.2f} V, auto-steer {'on' if not args.no_auto_steer else 'off'}.\n"
    )
    (out / "plant.toml").write_text(header + final.to_toml("plant"))
    (out / "distill.json").write_text(json.dumps(report, indent=2, default=float))
    print(f"\nwrote {out / 'plant.toml'}")
    print(f"wrote {out / 'distill.json'}")
    bad = [k for k, lim in ACCEPT.items() if best["sim_holdout"].get(k, math.inf) > lim]
    if bad:
        print(f"sim holdout misses {', '.join(bad)}: the M4 form is not absorbing the MuJoCo model")


if __name__ == "__main__":
    main()
