"""Forward step test: measured nose lift per step, an armature refit, and the slew verdict."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from auto_battlebot.mujoco_sim import mass_properties
from auto_battlebot.mujoco_sim.actuator import PlantParams
from auto_battlebot.mujoco_sim.closed_loop import load_fit_params
from auto_battlebot.mujoco_sim.mass_properties import MassProperties
from auto_battlebot.mujoco_sim.mjcf import CollisionSet
from auto_battlebot.mujoco_sim.step_response import (
    StickTape,
    fit_armature,
    ideal_step,
    loosest_slew,
    sim_lift_deg,
    slew_sweep,
)
from auto_battlebot.rl.envs.warp_nav import DEFAULT_FIT

REPO = Path(__file__).resolve().parents[2]
TELEMETRY = "/robot/espnow_telemetry"
REST_TOL = 0.05
REST_S = 0.5
HOLD_MIN = 0.25
HOLD_S = 0.4
TURNING = 0.1
PRE_S = 0.3  # rest pitch is the median over this long before the step
POST_S = 0.5  # the window runs this long past the hold, to catch a late peak
MAX_WINDOW_S = 2.5
ARMATURE_GRID = tuple(float(x) for x in np.geomspace(0.3, 2.0, 15))
MAX_MOTOR_RPM = 1500.0
SLEW_RATES = (1.0, 1.33, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0, 16.0, 24.0, 0.0)


@dataclass
class Step:
    recording: str
    t_start_s: float  # from the recording's first telemetry sample
    size: float
    rise_s: float  # rest to first reaching 90% of size
    hold_s: float
    mean_abs_angular: float
    turning: bool
    measured_lift_deg: float
    rest_pitch_deg: float
    sim_lift_fit_deg: float = math.nan  # at the current fit's armature
    sim_lift_refit_deg: float = math.nan


def load_telemetry(path: Path) -> dict[str, np.ndarray]:
    from auto_battlebot.recording.mcap_io import iter_messages

    rows = []
    for _topic, stamp, message in iter_messages(path, [TELEMETRY]):
        r = json.loads(bytes(message))
        if None in (r["a_percent"], r["b_percent"], r["orientation_y"]):
            continue
        rows.append(
            (
                stamp * 1e-9,
                -r["a_percent"] / 100.0,
                -r["b_percent"] / 100.0,
                r["orientation_y"],
                bool(r["armed"]) and not bool(r["is_upside_down"]),
            )
        )
    if not rows:
        raise SystemExit(f"{path}: no {TELEMETRY} samples")
    a = np.asarray(rows, dtype=float)
    return {
        "t": a[:, 0] - a[0, 0],
        "linear": a[:, 1],
        "angular": a[:, 2],
        "pitch": a[:, 3],
        "ok": a[:, 4] > 0.5,
    }


def find_steps(tel: dict[str, np.ndarray], name: str) -> tuple[list[Step], list[StickTape]]:
    t, lin, ang, pitch, ok = (tel[k] for k in ("t", "linear", "angular", "pitch", "ok"))
    steps: list[Step] = []
    tapes: list[StickTape] = []
    rest_since: float | None = None
    i = 0
    while i < len(t):
        if not ok[i]:
            rest_since = None
            i += 1
            continue
        if abs(lin[i]) < REST_TOL:
            rest_since = t[i] if rest_since is None else rest_since
            i += 1
            continue
        if rest_since is None or t[i] - rest_since < REST_S or lin[i] < 0:
            rest_since = None
            i += 1
            continue
        start = i - 1  # the last rest sample
        j = i
        while j < len(t) and ok[j] and lin[j] > REST_TOL and t[j] - t[start] < MAX_WINDOW_S:
            j += 1
        held = np.flatnonzero(lin[i:j] >= HOLD_MIN) + i
        rest_since = None
        if len(held) == 0 or t[held[-1]] - t[held[0]] < HOLD_S:
            i = j
            continue
        size = float(np.median(lin[held]))
        rise = int(np.argmax(lin[i:j] >= 0.9 * size)) + i
        end = min(np.searchsorted(t, t[held[-1]] + POST_S), len(t) - 1)
        pre = (t >= t[start] - PRE_S) & (t <= t[start])
        rest_pitch = float(np.median(pitch[pre]))
        lift = float(rest_pitch - np.min(pitch[start : end + 1]))
        mean_ang = float(np.mean(np.abs(ang[held])))
        steps.append(
            Step(
                recording=name,
                t_start_s=float(t[start]),
                size=size,
                rise_s=float(t[rise] - t[start]),
                hold_s=float(t[held[-1]] - t[held[0]]),
                mean_abs_angular=mean_ang,
                turning=mean_ang > TURNING,
                measured_lift_deg=max(lift, 0.0),
                rest_pitch_deg=rest_pitch,
            )
        )
        rel = t[start : end + 1] - t[start]
        tapes.append(
            StickTape.resample(rel, lin[start : end + 1], ang[start : end + 1], float(rel[-1]))
        )
        i = j
    return steps, tapes


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("recordings", type=Path, nargs="*")
    ap.add_argument("--ideal", type=float, nargs="*", default=[], help="ideal steps, sim only")
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--fit", type=Path, default=REPO / DEFAULT_FIT)
    ap.add_argument("--lift-limit-deg", type=float, default=30.0)
    ap.add_argument("--verdict-size", type=float, default=0.6)
    ap.add_argument("--full-step", type=float, default=1.0, help="step the slew sweep must hold")
    args = ap.parse_args()

    asset = mass_properties.asset_dir("mr_stabs_mk2")
    mp = mass_properties.load(asset / "mass_properties.toml")
    collision = CollisionSet.load(asset / "collision")
    params = load_fit_params(args.fit)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    report: dict = {"fit": str(args.fit), "fit_armature_scale": params.armature_scale}

    if args.ideal:
        rows = {
            f"{s:.2f}": {
                f"{a:.3f}": sim_lift_deg(
                    mp, collision, params.replace(armature_scale=a), ideal_step(s)
                )
                for a in ARMATURE_GRID
            }
            for s in args.ideal
        }
        report["ideal_lift_deg"] = rows
        for s, by_arm in rows.items():
            print(f"step {s}: " + ", ".join(f"{a} {v:.0f}" for a, v in by_arm.items()))

    steps: list[Step] = []
    tapes: list[StickTape] = []
    for path in args.recordings:
        found, found_tapes = find_steps(load_telemetry(path), path.name)
        print(f"{path.name}: {len(found)} steps")
        steps += found
        tapes += found_tapes
    if steps:
        for s, tape in zip(steps, tapes):
            s.sim_lift_fit_deg = sim_lift_deg(mp, collision, params, tape)
        use = [k for k, s in enumerate(steps) if not s.turning]
        refit = None
        if use:
            refit = fit_armature(
                mp,
                collision,
                params,
                [tapes[k] for k in use],
                [steps[k].measured_lift_deg for k in use],
                ARMATURE_GRID,
            )
            refit_params = params.replace(armature_scale=refit.armature_scale)
            for s, tape in zip(steps, tapes):
                s.sim_lift_refit_deg = sim_lift_deg(mp, collision, refit_params, tape)
            report["refit"] = asdict(refit)
        report["steps"] = [asdict(s) for s in steps]
        print(
            "| Recording | t (s) | Size | Rise s | Hold s | Turning | Lift deg | Sim fit "
            "| Sim refit |"
        )
        print("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        for s in steps:
            print(
                f"| {s.recording} | {s.t_start_s:.1f} | {s.size:.2f} | {s.rise_s:.2f} | "
                f"{s.hold_s:.2f} | {'yes' if s.turning else ''} | {s.measured_lift_deg:.1f} | "
                f"{s.sim_lift_fit_deg:.1f} | {s.sim_lift_refit_deg:.1f} |"
            )
        if refit is not None:
            print(
                f"armature_scale {params.armature_scale:.3f} -> {refit.armature_scale:.3f} "
                f"(RMS peak-lift error {refit.rms_error_deg:.1f} deg over {len(use)} steps)"
            )
            report["verdict"] = verdict(steps, mp, collision, refit_params, args)
    (args.out_dir / "step_test.json").write_text(json.dumps(report, indent=1))
    print(f"-> {args.out_dir / 'step_test.json'}")


def verdict(
    steps: list[Step],
    mp: MassProperties,
    collision: CollisionSet,
    params: PlantParams,
    args: argparse.Namespace,
) -> dict:
    near = [
        s.measured_lift_deg
        for s in steps
        if not s.turning and abs(s.size - args.verdict_size) <= 0.05
    ]
    out: dict = {"steps_near_size": len(near), "lift_limit_deg": args.lift_limit_deg}
    if not near:
        print(f"verdict: no straight step within 0.05 of {args.verdict_size}; drive one")
        return out
    lift = float(np.median(near))
    out["median_lift_deg"] = lift
    sweep = slew_sweep(mp, collision, params, args.full_step, SLEW_RATES)
    out["slew_sweep"] = sweep
    loosest = loosest_slew(sweep, args.lift_limit_deg)
    out["loosest_slew_per_s"] = loosest
    for rate, deg in sweep:
        label = "off" if rate <= 0 else f"{rate:.2f}/s"
        print(f"  slew {label}: {args.full_step:.1f} step lifts {deg:.1f} deg")
    if lift > args.lift_limit_deg:
        out["keep_slew_limit"] = True
        rpm = None if loosest is None else loosest * MAX_MOTOR_RPM
        print(
            f"verdict: KEEP a slew limit. {args.verdict_size} steps lift {lift:.1f} deg, past "
            f"{args.lift_limit_deg:.0f}. Loosest that holds a {args.full_step} step: "
            + ("none in the sweep" if loosest is None else f"{loosest:.2f}/s ({rpm:.0f} RPM/s)")
        )
    else:
        out["keep_slew_limit"] = False
        print(
            f"verdict: the {args.verdict_size} steps lift {lift:.1f} deg, under "
            f"{args.lift_limit_deg:.0f}; the slew limit can come off"
        )
    return out


if __name__ == "__main__":
    main()
