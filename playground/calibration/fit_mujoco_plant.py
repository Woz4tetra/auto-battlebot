"""Fit the Mr Stabs Mk2 MuJoCo plant to smoothed sysid sessions, then validate on held-out ones.

Inputs are session bundles from smooth_tag_poses.py (playground/calibration/out/mujoco_sysid/
<session>/). Hold out whole sessions, never windows from a training session.

    # fit on three sessions, validate on a fourth
    python playground/calibration/fit_mujoco_plant.py fit \\
        out/mujoco_sysid/s01 out/mujoco_sysid/s02 out/mujoco_sysid/s03 \\
        --holdout out/mujoco_sysid/s04 --restarts 4

    # re-validate an existing fit
    python playground/calibration/fit_mujoco_plant.py validate out/mujoco_sysid/fit_<ts> \\
        out/mujoco_sysid/s04

Runs go through the GPU queue like training. The first submission has no history:

    venv/bin/python training/gpu_queue.py submit --name mujoco_fit --by <agent> \\
        --profile mujoco-fit --eta 2h -- \\
        venv/bin/python playground/calibration/fit_mujoco_plant.py fit ...

`fit` prints its total rollout-steps up front; later submissions pass that as --work.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np

from auto_battlebot.control import plant
from auto_battlebot.mujoco_sim import greybox, mass_properties, validate
from auto_battlebot.mujoco_sim.actuator import PlantParams
from auto_battlebot.mujoco_sim.fit import (
    PARAM_SPECS,
    STAGES,
    FitResult,
    StageResult,
    fit,
    params_dict,
    warp_simulator,
    with_values,
)
from auto_battlebot.mujoco_sim.mjcf import CollisionSet
from auto_battlebot.mujoco_sim.rollout import WarpRollout, rest_height
from auto_battlebot.mujoco_sim.session import WindowBatch, build_batch, load_session

OUT = Path(__file__).resolve().parent / "out/mujoco_sysid"


def _batch(
    paths: list[Path], dt: float, horizon: float, mp: mass_properties.MassProperties
) -> WindowBatch:
    sessions = [load_session(p) for p in paths]
    return build_batch(
        sessions,
        horizon_s=horizon,
        dt=dt,
        track_half_width=mp.track_half_width,
        wheel_radius=mp.wheel_radius,
    )


def _subsample(batch: WindowBatch, limit: int, seed: int) -> WindowBatch:
    """Keep every nose-lift window (they are rare) and an even random draw of flat ones."""
    if len(batch) <= limit:
        return batch
    rng = np.random.default_rng(seed)
    lift = np.nonzero(batch.kind == "nose_lift")[0]
    flat = np.nonzero(batch.kind != "nose_lift")[0]
    keep_flat = rng.choice(flat, size=max(0, limit - len(lift)), replace=False)
    return batch.take(np.sort(np.concatenate([lift, keep_flat])))


def _simulator(mp: mass_properties.MassProperties, collision: CollisionSet, dt: float) -> Any:
    return warp_simulator(lambda n, steps: WarpRollout(mp, collision, n, steps, timestep=dt))


def _fit_to_json(result: FitResult, zero_mode: str, seed: int) -> dict[str, Any]:
    return {
        "zero_mode": zero_mode,
        "seed": seed,
        "loss": result.loss,
        "best": params_dict(result.best),
        "stages": [
            {"stage": s.stage, "loss": s.loss, "history": s.history, "best": params_dict(s.best)}
            for s in result.stages
        ],
    }


def _fit_from_json(data: dict[str, Any]) -> FitResult:
    best = PlantParams(**data["best"])
    stages = [
        StageResult(s["stage"], PlantParams(**s["best"]), s["loss"], s["history"])
        for s in data["stages"]
    ]
    return FitResult(best, data["loss"], stages)


def cmd_fit(args: argparse.Namespace) -> None:
    mp = mass_properties.load()
    collision = CollisionSet.load()
    batch = _subsample(
        _batch(args.sessions, args.dt, args.horizon, mp), args.max_windows, args.seed
    )
    n_lift = int((batch.kind == "nose_lift").sum())
    modes = ["brake", "coast"] if args.zero_mode == "both" else [args.zero_mode]
    per_gen = len(batch) * args.popsize * batch.steps
    total = per_gen * args.generations * len(STAGES) * args.restarts * len(modes)
    print(
        f"{len(batch)} windows ({n_lift} nose-lift) from {len(args.sessions)} sessions; "
        f"{len(batch) * args.popsize} worlds per generation; "
        f"up to {total:.3g} rollout-steps (pass as --work to gpu_queue.py)"
    )
    simulate = _simulator(mp, collision, args.dt)
    out_dir = args.out or OUT / time.strftime("fit_%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    runs: list[dict[str, Any]] = []
    rng = np.random.default_rng(args.seed)
    for mode in modes:
        for r in range(args.restarts):
            names = [s.name for s in PARAM_SPECS]
            start = PlantParams(brake_on_zero=mode == "brake")
            if r > 0:
                # Restarts begin from random points in the middle half of every range.
                start = with_values(start, names, rng.uniform(0.25, 0.75, len(names)))
            seed = args.seed + 1000 * r
            print(f"--- zero mode {mode}, restart {r + 1}/{args.restarts}")
            result = fit(
                simulate,
                batch,
                start=start,
                popsize=args.popsize,
                generations=args.generations,
                seed=seed,
            )
            runs.append(_fit_to_json(result, mode, seed))
            (out_dir / "fit.json").write_text(json.dumps(_fit_doc(args, batch, runs), indent=2))
    best_run = min(runs, key=lambda d: d["loss"] if math.isfinite(d["loss"]) else math.inf)
    print(f"best: zero mode {best_run['zero_mode']}, loss {best_run['loss']:.4f}")
    print(json.dumps(best_run["best"], indent=2))
    grey, grey_cost = greybox.fit_greybox(batch)
    doc = _fit_doc(args, batch, runs)
    doc["greybox"] = {"params": dataclasses.asdict(grey), "cost": grey_cost}
    (out_dir / "fit.json").write_text(json.dumps(doc, indent=2))
    print(f"wrote {out_dir / 'fit.json'}")
    if args.holdout:
        _validate(out_dir, args.holdout, args.dt, args.horizon)


def _fit_doc(
    args: argparse.Namespace, batch: WindowBatch, runs: list[dict[str, Any]]
) -> dict[str, Any]:
    return {
        "sessions": [str(p) for p in args.sessions],
        "dt": args.dt,
        "horizon_s": args.horizon,
        "windows": len(batch),
        "popsize": args.popsize,
        "generations": args.generations,
        "runs": runs,
    }


def _validate(fit_dir: Path, holdout: list[Path], dt: float, horizon: float) -> None:
    doc = json.loads((fit_dir / "fit.json").read_text())
    mp = mass_properties.load()
    collision = CollisionSet.load()
    batch = _batch(holdout, dt, horizon, mp)
    results = [_fit_from_json(r) for r in doc["runs"]]
    best = min(results, key=lambda r: r.loss if math.isfinite(r.loss) else math.inf).best
    grey_fields = doc.get("greybox", {}).get("params", {})
    grey = replace(plant.PlantParams(), **grey_fields) if grey_fields else plant.PlantParams()
    simulate = _simulator(mp, collision, dt)
    report: dict[str, Any] = {"holdout": [str(p) for p in holdout], "windows": len(batch)}
    compare = validate.compare_models(simulate, batch, best, grey)
    qpos, record_dt = compare.pop("_qpos"), compare.pop("_record_dt")
    report["check1_2_errors"] = compare
    radio = [load_session(p).radio_delay_s for p in holdout]
    radio_vals = [float(v) for v in radio if v is not None]
    report["check3_plausibility"] = validate.plausibility(
        mp, collision, best, float(np.median(radio_vals)) if radio_vals else None, dt
    )
    report["check4_nose_lift"] = validate.nose_lift_agreement(batch, qpos, record_dt)
    report["check5_firmware"] = validate.firmware_closed_loop(
        mp, collision, best, batch, rest_height(mp, collision, dt)
    )
    report["check6_identification"] = validate.identification(results)
    (fit_dir / "validation.json").write_text(json.dumps(report, indent=2, default=float))
    print(json.dumps(report, indent=2, default=float))
    print(f"wrote {fit_dir / 'validation.json'}")


def cmd_validate(args: argparse.Namespace) -> None:
    _validate(args.fit_dir, args.holdout, args.dt, args.horizon)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    p_fit = sub.add_parser("fit", help="staged CMA-ES fit, optionally validating afterwards")
    p_fit.add_argument("sessions", type=Path, nargs="+", help="training session bundles")
    p_fit.add_argument("--holdout", type=Path, nargs="*", default=[], help="held-out bundles")
    p_fit.add_argument("--zero-mode", choices=["brake", "coast", "both"], default="both")
    p_fit.add_argument("--restarts", type=int, default=3)
    p_fit.add_argument("--popsize", type=int, default=128)
    p_fit.add_argument("--generations", type=int, default=40)
    p_fit.add_argument("--max-windows", type=int, default=64)
    p_fit.add_argument("--seed", type=int, default=0)
    p_fit.add_argument("--out", type=Path, default=None)
    p_val = sub.add_parser("validate", help="validate an existing fit on held-out sessions")
    p_val.add_argument("fit_dir", type=Path)
    p_val.add_argument("holdout", type=Path, nargs="+")
    for p in (p_fit, p_val):
        p.add_argument("--dt", type=float, default=1e-3, help="MuJoCo timestep")
        p.add_argument("--horizon", type=float, default=1.0, help="window length scored, s")
    args = parser.parse_args()
    {"fit": cmd_fit, "validate": cmd_validate}[args.command](args)


if __name__ == "__main__":
    main()
