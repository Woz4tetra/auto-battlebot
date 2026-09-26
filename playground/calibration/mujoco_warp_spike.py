"""Step 2 spike for the Mr Stabs Mk2 MuJoCo model: sanity checks, timestep study, throughput.

1. The three sanity checks (nose lift vs the analytic threshold, top speed vs KV x V / N, decel
   tau and the resistance it implies) at each timestep in --dts.
2. Which fitted fields MuJoCo Warp batches per world (from the Model field specs).
3. World-steps per second of the captured rollout at each size in --worlds.

Writes playground/calibration/out/mujoco_sysid/spike.json and prints a summary.

    python playground/calibration/mujoco_warp_spike.py
    python playground/calibration/mujoco_warp_spike.py --worlds 1024 8192 --dts 1e-3 5e-4
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import time
from pathlib import Path

import numpy as np

from auto_battlebot.mujoco_sim import mass_properties
from auto_battlebot.mujoco_sim.actuator import PlantParams
from auto_battlebot.mujoco_sim.checks import run_checks
from auto_battlebot.mujoco_sim.mjcf import CollisionSet
from auto_battlebot.mujoco_sim.rollout import BATCHED_FIELDS, InitialState, WarpRollout

OUT = Path(__file__).resolve().parent / "out/mujoco_sysid"


def batchable_fields() -> dict[str, bool]:
    from mujoco_warp._src import types, warp_util

    specs = {
        f.name: getattr(f.type, "shape", ())
        for f in dataclasses.fields(types.Model)
        if warp_util.is_array_spec(f.type)
    }
    return {name: specs.get(name, ())[:1] == ("*",) for name in BATCHED_FIELDS}


def throughput(
    mp: mass_properties.MassProperties, collision: CollisionSet, nworld: int, steps: int
) -> float:
    rollout = WarpRollout(mp, collision, nworld, steps)
    rng = np.random.default_rng(0)
    rollout.set_params(
        [PlantParams(resistance_ohm=float(r)) for r in rng.uniform(0.1, 0.8, nworld)]
    )
    t = np.arange(steps) * rollout.timestep
    volts = np.stack([4.0 + 2.0 * np.sin(3.0 * t), 4.0 - 2.0 * np.sin(3.0 * t)], axis=-1)[
        None
    ].repeat(nworld, axis=0)
    rollout.set_tapes(volts, np.zeros_like(volts))
    zeros = np.zeros(nworld)
    state = InitialState(
        x=zeros,
        y=zeros,
        yaw=rng.uniform(-3, 3, nworld),
        vx=zeros,
        vy=zeros,
        yaw_rate=zeros,
        pitch=np.full(nworld, 0.19),
        wheel_left=zeros,
        wheel_right=zeros,
    )
    rollout.run(state)  # compile and capture
    start = time.perf_counter()
    rollout.run(state)
    return nworld * steps / (time.perf_counter() - start)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--dts", type=float, nargs="+", default=[1e-3, 5e-4, 2.5e-4])
    parser.add_argument("--worlds", type=int, nargs="+", default=[1024, 4096, 8192])
    parser.add_argument("--steps", type=int, default=1000, help="rollout length for throughput")
    parser.add_argument("--volts", type=float, default=15.2, help="top-speed check voltage")
    parser.add_argument("--out", type=Path, default=OUT / "spike.json")
    args = parser.parse_args()

    mp = mass_properties.load()
    collision = CollisionSet.load()
    report: dict[str, object] = {}
    checks = []
    for dt in args.dts:
        result = run_checks(mp, PlantParams(), collision, dt, args.volts)
        checks.append(dataclasses.asdict(result))
        print(
            f"dt {dt * 1e3:.3f} ms: rest pitch {result.rest_pitch_deg:.2f} deg | "
            f"lift {result.lift_accel_g:.3f} g (analytic {result.lift_accel_analytic_g:.3f}, "
            f"plan's N^2 J_r form {result.lift_accel_plan_reflected_g:.3f}, "
            f"skid mu {PlantParams().skid_mu} {result.lift_accel_nominal_skid_g:.3f}) | "
            f"top speed {result.top_speed:.3f} m/s (ideal {result.top_speed_ideal:.3f}) | "
            f"decel tau {result.decel_tau * 1e3:.1f} ms "
            f"(R for 78 ms, no traction limit: {result.resistance_for_measured_tau:.2f} ohm)"
        )
    report["checks"] = checks
    fields = batchable_fields()
    report["batched_fields"] = fields
    print("per-world fields:", ", ".join(f"{k}={'yes' if v else 'NO'}" for k, v in fields.items()))
    rates = {}
    for nworld in args.worlds:
        rate = throughput(mp, collision, nworld, args.steps)
        rates[str(nworld)] = rate
        print(f"{nworld} worlds: {rate / 1e6:.2f} M world-steps/s")
    report["world_steps_per_s"] = rates
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
