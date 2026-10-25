"""Fit Mrs Buff Mk3's MuJoCo plant to her jig-fitted grey-box plant, until a tagged session exists.

Mr Stabs Mk2's MuJoCo plant was fit to hand-driven sessions with AprilTags (fit_mujoco_plant.py).
Mrs Buff Mk3 has no such session, only the velocity jig's M4 grey-box fit
(playground/calibration/out/plant_stageA.toml: speeds, lags, deadzones, steer-brake and angular
droop, measured on the floor with the guard plates on). This drives the M4 model with jig-shaped
command tapes, treats its poses as measurements, and fits the MuJoCo plant to them with the same
staged CMA-ES and loss the session fit uses. M4 is flat, so this pins drive and scrub, not
tipping; with the weapon off she does not tip.

Commands are what the app sends: `DifferentialDriveProcessor` output (angular priority, the zero
deadzone), linear and angular in [-1, 1]. The radio mixes them per motor as left = linear -
angular, right = linear + angular (positive angular turns counterclockwise, the inverse
`greybox.predict` uses). Her ESCs run open loop: there is no eRPM feedback and no onboard yaw
loop, so the voltage-source actuator model is the whole command chain after the delay.

    python playground/calibration/fit_mrs_buff_mk3_interim_plant.py \\
        --out playground/calibration/out/mujoco_sysid/mk3_interim

Writes fit.json (load_fit_params reads it), holdout.json (per-horizon errors and response
checks against M4) and fit.log.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from auto_battlebot.control import plant
from auto_battlebot.mujoco_sim import mass_properties
from auto_battlebot.mujoco_sim.actuator import PlantParams, delay_tape
from auto_battlebot.mujoco_sim.fit import (
    ParamSpec,
    Simulator,
    Stage,
    fit,
    params_dict,
    score_candidates,
    warp_simulator,
)
from auto_battlebot.mujoco_sim.mjcf import CollisionSet
from auto_battlebot.mujoco_sim.rollout import InitialState, WarpRollout
from auto_battlebot.mujoco_sim.session import WindowBatch

REPO = Path(__file__).resolve().parents[2]
M4_FILE = REPO / "playground/calibration/out/plant_stageA.toml"
DT = 1e-3
PLANT_DT = 0.005
PAD_S = 0.1
PREROLL_S = 0.6
WINDOW_S = 1.0
HORIZONS = tuple(round(0.1 * k, 1) for k in range(1, 11))
MEAS_HZ = 60.0
NOISE_XY = 0.01
NOISE_YAW = 0.05
PACK_V = 15.2
ZERO_DEADZONE = 0.02  # DifferentialDriveProcessor

SPECS = {
    s.name: s
    for s in (
        ParamSpec("delay_s", 0.0, 0.08),
        ParamSpec("deadzone_left", 0.0, 0.06),
        ParamSpec("deadzone_right", 0.0, 0.06),
        ParamSpec("curve", -0.9, 2.0),
        ParamSpec("resistance_ohm", 0.2, 3.0, log=True),
        ParamSpec("efficiency", 0.6, 0.95),
        ParamSpec("lr_gain_ratio", 0.8, 1.2),
        ParamSpec("joint_frictionloss", 1e-4, 5e-2, log=True),
        ParamSpec("joint_damping", 1e-6, 1e-3, log=True),
        ParamSpec("wheel_mu_slide", 0.1, 3.0, log=True),
        ParamSpec("wheel_mu_torsion", 1e-4, 5e-2, log=True),
        # Capped at 0.02 s: the same contact softness applies to walls and opponents, and a
        # softer contact lets the hull sink into them in a collision. M4 is flat, so the floor
        # alone would happily trade stiffness away.
        ParamSpec("solref_timeconst", 0.004, 0.02, log=True),
        ParamSpec("solimp_dmax", 0.9, 0.9999),
        ParamSpec("armature_scale", 0.3, 3.0, log=True),
        # The ESC's current limit caps acceleration. The datasheet gives 10.4 A stall at 12 V;
        # without this term the fit pinned the rotor inertia at 3x to slow the launch.
        ParamSpec("current_limit_a", 3.0, 40.0, log=True),
    )
}
STAGES = (
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
            "armature_scale",
            "current_limit_a",
        ),
        ("flat",),
    ),
    Stage("turning", ("wheel_mu_slide", "wheel_mu_torsion", "lr_gain_ratio"), ("flat",)),
    Stage("joint", tuple(SPECS), ("flat",)),
)


def saturate(lin: np.ndarray, ang: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """`torch_plant.saturate` in numpy: angular priority, then each wheel's zero deadzone."""
    ang = np.clip(ang, -1.0, 1.0)
    headroom = np.clip(1.0 - np.abs(ang), 0.0, None)
    lin = np.clip(lin, -headroom, headroom)
    out = []
    for v in (lin + ang, lin - ang):
        mag = np.clip((np.abs(v) - ZERO_DEADZONE) / (1.0 - ZERO_DEADZONE), 0.0, 1.0)
        out.append(np.where(np.abs(v) <= ZERO_DEADZONE, 0.0, np.sign(v) * mag))
    a, b = out
    return 0.5 * (a + b), 0.5 * (a - b)


def command_tape(rng: np.random.Generator, n: int) -> tuple[np.ndarray, np.ndarray]:
    """One window's linear and angular commands on the PLANT_DT grid, piecewise constant.

    Each window draws one family, the way the jig protocol excites one term at a time: straight
    steps and reversals, spins, arcs on the steer/brake grid, and staircases near the deadzone.
    Holds end in zero about a third of the time, so the coast or brake decel is in the data.
    """
    lin = np.zeros(n)
    ang = np.zeros(n)
    family = rng.integers(0, 5)
    t = 0
    while t < n:
        hold = int(rng.uniform(0.15, 0.5) / PLANT_DT)
        if rng.random() < 0.3:
            v, w = 0.0, 0.0
        elif family == 0:  # straight
            v, w = rng.choice([-1, 1]) * rng.uniform(0.05, 1.0), 0.0
        elif family == 1:  # spin
            v, w = 0.0, rng.choice([-1, 1]) * rng.uniform(0.05, 1.0)
        elif family == 2:  # arc
            w = rng.uniform(-1.0, 1.0)
            v = rng.uniform(-1.0, 1.0) * (1.0 - abs(w))
        elif family == 3:  # full-speed arc with a growing turn: the steer-brake grid
            v, w = rng.choice([-1, 1]) * 1.0, rng.uniform(-0.6, 0.6)
        else:  # deadzone staircase
            v, w = rng.uniform(-0.08, 0.08), rng.uniform(-0.08, 0.08)
        lin[t : t + hold], ang[t : t + hold] = v, w
        t += hold
    return saturate(lin, ang)


def build_windows(m4: plant.PlantParams, count: int, seed: int) -> WindowBatch:
    rng = np.random.default_rng(seed)
    n_plant = int(round((PREROLL_S + WINDOW_S) / PLANT_DT))
    lin = np.empty((count, n_plant))
    ang = np.empty((count, n_plant))
    for i in range(count):
        lin[i], ang[i] = command_tape(rng, n_plant)
    # The app's commands pass the delay, then M4. Shift on the plant grid, as greybox.predict does.
    lin_d = delay_tape(lin, PLANT_DT, m4.delay_s)
    ang_d = delay_tape(ang, PLANT_DT, m4.delay_s)
    _, traj = plant.simulate(lin_d, ang_d, PLANT_DT, m4)
    start = int(round(PREROLL_S / PLANT_DT)) - 1  # the state after this step starts the window
    x0, y0, th0 = traj["x"][:, start], traj["y"][:, start], traj["theta"][:, start]
    v0, w0 = traj["v"][:, start], traj["w"][:, start]

    every = int(round(PLANT_DT / DT))
    pad = int(round(PAD_S / DT))
    steps = int(round(WINDOW_S / DT))
    first = (start + 1) * every - pad
    left = np.repeat(lin - ang, every, axis=1)[:, first : first + pad + steps]
    right = np.repeat(lin + ang, every, axis=1)[:, first : first + pad + steps]
    u_raw = np.stack([left, right], axis=-1)

    mp = mass_properties.load(mass_properties.asset_dir("mrs_buff_mk3") / "mass_properties.toml")
    b, r = mp.track_half_width, mp.wheel_radius
    state = InitialState(
        x=x0,
        y=y0,
        yaw=th0,
        vx=v0 * np.cos(th0),
        vy=v0 * np.sin(th0),
        yaw_rate=w0,
        pitch=np.zeros(count),
        wheel_left=(v0 - w0 * b) / r,
        wheel_right=(v0 + w0 * b) / r,
    )
    meas_k = np.arange(1, int(WINDOW_S * MEAS_HZ) + 1)
    meas_t = meas_k / MEAS_HZ
    plant_t = np.arange(1, n_plant - start) * PLANT_DT  # sample k: state after step start + k
    meas = {"x": [], "y": [], "yaw": []}
    for i in range(count):
        for key, src in (("x", "x"), ("y", "y"), ("yaw", "theta")):
            meas[key].append(np.interp(meas_t, plant_t, traj[src][i, start + 1 :]))
    return WindowBatch(
        dt=DT,
        steps=steps,
        pad=pad,
        horizons=HORIZONS,
        session=np.zeros(count, dtype=int),
        window_id=np.arange(count),
        kind=np.array(["flat"] * count),
        start_ns=np.zeros(count, dtype=np.int64),
        u_raw=u_raw,
        vbat=np.full((count, pad + steps), PACK_V),
        state=state,
        polarity=np.ones(count),
        meas_t=[meas_t.copy() for _ in range(count)],
        meas_x=meas["x"],
        meas_y=meas["y"],
        meas_yaw=meas["yaw"],
        meas_pitch=[np.full(len(meas_t), np.nan) for _ in range(count)],
        noise_xy=np.full(count, NOISE_XY),
        noise_yaw=np.full(count, NOISE_YAW),
        noise_pitch=np.full(count, 0.05),
        sessions=["m4_synthetic"],
    )


def response_checks(
    simulate: Simulator, m4: plant.PlantParams, params: PlantParams
) -> dict[str, float]:
    """Steady speed and yaw rate, MuJoCo against M4, on held full and half commands."""
    cases = {"lin_1.0": (1.0, 0.0), "lin_0.5": (0.5, 0.0), "lin_-1.0": (-1.0, 0.0)}
    cases.update({"ang_1.0": (0.0, 1.0), "ang_0.5": (0.0, 0.5), "arc_0.6_0.4": (0.6, 0.4)})
    out = {}
    for name, (v, w) in cases.items():
        hold = 1.2
        n = int(hold / PLANT_DT)
        lin, ang = saturate(np.full((1, n), v), np.full((1, n), w))
        _, traj = plant.simulate(
            delay_tape(lin, PLANT_DT, m4.delay_s),
            delay_tape(ang, PLANT_DT, m4.delay_s),
            PLANT_DT,
            m4,
        )
        steps = int(hold / DT)
        u = np.stack(
            [
                np.full(steps + 100, lin[0, 0] - ang[0, 0]),
                np.full(steps + 100, lin[0, 0] + ang[0, 0]),
            ],
            -1,
        )
        zeros = np.zeros(1)
        batch = WindowBatch(
            dt=DT,
            steps=steps,
            pad=100,
            horizons=(hold,),
            session=np.zeros(1, dtype=int),
            window_id=np.zeros(1, dtype=int),
            kind=np.array(["flat"]),
            start_ns=np.zeros(1, dtype=np.int64),
            u_raw=u[None],
            vbat=np.full((1, steps + 100), PACK_V),
            state=InitialState(zeros, zeros, zeros, zeros, zeros, zeros, zeros, zeros, zeros),
        )
        qpos, record_dt = simulate(batch, [params])
        tail = int(0.3 / record_dt)
        dx = np.diff(qpos[0, -tail:, 0]) / record_dt
        dy = np.diff(qpos[0, -tail:, 1]) / record_dt
        q = qpos[0, -tail:]
        yaw = np.unwrap(
            np.arctan2(
                2 * (q[:, 3] * q[:, 6] + q[:, 4] * q[:, 5]), 1 - 2 * (q[:, 5] ** 2 + q[:, 6] ** 2)
            )
        )
        speed = float(np.mean(dx * np.cos(yaw[1:]) + dy * np.sin(yaw[1:])))
        rate = float(np.mean(np.diff(yaw) / record_dt))
        out[f"{name}_speed_mujoco"] = round(speed, 3)
        out[f"{name}_speed_m4"] = round(float(traj["v"][0, -1]), 3)
        out[f"{name}_rate_mujoco"] = round(rate, 3)
        out[f"{name}_rate_m4"] = round(float(traj["w"][0, -1]), 3)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--windows", type=int, default=256)
    parser.add_argument("--popsize", type=int, default=32)
    parser.add_argument("--generations", type=int, default=40)
    parser.add_argument("--restarts", type=int, default=2)
    parser.add_argument("--modes", nargs="+", default=["brake", "coast"])
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    logf = (args.out / "fit.log").open("a")

    def log(msg: str) -> None:
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        print(line, flush=True)
        logf.write(line + "\n")
        logf.flush()

    m4 = plant.PlantParams.from_toml(M4_FILE)
    batch = build_windows(m4, args.windows, args.seed)
    holdout_n = args.windows // 4
    train = batch.take(np.arange(holdout_n, args.windows))
    holdout = batch.take(np.arange(holdout_n))
    asset = mass_properties.asset_dir("mrs_buff_mk3")
    mp = mass_properties.load(asset / "mass_properties.toml")
    collision = CollisionSet.load(asset / "collision")
    simulate = warp_simulator(lambda n, steps: WarpRollout(mp, collision, n, steps, DT, 5))
    log(f"{len(train)} training and {len(holdout)} held-out windows from {M4_FILE.name}")

    runs = []
    start = PlantParams(
        resistance_ohm=1.2, wheel_mu_slide=0.6, armature_scale=1.0, current_limit_a=12.0
    )
    for mode in args.modes:
        for r in range(args.restarts):
            base = replace(start, brake_on_zero=(mode == "brake"))
            res = fit(
                simulate,
                train,
                base,
                STAGES,
                args.popsize,
                args.generations,
                seed=args.seed + 1000 * r + (0 if mode == "brake" else 500),
                log=lambda m, mode=mode, r=r: log(f"{mode}/{r} {m}"),
                specs=SPECS,
            )
            hold_loss, _ = score_candidates(simulate, holdout, [res.best])
            log(f"{mode}/{r}: train {res.loss:.4f} holdout {float(hold_loss[0]):.4f}")
            runs.append(
                {
                    "mode": mode,
                    "restart": r,
                    "loss": res.loss,
                    "holdout_loss": float(hold_loss[0]),
                    "best": params_dict(res.best),
                    "stages": [
                        {"name": s.stage, "loss": s.loss, "history": s.history} for s in res.stages
                    ],
                }
            )
            (args.out / "fit.json").write_text(
                json.dumps({"source": str(M4_FILE.relative_to(REPO)), "runs": runs}, indent=1)
            )

    best = min(
        (r for r in runs if math.isfinite(r["holdout_loss"])), key=lambda r: r["holdout_loss"]
    )
    params = PlantParams(**best["best"])
    _, errs = score_candidates(simulate, holdout, [params])
    err = errs[0]
    report = {
        "best": {
            "mode": best["mode"],
            "restart": best["restart"],
            "holdout_loss": best["holdout_loss"],
        },
        "holdout_position_error_m_median_by_horizon": {
            str(h): float(np.nanmedian(err.position[:, j])) for j, h in enumerate(HORIZONS)
        },
        "holdout_heading_error_rad_median_by_horizon": {
            str(h): float(np.nanmedian(np.abs(err.heading[:, j]))) for j, h in enumerate(HORIZONS)
        },
        "responses": response_checks(simulate, m4, params),
    }
    (args.out / "holdout.json").write_text(json.dumps(report, indent=1))
    log(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
