"""Held-out validation of a MuJoCo plant fit (the plan's six acceptance checks).

1. MuJoCo beats or matches the grey-box plant at 0.4 and 1.0 s on flat windows.
2. Errors as multiples of the noise floor.
3. Physical numbers inside physical ranges; coast tau near the measured 0.078 s.
4. Nose lift on the same held-out windows as the real robot, and not on the others.
5. The firmware layer in closed loop tracks the logged pid_output and per-motor commands on
   straight holds.
6. Restart agreement and the parameter pairs that trade off.

Every function returns plain dicts so the CLI can dump them to JSON.
"""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from auto_battlebot.control import plant
from auto_battlebot.mujoco_sim import greybox
from auto_battlebot.mujoco_sim.actuator import (
    MotorConstants,
    PlantParams,
    shape_command,
    side_gains,
)
from auto_battlebot.mujoco_sim.checks import make, simulated_decel_tau
from auto_battlebot.mujoco_sim.firmware import FirmwareMixer, heading_from_yaw
from auto_battlebot.mujoco_sim.fit import (
    FitResult,
    Simulator,
    WindowErrors,
    restart_spread,
    tradeoff_pairs,
    window_errors,
)
from auto_battlebot.mujoco_sim.mass_properties import MassProperties
from auto_battlebot.mujoco_sim.mjcf import CollisionSet, build_mjcf
from auto_battlebot.mujoco_sim.rollout import ModelIndex, initial_qpos_qvel, pose_from_qpos
from auto_battlebot.mujoco_sim.session import WindowBatch

MEASURED_COAST_TAU_S = 0.078
LIFT_DEG = 3.0
PLAUSIBLE = {
    "resistance_ohm": (0.05, 1.0),
    "efficiency": (0.6, 0.95),
    "wheel_mu_slide": (0.5, 2.5),
    "skid_mu": (0.05, 1.0),
    "armature_scale": (0.8, 1.2),
}


def _horizon_table(err: WindowErrors, batch: WindowBatch, mask: np.ndarray) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for j, h in enumerate(batch.horizons):
        pos = err.position[mask, j]
        yaw = err.heading[mask, j]
        ok = np.isfinite(pos)
        if not ok.any():
            continue
        pos_floor = pos[ok] / batch.noise_xy[mask][ok]
        yaw_floor = np.abs(yaw[ok]) / batch.noise_yaw[mask][ok]
        out[f"{h:.1f}s"] = {
            "windows": int(ok.sum()),
            "position_median_m": float(np.median(pos[ok])),
            "position_p90_m": float(np.percentile(pos[ok], 90)),
            "heading_median_rad": float(np.median(np.abs(yaw[ok]))),
            "position_median_x_noise": float(np.median(pos_floor)),
            "heading_median_x_noise": float(np.median(yaw_floor)),
        }
    return out


def compare_models(
    simulate: Simulator,
    batch: WindowBatch,
    params: PlantParams,
    grey: plant.PlantParams,
) -> dict[str, Any]:
    """Checks 1 and 2 on flat windows, plus the MuJoCo errors on nose-lift windows."""
    qpos, record_dt = simulate(batch, [params] * len(batch))
    mj = window_errors(batch, qpos, record_dt)
    gq, gdt = greybox.predict(batch, grey)
    gb = window_errors(batch, gq, gdt)
    flat = batch.kind == "flat"
    out: dict[str, Any] = {
        "mujoco_flat": _horizon_table(mj, batch, flat),
        "greybox_flat": _horizon_table(gb, batch, flat),
        "mujoco_nose_lift": _horizon_table(mj, batch, ~flat),
    }
    verdict = {}
    for h in ("0.4s", "1.0s"):
        m = out["mujoco_flat"].get(h)
        g = out["greybox_flat"].get(h)
        if m and g:
            verdict[h] = {
                "position_ratio": m["position_median_m"] / max(g["position_median_m"], 1e-9),
                "heading_ratio": m["heading_median_rad"] / max(g["heading_median_rad"], 1e-9),
                "mujoco_beats_or_matches": m["position_median_m"] <= 1.05 * g["position_median_m"],
            }
    out["check1_vs_greybox"] = verdict
    out["_qpos"] = qpos  # for the nose-lift check; the CLI drops it before writing JSON
    out["_record_dt"] = record_dt
    return out


def nose_lift_agreement(batch: WindowBatch, qpos: np.ndarray, record_dt: float) -> dict[str, Any]:
    """Check 4: did the model lift where the robot lifted (pitch LIFT_DEG under its start)?"""
    pose = pose_from_qpos(qpos)
    real, sim = [], []
    for i in range(len(batch)):
        if len(batch.meas_pitch[i]) == 0:
            continue
        start = batch.state.pitch[i]
        real.append(bool(np.degrees(start - np.min(batch.meas_pitch[i])) > LIFT_DEG))
        sim.append(bool(np.degrees(start - np.min(pose["pitch"][i])) > LIFT_DEG))
    r, s = np.array(real), np.array(sim)
    return {
        "windows": int(len(r)),
        "both_lift": int((r & s).sum()),
        "real_only": int((r & ~s).sum()),
        "sim_only": int((~r & s).sum()),
        "neither": int((~r & ~s).sum()),
    }


def plausibility(
    mp: MassProperties,
    collision: CollisionSet | None,
    params: PlantParams,
    radio_delay_s: float | None,
    timestep: float = 1e-3,
) -> dict[str, Any]:
    """Check 3."""
    out: dict[str, Any] = {}
    for name, (lo, hi) in PLAUSIBLE.items():
        value = float(getattr(params, name))
        out[name] = {"value": value, "range": [lo, hi], "ok": lo <= value <= hi}
    motor = MotorConstants(gear_ratio=mp.gear_ratio, kt=mp.motor_kt)
    tau = simulated_decel_tau(*make(mp, params, collision, timestep), params=params, motor=motor)
    out["coast_tau_s"] = {"value": tau, "measured": MEASURED_COAST_TAU_S}
    total = params.delay_s + (radio_delay_s or 0.0)
    out["total_delay_s"] = {
        "drivetrain": params.delay_s,
        "radio": radio_delay_s,
        "total": total,
        "near_60ms": abs(total - 0.06) < 0.02 if radio_delay_s is not None else None,
    }
    return out


def firmware_closed_loop(
    mp: MassProperties,
    collision: CollisionSet | None,
    params: PlantParams,
    batch: WindowBatch,
    height: float,
    max_windows: int = 20,
) -> dict[str, Any]:
    """Check 5, on CPU MuJoCo: drive the fitted drivetrain from the logged sticks through the
    firmware mixer with the sim's heading, and compare its outputs with the logged ones.

    Only windows where the turn stick stays inside the 1% threshold (straight holds, where the
    heading hold acts) are used. The sim heading is offset to match the logged BNO055 heading at
    the window start, so only the change in heading drives the PID.
    """
    if batch.firmware.size == 0:
        return {"windows": 0}
    fw = batch.firmware[:, batch.pad :]
    straight = [
        i
        for i in range(len(batch))
        if np.all(np.abs(fw[i, :, 1]) <= 1.0) and np.isfinite(fw[i]).all()
    ][:max_windows]
    motor = MotorConstants(gear_ratio=mp.gear_ratio, kt=mp.motor_kt)
    model = mujoco.MjModel.from_xml_string(build_mjcf(mp, params, collision, batch.dt))
    data = mujoco.MjData(model)
    index = ModelIndex.of(model)
    gain_l, gain_r = side_gains(params.lr_gain_ratio)
    delay_steps = int(round(params.delay_s / batch.dt))
    rms_pid, rms_cmd = [], []
    for i in straight:
        mujoco.mj_resetData(model, data)
        qpos, qvel = initial_qpos_qvel(
            model, index, batch.take(np.array([i])).state, height, mp.gear_ratio
        )
        data.qpos[:], data.qvel[:] = qpos[0], qvel[0]
        mixer = FirmwareMixer()
        heading_offset = fw[i, 0, 5] - heading_from_yaw(batch.state.yaw[i])
        queue = [(0.0, 0.0)] * delay_steps
        sim_pid, sim_cmd = [], []
        for t in range(batch.steps):
            yaw = pose_from_qpos(data.qpos[:7])["yaw"]
            heading = (heading_from_yaw(float(yaw)) + heading_offset) % 360.0
            left, right = mixer.step(fw[i, t, 0], fw[i, t, 1], heading, batch.dt)
            sim_pid.append(mixer.pid_output)
            sim_cmd.append((left, right))
            queue.append((left, right))
            left_d, right_d = queue.pop(0)
            volts = []
            for cmd, dz, g, vb in (
                (left_d, params.deadzone_left, gain_l, batch.vbat[i, batch.pad + t]),
                (right_d, params.deadzone_right, gain_r, batch.vbat[i, batch.pad + t]),
            ):
                u = batch.polarity[i] * cmd / 100.0
                shaped = float(shape_command(np.array(u), dz, params.curve))
                volts.append(shaped * vb * float(g))
            ctrl = np.array(volts)
            if not params.brake_on_zero:
                emf = motor.back_emf_volts(data.qvel[index.wheel_dofs])
                ctrl = np.where(ctrl == 0.0, emf, ctrl)
            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)
        sim_cmd_arr = np.array(sim_cmd)
        rms_pid.append(float(np.sqrt(np.mean((np.array(sim_pid) - fw[i, :, 2]) ** 2))))
        rms_cmd.append(float(np.sqrt(np.mean((sim_cmd_arr - fw[i, :, 3:5]) ** 2))))
    return {
        "windows": len(straight),
        "pid_output_rms_percent": float(np.median(rms_pid)) if rms_pid else None,
        "motor_command_rms_percent": float(np.median(rms_cmd)) if rms_cmd else None,
    }


def identification(results: list[FitResult]) -> dict[str, Any]:
    """Check 6."""
    return {
        "restarts": len(results),
        "losses": [r.loss for r in results],
        "spread": restart_spread(results),
        "tradeoff_pairs": [{"a": a, "b": b, "corr": c} for a, b, c in tradeoff_pairs(results)],
    }
