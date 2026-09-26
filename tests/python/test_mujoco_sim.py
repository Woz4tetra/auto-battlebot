"""Tests for auto_battlebot.mujoco_sim: lumping, the MJCF, the firmware mirror and the fit.

The end-to-end test builds a synthetic session bundle from a CPU MuJoCo run with known
parameters, then checks the fit's loss prefers the truth over a perturbed model.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from auto_battlebot.mujoco_sim import greybox, mass_properties
from auto_battlebot.mujoco_sim.actuator import PlantParams, delay_tape, shape_command
from auto_battlebot.mujoco_sim.checks import (
    analytic_lift_accel,
    ideal_top_speed,
    make,
    pitch_of,
    settle,
    simulated_lift_accel,
    simulated_top_speed,
)
from auto_battlebot.mujoco_sim.closed_loop import ClosedLoopSim, Disc, load_fit_params
from auto_battlebot.mujoco_sim.firmware import FirmwareMixer, Pid
from auto_battlebot.mujoco_sim.fit import params_dict, score_candidates
from auto_battlebot.mujoco_sim.mjcf import CollisionSet
from auto_battlebot.mujoco_sim.onshape_export import (
    WHEEL_JOINTS,
    MassBody,
    UrdfAssembly,
    body_from_root,
    combine,
    subtract,
)
from auto_battlebot.mujoco_sim.rollout import (
    InitialState,
    cpu_rollout,
    rest_height,
)
from auto_battlebot.mujoco_sim.session import WindowBatch, build_batch, load_session

EXPORT = mass_properties.ASSET_DIR / "onshape_export"
DT = 1e-3


@pytest.fixture(scope="module")
def mp() -> mass_properties.MassProperties:
    return mass_properties.load()


@pytest.fixture(scope="module")
def collision() -> CollisionSet:
    return CollisionSet.load()


def test_wheels_lump_to_cad_values() -> None:
    assembly = UrdfAssembly(EXPORT / "mr_stabs_mk2.urdf")
    rot, trans = body_from_root(assembly)
    wheels = [
        assembly.lumped(assembly.wheel_links(j)).transformed(rot, trans) for j in WHEEL_JOINTS
    ]
    for wheel in wheels:
        assert wheel.mass == pytest.approx(0.018445, abs=2e-6)
        assert abs(wheel.com[1]) == pytest.approx(0.06526, abs=2e-5)
        assert wheel.inertia[1, 1] == pytest.approx(4.2528e-6, rel=1e-3)


def test_subtract_inverts_combine() -> None:
    rng = np.random.default_rng(0)
    parts = []
    for _ in range(3):
        a = rng.normal(size=(3, 3))
        parts.append(
            MassBody(float(rng.uniform(0.1, 1)), rng.normal(size=3) * 0.05, a @ a.T * 1e-4)
        )
    whole = combine(parts)
    back = subtract(whole, parts[1:])
    assert back.mass == pytest.approx(parts[0].mass)
    np.testing.assert_allclose(back.com, parts[0].com, atol=1e-12)
    np.testing.assert_allclose(back.inertia, parts[0].inertia, atol=1e-12)


def test_mass_properties_table(mp: mass_properties.MassProperties) -> None:
    assert mp.total_mass == pytest.approx(0.4959, abs=1e-6)
    assert mp.chassis.com[0] == pytest.approx(0.03194, abs=5e-5)
    # Tags decode from the CAD faces: 76 faces up, 41 faces the floor, both 64 mm.
    assert not mp.tags[76].upside_down and mp.tags[41].upside_down
    for tag in mp.tags.values():
        assert tag.size_m == pytest.approx(0.064, abs=2e-4)
        np.testing.assert_allclose(tag.rotation @ tag.rotation.T, np.eye(3), atol=1e-5)
        assert np.linalg.det(tag.rotation) == pytest.approx(1.0, abs=1e-5)


def test_model_rests_on_nose_and_hits_free_speed(
    mp: mass_properties.MassProperties, collision: CollisionSet
) -> None:
    model, data = make(mp, PlantParams(), collision, DT)
    settle(model, data)
    assert np.degrees(pitch_of(data.qpos[3:7])) == pytest.approx(10.9, abs=0.5)
    speed = simulated_top_speed(*make(mp, PlantParams(), collision, DT), volts=8.0)
    assert speed == pytest.approx(ideal_top_speed(mp, 8.0), rel=0.02)


def test_nose_lift_matches_momentum_analysis(
    mp: mass_properties.MassProperties, collision: CollisionSet
) -> None:
    model, data = make(mp, PlantParams(), collision, DT)
    settle(model, data)
    rest = pitch_of(data.qpos[3:7])
    lift = simulated_lift_accel(*make(mp, PlantParams(), collision, DT))
    assert lift == pytest.approx(analytic_lift_accel(mp, rest), rel=0.05)


def test_pid_mirrors_firmware_quirks() -> None:
    pid = Pid()
    assert pid.update(10.0, 9.0, 0.01) == 0.0  # inside the 2 degree tolerance
    out = pid.update(10.0, 0.0, 0.01)  # first call past tolerance: no derivative yet
    assert out == pytest.approx(0.08 * 10 + 0.01 * 10 * 0.01)
    assert pid._wrap(350.0) == pytest.approx(-10.0)


def test_mixer_normalizes_and_holds_heading() -> None:
    mixer = FirmwareMixer()
    left, right = mixer.step(-100.0, 50.0, 0.0, 0.01)
    assert max(abs(left), abs(right)) == pytest.approx(100.0)
    mixer = FirmwareMixer()
    mixer.step(-50.0, 0.0, 90.0, 0.01)
    mixer.step(-50.0, 0.0, 80.0, 0.01)  # drifted 10 degrees: heading hold acts
    assert mixer.pid_output > 0.0


def test_shape_and_delay() -> None:
    u = np.array([-1.0, -0.03, 0.0, 0.03, 0.5, 1.0])
    shaped = shape_command(u, 0.04, 0.0)
    np.testing.assert_allclose(shaped[[1, 2, 3]], 0.0)
    assert shaped[-1] == pytest.approx(1.0) and shaped[0] == pytest.approx(-1.0)
    tape = np.zeros(100)
    tape[50:] = 1.0
    shifted = delay_tape(tape, 1e-3, 0.010)
    assert shifted[59] == 0.0 and shifted[60] == 1.0


# --------------------------------------------------------------------------- end to end


def _synthetic_bundle(
    tmp: Path, mp: mass_properties.MassProperties, collision: CollisionSet, truth: PlantParams
) -> Path:
    """Drive one continuous CPU run and write it out as a smooth_tag_poses.py bundle."""
    seconds = 4.0
    steps = int(seconds / DT)
    t = np.arange(steps) * DT
    left = 30 * np.sin(1.3 * t) + 20 * (t > 2.0)
    right = 30 * np.sin(1.3 * t + 1.0) + 20 * (t > 2.0)
    u = np.stack([left, right], -1) / 100.0
    vbat = 15.0
    batch_like = WindowBatch(
        dt=DT,
        steps=steps,
        pad=0,
        horizons=(0.2,),
        session=np.zeros(1, int),
        window_id=np.zeros(1, int),
        kind=np.array(["flat"]),
        start_ns=np.zeros(1, np.int64),
        u_raw=u[None],
        vbat=np.full((1, steps), vbat),
        state=InitialState(*(np.zeros(1) for _ in range(9))),
    )
    batch_like.state.pitch[:] = 0.19
    volts, zero = batch_like.tapes(truth)
    height = rest_height(mp, collision, DT)
    qpos = cpu_rollout(mp, collision, truth, batch_like.state, volts[0], zero[0], DT, 1, height)
    from auto_battlebot.mujoco_sim.rollout import pose_from_qpos

    pose = pose_from_qpos(qpos)
    yaw = np.unwrap(pose["yaw"])
    vx, vy, wz = (np.gradient(a, DT) for a in (pose["x"], pose["y"], yaw))
    stamp = (t * 1e9).astype(np.int64)
    sm = pd.DataFrame(
        {
            "stamp_ns": stamp,
            "x": pose["x"],
            "y": pose["y"],
            "yaw": yaw,
            "vx": vx,
            "vy": vy,
            "yaw_rate": wz,
            "pitch": pose["pitch"],
        }
    )[::5]
    sm.to_csv(tmp / "smoothed.csv", index=False)
    frames = np.arange(0, steps, int(round(1 / 60 / DT)))
    pd.DataFrame(
        {
            "stamp_ns": stamp[frames],
            "tag_id": 76,
            "x": pose["x"][frames],
            "y": pose["y"][frames],
            "z": 0.0,
            "yaw": yaw[frames],
            "pitch": pose["pitch"][frames],
            "roll": 0.0,
            "rejected": False,
        }
    ).to_csv(tmp / "measurements.csv", index=False)
    ev = np.arange(0, steps, 5)
    pd.DataFrame(
        {
            "stamp_ns": stamp[ev],
            "left_cmd": left[ev],
            "right_cmd": right[ev],
            "vbat": vbat,
            "a_percent": -(left[ev] + right[ev]) / 2,
            "b_percent": (left[ev] - right[ev]) / 2,
            "pid_output": 0.0,
            "orientation_x": 0.0,
        }
    ).to_csv(tmp / "commands.csv", index=False)
    starts = [0.5, 1.5, 2.5]
    rows = []
    for k, s in enumerate(starts):
        i = int(s / DT)
        rows.append(
            {
                "window_id": k,
                "start_ns": int(stamp[i]),
                "end_ns": int(stamp[i] + 1e9),
                "kind": "flat",
                "coverage": 1.0,
                "x0": pose["x"][i],
                "y0": pose["y"][i],
                "yaw0": yaw[i],
                "vx0": vx[i],
                "vy0": vy[i],
                "yaw_rate0": wz[i],
                "pitch0": pose["pitch"][i],
                "wheel_left0": 0.0,
                "wheel_right0": 0.0,
            }
        )
    pd.DataFrame(rows).to_csv(tmp / "windows.csv", index=False)
    (tmp / "session.json").write_text(
        json.dumps({"noise_floor": {"sigma_xy": 0.002, "sigma_yaw": 0.01, "sigma_pitch": 0.01}})
    )
    return tmp


def _cpu_simulator(mp: mass_properties.MassProperties, collision: CollisionSet):  # type: ignore[no-untyped-def]
    height = rest_height(mp, collision, DT)

    def simulate(batch: WindowBatch, params: list[PlantParams]) -> tuple[np.ndarray, float]:
        volts, zero = batch.tapes(params)
        out = []
        for i in range(len(batch)):
            state = batch.take(np.array([i])).state
            out.append(
                cpu_rollout(mp, collision, params[i], state, volts[i], zero[i], DT, 5, height)
            )
        return np.stack(out), 5 * DT

    return simulate


def test_fit_loss_prefers_truth(
    tmp_path: Path, mp: mass_properties.MassProperties, collision: CollisionSet
) -> None:
    truth = PlantParams(resistance_ohm=0.4, delay_s=0.02)
    bundle = _synthetic_bundle(tmp_path, mp, collision, truth)
    batch = build_batch([load_session(bundle)], horizon_s=0.4, dt=DT, horizons=(0.2, 0.4))
    assert len(batch) == 3
    losses, errors = score_candidates(
        _cpu_simulator(mp, collision),
        batch,
        [truth, truth.replace(resistance_ohm=0.15), truth.replace(delay_s=0.05)],
    )
    assert losses[0] < losses[1] and losses[0] < losses[2]
    # The truth reproduces the run it came from to within a few noise floors at 0.2 s.
    assert np.nanmedian(errors[0].position[:, 0]) < 0.01


def test_greybox_runs_on_the_same_windows(
    tmp_path: Path, mp: mass_properties.MassProperties, collision: CollisionSet
) -> None:
    from auto_battlebot.control import plant

    bundle = _synthetic_bundle(tmp_path, mp, collision, PlantParams())
    batch = build_batch([load_session(bundle)], horizon_s=0.4, dt=DT, horizons=(0.2, 0.4))
    qpos, record_dt = greybox.predict(batch, plant.PlantParams())
    assert qpos.shape[0] == len(batch) and np.isfinite(qpos).all()
    assert record_dt == greybox.PLANT_DT


def _has_cuda() -> bool:
    if importlib.util.find_spec("mujoco_warp") is None:
        return False
    import warp as wp

    return bool(wp.get_cuda_device_count())


@pytest.mark.skipif(not _has_cuda(), reason="needs mujoco_warp and a CUDA device")
def test_warp_rollout_matches_cpu_on_a_straight_drive(
    mp: mass_properties.MassProperties, collision: CollisionSet
) -> None:
    from auto_battlebot.mujoco_sim.rollout import WarpRollout, pose_from_qpos

    steps = 600
    rollout = WarpRollout(mp, collision, 4, steps)
    params = [PlantParams(resistance_ohm=r) for r in (0.15, 0.3, 0.5, 0.8)]
    rollout.set_params(params)
    volts = np.zeros((4, steps, 2))
    volts[:, :400] = 4.0
    zero = (volts == 0.0).astype(float)
    rollout.set_tapes(volts, zero)
    z = np.zeros(4)
    state = InitialState(z, z, z, z, z, z, np.full(4, 0.19), z, z)
    gpu = pose_from_qpos(rollout.run(state))
    for w in range(4):
        one = InitialState(*(np.asarray(v)[w : w + 1] for v in vars(state).values()))
        cpu = pose_from_qpos(
            cpu_rollout(mp, collision, params[w], one, volts[w], zero[w], DT, 5, rollout.height)
        )
        assert gpu["x"][w, -1] == pytest.approx(cpu["x"][-1], abs=0.01)


def _drive(sim: ClosedLoopSim, linear: float, angular: float, seconds: float) -> None:
    for _ in range(round(seconds * 30)):
        sim.step(linear, angular, 1.0 / 30.0)


def test_closed_loop_signs_match_the_app(mp: mass_properties.MassProperties) -> None:
    """Positive linear drives along the heading, positive angular turns counterclockwise."""
    sim = ClosedLoopSim(mp, None, PlantParams(), start=(0.0, 0.0, math.pi / 2))
    _drive(sim, 0.2, 0.0, 0.5)
    x, y, yaw = sim.pose()
    assert y > 0.2 and abs(x) < 0.02
    assert abs(yaw - math.pi / 2) < 0.02
    sim = ClosedLoopSim(mp, None, PlantParams(), start=(0.0, 0.0, 0.0))
    _drive(sim, 0.0, 0.2, 0.1)
    assert sim.yaw_rate > 0.5 and sim.pose()[2] > 0.0


def test_closed_loop_delay_holds_the_command(mp: mass_properties.MassProperties) -> None:
    sim = ClosedLoopSim(mp, None, PlantParams(delay_s=0.1), start=(0.0, 0.0, 0.0))
    _drive(sim, 0.0, 0.0, 0.5)  # the hull-less test model starts level and rocks onto its skid
    sim.step(1.0, 0.0, 0.09)
    assert abs(sim.forward_speed) < 1e-3
    sim.step(1.0, 0.0, 0.05)
    assert sim.forward_speed > 0.05


def test_closed_loop_walls_and_blocks_stop_the_robot(mp: mass_properties.MassProperties) -> None:
    sim = ClosedLoopSim(mp, None, PlantParams(), start=(0.0, 0.0, 0.0), arena=(1.0, 1.0))
    _drive(sim, 0.5, 0.0, 1.5)
    assert sim.pose()[0] < 0.5
    assert sim.wall_contact and not sim.block_contact

    sim = ClosedLoopSim(mp, None, PlantParams(), start=(0.0, 0.0, 0.0), moving_block_radii=[0.1])
    sim.set_moving_blocks([(0.4, 0.0)])
    _drive(sim, 0.5, 0.0, 1.0)
    assert sim.pose()[0] < 0.3
    assert sim.block_contact and not sim.wall_contact

    sim = ClosedLoopSim(
        mp, None, PlantParams(), start=(0.0, 0.0, 0.0), blocks=[Disc(0.4, 0.0, 0.1)]
    )
    _drive(sim, 0.5, 0.0, 1.0)
    assert sim.pose()[0] < 0.3 and sim.block_contact


def test_closed_loop_heading_hold_fights_a_gain_mismatch(
    mp: mass_properties.MassProperties,
) -> None:
    drift = {}
    for auto_steer in (False, True):
        sim = ClosedLoopSim(
            mp, None, PlantParams(lr_gain_ratio=1.15), start=(0.0, 0.0, 0.0), auto_steer=auto_steer
        )
        _drive(sim, 0.15, 0.0, 1.0)
        drift[auto_steer] = abs(sim.pose()[2])
    assert drift[True] < 0.5 * drift[False]


def test_load_fit_params_takes_the_lowest_finite_loss(tmp_path: Path) -> None:
    runs = [
        {"loss": float("nan"), "best": params_dict(PlantParams(delay_s=0.01))},
        {"loss": 2.0, "best": params_dict(PlantParams(delay_s=0.02))},
        {"loss": 1.0, "best": params_dict(PlantParams(delay_s=0.04))},
    ]
    path = tmp_path / "fit.json"
    path.write_text(json.dumps({"runs": runs}))
    assert load_fit_params(path).delay_s == 0.04
