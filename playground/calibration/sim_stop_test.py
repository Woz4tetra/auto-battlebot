"""Closed-loop stop test: MotionProfileNavigation on Mr Stabs Mk2 against the MuJoCo plant.

The last check before a distilled [plant] table goes on the robot. The app plans its brake
schedule and feedforward from the [plant] table while the sim moves the robot with the fitted
MuJoCo model, so every error in the table shows up as a stop that lands short, overshoots, or
oscillates. Each arm is one [plant] table, and every arm drives the same goals.

Each run is the real binary (build/auto_battlebot) against kinematic_sim_server.py, headless,
one goal per run: a static "opponent" at the goal, attack mode with a zero terminal speed, so the
controller drives there and stops within stop_distance. Scored from the sim's own per-tick
trace, which is ground truth:

- stop_along_m: where the robot came to rest, along the final approach, relative to the goal.
  Negative is short. Anything in [-stop_distance, +stop_distance] counts as a stop on target.
- peak_along_m: furthest past the goal it got on the way. Overshoot that it later backed out of
  shows here and not in stop_along_m.
- arrive_s: first time inside stop_distance. settle_s: from then until speed stays under 0.05 m/s.
- reentries: how many times it left the stop circle again after arriving (oscillation).
- flipped: pitch left its rest value by more than 60 deg. Nothing else in the row means anything
  then. The flat [plant] table cannot predict a backflip, so a flipping arm needs a lower
  --max-linear-command, not a better table.

    python playground/calibration/sim_stop_test.py \\
        --fit-file playground/calibration/out/mujoco_sysid/fit_<ts>/fit.json \\
        --arm current=config/mr_stabs_mk2_zed_box.toml \\
        --arm distilled=playground/calibration/out/mujoco_sysid/fit_<ts>/distilled/plant.toml

Build first (scripts/build.sh). Runs are sequential: the C++ side has the sim port hard-coded.
Writes <out>/results.csv and one trace and log per run; --out defaults next to the fit file.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import tomli_w

from auto_battlebot.compat import tomllib
from auto_battlebot.control.plant import PlantParams

REPO_ROOT = Path(__file__).resolve().parents[2]
SERVER = REPO_ROOT / "simulation" / "kinematic_sim_server.py"
BINARY = REPO_ROOT / "build" / "auto_battlebot"
SIM_CONFIG = REPO_ROOT / "simulation" / "sim_mr_stabs_mk2.toml"
CPP_BASE = "simulation/mr_stabs_mk2_motion_profile_sim"
SIM_PORT = 14882  # hard-coded in src/simulation/sim_connection.cpp
SETTLED_SPEED = 0.05  # m/s
FLIP_DEG = 60.0


@dataclass(frozen=True)
class Case:
    """Start pose and goal, inside the 2.4 m sim arena and clear of the 0.25 m wall reversal."""

    name: str
    start: tuple[float, float, float]  # x, y, yaw deg
    goal: tuple[float, float]


CASES = (
    Case("straight_0.6m", (-0.9, -0.9, 45.0), (-0.476, -0.476)),
    Case("straight_1.2m", (-0.9, -0.9, 45.0), (-0.051, -0.051)),
    Case("straight_2.0m", (-0.9, -0.9, 45.0), (0.514, 0.514)),
    Case("turn_45deg_0.8m", (0.0, -0.4, 0.0), (0.566, 0.166)),
    Case("turn_90deg_0.8m", (0.0, -0.4, 0.0), (0.0, 0.4)),
    Case("turn_180deg_0.8m", (0.4, 0.0, 0.0), (-0.4, 0.0)),
)


def _port_open(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.2)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def _wait_for_port(port: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _port_open(port):
            return True
        time.sleep(0.1)
    return False


def write_sim_config(
    case: Case, fit_file: Path, radio_ms: float, seconds: float, trace: Path, path: Path
) -> None:
    cfg: dict[str, Any] = tomllib.loads(SIM_CONFIG.read_text())
    cfg["sim"].update(
        max_ticks=int(round(seconds / cfg["sim"]["dt"])), stop_on_outcome=True, trace_csv=str(trace)
    )
    cfg["viewer"] = {**cfg.get("viewer", {}), "enable": False}
    # The MuJoCo plant holds the drivetrain delay itself; the sim's own buffer is the radio part.
    cfg["latency"]["command_ms"] = radio_ms
    robot = cfg["our_robot"]
    robot["plant"] = "mujoco"
    robot["start_pos"] = [case.start[0], case.start[1]]
    robot["start_yaw_deg"] = case.start[2]
    robot["mujoco"]["fit_file"] = str(fit_file.resolve())
    opp = cfg["opponents"][0]
    opp.update(behavior="static", start_pos=list(case.goal), hazard_radius=0.0)
    cfg["opponents"] = [opp]
    with open(path, "wb") as handle:
        tomli_w.dump(cfg, handle)


def write_cpp_overlay(table: PlantParams, max_linear: float | None, path: Path) -> None:
    # Same type as the base, so these merge key by key: arrive stopped at the goal.
    nav: dict[str, Any] = {
        "type": "MotionProfileNavigation",
        "attack_terminal_speed_fraction": 0.0,
    }
    if max_linear is not None:
        nav["max_linear_command"] = max_linear
    data = {
        "extends": CPP_BASE,
        "navigation": nav,
        "plant": table.to_dict(),
        "ui": {"enable": False},
        "mcap": {"enable": False},
    }
    with open(path, "wb") as handle:
        tomli_w.dump(data, handle)


def run_once(sim_cfg: Path, cpp_cfg: Path, log: Path, timeout_s: float) -> bool:
    with open(log, "wb") as out:
        server = subprocess.Popen(
            [sys.executable, str(SERVER), str(sim_cfg)],
            cwd=REPO_ROOT,
            stdout=out,
            stderr=subprocess.STDOUT,
        )
        try:
            if not _wait_for_port(SIM_PORT, 30.0):
                raise RuntimeError(f"sim server did not start, see {log}")
            result = subprocess.run(
                [str(BINARY), "-c", str(cpp_cfg)],
                cwd=REPO_ROOT,
                stdout=out,
                stderr=subprocess.STDOUT,
                timeout=timeout_s,
                check=False,
            )
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            return False
        finally:
            server.terminate()
            try:
                server.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                server.kill()


def score_trace(path: Path, stop_distance: float) -> dict[str, float]:
    rows = np.genfromtxt(path, delimiter=",", names=True)
    if rows.size < 2:
        return {"ticks": float(rows.size)}
    t, x, y, v, pitch = rows["t"], rows["x"], rows["y"], rows["v"], rows["pitch"]
    gx, gy = float(rows["goal_x"][0]), float(rows["goal_y"][0])
    dist = np.hypot(gx - x, gy - y)
    # Approach direction: where the robot was when it got within 0.5 m (or its start).
    near = np.flatnonzero(dist < 0.5)
    k0 = int(near[0]) if len(near) else 0
    ux, uy = gx - x[k0], gy - y[k0]
    norm = math.hypot(ux, uy) or 1.0
    ux, uy = ux / norm, uy / norm
    along = (x - gx) * ux + (y - gy) * uy
    inside = dist < stop_distance
    arrived = np.flatnonzero(inside)
    out = {
        "ticks": float(len(t)),
        "stop_along_m": float(along[-1]),
        "stop_dist_m": float(dist[-1]),
        "peak_along_m": float(along[k0:].max()),
        "final_speed_m_s": float(abs(v[-1])),
        "peak_speed_m_s": float(np.abs(v).max()),
        "max_pitch_dev_deg": float(np.degrees(np.abs(pitch - pitch[0]).max())),
    }
    out["flipped"] = float(out["max_pitch_dev_deg"] > FLIP_DEG)
    if len(arrived) == 0:
        out.update(arrive_s=math.nan, settle_s=math.nan, reentries=math.nan)
        return out
    a = int(arrived[0])
    moving = np.flatnonzero(np.abs(v[a:]) > SETTLED_SPEED)
    settle = t[a + moving[-1]] - t[a] if len(moving) else 0.0
    exits = np.flatnonzero(np.diff(inside[a:].astype(int)) == 1)
    out.update(arrive_s=float(t[a]), settle_s=float(settle), reentries=float(len(exits)))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--fit-file", type=Path, required=True, help="fit_mujoco_plant.py fit.json")
    ap.add_argument(
        "--arm", action="append", required=True, help="name=path to a TOML with a [plant] table"
    )
    ap.add_argument("--radio-delay-ms", type=float, default=None, help="default: from the arm's")
    ap.add_argument("--seconds", type=float, default=8.0, help="sim time per run")
    ap.add_argument("--stop-distance", type=float, default=0.15, help="match [navigation]")
    ap.add_argument(
        "--max-linear-command",
        type=float,
        default=None,
        help="override [navigation] max_linear_command (0.8 inherited)",
    )
    ap.add_argument("--timeout", type=float, default=300.0, help="wall seconds per run")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    if not BINARY.exists():
        raise SystemExit(f"{BINARY} not built; run scripts/build.sh")
    if _port_open(SIM_PORT):
        raise SystemExit(f"port {SIM_PORT} is in use: stop the running sim first")
    arms: dict[str, PlantParams] = {}
    for spec in args.arm:
        name, _, path = spec.partition("=")
        arms[name] = PlantParams.from_toml(REPO_ROOT / path)
    radio_ms = args.radio_delay_ms
    if radio_ms is None:
        distill = args.fit_file.parent / "distilled" / "distill.json"
        if not distill.exists():
            raise SystemExit("pass --radio-delay-ms (no distilled/distill.json to read it from)")
        radio_ms = 1000.0 * float(json.loads(distill.read_text())["radio_delay_s"])
    out = args.out or args.fit_file.parent / "stop_test"
    out.mkdir(parents=True, exist_ok=True)
    print(f"radio delay {radio_ms:.1f} ms, {len(arms)} arms x {len(CASES)} goals -> {out}")

    results = []
    for arm, table in arms.items():
        for case in CASES:
            tag = f"{arm}__{case.name}"
            sim_cfg, cpp_cfg = out / f"{tag}.sim.toml", out / f"{tag}.cpp.toml"
            trace, log = out / f"{tag}.trace.csv", out / f"{tag}.log"
            trace.unlink(missing_ok=True)
            write_sim_config(case, args.fit_file, radio_ms, args.seconds, trace, sim_cfg)
            write_cpp_overlay(table, args.max_linear_command, cpp_cfg)
            start = time.monotonic()
            clean = run_once(sim_cfg, cpp_cfg, log, args.timeout)
            row: dict[str, Any] = {"arm": arm, "case": case.name, "clean_exit": clean}
            if trace.exists():
                row.update(score_trace(trace, args.stop_distance))
            results.append(row)
            print(
                f"  {tag:40} {time.monotonic() - start:5.1f}s"
                f"  stop {row.get('stop_along_m', math.nan):+.3f} m"
                f"  peak {row.get('peak_along_m', math.nan):+.3f} m"
                f"  arrive {row.get('arrive_s', math.nan):.2f} s"
                f"  reentries {row.get('reentries', math.nan):.0f}"
                + ("  FLIPPED" if row.get("flipped") else "")
                + ("" if clean else f"  (unclean exit, see {log.name})")
            )

    keys = sorted({k for r in results for k in r}, key=lambda k: (k not in ("arm", "case"), k))
    with open(out / "results.csv", "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(results)

    print(
        f"\n{'arm':12}{'on target':>10}{'|stop| mean':>13}{'peak past':>11}{'arrive':>9}"
        f"{'flipped':>9}"
    )
    for arm in arms:
        rows = [r for r in results if r["arm"] == arm and "stop_along_m" in r]
        flips = sum(int(r.get("flipped", 0)) for r in rows)
        if not rows:
            print(f"{arm:12} no traces")
            continue
        stop = np.array([r["stop_along_m"] for r in rows])
        hit = int(np.sum(np.abs(stop) <= args.stop_distance))
        print(
            f"{arm:12}{hit:>6}/{len(rows):<3}{np.mean(np.abs(stop)):11.3f} m"
            f"{max(r['peak_along_m'] for r in rows):9.3f} m"
            f"{np.nanmean([r['arrive_s'] for r in rows]):7.2f} s"
            f"{flips:>9}"
        )
    print(f"wrote {out / 'results.csv'}")


if __name__ == "__main__":
    main()
