"""Score a C++ navigation on exported RL evaluation scenarios in the lockstep sim (gates G4, G5).

For each scenario: write a hazards file and a C++ config overlay (arena size, hazards, measurement
labels, behaviour mode, navigation), start `simulation/rl_eval_server.py` on its own port, run the
release binary against it, and collect the episode score the server appends to the results file.
Episodes run in parallel, one server and one binary per worker.

    source scripts/activate_python.sh
    # export 128 held-out evaluation scenarios from the Warp environment
    python playground/rl/lockstep_eval.py export --count 128 --seed 2026 \\
        --out playground/rl/out/scenarios.json
    # score MotionProfileNavigation and a trained bundle on them
    python playground/rl/lockstep_eval.py run playground/rl/out/scenarios.json --nav motion_profile
    python playground/rl/lockstep_eval.py run playground/rl/out/scenarios.json --nav rl \\
        --bundle <dir>

`export --robot mrs_buff_mk3` exports Mrs Buff Mk3 episodes; `run` reads the robot from the
scenarios and runs the binary on that robot's sim profile (`BASE_CONFIG`). Scenarios that drew
a policy-driven opponent (`policy_opponents.py`) replay it.

`--record <dir>` keeps each episode's work directory under `<dir>/ep<index>` and records an MCAP
there. The binary writes recordings to `<cwd>/data/recordings`, so it runs from that directory
with a `config` symlink, which keeps the recordings out of the repo's `data/`.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import subprocess
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from auto_battlebot.rl import obs_spec

REPO = Path(__file__).resolve().parents[2]
BINARY = REPO / "build/auto_battlebot"
BASE_CONFIG = {
    "mr_stabs_mk2": "simulation/mr_stabs_mk2_kinematic_sim_sensing",
    "mrs_buff_mk3": "simulation/mrs_buff_mk3_kinematic_sim_sensing",
}
OUR_LABEL = {"mr_stabs_mk2": "MR_STABS_MK2", "mrs_buff_mk3": "MRS_BUFF_MK3"}
MOTION_PROFILE = """[navigation]
type = "MotionProfileNavigation"
stop_distance = 0.15
attack_terminal_speed_fraction = 1.0
run_away_terminal_speed_fraction = 0.0
wall_reverse_distance = 0.25
wall_heading_threshold = 1.047
wall_reverse_min_speed = 0.25
max_linear_command = 0.8
max_angular_command = 1.0
"""


def export(
    count: int, seed: int, out: Path, episode_s: float, pursuit_grid: int, robot: str
) -> None:
    """`pursuit_grid` > 0 exports that many pursuit-only attack episodes per cell of
    `PURSUIT_GRID` instead of the `EVAL` mix."""
    import torch

    from auto_battlebot.rl.envs.opponents import PURSUIT_GRID
    from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv

    if pursuit_grid > 0:
        count = len(PURSUIT_GRID) * pursuit_grid
        cfg = WarpNavConfig(worlds=count, stage="PURSUIT", episode_s=episode_s, robot=robot)
    else:
        cfg = WarpNavConfig(
            worlds=count, stage="EVAL", holdout=True, episode_s=episode_s, robot=robot
        )
    env = WarpNavEnv(cfg, "cuda", seed=seed)
    if pursuit_grid > 0:
        grid = torch.tensor(PURSUIT_GRID, device="cuda", dtype=torch.float32)
        env.opponents.pursuit_fixed = grid.repeat_interleave(pursuit_grid, 0)
    env.reset(seed)
    scenarios = env.export_scenarios()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(scenarios, indent=1))
    print(f"wrote {len(scenarios)} scenarios to {out}")


def overlay(
    scenario: dict, nav: str, bundle: Path | None, port: int, workdir: Path, barrier: bool = False
) -> Path:
    hazards = workdir / "hazards.toml"
    lines = []
    for s in scenario["slots"]:
        if s["present"]:
            kind = "hole" if s["kind"] == obs_spec.HOLE else "wall_block"
            lines += [
                "[[hazards]]",
                f'kind = "{kind}"',
                f"center = [{s['x']:.6f}, {s['y']:.6f}]",
                f"radius = {s['r']:.6f}",
                "",
            ]
    hazards.write_text("\n".join(lines) or "hazards = []\n")
    robot = str(scenario.get("robot", "mr_stabs_mk2"))
    labels = [f'"{OUR_LABEL[robot]}"', '"OPPONENT"'] + (
        ['"HOUSE_BOT"'] if scenario["house"]["present"] else []
    )
    mode = "RUN_AWAY" if scenario["run_away"] > 0.5 else "ATTACK"
    size = scenario["opp_size"]
    text = f"""extends = "{BASE_CONFIG[robot]}"

[rgbd_camera]
sim_port = {port}

[field_filter]
size_x = {2 * scenario["arena_half"][0]:.6f}
size_y = {2 * scenario["arena_half"][1]:.6f}
hazards_file = "{hazards}"

[keypoint_model]
labels = [{", ".join(labels)}]

[runner]
default_behavior_mode = "{mode}"

# The sim sends front and back keypoints and no box, so without this the filter sizes the
# opponent from the keypoint spacing (about 0.1 m) instead of its footprint, which the Warp
# environment and a detector's box both report.
[robot_filter.robot_size_meters_per_label]
OPPONENT = [{size[0]:.4f}, {size[1]:.4f}, {size[2]:.4f}]
"""
    if nav == "motion_profile":
        text += "\n" + MOTION_PROFILE
    elif nav == "rl":
        assert bundle is not None
        text += (
            '\n[navigation]\ntype = "RlPolicyNavigation"\n'
            + ("hazard_barrier_enable = true\n" if barrier else "")
            + f'\n[navigation.policy]\nbundle = "{bundle.resolve()}"\n'
        )
    elif nav != "pursuit":
        raise ValueError(nav)
    path = workdir / "overlay.toml"
    path.write_text(text)
    return path


Job = tuple[Path, int, str, Path | None, Path, int, Path | None, str, bool]
RECORD = '\n[mcap]\nenable = true\nignored_topics = ["/camera/image"]\n'


def work_dir(index: int, record: Path | None) -> contextlib.AbstractContextManager[str]:
    if record is None:
        return tempfile.TemporaryDirectory(prefix="rl_eval_")
    work = record / f"ep{index:03d}"
    (work / "data/recordings").mkdir(parents=True, exist_ok=True)
    link = work / "config"
    if not link.exists():
        link.symlink_to(REPO / "config")
    return contextlib.nullcontext(str(work))


def run_one(args: Job) -> str:
    scenarios, index, nav, bundle, results, port, record, extra, barrier = args
    scenario = json.loads(scenarios.read_text())[index]
    with work_dir(index, record) as tmp:
        work = Path(tmp)
        cfg = overlay(scenario, nav, bundle, port, work, barrier)
        if record is not None:
            cfg.write_text(cfg.read_text() + RECORD)
        if extra:
            cfg.write_text(cfg.read_text() + "\n" + extra)
        # One thread per server: torch and BLAS pools otherwise spin about 7 cores per episode.
        threads = {k: "1" for k in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")}
        env = {**os.environ, "PYTHONUNBUFFERED": "1", **threads}
        server = subprocess.Popen(
            [
                str(REPO / "venv/bin/python"),
                str(REPO / "simulation/rl_eval_server.py"),
                "--scenarios",
                str(scenarios),
                "--index",
                str(index),
                "--port",
                str(port),
                "--results",
                str(results),
                "--label",
                nav,
            ],
            cwd=REPO,
            env=env,
            stdout=open(work / "server.log", "w"),
            stderr=subprocess.STDOUT,
        )
        log = work / "server.log"
        for _ in range(600):
            if "ready on" in log.read_text() or server.poll() is not None:
                break
            time.sleep(0.05)
        client = subprocess.Popen(
            [str(BINARY), "-c", str(cfg)],
            cwd=REPO if record is None else work,
            stdout=open(work / "client.log", "w"),
            stderr=subprocess.STDOUT,
        )
        try:
            server.wait(timeout=300)
        except subprocess.TimeoutExpired:
            server.kill()
        try:
            client.wait(timeout=20)
        except subprocess.TimeoutExpired:
            client.kill()
        tail = (work / "client.log").read_text()[-2000:]
        if server.returncode != 0:
            return f"{index}: server exit {server.returncode}\n{log.read_text()[-1500:]}\n{tail}"
        return f"{index}: ok"


def run(
    scenarios: Path,
    nav: str,
    bundle: Path | None,
    out: Path,
    workers: int,
    limit: int | None,
    port_base: int,
    record: Path | None = None,
    extra: Path | None = None,
    barrier: bool = False,
) -> None:
    count = len(json.loads(scenarios.read_text()))
    indices = list(range(count if limit is None else min(count, limit)))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.unlink(missing_ok=True)  # the server appends; a rerun must not stack on old results
    # The binary runs from the record directory, so every path it is handed must be absolute.
    record = record.resolve() if record is not None else None
    text = extra.read_text() if extra is not None else ""
    jobs = [
        (scenarios, i, nav, bundle, out, port_base + (i % workers), record, text, barrier)
        for i in indices
    ]
    # Ports are reused by worker slot; a slot runs its jobs one after another.
    slots: list[list[Job]] = [[] for _ in range(workers)]
    for i, job in enumerate(jobs):
        slots[i % workers].append(job)

    def worker(batch: list[Job]) -> list[str]:
        return [run_one(job) for job in batch]

    start = time.time()
    with ThreadPoolExecutor(workers) as pool:
        for lines in pool.map(worker, slots):
            for line in lines:
                if not line.endswith("ok"):
                    print(line)
    print(f"{len(jobs)} episodes in {time.time() - start:.0f} s -> {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export")
    e.add_argument("--count", type=int, default=128)
    e.add_argument("--seed", type=int, default=2026)
    e.add_argument("--out", type=Path, default=REPO / "playground/rl/out/scenarios.json")
    e.add_argument(
        "--pursuit-grid",
        type=int,
        default=0,
        help="episodes per pursuit grid cell; exports the pursuit evaluation set instead",
    )
    e.add_argument("--episode-s", type=float, default=20.0, help="time limit per episode")
    e.add_argument("--robot", default="mr_stabs_mk2", choices=list(BASE_CONFIG))
    r = sub.add_parser("run")
    r.add_argument("scenarios", type=Path)
    r.add_argument("--nav", choices=["motion_profile", "rl", "pursuit"], required=True)
    r.add_argument("--bundle", type=Path, default=None)
    r.add_argument("--out", type=Path, default=None)
    r.add_argument("--workers", type=int, default=6)
    r.add_argument("--limit", type=int, default=None)
    r.add_argument("--record", type=Path, default=None, help="keep work dirs and record MCAPs")
    r.add_argument(
        "--port-base", type=int, default=15000, help="two runs at once need distinct bases"
    )
    r.add_argument(
        "--extra-config",
        type=Path,
        default=None,
        help="TOML appended to every episode's overlay, e.g. a [plant] table to score against",
    )
    r.add_argument(
        "--hazard-barrier",
        action="store_true",
        help="--nav rl: run the motion profile's hazard velocity barrier on the policy command",
    )
    args = ap.parse_args()
    if args.cmd == "export":
        export(args.count, args.seed, args.out, args.episode_s, args.pursuit_grid, args.robot)
    else:
        out = args.out or args.scenarios.parent / f"lockstep_{args.nav}.jsonl"
        run(
            args.scenarios,
            args.nav,
            args.bundle,
            out,
            args.workers,
            args.limit,
            args.port_base,
            args.record,
            args.extra_config,
            args.hazard_barrier,
        )


if __name__ == "__main__":
    main()
