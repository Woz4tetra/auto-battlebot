"""Render one venue's domain-mix dataset across several GPUs, one view per run, then merge it.

Step 3 of docs/experiments/perception_performance/synthetic_domain_mix_plan_2026-09-12.md. The GPU
queue runs one job at a time, so this is that one job: one worker per GPU renders its shard as one
``render_scenes.py`` container per view, back to back, and once every run has finished the runs are
hardlinked into one flat dataset.

    venv/bin/python training/gpu_queue.py submit --name render_cage_nhrl --by <agent> -d 0 1 2 -- \\
      venv/bin/python training/synthetic/render_shards.py config_cage_nhrl.toml \\
        --out ../data/synth_cage_nhrl_2026-09-13 --total 20000 --gpus 0 1 2 --seed-base 0

``render_scenes.py`` runs inside the container, whose working directory is training/synthetic, so
the config and ``--out`` are relative to that directory. Anything after a bare ``--`` goes to every
``render_scenes.py`` run, for smoke tests (``-- --images-per-scene 2 --damage all``).

Runs write to ``<out>_parts/shard<i>_<view>/``. A rerun resumes: a finished run is skipped, a run a
crash cut short continues one past its last frame on disk under a fresh seed, and the merge happens
only once every run is complete. The parts stay after the merge; the dataset's files are hardlinks
to them, so deleting the parts costs nothing but the directory entries.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

from synthgen.constants import VIEW_DISTORTED, VIEW_PINHOLE, VIEW_RECTIFIED, VIEWS
from synthgen.shards import (
    ShardRun,
    existing_frames,
    merge_parts,
    plan_shard_runs,
    resume_point,
)

SYNTHETIC_DIR = Path(__file__).resolve().parent
RUN_SYNTHETIC = SYNTHETIC_DIR / "docker" / "run_synthetic.sh"
DEFAULT_IMAGE = "auto-battlebot-synthetic"
# The view that runs one frame short comes last. NHRL keeps this order; MassD passes
# `--views rectified distorted pinhole` so the two venues are short on different views.
DEFAULT_VIEW_ORDER = (VIEW_PINHOLE, VIEW_RECTIFIED, VIEW_DISTORTED)
# A resumed run reseeds so it does not replay the scenes its first attempt already drew. The
# stride keeps resumed seeds clear of every run's base seed.
RESUME_SEED_STRIDE = 100_000
PROGRESS_SECONDS = 600
SUMMARY_NAME = "render_shards.json"

_print_lock = threading.Lock()
_children: list[subprocess.Popen] = []
_stopping = threading.Event()


def log(message: str) -> None:
    """One timestamped line to stdout, which the GPU queue keeps as the job log."""
    with _print_lock:
        print(f"[{time.strftime('%Y-%m-%dT%H:%M:%S')}] {message}", flush=True)


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    """The script's own arguments, and the ones after ``--`` that go to render_scenes.py."""
    if "--" in argv:
        split = argv.index("--")
        own, passthrough = argv[:split], argv[split + 1 :]
    else:
        own, passthrough = argv, []
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("config", help="Render config, relative to training/synthetic")
    parser.add_argument(
        "--out", type=Path, required=True, help="Merged dataset, relative to training/synthetic"
    )
    parser.add_argument("--total", type=int, required=True, help="Frames across every run")
    parser.add_argument(
        "--gpus", type=int, nargs="+", default=[0, 1, 2], help="One shard per GPU, host numbering"
    )
    parser.add_argument(
        "--views",
        nargs="+",
        choices=VIEWS,
        default=list(DEFAULT_VIEW_ORDER),
        help="Views each shard renders, in order; the last view runs short on an uneven split",
    )
    parser.add_argument("--seed-base", type=int, default=0, help="Seed of the first run")
    parser.add_argument("--image", default=DEFAULT_IMAGE, help="Synthetic docker image")
    parser.add_argument("--dry-run", action="store_true", help="Print the plan and exit")
    return parser.parse_args(own), passthrough


def host_path(relative: Path) -> Path:
    """A path given relative to the container's working directory, on the host."""
    return (SYNTHETIC_DIR / relative).resolve()


def render_command(
    run: ShardRun, start: int, count: int, config: str, container_part: Path, image: str
) -> list[str]:
    """The docker invocation for the remaining frames of *run*."""
    seed = run.seed + RESUME_SEED_STRIDE * (start - run.start_index)
    return [
        "bash",
        str(RUN_SYNTHETIC),
        "--require-gpu",
        image,
        "blenderproc",
        "run",
        "render_scenes.py",
        "--",
        config,
        "--num-images",
        str(count),
        "--start-index",
        str(start),
        "--seed",
        str(seed),
        "--view",
        run.view,
        "--out",
        str(container_part),
    ]


def render_shard(
    runs: list[ShardRun],
    gpu: int,
    args: argparse.Namespace,
    passthrough: list[str],
    parts_dir: Path,
    timings: dict[str, dict[str, Any]],
    failures: list[str],
) -> None:
    """Render one shard's runs back to back on one GPU."""
    for run in runs:
        if _stopping.is_set():
            return
        part = parts_dir / run.name
        start, count = resume_point(existing_frames(part / "images", run), run)
        if count <= 0:
            log(f"{run.name}: complete ({run.num_images} frames), skipping")
            continue
        part.mkdir(parents=True, exist_ok=True)
        container_part = args.out.parent / parts_dir.name / run.name
        cmd = render_command(run, start, count, args.config, container_part, args.image)
        cmd += passthrough
        env = {**os.environ, "CUDA_VISIBLE_DEVICES": str(gpu)}
        resumed = "" if start == run.start_index else f", resuming at {start}"
        log(f"{run.name}: GPU {gpu}, {count} frames from {start}{resumed}")
        began = time.monotonic()
        with (part / "render.log").open("a", encoding="utf-8") as handle:
            handle.write(f"# {' '.join(cmd)}\n")
            handle.flush()
            proc = subprocess.Popen(cmd, env=env, stdout=handle, stderr=subprocess.STDOUT)
            _children.append(proc)
            code = proc.wait()
        seconds = time.monotonic() - began
        written = len(existing_frames(part / "images", run)) - (start - run.start_index)
        per_frame = seconds / written if written > 0 else float("nan")
        timings[run.name] = {
            "gpu": gpu,
            "frames": written,
            "seconds": round(seconds, 1),
            "seconds_per_frame": round(per_frame, 3),
            "exit": code,
        }
        log(
            f"{run.name}: exit {code}, {written} frames in {seconds:.0f} s"
            f" ({per_frame:.2f} s/frame)"
        )
        if code != 0:
            failures.append(f"{run.name} exited {code}; see {part / 'render.log'}")
            return


def report_progress(runs: list[ShardRun], parts_dir: Path, done: threading.Event) -> None:
    """Frames on disk per view, every PROGRESS_SECONDS, so `gpu_queue.py logs` shows movement."""
    total = sum(run.num_images for run in runs)
    while not done.wait(PROGRESS_SECONDS):
        per_view: dict[str, int] = {}
        for run in runs:
            found = len(existing_frames(parts_dir / run.name / "images", run))
            per_view[run.view] = per_view.get(run.view, 0) + found
        written = sum(per_view.values())
        views = ", ".join(f"{view} {count}" for view, count in per_view.items())
        log(f"progress: {written}/{total} frames ({views})")


def record_source(parts_dir: Path) -> Path:
    """The commit and uncommitted training/synthetic diff this attempt renders from.

    The containers mount the repo live, so a run started hours in reads whatever synthgen holds
    then. The patch is what makes the render traceable when the tree was not committed first.
    """
    parts_dir.mkdir(parents=True, exist_ok=True)

    def git(*args: str) -> str:
        return subprocess.run(
            ["git", *args], cwd=SYNTHETIC_DIR, capture_output=True, text=True, check=False
        ).stdout

    untracked = git("ls-files", "--others", "--exclude-standard", "--", ".").split()
    patch = parts_dir / f"source_{time.strftime('%Y-%m-%dT%H-%M-%S')}.patch"
    header = [f"# HEAD {git('rev-parse', 'HEAD').strip()}"]
    header += [f"# untracked: {name}" for name in untracked]
    patch.write_text("\n".join(header) + "\n" + git("diff", "HEAD", "--", "."), encoding="utf-8")
    return patch


def stop_children(signum: int, _frame: object) -> None:
    """Pass a queue cancel on to the containers instead of orphaning them."""
    _stopping.set()
    log(f"signal {signum}: stopping {len(_children)} render(s)")
    for proc in _children:
        if proc.poll() is None:
            proc.terminate()


def render_all(
    runs: list[ShardRun], args: argparse.Namespace, passthrough: list[str], parts_dir: Path
) -> tuple[dict[str, dict[str, Any]], list[str], float]:
    """Every shard at once, one worker thread per GPU: (timings, failures, wall seconds)."""
    timings: dict[str, dict[str, Any]] = {}
    failures: list[str] = []
    done = threading.Event()
    progress = threading.Thread(target=report_progress, args=(runs, parts_dir, done), daemon=True)
    progress.start()
    began = time.monotonic()
    workers = [
        threading.Thread(
            target=render_shard,
            args=(
                [run for run in runs if run.shard == shard],
                gpu,
                args,
                passthrough,
                parts_dir,
                timings,
                failures,
            ),
        )
        for shard, gpu in enumerate(args.gpus)
    ]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join()
    done.set()
    return timings, failures, time.monotonic() - began


def log_plan(args: argparse.Namespace, runs: list[ShardRun]) -> None:
    """The runs this job will make, first thing in the queue log."""
    log(f"{args.config}: {args.total} frames over GPUs {args.gpus}, views {args.views}")
    for run in runs:
        log(
            f"  {run.name}: frames {run.start_index}..{run.end_index - 1}"
            f" ({run.num_images}), seed {run.seed}"
        )


def main() -> int:
    args, passthrough = parse_args(sys.argv[1:])
    if len(set(args.gpus)) != len(args.gpus):
        raise SystemExit(f"--gpus repeats a GPU: {args.gpus}")
    runs = plan_shard_runs(args.total, len(args.gpus), args.views, args.seed_base)
    out = host_path(args.out)
    parts_dir = out.parent / f"{out.name}_parts"
    log_plan(args, runs)
    if args.dry_run:
        return 0
    if (out / "images").exists():
        raise SystemExit(f"{out} already holds a merged dataset; move it aside first")
    patch = record_source(parts_dir)
    log(f"source recorded in {patch}; containers read training/synthetic live until this ends")

    signal.signal(signal.SIGTERM, stop_children)
    signal.signal(signal.SIGINT, stop_children)
    timings, failures, wall = render_all(runs, args, passthrough, parts_dir)
    if _stopping.is_set():
        log("stopped before finishing; rerun the same command to resume")
        return 1
    for failure in failures:
        log(f"FAILED {failure}")
    if failures:
        return 1
    short = [
        run.name
        for run in runs
        if len(existing_frames(parts_dir / run.name / "images", run)) != run.num_images
    ]
    if short:
        log(f"runs short of their frame count: {short}; rerun to resume")
        return 1

    summary = merge_parts([parts_dir / run.name for run in runs if run.num_images > 0], out)
    log(f"merged {summary.images} frames into {out}: {summary.per_view}")
    record = {
        "config": args.config,
        "command": sys.argv,
        "gpus": args.gpus,
        "wall_seconds": round(wall, 1),
        "runs": [asdict(run) | timings.get(run.name, {}) for run in runs],
        "merged": asdict(summary),
    }
    (out / SUMMARY_NAME).write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
