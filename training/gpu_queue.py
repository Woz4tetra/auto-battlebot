#!/usr/bin/env python3
"""Serial GPU job queue, so several agents can share megamind's three A6000s.

Every training arm in the experiment plans runs DDP across all three GPUs, so the
scheduling unit is the whole box: one job at a time, highest priority first, FIFO
within a priority. Agents submit and poll rather than launching training directly.

    training/gpu_queue.py submit --name B_s384x640 -- \\
        venv/bin/python training/yolo/train.py training/data/... yolo26s -d 0 1 2 -b 96
    training/gpu_queue.py status
    training/gpu_queue.py logs 3 --tail 40
    training/gpu_queue.py logs -f            # follow whatever is training, across jobs

A worker process pops jobs; `submit` starts one if none is alive. The worker waits
for the GPUs to go idle before each job, so a run launched outside the queue (by
hand or by an agent that has not adopted it) delays the queue instead of colliding
with it.
"""

import argparse
import contextlib
import fcntl
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from collections.abc import Iterator
from datetime import datetime
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
QUEUE_DIR = REPO_ROOT / "runs" / "queue"
STATE_PATH = QUEUE_DIR / "state.json"
STATE_LOCK = QUEUE_DIR / "state.lock"
WORKER_LOCK = QUEUE_DIR / "worker.lock"
LOG_DIR = QUEUE_DIR / "logs"

# A compute process holding more than this is treated as somebody using the GPUs.
# Sunshine's desktop streamer sits at ~260 MiB and must not count as busy.
BUSY_MIB = int(os.environ.get("AB_GPU_QUEUE_BUSY_MIB", "1024"))
POLL_SECONDS = 10
TERMINAL_STATES = ("done", "failed", "cancelled")
# Never inherited from the agent that started the worker; see job_env.
UNSAFE_ENV = frozenset(
    {
        "CUDA_VISIBLE_DEVICES",
        "NVIDIA_VISIBLE_DEVICES",
        "PYTHONPATH",
        "AB_GPU_QUEUE_BUSY_MIB",
    }
)


def now() -> str:
    return datetime.now().isoformat(timespec="seconds")


@contextlib.contextmanager
def locked_state() -> Iterator[dict[str, Any]]:
    """Read-modify-write state.json under an exclusive flock."""
    QUEUE_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    with open(STATE_LOCK, "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if STATE_PATH.exists():
            state = json.loads(STATE_PATH.read_text())
        else:
            state = {"next_id": 1, "jobs": [], "worker_stop": False}
        yield state
        tmp = STATE_PATH.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(state, indent=2))
        tmp.replace(STATE_PATH)


def read_state() -> dict[str, Any]:
    if not STATE_PATH.exists():
        return {"next_id": 1, "jobs": [], "worker_stop": False}
    return json.loads(STATE_PATH.read_text())


def find_job(state: dict[str, Any], job_id: int) -> dict[str, Any] | None:
    return next((job for job in state["jobs"] if job["id"] == job_id), None)


def gpu_busy_processes(ignore_pids: set[int]) -> list[tuple[int, int]]:
    """Compute processes holding real memory, excluding our own job's pids."""
    try:
        out = subprocess.run(
            [
                "nvidia-smi",
                "--query-compute-apps=pid,used_memory",
                "--format=csv,noheader,nounits",
            ],
            capture_output=True,
            text=True,
            timeout=30,
            check=True,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        # If nvidia-smi cannot be read, assume busy rather than stacking a second run.
        return [(-1, -1)]
    busy = []
    for line in out.splitlines():
        if not line.strip():
            continue
        pid_text, _, mem_text = line.partition(",")
        try:
            pid, mib = int(pid_text.strip()), int(mem_text.strip())
        except ValueError:
            continue
        if pid not in ignore_pids and mib >= BUSY_MIB:
            busy.append((pid, mib))
    return busy


def worker_alive() -> bool:
    """True if some process holds the worker lock."""
    QUEUE_DIR.mkdir(parents=True, exist_ok=True)
    with open(WORKER_LOCK, "w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        fcntl.flock(lock, fcntl.LOCK_UN)
        return False


def spawn_worker() -> None:
    """Start a detached worker. A duplicate exits on its own; the lock decides."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    with open(QUEUE_DIR / "worker.log", "a") as log:
        subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "worker"],
            cwd=REPO_ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )


def cmd_submit(args: argparse.Namespace) -> int:
    if not args.command:
        print("nothing to run: put the command after --", file=sys.stderr)
        return 2
    devices = list(args.devices)
    env: dict[str, str] = {}
    if len(devices) > 1:
        # NCCL peer-to-peer is broken on this box; DDP hangs silently without this.
        env["NCCL_P2P_DISABLE"] = "1"
    with locked_state() as state:
        job = {
            "id": state["next_id"],
            "name": args.name,
            "command": args.command,
            "cwd": str(Path(args.cwd).resolve()),
            "env": env,
            "devices": devices,
            "priority": args.priority,
            "submitted_by": args.by or os.environ.get("CLAUDE_AGENT_NAME", "unknown"),
            "submitted_at": now(),
            "state": "queued",
            "started_at": None,
            "finished_at": None,
            "exit_code": None,
            "pid": None,
            "log": None,
            "cancel_requested": False,
        }
        state["next_id"] += 1
        state["jobs"].append(job)
        state["worker_stop"] = False
        # Test liveness while still holding the state lock. An idle worker decides to
        # exit under the same lock, so it cannot see an empty queue and then die after
        # we have both appended a job and observed it alive.
        needs_worker = not worker_alive()
    print(f"queued job {job['id']} ({job['name']})")
    if needs_worker:
        spawn_worker()
        print("started worker")
    return 0


def next_queued(state: dict[str, Any]) -> dict[str, Any] | None:
    queued = [job for job in state["jobs"] if job["state"] == "queued"]
    if not queued:
        return None
    return sorted(queued, key=lambda job: (-job["priority"], job["id"]))[0]


def job_env(job: dict[str, Any]) -> dict[str, str]:
    """Environment for a job, with the spawning agent's device settings stripped.

    The worker inherits the environment of whichever agent happened to start it, and
    then hands that environment to every later job from every other agent. A stray
    CUDA_VISIBLE_DEVICES or PYTHONPATH would silently override `-d 0 1 2` hours later,
    so drop those and let the job's own env win.
    """
    inherited = {key: value for key, value in os.environ.items() if key not in UNSAFE_ENV}
    return inherited | job["env"]


def run_job(job: dict[str, Any]) -> None:
    """Run one job to completion, honouring a cancel request mid-flight."""
    log_path = LOG_DIR / f"{job['id']:04d}-{job['name']}.log"
    env = job_env(job)
    with open(log_path, "w") as log:
        log.write(f"# {shlex.join(job['command'])}\n# started {now()}\n\n")
        log.flush()
        proc = subprocess.Popen(
            job["command"],
            cwd=job["cwd"],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
    with locked_state() as state:
        record = find_job(state, job["id"])
        if record is not None:
            record.update(state="running", started_at=now(), pid=proc.pid, log=str(log_path))

    killed = False
    while proc.poll() is None:
        time.sleep(POLL_SECONDS)
        record = find_job(read_state(), job["id"])
        if record is not None and record["cancel_requested"] and not killed:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(proc.pid, signal.SIGTERM)
            killed = True

    with locked_state() as state:
        record = find_job(state, job["id"])
        if record is None:
            return
        if killed:
            record["state"] = "cancelled"
        else:
            record["state"] = "done" if proc.returncode == 0 else "failed"
        record.update(finished_at=now(), exit_code=proc.returncode, pid=None)


def wait_for_gpus(job_id: int) -> bool:
    """Block until no outside process is using the GPUs. False if the job vanished."""
    announced = False
    while True:
        record = find_job(read_state(), job_id)
        if record is None or record["state"] != "queued" or record["cancel_requested"]:
            return False
        busy = gpu_busy_processes(ignore_pids=set())
        if not busy:
            return True
        if not announced:
            pids = ", ".join(f"pid {pid} ({mib} MiB)" for pid, mib in busy)
            print(f"[{now()}] job {job_id} waiting on GPUs held by {pids}", flush=True)
            announced = True
        time.sleep(POLL_SECONDS)


def exit_if_idle() -> None:
    """Exit the worker if the queue is still empty, holding the state lock until death.

    A submitter appends its job and tests worker liveness under the same lock, so
    ending the process here — rather than after releasing — closes the window where it
    would enqueue a job, see this worker as alive, and skip spawning a replacement.
    os._exit skips the state write-back, which is correct: nothing was modified.
    """
    with locked_state() as state:
        if next_queued(state) is not None:
            return
        print(f"[{now()}] worker idle, exiting", flush=True)
        sys.stdout.flush()
        os._exit(0)


def cmd_worker(args: argparse.Namespace) -> int:
    QUEUE_DIR.mkdir(parents=True, exist_ok=True)
    lock = open(WORKER_LOCK, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("another worker is already running", file=sys.stderr)
        return 0
    print(f"[{now()}] worker up (pid {os.getpid()})", flush=True)
    idle_since = time.monotonic()
    while True:
        state = read_state()
        if state.get("worker_stop"):
            print(f"[{now()}] worker stopping on request", flush=True)
            return 0
        job = next_queued(state)
        if job is None:
            if args.idle_exit and time.monotonic() - idle_since > args.idle_exit:
                exit_if_idle()
            time.sleep(POLL_SECONDS)
            continue
        idle_since = time.monotonic()
        if not wait_for_gpus(job["id"]):
            continue
        with locked_state() as fresh:
            record = find_job(fresh, job["id"])
            if record is None or record["state"] != "queued":
                continue
            if record["cancel_requested"]:
                record.update(state="cancelled", finished_at=now())
                continue
            record["state"] = "claimed"
        print(f"[{now()}] job {job['id']} ({job['name']}) starting", flush=True)
        run_job(job)
        final = find_job(read_state(), job["id"])
        print(f"[{now()}] job {job['id']} {final['state'] if final else 'gone'}", flush=True)


def format_row(job: dict[str, Any]) -> str:
    when = job["started_at"] or job["submitted_at"]
    extra = "" if job["exit_code"] is None else f" exit={job['exit_code']}"
    return (
        f"{job['id']:>4}  {job['state']:<9} {job['name']:<28} "
        f"{job['submitted_by']:<12} {when}{extra}"
    )


def cmd_status(args: argparse.Namespace) -> int:
    state = read_state()
    jobs = state["jobs"]
    if not args.all:
        jobs = [job for job in jobs if job["state"] not in TERMINAL_STATES][:] + [
            job for job in jobs if job["state"] in TERMINAL_STATES
        ][-5:]
    if args.json:
        print(
            json.dumps(
                {
                    "worker_alive": worker_alive(),
                    "gpu_busy": gpu_busy_processes(ignore_pids=set()),
                    "jobs": jobs,
                },
                indent=2,
            )
        )
        return 0
    busy = gpu_busy_processes(ignore_pids=set())
    print(f"worker: {'alive' if worker_alive() else 'not running'}")
    print(f"gpus:   {'busy - ' + str(busy) if busy else 'idle'}")
    if not jobs:
        print("queue is empty")
        return 0
    print(f"{'id':>4}  {'state':<9} {'name':<28} {'by':<12} when")
    for job in jobs:
        print(format_row(job))
    return 0


def cmd_cancel(args: argparse.Namespace) -> int:
    with locked_state() as state:
        job = find_job(state, args.job_id)
        if job is None:
            print(f"no job {args.job_id}", file=sys.stderr)
            return 1
        if job["state"] in TERMINAL_STATES:
            print(f"job {args.job_id} already {job['state']}")
            return 0
        job["cancel_requested"] = True
        if job["state"] == "queued":
            job.update(state="cancelled", finished_at=now())
            print(f"cancelled queued job {args.job_id}")
        else:
            print(f"asked worker to stop job {args.job_id}")
    return 0


def cmd_wait(args: argparse.Namespace) -> int:
    deadline = time.monotonic() + args.timeout if args.timeout else None
    while True:
        job = find_job(read_state(), args.job_id)
        if job is None:
            print(f"no job {args.job_id}", file=sys.stderr)
            return 1
        if job["state"] in TERMINAL_STATES:
            print(f"job {args.job_id} {job['state']} (exit {job['exit_code']})")
            return 0 if job["state"] == "done" else 1
        if deadline is not None and time.monotonic() > deadline:
            print(f"job {args.job_id} still {job['state']}")
            return 2
        time.sleep(POLL_SECONDS)


def running_job(state: dict[str, Any]) -> dict[str, Any] | None:
    return next((job for job in state["jobs"] if job["state"] == "running"), None)


def latest_job_with_log(state: dict[str, Any]) -> dict[str, Any] | None:
    """Most recently started job that produced a log, running or not."""
    started = [job for job in state["jobs"] if job.get("log") and job.get("started_at")]
    if not started:
        return None
    return max(started, key=lambda job: (job["started_at"], job["id"]))


def tail_offset(path: Path, lines: int) -> int:
    """Byte offset `lines` newlines back from the end of the file.

    Counts only real newlines: Ultralytics redraws its progress bar with carriage
    returns, so a `\\r`-aware split would rewind thousands of redraws of one epoch."""
    data = path.read_bytes()
    idx = len(data)
    for _ in range(lines):
        newline = data.rfind(b"\n", 0, idx)
        if newline < 0:
            return 0
        idx = newline
    return idx + 1


def follow_log(job: dict[str, Any], tail: int, pinned: bool) -> int:
    """Stream a job's log until it ends, then roll onto whatever runs next.

    The rollover is the point. The queue runs one job at a time for hours, so the
    log path worth watching changes several times over a sweep, and a plain
    `tail -f` on one path goes quiet exactly when the next arm starts. With an
    explicit job id (`pinned`) it stops when that job does instead.

    Bytes are copied straight through rather than line-buffered, so the progress
    bar's carriage returns still redraw in place."""
    current = job
    handle: Any = None
    try:
        while True:
            handle = _open_log(current, handle, tail)
            _drain(handle)
            time.sleep(1.0)

            state = read_state()
            fresh = find_job(state, current["id"]) or current
            if fresh["state"] not in TERMINAL_STATES:
                current = fresh
                continue

            _drain(handle)  # what the job wrote between the last read and its exit
            print(
                f"\njob {fresh['id']} {fresh['state']} (exit {fresh['exit_code']})",
                file=sys.stderr,
            )
            if pinned:
                return 0 if fresh["state"] == "done" else 1

            nxt = running_job(state)
            if nxt is None or nxt["id"] == current["id"]:
                continue  # nothing running yet; wait for the worker to pop the next job
            if handle is not None:
                handle.close()
            handle = None
            current = nxt
    except KeyboardInterrupt:
        print(file=sys.stderr)
        return 0
    finally:
        if handle is not None:
            handle.close()


def _open_log(job: dict[str, Any], handle: Any, tail: int) -> Any:
    """Open the job's log once it exists, seeked `tail` lines back. Idempotent."""
    if handle is not None or not job["log"]:
        return handle
    path = Path(job["log"])
    if not path.exists():
        return None
    print(f"==> {path} (job {job['id']} {job['name']})", file=sys.stderr)
    opened = path.open("rb")
    opened.seek(tail_offset(path, tail))
    return opened


def _drain(handle: Any) -> None:
    """Copy everything written since the last read straight to stdout."""
    if handle is None:
        return
    chunk = handle.read()
    if chunk:
        sys.stdout.buffer.write(chunk)
        sys.stdout.buffer.flush()


def cmd_logs(args: argparse.Namespace) -> int:
    state = read_state()
    if args.job_id is None:
        # No id means "whatever is training now"; fall back to the last job that ran so
        # the command still shows something in the gap between two jobs.
        job = running_job(state) or latest_job_with_log(state)
        if job is None:
            print("no job has produced a log yet", file=sys.stderr)
            return 1
    else:
        job = find_job(state, args.job_id)
        if job is None or not job["log"]:
            print(f"no log for job {args.job_id}", file=sys.stderr)
            return 1

    if args.follow:
        return follow_log(job, args.tail, pinned=args.job_id is not None)

    print(job["log"], file=sys.stderr)
    lines = Path(job["log"]).read_text().splitlines()
    print("\n".join(lines[-args.tail :]))
    return 0


def cmd_stop(_: argparse.Namespace) -> int:
    with locked_state() as state:
        state["worker_stop"] = True
    print("worker will exit after the running job finishes")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)

    submit = sub.add_parser("submit", help="add a job to the queue")
    submit.add_argument("--name", required=True, help="short label, used for the log filename")
    submit.add_argument("--by", default="", help="who submitted this (agent or person)")
    submit.add_argument("--priority", type=int, default=0, help="higher runs first")
    submit.add_argument("--cwd", default=str(REPO_ROOT), help="working directory for the job")
    submit.add_argument(
        "-d", "--devices", nargs="+", type=int, default=[0, 1, 2], help="GPUs the job will use"
    )
    submit.add_argument("command", nargs=argparse.REMAINDER, help="command after --")
    submit.set_defaults(func=cmd_submit)

    status = sub.add_parser("status", help="show the queue")
    status.add_argument("--json", action="store_true", help="machine-readable output")
    status.add_argument("--all", action="store_true", help="include every finished job")
    status.set_defaults(func=cmd_status)

    worker = sub.add_parser("worker", help="run the worker loop (started automatically)")
    worker.add_argument(
        "--idle-exit", type=int, default=600, help="exit after N idle seconds, 0 to stay up"
    )
    worker.set_defaults(func=cmd_worker)

    cancel = sub.add_parser("cancel", help="cancel a queued or running job")
    cancel.add_argument("job_id", type=int)
    cancel.set_defaults(func=cmd_cancel)

    wait = sub.add_parser("wait", help="block until a job finishes")
    wait.add_argument("job_id", type=int)
    wait.add_argument("--timeout", type=int, default=0, help="give up after N seconds")
    wait.set_defaults(func=cmd_wait)

    logs = sub.add_parser("logs", help="tail a job log (default: the running job)")
    logs.add_argument(
        "job_id",
        type=int,
        nargs="?",
        help="job to read; omit for the running job, or the last one that ran",
    )
    logs.add_argument("--tail", type=int, default=20)
    logs.add_argument(
        "-f",
        "--follow",
        action="store_true",
        help="stream new output as it is written. Without JOB_ID this follows the queue: "
        "when the running job ends it rolls onto the next one, so a whole sweep can be "
        "watched with one command. With JOB_ID it stops when that job does.",
    )
    logs.set_defaults(func=cmd_logs)

    stop = sub.add_parser("stop", help="stop the worker after the current job")
    stop.set_defaults(func=cmd_stop)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.cmd == "submit" and args.command and args.command[0] == "--":
        args.command = args.command[1:]
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
