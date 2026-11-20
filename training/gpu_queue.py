#!/usr/bin/env python3
"""Serial GPU job queue, so several agents can share megamind's three A6000s."""

import argparse
import contextlib
import fcntl
import json
import os
import re
import shlex
import signal
import statistics
import subprocess
import sys
import time
from collections.abc import Iterator
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, NamedTuple

REPO_ROOT = Path(__file__).resolve().parent.parent
QUEUE_DIR = REPO_ROOT / "runs" / "queue"
STATE_PATH = QUEUE_DIR / "state.json"
STATE_LOCK = QUEUE_DIR / "state.lock"
WORKER_LOCK = QUEUE_DIR / "worker.lock"
LOG_DIR = QUEUE_DIR / "logs"

BUSY_MIB = int(os.environ.get("AB_GPU_QUEUE_BUSY_MIB", "1024"))
POLL_SECONDS = 10
YIELD_DIR = Path.home() / ".config" / "gpu_queue" / "yield.d"
HOOK_TIMEOUT = 300
YIELD_RESUME_IDLE_SECONDS = 600
LEGACY_JOB_DEVICES = (0, 1, 2)
TERMINAL_STATES = ("done", "failed", "cancelled")
ACTIVE_STATES = ("running", "claimed")
EPOCH_RE = re.compile(r"\s(\d+)/(\d+)\s+[\d.]+G\s")
BATCH_RE = re.compile(r"(\d+)/(\d+)\s+[\d.]+it/s")
LOG_TAIL_BYTES = 256 * 1024
OWNER_SEARCH_PARENT_DEPTH = 12
LOG_PROGRESS_TRUST_FRACTION = 0.15
MIN_HISTORY_SECONDS = 300
SMOKE_EPOCHS = 10
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


def nvidia_smi_rows(query: str) -> list[list[str]]:
    out = subprocess.run(
        ["nvidia-smi", query, "--format=csv,noheader,nounits"],
        capture_output=True,
        text=True,
        timeout=30,
        check=True,
    ).stdout
    return [[field.strip() for field in line.split(",")] for line in out.splitlines() if line]


def gpu_busy_processes(
    ignore_pids: set[int], devices: list[int] | None = None
) -> list[tuple[int, int]]:
    """Compute processes holding real memory, excluding our own job's pids."""
    try:
        apps = nvidia_smi_rows("--query-compute-apps=pid,gpu_uuid,used_memory")
        uuids = {uuid: int(index) for index, uuid in nvidia_smi_rows("--query-gpu=index,uuid")}
    except (OSError, subprocess.SubprocessError, ValueError):
        return [(-1, -1)]
    busy = []
    for row in apps:
        if len(row) != 3:
            continue
        pid_text, uuid, mem_text = row
        try:
            pid, mib = int(pid_text), int(mem_text)
        except ValueError:
            continue
        if devices is not None and uuids.get(uuid) not in devices:
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
            "submitted_by": args.by or os.environ.get("CLAUDE_AGENT_NAME", "unknown"),
            "submitted_at": now(),
            "state": "queued",
            "started_at": None,
            "finished_at": None,
            "exit_code": None,
            "pid": None,
            "log": None,
            "cancel_requested": False,
            "hint": {
                key: value
                for key, value in (
                    ("work", args.work),
                    ("profile", args.profile),
                    ("eta_seconds", args.eta),
                )
                if value
            },
        }
        state["next_id"] += 1
        state["jobs"].append(job)
        state["worker_stop"] = False
        needs_worker = not worker_alive()
    print(f"queued job {job['id']} ({job['name']})")
    if needs_worker:
        spawn_worker()
        print("started worker")
    return 0


def queued_in_order(state: dict[str, Any]) -> list[dict[str, Any]]:
    """Queued jobs in the order the worker will pop them: FIFO by id."""
    return sorted(
        (job for job in state["jobs"] if job["state"] == "queued"), key=lambda job: job["id"]
    )


def next_queued(state: dict[str, Any]) -> dict[str, Any] | None:
    queued = queued_in_order(state)
    return queued[0] if queued else None


def job_env(job: dict[str, Any]) -> dict[str, str]:
    """Environment for a job, with the spawning agent's device settings stripped."""
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


def job_devices(job: dict[str, Any]) -> list[int]:
    return list(job.get("devices") or LEGACY_JOB_DEVICES)


def wait_for_gpus(job_id: int) -> bool:
    """Block until no outside process is using the job's GPUs. False if the job vanished."""
    announced = False
    while True:
        record = find_job(read_state(), job_id)
        if record is None or record["state"] != "queued" or record["cancel_requested"]:
            return False
        busy = gpu_busy_processes(ignore_pids=set(), devices=job_devices(record))
        if not busy:
            return True
        if not announced:
            held = "; ".join(describe_busy(busy, read_state()))
            print(f"[{now()}] job {job_id} waiting on GPUs held by {held}", flush=True)
            announced = True
        time.sleep(POLL_SECONDS)


def run_yield_hooks(action: str, devices: list[int] | None = None) -> None:
    """Run every tenant hook with `pause` or `resume`; output lands in the worker log."""
    if not YIELD_DIR.is_dir():
        return
    env = dict(os.environ)
    if devices is not None:
        env["GPU_QUEUE_DEVICES"] = ",".join(str(device) for device in devices)
    for hook in sorted(YIELD_DIR.iterdir()):
        if not (hook.is_file() and os.access(hook, os.X_OK)):
            continue
        print(f"[{now()}] {action} {hook.name}", flush=True)
        try:
            code = subprocess.run(
                [str(hook), action],
                stdin=subprocess.DEVNULL,
                env=env,
                timeout=HOOK_TIMEOUT,
                check=False,
            ).returncode
        except (OSError, subprocess.SubprocessError) as err:
            print(f"[{now()}] {action} {hook.name} failed: {err}", flush=True)
            continue
        if code != 0:
            print(f"[{now()}] {action} {hook.name} exited {code}", flush=True)


def exit_if_idle() -> None:
    """Exit the worker if the queue is still empty, holding the state lock until death."""
    with locked_state() as state:
        if next_queued(state) is not None:
            return
        print(f"[{now()}] worker idle, exiting", flush=True)
        sys.stdout.flush()
        os._exit(0)


def idle_tick(args: argparse.Namespace, idle: float, yielded: bool) -> bool:
    """One poll of an empty queue. Returns whether the tenants are still paused."""
    if yielded and idle > (args.idle_exit or YIELD_RESUME_IDLE_SECONDS):
        run_yield_hooks("resume")
        yielded = False
    if args.idle_exit and idle > args.idle_exit:
        exit_if_idle()
    return yielded


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
    yielded = False
    while True:
        state = read_state()
        if state.get("worker_stop"):
            print(f"[{now()}] worker stopping on request", flush=True)
            if yielded:
                run_yield_hooks("resume")
            return 0
        job = next_queued(state)
        if job is None:
            yielded = idle_tick(args, time.monotonic() - idle_since, yielded)
            time.sleep(POLL_SECONDS)
            continue
        idle_since = time.monotonic()
        run_yield_hooks("pause", job_devices(job))
        yielded = True
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
        idle_since = time.monotonic()
        final = find_job(read_state(), job["id"])
        print(f"[{now()}] job {job['id']} {final['state'] if final else 'gone'}", flush=True)


def parent_pid(pid: int) -> int:
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("PPid:"):
                return int(line.split()[1])
    except (OSError, ValueError):
        pass
    return 0


def process_ancestry(pid: int) -> set[int]:
    """A pid and everything that spawned it."""
    seen = {pid}
    for _ in range(OWNER_SEARCH_PARENT_DEPTH):
        pid = parent_pid(pid)
        if pid <= 1:
            break
        seen.add(pid)
    return seen


def process_command(pid: int) -> str:
    """What a process outside the queue is, e.g. "python train.py" or "sunshine"."""
    try:
        parts = [
            part
            for part in Path(f"/proc/{pid}/cmdline")
            .read_bytes()
            .decode("utf-8", "replace")
            .split("\0")
            if part
        ]
    except OSError:
        return "exited"
    if not parts:
        return "unknown"
    script = next((Path(part).name for part in parts[1:] if part.endswith(".py")), "")
    return f"{Path(parts[0]).name} {script}".strip()


def gpu_owner(pid: int, state: dict[str, Any]) -> str:
    ancestry = process_ancestry(pid)
    for job in state["jobs"]:
        if job.get("pid") and job["pid"] in ancestry:
            return f"job {job['id']} {job['name']}"
    command = process_command(pid)
    return command if command in ("exited", "unknown") else f"{command} (outside the queue)"


def format_mib(mib: int) -> str:
    return f"{mib / 1024:.1f} GiB" if mib >= 1024 else f"{mib} MiB"


def describe_busy(busy: list[tuple[int, int]], state: dict[str, Any]) -> list[str]:
    """Who is holding the GPUs, in words: one line per owner, memory summed per owner."""
    if busy == [(-1, -1)]:
        return ["nvidia-smi unreadable, so the queue assumes the GPUs are in use"]
    owners: dict[str, list[tuple[int, int]]] = {}
    for pid, mib in busy:
        owners.setdefault(gpu_owner(pid, state), []).append((pid, mib))
    lines = []
    for label, procs in owners.items():
        memory = format_mib(sum(mib for _, mib in procs))
        detail = f"{len(procs)} processes, {memory}" if len(procs) > 1 else memory
        pids = ", ".join(str(pid) for pid, _ in procs)
        lines.append(f"{label}, {detail} (pid {pids})")
    return lines


def parse_time(text: str | None) -> datetime | None:
    return datetime.fromisoformat(text) if text else None


def job_epochs(job: dict[str, Any]) -> int | None:
    """Epoch count from a training command's -e/--epochs flag, if it has one."""
    value = flag_value(job["command"], "-e", "--epochs")
    try:
        return int(value) if value is not None else None
    except ValueError:
        return None


ENV_ASSIGN_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*=")


def script_name(job: dict[str, Any]) -> str:
    """The script a job runs, so a 3-minute export is never averaged with a train."""
    command = list(job["command"])
    if command and Path(command[0]).name == "env":
        command = command[1:]
    while command and ENV_ASSIGN_RE.match(command[0]):
        command = command[1:]
    scripts = [part for part in command if part.endswith(".py")] or [
        part for part in command if part.endswith(".sh")
    ]
    fallback = command[0] if command else job["command"][0]
    return Path(scripts[-1] if scripts else fallback).name


class LogProgress(NamedTuple):
    fraction: float
    label: str


def log_progress(job: dict[str, Any]) -> LogProgress | None:
    """Progress of a running job read from its Ultralytics epoch counter, None without one."""
    if not job.get("log"):
        return None
    path = Path(job["log"])
    if not path.exists():
        return None
    with path.open("rb") as handle:
        handle.seek(max(0, path.stat().st_size - LOG_TAIL_BYTES))
        tail = handle.read().decode("utf-8", "replace")
    for line in reversed(tail.replace("\r", "\n").splitlines()):
        epoch_match = EPOCH_RE.search(line)
        if not epoch_match:
            continue
        epoch, epochs = int(epoch_match.group(1)), int(epoch_match.group(2))
        if epochs <= 0 or not 0 < epoch <= epochs:
            return None
        done = float(epoch - 1)
        batch_match = BATCH_RE.search(line)
        if batch_match:
            batch, batches = int(batch_match.group(1)), int(batch_match.group(2))
            if batches > 0:
                done += min(batch / batches, 1.0)
        return LogProgress(done / epochs, f"{epoch}/{epochs}")
    return None


def flag_value(command: list[str], *flags: str) -> str | None:
    """The argument following the first of `flags` present in a command."""
    for flag in flags:
        if flag in command:
            index = command.index(flag)
            if index + 1 < len(command):
                return command[index + 1]
    return None


def job_shape(job: dict[str, Any]) -> dict[str, str]:
    """What makes two jobs comparable: same script, model, input size, dataset."""
    command = job["command"]
    data = next((part for part in command if "training/data/" in part), "")
    if data.endswith(".yml") or data.endswith(".yaml"):
        data = str(Path(data).parent)
    return {
        "script": script_name(job),
        "model": next((part for part in command if part.startswith("yolo")), ""),
        "imgsz": flag_value(command, "--imgsz") or "default",
        "data": Path(data).name,
    }


MATCH_TIERS = (
    ("script", "model", "imgsz", "data"),
    ("script", "model", "imgsz"),
    ("script", "model"),
    ("script",),
)


def describe_match(shape: dict[str, str], keys: tuple[str, ...], count: int) -> str:
    """Name the jobs an estimate leans on, e.g. "2 past yolo26s @640 runs"."""
    if "model" in keys and shape["model"]:
        size = "" if shape["imgsz"] == "default" or "imgsz" not in keys else f" @{shape['imgsz']}"
        what = f"{shape['model']}{size}"
    else:
        what = shape["script"]
    return f"{count} past {what} run{'s' if count > 1 else ''}"


def past_durations(state: dict[str, Any]) -> list[tuple[dict[str, Any], float]]:
    """Every job that ran to completion, with how long it took."""
    out = []
    for job in state["jobs"]:
        if job["state"] != "done":
            continue
        started, finished = parse_time(job["started_at"]), parse_time(job["finished_at"])
        if started and finished and finished > started:
            out.append((job, (finished - started).total_seconds()))
    return out


def usable_history(
    state: dict[str, Any], job: dict[str, Any]
) -> list[tuple[dict[str, Any], float]]:
    """Finished jobs worth learning from when predicting this one."""
    history = [
        (past, seconds) for past, seconds in past_durations(state) if past["id"] != job["id"]
    ]
    epochs = job_epochs(job)
    if epochs is None or epochs < SMOKE_EPOCHS:
        return history
    return [pair for pair in history if pair[1] >= MIN_HISTORY_SECONDS] or history


def matching_history(
    state: dict[str, Any], job: dict[str, Any]
) -> tuple[list[tuple[dict[str, Any], float]], str]:
    """The finished jobs most like this one, and a phrase naming what they are."""
    history = usable_history(state, job)
    shape = job_shape(job)
    for keys in MATCH_TIERS:
        matches = [
            pair for pair in history if all(job_shape(pair[0])[key] == shape[key] for key in keys)
        ]
        if matches:
            return matches, describe_match(shape, keys, len(matches))
    return [], "no comparable runs yet"


DURATION_RE = re.compile(r"^(?:(\d+(?:\.\d+)?)h)?(?:(\d+(?:\.\d+)?)m)?(?:(\d+(?:\.\d+)?)s)?$")


def parse_duration(text: str) -> float:
    """Seconds from "6h30m", "90m", "3600s", or a bare count of seconds."""
    cleaned = text.strip().lower()
    try:
        return float(cleaned)
    except ValueError:
        pass
    match = DURATION_RE.match(cleaned)
    if not match or not any(match.groups()):
        raise argparse.ArgumentTypeError(f"bad duration {text!r}; use 6h30m, 90m or 3600s")
    hours, minutes, seconds = (float(part) if part else 0.0 for part in match.groups())
    return hours * 3600 + minutes * 60 + seconds


def job_hint(job: dict[str, Any]) -> dict[str, Any]:
    """The submitter's cost parameters. Empty for jobs queued before these existed."""
    return job.get("hint") or {}


def job_work(job: dict[str, Any]) -> float | None:
    """How much work the submitter declared, in its own unit, or None."""
    work = job_hint(job).get("work")
    return float(work) if work else None


def job_profile(job: dict[str, Any]) -> str:
    """The cost class a job's seconds-per-unit rate is learned within."""
    declared = job_hint(job).get("profile")
    if declared:
        return str(declared)
    shape = job_shape(job)
    return f"{shape['model']}@{shape['imgsz']}" if shape["model"] else shape["script"]


def format_units(work: float) -> str:
    for scale, suffix in ((1e9, "G"), (1e6, "M"), (1e3, "k")):
        if work >= scale:
            return f"{work / scale:.1f}{suffix}"
    return f"{work:.0f}"


def work_estimate(state: dict[str, Any], job: dict[str, Any]) -> tuple[float | None, str]:
    """Seconds from the declared work size and the measured rate of jobs like it."""
    work = job_work(job)
    if work is None:
        return None, ""
    profile, model = job_profile(job), job_shape(job)["model"]
    history = [pair for pair in usable_history(state, job) if job_work(pair[0])]
    tiers = (
        (profile, [pair for pair in history if job_profile(pair[0]) == profile]),
        (model, [pair for pair in history if model and job_shape(pair[0])["model"] == model]),
    )
    for label, matches in tiers:
        if not matches:
            continue
        rates = [seconds / work_done for past, seconds in matches if (work_done := job_work(past))]
        if rates:
            plural = "s" if len(matches) > 1 else ""
            basis = f"{len(matches)} past {label} run{plural} at {format_units(work)} units"
            return statistics.median(rates) * work, basis
    return None, ""


def typical_seconds(state: dict[str, Any], job: dict[str, Any]) -> tuple[float | None, str]:
    """What a job that has not started should take, learned from the jobs like it."""
    eta = job_hint(job).get("eta_seconds")
    if eta:
        return float(eta), "as submitted"
    seconds, basis = work_estimate(state, job)
    if seconds is not None:
        return seconds, basis
    if job_work(job):
        return None, "no comparable runs yet"
    history, basis = matching_history(state, job)
    if not history:
        return None, basis
    epochs = job_epochs(job)
    if epochs:
        rates = [
            seconds / past_epochs
            for past, seconds in history
            if (past_epochs := job_epochs(past)) is not None and past_epochs > 0
        ]
        if rates:
            return statistics.median(rates) * epochs, basis
    return statistics.median([seconds for _, seconds in history]), basis


def run_order(state: dict[str, Any]) -> list[dict[str, Any]]:
    """Unfinished jobs in the order they will run: whatever holds the GPUs, then FIFO."""
    active = sorted(
        (job for job in state["jobs"] if job["state"] in ACTIVE_STATES), key=lambda job: job["id"]
    )
    return active + queued_in_order(state)


def forecast(state: dict[str, Any], ref: datetime) -> dict[int, dict[str, Any]]:
    """Predicted duration and finish time for each unfinished job, in run order."""
    plan: dict[int, dict[str, Any]] = {}
    cursor: datetime | None = ref
    for job in run_order(state):
        history, basis = typical_seconds(state, job)
        seconds, measured, progress = history, False, None
        if job["state"] in ACTIVE_STATES:
            started = parse_time(job["started_at"]) or ref
            elapsed = max((ref - started).total_seconds(), 0.0)
            reading = log_progress(job)
            progress = reading.label if reading else None
            fraction = reading.fraction if reading else 0.0
            if fraction >= LOG_PROGRESS_TRUST_FRACTION or (history is None and fraction > 0.02):
                seconds, measured, basis = elapsed / fraction, True, ""
            if seconds is not None:
                seconds = max(seconds, elapsed)
            cursor = started + timedelta(seconds=seconds) if seconds is not None else None
            if cursor is not None and cursor < ref:
                cursor = ref
        elif cursor is not None and seconds is not None:
            cursor = cursor + timedelta(seconds=seconds)
        else:
            cursor = None
        plan[job["id"]] = {
            "seconds": seconds,
            "finish": cursor,
            "measured": measured,
            "progress": progress,
            "basis": "" if measured else basis,
        }
    return plan


def format_duration(seconds: float | None) -> str:
    if seconds is None:
        return "?"
    minutes = int(round(seconds / 60))
    hours, minutes = divmod(minutes, 60)
    return f"{hours}h{minutes:02d}m" if hours else f"{minutes}m"


def format_clock(when: datetime | None, ref: datetime) -> str:
    """Local time, dated only when it is not today."""
    if when is None:
        return "?"
    return when.strftime("%H:%M") if when.date() == ref.date() else when.strftime("%m-%d %H:%M")


ACTIVE_HEADER = (
    f"{'id':>4}  {'run':>3}  {'state':<9} {'name':<28} {'by':<22} {'finish':<12} {'est':<7} note"
)
RECENT_HEADER = f"{'id':>4}  {'':>3}  {'state':<9} {'name':<28} {'by':<22} {'finished':<12} took"


def format_active_row(job: dict[str, Any], slot: str, plan: dict[str, Any], ref: datetime) -> str:
    mark = "" if plan["measured"] else "~"
    finish = format_clock(plan["finish"], ref)
    duration = format_duration(plan["seconds"])
    if plan["finish"] is not None:
        finish, duration = mark + finish, mark + duration
    note = ", ".join(part for part in (plan["progress"], plan["basis"]) if part)
    return (
        f"{job['id']:>4}  {slot:>3}  {job['state']:<9} {job['name']:<28} "
        f"{job['submitted_by']:<22} {finish:<12} {duration:<7} {note}".rstrip()
    )


def format_recent_row(job: dict[str, Any], ref: datetime) -> str:
    started, finished = parse_time(job["started_at"]), parse_time(job["finished_at"])
    took = (finished - started).total_seconds() if started and finished else None
    when = format_clock(finished or parse_time(job["submitted_at"]), ref)
    exit_code = "" if job["exit_code"] is None else f"  exit={job['exit_code']}"
    return (
        f"{job['id']:>4}  {'':>3}  {job['state']:<9} {job['name']:<28} "
        f"{job['submitted_by']:<22} {when:<12} {format_duration(took)}{exit_code}"
    )


def print_machines(state: dict[str, Any]) -> None:
    print(f"worker: {'alive' if worker_alive() else 'not running'}")
    busy = gpu_busy_processes(ignore_pids=set())
    if not busy:
        print("gpus:   idle")
    for index, line in enumerate(describe_busy(busy, state)):
        print(f"gpus:   busy - {line}" if index == 0 else f"{'':14}{line}")


def print_queue(
    ordered: list[dict[str, Any]], plan: dict[int, dict[str, Any]], ref: datetime
) -> None:
    """The unfinished jobs, top to bottom in the order the worker will run them."""
    if not ordered:
        print("queue is empty")
        return
    last = plan[ordered[-1]["id"]]["finish"]
    print(f"\nqueue ({len(ordered)} unfinished, empty by {format_clock(last, ref)}):")
    print(ACTIVE_HEADER)
    position = 0
    for job in ordered:
        if job["state"] in ACTIVE_STATES:
            slot = "now"
        else:
            position += 1
            slot = str(position)
        print(format_active_row(job, slot, plan[job["id"]], ref))
    if any(plan[job["id"]]["finish"] and not plan[job["id"]]["measured"] for job in ordered):
        print("~ estimated from what past jobs took, not from this job's own progress")


def print_finished(recent: list[dict[str, Any]], ref: datetime, everything: bool) -> None:
    if not recent:
        return
    print(f"\nfinished ({'all' if everything else f'last {len(recent)}'}):")
    print(RECENT_HEADER)
    for job in recent:
        print(format_recent_row(job, ref))


def cmd_status(args: argparse.Namespace) -> int:
    state = read_state()
    ref = datetime.now()
    ordered = run_order(state)
    plan = forecast(state, ref)
    recent = [job for job in state["jobs"] if job["state"] in TERMINAL_STATES]
    if not args.all:
        recent = recent[-5:]
    if args.json:
        print(
            json.dumps(
                {
                    "worker_alive": worker_alive(),
                    "gpu_busy": gpu_busy_processes(ignore_pids=set()),
                    "run_order": [job["id"] for job in ordered],
                    "jobs": [job | json_forecast(plan[job["id"]]) for job in ordered] + recent,
                },
                indent=2,
            )
        )
        return 0
    print_machines(state)
    print_queue(ordered, plan, ref)
    print_finished(recent, ref, args.all)
    return 0


def json_forecast(plan: dict[str, Any]) -> dict[str, Any]:
    return {
        "estimated_seconds": None if plan["seconds"] is None else round(plan["seconds"]),
        "estimated_finish": None
        if plan["finish"] is None
        else plan["finish"].isoformat(timespec="seconds"),
        "estimate_from_progress": plan["measured"],
        "estimate_basis": plan["basis"],
        "progress": plan["progress"],
    }


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
    """Byte offset `lines` newlines back from the end of the file."""
    data = path.read_bytes()
    idx = len(data)
    for _ in range(lines):
        newline = data.rfind(b"\n", 0, idx)
        if newline < 0:
            return 0
        idx = newline
    return idx + 1


def follow_log(job: dict[str, Any], tail: int, pinned: bool) -> int:
    """Stream a job's log until it ends, then roll onto whatever runs next."""
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

            _drain(handle)
            print(
                f"\njob {fresh['id']} {fresh['state']} (exit {fresh['exit_code']})",
                file=sys.stderr,
            )
            if pinned:
                return 0 if fresh["state"] == "done" else 1

            nxt = running_job(state)
            if nxt is None or nxt["id"] == current["id"]:
                continue
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
    submit.add_argument("--cwd", default=str(REPO_ROOT), help="working directory for the job")
    submit.add_argument(
        "-d", "--devices", nargs="+", type=int, default=[0, 1, 2], help="GPUs the job will use"
    )
    submit.add_argument(
        "--work",
        type=float,
        default=None,
        help="job size in one consistent unit, frames x epochs for training",
    )
    submit.add_argument(
        "--profile",
        default=None,
        help="cost class a rate is learned within, e.g. yolo26x-pose@640",
    )
    submit.add_argument(
        "--eta",
        type=parse_duration,
        default=None,
        help="expected duration (6h30m, 90m, 3600s) until real progress takes over",
    )
    submit.add_argument("command", nargs=argparse.REMAINDER, help="command after --")
    submit.set_defaults(func=cmd_submit)

    status = sub.add_parser("status", help="show the queue in run order, with finish estimates")
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
        help="stream new output; without JOB_ID, roll onto the next job when one ends",
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
