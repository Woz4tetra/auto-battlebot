"""Drive scripted excitation for a velocity jig session, and bring the logs home.

Replaces the browser tool at `firmware/velocity_jig/web_tool`. One run produces a triple in
the output directory: the downloaded log, a sidecar TOML holding the experiment parameters,
and a CSV of the command timeline.

    source scripts/activate_python.sh

    # inspect without hardware
    python playground/calibration/velocity_jig_drive.py --list-waveforms
    python playground/calibration/velocity_jig_drive.py --waveform lin_step_full --dry-run

    # record; --out defaults to out/<date>-<name>
    python playground/calibration/velocity_jig_drive.py \\
        --waveform lin_step_full --waveform lin_coast \\
        --name "garage floor sweep"

Two USB serial devices are open at once: the jig (`/dev/ttyACM*`) and the OpenTX radio
(0483:5740). `--list-ports` shows every candidate with its ids.

PAUSES. A piecewise waveform stops between cells and waits for Enter, so the robot can be
put back on its mark before the next one. The program clock stops with it and the sidecar
records each window. `--no-pause` plays straight through.

ONE CLOCK. Command timestamps and clock probes both come from `time.monotonic()` in this
process. Splitting them across processes reintroduces exactly the offset the `TIME` probe
exists to remove, and the transport delay fitted afterwards would be that offset plus the
real delay with no way to tell them apart.

SAFETY. Trainer mode ADDS to the human driver's sticks, so a run is only as safe as the
person holding the radio. The channels are zeroed and the link disarmed on every exit path:
normal return, Ctrl-C, exception, and a hard wall-clock timeout. Verify that once per
session with a deliberate Ctrl-C before trusting it for the rest of the day.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from auto_battlebot.plant import PlantParams, simulate
from auto_battlebot.velocity_jig import SIDECAR_SCHEMA, ClockFit, ClockProbe, PauseWindow
from calib_lib import drive_protocol as dp
from calib_lib import excitation as ex
from calib_lib import jig_link as jl

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CATALOG = Path(__file__).resolve().parent / "waveforms.toml"


def _repo_relative(path: Path) -> Path:
    """Path relative to the repo root when it lives inside it, else unchanged.

    Session files are committed, so an absolute path in one goes stale the moment the tree is
    cloned somewhere else.
    """
    try:
        return path.resolve().relative_to(REPO_ROOT)
    except ValueError:
        return path
DEFAULT_OUT_ROOT = Path(__file__).resolve().parent / "out"
# Long enough to bound the gyro bias estimate, short enough that an operator will actually
# stand still for it. The fit needs two of these per run to bracket the bias drift.
DEFAULT_HOLD_S = 5.0
DEFAULT_PROBES = 200


# ---------------------------------------------------------------------------
# Operator console
# ---------------------------------------------------------------------------


def say(msg: str = "") -> None:
    print(msg, flush=True)


def step(n: int, total: int, msg: str) -> None:
    say(f"\n[{n}/{total}] {msg}")


# How long to keep discarding keystrokes before a prompt will accept one. A double-tapped
# Enter lands 50-200 ms after the first, so a single flush at prompt time is too early to
# catch it: the stray press arrives afterwards and satisfies the prompt that just appeared.
SETTLE_S = 0.35


def drain_stdin(settle_s: float = SETTLE_S) -> None:
    """Throw away anything typed before a prompt is ready to accept it.

    Without this, a double-tapped Enter at one pause silently answers the next prompt, so
    the robot arms and drives while someone still has a hand on it. The whole point of the
    pause is that nothing moves until the operator says so, and a buffered keystroke is not
    the operator saying so.

    Only meaningful on a terminal. A piped or redirected stdin has no driver queue to flush
    and termios would raise on it, so `--no-prompt` and any scripted run are unaffected.
    """
    if not sys.stdin.isatty():
        return
    try:
        import termios
    except ImportError:  # not POSIX
        return
    deadline = time.monotonic() + settle_s
    while True:
        try:
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
        except (OSError, termios.error):
            return
        if time.monotonic() >= deadline:
            return
        time.sleep(0.02)


def prompt(msg: str) -> None:
    drain_stdin()
    try:
        input(f"      {msg} ")
    except EOFError:
        raise SystemExit("\naborted: stdin closed") from None


# Long enough for the robot to stop from full command once the excitation ends. The piecewise
# waveforms give themselves 4 s through a trailing coast cell, and that is many decel
# constants, so match it rather than invent a second number.
SETTLE_COAST_S = 4.0


# How long the command must have been zero at the end of a program for the robot to be at
# rest when it returns. Many decel constants, so what follows is a stopped robot and not a
# slow one.
REST_TAIL_S = 1.0


def ends_at_rest(program: ex.Program, eps: float = 1e-3, tail_s: float = REST_TAIL_S) -> bool:
    """Whether the program leaves the robot stopped, judged over a trailing window.

    The last command alone is not enough. A sine ends on a zero crossing and a PRBS can end
    on a zero bit, but the plant lags the command: at the moment a sine's command reaches
    zero the robot is near peak speed, because the first-order response is phase shifted. So
    ask whether the command has been zero for long enough to have stopped it, not whether the
    final sample happens to be zero.

    Sampled from `at`, not inferred from the kind, so a piecewise program given a nonzero last
    cell is caught too.
    """
    step = 0.005
    n = max(1, int(round(min(tail_s, program.duration_s) / step)))
    for i in range(n + 1):
        lin, ang = program.at(max(0.0, program.duration_s - i * step))
        if abs(lin) >= eps or abs(ang) >= eps:
            return False
    return True


def upcoming(program: ex.Program, pause_at: Sequence[float], index: int) -> list[Any]:
    """The segments that will play once this pause is released, up to the next stop.

    `index` is 1-based, matching what `play` hands to `on_pause`.

    Empty for a continuous program. A chirp, sine, triangle or PRBS has `segments = None`
    because it has no cell structure to slice by, and it never pauses either, so the only
    caller that reaches it is the arm-time preview. `describe` covers that case.
    """
    start = pause_at[index - 1]
    stop = pause_at[index] if index < len(pause_at) else float("inf")
    return [s for s in (program.segments or []) if start <= s.t0 < stop]


def describe_continuous(program: ex.Program, step_s: float = 0.005) -> str:
    """One line for a program with no segments: what it sweeps and how hard.

    Sampled from `at`, which is the authority for what actually gets sent, rather than read
    off the catalog amplitude. A capped or clipped waveform would otherwise be described by
    the number it asked for instead of the one the robot receives.
    """
    n = max(2, int(program.duration_s / step_s) + 1)
    lin = np.array([program.at(i * step_s)[0] for i in range(n)])
    ang = np.array([program.at(i * step_s)[1] for i in range(n)])
    return (
        f"{program.duration_s:5.2f} s  continuous {program.kind}  "
        f"lin {lin.min():+.3f} to {lin.max():+.3f}  "
        f"ang {ang.min():+.3f} to {ang.max():+.3f}"
    )


# Longest "about to send" list printed in full. A staircase has no rest between rungs, so
# its first cell is the whole ladder: 60 lines that bury the prompt underneath them and tell
# the operator nothing the summary does not.
PREVIEW_LINES = 4


def preview(program: ex.Program, pause_at: Sequence[float], index: int) -> list[str]:
    """The 'about to send' lines, for a piecewise or a continuous program."""
    cells = upcoming(program, pause_at, index)
    if not cells:
        # No segments at all means a continuous program. Segments that exist but do not
        # match means this pause is the last one and nothing follows it.
        return [] if program.segments else [describe_continuous(program)]

    def fmt(seg: Any) -> str:
        return (
            f"{seg.t1 - seg.t0:5.2f} s  lin {seg.linear:+.3f}  "
            f"ang {seg.angular:+.3f}   {seg.label}"
        )

    if len(cells) <= PREVIEW_LINES:
        return [fmt(seg) for seg in cells]
    rest = cells[PREVIEW_LINES - 1 :]
    span = sum(seg.t1 - seg.t0 for seg in rest)
    lins = [seg.linear for seg in rest]
    angs = [seg.angular for seg in rest]
    return [fmt(seg) for seg in cells[: PREVIEW_LINES - 1]] + [
        f"{span:5.2f} s  lin {min(lins):+.3f} to {max(lins):+.3f}  "
        f"ang {min(angs):+.3f} to {max(angs):+.3f}   + {len(rest)} more segments"
    ]


def pause_prompt(index: int, count: int, program: ex.Program, pause_at: Sequence[float]) -> None:
    """Between-cell stop. The channels are already zeroed when this is called.

    Prints the cell about to play before waiting. Knowing that the next command is -0.4 and
    not +0.6 is what tells the operator which way to aim the robot, and how much floor it
    needs, while there is still time to do something about it.
    """
    say(f"\n      --- paused {index}/{count} ---")
    lines = preview(program, pause_at, index)
    if lines:
        say("      about to send:")
        for line in lines:
            say(f"        {line}")
    prompt("Put the robot back on its mark, then press Enter to continue:")


def confirm_stopped(link: jl.JigLink, grace: float = 1.0) -> tuple[int, int]:
    """Stop the recording on an Enter press, not on the jig's `stopped` line.

    Waiting on the line alone hangs the run whenever the press is mistimed: a press that
    lands before the cable is back in never puts a summary on the wire, and the step then
    sits there until the 300 s timeout. Gating on Enter costs only the counts, which are a
    reporting nicety. gates() already reads samples=0 as unknown, and the log on the card
    is complete either way.
    """
    prompt("Press B on the jig to stop recording, then press Enter:")
    for attempt in (1, 2):
        try:
            return link.wait_stopped(timeout=grace)
        except TimeoutError:
            pass
        if attempt == 1:
            say("      no 'stopped' line came back.")
            prompt("If the jig is still recording, press B now, then press Enter:")
    say("      WARNING: no stop summary, so n and dropped are unknown for this run.")
    say("      If the download fails as BUSY, the jig never stopped: press B and rerun.")
    return 0, 0


def countdown(seconds: float, msg: str) -> tuple[float, float]:
    """Hold still. Returns the (start, end) host window the sidecar records."""
    start = time.monotonic()
    end = start + seconds
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        print(f"\r      {msg}: {remaining:4.1f} s ", end="", flush=True)
        time.sleep(min(0.1, remaining))
    print(f"\r      {msg}: done      ", flush=True)
    return start, time.monotonic()


# ---------------------------------------------------------------------------
# Dry run
# ---------------------------------------------------------------------------


@dataclass
class Footprint:
    forward_m: float
    lateral_m: float
    span_m: float
    peak_speed: float
    peak_yaw: float
    heading_turns: float


def predict_footprint(program: ex.Program, params: PlantParams, dt: float = 0.005) -> Footprint:
    """Where the robot ends up, using the real plant model rather than a hand-set one.

    Lateral excursion matters as much as forward span. With a non-zero drift term a straight
    run arcs, and a 3 m step that fits the length of the board can still leave it sideways.
    """
    n = int((program.duration_s + 2.0) / dt)
    lin = []
    ang = []
    for i in range(n):
        a, b = program.at(i * dt)
        lin.append(a)
        ang.append(b)

    import numpy as np

    _, traj = simulate(
        np.array(lin)[None, :], np.array(ang)[None, :], dt, params
    )
    x = traj["x"][0]
    y = traj["y"][0]
    v = traj["v"][0]
    w = traj["w"][0]
    return Footprint(
        forward_m=float(x.max() - x.min()),
        lateral_m=float(y.max() - y.min()),
        span_m=float(np.hypot(x - x[0], y - y[0]).max()),
        peak_speed=float(np.abs(v).max()),
        peak_yaw=float(np.abs(w).max()),
        heading_turns=float(np.abs(traj["theta"][0]).max() / (2 * np.pi)),
    )


def do_dry_run(programs: Sequence[ex.Program], params: PlantParams, half_width_m: float) -> None:
    for program in programs:
        say(f"\n=== {program.name} ({program.kind}/{program.channel}, {program.role}) ===")
        say(f"duration {program.duration_s:.1f} s")
        if program.segments:
            say(f"{len(program.segments)} segments:")
            for seg in program.segments[:40]:
                say(
                    f"  {seg.t0:6.2f} - {seg.t1:6.2f}  lin {seg.linear:+.3f}  "
                    f"ang {seg.angular:+.3f}  {seg.label}"
                )
            if len(program.segments) > 40:
                say(f"  ... {len(program.segments) - 40} more")
            say(f"{len(ex.pause_points(program))} operator stops (--no-pause to run straight)")
        else:
            say("continuous waveform, sampled at 50 Hz:")
            for t, lin, ang in program.sample(5.0)[:10]:
                say(f"  {t:6.2f}  lin {lin:+.3f}  ang {ang:+.3f}")

        fp = predict_footprint(program, params)
        say(
            f"predicted: forward {fp.forward_m:.2f} m, lateral {fp.lateral_m:.2f} m, "
            f"peak {fp.peak_speed:.2f} m/s / {fp.peak_yaw:.1f} rad/s, "
            f"{fp.heading_turns:.1f} turns"
        )
        if half_width_m > 0 and fp.lateral_m > half_width_m:
            say(
                f"  WARNING: lateral excursion {fp.lateral_m:.2f} m exceeds the "
                f"{half_width_m:.2f} m half-width. Trim it, or shorten the holds."
            )


# ---------------------------------------------------------------------------
# Sidecar output
# ---------------------------------------------------------------------------


def _toml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int,)):
        return str(value)
    if isinstance(value, float):
        return repr(value) if value != int(value) else f"{value:.1f}"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_toml_scalar(v) for v in value) + "]"
    return '"' + str(value).replace("\\", "\\\\").replace('"', '\\"') + '"'


def _table(name: str, values: dict[str, Any]) -> list[str]:
    out = [f"[{name}]"]
    for key, value in values.items():
        if value is None:
            continue
        out.append(f"{key} = {_toml_scalar(value)}")
    out.append("")
    return out


def write_session_toml(path: Path, meta: dict[str, Any], provenance: dict[str, Any]) -> None:
    lines = [
        "# Velocity jig capture session. Written by velocity_jig_drive.py.",
        f"schema = {SIDECAR_SCHEMA}",
        "",
    ]
    lines += _table("session", meta)
    lines += _table("provenance", provenance)
    path.write_text("\n".join(lines), encoding="utf-8")


def write_run_toml(
    path: Path,
    *,
    run: dict[str, Any],
    spec: dict[str, Any],
    params: dict[str, Any],
    segments: Sequence[Any],
    pauses: Sequence[PauseWindow],
    transfer: dict[str, Any],
    clock_pre: ClockProbe | None,
    clock_post: ClockProbe | None,
    skew_ppm: float | None,
    holds: dict[str, tuple[float, float]],
    session: dict[str, Any],
    provenance: dict[str, Any],
) -> None:
    lines = [
        "# One velocity jig run. Written by velocity_jig_drive.py.",
        f"schema = {SIDECAR_SCHEMA}",
        "",
    ]
    lines += _table("run", run)
    lines += _table("waveform", spec)
    if params:
        lines += _table("waveform.params", params)
    for seg in segments:
        lines += _table(
            "[segment]",
            {
                "t0": seg.t0,
                "t1": seg.t1,
                "linear": seg.linear,
                "angular": seg.angular,
                "label": seg.label,
            },
        )
    # Where the operator stopped the run to reset the robot's position. The segment times
    # above already carry these, so a reader that ignores pauses still labels the run
    # correctly. `t_start`/`t_end` are host seconds, the same clock the command log and the
    # still holds use: the robot was handled inside each window, so the gates, the saturation
    # count and the commanded-seconds total all skip them.
    for p in pauses:
        lines += _table(
            "[pause]",
            {
                "program_t": round(p.program_t, 3),
                "held_s": round(p.held_s, 3),
                "t_start": round(p.t_start, 6),
                "t_end": round(p.t_end, 6),
            },
        )
    lines += _table("transfer", transfer)
    if skew_ppm is not None:
        lines += _table("clock", {"skew_ppm": skew_ppm})
    if clock_pre:
        lines += _table("clock.pre", clock_pre.to_toml_table())
    if clock_post:
        lines += _table("clock.post", clock_post.to_toml_table())
    for kind, window in holds.items():
        lines += _table(f"hold.{kind}", {"start": window[0], "end": window[1]})
    lines += _table("session", session)
    lines += _table("provenance", provenance)
    path.write_text("\n".join(lines), encoding="utf-8")


COMMAND_COLUMNS = (
    "t_host_s,linear,angular,trim,meas_ch_a,meas_ch_b,"
    "meas_linear,meas_angular,meas_weapon,meas_arm"
)
COMMAND_PREAMBLE = [
    "# auto-battlebot velocity jig command log",
    "# linear/angular are what was asked for; meas_ch_a/meas_ch_b are what the radio",
    "# reported on the two drive channels. meas_linear/meas_angular are those channels",
    "# read through the configured mix, so they are an interpretation and the channels",
    "# are the measurement. A wrong mix can be corrected from the channels offline.",
    "# meas_weapon is the weapon throttle and meas_arm the weapon arm switch, CH3 and CH5",
    "# on the radio. Neither drives the plant; both say what the robot was doing. On a",
    "# hand-driven run the measured columns are the whole record: nothing was commanded.",
    f"# columns: {COMMAND_COLUMNS}",
]


def command_row(s: dp.CommandSample) -> str:
    """One CSV line. Shared so a rewrite cannot drift from the header it is written under."""
    return (
        f"{s.t:.6f},{s.linear:.4f},{s.angular:.4f},{s.trim:.4f},"
        f"{s.meas_ch_a:.4f},{s.meas_ch_b:.4f},"
        f"{s.meas_linear:.4f},{s.meas_angular:.4f},"
        f"{s.meas_weapon:.4f},{s.meas_arm:.4f}"
    )


def write_command_csv(path: Path, samples: Sequence[dp.CommandSample], meta: str) -> None:
    lines = COMMAND_PREAMBLE + [f"# {meta}"] + [command_row(s) for s in samples]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def slug(text: str) -> str:
    """Filesystem-safe form of a free-text name."""
    out = "".join(c.lower() if c.isalnum() else "-" for c in text)
    while "--" in out:
        out = out.replace("--", "-")
    return out.strip("-")


def default_out_dir(name: str) -> Path:
    """Where a session lands when --out is not given.

    One directory per day per name. Re-running the same name on the same day appends to it,
    which is what you want: a session is the leave-one-out unit for the fit, and one battery
    on one floor is one session whether it took one invocation or five.
    """
    stamp = f"{datetime.now():%Y-%m-%d}"
    tail = slug(name)
    return DEFAULT_OUT_ROOT / (f"{stamp}-{tail}" if tail else stamp)


def git_sha() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except Exception:
        return ""


# ---------------------------------------------------------------------------
# The run card
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Gate:
    """One thing a gate found.

    `fatal` is the difference between "this run's data is unusable" and "something went
    wrong at the bench that you want to fix before the next run". Conflating the two throws
    away complete recordings: a jig that reboots after the post-hold loses the skew
    correction and the dropped-sample count, neither of which is a defect in the samples it
    already wrote.
    """

    reason: str
    fatal: bool = True


@dataclass
class RunOutcome:
    log_file: str
    samples: int
    dropped: int
    verdict: str
    problems: list[str]
    warnings: list[str] = field(default_factory=list)


def reopen_after_replug(link: jl.JigLink, timeout: float = 300.0) -> jl.JigLink:
    """Wait for the jig to come back and reopen it, matching on USB serial number.

    Not on the device path: /dev/ttyACM0 can come back as ttyACM1 if anything else
    enumerated while the cable was out.
    """
    serial_number = link.serial_number
    link.close()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        info = (
            jl.find_port_by_serial(serial_number) if serial_number else jl.find_jig_port()
        )
        if info is not None:
            time.sleep(0.5)  # let the CDC endpoint settle before opening it
            try:
                return jl.JigLink(info.device, serial_number=info.serial_number)
            except OSError:
                pass
        time.sleep(0.25)
    raise TimeoutError("timed out waiting for the jig to be plugged back in")


def wait_for_unplug(link: jl.JigLink, timeout: float = 300.0) -> None:
    serial_number = link.serial_number
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if serial_number and jl.find_port_by_serial(serial_number) is None:
            return
        if not serial_number and jl.find_jig_port() is None:
            return
        time.sleep(0.25)
    raise TimeoutError("timed out waiting for the USB cable to be unplugged")


def run_one(
    args: argparse.Namespace,
    decl: ex.WaveformDecl,
    program: ex.Program,
    link: jl.JigLink,
    trainer: dp.TrainerLink | None,
    out_dir: Path,
    rep: int,
    session_meta: dict[str, Any],
    provenance: dict[str, Any],
) -> tuple[jl.JigLink, RunOutcome]:
    # A moving run adds the unplug, the B press, the replug, and the operator's own pauses.
    # The B press became its own step when it moved ahead of the replug.
    total = 12 if decl.motion else 9
    if decl.motion and not ends_at_rest(program):
        total += 1  # the coast-to-a-stop step
    n = 0
    say(f"\n{'=' * 70}")
    say(f"RUN  {program.name}#{rep}  ({program.kind}/{program.channel}, {program.role})")
    say(f"     {program.duration_s:.1f} s of excitation")
    say("=" * 70)

    n += 1
    step(n, total, f"Clock probe, pre ({args.probes} TIME round trips)")
    clock_pre = jl.probe_clock(link, args.probes)
    say(
        f"      offset {clock_pre.offset_ms:+.2f} ms, residual {clock_pre.residual_ms:.2f} ms "
        f"({clock_pre.kept}/{clock_pre.total} kept)"
    )
    if clock_pre.residual_ms > 2.0:
        say("      WARNING: residual over 2 ms. Fix the USB path before recording more.")

    n += 1
    step(n, total, "Press A on the jig to start recording")
    log_file = link.wait_recording()
    say(f"      recording {log_file}")

    n += 1
    step(n, total, f"Hold still {args.hold_s:.0f} s (gyro bias estimate)")
    hold_pre = countdown(args.hold_s, "hold still")

    if decl.motion:
        n += 1
        step(n, total, "Unplug the USB cable from the jig")
        wait_for_unplug(link)
        link.close()
        say("      cable out; the jig keeps logging on its own battery")

    n += 1
    step(n, total, f"Play {program.name}")
    commands: list[dp.CommandSample] = []
    contamination = 0.0
    pauses: list[PauseWindow] = []
    if trainer is not None and decl.kind != "manual":
        pause_at = [] if (args.no_pause or args.no_prompt) else ex.pause_points(program)
        if pause_at:
            say(f"      stopping {len(pause_at)} times to reset the robot's position")
        # The first cell has no pause in front of it, so it is the one the operator would
        # otherwise arm into blind.
        first = preview(program, [0.0] + list(pause_at), 1)
        if first:
            say("      about to send:")
            for line in first:
                say(f"        {line}")
        prompt("Type Enter to ARM and play (Ctrl-C aborts and disarms):")
        with dp.armed(trainer):
            result = dp.play(
                trainer,
                program,
                rate_hz=args.rate,
                trim=args.trim,
                timeout_s=program.duration_s + 10.0,
                pause_at=pause_at,
                on_pause=lambda i, n: pause_prompt(i, n, program, pause_at),
            )
        commands = result.commands
        contamination = result.contamination
        pauses = result.pauses
        say(f"      {len(commands)} commands, completed={result.completed}")
        if pauses:
            say(f"      {len(pauses)} pauses, {sum(p.held_s for p in pauses):.0f} s held")
        if result.aborted_reason:
            say(f"      ABORTED: {result.aborted_reason}")
        if contamination > 0.05:
            say(
                f"      WARNING: measured command differs from commanded by "
                f"{contamination:.3f}. The driver's sticks were not centered."
            )
    elif trainer is not None:
        # Hand-driven. Nothing is sent; the radio's own mixer output is the command log.
        say(f"      drive the robot for {program.duration_s:.0f} s. Recording the radio.")
        for sample in dp.stream_measured(trainer, program.duration_s, args.rate):
            commands.append(replace(sample, label=program.kind))
        say(f"      {len(commands)} radio samples")
        live = [c for c in commands if c.meas_arm > 0.0]
        if live:
            span = live[-1].t - live[0].t
            peak = max(abs(c.meas_weapon) for c in live)
            say(f"      weapon armed for {span:.0f} s, throttle peak {peak:+.2f}")
    else:
        countdown(program.duration_s, "run the maneuver")

    # A piecewise program ends on a rest cell, so the robot is already stopped by the time
    # the excitation returns. A continuous one does not: a PRBS, chirp or sine ends on
    # whatever value its last tick lands on, and `play` sends a single zero and returns with
    # the robot at speed. The post-hold countdown then measures a coasting robot, the
    # stillness test rejects it, and the run is thrown out for having one still hold.
    #
    # LOG-134, LOG-135, LOG-137 and LOG-145 were all lost this way, each with 0.02 s between
    # the last live command and the start of the hold. The piecewise runs beside them had 3
    # to 4 s of trailing coast and passed.
    if not ends_at_rest(program):
        n += 1
        step(n, total, "Let the robot coast to a stop")
        countdown(SETTLE_COAST_S, "coasting")

    n += 1
    step(n, total, f"Hold still {args.hold_s:.0f} s (bias drift bound)")
    hold_post = countdown(args.hold_s, "hold still")

    # B first, cable second. Plugging in resets this board, and a reset lands wherever the
    # firmware happens to be: on LOG-119 it landed mid-write and left a truncated final row
    # and a post-probe on a fresh boot epoch. Pressing B while unplugged closes the file on
    # the card before USB can do anything, so the reset has nothing left to interrupt.
    #
    # The cost is the stop summary, which goes to the wire unbuffered and so is lost with
    # the cable out. That is n and dropped, both reporting niceties: gates() reads samples=0
    # as unknown, and the log's own timestamps show the drops anyway. A truncated recording
    # is not recoverable, and a lost counter is.
    samples, dropped = 0, 0
    if decl.motion:
        n += 1
        step(n, total, "Press B on the jig to stop recording, with the cable still out")
        prompt("Press B now, then press Enter:")

        n += 1
        step(n, total, "Plug the USB cable back in")
        link = reopen_after_replug(link)
        say(f"      reconnected on {link.port}")
        say("      stop counts are not available over a replug; the log timestamps carry them")
    else:
        n += 1
        step(n, total, "Press B on the jig to stop recording")
        samples, dropped = confirm_stopped(link)
        if samples:
            say(f"      stopped: n={samples} dropped={dropped}")

    n += 1
    step(n, total, "Clock probe, post")
    clock_post = jl.probe_clock(link, args.probes)
    skew = jl.skew_ppm(clock_pre, clock_post)
    say(
        f"      offset {clock_post.offset_ms:+.2f} ms, residual {clock_post.residual_ms:.2f} ms, "
        f"skew {skew:+.1f} ppm"
    )

    n += 1
    step(n, total, f"Download {log_file}")
    dest = out_dir / log_file
    if dest.exists():
        # The jig picks the first free name on its own card, so names never repeat within a
        # card. They do repeat after a card wipe, and appending to an existing session
        # directory would then overwrite the earlier run's log without saying so.
        stamp = f"{datetime.now():%H%M%S}"
        dest = out_dir / f"{dest.stem}-{stamp}{dest.suffix}"
        say(f"      {log_file} already here from an earlier card; saving as {dest.name}")
    written = link.download(log_file, dest)
    say(f"      {written} bytes -> {dest}")

    n += 1
    step(n, total, "Write metadata")
    stem = dest.stem
    cmd_name = f"{stem}.cmd.csv"
    if commands:
        write_command_csv(
            out_dir / cmd_name,
            commands,
            f"rate_hz={args.rate} waveform={program.name} run={log_file}",
        )

    holds = {"pre": hold_pre, "post": hold_post}
    found = gates(dropped, clock_pre, clock_post, skew, samples, args.hold_s, contamination)
    found += log_gates(dest, pauses, ClockFit.from_probes(clock_pre, clock_post), holds)
    problems = [g.reason for g in found if g.fatal]
    warnings = [g.reason for g in found if not g.fatal]
    verdict = "discard" if problems else "pass"
    # The reasons go in the sidecar too. A verdict with no reason means working out months
    # later whether a run was thrown away for a real fault or a since-fixed tool bug. The
    # warnings go beside them: a run can be worth keeping and still be worth fixing.
    gate_note = "; ".join(problems)
    for warning in warnings:
        say(f"      WARNING: {warning}")
    notes = args.notes
    if not args.no_prompt:
        try:
            drain_stdin()
            entered = input("      Notes (blank to keep, 'discard' to reject): ").strip()
        except EOFError:
            entered = ""
        if entered == "discard":
            verdict = "discard"
        elif entered:
            notes = entered

    write_run_toml(
        out_dir / f"{stem}.toml",
        run={
            "id": f"{datetime.now(timezone.utc):%Y-%m-%dT%H:%M:%SZ}-{program.name}-{rep}",
            # dest.name, not the jig's name: a collision after a card wipe renames the file
            # on disk, and a sidecar pointing at the original name would load the wrong log.
            "log_file": dest.name,
            "command_file": cmd_name if commands else None,
            "rep": rep,
            "started_utc": f"{datetime.now(timezone.utc):%Y-%m-%dT%H:%M:%SZ}",
            "duration_s": program.duration_s + sum(p.held_s for p in pauses),
            "verdict": verdict,
            "gates": gate_note,
            "warnings": "; ".join(warnings),
            "contamination": round(contamination, 4),
            "notes": notes,
            "encoder": args.encoder,
            "trim": args.trim,
        },
        spec={
            "name": program.name,
            "kind": program.kind,
            "channel": program.channel,
            "role": program.role,
            "label": program.label,
        },
        params=decl.params,
        segments=ex.shift_segments(program.segments or [], pauses),
        pauses=pauses,
        transfer={"samples": samples, "dropped": dropped, "bytes": written},
        clock_pre=clock_pre,
        clock_post=clock_post,
        skew_ppm=skew,
        holds={"pre": hold_pre, "post": hold_post},
        session={
            "id": session_meta["id"],
            "name": session_meta.get("name", ""),
            "robot": session_meta.get("robot", ""),
        },
        provenance=provenance,
    )
    say(f"      wrote {stem}.toml")

    if problems:
        say(f"\n      VERDICT: discard -- {'; '.join(problems)}")
    else:
        say("\n      VERDICT: pass")
    return link, RunOutcome(dest.name, samples, dropped, verdict, problems, warnings)


def log_gates(
    log_path: Path,
    pauses: Sequence[Any],
    clock: ClockFit,
    holds: dict[str, tuple[float, float]] | None = None,
) -> list[Gate]:
    """Gates that need the downloaded log parsed, checked here rather than at fit time.

    Cheap enough at 1 kHz that it is not worth deferring: knowing at the robot beats finding
    out a week later when the fit excludes the run.

    Accel clipping is a warning here, not a gate. It was a gate until 2026-08-22, when it
    turned out to be discarding runs for the run card rather than for the data: every run
    ends with the operator picking the robot up to plug the USB cable back in, and that
    clips the accel outside the pause mask. Nothing in the plant model reads acceleration
    anyway, so the clip is worth seeing and never worth discarding a recording over.
    """
    try:
        from auto_battlebot.velocity_jig import pause_mask, read_jig_log

        log = read_jig_log(log_path)
    except (OSError, ValueError) as err:
        return [Gate(f"log unreadable: {err}")]

    out: list[Gate] = []
    # Whether the run survived is a question about the samples, not about how tidily the
    # session ended. A recording cut short by a reset, a card that stopped taking writes, or
    # a battery that browned out all look the same here: the log stops before the second
    # still hold, or it has holes in the middle.
    step = np.diff(log.t)
    late = int(np.count_nonzero(step > 5 * log.dt)) if len(step) else 0
    if late:
        out.append(Gate(f"{late} gaps in the log, the 1 kHz stream is not continuous"))
    for kind, window in (holds or {}).items():
        covered = clock.host_seconds(log.t[[0, -1]])
        if window[0] < covered[0] - 0.1 or window[1] > covered[1] + 0.1:
            out.append(Gate(f"the log stops short of the {kind} still hold"))

    # Setting the robot back on its mark clips the accel, so the operator's pauses come out
    # before anything is counted. What is left is the robot's own data.
    keep = ~pause_mask(clock.host_seconds(log.t), pauses)
    sat = [a for a in log.saturation(keep=keep) if a.count]
    gyro = [a for a in sat if a.name.startswith("g")]
    accel = [a for a in sat if a.name.startswith("a")]
    if gyro:
        axes = ", ".join(f"{a.name} x{a.count}" for a in gyro)
        out.append(Gate(f"gyro saturation on {axes}: the yaw rate is wrong from there on"))
    if accel:
        axes = ", ".join(f"{a.name} x{a.count}" for a in accel)
        peak = max(a.peak_raw for a in accel) * log.header.accel_g_per_lsb
        out.append(
            Gate(
                f"accel clipped on {axes}, peak {peak:.1f} g. Check the plot: an impact"
                " mid-run is worth knowing about, handling at either end is not",
                fatal=False,
            )
        )
    interior = log.malformed_rows - log.malformed_tail
    if interior:
        out.append(Gate(f"{interior} malformed rows inside the log"))
    if log.malformed_tail:
        out.append(
            Gate(
                "the log's last row is truncated: the board reset or lost power between the"
                " write and the flush. Every row before it parsed",
                fatal=False,
            )
        )
    if not log.encoder_moved:
        out.append(Gate("encoder count never changed; check the connector"))
    return out


def gates(
    dropped: int,
    clock_pre: ClockProbe,
    clock_post: ClockProbe,
    skew: float,
    samples: int,
    hold_s: float,
    contamination: float,
) -> list[Gate]:
    """Capture-time gates. The offline ones (bias drift, saturation, encoder slip) need the
    log parsed and live in `RunQuality.problems()`, not here."""
    out: list[Gate] = []
    if dropped:
        out.append(Gate(f"dropped={dropped} samples on the SD path"))
    if clock_pre.residual_ms > 2.0:
        out.append(Gate(f"clock pre residual {clock_pre.residual_ms:.2f} ms"))
    if clock_post.residual_ms > 2.0:
        out.append(Gate(f"clock post residual {clock_post.residual_ms:.2f} ms"))
    if clock_post.at_jig_ms < clock_pre.at_jig_ms:
        # The jig counts microseconds since its own boot, so a post-probe below the pre-probe
        # is a board that restarted rather than a clock that drifted. Reporting the ppm here
        # would be arithmetic on a counter that went backwards. Replugging the USB cable did
        # this on LOG-62, and the recording ended with it.
        #
        # Not fatal on its own. What the reboot costs is the skew correction, worth about
        # 0.9 ms per 30 s at the RP2040's 30 ppm and measured at 2.77 ppm on this board, and
        # the dropped-sample count, which the log's own timestamps show anyway. Whether the
        # reset truncated the run is a separate question, and `log_gates` answers it from the
        # samples instead of assuming the worst.
        out.append(
            Gate(
                f"the jig rebooted between clock probes (uptime "
                f"{clock_pre.at_jig_ms / 1e3:.0f} s -> {clock_post.at_jig_ms / 1e3:.0f} s): "
                f"no skew correction, and the stop counts are lost",
                fatal=False,
            )
        )
    elif abs(skew) > 200.0:
        out.append(Gate(f"clock skew {skew:.0f} ppm"))
    # The firmware logs on a fixed 1 kHz timer, not at the IMU's 1660 Hz ODR. The old tool
    # compared against 1660 and so failed every real run while passing against its mock.
    if samples and samples < (2 * hold_s) * 1000.0 * 0.5:
        out.append(Gate(f"only {samples} samples, short for the still holds alone"))
    if contamination > 0.05:
        out.append(Gate(f"stick contamination {contamination:.3f}"))
    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--waveforms", type=Path, default=DEFAULT_CATALOG, help="catalog TOML")
    parser.add_argument(
        "--waveform",
        action="append",
        default=[],
        help="waveform to play; repeat to queue several runs back to back",
    )
    parser.add_argument("--name", default="", help="experiment name, recorded in the metadata")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help=(
            "session output directory. Defaults to "
            f"{DEFAULT_OUT_ROOT}/<date>-<name>, so repeated runs on the same day land in "
            "one session, which is the unit leave-one-out validation holds out."
        ),
    )
    parser.add_argument("--session", type=Path, default=None, help="session metadata TOML")
    parser.add_argument("--reps", type=int, default=1, help="repetitions of each waveform")
    parser.add_argument("--jig-port", default=None)
    parser.add_argument("--tx-port", default=None)
    parser.add_argument("--probes", type=int, default=DEFAULT_PROBES, help="TIME probes per burst")
    parser.add_argument("--hold-s", type=float, default=DEFAULT_HOLD_S, help="still hold seconds")
    parser.add_argument("--rate", type=float, default=50.0, help="command send rate in Hz")
    parser.add_argument("--mix", choices=("tank", "direct"), default="tank")
    parser.add_argument("--no-reverse-angular", action="store_true")
    # Readback wiring, from --check-radio. These affect only how the radio's own report is
    # interpreted, never what gets sent, so a wrong value costs a false contamination flag
    # rather than a wrong command.
    parser.add_argument("--read-linear", type=int, default=0)
    parser.add_argument("--read-angular", type=int, default=1)
    parser.add_argument("--read-arm", type=int, default=4)
    parser.add_argument("--read-invert-a", action="store_true")
    # The radio has servo-reverse set on drive channel B, so it reports that channel negated
    # while the ESC on that side compensates. Reading it straight un-mixes every straight-line
    # command into a pure turn, which is what discarded LOG-57 and LOG-62 on 2026-08-20.
    parser.add_argument("--no-read-invert-b", action="store_true")
    parser.add_argument("--channel-scale", type=float, default=1024.0)
    parser.add_argument(
        "--trim",
        type=float,
        default=0.0,
        help="angular offset added to counter the straight-line arc; logged, never hidden",
    )
    parser.add_argument(
        "--no-drive",
        action="store_true",
        help="run the full ceremony without arming the transmitter (hand-pushed runs)",
    )
    parser.add_argument("--encoder", default="attached", choices=("attached", "detached", "either"))
    parser.add_argument("--notes", default="")
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="skip the per-run notes prompt, and play each waveform straight through",
    )
    parser.add_argument(
        "--no-pause",
        action="store_true",
        help=(
            "play each waveform straight through. By default the run stops between cells so "
            "the robot can be put back on its mark; a grid waveform has one stop per cell."
        ),
    )
    parser.add_argument("--half-width-m", type=float, default=0.0, help="usable floor half-width")
    parser.add_argument("--params", type=Path, default=None, help="plant TOML for --dry-run")
    parser.add_argument(
        "--check-radio",
        action="store_true",
        help="infer the radio's channel mapping, signs and scale. Run with the robot OFF.",
    )
    parser.add_argument("--check-amplitude", type=float, default=0.5)
    parser.add_argument("--list-waveforms", action="store_true")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="inspect programs, touch no hardware")
    return parser


def do_list_ports() -> None:
    say("Serial ports:")
    for info in jl.list_ports():
        tag = ""
        if info.usb_id in jl.JIG_USB_IDS:
            tag = "  <- jig (by usb id)"
        elif info.usb_id in jl.TRANSMITTER_USB_IDS:
            tag = "  <- transmitter"
        say(f"  {info.describe()}{tag}")
    found = jl.find_jig_port()
    say(f"\nfind_jig_port() -> {found.device if found else None}")
    say(f"find_transmitter_port() -> {dp.find_transmitter_port()}")
    if found is not None and found.usb_id not in jl.JIG_USB_IDS:
        say(
            f"\nNOTE: the jig matched by name, not by usb id. Add {found.usb_id} to "
            "JIG_USB_IDS in calib_lib/jig_link.py so it is matched exactly."
        )


def mix_from_args(args: argparse.Namespace) -> dp.MixConfig:
    return dp.MixConfig(
        mode=args.mix,
        reverse_angular=not args.no_reverse_angular,
        channel_scale=args.channel_scale,
        read_linear=args.read_linear,
        read_angular=args.read_angular,
        read_arm=args.read_arm,
        read_invert_a=args.read_invert_a,
        read_invert_b=not args.no_read_invert_b,
    )


def do_check_radio(link: dp.TrainerLink, amplitude: float = 0.5) -> None:
    """Work out how the radio reports what we send it, without guessing.

    Run this with the ROBOT POWERED OFF. The radio's mixer responds to trainer input whether
    or not anything is listening downstream, so nothing has to move for this to work.

    Sends five known commands and regresses every returned channel against them. That
    recovers which channel carries what, its sign, and its scale. A radio with servo-reverse
    set on one drive channel reports it negated while the ESC compensates, so the robot
    drives correctly and only the readback disagrees. Left unfound, that shows up as a
    contamination gate firing on runs that were fine.
    """
    probes = [(0.0, 0.0), (amplitude, 0.0), (-amplitude, 0.0), (0.0, amplitude), (0.0, -amplitude)]
    say("Radio check. Robot OFF; nothing needs to move.")
    say(f"Sending {len(probes)} known commands at amplitude {amplitude:g}.\n")

    rows: list[tuple[float, float, list[float]]] = []
    with dp.armed(link):
        for lin, ang in probes:
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                link.send(lin, ang)
                time.sleep(0.02)
            raw = link.measured_raw()
            if not raw:
                raise SystemExit(
                    "No channel packets from the radio. Is it streaming? The link primes "
                    "'telemetry on' and 'channels on' at open."
                )
            rows.append((lin, ang, raw))
            say(f"  sent lin {lin:+.2f} ang {ang:+.2f}")

    import numpy as np

    design = np.array([[1.0, lin, ang] for lin, ang, _ in rows])
    n_ch = min(len(r[2]) for r in rows)
    observed = np.array([r[2][:n_ch] for r in rows])
    coeff, *_ = np.linalg.lstsq(design, observed, rcond=None)

    say("\nChannels that respond (offset, per unit linear, per unit angular):")
    responders = []
    for ch in range(n_ch):
        offset, to_lin, to_ang = coeff[:, ch]
        if max(abs(to_lin), abs(to_ang)) < 0.05:
            continue
        responders.append((ch, offset, to_lin, to_ang))
        say(f"  ch{ch:<3d} offset {offset:+.3f}   lin {to_lin:+.3f}   ang {to_ang:+.3f}")
    if len(responders) < 2:
        say("\nFewer than two channels responded. Check that the trainer input is enabled")
        say("on the radio and that the model's mixer routes it to the drive channels.")
        return

    a_ch, _, a_lin, a_ang = responders[0]
    b_ch, _, b_lin, b_ang = responders[1]
    # Under tank, both channels carry the linear command; under direct they split it.
    tank = abs(a_lin) > 0.05 and abs(b_lin) > 0.05
    mode = "tank" if tank else "direct"
    scale = max(abs(a_lin), abs(a_ang), abs(b_lin), abs(b_ang))

    say(f"\nInferred mix: {mode}")
    say(f"Response magnitude {scale:.3f} per unit command.")
    if abs(scale - 1.0) > 0.05:
        say(
            f"  That is not 1.0, so channel_scale is off by {1.0 / scale:.3f}x. "
            f"Set channel_scale = {link.mix.channel_scale / scale:.0f}."
        )
    invert_a = (a_lin if tank else a_lin or a_ang) < 0
    invert_b = (b_lin if tank else b_ang) < 0
    if invert_a or invert_b:
        say(
            "  One or both drive channels come back negated, which is servo-reverse on the "
            "radio. The robot still drives correctly; only the readback needs to undo it."
        )

    say("\nUse:")
    say(f"  --mix {mode}")
    say(f"  read_linear = {a_ch}, read_angular = {b_ch}")
    say(f"  read_invert_a = {str(invert_a).lower()}, read_invert_b = {str(invert_b).lower()}")
    say(
        "\nThese live in MixConfig (calib_lib/drive_protocol.py). Set them before recording, "
        "or the contamination gate will discard good runs."
    )


def do_list_waveforms(catalog: dict[str, ex.WaveformDecl]) -> None:
    say(f"{'name':<20} {'kind':<10} {'channel':<9} {'role':<8} {'duration':>9}")
    say("-" * 62)
    for name, decl in catalog.items():
        program = ex.build(decl)
        say(
            f"{name:<20} {decl.kind:<10} {decl.channel:<9} {decl.role:<8} "
            f"{program.duration_s:>7.1f} s"
        )


def main() -> None:
    args = build_parser().parse_args()

    if args.list_ports:
        do_list_ports()
        return

    if args.check_radio:
        tx_port = args.tx_port or dp.find_transmitter_port()
        if tx_port is None:
            raise SystemExit("No transmitter found. Pass --tx-port.")
        link = dp.TrainerLink(tx_port, mix_from_args(args))
        try:
            do_check_radio(link, args.check_amplitude)
        finally:
            link.close()
        return

    catalog = ex.load_catalog(args.waveforms)
    if args.list_waveforms:
        do_list_waveforms(catalog)
        return

    if not args.waveform:
        raise SystemExit("--waveform is required (see --list-waveforms)")
    unknown = [w for w in args.waveform if w not in catalog]
    if unknown:
        raise SystemExit(f"unknown waveform(s): {', '.join(unknown)} (see --list-waveforms)")

    decls = [catalog[w] for w in args.waveform]
    programs = [ex.build(d) for d in decls]

    if args.dry_run:
        params = PlantParams.from_toml(args.params) if args.params else PlantParams()
        do_dry_run(programs, params, args.half_width_m)
        return

    out_dir = args.out or default_out_dir(args.name)
    out_dir.mkdir(parents=True, exist_ok=True)

    info = jl.find_jig_port(args.jig_port)
    if info is None:
        raise SystemExit("No jig found. Run --list-ports, or pass --jig-port.")
    if info.usb_id is not None and info.usb_id not in jl.JIG_USB_IDS:
        say(
            f"NOTE: jig matched by name on {info.device} ({info.usb_id}), not by usb id. "
            "Add it to JIG_USB_IDS."
        )

    mix = mix_from_args(args)
    trainer: dp.TrainerLink | None = None
    if not args.no_drive:
        tx_port = args.tx_port or dp.find_transmitter_port()
        if tx_port is None:
            raise SystemExit("No transmitter found. Pass --tx-port, or use --no-drive.")
        trainer = dp.TrainerLink(tx_port, mix)
        say(f"transmitter: {tx_port} (mix={mix.mode})")

    provenance = {
        "drive_cli_sha": git_sha(),
        # Repo-relative so the session file stays readable after the tree moves.
        "waveform_toml": str(_repo_relative(args.waveforms)),
        # How the radio's channels were read. Recorded because meas_linear/meas_angular are
        # these settings applied to the raw channels, so reinterpreting a run later needs to
        # know which settings produced them.
        "mix": mix.mode,
        "reverse_angular": mix.reverse_angular,
        "channel_scale": mix.channel_scale,
        "read_linear": mix.read_linear,
        "read_angular": mix.read_angular,
        "read_invert_a": mix.read_invert_a,
        "read_invert_b": mix.read_invert_b,
    }
    session_meta = {
        "id": out_dir.name,
        "name": args.name,
        "started_utc": f"{datetime.now(timezone.utc):%Y-%m-%dT%H:%M:%SZ}",
    }
    if args.session is not None:
        import tomllib

        with open(args.session, "rb") as handle:
            session_meta.update(tomllib.load(handle).get("session", {}))
        session_meta.setdefault("id", out_dir.name)
        session_meta["name"] = args.name or session_meta.get("name", "")
    write_session_toml(out_dir / "session.toml", session_meta, provenance)

    say(f"jig: {info.device} (sn={info.serial_number})")
    say(f"session: {out_dir}  name={session_meta['name']!r}")
    say(f"queued: {', '.join(f'{w} x{args.reps}' for w in args.waveform)}")
    say("\nSAFETY: guard plates on, weapon disabled, driver's sticks CENTERED.")
    say("Verify the disarm now with a deliberate Ctrl-C during the first run.")

    link = jl.JigLink(info.device, serial_number=info.serial_number)
    outcomes: list[RunOutcome] = []
    try:
        for rep in range(args.reps):
            for decl, program in zip(decls, programs):
                link, outcome = run_one(
                    args, decl, program, link, trainer, out_dir, rep, session_meta, provenance
                )
                outcomes.append(outcome)
    except KeyboardInterrupt:
        say("\n\naborted by Ctrl-C")
    finally:
        # Every exit path lands here: normal return, exception, and Ctrl-C.
        if trainer is not None:
            trainer.close()
            say("transmitter disarmed")
        link.close()

    if outcomes:
        say(f"\n{'=' * 70}\nSESSION SUMMARY  {out_dir}")
        for o in outcomes:
            flag = "pass " if o.verdict == "pass" else "DISC "
            say(f"  {flag} {o.log_file:<14} n={o.samples:<8} dropped={o.dropped}")
            for problem in o.problems:
                say(f"         {problem}")
        kept = sum(1 for o in outcomes if o.verdict == "pass")
        say(f"\n{kept}/{len(outcomes)} runs usable")


if __name__ == "__main__":
    main()
