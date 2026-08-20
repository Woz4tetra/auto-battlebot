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
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence


from auto_battlebot.plant import PlantParams, simulate
from auto_battlebot.velocity_jig import SIDECAR_SCHEMA, ClockProbe
from calib_lib import drive_protocol as dp
from calib_lib import excitation as ex
from calib_lib import jig_link as jl

DEFAULT_CATALOG = Path(__file__).resolve().parent / "waveforms.toml"
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


def prompt(msg: str) -> None:
    try:
        input(f"      {msg} ")
    except EOFError:
        raise SystemExit("\naborted: stdin closed") from None


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


def write_command_csv(path: Path, samples: Sequence[dp.CommandSample], meta: str) -> None:
    header = [
        "# auto-battlebot velocity jig command log",
        "# linear/angular are what was asked for; meas_ch_a/meas_ch_b are what the radio",
        "# reported on the two drive channels. meas_linear/meas_angular are those channels",
        "# read through the configured mix, so they are an interpretation and the channels",
        "# are the measurement. A wrong mix can be corrected from the channels offline.",
        "# columns: t_host_s,linear,angular,trim,meas_ch_a,meas_ch_b,"
        "meas_linear,meas_angular,meas_arm",
        f"# {meta}",
    ]
    rows = [
        f"{s.t:.6f},{s.linear:.4f},{s.angular:.4f},{s.trim:.4f},"
        f"{s.meas_ch_a:.4f},{s.meas_ch_b:.4f},"
        f"{s.meas_linear:.4f},{s.meas_angular:.4f},{s.meas_arm:.4f}"
        for s in samples
    ]
    path.write_text("\n".join(header + rows) + "\n", encoding="utf-8")


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


@dataclass
class RunOutcome:
    log_file: str
    samples: int
    dropped: int
    verdict: str
    problems: list[str]


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
    total = 11 if decl.motion else 9
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
    if trainer is not None and decl.kind != "manual":
        prompt("Type Enter to ARM and play (Ctrl-C aborts and disarms):")
        with dp.armed(trainer):
            result = dp.play(
                trainer,
                program,
                rate_hz=args.rate,
                trim=args.trim,
                timeout_s=program.duration_s + 10.0,
            )
        commands = result.commands
        contamination = result.contamination
        say(f"      {len(commands)} commands, completed={result.completed}")
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
        for t, ch_a, ch_b, lin, ang, arm in dp.stream_measured(
            trainer, program.duration_s, args.rate
        ):
            commands.append(
                dp.CommandSample(
                    t=t,
                    linear=0.0,
                    angular=0.0,
                    trim=0.0,
                    channel_a=0,
                    channel_b=0,
                    meas_ch_a=ch_a,
                    meas_ch_b=ch_b,
                    meas_linear=lin,
                    meas_angular=ang,
                    meas_arm=arm,
                    label=program.kind,
                )
            )
        say(f"      {len(commands)} radio samples")
    else:
        countdown(program.duration_s, "run the maneuver")

    n += 1
    step(n, total, f"Hold still {args.hold_s:.0f} s (bias drift bound)")
    hold_post = countdown(args.hold_s, "hold still")

    if decl.motion:
        n += 1
        step(n, total, "Plug the USB cable back in")
        link = reopen_after_replug(link)
        say(f"      reconnected on {link.port}")

    n += 1
    # Deliberately after the replug: the stop summary goes to the wire unbuffered, so a
    # press while unplugged loses the sample and dropped counts for good.
    step(n, total, "Press B on the jig to stop recording")
    samples, dropped = link.wait_stopped()
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

    problems = gates(dropped, clock_pre, clock_post, skew, samples, args.hold_s, contamination)
    problems += log_gates(dest)
    verdict = "discard" if problems else "pass"
    # The reasons go in the sidecar too. A verdict with no reason means working out months
    # later whether a run was thrown away for a real fault or a since-fixed tool bug.
    gate_note = "; ".join(problems)
    notes = args.notes
    if not args.no_prompt:
        try:
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
            "duration_s": program.duration_s,
            "verdict": verdict,
            "gates": gate_note,
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
        segments=program.segments or [],
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
    return link, RunOutcome(dest.name, samples, dropped, verdict, problems)


def log_gates(log_path: Path) -> list[str]:
    """Gates that need the downloaded log parsed, checked here rather than at fit time.

    Saturation is the one that matters at the robot: a clipped IMU channel usually means an
    impact, and knowing that before the next run beats finding out a week later when the fit
    excludes it. Cheap enough at 1 kHz that it is not worth deferring.
    """
    try:
        from auto_battlebot.velocity_jig import read_jig_log

        log = read_jig_log(log_path)
    except (OSError, ValueError) as err:
        return [f"log unreadable: {err}"]

    out: list[str] = []
    sat = [a for a in log.saturation() if a.count]
    if sat:
        axes = ", ".join(f"{a.name} x{a.count}" for a in sat)
        out.append(f"IMU saturation on {axes} (impact, or the range is set too low)")
    if log.malformed_rows:
        out.append(f"{log.malformed_rows} malformed rows in the log")
    if not log.encoder_moved:
        out.append("encoder count never changed; check the connector")
    return out


def gates(
    dropped: int,
    clock_pre: ClockProbe,
    clock_post: ClockProbe,
    skew: float,
    samples: int,
    hold_s: float,
    contamination: float,
) -> list[str]:
    """Capture-time gates. The offline ones (bias drift, saturation, encoder slip) need the
    log parsed and live in `RunQuality.problems()`, not here."""
    out: list[str] = []
    if dropped:
        out.append(f"dropped={dropped} samples on the SD path")
    if clock_pre.residual_ms > 2.0:
        out.append(f"clock pre residual {clock_pre.residual_ms:.2f} ms")
    if clock_post.residual_ms > 2.0:
        out.append(f"clock post residual {clock_post.residual_ms:.2f} ms")
    if abs(skew) > 200.0:
        out.append(f"clock skew {skew:.0f} ppm")
    # The firmware logs on a fixed 1 kHz timer, not at the IMU's 1660 Hz ODR. The old tool
    # compared against 1660 and so failed every real run while passing against its mock.
    if samples and samples < (2 * hold_s) * 1000.0 * 0.5:
        out.append(f"only {samples} samples, short for the still holds alone")
    if contamination > 0.05:
        out.append(f"stick contamination {contamination:.3f}")
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
    parser.add_argument("--read-invert-b", action="store_true")
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
    parser.add_argument("--no-prompt", action="store_true", help="skip the per-run notes prompt")
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
        read_invert_b=args.read_invert_b,
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
        "waveform_toml": str(args.waveforms),
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
