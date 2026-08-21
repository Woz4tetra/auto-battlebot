#!/usr/bin/env python3
"""Recompute the capture-time verdict of runs already on disk.

A sidecar records what the gates decided at the bench, and the gates change. Two of them
changed on 2026-08-20: operator pauses are now cut out before anything is counted, and the
radio's channel B is now known to come back negated. Both of those turned a discard into a
pass without a single sample of the recording being different, and the only honest way to
carry that forward is to run the current gates over the runs that are already captured.

What this rewrites, per run:

- `[[pause]]` gains the host window it always described. Sidecars written before the runner
  recorded it carry only the program time and the seconds held, so the window comes back
  from the gap it left in the command stream.
- `meas_linear`/`meas_angular` in the command CSV, re-derived from `meas_ch_a`/`meas_ch_b`
  through the corrected mix. The channels are the measurement and are left alone; those two
  columns were only ever an interpretation of them.
- `contamination`, `gates` and `verdict`, recomputed from the above.

An operator's own judgement is never overwritten. A run whose gate note starts with
`operator:`, or whose notes say to discard it, keeps its verdict no matter what the gates
now say: the tool cannot see the wall the robot hit.

    ./rederive_sidecars.py out/2026-08-20            # show what would change
    ./rederive_sidecars.py out/2026-08-20 --write    # apply it
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import tomllib

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calib_lib import drive_protocol as dp  # noqa: E402
from velocity_jig_drive import (  # noqa: E402
    COMMAND_PREAMBLE,
    command_row,
    gates,
    log_gates,
    mix_from_args,
    write_run_toml,
)

from auto_battlebot.velocity_jig import (  # noqa: E402
    ClockFit,
    ClockProbe,
    PauseWindow,
    ProtocolSegment,
    pause_windows_from_commands,
    read_command_log,
)

OPERATOR_PREFIX = "operator:"


def operator_held(run: dict[str, Any]) -> str:
    """The operator's own reason for discarding, or empty when the gates decided."""
    note = str(run.get("gates", ""))
    if note.startswith(OPERATOR_PREFIX):
        return note
    if run.get("verdict") == "discard" and str(run.get("notes", "")).strip().lower() == "discard":
        return f"{OPERATOR_PREFIX} discarded at the bench"
    return ""


def commands_from_csv(path: Path, mix: dp.MixConfig) -> list[dp.CommandSample]:
    """Rebuild the tick log, reading the radio's channels through the corrected mix."""
    cols = read_command_log(path)
    t = cols["t_host_s"]
    out: list[dp.CommandSample] = []
    for i in range(len(t)):
        ch_a = float(cols["meas_ch_a"][i]) if "meas_ch_a" in cols else float("nan")
        ch_b = float(cols["meas_ch_b"][i]) if "meas_ch_b" in cols else float("nan")
        lin, ang = (
            (float("nan"), float("nan"))
            if ch_a != ch_a or ch_b != ch_b
            else mix.from_channels(ch_a, ch_b)
        )
        out.append(
            dp.CommandSample(
                t=float(t[i]),
                linear=float(cols["linear"][i]),
                angular=float(cols["angular"][i]),
                trim=float(cols["trim"][i]) if "trim" in cols else 0.0,
                channel_a=0,
                channel_b=0,
                meas_ch_a=ch_a,
                meas_ch_b=ch_b,
                meas_linear=lin,
                meas_angular=ang,
                meas_weapon=(
                    float(cols["meas_weapon"][i]) if "meas_weapon" in cols else float("nan")
                ),
                meas_arm=float(cols["meas_arm"][i]) if "meas_arm" in cols else float("nan"),
            )
        )
    return out


def rewrite_command_csv(path: Path, samples: list[dp.CommandSample], note: str) -> None:
    """Rewrite only the derived columns, keeping every recorded channel byte for byte.

    The header is reissued from the current preamble rather than preserved, because a file
    written before a column existed carries a `# columns:` line that no longer describes its
    own rows. Channels the older run never recorded come back as nan, which is the truth.
    """
    kept = ("# rate_hz=", "# meas_linear/meas_angular re-derived")
    meta = [line for line in path.read_text().splitlines() if line.startswith(kept)]
    lines = COMMAND_PREAMBLE + meta + [f"# {note}"] + [command_row(s) for s in samples]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def rederive(toml_path: Path, mix: dp.MixConfig, hold_s: float, write: bool) -> list[str]:
    """Recompute one run. Returns the lines describing what changed."""
    data = tomllib.loads(toml_path.read_text())
    run = dict(data.get("run", {}))
    log_path = toml_path.parent / str(run.get("log_file", ""))
    if not log_path.exists():
        return [f"{toml_path.name}: log {log_path.name} missing, skipped"]

    clock_pre = ClockProbe.from_toml(data.get("clock", {}).get("pre"))
    clock_post = ClockProbe.from_toml(data.get("clock", {}).get("post"))
    clock = ClockFit.from_probes(clock_pre, clock_post)

    csv_path = toml_path.parent / str(run.get("command_file", ""))
    samples: list[dp.CommandSample] = []
    if csv_path.exists():
        samples = commands_from_csv(csv_path, mix)

    entries = data.get("pause", [])
    parsed = [PauseWindow.from_toml(entry) for entry in entries]
    if entries and all(p is not None for p in parsed):
        pauses = [p for p in parsed if p is not None]
    else:
        program_t = [float(e.get("program_t", 0.0)) for e in entries]
        cmd_t = read_command_log(csv_path)["t_host_s"] if csv_path.exists() else []
        pauses = pause_windows_from_commands(cmd_t, program_t) if entries else []

    result = dp.PlayResult(commands=samples, pauses=pauses)
    contamination = result.contamination if samples else float(run.get("contamination", 0.0))

    hold = data.get("hold", {})
    holds = {
        kind: (float(w["start"]), float(w["end"]))
        for kind, w in hold.items()
        if "start" in w and "end" in w
    }
    transfer = dict(data.get("transfer", {}))
    found = gates(
        int(transfer.get("dropped", 0) or 0),
        clock_pre,
        clock_post,
        float(data.get("clock", {}).get("skew_ppm", 0.0) or 0.0),
        int(transfer.get("samples", 0) or 0),
        hold_s,
        contamination,
    )
    found += log_gates(log_path, pauses, clock, holds)
    problems = [g.reason for g in found if g.fatal]
    warnings = [g.reason for g in found if not g.fatal]

    held = operator_held(run)
    verdict = "discard" if (problems or held) else "pass"
    gate_note = held if held else "; ".join(problems)
    warning_note = "; ".join(warnings)

    changes = []
    if run.get("verdict") != verdict:
        changes.append(f"verdict {run.get('verdict')!r} -> {verdict!r}")
    if str(run.get("gates", "")) != gate_note:
        changes.append(f"gates {run.get('gates', '')!r} -> {gate_note!r}")
    if str(run.get("warnings", "")) != warning_note:
        changes.append(f"warnings {run.get('warnings', '')!r} -> {warning_note!r}")
    if abs(float(run.get("contamination", 0.0)) - contamination) > 1e-6:
        changes.append(
            f"contamination {float(run.get('contamination', 0.0)):.3f} -> {contamination:.3f}"
        )
    if entries and not all(p is not None for p in parsed):
        changes.append(f"{len(pauses)} pause windows recovered from the command gaps")
    if not changes:
        return [f"{toml_path.name}: unchanged"]
    if not write:
        return [f"{toml_path.name}: " + "; ".join(changes)]

    run["verdict"] = verdict
    run["gates"] = gate_note
    run["warnings"] = warning_note
    run["contamination"] = round(contamination, 6)
    provenance = dict(data.get("provenance", {}))
    provenance["read_invert_a"] = mix.read_invert_a
    provenance["read_invert_b"] = mix.read_invert_b
    write_run_toml(
        toml_path,
        run=run,
        spec={k: v for k, v in data.get("waveform", {}).items() if k != "params"},
        params=dict(data.get("waveform", {}).get("params", {})),
        segments=[ProtocolSegment.from_toml(seg) for seg in data.get("segment", [])],
        pauses=pauses,
        transfer=transfer,
        clock_pre=clock_pre,
        clock_post=clock_post,
        skew_ppm=data.get("clock", {}).get("skew_ppm"),
        holds=holds,
        session=dict(data.get("session", {})),
        provenance=provenance,
    )
    if samples:
        rewrite_command_csv(
            csv_path,
            samples,
            f"meas_linear/meas_angular re-derived from the channels with "
            f"read_invert_a={str(mix.read_invert_a).lower()} "
            f"read_invert_b={str(mix.read_invert_b).lower()}",
        )
    return [f"{toml_path.name}: " + "; ".join(changes)]


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("session", type=Path, nargs="+", help="session directories to re-derive")
    parser.add_argument(
        "--write", action="store_true", help="apply the changes, not just print them"
    )
    parser.add_argument(
        "--hold-s", type=float, default=5.0, help="still hold the runs were captured with"
    )
    parser.add_argument("--mix", choices=("tank", "direct"), default="tank")
    parser.add_argument("--no-reverse-angular", action="store_true")
    parser.add_argument("--read-linear", type=int, default=0)
    parser.add_argument("--read-angular", type=int, default=1)
    parser.add_argument("--read-arm", type=int, default=4)
    parser.add_argument("--read-invert-a", action="store_true")
    parser.add_argument("--no-read-invert-b", action="store_true")
    parser.add_argument("--channel-scale", type=float, default=1024.0)
    args = parser.parse_args()

    mix = mix_from_args(args)
    for session in args.session:
        for toml_path in sorted(session.glob("LOG-*.toml")):
            for line in rederive(toml_path, mix, args.hold_s, args.write):
                print(line)
    if not args.write:
        print("\nnothing written. Re-run with --write to apply.")


if __name__ == "__main__":
    main()
