"""Plot each velocity jig run with its commands overlaid, skipping ones already plotted."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

DEFAULT_CALIBRATION = Path(__file__).resolve().parent / "jig_calibration.toml"

from auto_battlebot.calibration.jig.velocity_jig import (
    G_MPS2,
    RAW_SATURATION,
    ClockFit,
    JigCalibration,
    JigLog,
    PauseWindow,
    RunRecord,
    find_still_segments,
    overlaps_pause,
    pause_mask,
    read_jig_log,
    still_stats,
)

DEFAULT_OUT_ROOT = Path(__file__).resolve().parent / "out"
SMOOTH_S = 0.05


def smooth(values: np.ndarray, t: np.ndarray, window_s: float) -> np.ndarray:
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 1.0
    n = max(1, int(round(window_s / max(dt, 1e-9))))
    if n < 2 or len(values) < n:
        return values
    kernel = np.ones(n) / n
    pad = n // 2
    padded = np.concatenate([np.full(pad, values[0]), values, np.full(n - 1 - pad, values[-1])])
    return np.convolve(padded, kernel, mode="valid")


def yaw_rate_dps(log: JigLog) -> np.ndarray:
    """Yaw about the measured gravity axis, in deg/s.

    JigLog.gyro is rad/s, so this converts. The panel axis says deg/s and a robot spinning
    at 700 deg/s plotted as 12 reads as a robot going perfectly straight.

    Taken from gravity rather than from a named axis so a tilted or rotated remount does not
    silently invalidate the plot. Falls back to the z row when there is no still segment to
    measure gravity from.
    """
    stills = find_still_segments(log)
    if not stills:
        return np.degrees(log.gyro[:, 2])
    st = still_stats(log, stills[0])
    return np.degrees((log.gyro - st.bias) @ st.up)


def forward_speed(log: JigLog, calib: JigCalibration | None) -> tuple[np.ndarray, str]:
    v = np.gradient(log.count.astype(float), log.t)
    v = smooth(v, log.t, SMOOTH_S)
    if calib is not None:
        return v * calib.meters_per_count, "forward speed (m/s)"
    return v, "encoder rate (counts/s)"


REGION_STYLE = {
    "still hold": ("#cfe3cf", 0.55),
    "not commanded": ("#d9d9d9", 0.55),
    "operator pause": ("#cdd8f0", 0.75),
    "gyro saturated": ("#f3b3ae", 0.85),
}

ACCEL_CLIP_COLOUR = "#c2185b"


def excluded_regions(
    log: JigLog,
    t: np.ndarray,
    record: RunRecord | None,
) -> dict[str, list[tuple[float, float]]]:
    """Stretches the plant fit does not learn from, in plot time."""
    out: dict[str, list[tuple[float, float]]] = {}

    clock = record.clock if record is not None else ClockFit.identity()
    offset = t[0] - clock.host_seconds(log.t[:1])[0] if len(t) else 0.0
    pauses = record.pauses if record is not None else []
    if pauses:
        out["operator pause"] = [(p.t_start + offset, p.t_end + offset) for p in pauses]

    # A pause looks motionless once the robot is back down, so those stretches are not still
    # holds. Same rule the fit uses.
    stills = [
        seg
        for seg in find_still_segments(log)
        if not overlaps_pause(clock.host_seconds(np.array([seg.t0, seg.t1])), pauses)
    ]
    if stills:
        out["still hold"] = [
            tuple(clock.host_seconds(np.array([seg.t0, seg.t1])) + offset) for seg in stills
        ]

    if record is not None and record.has_commands:
        anchor = float(record.cmd_t[0])
        lo = 0.0
        hi = float(record.cmd_t[-1]) + 2.0 - anchor
        edges = []
        if t[0] < lo:
            edges.append((float(t[0]), lo))
        if t[-1] > hi:
            edges.append((hi, float(t[-1])))
        if edges:
            out["not commanded"] = edges

    sat, _ = saturated_mask(log, pauses, clock, sensors="g")
    if sat.any():
        out["gyro saturated"] = spans(sat, t)
    return out


def plot_radio_panel(axes, record, have_cmd, t0):
    ax = axes[1]
    ch_a, ch_b = _radio_channels(record) if have_cmd else (None, None)
    if ch_a is not None:
        names = ("left", "right") if _is_tank(record) else ("channel A", "channel B")
        ct, ca, cb = from_rest(record.cmd_t - t0, ch_a, ch_b)
        ax.step(ct, ca, where="post", lw=1.2, color="C2", label=names[0])
        ax.step(ct, cb, where="post", lw=1.2, color="C3", label=names[1])
        ax.set_ylim(-1.05, 1.05)
        ax.legend(fontsize=8, ncol=2, loc="upper right")
        ax.set_ylabel("radio reported", fontsize=9)
        inverted = bool(record.provenance.get("read_invert_a")) or bool(
            record.provenance.get("read_invert_b")
        )
        if _channels_opposed(ch_a, ch_b) and not inverted:
            ax.text(
                0.01,
                0.06,
                "channels move opposite while a straight command is held: one is reversed "
                "on the radio (see --check-radio)",
                transform=ax.transAxes,
                fontsize=7.5,
                color="C3",
            )
    else:
        ax.text(
            0.5,
            0.5,
            "no radio readback in this run",
            ha="center",
            va="center",
            transform=ax.transAxes,
            color="0.5",
        )
        ax.set_ylabel("radio reported", fontsize=9)


def plot_accel_panel(axes, log, t):
    ax = axes[4]
    limit = log.header.accel_raw_limit * log.header.accel_g_per_lsb
    for i, axis in enumerate(("x", "y", "z")):
        ax.plot(t, log.accel[:, i] / G_MPS2, lw=0.7, label=f"a{axis}")
    ax.axhline(limit, color="C3", ls="--", lw=0.9)
    ax.axhline(-limit, color="C3", ls="--", lw=0.9, label=f"clip at +/-{limit:.1f} g")
    ax.set_ylabel("accel (g)", fontsize=9)
    return ax


def plot_dead_reckoning_panel(axes, calib, log, t, have_cmd):
    ax = axes[5]
    if calib is not None:
        distance = log.count.astype(float) * calib.meters_per_count
        ax.plot(t, distance, lw=1.0, color="C4", label="distance")
        ax.set_ylabel("distance (m)", fontsize=9)
    else:
        ax.plot(t, log.count, lw=1.0, color="C4", label="encoder count")
        ax.set_ylabel("enc count", fontsize=9)
    ax.axhline(0.0, color="0.55", lw=1.0, zorder=1)
    ax.set_xlabel(
        "time (s, from the first command)" if have_cmd else "time (s, jig clock)", fontsize=10
    )
    return ax


def shade_excluded_regions(regions, axes, fig):
    handles = []
    for name, windows in regions.items():
        colour, alpha = REGION_STYLE[name]
        for lo, hi in windows:
            for a in axes:
                a.axvspan(lo, hi, color=colour, alpha=alpha, lw=0, zorder=0)
        handles.append(Patch(facecolor=colour, alpha=alpha, label=f"{name} (not fitted)"))
    if handles:
        fig.legend(
            handles=handles,
            loc="lower center",
            ncol=len(handles),
            fontsize=8,
            frameon=False,
            bbox_to_anchor=(0.5, 0.0),
        )


def plot_run(
    log_path: Path,
    out_path: Path,
    record: RunRecord | None,
    calib: JigCalibration | None,
) -> None:
    log = read_jig_log(log_path)
    clock = record.clock if record is not None else ClockFit.identity()
    t = clock.host_seconds(log.t)
    have_cmd = record is not None and record.has_commands
    t0 = float(record.cmd_t[0]) if have_cmd else float(t[0])
    t = t - t0

    speed, speed_label = forward_speed(log, calib)
    yaw = yaw_rate_dps(log)
    if calib is not None:
        yaw = yaw * calib.gyro_scale

    regions = excluded_regions(log, t, record)

    fig, axes = plt.subplots(
        6,
        1,
        figsize=(13, 12.5),
        sharex=True,
        gridspec_kw={"height_ratios": [1.0, 1.0, 1.5, 1.5, 1.2, 1.0]},
    )

    # 1. What the tool asked for, in body units.
    ax = axes[0]
    if have_cmd:
        req_lin, req_ang = _requested(record)
        ct, rl, ra = from_rest(record.cmd_t - t0, req_lin, req_ang)
        ax.step(ct, rl, where="post", lw=1.4, color="C0", label="linear")
        ax.step(ct, ra, where="post", lw=1.4, color="C1", label="angular")
        for seg in record.segments:
            ax.axvline(seg.t0, color="0.85", lw=0.6, zorder=0)
        ax.set_ylim(-1.05, 1.05)
        ax.legend(fontsize=8, ncol=2, loc="upper right")
        ax.set_ylabel("commanded", fontsize=9)
    else:
        ax.text(
            0.5,
            0.5,
            "no sidecar, so no command log",
            ha="center",
            va="center",
            transform=ax.transAxes,
            color="0.5",
        )
        ax.set_ylabel("commanded", fontsize=9)

    plot_radio_panel(axes, record, have_cmd, t0)

    # 3 and 4. Measured against commanded.
    req_lin, req_ang = _requested(record) if have_cmd else (None, None)
    for ax, measured, label, cmd, cmd_label, colour in (
        (axes[2], speed, speed_label, req_lin, "linear command", "C0"),
        (axes[3], yaw, "yaw rate (deg/s)", req_ang, "angular command", "C1"),
    ):
        ax.plot(t, measured, lw=0.9, color="0.25", label="measured")
        ax.set_ylabel(label, fontsize=9)
        ax.axhline(0.0, color="0.55", lw=1.0, zorder=1)
        if have_cmd:
            twin = ax.twinx()
            ctx, cv = from_rest(record.cmd_t - t0, cmd)
            twin.step(ctx, cv, where="post", lw=1.1, color=colour, alpha=0.75, label=cmd_label)
            twin.set_ylabel(cmd_label, fontsize=8, color=colour)
            twin.tick_params(axis="y", labelcolor=colour, labelsize=8)
            align_zero(ax, twin)
        ax.legend(fontsize=8, loc="upper left")

    ax = plot_accel_panel(axes, log, t)

    clip, clip_axes = saturated_mask(
        log, record.pauses if record is not None else [], clock, sensors="a"
    )
    if clip.any():
        marks = spans(clip, t)
        for lo, hi in marks:
            ax.axvspan(lo, hi, color=ACCEL_CLIP_COLOUR, alpha=0.3, lw=0, zorder=2)
        ax.plot(
            [0.5 * (lo + hi) for lo, hi in marks],
            [0.94] * len(marks),
            marker="v",
            ls="none",
            ms=5,
            color=ACCEL_CLIP_COLOUR,
            transform=ax.get_xaxis_transform(),
            clip_on=False,
            zorder=5,
        )
        peak = float(np.max(np.abs(log.accel[clip])) / G_MPS2)
        ax.annotate(
            f"clipped on {', '.join(clip_axes)}: {int(clip.sum())} samples, peak {peak:.1f} g"
            "   (not a discard, and not excluded from the fit)",
            xy=(0.01, 0.13),
            xycoords="axes fraction",
            fontsize=7.5,
            color=ACCEL_CLIP_COLOUR,
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.8, "pad": 1.5},
        )
    ax.legend(fontsize=7, ncol=4, loc="upper right")

    ax = plot_dead_reckoning_panel(axes, calib, log, t, have_cmd)

    heading = integrate_deg(yaw, t)
    zero = int(np.searchsorted(t, 0.0)) if have_cmd else 0
    heading = heading - heading[min(zero, len(heading) - 1)]
    twin = ax.twinx()
    twin.plot(t, heading, lw=1.2, color="C5", label="heading")
    twin.set_ylabel("heading (deg)", fontsize=8, color="C5")
    twin.tick_params(axis="y", labelcolor="C5", labelsize=8)
    aligned = align_zero(ax, twin, lo=float(heading.min()) * 1.05, hi=float(heading.max()) * 1.05)
    if not aligned:
        twin.axhline(0.0, color="C5", ls=":", lw=1.0)

    if have_cmd:
        end = int(np.searchsorted(t, float(record.cmd_t[-1] - record.cmd_t[0])))
        net = float(heading[min(end, len(heading) - 1)])
        twin.annotate(
            f"net {net:+.1f} deg over the commanded stretch",
            xy=(0.99, 0.06),
            xycoords="axes fraction",
            ha="right",
            fontsize=8,
            color="C5",
        )
    lines = ax.get_lines()[:1] + twin.get_lines()[:1]
    ax.legend(lines, [ln.get_label() for ln in lines], fontsize=8, loc="upper left")

    shade_excluded_regions(regions, axes, fig)

    title = log_path.name
    if record is not None:
        bits = [f"{record.waveform} ({record.kind}/{record.channel}, {record.role})"]
        if record.verdict:
            bits.append(f"verdict: {record.verdict}")
        if record.gate_note:
            bits.append(record.gate_note)
        if record.dropped:
            bits.append(f"DROPPED {record.dropped}")
        if not clock.measured:
            bits.append("clock not probed, times are jig seconds")
        title += "   " + "   ".join(bits)
    _, gyro_axes = saturated_mask(
        log, record.pauses if record is not None else [], clock, sensors="g"
    )
    if gyro_axes:
        title += f"   GYRO SATURATED: {', '.join(gyro_axes)}"
    if clip_axes:
        title += f"   accel clipped: {', '.join(clip_axes)}"
    fig.suptitle(title, fontsize=11)
    fig.tight_layout(rect=(0, 0.035, 1, 0.98))
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def _radio_channels(record: RunRecord) -> tuple[np.ndarray | None, np.ndarray | None]:
    """The two drive channels the radio reported.

    Logged directly since the format change. Older runs stored only the mixed body units, so
    reconstruct the channels by inverting the tank un-mix, which is what produced them.
    """
    a = getattr(record, "meas_ch_a", None)
    b = getattr(record, "meas_ch_b", None)
    if a is not None and b is not None:
        return a, b
    if record.command_source != "measured":
        return None, None
    # from_channels gave lin = (a+b)/2 and ang = -(a-b)/2, so a = lin - ang, b = lin + ang.
    return record.cmd_lin - record.cmd_ang, record.cmd_lin + record.cmd_ang


def _is_tank(record: RunRecord) -> bool:
    return str(record.provenance.get("mix", "tank")) == "tank"


def _channels_opposed(a: np.ndarray, b: np.ndarray) -> bool:
    """True when the two channels consistently move against each other."""
    moving = (np.abs(a) > 0.05) | (np.abs(b) > 0.05)
    if np.count_nonzero(moving) < 10:
        return False
    return bool(np.mean(np.sign(a[moving]) * np.sign(b[moving])) < -0.8)


def from_rest(t: np.ndarray, *series: np.ndarray, lead_s: float = 0.05) -> tuple[np.ndarray, ...]:
    """Anchor a command trace at zero just before it starts, and again after it ends."""
    if len(t) == 0:
        return (t, *series)
    t_ext = np.concatenate([[t[0] - lead_s], t, [t[-1] + lead_s]])
    return (t_ext, *(np.concatenate([[0.0], v, [0.0]]) for v in series))


def saturated_mask(
    log: JigLog,
    pauses: Sequence[PauseWindow] = (),
    clock: ClockFit | None = None,
    sensors: str = "ga",
) -> tuple[np.ndarray, list[str]]:
    """Samples where an IMU axis is past its limit, and which axes did it."""
    names: list[str] = []
    mask = np.zeros(len(log.t), dtype=bool)
    clock = clock or ClockFit.identity()
    keep = ~pause_mask(clock.host_seconds(log.t), pauses)
    # Accel uses the header-derived limit, gyro the raw rail. Same split as JigLog.saturation,
    # so what is drawn is exactly what the sidecar counted.
    limits = [
        (prefix, raw, limit)
        for prefix, raw, limit in (
            ("g", log.gyro_raw, RAW_SATURATION),
            ("a", log.accel_raw, log.header.accel_raw_limit),
        )
        if prefix in sensors
    ]
    for prefix, raw, limit in limits:
        for i, axis in enumerate(("x", "y", "z")):
            over = (np.abs(raw[:, i]) > limit) & keep
            if over.any():
                names.append(f"{prefix}{axis}")
                mask |= over
    return mask, names


def spans(mask: np.ndarray, t: np.ndarray, min_width_s: float = 0.05) -> list[tuple[float, float]]:
    """Contiguous True runs as (start, end), widened so single samples stay visible."""
    if not mask.any():
        return []
    edges = np.flatnonzero(np.diff(mask.astype(int)))
    bounds = np.concatenate([[0], edges + 1, [len(mask)]])
    out = []
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        if not mask[lo]:
            continue
        t0, t1 = float(t[lo]), float(t[min(hi, len(t) - 1)])
        if t1 - t0 < min_width_s:
            mid = 0.5 * (t0 + t1)
            t0, t1 = mid - min_width_s / 2, mid + min_width_s / 2
        out.append((t0, t1))
    return out


def integrate_deg(rate_dps: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Cumulative trapezoid of a rate, in the rate's own units times seconds."""
    if len(t) < 2:
        return np.zeros_like(rate_dps)
    steps = 0.5 * (rate_dps[1:] + rate_dps[:-1]) * np.diff(t)
    return np.concatenate([[0.0], np.cumsum(steps)])


def align_zero(
    primary,
    twin,
    span: float = 1.05,
    lo: float | None = None,
    hi: float | None = None,
    max_stretch: float = 3.0,
) -> bool:
    """Put the twin axis's zero at the same height as the primary axis's."""
    data_lo, data_hi = lo, hi
    lo, hi = primary.get_ylim()
    if hi - lo <= 0:
        return False
    keep = 0.15
    frac = (0.0 - lo) / (hi - lo)
    if frac < keep:
        lo = -keep * hi / (1.0 - keep)
    elif frac > 1.0 - keep:
        hi = -keep * lo / (1.0 - keep)
    primary.set_ylim(lo, hi)

    frac = (0.0 - lo) / (hi - lo)
    del lo, hi
    want_hi = span if data_hi is None else max(data_hi, 0.0)
    want_lo = -span if data_lo is None else min(data_lo, 0.0)
    total = max(
        want_hi / (1.0 - frac) if want_hi > 0 else 0.0,
        -want_lo / frac if want_lo < 0 else 0.0,
        1e-9,
    )
    natural = max(want_hi - want_lo, 1e-9)
    if total > max_stretch * natural:
        twin.set_ylim(want_lo - 0.05 * natural, want_hi + 0.05 * natural)
        return False
    twin.set_ylim(-frac * total, (1.0 - frac) * total)
    return True


def _requested(record: RunRecord) -> tuple[np.ndarray, np.ndarray]:
    """What the tool asked for, whichever stream the record kept as primary."""
    if record.cmd_lin_requested is not None and record.cmd_ang_requested is not None:
        return record.cmd_lin_requested, record.cmd_ang_requested
    return record.cmd_lin, record.cmd_ang


def newest_session() -> Path:
    dirs = [p for p in DEFAULT_OUT_ROOT.glob("*") if p.is_dir() and any(p.glob("LOG-*.TXT"))]
    if not dirs:
        raise SystemExit(f"no session directories with logs under {DEFAULT_OUT_ROOT}")
    return max(dirs, key=lambda p: p.stat().st_mtime)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "sessions",
        type=Path,
        nargs="*",
        help="session directories, or individual LOG-N.TXT files (default: newest session)",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION,
        help=(
            "jig calibration TOML. Plots in m/s when it has a real meters_per_count;"
            " falls back to encoder counts/s when it does not."
        ),
    )
    parser.add_argument("--force", action="store_true", help="redraw plots that already exist")
    parser.add_argument("--suffix", default=".png", help="plot extension (default .png)")
    args = parser.parse_args()

    calib = None
    if args.calibration and args.calibration.exists():
        calib = JigCalibration.from_toml(args.calibration)
        if calib.meters_per_count <= 0.0:
            print(f"{args.calibration}: meters_per_count is unset, plotting encoder counts/s")
            calib = None

    targets = args.sessions or [newest_session()]
    logs: list[Path] = []
    for target in targets:
        if target.is_dir():
            logs.extend(sorted(target.glob("LOG-*.TXT"), key=lambda p: _index(p.name)))
        elif target.exists():
            logs.append(target)
        else:
            print(f"skipping {target}: not found")

    if not logs:
        raise SystemExit("no LOG-*.TXT files found")

    records: dict[Path, RunRecord] = {}
    for directory in {p.parent for p in logs}:
        try:
            from auto_battlebot.calibration.jig.velocity_jig import load_session_dir

            for record in load_session_dir(directory).runs:
                if record.log_file:
                    records[directory / record.log_file] = record
        except (OSError, ValueError) as err:
            print(f"  {directory}: no usable sidecars ({err}); plotting logs alone")

    made = skipped = failed = 0
    for log_path in logs:
        out_path = log_path.with_suffix(args.suffix)
        if out_path.exists() and not args.force:
            skipped += 1
            continue
        try:
            plot_run(log_path, out_path, records.get(log_path), calib)
        except (OSError, ValueError) as err:
            print(f"  {log_path.name}: {err}")
            failed += 1
            continue
        made += 1
        tag = "" if log_path in records else "  (no sidecar)"
        print(f"  wrote {out_path.name}{tag}")

    print(f"\n{made} plotted, {skipped} already had one, {failed} failed")


def _index(name: str) -> tuple[int, str]:
    """Sort LOG-2 before LOG-10, unlike a plain string sort."""
    import re

    m = re.search(r"(\d+)", name)
    return (int(m.group(1)) if m else 1 << 30, name)


if __name__ == "__main__":
    main()
