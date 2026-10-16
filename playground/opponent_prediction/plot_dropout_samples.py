"""Plot the opponent filter coasting through 0.2 s, 1 s and 3 s dropouts.

Two sources:

- `sysid`: the Mr Stabs Mk2 sysid sessions, scored against AprilTag truth
  (`auto_battlebot/perception/sysid_sessions.py`).
- `replay`: an MCAP from a playback replay, which logs every opponent measurement the C++
  estimator received (`kalman_motion_estimator` diagnostics, `opponent_measurement_<id>`). There is
  no truth there, so the withheld measurements stand in for it: the dropout is scored against what
  the camera saw while the filter was not allowed to.

For each source and gap length the script picks samples spread across the recording where the
robot is moving, withholds every measurement in the gap, and runs three arms over the same
stream: the 6-state filter (`phase1`), the previous 4-state arm (`kalman4`), and HOLD. Each sample
gets a figure: the top-down path, position error against time into the gap with the filter's own
2-sigma, heading, and yaw rate. A summary figure per source gives the median error against time
into the gap over every periodic gap in the recording.

Usage:
    source scripts/activate_python.sh
    python playground/opponent_prediction/plot_dropout_samples.py sysid
    python playground/opponent_prediction/plot_dropout_samples.py replay out/replays/<name>.mcap
Figures go under playground/opponent_prediction/out/samples/.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (backend must be chosen first)
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from auto_battlebot.perception.sysid_sessions import OUT, SessionData, load_session  # noqa: E402
from auto_battlebot.recording.mcap_io import decode_json, iter_messages  # noqa: E402
from playground.opponent_prediction import evaluate_opponent_prediction as ev  # noqa: E402

GAPS_S = (0.2, 1.0, 3.0)
SAMPLES_PER_GAP = 3
PRE_S = 3.0
POST_S = 1.5
MIN_SPEED = 0.3
SUMMARY_PERIOD_S = 8.0

COLORS = {
    "truth": "#0b0b0b",
    "phase1": "#2a78d6",
    "kalman4": "#1baf7a",
    "hold": "#eb6834",
    "meas": "#9a9893",
}
STYLES = {"phase1": "-", "kalman4": "--", "hold": "-"}
LABELS = {
    "phase1": "6-state filter",
    "kalman4": "previous 4-state",
    "hold": "HOLD (production)",
}


# --------------------------------------------------------------------------------- sources


def replay_session(mcap: Path, frame_id: str = "THEIR_ROBOT_1") -> SessionData:
    """Opponent measurements from a replay's estimator diagnostics, as their own truth."""
    rows = []
    key = f"opponent_measurement_{frame_id}"
    for _topic, _log_ns, data in iter_messages(mcap, ["/diagnostics/kalman_motion_estimator"]):
        entry = _find(decode_json(data), key)
        # The diagnostics re-send a channel's last value with held = true; only fresh ones count.
        if entry is not None and not entry.get("held", False):
            rows.append(entry["values"])
    if not rows:
        raise ValueError(f"no {key} diagnostics in {mcap}")
    m = pd.DataFrame(rows).drop_duplicates("stamp").sort_values("stamp")
    m = m[m["keypoint"].astype(int) == 1]
    stamp_ns = np.round(m["stamp"].to_numpy(float) * 1e9).astype(np.int64)
    x, y, yaw = (m[c].to_numpy(float) for c in ("x", "y", "yaw"))
    t = (stamp_ns - stamp_ns[0]) / 1e9
    vx, vy = np.gradient(x, t), np.gradient(y, t)
    yaw_rate = np.gradient(np.unwrap(yaw), t)
    truth = pd.DataFrame(
        {"stamp_ns": stamp_ns, "x": x, "y": y, "yaw": yaw, "vx": vx, "vy": vy, "yaw_rate": yaw_rate}
    )
    # A truth sample is only a measurement; between two more than 70 ms apart nothing is known.
    gap_ok = np.ones(len(t), dtype=bool)
    if len(t) > 1:
        d = np.diff(t)
        gap_ok[1:] &= d < 0.07
        gap_ok[:-1] &= d < 0.07
    return SessionData(
        name=mcap.stem,
        role="replay",
        meas_stamp_ns=stamp_ns,
        meas_x=x,
        meas_y=y,
        meas_yaw=yaw,
        truth=truth,
        truth_valid=gap_ok,
    )


def _find(node: object, key: str) -> dict | None:
    if isinstance(node, dict):
        if key in node and isinstance(node[key], dict):
            return node[key]
        for value in node.values():
            found = _find(value, key)
            if found is not None:
                return found
    return None


# --------------------------------------------------------------------------------- samples


def pick_samples(data: SessionData, gap_s: float, count: int) -> list[int]:
    """Gap start stamps (ns) spread across the recording where the robot moves and is seen."""
    stamps = data.meas_stamp_ns
    step_ns = int(0.5e9)
    candidates = []
    for start in range(int(stamps[0] + 4e9), int(stamps[-1] - (gap_s + 2.0) * 1e9), step_ns):
        span = np.arange(start - int(0.5e9), start + int((gap_s + 0.5) * 1e9), int(0.02e9))
        tr = data.truth_at(span)
        if tr["valid"].mean() < 0.8:
            continue
        inside = (span >= start) & (span < start + int(gap_s * 1e9))
        speed = np.hypot(tr["vx"], tr["vy"])[inside & tr["valid"]]
        if speed.size == 0 or np.median(speed) < MIN_SPEED:
            continue
        before = ((stamps >= start - int(0.3e9)) & (stamps < start)).sum()
        after = (
            (stamps >= start + int(gap_s * 1e9)) & (stamps < start + int((gap_s + 0.5) * 1e9))
        ).sum()
        if before < 6 or after < 6:
            continue
        candidates.append(start)
    if not candidates:
        return []
    picks = np.linspace(0, len(candidates) - 1, min(count, len(candidates))).round().astype(int)
    return [candidates[i] for i in picks]


def run_arms(
    data: SessionData, start_ns: int, gap_s: float
) -> tuple[dict[str, ev.Trace], tuple[int, int]]:
    cfg, sigma = ev.tuned_config()
    window = (start_ns - int(PRE_S * 1e9), start_ns + int((gap_s + POST_S) * 1e9))
    keep = ~((data.meas_stamp_ns >= start_ns) & (data.meas_stamp_ns < start_ns + int(gap_s * 1e9)))
    keep = keep[None, :]
    traces = {
        "phase1": ev.simulate_filter(
            data, ev.OpponentFilter(1, cfg), keep, sigma, window_ns=window
        ),
        "kalman4": ev.simulate_filter(
            data, ev.LegacyOpponentFilter(1, cfg), keep, 0.03, window_ns=window
        ),
        "hold": ev.simulate_hold(data, keep, cfg.render_lead_s, window_ns=window),
    }
    return traces, window


def plot_sample(data: SessionData, start_ns: int, gap_s: float, path: Path) -> dict:
    traces, window = run_arms(data, start_ns, gap_s)
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    ax_xy, ax_err, ax_yaw, ax_w = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]
    end_ns = start_ns + int(gap_s * 1e9)
    show = (start_ns - int(0.6e9), end_ns + int(0.6e9))

    # Truth path and the measurements, the withheld ones hollow.
    grid = np.arange(show[0], show[1], int(0.01e9))
    tr = data.truth_at(grid)
    ax_xy.plot(tr["x"], tr["y"], color=COLORS["truth"], lw=1.6, label="truth")
    ms = data.meas_stamp_ns
    shown = (ms >= show[0]) & (ms < show[1])
    held = shown & (ms >= start_ns) & (ms < end_ns)
    seen = shown & ~held
    ax_xy.scatter(
        data.meas_x[seen], data.meas_y[seen], s=10, color=COLORS["meas"], label="measurement"
    )
    ax_xy.scatter(
        data.meas_x[held],
        data.meas_y[held],
        s=14,
        facecolors="none",
        edgecolors=COLORS["meas"],
        label="withheld",
    )

    summary: dict = {"start_s": (start_ns - data.meas_stamp_ns[0]) / 1e9, "gap_s": gap_s}
    worst = 0.0
    for arm, trace in traces.items():
        t_rel = (trace.target_ns - start_ns) / 1e9
        sel = (trace.target_ns >= show[0]) & (trace.target_ns < show[1])
        x, y = trace.values["x"][sel, 0], trace.values["y"][sel, 0]
        ax_xy.plot(x, y, color=COLORS[arm], lw=1.4, ls=STYLES[arm], label=LABELS[arm])
        truth = data.truth_at(trace.target_ns[sel])
        err = np.hypot(x - truth["x"], y - truth["y"])
        err = np.where(truth["valid"], err, np.nan)
        ax_err.plot(
            t_rel[sel], 1e3 * err, color=COLORS[arm], lw=1.4, ls=STYLES[arm], label=LABELS[arm]
        )
        worst = max(worst, float(np.nanmax(1e3 * err)) if np.isfinite(err).any() else 0.0)
        yaw = trace.values["yaw"][sel, 0]
        ax_yaw.plot(
            t_rel[sel],
            np.degrees(np.unwrap(yaw)),
            color=COLORS[arm],
            lw=1.2,
            ls=STYLES[arm],
            label=LABELS[arm],
        )
        in_gap = (t_rel[sel] >= 0) & (t_rel[sel] < gap_s)
        after = (t_rel[sel] >= gap_s) & (t_rel[sel] < gap_s + 0.3)
        summary[arm] = {
            "gap_p50_mm": float(np.nanmedian(1e3 * err[in_gap])) if in_gap.any() else None,
            "gap_max_mm": float(np.nanmax(1e3 * err[in_gap])) if in_gap.any() else None,
            "after_p50_mm": float(np.nanmedian(1e3 * err[after])) if after.any() else None,
        }
        if arm == "phase1":
            sig = trace.values["position_sigma_m"][sel, 0]
            ax_err.plot(
                t_rel[sel], 2e3 * sig, color=COLORS[arm], lw=1.0, ls="--", label="6-state 2σ"
            )
            # 2-sigma rings at a few points through the gap.
            for frac in np.linspace(0.0, 1.0, 5):
                k = np.argmin(np.abs(t_rel[sel] - frac * gap_s))
                circle = plt.Circle(
                    (x[k], y[k]), 2.0 * sig[k], fill=False, color=COLORS[arm], lw=0.8, alpha=0.6
                )
                ax_xy.add_patch(circle)
            hs = trace.values["heading_sigma_rad"][sel, 0]
            base = np.degrees(np.unwrap(yaw))
            finite = np.isfinite(hs)
            ax_yaw.fill_between(
                t_rel[sel][finite],
                (base - 2 * np.degrees(hs))[finite],
                (base + 2 * np.degrees(hs))[finite],
                color=COLORS[arm],
                alpha=0.15,
                lw=0,
            )
            w = trace.values["omega"][sel, 0]
            ws = trace.values["yaw_rate_sigma_rad_s"][sel, 0]
            ax_w.plot(t_rel[sel], w, color=COLORS[arm], lw=1.2, label=LABELS[arm])
            ax_w.fill_between(
                t_rel[sel], w - 2 * ws, w + 2 * ws, color=COLORS[arm], alpha=0.15, lw=0
            )

    t_truth = (grid - start_ns) / 1e9
    yaw_truth = np.where(tr["valid"], np.degrees(np.unwrap(tr["yaw"])), np.nan)
    # Put truth on the filter's branch of the unwrap so the curves overlay.
    ax_yaw.plot(t_truth, yaw_truth, color=COLORS["truth"], lw=1.6, label="truth")
    ax_w.plot(
        t_truth,
        np.where(tr["valid"], tr["yaw_rate"], np.nan),
        color=COLORS["truth"],
        lw=1.6,
        label="truth",
    )

    for ax in (ax_err, ax_yaw, ax_w):
        ax.axvspan(0.0, gap_s, color="#e8e6df", zorder=0)
        ax.set_xlim((show[0] - start_ns) / 1e9, (show[1] - start_ns) / 1e9)
        ax.set_xlabel("time from dropout start (s)")
        ax.grid(alpha=0.3)
    # Frame the path, not the rings: past the coast horizon the 2-sigma reaches metres.
    px = np.concatenate([tr["x"][tr["valid"]], data.meas_x[shown]])
    py = np.concatenate([tr["y"][tr["valid"]], data.meas_y[shown]])
    cx, cy = 0.5 * (px.min() + px.max()), 0.5 * (py.min() + py.max())
    half = 0.5 * max(px.max() - px.min(), py.max() - py.min()) + 0.15
    ax_xy.set_xlim(cx - half, cx + half)
    ax_xy.set_ylim(cy - half, cy + half)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.set_xlabel("x (m)")
    ax_xy.set_ylabel("y (m)")
    ax_xy.set_title("Top-down path; rings are the 6-state 2σ through the gap")
    ax_xy.legend(fontsize=8, loc="best")
    ax_err.set_ylabel("position error (mm)")
    ax_err.set_ylim(0, 1.3 * worst if worst > 0 else None)
    ax_err.set_title("Rendered position error (render time = now + lead)")
    ax_err.legend(fontsize=8)
    ax_yaw.set_ylabel("heading (deg, unwrapped)")
    ax_yaw.set_title("Heading; band is the 6-state 2σ")
    ax_w.set_ylabel("yaw rate (rad/s)")
    ax_w.set_title("Yaw rate; band is the 6-state 2σ")
    truth_name = "AprilTag truth" if data.role != "replay" else "withheld measurements as truth"
    fig.suptitle(
        f"{data.name}: {gap_s:g} s dropout at t = {summary['start_s']:.1f} s ({truth_name})",
        fontsize=12,
    )
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return summary


# --------------------------------------------------------------------------------- summary


def plot_summary(data: SessionData, path: Path) -> dict:
    """Median position and heading error against time into every periodic gap, per arm."""
    cfg, sigma = ev.tuned_config()
    keep = np.stack([ev.periodic_gaps(data.meas_stamp_ns, g, SUMMARY_PERIOD_S) for g in GAPS_S])
    b = keep.shape[0]
    traces = {
        "phase1": ev.simulate_filter(data, ev.OpponentFilter(b, cfg), keep, sigma),
        "kalman4": ev.simulate_filter(data, ev.LegacyOpponentFilter(b, cfg), keep, 0.03),
        "hold": ev.simulate_hold(data, keep, cfg.render_lead_s),
    }
    fig, axes = plt.subplots(2, len(GAPS_S), figsize=(5 * len(GAPS_S), 8), squeeze=False)
    out: dict = {}
    bins = np.arange(-0.3, 4.0, 0.05)
    for col, gap in enumerate(GAPS_S):
        out[f"{gap:g}"] = {}
        for arm, trace in traces.items():
            truth = data.truth_at(trace.target_ns)
            rel = (trace.target_ns - data.meas_stamp_ns[0]) / 1e9 - 1.0  # periodic_gaps phase
            into = rel - np.floor(rel / SUMMARY_PERIOD_S) * SUMMARY_PERIOD_S
            into = np.where(into > SUMMARY_PERIOD_S - 0.3, into - SUMMARY_PERIOD_S, into)
            ok = truth["valid"] & (trace.values["active"][:, col] > 0) & (rel > 0)
            pe = np.hypot(
                trace.values["x"][:, col] - truth["x"], trace.values["y"][:, col] - truth["y"]
            )
            ye = np.degrees(np.abs(ev.wrap_np(trace.values["yaw"][:, col] - truth["yaw"])))
            centers, med_p, med_y = [], [], []
            for lo, hi in zip(bins[:-1], bins[1:], strict=True):
                m = ok & (into >= lo) & (into < hi)
                if m.sum() >= 5:
                    centers.append(0.5 * (lo + hi))
                    med_p.append(1e3 * np.median(pe[m]))
                    med_y.append(np.median(ye[m]))
            style = {"color": COLORS[arm], "lw": 1.6, "ls": STYLES[arm], "label": LABELS[arm]}
            axes[0, col].plot(centers, med_p, **style)
            axes[1, col].plot(centers, med_y, **style)
            in_gap = ok & (into >= 0) & (into < gap)
            reacq = ok & (into >= gap + 0.15) & (into < gap + 0.5)
            out[f"{gap:g}"][arm] = {
                "gap_pos_p50_mm": float(1e3 * np.median(pe[in_gap])) if in_gap.any() else None,
                "gap_pos_p90_mm": float(1e3 * np.percentile(pe[in_gap], 90))
                if in_gap.any()
                else None,
                "gap_yaw_p50_deg": float(np.median(ye[in_gap])) if in_gap.any() else None,
                "reacquired_pos_p50_mm": float(1e3 * np.median(pe[reacq])) if reacq.any() else None,
            }
        for row in range(2):
            ax = axes[row, col]
            ax.axvspan(0.0, gap, color="#e8e6df", zorder=0)
            ax.set_xlim(-0.3, gap + 1.0)
            ax.grid(alpha=0.3)
            ax.set_xlabel("time into dropout (s)")
        axes[0, col].set_title(f"{gap:g} s dropouts")
        axes[0, col].set_ylabel("median position error (mm)")
        axes[1, col].set_ylabel("median heading error (deg)")
    axes[0, 0].legend(fontsize=8)
    fig.suptitle(f"{data.name}: median error through every {SUMMARY_PERIOD_S:g} s-periodic dropout")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("source", choices=["sysid", "replay"])
    parser.add_argument("mcaps", nargs="*", type=Path)
    args = parser.parse_args()
    if args.source == "sysid":
        sessions = [load_session(n) for n in ("2026-10-15_00-29-00", "2026-10-15_00-20-58")]
    else:
        sessions = [replay_session(p) for p in args.mcaps]
    out_dir = OUT / "samples" / args.source
    report: dict = {}
    for data in sessions:
        entry: dict = {"summary": plot_summary(data, out_dir / f"{data.name}_summary.png")}
        for gap in GAPS_S:
            for k, start in enumerate(pick_samples(data, gap, SAMPLES_PER_GAP)):
                name = f"{data.name}_gap{gap:g}s_{k}.png"
                entry[name] = plot_sample(data, start, gap, out_dir / name)
                print(f"wrote {out_dir / name}")
        report[data.name] = entry
    (out_dir / "samples.json").write_text(json.dumps(report, indent=1))
    print(json.dumps({k: v["summary"] for k, v in report.items()}, indent=1))


if __name__ == "__main__":
    main()
