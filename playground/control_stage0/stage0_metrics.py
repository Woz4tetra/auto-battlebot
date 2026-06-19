#!/usr/bin/env python3
"""Stage 0 control baseline metrics from existing Jetson fight recordings.

Produces the numbers the control improvement plan needs before any controller
work: end-to-end latency, perception reliability, aim, an overshoot/wall-contact
baseline, and an estimate of how much the flat-plane keypoint projection error
contributes to wall collisions.

Everything is computed from existing recordings in data/recordings/. Nothing is
re-run (laptop results differ from the Jetson).

Run (activate the project venv first):

    source scripts/activate_python.sh
    python playground/control_stage0/stage0_metrics.py <file.mcap> [...] --plots docs/experiments/control_improvement/assets

Reuses playground/analyze_nav_diagnostics.py helpers for time-to-contact, and
playground/control_stage0/diag_io.py for all loading.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless: render plots to files, no display
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import diag_io  # noqa: E402

# Reuse the sibling analyzer's contact metric.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from analyze_nav_diagnostics import compute_time_to_target  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _pct(num: int, den: int) -> float:
    return 100.0 * num / den if den else 0.0


def _stats(s: pd.Series) -> dict[str, float]:
    s = s.dropna()
    if s.empty:
        return {
            "mean": float("nan"),
            "p50": float("nan"),
            "p95": float("nan"),
            "max": float("nan"),
        }
    return {
        "mean": float(s.mean()),
        "p50": float(s.quantile(0.50)),
        "p95": float(s.quantile(0.95)),
        "max": float(s.max()),
    }


def _episodes(mask: pd.Series, t: pd.Series) -> list[tuple[float, float]]:
    """Contiguous (start_t, end_t) runs where mask is True."""
    out = []
    start = None
    prev_t = None
    for is_on, tt in zip(mask.to_numpy(), t.to_numpy()):
        if is_on and start is None:
            start = tt
        elif not is_on and start is not None:
            out.append((start, prev_t if prev_t is not None else tt))
            start = None
        prev_t = tt
    if start is not None:
        out.append((start, prev_t))
    return out


def auto_window(df: pd.DataFrame) -> pd.DataFrame:
    """Restrict to the autonomous match window: first auto-engaged frame to the
    last auto->manual transition (or last auto frame). Mirrors the windowing in
    scripts/mcap_auto_percentage.py."""
    if "is_auto" not in df.columns or not df["is_auto"].any():
        return df
    auto = df["is_auto"].to_numpy()
    first = int(np.argmax(auto))
    last_to_manual = None
    for i in range(1, len(auto)):
        if auto[i - 1] and not auto[i]:
            last_to_manual = i
    last = (
        last_to_manual
        if last_to_manual is not None
        else (len(auto) - 1 - int(np.argmax(auto[::-1])))
    )
    return df.iloc[first : last + 1].reset_index(drop=True)


def merge_sources(
    diag: pd.DataFrame, tracks: pd.DataFrame, cam: pd.DataFrame
) -> pd.DataFrame:
    diag = diag.sort_values("timestamp_ns").reset_index(drop=True)
    tol = int(60e6)  # 60 ms
    if not tracks.empty:
        diag = pd.merge_asof(
            diag,
            tracks.sort_values("timestamp_ns"),
            on="timestamp_ns",
            direction="nearest",
            tolerance=tol,
        )
    if not cam.empty:
        diag = pd.merge_asof(
            diag,
            cam.sort_values("timestamp_ns"),
            on="timestamp_ns",
            direction="nearest",
            tolerance=tol,
        )
    return diag


# ---------------------------------------------------------------------------
# Metric sections
# ---------------------------------------------------------------------------


def report_latency(df: pd.DataFrame) -> None:
    print(
        "LATENCY (perception + compute only; excludes Crossfire ~20ms and ESC/mechanical tail)"
    )
    if "pipeline/latency_ms" in df.columns:
        s = _stats(df["pipeline/latency_ms"])
        print(
            f"  end-to-end   mean {s['mean']:6.1f}  p50 {s['p50']:6.1f}  p95 {s['p95']:6.1f}  max {s['max']:6.1f}  ms"
        )
    for col in sorted(
        c for c in df.columns if c.startswith("stage/") and c.endswith("/elapsed_ms")
    ):
        name = col[len("stage/") : -len("/elapsed_ms")]
        s = _stats(df[col])
        print(
            f"  {name:24s} p50 {s['p50']:6.1f}  p95 {s['p95']:6.1f}  max {s['max']:6.1f}  ms"
        )
    print()


def report_reliability(df: pd.DataFrame, duration_s: float) -> None:
    print("PERCEPTION RELIABILITY")
    n = len(df)
    if "nav/using_previous_robots" in df.columns:
        sub = df["nav/using_previous_robots"].fillna(0) > 0.5
        eps = _episodes(sub, df["t"])
        durs = [e - s for s, e in eps]
        print(
            f"  cache substituted (lost a critical robot): {_pct(int(sub.sum()), n):5.1f}% of frames, "
            f"{len(eps)} dropout episodes"
        )
        if durs:
            print(
                f"    dropout episode duration: median {np.median(durs) * 1000:.0f} ms  max {max(durs) * 1000:.0f} ms"
            )
    if "n_their" in df.columns:
        present = df["n_their"].fillna(0) > 0
        print(
            f"  opponent marker present: {_pct(int(present.sum()), n):5.1f}% of frames "
            f"[includes filter-held poses; markers do not carry is_stale]"
        )
        prim = df.get("primary_their")
        if prim is not None:
            nonempty = prim.fillna("").astype(str)
            switches = (
                (nonempty != nonempty.shift(1))
                & (nonempty != "")
                & (nonempty.shift(1) != "")
            ).sum()
            rate = switches / duration_s if duration_s else 0.0
            print(
                f"  opponent frame-id switches: {int(switches)} ({rate:.2f}/s) "
                f"[frame-id level; mislabel cause not in this recording]"
            )
    print()


def report_aim(df: pd.DataFrame) -> None:
    print("AIM")
    n = len(df)
    if "facing_target" in df.columns:
        facing = df["facing_target"].fillna(0) > 0.5
        print(
            f"  facing target (drive-enabled): {_pct(int(facing.sum()), n):5.1f}% of frames"
        )
    if "angle_error_deg" in df.columns:
        s = _stats(df["angle_error_deg"].abs())
        print(f"  |angle error|  median {s['p50']:5.1f}  p95 {s['p95']:5.1f}  deg")
    print()


def report_overshoot(
    df: pd.DataFrame, field_size, wall_margin: float, contact_dist: float
) -> None:
    print("OVERSHOOT / WALL CONTACT")
    if field_size is None or "our_x" not in df.columns:
        print("  field size or our pose unavailable; skipped\n")
        return None
    half_x, half_y = field_size[0] / 2.0, field_size[1] / 2.0
    wall_dist = np.minimum(half_x - df["our_x"].abs(), half_y - df["our_y"].abs())
    df = df.assign(wall_dist=wall_dist)
    near = df["wall_dist"] < wall_margin
    eps = _episodes(near, df["t"])
    total = sum(e - s for s, e in eps)
    print(
        f"  field size: {field_size[0]:.2f} x {field_size[1]:.2f} m (half {half_x:.2f}, {half_y:.2f})"
    )
    print(
        f"  near-wall (< {wall_margin:.2f} m): {len(eps)} episodes, {total:.1f} s total, "
        f"{_pct(int(near.sum()), len(df)):.1f}% of frames"
    )
    if "distance" in df.columns:
        ttc = compute_time_to_target(df, threshold_m=contact_dist)
        reached = df["distance"] < contact_dist
        ttc_str = (
            f"first contact at {ttc:.1f} s into window"
            if ttc is not None
            else "never reached"
        )
        print(
            f"  target distance: median {df['distance'].median():.2f} m; "
            f"within contact ({contact_dist:.2f} m): {_pct(int(reached.sum()), len(df)):.1f}% of frames; "
            f"{ttc_str}"
        )
    print()
    return df


def report_projection_error(
    df: pd.DataFrame, field_size, keypoint_height: float, wall_margin: float
) -> None:
    print(
        f"FLAT-PLANE PROJECTION ERROR (assumed keypoint height {keypoint_height * 100:.0f} cm)"
    )
    if "cam_x" not in df.columns or "our_x" not in df.columns:
        print("  camera pose or our pose unavailable; skipped\n")
        return
    cz = df["cam_z"]
    horiz = np.sqrt((df["our_x"] - df["cam_x"]) ** 2 + (df["our_y"] - df["cam_y"]) ** 2)
    # error = (h / cam_z) * horizontal distance from camera; vector points radially
    # outward (away from the camera nadir): true keypoint sits closer to camera.
    err = (keypoint_height / cz) * horiz
    df = df.assign(proj_err=err)
    s = _stats(df["proj_err"])
    print(f"  camera height above field: median {cz.median():.2f} m")
    print(
        f"  position error: median {s['p50'] * 100:.1f}  p95 {s['p95'] * 100:.1f}  max {s['max'] * 100:.1f}  cm"
    )

    if field_size is not None:
        half_x, half_y = field_size[0] / 2.0, field_size[1] / 2.0
        wall_dist = np.minimum(half_x - df["our_x"].abs(), half_y - df["our_y"].abs())
        near = wall_dist < wall_margin
        if int(near.sum()) > 0:
            sn = _stats(df.loc[near, "proj_err"])
            # Alignment: does the error vector (radially outward from camera) point
            # toward the contacted wall? Positive => perceived position is inward of
            # truth, so the controller sees more room than it has.
            ex = df["our_x"] - df["cam_x"]
            ey = df["our_y"] - df["cam_y"]
            en = np.sqrt(ex**2 + ey**2).replace(0, np.nan)
            ex, ey = ex / en, ey / en
            # Outward normal of nearest wall.
            use_x = (half_x - df["our_x"].abs()) <= (half_y - df["our_y"].abs())
            nx = np.where(use_x, np.sign(df["our_x"]), 0.0)
            ny = np.where(use_x, 0.0, np.sign(df["our_y"]))
            align = ex * nx + ey * ny
            print(
                f"  during near-wall frames: error median {sn['p50'] * 100:.1f} cm, "
                f"mean alignment with contacted wall {np.nanmean(align[near]):+.2f} "
                f"(+1 = error points into the wall)"
            )
    print(
        "  -> upper bound on wall-collision contribution; height compensation would remove most of this."
    )
    print()


# ---------------------------------------------------------------------------
# Derived columns, summary, and plots
# ---------------------------------------------------------------------------


def augment(win: pd.DataFrame, field_size, keypoint_height: float) -> pd.DataFrame:
    """Add wall_dist and proj_err columns used by the summary and the plots."""
    win = win.copy()
    if field_size is not None and "our_x" in win.columns:
        half_x, half_y = field_size[0] / 2.0, field_size[1] / 2.0
        win["wall_dist"] = np.minimum(
            half_x - win["our_x"].abs(), half_y - win["our_y"].abs()
        )
    if "cam_x" in win.columns and "our_x" in win.columns:
        horiz = np.sqrt(
            (win["our_x"] - win["cam_x"]) ** 2 + (win["our_y"] - win["cam_y"]) ** 2
        )
        win["proj_err"] = (keypoint_height / win["cam_z"]) * horiz
    return win


def build_summary(win: pd.DataFrame, label: str, wall_margin: float) -> dict:
    n = len(win)
    s = {"label": label}
    if "pipeline/latency_ms" in win.columns:
        lat = _stats(win["pipeline/latency_ms"])
        s["lat_p50"], s["lat_p95"] = lat["p50"], lat["p95"]
    if "facing_target" in win.columns:
        s["facing_pct"] = _pct(int((win["facing_target"].fillna(0) > 0.5).sum()), n)
    if "wall_dist" in win.columns:
        s["nearwall_pct"] = _pct(int((win["wall_dist"] < wall_margin).sum()), n)
    if "proj_err" in win.columns:
        s["proj_err_cm"] = float(win["proj_err"].median() * 100.0)
    return s


def plot_fight_detail(win: pd.DataFrame, field_size, label: str, out: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(f"Stage 0 detail: {label}", fontsize=13)

    # (a) Trajectory coloured by distance to nearest wall.
    ax = axes[0, 0]
    if "our_x" in win.columns and "wall_dist" in win.columns:
        sc = ax.scatter(
            win["our_x"], win["our_y"], c=win["wall_dist"], s=4, cmap="viridis"
        )
        fig.colorbar(sc, ax=ax, label="distance to wall (m)")
        if field_size is not None:
            hx, hy = field_size[0] / 2.0, field_size[1] / 2.0
            ax.add_patch(
                plt.Rectangle((-hx, -hy), 2 * hx, 2 * hy, fill=False, ec="red", lw=1.5)
            )
        ax.set_aspect("equal")
    ax.set_title("Our trajectory (field frame)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    # (b) End-to-end latency over time.
    ax = axes[0, 1]
    if "pipeline/latency_ms" in win.columns:
        ax.plot(win["t"], win["pipeline/latency_ms"], lw=0.6)
        ax.axhline(60, color="red", ls="--", lw=1, label="60 ms budget")
        ax.legend()
    ax.set_title("End-to-end latency")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("ms")

    # (c) Heading error over time.
    ax = axes[1, 0]
    if "angle_error_deg" in win.columns:
        ax.plot(win["t"], win["angle_error_deg"].abs(), lw=0.6)
        ax.axhline(90, color="gray", ls=":", lw=1)
    ax.set_title("|heading error|")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("deg")

    # (d) Flat-plane projection error over time.
    ax = axes[1, 1]
    if "proj_err" in win.columns:
        ax.plot(win["t"], win["proj_err"] * 100.0, lw=0.6, color="darkorange")
    ax.set_title("Flat-plane projection error")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("cm")

    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def plot_aggregate(summaries: list[dict], out: Path) -> None:
    labels = [s["label"] for s in summaries]
    x = np.arange(len(labels))
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle("Stage 0 baseline across May-02 recordings", fontsize=13)

    ax = axes[0, 0]
    p50 = [s.get("lat_p50", np.nan) for s in summaries]
    p95 = [s.get("lat_p95", np.nan) for s in summaries]
    ax.bar(x - 0.2, p50, 0.4, label="p50")
    ax.bar(x + 0.2, p95, 0.4, label="p95")
    ax.axhline(60, color="red", ls="--", lw=1, label="60 ms budget")
    ax.set_title("End-to-end latency")
    ax.set_ylabel("ms")
    ax.legend()

    ax = axes[0, 1]
    ax.bar(x, [s.get("facing_pct", np.nan) for s in summaries], color="seagreen")
    ax.set_title("Frames facing target")
    ax.set_ylabel("%")

    ax = axes[1, 0]
    ax.bar(x, [s.get("nearwall_pct", np.nan) for s in summaries], color="indianred")
    ax.set_title("Frames near a wall")
    ax.set_ylabel("%")

    ax = axes[1, 1]
    ax.bar(x, [s.get("proj_err_cm", np.nan) for s in summaries], color="darkorange")
    ax.set_title("Median flat-plane projection error")
    ax.set_ylabel("cm")

    for ax in axes.flat:
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def analyze(path: Path, args) -> dict:
    print("=" * 90)
    print(f"File: {path.name}")
    diag = diag_io.load_diagnostics(path)
    tracks = diag_io.load_robot_tracks(path)
    cam = diag_io.load_camera_in_field(path)
    field_size = diag_io.load_field_size(path)

    df = merge_sources(diag, tracks, cam)
    has_auto = "is_auto" in df.columns and bool(df["is_auto"].any())
    win = auto_window(df).copy()
    if len(win) > 1:
        win["t"] = win["t"] - win["t"].iloc[0]  # window-relative time
    duration = float(win["t"].iloc[-1] - win["t"].iloc[0]) if len(win) > 1 else 0.0
    if has_auto:
        print(
            f"Auto window: {duration:.1f} s, {len(win)} ticks "
            f"({_pct(len(win), len(df)):.0f}% of {len(df)} total)\n"
        )
    else:
        print(
            f"No autonomous frames detected; analyzing full recording: "
            f"{duration:.1f} s, {len(win)} ticks (likely a setup/test session, not a fight)\n"
        )

    report_latency(win)
    report_reliability(win, duration)
    report_aim(win)
    report_overshoot(win, field_size, args.wall_contact_margin, args.contact_distance)
    report_projection_error(
        win, field_size, args.keypoint_height, args.wall_contact_margin
    )

    win = augment(win, field_size, args.keypoint_height)
    # Short label: the fight time-of-day from the filename.
    label = path.stem.replace("auto_battlebot_main_2026-05-02_", "").replace(
        "_repaired", ""
    )
    summary = build_summary(win, label, args.wall_contact_margin)

    if args.csv:
        out = path.with_suffix(".stage0.csv")
        win.to_csv(out, index=False)
        print(f"  wrote {out}")
    if args.plots:
        args.plots.mkdir(parents=True, exist_ok=True)
        out = args.plots / f"detail_{label}.png"
        plot_fight_detail(win, field_size, label, out)
        print(f"  wrote {out}")

    return summary


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("files", nargs="+", type=Path, help="MCAP recording(s)")
    ap.add_argument(
        "--keypoint-height",
        type=float,
        default=0.06,
        help="assumed robot keypoint height above field plane, m (Mrs Buff CAD)",
    )
    ap.add_argument(
        "--wall-contact-margin",
        type=float,
        default=0.11,
        help="distance to a wall counted as a contact, m (robot half-length)",
    )
    ap.add_argument(
        "--contact-distance",
        type=float,
        default=0.15,
        help="distance to target counted as contact, m",
    )
    ap.add_argument(
        "--csv", action="store_true", help="also write a per-tick CSV next to each file"
    )
    ap.add_argument(
        "--plots",
        type=Path,
        default=None,
        help="directory to write per-fight detail plots and an aggregate plot",
    )
    args = ap.parse_args()

    summaries = []
    for path in args.files:
        if not path.exists():
            print(f"File not found: {path}", file=sys.stderr)
            sys.exit(1)
        summaries.append(analyze(path, args))

    if args.plots and len(summaries) > 1:
        args.plots.mkdir(parents=True, exist_ok=True)
        out = args.plots / "aggregate.png"
        plot_aggregate(summaries, out)
        print(f"\nWrote aggregate plot: {out}")


if __name__ == "__main__":
    main()
