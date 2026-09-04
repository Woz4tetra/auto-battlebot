"""Fit metres per encoder count from hand-pushed runs over measured distances (runbook E4).

The encoder scale is the constant every linear measurement depends on, so it is measured
rather than inferred from a top speed. Push the robot with the motors disarmed between two
marked points, dwelling at each so the stops are unambiguous, and this reads the count at
each dwell and fits counts against distance through the origin.

    source scripts/activate_python.sh
    python playground/calibration/fit_encoder_scale.py \\
        --run 0.5  LOG-38.TXT LOG-39.TXT LOG-40.TXT \\
        --run 0.75 LOG-42.TXT LOG-43.TXT LOG-44.TXT \\
        --run 1.0  LOG-45.TXT LOG-46.TXT LOG-47.TXT \\
        --plot out/encoder_scale.png

Several distances beat one distance repeated. A fixed error (the marks, or the robot not
quite stopping on them) is a constant offset in counts, so it bends a counts-against-distance
line away from the origin. With one distance that offset is invisible and lands entirely in
the scale; with three it shows up as an intercept, which this reports.

An out-and-back is worth more than a one-way push, because returning to the start makes the
closure error visible, and closure is the only check a single pass affords.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from auto_battlebot.velocity_jig import read_jig_log


@dataclass(frozen=True)
class Dwell:
    """A stretch where the count held still, meaning the robot was parked on a mark."""

    t0: float
    t1: float
    count: float

    @property
    def duration(self) -> float:
        return self.t1 - self.t0


@dataclass
class Pass:
    """One hand-pushed out-and-back."""

    path: Path
    distance_m: float
    origin: float
    far: float
    closure_counts: float
    drift_counts: float  # spread among candidate far dwells, so wheel creep is visible
    dwells: list[Dwell]
    t: np.ndarray
    count: np.ndarray

    @property
    def span(self) -> float:
        return abs(self.far - self.origin)


def find_dwells(
    t: np.ndarray, count: np.ndarray, *, min_s: float = 0.75, tol_counts: float = 40.0
) -> list[Dwell]:
    """Stretches where the count barely moves, which is where the marks are.

    The tolerance is in counts rather than a velocity threshold: a hand push is slow, so a
    velocity bar tight enough to exclude the push would also split every dwell in two.
    """
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.001
    window = max(2, int(round(min_s / max(dt, 1e-9))))
    if len(count) < window:
        return []

    still = np.zeros(len(count), dtype=bool)
    for i in range(0, len(count) - window):
        chunk = count[i : i + window]
        if chunk.max() - chunk.min() <= tol_counts:
            still[i : i + window] = True

    out: list[Dwell] = []
    if not still.any():
        return out
    edges = np.flatnonzero(np.diff(still.astype(int)))
    bounds = np.concatenate([[0], edges + 1, [len(still)]])
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        if not still[lo] or (t[min(hi, len(t) - 1)] - t[lo]) < min_s:
            continue
        out.append(
            Dwell(float(t[lo]), float(t[min(hi, len(t) - 1)]), float(np.median(count[lo:hi])))
        )
    return out


def read_pass(path: Path, distance_m: float, *, min_s: float, tol: float) -> Pass:
    log = read_jig_log(path)
    if not log.encoder_moved:
        raise ValueError(f"{path.name}: the encoder count never changed; check the connector")
    t = log.t - log.t[0]
    count = log.count.astype(float)
    dwells = find_dwells(t, count, min_s=min_s, tol_counts=tol)
    if len(dwells) < 2:
        raise ValueError(
            f"{path.name}: found {len(dwells)} dwells, need at least the start and the far "
            "mark. Loosen --tol-counts or shorten --min-dwell-s."
        )

    origin = dwells[0].count
    far_dwell = max(dwells, key=lambda d: abs(d.count - origin))
    # Dwells within a tenth of the span of the far one are all "at the far mark"; the spread
    # among them is the wheel creeping while the robot sat still, which is a real error term.
    span = abs(far_dwell.count - origin)
    near_far = [d.count for d in dwells if abs(abs(d.count - origin) - span) < 0.1 * span]
    return Pass(
        path=path,
        distance_m=distance_m,
        origin=origin,
        far=far_dwell.count,
        closure_counts=dwells[-1].count - origin,
        drift_counts=float(max(near_far) - min(near_far)) if len(near_far) > 1 else 0.0,
        dwells=dwells,
        t=t,
        count=count,
    )


@dataclass
class Fit:
    """Both fits, and which one to believe.

    Through the origin is the model you want to be true: zero distance, zero counts. The
    two-parameter fit exists to test it. When the intercept is real, the through-origin slope
    is pulled up by a fixed per-pass error and overstates the scale, while the differential
    slope is honest because a constant offset cancels out of it.
    """

    counts_per_m: float  # through the origin
    stderr: float
    slope_counts_per_m: float  # allowing an intercept
    slope_stderr: float
    intercept_counts: float
    intercept_stderr: float
    n: int
    distances: int

    @property
    def intercept_real(self) -> bool:
        """Whether the offset is bigger than the fit can call noise."""
        return (
            self.distances > 1
            and np.isfinite(self.intercept_stderr)
            and abs(self.intercept_counts) > 2 * self.intercept_stderr
        )

    @property
    def best_counts_per_m(self) -> float:
        return self.slope_counts_per_m if self.intercept_real else self.counts_per_m

    @property
    def best_stderr(self) -> float:
        return self.slope_stderr if self.intercept_real else self.stderr

    @property
    def meters_per_count(self) -> float:
        return 1.0 / self.best_counts_per_m

    @property
    def rel_stderr(self) -> float:
        return self.best_stderr / abs(self.best_counts_per_m)


def fit_scale(passes: list[Pass]) -> Fit:
    """Counts against distance, through the origin, plus an intercept as a diagnostic.

    Through the origin is the model: zero distance is zero counts. The intercept is fitted
    separately only to check that assumption. An intercept far from zero means a fixed error
    per pass, most likely the marks or where the robot actually came to rest, and it inflates
    the scale fitted from short pushes.
    """
    x = np.array([p.distance_m for p in passes])
    y = np.array([p.span for p in passes])

    k = float(np.sum(x * y) / np.sum(x * x))
    resid = y - k * x
    dof = max(len(x) - 1, 1)
    se = float(np.sqrt(np.sum(resid**2) / dof / np.sum(x * x)))

    if len(set(x)) > 1:
        design = np.stack([x, np.ones_like(x)], axis=1)
        coeff, *_ = np.linalg.lstsq(design, y, rcond=None)
        slope, intercept = float(coeff[0]), float(coeff[1])
        r2 = y - design @ coeff
        cov = np.linalg.inv(design.T @ design) * float(r2 @ r2) / max(len(x) - 2, 1)
        slope_se, intercept_se = float(np.sqrt(cov[0, 0])), float(np.sqrt(cov[1, 1]))
    else:
        # One distance cannot separate a scale from an offset, so there is nothing to test.
        slope, intercept = k, 0.0
        slope_se, intercept_se = se, float("nan")
    return Fit(k, se, slope, slope_se, intercept, intercept_se, len(x), len(set(x)))


def make_plot(path: Path, passes: list[Pass], fit: Fit, mark_error_mm: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    by_distance: dict[float, list[Pass]] = {}
    for p in passes:
        by_distance.setdefault(p.distance_m, []).append(p)
    distances = sorted(by_distance)

    fig = plt.figure(figsize=(13, 8.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 1.0], hspace=0.32, wspace=0.24)

    # Every pass end to end, so a stall, a bounce or a bad dwell is visible rather than
    # reduced to one number. Only two dwells per pass reach the fit, the start and the far
    # mark, so they are drawn filled and every other dwell hollow: the difference between
    # them is the whole reason a pass can look fine and still contribute a bad point.
    ax = fig.add_subplot(gs[0, :])
    for i, dist in enumerate(distances):
        for j, p in enumerate(by_distance[dist]):
            ax.plot(
                p.t,
                p.count * fit.meters_per_count,
                lw=1.0,
                color=f"C{i}",
                alpha=0.8,
                label=f"{dist:g} m" if j == 0 else None,
            )
            for d in p.dwells:
                used = d.count in (p.origin, p.far)
                y = d.count * fit.meters_per_count
                ax.plot(
                    [d.t0, d.t1],
                    [y] * 2,
                    lw=4.0 if used else 2.0,
                    color=f"C{i}",
                    alpha=0.9 if used else 0.3,
                    solid_capstyle="butt",
                )
                ax.plot(
                    [0.5 * (d.t0 + d.t1)],
                    [y],
                    marker="o" if used else "x",
                    ms=7 if used else 6,
                    mfc=f"C{i}" if used else "none",
                    mec=f"C{i}",
                    mew=1.6,
                    ls="none",
                )
        ax.axhline(dist, color=f"C{i}", ls=":", lw=1.0)
    ax.axhline(0.0, color="0.5", lw=1.0)
    handles, labels = ax.get_legend_handles_labels()
    handles += [
        Line2D([], [], marker="o", color="0.35", ls="none", ms=7, label="dwell used by the fit"),
        Line2D([], [], marker="x", color="0.35", ls="none", ms=6, label="dwell ignored"),
    ]
    labels += ["dwell used by the fit", "dwell ignored"]
    ax.set_xlabel("time (s)", fontsize=9)
    ax.set_ylabel("position (m, from the fitted scale)", fontsize=9)
    ax.set_title(
        "Each pass. Two dwells per pass reach the fit: the start and the far mark.",
        fontsize=10,
    )
    ax.legend(handles=handles, labels=labels, fontsize=8, ncol=len(distances) + 2)

    # The fit itself.
    ax = fig.add_subplot(gs[1, 0])
    x = np.array([p.distance_m for p in passes])
    y = np.array([p.span for p in passes])
    for i, dist in enumerate(distances):
        sel = x == dist
        ax.plot(x[sel], y[sel], "o", ms=6, color=f"C{i}")
    xs = np.linspace(0, x.max() * 1.05, 10)
    ax.plot(
        xs,
        fit.counts_per_m * xs,
        "--",
        lw=1.0,
        color="0.6",
        label=f"through origin: {fit.counts_per_m:,.0f} counts/m",
    )
    if fit.distances > 1:
        ax.plot(
            xs,
            fit.slope_counts_per_m * xs + fit.intercept_counts,
            "-",
            lw=1.4,
            color="0.2",
            label=f"with offset: {fit.slope_counts_per_m:,.0f} counts/m "
            f"{fit.intercept_counts:+,.0f}",
        )
    ax.set_xlabel("marked distance (m)", fontsize=9)
    ax.set_ylabel("encoder counts", fontsize=9)
    ax.legend(fontsize=8)
    ax.set_title("Counts against distance, through the origin", fontsize=10)

    # Residuals in millimetres, against the marking error that bounds them.
    ax = fig.add_subplot(gs[1, 1])
    model = (
        fit.slope_counts_per_m * x + fit.intercept_counts
        if fit.intercept_real
        else fit.counts_per_m * x
    )
    resid_mm = (y - model) * fit.meters_per_count * 1e3
    for i, dist in enumerate(distances):
        sel = x == dist
        ax.plot(x[sel], resid_mm[sel], "o", ms=6, color=f"C{i}")
    ax.axhline(0.0, color="0.5", lw=1.0)
    ax.axhspan(
        -mark_error_mm,
        mark_error_mm,
        color="0.85",
        zorder=0,
        label=f"+/-{mark_error_mm:g} mm marking error",
    )
    ax.set_xlabel("marked distance (m)", fontsize=9)
    ax.set_ylabel("residual (mm)", fontsize=9)
    ax.legend(fontsize=8)
    ax.set_title(
        "Residual from the " + ("offset fit" if fit.intercept_real else "through-origin fit"),
        fontsize=10,
    )

    fig.suptitle(
        f"Encoder scale: meters_per_count = {fit.meters_per_count:.6e} "
        f"+/- {fit.rel_stderr:.2%}  ({fit.n} passes)",
        fontsize=12,
    )
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"\nwrote {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--run",
        nargs="+",
        action="append",
        required=True,
        metavar=("DISTANCE", "LOG"),
        help="a measured distance in metres followed by the logs pushed over it; repeatable",
    )
    parser.add_argument("--min-dwell-s", type=float, default=0.75)
    parser.add_argument("--tol-counts", type=float, default=40.0)
    parser.add_argument(
        "--mark-error-mm",
        type=float,
        default=2.0,
        help="how well the marks and the robot's stopping point were measured",
    )
    parser.add_argument("--plot", type=Path, default=None)
    args = parser.parse_args()

    passes: list[Pass] = []
    for group in args.run:
        if len(group) < 2:
            parser.error(f"--run {group[0]} needs at least one log file")
        try:
            distance = float(group[0])
        except ValueError:
            parser.error(f"--run takes a distance first, got {group[0]!r}")
        for name in group[1:]:
            passes.append(
                read_pass(Path(name), distance, min_s=args.min_dwell_s, tol=args.tol_counts)
            )

    fit = fit_scale(passes)

    print(
        f"{'log':<14}{'dist m':>8}{'counts':>10}{'counts/m':>11}{'closure mm':>12}"
        f"{'dwell drift mm':>16}"
    )
    for p in sorted(passes, key=lambda q: (q.distance_m, q.path.name)):
        print(
            f"{p.path.name:<14}{p.distance_m:>8.3f}{p.span:>10.0f}"
            f"{p.span / p.distance_m:>11,.0f}"
            f"{p.closure_counts * fit.meters_per_count * 1e3:>12.1f}"
            f"{p.drift_counts * fit.meters_per_count * 1e3:>16.1f}"
        )

    print(f"\nfit over {fit.n} passes at {fit.distances} distance(s):")
    print(
        f"  through the origin : {fit.counts_per_m:>10,.1f} +/- {fit.stderr:>6,.1f} counts/m"
        f"   -> {1.0 / fit.counts_per_m:.6e} m/count"
    )
    if fit.distances > 1:
        print(
            f"  allowing an offset : {fit.slope_counts_per_m:>10,.1f} "
            f"+/- {fit.slope_stderr:>6,.1f} counts/m"
            f"   -> {1.0 / fit.slope_counts_per_m:.6e} m/count"
        )
        offset_mm = fit.intercept_counts / fit.slope_counts_per_m * 1e3
        print(
            f"  fixed offset       : {fit.intercept_counts:>+10,.0f} "
            f"+/- {fit.intercept_stderr:>6,.0f} counts   ({offset_mm:+.1f} mm per pass)"
        )
        if fit.intercept_real:
            gap = fit.counts_per_m / fit.slope_counts_per_m - 1.0
            print(
                f"\n  The offset is real, so the SLOPE is the scale. Every pass covers about"
                f" {offset_mm:+.0f} mm more than its mark says, most likely coasting past the"
                f" mark or backlash taken up at the start. A through-origin fit folds that"
                f" into the scale and comes out {gap:+.1%} off, worst at the shortest push."
            )
        else:
            print(
                "\n  The offset is consistent with zero, so the line goes through the"
                " origin and no fixed per-pass error is inflating the scale."
            )
    else:
        print(
            "  Only one distance, so a fixed per-pass offset cannot be told apart from the"
            " scale. Push over a second distance to test for one."
        )

    print(f"\nmeters_per_count = {fit.meters_per_count:.6e}  +/- {fit.rel_stderr:.2%}")

    worst_closure = max(abs(p.closure_counts * fit.meters_per_count) for p in passes)
    print(f"\nworst closure: {worst_closure * 1e3:.1f} mm")

    longest = max(p.distance_m for p in passes)
    mark_frac = (args.mark_error_mm * 1e-3) / longest
    print(
        f"marking error of {args.mark_error_mm:g} mm over the longest push ({longest:g} m)"
        f" is {mark_frac:.2%}"
    )
    total = max(fit.rel_stderr, mark_frac)
    print(
        f"\nuse {total:.2%} as the honest uncertainty on every fitted speed"
        + (", set by the marks rather than the spread" if mark_frac > fit.rel_stderr else "")
    )
    if total > 0.003:
        print("  Over the runbook's 0.3% bar. Push over a longer line to get under it.")

    if args.plot:
        make_plot(args.plot, passes, fit, args.mark_error_mm)

    print(f"\n[encoder]\nmeters_per_count = {fit.meters_per_count:.6e}")


if __name__ == "__main__":
    main()
