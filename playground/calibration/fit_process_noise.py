"""Fit the process noise model from velocity jig validation runs.

Takes the plant parameters from `fit_jig_plant.py` and the same session data, runs the model open
loop over sliding windows on the held-out runs, and turns the residuals into a covariance the EKF
can use. The point is getting the covariance *right*, not small: a confident wrong estimate is
worse in a match than an honestly wide one.

Method, following the plan's part 1.7:

1. Collect body-frame residuals (along-track, cross-track, heading) per window start and horizon,
   with the conditioning variables that might explain them (speed, yaw rate).
2. Fit the growth law `sigma^2(h) = a*h + b*h^2 + c*h^3` per axis. Each coefficient is a
   mechanism, so the fit says which mechanism dominates instead of just drawing a curve:
     - `c*h^3` is white-noise acceleration with PSD q, giving sigma_p^2 = q h^3 / 3.
     - `b*h^2` is a velocity scale-factor error: a constant fractional speed error eps gives
       sigma_p = |v| h eps.
     - `a*h`   is a random walk, which is how gyro angle random walk reaches cross-track error.
     - a constant term, if the fit demands one, is delay jitter: sigma_p ~ v * sigma_delay.
3. Map the coefficients into a continuous-time Q, so the runtime covariance is derived rather
   than hand-tuned.
4. Test consistency: coverage at 68/95/99, mean NEES against its chi-square band, and the tail
   behaviour a coverage number hides.

Usage:
    source scripts/activate_python.sh
    python playground/calibration/fit_process_noise.py \\
        playground/calibration/out/session_A.json \\
        --calibration playground/calibration/out/jig_calibration.toml \\
        --params playground/calibration/out/plant_params.toml \\
        --out playground/calibration/out/process_noise.toml
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.optimize import nnls
from scipy.stats import chi2

DEFAULT_CALIBRATION = Path(__file__).resolve().parent / "jig_calibration.toml"

from auto_battlebot.calibration.jig_fit import build_windows, load_all
from auto_battlebot.plant import (
    MODEL_LADDER,
    ModelStructure,
    PlantParams,
    WindowErrors,
    predict_windows,
    toml_float,
)
from auto_battlebot.velocity_jig import (
    JigCalibration,
)

# Horizon grid for the noise model. Denser than the fit grid because the covariance has to be
# honest at every horizon the filter coasts over, not only at the ones the fit optimized.
NOISE_HORIZONS = (0.033, 0.066, 0.100, 0.150, 0.200, 0.300, 0.400, 0.500)

# Speed bins for the conditional fits. The scale-factor term should dominate at high speed and
# the acceleration term near zero, and binning is what shows whether it does.
SPEED_BINS = (0.0, 0.5, 1.5, 3.0, 10.0)

# 95% of a 2D Gaussian sits inside 2.448 sigma, not 2. Getting this wrong is the most common way
# a "within 2 sigma" claim turns out to mean 86.5% coverage.
CHI2_2D_95 = float(np.sqrt(chi2.ppf(0.95, df=2)))


# ---------------------------------------------------------------------------
# Growth law
# ---------------------------------------------------------------------------


@dataclass
class GrowthFit:
    """One axis's variance growth law, with each term named by its mechanism."""

    axis: str
    const: float  # m^2 or rad^2, delay jitter
    a: float  # per second, random walk
    b: float  # per second^2, scale-factor error
    c: float  # per second^3, white-noise acceleration
    horizons: np.ndarray
    measured_var: np.ndarray
    n: int

    def variance(self, h: np.ndarray | float) -> np.ndarray:
        h = np.asarray(h, dtype=float)
        return self.const + self.a * h + self.b * h**2 + self.c * h**3

    def sigma(self, h: np.ndarray | float) -> np.ndarray:
        return np.sqrt(np.maximum(self.variance(h), 0.0))

    def dominant(self) -> str:
        """Which mechanism carries the variance at 400 ms, the horizon that matters most."""
        h = 0.4
        terms = {
            "delay jitter": self.const,
            "random walk": self.a * h,
            "scale factor": self.b * h**2,
            "white-noise accel": self.c * h**3,
        }
        return max(terms.items(), key=lambda kv: kv[1])[0]

    def rms_fit_error(self) -> float:
        pred = self.variance(self.horizons)
        return float(np.sqrt(np.mean((pred - self.measured_var) ** 2)))


def fit_growth(axis: str, horizons: np.ndarray, residuals: np.ndarray) -> GrowthFit:
    """Fit `sigma^2(h) = const + a h + b h^2 + c h^3` with non-negative coefficients.

    Non-negativity is not a convenience. A negative coefficient would mean a mechanism that
    removes variance as the horizon grows, which no term here can do, and an unconstrained fit
    will happily produce one and then predict negative variance somewhere.
    """
    var = np.var(residuals, axis=0)
    basis = np.stack([np.ones_like(horizons), horizons, horizons**2, horizons**3], axis=1)
    coeff, _ = nnls(basis, var)
    return GrowthFit(
        axis=axis,
        const=float(coeff[0]),
        a=float(coeff[1]),
        b=float(coeff[2]),
        c=float(coeff[3]),
        horizons=horizons,
        measured_var=var,
        n=residuals.shape[0],
    )


# ---------------------------------------------------------------------------
# Q
# ---------------------------------------------------------------------------


@dataclass
class ProcessNoise:
    """Continuous-time process noise for the our-robot EKF, derived from the growth law.

    The along-track and cross-track white-noise acceleration PSDs are kept separate: they come
    from different mechanisms (speed error along track, heading error across it) and forcing them
    equal would inflate one to cover the other.
    """

    q_along: float  # (m/s^2)^2 / Hz, from c_along = q/3
    q_cross: float
    q_heading: float  # (rad/s^2)^2 / Hz
    scale_factor: float  # fractional speed error, dimensionless
    heading_scale_factor: float  # fractional yaw-rate error, dimensionless
    heading_rw: float  # rad^2/s, angle random walk feeding cross-track
    delay_jitter_s: float  # s, from the constant term at the median speed
    median_speed: float
    median_yaw_rate: float

    def to_toml(self, table: str = "process_noise") -> str:
        lines = [
            f"[{table}]",
            "# Derived from windowed open-loop residuals, not hand-tuned. See",
            "# playground/calibration/fit_process_noise.py.",
            f"q_along = {toml_float(self.q_along)}  # (m/s^2)^2/Hz, sigma_p^2 = q h^3 / 3",
            f"q_cross = {toml_float(self.q_cross)}",
            f"q_heading = {toml_float(self.q_heading)}  # (rad/s^2)^2/Hz",
            f"scale_factor = {toml_float(self.scale_factor)}  # sigma = |v| h eps",
            f"heading_scale_factor = {toml_float(self.heading_scale_factor)}  # sigma = |w| h eps",
            f"heading_random_walk = {toml_float(self.heading_rw)}  # rad^2/s",
            f"delay_jitter_s = {toml_float(self.delay_jitter_s)}",
        ]
        return "\n".join(lines) + "\n"


def build_q(
    fits: dict[str, GrowthFit], median_speed: float, median_yaw_rate: float
) -> ProcessNoise:
    """Map the fitted coefficients onto the EKF's continuous-time noise terms.

    sigma_p^2 = q h^3 / 3 is the standard position variance under white-noise acceleration, so
    q = 3c. The scale-factor terms are state dependent (they grow with speed and yaw rate), so
    they are stored as fractions and applied at runtime as |v| h eps and |w| h eps rather than
    folded into q here. The heading b term is the dominant heading mechanism at 400 ms; dropping
    it would understate heading sigma about 4x, a confidently wrong covariance.
    """
    along = fits["along"]
    cross = fits["cross"]
    heading = fits["heading"]
    eps = float(np.sqrt(along.b)) / max(median_speed, 1e-6)
    eps_heading = float(np.sqrt(heading.b)) / max(median_yaw_rate, 1e-6)
    jitter = float(np.sqrt(along.const)) / max(median_speed, 1e-6)
    return ProcessNoise(
        q_along=3.0 * along.c,
        q_cross=3.0 * cross.c,
        q_heading=3.0 * heading.c,
        scale_factor=eps,
        heading_scale_factor=eps_heading,
        heading_rw=heading.a,
        delay_jitter_s=jitter,
        median_speed=median_speed,
        median_yaw_rate=median_yaw_rate,
    )


# ---------------------------------------------------------------------------
# Consistency
# ---------------------------------------------------------------------------


@dataclass
class CoverageRow:
    horizon: float
    n: int
    cover68: float
    cover95: float
    cover99: float
    nees_mean: float
    nees_lo: float
    nees_hi: float


def coverage(err: WindowErrors, along: GrowthFit, cross: GrowthFit) -> list[CoverageRow]:
    """Empirical coverage and mean NEES per horizon bin, for the 2D position error.

    NEES is the normalized estimation error squared: with the covariance right it averages the
    state dimension (2 here) and its sample mean over N windows sits inside a chi-square band.
    Coverage alone can look fine while the tails are wrong, which is why both are reported.
    """
    rows: list[CoverageRow] = []
    for i, h in enumerate(err.horizons):
        s_along = float(along.sigma(h))
        s_cross = float(cross.sigma(h))
        if s_along <= 0 or s_cross <= 0:
            continue
        d2 = (err.along[:, i] / s_along) ** 2 + (err.cross[:, i] / s_cross) ** 2
        n = len(d2)
        rows.append(
            CoverageRow(
                horizon=float(h),
                n=n,
                cover68=float(np.mean(d2 <= chi2.ppf(0.68, df=2))),
                cover95=float(np.mean(d2 <= chi2.ppf(0.95, df=2))),
                cover99=float(np.mean(d2 <= chi2.ppf(0.99, df=2))),
                nees_mean=float(np.mean(d2)),
                nees_lo=float(chi2.ppf(0.025, df=2 * n) / n),
                nees_hi=float(chi2.ppf(0.975, df=2 * n) / n),
            )
        )
    return rows


def tail_report(err: WindowErrors) -> list[tuple[float, float, float]]:
    """p99 and p99.9 position error per horizon.

    Combat means getting hit, and an impact produces accelerations far outside the Gaussian body.
    Those belong in a separate line of the report, not inside a Q large enough to cover them,
    which would ruin the estimate everywhere else.
    """
    out = []
    for i, h in enumerate(err.horizons):
        pos = err.position[:, i] * 1e3
        out.append((float(h), float(np.percentile(pos, 99)), float(np.percentile(pos, 99.9))))
    return out


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


def print_growth(fit: GrowthFit) -> None:
    heading = fit.axis == "heading"
    unit, label = (np.degrees(1.0) * 1e3, "mdeg") if heading else (1e3, "mm")
    print(f"\n  {fit.axis} (n={fit.n} windows, sigma in {label})")
    print(
        f"    const {fit.const:.3e}   a {fit.a:.3e}   b {fit.b:.3e}   c {fit.c:.3e}"
        f"   dominant at 400 ms: {fit.dominant()}"
    )
    print("    horizon ms |" + "".join(f"{h * 1e3:10.0f}" for h in fit.horizons))
    print("    measured   |" + "".join(f"{np.sqrt(v) * unit:10.3g}" for v in fit.measured_var))
    print("    modelled   |" + "".join(f"{s * unit:10.3g}" for s in fit.sigma(fit.horizons)))
    if fit.const == 0.0 and fit.a == 0.0 and fit.b == 0.0 and fit.c == 0.0:
        print(
            "    WARNING: every coefficient fit to zero, so this axis has no covariance growth"
            " at all.\n             Do not ship that: check the residuals before using this Q."
        )


def print_coverage(rows: Sequence[CoverageRow]) -> None:
    print("\n=== Coverage and NEES (2D position) ===")
    print("    horizon ms |   68%   |   95%   |   99%   | mean NEES (95% band)")
    for row in rows:
        band = f"[{row.nees_lo:.2f}, {row.nees_hi:.2f}]"
        flag = "" if row.nees_lo <= row.nees_mean <= row.nees_hi else "   <- outside band"
        print(
            f"    {row.horizon * 1e3:10.0f} | {row.cover68:7.3f} | {row.cover95:7.3f} |"
            f" {row.cover99:7.3f} | {row.nees_mean:5.2f} {band}{flag}"
        )
    print("    Nominal coverage is 0.68 / 0.95 / 0.99. The 95% ellipse is at 2.448 sigma in 2D.")


def make_plot(path: Path, fits: dict[str, GrowthFit], rows: Sequence[CoverageRow]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    for ax, (name, fit) in zip(axes[:2], [(k, v) for k, v in fits.items() if k != "heading"]):
        ax.plot(fit.horizons * 1e3, np.sqrt(fit.measured_var) * 1e3, "o", label="measured")
        hh = np.linspace(0, fit.horizons.max(), 100)
        ax.plot(hh * 1e3, fit.sigma(hh) * 1e3, "-", label="model")
        ax.set_title(f"{name} sigma, dominant: {fit.dominant()}", fontsize=9)
        ax.set_xlabel("horizon (ms)", fontsize=8)
        ax.set_ylabel("sigma (mm)", fontsize=8)
        ax.legend(fontsize=7)

    ax = axes[2]
    ax.plot([r.horizon * 1e3 for r in rows], [r.cover95 for r in rows], "o-", label="95% coverage")
    ax.axhline(0.95, color="0.5", ls=":")
    ax.set_ylim(0.5, 1.02)
    ax.set_xlabel("horizon (ms)", fontsize=8)
    ax.set_ylabel("empirical coverage", fontsize=8)
    ax.legend(fontsize=7)

    fig.suptitle("Process noise model", fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "sessions",
        type=Path,
        nargs="+",
        help="session directories written by velocity_jig_drive.py",
    )
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--params", type=Path, required=True, help="plant TOML from fit_jig_plant")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--plot", type=Path, default=None)
    parser.add_argument("--fit-hz", type=float, default=200.0)
    parser.add_argument("--stride-ms", type=float, default=25.0)
    parser.add_argument("--max-windows", type=int, default=20000)
    parser.add_argument("--model", default="M4", choices=[m.name for m in MODEL_LADDER])
    parser.add_argument("--keep-bad", action="store_true")
    parser.add_argument(
        "--include-train",
        action="store_true",
        help="also use the runs the plant was fit on (they flatter the noise model)",
    )
    args = parser.parse_args()

    calib = JigCalibration.from_toml(args.calibration)
    params = PlantParams.from_toml(args.params)
    structure: ModelStructure = next(m for m in MODEL_LADDER if m.name == args.model)
    print(f"plant: {args.params}, model {structure.name}, delay {params.delay_s * 1e3:.1f} ms")

    loaded = load_all(args.sessions, calib, args.fit_hz, args.keep_bad)
    runs = loaded.runs if args.include_train else loaded.select(roles=("holdout",))
    if not runs:
        print("no validation runs found (E14-E19). Use --include-train to fall back to all runs.")
        return
    print(f"using {len(runs)} runs: {', '.join(sorted({r.waveform for r in runs}))}")

    windows = build_windows(
        runs,
        params.delay_s,
        NOISE_HORIZONS,
        args.stride_ms / 1000.0,
        args.max_windows,
    )
    if windows is None:
        print("no usable windows")
        return
    err = predict_windows(windows, params, structure)
    print(f"{windows.count()} windows, {len(err.horizons)} horizons")

    print("\n=== Error growth law: sigma^2(h) = const + a h + b h^2 + c h^3 ===")
    fits = {
        "along": fit_growth("along", err.horizons, err.along),
        "cross": fit_growth("cross", err.horizons, err.cross),
        "heading": fit_growth("heading", err.horizons, err.heading),
    }
    for fit in fits.values():
        print_growth(fit)

    print("\n=== Growth law by speed bin ===")
    for lo, hi in zip(SPEED_BINS[:-1], SPEED_BINS[1:]):
        sel = (err.speed >= lo) & (err.speed < hi)
        n = int(np.count_nonzero(sel))
        if n < 50:
            print(f"  {lo:.1f}-{hi:.1f} m/s: {n} windows, too few to fit")
            continue
        binned = fit_growth("along", err.horizons, err.along[sel])
        print(
            f"  {lo:.1f}-{hi:.1f} m/s: n={n}, b {binned.b:.3e}, c {binned.c:.3e},"
            f" dominant {binned.dominant()}"
        )

    median_speed = float(np.median(err.speed[err.speed > 0.1])) if np.any(err.speed > 0.1) else 1.0
    median_yaw_rate = (
        float(np.median(err.yaw_rate[err.yaw_rate > 0.1])) if np.any(err.yaw_rate > 0.1) else 1.0
    )
    noise = build_q(fits, median_speed, median_yaw_rate)
    print("\n=== Process noise ===")
    print(f"  q_along        {noise.q_along:.4g} (m/s^2)^2/Hz")
    print(f"  q_cross        {noise.q_cross:.4g} (m/s^2)^2/Hz")
    print(f"  q_heading      {noise.q_heading:.4g} (rad/s^2)^2/Hz")
    print(f"  scale factor   {noise.scale_factor:.4g} ({noise.scale_factor * 100:.2f}% of speed)")
    print(
        f"  heading scale  {noise.heading_scale_factor:.4g}"
        f" ({noise.heading_scale_factor * 100:.2f}% of yaw rate)"
    )
    print(f"  heading RW     {noise.heading_rw:.4g} rad^2/s")
    print(f"  delay jitter   {noise.delay_jitter_s * 1e3:.2f} ms at {median_speed:.2f} m/s")

    rows = coverage(err, fits["along"], fits["cross"])
    print_coverage(rows)
    passed = sum(1 for r in rows if r.nees_lo <= r.nees_mean <= r.nees_hi)
    print(f"  C4: {passed}/{len(rows)} horizon bins have mean NEES inside the 95% band.")

    print("\n=== Tails (impacts live here) ===")
    print("    horizon ms |  p99 mm | p99.9 mm")
    for h, p99, p999 in tail_report(err):
        print(f"    {h * 1e3:10.0f} | {p99:7.1f} | {p999:8.1f}")
    print(
        "    Do not widen Q to cover these. An impact is a gated innovation and, if it persists,"
        "\n    a reinitialization."
    )

    if args.out:
        header = (
            "# Process noise for the our-robot EKF, fit from velocity jig validation runs.\n"
            f"# sessions: {', '.join(str(s) for s in args.sessions)}\n"
            f"# plant: {args.params}, model {structure.name}\n\n"
        )
        args.out.write_text(header + noise.to_toml(), encoding="utf-8")
        print(f"\nwrote {args.out}")
    if args.plot:
        make_plot(args.plot, fits, rows)


if __name__ == "__main__":
    main()
