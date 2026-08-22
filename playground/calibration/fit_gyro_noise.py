"""Fit the gyro noise model from a long stationary recording (runbook E2).

A bias estimate taken from a 10 s still hold is only good for as long as the bias stays
put. The Allan deviation curve says how long that is: it falls while averaging still helps,
bottoms out where bias instability takes over, and climbs once the bias itself is wandering.
The tau at the minimum is the answer to "how often do the still holds need retaking", and
the two slopes on either side are the process-noise terms the filter needs for heading.

    source scripts/activate_python.sh
    python playground/calibration/fit_gyro_noise.py \\
        playground/calibration/out/gyro_bias/LOG-97.TXT \\
        --plot playground/calibration/out/gyro_noise.png

Runs on one log, so a 30 min recording of a robot doing nothing is the whole experiment.
Longer is better: the curve is only trustworthy out to about a tenth of the record, because
past that there are too few independent clusters left to average.

Three axes plus yaw. Yaw is taken along measured gravity rather than off whichever gyro
channel happens to point up, matching E0 and fit_gyro_scale.py: at rest the accelerometer
reads +1 g on the axis pointing up, so the unit vector along it is the yaw axis whatever
the mount looks like. That survives a tilted remount where a fixed channel would not.

The estimators follow IEEE 952:

    N  angle random walk    sigma = N / sqrt(tau)     slope -1/2, read at tau = 1 s
    B  bias instability     sigma = 0.664 * B         slope 0, the floor of the curve
    K  rate random walk     sigma = K * sqrt(tau / 3) slope +1/2, read at tau = 3 s

Nothing here reads the encoder or the command log, so this is the one experiment that needs
no radio and no clear floor.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from auto_battlebot.velocity_jig import JigLog, read_jig_log

# Clusters longer than this fraction of the record average too few independent samples to
# mean anything: at tau = T/3 there are three clusters, and the curve turns into noise about
# its own noise. IEEE 952 draws confidence bands instead; truncating is the blunt version and
# is enough to keep a reader from over-reading the right-hand tail.
MAX_CLUSTER_FRACTION = 0.1

# Points per decade on the tau axis. Log-spaced, because that is how the curve is read.
POINTS_PER_DECADE = 12


@dataclass(frozen=True)
class AllanCurve:
    """One axis: the deviation curve and the three terms fitted off it."""

    name: str
    tau: np.ndarray  # s
    sigma: np.ndarray  # deg/s
    bias_dps: float
    arw_deg_sqrt_hr: float
    bias_instability_dps: float
    tau_min_s: float
    # NaN when the curve never turned up inside this record, which is a statement about the
    # record's length, not about the gyro.
    rrw_deg_per_s_sqrt_s: float

    @property
    def bias_instability_deg_hr(self) -> float:
        return self.bias_instability_dps * 3600.0

    @property
    def rrw_deg_hr_sqrt_hr(self) -> float:
        """K in the conventional unit. One deg/s^1.5 is 3600^1.5 deg/hr^1.5."""
        return float(self.rrw_deg_per_s_sqrt_s * 3600.0**1.5)


def overlapping_allan(rate: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Overlapping Allan deviation of a rate signal.

    Works on the integrated angle, which turns the three-sample second difference into two
    array slices and makes the whole sweep a handful of passes over the data instead of a
    Python loop per cluster.

        sigma^2(tau) = sum (theta[k+2m] - 2 theta[k+m] + theta[k])^2 / (2 tau^2 (n - 2m))

    Overlapping rather than plain: every offset within a cluster contributes, which buys
    roughly sqrt(3) tighter confidence for the same record at no extra cost here.
    """
    n = len(rate)
    theta = np.concatenate([[0.0], np.cumsum(rate) * dt])
    m_max = int(n * MAX_CLUSTER_FRACTION)
    if m_max < 1:
        raise ValueError(f"record is only {n} samples, too short for an Allan curve")
    decades = np.log10(m_max)
    m = np.unique(np.round(np.logspace(0, decades, max(2, int(decades * POINTS_PER_DECADE + 1)))))
    m = m[m <= m_max].astype(int)

    tau = m * dt
    sigma = np.empty(len(m))
    for i, mi in enumerate(m):
        d = theta[2 * mi :] - 2.0 * theta[mi:-mi] + theta[: -2 * mi]
        sigma[i] = np.sqrt(np.sum(d * d) / (2.0 * tau[i] ** 2 * len(d)))
    return tau, sigma


def _read_at(
    tau: np.ndarray,
    sigma: np.ndarray,
    target: float,
    slope: float,
    band: np.ndarray | None = None,
) -> float:
    """Value of the slope-`slope` asymptote at tau = `target`.

    Reading a single point off a noisy curve throws away most of the record, so this fits the
    line of the known slope through `band` and evaluates it at the target. `band` defaults to
    the decade around the target, which keeps the fit inside the region where that slope
    actually holds instead of dragging in the part of the curve governed by a different term.
    Pass a band explicitly when the asymptote lives somewhere other than near the target,
    which is the case for rate random walk on any record short enough that the +1/2 region
    has not opened up by tau = 3 s.
    """
    if band is None:
        lo, hi = target / np.sqrt(10.0), target * np.sqrt(10.0)
        band = (tau >= lo) & (tau <= hi)
    if np.count_nonzero(band) < 2:
        band = np.argsort(np.abs(np.log(tau / target)))[:3]
    # log(sigma) = log(c) + slope * log(tau), only the intercept is free.
    c = np.mean(np.log(sigma[band]) - slope * np.log(tau[band]))
    return float(np.exp(c + slope * np.log(target)))


# A record only measures rate random walk if its curve has actually turned up. Requiring the
# far end to sit this far above the floor rejects a curve that merely stopped falling, where
# fitting a +1/2 line would report the noise floor dressed up as a drift term.
RRW_RISE_RATIO = 1.15


def rising_band(tau: np.ndarray, sigma: np.ndarray) -> np.ndarray | None:
    """Cluster times past the floor, if the curve climbs far enough there to fit a slope."""
    floor = int(np.argmin(sigma))
    band = np.zeros(len(tau), dtype=bool)
    band[floor:] = True
    if np.count_nonzero(band) < 3:
        return None
    if sigma[band].max() < sigma[floor] * RRW_RISE_RATIO:
        return None
    return band


def fit_axis(name: str, rate_dps: np.ndarray, dt: float) -> AllanCurve:
    tau, sigma = overlapping_allan(rate_dps, dt)
    floor = int(np.argmin(sigma))
    n_at_1s = _read_at(tau, sigma, 1.0, -0.5)
    band = rising_band(tau, sigma)
    k_at_3s = float("nan") if band is None else _read_at(tau, sigma, 3.0, +0.5, band)
    return AllanCurve(
        name=name,
        tau=tau,
        sigma=sigma,
        bias_dps=float(np.mean(rate_dps)),
        # sigma(1 s) is N in deg/sqrt(s); deg/sqrt(hr) is sqrt(3600) = 60 times that.
        arw_deg_sqrt_hr=n_at_1s * 60.0,
        bias_instability_dps=float(sigma[floor]) / 0.664,
        tau_min_s=float(tau[floor]),
        rrw_deg_per_s_sqrt_s=k_at_3s / np.sqrt(3.0),
    )


def yaw_rate(log: JigLog) -> np.ndarray:
    """Gyro projected onto measured gravity, in deg/s.

    The whole record is stationary, so the mean accelerometer vector is gravity and there is
    no centripetal term to swamp it. A 20 mg accel bias tilts this by about 20 mrad, which
    leaks 2% of the roll and pitch rates into yaw; on a robot sitting still that is 2% of
    nothing.
    """
    up = log.accel.mean(axis=0)
    up = up / np.linalg.norm(up)
    return np.asarray((log.gyro @ up) * 180.0 / np.pi, dtype=float)


def make_plot(path: Path, curves: list[AllanCurve], duration_s: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 6))
    for curve in curves:
        (line,) = ax.loglog(curve.tau, curve.sigma, lw=1.5, label=curve.name)
        ax.plot(
            curve.tau_min_s,
            curve.sigma.min(),
            "o",
            color=line.get_color(),
            ms=6,
            mfc="none",
        )

    # Slope guides, placed off the first curve so they sit on the data rather than floating.
    ref = curves[0]
    c = _read_at(ref.tau, ref.sigma, 1.0, -0.5)
    guide = c * ref.tau**-0.5
    ax.loglog(ref.tau, guide, "--", color="0.6", lw=0.9)
    ax.annotate("-1/2  angle random walk", (ref.tau[2], guide[2]), color="0.4", fontsize=8)

    # The +1/2 guide is anchored in the region that actually rises, and only drawn over that
    # region. Drawn across the whole axis from a tau = 3 s anchor it would sit a decade under
    # the data and imply a drift term this record cannot see.
    rise = next((c for c in curves if np.isfinite(c.rrw_deg_per_s_sqrt_s)), None)
    if rise is not None:
        band = rising_band(rise.tau, rise.sigma)
        assert band is not None
        k = _read_at(rise.tau, rise.sigma, 3.0, +0.5, band)
        span = rise.tau[band]
        ax.loglog(span, k * (span / 3.0) ** 0.5, "--", color="0.6", lw=0.9)
        ax.annotate(
            f"+1/2  rate random walk ({rise.name})",
            (span[0], k * (span[0] / 3.0) ** 0.5),
            color="0.4",
            fontsize=8,
            ha="left",
            va="bottom",
        )

    ax.set_xlabel("cluster time tau (s)")
    ax.set_ylabel("Allan deviation (deg/s)")
    ax.set_title(f"Gyro Allan deviation, {duration_s / 60:.0f} min stationary")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    ax.set_ylim(bottom=min(c.sigma.min() for c in curves) * 0.6)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    print(f"\nwrote {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("log", type=Path, help="stationary jig log (runbook E2)")
    parser.add_argument("--plot", type=Path, default=None)
    parser.add_argument(
        "--min-minutes",
        type=float,
        default=20.0,
        help="warn below this; the curve cannot reach the bias-instability floor on a short record",
    )
    args = parser.parse_args()

    log = read_jig_log(args.log)
    gyro_dps = log.gyro * 180.0 / np.pi

    print(
        f"{args.log.name}: {len(log.t):,} samples, {log.duration / 60:.1f} min, "
        f"dt {log.dt * 1000:.3f} ms"
    )
    print(
        f"  gyro range {log.header.gyro_range_dps:.0f} dps, "
        f"accel range {log.header.accel_range_g:.2f} g"
    )

    gaps = int(np.count_nonzero(np.diff(log.t) > 5 * log.dt))
    if gaps:
        print(f"  {gaps} gaps in the 1 kHz stream. The curve past those cluster times is wrong.")
    saturated = [a.name for a in log.saturation() if a.count]
    if saturated:
        print(f"  SATURATED on {', '.join(saturated)}. This run was not stationary.")
    if log.duration < args.min_minutes * 60.0:
        print(
            f"  Only {log.duration / 60:.1f} min. Clusters stop at "
            f"{log.duration * MAX_CLUSTER_FRACTION:.0f} s, which may be short of the floor."
        )

    curves = [fit_axis(f"g{axis}", gyro_dps[:, i], log.dt) for i, axis in enumerate("xyz")]
    curves.append(fit_axis("yaw", yaw_rate(log), log.dt))
    log_duration_hint = log.duration * MAX_CLUSTER_FRACTION

    print(f"\n{'axis':<6}{'bias dps':>10}{'ARW':>12}{'B':>12}{'tau_min':>10}{'RRW':>14}")
    print(f"{'':6}{'':>10}{'deg/sqrt(hr)':>12}{'dps':>12}{'s':>10}{'deg/hr^1.5':>14}")
    for c in curves:
        rrw = "n/a" if not np.isfinite(c.rrw_deg_hr_sqrt_hr) else f"{c.rrw_deg_hr_sqrt_hr:.1f}"
        print(
            f"{c.name:<6}{c.bias_dps:>10.4f}{c.arw_deg_sqrt_hr:>12.4f}"
            f"{c.bias_instability_dps:>12.5f}{c.tau_min_s:>10.1f}{rrw:>14}"
        )
    if any(not np.isfinite(c.rrw_deg_per_s_sqrt_s) for c in curves):
        flat = ", ".join(c.name for c in curves if not np.isfinite(c.rrw_deg_per_s_sqrt_s))
        print(
            f"\n  RRW is n/a on {flat}: the curve had not turned up by "
            f"{log_duration_hint:.0f} s, the longest cluster this record supports. That bounds"
        )
        print("  the drift from above rather than measuring it. A longer record would fix it.")

    yaw = curves[-1]
    print(f"\nThe curve bottoms out at tau = {yaw.tau_min_s:.0f} s on yaw, so a bias estimate")
    print("averaged over about that long is as good as this gyro gets. Averaging a still hold")
    print("longer than that makes it worse, not better.")
    hold = 10.0
    at_hold = _read_at(yaw.tau, yaw.sigma, hold, -0.5)
    print(f"  A {hold:.0f} s still hold pins the yaw bias to {at_hold:.4f} deg/s.")
    print(f"  Over a 30 s run that is {at_hold * 30.0:.3f} deg of unmodelled heading drift.")
    if yaw.tau_min_s < hold:
        print("  That hold is longer than tau_min, so it is averaging into the rising side of")
        print(f"  the curve. {yaw.tau_min_s:.0f} s would pin it tighter.")

    if args.plot:
        make_plot(args.plot, curves, log.duration)

    print("\n[gyro]")
    print(f"bias_dps = [{', '.join(f'{c.bias_dps:.5f}' for c in curves[:3])}]")
    print(f"arw_deg_sqrt_hr = {yaw.arw_deg_sqrt_hr:.4f}")
    print(f"bias_instability_dps = {yaw.bias_instability_dps:.5f}")
    print(f"bias_tau_s = {yaw.tau_min_s:.1f}")
    if np.isfinite(yaw.rrw_deg_per_s_sqrt_s):
        print(f"rrw_deg_per_s_sqrt_s = {yaw.rrw_deg_per_s_sqrt_s:.3e}")
    else:
        print("# rrw_deg_per_s_sqrt_s not measurable: the yaw curve never turned up")


if __name__ == "__main__":
    main()
