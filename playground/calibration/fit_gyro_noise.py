"""Fit the gyro noise model from a long stationary recording (runbook E2)."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from auto_battlebot.calibration.jig.velocity_jig import JigLog, read_jig_log

MAX_CLUSTER_FRACTION = 0.1

POINTS_PER_DECADE = 12
DEG_SQRT_HR_PER_DEG_SQRT_S = 60.0


@dataclass(frozen=True)
class AllanCurve:
    """One axis: the deviation curve and the three terms fitted off it."""

    name: str
    tau_s: np.ndarray
    sigma_dps: np.ndarray
    bias_dps: float
    arw_deg_sqrt_hr: float
    bias_instability_dps: float
    tau_min_s: float
    rrw_deg_per_s_sqrt_s: float | None

    @property
    def bias_instability_deg_hr(self) -> float:
        return self.bias_instability_dps * 3600.0

    @property
    def rrw_deg_hr_sqrt_hr(self) -> float | None:
        """K in the conventional unit. One deg/s^1.5 is 3600^1.5 deg/hr^1.5."""
        if self.rrw_deg_per_s_sqrt_s is None:
            return None
        return float(self.rrw_deg_per_s_sqrt_s * 3600.0**1.5)


def overlapping_allan(rate: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Overlapping Allan deviation of a rate signal."""
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
    """Value of the slope-`slope` asymptote at tau = `target`."""
    if band is None:
        lo, hi = target / np.sqrt(10.0), target * np.sqrt(10.0)
        band = (tau >= lo) & (tau <= hi)
    if np.count_nonzero(band) < 2:
        band = np.argsort(np.abs(np.log(tau / target)))[:3]
    c = np.mean(np.log(sigma[band]) - slope * np.log(tau[band]))
    return float(np.exp(c + slope * np.log(target)))


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
    k_at_3s = None if band is None else _read_at(tau, sigma, 3.0, +0.5, band)
    return AllanCurve(
        name=name,
        tau_s=tau,
        sigma_dps=sigma,
        bias_dps=float(np.mean(rate_dps)),
        arw_deg_sqrt_hr=n_at_1s * DEG_SQRT_HR_PER_DEG_SQRT_S,
        bias_instability_dps=float(sigma[floor]) / 0.664,
        tau_min_s=float(tau[floor]),
        rrw_deg_per_s_sqrt_s=None if k_at_3s is None else k_at_3s / np.sqrt(3.0),
    )


def yaw_rate(log: JigLog) -> np.ndarray:
    """Gyro projected onto measured gravity, in deg/s."""
    up = log.accel.mean(axis=0)
    up = up / np.linalg.norm(up)
    return np.asarray((log.gyro_rad_s @ up) * 180.0 / np.pi, dtype=float)


def draw_slope_guides(curves, ax):
    ref = curves[0]
    c = _read_at(ref.tau_s, ref.sigma_dps, 1.0, -0.5)
    guide = c * ref.tau_s**-0.5
    ax.loglog(ref.tau_s, guide, "--", color="0.6", lw=0.9)
    ax.annotate("-1/2  angle random walk", (ref.tau_s[2], guide[2]), color="0.4", fontsize=8)


def make_plot(path: Path, curves: list[AllanCurve], duration_s: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 6))
    for curve in curves:
        (line,) = ax.loglog(curve.tau_s, curve.sigma_dps, lw=1.5, label=curve.name)
        ax.plot(
            curve.tau_min_s,
            curve.sigma_dps.min(),
            "o",
            color=line.get_color(),
            ms=6,
            mfc="none",
        )

    draw_slope_guides(curves, ax)

    rise = next((c for c in curves if c.rrw_deg_per_s_sqrt_s is not None), None)
    if rise is not None:
        band = rising_band(rise.tau_s, rise.sigma_dps)
        assert band is not None
        k = _read_at(rise.tau_s, rise.sigma_dps, 3.0, +0.5, band)
        span = rise.tau_s[band]
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
    ax.set_ylim(bottom=min(c.sigma_dps.min() for c in curves) * 0.6)
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
    gyro_dps = log.gyro_rad_s * 180.0 / np.pi

    print(
        f"{args.log.name}: {len(log.t_boot_s):,} samples, {log.duration / 60:.1f} min, "
        f"dt {log.dt * 1000:.3f} ms"
    )
    print(
        f"  gyro range {log.header.gyro_range_dps:.0f} dps, "
        f"accel range {log.header.accel_range_g:.2f} g"
    )

    gaps = int(np.count_nonzero(np.diff(log.t_boot_s) > 5 * log.dt))
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
        rrw_hr = c.rrw_deg_hr_sqrt_hr
        rrw = "n/a" if rrw_hr is None else f"{rrw_hr:.1f}"
        print(
            f"{c.name:<6}{c.bias_dps:>10.4f}{c.arw_deg_sqrt_hr:>12.4f}"
            f"{c.bias_instability_dps:>12.5f}{c.tau_min_s:>10.1f}{rrw:>14}"
        )
    if any(c.rrw_deg_per_s_sqrt_s is None for c in curves):
        flat = ", ".join(c.name for c in curves if c.rrw_deg_per_s_sqrt_s is None)
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
    at_hold = _read_at(yaw.tau_s, yaw.sigma_dps, hold, -0.5)
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
    if yaw.rrw_deg_per_s_sqrt_s is not None:
        print(f"rrw_deg_per_s_sqrt_s = {yaw.rrw_deg_per_s_sqrt_s:.3e}")
    else:
        print("# rrw_deg_per_s_sqrt_s not measurable: the yaw curve never turned up")


if __name__ == "__main__":
    main()
