"""Put ESP32 events on the app clock and measure the delays around them."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

_NS = 1_000_000_000


@dataclass
class ClockSegment:
    """host_ns = host_ref_ns + offset_s * 1e9 + (1 + drift) * (robot_ms - robot_ref_ms) * 1e6"""

    first_index: int
    last_index_inclusive: int
    robot_ref_ms: int
    host_ref_ns: int
    offset_s: float
    drift: float
    residual_ms_percentiles: dict[str, float] = field(default_factory=dict)

    @property
    def drift_ppm(self) -> float:
        return self.drift * 1e6

    def to_host_ns(self, robot_ms: np.ndarray) -> np.ndarray:
        dt_s = (np.asarray(robot_ms, dtype=np.int64) - self.robot_ref_ms).astype(np.float64) / 1e3
        rel = self.offset_s + (1.0 + self.drift) * dt_s
        return self.host_ref_ns + np.round(rel * 1e9).astype(np.int64)


@dataclass
class ClockFit:
    segments: list[ClockSegment]
    app_clock_stamp_ns: np.ndarray
    receive_minus_mapped_ms: np.ndarray

    def summary(self) -> dict[str, object]:
        pct = _percentiles(self.receive_minus_mapped_ms)
        return {
            "segments": len(self.segments),
            "offset_s": [s.offset_s for s in self.segments],
            "drift_ppm": [s.drift_ppm for s in self.segments],
            "robot_ref_ms": [s.robot_ref_ms for s in self.segments],
            "host_ref_ns": [s.host_ref_ns for s in self.segments],
            "residual_ms_percentiles": pct,
        }


def _percentiles(values: np.ndarray) -> dict[str, float]:
    if values.size == 0:
        return {}
    return {f"p{p}": float(np.percentile(values, p)) for p in (1, 5, 25, 50, 75, 95, 99)} | {
        "max": float(np.max(values))
    }


def _lower_hull(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Indices of the lower convex hull of points sorted by x (Andrew's monotone chain)."""
    hull: list[int] = []
    for i in range(x.shape[0]):
        while len(hull) >= 2:
            o, a = hull[-2], hull[-1]
            cross = (x[a] - x[o]) * (y[i] - y[o]) - (y[a] - y[o]) * (x[i] - x[o])
            if cross <= 0.0:
                hull.pop()
            else:
                break
        hull.append(i)
    return np.asarray(hull, dtype=np.int64)


def _lower_envelope_line(x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    """(intercept, slope) of the line under every point that minimises the summed gap."""
    if x.shape[0] == 1:
        return float(y[0]), 0.0
    order = np.argsort(x, kind="stable")
    xs, ys = x[order], y[order]
    keep = np.ones(xs.shape[0], dtype=bool)
    keep[1:] = xs[1:] != xs[:-1]
    first = np.flatnonzero(keep)
    ys = np.minimum.reduceat(ys, first)
    xs = xs[first]
    if xs.shape[0] == 1:
        return float(ys[0]), 0.0
    hull = _lower_hull(xs, ys)
    x_mean = float(np.mean(x))
    k = int(np.searchsorted(xs[hull], x_mean, side="right")) - 1
    k = min(max(k, 0), hull.shape[0] - 2)
    i, j = hull[k], hull[k + 1]
    slope = float((ys[j] - ys[i]) / (xs[j] - xs[i]))
    return float(ys[i] - slope * xs[i]), slope


def split_reboots(robot_ms: np.ndarray, max_backstep_ms: int = 1000) -> list[tuple[int, int]]:
    """(first, last) index ranges of increasing robot time."""
    t = np.asarray(robot_ms, dtype=np.int64)
    if t.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(t) < -max_backstep_ms) + 1
    starts = np.concatenate([[0], breaks])
    ends = np.concatenate([breaks - 1, [t.size - 1]])
    return [(int(s), int(e)) for s, e in zip(starts, ends)]


def fit_robot_clock(timestamp_ms: np.ndarray, host_receive_ns: np.ndarray) -> ClockFit:
    """Map robot ``timestamp_ms`` onto the app clock, one lower-envelope line per boot."""
    robot = np.asarray(timestamp_ms, dtype=np.int64)
    host = np.asarray(host_receive_ns, dtype=np.int64)
    if robot.shape != host.shape:
        raise ValueError("timestamp_ms and host_receive_ns differ in length")
    stamp = np.zeros_like(host)
    segments = []
    for first, last in split_reboots(robot):
        r = robot[first : last + 1]
        h = host[first : last + 1]
        robot_ref = int(r[0])
        host_ref = int(h[0])
        x = (r - robot_ref).astype(np.float64) / 1e3
        y = (h - host_ref).astype(np.float64) / 1e9 - x
        intercept, slope = _lower_envelope_line(x, y)
        seg = ClockSegment(
            first_index=first,
            last_index_inclusive=last,
            robot_ref_ms=robot_ref,
            host_ref_ns=host_ref,
            offset_s=intercept,
            drift=slope,
        )
        mapped = seg.to_host_ns(r)
        stamp[first : last + 1] = mapped
        seg.residual_ms_percentiles = _percentiles((h - mapped).astype(np.float64) / 1e6)
        segments.append(seg)
    residual = (host - stamp).astype(np.float64) / 1e6
    return ClockFit(segments=segments, app_clock_stamp_ns=stamp, receive_minus_mapped_ms=residual)


@dataclass
class LagEstimate:
    b_lag_behind_a_s: float
    correlation: float
    sign: int
    samples: int
    curve_lags_s: np.ndarray
    curve: np.ndarray

    def as_dict(self) -> dict[str, float | int]:
        return {
            "lag_s": self.b_lag_behind_a_s,
            "lag_ms": self.b_lag_behind_a_s * 1e3,
            "correlation": self.correlation,
            "sign": self.sign,
            "samples": self.samples,
        }


def resample_hold(t_ns: np.ndarray, values: np.ndarray, grid_ns: np.ndarray) -> np.ndarray:
    """Zero-order hold of a signal logged on change; NaN before its first sample."""
    t = np.asarray(t_ns, dtype=np.int64)
    idx = np.searchsorted(t, grid_ns, side="right") - 1
    out = np.full(grid_ns.shape, np.nan)
    ok = idx >= 0
    out[ok] = np.asarray(values, dtype=np.float64)[idx[ok]]
    return out


def resample_linear(
    t_ns: np.ndarray, values: np.ndarray, grid_ns: np.ndarray, max_gap_s: float
) -> np.ndarray:
    """Linear interpolation that leaves NaN across gaps longer than ``max_gap_s``."""
    t = np.asarray(t_ns, dtype=np.int64)
    v = np.asarray(values, dtype=np.float64)
    ok = np.isfinite(v)
    t, v = t[ok], v[ok]
    out = np.full(grid_ns.shape, np.nan)
    if t.size < 2:
        return out
    ref = int(t[0])
    tf = (t - ref).astype(np.float64)
    gf = (grid_ns - ref).astype(np.float64)
    out = np.interp(gf, tf, v, left=np.nan, right=np.nan)
    idx = np.clip(np.searchsorted(tf, gf, side="right"), 1, tf.size - 1)
    gap = (tf[idx] - tf[idx - 1]) / 1e9
    out[gap > max_gap_s] = np.nan
    return np.asarray(out)


def cross_correlation_delay(
    grid_dt_s: float,
    a: np.ndarray,
    b: np.ndarray,
    max_lag_s: float,
    allow_sign_flip: bool = True,
) -> LagEstimate:
    """Lag of b behind a, two signals on the same uniform grid."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    max_lag = int(round(max_lag_s / grid_dt_s))
    lags = np.arange(-max_lag, max_lag + 1)
    curve = np.full(lags.shape, np.nan)
    counts = np.zeros(lags.shape, dtype=np.int64)
    n = a.shape[0]
    for i, lag in enumerate(lags):
        lo = max(0, -lag)
        hi = min(n, n - lag)
        if hi - lo < 10:
            continue
        aa = a[lo:hi]
        bb = b[lo + lag : hi + lag]
        ok = np.isfinite(aa) & np.isfinite(bb)
        if ok.sum() < 10:
            continue
        aa = aa[ok] - aa[ok].mean()
        bb = bb[ok] - bb[ok].mean()
        denom = math.sqrt(float(aa @ aa) * float(bb @ bb))
        if denom <= 0.0:
            continue
        curve[i] = float(aa @ bb) / denom
        counts[i] = int(ok.sum())
    if not np.any(np.isfinite(curve)):
        return LagEstimate(math.nan, math.nan, 1, 0, lags * grid_dt_s, curve)
    score = np.abs(curve) if allow_sign_flip else curve
    k = int(np.nanargmax(score))
    frac = 0.0
    if 0 < k < lags.size - 1 and np.isfinite(score[k - 1]) and np.isfinite(score[k + 1]):
        denom = score[k - 1] - 2.0 * score[k] + score[k + 1]
        if denom < 0.0:
            frac = 0.5 * (score[k - 1] - score[k + 1]) / denom
    return LagEstimate(
        b_lag_behind_a_s=float((lags[k] + frac) * grid_dt_s),
        correlation=float(curve[k]),
        sign=1 if curve[k] >= 0.0 else -1,
        samples=int(counts[k]),
        curve_lags_s=lags * grid_dt_s,
        curve=curve,
    )


def crsf_to_unit(channel: np.ndarray) -> np.ndarray:
    """CRSF channel units (172..1811, center 992) to [-1, 1]."""
    return (np.asarray(channel, dtype=np.float64) - 992.0) / 819.5


def radio_link_delay(
    stick_stamp_ns: np.ndarray,
    stick: np.ndarray,
    esp32_stamp_ns: np.ndarray,
    esp32_percent: np.ndarray,
    grid_dt_s: float = 0.002,
    max_lag_s: float = 0.25,
    smooth_s: float = 0.006,
) -> LagEstimate:
    """How long a stick move takes to show up in the ESP32's ``a_percent``/``b_percent``."""
    if len(stick_stamp_ns) < 2 or len(esp32_stamp_ns) < 2:
        return LagEstimate(math.nan, math.nan, 1, 0, np.zeros(0), np.zeros(0))
    start = max(int(stick_stamp_ns[0]), int(esp32_stamp_ns[0]))
    end = min(int(stick_stamp_ns[-1]), int(esp32_stamp_ns[-1]))
    step = int(round(grid_dt_s * _NS))
    if end - start < 20 * step:
        return LagEstimate(math.nan, math.nan, 1, 0, np.zeros(0), np.zeros(0))
    grid = np.arange(start, end, step, dtype=np.int64)
    a = _smooth(np.diff(resample_hold(stick_stamp_ns, stick, grid)), smooth_s / grid_dt_s)
    b = _smooth(np.diff(resample_hold(esp32_stamp_ns, esp32_percent, grid)), smooth_s / grid_dt_s)
    return cross_correlation_delay(grid_dt_s, a, b, max_lag_s)


def _smooth(x: np.ndarray, sigma_samples: float) -> np.ndarray:
    """Gaussian blur that treats NaN as zero."""
    if sigma_samples <= 0.0:
        return x
    half = int(math.ceil(3.0 * sigma_samples))
    k = np.exp(-0.5 * (np.arange(-half, half + 1) / sigma_samples) ** 2)
    k /= k.sum()
    filled = np.where(np.isfinite(x), x, 0.0)
    return np.convolve(filled, k, mode="same")
