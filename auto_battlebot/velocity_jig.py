"""Velocity jig log reading, calibration, and dead-reckoned ground truth.

The jig is an RP2040 datalogger that rides on the robot and records a wheel encoder plus an
ISM330DHCX IMU at 1 kHz (`firmware/velocity_jig`). It writes plain text with a comment header:

    # auto-battlebot velocity jig
    # columns: t_us,count,gx,gy,gz,ax,ay,az  (raw int16 imu)
    # encoder: quadrature x4 (constant column if unplugged)
    # sample_rate_hz=1000 gyro_dps_per_lsb=0.070 accel_g_per_lsb=0.000244

Everything here turns those raw counts into the ground truth the plant fit consumes: yaw rate
about the measured gravity axis, forward speed from encoder arc length with the encoder wheel's
lever arm removed, and a dead-reckoned pose. It also carries the two things a log alone cannot
tell you: which host clock a sample belongs to (`ClockFit`), and which run it came from
(`load_session_dir`, reading the sidecar TOML that `velocity_jig_drive.py` writes next to
every downloaded log).

Two failure modes get flagged rather than silently absorbed, because stage 2 shipped
parameters from silently degraded ground truth:

- Saturated gyro channels (`|raw| > 32000`), per axis, per run. This one disqualifies a run:
  a railed yaw axis makes the heading integral wrong from that sample on.
- Clipped accelerometer channels, past `ACCEL_SATURATION_G`. Reported, never gated. Nothing
  in the plant model reads acceleration, so a clip marks an impact worth looking at rather
  than data the fit would be wrong to use.
- Gyro bias drift between the pre-run and post-run still holds.

See `docs/experiments/kalman_filter/kalman_filter_plan.md` part 1.2 and the companion runbook.
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

G_MPS2 = 9.80665
DEG_TO_RAD = np.pi / 180.0

# int16 full scale is 32767. Anything at or past this is either clipped or one LSB away from it,
# and a clipped channel biases every fit downstream of it.
RAW_SATURATION = 32000

# Accelerometer gate, in g rather than raw counts. A raw threshold means a different physical
# acceleration on every range the firmware runs: 32000 counts is 7.81 g on the 8 g range and
# 15.62 g on the 16 g one, so the same number silently changed what the gate meant when the
# range went up. Anything past 15 g on this robot is an impact or a hard landing.
ACCEL_SATURATION_G = 15.0

AXIS_NAMES = ("x", "y", "z")

# What a run played, in the terms the fit routes on. Replaces the old runbook experiment
# ids: a waveform is declared in a TOML catalog, so the fit cannot key on a fixed list of
# experiments without a code change every time a new excitation is tried.
WAVEFORM_KINDS = (
    "step",
    "staircase",
    "coast",
    "grid",
    "prbs",
    "chirp",
    "sine",
    "triangle",
    "trim",
    "manual",
)
CHANNELS = ("linear", "angular", "combined")
# Whether a run trains the fit or validates it. Declared per waveform rather than derived,
# so a holdout stays a holdout even if someone later reuses the same excitation for fitting.
ROLES = ("fit", "holdout")

# Sidecar metadata format. Checked on load: reading a future file as if it were this
# version is how a silently wrong fit happens.
SIDECAR_SCHEMA = 1


# ---------------------------------------------------------------------------
# Log file
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JigHeader:
    """Scale factors the firmware wrote into the log header.

    Defaults match `firmware/velocity_jig/include/config.h` at the time of writing. A log that
    carries its own header always wins: the ranges change (2000 -> 4000 dps for the spin runs)
    and a stale default would rescale every sample by 2x without complaining.
    """

    sample_rate_hz: float = 1000.0
    gyro_dps_per_lsb: float = 0.070
    accel_g_per_lsb: float = 0.000244

    @property
    def gyro_range_dps(self) -> float:
        return self.gyro_dps_per_lsb * 32768.0

    @property
    def accel_range_g(self) -> float:
        return self.accel_g_per_lsb * 32768.0

    @property
    def accel_raw_limit(self) -> float:
        """Raw count past which an accel sample is not worth fitting through.

        `ACCEL_SATURATION_G` where the range can reach it, the rail otherwise. On the 8 g range
        15 g sits at 61475 counts, past int16, so a bare conversion would produce a gate that
        can never fire. Falling back to `RAW_SATURATION` keeps clip detection on those logs,
        which is the only thing an 8 g recording can tell you about a 15 g event anyway.
        """
        return min(ACCEL_SATURATION_G / self.accel_g_per_lsb, RAW_SATURATION)


@dataclass(frozen=True)
class AxisSaturation:
    """Saturation count for one IMU axis over one run."""

    name: str
    count: int
    total: int
    peak_raw: int

    @property
    def fraction(self) -> float:
        return self.count / self.total if self.total else 0.0


@dataclass
class JigLog:
    """One `LOG-N.TXT`, in physical units, on the jig's own microsecond clock."""

    path: Path
    header: JigHeader
    t: np.ndarray  # seconds since RP2040 boot
    count: np.ndarray  # quadrature counts, x4, int64
    gyro_raw: np.ndarray  # (N, 3) int16 as written
    accel_raw: np.ndarray  # (N, 3) int16 as written
    malformed_rows: int = 0
    # Of `malformed_rows`, how many were the truncated final line rather than damage inside
    # the stream. A reset during the last write leaves one; real corruption does not.
    malformed_tail: int = 0

    @property
    def gyro(self) -> np.ndarray:
        """(N, 3) rad/s in the IMU frame."""
        return self.gyro_raw.astype(float) * self.header.gyro_dps_per_lsb * DEG_TO_RAD

    @property
    def accel(self) -> np.ndarray:
        """(N, 3) m/s^2 specific force in the IMU frame (so +1 g on whichever axis points up)."""
        return self.accel_raw.astype(float) * self.header.accel_g_per_lsb * G_MPS2

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0]) if len(self.t) > 1 else 0.0

    @property
    def dt(self) -> float:
        """Median sample interval. Use this, not 1 / sample_rate_hz, since drops shift it."""
        return float(np.median(np.diff(self.t))) if len(self.t) > 1 else 0.0

    @property
    def encoder_moved(self) -> bool:
        """False for a detached encoder: the inputs are pulled up, so the count freezes."""
        return bool(len(self.count) and self.count.max() != self.count.min())

    def saturation(self, keep: np.ndarray | None = None) -> list[AxisSaturation]:
        """Per-axis saturation report, gyro first then accel.

        `keep` restricts the report to the samples that describe the plant. Handling the
        robot at an operator pause clips the accel for a sample or two, which says nothing
        about the run, so callers that know where the pauses were mask them out.
        """
        out: list[AxisSaturation] = []
        # The gyro keeps the raw rail test. Its ranges are chosen to sit just above the spin
        # rates being measured, so "near full scale" is the whole question there. The accel
        # runs wide of anything commanded, so what matters is the physical number.
        limits = (
            ("g", self.gyro_raw, RAW_SATURATION),
            ("a", self.accel_raw, self.header.accel_raw_limit),
        )
        for prefix, raw, limit in limits:
            for i, axis in enumerate(AXIS_NAMES):
                col = raw[:, i] if keep is None else raw[keep, i]
                over = int(np.count_nonzero(np.abs(col) > limit))
                peak = int(np.max(np.abs(col))) if len(col) else 0
                out.append(AxisSaturation(f"{prefix}{axis}", over, len(col), peak))
        return out

    def saturated_fraction(self, keep: np.ndarray | None = None) -> float:
        """Worst per-axis saturated fraction, the single number a run gate needs."""
        report = self.saturation(keep)
        return max((a.fraction for a in report), default=0.0)

    def slice_time(self, t0: float, t1: float) -> slice:
        """Index slice covering [t0, t1] in jig seconds."""
        lo = int(np.searchsorted(self.t, t0, side="left"))
        hi = int(np.searchsorted(self.t, t1, side="right"))
        return slice(lo, hi)


def _parse_header(lines: Iterable[str]) -> JigHeader:
    values: dict[str, float] = {}
    for line in lines:
        for token in line.lstrip("#").split():
            if "=" not in token:
                continue
            key, _, raw = token.partition("=")
            try:
                values[key] = float(raw)
            except ValueError:
                continue
    return JigHeader(
        sample_rate_hz=values.get("sample_rate_hz", JigHeader.sample_rate_hz),
        gyro_dps_per_lsb=values.get("gyro_dps_per_lsb", JigHeader.gyro_dps_per_lsb),
        accel_g_per_lsb=values.get("accel_g_per_lsb", JigHeader.accel_g_per_lsb),
    )


def read_jig_log(path: Path | str) -> JigLog:
    """Read one jig log into physical units.

    Rows that do not parse are counted and dropped rather than raising. A log whose tail was
    truncated by a power cut is still worth the 30 seconds before the cut, and the count shows up
    in the run report so a partially unreadable file cannot pass as a clean one.
    """
    path = Path(path)
    header_lines: list[str] = []
    rows: list[list[int]] = []
    malformed = 0
    # Line index of the last unparseable row, and of the last row seen at all. When they are
    # the same, the only bad row was the file's last one.
    last_bad = -1
    index = -1
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for index, raw in enumerate(handle):
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                header_lines.append(line)
                continue
            parts = line.split(",")
            if len(parts) != 8:
                malformed += 1
                last_bad = index
                continue
            try:
                rows.append([int(p) for p in parts])
            except ValueError:
                malformed += 1
                last_bad = index

    if not rows:
        raise ValueError(f"{path}: no sample rows")

    data = np.array(rows, dtype=np.int64)
    header = _parse_header(header_lines)
    # time_us_64() wraps after 584,000 years, so a decreasing timestamp means a corrupt row, not
    # a rollover. Keep the monotone prefix and count the rest as malformed.
    good = np.concatenate([[True], np.diff(data[:, 0]) > 0])
    interior_bad = int(np.count_nonzero(~good))
    malformed += interior_bad
    data = data[good]

    # A single bad row at the very end is a half-written line, not corruption: the board lost
    # power or reset between the write and the flush, and the card kept the sector with a NUL
    # tail. Everything before it parsed. Separating the two lets the run gate on real damage
    # without throwing away a complete recording over its last millisecond. LOG-119 was
    # discarded for exactly one such row out of 239,647.
    tail_only = malformed == 1 and interior_bad == 0 and last_bad == index

    return JigLog(
        path=path,
        header=header,
        t=data[:, 0].astype(float) * 1e-6,
        count=data[:, 1].copy(),
        gyro_raw=data[:, 2:5].copy(),
        accel_raw=data[:, 5:8].copy(),
        malformed_rows=malformed,
        malformed_tail=1 if tail_only else 0,
    )


# ---------------------------------------------------------------------------
# Still holds: bias, gravity axis, drift
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StillSegment:
    """A stationary stretch, from the 10 s holds that open and close every run card."""

    start: int
    stop: int
    t0: float
    t1: float

    @property
    def duration(self) -> float:
        return self.t1 - self.t0

    def as_slice(self) -> slice:
        return slice(self.start, self.stop)


@dataclass(frozen=True)
class StillStats:
    """What a still hold measures: gyro bias, and which way is up."""

    bias: np.ndarray  # (3,) rad/s in the IMU frame
    up: np.ndarray  # (3,) unit vector along measured gravity
    gyro_noise: np.ndarray  # (3,) rad/s standard deviation
    t_mid: float


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous [start, stop) index ranges where mask is True."""
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.concatenate([[idx[0]], idx[breaks + 1]])
    stops = np.concatenate([idx[breaks], [idx[-1]]])
    return list(zip(starts.tolist(), (stops + 1).tolist()))


def _bridge_blips(mask: np.ndarray, max_run: int) -> np.ndarray:
    """Fill False runs no longer than `max_run`, leaving longer ones alone.

    A held segment is one contiguous run, so without this a single sample over threshold
    splits a 5 s hold into two 2.5 s halves and neither clears `min_duration`. That is not a
    hypothetical: LOG-143's pre-hold peaked at 2.01 dps against the 2.0 bar with the encoder
    dead still and gravity flat to 0.5%, and it was thrown out, while LOG-115 peaked at 1.90
    and passed. The two are the same physical state.

    Real motion lasts far longer than this window. A blip is a footfall on the floor, a bump
    of the bench, or the sensor's own tail.
    """
    if max_run < 1 or mask.all() or not mask.any():
        return np.asarray(mask, dtype=bool)
    out = mask.copy()
    idx = np.flatnonzero(~mask)
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.concatenate([[idx[0]], idx[breaks + 1]])
    stops = np.concatenate([idx[breaks], [idx[-1]]]) + 1
    for a, b in zip(starts.tolist(), stops.tolist()):
        # Only interior runs. A leading or trailing violation is the boundary of the hold,
        # not a blip inside it, and bridging it would extend the segment into real motion.
        if b - a <= max_run and a > 0 and b < len(mask):
            out[a:b] = True
    return np.asarray(out, dtype=bool)


def find_still_segments(
    log: JigLog,
    min_duration: float = 5.0,
    gyro_thresh_dps: float = 2.0,
    accel_tol_g: float = 0.05,
    max_blip_s: float = 0.05,
) -> list[StillSegment]:
    """Find stationary stretches at least `min_duration` long.

    Stillness is tested three ways because each alone has a blind spot. The gyro catches
    rotation. The accelerometer magnitude near 1 g catches acceleration but says nothing about
    constant speed, so a robot cruising in a straight line looks perfectly still to it. The
    encoder closes that hole, which matters because the deadzone staircases spend their first
    several seconds barely moving and would otherwise get absorbed into the opening still hold.

    The thresholds are loose on purpose. A hold that is not actually still shows up later as bias
    disagreement between the pre and post holds.
    """
    gyro_dps = np.linalg.norm(log.gyro_raw.astype(float), axis=1) * log.header.gyro_dps_per_lsb
    accel_g = np.linalg.norm(log.accel_raw.astype(float), axis=1) * log.header.accel_g_per_lsb
    still = (gyro_dps < gyro_thresh_dps) & (np.abs(accel_g - 1.0) < accel_tol_g)
    if log.encoder_moved:
        half = max(1, int(round(0.025 / max(log.dt, 1e-9))))
        counts = log.count.astype(float)
        ahead = np.concatenate([counts[half:], np.full(half, counts[-1])])
        behind = np.concatenate([np.full(half, counts[0]), counts[:-half]])
        still &= np.abs(ahead - behind) <= 1.0

    still = _bridge_blips(still, max(1, int(round(max_blip_s / max(log.dt, 1e-9)))))

    out: list[StillSegment] = []
    for a, b in _runs(still):
        t0, t1 = float(log.t[a]), float(log.t[b - 1])
        if t1 - t0 >= min_duration:
            out.append(StillSegment(a, b, t0, t1))
    return out


def still_stats(log: JigLog, seg: StillSegment) -> StillStats:
    """Gyro bias and the gravity ("up") unit vector over one still hold.

    The yaw axis comes from gravity rather than from a note about how the jig is bolted on. The
    accelerometer measures specific force, so at rest it reads +1 g on whichever axis points up,
    and the ISM330DHCX puts gyro and accel on one right-handed triad. That makes positive yaw
    counter-clockwise viewed from above with no further assumption, and it survives a tilted
    remount that no single gyro axis would.
    """
    sl = seg.as_slice()
    gyro = log.gyro[sl]
    accel = log.accel[sl]
    up = accel.mean(axis=0)
    norm = float(np.linalg.norm(up))
    if norm < 1e-6:
        raise ValueError("still segment has no gravity vector")
    return StillStats(
        bias=gyro.mean(axis=0),
        up=up / norm,
        gyro_noise=gyro.std(axis=0),
        t_mid=0.5 * (seg.t0 + seg.t1),
    )


def bias_drift_dps(pre: StillStats, post: StillStats, up: np.ndarray | None = None) -> float:
    """Yaw-axis bias difference between the two holds, in deg/s.

    This is the run's in-run drift bound. The runbook discards a run above 0.05 deg/s, which over
    a 30 s run is 1.5 deg of accumulated heading error.
    """
    axis = pre.up if up is None else up
    return float(abs(np.dot(post.bias - pre.bias, axis)) / DEG_TO_RAD)


# ---------------------------------------------------------------------------
# Calibration constants (E3, E4, E5)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JigCalibration:
    """Bench calibration from runbook block 0, one set per hardware configuration.

    `meters_per_count` (E4) and `gyro_scale` (E3) set the units of every fitted parameter.
    `r_enc_perp` (E5) is the encoder wheel's lever arm, removed from every speed measurement,
    which is what makes combined linear-plus-angular runs usable instead of discarded. `r_imu` is
    kept for the report and for E5's own sanity check against a tape measure.
    """

    meters_per_count: float
    gyro_scale: float = 1.0
    r_enc_perp: float = 0.0  # m, encoder wheel offset perpendicular to the forward axis
    r_imu: float = 0.0  # m, IMU offset from the yaw axis
    encoder_rate_limit: float = 8.0  # rad/s, above which the encoder wheel scrubs (E5)
    source: str = ""

    @classmethod
    def from_toml(cls, path: Path | str) -> JigCalibration:
        path = Path(path)
        with open(path, "rb") as handle:
            data = tomllib.load(handle)
        enc = data.get("encoder", {})
        gyro = data.get("gyro", {})
        imu = data.get("imu", {})
        if "meters_per_count" not in enc:
            raise ValueError(f"{path}: [encoder] meters_per_count is required (E4)")
        if float(enc["meters_per_count"]) <= 0.0:
            # A zero scale makes every speed zero, and a fit over zero speed produces
            # parameters that look plausible and are entirely wrong. Refuse instead.
            raise ValueError(
                f"{path}: [encoder] meters_per_count is {enc['meters_per_count']}, which "
                "would make every measured speed zero. Measure it with runbook E4 and "
                "playground/calibration/fit_encoder_scale.py."
            )
        return cls(
            meters_per_count=float(enc["meters_per_count"]),
            gyro_scale=float(gyro.get("scale", 1.0)),
            r_enc_perp=float(enc.get("r_enc_perp", 0.0)),
            r_imu=float(imu.get("r_imu", 0.0)),
            encoder_rate_limit=float(enc.get("rate_limit", 8.0)),
            source=str(path),
        )


CALIBRATION_TEMPLATE = """\
# Velocity jig bench calibration. Fill in from runbook block 0 before fitting anything.
# docs/experiments/kalman_filter/velocity_jig_runbook.md

[encoder]
meters_per_count = 0.0    # E4, least squares through the origin over the 3.000 m passes
r_enc_perp = 0.0          # E5, encoder rate vs gyro rate slope during pure spins, meters
rate_limit = 8.0          # E5, yaw rate above which the wheel scrubs, rad/s

[gyro]
scale = 1.0               # E3, integrated gyro angle vs 3600 deg

[imu]
r_imu = 0.0               # E5, lateral accel vs yaw rate squared slope, meters
"""


# ---------------------------------------------------------------------------
# Clock alignment (E1)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ClockProbe:
    """One `TIME` probe burst, as `velocity_jig_drive.py` records it.

    The probe sends `TIME` many times, keeps the fastest decile by round trip, and reports
    the median host-minus-jig offset with the RMS about that median as `residual_ms`. Only
    the fastest probes are kept because USB Full Speed polls in 1 ms frames: most of the
    spread is host scheduling, and the fastest round trips are the ones whose transport was
    most nearly symmetric.
    """

    offset_ms: float  # host minus jig
    residual_ms: float
    at_host_ms: float
    at_jig_ms: float
    kept: int = 0
    total: int = 0

    @classmethod
    def from_toml(cls, data: dict[str, Any] | None) -> ClockProbe | None:
        if not data:
            return None
        return cls(
            offset_ms=float(data["offset_ms"]),
            residual_ms=float(data.get("residual_ms", 0.0)),
            at_host_ms=float(data.get("at_host_ms", 0.0)),
            at_jig_ms=float(data.get("at_jig_ms", 0.0)),
            kept=int(data.get("kept", 0)),
            total=int(data.get("total", 0)),
        )

    def to_toml_table(self) -> dict[str, Any]:
        return {
            "offset_ms": self.offset_ms,
            "residual_ms": self.residual_ms,
            "at_host_ms": self.at_host_ms,
            "at_jig_ms": self.at_jig_ms,
            "kept": self.kept,
            "total": self.total,
        }


@dataclass(frozen=True)
class ClockFit:
    """Jig microseconds to host seconds, as offset plus linear skew.

    Cross-correlating command edges against measured acceleration only ever recovers
    `clock_offset + transport_delay`. Measuring the offset separately is what makes the fitted
    transport delay a real number instead of a number plus however far the clocks are apart.
    """

    offset_ms: float
    skew_ppm: float = 0.0
    ref_jig_ms: float = 0.0
    residual_ms: float = 0.0
    measured: bool = True

    @classmethod
    def identity(cls) -> ClockFit:
        """No probe available. Jig time is used as host time, so the fitted delay absorbs the
        offset. Callers must report this: a delay fit on an identity clock is not a measurement
        of transport delay."""
        return cls(offset_ms=0.0, measured=False)

    @classmethod
    def from_probes(cls, pre: ClockProbe | None, post: ClockProbe | None) -> ClockFit:
        if pre is None and post is None:
            return cls.identity()
        if pre is None or post is None:
            probe = pre or post
            assert probe is not None
            return cls(
                offset_ms=probe.offset_ms,
                ref_jig_ms=probe.at_jig_ms,
                residual_ms=probe.residual_ms,
            )
        span = post.at_jig_ms - pre.at_jig_ms
        skew = (post.offset_ms - pre.offset_ms) / span * 1e6 if span > 0 else 0.0
        return cls(
            offset_ms=pre.offset_ms,
            skew_ppm=skew,
            ref_jig_ms=pre.at_jig_ms,
            residual_ms=max(pre.residual_ms, post.residual_ms),
        )

    def host_seconds(self, t_jig_s: np.ndarray | float) -> np.ndarray:
        """Map jig seconds to host seconds. Host time is what the command log is stamped in."""
        jig_ms = np.asarray(t_jig_s, dtype=float) * 1e3
        offset = self.offset_ms + self.skew_ppm * 1e-6 * (jig_ms - self.ref_jig_ms)
        return (jig_ms + offset) * 1e-3

    def jig_seconds(self, t_host_s: np.ndarray | float) -> np.ndarray:
        """Map host seconds back to jig seconds, to place a host-time window on log samples."""
        host_ms = np.asarray(t_host_s, dtype=float) * 1e3
        rate = 1.0 + self.skew_ppm * 1e-6
        jig_ms = (host_ms - self.offset_ms + self.skew_ppm * 1e-6 * self.ref_jig_ms) / rate
        return jig_ms * 1e-3


# ---------------------------------------------------------------------------
# Session directory (sidecar TOML written by velocity_jig_drive.py)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ProtocolSegment:
    """One held command cell of a piecewise program, in program-relative seconds."""

    t0: float
    t1: float
    linear: float
    angular: float
    label: str

    @classmethod
    def from_toml(cls, data: dict[str, Any]) -> ProtocolSegment:
        return cls(
            t0=float(data["t0"]),
            t1=float(data["t1"]),
            linear=float(data.get("linear", 0.0)),
            angular=float(data.get("angular", 0.0)),
            label=str(data.get("label", "")),
        )


@dataclass(frozen=True)
class PauseWindow:
    """One operator stop, in host seconds.

    The robot is picked up, walked back to its mark and set down inside this window, so the
    motion there is the operator's rather than the plant's. Every gate and every duration
    that describes the run has to skip it: setting the robot down clips the accelerometer,
    and counting the wait as commanded time makes a 17 s excitation look like 54 s.
    """

    t_start: float
    t_end: float
    program_t: float = 0.0

    @property
    def held_s(self) -> float:
        return self.t_end - self.t_start

    @classmethod
    def from_toml(cls, data: dict[str, Any]) -> PauseWindow | None:
        """None for a sidecar written before host windows were recorded."""
        if "t_start" not in data or "t_end" not in data:
            return None
        return cls(
            t_start=float(data["t_start"]),
            t_end=float(data["t_end"]),
            program_t=float(data.get("program_t", 0.0)),
        )


def pause_windows_from_commands(
    cmd_t: np.ndarray, program_t: Sequence[float] = (), min_gap_s: float = 0.5
) -> list[PauseWindow]:
    """Recover pause windows from a gap in the command stream.

    The runner stops sending while the operator resets the robot, so a pause is a hole in
    the command timestamps an order of magnitude wider than the 20 ms tick. This is the only
    way to place the pauses of a run captured before the windows were written out, and it
    agrees with the recorded hold times to within a tick.
    """
    if len(cmd_t) < 2:
        return []
    gaps = np.flatnonzero(np.diff(np.asarray(cmd_t, dtype=float)) > min_gap_s)
    out = []
    for i, k in enumerate(gaps):
        at = float(program_t[i]) if i < len(program_t) else 0.0
        out.append(PauseWindow(float(cmd_t[k]), float(cmd_t[k + 1]), at))
    return out


def pause_mask(t_host: np.ndarray, pauses: Sequence[PauseWindow]) -> np.ndarray:
    """True where the sample is inside an operator pause."""
    t_host = np.asarray(t_host, dtype=float)
    out = np.zeros(t_host.shape, dtype=bool)
    for p in pauses:
        out |= (t_host >= p.t_start) & (t_host <= p.t_end)
    return out


def overlaps_pause(window: np.ndarray, pauses: Sequence[PauseWindow]) -> bool:
    """True when a host-time [start, end] window touches any pause."""
    t0, t1 = float(window[0]), float(window[-1])
    return any(p.t_start <= t1 and t0 <= p.t_end for p in pauses)


@dataclass(frozen=True)
class WaveformSpec:
    """What a run played, in the terms the fit routes on.

    This replaces the runbook experiment id. A fit that keys on "E8" needs editing every
    time a new excitation is worth trying; a fit that keys on "a step on the linear
    channel" does not. `params` is the catalog entry verbatim, which is what lets the
    report say that an amplitude was declared but never actually reached.
    """

    name: str
    kind: str
    channel: str
    role: str = "fit"
    label: str = ""
    params: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_toml_table(cls, data: dict[str, Any]) -> WaveformSpec:
        kind = str(data.get("kind", "")).strip()
        channel = str(data.get("channel", "")).strip()
        role = str(data.get("role", "fit")).strip()
        if kind not in WAVEFORM_KINDS:
            raise ValueError(f"unknown waveform kind {kind!r}, expected one of {WAVEFORM_KINDS}")
        if channel not in CHANNELS:
            raise ValueError(f"unknown channel {channel!r}, expected one of {CHANNELS}")
        if role not in ROLES:
            raise ValueError(f"unknown role {role!r}, expected one of {ROLES}")
        return cls(
            name=str(data.get("name", "")),
            kind=kind,
            channel=channel,
            role=role,
            label=str(data.get("label", "")),
            params=dict(data.get("params", {})),
        )

    def matches(
        self,
        *,
        kinds: Sequence[str] | None = None,
        channels: Sequence[str] | None = None,
        roles: Sequence[str] | None = None,
    ) -> bool:
        """None means any. Channel matching is exact on purpose.

        A combined grid run is not a linear run. Letting one through would feed coupled
        segments to the linear deadzone fit, which reads the first command level that
        produces motion and would find it early because the angular channel was also
        pushing.
        """
        if kinds is not None and self.kind not in kinds:
            return False
        if channels is not None and self.channel not in channels:
            return False
        if roles is not None and self.role not in roles:
            return False
        return True

    @property
    def amplitudes(self) -> tuple[float, ...]:
        """Declared amplitude set, for the coverage view. Empty when the kind has none."""
        for key in ("amplitudes", "linear", "angular"):
            value = self.params.get(key)
            if isinstance(value, (list, tuple)) and value:
                return tuple(float(v) for v in value)
        value = self.params.get("amplitude")
        return (float(value),) if isinstance(value, (int, float)) else ()

    @property
    def band_hz(self) -> tuple[float, float] | None:
        """Excitation band, for comparing command energy against the plant corner.

        A step battery has almost no energy above 1/(2*hold), and the plant corner sits
        near 2.7 Hz, so this is usually the number that explains a weak time constant.
        """
        p = self.params
        if self.kind == "chirp":
            return float(p.get("f0_hz", 0.2)), float(p.get("f1_hz", 8.0))
        if self.kind in ("sine", "triangle"):
            f = float(p.get("freq_hz", 1.0))
            return f, f
        if self.kind == "prbs":
            bit_s = max(float(p.get("bit_ms", 60.0)) * 1e-3, 1e-6)
            duration = max(float(p.get("duration_s", 30.0)), bit_s)
            return 1.0 / duration, 1.0 / (2.0 * bit_s)
        if self.kind in ("step", "staircase", "coast", "grid"):
            hold = max(float(p.get("hold_s", 2.0)), 1e-6)
            return 0.0, 1.0 / (2.0 * hold)
        return None


@dataclass
class RunRecord:
    """One run's sidecar: what was played, when, and whether it cleared the gates."""

    run_id: str
    session_id: str
    spec: WaveformSpec
    rep: int
    encoder: str  # "attached", "detached", or "either"
    log_file: str | None
    command_file: str | None
    verdict: str | None
    # Why the capture-time gates rejected it, when they did. Empty on a pass.
    gate_note: str
    notes: str
    trim: float | None
    samples: int | None
    dropped: int | None
    clock_pre: ClockProbe | None
    clock_post: ClockProbe | None
    skew_ppm: float | None
    hold_pre: tuple[float, float] | None  # host seconds
    hold_post: tuple[float, float] | None
    segments: list[ProtocolSegment]
    # Operator stops, in host seconds. Empty for a straight-through run, and derived from
    # the command stream for a sidecar written before the windows themselves were recorded.
    pauses: list[PauseWindow]
    cmd_t: np.ndarray  # host seconds
    cmd_lin: np.ndarray
    cmd_ang: np.ndarray
    # Where the command came from. "measured" is the radio's own mixer output read back
    # over the trainer link, which is what the robot actually received; "commanded" is what
    # the CLI asked for. They differ when the driver's stick was not centered, and for a
    # hand-driven run only the measured stream exists at all.
    command_source: str = "commanded"
    cmd_lin_requested: np.ndarray | None = None
    cmd_ang_requested: np.ndarray | None = None
    # The two drive channels exactly as the radio reported them. This is the measurement;
    # cmd_lin/cmd_ang are these read through a mix, so a mix set wrong can be corrected from
    # these without re-recording.
    meas_ch_a: np.ndarray | None = None
    meas_ch_b: np.ndarray | None = None
    provenance: dict[str, Any] = field(default_factory=dict)
    # Operator waiver for the still-hold requirement, set per run in the sidecar. See
    # `RunQuality.still_holds_waived` for what it costs.
    waive_still_holds: bool = False

    @property
    def clock(self) -> ClockFit:
        return ClockFit.from_probes(self.clock_pre, self.clock_post)

    @property
    def has_commands(self) -> bool:
        return len(self.cmd_t) > 1

    @property
    def waveform(self) -> str:
        return self.spec.name

    @property
    def kind(self) -> str:
        return self.spec.kind

    @property
    def channel(self) -> str:
        return self.spec.channel

    @property
    def role(self) -> str:
        return self.spec.role


@dataclass
class Session:
    """One capture directory: the only metadata a `LOG-N.TXT` has."""

    path: Path
    session_id: str
    name: str = ""
    started_utc: str = ""
    robot: str = ""
    operator: str = ""
    floor_surface: str = ""
    guard_plates_on: bool = False
    weapon_disabled: bool = False
    encoder_rate_limit: float | None = None
    provenance: dict[str, Any] = field(default_factory=dict)
    runs: list[RunRecord] = field(default_factory=list)
    # Logs the drive CLI downloaded but never annotated, usually because it crashed or was
    # interrupted between the download and the sidecar write. Reported rather than dropped:
    # the data is recoverable by hand and a silently vanishing run is how a session ends up
    # short without anyone noticing.
    orphans: list[str] = field(default_factory=list)

    def select(
        self,
        *,
        kinds: Sequence[str] | None = None,
        channels: Sequence[str] | None = None,
        roles: Sequence[str] | None = None,
        passing_only: bool = False,
    ) -> list[RunRecord]:
        out = [r for r in self.runs if r.spec.matches(kinds=kinds, channels=channels, roles=roles)]
        if passing_only:
            out = [r for r in out if r.verdict != "discard"]
        return out


def _read_command_rows(path: Path) -> tuple[list[str], list[list[float]]]:
    """Column names from the `# columns:` header, and every well-formed numeric row.

    Rows whose width or contents do not match the header are skipped rather than raised on:
    a run killed mid-write leaves a torn final line, and that should not lose the run.
    """
    names: list[str] = []
    rows: list[list[float]] = []
    with open(path, "r", encoding="utf-8") as handle:
        for raw in handle:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                if "columns:" in line:
                    names = [c.strip() for c in line.split("columns:", 1)[1].split(",")]
                continue
            if not names:
                raise ValueError(f"{path}: rows before the '# columns:' header")
            parts = line.split(",")
            if len(parts) != len(names):
                continue
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                continue
    return names, rows


def read_command_log(path: Path | str) -> dict[str, np.ndarray]:
    """Read a `LOG-N.cmd.csv` written by the drive CLI.

    Columns are named in a `# columns:` comment so the file can grow without breaking
    older readers. `t_host_s` shares its origin with the clock probes' `at_host_ms`, which
    is the identity `ClockFit` rests on.
    """
    path = Path(path)
    names, rows = _read_command_rows(path)

    if not rows:
        raise ValueError(f"{path}: no command rows")
    data = np.array(rows, dtype=float)
    out = {name: data[:, i] for i, name in enumerate(names)}

    # A host clock can step backwards; jig microseconds cannot. A stepped command log
    # misaligns every window in the run, and it does so quietly, so refuse it here.
    t = out.get("t_host_s")
    if t is None:
        raise ValueError(f"{path}: no t_host_s column")
    if len(t) > 1 and np.any(np.diff(t) < 0.0):
        raise ValueError(f"{path}: t_host_s is not monotone; the host clock stepped mid-run")
    return out


def _hold_window(data: dict[str, Any] | None) -> tuple[float, float] | None:
    if not data or "start" not in data or "end" not in data:
        return None
    return float(data["start"]), float(data["end"])


def _pause_windows(entries: list[dict[str, Any]], cmd_t: np.ndarray) -> list[PauseWindow]:
    """Pause windows from the sidecar, or reconstructed from the command stream.

    Sidecars written before the host windows were recorded carry only the program time and
    the seconds held, neither of which places the pause on the log. The command gaps do, and
    they are the same measurement the runner would have written.
    """
    if not entries:
        return []
    parsed = [PauseWindow.from_toml(entry) for entry in entries]
    if all(p is not None for p in parsed):
        return [p for p in parsed if p is not None]
    return pause_windows_from_commands(cmd_t, [float(e.get("program_t", 0.0)) for e in entries])


def _log_index(name: str) -> tuple[int, str]:
    """Sort key that puts LOG-2 before LOG-10, unlike a plain string sort."""
    match = re.search(r"(\d+)", name)
    return (int(match.group(1)) if match else 1 << 30, name)


def load_run_record(
    toml_path: Path | str,
    session: Session | None = None,
    *,
    commands: str = "measured",
) -> RunRecord:
    """Read one sidecar TOML and its command CSV.

    `commands` picks which stream the fit sees. "measured" is the radio's own report, which
    is what the robot actually received and the right default. "requested" is what the tool
    asked for, and is the escape hatch for a run whose readback was mis-decoded: a wrong mix
    or channel sign corrupts the measured stream while leaving the requested one exact.
    """
    toml_path = Path(toml_path)
    with open(toml_path, "rb") as handle:
        data = tomllib.load(handle)

    schema = int(data.get("schema", 0))
    if schema != SIDECAR_SCHEMA:
        raise ValueError(
            f"{toml_path}: sidecar schema {schema}, this build reads {SIDECAR_SCHEMA}. "
            "Reading it anyway would risk a silently wrong fit."
        )

    run = data.get("run", {})
    spec_table = dict(data.get("waveform", {}))
    spec_table.setdefault("name", toml_path.stem)
    spec = WaveformSpec.from_toml_table(spec_table)

    transfer = data.get("transfer", {})
    clock = data.get("clock", {})
    hold = data.get("hold", {})

    cmd_t = np.zeros(0)
    cmd_lin = np.zeros(0)
    cmd_ang = np.zeros(0)
    cmd_lin_req: np.ndarray | None = None
    cmd_ang_req: np.ndarray | None = None
    ch_a: np.ndarray | None = None
    ch_b: np.ndarray | None = None
    source = "commanded"
    command_file = run.get("command_file")
    if command_file:
        cols = read_command_log(toml_path.parent / str(command_file))
        cmd_t = cols["t_host_s"]
        ch_a = cols.get("meas_ch_a")
        ch_b = cols.get("meas_ch_b")
        # Prefer what the radio reported sending. Trainer mode adds the human's sticks to
        # the scripted command, so the mixer output is the only record of what the robot
        # actually received, and for a hand-driven run it is the only command signal there is.
        use_measured = commands != "requested"
        if use_measured and "meas_linear" in cols and np.any(np.isfinite(cols["meas_linear"])):
            cmd_lin = cols["meas_linear"]
            cmd_ang = cols.get("meas_angular", np.zeros_like(cmd_t))
            cmd_lin_req = cols.get("linear")
            cmd_ang_req = cols.get("angular")
            source = "measured"
        else:
            cmd_lin = cols.get("linear", np.zeros_like(cmd_t))
            cmd_ang = cols.get("angular", np.zeros_like(cmd_t))

    return RunRecord(
        run_id=str(run.get("id", toml_path.stem)),
        session_id=str(data.get("session", {}).get("id", session.session_id if session else "")),
        spec=spec,
        rep=int(run.get("rep", 0)),
        encoder=str(run.get("encoder", "either")),
        log_file=run.get("log_file"),
        command_file=command_file,
        verdict=run.get("verdict"),
        gate_note=str(run.get("gates", "")),
        notes=str(run.get("notes", "")),
        trim=(None if run.get("trim") is None else float(run["trim"])),
        samples=(None if transfer.get("samples") is None else int(transfer["samples"])),
        dropped=(None if transfer.get("dropped") is None else int(transfer["dropped"])),
        clock_pre=ClockProbe.from_toml(clock.get("pre")),
        clock_post=ClockProbe.from_toml(clock.get("post")),
        skew_ppm=(None if clock.get("skew_ppm") is None else float(clock["skew_ppm"])),
        hold_pre=_hold_window(hold.get("pre")),
        hold_post=_hold_window(hold.get("post")),
        waive_still_holds=bool(run.get("waive_still_holds", False)),
        segments=[ProtocolSegment.from_toml(seg) for seg in data.get("segment", [])],
        pauses=_pause_windows(data.get("pause", []), cmd_t),
        cmd_t=cmd_t,
        cmd_lin=cmd_lin,
        cmd_ang=cmd_ang,
        command_source=source,
        cmd_lin_requested=cmd_lin_req,
        cmd_ang_requested=cmd_ang_req,
        meas_ch_a=ch_a,
        meas_ch_b=ch_b,
        provenance=dict(data.get("provenance", {})),
    )


def load_session_dir(path: Path | str, *, commands: str = "measured") -> Session:
    """Load a capture directory written by `velocity_jig_drive.py`.

    Reads `session.toml` when present, then every `LOG-*.toml` beside it. A log with no
    sidecar goes to `Session.orphans` rather than being skipped silently.
    """
    path = Path(path)
    if not path.is_dir():
        raise ValueError(f"{path}: not a session directory")

    meta: dict[str, Any] = {}
    session_toml = path / "session.toml"
    if session_toml.exists():
        with open(session_toml, "rb") as handle:
            meta = tomllib.load(handle)
        schema = int(meta.get("schema", 0))
        if schema != SIDECAR_SCHEMA:
            raise ValueError(f"{session_toml}: schema {schema}, this build reads {SIDECAR_SCHEMA}")

    info = meta.get("session", {})
    session = Session(
        path=path,
        session_id=str(info.get("id", path.name)),
        name=str(info.get("name", "")),
        started_utc=str(info.get("started_utc", "")),
        robot=str(info.get("robot", "")),
        operator=str(info.get("operator", "")),
        floor_surface=str(info.get("floor_surface", "")),
        guard_plates_on=bool(info.get("guard_plates_on", False)),
        weapon_disabled=bool(info.get("weapon_disabled", False)),
        encoder_rate_limit=(
            None if info.get("encoder_rate_limit") is None else float(info["encoder_rate_limit"])
        ),
        provenance=dict(meta.get("provenance", {})),
    )

    sidecars = sorted(
        (p for p in path.glob("*.toml") if p.name != "session.toml"),
        key=lambda p: _log_index(p.name),
    )
    annotated: set[str] = set()
    for sidecar in sidecars:
        record = load_run_record(sidecar, session, commands=commands)
        session.runs.append(record)
        if record.log_file:
            annotated.add(record.log_file)

    for log in sorted(path.glob("LOG-*.TXT"), key=lambda p: _log_index(p.name)):
        if log.name not in annotated:
            session.orphans.append(log.name)

    return session


def load_sessions(paths: Sequence[Path | str], *, commands: str = "measured") -> list[Session]:
    return [load_session_dir(p, commands=commands) for p in paths]


# ---------------------------------------------------------------------------
# Ground truth
# ---------------------------------------------------------------------------


def _moving_average(a: np.ndarray, window: int) -> np.ndarray:
    if window < 2 or len(a) < window:
        return a
    kernel = np.ones(window) / window
    # Edge-pad so the ends are not pulled toward zero, which would fake a decel at both ends of
    # every run.
    pad = window // 2
    padded = np.concatenate([np.full(pad, a[0]), a, np.full(window - 1 - pad, a[-1])])
    return np.convolve(padded, kernel, mode="valid")


@dataclass
class GroundTruth:
    """Dead-reckoned truth for one run, on the jig's 1 kHz sample grid.

    Heading drift dominates position drift, so this is trustworthy over the 500 ms windows the
    plant fit uses and not over a whole 30 s run. Validation is windowed for exactly that reason.
    """

    t: np.ndarray  # host seconds if a ClockFit was supplied, else jig seconds
    v: np.ndarray  # body forward speed, m/s
    v_raw: np.ndarray  # forward speed with only light smoothing, for onset timing
    w: np.ndarray  # yaw rate, rad/s
    theta: np.ndarray  # heading, rad, unwrapped
    x: np.ndarray
    y: np.ndarray
    encoder_valid: bool
    bias_drift_dps: float
    yaw_scale: float
    clock: ClockFit

    def closure_error(self) -> tuple[float, float]:
        """(position error, path length) between the run's start and end, both meters.

        The runbook excludes a run whose closure error exceeds 1% of path length. Only meaningful
        on E19 loops that were driven back to the same tape mark.
        """
        ds = np.abs(self.v[:-1]) * np.diff(self.t)
        path_len = float(np.sum(ds))
        err = float(np.hypot(self.x[-1] - self.x[0], self.y[-1] - self.y[0]))
        return err, path_len


def solve_ground_truth(
    log: JigLog,
    calib: JigCalibration,
    *,
    still_pre: StillStats | None = None,
    still_post: StillStats | None = None,
    clock: ClockFit | None = None,
    smooth_s: float = 0.02,
    onset_smooth_s: float = 0.005,
) -> GroundTruth:
    """Unicycle dead reckoning from encoder arc length and gyro yaw.

        w(t)     = (gyro(t) . up - bias . up) * gyro_scale
        theta(t) = theta_0 + integral w dt              (trapezoidal, at the log rate)
        ds(t)    = (count(t) - count(t-1)) * meters_per_count
        v(t)     = ds/dt - w(t) * r_enc_perp            (encoder lever arm removed)
        p(t)     = p_0 + integral [v cos(theta), v sin(theta)] dt

    Bias is taken from the pre-run still hold and, when a post-run hold is given, ramped linearly
    to it across the run. A linear ramp is the right shape for thermal drift and it removes the
    part of the heading error that a single-point bias leaves behind.

    Encoder speed is smoothed over `smooth_s` before use. Differentiating a quantized count at
    1 kHz otherwise produces a square wave whose amplitude is meters_per_count / dt, which is
    much larger than the speeds being fit.
    """
    n = len(log.t)
    up = still_pre.up if still_pre is not None else np.array([0.0, 0.0, 1.0])
    bias = still_pre.bias if still_pre is not None else np.zeros(3)

    gyro = log.gyro
    if still_pre is not None and still_post is not None:
        span = still_post.t_mid - still_pre.t_mid
        frac = np.clip((log.t - still_pre.t_mid) / span, 0.0, 1.0) if span > 1e-6 else np.zeros(n)
        bias_t = bias[None, :] + frac[:, None] * (still_post.bias - bias)[None, :]
        gyro = gyro - bias_t
    else:
        gyro = gyro - bias[None, :]

    w = (gyro @ up) * calib.gyro_scale

    dt = np.diff(log.t, prepend=log.t[0] - log.dt)
    theta = np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(log.t))])

    encoder_valid = log.encoder_moved
    window = max(1, int(round(smooth_s / max(log.dt, 1e-9))))
    onset_window = max(1, int(round(onset_smooth_s / max(log.dt, 1e-9))))
    v_raw = np.zeros(n)
    if encoder_valid:
        ds = np.diff(log.count.astype(float), prepend=float(log.count[0])) * calib.meters_per_count
        rate = ds / np.maximum(dt, 1e-9)
        v = _moving_average(rate, window) - w * calib.r_enc_perp
        # A second, barely smoothed copy for motion-onset timing. Smoothing a first-order rise
        # moves the peak of dv/dt later by about half the window, which would inflate the
        # transport delay by 10 ms at the 20 ms smoothing the rest of the fit needs. The light
        # window still divides the encoder's quantization noise by sqrt(5).
        v_raw = _moving_average(rate, onset_window) - w * calib.r_enc_perp
    else:
        # Detached encoder. Heading is still measured; forward speed is not, and the caller has
        # to treat this run as angular-only rather than as a run that stood still.
        v = np.zeros(n)

    x = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.cos(theta[:-1]) * dt[1:])])
    y = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.sin(theta[:-1]) * dt[1:])])

    drift = (
        bias_drift_dps(still_pre, still_post, up)
        if still_pre is not None and still_post is not None
        else float("nan")
    )
    clock = clock or ClockFit.identity()
    return GroundTruth(
        t=clock.host_seconds(log.t),
        v=v,
        v_raw=v_raw,
        w=w,
        theta=theta,
        x=x,
        y=y,
        encoder_valid=encoder_valid,
        bias_drift_dps=drift,
        yaw_scale=calib.gyro_scale,
        clock=clock,
    )


# ---------------------------------------------------------------------------
# Uniform-rate run assembly
# ---------------------------------------------------------------------------


@dataclass
class RunQuality:
    """Everything that decides whether a run is fit for fitting, in one place."""

    log_file: str
    session_id: str
    waveform: str
    kind: str
    channel: str
    role: str
    rep: int
    verdict: str | None
    dropped: int | None
    malformed_rows: int
    # Gyro only. A railed gyro axis means the yaw rate itself is wrong, and yaw rate is the
    # measurement every angular parameter and the whole dead-reckoned heading rest on.
    worst_saturation: float
    saturated_axes: list[str]
    bias_drift_dps: float
    clock_measured: bool
    clock_residual_ms: float
    skew_ppm: float | None
    still_holds: int
    encoder_valid: bool
    saturated_samples: int = 0
    # Of `malformed_rows`, how many were the truncated final line. A reset during the last
    # write leaves one; damage inside the stream does not.
    malformed_tail: int = 0
    # Set per run in the sidecar. The post hold exists to bracket gyro bias drift, and
    # without it `solve_ground_truth` holds the pre-hold bias constant instead of ramping it.
    # Measured on this board that is worth very little: rate random walk is 1.862e-4
    # deg/s/sqrt(s) (E2, LOG-97), so 60 s of run moves the bias about 1.4e-3 deg/s and leaves
    # roughly 0.04 deg of heading error. Even the worst drift seen across 33 two-hold runs,
    # 0.0155 deg/s, is only about 0.5 deg over 60 s. Waive it when a run is worth more than
    # that, and never on a run whose still holds failed because the robot was being handled.
    still_holds_waived: bool = False
    # Accel clipping, reported and never gated. The model has no acceleration state: the fit
    # reads speed from the encoder and yaw from the gyro, and the accelerometer is used only
    # to find the gravity axis during a still hold. A clipped sample mid-run says an impact
    # happened, which is worth seeing on the plot, but it does not corrupt anything fitted.
    accel_clip_axes: list[str] = field(default_factory=list)
    accel_clip_samples: int = 0
    accel_peak_g: float = 0.0
    # How much of the run the fit can actually use. Nothing computed this before, so a run
    # that was mostly uncommanded counted the same as a fully driven one when deciding
    # whether an experiment had enough data.
    duration_s: float = 0.0
    commanded_s: float = 0.0
    # Seconds of each commanded still hold that the log does not show as motionless. Large
    # means the jig and the CLI disagree about when the run started, which otherwise surfaces
    # only as a mysteriously bad fit.
    hold_pre_gap_s: float = 0.0
    hold_post_gap_s: float = 0.0
    command_source: str = "commanded"

    def problems(
        self,
        max_bias_drift_dps: float = 0.05,
        # Gyro only, and zero tolerance. A railed yaw axis is unrecoverable: the true rate is
        # unknown, so the heading integral past that sample is wrong and stays wrong.
        #
        # The accelerometer used to gate here too and no longer does. It cost more runs than
        # it saved, and it discarded them for the procedure rather than for the data: on
        # 2026-08-22 the clip that discarded LOG-108 landed 1.29 s before the first command
        # (unplugging the USB cable) and the one that discarded LOG-110 landed 18.8 s after
        # the last (picking the robot up to plug it back in). See `accel_clip_samples`.
        max_saturation: float = 0.0,
        max_clock_residual_ms: float = 2.0,
        min_commanded_s: float = 1.0,
        # Half a ten second hold. Detection trims the edges of a still stretch, so a little
        # uncovered time is normal; half of it missing is not.
        max_hold_gap_s: float = 5.0,
    ) -> list[str]:
        """Reasons to exclude this run. Empty means usable."""
        out: list[str] = []
        if self.verdict == "discard":
            out.append("capture-time verdict: discard")
        if self.dropped:
            out.append(f"dropped={self.dropped} samples on the SD path")
        # A truncated final row is a reset during the last write, not damage to the stream.
        interior = self.malformed_rows - self.malformed_tail
        if interior:
            out.append(f"{interior} malformed rows")
        if self.saturated_axes and self.worst_saturation > max_saturation:
            axes = ", ".join(self.saturated_axes)
            out.append(
                f"gyro saturation on {axes}: {self.saturated_samples} samples "
                f"({self.worst_saturation:.3%} of the worst axis)"
            )
        if np.isfinite(self.bias_drift_dps) and self.bias_drift_dps > max_bias_drift_dps:
            out.append(f"gyro bias drift {self.bias_drift_dps:.3f} deg/s")
        if self.clock_measured and self.clock_residual_ms > max_clock_residual_ms:
            out.append(f"clock residual {self.clock_residual_ms:.2f} ms")
        if self.still_holds < 2 and not self.still_holds_waived:
            out.append(f"{self.still_holds} still holds found, need 2")
        if self.commanded_s < min_commanded_s:
            out.append(f"only {self.commanded_s:.1f} s of commanded data")
        gap = max(self.hold_pre_gap_s, self.hold_post_gap_s)
        if gap > max_hold_gap_s and not self.still_holds_waived:
            out.append(
                f"{gap:.1f} s of a commanded still hold was not motionless: "
                "the jig and the drive CLI disagree on when the run started"
            )
        return out

    def notices(self) -> list[str]:
        """Things worth looking at that do not disqualify the run.

        Separate from `problems` on purpose. A reason to open the plot is not a reason to
        throw the recording away, and folding the two together is what made an unplugged
        cable look identical to a wall strike.
        """
        out: list[str] = []
        if self.still_holds_waived and (
            self.still_holds < 2 or max(self.hold_pre_gap_s, self.hold_post_gap_s) > 0.5
        ):
            gap = max(self.hold_pre_gap_s, self.hold_post_gap_s)
            out.append(
                f"still-hold check waived: {self.still_holds} hold(s) found, {gap:.1f} s of a "
                "commanded hold uncovered. Gyro bias is held constant instead of ramped"
            )
        if self.accel_clip_samples:
            axes = ", ".join(self.accel_clip_axes)
            out.append(
                f"accel clipped on {axes}: {self.accel_clip_samples} samples, "
                f"peak {self.accel_peak_g:.1f} g. Impact or handling, not a fit problem"
            )
        return out


@dataclass
class Run:
    """One run resampled onto a uniform grid, with commands aligned to it.

    Fits should not depend on the log rate, and the command log is asynchronous, so both get put
    on one grid here. Everything downstream (per-phase fits, window prediction, residuals) reads
    these arrays and nothing else.
    """

    record: RunRecord
    quality: RunQuality
    t: np.ndarray  # host seconds, uniform
    dt: float
    v: np.ndarray
    v_raw: np.ndarray  # lightly smoothed, for onset timing only
    w: np.ndarray
    theta: np.ndarray
    x: np.ndarray
    y: np.ndarray
    cmd_lin: np.ndarray
    cmd_ang: np.ndarray
    commanded: np.ndarray  # bool, the command is known here (see load_run)
    label: np.ndarray  # per-sample phase label from the protocol segments
    v_noise: float
    w_noise: float
    truth: GroundTruth

    @property
    def spec(self) -> WaveformSpec:
        return self.record.spec

    @property
    def waveform(self) -> str:
        return self.record.spec.name

    @property
    def kind(self) -> str:
        return self.record.spec.kind

    @property
    def channel(self) -> str:
        return self.record.spec.channel

    @property
    def role(self) -> str:
        return self.record.spec.role

    @property
    def name(self) -> str:
        # The session id is part of the name because two sessions both start at LOG-0, and
        # a report that lists them both would otherwise show two rows with one label.
        log = self.record.log_file or self.record.run_id
        return f"{self.record.session_id}/{log} [{self.waveform}#{self.record.rep}]"

    @property
    def encoder_valid(self) -> bool:
        return self.truth.encoder_valid

    def mask(self, prefix: str) -> np.ndarray:
        return np.array([str(lbl).startswith(prefix) for lbl in self.label])


def zoh(src_t: np.ndarray, src_v: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    """Zero-order-hold resample. Commands are piecewise constant, so this is exact for them.

    Outside the command log the value is zero, not the nearest command held forever. Before the
    first command the robot is idle, and after the last one the runner has zeroed the channels
    and disarmed. Holding the last command through the closing still hold would tell the fit the
    robot was being driven at full command while it sat on the floor.
    """
    if len(src_t) == 0:
        return np.zeros(len(dst_t))
    idx = np.clip(np.searchsorted(src_t, dst_t, side="right") - 1, 0, len(src_v) - 1)
    out = np.where((dst_t < src_t[0]) | (dst_t > src_t[-1]), 0.0, src_v[idx])
    return out


def _hold_uncovered_s(
    commanded: tuple[float, float] | None,
    stills: Sequence[StillSegment],
    clock: ClockFit,
) -> float:
    """Seconds of a commanded still hold that the log does not show as still.

    Overlap, not edge alignment. The jig starts recording when the operator presses A, which
    is before the CLI starts its hold countdown, so the detected still segment routinely
    begins earlier than the commanded one and ends later. An edge comparison reads that
    normal offset as a fault.

    What actually matters is whether the robot was motionless for the window the fit will
    take its gyro bias from. A commanded hold that the log shows as moving means the jig was
    not recording when the CLI thought it was, and every downstream alignment is off.
    """
    if commanded is None:
        return 0.0
    lo, hi = commanded
    span = max(hi - lo, 0.0)
    if span <= 0.0 or not stills:
        return span
    covered = 0.0
    for seg in stills:
        s0, s1 = clock.host_seconds(np.array([seg.t0, seg.t1]))
        covered += max(0.0, min(hi, float(s1)) - max(lo, float(s0)))
    return max(0.0, span - covered)


def _labels_for(record: RunRecord, t: np.ndarray) -> np.ndarray:
    """Per-sample phase label. Segments are program-relative; the first command anchors them."""
    if not record.has_commands:
        return np.array([record.spec.name] * len(t))
    t0 = float(record.cmd_t[0])
    if not record.segments:
        return np.array([record.spec.name] * len(t))
    rel = t - t0
    labels = np.array(["idle"] * len(t), dtype=object)
    for seg in record.segments:
        labels[(rel >= seg.t0) & (rel < seg.t1)] = seg.label
    return labels.astype(str)


def load_run(
    record: RunRecord,
    log_dir: Path | str,
    calib: JigCalibration,
    *,
    fit_hz: float = 200.0,
    smooth_s: float = 0.02,
) -> Run:
    """Read a run's log, solve ground truth, and resample it with its commands onto one grid.

    `fit_hz` of 200 keeps five samples per 10 ms and is far above the 2.7 Hz corner of the plant,
    while cutting the 1 kHz log down to something a windowed simulation fit can iterate over.
    """
    if not record.log_file:
        raise ValueError(f"run {record.run_id} has no log file recorded")
    log = read_jig_log(Path(log_dir) / record.log_file)

    clock = record.clock
    # Every stretch the operator spent handling the robot is dropped before anything is
    # measured. A pause is not a still hold, not commanded time, and not a place where a
    # clipped accelerometer says anything about the plant.
    log_pause = pause_mask(clock.host_seconds(log.t), record.pauses)
    stills = [
        seg
        for seg in find_still_segments(log)
        if not overlaps_pause(clock.host_seconds(np.array([seg.t0, seg.t1])), record.pauses)
    ]
    pre = still_stats(log, stills[0]) if stills else None
    post = still_stats(log, stills[-1]) if len(stills) > 1 else None
    truth = solve_ground_truth(
        log, calib, still_pre=pre, still_post=post, clock=clock, smooth_s=smooth_s
    )

    dt = 1.0 / fit_hz
    grid = np.arange(truth.t[0], truth.t[-1], dt)
    v = np.interp(grid, truth.t, truth.v)
    v_raw = np.interp(grid, truth.t, truth.v_raw)
    w = np.interp(grid, truth.t, truth.w)
    theta = np.interp(grid, truth.t, truth.theta)
    x = np.interp(grid, truth.t, truth.x)
    y = np.interp(grid, truth.t, truth.y)

    cmd_lin = zoh(record.cmd_t, record.cmd_lin, grid)
    cmd_ang = zoh(record.cmd_t, record.cmd_ang, grid)
    labels = _labels_for(record, grid)

    # Where the command is actually known. The still holds that bracket every run sit outside the
    # command log, and so does an operator-driven run whose sticks were never logged. Coasting
    # for a couple of seconds past the last command is real data (the runner zeroes and disarms
    # on exit), so that much grace is kept and the rest is dropped.
    if record.has_commands:
        commanded = (grid >= record.cmd_t[0]) & (grid <= record.cmd_t[-1] + 2.0)
        commanded &= ~pause_mask(grid, record.pauses)
    else:
        commanded = np.zeros(len(grid), dtype=bool)

    # Noise floors come from the still holds, which are the only stretches known to be motionless.
    idle = np.zeros(len(grid), dtype=bool)
    for seg in stills:
        t0, t1 = clock.host_seconds(np.array([seg.t0, seg.t1]))
        idle |= (grid >= t0) & (grid <= t1)
    v_noise = float(np.std(v[idle])) if np.count_nonzero(idle) > 10 else 0.0
    w_noise = float(np.std(w[idle])) if np.count_nonzero(idle) > 10 else 0.0

    # Usable seconds, which is what the report ranks waveforms by when deciding what to
    # collect more of. A run that was mostly still holds contributes far less than its length.
    commanded_s = float(dt * np.count_nonzero(commanded))
    hold_pre_gap = _hold_uncovered_s(record.hold_pre, stills, clock)
    hold_post_gap = _hold_uncovered_s(record.hold_post, stills, clock)

    # Split by sensor: the gyro half gates the run, the accel half is only reported.
    sat = log.saturation(keep=~log_pause)
    gyro_sat = [a for a in sat if a.name.startswith("g")]
    accel_sat = [a for a in sat if a.name.startswith("a")]
    accel_peak_g = max(
        (a.peak_raw * log.header.accel_g_per_lsb for a in accel_sat if a.count), default=0.0
    )
    quality = RunQuality(
        log_file=record.log_file,
        session_id=record.session_id,
        waveform=record.spec.name,
        kind=record.spec.kind,
        channel=record.spec.channel,
        role=record.spec.role,
        rep=record.rep,
        verdict=record.verdict,
        dropped=record.dropped,
        malformed_rows=log.malformed_rows,
        malformed_tail=log.malformed_tail,
        still_holds_waived=record.waive_still_holds,
        worst_saturation=max((a.fraction for a in gyro_sat), default=0.0),
        saturated_axes=[a.name for a in gyro_sat if a.count],
        saturated_samples=sum(a.count for a in gyro_sat),
        accel_clip_axes=[a.name for a in accel_sat if a.count],
        accel_clip_samples=sum(a.count for a in accel_sat),
        accel_peak_g=accel_peak_g,
        bias_drift_dps=truth.bias_drift_dps,
        clock_measured=clock.measured,
        clock_residual_ms=clock.residual_ms,
        skew_ppm=record.skew_ppm,
        still_holds=len(stills),
        encoder_valid=truth.encoder_valid,
        duration_s=float(grid[-1] - grid[0]) if len(grid) > 1 else 0.0,
        commanded_s=commanded_s,
        hold_pre_gap_s=hold_pre_gap,
        hold_post_gap_s=hold_post_gap,
        command_source=record.command_source,
    )

    return Run(
        record=record,
        quality=quality,
        t=grid,
        dt=dt,
        v=v,
        v_raw=v_raw,
        w=w,
        theta=theta,
        x=x,
        y=y,
        cmd_lin=cmd_lin,
        cmd_ang=cmd_ang,
        commanded=commanded,
        label=labels,
        v_noise=v_noise,
        w_noise=w_noise,
        truth=truth,
    )


def load_runs(
    session: Session,
    calib: JigCalibration,
    *,
    log_dir: Path | str | None = None,
    kinds: Sequence[str] | None = None,
    channels: Sequence[str] | None = None,
    roles: Sequence[str] | None = None,
    include_discarded: bool = False,
    fit_hz: float = 200.0,
    smooth_s: float = 0.02,
) -> tuple[list[Run], list[tuple[RunRecord, str]]]:
    """Load every run that has a log file. Returns (runs, skipped) with a reason per skip."""
    runs: list[Run] = []
    skipped: list[tuple[RunRecord, str]] = []
    log_dir = session.path if log_dir is None else log_dir
    for record in session.runs:
        if not record.spec.matches(kinds=kinds, channels=channels, roles=roles):
            continue
        if not record.log_file:
            skipped.append((record, "no log file"))
            continue
        if record.verdict == "discard" and not include_discarded:
            skipped.append((record, "verdict: discard"))
            continue
        try:
            runs.append(load_run(record, log_dir, calib, fit_hz=fit_hz, smooth_s=smooth_s))
        except (OSError, ValueError) as err:
            skipped.append((record, str(err)))
    return runs, skipped
