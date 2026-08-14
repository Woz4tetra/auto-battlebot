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
(`load_session`, reading the web tool's exported session JSON).

Three failure modes get flagged rather than silently absorbed, because stage 2 shipped
parameters from silently degraded ground truth:

- Saturated IMU channels (`|raw| > 32000`), per axis, per run.
- Gyro bias drift between the pre-run and post-run still holds.
- Encoder slip, from disagreement between encoder acceleration and IMU longitudinal
  acceleration.

See `docs/experiments/kalman_filter/kalman_filter_plan.md` part 1.2 and the companion runbook.
"""

from __future__ import annotations

import json
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

AXIS_NAMES = ("x", "y", "z")

# Experiment ids whose commands are scripted excitation the fit trains on, versus the holdout
# set. Kept here because both fit scripts and any future report need the same split.
FIT_EXPERIMENTS = ("E7", "E8", "E9", "E10", "E11", "E12", "E13", "E20")
HOLDOUT_EXPERIMENTS = ("E14", "E15", "E16", "E17", "E18", "E19")


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

    def saturation(self) -> list[AxisSaturation]:
        """Per-axis saturation report, gyro first then accel."""
        out: list[AxisSaturation] = []
        for prefix, raw in (("g", self.gyro_raw), ("a", self.accel_raw)):
            for i, axis in enumerate(AXIS_NAMES):
                col = raw[:, i]
                over = int(np.count_nonzero(np.abs(col) > RAW_SATURATION))
                peak = int(np.max(np.abs(col))) if len(col) else 0
                out.append(AxisSaturation(f"{prefix}{axis}", over, len(col), peak))
        return out

    def saturated_fraction(self) -> float:
        """Worst per-axis saturated fraction, the single number a run gate needs."""
        report = self.saturation()
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
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                header_lines.append(line)
                continue
            parts = line.split(",")
            if len(parts) != 8:
                malformed += 1
                continue
            try:
                rows.append([int(p) for p in parts])
            except ValueError:
                malformed += 1

    if not rows:
        raise ValueError(f"{path}: no sample rows")

    data = np.array(rows, dtype=np.int64)
    header = _parse_header(header_lines)
    # time_us_64() wraps after 584,000 years, so a decreasing timestamp means a corrupt row, not
    # a rollover. Keep the monotone prefix and count the rest as malformed.
    good = np.concatenate([[True], np.diff(data[:, 0]) > 0])
    malformed += int(np.count_nonzero(~good))
    data = data[good]

    return JigLog(
        path=path,
        header=header,
        t=data[:, 0].astype(float) * 1e-6,
        count=data[:, 1].copy(),
        gyro_raw=data[:, 2:5].copy(),
        accel_raw=data[:, 5:8].copy(),
        malformed_rows=malformed,
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


def find_still_segments(
    log: JigLog,
    min_duration: float = 5.0,
    gyro_thresh_dps: float = 2.0,
    accel_tol_g: float = 0.05,
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


def plane_basis(up: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two orthonormal vectors spanning the horizontal plane, for lever-arm and slip work.

    The basis is arbitrary in rotation about `up`. Nothing here needs a particular forward axis;
    `forward_axis` finds the real one from motion.
    """
    seed = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(seed, up))) > 0.9:
        seed = np.array([0.0, 1.0, 0.0])
    e1 = seed - np.dot(seed, up) * up
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    return e1, e2


# ---------------------------------------------------------------------------
# Calibration constants (E3, E4, E5)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JigCalibration:
    """Bench calibration from runbook block 0, one set per hardware configuration.

    `meters_per_count` (E4) and `gyro_scale` (E3) set the units of every fitted parameter.
    `r_enc_perp` (E5) is the encoder wheel's lever arm, removed from every speed measurement,
    which is what makes combined linear-plus-angular runs usable instead of discarded. `r_imu` is
    kept for the report and for E5's own sanity check against a tape measure: the slip detector
    fits the IMU's lever-arm terms per run instead, because a single scalar cannot say which axis
    the offset lies on and the two terms land on different channels depending on that.
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
    """One `TIME` probe burst, as the web tool records it (`jig.js probeClock`)."""

    offset_ms: float  # host minus jig
    residual_ms: float
    at_host_ms: float
    at_jig_ms: float
    kept: int = 0
    total: int = 0

    @classmethod
    def from_json(cls, data: dict[str, Any] | None) -> ClockProbe | None:
        if not data:
            return None
        return cls(
            offset_ms=float(data["offsetMs"]),
            residual_ms=float(data.get("residualMs", 0.0)),
            at_host_ms=float(data.get("atHostMs", 0.0)),
            at_jig_ms=float(data.get("atJigMs", 0.0)),
            kept=int(data.get("kept", 0)),
            total=int(data.get("total", 0)),
        )


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


# ---------------------------------------------------------------------------
# Session JSON (web tool export)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ProtocolSegment:
    """One held command cell of a piecewise program, in program-relative seconds."""

    t0: float
    t1: float
    linear: float
    angular: float
    label: str


@dataclass
class RunRecord:
    """One row of the exported session: what was played, when, and whether it passed its gates."""

    run_id: str
    experiment_id: str
    variant: str
    encoder: str  # "attached", "detached", or "either"
    log_file: str | None
    verdict: str | None
    notes: str
    trim: float | None
    samples: int | None
    dropped: int | None
    clock_pre: ClockProbe | None
    clock_post: ClockProbe | None
    skew_ppm: float | None
    hold_pre: tuple[float, float] | None  # host seconds
    hold_post: tuple[float, float] | None
    protocol_label: str
    segments: list[ProtocolSegment]
    cmd_t: np.ndarray  # host seconds
    cmd_lin: np.ndarray
    cmd_ang: np.ndarray

    @property
    def clock(self) -> ClockFit:
        return ClockFit.from_probes(self.clock_pre, self.clock_post)

    @property
    def has_commands(self) -> bool:
        return len(self.cmd_t) > 1


@dataclass
class Session:
    """The web tool's session export, which is the only metadata a `LOG-N.TXT` has."""

    path: Path
    session_id: str
    robot: str
    operator: str
    floor_surface: str
    guard_plates_on: bool
    weapon_disabled: bool
    encoder_rate_limit: float | None
    runs: list[RunRecord] = field(default_factory=list)

    def passing(self, experiments: Sequence[str] | None = None) -> list[RunRecord]:
        """Runs that cleared their capture-time gates, optionally filtered by experiment id."""
        out = [r for r in self.runs if r.verdict == "pass"]
        if experiments is not None:
            keep = set(experiments)
            out = [r for r in out if r.experiment_id in keep]
        return out


def _hold_seconds(data: dict[str, Any] | None) -> tuple[float, float] | None:
    if not data or "start" not in data or "end" not in data:
        return None
    return float(data["start"]) * 1e-3, float(data["end"]) * 1e-3


def _segments(protocol: dict[str, Any] | None) -> list[ProtocolSegment]:
    if not protocol or not protocol.get("segments"):
        return []
    return [
        ProtocolSegment(
            t0=float(s["t0"]),
            t1=float(s["t1"]),
            linear=float(s.get("linear", 0.0)),
            angular=float(s.get("angular", 0.0)),
            label=str(s.get("label", "")),
        )
        for s in protocol["segments"]
    ]


def load_session(path: Path | str) -> Session:
    """Load a session exported by the velocity jig web tool (`session.js toJson`).

    Command timestamps and clock probes share one `performance.now()` origin, so the only
    unknown left between a command and a sample is the jig offset, which `ClockFit` covers.
    """
    path = Path(path)
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)

    runs: list[RunRecord] = []
    for raw in data.get("runs", []):
        commands = raw.get("commands") or []
        cmd_t = np.array([float(c["tHost"]) * 1e-3 for c in commands], dtype=float)
        cmd_lin = np.array([float(c.get("linear", 0.0)) for c in commands], dtype=float)
        cmd_ang = np.array([float(c.get("angular", 0.0)) for c in commands], dtype=float)
        protocol = raw.get("protocol")
        runs.append(
            RunRecord(
                run_id=str(raw.get("id", "")),
                experiment_id=str(raw.get("experimentId", "")),
                variant=str(raw.get("variant", "main")),
                encoder=str(raw.get("encoder", "either")),
                log_file=raw.get("logFile"),
                verdict=raw.get("verdict"),
                notes=str(raw.get("notes", "")),
                trim=(None if raw.get("trim") is None else float(raw["trim"])),
                samples=(None if raw.get("samples") is None else int(raw["samples"])),
                dropped=(None if raw.get("dropped") is None else int(raw["dropped"])),
                clock_pre=ClockProbe.from_json(raw.get("clockPre")),
                clock_post=ClockProbe.from_json(raw.get("clockPost")),
                skew_ppm=(None if raw.get("skewPpm") is None else float(raw["skewPpm"])),
                hold_pre=_hold_seconds(raw.get("holdPre")),
                hold_post=_hold_seconds(raw.get("holdPost")),
                protocol_label=str((protocol or {}).get("label", "")),
                segments=_segments(protocol),
                cmd_t=cmd_t,
                cmd_lin=cmd_lin,
                cmd_ang=cmd_ang,
            )
        )

    return Session(
        path=path,
        session_id=str(data.get("id", path.stem)),
        robot=str(data.get("robot", "")),
        operator=str(data.get("operator", "")),
        floor_surface=str(data.get("floorSurface", "")),
        guard_plates_on=bool(data.get("guardPlatesOn", False)),
        weapon_disabled=bool(data.get("weaponDisabled", False)),
        encoder_rate_limit=(
            None if data.get("encoderRateLimit") is None else float(data["encoderRateLimit"])
        ),
        runs=runs,
    )


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


def forward_axis(
    up: np.ndarray, accel: np.ndarray, a_ref: np.ndarray, mask: np.ndarray | None = None
) -> np.ndarray:
    """Recover the IMU's forward axis by regressing horizontal accel on encoder acceleration.

    Gravity fixes the yaw axis but says nothing about which horizontal direction is forward, and
    that direction is needed to cross-check the encoder against the IMU.

    Each horizontal component is regressed *on* the encoder acceleration, not the other way
    round. That matches the physics (forward acceleration causes the reading) and, more
    practically, it stays well posed: on a run with no turning, the lateral component is pure
    noise, and regressing the encoder on both components hands that near-degenerate column a huge
    coefficient which then dominates the normalized direction.
    """
    e1, e2 = plane_basis(up)
    h1 = accel @ e1
    h2 = accel @ e2
    if mask is not None:
        h1, h2, a_ref = h1[mask], h2[mask], a_ref[mask]
    denom = float(np.sum(a_ref * a_ref))
    if len(a_ref) < 10 or denom < 1e-9:
        return e1
    vec = (float(np.sum(h1 * a_ref)) / denom) * e1 + (float(np.sum(h2 * a_ref)) / denom) * e2
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9:
        return e1
    return vec / norm


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
    slip: np.ndarray  # bool, encoder and IMU disagree on longitudinal acceleration
    encoder_valid: bool
    bias_drift_dps: float
    yaw_scale: float
    clock: ClockFit

    @property
    def slip_fraction(self) -> float:
        return float(np.mean(self.slip)) if len(self.slip) else 0.0

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
    slip_sigma: float = 4.0,
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

    slip = np.zeros(n, dtype=bool)
    if encoder_valid:
        slip = _slip_flags(log, up, w, v, calib, window, slip_sigma)

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
        slip=slip,
        encoder_valid=encoder_valid,
        bias_drift_dps=drift,
        yaw_scale=calib.gyro_scale,
        clock=clock,
    )


def _slip_flags(
    log: JigLog,
    up: np.ndarray,
    w: np.ndarray,
    v: np.ndarray,
    calib: JigCalibration,
    window: int,
    sigma: float,
) -> np.ndarray:
    """Flag stretches where the encoder and the IMU disagree about how the speed changed.

    The trailing wheel can slip under hard accel or lift during a wheelie, which is exactly the
    regime the plant fit cares most about. The IMU does not slip, so the two only disagree when
    the encoder is wrong.

    The comparison is on speed change over a 100 ms window, not on instantaneous acceleration.
    Encoder acceleration needs two differentiations of a quantized count and therefore heavy
    smoothing, while IMU acceleration is measured directly, so comparing them sample by sample
    mostly measures the difference between two filters. Integrating the IMU over the same window
    the encoder is differenced over puts both through one operator, and the result is in m/s
    where a threshold means something physical.

    Centripetal (`w^2 r`) and tangential (`alpha x r`) terms come out first: both appear in the
    IMU during a turn and neither is forward acceleration.
    """
    dt = max(log.dt, 1e-9)
    a_enc = _moving_average(np.gradient(v, log.t), window)
    accel = log.accel
    # Remove gravity, then project onto the measured forward direction.
    horiz = accel - np.outer(accel @ up, up)
    alpha = np.gradient(w, log.t)
    fwd = forward_axis(up, accel, a_enc, mask=np.abs(v) > 0.2)
    a_fwd = horiz @ fwd

    # Remove the lever-arm terms by regression rather than by assuming the geometry. An IMU
    # offset from the yaw axis adds a centripetal term (w^2 r) and a tangential term (alpha r) to
    # the forward channel, and which one lands where depends on which way the offset points. E5
    # measures a single scalar, which cannot say. Fitting the two coefficients against the
    # residual that the encoder does not explain covers any mounting.
    design = np.stack([np.ones_like(w), w**2, alpha], axis=1)
    coeff, *_ = np.linalg.lstsq(design, a_fwd - a_enc, rcond=None)
    a_imu = a_fwd - design @ coeff

    # Integrate the IMU into a relative velocity and put it through the same smoothing the
    # encoder velocity went through. Without that, a fast reversal shows up as disagreement
    # purely because one signal is filtered and the other is not.
    span = max(2, int(round(0.1 / dt)))
    integral = np.concatenate([[0.0], np.cumsum(a_imu[1:] * np.diff(log.t))])
    integral = _moving_average(integral, window)
    diff = (v[span:] - v[:-span]) - (integral[span:] - integral[:-span])

    # Robust scale, because the slips themselves are in this signal and a plain standard
    # deviation would widen the threshold until it stopped catching them.
    scale = 1.4826 * float(np.median(np.abs(diff - np.median(diff))))
    if not np.isfinite(scale):
        return np.zeros(len(v), dtype=bool)
    # Absolute floor: a disagreement under 0.1 m/s over 100 ms is below the noise of this
    # comparison and not worth throwing a window away for.
    flagged = (np.abs(diff) > max(sigma * scale, 0.1)).astype(float)

    # A flag covers the window it was measured over, not just that window's first sample.
    dilated = np.convolve(flagged, np.ones(span))[: len(v)] > 0.0
    out = np.zeros(len(v), dtype=bool)
    out[: len(dilated)] = dilated
    return out


# ---------------------------------------------------------------------------
# Uniform-rate run assembly
# ---------------------------------------------------------------------------


@dataclass
class RunQuality:
    """Everything that decides whether a run is fit for fitting, in one place."""

    log_file: str
    experiment_id: str
    variant: str
    verdict: str | None
    dropped: int | None
    malformed_rows: int
    worst_saturation: float
    saturated_axes: list[str]
    bias_drift_dps: float
    slip_fraction: float
    clock_measured: bool
    clock_residual_ms: float
    skew_ppm: float | None
    still_holds: int
    encoder_valid: bool

    def problems(
        self,
        max_bias_drift_dps: float = 0.05,
        # A handful of clipped samples on one axis is not the same failure as a channel that
        # spends a fight against its rail, so the gate is a fraction rather than zero. The
        # fraction is always reported, and E11's own pass bar is still zero saturated samples.
        max_saturation: float = 0.001,
        max_slip_fraction: float = 0.05,
        max_clock_residual_ms: float = 2.0,
    ) -> list[str]:
        """Reasons to exclude this run. Empty means usable."""
        out: list[str] = []
        if self.verdict == "discard":
            out.append("capture-time verdict: discard")
        if self.dropped:
            out.append(f"dropped={self.dropped} samples on the SD path")
        if self.malformed_rows:
            out.append(f"{self.malformed_rows} malformed rows")
        if self.worst_saturation > max_saturation:
            axes = ", ".join(self.saturated_axes)
            out.append(f"IMU saturation {self.worst_saturation:.2%} on {axes}")
        if np.isfinite(self.bias_drift_dps) and self.bias_drift_dps > max_bias_drift_dps:
            out.append(f"gyro bias drift {self.bias_drift_dps:.3f} deg/s")
        if self.slip_fraction > max_slip_fraction:
            out.append(f"encoder slip on {self.slip_fraction:.1%} of samples")
        if self.clock_measured and self.clock_residual_ms > max_clock_residual_ms:
            out.append(f"clock residual {self.clock_residual_ms:.2f} ms")
        if self.still_holds < 2:
            out.append(f"{self.still_holds} still holds found, need 2")
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
    slip: np.ndarray
    v_noise: float
    w_noise: float
    truth: GroundTruth

    @property
    def experiment_id(self) -> str:
        return self.record.experiment_id

    @property
    def name(self) -> str:
        log = self.record.log_file or self.record.run_id
        return f"{log} [{self.record.experiment_id}/{self.record.variant}]"

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


def _labels_for(record: RunRecord, t: np.ndarray) -> np.ndarray:
    """Per-sample phase label. Segments are program-relative; the first command anchors them."""
    if not record.has_commands:
        return np.array([record.experiment_id] * len(t))
    t0 = float(record.cmd_t[0])
    if not record.segments:
        return np.array([record.experiment_id] * len(t))
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

    stills = find_still_segments(log)
    pre = still_stats(log, stills[0]) if stills else None
    post = still_stats(log, stills[-1]) if len(stills) > 1 else None
    clock = record.clock
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
    slip = np.interp(grid, truth.t, truth.slip.astype(float)) > 0.5

    cmd_lin = zoh(record.cmd_t, record.cmd_lin, grid)
    cmd_ang = zoh(record.cmd_t, record.cmd_ang, grid)
    labels = _labels_for(record, grid)

    # Where the command is actually known. The still holds that bracket every run sit outside the
    # command log, and so does an operator-driven run whose sticks were never logged. Coasting
    # for a couple of seconds past the last command is real data (the runner zeroes and disarms
    # on exit), so that much grace is kept and the rest is dropped.
    if record.has_commands:
        commanded = (grid >= record.cmd_t[0]) & (grid <= record.cmd_t[-1] + 2.0)
    else:
        commanded = np.zeros(len(grid), dtype=bool)

    # Noise floors come from the still holds, which are the only stretches known to be motionless.
    idle = np.zeros(len(grid), dtype=bool)
    for seg in stills:
        t0, t1 = clock.host_seconds(np.array([seg.t0, seg.t1]))
        idle |= (grid >= t0) & (grid <= t1)
    v_noise = float(np.std(v[idle])) if np.count_nonzero(idle) > 10 else 0.0
    w_noise = float(np.std(w[idle])) if np.count_nonzero(idle) > 10 else 0.0

    sat = log.saturation()
    quality = RunQuality(
        log_file=record.log_file,
        experiment_id=record.experiment_id,
        variant=record.variant,
        verdict=record.verdict,
        dropped=record.dropped,
        malformed_rows=log.malformed_rows,
        worst_saturation=max((a.fraction for a in sat), default=0.0),
        saturated_axes=[a.name for a in sat if a.count],
        bias_drift_dps=truth.bias_drift_dps,
        slip_fraction=truth.slip_fraction,
        clock_measured=clock.measured,
        clock_residual_ms=clock.residual_ms,
        skew_ppm=record.skew_ppm,
        still_holds=len(stills),
        encoder_valid=truth.encoder_valid,
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
        slip=slip,
        v_noise=v_noise,
        w_noise=w_noise,
        truth=truth,
    )


def load_runs(
    session: Session,
    log_dir: Path | str,
    calib: JigCalibration,
    *,
    experiments: Sequence[str] | None = None,
    include_discarded: bool = False,
    fit_hz: float = 200.0,
    smooth_s: float = 0.02,
) -> tuple[list[Run], list[tuple[RunRecord, str]]]:
    """Load every run that has a log file. Returns (runs, skipped) with a reason per skip."""
    runs: list[Run] = []
    skipped: list[tuple[RunRecord, str]] = []
    keep = set(experiments) if experiments is not None else None
    for record in session.runs:
        if keep is not None and record.experiment_id not in keep:
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
