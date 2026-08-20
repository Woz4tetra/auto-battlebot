"""Excitation waveforms for velocity jig runs, declared in TOML.

This replaces the web tool's `excitation.js` plus its hard-coded 21-entry experiment
catalog. The catalog was the runbook compiled into JavaScript, so changing an amplitude or
trying a new excitation meant editing code. Here a waveform is a TOML table:

    [waveform.lin_step_full]
    kind = "step"
    channel = "linear"
    role = "fit"
    amplitudes = [0.25, 0.5, 0.75, 1.0]
    hold_s = 2.0

`kind`, `channel` and `role` are also what the fit routes on, so a new waveform reaches the
right fit without touching the fitter.

Commands are normalized to [-1, 1] and dimensionless. Mapping to the radio's integer range
is the transmitter link's job, and deadzone compensation is deliberately not applied
anywhere: the physical deadzone is one of the things being measured.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Sequence

from auto_battlebot.velocity_jig import CHANNELS, ROLES, WAVEFORM_KINDS, ProtocolSegment

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

# A held command shorter than this cannot show a plateau to read a steady state off. Stage 2
# held roughly a fifth of a second, which sat under the rise, and no maximum speed could be
# read from it. The floor is applied to every generated hold rather than trusted to config.
MIN_HOLD_S = 0.5
# Time to let the robot come to rest between cells, so each step's decay is measured from a
# standstill rather than from whatever the previous cell left behind.
MIN_COAST_S = 1.5

DEFAULT_RATE_HZ = 50.0


@dataclass
class Program:
    """A playable excitation.

    `segments` is present for piecewise programs and None for continuous ones (chirp, sine,
    triangle, PRBS, manual), where there is no cell structure to slice by. `at(t)` is the
    authority either way: the player samples it, and segments exist so the fit can label
    samples by phase.
    """

    name: str
    kind: str
    channel: str
    role: str
    duration_s: float
    at: Callable[[float], tuple[float, float]]
    segments: list[ProtocolSegment] | None = None
    label: str = ""
    params: dict[str, Any] = field(default_factory=dict)

    def sample(self, rate_hz: float = DEFAULT_RATE_HZ) -> list[tuple[float, float, float]]:
        """(t, linear, angular) on a uniform grid. For --dry-run and footprint prediction."""
        n = max(1, int(round(self.duration_s * rate_hz)))
        out = []
        for i in range(n + 1):
            t = i / rate_hz
            lin, ang = self.at(t)
            out.append((t, lin, ang))
        return out


@dataclass(frozen=True)
class WaveformDecl:
    """One catalog entry, before it is built into a Program."""

    name: str
    kind: str
    channel: str
    role: str
    params: dict[str, Any]
    label: str = ""
    trim: bool = False
    motion: bool = True


# ---------------------------------------------------------------------------
# Segment assembly
# ---------------------------------------------------------------------------


class _Builder:
    """Accumulates held cells into a piecewise program."""

    def __init__(self) -> None:
        self.segments: list[ProtocolSegment] = []
        self.t = 0.0

    def hold(self, duration: float, linear: float, angular: float, label: str) -> None:
        if duration <= 0.0:
            return
        self.segments.append(
            ProtocolSegment(self.t, self.t + duration, linear, angular, label)
        )
        self.t += duration

    def coast(self, duration: float, label: str) -> None:
        self.hold(max(duration, MIN_COAST_S), 0.0, 0.0, label)


def _piecewise(segments: Sequence[ProtocolSegment]) -> Callable[[float], tuple[float, float]]:
    """Zero outside the program. Holding the last command past the end would tell the fit
    the robot was driven at full command while it sat on the floor."""
    bounds = [s.t0 for s in segments]

    def at(t: float) -> tuple[float, float]:
        if not segments or t < segments[0].t0 or t >= segments[-1].t1:
            return 0.0, 0.0
        lo, hi = 0, len(segments) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if bounds[mid] <= t:
                lo = mid
            else:
                hi = mid - 1
        seg = segments[lo]
        return (seg.linear, seg.angular) if seg.t0 <= t < seg.t1 else (0.0, 0.0)

    return at


def _axes(channel: str, primary: float, secondary: float = 0.0) -> tuple[float, float]:
    """Map an amplitude onto (linear, angular) for the declared channel."""
    if channel == "linear":
        return primary, secondary
    if channel == "angular":
        return secondary, primary
    return primary, secondary


def cap_amplitudes(amplitudes: Sequence[float], cap: float | None) -> list[float]:
    """Scale a ladder so its peak lands on `cap`, rather than clamping to it.

    Clamping would collapse the top rungs onto one value and destroy the spacing the fit
    reads a gain slope from. Scaling keeps the ratios and just makes the whole ladder
    smaller, which is what the encoder-wheel rate limit actually requires.
    """
    values = [float(a) for a in amplitudes]
    if cap is None or not values:
        return values
    peak = max(abs(a) for a in values)
    if peak <= cap or peak <= 0.0:
        return values
    scale = cap / peak
    return [round(a * scale, 3) for a in values]


# ---------------------------------------------------------------------------
# Generators
# ---------------------------------------------------------------------------


def _build_step(d: WaveformDecl) -> Program:
    p = d.params
    amps = cap_amplitudes(p.get("amplitudes", [0.25, 0.5, 0.75, 1.0]), p.get("cap"))
    hold = max(float(p.get("hold_s", MIN_HOLD_S)), MIN_HOLD_S)
    coast = float(p.get("coast_s", MIN_COAST_S))
    shuttle = bool(p.get("shuttle", True))
    b = _Builder()
    for amp in amps:
        signs = (1.0, -1.0) if shuttle else (1.0,)
        for sign in signs:
            lin, ang = _axes(d.channel, sign * amp, sign * float(p.get("secondary", 0.0)))
            b.hold(hold, lin, ang, f"step_{amp:g}")
            b.coast(coast, "coast")
    return _finish(d, b)


def _build_staircase(d: WaveformDecl) -> Program:
    """Creep the command up until the wheels break static friction.

    Stage 2 reported a deadzone of 0.04 because 0.04 was the smallest command it tested and
    the robot already moved there. Starting at 0.01 is the point: several rungs must sit
    below the motion threshold or the crossing is an extrapolation.
    """
    p = d.params
    start = float(p.get("from", 0.01))
    stop = float(p.get("to", 0.10))
    step = float(p.get("step", 0.01))
    hold = max(float(p.get("hold_s", MIN_HOLD_S)), MIN_HOLD_S)
    b = _Builder()
    for sign in ((1.0, -1.0) if bool(p.get("both_directions", True)) else (1.0,)):
        level = start
        while level <= stop + 1e-9:
            lin, ang = _axes(d.channel, sign * level)
            b.hold(hold, lin, ang, f"staircase_{level:.3f}")
            level += step
        b.coast(MIN_COAST_S, "coast")
    return _finish(d, b)


def _build_coast(d: WaveformDecl) -> Program:
    """Accelerate, hold, then drop to zero instantly and let the robot roll to a stop.

    The drop must be instantaneous. A ramp down is a second input and the decel constant
    fitted through it is the ramp's, not the robot's.
    """
    p = d.params
    amp = float(p.get("amplitude", 0.6))
    hold = max(float(p.get("hold_s", MIN_HOLD_S)), MIN_HOLD_S)
    reps = int(p.get("reps", 10))
    shuttle = bool(p.get("shuttle", True))
    coast = float(p.get("coast_s", MIN_COAST_S))
    b = _Builder()
    for i in range(reps):
        sign = -1.0 if (shuttle and i % 2) else 1.0
        lin, ang = _axes(d.channel, sign * amp)
        b.hold(hold, lin, ang, "coast_entry")
        b.coast(coast, "coast_tail")
    return _finish(d, b)


def _build_grid(d: WaveformDecl) -> Program:
    """Cross linear against angular to separate the coupling terms.

    The angular list must span both signs. Steer-brake and angular droop flip with the turn
    direction while straight-line drift does not, so a grid that only turns one way cannot
    tell them apart and will fold the drift into the droop coefficient.
    """
    p = d.params
    lins = [float(v) for v in p.get("linear", [0.25, 0.5, 0.75, 1.0])]
    angs = cap_amplitudes(
        p.get("angular", [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]), p.get("angular_cap")
    )
    hold = max(float(p.get("hold_s", MIN_HOLD_S)), MIN_HOLD_S)
    b = _Builder()
    for lin in lins:
        for ang in angs:
            b.hold(hold, lin, ang, f"grid_{lin:g}_{ang:g}")
            b.coast(MIN_COAST_S, "coast")
    return _finish(d, b)


def _build_prbs(d: WaveformDecl) -> Program:
    """Maximal-length 16-bit LFSR, taps 16/14/13/11.

    Reproduced bit-exactly from the web tool so runs recorded either side of the rewrite
    stay comparable. A 60 ms bit period covers roughly 0.2 to 8 Hz, which brackets both the
    58-78 ms plant time constants and the 33 ms frame period.
    """
    p = d.params
    amp = float(p.get("amplitude", 0.6))
    bit_ms = float(p.get("bit_ms", 60.0))
    duration = float(p.get("duration_s", 30.0))
    seed = int(p.get("seed", 0xACE1)) & 0xFFFF or 0xACE1
    ang_amp = float(p.get("angular_amplitude", amp))

    def bits(state: int, n: int) -> tuple[list[int], int]:
        out = []
        s = state
        for _ in range(n):
            bit = ((s >> 0) ^ (s >> 2) ^ (s >> 3) ^ (s >> 5)) & 1
            s = ((s >> 1) | (bit << 15)) & 0xFFFF
            out.append(1 if bit else -1)
        return out, s

    n = max(1, math.ceil(duration * 1000.0 / bit_ms))
    seq_a, state = bits(seed, n)
    # A second, decorrelated sequence for the angular channel of a combined run. Reusing one
    # sequence on both channels would make them perfectly correlated, and the coupling terms
    # would be unidentifiable no matter how long the run.
    seq_b, _ = bits((state * 1103515245 + 12345) & 0xFFFF or 0xBEEF, n)

    def at(t: float) -> tuple[float, float]:
        if t < 0.0 or t >= duration:
            return 0.0, 0.0
        i = min(n - 1, int(t * 1000.0 / bit_ms))
        if d.channel == "linear":
            return amp * seq_a[i], 0.0
        if d.channel == "angular":
            return 0.0, ang_amp * seq_a[i]
        return amp * seq_a[i], ang_amp * seq_b[i]

    return Program(
        name=d.name,
        kind=d.kind,
        channel=d.channel,
        role=d.role,
        duration_s=duration,
        at=at,
        segments=None,
        label=d.label,
        params=dict(p),
    )


def _build_chirp(d: WaveformDecl) -> Program:
    """Swept sine, logarithmic by default.

    A first-order model with tau 58 ms puts its corner near 2.7 Hz. A sweep through that
    corner is the frequency-domain cross-check on a time-constant fitted from steps.
    """
    p = d.params
    f0 = float(p.get("f0_hz", 0.2))
    f1 = float(p.get("f1_hz", 8.0))
    amp = float(p.get("amplitude", 0.4))
    duration = float(p.get("duration_s", 30.0))
    log_sweep = bool(p.get("log_sweep", True))

    def phase(t: float) -> float:
        if log_sweep and f0 > 0.0 and f1 > 0.0 and abs(f1 - f0) > 1e-9:
            k = math.log(f1 / f0) / duration
            return 2.0 * math.pi * f0 * (math.exp(k * t) - 1.0) / k
        return 2.0 * math.pi * (f0 * t + 0.5 * (f1 - f0) / duration * t * t)

    def at(t: float) -> tuple[float, float]:
        if t < 0.0 or t >= duration:
            return 0.0, 0.0
        return _axes(d.channel, amp * math.sin(phase(t)))

    return Program(d.name, d.kind, d.channel, d.role, duration, at, None, d.label, dict(p))


def _periodic(d: WaveformDecl, shape: Callable[[float], float]) -> Program:
    p = d.params
    freq = float(p.get("freq_hz", 1.0))
    amp = float(p.get("amplitude", 0.5))
    offset = float(p.get("offset", 0.0))
    if p.get("cycles") is not None and freq > 0.0:
        duration = float(p["cycles"]) / freq
    else:
        duration = float(p.get("duration_s", 20.0))

    def at(t: float) -> tuple[float, float]:
        if t < 0.0 or t >= duration:
            return 0.0, 0.0
        value = offset + amp * shape(freq * t)
        return _axes(d.channel, max(-1.0, min(1.0, value)))

    return Program(d.name, d.kind, d.channel, d.role, duration, at, None, d.label, dict(p))


def _build_sine(d: WaveformDecl) -> Program:
    return _periodic(d, lambda cycles: math.sin(2.0 * math.pi * cycles))


def _build_triangle(d: WaveformDecl) -> Program:
    # Constant slew between the peaks, so unlike a sine it excites the rate limit evenly
    # rather than concentrating the fastest command changes at the zero crossings.
    return _periodic(d, lambda cycles: 4.0 * abs(((cycles - 0.25) % 1.0) - 0.5) - 1.0)


def _build_manual(d: WaveformDecl) -> Program:
    """Emits zeros. The operator drives or pushes; the radio readback is the command log."""
    duration = float(d.params.get("duration_s", 60.0))
    return Program(
        name=d.name,
        kind=d.kind,
        channel=d.channel,
        role=d.role,
        duration_s=duration,
        at=lambda t: (0.0, 0.0),
        segments=None,
        label=d.label,
        params=dict(d.params),
    )


def _build_trim(d: WaveformDecl) -> Program:
    """Low-amplitude straight drive used to measure the arc, not to fit anything.

    Run before a session to find out whether a straight run will stay on the board, and to
    seed the drift parameter before any fit exists. Driven open loop here; the closed-loop
    correction lives in the CLI, which can see the jig's live stream.
    """
    p = d.params
    amp = float(p.get("amplitude", 0.15))
    duration = float(p.get("duration_s", 5.0))
    b = _Builder()
    b.hold(duration, amp, 0.0, "trim")
    b.coast(MIN_COAST_S, "coast")
    return _finish(d, b)


def _finish(d: WaveformDecl, b: _Builder) -> Program:
    return Program(
        name=d.name,
        kind=d.kind,
        channel=d.channel,
        role=d.role,
        duration_s=b.t,
        at=_piecewise(b.segments),
        segments=b.segments,
        label=d.label,
        params=dict(d.params),
    )


BUILDERS: dict[str, Callable[[WaveformDecl], Program]] = {
    "step": _build_step,
    "staircase": _build_staircase,
    "coast": _build_coast,
    "grid": _build_grid,
    "prbs": _build_prbs,
    "chirp": _build_chirp,
    "sine": _build_sine,
    "triangle": _build_triangle,
    "trim": _build_trim,
    "manual": _build_manual,
}


def build(decl: WaveformDecl) -> Program:
    try:
        builder = BUILDERS[decl.kind]
    except KeyError:
        raise ValueError(f"{decl.name}: no builder for kind {decl.kind!r}") from None
    return builder(decl)


# ---------------------------------------------------------------------------
# Catalog
# ---------------------------------------------------------------------------

_META_KEYS = {"kind", "channel", "role", "label", "trim", "motion"}


def load_catalog(path: Path | str) -> dict[str, WaveformDecl]:
    """Read a waveform TOML. `[defaults]` is merged under every entry."""
    path = Path(path)
    with open(path, "rb") as handle:
        data = tomllib.load(handle)

    defaults = dict(data.get("defaults", {}))
    out: dict[str, WaveformDecl] = {}
    for name, table in (data.get("waveform") or {}).items():
        if not isinstance(table, dict):
            raise ValueError(f"{path}: [waveform.{name}] is not a table")
        kind = str(table.get("kind", "")).strip()
        channel = str(table.get("channel", "")).strip()
        role = str(table.get("role", "fit")).strip()
        if kind not in WAVEFORM_KINDS:
            raise ValueError(f"{path}: [waveform.{name}] kind {kind!r} not in {WAVEFORM_KINDS}")
        if channel not in CHANNELS:
            raise ValueError(f"{path}: [waveform.{name}] channel {channel!r} not in {CHANNELS}")
        if role not in ROLES:
            raise ValueError(f"{path}: [waveform.{name}] role {role!r} not in {ROLES}")
        params = {k: v for k, v in defaults.items() if k not in _META_KEYS}
        params.update({k: v for k, v in table.items() if k not in _META_KEYS})
        out[name] = WaveformDecl(
            name=name,
            kind=kind,
            channel=channel,
            role=role,
            params=params,
            label=str(table.get("label", "")),
            trim=bool(table.get("trim", defaults.get("trim", False))),
            motion=bool(table.get("motion", True)),
        )
    if not out:
        raise ValueError(f"{path}: no [waveform.*] entries")
    return out
