"""What the driver did with the sticks: holds, edges, creeps, arcs, spins.

Two consumers:

- ``ManeuverSegmenter`` labels the stick stream as it arrives for the live coverage monitor. It
  works on the uniform 10 ms grid the monitor builds, with sticks as fractions, throttle forward
  positive and steer left positive.
- ``menu_from_grid`` is the offline session summary that
  ``playground/calibration/check_sysid_recordings.py`` prints against its ``MENU_TARGETS``. It
  moved here unchanged when the live monitor became its second caller.

Live definitions, ported from the prototype page:

- Plateau: a maximal run where a channel stays within ``PLATEAU_TOL`` of every other sample in
  it, the jitter of a held thumb.
- Hold: a plateau of ``HOLD_S`` or longer away from zero. A throttle hold with the steer mean
  under ``ARC_STEER`` is straight, with more it is an arc; a steer hold with the throttle mean
  under ``ARC_STEER`` is a spin.
- Edge: the move between two long plateaus (rest counts at any length) that differ by at least
  ``EDGE_MIN``. Clean when it took ``SNAP_S`` or less. Only clean edges count toward delay and
  lag; a slower one is a slew.
- Creep: a monotonic climb out of zero past ``CREEP_PAST`` that took ``CREEP_MIN_S`` or longer.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

# ---------------------------------------------------------------------------
# Live segmentation
# ---------------------------------------------------------------------------

PLATEAU_TOL = 0.06
HOLD_S = 0.5
SNAP_S = 0.08
EDGE_MIN = 0.1
ARC_STEER = 0.12
REST = 0.03
CREEP_PAST = 0.12
CREEP_FROM = 0.02
CREEP_MIN_S = 0.8
HANDS_OFF = 0.03  # steer under this for a whole straight hold reads as hands off

CHANNELS = ("Throttle", "Steer")
# Coverage map bins, stick fractions: finer near zero, where the deadzones are.
BINS = (-1.0, -0.75, -0.5, -0.25, -0.1, -0.05, 0.0, 0.05, 0.1, 0.25, 0.5, 0.75, 1.0)
AMPS = (0.25, 0.5, 0.75, 1.0)
# Rows of the transition table. The loop rows are steer edges with the flip switch down, where
# the firmware yaw loop answers instead of the open-loop drivetrain.
EDGE_ROWS = (
    "Forward",
    "Reverse",
    "Spin left",
    "Spin right",
    "Steer, yaw loop",
    "Steer, yaw loop, reversing",
)


@dataclass
class Plateau:
    start: int  # grid index, inclusive
    end: int  # exclusive
    mean: float
    other_mean: float  # the other channel's mean over the same samples
    loop_frac: float  # fraction of samples with the yaw loop on

    def seconds(self, dt: float) -> float:
        return (self.end - self.start) * dt


@dataclass
class Event:
    """One labeled motion. ``t0``/``t1`` bound the samples it rests on, in grid seconds."""

    id: str
    kind: str  # hold | arc | spin | edge | slew | creep
    t0: float
    t1: float
    text: str
    informs: list[str] = field(default_factory=list)
    edge_cell: tuple[int, int] | None = None  # (row, col) in the transition table
    hold_cell: tuple[float, float] | None = None  # (throttle, steer) mean of a held plateau
    hold_s: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "kind": self.kind,
            "t0": round(self.t0, 3),
            "t1": round(self.t1, 3),
            "text": self.text,
            "informs": self.informs,
        }


def describe(v: float, ch: int) -> str:
    if abs(v) < REST:
        return "centered"
    word = ("forward" if v > 0 else "reverse") if ch == 0 else ("left" if v > 0 else "right")
    return f"{round(abs(v) * 100)}% {word}"


def bin_of(u: float) -> int:
    return int(np.argmin([abs(u - b) for b in BINS]))


def amp_index(amp: float) -> int:
    return int(np.argmin([abs(amp - a) for a in AMPS]))


class _Channel:
    """Plateau tracking for one stick channel."""

    def __init__(self) -> None:
        self.start = 0
        self.lo = 0.0
        self.hi = 0.0
        self.sum = 0.0
        self.other_sum = 0.0
        self.loop_sum = 0.0
        self.last_long: Plateau | None = None
        self.edge_pending = True  # the open plateau has not yet emitted its edge


class ManeuverSegmenter:
    """Streaming labels for a uniform stick grid. ``push`` returns the events it finished."""

    def __init__(self, dt: float = 0.01) -> None:
        self.dt = dt
        self.hold_n = int(round(HOLD_S / dt))
        self.k = 0
        self.ch = [_Channel(), _Channel()]
        self.history: deque[tuple[float, float]] = deque(maxlen=int(round(6.0 / dt)))
        # 2-D plateau for the coverage map's "held" rings.
        self._held_start = 0
        self._held_ref = (0.0, 0.0)
        self._held_loop = 0.0

    def push(self, throttle: float, steer: float, loop_on: bool) -> list[Event]:
        k = self.k
        events: list[Event] = []
        u = (float(throttle), float(steer))
        if k == 0:
            for c, ch in enumerate(self.ch):
                ch.lo = ch.hi = ch.sum = u[c]
                ch.other_sum = u[1 - c]
                ch.loop_sum = float(loop_on)
            self._held_ref = u
        else:
            for c, ch in enumerate(self.ch):
                lo, hi = min(ch.lo, u[c]), max(ch.hi, u[c])
                if hi - lo > PLATEAU_TOL:
                    events += self._close(c, k)
                    ch.start = k
                    ch.lo = ch.hi = ch.sum = u[c]
                    ch.other_sum = u[1 - c]
                    ch.loop_sum = float(loop_on)
                    ch.edge_pending = True
                else:
                    ch.lo, ch.hi = lo, hi
                    ch.sum += u[c]
                    ch.other_sum += u[1 - c]
                    ch.loop_sum += float(loop_on)
                events += self._maybe_edge(c, k + 1)
            events += self._held(k, u, loop_on)
            events += self._creep(k, u)
        self.history.append(u)
        self.k = k + 1
        return events

    # -- plateaus, holds, edges ---------------------------------------------------------

    def _plateau(self, c: int, end: int) -> Plateau:
        ch = self.ch[c]
        n = max(end - ch.start, 1)
        return Plateau(ch.start, end, ch.sum / n, ch.other_sum / n, ch.loop_sum / n)

    def _is_long(self, p: Plateau, rest_ok: bool) -> bool:
        return p.end - p.start >= self.hold_n or (rest_ok and abs(p.mean) < REST)

    def _close(self, c: int, end: int) -> list[Event]:
        p = self._plateau(c, end)
        out: list[Event] = []
        long_enough = p.end - p.start >= self.hold_n
        if long_enough and abs(p.mean) >= EDGE_MIN:
            hold = self._hold_event(c, p)
            if hold is not None:
                out.append(hold)
        if self._is_long(p, rest_ok=long_enough or p.start == 0):
            self.ch[c].last_long = p
        return out

    def _maybe_edge(self, c: int, end: int) -> list[Event]:
        """Emit the edge into the open plateau once it is long enough to count as the target."""
        ch = self.ch[c]
        if not ch.edge_pending or ch.last_long is None or end - ch.start < self.hold_n:
            return []
        ch.edge_pending = False
        a, b = ch.last_long, self._plateau(c, end)
        if abs(b.mean - a.mean) < EDGE_MIN:
            return []
        return [self._edge_event(c, a, b)]

    def _edge_event(self, c: int, a: Plateau, b: Plateau) -> Event:
        dt = self.dt
        dur = (b.start - a.end) * dt
        at = a.end * dt
        accel = abs(b.mean) > abs(a.mean)
        clean = dur <= SNAP_S + 1e-9
        if abs(b.mean) < 0.05:
            what = f"{CHANNELS[c]} snap release from {describe(a.mean, c)}"
        elif abs(a.mean) < 0.05:
            what = f"{CHANNELS[c]} snap to {describe(b.mean, c)}"
        else:
            what = f"{CHANNELS[c]} {describe(a.mean, c)} to {describe(b.mean, c)}"
        val = b.mean if accel else a.mean
        cell: tuple[int, int] | None = None
        loop = b.loop_frac > 0.5 if accel else a.loop_frac > 0.5
        if abs(val) >= 0.15:
            col = (0 if accel else 4) + amp_index(abs(val))
            if c == 0:
                row = 0 if val > 0 else 1
            elif loop:
                reversing = (b.other_mean if accel else a.other_mean) < -0.1
                row = 5 if reversing else 4
            else:
                row = 2 if val > 0 else 3
            cell = (row, col)
        if not clean:
            return Event(
                id=f"e{c}-{a.end}",
                kind="slew",
                t0=at,
                t1=b.start * dt,
                text=(
                    f"{CHANNELS[c]} slewed {describe(a.mean, c)} to {describe(b.mean, c)} over "
                    f"{round(dur * 1000)} ms. Not used for lag; snap it next time."
                ),
            )
        if c == 1 and loop:
            informs = ["yaw loop", "delay"] if accel else ["yaw loop", "yaw decel lag"]
            what += ", yaw loop on"
        elif accel:
            informs = ["delay", "yaw accel lag" if c else "linear accel lag"]
        else:
            informs = ["yaw decel lag" if c else "linear decel lag"]
        return Event(
            id=f"e{c}-{a.end}",
            kind="edge",
            t0=max(a.end - 5, a.start) * dt,
            t1=(b.start + 5) * dt,
            text=what,
            informs=informs,
            edge_cell=None if cell is None else cell,
        )

    def _hold_event(self, c: int, p: Plateau) -> Event | None:
        dt = self.dt
        dur = f"{p.seconds(dt):.1f} s"
        om = p.other_mean
        loop = " with the yaw loop on" if p.loop_frac > 0.5 else ""
        if c == 0 and abs(om) < ARC_STEER:
            hands_off = p.seconds(dt) >= 1.0 and abs(om) < HANDS_OFF
            informs = ["forward gain" if p.mean > 0 else "reverse gain"]
            if hands_off:
                informs.append("drift")
            kind, text = (
                "hold",
                (
                    f"{'Forward' if p.mean > 0 else 'Reverse'} hold {round(abs(p.mean) * 100)}%, "
                    f"{dur}{loop}"
                ),
            )
        elif c == 0:
            kind, text = (
                "arc",
                (
                    f"Arc {'forward' if p.mean > 0 else 'reverse'}-"
                    f"{'left' if om > 0 else 'right'}, "
                    f"{round(abs(p.mean) * 100)}% throttle with {round(abs(om) * 100)}% steer, "
                    f"{dur}{loop}"
                ),
            )
            informs = ["steer-brake", "angular droop"]
        elif abs(om) < ARC_STEER:
            kind, text = (
                "spin",
                (
                    f"Spin {'left' if p.mean > 0 else 'right'} "
                    f"{round(abs(p.mean) * 100)}%, {dur}{loop}"
                ),
            )
            informs = ["yaw gain"]
        else:
            return None  # an arc, reported from the throttle side
        return Event(
            id=f"h{c}-{p.end}",
            kind=kind,
            t0=p.start * dt,
            t1=p.end * dt,
            text=text,
            informs=informs,
        )

    # -- 2-D held cells and creeps ------------------------------------------------------

    def _held(self, k: int, u: tuple[float, float], loop_on: bool) -> list[Event]:
        ref = self._held_ref
        if abs(u[0] - ref[0]) <= PLATEAU_TOL and abs(u[1] - ref[1]) <= PLATEAU_TOL:
            return []
        out: list[Event] = []
        n = k - self._held_start
        if n >= self.hold_n and (abs(ref[0]) > REST or abs(ref[1]) > REST):
            out.append(
                Event(
                    id=f"p-{k}",
                    kind="plateau",
                    t0=self._held_start * self.dt,
                    t1=k * self.dt,
                    text="",
                    hold_cell=ref,
                    hold_s=n * self.dt,
                )
            )
        self._held_start = k
        self._held_ref = u
        return out

    def _creep(self, k: int, u: tuple[float, float]) -> list[Event]:
        out: list[Event] = []
        if not self.history:
            return out
        prev = self.history[-1]
        hist = list(self.history) + [u]
        for c in (0, 1):
            if not (abs(u[c]) >= CREEP_PAST > abs(prev[c])):
                continue
            j = len(hist) - 1
            i = j - 1
            while (
                i > 0
                and abs(hist[i][c]) >= CREEP_FROM
                and abs(hist[i][c]) <= abs(hist[i + 1][c]) + 0.03
            ):
                i -= 1
            span = (j - i) * self.dt
            if abs(hist[i][c]) >= CREEP_FROM or span < CREEP_MIN_S:
                continue
            if c == 0:
                direction = "forward" if u[c] > 0 else "reverse"
                name = f"{direction} deadzone"
            else:
                direction = "left" if u[c] > 0 else "right"
                name = f"spin-{direction} deadzone"
            out.append(
                Event(
                    id=f"c{c}-{k}",
                    kind="creep",
                    t0=(k - (j - i)) * self.dt,
                    t1=k * self.dt,
                    text=(
                        f"Slow {'steer' if c else 'throttle'} creep {direction} out of zero, "
                        f"{span:.1f} s"
                    ),
                    informs=[name],
                )
            )
        return out


def edges_of(
    throttle: np.ndarray, steer: np.ndarray, loop_on: np.ndarray, dt: float
) -> list[Event]:
    """Every clean edge event in a stick trace, for scoring a suggested motion."""
    seg = ManeuverSegmenter(dt)
    out: list[Event] = []
    for a, b, lp in zip(throttle, steer, loop_on):
        out += seg.push(float(a), float(b), bool(lp))
    # Close the trailing plateaus so a final hold still yields its edge.
    for _ in range(seg.hold_n + 1):
        out += seg.push(float(throttle[-1]), float(steer[-1]), bool(loop_on[-1]))
    return [e for e in out if e.kind == "edge"]


# ---------------------------------------------------------------------------
# Offline session menu (check_sysid_recordings.py)
# ---------------------------------------------------------------------------

MOVING = 0.05  # |stick| above this counts as driving
HOLD_TOL = 0.03
HOLD_MIN_S = 0.5
CREEP_MAX = 0.08
PUNCH_RISE_S = 0.15
PUNCH_LEVEL = 0.5
STOP_FROM = 0.2
LEVEL_BINS = (0.05, 0.15, 0.3, 0.5, 0.75, 1.01)


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    if not mask.any():
        return []
    edges = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def held_segments(u: np.ndarray, gate: np.ndarray, grid_s: float) -> list[tuple[int, int, float]]:
    """(start, stop, mean level) for stretches where u stays within HOLD_TOL of its start."""
    out = []
    for a, b in _runs(gate):
        i = a
        while i < b:
            j = i + 1
            while j < b and abs(u[j] - u[i]) <= HOLD_TOL:
                j += 1
            if (j - i) * grid_s >= HOLD_MIN_S:
                out.append((i, j, float(np.mean(u[i:j]))))
            i = j
    return out


def menu_from_grid(lin: np.ndarray, ang: np.ndarray, grid_s: float) -> dict[str, Any]:
    """Session totals from linear and angular stick fractions on a uniform grid."""
    alin, aang = np.abs(lin), np.abs(ang)
    straight = (alin > MOVING) & (aang < HOLD_TOL)
    spin = (aang > MOVING) & (alin < HOLD_TOL)
    arc = (alin > MOVING) & (aang > MOVING)

    holds = held_segments(lin, straight, grid_s)
    levels: dict[str, float] = {}
    for a, b, level in holds:
        k = int(np.searchsorted(LEVEL_BINS, abs(level), side="right")) - 1
        k = min(max(k, 0), len(LEVEL_BINS) - 2)
        key = f"{'+' if level > 0 else '-'}{LEVEL_BINS[k]:.2f}"
        levels[key] = levels.get(key, 0.0) + (b - a) * grid_s
    spins = held_segments(ang, spin, grid_s)

    rise = int(round(PUNCH_RISE_S / grid_s))
    punches = int(
        np.sum(
            (alin[rise:] > PUNCH_LEVEL)
            & (alin[:-rise] < MOVING)
            & (alin[rise - 1 : -1] <= PUNCH_LEVEL)
        )
    )
    stops = int(
        np.sum(
            (alin[rise:] < HOLD_TOL)
            & (alin[:-rise] > STOP_FROM)
            & (alin[rise - 1 : -1] >= HOLD_TOL)
        )
    )
    sign = np.sign(np.where(alin > 0.15, lin, 0.0))
    nz = np.flatnonzero(sign != 0)
    reversals = (
        int(np.sum((np.diff(sign[nz]) != 0) & (np.diff(nz) * grid_s <= 0.3))) if len(nz) > 1 else 0
    )
    creep = float(np.sum((alin > 0.01) & (alin < CREEP_MAX) & (aang < HOLD_TOL)) * grid_s)
    near_center = float(np.sum((alin > 0.005) & (alin < 0.02)) * grid_s)

    return {
        "driving_s": float(np.sum((alin > MOVING) | (aang > MOVING)) * grid_s),
        "total_s": len(lin) * grid_s,
        "hold_s_fwd": float(sum((b - a) * grid_s for a, b, v in holds if v > 0)),
        "hold_s_rev": float(sum((b - a) * grid_s for a, b, v in holds if v < 0)),
        "holds_1s": int(sum((b - a) * grid_s >= 1.0 for a, b, _ in holds)),
        "hold_levels_s": dict(sorted(levels.items())),
        "creep_s": creep,
        "jitter_near_center_s": near_center,
        "spin_s_left": float(sum((b - a) * grid_s for a, b, v in spins if v > 0)),
        "spin_s_right": float(sum((b - a) * grid_s for a, b, v in spins if v < 0)),
        "spin_levels": sorted({round(abs(v), 1) for _a, _b, v in spins}),
        "arc_s": float(np.sum(arc) * grid_s),
        "reversals": reversals,
        "punches": punches,
        "hard_stops": stops,
        "max_linear": float(alin.max()),
        "max_angular": float(aang.max()),
    }
