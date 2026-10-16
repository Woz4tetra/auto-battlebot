"""Generate the golden fixture that holds the C++ opponent filter to its Python mirror.

Runs a scripted opponent through `auto_battlebot.perception.opponent_filter` the way the app
drives KalmanMotionEstimator: coast() every 4 ms control tick, update() when a frame's detections
arrive 63 ms after its shutter (so most measurements rewind). Writes the events and the mirror's
outputs as a C++ header; `tests/test_kalman_motion_estimator.cpp` replays the events through the
C++ estimator and must match to 1e-6, and `tests/python/test_opponent_filter.py` reruns the
mirror against the same header.

The scenario covers: a blob-born track (heading unknown), the first keypoint heading, arcs at
both turn directions, a single heading flip, a run of three flips that re-initializes heading and
the run back, a measurement late enough to miss the snapshot ring, a 0.5 s dropout past
opponent_max_coast_s, a position jump the gate rejects until it re-initializes, and a final coast
into the +x wall.

Usage:
    source scripts/activate_python.sh
    python playground/opponent_prediction/make_opponent_filter_golden_fixture.py
"""

from __future__ import annotations

import math
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

from auto_battlebot.perception.opponent_filter import OpponentFilter, OpponentFilterConfig

REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_HEADER = REPO_ROOT / "tests/robot_filter/opponent_filter_golden_data.hpp"
PLANT_GOLDEN = REPO_ROOT / "tests/plant/plant_golden_data.hpp"

T0 = 100.0
FRAME_DT = 1.0 / 30.0
TICK_DT = 0.004
LATENCY_S = 0.063
NUM_FRAMES = 108
END_S = NUM_FRAMES * FRAME_DT + 0.45
FIELD_SIZE = 4.0
MARGIN = 0.2
KEYPOINT_SIGMA = 0.03  # KalmanMotionEstimatorConfiguration defaults; per-meter term off
BLOB_SIGMA = 0.10
MAX_COAST_S = 0.2
COAST_RECORD_STRIDE = 10
SEED = 7

BLOB_FRAMES = set(range(5))
FLIP_FRAMES = {30, 45, 46, 47}
LATE_FRAME = 52
LATE_LATENCY_S = 0.30
DROPPED_FRAMES = set(range(53, 61)) | set(range(70, 85)) | set(range(100, NUM_FRAMES))
JUMP_FRAMES = set(range(88, 93))
JUMP_Y = 0.8

KIND_COAST = 0
KIND_MEASURED = 1
KIND_EMPTY = 2
RENDER_FIELDS = (
    "x",
    "y",
    "yaw",
    "vx",
    "vy",
    "omega",
    "position_sigma_m",
    "heading_sigma_rad",
    "yaw_rate_sigma_rad_s",
    "is_stale",
)
UPPER = [(i, j) for i in range(6) for j in range(i, 6)]


def plant_delay_s() -> float:
    """The render lead the C++ test's plant carries (mrs_buff_mk3_plant uses this value)."""
    match = re.search(r"kParam_delay_s = ([0-9.eE+-]+);", PLANT_GOLDEN.read_text())
    if match is None:
        raise RuntimeError(f"kParam_delay_s not found in {PLANT_GOLDEN}")
    return float(match.group(1))


def config() -> OpponentFilterConfig:
    return OpponentFilterConfig(opponent_max_coast_s=MAX_COAST_S, render_lead_s=plant_delay_s())


def truth_at_frames() -> np.ndarray:
    """(NUM_FRAMES, 3) x, y, heading of a unicycle: left arc, right arc, then straight at +x."""
    x, y, heading = -0.6, -0.9, 0.6
    out = []
    t = 0.0
    for _ in range(NUM_FRAMES):
        out.append((x, y, heading))
        if t < 1.0:
            v, w = 1.0, 1.2
        elif t < 2.0:
            v, w = 1.0, -1.5
        else:
            v, w = 1.5, -3.0 * math.atan2(math.sin(heading), math.cos(heading))
        sub = FRAME_DT / 10
        for _ in range(10):
            x += v * math.cos(heading) * sub
            y += v * math.sin(heading) * sub
            heading += w * sub
        t += FRAME_DT
    return np.asarray(out)


@dataclass
class Event:
    kind: int
    time: float  # coast: control-clock now; update: the measurement's shutter stamp
    deliver: float
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    keypoint: int = 0


def scenario() -> list[Event]:
    rng = np.random.default_rng(SEED)
    truth = truth_at_frames()
    events: list[Event] = []
    for k in range(NUM_FRAMES):
        stamp = T0 + k * FRAME_DT
        latency = LATE_LATENCY_S if k == LATE_FRAME else LATENCY_S
        if k in DROPPED_FRAMES:
            events.append(Event(KIND_EMPTY, stamp, stamp + latency))
            continue
        x, y, heading = truth[k]
        x += rng.normal(0.0, 0.01)
        y += rng.normal(0.0, 0.01)
        yaw = heading + rng.normal(0.0, 0.05)
        if k in JUMP_FRAMES:
            y += JUMP_Y
        if k in FLIP_FRAMES:
            yaw += math.pi
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        if k in BLOB_FRAMES:
            events.append(Event(KIND_MEASURED, stamp, stamp + latency, x, y, 0.0, 0))
        else:
            events.append(Event(KIND_MEASURED, stamp, stamp + latency, x, y, yaw, 1))
    num_ticks = int(round(END_S / TICK_DT))
    for i in range(num_ticks):
        now = T0 + i * TICK_DT
        events.append(Event(KIND_COAST, now, now))
    # Updates land before the coast tick that shares their delivery time.
    events.sort(key=lambda e: (round(e.deliver, 9), 0 if e.kind != KIND_COAST else 1))
    return events


@dataclass
class Records:
    events: list[Event]
    recorded: list[int]
    render: list[list[float]]
    state: list[list[float]]  # update events only: 6 state, 21 upper covariance, stamp, heading


def run_mirror(events: list[Event]) -> Records:
    half = torch.tensor([[FIELD_SIZE / 2, FIELD_SIZE / 2]], dtype=torch.float64)
    filt = OpponentFilter(1, config(), field_half_extent=half, margin=MARGIN)
    recorded: list[int] = []
    render: list[list[float]] = []
    state: list[list[float]] = []
    coast_count = 0
    for ev in events:
        if ev.kind == KIND_COAST:
            out = filt.coast(ev.time)
            coast_count += 1
            if not bool(out.active[0]) or coast_count % COAST_RECORD_STRIDE != 0:
                recorded.append(0)
                continue
        else:
            measured = torch.tensor([ev.kind == KIND_MEASURED])
            variance = (KEYPOINT_SIGMA if ev.keypoint else BLOB_SIGMA) ** 2
            out, _ = filt.update(
                ev.time,
                measured,
                torch.tensor([[ev.x, ev.y]], dtype=torch.float64),
                torch.tensor([ev.yaw], dtype=torch.float64),
                torch.tensor([bool(ev.keypoint)]),
                variance,
            )
            if not bool(out.active[0]):
                recorded.append(0)
                continue
            p = filt.p[0]
            state.append(
                [float(v) for v in filt.x[0]]
                + [float(p[i, j]) for i, j in UPPER]
                + [float(filt.stamp[0]), float(filt.heading_initialized[0])]
            )
        recorded.append(1)
        render.append([float(getattr(out, name)[0]) for name in RENDER_FIELDS])
    return Records(events, recorded, render, state)


def fmt(value: float) -> str:
    if math.isinf(value):
        return "kInf" if value > 0 else "-kInf"
    return f"{value:.17g}"


def emit_array(ctype: str, name: str, values: list[float] | list[int], per_line: int) -> str:
    text = [fmt(v) if ctype == "double" else str(int(v)) for v in values]
    body = ",\n    ".join(", ".join(text[i : i + per_line]) for i in range(0, len(text), per_line))
    return f"inline constexpr {ctype} {name}[] = {{\n    {body},\n}};\n"


def clang_format(text: str) -> str:
    """The header as scripts/lint will want it: the pinned clang-format, when installed."""
    version = (REPO_ROOT / ".llvm-version").read_text().strip()
    binary = shutil.which(f"clang-format-{version}")
    if binary is None:
        print(f"clang-format-{version} not found; the header is left unformatted")
        return text
    result = subprocess.run(
        [binary, f"--assume-filename={OUT_HEADER}"],
        input=text,
        capture_output=True,
        text=True,
        check=True,
        cwd=REPO_ROOT,
    )
    return result.stdout


def main() -> None:
    records = run_mirror(scenario())
    ev = records.events
    parts = [
        "#pragma once\n\n",
        "// Generated by playground/opponent_prediction/make_opponent_filter_golden_fixture.py.\n",
        "// Do not edit by hand: change the C++ opponent arm and the Python mirror together,\n",
        "// then regenerate.\n\n",
        "#include <limits>\n\n",
        "namespace auto_battlebot::opponent_filter_golden {\n\n",
        "inline constexpr double kInf = std::numeric_limits<double>::infinity();\n",
        f"inline constexpr double kFieldSize = {fmt(FIELD_SIZE)};\n",
        f"inline constexpr double kMargin = {fmt(MARGIN)};\n",
        f"inline constexpr double kMaxCoastS = {fmt(MAX_COAST_S)};\n",
        f"inline constexpr double kRenderLeadS = {fmt(plant_delay_s())};\n",
        f"inline constexpr int kNumEvents = {len(ev)};\n",
        f"inline constexpr int kNumRecorded = {len(records.render)};\n",
        f"inline constexpr int kNumStateRecords = {len(records.state)};\n",
        f"inline constexpr int kRenderWidth = {len(RENDER_FIELDS)};\n",
        f"inline constexpr int kStateWidth = {len(records.state[0])};\n",
        f"inline constexpr int kKindCoast = {KIND_COAST};\n",
        f"inline constexpr int kKindMeasured = {KIND_MEASURED};\n",
        f"inline constexpr int kKindEmpty = {KIND_EMPTY};\n\n",
        "// Render rows: " + ", ".join(RENDER_FIELDS[:5]) + ",\n",
        "// " + ", ".join(RENDER_FIELDS[5:]) + ".\n",
        "// State rows: 6 state, the covariance upper triangle row by row, stamp,\n",
        "// heading_initialized. One per recorded update event, in event order.\n\n",
        emit_array("int", "kEventKind", [e.kind for e in ev], 24),
        emit_array("double", "kEventTime", [e.time for e in ev], 4),
        emit_array("double", "kMeasX", [e.x for e in ev], 4),
        emit_array("double", "kMeasY", [e.y for e in ev], 4),
        emit_array("double", "kMeasYaw", [e.yaw for e in ev], 4),
        emit_array("int", "kMeasKeypoint", [e.keypoint for e in ev], 24),
        emit_array("int", "kRecorded", records.recorded, 24),
        emit_array("double", "kRender", [v for row in records.render for v in row], 5),
        emit_array("double", "kState", [v for row in records.state for v in row], 4),
        "\n}  // namespace auto_battlebot::opponent_filter_golden\n",
    ]
    OUT_HEADER.parent.mkdir(parents=True, exist_ok=True)
    OUT_HEADER.write_text(clang_format("".join(parts)))
    print(f"wrote {OUT_HEADER}: {len(ev)} events, {len(records.render)} recorded")


if __name__ == "__main__":
    main()
