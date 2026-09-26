"""Fit windows for the drivetrain sysid: gating and initial state (plan "Data pipeline" 3 and 4).

Input is the smoothed grid from ``tag_pose_smoother.smooth_session``. Three per-sample gates run
first and cut the session into spans:

- **near_rail**: the axle center within ``robot_length_m`` of a box rail, where wall contact is
  possible.
- **esp32_gap**: inside a gap in the ESP32 stream longer than ``esp32_max_gap_s``, or outside the
  stream altogether, since the fit has no command tape there.
- **flip**: the upright/inverted state changes, which the fit cannot start or end across.

Each span shorter than ``window_s`` is dropped; the rest are cut into ``floor(L / window_s)``
equal windows, so every window is between ``window_s`` and ``2 * window_s`` long (1 to 2 s by
default). A window is then dropped if accepted tag detections cover less than
``coverage_min`` of its camera frames. A kept window is ``nose_lift`` when its pitch leaves the
rest pitch by more than ``pitch_limit_deg`` anywhere, otherwise ``flat``.

Initial state per window is the smoothed state at its first sample. Wheel speeds come from the
no-slip relation in the body frame, ``(v -/+ w * track_half_width) / wheel_radius`` for left
and right, with v the forward speed along body x and w the yaw rate about body z. A wheel
spinning to roll the robot forward is positive, which is positive rotation about body +y. On an
inverted robot body z points down, so the body-frame yaw rate is minus the field one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
import pandas as pd

from auto_battlebot.perception.tag_pose_smoother import RobotGeometry

_NS = 1_000_000_000

WINDOW_COLUMNS = [
    "window_id",
    "start_ns",
    "end_ns",
    "kind",
    "coverage",
    "x0",
    "y0",
    "yaw0",
    "vx0",
    "vy0",
    "yaw_rate0",
    "pitch0",
    "wheel_left0",
    "wheel_right0",
]


@dataclass
class WindowOptions:
    window_s: float = 1.0
    coverage_min: float = 0.7
    box_half_extent_m: float = 0.76  # the 1.52 m test box
    box_center_xy: tuple[float, float] = (0.0, 0.0)
    robot_length_m: float = 0.15
    rest_pitch_rad: float = 0.198
    rest_pitch_upside_down_rad: float = 0.0
    pitch_limit_deg: float = 4.0
    esp32_max_gap_s: float = 0.05


def wheel_speeds(
    vx: np.ndarray,
    vy: np.ndarray,
    yaw: np.ndarray,
    yaw_rate: np.ndarray,
    upside_down: np.ndarray,
    geometry: RobotGeometry,
) -> tuple[np.ndarray, np.ndarray]:
    """(left, right) wheel speeds in rad/s from field-frame velocity, no-slip."""
    v = np.asarray(vx) * np.cos(yaw) + np.asarray(vy) * np.sin(yaw)
    w_body = np.where(np.asarray(upside_down, dtype=bool), -1.0, 1.0) * np.asarray(yaw_rate)
    left = (v - w_body * geometry.track_half_width_m) / geometry.wheel_radius_m
    right = (v + w_body * geometry.track_half_width_m) / geometry.wheel_radius_m
    return np.asarray(left), np.asarray(right)


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """(first, last) index ranges where ``mask`` is True."""
    if mask.size == 0:
        return []
    padded = np.concatenate([[False], mask, [False]]).astype(np.int8)
    edges = np.diff(padded)
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1) - 1
    return [(int(s), int(e)) for s, e in zip(starts, ends)]


def esp32_coverage_mask(
    grid_ns: np.ndarray, esp32_stamp_ns: np.ndarray, max_gap_s: float
) -> np.ndarray:
    """True where the grid sample lies inside the ESP32 stream and not inside a long gap."""
    t = np.sort(np.asarray(esp32_stamp_ns, dtype=np.int64))
    ok = np.zeros(grid_ns.shape, dtype=bool)
    if t.size < 2:
        return ok
    idx = np.searchsorted(t, grid_ns, side="right")
    inside = (idx > 0) & (idx < t.size)
    i = np.clip(idx, 1, t.size - 1)
    gap = (t[i] - t[i - 1]) / _NS
    ok[inside] = gap[inside] <= max_gap_s
    return ok


def make_windows(
    grid: pd.DataFrame,
    frame_stamps_ns: np.ndarray,
    accepted_stamps_ns: np.ndarray,
    esp32_stamps_ns: np.ndarray,
    geometry: RobotGeometry,
    options: WindowOptions | None = None,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    """Gate and cut windows. Returns the windows table and a report of what each gate dropped.

    ``grid`` needs the ``smoothed.csv`` columns. ``frame_stamps_ns`` are every camera frame's
    stamp (frames without a detection included) and ``accepted_stamps_ns`` the stamps of tag
    detections that passed the innovation gate.
    """
    opts = options or WindowOptions()
    grid_ns = grid["stamp_ns"].to_numpy().astype(np.int64)
    n = grid_ns.size
    dt_s = float(np.median(np.diff(grid_ns))) / _NS if n > 1 else 0.0
    x = grid["x"].to_numpy()
    y = grid["y"].to_numpy()
    inverted = grid["upside_down"].to_numpy().astype(bool)

    limit = opts.box_half_extent_m - opts.robot_length_m
    near_rail = (
        np.maximum(np.abs(x - opts.box_center_xy[0]), np.abs(y - opts.box_center_xy[1])) > limit
    )
    esp32_ok = esp32_coverage_mask(grid_ns, esp32_stamps_ns, opts.esp32_max_gap_s)
    flip = np.zeros(n, dtype=bool)
    if n > 1:
        changed = np.flatnonzero(inverted[1:] != inverted[:-1])
        flip[changed] = True
        flip[changed + 1] = True

    report: dict[str, Any] = {
        "gate_drop_seconds": {
            "near_rail": float(near_rail.sum() * dt_s),
            "esp32_gap": float((~esp32_ok & ~near_rail).sum() * dt_s),
            "flip": float((flip & esp32_ok & ~near_rail).sum() * dt_s),
        },
        "windows_dropped": {"span_too_short": 0, "coverage": 0},
        "windows": {"flat": 0, "nose_lift": 0},
        "options": {
            "window_s": opts.window_s,
            "coverage_min": opts.coverage_min,
            "box_half_extent_m": opts.box_half_extent_m,
            "box_center_xy": list(opts.box_center_xy),
            "robot_length_m": opts.robot_length_m,
            "pitch_limit_deg": opts.pitch_limit_deg,
            "rest_pitch_rad": opts.rest_pitch_rad,
            "esp32_max_gap_s": opts.esp32_max_gap_s,
        },
    }
    usable = ~near_rail & esp32_ok & ~flip
    frames = np.sort(np.asarray(frame_stamps_ns, dtype=np.int64))
    accepted = np.unique(np.asarray(accepted_stamps_ns, dtype=np.int64))
    per_window = max(int(round(opts.window_s / dt_s)), 1) if dt_s > 0 else 1
    rows: list[dict[str, Any]] = []
    for first, last in _runs(usable):
        length = last - first + 1
        pieces = length // per_window
        if pieces == 0:
            report["windows_dropped"]["span_too_short"] += 1
            continue
        edges = np.linspace(first, last + 1, pieces + 1).round().astype(np.int64)
        for a, b in zip(edges[:-1], edges[1:]):
            row = _window_row(grid, int(a), int(b) - 1, frames, accepted, geometry, opts)
            if row is None:
                continue
            if row["coverage"] < opts.coverage_min:
                report["windows_dropped"]["coverage"] += 1
                continue
            report["windows"][row["kind"]] += 1
            row["window_id"] = len(rows)
            rows.append(row)
    return pd.DataFrame(rows, columns=WINDOW_COLUMNS), report


def _window_row(
    grid: pd.DataFrame,
    first: int,
    last: int,
    frames: np.ndarray,
    accepted: np.ndarray,
    geometry: RobotGeometry,
    opts: WindowOptions,
) -> dict[str, Any] | None:
    start_ns = int(grid["stamp_ns"].iloc[first])
    end_ns = int(grid["stamp_ns"].iloc[last])
    n_frames = int(np.searchsorted(frames, end_ns, "right") - np.searchsorted(frames, start_ns))
    if n_frames == 0:
        return None
    n_hits = int(np.searchsorted(accepted, end_ns, "right") - np.searchsorted(accepted, start_ns))
    s0 = grid.iloc[first]
    inverted = bool(s0["upside_down"])
    rest = opts.rest_pitch_upside_down_rad if inverted else opts.rest_pitch_rad
    pitch = grid["pitch"].to_numpy()[first : last + 1]
    lifted = np.max(np.abs(pitch - rest)) > math.radians(opts.pitch_limit_deg)
    left, right = wheel_speeds(
        np.array([s0["vx"]]),
        np.array([s0["vy"]]),
        np.array([s0["yaw"]]),
        np.array([s0["yaw_rate"]]),
        np.array([inverted]),
        geometry,
    )
    return {
        "start_ns": start_ns,
        "end_ns": end_ns,
        "kind": "nose_lift" if lifted else "flat",
        "coverage": min(n_hits / n_frames, 1.0),
        "x0": float(s0["x"]),
        "y0": float(s0["y"]),
        "yaw0": float(s0["yaw"]),
        "vx0": float(s0["vx"]),
        "vy0": float(s0["vy"]),
        "yaw_rate0": float(s0["yaw_rate"]),
        "pitch0": float(s0["pitch"]),
        "wheel_left0": float(left[0]),
        "wheel_right0": float(right[0]),
    }
