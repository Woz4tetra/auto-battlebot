"""Check Mr Stabs Mk2 drive recordings before they go into the MuJoCo plant fit.

Reads each MCAP once and reports, per recording, whether it can feed smooth_tag_poses.py and
fit_mujoco_plant.py, and what is missing if it cannot. Nothing here fits anything: it is the
look you take the morning after a session, before deciding whether to drive again.

Checks, each PASS / WARN / FAIL:

- file: the MCAP has a summary. A missing one means it was copied while still recording
  (re-download it) or the app died mid-write.
- camera: frame rate from /camera/frame_meta (60 fps target) and the longest gap.
- video: /camera/video against /camera/frame_meta. The ZED X One records its footage into the
  MCAP, not an SVO, so this is the only copy to replay.
- field: a field -> camera transform exists and stays put. The tripod does not move, so more
  than 5 mm or 0.5 deg of wander means a re-init or a bumped tripod partway through.
- robot tags: share of camera frames with a robot AprilTag, overall and while the sticks are
  off center, and the tag's pixel size. The smoother's window gate is 70% coverage.
  Recordings from the match profile carry no tags. They need /camera/video replayed through the
  sysid keypoint model first (config/playback/mr_stabs_mk2_sysid_playback.toml).
- our-robot track: share of frames with an OUR_ROBOT marker (YOLO path, match profile).
- ESP32 stream: events against the firmware loop rate, gaps, pack voltage. The fit's command tape
  is the per-motor left_cmd/right_cmd from this stream, so without it there is no command tape.
- sticks: the maneuver menu, measured from the radio's stick channels. Seconds of straight holds
  per level and direction, creep, spins, arcs, reversals, punches, and hard stops.
- SVO (ZED 2i only, with --svo-dir): an SVO whose frame stamps overlap this recording, and how
  many frames it holds.

    python playground/calibration/check_sysid_recordings.py data/recordings/*2026-10-11*.mcap \\
        --svo-dir data/svo --plots playground/calibration/out/mujoco_sysid/checks

Exit status is 1 if any recording has a FAIL, so it can gate a script.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from mcap.reader import make_reader

from auto_battlebot.recording import diag_io
from auto_battlebot.recording.mcap_io import (
    CAMERA_VIDEO_TOPIC,
    FRAME_META_TOPIC,
    decode_json,
    iter_messages,
)
from auto_battlebot.recording.svo2 import read_frame_stamps
from auto_battlebot.recording.sysid_io import (
    field_transform_constancy,
    load_esp32_diagnostics,
    load_field_from_camera,
    load_robot_tags,
    load_transmitter_channels,
    rotation_angle,
)

# Stick channels, from the sysid and zed_box profiles: CH1 linear, CH2 angular, +/-1024 full scale.
LINEAR_CH = 0
ANGULAR_CH = 1
STICK_FULL = 1024.0
GRID_S = 0.01

# Thresholds.
CAMERA_FPS_PASS = 55.0
CAMERA_GAP_WARN_S = 0.1
CAMERA_GAP_FAIL_S = 1.0  # a stall this long splits the session for the smoother anyway
VIDEO_COVERAGE_PASS = 0.99  # /camera/video messages per /camera/frame_meta
FIELD_DRIFT_M = 0.005
FIELD_DRIFT_DEG = 0.5
TAG_COVERAGE_PASS = 0.7  # smoother WindowOptions.coverage_min
TAG_COVERAGE_WARN = 0.5
TAG_MIN_PX = 40.0  # plan: at least ~40 px on the far side
ESP32_RATE_PASS = 0.95
ESP32_GAP_S = 0.05
VBAT_MIN_V = 9.0  # anything below this on a 4S pack is a bad reading, not a flat pack
MOVING = 0.05  # |stick| above this counts as driving

# Maneuver detection, stick fractions and seconds.
HOLD_TOL = 0.03
HOLD_MIN_S = 0.5
CREEP_MAX = 0.08
PUNCH_RISE_S = 0.15
PUNCH_LEVEL = 0.5
STOP_FROM = 0.2
LEVEL_BINS = (0.05, 0.15, 0.3, 0.5, 0.75, 1.01)

# Session totals worth aiming for before calling a recording day enough. Rough: enough repeats
# per stage of the staged fit to hold a session out and still fit on the rest.
MENU_TARGETS = {
    "hold_s_fwd": 20.0,
    "hold_s_rev": 15.0,
    "creep_s": 10.0,
    "spin_s_left": 8.0,
    "spin_s_right": 8.0,
    "arc_s": 15.0,
    "reversals": 10,
    "punches": 8,
    "hard_stops": 15,
}


@dataclass
class Check:
    name: str
    level: str  # PASS | WARN | FAIL | INFO
    detail: str


@dataclass
class Report:
    path: Path
    checks: list[Check] = field(default_factory=list)
    stats: dict[str, Any] = field(default_factory=dict)
    menu: dict[str, Any] = field(default_factory=dict)

    def add(self, name: str, level: str, detail: str) -> None:
        self.checks.append(Check(name, level, detail))

    @property
    def worst(self) -> str:
        for level in ("FAIL", "WARN", "PASS"):
            if any(c.level == level for c in self.checks):
                return level
        return "INFO"


# ---------------------------------------------------------------------------
# Individual checks
# ---------------------------------------------------------------------------


def check_file(path: Path, rep: Report) -> dict[str, int] | None:
    """Topic -> message count, or None if the file has no readable summary."""
    try:
        with open(path, "rb") as handle:
            summary = make_reader(handle).get_summary()
    except Exception as exc:  # the mcap reader raises a mix of types on a torn file
        rep.add("file", "FAIL", f"unreadable ({str(exc)[:60]}); re-download or drop it")
        return None
    if summary is None or summary.statistics is None:
        rep.add("file", "FAIL", "no summary: copied while recording, or the app died mid-write")
        return None
    st = summary.statistics
    duration = (st.message_end_time - st.message_start_time) / 1e9
    counts = {
        ch.topic: st.channel_message_counts.get(cid, 0) for cid, ch in summary.channels.items()
    }
    rep.stats["duration_s"] = duration
    rep.stats["size_mb"] = path.stat().st_size / 1e6
    level = "PASS" if duration >= 30.0 else "WARN"
    rep.add("file", level, f"{duration:.0f} s, {rep.stats['size_mb']:.0f} MB")
    return counts


def check_video(counts: dict[str, int], rep: Report) -> None:
    """/camera/video against /camera/frame_meta: is the footage there to replay?"""
    frames = counts.get(FRAME_META_TOPIC, 0)
    video = counts.get(CAMERA_VIDEO_TOPIC, 0)
    if video == 0:
        rep.add(
            "video",
            "FAIL",
            "no /camera/video: nothing to replay. Builds since the GStreamer encoder refuse to"
            " start like this; an older build's encoder failed to open",
        )
        return
    share = video / frames if frames else math.nan
    rep.stats["video_coverage"] = share
    level = "PASS" if share >= VIDEO_COVERAGE_PASS else "WARN"
    # Over 100% is normal: the encoder starts at camera open, a second or so before the first
    # pipeline tick writes frame_meta. Under it means the encoder dropped frames.
    rep.add("video", level, f"{video} frames for {frames} camera frames ({share:.1%})")


def frame_stamps(path: Path) -> np.ndarray:
    stamps = [
        int(decode_json(data)["image_stamp_ns"])
        for _t, _ts, data in iter_messages(path, [FRAME_META_TOPIC])
    ]
    return np.asarray(stamps, dtype=np.int64)


def check_camera(stamps: np.ndarray, rep: Report) -> None:
    if len(stamps) < 2:
        rep.add("camera", "FAIL", "no /camera/frame_meta")
        return
    dt = np.diff(np.sort(stamps)) / 1e9
    fps = 1.0 / float(np.median(dt))
    span = (stamps.max() - stamps.min()) / 1e9
    mean_fps = (len(stamps) - 1) / span if span > 0 else math.nan
    gaps = int(np.sum(dt > CAMERA_GAP_WARN_S))
    rep.stats.update(camera_fps=fps, camera_mean_fps=mean_fps, camera_max_gap_s=float(dt.max()))
    level = "PASS" if mean_fps >= CAMERA_FPS_PASS and gaps == 0 else "WARN"
    if dt.max() > CAMERA_GAP_FAIL_S:
        level = "FAIL"
    rep.add(
        "camera",
        level,
        f"{mean_fps:.1f} fps mean ({fps:.1f} median), longest gap {dt.max() * 1e3:.0f} ms,"
        f" {gaps} gaps > {CAMERA_GAP_WARN_S * 1e3:.0f} ms",
    )


def check_field(path: Path, rep: Report, counts: dict[str, int]) -> None:
    """Each field init redefines the field frame, so a session is a run of locks. The smoother
    wants one; report how many there were, how long the longest held, and whether the camera
    wandered within it (a bumped tripod)."""
    series = load_field_from_camera(path)
    reinits = counts.get("/command/reinit_field", 0)
    if len(series) == 0:
        rep.add("field", "FAIL", "no field -> camera transform: field never locked")
        return
    t = series.matrices[:, :3, 3]
    step = np.linalg.norm(np.diff(t, axis=0), axis=1)
    rot = np.array(
        [
            rotation_angle(a[:3, :3].T @ b[:3, :3])
            for a, b in zip(series.matrices, series.matrices[1:])
        ]
    )
    jumps = np.flatnonzero((step > 0.02) | (np.degrees(rot) > 2.0)) + 1
    bounds = np.concatenate([[0], jumps, [len(series)]])
    locks = [(int(a), int(b)) for a, b in zip(bounds[:-1], bounds[1:]) if b - a > 1]
    a, b = max(locks, key=lambda ab: series.stamps_ns[ab[1] - 1] - series.stamps_ns[ab[0]])
    longest_s = (series.stamps_ns[b - 1] - series.stamps_ns[a]) / 1e9
    sub = type(series)(series.stamps_ns[a:b], series.matrices[a:b])
    drift = field_transform_constancy(sub)
    rep.stats.update(field_locks=len(locks), field_longest_lock_s=longest_s, field_drift=drift)
    moved = (
        drift["max_translation_m"] > FIELD_DRIFT_M or drift["max_rotation_deg"] > FIELD_DRIFT_DEG
    )
    level = "WARN" if moved or len(locks) > 1 else "PASS"
    note = ""
    if len(locks) > 1:
        note = " -- smooth each lock as its own session"
    if moved:
        note += " -- camera moved within a lock: bumped tripod"
    rep.add(
        "field",
        level,
        f"{len(locks)} lock(s) ({reinits} re-init commands), longest {longest_s:.0f} s;"
        f" wander within it {drift['max_translation_m'] * 1e3:.1f} mm,"
        f" {drift['max_rotation_deg']:.2f} deg{note}",
    )


def check_tags(path: Path, rep: Report, cam_stamps: np.ndarray, sticks: pd.DataFrame) -> bool:
    frames = load_robot_tags(path)
    if not frames:
        rep.add(
            "robot tags",
            "FAIL",
            "no /apriltag/robot_tags: recorded without the sysid profile. Replay /camera/video"
            " through config/playback/mr_stabs_mk2_sysid_playback.toml to recover them",
        )
        return False
    seen = np.array([bool(f.detections) for f in frames])
    stamps = np.array([f.image_stamp_ns for f in frames], dtype=np.int64)
    sizes = [d.size_px for f in frames for d in f.detections]
    coverage = float(seen.mean())
    moving_cov = math.nan
    if not sticks.empty:
        lin, ang = stick_at(sticks, stamps)
        moving = (np.abs(lin) > MOVING) | (np.abs(ang) > MOVING)
        if moving.any():
            moving_cov = float(seen[moving].mean())
    small = float(np.mean(np.asarray(sizes) < TAG_MIN_PX)) if sizes else math.nan
    rep.stats.update(tag_coverage=coverage, tag_coverage_moving=moving_cov, tag_frames=len(frames))
    worst = min(coverage, moving_cov) if math.isfinite(moving_cov) else coverage
    level = (
        "PASS" if worst >= TAG_COVERAGE_PASS else "WARN" if worst >= TAG_COVERAGE_WARN else "FAIL"
    )
    per_frame = len(frames) / len(cam_stamps) if len(cam_stamps) else math.nan
    rep.add(
        "robot tags",
        level,
        f"seen in {coverage:.0%} of frames, {moving_cov:.0%} while driving;"
        f" tag median {np.median(sizes) if sizes else math.nan:.0f} px,"
        f" {small:.0%} under {TAG_MIN_PX:.0f} px; {per_frame:.0%} of camera frames processed",
    )
    return True


def check_track(path: Path, rep: Report, has_tags: bool) -> pd.DataFrame:
    pos = diag_io.load_robot_positions(path)
    ours = pos[pos["group"] == "OURS"] if not pos.empty else pos
    ticks = pos["timestamp_ns"].nunique() if not pos.empty else 0
    if ticks == 0:
        rep.add("our-robot track", "INFO" if has_tags else "WARN", "no /robot_markers")
        return ours
    present = ours["timestamp_ns"].nunique() / ticks
    rep.stats["our_track_coverage"] = present
    # Informational when tags exist: the fit reads tags, this is only the live keypoint path.
    level = "INFO" if has_tags else ("PASS" if present >= TAG_COVERAGE_PASS else "WARN")
    extent = ""
    if not ours.empty:
        extent = f", drove over {np.ptp(ours['x']):.2f} x {np.ptp(ours['y']):.2f} m of field"
    rep.add("our-robot track", level, f"OUR_ROBOT in {present:.0%} of ticks{extent}")
    return ours


def check_esp32(path: Path, rep: Report) -> bool:
    esp = load_esp32_diagnostics(path)
    if esp.empty:
        rep.add(
            "ESP32 stream",
            "FAIL",
            "no /robot/esp32_diagnostics: no per-motor command tape and no pack voltage."
            " Record with mr_stabs_mk2_sysid_zed_box (box joined to MR-STABS)",
        )
        return False
    host = esp["log_time_ns"].to_numpy(np.int64)
    span = (host.max() - host.min()) / 1e9
    rate = (len(esp) - 1) / span if span > 0 else math.nan
    loop_us = esp["loop_us"].to_numpy(float) if "loop_us" in esp else np.array([math.nan])
    expected = 1e6 / float(np.nanmedian(loop_us)) if np.isfinite(loop_us).any() else math.nan
    robot_ms = esp["timestamp_ms"].to_numpy(float)
    rgaps = np.diff(robot_ms) / 1e3
    gaps = int(np.sum(rgaps > ESP32_GAP_S))
    ratio = rate / expected if math.isfinite(expected) else math.nan
    rep.stats.update(esp32_rate=rate, esp32_expected=expected, esp32_gaps=gaps)
    level = "PASS" if ratio >= ESP32_RATE_PASS and gaps == 0 else "WARN"
    rep.add(
        "ESP32 stream",
        level,
        f"{rate:.0f} events/s vs firmware loop {expected:.0f}/s ({ratio:.0%});"
        f" {gaps} robot-clock gaps > {ESP32_GAP_S * 1e3:.0f} ms,"
        f" longest {np.nanmax(rgaps) * 1e3 if len(rgaps) else math.nan:.0f} ms",
    )
    vbat = esp["vbat"].to_numpy(float) if "vbat" in esp else np.array([])
    good = vbat[np.isfinite(vbat) & (vbat > VBAT_MIN_V)]
    if len(good) == 0:
        rep.add("pack voltage", "FAIL", "vbat missing or NaN: the fit falls back to 15.2 V")
    else:
        bad = 1.0 - len(good) / len(vbat)
        rep.stats.update(vbat_start=float(good[:50].mean()), vbat_end=float(good[-50:].mean()))
        rep.add(
            "pack voltage",
            "PASS" if bad < 0.01 else "WARN",
            f"{good[:50].mean():.2f} V start, {good[-50:].mean():.2f} V end,"
            f" min {good.min():.2f} V (sag), {bad:.1%} bad readings",
        )
    armed = esp["armed"].astype(bool).mean() if "armed" in esp else math.nan
    flip_down = (~esp["is_upside_down"].astype(bool)).mean() if "is_upside_down" in esp else 1.0
    rep.add("robot state", "INFO", f"armed {armed:.0%} of events, upright {flip_down:.0%}")
    return True


# ---------------------------------------------------------------------------
# Sticks and the maneuver menu
# ---------------------------------------------------------------------------


def stick_centers(sticks: pd.DataFrame) -> tuple[float, float]:
    """Resting value of each stick channel: the value held longest. A trimmed or worn gimbal
    rests a few counts off zero, which would otherwise read as constant creep."""
    t = sticks["stamp_ns"].to_numpy(np.int64)
    held = np.diff(np.concatenate([t, [t[-1]]]))
    out = []
    for ch in (LINEAR_CH, ANGULAR_CH):
        v = sticks[f"ch{ch}"].to_numpy(float)
        near = np.abs(v) < 0.05 * STICK_FULL
        if not near.any():
            out.append(0.0)
            continue
        totals = pd.Series(held[near]).groupby(v[near]).sum()
        out.append(float(totals.idxmax()))
    return out[0], out[1]


def stick_at(sticks: pd.DataFrame, stamps_ns: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Linear and angular stick fraction held at each stamp (channels log only on change),
    with each channel's resting offset removed."""
    t = sticks["stamp_ns"].to_numpy(np.int64)
    c_lin, c_ang = stick_centers(sticks)
    idx = np.clip(np.searchsorted(t, stamps_ns, side="right") - 1, 0, len(t) - 1)
    lin = (sticks[f"ch{LINEAR_CH}"].to_numpy(float)[idx] - c_lin) / STICK_FULL
    ang = (sticks[f"ch{ANGULAR_CH}"].to_numpy(float)[idx] - c_ang) / STICK_FULL
    before = stamps_ns < t[0]
    return np.where(before, 0.0, lin), np.where(before, 0.0, ang)


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    if not mask.any():
        return []
    edges = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def held_segments(u: np.ndarray, gate: np.ndarray) -> list[tuple[int, int, float]]:
    """(start, stop, mean level) for stretches where u stays within HOLD_TOL of its start."""
    out = []
    for a, b in _runs(gate):
        i = a
        while i < b:
            j = i + 1
            while j < b and abs(u[j] - u[i]) <= HOLD_TOL:
                j += 1
            if (j - i) * GRID_S >= HOLD_MIN_S:
                out.append((i, j, float(np.mean(u[i:j]))))
            i = j
    return out


def maneuver_menu(sticks: pd.DataFrame) -> dict[str, Any]:
    t0, t1 = int(sticks["stamp_ns"].iloc[0]), int(sticks["stamp_ns"].iloc[-1])
    grid = np.arange(t0, t1, int(GRID_S * 1e9), dtype=np.int64)
    lin, ang = stick_at(sticks, grid)
    alin, aang = np.abs(lin), np.abs(ang)
    straight = (alin > MOVING) & (aang < HOLD_TOL)
    spin = (aang > MOVING) & (alin < HOLD_TOL)
    arc = (alin > MOVING) & (aang > MOVING)

    holds = held_segments(lin, straight)
    levels: dict[str, float] = {}
    for a, b, level in holds:
        k = int(np.searchsorted(LEVEL_BINS, abs(level), side="right")) - 1
        k = min(max(k, 0), len(LEVEL_BINS) - 2)
        key = f"{'+' if level > 0 else '-'}{LEVEL_BINS[k]:.2f}"
        levels[key] = levels.get(key, 0.0) + (b - a) * GRID_S
    spins = held_segments(ang, spin)

    rise = int(round(PUNCH_RISE_S / GRID_S))
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
        int(np.sum((np.diff(sign[nz]) != 0) & (np.diff(nz) * GRID_S <= 0.3))) if len(nz) > 1 else 0
    )
    creep = float(np.sum((alin > 0.01) & (alin < CREEP_MAX) & (aang < HOLD_TOL)) * GRID_S)
    near_center = float(np.sum((alin > 0.005) & (alin < 0.02)) * GRID_S)

    return {
        "driving_s": float(np.sum((alin > MOVING) | (aang > MOVING)) * GRID_S),
        "total_s": len(grid) * GRID_S,
        "hold_s_fwd": float(sum((b - a) * GRID_S for a, b, v in holds if v > 0)),
        "hold_s_rev": float(sum((b - a) * GRID_S for a, b, v in holds if v < 0)),
        "holds_1s": int(sum((b - a) * GRID_S >= 1.0 for a, b, _ in holds)),
        "hold_levels_s": dict(sorted(levels.items())),
        "creep_s": creep,
        "jitter_near_center_s": near_center,
        "spin_s_left": float(sum((b - a) * GRID_S for a, b, v in spins if v > 0)),
        "spin_s_right": float(sum((b - a) * GRID_S for a, b, v in spins if v < 0)),
        "spin_levels": sorted({round(abs(v), 1) for _a, _b, v in spins}),
        "arc_s": float(np.sum(arc) * GRID_S),
        "reversals": reversals,
        "punches": punches,
        "hard_stops": stops,
        "max_linear": float(alin.max()),
        "max_angular": float(aang.max()),
    }


def check_sticks(sticks: pd.DataFrame, rep: Report) -> None:
    if sticks.empty:
        rep.add("sticks", "FAIL", "no transmitter channel log: cannot see what was driven")
        return
    menu = maneuver_menu(sticks)
    rep.menu = menu
    c_lin, c_ang = stick_centers(sticks)
    if c_lin or c_ang:
        rep.add(
            "stick center",
            "WARN" if max(abs(c_lin), abs(c_ang)) / STICK_FULL >= 0.02 else "INFO",
            f"rests at linear {c_lin:+.0f}, angular {c_ang:+.0f} of {STICK_FULL:.0f}"
            " (subtracted below). Re-center the radio trims if this reaches the 2% deadzone",
        )
    rep.add(
        "sticks",
        "PASS" if menu["driving_s"] >= 30.0 else "WARN",
        f"driving {menu['driving_s']:.0f} of {menu['total_s']:.0f} s,"
        f" max |linear| {menu['max_linear']:.2f}, max |angular| {menu['max_angular']:.2f}",
    )
    if menu["jitter_near_center_s"] > 5.0:
        rep.add(
            "stick jitter",
            "WARN",
            f"{menu['jitter_near_center_s']:.0f} s with the linear stick barely off center"
            " (0.5-2%): blurs the deadzone",
        )


# ---------------------------------------------------------------------------
# SVO pairing
# ---------------------------------------------------------------------------


def svo_spans(svo_dir: Path) -> list[tuple[Path, int, int, int]]:
    """(path, first stamp, last stamp, frames) for every .svo2 under svo_dir."""
    out = []
    for svo in sorted(svo_dir.rglob("*.svo2")):
        try:
            with open(svo, "rb") as handle:
                summary = make_reader(handle).get_summary()
            if summary is None:
                continue
            video = [
                ch.topic
                for ch in summary.channels.values()
                if ch.topic not in ("svo_header", "svo_footer") and "sensors" not in ch.topic
            ]
            if not video:
                continue
            stamps = read_frame_stamps(svo, video[0])
        except Exception:
            continue
        if stamps:
            out.append((svo, min(stamps), max(stamps), len(stamps)))
    return out


def check_svo(
    stamps: np.ndarray, spans: list[tuple[Path, int, int, int]] | None, rep: Report
) -> None:
    if spans is None:
        return
    if len(stamps) == 0:
        rep.add("SVO", "WARN", "no camera stamps to pair on")
        return
    lo, hi = int(stamps.min()), int(stamps.max())
    best = max(spans, key=lambda s: min(hi, s[2]) - max(lo, s[1]), default=None)
    if best is None or min(hi, best[2]) - max(lo, best[1]) <= 0:
        rep.add("SVO", "FAIL", "no SVO overlaps this recording: download it from the box")
        return
    svo, a, b, n = best
    covered = (min(hi, b) - max(lo, a)) / max(hi - lo, 1)
    rep.stats["svo"] = str(svo)
    rep.add(
        "SVO",
        "PASS" if covered >= 0.98 else "WARN",
        f"{svo.name}: {n} frames, covers {covered:.0%} of the MCAP's camera span",
    )


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------


def plot(rep: Report, sticks: pd.DataFrame, ours: pd.DataFrame, out_dir: Path) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5), gridspec_kw={"width_ratios": [1, 2.2, 1]})
    starts = [int(d["timestamp_ns"].min()) for d in (ours,) if not d.empty]
    starts += [int(sticks["stamp_ns"].iloc[0])] if not sticks.empty else []
    t0 = min(starts) if starts else 0
    ax = axes[0]
    if not ours.empty:
        t = (ours["timestamp_ns"] - t0) / 1e9
        sc = ax.scatter(ours["x"], ours["y"], c=t, s=2, cmap="viridis")
        fig.colorbar(sc, ax=ax, label="s")
    ax.set_aspect("equal")
    ax.set_title("OUR_ROBOT track (field frame, m)")

    ax = axes[1]
    if not sticks.empty:
        ts = (sticks["stamp_ns"] - t0) / 1e9
        c_lin, c_ang = stick_centers(sticks)
        lin = (sticks[f"ch{LINEAR_CH}"] - c_lin) / STICK_FULL
        ang = (sticks[f"ch{ANGULAR_CH}"] - c_ang) / STICK_FULL
        ax.step(ts, lin, where="post", lw=0.8, label="linear")
        ax.step(ts, ang, where="post", lw=0.8, label="angular")
        if not ours.empty:
            seen = np.sort(ours["timestamp_ns"].unique())
            gaps = np.flatnonzero(np.diff(seen) > 0.2e9)
            for g in gaps:
                ax.axvspan((seen[g] - t0) / 1e9, (seen[g + 1] - t0) / 1e9, color="red", alpha=0.15)
        ax.legend(loc="upper right")
    ax.set_ylim(-1.05, 1.05)
    ax.set_title("sticks (red: OUR_ROBOT track lost > 200 ms)")
    ax.set_xlabel("s")

    ax = axes[2]
    levels = rep.menu.get("hold_levels_s", {})
    if levels:
        ax.barh(list(levels), list(levels.values()))
    ax.set_title("straight-hold seconds by stick level")
    fig.suptitle(rep.path.name)
    fig.tight_layout()
    out_dir.mkdir(parents=True, exist_ok=True)
    png = out_dir / f"{rep.path.stem}.png"
    fig.savefig(png, dpi=90)
    plt.close(fig)
    return png


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def check_recording(
    path: Path, spans: list[tuple[Path, int, int, int]] | None, plots: Path | None
) -> Report:
    rep = Report(path)
    counts = check_file(path, rep)
    if counts is None:
        return rep
    stamps = frame_stamps(path)
    check_camera(stamps, rep)
    check_video(counts, rep)
    check_field(path, rep, counts)
    sticks = load_transmitter_channels(path).dropna(subset=[f"ch{LINEAR_CH}", f"ch{ANGULAR_CH}"])
    has_tags = check_tags(path, rep, stamps, sticks)
    ours = check_track(path, rep, has_tags)
    check_esp32(path, rep)
    check_sticks(sticks, rep)
    check_svo(stamps, spans, rep)
    if plots is not None:
        rep.stats["plot"] = str(plot(rep, sticks, ours, plots))
    return rep


def print_report(rep: Report) -> None:
    print(f"\n{rep.worst:4}  {rep.path.name}")
    for c in rep.checks:
        print(f"   {c.level:4}  {c.name:16} {c.detail}")
    if "plot" in rep.stats:
        print(f"         plot {rep.stats['plot']}")


def print_menu(reports: list[Report]) -> None:
    menus = [r.menu for r in reports if r.menu]
    if not menus:
        return
    print("\nmaneuver menu, per recording and total (target is a rough total for the whole day):")
    keys = list(MENU_TARGETS)
    names = [r.path.stem[-19:] for r in reports if r.menu]
    print(f"   {'':14}" + "".join(f"{n[-8:]:>10}" for n in names) + f"{'total':>10}{'target':>9}")
    for k in keys:
        vals = [float(m[k]) for m in menus]
        total = sum(vals)
        flag = "" if total >= MENU_TARGETS[k] else "  <- short"
        print(
            f"   {k:14}"
            + "".join(f"{v:10.0f}" for v in vals)
            + f"{total:10.0f}{MENU_TARGETS[k]:9.0f}{flag}"
        )
    levels: dict[str, float] = {}
    for m in menus:
        for lvl, s in m["hold_levels_s"].items():
            levels[lvl] = levels.get(lvl, 0.0) + s
    print("   straight-hold seconds by stick level (sign is CH1's, not yet tied to forward):")
    print("     " + ", ".join(f"{k}: {v:.0f}" for k, v in sorted(levels.items())) or "     none")
    spin_levels = sorted({lv for m in menus for lv in m["spin_levels"]})
    print(f"   spin levels held: {spin_levels or 'none'} (menu asks for 3 to 4)")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("mcaps", type=Path, nargs="+")
    ap.add_argument("--svo-dir", type=Path, default=None, help="pair each MCAP with its SVO")
    ap.add_argument("--plots", type=Path, default=None, help="write one PNG per recording here")
    ap.add_argument("--min-mb", type=float, default=1.0, help="skip smaller files (aborted starts)")
    ap.add_argument("--json", type=Path, default=None, help="write every check and stat here")
    args = ap.parse_args()

    paths = [p for p in args.mcaps if p.stat().st_size >= args.min_mb * 1e6]
    skipped = len(args.mcaps) - len(paths)
    if skipped:
        print(f"skipping {skipped} file(s) under {args.min_mb} MB")
    spans = svo_spans(args.svo_dir) if args.svo_dir else None
    if spans is not None:
        print(f"{len(spans)} SVO(s) found under {args.svo_dir}")

    reports = []
    for path in paths:
        rep = check_recording(path, spans, args.plots)
        print_report(rep)
        reports.append(rep)
    print_menu(reports)

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        doc = [
            {
                "path": str(r.path),
                "worst": r.worst,
                "checks": [c.__dict__ for c in r.checks],
                "stats": r.stats,
                "menu": r.menu,
            }
            for r in reports
        ]
        args.json.write_text(json.dumps(doc, indent=2, default=float))
        print(f"\nwrote {args.json}")
    fails = [r for r in reports if r.worst == "FAIL"]
    print(f"\n{len(reports) - len(fails)} of {len(reports)} recording(s) without a FAIL")
    raise SystemExit(1 if fails else 0)


if __name__ == "__main__":
    os.environ.setdefault("MPLBACKEND", "Agg")
    main()
