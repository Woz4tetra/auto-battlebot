"""Fit the sensing model the RL environment and the lockstep sim sample from.

`docs/plans/sensing_model_plan.md` section 8. Until the test-box sessions exist the model is
provisional and fitted from the NHRL May and MassD KALMAN replays alone
(`playground/opponent_prediction/run_replays.py --mode KALMAN`):

- Dropouts conditioned on the distance to the nearest robot and wall, flip onsets and flipped-run
  lengths.
- Position and heading noise by speed bin, from the opponent measurements with no truth: each
  measurement against a quadratic fit through its neighbours. That sees frame-to-frame jitter
  only. Bias and slow errors (plane height, field lock, the laptop playback warp) are invisible
  to it, so the bias tables are zero. Both roles get these numbers, so the opponents stand in for
  Mr Stabs.

The 2026-10-15 sysid sessions have tag truth but were shot on the apartment floor, which the
detector never trained on (its synthetic scenes are NHRL, MassD and the AER test box), so they
are not used.

Output: `simulation/sensing_models/provisional.toml`, and `playground/sensing_model/out/
fit_report.json` with the per-cell sample counts, the holdout session's noise and the flip
analysis.

Usage:
    source scripts/activate_python.sh
    python playground/sensing_model/fit_sensing_model.py
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import tomli_w

from auto_battlebot.perception.sensing_model import SPEC_VERSION, SensingModelFile
from auto_battlebot.perception.sysid_sessions import OUT as OPPONENT_OUT
from auto_battlebot.perception.sysid_sessions import REPO_ROOT
from auto_battlebot.recording.mcap_io import decode_json, iter_messages

OUTPUT = REPO_ROOT / "simulation/sensing_models/provisional.toml"
REPORT = Path(__file__).resolve().parent / "out/fit_report.json"
REPLAYS = OPPONENT_OUT / "replays"

# The replays measure at 30 Hz. The lockstep sim runs the model at the ZED Box camera rate.
FRAME_HZ = 30.0
FRAME_S = 1.0 / FRAME_HZ
SIM_FRAME_HZ = 60.0
LATENCY_S = 0.063
SPEED_BINS = [0.0, 0.25, 0.5, 1.0, 1.5]
ROBOT_DISTANCE_BINS = [0.0, 0.05, 0.2, 0.5]
WALL_DISTANCE_BINS = [0.0, 0.15, 0.3]
QUANTILES = [0.0, 0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.95, 0.99, 1.0]
MIN_CELL_SAMPLES = 30
# Spans are clipped to the RL episode length. A longer gap acts the same within one episode, and
# the few there are (a robot dead against a wall for 125 s in 14-12-27) would otherwise set the
# top quantiles alone.
MAX_SPAN_S = 20.0
# A gap is a step longer than this many frames. Single missing frames are left out: laptop
# playback drops about one frame per 0.5 s for pacing, which is not the robot going unseen.
GAP_MIN_FRAMES = 2.5
# Jitter fit: this many contiguous neighbours on each side of a measurement, 0.1 s at 30 Hz.
JITTER_HALF = 3
FLIP_RAD = math.radians(150.0)
# A heading jump and its reversal within this long bound one flipped run.
MAX_FLIPPED_RUN_S = 1.0
# A role with fewer heading jumps than this takes the onset rate pooled over both roles.
MIN_ROLE_JUMPS = 10
# Footprint radius per tracked label, for edge-to-edge distance: half the mean of the x and y
# extents in config/_common.toml robot_size_meters_per_label. The generic opponent has no size;
# 0.12 m is a 3 lb robot.
RADIUS_BY_LABEL = {"mrs_buff_mk3": 0.119, "mr_stabs_mk2": 0.074, "house_bot": 0.146}
DEFAULT_RADIUS_M = 0.12
RANDOMIZATION = {
    "frame_rate_hz": [30.0, 60.0],
    "latency_s": [0.04, 0.08],
    "frame_drop_scale": [0.5, 2.0],
    "error_scale": [0.7, 1.5],
    "offset_sigma_m": 0.0,
    "flip_scale": [0.5, 5.0],
    "dropout_stretch": [0.7, 2.0],
    "ekf_sigma_scale": [0.5, 1.5],
    "ekf_tau_scale": [0.5, 2.0],
}


def _robust(x: np.ndarray) -> float:
    return float(1.4826 * np.median(np.abs(x - np.median(x)))) if x.size else float("nan")


def _wrap(a: np.ndarray) -> np.ndarray:
    return np.asarray(np.angle(np.exp(1j * a)))


def _r(x: float, digits: int = 5) -> float:
    return round(float(x), digits)


# ---------------------------------------------------------------------------
# Jitter noise
# ---------------------------------------------------------------------------


def _windows(t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Measurements with JITTER_HALF contiguous neighbours on each side, and those neighbours'
    indices, one row per centre."""
    k = JITTER_HALF
    offsets = np.concatenate([np.arange(-k, 0), np.arange(1, k + 1)])
    if t.size < 2 * k + 1:
        return np.zeros(0, int), np.zeros((0, 2 * k), int)
    run = np.concatenate([[0], np.cumsum(np.diff(t) >= 1.5 * FRAME_S)])
    centres = np.arange(k, t.size - k)
    centres = centres[run[centres - k] == run[centres + k]]
    return centres, centres[:, None] + offsets[None, :]


def _fit(
    t: np.ndarray, centres: np.ndarray, idx: np.ndarray, values: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Least-squares quadratic through each centre's neighbours (`values`, one row per centre).

    Returns the fit's value and slope at the centre, and the variance factor h: with white noise
    of sigma, measurement minus fit has variance sigma^2 (1 + h).
    """
    dt = t[idx] - t[centres][:, None]
    design = np.stack([np.ones_like(dt), dt, dt * dt], axis=-1)
    gram_inv = np.linalg.inv(np.einsum("cni,cnj->cij", design, design))
    coef = np.einsum("cij,cnj,cn->ci", gram_inv, design, values)
    return coef[:, 0], coef[:, 1], gram_inv[:, 0, 0]


def jitter_noise(tracks: list[Track]) -> tuple[dict[str, Any], dict[str, Any]]:
    """Body-frame position and heading jitter by speed bin, from the opponent measurements.

    A residual is a measurement minus the quadratic through its neighbours, scaled by
    1 / sqrt(1 + h) to undo the fit's own noise. Speed is the fit's slope. A heading residual past
    pi / 2 is a flip and is left out, and so is a window that holds a heading jump past FLIP_RAD.
    """
    along_all, cross_all, herr_all, speed_all, hspeed_all = [], [], [], [], []
    for tr in tracks:
        if tr.role != "opponent":
            continue
        centres, idx = _windows(tr.stamps)
        if centres.size == 0:
            continue
        x_fit, vx, h = _fit(tr.stamps, centres, idx, tr.x[idx])
        y_fit, vy, _ = _fit(tr.stamps, centres, idx, tr.y[idx])
        scale = 1.0 / np.sqrt(1.0 + h)
        ex = (tr.x[centres] - x_fit) * scale
        ey = (tr.y[centres] - y_fit) * scale
        yaw = tr.yaw[centres]
        c, sn = np.cos(yaw), np.sin(yaw)
        along_all.append(c * ex + sn * ey)
        cross_all.append(-sn * ex + c * ey)
        speed = np.hypot(vx, vy)
        speed_all.append(speed)

        # Heading: neighbours taken relative to the centre, so the fit sees no 2 pi steps.
        yaw_fit, _, _ = _fit(tr.stamps, centres, idx, _wrap(tr.yaw[idx] - yaw[:, None]))
        herr = _wrap(-yaw_fit) * scale
        window = centres[:, None] + np.arange(-JITTER_HALF, JITTER_HALF)[None, :]
        jumps = (np.abs(_wrap(np.diff(tr.yaw)))[window] > FLIP_RAD).any(axis=1)
        keep = ~jumps & (np.abs(herr) < math.pi / 2)
        herr_all.append(herr[keep])
        hspeed_all.append(speed[keep])

    along = np.concatenate(along_all)
    cross = np.concatenate(cross_all)
    speed = np.concatenate(speed_all)
    herr = np.concatenate(herr_all)
    hspeed = np.concatenate(hspeed_all)

    edges = SPEED_BINS + [math.inf]
    zeros = [0.0] * len(SPEED_BINS)
    sigma_along, sigma_cross, sigma_rad, samples, heading_samples = [], [], [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (speed >= lo) & (speed < hi)
        mh = (hspeed >= lo) & (hspeed < hi)
        if m.sum() < MIN_CELL_SAMPLES:
            m = np.ones_like(m)
        if mh.sum() < MIN_CELL_SAMPLES:
            mh = np.ones_like(mh)
        sigma_along.append(_r(_robust(along[m])))
        sigma_cross.append(_r(_robust(cross[m])))
        sigma_rad.append(_r(_robust(herr[mh])))
        samples.append(int(((speed >= lo) & (speed < hi)).sum()))
        heading_samples.append(int(((hspeed >= lo) & (hspeed < hi)).sum()))
    position = {
        "sigma_along_m": sigma_along,
        "sigma_cross_m": sigma_cross,
        "bias_along_m": zeros,
        "bias_along_reverse_m": zeros,
        "bias_cross_m": zeros,
    }
    report = {
        "position_samples_by_speed_bin": samples,
        "heading_samples_by_speed_bin": heading_samples,
        "position": position,
        "heading_sigma_rad": sigma_rad,
        "speed_quantiles_mps": [_r(q, 3) for q in np.quantile(speed, [0.5, 0.9, 0.99])],
    }
    return {"position": position, "heading_sigma_rad": sigma_rad}, report


# ---------------------------------------------------------------------------
# Match replays
# ---------------------------------------------------------------------------


@dataclass
class Track:
    """One robot's measurement stream in one replay."""

    run: str
    role: str  # "ours" or "opponent"
    track_id: str  # id in /status/tracks
    stamps: np.ndarray
    yaw: np.ndarray
    # Field-frame position, NaN for our robot: its heading is the only measurement logged.
    x: np.ndarray
    y: np.ndarray


@dataclass
class Scene:
    """Rendered robot positions and field size over one replay, for the dropout context."""

    stamps: np.ndarray  # [T]
    ids: list[str]
    xy: np.ndarray  # [T, robots, 2], NaN where a robot is absent
    radius: np.ndarray  # [robots]
    half_extent: np.ndarray  # [T, 2]

    def context(self, track_id: str, t: float) -> tuple[float, float]:
        """Edge-to-edge distance to the nearest other robot and distance to the nearest wall."""
        k = int(np.clip(np.searchsorted(self.stamps, t), 0, len(self.stamps) - 1))
        if k > 0 and abs(self.stamps[k - 1] - t) < abs(self.stamps[k] - t):
            k -= 1
        i = self.ids.index(track_id)
        me = self.xy[k, i]
        if np.any(np.isnan(me)):
            return math.inf, math.inf
        wall = float(np.max([0.0, np.min(self.half_extent[k] - np.abs(me))]))
        others = [
            float(np.hypot(*(self.xy[k, j] - me)) - self.radius[i] - self.radius[j])
            for j in range(len(self.ids))
            if j != i and not np.any(np.isnan(self.xy[k, j]))
        ]
        return (max(0.0, min(others)) if others else math.inf), wall


def load_replay(mcap: Path) -> tuple[list[Track], Scene]:
    run = mcap.stem
    opp: dict[str, list[tuple[float, ...]]] = {}
    ours: list[tuple[float, ...]] = []
    for _topic, log_ns, data in iter_messages(mcap, ["/diagnostics/kalman_motion_estimator"]):
        msg = decode_json(data)
        heading = msg.get("our_heading")
        if heading and not heading["held"]:
            values = heading["values"]
            stamp = log_ns / 1e9 - float(values["stamp_age_s"])
            yaw = math.radians(float(values["measured_yaw_deg"]))
            ours.append((stamp, yaw, math.nan, math.nan))
        for key, entry in msg.items():
            if not key.startswith("opponent_measurement_THEIR_ROBOT") or entry["held"]:
                continue
            values = entry["values"]
            if int(values["keypoint"]) != 1:
                continue
            opp.setdefault(key[len("opponent_measurement_") :], []).append(
                tuple(float(values[k]) for k in ("stamp", "yaw", "x", "y"))
            )
    tracks = [_track(run, "ours", "our_robot_1", ours)]
    tracks += [_track(run, "opponent", fid.lower(), rows) for fid, rows in sorted(opp.items())]

    stamps, frames, half = [], [], []
    for _topic, log_ns, data in iter_messages(mcap, ["/status/tracks"]):
        msg = decode_json(data)
        stamps.append(log_ns / 1e9)
        frames.append({r["id"]: (r["x"], r["y"], r.get("label", "")) for r in msg["robots"]})
        half.append((0.5 * float(msg["field_x"]), 0.5 * float(msg["field_y"])))
    ids = sorted({i for f in frames for i in f})
    xy = np.full((len(frames), len(ids), 2), np.nan)
    labels: dict[str, str] = {}
    for k, f in enumerate(frames):
        for rid, (x, y, label) in f.items():
            xy[k, ids.index(rid)] = (x, y)
            labels[rid] = label
    radius = np.array([RADIUS_BY_LABEL.get(labels.get(i, ""), DEFAULT_RADIUS_M) for i in ids])
    scene = Scene(np.array(stamps), ids, xy, radius, np.array(half))
    return [t for t in tracks if t.track_id in ids and t.stamps.size > 1], scene


def _track(run: str, role: str, track_id: str, rows: list[tuple[float, ...]]) -> Track:
    if not rows:
        return Track(run, role, track_id, *np.zeros((4, 0)))
    a = np.array(sorted(rows))
    # The same measurement can be logged on two consecutive diagnostics messages.
    a = a[np.concatenate([[True], np.diff(a[:, 0]) > 0.010])]
    return Track(run, role, track_id, a[:, 0], a[:, 1], a[:, 2], a[:, 3])


@dataclass
class Spans:
    """Measured runs and dropped gaps of one role, with the context at each span's start."""

    measured_s: list[float]
    measured_ctx: list[tuple[float, float]]
    dropped_s: list[float]
    dropped_ctx: list[tuple[float, float]]


def spans(tracks: list[Track], scenes: dict[str, Scene]) -> Spans:
    out = Spans([], [], [], [])
    for tr in tracks:
        scene = scenes[tr.run]
        t = tr.stamps
        d = np.diff(t)
        gap = np.flatnonzero(d > GAP_MIN_FRAMES * FRAME_S)
        starts = np.concatenate([[0], gap + 1])
        ends = np.concatenate([gap, [len(t) - 1]])
        for a, b in zip(starts, ends):
            out.measured_s.append(min(MAX_SPAN_S, float(t[b] - t[a] + FRAME_S)))
            out.measured_ctx.append(scene.context(tr.track_id, float(t[a])))
        for g in gap:
            out.dropped_s.append(min(MAX_SPAN_S, float(d[g] - FRAME_S)))
            out.dropped_ctx.append(scene.context(tr.track_id, float(t[g])))
    return out


def _cell(ctx: tuple[float, float]) -> tuple[int, int]:
    robot, wall = ctx
    i = int(np.searchsorted(ROBOT_DISTANCE_BINS, robot, side="right") - 1)
    j = int(np.searchsorted(WALL_DISTANCE_BINS, wall, side="right") - 1)
    return i, j


def conditioned_table(
    durations: list[float], contexts: list[tuple[float, float]]
) -> tuple[list[list[float]], list[int], list[str]]:
    """Quantile rows per (robot bin, wall bin) cell, row-major, with the borrowing rule.

    A cell with fewer than MIN_CELL_SAMPLES durations adds its wall-axis neighbours, then its
    robot-axis neighbours, then falls back to the pooled durations.
    """
    nr, nw = len(ROBOT_DISTANCE_BINS), len(WALL_DISTANCE_BINS)
    by_cell: dict[tuple[int, int], list[float]] = {(i, j): [] for i in range(nr) for j in range(nw)}
    for d, ctx in zip(durations, contexts):
        by_cell[_cell(ctx)].append(d)
    pooled = np.array(durations)
    rows, counts, borrowed = [], [], []
    for i in range(nr):
        for j in range(nw):
            own = by_cell[(i, j)]
            counts.append(len(own))
            pool = list(own)
            how = ""
            if len(pool) < MIN_CELL_SAMPLES:
                pool += [x for jj in (j - 1, j + 1) if 0 <= jj < nw for x in by_cell[(i, jj)]]
                how = "wall"
            if len(pool) < MIN_CELL_SAMPLES:
                pool += [x for ii in (i - 1, i + 1) if 0 <= ii < nr for x in by_cell[(ii, j)]]
                how = "robot"
            values = np.array(pool) if len(pool) >= MIN_CELL_SAMPLES else pooled
            if len(pool) < MIN_CELL_SAMPLES:
                how = "pooled"
            if how:
                borrowed.append(f"robot{i}_wall{j}:{how}")
            rows.append([_r(q, 4) for q in np.quantile(values, QUANTILES)])
    return rows, counts, borrowed


def replay_flips(tracks: list[Track]) -> dict[str, Any]:
    """Heading jumps past FLIP_RAD between contiguous measurements. A jump reversed by another
    within MAX_FLIPPED_RUN_S bounds a run of flipped frames. Each flip has an onset and an end, so
    the onset rate is half the jump rate."""
    flipped_s: list[float] = []
    measured_s = 0.0
    jump_count = 0
    for tr in tracks:
        t, yaw = tr.stamps, tr.yaw
        if t.size < 2:
            continue
        d = np.diff(t)
        contiguous = d < 1.5 * FRAME_S
        measured_s += float(d[contiguous].sum())
        jumps = np.flatnonzero(contiguous & (np.abs(_wrap(np.diff(yaw))) > FLIP_RAD))
        jump_count += len(jumps)
        k = 0
        while k + 1 < len(jumps):
            a, b = jumps[k], jumps[k + 1]
            if t[b + 1] - t[a + 1] <= MAX_FLIPPED_RUN_S and np.all(contiguous[a:b]):
                flipped_s.append(float(t[b + 1] - t[a + 1]))
                k += 2
            else:
                k += 1
    return {"flipped_s": flipped_s, "measured_s": measured_s, "jumps": jump_count}


def flip_runs_table(flipped_s: list[float], onset_per_s: float) -> dict[str, Any]:
    """Bursty flip tables: flipped-run quantiles from data, normal runs exponential with the
    measured onset rate. The exponential's q = 1 point is its p99.9."""
    mean_normal = 1.0 / onset_per_s
    q = np.array(QUANTILES)
    normal = -np.log(1.0 - np.minimum(q, 0.999)) * mean_normal
    normal[0] = FRAME_S
    return {
        "quantiles": QUANTILES,
        "normal_s": [_r(x, 4) for x in np.maximum.accumulate(np.maximum(normal, FRAME_S))],
        "flipped_s": [_r(x, 4) for x in np.quantile(flipped_s, QUANTILES)],
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    mcaps = sorted(REPLAYS.glob("*_KALMAN.mcap"))
    if not mcaps:
        raise SystemExit(f"no KALMAN replays under {REPLAYS}; run run_replays.py --mode KALMAN")
    tracks: list[Track] = []
    scenes: dict[str, Scene] = {}
    for mcap in mcaps:
        tr, scene = load_replay(mcap)
        tracks += tr
        scenes[mcap.stem] = scene
    noise, noise_report = jitter_noise(tracks)

    report: dict[str, Any] = {
        "replays": [m.stem for m in mcaps],
        "noise": noise_report,
        "roles": {},
    }
    # Flipped-run lengths pool both roles: each alone has too few.
    all_flips = replay_flips(tracks)
    flipped_s = all_flips["flipped_s"]
    pooled_onset = 0.5 * all_flips["jumps"] / all_flips["measured_s"]
    report["flips"] = {
        "replay_jumps": all_flips["jumps"],
        "replay_flipped_runs": len(flipped_s),
        "flipped_run_quantiles_s": [_r(q, 4) for q in np.quantile(flipped_s, QUANTILES)],
        "pooled_onsets_per_s": _r(pooled_onset),
    }
    robots: dict[str, Any] = {}
    for role in ("ours", "opponent"):
        role_tracks = [t for t in tracks if t.role == role]
        sp = spans(role_tracks, scenes)
        m_rows, m_counts, m_borrow = conditioned_table(sp.measured_s, sp.measured_ctx)
        d_rows, d_counts, d_borrow = conditioned_table(sp.dropped_s, sp.dropped_ctx)
        flips = replay_flips(role_tracks)
        onset = (
            0.5 * flips["jumps"] / flips["measured_s"]
            if flips["jumps"] >= MIN_ROLE_JUMPS
            else pooled_onset
        )
        robots[role] = {
            "speed_bins_mps": SPEED_BINS,
            "position": noise["position"],
            "heading": {
                "sigma_rad": noise["heading_sigma_rad"],
                "flip_model": "bursty",
                "flip_runs": flip_runs_table(flipped_s, onset),
            },
            "dropout": {
                "robot_distance_bins_m": ROBOT_DISTANCE_BINS,
                "wall_distance_bins_m": WALL_DISTANCE_BINS,
                "quantiles": QUANTILES,
                "measured_s": m_rows,
                "dropped_s": d_rows,
                "borrowed": sorted(set(m_borrow) | set(d_borrow)),
            },
        }
        report["roles"][role] = {
            "tracks": len(role_tracks),
            "measured_runs_per_cell": m_counts,
            "dropped_gaps_per_cell": d_counts,
            "borrowed_measured": m_borrow,
            "borrowed_dropped": d_borrow,
            "pooled_gap_quantiles_s": [_r(q, 4) for q in np.quantile(sp.dropped_s, QUANTILES)],
            "clipped_spans": int(sum(x >= MAX_SPAN_S for x in sp.measured_s + sp.dropped_s)),
            "measured_fraction": _r(sum(sp.measured_s) / (sum(sp.measured_s) + sum(sp.dropped_s))),
            "heading_jumps": flips["jumps"],
            "flip_onsets_per_s": _r(onset),
            "onset_pooled": flips["jumps"] < MIN_ROLE_JUMPS,
        }

    model = {
        "spec_version": SPEC_VERSION,
        "venue": "provisional",
        "note": (
            "Provisional until the test-box sessions: everything from the NHRL May and MassD "
            "KALMAN replays. Noise is opponent frame-to-frame jitter with no truth, used for both "
            "roles; bias not measured (zero). Dropouts, flip onsets and flipped runs from the "
            "same replays. Latency nominal (not measured), frame drops not measured, "
            "randomization ranges from the plan. Generated by "
            "playground/sensing_model/fit_sensing_model.py."
        ),
        "fitted_from": report["replays"],
        "frame_rate_hz": SIM_FRAME_HZ,
        "latency_s": LATENCY_S,
        "frame_drop_prob": 0.0,
        "robots": robots,
        "randomization": RANDOMIZATION,
    }
    SensingModelFile.from_dict(model, "fit output")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(tomli_w.dumps(model))
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, indent=1))
    print(f"wrote {OUTPUT.relative_to(REPO_ROOT)} and {REPORT.relative_to(REPO_ROOT)}")
    print(json.dumps(_summary(report), indent=1))


def _summary(report: dict[str, Any]) -> dict[str, Any]:
    roles = {
        role: {k: r[k] for k in r if not k.startswith("borrowed")}
        for role, r in report["roles"].items()
    }
    return {
        "replays": len(report["replays"]),
        "roles": roles,
        "flips": report["flips"],
        "noise": report["noise"],
    }


if __name__ == "__main__":
    main()
