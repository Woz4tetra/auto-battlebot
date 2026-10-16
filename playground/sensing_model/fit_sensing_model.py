"""Fit the sensing model the RL environment and the lockstep sim sample from.

`docs/plans/sensing_model_plan.md` section 8. Until the apartment-floor sessions exist the model
is provisional and mixes two sources:

- Position and heading noise, and flipped-run lengths with truth: the 2026-10-15 Mr Stabs Mk2
  sysid training session against AprilTag truth (`auto_battlebot.perception.sysid_sessions`).
  The holdout session is reported, never fitted. Both roles get these numbers, so Mr Stabs stands
  in for the opponent.
- Dropouts conditioned on the distance to the nearest robot and wall, and flip onsets: the NHRL
  May and MassD KALMAN replays (`playground/opponent_prediction/run_replays.py --mode KALMAN`).
  Replays are laptop SVO playback, so they are never used for position noise.

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
from auto_battlebot.perception.sysid_sessions import REPO_ROOT, SESSIONS, load_session
from auto_battlebot.recording.mcap_io import decode_json, iter_messages

OUTPUT = REPO_ROOT / "simulation/sensing_models/provisional.toml"
REPORT = Path(__file__).resolve().parent / "out/fit_report.json"
REPLAYS = OPPONENT_OUT / "replays"

FRAME_HZ = 30.0
FRAME_S = 1.0 / FRAME_HZ
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
# Below this forward speed the direction of travel is too weak to pick the along-track bias.
DIRECTION_MIN_MPS = 0.1
MIN_DIRECTION_SAMPLES = 20
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
# Sysid noise
# ---------------------------------------------------------------------------


def sysid_noise(name: str) -> tuple[dict[str, Any], dict[str, Any]]:
    """Body-frame position noise and bias, heading noise and flipped runs for one session.

    Returns the position/heading tables and a report block.
    """
    s = load_session(name)
    tr = s.truth_at(s.meas_stamp_ns)
    v = tr["valid"]
    ex = (s.meas_x - tr["x"])[v]
    ey = (s.meas_y - tr["y"])[v]
    yaw = tr["yaw"][v]
    c, sn = np.cos(yaw), np.sin(yaw)
    along = c * ex + sn * ey
    cross = -sn * ex + c * ey
    forward = tr["vx"][v] * c + tr["vy"][v] * sn
    speed = np.hypot(tr["vx"], tr["vy"])[v]
    herr = _wrap(s.meas_yaw - tr["yaw"])[v]
    flipped = np.abs(herr) > math.pi / 2

    edges = SPEED_BINS + [math.inf]
    cols: dict[str, list[float]] = {
        k: []
        for k in (
            "sigma_along_m",
            "sigma_cross_m",
            "bias_along_m",
            "bias_along_reverse_m",
            "bias_cross_m",
            "sigma_rad",
        )
    }
    samples = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (speed >= lo) & (speed < hi) & ~flipped
        fwd = m & (forward >= DIRECTION_MIN_MPS)
        rev = m & (forward <= -DIRECTION_MIN_MPS)
        pooled = float(np.median(along[m]))
        b_fwd = float(np.median(along[fwd])) if fwd.sum() >= MIN_DIRECTION_SAMPLES else pooled
        b_rev = float(np.median(along[rev])) if rev.sum() >= MIN_DIRECTION_SAMPLES else pooled
        resid = along - np.where(forward < 0.0, b_rev, b_fwd)
        cols["bias_along_m"].append(_r(b_fwd))
        cols["bias_along_reverse_m"].append(_r(b_rev))
        cols["sigma_along_m"].append(_r(_robust(resid[m])))
        cols["bias_cross_m"].append(_r(np.median(cross[m])))
        cols["sigma_cross_m"].append(_r(_robust(cross[m])))
        cols["sigma_rad"].append(_r(_robust(herr[m])))
        samples.append(int(m.sum()))

    stamps = s.meas_stamp_ns[v] / 1e9
    runs = _flipped_runs_frames(stamps, flipped)
    position = {k: cols[k] for k in cols if k != "sigma_rad"}
    report = {
        "samples_by_speed_bin": samples,
        "position": position,
        "heading_sigma_rad": cols["sigma_rad"],
        "flipped_fraction": _r(flipped.mean()),
        "flipped_runs_frames": runs,
        "field_median_residual_m": [_r(np.median(ex)), _r(np.median(ey))],
        "measured_s": _r(len(stamps) * FRAME_S, 1),
    }
    return {"position": position, "heading_sigma_rad": cols["sigma_rad"]}, report


def _flipped_runs_frames(stamps_s: np.ndarray, flipped: np.ndarray) -> list[int]:
    """Lengths, in measurements, of consecutive flipped measurements on contiguous frames."""
    runs: list[int] = []
    current = 0
    previous = -math.inf
    for t, f in zip(stamps_s, flipped):
        contiguous = t - previous < 1.5 * FRAME_S
        previous = t
        if f and contiguous and current:
            current += 1
            continue
        if current:
            runs.append(current)
        current = 1 if f else 0
    if current:
        runs.append(current)
    return runs


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
    opp: dict[str, list[tuple[float, float]]] = {}
    ours: list[tuple[float, float]] = []
    for _topic, log_ns, data in iter_messages(mcap, ["/diagnostics/kalman_motion_estimator"]):
        msg = decode_json(data)
        heading = msg.get("our_heading")
        if heading and not heading["held"]:
            values = heading["values"]
            stamp = log_ns / 1e9 - float(values["stamp_age_s"])
            ours.append((stamp, math.radians(float(values["measured_yaw_deg"]))))
        for key, entry in msg.items():
            if not key.startswith("opponent_measurement_THEIR_ROBOT") or entry["held"]:
                continue
            values = entry["values"]
            if int(values["keypoint"]) != 1:
                continue
            opp.setdefault(key[len("opponent_measurement_") :], []).append(
                (float(values["stamp"]), float(values["yaw"]))
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


def _track(run: str, role: str, track_id: str, rows: list[tuple[float, float]]) -> Track:
    if not rows:
        return Track(run, role, track_id, np.zeros(0), np.zeros(0))
    a = np.array(sorted(rows))
    # The same measurement can be logged on two consecutive diagnostics messages.
    keep = np.concatenate([[True], np.diff(a[:, 0]) > 0.010])
    return Track(run, role, track_id, a[keep, 0], a[keep, 1])


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
    train = [n for n, role in SESSIONS.items() if role == "train"]
    holdout = [n for n, role in SESSIONS.items() if role == "holdout"]
    noise, train_report = sysid_noise(train[0])
    _, holdout_report = sysid_noise(holdout[0])

    mcaps = sorted(REPLAYS.glob("*_KALMAN.mcap"))
    if not mcaps:
        raise SystemExit(f"no KALMAN replays under {REPLAYS}; run run_replays.py --mode KALMAN")
    tracks: list[Track] = []
    scenes: dict[str, Scene] = {}
    for mcap in mcaps:
        tr, scene = load_replay(mcap)
        tracks += tr
        scenes[mcap.stem] = scene

    report: dict[str, Any] = {
        "replays": [m.stem for m in mcaps],
        "sysid_train": {train[0]: train_report},
        "sysid_holdout": {holdout[0]: holdout_report},
        "roles": {},
    }
    # Flipped-run lengths pool both roles and the sysid truth runs: each source alone has too few.
    sysid_flipped_s = [n * FRAME_S for n in train_report["flipped_runs_frames"]]
    all_flips = replay_flips(tracks)
    flipped_s = all_flips["flipped_s"] + sysid_flipped_s
    pooled_onset = 0.5 * all_flips["jumps"] / all_flips["measured_s"]
    report["flips"] = {
        "replay_jumps": all_flips["jumps"],
        "replay_flipped_runs": len(all_flips["flipped_s"]),
        "sysid_flipped_runs": len(sysid_flipped_s),
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
            "Provisional until the apartment-floor sessions: noise from the 2026-10-15 Mr Stabs "
            "Mk2 sysid training session for both roles, dropouts and flip onsets from the NHRL "
            "May and MassD KALMAN replays, latency nominal (not measured), frame drops not "
            "measured, randomization ranges from the plan. Generated by "
            "playground/sensing_model/fit_sensing_model.py."
        ),
        "fitted_from": [train[0], *report["replays"]],
        "frame_rate_hz": FRAME_HZ,
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
        "train": {k: v for k, v in next(iter(report["sysid_train"].values())).items()},
        "holdout": {k: v for k, v in next(iter(report["sysid_holdout"].values())).items()},
    }


if __name__ == "__main__":
    main()
