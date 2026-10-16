"""Fit the opponent keypoint measurement model the RL training environment injects.

From the Mr Stabs Mk2 sysid sessions against AprilTag truth (`sysid_data.py`):
- position noise per axis, robust sigma and bias, by truth speed bin
- heading noise (robust sigma, flips excluded) and the 180 degree flip rate, by speed bin
- measured-run and dropout-gap lengths at the 30 Hz perception rate

From the KALMAN playback replays of NHRL May and MassD (`run_replays.py`), which have no truth:
- dropout-gap and measured-run lengths of the opponent track, which is what the policy sees
  in a match and what the sysid data underrepresents (one robot, a clean floor)
- the flip rate by label, counted as consecutive measurements under 0.1 s apart whose headings
  differ by more than 150 degrees

Latency is not measurable offline; the nominal 63 ms is recorded with the RL plan's 40 to 80 ms
range. Output: `playground/opponent_prediction/sensing_model_opponent.toml`, the opponent section
of the RL plan's `sensing_model.toml`.

Usage:
    source scripts/activate_python.sh
    python playground/opponent_prediction/fit_opponent_sensing_model.py
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pandas as pd
import tomli_w

from auto_battlebot.perception.sysid_sessions import OUT, SESSIONS, load_session
from auto_battlebot.recording.mcap_io import decode_json, iter_messages

HERE = Path(__file__).resolve().parent
OUTPUT = HERE / "sensing_model_opponent.toml"
SPEED_BINS = [0.0, 0.25, 0.5, 1.0, 1.5]
FRAME_S = 1.0 / 30.0
FLIP_RAD = math.radians(150.0)
GAP_MIN_FRAMES = 2.5
QUANTILES = [0.1, 0.25, 0.5, 0.75, 0.9, 0.95, 0.99]


def _robust(x: np.ndarray) -> float:
    return float(1.4826 * np.median(np.abs(x - np.median(x)))) if x.size else float("nan")


def _wrap(a: np.ndarray) -> np.ndarray:
    return np.asarray(np.angle(np.exp(1j * a)))


def _runs(stamps_s: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Measured-run and gap durations (s) of a 30 Hz stream.

    A gap is a step longer than `GAP_MIN_FRAMES` frames. Single missing frames are left out on
    both sources: decimating the sysid video from 60 fps leaves 50 ms steps, and laptop playback
    drops about one frame per 0.5 s for pacing, neither of which is the robot going unseen.
    """
    t = np.sort(stamps_s)
    d = np.diff(t)
    gap = d > GAP_MIN_FRAMES * FRAME_S
    gaps = d[gap] - FRAME_S
    starts = np.concatenate([[0], np.nonzero(gap)[0] + 1])
    ends = np.concatenate([np.nonzero(gap)[0], [len(t) - 1]])
    runs = t[ends] - t[starts] + FRAME_S
    return runs, gaps


def _quantiles(x: np.ndarray) -> dict[str, float]:
    if x.size == 0:
        return {}
    return {f"p{int(q * 100)}": round(float(np.quantile(x, q)), 4) for q in QUANTILES}


def sysid_model() -> dict:
    pos_x, pos_y, yaw_err, speed = [], [], [], []
    runs, gaps = [], []
    for name in SESSIONS:
        s = load_session(name)
        tr = s.truth_at(s.meas_stamp_ns)
        v = tr["valid"]
        pos_x.append((s.meas_x - tr["x"])[v])
        pos_y.append((s.meas_y - tr["y"])[v])
        yaw_err.append(_wrap(s.meas_yaw - tr["yaw"])[v])
        speed.append(np.hypot(tr["vx"], tr["vy"])[v])
        r, g = _runs(s.meas_stamp_ns / 1e9)
        runs.append(r)
        gaps.append(g)
    ex, ey, ey_aw, sp = (np.concatenate(a) for a in (pos_x, pos_y, yaw_err, speed))
    out: dict = {"speed_bins_mps": SPEED_BINS, "samples": [], "position": {}, "heading": {}}
    sig_x, sig_y, bias_x, bias_y, sig_h, flip = [], [], [], [], [], []
    edges = SPEED_BINS + [math.inf]
    for lo, hi in zip(edges[:-1], edges[1:], strict=True):
        m = (sp >= lo) & (sp < hi)
        out["samples"].append(int(m.sum()))
        sig_x.append(round(_robust(ex[m]), 5))
        sig_y.append(round(_robust(ey[m]), 5))
        bias_x.append(round(float(np.median(ex[m])), 5))
        bias_y.append(round(float(np.median(ey[m])), 5))
        flipped = np.abs(ey_aw[m]) > math.pi / 2
        flip.append(round(float(flipped.mean()), 5))
        sig_h.append(round(_robust(ey_aw[m][~flipped]), 5))
    out["position"] = {
        "sigma_x_m": sig_x,
        "sigma_y_m": sig_y,
        "bias_x_m": bias_x,
        "bias_y_m": bias_y,
    }
    out["heading"] = {"sigma_rad": sig_h, "flip_rate_per_measurement": flip}
    r, g = np.concatenate(runs), np.concatenate(gaps)
    out["dropout"] = {
        "measured_run_s": _quantiles(r),
        "gap_s": _quantiles(g),
        "gap_rate_per_s": round(float(len(g) / (r.sum() + g.sum())), 4),
    }
    return out


def match_model() -> dict:
    replays = sorted((OUT / "replays").glob("*_KALMAN.mcap"))
    rows = []
    for mcap in replays:
        for _t, _ns, data in iter_messages(mcap, ["/diagnostics/kalman_motion_estimator"]):
            msg = decode_json(data)
            for key, entry in msg.items():
                if key.startswith("opponent_measurement_") and not entry.get("held", False):
                    rows.append({"run": mcap.stem, "frame_id": key[21:], **entry["values"]})
    if not rows:
        return {}
    m = pd.DataFrame(rows).drop_duplicates(["run", "frame_id", "stamp"])
    m = m[m["keypoint"].astype(int) == 1].sort_values(["run", "frame_id", "stamp"])
    runs, gaps = [], []
    flips: dict[str, list[int]] = {}
    for (_run, fid), g in m.groupby(["run", "frame_id"]):
        if not str(fid).startswith("THEIR_ROBOT"):
            continue
        st = g["stamp"].to_numpy(float)
        r, gp = _runs(st)
        runs.append(r)
        gaps.append(gp)
        yaw = g["yaw"].to_numpy(float)
        close = np.diff(st) < 0.1
        jump = np.abs(_wrap(np.diff(yaw))) > FLIP_RAD
        label = str(g["label"].iloc[0])
        flips.setdefault(label, [0, 0])
        flips[label][0] += int((close & jump).sum())
        flips[label][1] += int(close.sum())
    r, g = np.concatenate(runs), np.concatenate(gaps)
    return {
        "replays": [p.stem for p in replays],
        "dropout": {
            "measured_run_s": _quantiles(r),
            "gap_s": _quantiles(g),
            "gap_rate_per_s": round(float(len(g) / (r.sum() + g.sum())), 4),
        },
        "heading_jump_rate_by_label": {
            label: round(n / max(total, 1), 5) for label, (n, total) in flips.items()
        },
    }


def main() -> None:
    model = {
        "note": (
            "Opponent keypoint measurement model. sysid: Mr Stabs Mk2 YOLO keypoints against "
            "AprilTag truth, 2026-10-15 sessions. match: NHRL May and MassD KALMAN replays, no "
            "truth. Generated by playground/opponent_prediction/fit_opponent_sensing_model.py."
        ),
        "frame_rate_hz": 30.0,
        "latency": {"nominal_s": 0.063, "range_s": [0.04, 0.08]},
        "sysid": sysid_model(),
        "match": match_model(),
    }
    OUTPUT.write_text(tomli_w.dumps(model))
    print(OUTPUT.read_text())


if __name__ == "__main__":
    main()
