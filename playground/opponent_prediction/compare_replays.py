"""Compare HOLD and KALMAN playback replays (plan step 4), and score the filter on match data.

Reads the pairs `run_replays.py` writes to `out/replays/<recording>_<mode>.mcap`.

Per run, from the recording:
- estimator counters: `num_gated`, `num_reinit`, `num_rewind_missed`, opponent heading flips and
  heading re-initializations, summed over perception frames
- identity swaps: consecutive opponent measurements under 0.2 s and 0.15 m apart that carry
  different `THEIR_ROBOT_*` ids. Association runs on the estimator's predicted position, so this
  is where the mode can change who is who
- opponent coverage: share of `/status/tracks` messages with a fresh (not stale) opponent
- navigation: share of `/diagnostics/motion_profile_nav` messages without our robot

On the KALMAN runs' `THEIR_ROBOT_1` measurement stream, offline through the mirror:
- the plan's "against later measurements" metric: periodic dropouts withhold measurements and
  the rendered estimate is scored against them
- the opponent position sigma that makes the mean position NIS 2 on match data, the cage
  counterpart of the sysid tuning

Usage:
    source scripts/activate_python.sh
    python playground/opponent_prediction/compare_replays.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd
import torch

from auto_battlebot.recording.mcap_io import decode_json, iter_messages
from playground.opponent_prediction import evaluate_opponent_prediction as ev
from playground.opponent_prediction.plot_dropout_samples import replay_session
from playground.opponent_prediction.sysid_data import OUT

REPLAYS = OUT / "replays"
SIGMAS_M = [0.01, 0.02, 0.03, 0.05, 0.08, 0.12]
COUNTERS = [
    "num_gated",
    "num_reinit",
    "num_rewind_missed",
    "num_opponent_heading_flips",
    "num_opponent_heading_reinit",
]


def _fresh(msg: dict, key: str) -> dict | None:
    entry = msg.get(key)
    if isinstance(entry, dict) and not entry.get("held", False):
        return entry.get("values", {})
    return None


def run_stats(mcap: Path) -> dict:
    counters = dict.fromkeys(COUNTERS, 0)
    frames = 0
    meas = []
    for _t, _ns, data in iter_messages(mcap, ["/diagnostics/kalman_motion_estimator"]):
        msg = decode_json(data)
        top = _fresh(msg, "kalman_motion_estimator")
        if top is not None:
            frames += 1
            for k in COUNTERS:
                counters[k] += int(top.get(k, 0))
        for key in msg:
            if key.startswith("opponent_measurement_THEIR_ROBOT"):
                values = _fresh(msg, key)
                if values is not None:
                    meas.append({"frame_id": key.removeprefix("opponent_measurement_"), **values})
    swaps = 0
    if meas:
        m = pd.DataFrame(meas).drop_duplicates(["frame_id", "stamp"]).sort_values("stamp")
        st, x, y, fid = (m[c].to_numpy() for c in ("stamp", "x", "y", "frame_id"))
        close = (np.diff(st) < 0.2) & (np.hypot(np.diff(x), np.diff(y)) < 0.15)
        swaps = int((close & (fid[1:] != fid[:-1])).sum())

    tracks = fresh_opponent = 0
    for _t, _ns, data in iter_messages(mcap, ["/status/tracks"]):
        msg = decode_json(data)
        tracks += 1
        robots = msg.get("robots", [])
        fresh_opponent += any(
            not r["ours"] and r["label"] == "opponent" and not r["stale"] for r in robots
        )
    nav = no_robot = 0
    for _t, _ns, data in iter_messages(mcap, ["/diagnostics/motion_profile_nav"]):
        msg = decode_json(data)
        nav += 1
        no_robot += "no_robot" in msg
    return {
        "frames": frames,
        **counters,
        "opponent_measurements": len(meas),
        "identity_swaps": swaps,
        "fresh_opponent_frac": fresh_opponent / tracks if tracks else None,
        "nav_no_robot_frac": no_robot / nav if nav else None,
    }


def match_scores(mcap: Path) -> dict:
    data = replay_session(mcap)
    cfg, sigma = ev.tuned_config()
    keep = ev.scenario_keep(data)
    b = keep.shape[0]
    traces = {
        "hold": ev.simulate_hold(data, keep, cfg.render_lead_s),
        "kalman4": ev.simulate_filter(data, ev.LegacyOpponentFilter(b, cfg), keep, 0.03),
        "phase1": ev.simulate_filter(data, ev.OpponentFilter(b, cfg), keep, sigma),
    }
    scores = {
        arm: {s: ev.score_trace(data, trace, r) for r, s in enumerate(ev.SCENARIOS)}
        for arm, trace in traces.items()
    }
    # Position NIS against the measurement sigma, natural stream only.
    sig = torch.tensor(SIGMAS_M, dtype=torch.float64)
    nis_keep = np.tile(keep[0], (len(SIGMAS_M), 1))
    trace = ev.simulate_filter(
        data, ev.OpponentFilter(len(SIGMAS_M), cfg), nis_keep, sig, record_stride=50
    )
    st = trace.update_stats
    nis = {}
    for r, s in enumerate(SIGMAS_M):
        acc = st["accepted"][:, r]
        nis[f"{s:g}"] = {
            "position_nis_mean": float(np.mean(st["position_nis"][acc, r])),
            "position_nis_median": float(np.median(st["position_nis"][acc, r])),
            "accepted_frac": float(acc[st["measured"][:, r]].mean()),
        }
    return {"measurements": len(data.meas_stamp_ns), "scores": scores, "nis_by_sigma": nis}


def main() -> None:
    torch.set_num_threads(4)
    report: dict = {}
    for kalman in sorted(REPLAYS.glob("*_KALMAN.mcap")):
        name = kalman.name.removesuffix("_KALMAN.mcap")
        hold = REPLAYS / f"{name}_HOLD.mcap"
        entry: dict = {"KALMAN": run_stats(kalman)}
        if hold.exists():
            entry["HOLD"] = run_stats(hold)
        try:
            entry["match"] = match_scores(kalman)
        except ValueError as exc:
            entry["match"] = {"error": str(exc)}
        report[name] = entry
        print(name, json.dumps({k: v for k, v in entry.items() if k != "match"}))
    (OUT / "replay_comparison.json").write_text(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
