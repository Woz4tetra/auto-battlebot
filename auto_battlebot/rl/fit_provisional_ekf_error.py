"""Fit a provisional our-robot EKF error table and write a sensing model the RL toy can load."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from auto_battlebot.compat import tomllib
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.recording.mcap_io import decode_json, iter_messages

REPO = REPO_ROOT
SESSIONS = (
    (
        "data/recordings/auto_battlebot_mr_stabs_mk2_sysid_zed_box_2026-10-15_00-20-58.mcap",
        "data/calibration/mujoco_sysid/2026-10-15_00-20-58_tag",
    ),
    (
        "data/recordings/auto_battlebot_mr_stabs_mk2_sysid_zed_box_2026-10-15_00-29-00.mcap",
        "data/calibration/mujoco_sysid/2026-10-15_00-29-00_tag",
    ),
)
POSITION_TAU_S = 0.1
MEASURED_GAP_QUANTILES_S = (
    0.040,
    0.041,
    0.043,
    0.045,
    0.050,
    0.056,
    0.144,
    0.338,
    0.497,
    1.423,
    5.776,
)
ACF_LAG_S = 0.1


def ekf_rows(mcap: Path) -> pd.DataFrame:
    rows = []
    for _topic, log_ns, data in iter_messages(mcap, ["/diagnostics/kalman_motion_estimator"]):
        d = decode_json(data)
        h = d.get("our_heading")
        if not h or h.get("held") or h["values"].get("frame_id") != "OUR_ROBOT_1":
            continue
        v = h["values"]
        if not v.get("accepted"):
            continue
        rows.append(
            {
                "stamp_ns": log_ns - int(float(v["stamp_age_s"]) * 1e9),
                "theta": np.deg2rad(float(v["state_theta_deg"])),
                "v": float(v["state_v"]),
                "omega": np.deg2rad(float(v["state_w_dps"])),
            }
        )
    return pd.DataFrame(rows).sort_values("stamp_ns").drop_duplicates("stamp_ns")


def errors(mcap: Path, bundle: Path) -> pd.DataFrame:
    ekf = ekf_rows(mcap)
    truth = pd.read_csv(bundle / "smoothed.csv")
    truth = truth[truth["upside_down"] == 0]
    t = truth["stamp_ns"].to_numpy(dtype=np.int64)
    s = ekf["stamp_ns"].to_numpy(dtype=np.int64)
    inside = (s >= t[0]) & (s <= t[-1])
    ekf, s = ekf[inside], s[inside]

    def at(col: str) -> np.ndarray:
        return np.asarray(np.interp(s.astype(float), t.astype(float), truth[col].to_numpy()))

    yaw = np.unwrap(truth["yaw"].to_numpy())
    true_yaw = np.interp(s.astype(float), t.astype(float), yaw)
    vx, vy = at("vx"), at("vy")
    forward = vx * np.cos(true_yaw) + vy * np.sin(true_yaw)
    out = pd.DataFrame(
        {
            "stamp_s": s / 1e9,
            "speed": np.hypot(vx, vy),
            "theta": (ekf["theta"].to_numpy() - true_yaw + np.pi) % (2 * np.pi) - np.pi,
            "v": ekf["v"].to_numpy() - forward,
            "omega": ekf["omega"].to_numpy() - at("yaw_rate"),
        }
    )
    return out[out["theta"].abs() < np.pi / 2]


TICK_S = 0.02


def lockstep_errors(mcap: Path) -> pd.DataFrame:
    """Per policy step: the C++ estimate minus the sim truth of the same tick."""
    truth, est = [], []
    topics = ["/diagnostics/sim_camera", "/diagnostics/rl_policy_nav"]
    for topic, _log_ns, data in iter_messages(mcap, topics):
        d = decode_json(data)
        if topic == "/diagnostics/sim_camera":
            g = d["ground_truth"]["values"]
            truth.append((g["our_x"], g["our_y"], np.deg2rad(g["our_yaw_deg"])))
            continue
        p = d.get("policy")
        fresh = p is not None and not p["held"] and "our_yaw" in p["values"]
        est.append(p["values"] if fresh else None)
    n = min(len(truth), len(est))
    tr = np.asarray(truth[:n], dtype=float)
    yaw = np.unwrap(tr[:, 2])
    vx, vy, w = (np.gradient(a, TICK_S) for a in (tr[:, 0], tr[:, 1], yaw))
    rows = []
    for k in range(1, n - 1):
        e = est[k]
        if e is None:
            continue
        c, s_ = np.cos(yaw[k]), np.sin(yaw[k])
        dx, dy = e["our_x"] - tr[k, 0], e["our_y"] - tr[k, 1]
        rows.append(
            {
                "stamp_s": k * TICK_S,
                "speed": float(np.hypot(vx[k], vy[k])),
                "yaw_rate": float(abs(w[k])),
                "age": float(e["our_measurement_age_s"]),
                "along": c * dx + s_ * dy,
                "cross": -s_ * dx + c * dy,
                "theta": (e["our_yaw"] - yaw[k] + np.pi) % (2 * np.pi) - np.pi,
                "v": e["our_speed"] - (vx[k] * c + vy[k] * s_),
                "omega": e["our_yaw_rate"] - w[k],
            }
        )
    out = pd.DataFrame(rows)
    if out.empty:
        return out
    fresh = out["age"] - out["age"].min() <= 1.5 * TICK_S
    return out[fresh & (out["theta"].abs() < np.pi / 2)]


def tau_from_acf(series: np.ndarray, stamps: np.ndarray) -> float:
    """Correlation time from the autocorrelation at ACF_LAG_S, on a 10 ms resampled grid."""
    grid = np.arange(stamps[0], stamps[-1], 0.01)
    x = np.interp(grid, stamps, series)
    x = x - x.mean()
    lag = int(round(ACF_LAG_S / 0.01))
    rho = float(np.dot(x[:-lag], x[lag:]) / np.dot(x, x))
    rho = min(max(rho, 0.02), 0.98)
    return float(-ACF_LAG_S / np.log(rho))


def replace_dropped_tables(text: str, base: dict[str, object]) -> str:
    """Every `dropped_s = [[...], ...]` block, rewritten with the measured pooled gap row."""
    import re

    row = "  [" + ", ".join(f"{v:.4g}" for v in MEASURED_GAP_QUANTILES_S) + "],\n"
    count = 0

    def table(match: re.Match[str]) -> str:
        nonlocal count
        count += 1
        cells = match.group(0).count("\n  [")
        return "dropped_s = [\n" + row * cells + "]\n"

    out = re.sub(r"dropped_s = \[\n.*?\n\]\n", table, text, flags=re.DOTALL)
    if count != len(base["robots"]):  # type: ignore[arg-type]
        raise ValueError(f"replaced {count} dropped_s tables, expected one per robot")
    return out


def append_ekf_error(
    text: str, source: str, sigma: dict[str, list[float]], tau: dict[str, float]
) -> str:
    """Add the `[robots.ours.ekf_error]` table; its full-path header is valid after any table."""

    def fmt(values: list[float]) -> str:
        return "[" + ", ".join(f"{v:.5g}" for v in values) + "]"

    keys = ("along", "cross", "theta", "v", "omega")
    if not text.endswith("\n"):
        text += "\n"
    return (
        text
        + "\n# Provisional, from auto_battlebot/rl/fit_provisional_ekf_error.py:\n"
        + source
        + "# Replace with the sensing plan step 5 fit.\n"
        "[robots.ours.ekf_error]\n"
        "sigma = { " + ", ".join(f"{k} = {fmt(sigma[k])}" for k in keys) + " }\n"
        "tau_s = { " + ", ".join(f"{k} = {tau[k]:.4g}" for k in keys) + " }\n"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument(
        "--base", type=Path, default=REPO / "simulation/sensing_models/provisional.toml"
    )
    ap.add_argument(
        "--out", type=Path, default=REPO / "simulation/sensing_models/provisional_rl.toml"
    )
    ap.add_argument(
        "--lockstep",
        type=Path,
        default=None,
        help="directory of recorded lockstep episodes; fit the EKF error from them",
    )
    args = ap.parse_args()

    with open(args.base, "rb") as f:
        base = tomllib.load(f)
    edges = np.asarray(base["robots"]["ours"]["speed_bins_mps"], dtype=float)
    position = base["robots"]["ours"]["position"]

    if args.lockstep is not None:
        mcaps = sorted(args.lockstep.glob("**/*.mcap"))
        frames = [f for f in (lockstep_errors(m) for m in mcaps) if len(f) >= 50]
        components: tuple[str, ...] = ("along", "cross", "theta", "v", "omega")
    else:
        frames = [errors(REPO / m, REPO / b) for m, b in SESSIONS]
        components = ("theta", "v", "omega")
    err = pd.concat(frames, ignore_index=True)
    bins = np.clip(
        np.searchsorted(edges, err["speed"].to_numpy(), side="right") - 1, 0, len(edges) - 1
    )
    sigma: dict[str, list[float]] = {
        "along": [float(x) for x in position["sigma_along_m"]],
        "cross": [float(x) for x in position["sigma_cross_m"]],
    }
    report: dict[str, object] = {
        "samples": int(len(err)),
        "per_bin_counts": np.bincount(bins, minlength=len(edges)).tolist(),
    }
    for comp in components:
        values = err[comp].to_numpy()
        per_bin = []
        for b in range(len(edges)):
            sel = values[bins == b]
            mad = float(np.median(np.abs(sel - np.median(sel)))) if sel.size else 0.0
            per_bin.append(1.4826 * mad if sel.size >= 50 else float("nan"))
        for b in range(len(per_bin)):
            if np.isnan(per_bin[b]):
                per_bin[b] = next(x for x in reversed(per_bin[:b]) if not np.isnan(x))
        sigma[comp] = per_bin
    tau = {"along": POSITION_TAU_S, "cross": POSITION_TAU_S}
    for comp in components:
        taus = [tau_from_acf(f[comp].to_numpy(), f["stamp_s"].to_numpy()) for f in frames]
        tau[comp] = float(np.median(taus))
    report["sigma"] = sigma
    report["tau_s"] = tau

    if args.lockstep is not None:
        source = (
            f"# from the C++ filter in {len(frames)} lockstep episodes ({args.lockstep.name}),\n"
            "# its output at control time against sim truth, all five components measured.\n"
        )
    else:
        source = (
            "# theta, v and omega from the real EKF on the 2026-10-15 sysid sessions (AprilTag\n"
            "# keypoints) against tag truth; along and cross copy the measurement sigmas above\n"
            "# (assumed).\n"
        )
    text = append_ekf_error(replace_dropped_tables(args.base.read_text(), base), source, sigma, tau)
    args.out.write_text(text)
    print(json.dumps(report, indent=2))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
