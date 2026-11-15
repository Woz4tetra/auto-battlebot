"""Reproduce the 2026-10-25 reverse freezes in lockstep (nav scoring plan, section 0.1)."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
RECORDINGS = tuple(
    REPO / f"data/recordings/auto_battlebot_mr_stabs_mk2_fid_rl_zed_box_{stamp}.mcap"
    for stamp in ("2026-10-25_22-09-22", "2026-10-26_00-28-40", "2026-10-26_00-49-06")
)
BUNDLE = REPO / "data/models/nav_policies/rl_nav_p4"
FIELD_SLEW_PER_S = 2000.0 / 1500.0
FIELD_GUARD_SCALE = 0.48
TICK_S = 0.02
ARM_NAMES = ("both", "slew", "guard", "neither")


def arms(guard_scale: float = FIELD_GUARD_SCALE) -> dict[str, tuple[float, float]]:
    """Arm name -> (slew per s, guard scale)."""
    return {
        "both": (FIELD_SLEW_PER_S, guard_scale),
        "slew": (FIELD_SLEW_PER_S, 1.0),
        "guard": (0.0, guard_scale),
        "neither": (0.0, 1.0),
    }


@dataclass
class Windows:
    """Reverse windows with wheel samples and commanded and sent linear commands."""

    seconds: list[float]
    wheel: list[np.ndarray]
    commanded: list[np.ndarray]
    sent: list[np.ndarray]

    @classmethod
    def empty(cls) -> Windows:
        return cls([], [], [], [])

    def extend(self, other: Windows) -> None:
        self.seconds += other.seconds
        self.wheel += other.wheel
        self.commanded += other.commanded
        self.sent += other.sent


def reverse_windows(
    t: np.ndarray,
    linear: np.ndarray,
    active: np.ndarray,
    threshold: float = -0.5,
    span_s: float = 0.5,
    min_s: float = 0.5,
) -> list[tuple[int, int]]:
    """(first, last) policy-tick indices of each reverse window."""
    if len(t) == 0:
        return []
    sums = np.concatenate([[0.0], np.cumsum(linear)])
    lo = np.searchsorted(t, t - span_s)
    hi = np.arange(1, len(t) + 1)
    trailing = (sums[hi] - sums[lo]) / (hi - lo)
    mask = (trailing < threshold) & active
    out: list[tuple[int, int]] = []
    start = None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if start is not None and (not m or i == len(mask) - 1):
            end = i if m else i - 1
            if t[end] - t[start] >= min_s:
                out.append((start, end))
            start = None
    return out


def collect(
    t: np.ndarray,
    linear: np.ndarray,
    active: np.ndarray,
    wheel_t: np.ndarray,
    wheel: np.ndarray,
    sent_t: np.ndarray,
    sent: np.ndarray,
) -> Windows:
    w = Windows.empty()
    for a, b in reverse_windows(t, linear, active):
        lo, hi = t[a], t[b]
        w.seconds.append(float(hi - lo))
        w.wheel.append(wheel[(wheel_t >= lo) & (wheel_t <= hi)])
        w.commanded.append(linear[a : b + 1])
        w.sent.append(sent[(sent_t >= lo) & (sent_t <= hi)])
    return w


def summarize(w: Windows, resamples: int = 2000, seed: int = 0) -> dict[str, float]:
    """Pooled median wheel command with a window-resampled 95% CI, plus mean commands."""
    if not w.wheel:
        return {"windows": 0, "seconds": 0.0}
    pooled = np.concatenate(w.wheel)
    rng = np.random.default_rng(seed)
    medians = []
    for _ in range(resamples):
        pick = rng.integers(0, len(w.wheel), len(w.wheel))
        sample = np.concatenate([w.wheel[i] for i in pick])
        if len(sample):
            medians.append(float(np.nanmedian(sample)))
    lo, hi = np.percentile(medians, [2.5, 97.5])
    return {
        "windows": len(w.wheel),
        "seconds": float(sum(w.seconds)),
        "median_wheel_pct": float(np.nanmedian(pooled)),
        "ci_low": float(lo),
        "ci_high": float(hi),
        "mean_commanded": float(np.mean(np.concatenate(w.commanded))),
        "mean_sent": float(np.mean(np.concatenate(w.sent))),
        "chatter": float(np.mean(np.concatenate([np.abs(np.diff(c)) for c in w.commanded]))),
    }


def field_windows(path: Path) -> tuple[Windows, float]:
    """The reverse windows in one recording, and the median guard scale inside them."""
    from auto_battlebot.recording.diag_io import iter_diagnostic_statuses
    from auto_battlebot.recording.mcap_io import iter_messages

    policy: list[tuple[float, float]] = []
    sent: list[tuple[float, float]] = []
    autonomy: list[tuple[float, float]] = []
    for stamp, status in iter_diagnostic_statuses(path):
        hw, name, values = status.get("hardware_id"), status.get("name"), status.get("values", {})
        s = stamp * 1e-9
        if hw == "rl_policy_nav" and name == "policy":
            policy.append((s, float(values["linear"])))
        elif hw == "espnow_transmitter" and name == "process":
            sent.append((s, float(values["linear_out"])))
        elif hw == "runner" and "autonomy_on_s" in values:
            autonomy.append((s, float(values["autonomy_on_s"])))
    telemetry: list[tuple[float, float, float]] = []
    for _topic, stamp, message in iter_messages(path, ["/robot/espnow_telemetry"]):
        row = json.loads(bytes(message))
        if row["left_cmd"] is None or row["right_cmd"] is None:
            continue
        scale = row["guard_scale"] if row["guard_scale"] is not None else np.nan
        telemetry.append((stamp * 1e-9, 0.5 * (row["left_cmd"] + row["right_cmd"]), scale))
    pol, snt, aut, tel = (np.asarray(x, dtype=float) for x in (policy, sent, autonomy, telemetry))
    k = np.clip(np.searchsorted(aut[:, 0], pol[:, 0]), 0, len(aut) - 1)
    active = aut[k, 1] > 0.0
    w = collect(pol[:, 0], pol[:, 1], active, tel[:, 0], tel[:, 1], snt[:, 0], snt[:, 1])
    scales = [
        tel[(tel[:, 0] >= pol[a, 0]) & (tel[:, 0] <= pol[b, 0]), 2]
        for a, b in reverse_windows(pol[:, 0], pol[:, 1], active)
    ]
    guard = float(np.nanmedian(np.concatenate(scales))) if scales else float("nan")
    return w, guard


def run_field(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    pooled = Windows.empty()
    rows = {}
    for path in RECORDINGS:
        w, guard = field_windows(path)
        rows[path.name] = {**summarize(w), "median_guard_scale": guard}
        pooled.extend(w)
        print(path.name, json.dumps(rows[path.name]))
    rows["pooled"] = summarize(pooled)
    print("pooled", json.dumps(rows["pooled"]))
    (out_dir / "field.json").write_text(json.dumps(rows, indent=1))


def run_sim(
    scenarios: Path,
    out_dir: Path,
    bundle: Path,
    workers: int,
    limit: int | None,
    guard_scale: float,
    names: list[str],
) -> None:
    from playground.rl.lockstep_eval import run

    out_dir.mkdir(parents=True, exist_ok=True)
    base = json.loads(scenarios.read_text())
    chosen = {k: v for k, v in arms(guard_scale).items() if k in names}
    record = out_dir / "arms.json"
    done = json.loads(record.read_text()) if record.exists() else {}
    record.write_text(json.dumps({**done, **chosen}, indent=1))
    for arm, (slew, guard) in chosen.items():
        arm_scenarios = out_dir / f"scenarios_{arm}.json"
        arm_scenarios.write_text(
            json.dumps(
                [{**s, "slew_per_s": slew, "guard_scale": guard, "trace": True} for s in base]
            )
        )
        print(f"arm {arm}: slew {slew:.3f}/s, guard scale {guard}")
        run(
            arm_scenarios,
            "rl",
            bundle,
            out_dir / f"arm_{arm}.jsonl",
            workers,
            limit,
            port_base=15000,
        )


def sim_windows(results: Path) -> tuple[Windows, int]:
    pooled = Windows.empty()
    episodes = 0
    for line in results.read_text().splitlines():
        row = json.loads(line)
        trace = np.asarray(row["extra"].get("trace", []), dtype=float).reshape(-1, 3)
        if row["outcome"] == "CLIENT_LOST" or len(trace) == 0:
            continue
        episodes += 1
        t = TICK_S * np.arange(len(trace))
        commanded, sent, wheel = trace.T
        pooled.extend(collect(t, commanded, np.ones(len(t), bool), t, wheel, t, sent))
    return pooled, episodes


def run_report(out_dir: Path) -> None:
    field = json.loads((out_dir / "field.json").read_text())["pooled"]
    lines = [
        "| Arm | Slew (/s) | Guard scale | Episodes | Windows | Window s | Median wheel % "
        "(95% CI) | Mean commanded | Mean sent | Chatter |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
        f"| field 10/25 | {FIELD_SLEW_PER_S:.2f} | varies | 3 recordings | {field['windows']} | "
        f"{field['seconds']:.1f} | {field['median_wheel_pct']:.1f} ({field['ci_low']:.1f} to "
        f"{field['ci_high']:.1f}) | {field['mean_commanded']:.2f} | {field['mean_sent']:.2f} | "
        f"{field['chatter']:.3f} |",
    ]
    summary = {"field": field}
    for arm, (slew, guard) in json.loads((out_dir / "arms.json").read_text()).items():
        path = out_dir / f"arm_{arm}.jsonl"
        if not path.exists():
            continue
        w, episodes = sim_windows(path)
        s = summarize(w)
        summary[arm] = {**s, "episodes": episodes}
        if not s["windows"]:
            lines.append(f"| {arm} | {slew:.2f} | {guard} | {episodes} | 0 | 0 | - | - | - | - |")
            continue
        lines.append(
            f"| {arm} | {slew:.2f} | {guard} | {episodes} | {s['windows']} | {s['seconds']:.1f} | "
            f"{s['median_wheel_pct']:.1f} ({s['ci_low']:.1f} to {s['ci_high']:.1f}) | "
            f"{s['mean_commanded']:.2f} | {s['mean_sent']:.2f} | {s['chatter']:.3f} |"
        )
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=1))
    print("\n".join(lines))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("field", help="the reverse-window statistic on the 10/25 recordings")
    f.add_argument("--out-dir", type=Path, required=True)
    s = sub.add_parser("sim", help="run the four lockstep arms")
    s.add_argument("scenarios", type=Path)
    s.add_argument("--out-dir", type=Path, required=True)
    s.add_argument("--bundle", type=Path, default=BUNDLE)
    s.add_argument("--workers", type=int, default=6)
    s.add_argument("--limit", type=int, default=None)
    s.add_argument("--guard-scale", type=float, default=FIELD_GUARD_SCALE)
    s.add_argument("--arms", nargs="+", choices=ARM_NAMES, default=list(ARM_NAMES))
    r = sub.add_parser("report", help="the four-arm table against the field number")
    r.add_argument("--out-dir", type=Path, required=True)
    args = ap.parse_args()
    if args.cmd == "field":
        run_field(args.out_dir)
    elif args.cmd == "sim":
        run_sim(
            args.scenarios,
            args.out_dir,
            args.bundle,
            args.workers,
            args.limit,
            args.guard_scale,
            args.arms,
        )
    else:
        run_report(args.out_dir)


if __name__ == "__main__":
    main()
