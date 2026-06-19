"""Batch driver for the headless kinematic sim: one command instead of three processes.

For each run in a sweep it:
  1. builds a per-run C++ config overlay that `extends` the base headless config and overrides only
     the swept fields (the project's overlay config system), and a per-run kinematic sim config
     (deep-merged onto the base TOML);
  2. launches the ROS master (once), the kinematic sim server, and build/auto_battlebot;
  3. waits for the binary to exit (the server closes after [sim].max_ticks);
  4. scores the produced MCAP with the Stage 0 loaders and tabulates the results.

Sim-time metrics are derived from the tick index x [sim].dt, not the wall-clock MCAP timestamps
(the run is accelerated, so wall time != sim time).

Run:
    source scripts/activate_python.sh
    python playground/control_stage0/sim_sweep.py                 # built-in latency sweep
    python playground/control_stage0/sim_sweep.py --sweep my_sweep.toml --out sweep_out
"""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")  # headless: render plots to files
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import tomli_w  # noqa: E402
import tomllib  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import diag_io  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
BINARY = REPO_ROOT / "build" / "auto_battlebot"
MASTER = REPO_ROOT / "build" / "bin" / "miniroscore"
SERVER = REPO_ROOT / "simulation" / "kinematic_sim_server.py"
RECORDINGS = REPO_ROOT / "data" / "recordings"
BASE_CPP_CONFIG = REPO_ROOT / "config" / "headless_sim"  # no extension; overlays `extends` this
BASE_SIM_CONFIG = REPO_ROOT / "simulation" / "kinematic_sim.toml"
MASTER_PORT = 11311
SIM_PORT = 14882


@dataclass
class Run:
    """One sweep run: dotted-key overrides for the sim config and the C++ (cpp) config."""

    name: str
    sim: dict[str, Any] = field(default_factory=dict)
    cpp: dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Config assembly
# ---------------------------------------------------------------------------


def _deep_set(table: dict[str, Any], dotted_key: str, value: Any) -> None:
    """Set table[a][b][c] = value for dotted_key 'a.b.c', creating sub-tables as needed."""
    keys = dotted_key.split(".")
    node = table
    for key in keys[:-1]:
        sub = node.setdefault(key, {})
        if not isinstance(sub, dict):
            raise ValueError(f"override '{dotted_key}' conflicts with a non-table at '{key}'")
        node = sub
    node[keys[-1]] = value


def _load_toml(path: Path) -> dict[str, Any]:
    with open(path, "rb") as f:
        return tomllib.load(f)


def write_sim_config(run: Run, out_dir: Path) -> Path:
    """Deep-merge the run's sim overrides onto the base kinematic config; write a temp TOML."""
    data = _load_toml(BASE_SIM_CONFIG)
    for dotted_key, value in run.sim.items():
        _deep_set(data, dotted_key, value)
    path = out_dir / f"{run.name}.sim.toml"
    with open(path, "wb") as f:
        tomli_w.dump(data, f)
    return path


def write_cpp_overlay(run: Run, out_dir: Path) -> Path:
    """Write a C++ overlay that extends the base headless config and overrides swept fields.

    `extends` is an absolute path so it resolves regardless of where the overlay lives.
    """
    data: dict[str, Any] = {"extends": str(BASE_CPP_CONFIG)}
    for dotted_key, value in run.cpp.items():
        _deep_set(data, dotted_key, value)
    path = out_dir / f"{run.name}.toml"
    with open(path, "wb") as f:
        tomli_w.dump(data, f)
    return path


# ---------------------------------------------------------------------------
# Process orchestration
# ---------------------------------------------------------------------------


def _port_open(port: int, host: str = "127.0.0.1") -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.2)
        return sock.connect_ex((host, port)) == 0


def _wait_for_port(port: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _port_open(port):
            return True
        time.sleep(0.1)
    return False


def ensure_master() -> subprocess.Popen[bytes] | None:
    """Start build/bin/miniroscore if no master is listening. Returns the process we started."""
    if _port_open(MASTER_PORT):
        return None
    proc = subprocess.Popen(
        [str(MASTER)], cwd=REPO_ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    if not _wait_for_port(MASTER_PORT, timeout_s=10.0):
        proc.terminate()
        raise RuntimeError("ROS master (miniroscore) failed to start")
    return proc


def run_once(
    run: Run, sim_config: Path, cpp_overlay: Path, timeout_s: float, log_dir: Path
) -> bool:
    """Launch sim server + binary for one run. Returns True if the binary exited cleanly."""
    log = open(log_dir / f"{run.name}.log", "wb")
    server = subprocess.Popen(
        [sys.executable, str(SERVER), str(sim_config)], cwd=REPO_ROOT, stdout=log, stderr=log
    )
    try:
        if not _wait_for_port(SIM_PORT, timeout_s=15.0):
            raise RuntimeError(f"[{run.name}] sim server did not start")
        result = subprocess.run(
            [str(BINARY), "-c", str(cpp_overlay)],
            cwd=REPO_ROOT,
            stdout=log,
            stderr=log,
            timeout=timeout_s,
            check=False,
        )
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print(f"[{run.name}] binary timed out after {timeout_s}s")
        return False
    finally:
        server.terminate()
        try:
            server.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            server.kill()
        log.close()


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------


def latest_mcap(run_name: str, after: float) -> Path | None:
    candidates = [
        p
        for p in RECORDINGS.glob(f"auto_battlebot_{run_name}_*.mcap")
        if p.stat().st_mtime >= after
    ]
    return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None


def score_run(
    df: pd.DataFrame,
    field_size: tuple[float, float] | None,
    dt: float,
    contact_dist: float,
    wall_margin: float,
) -> dict[str, Any]:
    """Control-quality metrics for a sim run. Times are sim-seconds (tick index x dt)."""
    n = len(df)
    result: dict[str, Any] = {"ticks": n}

    distance = df.get("distance")
    if distance is not None:
        reached_idx = distance.index[distance < contact_dist]
        result["min_dist_m"] = round(float(distance.min()), 3)
        result["reached"] = bool(len(reached_idx) > 0)
        result["t_contact_s"] = round(int(reached_idx[0]) * dt, 2) if len(reached_idx) else None

    if "facing_target" in df.columns:
        result["facing_pct"] = round(100.0 * (df["facing_target"].fillna(0) > 0.5).mean(), 1)
    if "angle_error_deg" in df.columns:
        result["mean_abs_ang_deg"] = round(float(df["angle_error_deg"].abs().mean()), 1)

    if field_size is not None and "our_x" in df.columns:
        half_x, half_y = field_size[0] / 2.0, field_size[1] / 2.0
        wall_dist = pd.concat([half_x - df["our_x"].abs(), half_y - df["our_y"].abs()], axis=1).min(
            axis=1
        )
        result["nearwall_pct"] = round(100.0 * (wall_dist < wall_margin).mean(), 1)

    # Approximate impact speed: our-pose displacement per dt at the first contact tick.
    if distance is not None and result.get("reached") and {"our_x", "our_y"}.issubset(df.columns):
        i = int(distance.index.get_loc(reached_idx[0]))
        if i > 0:
            dx = df["our_x"].iloc[i] - df["our_x"].iloc[i - 1]
            dy = df["our_y"].iloc[i] - df["our_y"].iloc[i - 1]
            result["impact_speed_mps"] = round(float((dx**2 + dy**2) ** 0.5) / dt, 2)
    return result


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------


def _opponent_moves(df: pd.DataFrame) -> bool:
    return {"target_x", "target_y"}.issubset(df.columns) and bool(
        df["target_x"].std() > 0.05 or df["target_y"].std() > 0.05
    )


def _plot_opponent(
    ax: Any, df: pd.DataFrame, color: str = "crimson", label: str = "opponent"
) -> None:
    """Draw the opponent (the chased target pose): a path if it moves, else a single marker."""
    if not {"target_x", "target_y"}.issubset(df.columns):
        return
    ox, oy = df["target_x"], df["target_y"]
    if _opponent_moves(df):
        ax.plot(ox, oy, "--", color=color, lw=1.0, alpha=0.6, label=f"{label} path")
        ax.plot(ox.iloc[0], oy.iloc[0], "X", color=color, ms=11)  # opponent start
    else:
        ax.plot(ox.median(), oy.median(), "X", color=color, ms=14, label=label)


def _draw_arena(ax: Any, field_size: tuple[float, float] | None) -> None:
    if field_size is not None:
        half_x, half_y = field_size[0] / 2.0, field_size[1] / 2.0
        ax.add_patch(
            plt.Rectangle((-half_x, -half_y), 2 * half_x, 2 * half_y, fill=False, ec="red", lw=1.5)
        )


def plot_run(
    df: pd.DataFrame,
    field_size: tuple[float, float] | None,
    dt: float,
    contact_dist: float,
    title: str,
    out_path: Path,
) -> None:
    """Per-run physical view: top-down path (time-coloured) plus distance, heading, speed."""
    t = np.arange(len(df)) * dt
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(title, fontsize=13)

    ax = axes[0, 0]
    if {"our_x", "our_y"}.issubset(df.columns):
        sc = ax.scatter(df["our_x"], df["our_y"], c=t, s=6, cmap="viridis")
        fig.colorbar(sc, ax=ax, label="sim time (s)")
        ax.plot(df["our_x"].iloc[0], df["our_y"].iloc[0], "ko", ms=9, label="our start")
        _plot_opponent(ax, df)
        _draw_arena(ax, field_size)
        ax.set_aspect("equal")
        ax.legend(loc="upper left", fontsize=8)
    ax.set_title("top-down trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    ax = axes[0, 1]
    if "distance" in df.columns:
        ax.plot(t, df["distance"], lw=0.9)
        ax.axhline(contact_dist, color="red", ls="--", lw=1, label=f"contact {contact_dist:.2f} m")
        ax.legend()
    ax.set_title("distance to opponent")
    ax.set_xlabel("sim time (s)")
    ax.set_ylabel("m")

    ax = axes[1, 0]
    if "angle_error_deg" in df.columns:
        ax.plot(t, df["angle_error_deg"].abs(), lw=0.9)
        ax.axhline(90, color="gray", ls=":", lw=1)
    ax.set_title("|heading error|")
    ax.set_xlabel("sim time (s)")
    ax.set_ylabel("deg")

    ax = axes[1, 1]
    if {"our_x", "our_y"}.issubset(df.columns):
        speed = np.sqrt(df["our_x"].diff() ** 2 + df["our_y"].diff() ** 2) / dt
        ax.plot(t, speed, lw=0.9, color="darkorange")
    ax.set_title("actual speed (from pose deltas)")
    ax.set_xlabel("sim time (s)")
    ax.set_ylabel("m/s")

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_combined_trajectories(
    trajectories: list[tuple[str, pd.DataFrame]],
    field_size: tuple[float, float] | None,
    out_path: Path,
) -> None:
    """All runs' paths overlaid in one arena, to compare how the path bends per run."""
    fig, ax = plt.subplots(figsize=(8, 8))
    cmap = plt.get_cmap("turbo")
    denom = max(1, len(trajectories) - 1)
    for i, (name, df) in enumerate(trajectories):
        color = cmap(i / denom)
        if {"our_x", "our_y"}.issubset(df.columns):
            ax.plot(df["our_x"], df["our_y"], "-", lw=1.6, label=name, color=color)
            ax.plot(df["our_x"].iloc[0], df["our_y"].iloc[0], "o", color=color, ms=6)
        # Opponent path (dashed) in the run's colour, so each chaser/evader pair matches.
        if _opponent_moves(df):
            ax.plot(df["target_x"], df["target_y"], "--", lw=1.0, alpha=0.5, color=color)
    df0 = trajectories[0][1]
    if not _opponent_moves(df0) and {"target_x", "target_y"}.issubset(df0.columns):
        ax.plot(df0["target_x"].median(), df0["target_y"].median(), "rX", ms=15, label="opponent")
    _draw_arena(ax, field_size)
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.set_title("Trajectories by run (solid = ours, dashed = opponent)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Sweep spec
# ---------------------------------------------------------------------------


def default_runs() -> list[Run]:
    """Built-in latency sweep against a static opponent at the base gains."""
    return [Run(name=f"lat{ms}", sim={"latency.command_ms": float(ms)}) for ms in (0, 30, 60, 90)]


def load_sweep(path: Path) -> list[Run]:
    raw = _load_toml(path)
    runs = []
    for entry in raw.get("runs", []):
        runs.append(
            Run(name=str(entry["name"]), sim=entry.get("sim", {}), cpp=entry.get("cpp", {}))
        )
    if not runs:
        raise ValueError(f"no [[runs]] found in {path}")
    return runs


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--sweep", type=Path, default=None, help="sweep TOML with [[runs]]; omit for built-in"
    )
    ap.add_argument("--out", type=Path, default=Path("playground/control_stage0/sweep_out"))
    ap.add_argument("--timeout", type=float, default=180.0, help="per-run binary timeout (s)")
    ap.add_argument("--contact-distance", type=float, default=0.15)
    ap.add_argument("--wall-contact-margin", type=float, default=0.13)
    args = ap.parse_args()

    if not BINARY.exists():
        sys.exit(f"binary not found: {BINARY} (run ./scripts/build.sh)")

    runs = load_sweep(args.sweep) if args.sweep else default_runs()
    out_dir = (REPO_ROOT / args.out) if not args.out.is_absolute() else args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    master = ensure_master()
    rows: list[dict[str, Any]] = []
    trajectories: list[tuple[str, pd.DataFrame]] = []
    field_size: tuple[float, float] | None = None
    try:
        for run in runs:
            print(f"=== run: {run.name}  sim={run.sim or '-'}  cpp={run.cpp or '-'} ===")
            sim_config = write_sim_config(run, out_dir)
            cpp_overlay = write_cpp_overlay(run, out_dir)
            dt = float(_load_toml(sim_config).get("sim", {}).get("dt", 1.0 / 30.0))

            started = time.time()
            ok = run_once(run, sim_config, cpp_overlay, args.timeout, out_dir)
            mcap = latest_mcap(run.name, after=started)
            row: dict[str, Any] = {"name": run.name, **run.sim, **run.cpp, "ok": ok}
            if mcap is None:
                print(f"[{run.name}] no MCAP produced; see {out_dir / (run.name + '.log')}")
            else:
                df = diag_io.load_diagnostics(mcap)
                field_size = diag_io.load_field_size(mcap)
                row.update(
                    score_run(df, field_size, dt, args.contact_distance, args.wall_contact_margin)
                )
                title = f"{run.name}  (sim={run.sim or '-'}, cpp={run.cpp or '-'})"
                plot_run(
                    df, field_size, dt, args.contact_distance, title, out_dir / f"{run.name}.png"
                )
                trajectories.append((run.name, df))
                print(f"  wrote {out_dir / (run.name + '.png')}")
            rows.append(row)
    finally:
        if master is not None:
            master.terminate()

    if trajectories:
        plot_combined_trajectories(trajectories, field_size, out_dir / "trajectories.png")
        print(f"wrote {out_dir / 'trajectories.png'}")

    table = pd.DataFrame(rows)
    print("\n" + table.to_string(index=False))
    csv_path = out_dir / "results.csv"
    table.to_csv(csv_path, index=False)
    print(f"\nWrote {csv_path}")


if __name__ == "__main__":
    main()
