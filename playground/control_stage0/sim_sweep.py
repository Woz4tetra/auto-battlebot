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
import diag_io  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import tomli_w  # noqa: E402
import tomllib  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
BINARY = REPO_ROOT / "build" / "auto_battlebot"
MASTER = REPO_ROOT / "build" / "bin" / "miniroscore"
SERVER = REPO_ROOT / "simulation" / "kinematic_sim_server.py"
RECORDINGS = REPO_ROOT / "data" / "recordings"
BASE_CPP_CONFIG = REPO_ROOT / "config" / "simulation" / "kinematic_sim"  # overlays `extends` this
BASE_SIM_CONFIG = REPO_ROOT / "simulation" / "kinematic_sim.toml"
# Episode length for every swept run, in sim ticks. At the 30 Hz sim dt this is 30 s, long enough
# for a goto-stop or a hazard detour to settle and short enough that a 30-run sweep finishes.
SWEEP_MAX_TICKS = 900
MASTER_PORT = 11311
SIM_PORT = 14882


@dataclass
class Run:
    """One sweep run: dotted-key overrides for the sim config and the C++ (cpp) config."""

    name: str
    sim: dict[str, Any] = field(default_factory=dict)
    cpp: dict[str, Any] = field(default_factory=dict)
    # Arena hazards for this run, as [[hazards]]-shaped dicts. Written to one shared TOML that
    # both the sim ([sim] obstacles_file) and the C++ field filter
    # ([field_filter] hazards_file) are pointed at, so the simulated floor and the controller's
    # keep-out discs cannot disagree.
    hazards: list[dict[str, Any]] = field(default_factory=list)
    # Name of another run in this sweep to measure "time lost to avoidance" against. Usually the
    # same geometry with no hazards.
    baseline: str = ""


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


def write_sim_config(run: Run, out_dir: Path, base_sim_config: Path) -> Path:
    """Deep-merge the run's sim overrides onto the base kinematic config; write a temp TOML."""
    data = _load_toml(base_sim_config)
    # A sweep is the opposite of an interactive run, so it overrides both interactive defaults: no
    # window (a sweep would open one per run) and a hard episode cap (the driver waits for the
    # binary to exit, which only happens when the server closes the connection). A sweep that wants
    # longer or shorter episodes sets sim.max_ticks itself, below.
    _deep_set(data, "viewer.enable", False)
    _deep_set(data, "sim.max_ticks", SWEEP_MAX_TICKS)
    for dotted_key, value in run.sim.items():
        _deep_set(data, dotted_key, value)
    path = out_dir / f"{run.name}.sim.toml"
    with open(path, "wb") as f:
        tomli_w.dump(data, f)
    return path


def write_hazard_file(run: Run, out_dir: Path) -> Path | None:
    """Write this run's arena geometry to one file that both sides read.

    The sim loads it through [sim] obstacles_file and the C++ field filter through
    [field_filter] hazards_file. Generating it once per run is what makes drift between the
    simulated floor and the controller's keep-out discs structurally impossible, rather than
    something a startup assert has to catch.
    """
    if not run.hazards:
        return None
    path = out_dir / f"{run.name}.hazards.toml"
    with open(path, "wb") as f:
        tomli_w.dump({"hazards": run.hazards}, f)
    # Both sides resolve a relative hazard path against the project root, so write it that way
    # rather than absolute: the generated configs then read the same as a hand-written one and
    # stay valid if the tree moves. An --out outside the repo keeps its absolute path.
    try:
        return path.resolve().relative_to(REPO_ROOT)
    except ValueError:
        return path.resolve()


def parse_episode_line(log_path: Path) -> dict[str, Any]:
    """Pull the sim server's per-episode summary out of the run log.

    A fall-in ends the run, so it cannot be recovered from the MCAP: the recording simply stops.
    The server prints one `EPISODE ...` line instead.
    """
    if not log_path.exists():
        return {}
    line = ""
    for raw in log_path.read_text(errors="replace").splitlines():
        if raw.startswith("EPISODE "):
            line = raw
    if not line:
        return {}
    fields: dict[str, Any] = {}
    for token in line.split()[1:]:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        try:
            fields[key] = float(value)
        except ValueError:
            fields[key] = value
    out: dict[str, Any] = {}
    if "fell_in" in fields:
        out["fell_in"] = int(fields["fell_in"])
    if "min_clearance" in fields and fields["min_clearance"] == fields["min_clearance"]:
        out["min_clearance_m"] = round(float(fields["min_clearance"]), 4)
    if "block_hits" in fields:
        out["block_hits"] = int(fields["block_hits"])
    if "outcome" in fields:
        out["episode_outcome"] = fields["outcome"]
    if "sim_time" in fields:
        out["episode_s"] = round(float(fields["sim_time"]), 2)
    return out


def add_time_lost(rows: list[dict[str, Any]], runs: list[Run]) -> None:
    """Time lost to avoidance, against the run each entry names as its baseline.

    This is the column that carries the weight: an avoidance layer that never moves scores
    perfectly on hazard entries and minimum clearance and is still useless.
    """
    by_name = {row["name"]: row for row in rows}
    for run in runs:
        if not run.baseline or run.name not in by_name:
            continue
        base = by_name.get(run.baseline)
        if base is None:
            continue
        for metric, column in (("t_to_goal_s", "time_lost_s"), ("t_contact_s", "time_lost_contact_s")):
            mine, theirs = by_name[run.name].get(metric), base.get(metric)
            if mine is None or theirs is None:
                continue
            try:
                by_name[run.name][column] = round(float(mine) - float(theirs), 2)
            except (TypeError, ValueError):
                continue


def write_cpp_overlay(run: Run, out_dir: Path) -> Path:
    """Write a C++ overlay that extends the base headless config and overrides swept fields.

    `extends` is an absolute path so it resolves regardless of where the overlay lives.
    """
    data: dict[str, Any] = {"extends": str(BASE_CPP_CONFIG)}
    # Scoring reads the MCAP the run writes, so the sweep turns recording on itself rather than
    # depending on the base config leaving it on. A run override can still switch it back off.
    _deep_set(data, "mcap.enable", True)
    # A sweep is a batch job: the sim profile inherits ui.enable from the desktop base, which pops
    # a window per run. Force it off here so a sweep does not take over the screen.
    _deep_set(data, "ui.enable", False)
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
    """Newest recording written after `after`. The recorder names files by the (sanitized) config path,
    which for a sweep overlay is a mangled slug, not the run name, so match on `run_name` appearing before
    the timestamp when possible and otherwise fall back to the newest recording (runs are sequential)."""
    fresh = [p for p in RECORDINGS.glob("auto_battlebot_*.mcap") if p.stat().st_mtime >= after]
    if not fresh:
        return None
    named = [p for p in fresh if run_name in p.name]
    pool = named or fresh
    return max(pool, key=lambda p: p.stat().st_mtime)


def score_run(
    df: pd.DataFrame,
    field_size: tuple[float, float] | None,
    dt: float,
    contact_dist: float,
    wall_margin: float,
    goal_tolerance: float,
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
        near = wall_dist < wall_margin
        result["nearwall_pct"] = round(100.0 * near.mean(), 1)
        # Wall-contact episodes = rising edges of the near-wall mask (not per-tick fraction).
        result["wall_contacts"] = int((near.astype(int).diff() == 1).sum())

    # Approximate impact speed: our-pose displacement per dt at the first contact tick.
    if distance is not None and result.get("reached") and {"our_x", "our_y"}.issubset(df.columns):
        i = int(distance.index.get_loc(reached_idx[0]))
        if i > 0:
            dx = df["our_x"].iloc[i] - df["our_x"].iloc[i - 1]
            dy = df["our_y"].iloc[i] - df["our_y"].iloc[i - 1]
            result["impact_speed_mps"] = round(float((dx**2 + dy**2) ** 0.5) / dt, 2)

    # Stop-mission metrics: a static target is a "go to X and stop" goal. Terminal velocity is derived
    # from pose deltas (the sim sends no velocity), averaged over the last few ticks to reject noise.
    static_goal = not _opponent_moves(df) and {"our_x", "our_y", "target_x", "target_y"}.issubset(
        df.columns
    )
    if static_goal and distance is not None and n >= 2:
        tail = min(5, n - 1)
        result["terminal_pos_err_m"] = round(float(distance.iloc[-tail:].mean()), 3)
        speed = np.sqrt(df["our_x"].diff() ** 2 + df["our_y"].diff() ** 2) / dt
        result["terminal_vel_mps"] = round(float(speed.iloc[-tail:].mean()), 3)
        gx, gy = float(df["target_x"].median()), float(df["target_y"].median())
        sx, sy = float(df["our_x"].iloc[0]), float(df["our_y"].iloc[0])
        d_sg = float(np.hypot(gx - sx, gy - sy))
        if d_sg > 1e-6:
            # Overshoot: max travel past the goal along the start->goal axis.
            progress = (df["our_x"] - sx) * (gx - sx) / d_sg + (df["our_y"] - sy) * (gy - sy) / d_sg
            result["overshoot_m"] = round(max(0.0, float(progress.max()) - d_sg), 3)
        within = distance.index[distance < goal_tolerance]
        result["t_to_goal_s"] = round(int(within[0]) * dt, 2) if len(within) else None

    # Tracking-proxy metrics: for a moving target the steady-state distance to it is a cross-track
    # proxy (Stage 3 scenario 2). Measured over the tail half of the run to skip the initial
    # approach transient.
    if _opponent_moves(df) and distance is not None and n >= 4:
        tail_dist = distance.iloc[n // 2 :]
        result["track_err_mean_m"] = round(float(tail_dist.mean()), 3)
        result["track_err_rms_m"] = round(float(np.sqrt(float((tail_dist**2).mean()))), 3)
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


def _draw_hazards(ax: Any, hazards: list[dict[str, Any]]) -> None:
    """Raw hazard geometry as a filled disc; the controller's inflated keep-out is not drawn here
    because the inflation happens on the C++ side and depends on our robot's measured size."""
    for hazard in hazards:
        cx, cy = hazard.get("center", [0.0, 0.0])
        radius = float(hazard.get("radius", 0.0))
        is_hole = hazard.get("kind", "hole") == "hole"
        ax.add_patch(
            plt.Circle(
                (cx, cy),
                radius,
                fc="0.15" if is_hole else "tab:blue",
                ec="k",
                lw=1.0,
                alpha=0.8 if is_hole else 0.5,
                zorder=1,
            )
        )


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
    goal_tolerance: float,
    hazards: list[dict[str, Any]] | None = None,
) -> None:
    """Per-run physical view: top-down path (time-coloured) plus distance, heading, speed.

    When the target is static the run is a "go to X and stop" mission, so the goal-tolerance ring, the
    goal-distance panel, and the terminal speed are drawn to show whether the robot stopped on the goal.
    """
    t = np.arange(len(df)) * dt
    static_goal = not _opponent_moves(df) and {"target_x", "target_y"}.issubset(df.columns)
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(title, fontsize=13)

    ax = axes[0, 0]
    if {"our_x", "our_y"}.issubset(df.columns):
        sc = ax.scatter(df["our_x"], df["our_y"], c=t, s=6, cmap="viridis")
        fig.colorbar(sc, ax=ax, label="sim time (s)")
        ax.plot(df["our_x"].iloc[0], df["our_y"].iloc[0], "ko", ms=9, label="our start")
        _plot_opponent(ax, df, label="goal" if static_goal else "opponent")
        if static_goal:
            gx, gy = df["target_x"].median(), df["target_y"].median()
            ax.add_patch(
                plt.Circle((gx, gy), goal_tolerance, fill=False, ec="crimson", ls=":", lw=1.2)
            )
        _draw_arena(ax, field_size)
        ax.set_aspect("equal")
        ax.legend(loc="upper left", fontsize=8)
    ax.set_title("top-down trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    ax = axes[0, 1]
    if "distance" in df.columns:
        ax.plot(t, df["distance"], lw=0.9)
        if static_goal:
            ax.axhline(
                goal_tolerance,
                color="crimson",
                ls=":",
                lw=1,
                label=f"goal tol {goal_tolerance:.2f} m",
            )
        else:
            ax.axhline(
                contact_dist, color="red", ls="--", lw=1, label=f"contact {contact_dist:.2f} m"
            )
        ax.legend()
    ax.set_title("distance to goal" if static_goal else "distance to opponent")
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
        if static_goal and len(df) >= 2:
            v_term = float(speed.iloc[-min(5, len(df) - 1) :].mean())
            ax.axhline(v_term, color="crimson", ls=":", lw=1, label=f"terminal {v_term:.2f} m/s")
            ax.legend()
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


def load_sweep(path: Path) -> tuple[Path | None, list[Run]]:
    """Parse a sweep TOML. Returns (base sim-config override or None, runs).

    A top-level `sim_config` key selects the per-robot kinematic config (path relative to the repo
    root), e.g. `sim_config = "simulation/kinematic_sim_mr_stabs_mk2.toml"`. Omit it to use the default.
    """
    raw = _load_toml(path)
    sim_config_key = raw.get("sim_config")
    sim_config = (REPO_ROOT / str(sim_config_key)) if sim_config_key else None
    runs = []
    for entry in raw.get("runs", []):
        runs.append(
            Run(
                name=str(entry["name"]),
                sim=entry.get("sim", {}),
                cpp=entry.get("cpp", {}),
                hazards=entry.get("hazards", []),
                baseline=str(entry.get("baseline", "")),
            )
        )
    if not runs:
        raise ValueError(f"no [[runs]] found in {path}")
    return sim_config, runs


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
    ap.add_argument(
        "--sim-config",
        type=Path,
        default=None,
        help="base kinematic sim TOML; overrides the sweep's sim_config and the default",
    )
    ap.add_argument(
        "--goal-tolerance",
        type=float,
        default=0.10,
        help="stop-mission goal radius (m); terminal error and time-to-goal are measured against it",
    )
    args = ap.parse_args()

    if not BINARY.exists():
        sys.exit(f"binary not found: {BINARY} (run ./scripts/build.sh)")

    if args.sweep:
        sweep_sim_config, runs = load_sweep(args.sweep)
    else:
        sweep_sim_config, runs = None, default_runs()
    base_sim_config = args.sim_config or sweep_sim_config or BASE_SIM_CONFIG
    if not base_sim_config.is_absolute():
        base_sim_config = REPO_ROOT / base_sim_config
    if not base_sim_config.exists():
        sys.exit(f"sim config not found: {base_sim_config}")
    print(f"base sim config: {base_sim_config}")
    out_dir = (REPO_ROOT / args.out) if not args.out.is_absolute() else args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    master = ensure_master()
    rows: list[dict[str, Any]] = []
    trajectories: list[tuple[str, pd.DataFrame]] = []
    field_size: tuple[float, float] | None = None
    try:
        for run in runs:
            print(f"=== run: {run.name}  sim={run.sim or '-'}  cpp={run.cpp or '-'} ===")
            hazard_file = write_hazard_file(run, out_dir)
            if hazard_file is not None:
                run.sim = {**run.sim, "obstacles_file": str(hazard_file)}
                run.cpp = {**run.cpp, "field_filter.hazards_file": str(hazard_file)}
            sim_config = write_sim_config(run, out_dir, base_sim_config)
            cpp_overlay = write_cpp_overlay(run, out_dir)
            dt = float(_load_toml(sim_config).get("sim", {}).get("dt", 1.0 / 30.0))

            started = time.time()
            ok = run_once(run, sim_config, cpp_overlay, args.timeout, out_dir)
            mcap = latest_mcap(run.name, after=started)
            row: dict[str, Any] = {"name": run.name, **run.sim, **run.cpp, "ok": ok}
            row["hazard_count"] = len(run.hazards)
            row.update(parse_episode_line(out_dir / f"{run.name}.log"))
            if mcap is None:
                print(f"[{run.name}] no MCAP produced; see {out_dir / (run.name + '.log')}")
            else:
                df = diag_io.load_diagnostics(mcap)
                field_size = diag_io.load_field_size(mcap)
                row.update(
                    score_run(
                        df,
                        field_size,
                        dt,
                        args.contact_distance,
                        args.wall_contact_margin,
                        args.goal_tolerance,
                    )
                )
                title = f"{run.name}  (sim={run.sim or '-'}, cpp={run.cpp or '-'})"
                plot_run(
                    df,
                    field_size,
                    dt,
                    args.contact_distance,
                    title,
                    out_dir / f"{run.name}.png",
                    args.goal_tolerance,
                    run.hazards,
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

    add_time_lost(rows, runs)
    table = pd.DataFrame(rows)
    print("\n" + table.to_string(index=False))
    csv_path = out_dir / "results.csv"
    table.to_csv(csv_path, index=False)
    print(f"\nWrote {csv_path}")


if __name__ == "__main__":
    main()
