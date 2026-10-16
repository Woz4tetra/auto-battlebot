"""Replay NHRL May and MassD matches with opponent_mode HOLD and KALMAN (plan step 4).

Each recording runs twice through the release binary in real-time playback, once per mode, one
at a time so the two runs see the same GPU load. The overlays live under `out/replay_configs/`
and each run's MCAP is moved to `out/replays/<recording>_<mode>.mcap`. `compare_replays.py`
reads them.

Usage:
    source scripts/activate_python.sh
    python playground/opponent_prediction/run_replays.py            # all recordings
    python playground/opponent_prediction/run_replays.py --only 14-12-27
    python playground/opponent_prediction/run_replays.py --mode KALMAN
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import time
from pathlib import Path

from playground.opponent_prediction.sysid_data import OUT, REPO_ROOT

BINARY = REPO_ROOT / "build/auto_battlebot"
RECORDINGS_DIR = REPO_ROOT / "data/recordings"
MAX_RUN_S = 300

# (svo path, start frame). NHRL start frames are the ones that land field init on match state
# (config/playback/_playback.toml); MassD replays from the top.
RECORDINGS = [
    ("data/svo/NHRL_2026-05-02/2026-05-02T14-12-27.svo2", 1540),
    ("data/svo/NHRL_2026-05-02/2026-05-02T11-45-08.svo2", 13000),
    ("data/svo/NHRL_2026-05-02/2026-05-02T16-18-07.svo2", 3148),
    ("data/svo/NHRL_2026-05-02/2026-05-02T17-26-14.svo2", 6400),
    ("data/svo/MassD_2026-08-29/2026-08-29T09-05-14.svo2", 0),
    ("data/svo/MassD_2026-08-29/2026-08-29T11-17-26.svo2", 0),
    ("data/svo/MassD_2026-08-29/2026-08-29T13-20-08.svo2", 0),
    ("data/svo/MassD_2026-08-29/2026-08-29T15-21-23.svo2", 0),
]
MODES = ("HOLD", "KALMAN")

OVERLAY = """extends = "playback/mrs_buff_mk3_playback"

[rgbd_camera]
svo_file_path = "{svo}"
svo_start_frame = {start}

[mcap]
enable = true

[robot_filter.motion_estimator]
type = "KalmanMotionEstimator"
opponent_mode = "{mode}"
"""


def run(svo: str, start: int, mode: str) -> Path | None:
    name = f"{Path(svo).stem}_{mode}"
    config = OUT / "replay_configs" / f"{name}.toml"
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(OVERLAY.format(svo=svo, start=start, mode=mode))
    before = set(RECORDINGS_DIR.glob("*.mcap"))
    log = OUT / "replays" / f"{name}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.monotonic()
    with open(log, "w") as f:
        proc = subprocess.Popen(
            [str(BINARY), "-c", str(config)], cwd=REPO_ROOT, stdout=f, stderr=subprocess.STDOUT
        )
        try:
            proc.wait(timeout=MAX_RUN_S)
        except subprocess.TimeoutExpired:
            proc.send_signal(subprocess.signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
    elapsed = time.monotonic() - t0
    new = sorted(set(RECORDINGS_DIR.glob("*.mcap")) - before, key=lambda p: p.stat().st_size)
    if not new:
        print(f"{name}: no recording ({elapsed:.0f} s, exit {proc.returncode}); see {log}")
        return None
    # The recorder splits video into a second file; keep the main one (the diagnostics).
    dest_dir = OUT / "replays"
    kept = None
    for path in new:
        suffix = "_video" if "__" in path.name else ""
        dest = dest_dir / f"{name}{suffix}.mcap"
        shutil.move(str(path), dest)
        if not suffix:
            kept = dest
    print(f"{name}: {elapsed:.0f} s, exit {proc.returncode}, {kept}")
    return kept


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--only", help="substring of the recordings to run")
    parser.add_argument("--rerun-hold", action="store_true", help="redo HOLD runs that exist")
    parser.add_argument("--mode", choices=MODES, help="run only this opponent_mode")
    args = parser.parse_args()
    if not BINARY.exists():
        raise SystemExit(f"{BINARY} not built; run scripts/build.sh")
    for svo, start in RECORDINGS:
        if args.only and args.only not in svo:
            continue
        for mode in (args.mode,) if args.mode else MODES:
            # HOLD runs do not exercise the opponent filter, so a filter change keeps them.
            done = OUT / "replays" / f"{Path(svo).stem}_{mode}.mcap"
            if mode == "HOLD" and done.exists() and not args.rerun_hold:
                print(f"{done.name}: kept")
                continue
            run(svo, start, mode)


if __name__ == "__main__":
    main()
