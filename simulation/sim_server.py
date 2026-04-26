"""MuJoCo simulation server for auto-battlebot.

Loads a scene from a TOML config, runs physics + rendering in MuJoCo,
and communicates with the C++ pipeline over a TCP socket using a simple
binary protocol.

Usage:
    python sim_server.py sim_config.toml
"""

from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path

from config import SimConfig, load_sim_config
from runner import SimRunner
from scene import build_scene
from sim_types import SceneHandles


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", help="Path to sim_config.toml")
    args: argparse.Namespace = parser.parse_args()

    config_dir: Path = Path(args.config).resolve().parent
    cfg: SimConfig = load_sim_config(args.config)

    handles: SceneHandles = build_scene(cfg, config_dir)

    runner = SimRunner(cfg, handles)

    def _on_signal(_signum: int, _frame: object | None) -> None:
        # Closing the server socket unblocks accept() immediately even under
        # PEP 475 (which would otherwise retry the system call forever).
        runner.shutdown()

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    try:
        runner.serve_forever()
    except KeyboardInterrupt:
        # Fallback for interactive Ctrl-C before serve_forever installs its handler.
        runner.shutdown()


if __name__ == "__main__":
    main()
