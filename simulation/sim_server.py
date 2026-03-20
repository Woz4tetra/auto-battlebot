"""Genesis simulation server for auto-battlebot.

Loads a scene from a TOML config, runs physics + rendering in Genesis,
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

import genesis as gs

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

    def _shutdown_on_sigterm(_signum: int, _frame: object | None) -> None:
        print("Caught shutdown signal")
        runner.close_diagnostics()
        gs.destroy()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown_on_sigterm)

    runner.serve_forever()


if __name__ == "__main__":
    main()
