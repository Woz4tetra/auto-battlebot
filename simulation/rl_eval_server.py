"""Lockstep server for RL evaluation scenarios (plan gates G4 and G5).

Serves one scenario exported by `WarpNavEnv.export_scenarios` to one C++ client, then writes the
episode's score as a JSON line and exits. The world is `auto_battlebot.rl.lockstep_world`: the
Warp environment's arena, opponent behaviours, command chain and scoring on CPU MuJoCo, so the
real C++ navigation (`RlPolicyNavigation` or `MotionProfileNavigation`) is scored on the same
episodes, with the same numbers, as the Python evaluation.

Perception is the sensing model at the tick rate, as in `kinematic_sim_server.py`, with one
measurement slot per robot: ours, the opponent, and the house bot when the scenario has one.

    python simulation/rl_eval_server.py --scenarios s.json --index 3 --port 14890 \\
        --results results.jsonl --label motion_profile
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
from dataclasses import asdict
from pathlib import Path

import numpy as np
import torch
from config.kinematic import KinematicSimConfig, OpponentConfig, PerceptionConfig
from kinematic_sim_server import KinematicServer
from protocol import REQUEST_FMT, REQUEST_SIZE, configure_socket, recv_all

from auto_battlebot.rl.envs.warp_nav import HOUSE_RADIUS_M
from auto_battlebot.rl.lockstep_world import LockstepWorld
from auto_battlebot.rl.torch_plant import saturate

REPO = Path(__file__).resolve().parents[1]
TICK_S = 0.02


def sim_config(scenario: dict, port: int) -> KinematicSimConfig:
    cfg = KinematicSimConfig()
    cfg.server.port = port
    cfg.sim.dt = TICK_S
    cfg.sim.seed = int(scenario["seed"])
    cfg.arena.width = 2 * scenario["arena_half"][0]
    cfg.arena.height = 2 * scenario["arena_half"][1]
    cfg.viewer.enable = False
    cfg.perception = PerceptionConfig(
        sensing_model="simulation/sensing_models/provisional_rl.toml", randomize=False
    )
    opp = OpponentConfig(behavior="static", start_pos=scenario["opp_pose"][:2])
    opp.hazard_radius = 0.0
    cfg.opponents = [opp]
    if scenario["house"]["present"]:
        house = OpponentConfig(
            behavior="static",
            start_pos=[scenario["house"]["x"], scenario["house"]["y"]],
            sprite="house_bot",
        )
        house.hazard_radius = HOUSE_RADIUS_M
        cfg.opponents.append(house)
    return cfg


class RlEvalServer(KinematicServer):
    def __init__(self, scenario: dict, port: int) -> None:
        self.scenario = scenario
        super().__init__(sim_config(scenario, port))

    def _render_camera(self) -> None:
        # The C++ sim profile reads measurements off the socket, not pixels; skip the compositor.
        return

    def run_one(self, results: Path, label: str) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._cfg.server.host, self._cfg.server.port))
        srv.listen(1)
        print(
            f"rl_eval_server ready on {self._cfg.server.host}:{self._cfg.server.port}", flush=True
        )
        conn, _ = srv.accept()
        configure_socket(conn)
        world = LockstepWorld(self.scenario)
        from kinematic_sim_server import Perception

        perception = Perception(self._cfg)
        self._sim_time = 0.0
        ticks = 0
        try:
            while not world.done:
                data = recv_all(conn, REQUEST_SIZE)
                linear, _lateral, angular = struct.unpack(REQUEST_FMT, data)
                cmd = saturate(torch.tensor([[linear, angular]], dtype=torch.float64))[0]
                world.step(float(cmd[0]), float(cmd[1]), TICK_S)
                truth = [world.pose(), world.opponent_pose()]
                house = world.house_pose()
                if house is not None:
                    truth.append(house)
                self._sim_time += TICK_S
                ticks += 1
                self._send_frame(conn, truth, perception.observe(self._sim_time, truth))
        except (ConnectionError, BrokenPipeError, OSError) as e:
            print(f"client disconnected: {e}", flush=True)
            world.score.outcome = "CLIENT_LOST"
        finally:
            conn.close()
            srv.close()
        row = {
            "label": label,
            "index": self.scenario["seed"],
            "ticks": ticks,
            **asdict(world.score),
        }
        row = {k: (None if isinstance(v, float) and math.isinf(v) else v) for k, v in row.items()}
        with open(results, "a") as f:
            f.write(json.dumps(row) + "\n")
        print(json.dumps(row), flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--scenarios", type=Path, required=True)
    ap.add_argument("--index", type=int, required=True)
    ap.add_argument("--port", type=int, default=14882)
    ap.add_argument("--results", type=Path, required=True)
    ap.add_argument("--label", default="")
    args = ap.parse_args()
    scenario = json.loads(args.scenarios.read_text())[args.index]
    np.random.seed(scenario["seed"])
    RlEvalServer(scenario, args.port).run_one(args.results, args.label)


if __name__ == "__main__":
    main()
