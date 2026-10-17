"""The RL training opponents (`opponents.py`) as one opponent the kinematic sim can step.

`simulation/kinematic_sim_server.py` uses it for `[[opponents]] behavior = "trained"`. It drives
the opponent with the same code the policy trained
against, one world at a time: `trained_behaviour` picks static, straight, circle, random_walk,
pursuit, corner or replay ("any" draws one per reset, as stage D does), and the parameters (speed,
radius, turn noise, pursuit lead and delay, corner, replay track and start time) are drawn the
way the training environment draws them. The rule that stops an opponent pushing through our
robot comes along too.

Pursuit's reaction delay counts sim ticks of the 20 ms policy step it was trained at; at another
[sim] dt it scales with the tick.
"""

from __future__ import annotations

import math

import torch

from auto_battlebot.rl.envs.opponents import BEHAVIOURS, OUR_HALF_DIAGONAL_M, REPLAY, Opponents
from auto_battlebot.rl.envs.replay_tracks import ReplayTracks

ANY = "any"
START_CLEARANCE_M = 0.3
TRAINED_BEHAVIOURS = (*BEHAVIOURS, ANY)


class TrainedOpponent:
    def __init__(
        self,
        behaviour: str,
        holdout: bool,
        arena_w: float,
        arena_h: float,
        opp_half: tuple[float, float],
        seed: int,
    ) -> None:
        """`opp_half` is the opponent's footprint as half length (along its heading) and half
        width: it keeps the opponent off the walls and stops it at contact with us."""
        if behaviour not in TRAINED_BEHAVIOURS:
            raise ValueError(
                f"unknown trained_behaviour {behaviour!r}; valid: {', '.join(TRAINED_BEHAVIOURS)}"
            )
        self._allowed = (
            tuple(range(len(BEHAVIOURS))) if behaviour == ANY else (BEHAVIOURS.index(behaviour),)
        )
        self._holdout = holdout
        self._half = torch.tensor([[arena_w / 2.0, arena_h / 2.0]], dtype=torch.float64)
        self._opp_half = torch.tensor([opp_half], dtype=torch.float64)
        self._foot = self._opp_half.norm(dim=1)
        gen = torch.Generator().manual_seed(seed)
        self._opp = Opponents(1, torch.device("cpu"), torch.float64, gen)
        self._opp.replay = ReplayTracks(holdout, torch.device("cpu"), torch.float64)
        self._us_xy = torch.zeros(1, 2, dtype=torch.float64)
        self._us_v = torch.zeros(1, 2, dtype=torch.float64)
        self._us_yaw = torch.zeros(1, dtype=torch.float64)

    @property
    def behaviour(self) -> str:
        return BEHAVIOURS[int(self._opp.p.behaviour[0])]

    def reset(self, us_xy: tuple[float, float]) -> tuple[float, float, float] | None:
        """Draw a behaviour and its parameters. Returns the start pose a replay track sets, or
        None to keep the configured start. The sim's robot start is fixed, so a replay start is
        redrawn until it clears our robot by START_CLEARANCE_M (training places us instead)."""
        self._opp.history[:, :, 0] = us_xy[0]
        self._opp.history[:, :, 1] = us_xy[1]
        self._opp.history[:, :, 2:] = 0.0
        clear = float(self._foot[0]) + OUR_HALF_DIAGONAL_M + START_CLEARANCE_M
        start: tuple[float, float, float] | None = None
        for _ in range(50):
            self._opp.reset(
                torch.ones(1, dtype=torch.bool),
                self._allowed,
                torch.ones(1, dtype=torch.float64),
                self._holdout,
            )
            if int(self._opp.p.behaviour[0]) != REPLAY or self._opp.replay is None:
                return None
            xy, yaw = self._opp.replay.sample(
                self._opp.replay_index, self._opp.replay_t, self._half - self._foot[:, None]
            )
            start = float(xy[0, 0]), float(xy[0, 1]), float(yaw[0])
            if math.hypot(start[0] - us_xy[0], start[1] - us_xy[1]) > clear:
                break
        return start

    def observe_us(
        self, xy: tuple[float, float], velocity: tuple[float, float], yaw: float
    ) -> None:
        """Our robot's pose and world-frame velocity this tick: pursuit aims at it, corner turns
        to face it, and every behaviour stops at contact with our footprint."""
        self._us_xy[0, 0], self._us_xy[0, 1] = xy
        self._us_v[0, 0], self._us_v[0, 1] = velocity
        self._us_yaw[0] = yaw

    def step(self, pose: tuple[float, float, float], dt: float) -> tuple[float, float, float]:
        """The opponent's next pose after `dt`."""
        x, y, yaw = pose
        self._opp.record_us(self._us_xy, self._us_v, self._us_yaw)
        v = self._opp.command(
            torch.tensor([[x, y]], dtype=torch.float64),
            torch.tensor([yaw], dtype=torch.float64),
            self._half,
            self._foot,
            dt,
            opp_half=self._opp_half,
        )[0]
        lim_x = float(self._half[0, 0] - self._foot[0])
        lim_y = float(self._half[0, 1] - self._foot[0])
        x = min(max(x + float(v[0]) * dt, -lim_x), lim_x)
        y = min(max(y + float(v[1]) * dt, -lim_y), lim_y)
        yaw = math.atan2(math.sin(yaw + float(v[2]) * dt), math.cos(yaw + float(v[2]) * dt))
        return x, y, yaw
