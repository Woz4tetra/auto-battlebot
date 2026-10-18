"""Run an exported policy bundle in Python, the way `RlPolicyNavigation` runs it on the robot.

`simulation/kinematic_sim_server.py` drives `[[opponents]] behavior = "policy"` with it, so the
opponent can run the same bundle as our robot. It reads the manifest, evaluates the RTNeural actor
in numpy (`rtneural.run_json`), decodes with `scale * tanh`, and keeps the 8-step action and speed
history the observation carries, pushed after each step as the C++ node pushes it. Loading runs
the bundle's self-test, as the C++ loader does.

The caller hands it ground truth: the target's sigmas, ages and stale flags read 0, which is the
observation the training environment builds with sensing off (`WarpNavEnv._observe`).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

from auto_battlebot.compat import tomllib
from auto_battlebot.rl import obs_spec
from auto_battlebot.rl.export.bundle import read_selftest
from auto_battlebot.rl.export.rtneural import run_json

Pose = tuple[float, float, float]  # x, y, yaw in the field frame


@dataclass
class Hazard:
    """One disc the observation's rays and hazard slots see, field frame."""

    x: float
    y: float
    radius: float
    kind: int  # obs_spec.HOLE, WALL_BLOCK or TRACKED
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class PolicyView:
    """Everything one policy step reads, field frame. `our_*` is the robot the policy drives."""

    our_pose: Pose
    our_speed: float  # forward, m/s
    our_yaw_rate: float  # rad/s
    target_pose: Pose
    target_velocity: tuple[float, float]  # m/s
    target_yaw_rate: float  # rad/s
    target_half_diagonal: float  # m
    field_half: tuple[float, float]  # m
    hazards: list[Hazard]


class BundlePolicy:
    def __init__(self, bundle: Path) -> None:
        manifest = tomllib.loads((bundle / "manifest.toml").read_text())
        if manifest["obs_spec_version"] != obs_spec.OBS_SPEC_VERSION:
            raise ValueError(
                f"{bundle}: obs spec {manifest['obs_spec_version']}, this code builds "
                f"{obs_spec.OBS_SPEC_VERSION}"
            )
        if manifest["decoder"]["kind"] != "SQUASHED":
            raise ValueError(f"{bundle}: decoder {manifest['decoder']['kind']}, need SQUASHED")
        (actor,) = [n for n in manifest["networks"] if n["name"] == "actor"]
        self._model = json.loads((bundle / actor["file"]).read_text())
        self._scale = np.asarray(manifest["decoder"]["scale"], dtype=np.float64)
        self.run_id = str(manifest["run_id"])
        obs, expected = read_selftest(bundle / manifest["selftest"]["file"])
        err = float(np.abs(self._decode(obs) - expected).max())
        if err > manifest["selftest"]["tolerance"]:
            raise ValueError(f"{bundle}: self-test off by {err:.3g}")
        self.reset()

    def reset(self) -> None:
        self._hist_action = np.zeros((obs_spec.HISTORY, 2))
        self._hist_speed = np.zeros(obs_spec.HISTORY)
        self._hist_yaw_rate = np.zeros(obs_spec.HISTORY)

    def _decode(self, obs: np.ndarray) -> np.ndarray:
        decoded: np.ndarray = self._scale * np.tanh(run_json(self._model, obs))
        return decoded

    def act(self, view: PolicyView) -> tuple[float, float]:
        """(linear, angular) command fractions for one policy step."""
        action = self._decode(self.observe(view))[0]
        self._hist_action = np.roll(self._hist_action, 1, axis=0)
        self._hist_speed = np.roll(self._hist_speed, 1)
        self._hist_yaw_rate = np.roll(self._hist_yaw_rate, 1)
        self._hist_action[0] = action
        self._hist_speed[0] = view.our_speed
        self._hist_yaw_rate[0] = view.our_yaw_rate
        return float(action[0]), float(action[1])

    def observe(self, view: PolicyView) -> np.ndarray:
        """The [1, 138] observation for `view` and the current history."""

        def t(x: object, dtype: torch.dtype = torch.float64) -> torch.Tensor:
            return torch.tensor(x, dtype=dtype)[None]

        zero = t(0.0)
        false = t(False, torch.bool)
        hazards = view.hazards
        inp = obs_spec.ObsInputs(
            our_xy=t(view.our_pose[:2]),
            our_yaw=t(view.our_pose[2]),
            our_speed=t(view.our_speed),
            our_yaw_rate=t(view.our_yaw_rate),
            our_age_s=zero,
            our_stale=false,
            run_away=false,
            hist_action=t(self._hist_action),
            hist_speed=t(self._hist_speed),
            hist_yaw_rate=t(self._hist_yaw_rate),
            opp_present=t(True, torch.bool),
            opp_xy=t(view.target_pose[:2]),
            opp_yaw=t(view.target_pose[2]),
            opp_velocity=t(view.target_velocity),
            opp_yaw_rate=t(view.target_yaw_rate),
            opp_position_sigma=zero,
            opp_heading_sigma=zero,
            opp_yaw_rate_sigma=zero,
            opp_half_diagonal=t(view.target_half_diagonal),
            opp_age_s=zero,
            opp_stale=false,
            field_half=t(view.field_half),
            hazard_xy=t([(h.x, h.y) for h in hazards]).reshape(1, -1, 2),
            hazard_radius=t([h.radius for h in hazards]).reshape(1, -1),
            hazard_kind=t([h.kind for h in hazards], torch.long).reshape(1, -1),
            hazard_velocity=t([(h.vx, h.vy) for h in hazards]).reshape(1, -1, 2),
            hazard_present=torch.ones(1, len(hazards), dtype=torch.bool),
        )
        return obs_spec.build_v1(inp).numpy()
