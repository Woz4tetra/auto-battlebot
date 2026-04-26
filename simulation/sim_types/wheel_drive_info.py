from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import mujoco

from config.robot import RobotConfig


@dataclass
class WheelDriveInfo:
    """Resolved actuator IDs and kinematic parameters for a wheeled robot."""

    left_act_ids: list[int]
    right_act_ids: list[int]
    n_left: int
    n_right: int
    wheel_radius: float
    half_track: float

    @staticmethod
    def from_model(
        model: mujoco.MjModel, cfg: RobotConfig, prefix: str
    ) -> WheelDriveInfo:
        left_ids = [
            model.actuator(f"{prefix}{j}_vel").id for j in cfg.left_wheel_joints
        ]
        right_ids = [
            model.actuator(f"{prefix}{j}_vel").id for j in cfg.right_wheel_joints
        ]
        return WheelDriveInfo(
            left_act_ids=left_ids,
            right_act_ids=right_ids,
            n_left=len(left_ids),
            n_right=len(right_ids),
            wheel_radius=cfg.wheel_radius,
            half_track=cfg.track_width / 2.0,
        )

    @property
    def all_act_ids(self) -> list[int]:
        return self.left_act_ids + self.right_act_ids
