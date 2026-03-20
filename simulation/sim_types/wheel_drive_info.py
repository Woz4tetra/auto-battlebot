from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from genesis.engine.entities.rigid_entity import RigidEntity

from config.robot import RobotConfig


@dataclass
class WheelDriveInfo:
    """Resolved wheel DOF indices and kinematic parameters."""

    left_dofs: list[int]
    right_dofs: list[int]
    all_dofs: list[int]
    n_left: int
    n_right: int
    wheel_radius: float
    half_track: float

    @staticmethod
    def from_entity(entity: RigidEntity, cfg: RobotConfig) -> WheelDriveInfo:
        left = [entity.get_joint(j).dofs_idx_local[0] for j in cfg.left_wheel_joints]
        right = [entity.get_joint(j).dofs_idx_local[0] for j in cfg.right_wheel_joints]
        info = WheelDriveInfo(
            left_dofs=left,
            right_dofs=right,
            all_dofs=left + right,
            n_left=len(left),
            n_right=len(right),
            wheel_radius=cfg.wheel_radius,
            half_track=cfg.track_width / 2.0,
        )
        info._configure_gains(entity, cfg)
        return info

    def _configure_gains(self, entity: RigidEntity, cfg: RobotConfig) -> None:
        n = len(self.all_dofs)
        entity.set_dofs_kp(np.full(n, cfg.wheel_kp), dofs_idx_local=self.all_dofs)
        entity.set_dofs_kv(np.full(n, cfg.wheel_kv), dofs_idx_local=self.all_dofs)
        entity.set_dofs_force_range(
            lower=np.full(n, -cfg.wheel_force_limit),
            upper=np.full(n, cfg.wheel_force_limit),
            dofs_idx_local=self.all_dofs,
        )
