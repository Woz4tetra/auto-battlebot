"""Our robot's plant, selected by [our_robot] plant in the sim config."""

from __future__ import annotations

from config.kinematic import ObstacleConfig, OpponentConfig, PlantConfig, PlantType

from plants.base import HazardMonitor, PlantInterface, Pose
from plants.kinematic import KinematicPlant

__all__ = ["HazardMonitor", "KinematicPlant", "PlantInterface", "Pose", "make_plant"]


def make_plant(
    cfg: PlantConfig,
    arena_w: float,
    arena_h: float,
    obstacles: list[ObstacleConfig],
    moving_blocks: list[OpponentConfig],
) -> PlantInterface:
    """`moving_blocks`: the opponents with a hazard_radius, in opponent order."""
    if cfg.plant == PlantType.MUJOCO:
        # Imported here: MuJoCo is an x86-only dependency, and the kinematic plant must not need it.
        from plants.mujoco_plant import MujocoPlant

        return MujocoPlant(cfg, arena_w, arena_h, obstacles, moving_blocks)
    return KinematicPlant(cfg, arena_w, arena_h, obstacles)
