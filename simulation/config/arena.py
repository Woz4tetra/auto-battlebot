from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ArenaConfig:
    width: float = 2.4
    height: float = 2.4
    floor_mesh: str = "./assets/cage/floor.obj"
    floor_friction: float = 1.0
    panorama: str | None = None
