from __future__ import annotations

from dataclasses import dataclass, field

from config.arena import ArenaConfig
from config.camera import CameraConfig
from config.robot import RobotConfig
from config.server import ServerConfig


@dataclass
class SimConfig:
    server: ServerConfig = field(default_factory=ServerConfig)
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    our_robot: RobotConfig = field(default_factory=RobotConfig)
    opponents: list[RobotConfig] = field(default_factory=list)
