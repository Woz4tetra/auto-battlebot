from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class CameraConfig:
    res_width: int = 1280
    res_height: int = 720
    fov: float = 70.0
    far: float = 10.0
    pos: list[float] = field(default_factory=lambda: [0.0, -1.5, 1.8])
    lookat: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
