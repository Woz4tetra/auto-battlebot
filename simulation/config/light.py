from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class LightConfig:
    type: str = "point"  # "point", "directional", or "ambient"
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 3.0])
    dir: list[float] = field(default_factory=lambda: [-1.0, -1.0, -1.0])
    color: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    intensity: float = 5.0
