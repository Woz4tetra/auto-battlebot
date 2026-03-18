from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ArenaConfig:
    width: float = 2.4
    height: float = 2.4
    panorama: str | None = None
