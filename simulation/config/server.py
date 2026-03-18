from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ServerConfig:
    host: str = "127.0.0.1"
    port: int = 14882
    show_viewer: bool = True
    physics_dt: float = 0.01
