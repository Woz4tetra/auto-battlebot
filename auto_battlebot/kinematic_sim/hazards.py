"""Shared arena obstacle geometry."""

from __future__ import annotations

from pathlib import Path

from auto_battlebot.compat import tomllib
from auto_battlebot.kinematic_sim.config.kinematic import ObstacleConfig

VALID_KINDS = ("hole", "wall_block")
REPO_ROOT = Path(__file__).resolve().parents[2]


def load_hazards(path: Path | str) -> list[ObstacleConfig]:
    """Parse a shared hazard TOML into obstacle configs; a missing file raises."""
    path = Path(path)
    if not path.is_absolute():
        path = REPO_ROOT / path
    if not path.exists():
        raise FileNotFoundError(f"hazard file not found: {path}")
    with open(path, "rb") as f:
        data = tomllib.load(f)

    unknown = set(data) - {"hazards"}
    if unknown:
        raise ValueError(
            f"{path}: unexpected top-level keys {sorted(unknown)}; expected [[hazards]]"
        )

    obstacles: list[ObstacleConfig] = []
    for i, entry in enumerate(data.get("hazards", [])):
        unknown_keys = set(entry) - {"kind", "center", "radius"}
        if unknown_keys:
            raise ValueError(f"{path}: hazard {i} has unexpected keys {sorted(unknown_keys)}")
        kind = str(entry.get("kind", "hole"))
        if kind not in VALID_KINDS:
            raise ValueError(f"{path}: hazard {i} kind '{kind}' not in {VALID_KINDS}")
        center = entry.get("center", [0.0, 0.0])
        if len(center) != 2:
            raise ValueError(f"{path}: hazard {i} center must be [x, y]")
        radius = float(entry.get("radius", 0.0))
        if radius <= 0.0:
            raise ValueError(f"{path}: hazard {i} radius must be > 0")
        obstacles.append(
            ObstacleConfig(kind=kind, center=[float(center[0]), float(center[1])], radius=radius)
        )
    return obstacles
