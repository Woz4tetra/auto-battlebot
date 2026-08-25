"""Shared arena obstacle geometry.

One TOML file describes the hazards for both sides of a run: the kinematic sim reads it here
(``[sim] obstacles_file``) and the C++ field filter reads the same path
(``[field_filter] hazards_file``) to build its keep-out discs. A single file means the simulated
floor and the controller's model of it cannot drift apart, which two hand-synced copies would.

The path is written repo-relative ("config/hazards/one_hole.toml") because the two configs that
name it have no directory in common. Each side resolves it against the project root itself.

File format, all in field-frame metres:

    [[hazards]]
    kind = "hole"          # hole | wall_block
    center = [0.3, -0.2]
    radius = 0.25          # raw geometry, before any inflation
"""

from __future__ import annotations

from pathlib import Path

import tomllib
from config.kinematic import ObstacleConfig

VALID_KINDS = ("hole", "wall_block")
REPO_ROOT = Path(__file__).resolve().parents[1]


def load_hazards(path: Path | str) -> list[ObstacleConfig]:
    """Parse a shared hazard TOML into obstacle configs. Missing file is an error, not an
    empty list: a typo in the path would otherwise silently remove the hazard from the sim
    while the controller still steered around it."""
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
