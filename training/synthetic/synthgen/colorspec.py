"""Color parsing and color-to-material matching (no Blender imports)."""

from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from synthgen.constants import COLOR_MATCH_FALLBACK_MAX_DIST

Rgb = tuple[int, int, int]


class MaterialMatch(NamedTuple):
    material: str | None
    distance: float
    used_fallback: bool


@dataclass(frozen=True)
class ColorMappingEntry:
    """One ``[[robots.color_mapping]]`` row: a part color mapped to a material."""

    color: Rgb
    tolerance: float
    material: str


def color_from_material_name(name: str) -> Rgb | None:
    """Parse RGB from material names like 0.501961_0.501961_0.501961_."""
    parts = name.split("_")
    if len(parts) >= 3:
        try:
            r, g, b = float(parts[0]), float(parts[1]), float(parts[2])
            if all(0.0 <= v <= 1.0 for v in (r, g, b)):
                return (int(r * 255), int(g * 255), int(b * 255))
        except ValueError:
            pass
    return None


def color_distance(c1: Rgb, c2: Rgb) -> float:
    """Euclidean distance between two RGB colors."""
    return float(np.sqrt(sum((a - b) ** 2 for a, b in zip(c1, c2))))


def match_material_type(
    color: Rgb, color_mapping: "tuple[ColorMappingEntry, ...] | list[ColorMappingEntry]"
) -> MaterialMatch:
    """Best material for *color*: in tolerance first, else nearest under the fallback cap."""
    best_match: str | None = None
    best_dist = float("inf")
    nearest_match: str | None = None
    nearest_dist = float("inf")
    for entry in color_mapping:
        dist = color_distance(color, entry.color)
        if dist < nearest_dist:
            nearest_dist = dist
            nearest_match = entry.material
        if dist <= entry.tolerance and dist < best_dist:
            best_dist = dist
            best_match = entry.material
    if best_match is not None:
        return MaterialMatch(best_match, best_dist, False)

    if nearest_match is not None and nearest_dist <= COLOR_MATCH_FALLBACK_MAX_DIST:
        return MaterialMatch(nearest_match, nearest_dist, True)
    return MaterialMatch(None, nearest_dist, False)
