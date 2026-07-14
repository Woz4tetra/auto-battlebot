"""Color parsing and color -> material-type matching (no Blender imports).

Robot GLTF exports encode part colors either in Principled BSDF inputs (read on
the Blender side) or in material names like ``0.501961_0.501961_0.501961_...``;
this module holds the pure matching logic against the config's color mapping.
"""

from dataclasses import dataclass

import numpy as np

from synthgen.constants import COLOR_MATCH_FALLBACK_MAX_DIST

Rgb = tuple[int, int, int]


@dataclass(frozen=True)
class ColorMappingEntry:
    """One ``[[robots.color_mapping]]`` row: a part color mapped to a material."""

    color: Rgb
    tolerance: float
    material: str


def color_from_material_name(name: str) -> Rgb | None:
    """Parse RGB from material names like ``0.501961_0.501961_0.501961_...``.

    Args:
        name: Blender material name.

    Returns:
        The 0-255 RGB triple, or None when the name is not color-encoded.
    """
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
) -> tuple[str | None, float, bool]:
    """Return the best material for a color.

    Strategy:
    1) Exact/near match within configured tolerance (inclusive).
    2) Conservative nearest-color fallback when nothing matches tolerance.

    Args:
        color: The part color to match (0-255 RGB).
        color_mapping: Configured color -> material entries.

    Returns:
        ``(material_name, distance, used_fallback)``; material_name is None when
        no entry is close enough even for the fallback.
    """
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
        return best_match, best_dist, False

    # CAD exports can introduce close shade variants for the same part color.
    if nearest_match is not None and nearest_dist <= COLOR_MATCH_FALLBACK_MAX_DIST:
        return nearest_match, nearest_dist, True
    return None, nearest_dist, False
