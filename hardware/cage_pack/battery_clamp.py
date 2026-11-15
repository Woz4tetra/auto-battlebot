"""Battery clamp: a printed PETG plate that holds the battery holder and cells down and
carries the OpenUPS on four standoff tubes.

The plate sits on the tray's four 25 mm bosses. Four M3 x 16 screws go down through the
OpenUPS, the tubes and the plate into the bosses' heat-set inserts, so one set of screws
holds the board, the plate and the battery. Taking the OpenUPS off frees the plate and the
cells.

Spec
- Process: FDM, black PETG, printed plate-down with the tubes rising from it (the build
  frame is the print frame). PROCESS = "fdm".
- Frame: cage_layout.py, underside at Z = CLAMP_Z0.
- Holes: 4 x Ø3.4 through plate and tubes on the OpenUPS's 92 x 47 pattern
  (cage_layout.UPS_HOLES).
- Size: the hole pattern + 6 mm margin, 5 mm corners, 3 mm thick. Tubes Ø8 x 6 tall. The
  battery holder (78.84 wide) sits between the boss columns. The plate clears the holder's
  walls by 0.4 and the cells by 2.1; a 2 mm foam strip stuck under the plate holds the
  cells' middle 59 mm down in their cradles.
- Model: features() (recipe.py), so the checked solid and the Onshape Part Studio are the
  same four features: plate sketch and plate, tube sketch and tubes.
"""

from dataclasses import dataclass

import b123d_fix  # noqa: F401
import cage_layout as cl
import recipe  # build123d-part/scripts, on the path when run through scripts/run
from build123d import Part

PROCESS = "fdm"
MATERIAL = "petg"


@dataclass
class Params:
    m3_clearance_dia: float = 3.4
    margin: float = 6.0
    corner: float = 5.0
    tube_d: float = cl.BOSS_D
    foot_chamfer: float = 0.5


def features(p: Params) -> list[dict]:
    """Plate on the OpenUPS hole pattern, then a standoff tube round each hole."""
    xs = [x for x, _ in cl.UPS_HOLES]
    ys = [y for _, y in cl.UPS_HOLES]
    w = max(xs) - min(xs) + 2 * p.margin
    h = max(ys) - min(ys) + 2 * p.margin
    xc, yc = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    zt = cl.CLAMP_Z0 + cl.CLAMP_T
    holes = [{"circle": (x, y, p.m3_clearance_dia)} for x, y in cl.UPS_HOLES]
    ring_midpoints = [(x + (p.m3_clearance_dia + p.tube_d) / 4, y) for x, y in cl.UPS_HOLES]
    return [
        {"op": "sketch", "name": "Plate outline", "plane": "Top", "profiles": [
            {"rect": (xc, yc, w, h), "r": p.corner}, *holes]},
        {"op": "extrude", "name": "Plate", "sketch": "Plate outline", "regions": [(xc, yc)],
         "depth": cl.CLAMP_T, "start": cl.CLAMP_Z0},
        {"op": "sketch", "name": "Tubes", "plane": "Top", "profiles": [
            *[{"circle": (x, y, p.tube_d)} for x, y in cl.UPS_HOLES], *holes]},
        {"op": "extrude", "name": "Standoff tubes", "sketch": "Tubes", "regions": ring_midpoints,
         "depth": cl.UPS_TUBE_H, "start": zt, "mode": "add"},
    ]  # fmt: skip


def build(p: Params) -> Part:
    return recipe.build(features(p))


def sections(p: Params) -> list[str]:
    return [f"y={cl.UPS_HOLES[0][1]}"]


def expect(p: Params) -> dict:
    return {
        "bbox": (92.0 + 2 * p.margin, 47.0 + 2 * p.margin, cl.CLAMP_T + cl.UPS_TUBE_H),
        "holes": {p.m3_clearance_dia: 4},
    }
