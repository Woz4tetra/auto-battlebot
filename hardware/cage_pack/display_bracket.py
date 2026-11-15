"""Display bracket that clamps the touch display to the lid."""

from dataclasses import dataclass

import b123d_fix  # noqa: F401
import cage_layout as cl
import recipe
from build123d import Part

PROCESS = "fdm"
MATERIAL = "petg"
Z0 = cl.DISP_BACK_Z - cl.DISP_BRACKET_T


GLASS_INSET = 1.03
EAR_MARGIN_PAST_SCREW_X = 6.0
EAR_MARGIN_PAST_SCREW_Y = 4.5
EAR_FRAME_OVERLAP = 1.0


@dataclass
class Params:
    m25_clearance_dia: float = 2.9
    m3_clearance_dia: float = 3.4
    frame_x: tuple = (
        cl.DISP_GLASS_X_FROM_TOUCH[0] + GLASS_INSET,
        cl.DISP_GLASS_X_FROM_TOUCH[1] - GLASS_INSET,
    )
    frame_y: float = 25.0
    window_x: tuple = (-27.0, 21.5)
    window_y: float = 15.5
    ear_x: float = max(abs(x) for x, _ in cl.DISP_EARS) + EAR_MARGIN_PAST_SCREW_X
    ear_y: float = max(abs(y) for _, y in cl.DISP_EARS) + EAR_MARGIN_PAST_SCREW_Y
    gh_notch_half_y: float = 11.5
    corner: float = 2.0


def _outline_corner_points(p: Params, zm: float) -> list[tuple[float, float, float]]:
    x, y = cl.DISP_TOUCH_X, cl.DISP_TOUCH_Y
    (fx0, fx1), fy = p.frame_x, p.frame_y
    corners = [(x + fx, y + s * fy, zm) for fx in (fx0, fx1) for s in (-1, 1)]
    corners += [(x + s * p.ear_x, y + u * p.ear_y, zm) for s in (-1, 1) for u in (-1, 1)]
    corners += [(x + s * p.ear_x, y + u * fy, zm) for s in (-1, 1) for u in (-1, 1)]
    return corners


def features(p: Params) -> list[dict]:
    """The bracket as CAD features, sketched on Top and extruded up from Z0."""
    x, y, t = cl.DISP_TOUCH_X, cl.DISP_TOUCH_Y, cl.DISP_BRACKET_T
    (fx0, fx1), fy = p.frame_x, p.frame_y
    (wx0, wx1), wy = p.window_x, p.window_y
    ear_y0 = fy - EAR_FRAME_OVERLAP
    ear_h = p.ear_y - ear_y0
    ear_c = (ear_y0 + p.ear_y) / 2
    zm = Z0 + t / 2
    corners = _outline_corner_points(p, zm)
    notch_x0, notch_x1 = x + fx0 - 1.0, x + wx0 + 1.0
    return [
        {
            "op": "sketch",
            "name": "Outline",
            "plane": "Top",
            "profiles": [
                {"rect": (x + (fx0 + fx1) / 2, y, fx1 - fx0, 2 * fy)},
                {"rect": (x, y + ear_c, 2 * p.ear_x, ear_h)},
                {"rect": (x, y - ear_c, 2 * p.ear_x, ear_h)},
            ],
        },
        {
            "op": "extrude",
            "name": "Plate",
            "sketch": "Outline",
            "depth": t,
            "start": Z0,
            "regions": [
                (x, y),
                *[(x, y + s * (fy - 0.5)) for s in (-1, 1)],
                *[(x, y + s * (fy + 2.0)) for s in (-1, 1)],
            ],
        },
        {
            "op": "fillet",
            "name": "Outline corners",
            "edges": corners,
            "size": p.corner,
            "tangent": False,
        },
        {
            "op": "sketch",
            "name": "Cut-outs",
            "plane": "Top",
            "profiles": [
                {"rect": (x + (wx0 + wx1) / 2, y, wx1 - wx0, 2 * wy), "r": p.corner},
                {
                    "rect": (
                        (notch_x0 + notch_x1) / 2,
                        y,
                        notch_x1 - notch_x0,
                        2 * p.gh_notch_half_y,
                    )
                },
                *[
                    {"circle": (x + dx, y + dy, p.m25_clearance_dia)}
                    for dx, dy in cl.DISP_STANDOFFS
                ],
                *[{"circle": (x + dx, y + dy, p.m3_clearance_dia)} for dx, dy in cl.DISP_EARS],
            ],
        },
        {
            "op": "extrude",
            "name": "Window, connector notch and screw holes",
            "sketch": "Cut-outs",
            "depth": t,
            "start": Z0,
            "mode": "remove",
            "regions": [
                (x, y),
                (x + wx0 + 0.5, y),
                (x + fx0 + 2.0, y),
                *[(x + dx, y + dy) for dx, dy in cl.DISP_STANDOFFS + cl.DISP_EARS],
            ],
        },
    ]


def build(p: Params) -> Part:
    return recipe.build(features(p))


def sections(p: Params) -> list[str]:
    return [
        f"x={cl.DISP_TOUCH_X + cl.DISP_EARS[0][0]}",
        f"y={cl.DISP_TOUCH_Y + cl.DISP_STANDOFFS[0][1]}",
    ]


def expect(p: Params) -> dict:
    return {
        "bbox": (p.frame_x[1] - p.frame_x[0], 2 * p.ear_y, cl.DISP_BRACKET_T),
        "holes": {p.m25_clearance_dia: 4, p.m3_clearance_dia: 4},
        "hole_at": {
            p.m25_clearance_dia: [
                {"x": cl.DISP_TOUCH_X + dx, "y": cl.DISP_TOUCH_Y + dy}
                for dx, dy in cl.DISP_STANDOFFS
            ],
            p.m3_clearance_dia: [
                {"x": cl.DISP_TOUCH_X + dx, "y": cl.DISP_TOUCH_Y + dy} for dx, dy in cl.DISP_EARS
            ],
        },
    }
