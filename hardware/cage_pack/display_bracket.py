"""Display bracket: a printed PETG frame behind the touch display that clamps its glass
against the lid's underside.

Four M2.5 x 6 screws go up through the frame into the display's threaded standoffs, and
four M3 x 8 screws go up through its ears into heat-set inserts in the lid's display
bosses. The display comes away with the lid; unplug its GH lead first.

Spec
- Process: FDM, black PETG, printed flat (the build frame is the print frame).
  PROCESS = "fdm".
- Frame: cage_layout.py. Top face on the display's standoff ends (DISP_BACK_Z), the
  bosses' end faces in the same plane.
- Holes: 4 x Ø2.9 (M2.5 normal clearance, fasteners.md) on the display's 60 x 41
  standoff pattern; 4 x Ø3.4 (M3 normal clearance) at the lid bosses (DISP_EARS).
- Size: frame 71 x 50 round the standoffs, ears out to ±36 in Y, 2.6 thick. A window
  through the middle, and a notch at the -X end round the display's GH connector, whose
  back face sits in the frame's top plane; its lead leaves through the notch.
- Critical dimensions: the standoff pattern (from Waveshare's STEP, inspect_step.py),
  the ear pattern (shared with enclosure_lid.py through cage_layout).
- Model: features() (recipe.py), so the checked solid and the Onshape Part Studio are the
  same five features: outline sketch, plate, corner fillets, cut-out sketch, cut.
"""

from dataclasses import dataclass

import b123d_fix  # noqa: F401
import cage_layout as cl
import recipe  # build123d-part/scripts, on the path when run through scripts/run
from build123d import Part

PROCESS = "fdm"
MATERIAL = "petg"
Z0 = cl.DISP_BACK_Z - cl.DISP_BRACKET_T


@dataclass
class Params:
    m25_clear: float = 2.9  # M2.5 normal clearance, fasteners.md
    m3_clear: float = 3.4  # M3 normal clearance, fasteners.md
    frame_x: tuple = (-38.5, 32.5)  # inside the glass's X ends (-39.53, 33.53)
    frame_y: float = 25.0  # half width; the glass is ±25.27
    window_x: tuple = (-27.0, 21.5)  # leaves 4 mm of frame round each standoff hole
    window_y: float = 15.5
    ear_x: float = 19.0  # ear half length: 6 past the screw
    ear_y: float = 36.0  # ear's outer edge: 4.5 past the screw
    notch_y: float = 11.5  # GH connector body is -10.3..9.2 in Y, +1 each side
    corner: float = 2.0


def features(p: Params) -> list[dict]:
    """The bracket as CAD features, sketched on Top and extruded up from Z0."""
    x, y, t = cl.DISP_X, cl.DISP_Y, cl.DISP_BRACKET_T
    (fx0, fx1), fy = p.frame_x, p.frame_y
    (wx0, wx1), wy = p.window_x, p.window_y
    ear_y0 = fy - 1.0  # the ears start 1 inside the frame, so the outline is one region
    ear_h = p.ear_y - ear_y0
    ear_c = (ear_y0 + p.ear_y) / 2
    zm = Z0 + t / 2
    # Vertical edges of the outline: the frame's corners, the ears' outer corners and the
    # inside corners where the ears meet the frame.
    corners = [(x + fx, y + s * fy, zm) for fx in (fx0, fx1) for s in (-1, 1)]
    corners += [(x + s * p.ear_x, y + u * p.ear_y, zm) for s in (-1, 1) for u in (-1, 1)]
    corners += [(x + s * p.ear_x, y + u * fy, zm) for s in (-1, 1) for u in (-1, 1)]
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
                {"rect": ((notch_x0 + notch_x1) / 2, y, notch_x1 - notch_x0, 2 * p.notch_y)},
                *[{"circle": (x + dx, y + dy, p.m25_clear)} for dx, dy in cl.DISP_STANDOFFS],
                *[{"circle": (x + dx, y + dy, p.m3_clear)} for dx, dy in cl.DISP_EARS],
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
    return [f"x={cl.DISP_X + cl.DISP_EARS[0][0]}", f"y={cl.DISP_Y + cl.DISP_STANDOFFS[0][1]}"]


def expect(p: Params) -> dict:
    return {
        "bbox": (p.frame_x[1] - p.frame_x[0], 2 * p.ear_y, cl.DISP_BRACKET_T),
        "holes": {p.m25_clear: 4, p.m3_clear: 4},
        "hole_at": {
            p.m25_clear: [
                {"x": cl.DISP_X + dx, "y": cl.DISP_Y + dy} for dx, dy in cl.DISP_STANDOFFS
            ],
            p.m3_clear: [{"x": cl.DISP_X + dx, "y": cl.DISP_Y + dy} for dx, dy in cl.DISP_EARS],
        },
    }
