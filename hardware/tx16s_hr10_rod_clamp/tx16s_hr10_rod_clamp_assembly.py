"""TX16S HR10 rod clamp on the bare handle rod, radio frame.

Both halves of tx16s_hr10_rod_clamp.py placed back on the rod's right corner, the
receptacle and a mated plug, and three M3 x 8 screws into hex nuts in the bottom half.

Spec
- Halves: each part file's print frame mapped back by radio_to_part().inverse().
- Rod: the nominal path from the part file (run halfway along its slot, 5 mm bend).
- Fasteners: M3 x 8 SHCS along -n, head on the counterbore floor (t - cbore_depth), into
  M3 nuts sunk 1.0 into the bottom half's underside (fasteners.md).
"""

import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location(
    "tx16s_hr10_rod_clamp", Path(__file__).resolve().parent / "tx16s_hr10_rod_clamp.py"
)
assert _spec and _spec.loader
clamp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(clamp)

P = clamp.Params()


def components():
    return {
        "top": {
            "part": "tx16s_hr10_rod_clamp.py",
            "params": {"half": "top"},
            "at": clamp.radio_to_part(P, "top").inverse(),
            "material": "pla",
            "description": "Rod clamp top half with receptacle tower, PLA",
        },
        "bottom": {
            "part": "tx16s_hr10_rod_clamp.py",
            "params": {"half": "bottom"},
            "at": clamp.radio_to_part(P, "bottom").inverse(),
            "material": "pla",
            "description": "Rod clamp bottom half, PLA, 3 x M3 nut pockets",
        },
        "rod": {
            "shape": clamp.rod_to_radio() * clamp.rod(P),
            "color": "#3a3f47",
            "description": "TX16S MK2 handle rod, Ø5, sheath removed",
        },
        "nuts": {
            "shape": clamp.rod_to_radio() * clamp.nuts_rod(P),
            "qty": 3,
            "description": "M3 hex nut, 5.5 AF x 2.4",
        },
        "receptacle": {
            "shape": clamp.receptacle_radio(P),
            "part_number": "HR10A-7R-4S(73)",
            "vendor": "Hirose",
            "description": "4-pin push-pull receptacle, solder cups",
        },
        "plug": {"shape": clamp.plug_radio(P), "description": "HR10A-7P-4S mated (envelope)"},
    }


def fasteners():
    o, u, v, n = clamp.rod_axes()
    seat = P.t - P.cbore_depth
    return [
        {
            "name": "m3_clamp",
            "size": "M3",
            "length": 8,
            "head": "shcs",
            "dir": tuple(-n),
            "at": [tuple(o + su * u + sv * v + seat * n) for su, sv in P.screws],
            "into": "nuts",
            "thread": "nut",
        }
    ]


EXPECT = {
    "gap": {("top", "bottom"): P.gap - 0.05},
    # Solid envelopes of a mated pair: the plug's coupling sleeve goes over the receptacle.
    "allow_interference": [("receptacle", "plug")],
}


def sections():
    o, u, v, n = clamp.rod_axes()
    s = o + 17.0 * u + 8.5 * v
    return [f"x={s[0]:.2f}", f"x={clamp.tower_xy()[0]:.2f}", f"y={clamp.tower_xy()[1]:.2f}"]
