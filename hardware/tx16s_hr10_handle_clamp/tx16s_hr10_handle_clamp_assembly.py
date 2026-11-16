"""TX16S HR10 handle clamp assembly on the carry-handle bar."""

import importlib.util
from pathlib import Path

from build123d import Pos

_spec = importlib.util.spec_from_file_location(
    "tx16s_hr10_handle_clamp", Path(__file__).resolve().parent / "tx16s_hr10_handle_clamp.py"
)
assert _spec and _spec.loader
clamp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(clamp)

P = clamp.Params()


def _placed(half: str) -> Pos:
    shift = clamp.radio_to_part(P, half)
    return Pos(-shift.position.X, -shift.position.Y, -shift.position.Z)


def components():
    return {
        "top": {
            "part": "tx16s_hr10_handle_clamp.py",
            "params": {"half": "top"},
            "at": _placed("top"),
            "material": "pla",
            "description": "Clamp top half with receptacle tower, PLA",
        },
        "bottom": {
            "part": "tx16s_hr10_handle_clamp.py",
            "params": {"half": "bottom"},
            "at": _placed("bottom"),
            "material": "pla",
            "description": "Clamp bottom half, PLA, 2 x M3 heat-set inserts",
        },
        "bar": {
            "shape": clamp.bar(),
            "color": "#3a3f47",
            "description": "TX16S MK2 handle bar (scan)",
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
    seat = P.ear_top - P.cbore_depth
    return [
        {
            "name": "m3_clamp",
            "size": "M3",
            "length": 14,
            "head": "shcs",
            "dir": "-Z",
            "at": [(clamp.clamp_x(P), y, seat) for y in (P.screw_y_front, P.screw_y_back)],
            "into": "bottom",
            "thread": "insert",
            "insert_len": 5.7,
        }
    ]


EXPECT = {
    "gap": {("top", "bottom"): P.gap - 0.05},
    "allow_interference": [("receptacle", "plug")],
}


def sections():
    return [f"x={clamp.clamp_x(P)}", f"x={P.tower_x}", f"y={P.tower_y}"]
