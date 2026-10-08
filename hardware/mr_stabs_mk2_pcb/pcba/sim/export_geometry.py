"""Export each high-current net's copper for copper_ir.py: every layer's pours, tracks and pads,
the vias and plated holes that join the layers, and the terminal pads where wires land.

    SKILL/scripts/kicad python3 sim/export_geometry.py      -> sim/copper_<NET>.json

Board frame mm (x right, y toward the rear).
"""

import json

import pcbnew

b = pcbnew.LoadBoard("mk2_routed.kicad_pcb")
LAYERS = {"F": pcbnew.F_Cu, "In1": pcbnew.In1_Cu, "In2": pcbnew.In2_Cu, "B": pcbnew.B_Cu}
ERR = pcbnew.FromMM(0.01)


def mm(p):
    return [round(pcbnew.ToMM(p.x) - 100, 4), round(100 - pcbnew.ToMM(p.y), 4)]


def polys_of(shape):
    out = []
    for i in range(shape.OutlineCount()):
        rings = [shape.Outline(i)] + [shape.Hole(i, h) for h in range(shape.HoleCount(i))]
        out.append([[mm(r.CPoint(k)) for k in range(r.PointCount())] for r in rings])
    return out


def shape_polys(item, lid):
    ps = pcbnew.SHAPE_POLY_SET()
    item.TransformShapeToPolygon(ps, lid, 0, ERR, pcbnew.ERROR_INSIDE)
    return polys_of(ps)


def layer_polys(net, lid):
    """Every piece of the net's copper on one layer: zone fills, tracks, pads."""
    polys = []
    for z in b.Zones():
        if z.GetNetname() == net and z.IsOnLayer(lid) and not z.GetIsRuleArea():
            polys += polys_of(z.GetFilledPolysList(lid))
    tracks = [t for t in b.GetTracks() if t.GetClass() != "PCB_VIA"]
    pads = [p for fp in b.GetFootprints() for p in fp.Pads()]
    for item in tracks + pads:
        if item.GetNetname() == net and item.IsOnLayer(lid):
            polys += shape_polys(item, lid)
    return polys


def net_holes(net):
    """Vias and plated holes: [x, y, drill] in mm."""
    holes = [
        mm(t.GetPosition()) + [pcbnew.ToMM(t.GetDrillValue())]
        for t in b.GetTracks()
        if t.GetClass() == "PCB_VIA" and t.GetNetname() == net
    ]
    for fp in b.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == net and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                holes.append(mm(p.GetPosition()) + [pcbnew.ToMM(p.GetDrillSizeX())])
    return holes


def terminal(value, pad_no):
    """The SMD pad a wire lands on, by footprint value and pad number."""
    fp = next(f for f in b.GetFootprints() if f.GetValue() == value)
    pad = next(
        p
        for p in fp.Pads()
        if p.GetNumber() == pad_no and p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD
    )
    lid = pcbnew.B_Cu if pad.IsOnLayer(pcbnew.B_Cu) else pcbnew.F_Cu
    return {"layer": "B" if lid == pcbnew.B_Cu else "F", "poly": shape_polys(pad, lid)}


def export(net, terminals):
    layers = {name: layer_polys(net, lid) for name, lid in LAYERS.items()}
    holes = net_holes(net)
    terms = {label: terminal(value, pad_no) for label, (value, pad_no) in terminals.items()}
    out = {"net": net, "layers": layers, "holes": holes, "terminals": terms}
    json.dump(out, open(f"sim/copper_{net.replace('+', 'P')}.json", "w"))
    counts = {k: len(v) for k, v in layers.items()}
    print(net, counts, "holes", len(holes), "terminals", list(terms))


# Each high-current path, wire pad to wire pad (footprint value, pad number).
export("PACK+", {"in": ("PACK_B+", "1"), "out": ("SW_OUT", "1")})
export("BAT_IN", {"in": ("SW_BACK", "1"), "out": ("0.5m", "1")})
export("VBATT", {"in": ("0.5m", "2"), "esc_l": ("ESC_L+", "1"), "esc_r": ("ESC_R+", "1")})
export("GND", {"pack": ("PACK_A-", "1"), "esc_l": ("ESC_L-", "1"), "esc_r": ("ESC_R-", "1")})
export("PACK_MID", {"in": ("PACK_A+", "1"), "out": ("PACK_B-", "1")})
