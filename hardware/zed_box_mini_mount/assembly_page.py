"""Build the assembly review page for the ZED Box Mini cheese plate adapter.

Run in the build123d-part skill venv from this directory:

    ~/.local/share/build123d-part/venv/bin/python assembly_page.py path/to/zed_box_mini.step

The STEP is Stereolabs' "ZED Box Mini Wifi with fan.step" from stereolabs.com/3dmodels. It is
33 MB, so it stays out of git. Writes out/assembly/: index.html (from assembly_page.html with
the measured data injected) and one mesh JSON per part group for the 3D viewer, and
out/step/: the adapter, cheese plate, and ZED Box Mini as STEP in the assembly frame.
"""

import argparse
import base64
import json
import math
import struct
from pathlib import Path

import innorel_cp10 as cp10
import zed_cheese_plate_adapter as adapter
from build123d import (
    Align,
    Axis,
    Box,
    Compound,
    Cylinder,
    GeomType,
    Location,
    Part,
    Plane,
    Pos,
    RegularPolygon,
    Shape,
    export_step,
    extrude,
    import_step,
)

HERE = Path(__file__).parent
C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)
INCH = 25.4

# STEP frame: X across the box, Y up (base sheet bottom at Y = 3.5), Z along the box (0..143).
# Adapter frame: X along the box, Y across, Z up, base sheet bottom on the adapter top face.
ZED_BASE_Y = 3.5
ZED_MID_Z = 71.5
ZED_MIN_SOLID_MM3 = 200.0
MESH_UNIT = 0.01  # mm per int16 step in the viewer meshes


def zed_location(top: float) -> Location:
    """STEP Z -> adapter X, STEP X -> adapter Y, STEP Y -> adapter Z (a proper rotation)."""
    return Location(
        Plane(origin=(-ZED_MID_Z, 0, top - ZED_BASE_Y), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    )


def write_mesh(shape: Shape, path: Path, tol: float, angular: float) -> None:
    """One mesh as JSON: base64 int16 positions in units of MESH_UNIT mm (Z up) and uint32
    triangle indices. Artifacts serve JSON but not GLB; the viewer computes normals."""
    pos: list[int] = []
    idx: list[int] = []
    for solid in shape.solids():
        verts, tris = solid.tessellate(tol, angular)
        base = len(pos) // 3
        for vtx in verts:
            pos.extend(round(c / MESH_UNIT) for c in (vtx.X, vtx.Y, vtx.Z))
        for t in tris:
            idx.extend(base + k for k in t)
    assert max(abs(c) for c in pos) < 32768, "mesh exceeds the int16 range"
    mesh = {
        "unit": MESH_UNIT,
        "pos": base64.b64encode(struct.pack(f"<{len(pos)}h", *pos)).decode(),
        "idx": base64.b64encode(struct.pack(f"<{len(idx)}I", *idx)).decode(),
    }
    path.write_text(json.dumps(mesh))


def socket_head(
    head_d: float, head_h: float, socket_af: float, shank_d: float, length: float
) -> Part:
    head = Cylinder(head_d / 2, head_h, align=C_MIN)
    socket = Pos(0, 0, head_h - 0.6 * head_h) * extrude(
        RegularPolygon(socket_af / 2, 6, major_radius=False), head_h
    )
    return head - socket + Cylinder(shank_d / 2, length, align=C_MAX)


def hardware(p: adapter.Params) -> dict[str, Part]:
    top = p.thickness
    inserts, m3, bolts = [], [], []
    for sx in (-1, 1):
        for sy in (-1, 1):
            at = Pos(sx * p.zed_hole_x, sy * p.zed_hole_y, top)
            body = Cylinder(p.insert_od / 2, p.insert_len, align=C_MAX) - Cylinder(
                1.25, p.insert_len, align=C_MAX
            )
            inserts.append(at * body)
            m3.append(at * Pos(0, 0, adapter.ZED_SHEET_T) * socket_head(5.5, 3.0, 2.5, 3.0, 8.0))
            seat = Pos(sx * p.plate_hole_x, sy * p.plate_hole_y, top - p.cbore_depth)
            bolts.append(
                seat
                * socket_head(0.375 * INCH, 0.25 * INCH, 0.1875 * INCH, 0.25 * INCH, 0.5 * INCH)
            )
    return {"inserts": Compound(inserts), "m3_screws": Compound(m3), "bolts": Compound(bolts)}


def tup(v) -> tuple[float, float, float]:
    return (v.X, v.Y, v.Z)


def wire_points(wire, u: int, v: int, step: float = 0.25) -> list[list[float]]:
    n = max(24, int(wire.length / step))
    pts = [wire.position_at(i / n) for i in range(n)]
    return [[round(tup(pt)[u], 3), round(tup(pt)[v], 3)] for pt in pts]


def cut(
    shape: Shape, axis: str, value: float, window: tuple[float, float, float, float] | None = None
) -> list[dict]:
    """Faces of `shape` on the plane axis = value, as 2D polygons with holes.

    `window` (u0, u1, v0, v1) limits the cut to a region of the plane, which keeps cuts through
    large compounds fast.
    """
    i = "XYZ".index(axis)
    u, v = [k for k in range(3) if k != i]
    big = 1000.0
    size = [big, big, big]
    center = [0.0, 0.0, 0.0]
    if window:
        size[u], size[v] = window[1] - window[0], window[3] - window[2]
        center[u], center[v] = (window[0] + window[1]) / 2, (window[2] + window[3]) / 2
    thin = 0.002
    size[i] = thin
    center[i] = value
    slab = Pos(*center) * Box(*size)
    polys = []
    for solid in shape.solids():
        bb = solid.bounding_box()
        if not (tup(bb.min)[i] < value < tup(bb.max)[i]):
            continue
        piece = solid & slab
        if piece is None:
            continue
        for f in piece.faces():
            n = tup(f.normal_at())
            c = tup(f.center())
            if abs(abs(n[i]) - 1) > 1e-6 or c[i] < value:
                continue
            polys.append(
                {
                    "outer": wire_points(f.outer_wire(), u, v),
                    "holes": [wire_points(w, u, v) for w in f.inner_wires()],
                }
            )
    return polys


def circle_centers(
    shape: Shape, dia: float, axis: Axis = Axis.Z, tol: float = 0.05
) -> list[list[float]]:
    out: list[list[float]] = []
    for f in shape.faces().filter_by(GeomType.CYLINDER):
        if abs(2 * f.radius - dia) > tol:
            continue
        a = f.axis_of_rotation
        if abs(abs(a.direction.Z) - 1) > 1e-6 and axis == Axis.Z:
            continue
        c = [round(a.position.X, 3), round(a.position.Y, 3)]
        if not any(abs(c[0] - o[0]) < 0.01 and abs(c[1] - o[1]) < 0.01 for o in out):
            out.append(c)
    return sorted(out)


def insert_analysis(p: adapter.Params) -> dict:
    edge_to_center = p.width / 2 - p.zed_hole_y
    rows = []
    for od in (4.9, 5.0, 5.2, 5.4, 5.6):
        for shrink in (0.0, 0.1, 0.2):
            hole = p.insert_hole - shrink
            rows.append(
                {
                    "od": od,
                    "shrink": shrink,
                    "hole": round(hole, 2),
                    "radial": round((od - hole) / 2, 3),
                    "displaced_mm3": round(math.pi / 4 * (od**2 - hole**2) * p.insert_len, 1),
                    "wall": round(edge_to_center - od / 2, 2),
                }
            )
    return {
        "hole": p.insert_hole,
        "len": p.insert_len,
        "od": p.insert_od,
        "thickness": p.thickness,
        "edge_to_center": round(edge_to_center, 2),
        "below_insert": round(p.thickness - p.insert_len, 2),
        "rows": rows,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("zed_step", type=Path)
    ap.add_argument("--out", type=Path, default=HERE / "out" / "assembly")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    p = adapter.Params()
    part = adapter.build(p)
    plate = cp10.build(cp10.Params())
    zed = import_step(args.zed_step).moved(zed_location(p.thickness))
    hw = hardware(p)

    zb = zed.bounding_box()
    print(
        f"ZED placed: x {zb.min.X:.2f}..{zb.max.X:.2f} y {zb.min.Y:.2f}..{zb.max.Y:.2f}",
        f"z {zb.min.Z:.2f}..{zb.max.Z:.2f}",
    )
    # The base sheet: the only solid wider than the adapter's 100 mm.
    sheets = [
        s
        for s in zed.solids()
        if s.bounding_box().size.Y > 100 and s.bounding_box().min.Z < p.thickness + 0.01
    ]
    assert len(sheets) == 1, f"expected one ZED base sheet, got {len(sheets)}"
    sheet = sheets[0]

    # STEP files in the shared assembly frame: they line up when imported together.
    step_dir = args.out.parent / "step"
    step_dir.mkdir(exist_ok=True)
    for name, shape in {
        "zed_cheese_plate_adapter": part,
        "innorel_cp10": plate,
        "zed_box_mini": zed,
    }.items():
        export_step(shape, str(step_dir / f"{name}.step"))
        print(f"{name}.step {(step_dir / f'{name}.step').stat().st_size / 1e6:.2f} MB")

    # Small internal components are hidden by the case and would triple the mesh size.
    zed_mesh = Compound([s for s in zed.solids() if s.volume > ZED_MIN_SOLID_MM3])
    meshes = {"adapter": part, "plate": plate, "zed": zed_mesh, **hw}
    for name, shape in meshes.items():
        tol = 0.1 if name == "zed" else 0.02
        write_mesh(shape, args.out / f"{name}.json", tol, 0.35 if name == "zed" else 0.2)
        print(f"{name}.json {(args.out / f'{name}.json').stat().st_size / 1e6:.2f} MB")

    # Plan-view layers for the hole overlap drawing.
    top = p.thickness
    zone = (-100.0, 100.0, -56.0, 56.0)
    plan = {
        "zed_sheet": cut(sheet, "Z", top + 0.5, zone),
        "adapter_top": cut(part, "Z", top - 0.5, zone),
        "adapter_bottom": cut(part, "Z", 2.0, zone),
        "plate": cut(plate, "Z", -5.0, zone),
    }
    zed_holes = circle_centers(sheet, adapter.ZED_EAR_HOLE)
    insert_holes = circle_centers(part, p.insert_hole)
    bolt_holes = circle_centers(part, p.bolt_clear)
    plate_holes = circle_centers(plate, cp10.TAP_1_4_20)

    def nearest(c: list[float], pool: list[list[float]]) -> list[float]:
        return min(pool, key=lambda q: math.dist(c, q))

    pairs = []
    for c in insert_holes:
        z = nearest(c, zed_holes)
        pairs.append(
            {
                "kind": "m3",
                "adapter": c,
                "mate": z,
                "dx": round(z[0] - c[0], 3),
                "dy": round(z[1] - c[1], 3),
            }
        )
    for c in bolt_holes:
        q = nearest(c, plate_holes)
        pairs.append(
            {
                "kind": "q",
                "adapter": c,
                "mate": q,
                "dx": round(q[0] - c[0], 3),
                "dy": round(q[1] - c[1], 3),
            }
        )

    # Cross-sections through one insert and one bolt, every part in the stack.
    stack = {"adapter": part, "plate": plate, "zed_sheet": sheet, **hw}
    ix, iy = p.zed_hole_x, p.zed_hole_y
    bx, by = p.plate_hole_x, p.plate_hole_y
    sections = {
        "insert": {
            "axis": "X",
            "at": ix,
            "window": [iy - 9, iy + 6, -10.0, top + 6],
            "parts": {
                k: cut(s, "X", ix, (iy - 9, iy + 6, -10.0, top + 6)) for k, s in stack.items()
            },
        },
        "bolt": {
            "axis": "X",
            "at": bx,
            "window": [by - 13, by + 13, -10.0, top + 3],
            "parts": {
                k: cut(s, "X", bx, (by - 13, by + 13, -10.0, top + 3)) for k, s in stack.items()
            },
        },
    }

    report = json.loads((HERE / "out" / "zed_cheese_plate_adapter" / "report.json").read_text())
    checks = [
        {"name": c["check"], "status": c["level"], "detail": c["message"]}
        for c in report["findings"]
    ]

    data = {
        "params": p.__dict__,
        "plan": plan,
        "pairs": pairs,
        "zed_holes": zed_holes,
        "plate_holes": {
            "q": cp10.mirrored(cp10.QUARTER_20),
            "t": cp10.mirrored(cp10.THREE_EIGHTHS_16),
            "a": cp10.mirrored(cp10.LOCATING_HOLES),
        },
        "plate_params": cp10.Params().__dict__,
        "sections": sections,
        "insert": insert_analysis(p),
        "checks": checks,
        "mass_g": round(report["basics"]["mass_g"], 1),
        "zed_bbox": [
            [round(zb.min.X, 2), round(zb.min.Y, 2), round(zb.min.Z, 2)],
            [round(zb.max.X, 2), round(zb.max.Y, 2), round(zb.max.Z, 2)],
        ],
    }
    html = (
        (HERE / "assembly_page.html")
        .read_text()
        .replace("/*DATA*/null", json.dumps(data, separators=(",", ":")))
    )
    (args.out / "index.html").write_text(html)
    print(f"index.html {(args.out / 'index.html').stat().st_size / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
