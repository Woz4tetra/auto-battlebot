"""Cage pack assembly: enclosure, electronics, camera and cables."""

import importlib.util
import math
import sys
from functools import cache
from typing import NamedTuple

import bom_data as bd
import cage_layout as cl
import camera_shield
import connectors as cn
import cup_arm
import display_bracket
import enclosure
import enclosure_lid
import hr10
import onshape_catalog
import pcba_cage
import smallrig_4122c as cup
from build123d import (
    Align,
    Axis,
    Box,
    Circle,
    Compound,
    Cylinder,
    FilletPolyline,
    Location,
    Plane,
    Polyline,
    Pos,
    Rot,
    Transition,
    Vector,
    fillet,
    sweep,
)

C_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
C_MAX = (Align.CENTER, Align.CENTER, Align.MAX)
CC = (Align.CENTER, Align.CENTER, Align.CENTER)
INCH = 25.4
CAM_HEAD_SCREW_1_4_20_X1_SHCS_PN = "91251A542"
RPSMA_THREAD_R = 3.15
BATTERY_LEAD_RISE = 16.0
GX12_CUPS_INNER_X = 12.75
WIFI_PLUG_ENVELOPE = (9.0, 12.0, 9.0)
GH_HOUSING_PROTRUSION = 6.5
DISP_PLUG_T = 3.0


def _sibling(name: str):
    if str(cl.ZED_MOUNT) not in sys.path:
        sys.path.append(str(cl.ZED_MOUNT))
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, cl.ZED_MOUNT / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


ballhead = _sibling("smallrig_2948_ball_head")
bracket = _sibling("zed_x_one_s_bracket")
BRP = bracket.Params()
HEAD = Location(
    Plane(origin=(cl.HEAD_SEAT_X, cl.CAM_Y, cl.CAM_AXIS_Z), x_dir=(0, 0, -1), z_dir=(1, 0, 0))
)
BALL = ballhead.Params(pan=0, tilt=0, tilt_dir=0, platform_spin_deg=cl.CAM_SPIN)
BRACKET_AT = HEAD * ballhead.top_location(BALL)
CAMERA_AT = BRACKET_AT * bracket.camera_location(BRP)
UPS_LOC = Location(Plane(origin=cl.ups(0.0, 0.0, 0.0), x_dir=(-1, 0, 0), z_dir=(0, 1, 0)))

UPS_TOP = cl.UPS_BOARD_Z0 + 1.6
UPS_VOUT = cl.ups(61.89, 0.0, 88.49)[:2]
UPS_VIN = cl.ups(-27.50, 0.0, 97.43)[:2]
UPS_BAT = cl.ups(-27.71, 0.0, 118.79)[:2]
UPS_USB_MINI_B_MOUTH = cl.ups(-31.44, 0.0, 87.68)[:2]


TITLE = "CAGE PACK"


def _world(loc: Location, v) -> tuple[float, float, float]:
    p = (loc * Pos(*v)).position
    return (p.X, p.Y, p.Z)


def _world_dir(loc: Location, v) -> tuple[float, float, float]:
    d = (loc * Pos(*v)).position - loc.position
    return (d.X, d.Y, d.Z)


FAKRA_PLUG_LEN = 26.0
FAKRA_ENGAGE = 9.0
FAKRA_CABLE_IN = 6.0
CAM_FAKRA_TIP = (3.30, -42.72, -23.75)


def fakra_cam():
    """Camera FAKRA plug seat point and cable direction."""
    tip = Vector(*_world(CAMERA_AT, CAM_FAKRA_TIP))
    d = Vector(*_world_dir(CAMERA_AT, (0, 1, 0)))
    seat = tip - d * FAKRA_ENGAGE
    return (seat.X, seat.Y, seat.Z), (d.X, d.Y, d.Z)


HOLDER_AT = Location(
    Plane(
        (cl.BRICK_XC, cl.BRICK_YC, cl.HOLDER_Z0 - cl.HOLDER_PIN),
        x_dir=(0, 1, 0),
        z_dir=(1, 0, 0),
    )
)


@cache
def _holder_solids():
    from build123d import import_step

    return [HOLDER_AT * s for s in import_step(str(cl.HOLDER_STEP)).solids()]


def _holder():
    """BK-18650-PC8 body and its eight contacts, PC pins clipped 1.0 below the carrier."""
    keep = Pos(0, 0, cl.CARRIER_Z0 - 1.0) * Box(600, 600, 200, align=C_MIN)
    return Compound([s & keep for s in _holder_solids()])


def _pins() -> list[tuple[float, float]]:
    """PC pin centres (x, y), from the contact solids below the holder base."""
    band = Pos(0, 0, cl.HOLDER_Z0 - 2.0) * Box(600, 600, 1.0, align=C_MIN)
    out = []
    for s in sorted(_holder_solids(), key=lambda s: s.volume)[:8]:
        c = (s & band).bounding_box().center()
        out.append((round(c.X, 2), round(c.Y, 2)))
    return out


def _cells():
    """Four Molicel P30B, flat top, in the holder's cradles, + ends toward -Y."""
    z = cl.HOLDER_Z0 + cl.CELL_AXIS_DZ
    can = Cylinder(cl.CELL_D / 2, cl.CELL_L, rotation=(90, 0, 0))
    can = fillet(can.edges(), 0.8)
    cap = Pos(0, -cl.CELL_L / 2 - 0.09, 0) * Cylinder(5.5, 0.2, rotation=(90, 0, 0))
    cell = can + cap
    return Compound([Pos(cl.BRICK_XC + x, cl.BRICK_YC, z) * cell for x in cl.CELL_XS])


def _carrier():
    """Bare 1.6 mm PCB: series links, B-, B1-B3, B+ pads on the +X tab, holes for the pins."""
    cx, cy = cl.CARRIER
    (tx0, ty0), (tx1, ty1) = cl.CARRIER_TAB
    board = Pos(cl.BRICK_XC, cl.BRICK_YC, cl.CARRIER_Z0) * Box(cx, cy, cl.CARRIER_T, align=C_MIN)
    lo = (Align.MIN, Align.MIN, Align.MIN)
    board += Pos(tx0 - 1.0, ty0, cl.CARRIER_Z0) * Box(
        tx1 - tx0 + 1.0, ty1 - ty0, cl.CARRIER_T, align=lo
    )
    for x, y in _pins():
        board -= Pos(x, y, cl.CARRIER_Z0 - 1) * Cylinder(0.8, cl.CARRIER_T + 2, align=C_MIN)
    return board


def _tray_inserts():
    """Heat-set inserts in the tray bosses and the cup arm screws."""
    tops = [(xy, cl.PANEL_Z1 + cl.BOSS_H) for xy in cl.ZED_EAR_HOLES + pcba_cage.holes()]
    tops += [(xy, cl.CLAMP_Z0) for xy in cl.UPS_HOLES]
    tops += [(xy, cl.LID_UNDERSIDE_Z) for xy in enclosure.lid_screws()]
    ins = Cylinder(2.3, cl.INSERT_LEN, align=C_MAX) - Cylinder(1.25, cl.INSERT_LEN, align=C_MAX)
    up = Cylinder(2.3, cl.INSERT_LEN, align=C_MIN) - Cylinder(1.25, cl.INSERT_LEN, align=C_MIN)
    out = [Pos(x, y, z) * ins for (x, y), z in tops]
    out += [Pos(x, y, cl.PANEL_Z0) * up for x, y in cl.arm_screws()]
    return Compound(out), len(out)


def _lid_inserts():
    """M3 x 6.4 heat-set inserts in the lid's display bosses, flush with each boss's end."""
    ins = Cylinder(2.3, cl.INSERT_LEN, align=C_MIN) - Cylinder(1.25, cl.INSERT_LEN, align=C_MIN)
    pts = [(cl.DISP_TOUCH_X + dx, cl.DISP_TOUCH_Y + dy) for dx, dy in cl.DISP_EARS]
    return Compound([Pos(x, y, cl.DISP_BACK_Z) * ins for x, y in pts])


def _cup_washers():
    w, t = cl.CUP_WASHER
    return Compound(
        [
            Pos(x, cl.CUP_Y, cl.ARM_SEAT_Z)
            * (Cylinder(w / 2, t, align=C_MIN) - Cylinder(5.2, t, align=C_MIN))
            for x in cl.CUP_X_NEG_POS
        ]
    )


def _head_screw_seat() -> tuple[float, float, float]:
    """Underside of the 1/4-20 screw's head: the camera pad's inside face."""
    return (cl.X_IN[1] - cl.CAM_PAD[1], cl.CAM_Y, cl.CAM_AXIS_Z)


def _shield_insert():
    """M3 x 6.4 heat-set insert pressed into the +X wall and pad from outside."""
    ins = Cylinder(2.3, cl.INSERT_LEN, align=C_MAX) - Cylinder(1.25, cl.INSERT_LEN, align=C_MAX)
    return Pos(cl.X_OUT[1], cl.CAM_Y, cl.SHIELD_SCREW_Z) * Rot(0, 90, 0) * ins


def _sma_bulkhead(y: float, z: float):
    """RP-SMA female bulkhead through the +X wall."""
    x = cl.X_OUT[1]
    inner = Pos(x - cl.WALL_T, y, z) * Rot(0, -90, 0) * Cylinder(4.0, 9.0, align=C_MIN)
    outer = Pos(x, y, z) * Rot(0, 90, 0) * Cylinder(4.0, 10.0, align=C_MIN)
    thread = Pos(x, y, z) * Rot(0, -90, 0) * Cylinder(RPSMA_THREAD_R, cl.WALL_T, align=C_MIN)
    return inner + thread + outer


@cache
def _vendor_solids(rel: str) -> tuple:
    from build123d import import_step

    return tuple(import_step(str(cl.VENDOR / rel)).solids())


def _vendor(rel: str, origin, x_dir, z_dir, keep=None) -> Compound:
    """Vendor STEP re-framed so origin, x_dir, z_dir (STEP coords) become the local frame."""
    inv = Location(Plane(origin, x_dir=x_dir, z_dir=z_dir)).inverse()
    return Compound([inv * s for s in _vendor_solids(rel) if keep is None or keep(s)])


def _gx12_jack() -> Compound:
    return _vendor("grabcad/gx12_2_jack.step", (1.0, 0, 0), (0, 1, 0), (-1, 0, 0))


def _gx12_plug() -> Compound:
    """GX12-2 cable plug mated: coupling nut's front face on the jack's flange (2.5 out)."""
    return _vendor("grabcad/gx12_2_plug.step", (2.5, 0, 0), (0, 1, 0), (-1, 0, 0))


GX12_PLUG_WALL_TO_CLAMP = 33.0


def _hr10_plug() -> Compound:
    """Hirose HR10A-7P plug, GrabCAD "Hirose HR10 6 pin plug end" (same size-7 shell as the
    4-pin HR10A-7P-4P), body's front face on the receptacle's 7.8 mm front, plus 40 mm of
    cable. hr10.py frame: +Z into the enclosure. Only the body: its nose and pins sit inside
    the receptacle, and the nose's boolean against the receptacle envelope segfaults OCC."""
    body = _vendor(
        "grabcad/hr10_plug_6p.step",
        (0, 0, 4.28),
        (1, 0, 0),
        (0, 0, 1),
        keep=lambda s: s.bounding_box().max.Z < 0.0,
    )
    cable = Pos(0, 0, -25.8) * Cylinder(2.5, 40.0, align=C_MAX)
    return Compound([*body.solids(), cable])


RJ45_FLANGE_BACK = -11.55
RJ45_FLANGE_TO_STUB_END = 44.5 + RJ45_FLANGE_BACK


def _rj45_coupler():
    x, z = cl.RJ45_AT
    o = (x, cl.Y_OUT[0] + RJ45_FLANGE_BACK, z)
    at = Location(Plane(o, x_dir=(0, -1, 0), z_dir=(0, 0, -1)))
    return Compound([at * s for s in _vendor_solids("grabcad/rj45_moddiy_cab105.step")])


def _minifit(rel: str) -> Compound:
    return _vendor(rel, (0, -5.0, -13.8), (1, 0, 0), (0, 0, 1))


def _tbp_plug() -> Compound:
    return _vendor(
        "samesky/Same_Sky_TBP01P1W-508-02GR.step", (9.5, 7.5, 2.54), (0, 0, 1), (-1, 0, 0)
    )


def _fakra_plug() -> Compound:
    return _vendor(
        "fakra/Rosenberger_59Z114-000Z_housing_plug.step", (0, 0, -0.2), (1, 0, 0), (0, 0, -1)
    )


def _antenna(x: float, y: float, z: float) -> Compound:
    solids = _vendor_solids("rf/Lcom_HG2402RD-RSF_3D.step")
    tip, hinge = 61.89, 36.25
    at = Pos(x + 4.0 + tip, y, z) * Rot(0, 0, 180)
    fold = Pos(x + 4.0 + tip - hinge, y, z) * Rot(0, 0, 90) * Pos(-(x + 4.0 + tip - hinge), -y, -z)
    out = []
    for sol in solids:
        placed = at * sol
        whip = sol.bounding_box().min.X < 0
        out.append(fold * placed if whip else placed)
    return Compound(out)


def _is_rj45_head(solid) -> bool:
    return solid.bounding_box().min.Z > -0.1


def _rj45_plug():
    rx, ry, rz = cl.ZED_RJ45_MOUTH
    at = Location(Plane((rx + 15.0 - 22.17, ry, rz), x_dir=(0, 1, 0), z_dir=(1, 0, 0)))
    head = [at * s for s in _vendor_solids("grabcad/rj45_plug.step") if _is_rj45_head(s)]
    boot = Pos(rx - 7.17, ry, rz) * Box(
        14.8, 11.7, 13.5, align=(Align.MAX, Align.CENTER, Align.CENTER)
    )
    boot = fillet(boot.edges().filter_by(Axis.X), 2.0)
    return Compound([*head, boot])


SWITCH_AT = (cl.SWITCH_X, cl.Y_OUT[0], cl.SWITCH_Z)
BUTTON_TABS_Y = cl.Y_OUT[0] + cl.SWITCH_DEPTH
UPS_J10 = cl.ups(6.2, 0.0, 77.5)[:2]
UPS_J8 = cl.ups(32.5, 0.0, 77.5)[:2]
DUPONT_TOP = UPS_TOP + 16.5
CHARGER = (167.0, 67.0, 35.0)
CHARGER_CORD_END_AT = (cl.X_OUT[0] - 230.0, cl.SERVICE_PORTS_YZ["gx12"][0], 5.0)


def _charger():
    x1, y, z0 = CHARGER_CORD_END_AT
    keep = Pos(233.3, 0, 0) * Box(400, 400, 400, align=(Align.MAX, Align.CENTER, Align.CENTER))
    at = Pos(x1 - 233.11, y - 4.5, z0 - 9.14)
    out = []
    for sol in _vendor_solids("meanwell/GS-120.stp"):
        cut = sol & keep
        out += [at * c for c in cut.solids()] if cut else []
    return Compound(out)


def _plugs() -> dict:
    out = {}
    out["tbp_plug"] = cn.place(
        Rot(0, 0, 90) * _tbp_plug(), cl.ZED_POWER_MOUTH, (-1, 0, 0), (0, 0, 1)
    )
    out["usb_a_plug"] = cn.place(cn.usb_a_plug_90(), cl.ZED_USB_A, (1, 0, 0), (0, -1, 0))
    fx, fy, fz = cl.ZED_FAKRA_A_TIP
    out["fakra_plug_zed"] = cn.place(
        _fakra_plug(), (fx - FAKRA_ENGAGE, fy, fz), (1, 0, 0), (0, 1, 0)
    )
    p, d = fakra_cam()
    out["fakra_plug_cam"] = cn.place(_fakra_plug(), p, d, (1, 0, 0))
    out["rj45_plug"] = _rj45_plug()
    out["rj45_coupler"] = _rj45_coupler()
    out_dir = (-1.0, 0.0, 0.0)
    ox = cl.X_OUT[0]
    gy, gz = cl.SERVICE_PORTS_YZ["gx12"]
    out["gx12_dc_in"] = cn.place(_gx12_jack(), (ox, gy, gz), out_dir, (0, 1, 0))
    out["gx12_charger_plug"] = cn.place(_gx12_plug(), (ox, gy, gz), out_dir, (0, 1, 0))
    out["power_button"] = cn.place(cn.cit_ah_button(cl.WALL_T), SWITCH_AT, (0, -1, 0), (1, 0, 0))
    hy, hz = cl.SERVICE_PORTS_YZ["hr10"]
    out["hr10_rcpt"] = cn.place(
        hr10.receptacle(cl.HR10_PANEL_T), (ox, hy, hz), (1, 0, 0), (0, 1, 0)
    )
    out["hr10_tx_plug"] = cn.place(_hr10_plug(), (ox, hy, hz), (1, 0, 0), (0, 1, 0))
    out["sma_bulkheads"] = Compound([_sma_bulkhead(y, z) for y, z in cl.ANTENNAS_YZ.values()])
    for name, (y, z) in cl.ANTENNAS_YZ.items():
        out[f"antenna_{name}"] = _antenna(cl.X_OUT[1], y, z)
    out["wifi_plugs"] = Compound(
        [Pos(x, y, z) * Box(*WIFI_PLUG_ENVELOPE, align=(Align.CENTER, Align.MIN, Align.CENTER))
         for x, y, z in cl.ZED_WIFI_SMA_JACKS]
    )  # fmt: skip
    up = (0.0, 0.0, 1.0)
    m22, m12 = _minifit("molex/39012040.stp"), _minifit("molex/39012020.stp")
    out["minifit_vout"] = cn.place(m22, (*UPS_VOUT, UPS_TOP), up, (0, 1, 0))
    out["minifit_vin"] = cn.place(m22, (*UPS_VIN, UPS_TOP), up, (0, 1, 0))
    out["minifit_bat"] = cn.place(m12, (*UPS_BAT, UPS_TOP), up, (0, 1, 0))
    for name, (x, y) in (("dupont_J10", UPS_J10), ("dupont_J8", UPS_J8)):
        out[name] = Pos(x, y, UPS_TOP) * cn.dupont_2p()
    out["mini_b_plug"] = cn.place(
        cn.mini_b_plug_90(),
        (UPS_USB_MINI_B_MOUTH[0], UPS_USB_MINI_B_MOUTH[1], UPS_TOP + 2.0),
        (1, 0, 0),
        (0, 0, 1),
    )
    top = pcba_cage.TOP
    jx, jy = pcba_cage.PLACEMENT["J1_usb_c"][2:]
    out["usb_c_plug"] = cn.place(
        cn.usb_c_plug_90(), (jx + 3.6, jy, top + 1.6), (1, 0, 0), (0, 1, 0)
    )
    for ref, n in (
        ("J2_gh4_ups_usb", 4),
        ("J3_gh4_tx_uart", 4),
        ("J4_gh2_ntc", 2),
        ("J5_gh4_button", 4),
        ("J6_gh6_ups_ctrl", 6),
        ("J7_gh13_display", 13),
    ):
        x, y = pcba_cage.PLACEMENT[ref][2:]
        out[f"gh_{ref[:2]}"] = Pos(x, y, top + 4.2) * cn.gh_plug(n)
    out["gh_display"] = _gh_display_plug()
    return out


DISP_PLUG_Z = (cl.DISP_BACK_Z, cl.DISP_BACK_Z + DISP_PLUG_T)
DISP_PLUG_END_X = cl.DISP_TOUCH_X + cl.DISP_GH_MOUTH_X - GH_HOUSING_PROTRUSION


def _gh_display_plug():
    """GHR-13V-S housing in the display GH socket."""
    x1 = cl.DISP_TOUCH_X + cl.DISP_GH_MOUTH_X
    z0, z1 = DISP_PLUG_Z
    return Pos((DISP_PLUG_END_X + x1) / 2, cl.DISP_TOUCH_Y + cl.DISP_GH[1], z0) * Box(
        x1 - DISP_PLUG_END_X, 13 * 1.25 + 1.75, z1 - z0, align=C_MIN
    )


def ufl_point() -> tuple[float, float, float]:
    x, y = pcba_cage.PLACEMENT["U1_esp32s3_wroom1u"][2:]
    return (x + 6.0, y + 7.5, pcba_cage.TOP + 4.5)


class CableSpec(NamedTuple):
    od: float
    min_bend_r: float
    source: str


CABLES = {
    "power": CableSpec(5.2, 10.4, "2 x 16 AWG silicone, Ø2.6 each, side by side: 4 x the wire OD"),
    "battery": CableSpec(5.2, 10.4, "XT30 leads, 2 x 16 AWG silicone: 4 x the wire OD"),
    "usb": CableSpec(4.0, 16.0, "USB 2.0 round cable Ø4.0: 4 x OD"),
    "gmsl": CableSpec(3.35, 17.0, "GMSL2 coax Ø3.35: 5 x OD, coax static rule"),
    "lan": CableSpec(3.8, 16.0, "Slim Cat6 Ø3.8 (28 AWG): TIA-568 4 x OD for UTP"),
    "esp": CableSpec(1.13, 6.0, "u.FL pigtail, 1.13 mm coax: 5 x OD"),
    "wifi": CableSpec(2.6, 13.0, "RG316 Ø2.6: 5 x OD"),
    "signal": CableSpec(2.4, 5.0, "4 to 6 x 28 AWG GH leads, twisted: about 2 x the bundle"),
    "display": CableSpec(4.0, 8.0, "13 x 28 AWG GH lead, bundled Ø4: 2 x the bundle"),
    "dc": CableSpec(5.5, 22.0, "Mean Well DC cord, 18 AWG pair, Ø5.5: 4 x OD"),
}


def _fillet_setback(r: float, turn: float) -> float:
    return r * math.tan(turn / 2)


def _run(kind: str, points: list[tuple[float, float, float]]):
    """A cable centreline through `points`; raises when a corner is tighter than the bend radius."""
    d, r, _ = CABLES[kind]
    pts = [Vector(*q) for q in points]
    segs = [(b - a).length for a, b in zip(pts, pts[1:], strict=False)]
    take = [0.0]
    for a, b, c in zip(pts, pts[1:], pts[2:], strict=False):
        turn = math.acos(max(-1.0, min(1.0, (b - a).normalized().dot((c - b).normalized()))))
        take.append(_fillet_setback(r, turn))
    take.append(0.0)
    for i, seg in enumerate(segs):
        # + 0.5: OCC cannot build a path whose straight run between fillets is zero long.
        need = take[i] + take[i + 1] + 0.5
        if seg < need:
            raise ValueError(
                f"{kind} cable: segment {i} is {seg:.1f} long, R{r} corners need {need:.1f}"
            )
    path = FilletPolyline(*pts, radius=r) if len(pts) > 2 else Polyline(*pts)
    edges = path.edges()
    plane = Plane(origin=edges[0].position_at(0), z_dir=edges[0].tangent_at(0))
    try:
        tube = sweep(plane * Circle(d / 2), path=path, transition=Transition.ROUND)
    except Exception:  # OCC's pipe shell fails ROUND on some smooth 3D paths
        tube = sweep(plane * Circle(d / 2), path=path, transition=Transition.TRANSFORMED)
    return tube, path.length


MINIFIT_PLUG_TOP = 16.6
TBP_PLUG_BACK = 10.0
GH_PLUG_TOP = 4.2 + 6.5


def _route_zed_power(plug_top: float) -> dict:
    vx, vy = UPS_VOUT
    zx, zy, zz = cl.ZED_POWER_MOUTH
    tbp_back = zx - TBP_PLUG_BACK
    return {
        "c4_zed_power": _run(
            "power",
            [
                (vx, vy, plug_top),
                (vx, vy, plug_top + 11.0),
                (tbp_back - 12.0, zy, plug_top + 11.0),
                (tbp_back - 12.0, zy, zz),
                (tbp_back, zy, zz),
            ],
        )  # fmt: skip
    }


def _route_battery_lead(plug_top: float) -> dict:
    bx1 = cl.CARRIER_TAB[1][0] - 7.0
    bz = cl.HOLDER_Z0 + 2.6
    by = UPS_BAT[1]
    over = cl.CARRIER_Z0 + BATTERY_LEAD_RISE
    return {
        "battery_lead": _run(
            "battery",
            [
                (bx1, by, bz),
                (bx1, by, over),
                (10.0, by, over),
                (10.0, by, plug_top + 11.0),
                (UPS_BAT[0], by, plug_top + 11.0),
                (UPS_BAT[0], by, plug_top),
            ],
        )  # fmt: skip
    }


def _route_button_lead(gh_top: float) -> dict:
    j5x, j5y = pcba_cage.PLACEMENT["J5_gh4_button"][2:]
    return {
        "button_lead": _run(
            "signal",
            [
                (cl.SWITCH_X, BUTTON_TABS_Y, cl.SWITCH_Z),
                (cl.SWITCH_X, j5y - 12.0, cl.SWITCH_Z),
                (j5x, j5y - 12.0, cl.SWITCH_Z),
                (j5x, j5y, cl.SWITCH_Z),
                (j5x, j5y, gh_top),
            ],
        )  # fmt: skip
    }


def _route_ups_ctrl_lead(gh_top: float) -> dict:
    j6x, j6y = pcba_cage.PLACEMENT["J6_gh6_ups_ctrl"][2:]
    cz = 110.0
    lo, side = 90.0, -2.0
    return {
        "ups_ctrl_lead": _run(
            "signal",
            [
                (j6x, j6y, gh_top),
                (j6x, j6y, lo),
                (side, j6y, lo),
                (side, j6y, cz),
                (side, UPS_J8[1], cz),
                (UPS_J8[0], UPS_J8[1], cz),
                (UPS_J8[0], UPS_J8[1], DUPONT_TOP),
            ],
        ),  # fmt: skip
        "ups_ctrl_j10": _run(
            "signal",
            [
                (UPS_J10[0], UPS_J10[1], cz),
                (UPS_J10[0], UPS_J10[1], DUPONT_TOP),
            ],
        ),  # fmt: skip
    }


def _route_charge_lead(plug_top: float) -> dict:
    gy, gz = cl.SERVICE_PORTS_YZ["gx12"]
    gx = cl.X_OUT[0] + GX12_CUPS_INNER_X
    hx = -80.0
    return {
        "charge_lead": _run(
            "power",
            [
                (gx, gy, gz),
                (hx, gy, gz),
                (hx, gy, plug_top + 11.0),
                (hx, UPS_VIN[1], plug_top + 11.0),
                (UPS_VIN[0], UPS_VIN[1], plug_top + 11.0),
                (UPS_VIN[0], UPS_VIN[1], plug_top),
            ],
        )  # fmt: skip
    }


def _route_charger_cord() -> dict:
    gy, gz = cl.SERVICE_PORTS_YZ["gx12"]
    cx1, cy, cz0 = CHARGER_CORD_END_AT
    tail = cl.X_OUT[0] - GX12_PLUG_WALL_TO_CLAMP
    drop = cl.CUP_X_NEG_POS[0] - cl.CUP_REACH - 25.0
    return {
        "charger_cord": _run(
            "dc",
            [
                (tail, gy, gz),
                (drop, gy, gz),
                (drop, gy, cz0 + CHARGER[2] / 2),
                (cx1, gy, cz0 + CHARGER[2] / 2),
            ],
        )  # fmt: skip
    }


def _route_usb() -> dict:
    ux, uy, uz = cl.ZED_USB_A
    jx, jy = pcba_cage.PLACEMENT["J1_usb_c"][2:]
    ex, ey, ez = jx + 5.2, jy + 6.0, pcba_cage.TOP + 1.6
    return {
        "c6_usb": _run(
            "usb",
            [
                (ux + 16.0, uy - 8.0, uz),
                (cl.X_IN[1] - 4.0, ey + 17.0, ez + 33.6),
                (ex, ey + 17.0, ez + 33.6),
                (ex, ey + 17.0, ez),
                (ex, ey, ez),
            ],
        )  # fmt: skip
    }


def _route_gmsl() -> dict:
    fx, fy, fz = cl.ZED_FAKRA_A_TIP
    p, d = fakra_cam()
    pv, dv = Vector(*p), Vector(*d)
    gmsl_start_x = fx - FAKRA_ENGAGE + FAKRA_PLUG_LEN - FAKRA_CABLE_IN
    gmsl_end = pv + dv * (FAKRA_PLUG_LEN - FAKRA_CABLE_IN)
    g3 = gmsl_end + dv * (14.0 + FAKRA_CABLE_IN)
    return {
        "c11_gmsl": _run(
            "gmsl",
            [
                (gmsl_start_x, fy, fz),
                (gmsl_start_x + 18.0 + FAKRA_CABLE_IN, fy, fz),
                (g3.X - 6.4, fy - 32.0, g3.Z),
                (g3.X, g3.Y, g3.Z),
                (gmsl_end.X, gmsl_end.Y, gmsl_end.Z),
            ],
        )  # fmt: skip
    }


def _route_lan_tail() -> dict:
    rx, ry, rz = cl.ZED_RJ45_MOUTH
    cx, cz = cl.RJ45_AT
    coupler_stub_end_y = cl.Y_OUT[0] + RJ45_FLANGE_TO_STUB_END
    hi, lane, across = 100.0, -86.0, -108.0
    return {
        "lan_tail": _run(
            "lan",
            [
                (rx - 18.4, ry, rz + 6.75),
                (rx - 18.4, ry, hi),
                (lane, -66.0, hi),
                (lane, across, hi),
                (cx, across, hi),
                (cx, -130.0, hi),
                (cx, -140.0, cz),
                (cx, coupler_stub_end_y, cz),
            ],
        )  # fmt: skip
    }


def _route_esp_pigtail() -> dict:
    ux_, uy_, uz_ = ufl_point()
    ay, az = cl.ANTENNAS_YZ["esp32"]
    xc = cl.ZED_FAKRA_A_TIP[0] + 1.25
    ez_ = 101.0
    return {
        "esp_pigtail": _run(
            "esp",
            [
                (ux_, uy_, uz_),
                (ux_, uy_, ez_),
                (xc, -112.0, ez_),
                (xc, ay, ez_),
                (xc, ay, az),
                (cl.X_IN[1] - 7.0, ay, az),
            ],
        )  # fmt: skip
    }


def _route_display_lead(gh_top: float) -> dict:
    j7x, j7y = pcba_cage.PLACEMENT["J7_gh13_display"][2:]
    dy, dz = cl.DISP_TOUCH_Y + cl.DISP_GH[1], sum(DISP_PLUG_Z) / 2
    dx, run_z = 2.0, 82.0
    return {
        "display_lead": _run(
            "display",
            [
                (DISP_PLUG_END_X, dy, dz),
                (dx, dy, dz),
                (dx, dy, run_z),
                (dx, j7y, run_z),
                (j7x, j7y, run_z),
                (j7x, j7y, gh_top),
            ],
        )  # fmt: skip
    }


def _route_wifi_pigtails() -> dict:
    (s1x, s1y, s1z), (s2x, s2y, s2z) = cl.ZED_WIFI_SMA_JACKS
    w1y, w1z = cl.ANTENNAS_YZ["zed_wifi_1"]
    w2y, w2z = cl.ANTENNAS_YZ["zed_wifi_2"]
    inner = cl.X_IN[1] - 9.0
    w1, l1 = _run("wifi", [(s1x, w1y, s1z + 4.5), (s1x, w1y, w1z), (inner, w1y, w1z)])
    w2, l2 = _run("wifi", [(s2x + 4.5, w2y, s2z), (inner, w2y, w2z)])
    return {"wifi_pigtails": (Compound([w1, w2]), l1 + l2)}


@cache
def _cable_runs() -> dict:
    plug_top = UPS_TOP + MINIFIT_PLUG_TOP
    gh_top = pcba_cage.TOP + GH_PLUG_TOP
    return {
        **_route_zed_power(plug_top),
        **_route_battery_lead(plug_top),
        **_route_button_lead(gh_top),
        **_route_ups_ctrl_lead(gh_top),
        **_route_charge_lead(plug_top),
        **_route_charger_cord(),
        **_route_usb(),
        **_route_gmsl(),
        **_route_lan_tail(),
        **_route_esp_pigtail(),
        **_route_display_lead(gh_top),
        **_route_wifi_pigtails(),
    }


def cables() -> dict:
    out = _cable_runs()
    tether = cn.tube(
        [
            (-100.0, cl.Y_IN[1] + cl.WALL_T, cl.PANEL_Z1 + 12.0),
            (-100.0, cl.Y_IN[1] + 20.0, 62.0),
            (-100.0, 150.0, 20.0),
        ],  # fmt: skip
        3.2,
        bend_r=8.0,
    )
    desc = {
        "c4_zed_power": ("#c0392b", "OpenUPS VOUT to ZED Box Mini power, 2 x 16 AWG"),
        "battery_lead": ("#d35400", "Battery to OpenUPS BAT (J4): XT30, 10 A fuse; balance to J6"),
        "button_lead": ("#7d3c98", "Power button tabs to PCBA J5: GND, NO, NC, LED+, 28 AWG"),
        "ups_ctrl_lead": ("#a569bd", "PCBA J6 to OpenUPS J8 (start/stop), plus VBAT and GND"),
        "ups_ctrl_j10": ("#a569bd", "J6 branch to OpenUPS J10 (motherboard on/off pulse)"),
        "charge_lead": ("#c0392b", "GX12 to OpenUPS VIN (J1), 2 x 16 AWG"),
        "charger_cord": ("#1c1c1c", "Mean Well DC cord, barrel plug cut off, GX12 plug fitted"),
        "c6_usb": ("#222831", "ZED USB-A to PCBA USB-C, 90° plugs"),
        "c11_gmsl": ("#1f2d3d", "GMSL2 FAKRA cable, Ø3.35, 0.3 m, slack at the camera"),
        "lan_tail": ("#3b82c4", "RJ45 coupler's slim Cat6 tail to the ZED's Ethernet jack"),
        "esp_pigtail": ("#7a7a7a", "ESP32 u.FL to RP-SMA bulkhead pigtail"),
        "display_lead": ("#5d6d7e", "Display GH 13-pin to PCBA J7 (GH 13-pin), 28 AWG"),
        "wifi_pigtails": ("#8a8a8a", "ZED WiFi: 90° RP-SMA to bulkhead extensions"),
    }
    res = {k: (shape, *desc[k]) for k, (shape, _) in out.items()}
    res["tether"] = (tether, "#9aa0a6", '1/8" steel lanyard to the cage frame')
    return res


def cable_lengths() -> dict[str, float]:
    """Centreline length of each run, plugs excluded (python cage_pack_assembly.py)."""
    return {k: v[1] for k, v in _cable_runs().items()}


def _glass():
    """Outer sheet (1/4"), the 1" air gap, the inner sheet, and the top retainer plate."""
    top = cl.CUP_BELOW_EDGE
    w, h = 600.0, 500.0
    outer = Pos(0, top - h / 2, 0) * Box(w, h, cl.GLASS_T, align=C_MAX)
    inner = Pos(0, top - h / 2, -cl.GLASS_T - cl.GLASS_GAP) * Box(w, h, cl.GLASS_T, align=C_MAX)
    retainer = Pos(0, top + 25.0, 0) * Box(w, 75.0, 3.0, align=C_MIN)
    return outer, inner, retainer


def _bd(key: str) -> dict:
    d = dict(bd.COMPONENTS_BY_ASSEMBLY_NAME[key])
    if "note" in d and d["note"].startswith("Owned"):
        d["owned"] = True
    return d


def components():
    outer, inner, retainer = _glass()
    comps = {
        "glass_outer": {"shape": outer, "color": "#cfe8f5", "name": 'Cage outer sheet, 1/4" PC'},
        "glass_inner": {"shape": inner, "color": "#dbeef8", "name": 'Cage inner sheet, 1/4" PC'},
        "retainer": {
            "shape": retainer,
            "color": "#55595e",
            "name": "Cage retainer plate (assumed)",
        },
    }
    for side, x, rot in zip(("left", "right"), cl.CUP_X_NEG_POS, cl.CUP_ROT_NEG_POS, strict=True):
        comps[f"cup_{side}"] = {
            "shape": Pos(x, cl.CUP_Y, 0) * Rot(0, 0, rot) * cup.build(),
            "color": "#2b2f36",
            "material": "al6061",
            **bd.COMPONENTS_BY_ASSEMBLY_NAME["cup"],
        }
    for side, wall in (("left", "-x"), ("right", "+x")):
        comps[f"cup_arm_{side}"] = {
            "part": "cup_arm.py",
            "at": cup_arm.ASSEMBLY_AT[wall],
            "color": "#4a4e55",
            "material": "petg",
            "name": "Cup arm, black PETG print (two, one per side)",
        }
    s_inserts, n_inserts = _tray_inserts()
    comps.update(
        {
            "enclosure": {
                "part": "enclosure.py",
                "color": "#3a3d42",
                "material": "petg",
                "name": "Enclosure tray, black PETG print (Bambu P1S)",
                "note": "About 400 g of filament solid; 3 walls + 25% infill prints lighter",
            },
            "enclosure_lid": {
                "part": "enclosure_lid.py",
                "at": enclosure_lid.PRINT_TO_ASSEMBLY,
                "color": "#4a4e55",
                "material": "petg",
                "name": "Enclosure lid, black PETG print",
            },
            "battery_clamp": {
                "part": "battery_clamp.py",
                "color": "#4a4e55",
                "material": "petg",
                "name": "Battery clamp + OpenUPS standoffs, black PETG print",
            },
            "camera_shield": {
                "part": "camera_shield.py",
                "at": camera_shield.PRINT_TO_ASSEMBLY,
                "color": "#2a2a2a",
                "material": "petg",
                "name": "Camera light shield, black PETG print",
            },
            "shield_insert": {
                "shape": _shield_insert(),
                "color": "#c8a24a",
                "name": "M3 brass tapered heat-set insert, 6.4 mm",
                **bd.mcmaster("94180A333"),
            },
            "tray_inserts": {
                "shape": s_inserts,
                "color": "#c8a24a",
                "qty": n_inserts,
                "name": "M3 brass tapered heat-set insert, 6.4 mm",
                **bd.mcmaster("94180A333"),
            },
            "cup_washers": {
                "shape": _cup_washers(),
                "color": "#c0c4c8",
                "material": "steel",
                "qty": 2,
                "name": '3/8" 18-8 oversized washer, 1" OD',
                **bd.mcmaster("92141A051"),
            },
            "zed_box_mini": {
                "step": str(cl.ZED_STEP),
                "min_volume": 200,
                "at": Pos(cl.ZED_XC, cl.ZED_YC, cl.ZED_DZ),
                "color": "#3a3f47",
                **_bd("zed_box_mini"),
            },
            "battery": {
                "shape": _cells(),
                "color": "#2b2f33",
                **_bd("battery"),
            },
            "battery_holder": {
                "shape": _holder(),
                "color": "#1c1c1c",
                **_bd("battery_holder"),
            },
            "battery_carrier": {
                "shape": _carrier(),
                "color": "#1b5e20",
                **_bd("battery_carrier"),
            },
            "openups": {
                "step": str(cl.OPENUPS_STEP),
                "min_volume": 20,
                "at": UPS_LOC,
                "color": "#2e7d32",
                **_bd("openups"),
            },
            "pcba": {
                "part": "pcba_cage.py",
                "color": "#1b5e20",
                "name": "Custom PCBA, 44 x 46 (space claim), JLCPCB",
            },
            "display": {
                "step": str(cl.DISP_STEP),
                "min_volume": 1,
                "at": Pos(cl.DISP_TOUCH_X, cl.DISP_TOUCH_Y, cl.DISP_FRONT_Z - 0.7) * Rot(0, 0, -90),
                "color": "#1d2229",
                "material": "brass",
                **_bd("display"),
            },
            "display_bracket": {
                "part": "display_bracket.py",
                "color": "#4a4e55",
                "material": "petg",
                "name": "Display bracket, black PETG print",
            },
            "lid_inserts": {
                "shape": _lid_inserts(),
                "color": "#c8a24a",
                "qty": len(cl.DISP_EARS),
                "name": "M3 brass tapered heat-set insert, 6.4 mm",
                **bd.mcmaster("94180A333"),
            },
            "pcba_parts": {
                "shape": pcba_cage.parts_compound(),
                "color": "#b0b6bd",
                "name": "PCBA parts",
                "bom": False,
            },
            "charger": {
                "shape": _charger(),
                "color": "#2b2b2b",
                **_bd("charger"),
            },
            "ball_head": {
                "shape": HEAD * ballhead.build(BALL),
                "color": "#2b2f36",
                "material": "al6061",
                **bd.COMPONENTS_BY_ASSEMBLY_NAME["ball_head"],
            },
            "camera_bracket": {
                "part": str(cl.ZED_MOUNT / "zed_x_one_s_bracket.py"),
                "at": BRACKET_AT,
                "color": "#d18f2e",
                "material": "pla",
                "name": "ZED X One S bracket (zed_box_mini_mount), PLA print",
            },
            "front_mount": {
                "shape": BRACKET_AT * bracket.placed_front_mount(BRP),
                "color": "#5b8fd9",
                **_bd("front_mount"),
            },
            "camera": {
                "step": str(bracket.camera_body_step()),
                "at": CAMERA_AT,
                "color": "#20242a",
                **_bd("camera"),
            },
            "m4_inserts": {
                "shape": BRACKET_AT * bracket.inserts(BRP)["m4_inserts_94180A353"],
                "color": "#c8a24a",
                "qty": 4,
                "name": "M4 brass tapered heat-set insert, 7.9 mm",
                "vendor": "McMaster-Carr",
                "part_number": "94180A353",
                "url": "https://www.mcmaster.com/94180A353",
                "note": "Bracket parts already built (zed_box_mini_mount)",
                "owned": True,
            },
            "m4_washers": {
                "shape": BRACKET_AT * bracket.washers(BRP),
                "color": "#c0c4c8",
                "material": "steel",
                "qty": 4,
                "name": "M4 18-8 washer, 9 mm OD, 0.8 mm",
                "vendor": "McMaster-Carr",
                "part_number": "93475A230",
                "url": "https://www.mcmaster.com/93475A230",
                "owned": True,
                "note": "Already on the bracket",
            },
            "quarter_inserts": {
                "shape": BRACKET_AT * bracket.inserts(BRP)["insert_1_4_20_93365A160"],
                "color": "#c8a24a",
                "name": '1/4-20 brass tapered heat-set insert, 0.300"',
                "part_number": "93365A160",
                "vendor": "McMaster-Carr",
                "url": "https://www.mcmaster.com/93365A160",
                "note": "Already in the bracket",
                "owned": True,
            },
        }
    )
    for name, shape in _plugs().items():
        comps[name] = {
            "shape": shape,
            "color": "#9ea7b0",
            "name": name.replace("_", " "),
            "bom": False,
        }
    for name, (shape, color, desc) in cables().items():
        comps[name] = {"shape": shape, "color": color, "name": desc, "bom": False}
    comps["power_button"].update(_bd("power_button"))
    comps["power_button"].pop("bom")
    comps["c11_gmsl"].update(_bd("c11_gmsl"))
    comps["c11_gmsl"].pop("bom")
    for name, spec in comps.items():
        spec["mate_to"] = ONSHAPE_MATES.get(name, "fixed")
    return comps


ONSHAPE_PARTS = onshape_catalog.PARTS

ONSHAPE_MATES = {
    "enclosure": "fixed",
    "cup_arm_left": "enclosure",
    "cup_arm_right": "enclosure",
    "cup_left": "cup_arm_left",
    "cup_right": "cup_arm_right",
    "cup_washers": "cup_arm_right",
    "tray_inserts": "enclosure",
    "enclosure_lid": "enclosure",
    "lid_inserts": "enclosure_lid",
    "display": "enclosure_lid",
    "display_bracket": "enclosure_lid",
    "zed_box_mini": "enclosure",
    "pcba": "enclosure",
    "battery_clamp": "enclosure",
    "openups": "battery_clamp",
    "camera_shield": "enclosure",
    "shield_insert": "enclosure",
    "ball_head": "camera_shield",
    "camera_bracket": "ball_head",
    "m4_inserts": "camera_bracket",
    "quarter_inserts": "camera_bracket",
    "front_mount": "camera",
    "fakra_plug_cam": "camera",
    "rj45_coupler": "enclosure",
    "gx12_dc_in": "enclosure",
    "gx12_charger_plug": "gx12_dc_in",
    "charger_cord": "gx12_charger_plug",
    "power_button": "enclosure",
    "hr10_rcpt": "enclosure",
    "sma_bulkheads": "enclosure",
    **{f"antenna_{name}": "sma_bulkheads" for name in cl.ANTENNAS_YZ},
    "wifi_pigtails": "sma_bulkheads",
}


def bom_extra() -> list[dict]:
    rows = [dict(r) for r in bd.UNMODELED_EXTRAS]
    rows += [
        {
            "name": "ModDIY CO905 right-angle RJ45 extension, panel mount, 30 cm",
            "vendor": "ModDIY",
            "part_number": "CO905",
            "url": "https://www.moddiy.com/products/Right-Angled-RJ45-Ethernet-Extension-Cable-with-Panel-Mount-30cm.html",
            "price": 9.99,
            "note": "Panel end modelled from ModDIY's CAB105 (27.5 screw pitch): check on arrival",
        },
        {
            "name": "RP-SMA male to RP-SMA female bulkhead extension, 15 cm",
            "qty": 2,
            "note": "Not yet sourced: ZED Box Mini WiFi jacks to the +X wall",
        },
    ]
    rows.append(dict(bd.PCBA))
    return rows


def _mc(pn: str) -> dict:
    d = bd.mcmaster(pn)
    return {"mcmaster": pn, "price": d.get("price"), "note": d["note"]}


def fasteners():
    bl = BRACKET_AT
    head_y = BRP.plate_y + BRP.plate_t + bracket.WASHER_T
    m4_seats = [
        _world(bl, (sx * BRP.slot_x, head_y, BRP.axis_z + sz * BRP.screw_dz))
        for sx in (-1, 1)
        for sz in (-1, 1)
    ]
    w_t = cl.CUP_WASHER[1]
    arm_head_z = cl.PANEL_Z0 - cl.ARM_TONGUE_T + cl.ARM_CBORE.depth
    return [
        {
            "name": "cup_screws",
            "size": "3/8-16",
            "length": 0.625 * INCH,
            "head": "button",
            "dir": "-Z",
            "at": [(x, cl.CUP_Y, cl.ARM_SEAT_Z + w_t) for x in cl.CUP_X_NEG_POS],
            "material": "al6061",
            "description": '3/8-16 x 5/8" button head, alloy steel, black oxide',
            "note": 'McMaster part not picked yet (the 1/2" was 91255A619)',
        },
        {
            "name": "m3_cup_arms",
            "size": "M3",
            "length": 12,
            "head": "shcs",
            "dir": "+Z",
            "at": [(x, y, arm_head_z) for x, y in cl.arm_screws()],
            "into": "tray_inserts",
            "thread": "insert",
            "insert_len": cl.INSERT_LEN,
            **_mc("91292A114"),
        },
        {
            "name": "m3_zed_ears",
            "size": "M3",
            "length": 6,
            "head": "shcs",
            "dir": "-Z",
            "at": [(x, y, cl.PANEL_Z1 + cl.BOSS_H + 1.0) for x, y in cl.ZED_EAR_HOLES],
            "into": "tray_inserts",
            "thread": "insert",
            "insert_len": cl.INSERT_LEN,
            **_mc("91292A111"),
        },
        {
            "name": "m3_pcba",
            "size": "M3",
            "length": 8,
            "head": "shcs",
            "dir": "-Z",
            "at": [(x, y, pcba_cage.TOP) for x, y in pcba_cage.holes()],
            "into": "tray_inserts",
            "thread": "insert",
            "insert_len": cl.INSERT_LEN,
            **_mc("91292A112"),
        },
        {
            "name": "m3_openups",
            "size": "M3",
            "length": 16,
            "head": "shcs",
            "dir": "-Z",
            "at": [(x, y, UPS_TOP) for x, y in cl.UPS_HOLES],
            "into": "tray_inserts",
            "thread": "insert",
            "insert_len": cl.INSERT_LEN,
            **_mc("91292A115"),
        },
        {
            "name": "m3_lid",
            "size": "M3",
            "length": 8,
            "head": "button",
            "dir": "-Z",
            "at": [(x, y, enclosure_lid.LID_TOP_Z) for x, y in enclosure.lid_screws()],
            "into": "tray_inserts",
            "thread": "insert",
            "insert_len": cl.INSERT_LEN,
            **_mc("92095A181"),
        },
        {
            "name": "m3_display_bracket",
            "size": "M3",
            "length": 8,
            "head": "shcs",
            "dir": "+Z",
            "at": [
                (cl.DISP_TOUCH_X + x, cl.DISP_TOUCH_Y + y, display_bracket.Z0)
                for x, y in cl.DISP_EARS
            ],
            "into": "lid_inserts",
            "thread": "insert",
            "insert_len": cl.INSERT_LEN,
            **_mc("91292A112"),
        },
        {
            "name": "m25_display",
            "size": "M2.5",
            "length": 6,
            "head": "shcs",
            "dir": "+Z",
            "at": [
                (cl.DISP_TOUCH_X + x, cl.DISP_TOUCH_Y + y, display_bracket.Z0)
                for x, y in cl.DISP_STANDOFFS
            ],
            "into": "display",
            "thread": "tapped",
            "material": "brass",
            "description": "M2.5 x 6 socket head, 18-8: bracket into the display's standoffs",
            "note": "McMaster part not picked yet",
        },
    ] + [
        {
            "name": "head_screw",
            "size": "1/4-20",
            "length": 1.0 * INCH,
            "head": "shcs",
            "dir": "+X",
            "at": [_head_screw_seat()],
            "into": "ball_head",
            "thread": "tapped",
            "material": "al6061",
            "description": '1/4-20 x 1" socket head, 18-8: tray pad + shield into the head',
            **_mc(CAM_HEAD_SCREW_1_4_20_X1_SHCS_PN),
        },
        {
            "name": "m3_shield",
            "size": "M3",
            "length": 8,
            "head": "button",
            "dir": "-X",
            "at": [(cl.X_OUT[1] + cl.SHIELD_T, cl.CAM_Y, cl.SHIELD_SCREW_Z)],
            "into": "shield_insert",
            "thread": "insert",
            "insert_len": cl.INSERT_LEN,
            **_mc("92095A181"),
        },
        {
            "name": "m4_front_mount",
            "size": "M4",
            "length": 12,
            "head": "shcs",
            "dir": _world_dir(bl, (0, -1, 0)),
            "at": m4_seats,
            "into": "m4_inserts",
            "thread": "insert",
            "insert_len": BRP.m4_insert_len,
            "mcmaster": "91292A117",
            "note": "Already on the bracket",
        },
    ]


def sections():
    return [
        "x=0",
        f"y={cl.CUP_Y}",
        f"y={cl.arm_screws()[0][1]}",
        f"y={cl.CAM_Y}",
        f"y={cl.ZED_RJ45_MOUTH[1]:.1f}",
        f"y={cl.UPS_YC}",
        f"y={cl.DISP_TOUCH_Y}",
    ]


EXPECT = {
    "allow_interference": [
        ("cup_left", "glass_outer"),
        ("cup_right", "glass_outer"),
        ("quarter_inserts", "ball_head"),
        ("tray_inserts", "enclosure"),
        ("battery_clamp", "openups"),
        ("battery", "battery_holder"),
        ("quarter_inserts", "camera_bracket"),
        ("shield_insert", "enclosure"),
        ("shield_insert", "camera_shield"),
        ("m4_inserts", "camera_bracket"),
        ("front_mount", "camera_bracket"),
        ("front_mount", "camera"),
        ("pcba_parts", "pcba"),
        ("tbp_plug", "zed_box_mini"),
        ("usb_a_plug", "zed_box_mini"),
        ("rj45_plug", "zed_box_mini"),
        ("fakra_plug_zed", "zed_box_mini"),
        ("fakra_plug_cam", "camera"),
        ("rj45_coupler", "enclosure"),
        ("gx12_dc_in", "enclosure"),
        ("gx12_dc_in", "gx12_charger_plug"),
        ("charger_cord", "gx12_charger_plug"),
        ("charger_cord", "charger"),
        ("charge_lead", "gx12_dc_in"),
        ("charge_lead", "minifit_vin"),
        ("dupont_J8", "openups"),
        ("dupont_J10", "openups"),
        ("ups_ctrl_lead", "gh_J6"),
        ("ups_ctrl_lead", "dupont_J8"),
        ("ups_ctrl_j10", "dupont_J10"),
        ("ups_ctrl_j10", "ups_ctrl_lead"),
        ("button_lead", "power_button"),
        ("button_lead", "gh_J5"),
        ("hr10_rcpt", "enclosure"),
        ("hr10_rcpt", "hr10_tx_plug"),
        ("sma_bulkheads", "enclosure"),
        *[(f"antenna_{name}", "sma_bulkheads") for name in cl.ANTENNAS_YZ],
        ("minifit_vout", "openups"),
        ("minifit_vin", "openups"),
        ("minifit_bat", "openups"),
        ("mini_b_plug", "openups"),
        ("usb_c_plug", "pcba_parts"),
        ("gh_J2", "pcba_parts"),
        ("gh_J3", "pcba_parts"),
        ("gh_J4", "pcba_parts"),
        ("gh_J5", "pcba_parts"),
        ("gh_J6", "pcba_parts"),
        ("c11_gmsl", "fakra_plug_zed"),
        ("c11_gmsl", "fakra_plug_cam"),
        ("c4_zed_power", "tbp_plug"),
        ("c4_zed_power", "minifit_vout"),
        ("c6_usb", "usb_a_plug"),
        ("c6_usb", "usb_c_plug"),
        ("battery_lead", "battery"),
        ("battery_lead", "minifit_bat"),
        ("power_button", "enclosure"),
        ("lan_tail", "rj45_coupler"),
        ("lan_tail", "rj45_plug"),
        ("esp_pigtail", "pcba_parts"),
        ("esp_pigtail", "sma_bulkheads"),
        ("wifi_pigtails", "zed_box_mini"),
        ("wifi_pigtails", "sma_bulkheads"),
        ("tether", "enclosure"),
        ("tether", "retainer"),
        ("tether", "glass_outer"),
        ("lid_inserts", "enclosure_lid"),
        ("gh_J7", "pcba_parts"),
        ("gh_display", "display"),
        ("display_lead", "gh_display"),
        ("display_lead", "gh_J7"),
    ],
    "waive": {},
}


if __name__ == "__main__":
    for k, length in cable_lengths().items():
        d, r, src = CABLES[{"c4_zed_power": "power", "battery_lead": "battery",
                            "button_lead": "signal", "ups_ctrl_lead": "signal",
                            "ups_ctrl_j10": "signal", "charge_lead": "power",
                            "charger_cord": "dc", "c6_usb": "usb",
                            "c11_gmsl": "gmsl", "lan_tail": "lan", "esp_pigtail": "esp",
                            "display_lead": "display",
                            "wifi_pigtails": "wifi"}[k]]  # fmt: skip
        print(f"{k:14s} {length:6.0f} mm  Ø{d} R{r}  ({src})")
