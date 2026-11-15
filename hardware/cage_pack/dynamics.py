"""Mass budget and shock / vibration check for the cage pack on two SmallRig 4122C cups.

    venv/bin/python hardware/cage_pack/dynamics.py            # writes out/dynamics/

Model
- Outer polycarbonate sheet of the NHRL 3 lb cage wall: one 4' x 8' sheet, 1/4" thick
  (NHRL wiki, Cage Design). Linear Kirchhoff plate, simply supported on all four edges, modal
  superposition over m = 1..20, n = 1..10 (to ~600 Hz), 3 % modal damping. The cage holds
  every sheet in channels on all four sides, so the edges slide in plane and the sheet bends
  without membrane stiffening. That is the case that pries the cups hardest.
- The pack: a rigid body with three DOF at the midpoint between the cups (normal
  translation w_p, rotation theta_x about X and theta_y about Y), CG out from the glass by
  h. It couples to the plate only through the two cups (cage_layout.CUP_X, CUP_Y: one off
  each side wall, level). Each cup is a normal spring k_cup and, as a disk on an elastic
  foundation of radius R_CUP, a rocking spring k_cup R^2 / 4 about each in-plane axis
  against the sheet's slope under it, all with 10 % damping. Shear stiffness is taken as
  rigid, so in-plane loads come from the pack's rotation (a_y = -h theta_x'').
- Each cup's pull is its normal force plus its rocking moment as edge peel on the same
  disk: P = T + 4 M / R, the force that would load the seal's rim as hard if spread
  evenly. Two level cups carry the gravity moment and any rocking that way, since they
  have no vertical spacing to make a couple from.
- Excitation: a hit on the inner sheet reaches the outer sheet when the inner sheet slaps
  it across the 1" air gap. two_sheet.py models that contact for a robot hit and gives the
  momentum that arrives; here it lands on the outer sheet as a half-sine force pulse. The
  spinner bite keeps a 50 % guess: a point mass is no model of a blade biting.
- Channel engagement: two_sheet.py also gives how far each sheet's edge slides out of its
  channel, against ENGAGE (10 mm, assumed).
- Cup stiffness is unknown until the cups arrive, so every case runs over K_SWEEP and the
  page's slider picks one.
- Cup capacity: SmallRig rates 50 kg "horizontal mounting" (pull off the surface) and 35 kg
  "vertical mounting" (shear), B&H spec table. Checked as a linear interaction
  T/T_rated + V/V_rated <= 1 per cup, with the static gravity loads included.

Not modelled, and why
- The pack's effect on the sheet-to-sheet contact: two_sheet.py runs without it.
- Frame base motion: a 1.36 kg robot at 10 m/s into a cage of 150+ kg of steel moves the
  frame by under 0.1 m/s; the sheet's own flexing dominates near the pack.
- Glass-to-cup slip and vacuum leak-down over time.
"""

# Matrix and physics symbols keep their textbook capitals (M, K, C, N, J).
# ruff: noqa: N806
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import cage_layout as cl  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).parent
OUT = HERE / "out" / "dynamics"
G = 9.81

# ---------------------------------------------------------------------------------------
# Mass budget. Positions in cage_layout coordinates (mm): X along the wall, Y up from the
# frame origin (the upper CP10 hole), Z out from the glass. Printed parts are print_mass.py's
# PrusaSlicer numbers (petg_p1s.ini: 4 perimeters, 25 % gyroid).


@dataclass
class Item:
    name: str
    g: float
    x: float
    y: float
    z: float
    source: str


PANEL = 51.2  # tray panel's inner face, cage_layout.PANEL_Z1
MASS = [
    Item("ZED Box Mini", 600.0, 0.0, -50.0, PANEL + 8.0 + 26.8, "Stereolabs store page, 600 g"),
    Item(
        "Enclosure tray, PETG print",
        282.0,
        11.0,
        -78.0,
        66.0,
        "print_mass.py, panel pocketed to 1.2 mm",
    ),
    Item("Enclosure lid, PETG print", 87.6, 11.0, -78.0, 123.0, "print_mass.py, 1.2 mm skin"),
    Item(
        "Touch display, Waveshare 2.8in",
        42.0,
        53.5,
        -140.0,
        113.2,
        "Waveshare product page, 0.042 kg; centre of its 10 mm depth",
    ),
    Item("Display bracket, PETG print", 7.5, 53.5, -140.0, 106.9, "print_mass.py"),
    Item("Suction cup, -X", 264.0, cl.CUP_X[0], cl.CUP_Y, 24.0, "B&H spec, 264 g"),
    Item("Suction cup, +X", 264.0, cl.CUP_X[1], cl.CUP_Y, 24.0, "B&H spec, 264 g"),
    Item(
        "Cup arms, 2 PETG prints",
        78.0,
        (cl.CUP_X[0] + cl.CUP_X[1]) / 2,
        cl.CUP_Y,
        cl.CUP_H,
        "check.py 51.7 g solid each, x 0.75 for 4 perimeters + 25 % infill (estimate)",
    ),
    Item(
        "Battery: 4 cells, holder, carrier PCB",
        248.0,
        -58.0,
        -142.0,
        PANEL + 1.5 + 1.6 + 11.0,
        "4 x 47 g P30B + 35.6 g BK-18650-PC8 (MPD drawing) + 19 g carrier + 5 g foam",
    ),
    Item("OpenUPS", 70.0, -51.0, -143.0, 90.0, "estimate: 100 x 55 FR4 + connectors"),
    Item("Battery clamp, PETG print", 19.0, -51.0, -143.0, 80.0, "print_mass.py"),
    Item("Heat-set inserts, cup washers", 22.0, 0.0, -90.0, 60.0, "estimate: 26 x 0.4 g + 2 x 6 g"),
    Item(
        "Service-wall parts (RJ45 coupler, GX12, HR10 pair)",
        90.0,
        -110.0,
        -50.0,
        88.0,
        "estimate",
    ),
    Item("RP-SMA bulkheads + antennas", 45.0, 150.0, -18.0, 88.0, "estimate, 3 x 15 g"),
    Item(
        "Power button, CIT AH1LBBG12 + leads",
        30.0,
        72.0,
        -170.0,
        84.0,
        "estimate: no weight published",
    ),
    Item("Custom PCBA", 14.0, 34.0, -127.0, 60.0, "estimate: 44 x 46 FR4 + WROOM-1U"),
    Item("Camera ZED X One S, wide", 37.0, 98.0, -120.0, 30.0, "Stereolabs datasheet, 36-37 g"),
    Item(
        "Ball head + bracket + front mount",
        92.0,
        98.0,
        -140.0,
        40.0,
        "SmallRig 47 g; bracket and mount check.py x 0.6",
    ),
    Item("Camera shield, PETG", 64.0, 125.0, -153.0, 108.0, "print_mass.py; its centroid"),
    Item("Cables (power, USB, GMSL, LAN, RF, battery)", 75.0, 20.0, -100.0, 95.0, "estimate"),
    Item("Fasteners, inserts", 25.0, 0.0, -90.0, 60.0, "estimate"),
    Item("Display screws, inserts, GH lead", 8.0, 50.0, -135.0, 100.0, "estimate"),
]
CUP_RUBBER_G = 60.0  # part of each cup that moves with the glass, not with the pack (estimate)


def mass_budget() -> dict:
    tot = sum(i.g for i in MASS)
    cg = [sum(i.g * getattr(i, a) for i in MASS) / tot for a in "xyz"]
    pack_g = tot - 2 * CUP_RUBBER_G
    return {"items": [asdict(i) for i in MASS], "total_g": tot, "cg_mm": cg, "pack_g": pack_g}


E_PC, NU_PC, RHO_PC = 2.38e9, 0.37, 1200.0  # polycarbonate, Covestro Makrolon typical
H = 0.00635  # 1/4"
A = 2.438  # 8' along the wall
B = 1.219  # 4' tall
D = E_PC * H**3 / (12 * (1 - NU_PC**2))
RHO_H = RHO_PC * H
# Frame origin below the sheet's supported top edge (assumed), and the cups from it, m.
CUP_BELOW_EDGE = cl.CUP_BELOW_EDGE / 1000
CUPS = [(x / 1000, cl.CUP_Y / 1000) for x in cl.CUP_X]
CUP_NAMES = ("-X", "+X")
R_CUP = cl.CUP_D / 2 / 1000  # rubber face radius
ZETA_PANEL = 0.03
ZETA_CUP = 0.10
T_RATED = 50.0 * G  # per cup, pull
V_RATED = 35.0 * G  # per cup, shear
# Cup axial stiffness sweep, N/m per cup: 10 kN/m to 1 MN/m in tenth-decade steps. Includes
# 20, 100 and 400 kN/m (to 0.3 %), the old three columns.
K_SWEEP = [float(k) for k in np.round(np.geomspace(1e4, 1e6, 21), -2)]
K_DEFAULT = 1e5
ENGAGE = 0.010  # m, sheet depth in each edge channel (assumed)
MODES = [(m, n) for m in range(1, 21) for n in range(1, 11)]


def phi(x, y):
    """Mode shapes and their Y slopes at (x, y), metres, origin at the bottom-left corner."""
    m = np.array([mn[0] for mn in MODES])
    n = np.array([mn[1] for mn in MODES])
    sx, sy = np.sin(m * np.pi * x / A), np.sin(n * np.pi * y / B)
    dy = np.sin(m * np.pi * x / A) * (n * np.pi / B) * np.cos(n * np.pi * y / B)
    return sx * sy, dy


def phi_dx(x, y):
    """Mode shapes' X slopes at (x, y)."""
    m = np.array([mn[0] for mn in MODES])
    n = np.array([mn[1] for mn in MODES])
    return (m * np.pi / A) * np.cos(m * np.pi * x / A) * np.sin(n * np.pi * y / B)


def cup_frame(pack: dict):
    """Cup midpoint, cup offsets and CG offset in frame metres."""
    x0 = sum(c[0] for c in CUPS) / len(CUPS)
    y0 = sum(c[1] for c in CUPS) / len(CUPS)
    rel = [(cx - x0, cy - y0) for cx, cy in CUPS]
    xc = pack["cg_mm"][0] / 1000 - x0
    yc = pack["cg_mm"][1] / 1000 - y0
    return (x0, y0), rel, (xc, yc)


def static_cups(pack: dict) -> list[dict]:
    """Gravity on the pack, per cup: normal pull T, rocking moments, shear, and the pull P
    with edge peel. The cups share the gravity moment M g h by their stiffness (k cancels)."""
    _, rel, _ = cup_frame(pack)
    M = pack["pack_g"] / 1000.0
    h = pack["cg_mm"][2] / 1000.0
    kr = R_CUP**2 / 4  # per unit k
    K = np.zeros((3, 3))
    for dx, dy in rel:
        r = np.array([1.0, dy, dx])
        K += np.outer(r, r)
    K[1, 1] += len(rel) * kr
    K[2, 2] += len(rel) * kr
    u = np.linalg.solve(K, [0.0, M * G * h, 0.0])
    out = []
    for dx, dy in rel:
        t = u[0] + u[1] * dy + u[2] * dx
        mx, my = kr * u[1], kr * u[2]
        p = t + 4 * math.hypot(mx, my) / R_CUP
        out.append({"T": t, "Mx": mx, "My": my, "V": M * G / len(rel), "P": p})
    return out


def omegas():
    m = np.array([mn[0] for mn in MODES])
    n = np.array([mn[1] for mn in MODES])
    return np.pi**2 * ((m / A) ** 2 + (n / B) ** 2) * math.sqrt(D / RHO_H)


@dataclass
class Case:
    name: str
    momentum: float  # N s reaching the outer sheet
    tau: float  # s, half-sine pulse length
    hit: tuple  # (x, y) m on the sheet
    pack_x: float  # m from the left post
    k_cup: float  # N/m per cup
    berger: bool = False  # True: edges pinned in plane (not this cage's channels)


def simulate(case: Case, pack: dict, t_end: float = 0.25, dt: float = 1e-5) -> dict:
    nm = len(MODES)
    w = omegas()
    mm = RHO_H * A * B / 4  # modal mass, unit-amplitude sine modes
    (x0, y0), rel, (xc, yc) = cup_frame(pack)
    ys = B - CUP_BELOW_EDGE + y0  # cup line on the sheet; pack_x is the frame origin's x
    M = pack["pack_g"] / 1000.0
    h = pack["cg_mm"][2] / 1000.0
    J = M * (0.08**2 + 0.05**2) / 3  # about its own CG, box ~ 160 x 100 x 100 mm
    # DOFs: [q_1..q_nm, w_p, theta_x, theta_y]; pack normal motion at (dx, dy) from the
    # reference is w_p + theta_x dy + theta_y dx.
    n = nm + 3
    Mm = np.zeros((n, n))
    Mm[:nm, :nm] = np.eye(nm) * mm
    Mm[nm:, nm:] = M * np.array(
        [
            [1.0, yc, xc],
            [yc, yc**2 + h**2 + J / M, yc * xc],
            [xc, yc * xc, xc**2 + h**2 + J / M],
        ]
    )
    K = np.zeros((n, n))
    K[:nm, :nm] = np.diag(mm * w**2)
    C = np.zeros((n, n))
    C[:nm, :nm] = np.diag(2 * ZETA_PANEL * mm * w)
    # Each cup: an axial row (stretch w_p + theta_x dy + theta_y dx - phi . q) and two
    # rocking rows (pack rotation less the sheet's slope under the cup).
    k, kr = case.k_cup, case.k_cup * R_CUP**2 / 4
    c = 2 * ZETA_CUP * M / 2 * math.sqrt(k * 2 / M)
    cr = c * R_CUP**2 / 4
    rows = []
    for dx, dy in rel:
        px, py = case.pack_x + x0 + dx, ys + dy
        ph, ph_y = phi(px, py)
        ph_x = phi_dx(px, py)
        ra = np.concatenate([-ph, [1.0, dy, dx]])
        rx = np.concatenate([-ph_y, [0.0, 1.0, 0.0]])
        ry = np.concatenate([-ph_x, [0.0, 0.0, 1.0]])
        rows.append((ra, rx, ry))
        K += k * np.outer(ra, ra) + kr * (np.outer(rx, rx) + np.outer(ry, ry))
        C += c * np.outer(ra, ra) + cr * (np.outer(rx, rx) + np.outer(ry, ry))
    pf, _ = phi(*case.hit)
    f_dir = np.concatenate([pf, [0.0, 0.0, 0.0]])
    # Berger membrane stiffening: a uniform in-plane tension N from the sheet's stretch,
    # N = E h / (2 (1 - nu^2) A B) * integral |grad w|^2, edges held in plane. Each mode then
    # gains stiffness N k_i^2 (A B / 4). Off when berger is False (linear upper bound).
    kk = np.pi**2 * np.array([(m / A) ** 2 + (nn / B) ** 2 for m, nn in MODES])
    nfac = E_PC * H / (2 * (1 - NU_PC**2) * A * B) * (A * B / 4)
    Minv = np.linalg.inv(Mm)
    u = np.zeros(n)
    v = np.zeros(n)
    steps = int(t_end / dt)
    amp = case.momentum * np.pi / (2 * case.tau)
    ncup = len(rel)
    rec = {key: np.zeros(steps) for key in ("t", "Vy", "Vx", "acc", "wmax")}
    rec |= {f"{key}{j}": np.zeros(steps) for key in ("T", "Mx", "My") for j in range(ncup)}
    pc, _ = phi(A / 2, B / 2)
    fnl = np.zeros(n)
    for i in range(steps):
        t = i * dt
        f = f_dir * (amp * math.sin(math.pi * t / case.tau) if t <= case.tau else 0.0)
        if case.berger:
            N = nfac * np.sum(kk * u[:nm] ** 2)
            fnl[:nm] = N * kk * (A * B / 4) * u[:nm]
        a = Minv @ (f - C @ v - K @ u - fnl)
        v = v + dt * a  # semi-implicit Euler: stable for dt << 1 / omega_max
        u = u + dt * v
        rec["t"][i] = t
        for j, (ra, rx, ry) in enumerate(rows):
            rec[f"T{j}"][i] = k * (ra @ u) + c * (ra @ v)
            rec[f"Mx{j}"][i] = kr * (rx @ u) + cr * (rx @ v)
            rec[f"My{j}"][i] = kr * (ry @ u) + cr * (ry @ v)
        rec["Vy"][i] = M * h * a[nm + 1]
        rec["Vx"][i] = M * h * a[nm + 2]
        rec["acc"][i] = a[nm] + a[nm + 1] * yc + a[nm + 2] * xc
        rec["wmax"][i] = max(abs(pc @ u[:nm]), abs(pf @ u[:nm]))
    # Static gravity loads added to the dynamic ones, then each cup's pull with edge peel.
    st = static_cups(pack)
    pulls = []
    for j in range(ncup):
        T = rec[f"T{j}"] + st[j]["T"]
        mom = np.hypot(rec[f"Mx{j}"] + st[j]["Mx"], rec[f"My{j}"] + st[j]["My"])
        pulls.append(T + 4 * mom / R_CUP)
    Vc = np.hypot(rec["Vy"] / ncup - st[0]["V"], rec["Vx"] / ncup)
    ratio = np.max(pulls, axis=0) / T_RATED + Vc / V_RATED
    return {
        "case": asdict(case),
        "peak_pull_N": [float(pp.max()) for pp in pulls],
        "peak_shear_per_cup_N": float(Vc.max()),
        "peak_pack_accel_g": float(np.abs(rec["acc"]).max() / G),
        "peak_sheet_deflection_mm": float(rec["wmax"].max() * 1000),
        "linear_valid": bool(rec["wmax"].max() < 3 * H),
        "peak_interaction": float(ratio.max()),
        "series": {key: rec[key][::25].tolist() for key in ("t", "acc")}
        | {"P": np.max(pulls, axis=0)[::25].tolist()},
    }


def free_frequencies(pack: dict, pack_x: float, k_cup: float) -> dict:
    """First panel frequencies, bare and with the pack, and the pack's own rocking mode."""
    w = omegas()
    order = np.argsort(w)
    bare = [(MODES[i], float(w[i] / 2 / np.pi)) for i in order[:6]]
    M = pack["pack_g"] / 1000.0
    h = pack["cg_mm"][2] / 1000.0
    J = M * (0.08**2 + 0.05**2) / 3 + M * h**2
    (_, y0), rel, _ = cup_frame(pack)
    # Rocking about X: the cups' vertical spacing as a couple plus each one's own disk.
    k_rot = sum(k_cup * dy**2 for _, dy in rel) + len(rel) * k_cup * R_CUP**2 / 4
    return {
        "bare_panel_hz": [{"mode": list(m), "hz": round(f, 2)} for m, f in bare],
        "pack_bounce_hz": round(math.sqrt(2 * k_cup / M) / 2 / np.pi, 1),
        "pack_rocking_hz": round(math.sqrt(k_rot / J) / 2 / np.pi, 1),
        "mode_amplitude_at_cups": round(math.sin(math.pi * (B - CUP_BELOW_EDGE + y0) / B), 3),
    }


# name: (robot speed m/s for two_sheet.py, or None; momentum N s if None; pulse s; description).
# Robot cases take the momentum that two_sheet.py finds reaching the outer sheet.
SCENARIOS = {
    "robot_driven_5ms": (5.0, None, 0.008, "3 lb robot driven into the wall at 5 m/s"),
    "robot_slam_10ms": (10.0, None, 0.008, "3 lb robot driven or thrown into the wall at 10 m/s"),
    "robot_thrown_20ms": (20.0, None, 0.006, "3 lb robot launched at 20 m/s by a vertical spinner"),
    "spinner_bite": (
        None,
        0.35 * 110.0 * 0.5,
        0.002,
        "weapon bar (0.35 kg) at 110 m/s tip speed bites the inner sheet; half transferred (guess)",
    ),
    "worst_full_transfer": (
        None,
        1.36 * 20.0,
        0.004,
        "20 m/s robot, all its momentum on the outer sheet in 4 ms",
    ),
}
PULL_OUT_SPEEDS = [5.0, 7.0, 10.0, 12.0, 14.0, 16.0, 20.0]


def wall_impacts() -> dict:
    """two_sheet.py over robot speeds: transferred momentum and channel pull-in per sheet."""
    import two_sheet

    out = {}
    for v in sorted(set(PULL_OUT_SPEEDS) | {sc[0] for sc in SCENARIOS.values() if sc[0]}):
        r = two_sheet.run(v, (A / 2, B / 2))
        out[v] = {
            "momentum_out": r["impulse_to_outer"],
            "inner_mm": r["max_inner"] * 1000,
            "outer_mm": r["max_outer"] * 1000,
            "pull_in_inner_mm": r["pull_in_inner"] * 1000,
            "pull_in_outer_mm": r["pull_in_outer"] * 1000,
            "sigma_outer_mpa": r["max_sigma_outer"] / 1e6,
        }
        print(f"two_sheet v={v:4.1f}", {k: round(x, 2) for k, x in out[v].items()})
    return out


def pull_out_speed(wall: dict, key: str) -> float | None:
    """Robot speed where a sheet's pull-in reaches ENGAGE, linear between sampled speeds."""
    vs = sorted(wall)
    for v0, v1 in zip(vs, vs[1:]):
        p0, p1 = wall[v0][key], wall[v1][key]
        if p0 < ENGAGE * 1000 <= p1:
            return v0 + (ENGAGE * 1000 - p0) / (p1 - p0) * (v1 - v0)
    return None


def _sim(args):
    c, pack = args
    r = simulate(c, pack)
    if c.k_cup != K_DEFAULT:
        r.pop("series")
    return r


def _kn(k: float) -> str:
    """kN/m to two significant figures: 501200 -> '500'."""
    return f"{float(f'{k / 1000:.2g}'):g}"


def write_fragment(summary: dict) -> None:
    """HTML section for page_assembly.py --extra with the stiffness slider."""
    import base64
    import html
    import string

    e = html.escape
    m = summary["mass"]
    rows = "".join(
        f"<tr><td>{e(i['name'])}</td><td class=num>{i['g']:.0f}</td><td>{e(i['source'])}</td></tr>"
        for i in sorted(m["items"], key=lambda i: -i["g"])
    )
    cg = m["cg_mm"]
    st = summary["static"]
    res = summary["results"]
    ks = summary["k_sweep"]
    scen = summary["scenarios"]

    def worst(sname, k):
        q = [r for r in res if r["scenario"] == sname and r["case"]["k_cup"] == k]
        return max(q, key=lambda r: r["peak_interaction"])

    # Per scenario: the worst case at every swept stiffness, and the stiffest cup that holds.
    data, holds = {}, {}
    for sname in scen:
        ws = [worst(sname, k) for k in ks]
        data[sname] = [
            [
                round(w["peak_interaction"], 3),
                round(w["peak_pull_N"][0]),
                round(w["peak_pull_N"][1]),
                round(w["peak_shear_per_cup_N"]),
                round(w["peak_pack_accel_g"], 1),
                round(w["peak_sheet_deflection_mm"], 1),
                w["hit"].replace("_", " ") + ", " + w["pack_pos"].replace("_", " "),
            ]
            for w in ws
        ]
        ok = [k for k, w in zip(ks, ws) if w["peak_interaction"] <= 1.0]
        # Holds up to the last stiffness before the first failure.
        first_fail = next((k for k, w in zip(ks, ws) if w["peak_interaction"] > 1.0), None)
        if first_fail is None:
            holds[sname] = f"all, to {_kn(ks[-1])}"
        elif not ok or ok[0] > first_fail:
            holds[sname] = f"none, from {_kn(ks[0])}"
        else:
            holds[sname] = f"up to {_kn(max(k for k in ok if k < first_fail))}"
    po = summary["pull_out_speed"]

    def desc(d):
        past = d["speed"] and po["inner"] and d["speed"] > po["inner"]
        return e(d["desc"]) + (
            " <b>Past the inner sheet's channel pull-out: the wall fails first.</b>" if past else ""
        )

    srows = "".join(
        f'<tr data-s="{e(n)}"><td><b>{e(n.replace("_", " "))}</b><br><small>{desc(d)}</small></td>'
        f'<td class=num>{d["momentum"]:.1f}</td><td class=num data-f="acc"></td>'
        f'<td class=num data-f="pull"></td>'
        f'<td class=num data-f="shear"></td><td class=num data-f="sheet"></td><td data-f="ir"></td>'
        f"<td class=num>{e(holds[n])}</td></tr>"
        for n, d in scen.items()
    )

    wall = summary["wall"]
    eng = summary["engage_mm"]

    def pull_cell(mm):
        cls = "PASS" if mm <= 0.5 * eng else ("WARN" if mm <= eng else "FAIL")
        return f'<td class="num {cls}">{mm:.1f}</td>'

    wrows = "".join(
        f"<tr><td class=num>{float(v):.0f}</td>"
        f"<td class=num>{w['momentum_out']:.1f} of {1.36 * float(v):.1f}</td>"
        f"<td class=num>{w['inner_mm']:.0f} / {w['outer_mm']:.0f}</td>"
        f"<td class=num>{w['sigma_outer_mpa']:.0f}</td>"
        f"{pull_cell(w['pull_in_inner_mm'])}{pull_cell(w['pull_in_outer_mm'])}</tr>"
        for v, w in sorted(wall.items(), key=lambda kv: float(kv[0]))
    )
    po_in = f"{po['inner']:.1f}" if po["inner"] else "over 20"
    po_out = f"{po['outer']:.1f}" if po["outer"] else "over 20"

    f = summary["frequencies"]
    modes = ", ".join(f"{d['hz']:.1f}" for d in f["100000.0"]["bare_panel_hz"])
    png = base64.b64encode((OUT / "governing_cases.png").read_bytes()).decode()
    vals = {
        "total_kg": f"{m['total_g'] / 1000:.2f}",
        "pack_kg": f"{m['pack_g'] / 1000:.2f}",
        "total_g": f"{m['total_g']:.0f}",
        "cg_x": f"{cg[0]:.0f}",
        "cg_y": f"{cg[1]:.0f}",
        "cg_z": f"{cg[2]:.0f}",
        "static_margin": f"{1 / st['interaction_worst']:.0f}",
        "static_shear": f"{st['shear_per_cup_N']:.1f}",
        "static_pull": f"{st['pull_worst_N']:.1f}",
        "cup_names": " / ".join(CUP_NAMES),
        "span": f"{(CUPS[1][0] - CUPS[0][0]) * 1000:.0f}",
        "cup_below": f"{(CUP_BELOW_EDGE - CUPS[0][1]) * 1000:.0f}",
        "srows": srows,
        "wrows": wrows,
        "rows": rows,
        "png": png,
        "modes": modes,
        "bounce_lo": f["20000.0"]["pack_bounce_hz"],
        "bounce_hi": f["400000.0"]["pack_bounce_hz"],
        "rock_lo": f["20000.0"]["pack_rocking_hz"],
        "rock_hi": f["400000.0"]["pack_rocking_hz"],
        "edge_amp": f"{f['100000.0']['mode_amplitude_at_cups']:.2f}",
        "engage": f"{eng:.0f}",
        "po_in": po_in,
        "po_out": po_out,
        "hold_5": holds["robot_driven_5ms"],
        "hold_10": holds["robot_slam_10ms"],
        "k_n": len(ks) - 1,
        "k_default": ks.index(K_DEFAULT),
        "k_json": json.dumps(ks),
        "data_json": json.dumps(data),
        "w10_out": f"{wall['10.0']['outer_mm']:.0f}",
        "w10_in": f"{wall['10.0']['inner_mm']:.0f}",
        "w10_frac": f"{wall['10.0']['momentum_out'] / 13.6 * 100:.0f}",
    }
    tpl = (HERE / "dynamics_report.html").read_text().split("\n", 1)[1]
    body = string.Template(tpl).substitute(vals)
    (OUT / "analysis.html").write_text("Mass and shock loads\n" + body)


def main() -> None:
    import sys

    if "--fragment-only" in sys.argv:
        write_fragment(json.loads((OUT / "report.json").read_text()))
        return
    OUT.mkdir(parents=True, exist_ok=True)
    pack = mass_budget()
    yb = B - CUP_BELOW_EDGE + CUPS[0][1]  # the cup line on the sheet
    wall = wall_impacts()
    # Momentum reaching the outer sheet. 3 lb robot = 1.36 kg.
    scen = {
        name: (wall[v]["momentum_out"] if v else p, tau, desc, v)
        for name, (v, p, tau, desc) in SCENARIOS.items()
    }
    hits = {
        "below_pack": lambda px: (px, B / 2),
        "centre": lambda px: (A / 2, B / 2),
        "near_pack": lambda px: (px + 0.15, yb - 0.15),
    }
    jobs = []
    for pack_x_name, pack_x in (("mid_span", A / 2), ("near_post", 0.35)):
        for k_cup in K_SWEEP:
            for sname, (p, tau, _desc, _v) in scen.items():
                for hname, hf in hits.items():
                    c = Case(
                        f"{sname}/{hname}/{pack_x_name}/k{int(k_cup / 1000)}",
                        p,
                        tau,
                        hf(pack_x),
                        pack_x,
                        k_cup,
                    )
                    jobs.append(
                        ((c, pack), {"scenario": sname, "hit": hname, "pack_pos": pack_x_name})
                    )
    from multiprocessing import Pool

    with Pool() as pool:
        sims = pool.map(_sim, [j[0] for j in jobs], chunksize=4)
    results = []
    for (_args, tags), r in zip(jobs, sims):
        r.update(tags | {"desc": scen[tags["scenario"]][2]})
        results.append(r)
    for r in results:
        if r["case"]["k_cup"] == K_DEFAULT and r["pack_pos"] == "mid_span":
            print(
                f"{r['case']['name']:46s} P={r['case']['momentum']:5.1f} "
                f"acc {r['peak_pack_accel_g']:6.1f} g P {max(r['peak_pull_N']):6.0f} N "
                f"V {r['peak_shear_per_cup_N']:5.0f} N IR {r['peak_interaction']:.2f} "
                f"sheet {r['peak_sheet_deflection_mm']:5.1f} mm"
            )
    freqs = {str(k): free_frequencies(pack, A / 2, k) for k in (2e4, 1e5, 4e5)}
    st = static_cups(pack)
    static_p = max(c["P"] for c in st)
    summary = {
        "mass": pack,
        "panel": {"E": E_PC, "h_m": H, "a_m": A, "b_m": B, "D_Nm": D},
        "static": {
            "shear_per_cup_N": st[0]["V"],
            "pull_worst_N": static_p,
            "per_cup": dict(zip(CUP_NAMES, st, strict=True)),
            "interaction_worst": static_p / T_RATED + st[0]["V"] / V_RATED,
        },
        "frequencies": freqs,
        "results": [{k: v for k, v in r.items() if k != "series"} for r in results],
        "wall": {str(v): w for v, w in wall.items()},
        "pull_out_speed": {
            "inner": pull_out_speed(wall, "pull_in_inner_mm"),
            "outer": pull_out_speed(wall, "pull_in_outer_mm"),
        },
        "engage_mm": ENGAGE * 1000,
        "k_sweep": K_SWEEP,
        "scenarios": {
            n: {"momentum": p, "tau": t, "desc": d, "speed": v} for n, (p, t, d, v) in scen.items()
        },
    }
    (OUT / "report.json").write_text(json.dumps(summary, indent=1))
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for sname in scen:
        rr = [r for r in results if r["scenario"] == sname and r["case"]["k_cup"] == K_DEFAULT]
        r = max(rr, key=lambda r: r["peak_interaction"])
        t = np.array(r["series"]["t"]) * 1000
        ax[0].plot(
            t, np.array(r["series"]["acc"]) / G, label=f"{sname} ({r['hit']}, {r['pack_pos']})"
        )
        ax[1].plot(t, r["series"]["P"], label=sname)
    ax[1].axhline(T_RATED, color="k", ls="--", lw=1, label="rated pull 490 N")
    ax[0].set_ylabel("pack normal accel (g)")
    ax[1].set_ylabel("worst cup pull, edge peel in (N)")
    ax[1].set_xlabel("time (ms)")
    ax[0].legend(fontsize=7)
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(OUT / "governing_cases.png", dpi=110)
    write_fragment(summary)
    print(
        "total",
        round(pack["total_g"]),
        "g, pack",
        round(pack["pack_g"]),
        "g, CG",
        [round(c, 1) for c in pack["cg_mm"]],
    )
    print(json.dumps(freqs, indent=1))
    print("static", summary["static"])


if __name__ == "__main__":
    main()
