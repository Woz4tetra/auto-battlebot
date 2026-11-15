"""Two-sheet cage wall impact: robot -> inner sheet -> 1" air gap -> outer sheet.

    venv/bin/python hardware/cage_pack/two_sheet.py --v 10 [--held] [--hit x,y]

Gives the momentum that reaches the outer sheet, which dynamics.py takes as its input,
and how far each sheet pulls out of its edge channels.

- Both sheets: 1/4" PC, 2.44 x 1.22 m, simply supported, modal (sin x sin) to FMAX = 600 Hz,
  3 % modal damping. The cage holds them in channels on all four sides, so the edges slide
  in plane: plain linear bending, no membrane stiffening. --held adds Berger stiffening
  (edges pinned in plane) for comparison.
- Robot: 1.36 kg point mass with a compression-only contact spring on the inner sheet.
- Sheet to sheet: compression-only penalty springs on a 61 x 31 grid, 25.4 mm gap.
- Channel pull-in: an edge slides in by half the arc-length excess of the bent sheet across
  that span, 1/2 * integral of slope^2, split between the two edges; the worst line across
  the sheet is reported.
- Semi-implicit Euler on modal coordinates, dt 10 us; halving dt changes results under 1 %.
"""

# Matrix and physics symbols keep their textbook capitals (M, P, N).
# ruff: noqa: N806
from __future__ import annotations

import argparse
import json

import numpy as np

A_M, B_M, H_M = 2.44, 1.22, 0.00635
E, NU, RHO = 2.38e9, 0.37, 1200.0
D = E * H_M**3 / (12 * (1 - NU**2))
GAP = 0.0254
ZETA = 0.03
FMAX = 600.0
YIELD_PA = 62e6
ROBOT_MASS_KG = 1.36
UPPER_CUP_BELOW_EDGE_M = 0.130
LOWER_CUP_BELOW_EDGE_M = 0.280


def modes():
    c = np.sqrt(D / (RHO * H_M))
    mn, k2 = [], []
    for m in range(1, 60):
        for n in range(1, 30):
            kk = (m * np.pi / A_M) ** 2 + (n * np.pi / B_M) ** 2
            if kk * c / (2 * np.pi) <= FMAX:
                mn.append((m, n))
                k2.append(kk)
    mn = np.array(mn)
    k2 = np.array(k2)
    omega = np.sqrt(k2) ** 2 * c
    return mn, k2, omega


def shapes(mn, x, y):
    """phi[i, j] = sin(m pi x_i / A) sin(n pi y_i / B)."""
    return np.sin(np.outer(x, mn[:, 0] * np.pi / A_M)) * np.sin(np.outer(y, mn[:, 1] * np.pi / B_M))


def run(
    v0, hit, berger=False, k_robot=1e6, k_sheet=2e5, nx=61, ny=31, t_end=0.15, dt=1e-5, probes=()
):
    """One impact. Returns peaks: deflections (m), forces (N), stress (Pa), impulse (N s)."""
    return _run(
        v0, hit, berger, k_robot, k_sheet, nx, ny, t_end, dt, list(probes) or [(A_M / 2, B_M / 2)]
    )


def _run(v0, hit, berger, k_robot, k_sheet, nx, ny, t_end, dt, probes):
    mn, k2, w = modes()
    modal_mass = RHO * H_M * A_M * B_M / 4
    gx, gy = np.meshgrid(np.linspace(0, A_M, nx + 2)[1:-1], np.linspace(0, B_M, ny + 2)[1:-1])
    gx, gy = gx.ravel(), gy.ravel()
    phi_grid = shapes(mn, gx, gy)
    ph = shapes(mn, np.array([hit[0]]), np.array([hit[1]]))[0]
    pr = shapes(mn, np.array([p[0] for p in probes]), np.array([p[1] for p in probes]))
    kx2 = (mn[:, 0] * np.pi / A_M) ** 2
    ky2 = (mn[:, 1] * np.pi / B_M) ** 2
    # Berger: N = (3 D / (2 h^2)) sum q^2 k^2 (tension per unit length, both directions)
    cb = 3 * D / (2 * H_M**2) if berger else 0.0

    qi = np.zeros(len(k2))
    vi = np.zeros(len(k2))
    qo = np.zeros(len(k2))
    vo = np.zeros(len(k2))
    robot_x, robot_v, robot_m = 0.0, v0, ROBOT_MASS_KG

    # Channel pull-in. Along Y at column x: 1/2 int w_y^2 dy = 1/2 sum_n (n pi/B)^2 (B/2) a_n(x)^2,
    # a_n(x) = sum_m q_mn sin(m pi x / A); the same along X. Sampled on 41 lines each way.
    xs, ys = np.linspace(0, A_M, 43)[1:-1], np.linspace(0, B_M, 23)[1:-1]
    sin_lines_by_mode = np.sin(np.outer(xs, mn[:, 0] * np.pi / A_M))
    syn = np.sin(np.outer(ys, mn[:, 1] * np.pi / B_M))
    nset, mset = sorted(set(mn[:, 1])), sorted(set(mn[:, 0]))
    nidx = [mn[:, 1] == n for n in nset]
    midx = [mn[:, 0] == m for m in mset]

    def pull_in(q):
        amp_lines_by_n = np.stack([(sin_lines_by_mode[:, k] * q[k]).sum(1) for k in nidx], 1)
        dy = (
            0.5 * ((np.array(nset) * np.pi / B_M) ** 2 * (B_M / 2) * amp_lines_by_n**2).sum(1).max()
        )
        ax = np.stack([(syn[:, k] * q[k]).sum(1) for k in midx], 1)
        dx = 0.5 * ((np.array(mset) * np.pi / A_M) ** 2 * (A_M / 2) * ax**2).sum(1).max()
        pull_in_per_edge = max(dx, dy) / 2
        return pull_in_per_edge

    out = {
        "max_outer": 0.0,
        "pull_in_inner": 0.0,
        "pull_in_outer": 0.0,
        "max_inner": 0.0,
        "max_robot_force": 0.0,
        "max_sheet_force": 0.0,
        "max_sigma_outer": 0.0,
        "probe_max": [0.0] * len(probes),
        "pry_max": 0.0,
        "impulse_to_outer": 0.0,
        "robot_rebound": None,
        "t_max_outer": 0.0,
    }
    steps = int(t_end / dt)
    for s in range(steps):
        wi_hit = ph @ qi
        pen_r = robot_x - wi_hit
        fr = k_robot * pen_r if pen_r > 0 else 0.0
        wi = phi_grid @ qi
        wo = phi_grid @ qo
        pen = wi - wo - GAP
        contact_force_per_point = np.where(pen > 0, k_sheet * pen, 0.0)
        Fs = contact_force_per_point.sum()
        Ni = cb * np.dot(qi * qi, k2)
        No = cb * np.dot(qo * qo, k2)
        gi = fr * ph - phi_grid.T @ contact_force_per_point
        go = phi_grid.T @ contact_force_per_point
        ai = (gi / modal_mass) - 2 * ZETA * w * vi - (w**2 + Ni * k2 / (RHO * H_M)) * qi
        ao = (go / modal_mass) - 2 * ZETA * w * vo - (w**2 + No * k2 / (RHO * H_M)) * qo
        vi += ai * dt
        qi += vi * dt
        vo += ao * dt
        qo += vo * dt
        robot_v -= fr / robot_m * dt
        robot_x += robot_v * dt
        out["impulse_to_outer"] += Fs * dt
        if s % 10 == 0:
            mo = wo.max()
            if mo > out["max_outer"]:
                out["max_outer"], out["t_max_outer"] = float(mo), s * dt
            out["max_inner"] = max(out["max_inner"], float(np.abs(wi).max()))
            if s % 50 == 0:
                out["pull_in_inner"] = max(out["pull_in_inner"], float(pull_in(qi)))
                out["pull_in_outer"] = max(out["pull_in_outer"], float(pull_in(qo)))
            pv = pr @ qo
            out["probe_max"] = [max(a, abs(float(b))) for a, b in zip(out["probe_max"], pv)]
            out["pry_max"] = max(out["pry_max"], abs(float(pv[0] - pv[1])) if len(pv) > 1 else 0)
            # Outer bending stress at the surface: sigma = 6 M / h^2, plus membrane N / h.
            wxx = -phi_grid @ (kx2 * qo)
            wyy = -phi_grid @ (ky2 * qo)
            mxx = -D * (wxx + NU * wyy)
            myy = -D * (wyy + NU * wxx)
            sig = 6 * np.maximum(np.abs(mxx), np.abs(myy)).max() / H_M**2 + No / H_M
            out["max_sigma_outer"] = max(out["max_sigma_outer"], float(sig))
        out["max_robot_force"] = max(out["max_robot_force"], fr)
        out["max_sheet_force"] = max(out["max_sheet_force"], Fs)
    out["robot_rebound"] = robot_v
    out["momentum_in"] = robot_m * v0
    out["modes"] = int(len(k2))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--v", type=float, default=10.0)
    ap.add_argument("--hit", default="1.22,0.61", help="x,y m from the lower-left corner")
    ap.add_argument("--held", action="store_true", help="edges pinned in plane: Berger stiffening")
    ap.add_argument("--k-robot", type=float, default=1e6)
    ap.add_argument("--k-sheet", type=float, default=2e5, help="per grid point, N/m")
    ap.add_argument("--nx", type=int, default=61)
    ap.add_argument("--ny", type=int, default=31)
    ap.add_argument("--t", type=float, default=0.15)
    ap.add_argument("--dt", type=float, default=1e-5)
    a = ap.parse_args()
    hit = tuple(float(c) for c in a.hit.split(","))
    probes = [(A_M / 2, B_M - UPPER_CUP_BELOW_EDGE_M), (A_M / 2, B_M - LOWER_CUP_BELOW_EDGE_M)]
    r = run(a.v, hit, a.held, a.k_robot, a.k_sheet, a.nx, a.ny, a.t, a.dt, probes)
    print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}))


if __name__ == "__main__":
    main()
