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

A, B, H = 2.44, 1.22, 0.00635  # m
E, NU, RHO = 2.38e9, 0.37, 1200.0
D = E * H**3 / (12 * (1 - NU**2))
GAP = 0.0254
ZETA = 0.03
FMAX = 600.0
YIELD = 62e6  # Pa, PC tensile yield (static)
ENGAGE = 0.010  # m, sheet depth in each edge channel (assumed)


def modes():
    c = np.sqrt(D / (RHO * H))
    mn, k2 = [], []
    for m in range(1, 60):
        for n in range(1, 30):
            kk = (m * np.pi / A) ** 2 + (n * np.pi / B) ** 2
            if kk * c / (2 * np.pi) <= FMAX:
                mn.append((m, n))
                k2.append(kk)
    mn = np.array(mn)
    k2 = np.array(k2)
    return mn, k2, np.sqrt(k2) ** 2 * c  # omega


def shapes(mn, x, y):
    """phi[i, j] = sin(m pi x_i / A) sin(n pi y_i / B)."""
    return np.sin(np.outer(x, mn[:, 0] * np.pi / A)) * np.sin(np.outer(y, mn[:, 1] * np.pi / B))


def run(
    v0, hit, berger=False, k_robot=1e6, k_sheet=2e5, nx=61, ny=31, t_end=0.15, dt=1e-5, probes=()
):
    """One impact. Returns peaks: deflections (m), forces (N), stress (Pa), impulse (N s)."""
    return _run(
        v0, hit, berger, k_robot, k_sheet, nx, ny, t_end, dt, list(probes) or [(A / 2, B / 2)]
    )


def _run(v0, hit, berger, k_robot, k_sheet, nx, ny, t_end, dt, probes):
    mn, k2, w = modes()
    M = RHO * H * A * B / 4  # modal mass, every mode
    gx, gy = np.meshgrid(np.linspace(0, A, nx + 2)[1:-1], np.linspace(0, B, ny + 2)[1:-1])
    gx, gy = gx.ravel(), gy.ravel()
    P = shapes(mn, gx, gy)  # grid x modes
    ph = shapes(mn, np.array([hit[0]]), np.array([hit[1]]))[0]
    pr = shapes(mn, np.array([p[0] for p in probes]), np.array([p[1] for p in probes]))
    # Curvatures on the grid for bending stress: w_xx = -sum (m pi/A)^2 q phi, etc.
    kx2 = (mn[:, 0] * np.pi / A) ** 2
    ky2 = (mn[:, 1] * np.pi / B) ** 2
    # Berger: N = (3 D / (2 h^2)) sum q^2 k^2 (tension per unit length, both directions)
    cb = 3 * D / (2 * H**2) if berger else 0.0

    qi = np.zeros(len(k2))
    vi = np.zeros(len(k2))
    qo = np.zeros(len(k2))
    vo = np.zeros(len(k2))
    xr, vr, mr = 0.0, v0, 1.36  # robot position (toward the outer sheet = +), velocity, mass

    # Channel pull-in. Along Y at column x: 1/2 int w_y^2 dy = 1/2 sum_n (n pi/B)^2 (B/2) a_n(x)^2,
    # a_n(x) = sum_m q_mn sin(m pi x / A); the same along X. Sampled on 41 lines each way.
    xs, ys = np.linspace(0, A, 43)[1:-1], np.linspace(0, B, 23)[1:-1]
    sxm = np.sin(np.outer(xs, mn[:, 0] * np.pi / A))  # lines x modes
    syn = np.sin(np.outer(ys, mn[:, 1] * np.pi / B))
    nset, mset = sorted(set(mn[:, 1])), sorted(set(mn[:, 0]))
    nidx = [mn[:, 1] == n for n in nset]
    midx = [mn[:, 0] == m for m in mset]

    def pull_in(q):
        ay = np.stack([(sxm[:, k] * q[k]).sum(1) for k in nidx], 1)  # lines x n
        dy = 0.5 * ((np.array(nset) * np.pi / B) ** 2 * (B / 2) * ay**2).sum(1).max()
        ax = np.stack([(syn[:, k] * q[k]).sum(1) for k in midx], 1)
        dx = 0.5 * ((np.array(mset) * np.pi / A) ** 2 * (A / 2) * ax**2).sum(1).max()
        return max(dx, dy) / 2  # per edge

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
        pen_r = xr - wi_hit
        fr = k_robot * pen_r if pen_r > 0 else 0.0
        wi = P @ qi
        wo = P @ qo
        pen = wi - wo - GAP
        fs = np.where(pen > 0, k_sheet * pen, 0.0)  # per grid point, pushes outer +, inner -
        Fs = fs.sum()
        Ni = cb * np.dot(qi * qi, k2)
        No = cb * np.dot(qo * qo, k2)
        gi = fr * ph - P.T @ fs
        go = P.T @ fs
        ai = (gi / M) - 2 * ZETA * w * vi - (w**2 + Ni * k2 / (RHO * H)) * qi
        ao = (go / M) - 2 * ZETA * w * vo - (w**2 + No * k2 / (RHO * H)) * qo
        vi += ai * dt
        qi += vi * dt
        vo += ao * dt
        qo += vo * dt
        vr -= fr / mr * dt
        xr += vr * dt
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
            wxx = -P @ (kx2 * qo)
            wyy = -P @ (ky2 * qo)
            mxx = -D * (wxx + NU * wyy)
            myy = -D * (wyy + NU * wxx)
            sig = 6 * np.maximum(np.abs(mxx), np.abs(myy)).max() / H**2 + No / H
            out["max_sigma_outer"] = max(out["max_sigma_outer"], float(sig))
        out["max_robot_force"] = max(out["max_robot_force"], fr)
        out["max_sheet_force"] = max(out["max_sheet_force"], Fs)
    out["robot_rebound"] = vr
    out["momentum_in"] = mr * v0
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
    # Pack at mid-span: upper cup 130 mm below the top edge, lower cup 150 mm below that.
    probes = [(A / 2, B - 0.130), (A / 2, B - 0.280)]
    r = run(a.v, hit, a.held, a.k_robot, a.k_sheet, a.nx, a.ny, a.t, a.dt, probes)
    print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}))


if __name__ == "__main__":
    main()
