"""DC current flow through each high-current net, every copper layer at once.

    ~/.local/share/pcba-design/venv/bin/python sim/copper_ir.py    (after export_geometry.py)

Finite volume on a 0.15 mm grid per layer: each cell couples to its neighbours by 1 / R_sheet;
each via or plated hole couples the cells it passes through on every layer by the barrel
resistance between those layers. A wire pad is a terminal: its cells are tied to one node (the
wire and its solder are far better conductors than the board). Current enters and leaves at
the terminals, split evenly between the ESCs. Reports, per net and load: the voltage drop,
the copper loss, the busiest via's current, and the 99.9th-percentile current density;
writes sim/copper_ir.json and sim/copper_ir_<NET>.png (20 A).

Copper: 2 oz outer (as ordered), 0.5 oz inner (JLCPCB's 4-layer default; this design does not
order 1 oz). Stack positions for a 1.6 mm JLCPCB 4-layer board: F 0, In1 0.21, In2 1.39, B 1.6.
This is DC and electrical only: no temperature.
"""

import json
import math
import os

import matplotlib
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
import shapely
from shapely.geometry import Polygon

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RHO = 1.72e-8
PLATING = 20e-6
H = 0.15
OZ = {"F": 2.0, "In1": 0.5, "In2": 0.5, "B": 2.0}
Z = {"F": 0.0, "In1": 0.21e-3, "In2": 1.39e-3, "B": 1.6e-3}
TIE = 1e4  # S per cell, terminal cells to their node


def barrel_g(drill_mm, length_m):
    r = drill_mm / 2e3
    return math.pi * ((r + PLATING) ** 2 - r**2) / (RHO * length_m)


def poly_union(polys):
    if not polys:
        return None
    return shapely.union_all([Polygon(p[0], p[1:]).buffer(0) for p in polys])


class Network:
    """The net's copper as a resistor network: grid cells per layer, vias, terminal nodes."""

    def __init__(self, geo):
        polys = {k: poly_union(v) for k, v in geo["layers"].items()}
        self.layers = {k: v for k, v in polys.items() if v is not None and not v.is_empty}
        x0, y0, x1, y1 = shapely.union_all(list(self.layers.values())).bounds
        self.xs = np.arange(x0 + H / 2, x1, H)
        self.ys = np.arange(y0 + H / 2, y1, H)
        self.gx, self.gy = np.meshgrid(self.xs, self.ys)
        self.idx, base = {}, 0
        for name, poly in self.layers.items():
            inside = shapely.contains_xy(poly, self.gx, self.gy)
            ix = -np.ones(inside.shape, int)
            ix[inside] = base + np.arange(inside.sum())
            base += int(inside.sum())
            self.idx[name] = ix
        self.terms = list(geo["terminals"])
        self.node = {t: base + i for i, t in enumerate(self.terms)}
        self.n = base + len(self.terms)
        self.rows, self.cols, self.vals = [], [], []
        self.diag = np.zeros(self.n)
        self.via_links = []  # (cell_a, cell_b, g, via_index)
        self._sheets()
        self._vias(geo["holes"])
        self._terminals(geo["terminals"])

    def couple(self, a, b, g):
        self.rows.extend([a, b])
        self.cols.extend([b, a])
        self.vals.extend([-g, -g])
        np.add.at(self.diag, a, g)
        np.add.at(self.diag, b, g)

    def _sheets(self):
        for name, ix in self.idx.items():
            gs = 1.0 / (RHO / (OZ[name] * 35e-6))
            for dy, dx in ((0, 1), (1, 0)):
                a = ix[: ix.shape[0] - dy, : ix.shape[1] - dx]
                b = ix[dy:, dx:]
                m = (a >= 0) & (b >= 0)
                self.couple(a[m], b[m], np.full(int(m.sum()), gs))

    def cell_at(self, name, x, y, reach=0.6):
        """The copper cell nearest (x, y) on a layer within reach (a via sits in its own
        clearance hole on planes of other nets), or -1."""
        ix = self.idx.get(name)
        if ix is None:
            return -1
        j, i = int(round((x - self.xs[0]) / H)), int(round((y - self.ys[0]) / H))
        r = int(reach / H)
        ii, jj = np.mgrid[i - r : i + r + 1, j - r : j + r + 1]
        ok = (ii >= 0) & (ii < ix.shape[0]) & (jj >= 0) & (jj < ix.shape[1])
        ii, jj = ii[ok], jj[ok]
        cells = ix[ii, jj]
        if not (cells >= 0).any():
            return -1
        d = (ii - i) ** 2 + (jj - j) ** 2
        d[cells < 0] = 10**9
        return int(cells[np.argmin(d)])

    def _vias(self, holes):
        order = sorted(self.layers, key=lambda k: Z[k])
        for vi, (vx, vy, drill) in enumerate(holes):
            cells = [(nm, self.cell_at(nm, vx, vy)) for nm in order]
            cells = [(nm, c) for nm, c in cells if c >= 0]
            for (n1, c1), (n2, c2) in zip(cells, cells[1:]):
                g = barrel_g(drill, max(Z[n2] - Z[n1], 0.1e-3))
                self.couple(np.array([c1]), np.array([c2]), np.array([g]))
                self.via_links.append((c1, c2, g, vi))

    def _terminals(self, terminals):
        for t, spec in terminals.items():
            ix = self.idx[spec["layer"]]
            m = shapely.contains_xy(poly_union(spec["poly"]), self.gx, self.gy) & (ix >= 0)
            cells = ix[m]
            self.couple(cells, np.full(cells.size, self.node[t]), np.full(cells.size, TIE))

    def solve(self, currents):
        """Node voltages for currents injected at the terminals; the first terminal is 0 V."""
        cat = [np.atleast_1d(x) for x in self.vals] + [self.diag]
        rows = [np.atleast_1d(x) for x in self.rows] + [np.arange(self.n)]
        cols = [np.atleast_1d(x) for x in self.cols] + [np.arange(self.n)]
        mat = sp.coo_matrix(
            (np.concatenate(cat), (np.concatenate(rows), np.concatenate(cols))),
            shape=(self.n, self.n),
        ).tocsr()
        rhs = np.zeros(self.n)
        for t, amps in currents.items():
            rhs[self.node[t]] = amps
        keep = np.arange(self.n) != self.node[self.terms[0]]
        v = np.zeros(self.n)
        v[keep] = spla.spsolve(mat[keep][:, keep].tocsc(), rhs[keep])
        return v

    def results(self, v):
        ref = v[self.node[self.terms[0]]]
        out = {"drop_mV": {t: round(float(v[self.node[t]] - ref) * 1e3, 1) for t in self.terms[1:]}}
        loss, dens = 0.0, {}
        for name, ix in self.idx.items():
            vg = np.full(ix.shape, np.nan)
            vg[ix >= 0] = v[ix[ix >= 0]]
            dvy, dvx = np.gradient(vg, H * 1e-3)
            gs = 1.0 / (RHO / (OZ[name] * 35e-6))
            loss += float(np.nansum((dvx**2 + dvy**2) * gs * (H * 1e-3) ** 2))
            dens[name] = np.hypot(dvx, dvy) * gs * 1e-3  # A per mm of width
        iv = {}
        for c1, c2, g, vi in self.via_links:
            iv[vi] = max(iv.get(vi, 0.0), abs((v[c1] - v[c2]) * g))
        via_loss = sum(((v[c1] - v[c2]) ** 2) * g for c1, c2, g, _ in self.via_links)
        out.update(
            {
                "copper_loss_W": round(loss + float(via_loss), 2),
                "busiest_via_A": round(float(max(iv.values())), 2) if iv else 0.0,
                "vias_over_1_5A": int(sum(1 for a in iv.values() if a > 1.5)),
                "peak_A_per_mm": {
                    k: round(float(np.nanpercentile(d, 99.9)), 1)
                    for k, d in dens.items()
                    if np.isfinite(d).any()
                },
                "_via_currents": iv,
            }
        )
        extent = (self.xs[0], self.xs[-1], self.ys[0], self.ys[-1])
        return out, dens, extent


def solve(geo, currents):
    net = Network(geo)
    return net.results(net.solve(currents))


STUDIES = [
    ("PACK+", "PACKP", lambda a: {"in": a, "out": -a}),
    ("BAT_IN", "BAT_IN", lambda a: {"in": a, "out": -a}),
    ("VBATT", "VBATT", lambda a: {"in": a, "esc_l": -a / 2, "esc_r": -a / 2}),
    ("GND", "GND", lambda a: {"pack": -a, "esc_l": a / 2, "esc_r": a / 2}),
    ("PACK_MID", "PACK_MID", lambda a: {"in": a, "out": -a}),
]
results = []
for net, fname, cur in STUDIES:
    geo = json.load(open(os.path.join(HERE, f"copper_{fname}.json")))
    for amps in (20.0, 70.0):
        r, dens, ext = solve(geo, cur(amps))
        r.pop("_via_currents")
        r.update({"net": net, "amps": amps})
        results.append(r)
        print(r)
        if amps == 20.0:
            worst = max(
                dens,
                key=lambda k: np.nanpercentile(dens[k], 99.9) if np.isfinite(dens[k]).any() else 0,
            )
            fig, ax = plt.subplots(figsize=(10, 3.8))
            d = dens[worst]
            im = ax.imshow(
                d, origin="lower", extent=ext, cmap="inferno", vmax=np.nanpercentile(d, 99.5)
            )
            fig.colorbar(im, ax=ax, label="A per mm of width")
            ax.set_title(
                f"{net}, 20 A: current density on {worst} ({OZ[worst]} oz), its densest layer"
            )
            ax.set_xlabel("x, mm")
            ax.set_ylabel("y, mm")
            fig.tight_layout()
            fig.savefig(os.path.join(HERE, f"copper_ir_{fname}.png"), dpi=100)
            plt.close(fig)
json.dump(results, open(os.path.join(HERE, "copper_ir.json"), "w"), indent=1)
