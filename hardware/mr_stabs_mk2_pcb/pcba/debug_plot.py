"""Scratch: plot the legalizer's F and B rectangles over the keep-outs (placement debugging)."""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from shapely.geometry import Polygon  # noqa: E402

HOLE_FC = "#e8f0e8"


def _draw(ax, g, **kw):
    for p in getattr(g, "geoms", [g]):
        if p.is_empty:
            continue
        x, y = p.exterior.xy
        ax.fill(x, y, **kw)
        for ring in p.interiors:  # holes: repaint with the board colour
            x, y = ring.xy
            ax.fill(x, y, fc=HOLE_FC, ec="none")


def plot(lay, solved, roof, under, outline):
    fig, axes = plt.subplots(2, 1, figsize=(16, 14))
    for ax, side, ko in ((axes[0], "F", roof), (axes[1], "B", under)):
        ol = Polygon(outline)
        _draw(ax, ol, fc="#e8f0e8", ec="k")
        _draw(ax, ko.intersection(ol.buffer(2)), fc="#f4b0b0", ec="none", alpha=0.7)
        for ref, pose in solved.items():
            x, y, rot = pose[:3]
            on_b = len(pose) == 4
            through = ref in lay.through or (
                ref in getattr(lay, "fixed_b", {}) and lay.fixed_b[ref][3]
            )
            if (side == "B") != on_b and not through:
                continue
            r = lay.rect(ref, x, y, rot)
            _draw(ax, r, fc="#7aa5f0" if (side == "B") == on_b else "#cccccc", ec="k", alpha=0.8)
            ax.text(
                x, y, f"{ref}\n{lay.info[ref]['value'][:8]}", fontsize=6, ha="center", va="center"
            )
        ax.set_aspect("equal")
        ax.set_xlim(-48, 48)
        ax.set_ylim(-22, 16)
        ax.grid(True, lw=0.3)
        ax.set_title(
            f"{side} side ({'roof face' if side == 'F' else 'wedge face, seen from the roof'})"
        )
    fig.tight_layout()
    fig.savefig(".scratch/debug_place.png", dpi=110)
