"""Cut TX16S carry-handle bar sections out of the radio scan."""

import argparse
import importlib.util
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STATIONS = np.arange(58.0, 80.01, 2.0)
BAR_Y_MIN = 40.0
BAR_Z_MIN = -40.0
SLAB = 0.2
BINS = 72


def _scan_patch():
    path = HERE.parent / "tx16s_hr10_mount" / "scan_patch.py"
    spec = importlib.util.spec_from_file_location("scan_patch", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def section_loop(v: np.ndarray, x: float) -> np.ndarray:
    s = v[(np.abs(v[:, 0] - x) < SLAB) & (v[:, 1] > BAR_Y_MIN) & (v[:, 2] > BAR_Z_MIN)][:, 1:]
    c = s.mean(axis=0)
    d = s - c
    ang = np.arctan2(d[:, 1], d[:, 0])
    rad = np.hypot(d[:, 0], d[:, 1])
    idx = ((ang + np.pi) / (2 * np.pi) * BINS).astype(int) % BINS
    r = np.array([np.median(rad[idx == k]) if (idx == k).any() else np.nan for k in range(BINS)])
    if np.isnan(r).any():
        raise SystemExit(f"x={x}: {np.isnan(r).sum()} empty angle bins; scan has a hole here")
    r = (np.roll(r, 1) + r + np.roll(r, -1)) / 3
    a = -np.pi + (np.arange(BINS) + 0.5) * 2 * np.pi / BINS
    return c + np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("scan_zip", type=Path)
    ap.add_argument("--out", type=Path, default=HERE / "tx16s_handle_bar_sections.npz")
    args = ap.parse_args()

    sp = _scan_patch()
    v = sp.read_vertices(args.scan_zip) @ sp.radio_rotation().T
    loops = np.stack([section_loop(v, x) for x in STATIONS])
    np.savez_compressed(args.out, x=STATIONS, loops=loops)
    for x, loop in zip(STATIONS, loops, strict=True):
        lo, hi = loop.min(axis=0), loop.max(axis=0)
        print(f"x={x:5.1f}  y {lo[0]:.1f}..{hi[0]:.1f}  z {lo[1]:.1f}..{hi[1]:.1f}")


if __name__ == "__main__":
    main()
