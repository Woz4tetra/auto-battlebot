#!/usr/bin/env python3
"""Render the BW-bots logo as a small transparent PNG for the UI top bar.

Produces the dark-background variant used by the splash animation: the logo's
black half-disc is drawn as a white outline (it would vanish against the dark
bar), the white half is plain fill, and the BW pill comes from the SVG. The
background is transparent so the icon sits directly on the bar.

The PNG is embedded into the app at build time (cmake/bin2c.cmake) and decoded
by LVGL's lodepng decoder.

Requires: inkscape on PATH. Run inside the project venv (numpy, Pillow).

Usage:
    logo/gen_logo_icon.py                  # 24 px tall -> logo/bwbots_logo_dark_h24.png
    logo/gen_logo_icon.py --height 48
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import gen_splash_animation as splash
import numpy as np
from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
SUPERSAMPLE = 4


def render_icon(svg_path: Path, height: int) -> Image.Image:
    """Composite the final splash frame (settled logo) on a transparent canvas."""
    radius_ss = (height * SUPERSAMPLE) // 2  # logo height = disc diameter
    # The reference stroke (r/30) is sub-pixel at icon sizes; keep it >= 1 output px.
    stroke_ss = max(SUPERSAMPLE, round(radius_ss / splash.REF_RADIUS))
    sprites = splash.HalfDiscSprites(radius_ss, stroke_ss)
    pill = splash.render_pill_layer(svg_path, float(radius_ss))

    left, right, angle, pill_frac = splash.KEYFRAMES[-1]
    frame = splash.render_frame(
        left * splash.SPLIT_RESCALE,
        right * splash.SPLIT_RESCALE,
        angle,
        pill_frac,
        (4 * radius_ss, 3 * radius_ss),
        radius_ss,
        sprites,
        pill,
        background=(0, 0, 0, 0),
    )

    alpha = np.asarray(frame)[:, :, 3]
    ys, xs = np.nonzero(alpha > 8)
    frame = frame.crop((int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1))
    width = round(frame.width * height / frame.height)
    return frame.resize((width, height), Image.Resampling.LANCZOS)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--height", type=int, default=24, help="output height px (default 24)")
    parser.add_argument("--svg", type=Path, default=SCRIPT_DIR / "bwbots_logo.svg")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    out_path: Path = args.out or SCRIPT_DIR / f"bwbots_logo_dark_h{args.height}.png"
    icon = render_icon(args.svg, args.height)
    icon.save(out_path)
    size_kb = out_path.stat().st_size / 1024
    print(f"wrote {out_path} ({icon.width}x{icon.height}, {size_kb:.1f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
