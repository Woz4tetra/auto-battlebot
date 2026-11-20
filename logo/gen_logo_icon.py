#!/usr/bin/env python3
"""Render the BWBots logo as a small transparent PNG for a dark top bar."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image

from auto_battlebot.branding import splash_animation as splash

SCRIPT_DIR = Path(__file__).resolve().parent
SUPERSAMPLE = 4
MIN_STROKE_PX = 1


def render_icon(svg_path: Path, height: int) -> Image.Image:
    """Composite the final splash frame (settled logo) on a transparent canvas."""
    radius_ss = (height * SUPERSAMPLE) // 2
    stroke_ss = max(MIN_STROKE_PX * SUPERSAMPLE, round(radius_ss / splash.REF_RADIUS))
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
